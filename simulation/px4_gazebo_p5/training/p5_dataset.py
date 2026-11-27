"""DAgger data collection (plan §4.5): synthetic closed-loop episodes with the shield always on.

Round 0 drives with P4's own L3 selection behind the P5 hook; later rounds drive with the production
`PolicyRanker` over the previous round's weights. Every `stride`-th ranker invocation records the
adapter tensor (stored as the per-slot candidate block plus the shared context block), the allowed
mask and the oracle costs. Labels always come from the oracle, never from the driver.
"""
import math

import numpy as np

import p5_episode
import p5_oracle
import p5_paths  # noqa: F401  (import paths)
from gwm_obstacle_avoidance import planner
from gwm_policy_control import candidate_ranker, observation_adapter, policy_runtime, safety_shield
from gwm_policy_control.contracts import ARCHITECTURE, CANDIDATE_FEATURES, STATES

SAMPLE_STRIDE = 8
ARRAYS = ("candidate", "context", "allowed", "costs", "driver_slot", "state", "layout_index")


def zero_clock():
    """Offline runs do not measure wall time (timing is an M2/M4 and SITL measurement)."""
    return 0.0


def intermediate_policy(weights, label):
    """A DAgger-round policy: training-time only, never exported, never `trained: true`."""
    meta = {"architecture": ARCHITECTURE, "trained": False, "provenance": "DAgger intermediate " + label}
    return policy_runtime.FrozenPolicy(weights, meta, sha256=label)


class RecordingRanker:
    def __init__(self, policy, schema, field, boxes, speed, yaw_rate, stride, phase):
        self.inner = None if policy is None else candidate_ranker.PolicyRanker(
            policy, schema, failure_wall_s=math.inf, clock=zero_clock)
        self.schema, self.field, self.boxes = schema, field, boxes
        self.speed, self.yaw_rate, self.stride, self.phase = speed, yaw_rate, stride, phase
        self.samples, self.calls = [], 0

    def choose(self, rows, progress, ctx, grid, traversed, state, settings):
        if self.inner is None:
            best, record = planner.select(progress, settings, grid.resolution), None
            features, allowed = observation_adapter.build(rows, ctx, settings, grid, traversed, state, self.schema)
        else:
            best, record = self.inner.choose(rows, progress, ctx, grid, traversed, state, settings)
            if best is None:
                return best, record
            features, allowed = record["features"], record["allowed"]
        if (self.calls + self.phase) % self.stride == 0:
            slots = [observation_adapter.slot_of(r, settings) for r in rows]
            costs = p5_oracle.candidate_costs(rows, allowed, slots, ctx, self.field, self.boxes, settings,
                                              self.speed, self.yaw_rate)
            context = features[:, CANDIDATE_FEATURES:]
            if not (context == context[0]).all():
                raise ValueError("context block differs between slots")
            self.samples.append((features[:, :CANDIDATE_FEATURES].copy(), context[0].copy(), allowed.copy(), costs,
                                 observation_adapter.slot_of(best, settings), STATES.index(state)))
        self.calls += 1
        return best, record


def collect_episode(job):
    """job: (layout index, layout, scan cells, weights or None, round, schema, stride)."""
    index, layout, scan, weights, round_index, schema, stride = job
    _, C, P2 = p5_episode.configs()
    field = p5_oracle.CostToGo(layout["boxes"], C)
    policy = None if weights is None else intermediate_policy(weights, "round-%d" % (round_index - 1))
    ranker = RecordingRanker(policy, schema, field, layout["boxes"], C["mission"]["transit_speed_m_s"],
                             math.radians(P2["yaw_ramp_deg_s"]), stride, (layout["seed"] + 3 * round_index) % stride)
    result = p5_episode.run_episode(layout["boxes"], lambda c, s: safety_shield.PolicySupervisor(c, s, ranker),
                                    scan_cells=scan)
    n = len(ranker.samples)
    arrays = {
        "candidate": np.array([s[0] for s in ranker.samples], dtype=np.float64).reshape(n, -1, CANDIDATE_FEATURES),
        "context": np.array([s[1] for s in ranker.samples], dtype=np.float64).reshape(n, -1),
        "allowed": np.array([s[2] for s in ranker.samples], dtype=bool).reshape(n, -1),
        "costs": np.array([s[3] for s in ranker.samples], dtype=np.float64).reshape(n, -1),
        "driver_slot": np.array([s[4] for s in ranker.samples], dtype=np.int64),
        "state": np.array([s[5] for s in ranker.samples], dtype=np.int64),
        "layout_index": np.full(n, index, dtype=np.int64)}
    episode = {"id": layout["id"], "outcome": result["outcome"], "reason": result["reason"],
               "transit_s": result["transit_s"], "samples": n, "invocations": ranker.calls,
               "policy_failures": result["failures"], "outside_mask": result["outside_mask"]}
    return arrays, episode


def concatenate(parts):
    return {name: np.concatenate([p[name] for p in parts]) for name in ARRAYS}
