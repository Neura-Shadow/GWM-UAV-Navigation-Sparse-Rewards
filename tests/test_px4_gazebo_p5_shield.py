"""P5 M2: ranker, planner replica and P5 supervisor — parity with P4, masking and fail-closed behaviour."""
import inspect
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
from gwm_obstacle_avoidance import local_map, planner, supervisor  # noqa: E402
from gwm_policy_control import candidate_ranker, decision, observation_adapter, safety_shield  # noqa: E402
from gwm_policy_control.candidate_ranker import P4Ranker, PolicyRanker  # noqa: E402
from gwm_policy_control.policy_runtime import PolicyError  # noqa: E402
from gwm_policy_control.safety_shield import PolicySupervisor  # noqa: E402

C = p5.syn.p4_config()
PL, ENV = C["planner"], C["envelope"]
SCENARIOS = p5.syn.scenarios()["scenarios"]


def random_world(seed):
    """Structured world: observed free space up to a random horizon and lateral band (unknown beyond),
    0-4 random pillars, a free disc at the target and a few traversed cells. Seeds cover TRANSLATE,
    YAW_TO, SCAN and HOLD (checked by the branch-coverage test below)."""
    rng = np.random.default_rng(seed)
    grid = local_map.Grid.around((0.0, 0.0), C["geofence"], C["perception"]["grid_margin_m"], C["perception"]["grid_resolution_m"])
    target = (rng.uniform(0.0, 6.0), rng.uniform(-1.5, 1.5))
    cx = grid.x_min + (np.arange(grid.nx)[:, None] + 0.5) * grid.resolution
    cy = grid.y_min + (np.arange(grid.ny)[None, :] + 0.5) * grid.resolution
    horizon, band = rng.uniform(1.5, 5.0), rng.uniform(1.0, 3.0)
    unknown = (cx > target[0] + horizon) | (np.abs(cy - target[1]) > band) | (cx < target[0] - 1.5)
    grid.cells[:] = np.where(unknown, local_map.UNKNOWN, local_map.FREE).astype(np.uint8)
    for _ in range(int(rng.integers(0, 5))):
        px, py, half = target[0] + rng.uniform(0.8, 4.0), target[1] + rng.uniform(-2.0, 2.0), rng.uniform(0.1, 0.3)
        grid.cells[(np.abs(cx - px) <= half) & (np.abs(cy - py) <= half)] = local_map.OCCUPIED
    grid.cells[np.hypot(cx - target[0], cy - target[1]) <= 0.6] = local_map.FREE
    traversed = rng.random(grid.cells.shape) < 0.01
    ctx = planner.Context(target, target, rng.uniform(-1.2, 1.2), tuple(C["mission"]["goal_enu_m"]), (0.0, 0.0),
                          0.1, C["geofence"], scanned={round(rng.uniform(-1, 1), 6)})
    return grid, traversed, ctx


def test_parity_worlds_cover_every_planner_branch():
    actions = {planner.plan(*random_world(seed), PL, ENV, 0.25)["action"] for seed in range(40)}
    assert actions == {planner.TRANSLATE, planner.YAW_TO, planner.SCAN, planner.HOLD}


# ---- planner replica parity ---------------------------------------------------------------------------
@pytest.mark.parametrize("seed", range(40))
def test_planner_replica_with_the_p4_ranker_equals_p4_plan(seed):
    grid, traversed, ctx = random_world(seed)
    expected = planner.plan(grid, traversed, ctx, PL, ENV, 0.25)
    got = decision.plan_with_ranker(grid, traversed, ctx, PL, ENV, 0.25, P4Ranker(), "AVOIDING")
    assert got.pop("policy") is None and got == expected


def test_replicas_differ_from_p4_only_in_the_selection_hook():
    """Source-level drift guard: removing the hook lines must give back P4's code exactly."""
    def body(func):
        return [line.strip() for line in inspect.getsource(func).splitlines()[1:]
                if line.strip() and not line.strip().startswith(('"""', "#"))]
    p4_plan, p5_plan = body(planner.plan), body(decision.plan_with_ranker)
    hook = {'best, record = ranker.choose(rows, progress, ctx, grid, traversed, state, settings)',
            'decision["policy"] = record', 'if best is None:', 'decision["reason"] = "policy_failure:" + record["failure"]',
            'return decision'}
    p4_core = [l.replace("planner.", "").replace("best = select(progress, settings, grid.resolution)", "") for l in p4_plan]
    p5_core = [l.replace("planner.", "") for l in p5_plan if l not in hook]
    assert [l for l in p4_core if l and l != "return decision"] == [
        l.replace(', "policy": None}', "}") for l in p5_core if l != "return decision"]


# ---- selection ---------------------------------------------------------------------------------------
def rows_and_mask():
    grid, traversed, ctx = random_world(3)
    rows = planner.evaluate_candidates(grid, traversed, ctx, PL, 0.25)
    _, allowed = observation_adapter.build(rows, ctx, PL, grid, traversed, "AVOIDING", p5.SCHEMA)
    return rows, allowed


def test_policy_choice_is_the_masked_argmax_with_p4_tie_order():
    rows, allowed = rows_and_mask()
    by_slot = {observation_adapter.slot_of(r, PL): r for r in rows}
    scores = np.full(26, -5.0)
    blocked = int(np.flatnonzero(~allowed)[0])
    scores[blocked] = 100.0  # a masked slot can never win
    open_slots = np.flatnonzero(allowed)
    scores[open_slots[:2]] = 3.0  # tie: P4 order decides
    choice = candidate_ranker.policy_choice(scores, allowed, by_slot)
    assert choice == min(open_slots[:2].tolist(), key=lambda i: planner.tie_key(by_slot[i]))
    scores[blocked] = np.nan  # non-finite on a masked slot is ignored
    assert candidate_ranker.policy_choice(scores, allowed, by_slot) == choice


@pytest.mark.parametrize("change,reason", [("nan_allowed", "output_not_finite"), ("shape", "output_shape"), ("empty", "empty_mask")])
def test_policy_choice_failures(change, reason):
    rows, allowed = rows_and_mask()
    by_slot = {observation_adapter.slot_of(r, PL): r for r in rows}
    scores = np.zeros(26)
    if change == "nan_allowed":
        scores[np.flatnonzero(allowed)[0]] = np.inf
    if change == "shape":
        scores = np.zeros(25)
    if change == "empty":
        allowed = np.zeros(26, dtype=bool)
    with pytest.raises(PolicyError, match=reason):
        candidate_ranker.policy_choice(scores, allowed, by_slot)


class FakePolicy:
    sha256 = "fake"

    def __init__(self, scores=None, error=None):
        self._scores, self._error = scores, error

    def scores(self, features):
        if self._error:
            raise self._error
        return self._scores


@pytest.mark.parametrize("policy,clock,reason", [
    (None, None, "checkpoint_missing"),
    (FakePolicy(scores=np.full(26, np.nan)), None, "output_not_finite"),
    (FakePolicy(error=RuntimeError("boom")), None, "exception:RuntimeError"),
    (FakePolicy(scores=np.zeros(26)), "slow", "timeout")])
def test_ranker_failures_return_no_choice_and_a_classified_record(policy, clock, reason):
    grid, traversed, ctx = random_world(7)
    ticks = iter([0.0, 0.5])
    ranker = PolicyRanker(policy, p5.SCHEMA, clock=(lambda: next(ticks)) if clock else __import__("time").perf_counter)
    rows = planner.evaluate_candidates(grid, traversed, ctx, PL, 0.25)
    progress = [r for r in rows if r["valid"] and r["progress"] > PL["min_progress_m"]]
    best, record = ranker.choose(rows, progress, ctx, grid, traversed, "AVOIDING", PL)
    assert best is None and record["failure"] == reason
    assert record["p4_slot"] == observation_adapter.slot_of(planner.select(progress, PL, grid.resolution), PL)


def test_ranker_rejects_a_progress_set_that_differs_from_the_mask():
    grid, traversed, ctx = random_world(6)
    rows = planner.evaluate_candidates(grid, traversed, ctx, PL, 0.25)
    progress = [r for r in rows if r["valid"] and r["progress"] > PL["min_progress_m"]]
    best, record = PolicyRanker(p5.interface_policy(), p5.SCHEMA).choose(rows, progress[:-1], ctx, grid, traversed, "AVOIDING", PL)
    assert best is None and record["failure"] == "mask_mismatch"


def test_policy_decisions_use_p4_geometry_and_are_reproducible():
    grid, traversed, ctx = random_world(7)
    first = decision.plan_with_ranker(grid, traversed, ctx, PL, ENV, 0.25, PolicyRanker(p5.interface_policy(), p5.SCHEMA), "AVOIDING")
    again = decision.plan_with_ranker(grid, traversed, ctx, PL, ENV, 0.25, PolicyRanker(p5.interface_policy(), p5.SCHEMA), "AVOIDING")
    record = first["policy"]
    assert record["failure"] is None and np.array_equal(record["scores"], again["policy"]["scores"])
    rows = planner.evaluate_candidates(grid, traversed, ctx, PL, 0.25)
    chosen = rows[record["choice_slot"]]
    assert record["allowed"][record["choice_slot"]] and first["best"]["end"] == chosen["end"]
    if first["action"] == planner.TRANSLATE:
        assert first["end"] == chosen["end"] and first["length"] == chosen["length"]


def test_a_ranker_failure_plans_hold_and_never_another_ranking():
    grid, traversed, ctx = random_world(3)
    out = decision.plan_with_ranker(grid, traversed, ctx, PL, ENV, 0.25, PolicyRanker(None, p5.SCHEMA), "AVOIDING")
    assert out["action"] == planner.HOLD and out["best"] is None and out["reason"] == "policy_failure:checkpoint_missing"


# ---- P5 supervisor ------------------------------------------------------------------------------------
@pytest.mark.parametrize("name,stop", [("S2", None), ("S4", None), ("S5", None), ("S6", 36.1 + 2.0 / 0.2 + 0.05)])
def test_p5_supervisor_with_the_p4_ranker_matches_p4_tick_for_tick(name, stop):
    run = p5.closed_loop(SCENARIOS[name]["boxes"], lambda c, s: supervisor.Supervisor(c, s),
                         shadow=lambda c, s: PolicySupervisor(c, s, P4Ranker()), stop_perception_at=stop)
    assert len(run["outs"]) > 100
    for p4_out, p5_out in zip(run["outs"], run["shadow_outs"]):
        assert p5_out["policy"] is None and p5.strip_policy(p5_out) == p4_out


class Flaky:
    """Fails on chosen calls, otherwise behaves like P4's ranker with a successful policy record."""

    def __init__(self, failures):
        self.calls, self.failures = 0, set(failures)

    def choose(self, rows, progress, ctx, grid, traversed, state, settings):
        self.calls += 1
        if self.calls in self.failures:
            return None, {"failure": "timeout"}
        return planner.select(progress, settings, grid.resolution), {"failure": None}


def supervisor_inputs():
    grid, traversed, ctx = random_world(11)
    return {"sim": 50.0, "snapshot": {"acquisition_sim_s": 49.95, "calibration_identity": "c"}, "grid": grid,
            "traversed": traversed, "adapter_health": {"status": "fresh", "calibration_identity": "c", "sim_s": 49.9},
            "consecutive_unusable": 0, "reset_sim": None, "vehicle": ctx.target, "vehicle_yaw": ctx.vehicle_yaw,
            "target": ctx.target, "goal": ctx.goal, "origin": ctx.origin, "geofence": C["geofence"]}


def test_policy_failures_hold_then_abort_after_three_consecutive():
    sup = PolicySupervisor(C, 40.0, Flaky({1, 2, 4, 5, 6}))
    inputs = supervisor_inputs()
    states = []
    for _ in range(6):
        out = sup.step(inputs)
        states.append((out["state"], out["action"], out["frozen_yaw"], out["reason"]))
        sim = inputs["sim"] + 0.05  # keep the snapshot and health fresh so only the policy rule acts
        inputs = dict(inputs, sim=sim, snapshot={"acquisition_sim_s": sim - 0.05, "calibration_identity": "c"},
                      adapter_health={"status": "fresh", "calibration_identity": "c", "sim_s": sim - 0.1})
    assert states[0][1:] == (planner.HOLD, True, "policy_failure:timeout") and states[0][0] == supervisor.NORMAL
    assert states[1][1] == planner.HOLD and states[2][3] != "policy_failure:timeout"  # success resets the count
    assert states[3][1] == planner.HOLD and states[4][1] == planner.HOLD
    assert states[5][0] == supervisor.ABORT and sup.abort_reason == "policy_unavailable"


def test_untrained_interface_policy_driving_the_loop_never_breaks_the_shield():
    """Arbitrary (random) rankings still execute only P4-valid candidates: S2 stays collision-free."""
    run = p5.closed_loop(SCENARIOS["S2"]["boxes"], lambda c, s: PolicySupervisor(c, s, PolicyRanker(p5.interface_policy(3), p5.SCHEMA)))
    records = [o["policy"] for o in run["outs"] if o["policy"] is not None]
    assert records and all(r["failure"] is None and r["allowed"][r["choice_slot"]] for r in records)
    assert run["min_clearance"] >= 0.20


def test_policy_modules_have_no_ros_or_px4_publication_path():
    package = p5.P5 / "ros2_ws/src/gwm_policy_control/gwm_policy_control"
    for name in ("policy_runtime", "observation_adapter", "candidate_ranker", "decision", "safety_shield", "contracts"):
        text = (package / (name + ".py")).read_text(encoding="utf-8")
        assert "rclpy" not in text and "create_publisher" not in text and "px4_msgs" not in text, name
    assert safety_shield.PolicySupervisor.__mro__[1] is supervisor.Supervisor
