"""Offline dry check of the scheduling diagnostic (no simulator, no ROS graph, nothing launched).

1. diag_runner refuses before launching anything when the explicit gates are missing, and when run outside
   the owned namespace.
2. diag_control's wrapper and polar-profile swap work on real reconstructed ticks: `control_node.main` is
   stubbed by a function that drives the production `PolicyRanker` on 5 ticks; the per-call JSON lines are
   checked, and the wrapped scores must equal the unwrapped production scores bit for bit.
Run under the controller interpreter environment.
"""
import json
import os
from pathlib import Path
import sys
import tempfile

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "p5_inference_benchmark"))

import common  # noqa: E402
import numpy as np  # noqa: E402

# 1. runner refusals
import diag_runner  # noqa: E402
for env, expect in (({}, "Missing explicit gates"),
                    ({k: "1" for k in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH",
                                       "GWM_ALLOW_SITL_COMMANDS")}, "Owned private PID/network namespace required")):
    saved = {k: os.environ.pop(k, None) for k in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS",
                                                   "GWM_ALLOW_PX4_LAUNCH", "GWM_ALLOW_SITL_COMMANDS")}
    os.environ.update(env)
    try:
        diag_runner.main(["--approval", "/nonexistent", "--manifest", "/nonexistent", "--scenario", "S2",
                          "--allow-simulated-flight"])
        print("REFUSAL MISSING", expect)
        sys.exit(1)
    except (ValueError, PermissionError) as exc:
        assert expect in str(exc), exc
        print("runner refused as expected:", exc)
    finally:
        for k, v in saved.items():
            os.environ.pop(k, None)
            if v is not None:
                os.environ[k] = v

# 2. controller wrapper on reconstructed ticks
policy, schema = common.load_policy()
ticks = common.reconstruct_ticks(5)
from gwm_policy_control import candidate_ranker, control_node, observation_adapter  # noqa: E402
production_polar = observation_adapter.polar_profile
reference = []
for t in ticks:
    tick, _, state, rows, progress, ctx, grid, traversed, settings, _ = t
    _, r = candidate_ranker.PolicyRanker(policy, schema, float("inf")).choose(rows, progress, ctx, grid, traversed, state, settings)
    reference.append((r["scores"].copy(), r["choice_slot"]))
seen = []


def fake_main(argv):
    assert observation_adapter.polar_profile is not production_polar, "polar profile not swapped"
    ranker = candidate_ranker.PolicyRanker(policy, schema, 0.020)
    for t in ticks:
        tick, _, state, rows, progress, ctx, grid, traversed, settings, _ = t
        _, r = ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
        seen.append((r["scores"].copy(), r["choice_slot"], r["failure"]))
    return 0


control_node.main = fake_main
import diag_control  # noqa: E402
with tempfile.TemporaryDirectory() as tmp:
    rc = diag_control.main(["--variant", "D", "--run-dir", tmp])
    lines = [json.loads(x) for x in (Path(tmp) / "p5diag/sched.jsonl").read_text().splitlines()]
assert rc == 0 and lines[0]["contract"] == "p5-sched-diag-v1" and lines[0]["variant"] == "D", lines[0]
assert lines[0]["polar_profile"] == "optimized.polar_profile", lines[0]
calls = lines[1:]
assert len(calls) == len(ticks) and all(c["failure"] is None for c in calls), calls
keys = {"inference_ms", "cpu_ms", "runqueue_wait_ms", "timeslices", "ivcsw", "procs_running", "cpu_before"}
assert keys <= set(calls[0]), calls[0]
for (rs, rc_), (ss, sc, sf) in zip(reference, seen):
    assert np.array_equal(rs, ss) and rc_ == sc and sf is None
print("diag_control wrapper OK:", len(calls), "calls logged; scores and choices bit-identical to production;",
      "sample:", {k: calls[0][k] for k in sorted(keys)})
