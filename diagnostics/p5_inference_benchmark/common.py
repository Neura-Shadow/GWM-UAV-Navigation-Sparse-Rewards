"""Shared, read-only set-up for the P5 inference benchmark (diagnostics only; never imported by the controller).

Loads the frozen P5 checkpoint through the production `FrozenPolicy.load` (SHA-256, architecture, schema and
`trained` checks), and rebuilds policy decision ticks from the retained Gate C S2 flight evidence with the
evaluator's `reproduced_ticks`: the first 120 TRANSIT ticks whose rebuilt inputs reproduce P4's recorded plan
field for field (the same selection as the plan §13.15 probe). The spike ticks that cannot be rebuilt (491 and
337) are taken from the policy ledger, which stores their exact feature tensor, mask and scores.
"""
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
P5 = ROOT / "simulation/px4_gazebo_p5"
for path in (ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
             ROOT / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
             P5 / "ros2_ws/src/gwm_policy_control", P5 / "validation"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

RUNS = Path(os.environ.get("GWM_RUNS", Path.home() / "uav_autonomy/runs"))
GATE_C_S2 = "20261031T121719Z-p5-flight-S2-2e9825f4"
SPIKE_TICKS = (491, 337)  # ledger ticks of the Gate C S2 tail (10.84 ms and 4.22 ms in flight)
MODEL = next((P5 / "models").glob("p5-ranker-*.npz"))
MODEL_SHA256 = MODEL.stem[len("p5-ranker-"):]
RESULTS = Path(__file__).resolve().parent / "results"
CONFIGS = {}  # ledger tick -> the run's P4 avoidance config, filled by reconstruct_ticks


def load_policy():
    from gwm_policy_control import contracts
    from gwm_policy_control.policy_runtime import FrozenPolicy
    schema = contracts.load_schema(P5 / "configs/p5_model_schema.json")
    return FrozenPolicy.load(MODEL, MODEL_SHA256, contracts.schema_sha256(schema), require_trained=True), schema


def reconstruct_ticks(limit=120, run=GATE_C_S2):
    """[(ledger_tick, sim_s, state, rows, progress, ctx, grid, traversed, settings, recorded)] from retained evidence."""
    from gwm_obstacle_avoidance import planner
    from gwm_policy_control.observation_adapter import progress_qualified
    import p5_policy_eval
    run_dir = RUNS / run
    ledger = {round(r["sim_s"], 3): r for r in map(json.loads, (run_dir / "p5/policy-ledger.jsonl").open())
              if r.get("invoked")}
    ticks = []
    for _, event, _row, grid, traversed, ctx, config in p5_policy_eval.reproduced_ticks(run_dir):
        recorded = ledger.get(round(event["sim_s"], 3))
        if recorded is None:
            continue
        settings = config["planner"]
        rows = planner.evaluate_candidates(grid, traversed, ctx, settings, config["perception"]["snapshot_max_age_sim_s"])
        progress = [r for r in rows if r["valid"] and progress_qualified(r, ctx, settings)[0]]
        ticks.append((recorded["tick"], event["sim_s"], event["state"], rows, progress, ctx, grid, traversed, settings, recorded))
        CONFIGS[recorded["tick"]] = config
        if len(ticks) >= limit:
            break
    return ticks


def ledger_ticks(ticks=SPIKE_TICKS, run=GATE_C_S2):
    """{tick: (features, allowed, recorded_scores, recorded_choice)} straight from the policy ledger
    (ticks=None: every invoked record)."""
    import numpy as np
    from gwm_policy_control.policy_ledger import decode_array
    out = {}
    for record in map(json.loads, (RUNS / run / "p5/policy-ledger.jsonl").open()):
        if (ticks is None or record["tick"] in ticks) and record.get("invoked") and record.get("features"):
            out[record["tick"]] = (decode_array(record["features"]), np.array(record["allowed"], dtype=bool),
                                   np.array(record["scores"], dtype=np.float64), record["choice_slot"])
    return out


def quantiles(values):
    v = sorted(values)
    if not v:
        return None
    pick = lambda q: v[min(len(v) - 1, int(q * len(v)))]  # noqa: E731 - nearest rank, as the evaluator
    return {"n": len(v), "p50": pick(0.50), "p95": pick(0.95), "p99": pick(0.99), "max": v[-1]}
