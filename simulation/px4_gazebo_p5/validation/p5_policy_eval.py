"""P5 policy checks that need the frozen runtime bit for bit (contract §8; Gate A items 4 and 5, revision 2).

`p5_evaluate.py` runs this file under the controller runtime interpreter recorded in the manifest
(`/usr/bin/python3`, OPENBLAS_NUM_THREADS=1) and requires its NumPy to equal the controller's. Score
reproduction must be bit-identical, and another NumPy/BLAS build already differs by about 1e-14 (plan
§13.1), so the evaluator's own interpreter cannot do it. Read-only. Prints one JSON object with
metrics and structural failures; `p5_evaluate` applies the contract limits.

* flight: the policy ledger against the `p4_decision` events (completeness), bit-identical scores
  from the stored features, the masked argmax, the executed segment or heading, fallbacks, failures.
* fixture (item 4): on every recorded ground snapshot, the frozen input and model (finite,
  bit-identical, timed), the mask against P4's progress set, the P5 decision against P4's, and no
  policy-induced move into unknown or occupied space.
* flight_snapshot (item 5): from the accepted P4 S2 flight evidence (read-only, integrity-checked),
  the first TRANSIT tick in an avoidance state whose rebuilt inputs reproduce P4's recorded plan
  exactly; on it, six injected policy faults through the production ranker and supervisor, each of
  which must HOLD and then ABORT with `policy_unavailable`. No ROS or PX4 import: nothing can publish.
"""
import argparse
import contextlib
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
for path in (REPO / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
             REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance", P5 / "ros2_ws/src/gwm_policy_control"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import numpy as np  # noqa: E402

from gwm_obstacle_avoidance import local_map, planner  # noqa: E402
from gwm_policy_control import candidate_ranker, contracts, decision, observation_adapter, policy_ledger  # noqa: E402
from gwm_policy_control.candidate_ranker import TIE_EPSILON, PolicyRanker  # noqa: E402
from gwm_policy_control.policy_runtime import FrozenPolicy, PolicyError  # noqa: E402
from gwm_policy_control.safety_shield import PolicySupervisor  # noqa: E402

PROGRESS = (planner.TRANSLATE, planner.YAW_TO)
EXPECTED_FAILURE = {"missing_checkpoint": "checkpoint_missing", "hash_mismatch": "checkpoint_missing",
                    "nan_output": "output_not_finite", "timeout": "timeout", "output_shape": "output_shape",
                    "selection_outside_mask": "selection_outside_mask"}


def jsonl(path):
    path = Path(path)
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else None


def quantiles(values):
    ordered = sorted(values)

    def pick(q):  # p4_evaluate.pick
        return ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))] if ordered else None
    return {"count": len(ordered), "p50": pick(0.50), "p95": pick(0.95), "p99": pick(0.99),
            "max": ordered[-1] if ordered else None}


# ---- flight: the policy ledger -------------------------------------------------------------------------
def ledger_checks(run, policy, config):
    run = Path(run)
    records = jsonl(run / "p5/policy-ledger.jsonl")
    events = [e for e in (jsonl(run / "ros-events.jsonl") or []) if e.get("event") == "p4_decision"]
    failures = []
    if records is None:
        return {"failures": ["ledger:missing"], "records": 0, "decisions": len(events)}
    if len(records) != len(events):
        failures.append("ledger:record_count_differs_from_decisions")
    if [r.get("tick") for r in records] != list(range(1, len(records) + 1)):
        failures.append("ledger:tick_sequence")
    if any(r.get("ledger") != policy_ledger.LEDGER_CONTRACT for r in records):
        failures.append("ledger:contract")
    if any(r["sim_s"] != e["sim_s"] or r["action"] != e["action"] or r["state"] != e["state"]
           for r, e in zip(records, events)):
        failures.append("ledger:decision_mismatch")
    steps = config["planner"]["heading_steps"]
    c = {"invoked": 0, "failure_ticks": 0, "fallback_decisions": 0, "score_mismatches": 0, "argmax_mismatches": 0,
         "ties": 0, "outside_mask": 0, "executed_mismatches": 0, "participating": 0, "sha_mismatches": 0,
         "disagreements": 0}
    reasons, interventions, times, first_bad = {}, {}, [], {}
    for index, record in enumerate(records):
        action = record["action"]
        if not record["invoked"]:
            if action in PROGRESS:
                c["fallback_decisions"] += 1  # a progress action no policy decision produced
                first_bad.setdefault("fallback", record["tick"])
            continue
        c["invoked"] += 1
        times.append(record["inference_s"])
        if record.get("policy_sha256") != policy.sha256:
            c["sha_mismatches"] += 1
        if record["failure"] is not None:
            c["failure_ticks"] += 1
            reasons[record["failure"]] = reasons.get(record["failure"], 0) + 1
            if action in PROGRESS:
                c["fallback_decisions"] += 1
            continue
        scores = policy.scores(policy_ledger.decode_array(record["features"]))
        if [float(s) for s in scores] != record["scores"]:
            c["score_mismatches"] += 1
            first_bad.setdefault("score", record["tick"])
        allowed = np.array(record["allowed"], dtype=bool)
        choice = record["choice_slot"]
        if allowed.shape != (contracts.SLOTS,) or not 0 <= choice < contracts.SLOTS or not allowed[choice]:
            c["outside_mask"] += 1
            continue
        recorded = np.array(record["scores"], dtype=np.float64)
        top = recorded[allowed].max()
        winners = [int(i) for i in np.flatnonzero(allowed) if recorded[i] >= top - TIE_EPSILON]  # P4 tie order inside
        c["ties"] += len(winners) > 1
        if choice not in winners:
            c["argmax_mismatches"] += 1
            first_bad.setdefault("argmax", record["tick"])
        best = record["best"]
        consistent = best is not None and best["k"] == choice // 2 - steps
        if consistent and action == planner.TRANSLATE:
            consistent = record["end"] == list(best["end"]) and record["heading"] == best["heading"]
        elif consistent and action == planner.YAW_TO:
            consistent = record["heading"] == best["heading"]
        elif consistent and action == planner.HOLD:
            consistent = str(record["reason"]).startswith("yaw_not_permitted")  # the L5 shield, as in P4
        elif consistent:
            consistent = False
        if consistent:
            c["participating"] += 1
        else:
            c["executed_mismatches"] += 1
            first_bad.setdefault("executed", record["tick"])
        if choice != record["p4_slot"]:
            c["disagreements"] += 1
        kind = (record["intervention"] or "none").split(":")[0]
        interventions[kind] = interventions.get(kind, 0) + 1
    # Structural rules; the counted limits (failures, mask, fallbacks, participation, timing) are p5_evaluate's.
    for key, label in (("score_mismatches", "ledger:score_not_reproduced"), ("argmax_mismatches", "ledger:argmax_differs"),
                       ("executed_mismatches", "ledger:executed_differs_from_choice"),
                       ("sha_mismatches", "ledger:policy_sha256_differs")):
        if c[key]:
            failures.append(label)
    successful = c["invoked"] - c["failure_ticks"]
    return {"failures": failures, "records": len(records), "decisions": len(events), **c,
            "failure_reasons": reasons, "interventions": interventions, "first_bad_tick": first_bad,
            "participation_fraction": (c["participating"] / c["invoked"]) if c["invoked"] else None,
            "disagreement_rate": (c["disagreements"] / successful) if successful else None,
            "inference_s": quantiles(times)}


# ---- fixtures: the frozen ranker on recorded ground snapshots ----------------------------------------
def snapshot_context(row, config):
    """The P4 fixture evaluator's frame: vehicle at the recorded pose, goal 8 m ahead along its heading."""
    grid = local_map.Grid.decode(row["grid"])
    n, e, _ = row["position_ned"]
    vehicle = (e, n)
    q = row["q_wxyz"]
    heading = math.pi / 2 - math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    goal = (vehicle[0] + 8.0 * math.cos(heading), vehicle[1] + 8.0 * math.sin(heading))
    traversed = planner.traversed_mask(grid, [vehicle], config["envelope"]["r_body_m"])
    ctx = planner.Context(vehicle, vehicle, heading, goal, vehicle, 0.0, config["geofence"])
    return grid, traversed, ctx


def ground_evaluation(run, window, policy, schema, config, failure_wall_s):
    """Item 4: inference correctness and safety masking on real ground snapshots (not AI-directed motion)."""
    rows = [r for r in (jsonl(Path(run) / "p4/local_map.jsonl") or [])
            if r.get("usable") and window[0] <= r["processing_sim_s"] <= window[1]]
    settings, max_age = config["planner"], config["perception"]["snapshot_max_age_sim_s"]
    c = {"snapshots": len(rows), "non_finite": 0, "not_repeatable": 0, "mask_mismatches": 0, "decision_mismatches": 0,
         "policy_invocations": 0, "policy_failures": 0, "unsafe_policy_moves": 0}
    actions, times = {}, []
    for row in rows:
        grid, traversed, ctx = snapshot_context(row, config)
        candidates = planner.evaluate_candidates(grid, traversed, ctx, settings, max_age)
        start = time.perf_counter()
        features, allowed = observation_adapter.build(candidates, ctx, settings, grid, traversed, "NORMAL", schema)
        scores = policy.scores(features)
        times.append(time.perf_counter() - start)
        again, _ = observation_adapter.build(candidates, ctx, settings, grid, traversed, "NORMAL", schema)
        c["non_finite"] += not bool(np.isfinite(scores).all())
        c["not_repeatable"] += not (np.array_equal(again, features) and np.array_equal(policy.scores(again), scores))
        progress = {observation_adapter.slot_of(r, settings) for r in candidates
                    if r["valid"] and observation_adapter.progress_qualified(r, ctx, settings)[0]}
        c["mask_mismatches"] += set(np.flatnonzero(allowed).tolist()) != progress
        p4 = planner.plan(grid, traversed, ctx, settings, config["envelope"], max_age)
        p5 = decision.plan_with_ranker(grid, traversed, ctx, settings, config["envelope"], max_age,
                                       PolicyRanker(policy, schema, failure_wall_s), "NORMAL")
        record = p5.pop("policy")
        actions[p5["action"]] = actions.get(p5["action"], 0) + 1
        if record is None:
            c["decision_mismatches"] += p5 != p4
            continue
        c["policy_invocations"] += 1
        if record["failure"] is not None:
            c["policy_failures"] += 1
            continue
        if p5["action"] in PROGRESS:  # the executed candidate must be a P4-valid row in the mask
            chosen = candidates[record["choice_slot"]] if record["choice_slot"] is not None else None
            c["unsafe_policy_moves"] += not (chosen and chosen["valid"] and record["allowed"][record["choice_slot"]]
                                             and p5["heading"] == chosen["heading"])
    return {**c, "actions": actions, "inference_s": quantiles(times)}


# ---- flight snapshot: the accepted P4 S2 evidence, read-only -----------------------------------------------
def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_integrity(source):
    """The source run must be a passed row of the accepted P4 gate result, with its evaluation unchanged."""
    run, result = Path(source["run_dir"]), json.loads(Path(source["gate_result"]).read_text())
    row = next((r for r in result["runs"] if Path(r["run"]).name == run.name and r["role"] == source["role"]), None)
    accepted = (result.get("accepted") is True and result.get("manifest_sha256") == source["p4_manifest_sha256"]
                and row is not None and row.get("evaluation_status") == "passed"
                and sha256_file(run / "p4-evaluation.json") == row.get("evaluation_sha256"))
    read = ("p4-evaluation.json", "controller-result.json", "ros-events.jsonl", "p4/local_map.jsonl", "p4_avoidance.yaml")
    return {"run": run.name, "gate": source["gate"], "role": source["role"], "accepted": bool(accepted),
            "gate_result_sha256": sha256_file(source["gate_result"]),
            "files_sha256": {name: sha256_file(run / name) for name in read if (run / name).exists()}}


def reproduced_ticks(run):
    """Rebuild each recorded TRANSIT decision with progress candidates from the run's own records and keep
    the ones where the unchanged P4 planner reproduces the recorded plan field for field."""
    run = Path(run)
    config = json.loads((run / "p4_avoidance.yaml").read_text())
    result = json.loads((run / "controller-result.json").read_text())
    maps = {r["acquisition_sim_s"]: r for r in jsonl(run / "p4/local_map.jsonl") if r.get("usable") and r.get("grid")}
    origin = tuple(result["p4"]["origin_enu"])
    goal = (origin[0] + config["mission"]["goal_enu_m"][0], origin[1] + config["mission"]["goal_enu_m"][1])
    events = jsonl(run / "ros-events.jsonl")
    prestream = next(e["ros_sim_s"] for e in events if e.get("event") == "transition" and e.get("to") == "PRESTREAM_SAFE_SETPOINTS")
    max_age = config["perception"]["snapshot_max_age_sim_s"]
    positions, samples, checked = [], {}, 0
    for e in events:
        if e.get("event") == "sample" and e["ros_sim_s"] >= prestream:
            n, east, _ = e["sample"]["position"]
            if not positions or math.dist(positions[-1], (east, n)) >= 0.02:  # P4Mission._record_position
                positions.append((east, n))
            samples[e["ros_sim_s"]] = (e["sample"], len(positions))
        if e.get("event") != "p4_decision" or not (e.get("plan") or {}).get("candidates_progress"):
            continue
        row, sample = maps.get(e["snapshot_acquisition_sim_s"]), samples.get(e.get("ros_sim_s"))
        if row is None or sample is None:
            continue
        checked += 1
        grid = local_map.Grid.decode(row["grid"])
        traversed = planner.traversed_mask(grid, positions[:sample[1]], config["envelope"]["r_body_m"])
        ctx = planner.Context(e["target_enu"], e["vehicle_enu"], math.pi / 2 - sample[0]["yaw"], goal, origin,
                              e["sim_s"] - e["snapshot_acquisition_sim_s"], config["geofence"])
        plan = planner.plan(grid, traversed, ctx, config["planner"], config["envelope"], max_age)
        if (plan["candidates_valid"], plan["candidates_progress"]) == (e["plan"]["candidates_valid"], e["plan"]["candidates_progress"]) \
                and json.loads(json.dumps(plan["best"])) == e["plan"]["best"]:
            yield checked, e, row, grid, traversed, ctx, config


class NanPolicy:
    """The frozen policy with its output replaced by NaN (injected fault)."""

    def __init__(self, policy):
        self.policy, self.sha256 = policy, policy.sha256

    def scores(self, features):
        return np.full_like(self.policy.scores(features), np.nan)


class ShortPolicy(NanPolicy):
    """The frozen policy with one output dropped (injected invalid output dimensions)."""

    def scores(self, features):
        return self.policy.scores(features)[:-1]


@contextlib.contextmanager
def masked_selection():
    """Injected invalid candidate selection: the selection step returns a slot outside the mask."""
    original = candidate_ranker.policy_choice
    candidate_ranker.policy_choice = lambda scores, allowed, rows: int(np.flatnonzero(~allowed)[0])
    try:
        yield
    finally:
        candidate_ranker.policy_choice = original


def fault_replay(event, row, grid, traversed, ctx, config, policy, schema, model_path, schema_sha, inference):
    """Each injected fault through the production ranker and supervisor, from the recorded supervisor state.
    Snapshot freshness and adapter health are held at the recorded tick so only the policy rule acts."""
    failures_to_abort = inference["consecutive_failures_to_abort"]
    identity, cases = row.get("calibration_identity"), {}
    for case, expected in EXPECTED_FAILURE.items():
        load_refusal, ranked, clock, injection = None, policy, time.perf_counter, contextlib.nullcontext()
        if case == "missing_checkpoint":
            ranked = None
        elif case == "hash_mismatch":
            try:
                FrozenPolicy.load(model_path, "0" * 64, schema_sha, require_trained=True)
            except PolicyError as exc:
                load_refusal = exc.reason
            ranked = None  # a refused checkpoint never becomes a policy
        elif case == "nan_output":
            ranked = NanPolicy(policy)
        elif case == "timeout":
            ticks = itertools.count(0.0, 0.25)
            clock = lambda: next(ticks)  # noqa: E731 - each inference appears to take 0.25 s
        elif case == "output_shape":
            ranked = ShortPolicy(policy)
        elif case == "selection_outside_mask":
            injection = masked_selection()
        supervisor = PolicySupervisor(config, event["sim_s"] - 10.0, PolicyRanker(ranked, schema, inference["failure_wall_s"], clock),
                                      failures_to_abort)
        supervisor.state = event["state"]
        supervisor.hover_since = event["sim_s"] if event["state"] == "HOVER" else None
        outs = []
        with injection:
            for k in range(failures_to_abort):
                t = event["sim_s"] + 0.05 * k
                outs.append(supervisor.step({
                    "sim": t, "snapshot": {"acquisition_sim_s": event["snapshot_acquisition_sim_s"] + 0.05 * k,
                                           "calibration_identity": identity},
                    "grid": grid, "traversed": traversed,
                    "adapter_health": {"status": "fresh", "calibration_identity": identity, "sim_s": t},
                    "consecutive_unusable": 0, "reset_sim": None, "vehicle": ctx.vehicle, "vehicle_yaw": ctx.vehicle_yaw,
                    "target": ctx.target, "goal": ctx.goal, "origin": ctx.origin, "geofence": config["geofence"]}))
        holds, last = outs[:-1], outs[-1]
        recorded = [(o["policy"] or {}).get("failure") for o in outs]
        passed = (all(o["state"] == event["state"] and o["action"] == planner.HOLD and o["frozen_yaw"]
                      and o["reason"] == "policy_failure:" + expected for o in holds)
                  and last["state"] == "ABORT" and last["reason"] == "policy_unavailable" and last["frozen_yaw"]
                  and recorded == [expected] * failures_to_abort
                  and (case != "hash_mismatch" or load_refusal == "checkpoint_sha256_mismatch"))
        cases[case] = {"expected_failure": expected, "load_refusal": load_refusal, "recorded_failures": recorded,
                       "states": [o["state"] for o in outs], "actions": [o["action"] for o in outs],
                       "reasons": [o["reason"] for o in outs], "passed": bool(passed)}
    return cases


def flight_snapshot_replay(sources, policy, schema, model_path, schema_sha, inference, avoidance_states):
    out = {"sources": [source_integrity(s) for s in sources], "selected": None, "cases": None}
    for source, integrity in zip(sources, out["sources"]):
        if not integrity["accepted"]:
            continue
        checked = 0
        for checked, event, row, grid, traversed, ctx, config in reproduced_ticks(source["run_dir"]):
            if event["state"] not in avoidance_states:
                continue
            settings = config["planner"]
            candidates = planner.evaluate_candidates(grid, traversed, ctx, settings, config["perception"]["snapshot_max_age_sim_s"])
            progress = [r for r in candidates if r["valid"] and observation_adapter.progress_qualified(r, ctx, settings)[0]]
            features, allowed = observation_adapter.build(candidates, ctx, settings, grid, traversed, event["state"], schema)
            _, record = PolicyRanker(policy, schema, inference["failure_wall_s"]).choose(
                candidates, progress, ctx, grid, traversed, event["state"], settings)
            out["selected"] = {
                "run": integrity["run"], "sim_s": event["sim_s"], "state": event["state"], "recorded_action": event["action"],
                "snapshot_acquisition_sim_s": event["snapshot_acquisition_sim_s"], "progress_candidates": len(progress),
                "masked_slots": int((~allowed).sum()), "decisions_checked_before": checked,
                "mask_equals_p4_progress": set(np.flatnonzero(allowed).tolist()) == {observation_adapter.slot_of(r, settings) for r in progress},
                "true_policy": {"failure": record["failure"], "choice_slot": record["choice_slot"], "p4_slot": record["p4_slot"],
                                "finite": bool(np.isfinite(policy.scores(features)).all()),
                                "inside_mask": record["choice_slot"] is not None and bool(allowed[record["choice_slot"]])}}
            out["cases"] = fault_replay(event, row, grid, traversed, ctx, config, policy, schema, model_path, schema_sha, inference)
            return out
    return out


# ---- entry -----------------------------------------------------------------------------------------------
def evaluate(run, mode, model_path, model_sha256, schema_path, window=None):
    run = Path(run)
    schema = contracts.load_schema(schema_path)
    schema_sha = contracts.schema_sha256(schema)
    policy = FrozenPolicy.load(model_path, model_sha256, schema_sha, require_trained=True)
    out = {"contract": "p5-policy-evaluation-v1", "mode": mode, "numpy": np.__version__, "python": sys.executable,
           "model_sha256": policy.sha256, "schema_sha256": schema_sha}
    p5 = json.loads((run / "p5_policy.yaml").read_text())
    if mode == "flight_snapshot":
        summary, v5 = json.loads((run / "summary.json").read_text()), json.loads((run / "p5_validation.yaml").read_text())
        out["flight_snapshot"] = flight_snapshot_replay(summary["replay_sources"], policy, schema, model_path, schema_sha,
                                                        p5["inference"], v5["policy_replay"]["avoidance_states"])
        return out
    config = json.loads((run / "p4_avoidance.yaml").read_text())
    if mode == "flight":
        out["ledger"] = ledger_checks(run, policy, config)
    else:
        out["ground"] = ground_evaluation(run, window, policy, schema, config, p5["inference"]["failure_wall_s"])
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", type=Path)
    parser.add_argument("--mode", choices=("flight", "fixture", "flight_snapshot"), required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--model-sha256", required=True)
    parser.add_argument("--schema", type=Path, required=True)
    parser.add_argument("--window", type=float, nargs=2)
    args = parser.parse_args(argv)
    print(json.dumps(evaluate(args.run, args.mode, args.model, args.model_sha256, args.schema, args.window),
                     allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
