"""Independent offline P5 evaluator for one finalized run directory (contract §6, §7, §8, §9).

Never launches anything; output is exclusive-create. It composes the unchanged P4 evaluator
functions (P3 stream and association, RGB, P2 integrity and timing, Policy A, renderer, MC causality,
cleanup, §6 truth safety, response latency, scenario outcomes, the S6 fault window, §6.1 performance,
ground-fixture reconstruction) with the P5 checks: controller identity `p5_control`, the checkpoint
identity at start against the manifest, and the policy ledger and policy replays. Those replays run
under the controller runtime interpreter (`p5_policy_eval.py`), so scores reproduce bit for bit.
Never trusts P5 self-reports.

* p3_readiness: the unchanged P3 readiness evaluation.
* p5_readiness / fixture: P3 sensor checks, ground-diagnostic integrity, §6.1 performance; fixture adds
  P4's reconstruction and trigger checks and the revision 2 ground policy evaluation (frozen input and
  model, bit-exact, timed; mask and decision equal to P4's; no policy-induced move).
* flight_snapshot_replay (offline Gate A row): the revision 2 injected-fault replay on an exactly
  reproduced TRANSIT tick of the accepted P4 S2 flights (HOLD, HOLD, then `policy_unavailable`).
* policy_fault: the mismatched checkpoint was refused before any node existed; the rest of the stack
  stayed nominal.
* flight: as P4 (P3, P2, §6 safety, §6.1, outcome, §9.1 for S6), plus the policy-ledger rules and the
  §6 P5 limits, plus the reported decision-quality and resource figures.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
P4 = REPO / "simulation/px4_gazebo_p4"
SIM = REPO / "simulation/px4_gazebo"
for path in (SIM / "validation", SIM / "ros2_ws/src/gwm_px4_control", SIM / "ros2_ws/src/gwm_sensor_adapter",
             P4 / "ros2_ws/src/gwm_obstacle_avoidance", P4 / "validation", P5 / "validation"):
    if str(path) not in sys.path:
        sys.path.append(str(path))

import p4_evaluate as p4e  # noqa: E402
from p3_provenance import atomic_json, digest, strict_json, strict_jsonl  # noqa: E402
from p3_stream_eval import Failures, evaluate_rgb, evaluate_rgb_absence, evaluate_stream, native_rtf  # noqa: E402

EVALUATOR_FILES = ("p5_evaluate.py", "p5_policy_eval.py")
POLICY_EVAL = P5 / "validation/p5_policy_eval.py"
CONTROLLER = "p5_control"
REFUSED_CONTROLLER_CLEANUP = "cleanup:ros-controller:nonzero_exit_1"  # the designed policy_fault refusal


def evaluator_identity():
    files = {name: digest(P5 / "validation" / name) for name in EVALUATOR_FILES}
    files.update({"p4:" + k: v for k, v in p4e.evaluator_identity().items()})
    return files


def events_of(run):
    path = Path(run) / "ros-events.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


# ---- the policy replay, under the controller runtime interpreter --------------------------------------------
def replay(run, summary, mode, window=None):
    """`p5_policy_eval.py` under the manifest's controller runtime (`python`, NumPy, single-threaded BLAS)."""
    runtime, model = summary["controller_runtime"], summary["p5_model"]
    command = [runtime["python"], "-I", "-s", str(POLICY_EVAL), str(run), "--mode", mode,
               "--model", str(P5 / model["path"]), "--model-sha256", model["sha256"],
               "--schema", str(Path(run) / "p5_model_schema.json")]
    if window is not None:
        command += ["--window", repr(float(window[0])), repr(float(window[1]))]
    env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "OPENBLAS_NUM_THREADS": runtime["openblas_num_threads"]}
    completed = subprocess.run(command, env=env, text=True, capture_output=True, timeout=900)
    if completed.returncode != 0:
        raise ValueError("policy_replay_failed:" + completed.stderr.strip()[-400:])
    return json.loads(completed.stdout)


def replay_runtime(replayed, summary, f):
    """The replay must run on the controller's own NumPy, or bit-identity means nothing."""
    controller = ((summary.get("controller_result") or {}).get("p5") or {}).get("numpy")
    f.check(replayed["numpy"] == summary["controller_runtime"]["numpy"] and controller in (None, replayed["numpy"]),
            "policy:replay_numpy_differs")
    f.check(replayed["model_sha256"] == summary["p5_model"]["sha256"]
            and replayed["schema_sha256"] == summary["p5_model"]["schema_sha256"], "policy:replay_model_differs")


# ---- P5 checks ------------------------------------------------------------------------------------------------
def checkpoint_identity(run, summary, f):
    """Contract §8: the controller verified, at start, exactly the checkpoint the manifest binds."""
    model = summary.get("p5_model") or {}
    p5 = (summary.get("controller_result") or {}).get("p5") or {}
    verified = [e for e in events_of(run) if e.get("event") == "p5_policy_verified"]
    out = {"manifest_model_sha256": model.get("sha256"), "controller_model_sha256": p5.get("model_sha256"),
           "verification_events": len(verified), "controller_numpy": p5.get("numpy")}
    f.check(len(verified) == 1, "policy:checkpoint_verification_event")
    f.check(bool(model.get("sha256")) and p5.get("model_sha256") == model.get("sha256")
            and all(e.get("model_sha256") == model.get("sha256") for e in verified), "policy:checkpoint_sha256_differs_from_manifest")
    f.check(p5.get("trained") is True and p5.get("architecture") == model.get("architecture")
            and p5.get("schema_sha256") == model.get("schema_sha256") == summary.get("schema_sha256"), "policy:checkpoint_identity")
    f.check(bool(model.get("path")) and (P5 / model["path"]).exists() and digest(P5 / model["path"]) == model.get("sha256"),
            "policy:bound_artifact_changed")
    f.check(p5.get("numpy") == (summary.get("controller_runtime") or {}).get("numpy"), "policy:controller_numpy_differs_from_manifest")
    return out


def policy_ledger(replayed, summary, limits, f):
    """The ledger rules (structural, from the replay) and the §6 P5 limits on the recorded decisions."""
    ledger = replayed["ledger"]
    f.items += ["policy:" + item for item in ledger["failures"]]
    timing = ledger.get("inference_s") or {}
    f.check(ledger.get("records", 0) > 0 and ledger.get("invoked", 0) > 0, "policy:no_policy_decisions")
    f.check(ledger.get("failure_ticks", 0) <= limits["failure_ticks_allowed"], "policy:failure_ticks")
    f.check(ledger.get("outside_mask", 0) <= limits["outside_mask_allowed"], "policy:outside_mask")
    f.check(ledger.get("fallback_decisions", 0) <= limits["fallback_decisions_allowed"], "policy:fallback_decisions")
    f.check(ledger.get("participation_fraction") is not None and ledger["participation_fraction"] >= limits["participation_fraction"],
            "policy:participation")
    f.check(timing.get("p99") is not None and timing["p99"] <= limits["inference_p99_s"], "policy:inference_p99")
    f.check(timing.get("max") is not None and timing["max"] <= limits["inference_max_s"], "policy:inference_max")
    return {k: v for k, v in ledger.items() if k != "failures"}


def ground_ledger(run, f):
    """Ground scopes never reach TRANSIT: the ledger exists and is empty, and no decision was made."""
    rows = strict_jsonl(Path(run) / "p5/policy-ledger.jsonl") if (Path(run) / "p5/policy-ledger.jsonl").exists() else None
    decisions = [e for e in events_of(run) if e.get("event") == "p4_decision"]
    f.check(rows == [] and not decisions, "policy:ground_scope_made_decisions")


def policy_ground(replayed, limits, f):
    """Gate A item 4 (revision 2): inference correctness and safety masking on real ground snapshots."""
    g = replayed["ground"]
    timing = g.get("inference_s") or {}
    f.check(g["snapshots"] > 0, "policy_ground:no_snapshots")
    f.check(g["non_finite"] == 0, "policy_ground:non_finite_scores")
    f.check(g["not_repeatable"] == 0, "policy_ground:not_bit_exact")
    f.check(g["mask_mismatches"] == 0, "policy_ground:mask_differs_from_p4")
    f.check(g["decision_mismatches"] == 0, "policy_ground:decision_differs_from_p4")
    f.check(g["policy_failures"] == 0, "policy_ground:policy_failure")
    f.check(g["unsafe_policy_moves"] == 0, "policy_ground:unsafe_policy_move")
    f.check(g["policy_invocations"] > 0 or set(g["actions"]) <= {"SCAN", "HOLD"}, "policy_ground:move_without_progress_candidate")
    f.check(timing.get("p99") is not None and timing["p99"] <= limits["inference_p99_s"], "policy_ground:inference_p99")
    f.check(timing.get("max") is not None and timing["max"] <= limits["inference_max_s"], "policy_ground:inference_max")
    return g


def flight_snapshot(replayed, summary, cases, f):
    """Gate A item 5 (revision 2): faults on an exactly reproduced TRANSIT tick of the accepted P4 S2 flights."""
    fs = replayed["flight_snapshot"]
    recorded = {s["run"]: s["files_sha256"] for s in summary["replay_sources"]}
    for source in fs["sources"]:
        f.check(source["accepted"], "policy_replay:source_not_accepted:" + source["run"])
        f.check(source["files_sha256"] == recorded.get(source["run"]), "policy_replay:source_changed:" + source["run"])
    selected = fs["selected"]
    f.check(selected is not None, "policy_replay:no_reproducible_valid_candidate_snapshot")
    if selected is not None:
        true = selected["true_policy"]
        f.check(selected["progress_candidates"] > 0 and selected["mask_equals_p4_progress"], "policy_replay:mask_differs_from_p4")
        f.check(true["failure"] is None and true["finite"] and true["inside_mask"], "policy_replay:true_policy")
    for case in cases:
        f.check(bool(fs["cases"]) and fs["cases"].get(case, {}).get("passed") is True, "policy_replay:" + case)
    return fs


def evaluate_flight_snapshot(run, summary, v5, out):
    """The offline Gate A replay row: no simulator, no process, no PX4 command path."""
    f = Failures()
    f.check(summary.get("frozen_manifest_sha256") is not None, "unfrozen_runtime")
    f.check(summary.get("finalized") is True and summary.get("status") == "passed", "replay_run_not_finalized")
    f.check(summary.get("simulator_started") is False and summary.get("px4_commands_possible") is False,
            "replay_run_not_offline")
    try:
        replayed = replay(run, summary, "flight_snapshot")
        replay_runtime(replayed, summary, f)
        out["flight_snapshot"] = flight_snapshot(replayed, summary, v5["policy_replay"]["failure_cases"], f)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        f.check(False, "policy_replay_error:" + type(exc).__name__ + ":" + str(exc))
    return finish(out, f)


def policy_fault(run, summary, refusal, f):
    """Gate A: one live start with a mismatched checkpoint refused before any node, file or publisher."""
    run = Path(run)
    fault = summary.get("policy_fault") or {}
    model = summary.get("p5_model") or {}
    f.check(fault.get("kind") == "checkpoint_mismatch" and fault.get("expected_sha256") == model.get("sha256")
            and fault.get("actual_sha256") not in (None, model.get("sha256")), "policy_fault:not_a_checkpoint_mismatch")
    path = Path(fault.get("model_path") or run / "missing")
    f.check(path.exists() and digest(path) == fault.get("actual_sha256"), "policy_fault:mismatched_checkpoint_record")
    f.check(summary.get("controller_exit_code") not in (None, 0), "policy_fault:controller_did_not_refuse")
    started = [name for name in ("controller-result.json", "ros-events.jsonl", "p5/policy-ledger.jsonl", "ros-graph.json",
                                 "rosbag/metadata.yaml", "timing-trace.jsonl") if (run / name).exists()]
    f.check(not started, "policy_fault:controller_started")
    log = (run / "ros-controller.log").read_text(errors="replace") if (run / "ros-controller.log").exists() else ""
    f.check(refusal in log, "policy_fault:refusal_reason")
    return {**fault, "started_artifacts": started, "refusal_found": refusal in log}


def policy_fault_native_commands(run, summary, f, out):
    """Policy A with no controller: no landing window exists, so any landing command is a violation."""
    from p3_mavlink import evaluate_policy
    closed = strict_json(Path(run) / "mavlink/closed.json")
    effective = summary["native_command_policy"]
    startup_end = summary["startup"]["origin_monotonic_ns"] + int(effective["policy"].get("startup_wall_s", 0) * 1e9)
    out["native_commands"] = evaluate_policy(closed, effective, startup_end, None)
    f.items += ["native_commands:" + x for x in out["native_commands"]["failures"]]


def reported(run, summary, events, track):
    """Contract §6 'reported, not limited': decision quality, latency, publication gaps and resources."""
    result = summary.get("controller_result") or {}
    transit = next((t["sim_s"] for t in result.get("transitions", []) if t["to"] == "TRANSIT"), None)
    goal_at = (result.get("p4") or {}).get("goal_reached_sim_s")
    airborne = [p for _, p in track if p[2] > 0.5]
    path = sum(math.dist(a[:2], b[:2]) for a, b in zip(airborne, airborne[1:]))
    decisions = [e for e in events if e.get("event") == "p4_decision" and e.get("snapshot_acquisition_sim_s") is not None]
    ages = [d["sim_s"] - d["snapshot_acquisition_sim_s"] for d in decisions]
    stamps = [e["ros_sim_s"] for e in events if e.get("event") == "setpoint" and e.get("phase") == "TRANSIT"]
    resources = [r for r in (strict_jsonl(Path(run) / "resources.jsonl") if (Path(run) / "resources.jsonl").exists() else [])
                 if "ros-controller" in r.get("processes", {})]
    cpu = None
    if len(resources) >= 2:
        first, last = resources[0], resources[-1]
        span = last["monotonic_s"] - first["monotonic_s"]
        cpu = (last["processes"]["ros-controller"]["cpu_s"] - first["processes"]["ros-controller"]["cpu_s"]) / span if span > 0 else None
    return {"path_length_m": path if airborne else None, "detour_ratio": path / 8.0 if airborne else None,
            "time_to_goal_sim_s": goal_at - transit if goal_at is not None and transit is not None else None,
            "decision_snapshot_age_sim_s": {"p50": p4e.pick(ages, 0.5), "max": max(ages, default=None)},
            "transit_setpoint_gap_max_sim_s": max((b - a for a, b in zip(stamps, stamps[1:])), default=None),
            "controller_cpu_fraction": cpu,
            "controller_rss_max_kb": max((r["processes"]["ros-controller"]["rss_kb"] for r in resources), default=None)}


# ---- entry ------------------------------------------------------------------------------------------------------
def evaluate(run):
    import p3_evaluate
    run = Path(run)
    summary = strict_json(run / "summary.json")
    scope = summary.get("p5_scope")
    if scope == "p3_readiness":
        out = p3_evaluate.evaluate(run)
        out.update(contract="p5-offline-evaluation-v1", p5_scope=scope, p3_evaluation_contract="p3-offline-evaluation-v1")
        return out
    v4 = strict_json(P4 / "configs/p4_validation.yaml")
    v5 = strict_json(P5 / "configs/p5_validation.yaml")
    out = {"contract": "p5-offline-evaluation-v1", "run_id": run.name, "p5_scope": scope, "gate": summary.get("gate"),
           "frozen_manifest_sha256": summary.get("frozen_manifest_sha256"), "p3_baseline": summary.get("p3_baseline"),
           "p4_baseline": summary.get("p4_baseline"), "model_sha256": (summary.get("p5_model") or {}).get("sha256"),
           "evaluator_sha256": evaluator_identity(), "runtime_status": summary.get("status"),
           "runtime_failure": summary.get("failure")}
    if scope == "flight_snapshot_replay":
        return evaluate_flight_snapshot(run, summary, v5, out)
    validation_p3, sensors = p3_evaluate.configs()
    config = json.loads((run / "p4_avoidance.yaml").read_text())
    f = Failures()
    flight = scope == "flight"
    f.check(summary.get("status") == "passed", "runtime_status:" + str(summary.get("failure")))
    p3_evaluate.common(run, summary, f, out)
    if scope == "policy_fault":
        refused = REFUSED_CONTROLLER_CLEANUP in f.items
        f.items = [item for item in f.items if item != REFUSED_CONTROLLER_CLEANUP]
        f.check(refused, "policy_fault:refusal_exit_not_recorded_by_cleanup")
        p3_evaluate.mandatory_artifacts(run, f, ("mavlink/closed.json", "source/probe-closed.json", "sensors/adapter-result.json",
                                                 "p4/local_map.jsonl", "p4/perception-result.json", "ros-controller.log"))
    else:
        p3_evaluate.mandatory_artifacts(run, f, ("controller-result.json", "ros-events.jsonl", "rosbag/metadata.yaml",
                                                 "mavlink/closed.json", "source/probe-closed.json", "sensors/adapter-result.json",
                                                 "p4/local_map.jsonl", "p4/perception-result.json", "p5/policy-ledger.jsonl"))
    f.check(summary.get("p3_baseline", {}).get("sha256") == v5["p3_baseline_manifest_sha256"], "p3_baseline_mismatch")
    f.check(summary.get("p4_baseline", {}).get("sha256") == v5["p4_baseline_manifest_sha256"], "p4_baseline_mismatch")
    declaration = strict_json(run / "p4/scenario.json") if (run / "p4/scenario.json").exists() else None
    is_s6 = bool(declaration and declaration.get("fault") == "bridge_stop_at_distance")
    f.check((run / "p4/fault_injection.json").exists() == is_s6, "fault_injection_outside_s6")
    window = summary["sensor_window_sim_s"]
    try:
        if is_s6:
            out["fault_window"] = p4e.fault_window(run, summary, validation_p3, sensors, v4["outcomes"], f)
        else:
            # State association proves reset brackets against the controller's received-PX4 ledger; a refused
            # controller never creates it (Gate A baef37a0), so the policy_fault scope checks the stream without it.
            stream = evaluate_stream(run, window, validation_p3, sensors, require_association=scope != "policy_fault")
            out["depth_stream"] = {"status": stream["status"], "failures": stream["failures"],
                                   "delivery": stream.get("delivery"), "processing": stream.get("processing")}
            f.items += ["depth:" + x for x in stream["failures"]]
        perf_window = [window[0], out["fault_window"]["pre_window_sim_s"][1]] if is_s6 and out.get("fault_window") else window
        out["performance"] = p4e.performance(run, perf_window, v4["performance"], f)
        if flight:
            transitions = {t["to"]: t["monotonic_s"] for t in summary["controller_result"]["transitions"]}
            end = transitions.get("COMPLETE") or transitions.get("RECOVERY")
            out["rgb_workload"] = "depth_only_flight"
            out["rgb_absence"] = evaluate_rgb_absence(run, (transitions["PRESTREAM_SAFE_SETPOINTS"], end),
                                                      validation_p3["rgb_absence_monitor"],
                                                      [p["pid"] for p in summary["processes"] if p.get("pid")])
            f.items += ["rgb_absence:" + x for x in out["rgb_absence"]["failures"]]
        else:
            out["rgb_stream"] = evaluate_rgb(run, window, validation_p3, sensors["rgb_sensor"])
            f.items += ["rgb:" + x for x in out["rgb_stream"]["failures"]]
    except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
        f.check(False, "sensor_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
    renderer = summary.get("renderer") or {}
    identity = strict_json(SIM / "configs/p3_d3d12_profile.json")["renderer_identity"]
    f.check(renderer.get("identified") is True and renderer.get("software") is False
            and identity in renderer.get("strings", []), "renderer_not_declared_d3d12")
    out["rtf"] = native_rtf(run)
    if scope == "policy_fault":
        try:
            out["policy_fault"] = policy_fault(run, summary, v5["policy_fault"]["checkpoint_mismatch"]["refusal"], f)
            policy_fault_native_commands(run, summary, f, out)
        except (ValueError, KeyError, FileNotFoundError) as exc:
            f.check(False, "policy_fault_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
        return finish(out, f)
    p2_report = None
    try:
        p2_report, _, _, _ = p4e.p2_integrity(run, summary, f, flight)
        out["p2_integrity"] = {k: v for k, v in p2_report.items() if k != "timing"}
        out["p2_timing_status"] = p2_report["timing"].get("status")
    except (ValueError, KeyError, FileNotFoundError, StopIteration, IndexError) as exc:
        f.check(False, "p2_integrity_error:" + type(exc).__name__ + ":" + str(exc))
    try:
        p4e.native_commands(run, summary, f, out)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        f.check(False, "native_command_evaluation_error:" + str(exc))
    p4e.mc(run, summary, f, out, flight, p2_report)
    f.check(summary.get("controller_publishers_only_from_controller") is True, "competing_flight_publisher")
    result = summary.get("controller_result") or {}
    f.check(result.get("controller") == CONTROLLER, "controller_not_p5")
    out["checkpoint"] = checkpoint_identity(run, summary, f)
    try:
        if flight:
            safety_out, offset, events = p4e.safety(run, declaration, v4["safety"], config["envelope"], f)
            out["safety"] = safety_out
            out["response_latency_sim_s"] = p4e.response_latency(events, v4["safety"]["response_latency_sim_s"], f)
            if not is_s6 and offset is not None:
                out["outcome"] = p4e.outcome(run, summary, declaration, events, offset, p4e.truth_track(run), v4["outcomes"], f)
            elif is_s6:
                out["outcome"] = {"label": "safe_stop_on_sensor_loss" if not f.items else "failed", "scenario": "S6"}
            replayed = replay(run, summary, "flight")
            replay_runtime(replayed, summary, f)
            out["policy"] = policy_ledger(replayed, summary, v5["policy"], f)
            out["reported"] = reported(run, summary, events, p4e.truth_track(run))
        else:
            ground_ledger(run, f)
            if scope == "fixture":
                out["fixture"] = p4e.fixture(run, declaration, summary, config, v4["ground_fixture"], f)
                replayed = replay(run, summary, "fixture", window)
                replay_runtime(replayed, summary, f)
                out["policy_ground"] = policy_ground(replayed, v5["policy"], f)
    except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
        f.check(False, "p5_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
    return finish(out, f)


def finish(out, f):
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    out["acceptance_credit"] = out["status"] == "passed"
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args(argv)
    output = args.run / "p5-evaluation.json"
    if output.exists():
        raise SystemExit("Evaluation already exists; the first evaluation is retained")
    try:
        result = evaluate(args.run)
    except Exception as exc:  # incomplete evidence is recorded as failed, never missing
        summary = {}
        try:
            summary = strict_json(args.run / "summary.json")
        except Exception:
            pass
        result = {"contract": "p5-offline-evaluation-v1", "run_id": args.run.name, "gate": summary.get("gate"),
                  "runtime_status": summary.get("status"), "runtime_failure": summary.get("failure"),
                  "failures": ["runtime_status:" + str(summary.get("failure")),
                               "evaluator_incomplete_evidence:" + type(exc).__name__ + ":" + str(exc)],
                  "status": "failed", "acceptance_credit": False}
    atomic_json(output, result, exclusive=True)
    print(result["status"], len(result["failures"]), "failures", flush=True)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
