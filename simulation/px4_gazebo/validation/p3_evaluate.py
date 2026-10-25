"""Independent offline P3 acceptance evaluator for one finalized runtime directory.

Never launches a simulator, ROS node or flight; never replays command topics.
Composition (each part recomputes from raw evidence):

* fixture   - analytic geometry, stream accounting, interruption/scene change;
* readiness - unchanged P2 read-only evaluator, six-source freshness, depth and
              RGB stream, native-command policy, MC acquisition, RTF, cleanup;
* flight    - unchanged P2 flight evaluator (ACK, windows, yaw, LAND), depth
              stream with PX4 state association, RGB, exact MC causal yaw,
              native-command policy and strict cleanup.

Missing data never becomes zero error or PASS. Output is exclusive-create.
"""
import argparse
from pathlib import Path
import sys

VALIDATION = Path(__file__).resolve().parent
SIM = VALIDATION.parent
sys.path.insert(0, str(VALIDATION))

from p3_provenance import atomic_json, digest, strict_json  # noqa: E402
from p3_process_exit import classify_all  # noqa: E402
from p3_stream_eval import Failures, evaluate_rgb, evaluate_stream, native_rtf  # noqa: E402

EVALUATOR_FILES = ("p3_evaluate.py", "p3_stream_eval.py", "p3_geometry_eval.py", "p3_process_exit.py",
                   "p3_mavlink.py", "p3_provenance.py", "collect_p2_evidence.py", "timing_evidence.py",
                   "reference_evidence.py", "yaw_evidence.py")


def evaluator_identity():
    files = {name: digest(VALIDATION / name) for name in EVALUATOR_FILES}
    for name in ("mc_trace_evidence.py", "mc_trace_schema.py"):
        if (VALIDATION / name).exists():
            files[name] = digest(VALIDATION / name)
    return files


def configs():
    return (strict_json(SIM / "configs/p3_validation.yaml"), strict_json(SIM / "configs/p3_sensors.yaml"))


def mandatory_artifacts(run, f, names):
    for name in names:
        f.check((Path(run) / name).exists(), "missing_artifact:" + name)


def common(run, summary, f, out):
    run = Path(run)
    f.check(summary.get("finalized") is True, "runtime_not_finalized")
    f.check(summary.get("frozen_manifest_sha256") is not None, "unfrozen_runtime")
    cleanup = classify_all(summary.get("processes", []))
    out["cleanup"] = cleanup
    f.items += ["cleanup:" + item for item in cleanup["failures"]]
    f.check(summary.get("residual_namespace_processes") == [], "residual_namespace_processes")
    for entry in summary.get("artifacts", []):
        f.check((run / entry["path"]).exists() and digest(run / entry["path"]) == entry["sha256"],
                "artifact_hash_changed:" + entry["path"])


def native_commands(run, summary, f, out):
    from p3_mavlink import evaluate_policy
    run = Path(run)
    closed = strict_json(run / "mavlink/closed.json")
    effective = summary["native_command_policy"]
    startup_end = summary["startup"]["origin_monotonic_ns"] + int(
        effective["policy"].get("startup_wall_s", 0) * 1e9)
    result = evaluate_policy(closed, effective, startup_end)
    out["native_commands"] = result
    f.items += ["native_commands:" + item for item in result["failures"]]


def p2_component(run, summary, f, out, flight):
    """Run the accepted P2 evaluator unchanged on the same run directory."""
    sys.path.insert(0, str(VALIDATION))
    import collect_p2_evidence
    report = collect_p2_evidence.verify(Path(run))
    out["p2_evaluator"] = report
    f.check(report.get("recording_integrity") == "passed", "p2_recording_integrity")
    if flight:
        f.check(report.get("flight_acceptance") == "passed", "p2_flight_acceptance")
        f.check(report.get("final_landed") is True and report.get("final_disarmed") is True, "final_landed_disarmed")
        f.check(report.get("command_ids") == [176, 400, 21], "unexpected_controller_command_sequence")
        f.check(not report.get("failsafe_observed"), "failsafe_observed")
    else:
        f.check(report.get("connectivity") == "passed", "p2_readiness_connectivity")
    f.check((report.get("timing") or {}).get("status") == "passed", "p2_timing_missing_or_failed")
    return report


def mc_component(run, summary, f, out, flight, p2_report):
    if "mc_trace" not in summary["identity"]:
        f.check(False, "mc_causal_trace_not_selected")
        return
    import mc_trace_evidence
    if flight:
        import collect_p2_evidence
        log, _ = collect_p2_evidence.read_ulog(summary["ulog_files"][0]["path"], True)
        ownership = (p2_report or {}).get("yaw_ownership") or {}
        try:
            report = mc_trace_evidence.exact_flight(Path(run), summary, log, summary["controller_result"], ownership)
        except (ValueError, KeyError, StopIteration, FileNotFoundError) as exc:
            report = {"status": "failed", "error": type(exc).__name__ + ":" + str(exc)}
    else:
        try:
            report = mc_trace_evidence.acquisition(Path(run), summary)
        except (ValueError, KeyError, FileNotFoundError) as exc:
            report = {"status": "failed", "error": type(exc).__name__ + ":" + str(exc)}
    out["mc_causality"] = report
    f.check(report.get("status") == "verified", "mc_causal_yaw:" + str(report.get("status")))
    if report.get("status") == "verified":
        f.check(report["max_yaw_error_rad"] is not None and report["max_yaw_error_rad"] <= 1e-5, "mc_yaw_error")


def evaluate(run):
    run = Path(run)
    summary = strict_json(run / "summary.json")
    validation, sensors = configs()
    f = Failures()
    kind = summary["p3_kind"]
    out = {"contract": "p3-offline-evaluation-v1", "run_id": run.name, "kind": kind,
           "gate": summary.get("gate"), "frozen_manifest_sha256": summary.get("frozen_manifest_sha256"),
           "evaluator_sha256": evaluator_identity(), "runtime_status": summary.get("status"),
           "runtime_failure": summary.get("failure")}
    f.check(summary.get("status") == "passed", "runtime_status:" + str(summary.get("failure")))
    common(run, summary, f, out)
    if kind == "fixture":
        from p3_geometry_eval import evaluate_fixture
        try:
            out["fixture"] = evaluate_fixture(run, validation, sensors)
            f.items += ["fixture:" + item for item in out["fixture"]["failures"]]
        except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
            f.check(False, "fixture_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
    else:
        flight = kind == "flight"
        mandatory_artifacts(run, f, ("controller-result.json", "ros-events.jsonl", "rosbag/metadata.yaml",
                                     "mavlink/closed.json", "source/probe-closed.json", "sensors/adapter-result.json"))
        p2 = None
        try:
            p2 = p2_component(run, summary, f, out, flight)
        except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
            f.check(False, "p2_evaluator_error:" + type(exc).__name__ + ":" + str(exc))
        window = summary["sensor_window_sim_s"]
        try:
            out["depth_stream"] = evaluate_stream(run, window, validation, sensors, require_association=True)
            f.items += ["depth:" + item for item in out["depth_stream"]["failures"]]
            out["rgb_stream"] = evaluate_rgb(run, window, validation, sensors["rgb_sensor"])
            f.items += ["rgb:" + item for item in out["rgb_stream"]["failures"]]
        except (ValueError, KeyError, FileNotFoundError) as exc:
            f.check(False, "sensor_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
        try:
            native_commands(run, summary, f, out)
        except (ValueError, KeyError, FileNotFoundError) as exc:
            f.check(False, "native_command_evaluation_error:" + str(exc))
        mc_component(run, summary, f, out, flight, p2)
        out["rtf"] = native_rtf(run)
        f.check(out["rtf"] is not None, "native_rtf_unmeasured")
        out["renderer"] = renderer = summary.get("renderer") or {}
        if summary.get("gate") == "V":
            # Validation V measures the declared software renderer; it must really be llvmpipe.
            f.check(renderer.get("software") is True
                    and any("llvmpipe" in s for s in renderer.get("strings", [])), "validation_renderer_not_llvmpipe")
        else:
            f.check(bool(renderer.get("identified")), "renderer_unidentified")
        out["resources"] = summary.get("resources")
        f.check(summary.get("controller_publishers_only_from_controller") is True, "competing_flight_publisher")
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    out["acceptance_credit"] = out["status"] == "passed" and summary.get("gate") != "V"
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args(argv)
    output = args.run / "p3-evaluation.json"
    if output.exists():
        raise SystemExit("Evaluation already exists; the first evaluation is retained")
    try:
        result = evaluate(args.run)
    except Exception as exc:  # incomplete evidence (e.g. early runtime abort) is recorded as failed, never missing
        summary = {}
        try:
            summary = strict_json(args.run / "summary.json")
        except Exception:
            pass
        result = {"contract": "p3-offline-evaluation-v1", "run_id": args.run.name, "gate": summary.get("gate"),
                  "evaluator_sha256": evaluator_identity(), "runtime_status": summary.get("status"),
                  "runtime_failure": summary.get("failure"),
                  "failures": ["runtime_status:" + str(summary.get("failure")),
                               "evaluator_incomplete_evidence:" + type(exc).__name__ + ":" + str(exc)],
                  "status": "failed", "acceptance_credit": False}
    atomic_json(output, result, exclusive=True)
    print(result["status"], len(result["failures"]), "failures", flush=True)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
