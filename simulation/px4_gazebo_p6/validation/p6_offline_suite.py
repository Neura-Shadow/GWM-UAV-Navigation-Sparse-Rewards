"""P6 M3 offline suite runner (contract §8.1 row 4): one hash-bound result for Gate A.

It runs the three P6 test files under the current interpreter, and requires:
* every required offline (R-O) item of `p6_faults.yaml` to be covered by named tests that all *passed*
  (a skip is not a pass);
* every one of the 12 flight rows to have a passing offline rehearsal;
* no failure anywhere.

It writes the outcomes, the characterization measurements and the SHA-256 of every P6 source, test,
spec and the recorded run's summary into one JSON. Run it under the controller interpreter
(`/usr/bin/python3`) with the retained recorded flight present; it starts no simulator.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
P6 = ROOT / "simulation/px4_gazebo_p6"
TESTS = ["tests/test_px4_gazebo_p6_specs.py", "tests/test_px4_gazebo_p6_injection.py", "tests/test_px4_gazebo_p6_offline.py"]
OFF = "test_px4_gazebo_p6_offline."
INJ = "test_px4_gazebo_p6_injection."
STATE = OFF + "test_R_app_side_state_faults_hit_the_intended_p2_check"
POLICY = OFF + "test_R_policy_faults_are_classified_by_the_accepted_ranker"
STALE = OFF + "test_R_stale_rules_hover_by_t_last_plus_0_35_with_frozen_target_then_abort_at_10_s"
ACKS = OFF + "test_R_ack_rewrites_raise_command_ack_denied_in_the_accepted_mission"
R_O = {
    "F1.06": [OFF + "test_R_F1_06_stamp_faults_on_recorded_frames_give_exactly_the_injected_rejections", STALE],
    "F1.07": [OFF + "test_R_F1_07_and_F1_12_layout_and_frame_id_faults_are_rejected_on_recorded_frames", STALE],
    "F1.08": [OFF + "test_R_F1_08_nan_region_never_creates_free_or_occupied_space_on_recorded_frames"],
    "F1.10": [OFF + "test_R_F1_10_coverage_loss_only_removes_information_on_recorded_frames"],
    "F1.11": [OFF + "test_R_F1_11_calibration_change_makes_health_not_fresh_and_the_supervisor_hover", STALE],
    "F1.12": [OFF + "test_R_F1_07_and_F1_12_layout_and_frame_id_faults_are_rejected_on_recorded_frames"],
    "F2.04": [OFF + "test_R_F2_04_reset_epoch_bracket_makes_frames_unusable_and_the_supervisor_hover", STALE],
    "F2.05": [STATE + "[F2.05"], "F2.06": [STATE + "[F2.06"], "F2.07": [STATE + "[F2.07"],
    "F2.09": [OFF + "test_R_F2_09_withheld_pose_makes_recorded_frames_unassociable", STALE],
    "F3.03": [OFF + "test_R_F3_03_schema_change_refuses_start_before_any_node"],
    "F3.05": [POLICY + "[F3.05"], "F3.11": [POLICY + "[F3.11"], "F3.12": [POLICY + "[F3.12"],
    "F3.06": [OFF + "test_R_F3_06_selection_outside_the_mask_is_classified"],
    "F3.09": [POLICY + "[F3.09", OFF + "test_R_F3_09_and_F6_03_overruns_trip_the_dispatch_guard_and_land_once"],
    "F4.11": [OFF + "test_R_F4_11_health_and_map_freshness_are_checked_independently"],
    "F4.06": [STALE + "[adapter_health_missing", OFF + "test_R_F4_11_health_and_map_freshness_are_checked_independently"],
    "F5.03": [ACKS + "[F5.03"], "F5.06": [ACKS + "[F5.06"],
    "F5.05": [OFF + "test_R_F5_05_wrong_nav_state_is_offboard_armed_state_lost"],
    "F5.07": [STATE + "[F5.07"], "F5.10": [STATE + "[F5.10"],
    "F6.03": [OFF + "test_R_F3_09_and_F6_03_overruns_trip_the_dispatch_guard_and_land_once",
              INJ + "test_tick_delay_sleeps_once_after_normal_at_two_metres"],
    "F7.02": [OFF + "test_R_F7_02_alternating_failures_never_reach_policy_unavailable"],
    "F7.04": [OFF + "test_R_F7_04_and_F7_06_fuzzed_supervisor_transitions_are_legal_and_abort_is_absorbing",
              OFF + "test_R_F7_04_every_recorded_accepted_flight_transition_is_legal"],
    "F7.05": [OFF + "test_R_F3_09_and_F6_03_overruns_trip_the_dispatch_guard_and_land_once"],
    "F7.06": [OFF + "test_R_F7_04_and_F7_06_fuzzed_supervisor_transitions_are_legal_and_abort_is_absorbing"],
    "F7.07": [OFF + "test_R_F7_07_target_moves_only_by_apply_action_and_never_while_held"],
    "F8.04": [OFF + "test_R_F8_04_policy_failure_rule_precedes_the_transition_table"],
    "F8.05": [OFF + "test_R_F8_05_a_controller_started_against_an_armed_vehicle_refuses"],
}
REHEARSALS = {
    "B-0": [INJ + "test_pass_through_shim_is_bit_identical_to_the_accepted_mission"],
    "B-1": [INJ + "test_policy_fault_rows_hold_three_ticks_then_abort_policy_unavailable[B-1"],
    "S-1": [OFF + "test_rehearsal_sensor_loss_rows_hover_then_abort_snapshot_stale_and_land[S-1"],
    "S-2'": [OFF + "test_rehearsal_sensor_loss_rows_hover_then_abort_snapshot_stale_and_land[S-2'"],
    "S-3": [INJ + "test_policy_fault_rows_hold_three_ticks_then_abort_policy_unavailable[S-3"],
    "S-4": [INJ + "test_map_loss_hovers_while_stale_then_recovers_and_completes"],
    "S-5'": [INJ + "test_tick_delay_sleeps_once_after_normal_at_two_metres"],
    "C-1": [OFF + "test_rehearsal_C1_depth_bursts_stale_exactly_when_old_and_recover_to_the_goal"],
    "C-2": [INJ + "test_isolated_timeouts_hold_only_their_ticks_and_the_mission_completes"],
    "C-3": [INJ + "test_adversarial_choice_stays_inside_the_mask_on_every_tick"],
    "C-4": [OFF + "test_rehearsal_C4_staleness_precedes_the_policy_and_the_run_aborts_and_lands"],
    "C-5": [OFF + "test_rehearsal_C5_map_loss_part_hovers_and_completes"],
}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def outcomes(junit):
    out = {}
    for case in ET.parse(junit).getroot().iter("testcase"):
        name = case.get("classname").split(".")[-1] + "." + case.get("name")
        status = "passed"
        if case.find("failure") is not None or case.find("error") is not None:
            status = "failed"
        elif case.find("skipped") is not None:
            status = "skipped"
        out[name] = status
    return out


def covered(results, patterns):
    """Each pattern must match at least one test, and every matching test must have passed."""
    detail = {}
    for pattern in patterns:
        matches = {name: status for name, status in results.items() if name == pattern or name.startswith(pattern)}
        detail[pattern] = matches
    ok = all(m and all(s == "passed" for s in m.values()) for m in detail.values())
    return ok, detail


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    sys.path.insert(0, str(P6))
    from injection import specs
    required = sorted(f["id"] for f in specs.faults().values() if "R-O" in f["classes"])
    if required != sorted(R_O):
        raise SystemExit("the R-O coverage map differs from p6_faults.yaml")
    with tempfile.TemporaryDirectory() as work:
        junit, measured = Path(work) / "junit.xml", Path(work) / "characterization.json"
        env = dict(os.environ, GWM_P6_CHARACTERIZATION_OUT=str(measured), PYTHONDONTWRITEBYTECODE="1")
        started = time.time()
        completed = subprocess.run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "--junitxml", str(junit)]
                                   + TESTS, cwd=ROOT, env=env, text=True, capture_output=True)
        results = outcomes(junit)
        characterization = json.loads(measured.read_text()) if measured.exists() else {}
    r_o = {fault: dict(zip(("passed", "tests"), covered(results, patterns))) for fault, patterns in R_O.items()}
    rehearsals = {row: dict(zip(("passed", "tests"), covered(results, patterns))) for row, patterns in REHEARSALS.items()}
    counts = {s: sum(1 for v in results.values() if v == s) for s in ("passed", "failed", "skipped")}
    from recorded import jsonl  # noqa: F401  (import check: the recorded reader is part of the bound sources)
    run = Path(os.environ.get("GWM_P6_RECORDED_RUN", Path.home() / "uav_autonomy/runs/20261106T132442Z-p5-flight-S2-184a5a5e"))
    files = sorted([*P6.rglob("*.py"), *P6.glob("configs/*"), *(ROOT / t for t in TESTS)])
    report = {
        "contract": "p6-offline-suite-v1",
        "created_unix_s": started, "wall_s": time.time() - started,
        "git_head": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, capture_output=True).stdout.strip(),
        "interpreter": sys.executable, "python": platform.python_version(),
        "numpy": __import__("numpy").__version__,
        "recorded_run": run.name, "recorded_run_summary_sha256": digest(run / "summary.json") if (run / "summary.json").exists() else None,
        "pytest_exit_code": completed.returncode, "counts": counts,
        "required_offline": r_o, "rehearsals": rehearsals, "characterization": characterization,
        "results": results,
        "bound_files": {str(p.relative_to(ROOT)).replace("\\", "/"): digest(p) for p in files if "__pycache__" not in p.parts},
    }
    report["passed"] = (completed.returncode == 0 and counts["failed"] == 0 and all(v["passed"] for v in r_o.values())
                        and all(v["passed"] for v in rehearsals.values()))
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({"passed": report["passed"], "counts": counts,
                      "required_offline_passed": sum(v["passed"] for v in r_o.values()), "required_offline": len(r_o),
                      "rehearsals_passed": sum(v["passed"] for v in rehearsals.values()), "rehearsals": len(rehearsals)}))
    if not report["passed"]:
        print(completed.stdout[-4000:])
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    raise SystemExit(main())
