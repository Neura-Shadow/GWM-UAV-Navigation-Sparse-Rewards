"""P4 M6/M7 offline: evaluator rules on synthetic evidence, runner and gate logic, P3 guard.

No PX4, Gazebo, ROS node or QGC is started. Synthetic run directories exercise the
independent evaluator's pass and fail decisions; runner hooks run against fakes.
"""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
import p4_evaluate  # noqa: E402
import p4_manifest  # noqa: E402
import run_p4_gates  # noqa: E402
from gwm_obstacle_avoidance import local_map, perception  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402
from p3_stream_eval import Failures  # noqa: E402

V4 = json.loads((syn.P4 / "configs/p4_validation.yaml").read_text())
CONFIG = syn.p4_config()
REPO = syn.REPO


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


# ---- P3 baseline guard -----------------------------------------------------------------------------
def test_no_p3_manifest_bound_file_changed_since_the_p3_release_commit():
    """The P3 manifest hashes these paths; P4 must never touch them (contract C5).

    Content is compared with line endings normalized, so the check is the same from the Windows
    checkout and from WSL (git's CRLF index state differs between them)."""
    paths = ["simulation/px4_gazebo", "docs/evidence/v3_sim_p2_summary.json"]
    tracked = subprocess.run(["git", "ls-tree", "-r", "--name-only", "f0f9c0c", "--"] + paths, cwd=REPO,
                             capture_output=True, text=True, check=True).stdout.split()
    assert len(tracked) > 100
    changed = []
    for name in tracked:
        committed = subprocess.run(["git", "show", "f0f9c0c:" + name], cwd=REPO, capture_output=True, check=True).stdout
        current = (REPO / name).read_bytes() if (REPO / name).exists() else None
        normalize = lambda data: data.replace(b"\r\n", b"\n")
        if current is None or normalize(current) != normalize(committed):
            changed.append(name)
    untracked = subprocess.run(["git", "ls-files", "--others", "--exclude-standard", "--"] + paths, cwd=REPO,
                               capture_output=True, text=True, check=True).stdout.split()
    assert changed == [] and untracked == []


def test_p4_sources_live_outside_every_p3_glob():
    sources = p4_manifest.p4_source_files()
    assert sources and all(not name.startswith("..") for name in sources)
    assert "scripts/p4_runner.py" in sources and "ros2_ws/src/gwm_obstacle_avoidance/gwm_obstacle_avoidance/planner.py" in sources
    assert not any("__pycache__" in name for name in sources)
    p3_folders = [syn.SIM / f for f in ("scripts", "validation", "configs", "ros2_ws/src/gwm_px4_control",
                                         "ros2_ws/src/gwm_sensor_adapter")]
    for name in sources:
        path = (syn.P4 / name).resolve()
        assert not any(folder.resolve() in path.parents for folder in p3_folders)


def test_namespace_owner_is_byte_identical_and_the_shim_dispatches_to_p4():
    owner = (syn.P4 / "scripts/p4_namespace_owner.py").read_bytes()
    assert hashlib.sha256(owner).digest() == hashlib.sha256((syn.SIM / "scripts/p3_namespace_owner.py").read_bytes()).digest()
    shim = (syn.P4 / "scripts/p3_runner.py").read_text()
    assert "from p4_runner import main" in shim


# ---- gates and authorizations ------------------------------------------------------------------------
def test_gate_plans_follow_the_contract():
    assert [r[0] for r in run_p4_gates.plan("A")] == ["p3_readiness", "p4_readiness", "fixture", "fixture", "fixture",
                                                       "fault", "fault"]
    assert run_p4_gates.plan("B") == [("flight", "S2", "smoke")]
    assert [r[1] for r in run_p4_gates.plan("S")] == ["S1", "S5", "S6"]
    assert [r[1] for r in run_p4_gates.plan("C")] == ["S2", "S3", "S4"]
    assert p4_manifest.FLIGHT_GATES == ("B", "S", "C")
    command = run_p4_gates.runtime_command("flight", "S6", "safety-S6", "S", Path("/m"), Path("/a"))
    assert command[-3:] == ["--scenario", "S6", "--allow-simulated-flight"] and "run_p4.sh" in command[1]
    assert run_p4_gates.runtime_command("fault", "bridge_stop", "r", "A", Path("/m"), Path("/a"))[-2:] == ["--fixture", "bridge_stop"]


def test_authorization_binds_gate_scope_and_manifest(tmp_path):
    sha = "f" * 64
    path = tmp_path / "auth.json"
    path.write_text(json.dumps(p4_manifest.authorization_template("S", sha, "op")))
    assert p4_manifest.require_authorization(path, "S", sha)["authorized_by"] == "op"
    with pytest.raises(PermissionError):
        p4_manifest.require_authorization(path, "C", sha)
    with pytest.raises(PermissionError):
        p4_manifest.require_authorization(path, "S", "0" * 64)
    p3_style = dict(json.loads(path.read_text()), contract="p3-runtime-authorization-v1")
    path.write_text(json.dumps(p3_style))
    with pytest.raises(PermissionError):
        p4_manifest.require_authorization(path, "S", sha)  # P3 authorizations never authorize P4


def test_predecessor_and_consumption_are_permanent(tmp_path):
    (tmp_path / "state").mkdir()
    auth = tmp_path / "auth.json"
    auth.write_text("{}")
    marker = p4_manifest.consume(tmp_path, "A", "a" * 64, auth)
    with pytest.raises(FileExistsError):
        p4_manifest.consume(tmp_path, "A", "a" * 64, auth)
    assert marker.name == "p4-gate-A-" + "a" * 64 + ".consumed.json"
    with pytest.raises(PermissionError, match="requires an accepted Gate A"):
        run_p4_gates.require_predecessor(tmp_path, "B", "a" * 64)
    assert run_p4_gates.require_predecessor(tmp_path, "A", "a" * 64) is None
    run = tmp_path / "run"
    run.mkdir()
    (run / "p4-evaluation.json").write_text("{}")
    rows = [{"run": str(run), "evaluation_sha256": hashlib.sha256(b"{}").hexdigest()}]
    run_p4_gates.result_path(tmp_path, "A", "a" * 64).write_text(json.dumps(
        {"accepted": True, "manifest_sha256": "a" * 64, "runs": rows}))
    assert run_p4_gates.require_predecessor(tmp_path, "B", "a" * 64)["sha256"]
    (run / "p4-evaluation.json").write_text('{"tampered": 1}')
    with pytest.raises(PermissionError, match="changed after acceptance"):
        run_p4_gates.require_predecessor(tmp_path, "B", "a" * 64)


# ---- runner hooks (fakes; no runtime) -----------------------------------------------------------------
@pytest.fixture
def runner_module():
    if sys.platform == "win32":
        pytest.skip("the accepted P3 runner imports POSIX-only modules")
    import p4_runner
    return p4_runner


def test_runner_scopes_map_to_the_unchanged_p3_stack(runner_module):
    args = runner_module.parse(["--run", "--scope", "flight", "--scenario", "S6", "--gate", "S", "--role", "r",
                                "--manifest", "/m", "--authorization", "/a", "--allow-simulated-flight"])
    assert runner_module.SCOPES[args.scope] == "flight" and args.scenario == "S6"
    with pytest.raises(SystemExit):
        runner_module.parse(["--run", "--scope", "fault", "--fixture", "calibration", "--gate", "A", "--role", "r",
                             "--manifest", "/m", "--authorization", "/a"])
    assert runner_module.SCOPES["p3_readiness"] == "readiness"


@pytest.mark.parametrize("scope,mode", [("p4_readiness", "--ground-diagnostic"), ("fixture", "--ground-diagnostic"),
                                        ("fault", "--ground-diagnostic"), ("flight", "--flight")])
def test_controller_environment_satisfies_the_p2_gate_rule(runner_module, tmp_path, scope, mode):
    """P4 Gate A 3785a54c regression: P2 gates(env, flight or ground_diagnostic) needs the SITL gate."""
    from gwm_px4_control.contracts import gates
    (tmp_path / "state").mkdir()
    (tmp_path / "state/p4-built.json").write_text(json.dumps({"install": str(tmp_path / "install")}))
    runner = object.__new__(runner_module.P4Runner)
    runner.root, runner.run, runner.p4_scope, runner.marker = tmp_path, tmp_path / "run", scope, tmp_path / "marker"
    base_env = {"GWM_ALLOW_OPTIONAL_RUNTIME": "1", "GWM_RUN_GAZEBO_PX4_TESTS": "1", "GWM_ALLOW_PX4_LAUNCH": "1"}
    command, env = runner.p4_controller_command(dict(base_env))
    gates(env, flight=True)  # the exact check p4_control performs for --flight and --ground-diagnostic
    assert env["GWM_P4_OWNED_NAMESPACE"] == "1" and env["GWM_ALLOW_SITL_COMMANDS"] == "1"
    assert command[-1] == mode and command.count("--flight") == (scope == "flight")
    assert command[0].endswith("gwm_obstacle_avoidance/lib/gwm_obstacle_avoidance/p4_control")


def test_runner_redirects_only_the_px4_world_line(runner_module, tmp_path):
    runner = object.__new__(runner_module.P4Runner)
    runner.run, runner.world_dir, runner.summary = tmp_path, tmp_path / "world", {}
    (tmp_path / "rootfs").mkdir()
    runner.world_dir.mkdir()
    (runner.world_dir / "default.sdf").write_text("<sdf/>")
    original = "export PX4_GZ_MODELS=/m\nexport PX4_GZ_WORLDS=/pinned/worlds\nexport GZ_SIM_RESOURCE_PATH=$X\n"
    (tmp_path / "rootfs/gz_env.sh").write_text(original)
    runner.point_px4_at_world()
    lines = (tmp_path / "rootfs/gz_env.sh").read_text().splitlines()
    assert lines == ["export PX4_GZ_MODELS=/m", "export PX4_GZ_WORLDS=" + str(runner.world_dir),
                     "export GZ_SIM_RESOURCE_PATH=$X"]


def test_runner_records_one_injection_and_orders_shutdown(runner_module, tmp_path):
    runner = object.__new__(runner_module.P4Runner)
    stopped = []
    bridge = SimpleNamespace(name="sensor-bridge", pid=42, record={"stop_requested_monotonic_s": None},
                             request_stop=lambda sig: bridge.record.update(stop_requested_monotonic_s=12.5))
    runner.owned, runner.run, runner.summary = [bridge], tmp_path, {}
    (tmp_path / "p4").mkdir()
    runner.p4_scope, runner.declaration, runner.injection = "flight", {"scenario": "S6"}, None
    runner.sim_now = lambda: 44.0
    runner.save = lambda: None
    runner.inject("sensor-bridge", vehicle_ned=[0.1, 2.0, -2.0])
    record = json.loads((tmp_path / "p4/fault_injection.json").read_text())
    assert record["t_inj_sim_s"] == 44.0 and record["process"] == "sensor-bridge" and record["request_monotonic_s"] == 12.5
    with pytest.raises(FileExistsError):
        runner.inject("sensor-bridge", vehicle_ned=None)
    base_stop = runner_module.base.Runner.stop
    runner_module.base.Runner.stop = lambda self, name, sig=None: stopped.append(name)
    try:
        runner.stop("sensor-bridge")
        runner.stop("clock-bridge")
    finally:
        runner_module.base.Runner.stop = base_stop
    assert stopped == ["p4-perception", "sensor-bridge", "clock-bridge", "p4-truth-observer"]


# ---- evaluator: §6.1 performance ----------------------------------------------------------------------
def snapshot_rows(n=100, start=10.0, latency=0.08, compute=0.02, gap=0.05, usable=True):
    return [{"processing_sim_s": start + k * 0.1, "acquisition_sim_s": start + k * 0.1 - latency, "latency_sim_s": latency,
             "compute_wall_s": compute, "receipt_gap_max_wall_s": gap, "usable": usable} for k in range(n)]


def perf_run(tmp_path, rows):
    write_jsonl(tmp_path / "p4/local_map.jsonl", rows)
    (tmp_path / "p4/perception-result.json").write_text(json.dumps({"status": "passed"}))
    return tmp_path


@pytest.mark.parametrize("override,failure", [
    ({}, None),
    ({"latency": 0.26}, "performance:snapshot_latency_max"),
    ({"compute": 0.06}, "performance:compute_p99"),
    ({"gap": 1.2}, "performance:perception_receipt_gap"),
    ({"n": 60}, "performance:snapshot_rate")])
def test_performance_limits(tmp_path, override, failure):
    f = Failures()
    p4_evaluate.performance(perf_run(tmp_path, snapshot_rows(**override)), [10.0, 19.9], V4["performance"], f)
    assert (failure in f.items) if failure else not f.items, f.items


# ---- evaluator: §6 safety, latency and outcomes ---------------------------------------------------------
def flight_run(tmp_path, path_xy, boxes, contacts=(), interval_ns=40_000_000, targets=None, decisions=None,
               transitions=(), outcome="completed", abort=None):
    write_jsonl(tmp_path / "p4/truth_poses.jsonl",
                [{"stamp_ns": 20_000_000_000 + k * interval_ns, "poses": {"x500_depth_71": {"position": [x, y, z]}}}
                 for k, (x, y, z) in enumerate(path_xy)])
    write_jsonl(tmp_path / "p4/contacts.jsonl", list(contacts))
    (tmp_path / "p4/truth-closed.json").write_text(json.dumps({"error": None}))
    events = [{"event": "transition", "to": "PRESTREAM_SAFE_SETPOINTS", "sim_s": 20.0,
               "evidence": {"origin_ned": [0.0, 0.0, 0.0]}}]
    for k, (x, y, z) in enumerate(targets or []):
        events.append({"event": "setpoint", "phase": "TRANSIT", "ros_sim_s": 40.0 + k * 0.05,
                       "fields": {"position": [y, x, -z], "yaw": 1.5707963}})
    events += decisions or [{"event": "p4_decision", "sim_s": 40.0, "state": "NORMAL", "target_enu": [0, 0],
                             "vehicle_enu": [0, 0], "path_valid": True, "occupied_in_corridor": False,
                             "snapshot_acquisition_sim_s": 39.95}]
    write_jsonl(tmp_path / "ros-events.jsonl", events)
    landed = outcome in ("completed", "supervisor_abort")  # revision 5: a supervisor abort lands to COMPLETE
    summary = {"config": {"recovery_wall_s": 45.0},
               "controller_result": {"p4_outcome": outcome, "final_state": "COMPLETE" if landed else "RECOVERY",
                                     "failure": None if landed else "p2_safety_error",
                                     "transactions": [{"command": 21, "status": "accepted", "sent_monotonic_s": 100.0}],
                                     "transitions": [{"to": "COMPLETE", "monotonic_s": 120.0}] if landed else [],
                                     "p4": {"origin_enu": [0.0, 0.0], "abort_reason": abort,
                                            "supervisor_transitions": [{"to": t, "sim_s": 41.0 + i} for i, t in enumerate(transitions)]}}}
    return tmp_path, {"boxes": boxes, "scenario": "S2", "outcome": "goal_reached_with_detour", "detour_side": None}, summary


PILLAR = [{"name": "pillar_a", "center_m": [4.0, 0.0, 2.0], "size_m": [0.4, 0.4, 4.0]}]


def detour_path(lateral):
    xs = np.linspace(0.0, 8.0, 400)
    return [(x, lateral * math.exp(-((x - 4.0) / 1.2) ** 2), 2.0 if 0.1 < x < 7.9 else 0.3) for x in xs]


def test_safe_detour_passes_safety_and_outcome(tmp_path):
    run, declaration, summary = flight_run(tmp_path, detour_path(1.3), PILLAR,
                                           transitions=["OBSTACLE_DETECTED", "AVOIDING", "RESUME", "NORMAL"])
    f = Failures()
    out, offset, events = p4_evaluate.safety(run, declaration, V4["safety"], CONFIG["envelope"], f)
    assert not f.items, f.items
    assert out["min_truth_clearance_m"] >= 0.20 and out["collisions"] == 0
    result = p4_evaluate.outcome(run, summary, declaration, events, offset, p4_evaluate.truth_track(run), V4["outcomes"], f)
    assert not f.items and result["label"] == "goal_reached_with_detour"


@pytest.mark.parametrize("case,failure", [
    ("close", "safety:truth_clearance"),
    ("contact", "safety:collision_contact"),
    ("sparse", "safety:truth_pose_interval"),
    ("target", "safety:setpoint_inside_inflated_obstacle")])
def test_safety_rejections(tmp_path, case, failure):
    path = detour_path(0.75 if case == "close" else 1.3)
    contacts = [{"stamp_ns": 1, "obstacle": "pillar_a", "contacts": [["pillar_a::l::c", "x500_depth_71::base_link::c", 1]]}] \
        if case == "contact" else ()
    targets = [(3.9, 0.0, 2.0)] if case == "target" else None
    run, declaration, _ = flight_run(tmp_path, path, PILLAR, contacts=contacts,
                                     interval_ns=80_000_000 if case == "sparse" else 40_000_000, targets=targets)
    f = Failures()
    p4_evaluate.safety(run, declaration, V4["safety"], CONFIG["envelope"], f)
    assert failure in f.items, f.items


def test_response_latency_from_the_first_invalidating_snapshot():
    decisions = [{"event": "p4_decision", "sim_s": 40.0, "state": "NORMAL", "occupied_in_corridor": False, "path_valid": True,
                  "snapshot_acquisition_sim_s": 39.95},
                 {"event": "p4_decision", "sim_s": 40.05, "state": "OBSTACLE_DETECTED", "occupied_in_corridor": True,
                  "path_valid": False, "snapshot_acquisition_sim_s": 39.98}]
    f = Failures()
    assert p4_evaluate.response_latency(decisions, 0.25, f) == pytest.approx(0.07) and not f.items
    late = [dict(decisions[1], state="NORMAL"), dict(decisions[1], sim_s=40.4, state="AVOIDING")]
    f = Failures()
    p4_evaluate.response_latency(late, 0.25, f)
    assert "safety:response_latency" in f.items


def test_clear_path_outcome_rejects_avoidance_and_safe_refusal_is_never_goal_reached(tmp_path):
    run, declaration, summary = flight_run(tmp_path / "s1", detour_path(0.0), [], transitions=["HOVER", "AVOIDING"])
    declaration.update(scenario="S1", outcome="goal_reached")
    f = Failures()
    _, offset, events = p4_evaluate.safety(run, declaration, V4["safety"], CONFIG["envelope"], f)
    p4_evaluate.outcome(run, summary, declaration, events, offset, p4_evaluate.truth_track(run), V4["outcomes"], f)
    assert "outcome:clear_path_avoided_or_hovered" in f.items
    wall = [{"name": "wall", "center_m": [4.0, 0.0, 2.0], "size_m": [0.3, 8.0, 4.0]}]
    stop = [(x, 0.0, 2.0) for x in np.linspace(0.0, 2.4, 200)]
    hover = [{"event": "p4_decision", "sim_s": 50.0 + k * 0.05, "state": "HOVER", "target_enu": [2.4, 0.0],
              "vehicle_enu": [2.4, 0.0]} for k in range(50)]
    run, declaration, summary = flight_run(tmp_path / "s5", stop, wall, decisions=hover, transitions=["HOVER", "ABORT"],
                                           outcome="supervisor_abort", abort="hover_timeout")
    summary["controller_result"]["p4"]["supervisor_transitions"] = [{"to": "HOVER", "sim_s": 50.0}, {"to": "ABORT", "sim_s": 70.1}]
    declaration.update(scenario="S5", outcome="safe_refusal")
    f = Failures()
    _, offset, events = p4_evaluate.safety(run, declaration, V4["safety"], CONFIG["envelope"], f)
    result = p4_evaluate.outcome(run, summary, declaration, events, offset, p4_evaluate.truth_track(run), V4["outcomes"], f)
    assert not f.items and result["label"] == "safe_refusal" and result["abort_landing_wall_s"] == 20.0
    # Revision 5: an abort that ends in P2 RECOVERY, or lands too slowly, is not a safe refusal.
    for change, failure in ((dict(final_state="RECOVERY", failure="p4_supervisor_abort:hover_timeout", transitions=[]),
                             "outcome:abort_not_landed_through_complete"),
                            (dict(transitions=[{"to": "COMPLETE", "monotonic_s": 145.5}]), "outcome:abort_landing_wall_limit")):
        bad = json.loads(json.dumps(summary))
        bad["controller_result"].update(change)
        f = Failures()
        result = p4_evaluate.outcome(run, bad, declaration, events, offset, p4_evaluate.truth_track(run), V4["outcomes"], f)
        assert failure in f.items and result["label"] == "failed"


# ---- evaluator: ground fixtures and fault replay ----------------------------------------------------------
def fixture_run(tmp_path, boxes):
    sensors = syn.sensors()
    processor = perception.P4Processor(sensors, CONFIG, "fixture")
    extrinsic = Extrinsic(sensors["optical_origin_base_flu_m"])
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], 2.0, 0.1)
    vehicle = (0.0, 0.0, 0.24)
    position, q = syn.px4_pose(vehicle, 0.0)
    image = syn.render(vehicle, 0.0, syn.boxes_3d([{"center": b["center_m"][:2], "size": b["size_m"][:2]} for b in boxes]),
                       columns=np.arange(2, 640, 4))
    camera_xy, rays = perception.extract_scan(image, syn.CAL, extrinsic, position, q, CONFIG["perception"],
                                              sensors["near_m"], sensors["far_m"], 0.0)
    perception.integrate(grid, camera_xy, rays, 8.0, sensors["near_m"])
    del processor
    write_jsonl(tmp_path / "p4/local_map.jsonl", [{"usable": True, "processing_sim_s": 15.0, "grid": grid.encode(),
                                                   "position_ned": position, "q_wxyz": q,
                                                   "scan": [[r["azimuth"], r["kind"], r["range_m"]] for r in rays]}])
    write_jsonl(tmp_path / "p4/truth_poses.jsonl", [{"stamp_ns": 15_000_000_000,
                                                     "poses": {"x500_depth_71": {"position": [0.0, 0.0, 0.24]}}}])
    return tmp_path, {"sensor_window_sim_s": [10.0, 20.0]}


@pytest.mark.parametrize("name", ["G2", "G4", "G6"])
def test_ground_fixture_reconstruction_and_trigger_decision(tmp_path, name):
    import p4_worlds
    base = '<sdf version="1.9"><world name="default"></world></sdf>'
    _, declaration = p4_worlds.generate(base, name, syn.scenarios())
    run, summary = fixture_run(tmp_path, declaration["boxes"])
    f = Failures()
    out = p4_evaluate.fixture(run, declaration, summary, CONFIG, V4["ground_fixture"], f)
    assert not f.items, (f.items, out)
    assert out["trigger_decision"] == declaration["expected_trigger"]


def test_fault_replay_reaches_hover_after_an_adapter_stop(tmp_path):
    write_jsonl(tmp_path / "p4/local_map.jsonl", [{"usable": True, "processing_sim_s": 30.0 + k * 0.1,
                                                   "acquisition_sim_s": 29.95 + k * 0.1, "calibration_identity": "c"}
                                                  for k in range(100)])
    write_jsonl(tmp_path / "sensors/health.jsonl", [{"status": "fresh", "calibration_identity": "c", "sim_s": 30.0 + k}
                                                    for k in range(6)])  # adapter silent after 35 s
    (tmp_path / "p4/fault_injection.json").write_text(json.dumps({"t_inj_sim_s": 35.2, "process": "sensor-adapter"}))
    f = Failures()
    out = p4_evaluate.fault_replay(tmp_path, {}, CONFIG, f)
    assert not f.items and out["replayed_hover"]["reason"] == "adapter_health_missing"
    assert out["replayed_hover"]["sim_s"] <= 35.0 + 2.0 + 0.1


# ---- evaluator: §9.1 S6 fault window ------------------------------------------------------------------
def s6_run(tmp_path, *, native_until=60.0, late_receipt=False, bridge_exit=0, detect_after=0.6,
           hover_at=44.30, abort_after=10.05, moving=False, lands=1, land_wall=20.0, recovery=False):
    t_inj, request_wall = 44.0, 1044.0
    (tmp_path / "p4").mkdir(parents=True, exist_ok=True)
    (tmp_path / "p4/fault_injection.json").write_text(json.dumps(
        {"process": "sensor-bridge", "signal": "SIGINT", "t_inj_sim_s": t_inj, "request_monotonic_s": request_wall}))
    write_jsonl(tmp_path / "source/depth-headers.jsonl",
                [{"stamp_ns": int(round((30.0 + k / 30) * 1e9))} for k in range(int((native_until - 30.0) * 30))])
    index = [{"acquisition_stamp_ns": int(round((30.0 + k / 30) * 1e9))} for k in range(int((t_inj - 30.0) * 30))]
    if late_receipt:
        index.append({"acquisition_stamp_ns": int(45e9)})
    write_jsonl(tmp_path / "sensors/depth-index.jsonl", index)
    write_jsonl(tmp_path / "sensors/health.jsonl", [{"status": "fresh", "wall_s": 1040.0, "sim_s": 40.0},
                                                   {"status": "stale", "wall_s": request_wall + detect_after, "sim_s": 44.6}])
    write_jsonl(tmp_path / "sensors/observations.jsonl", [{"processing_wall_s": 1043.9}])
    (tmp_path / "sensors/adapter-result.json").write_text(json.dumps({"recorder": {"complete": True, "overflow": 0}}))
    write_jsonl(tmp_path / "p4/local_map.jsonl", [{"usable": True, "acquisition_sim_s": 44.0 - 0.1 * k} for k in range(10)])
    abort_at = hover_at + abort_after
    decisions = [{"event": "p4_decision", "sim_s": hover_at + 0.05 * k,
                  "target_enu": [2.0 + (0.02 * k if moving else 0.0), 0.0], "vehicle_enu": [2.0, 0.0]}
                 for k in range(int((abort_at - hover_at) / 0.05))]
    setpoints = [{"event": "setpoint", "ros_sim_s": d["sim_s"], "fields": {"yaw": 1.5707963}} for d in decisions]
    write_jsonl(tmp_path / "ros-events.jsonl", decisions + setpoints)
    summary = {"sensor_window_sim_s": [30.0, 120.0], "config": {"recovery_wall_s": 45.0},
               "processes": [{"name": "sensor-bridge", "exit_code": bridge_exit, "stop_requested_monotonic_s": request_wall}],
               "controller_result": {"p4_outcome": "supervisor_abort", "final_state": "RECOVERY" if recovery else "COMPLETE",
                                     "failure": "p4_supervisor_abort:stale_inputs:snapshot_stale" if recovery else None,
                                     "transactions": [{"command": 21, "status": "accepted", "sent_monotonic_s": 200.0}] * lands,
                                     "transitions": [] if recovery else [{"to": "COMPLETE", "monotonic_s": 200.0 + land_wall}],
                                     "p4": {"supervisor_transitions": [{"to": "HOVER", "sim_s": hover_at, "reason": "stale_inputs:snapshot_stale"},
                                                                       {"to": "ABORT", "sim_s": abort_at, "reason": "stale_inputs:snapshot_stale"}]}}}
    return tmp_path, summary


@pytest.fixture
def stub_stream(monkeypatch):
    monkeypatch.setattr(p4_evaluate, "evaluate_stream", lambda run, window, *a, **k: {"status": "passed", "failures": []})


P3V = json.loads((syn.SIM / "configs/p3_validation.yaml").read_text())


def test_s6_fault_window_passes_for_the_designed_response(tmp_path, stub_stream):
    run, summary = s6_run(tmp_path)
    f = Failures()
    out = p4_evaluate.fault_window(run, summary, P3V, syn.sensors(), V4["outcomes"], f)
    assert not f.items, f.items
    assert out["pre_window_sim_s"] == [30.0, pytest.approx(43.88)] and out["label"] == "safe_stop_on_sensor_loss"


@pytest.mark.parametrize("kwargs,failure", [
    ({"native_until": 45.0}, "s6:f1_native_source_stopped"),
    ({"late_receipt": True}, "s6:f1_ros_delivery_continued"),
    ({"bridge_exit": -2}, "s6:f1_bridge_exit"),
    ({"detect_after": 1.5}, "s6:f2_detection"),
    ({"hover_at": 44.40}, "s6:f3_late_hover"),
    ({"moving": True}, "s6:f3_motion_while_stale"),
    ({"abort_after": 10.4}, "s6:f3_abort_timing"),
    ({"lands": 2}, "s6:f3_single_accepted_land"),
    ({"land_wall": 45.5}, "s6:f3_abort_landing_wall_limit"),
    ({"recovery": True}, "s6:f3_abort_not_landed_through_complete")])
def test_s6_fault_window_rejections(tmp_path, stub_stream, kwargs, failure):
    run, summary = s6_run(tmp_path, **kwargs)
    f = Failures()
    p4_evaluate.fault_window(run, summary, P3V, syn.sensors(), V4["outcomes"], f)
    assert failure in f.items, f.items
