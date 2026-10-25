"""P3 runtime-boundary regression: P2 preservation, cleanup classification, native
command policy, authorization/gates, QGC teardown evidence and probe parsing.

No PX4, Gazebo, ROS, QGC or flight is started. Real short-lived Python child
processes exercise exit classification only.
"""
import json
import os
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "simulation/px4_gazebo"
sys.path[:0] = [str(SIM / "scripts"), str(SIM / "validation"), str(SIM / "validation/qgc_unattended_v1")]
import p2_build  # noqa: E402
import p3_manifest  # noqa: E402
import p3_mavlink  # noqa: E402
import p3_process_exit  # noqa: E402
import p3_source_probe  # noqa: E402
import run_p3_gates  # noqa: E402
from p3_provenance import atomic_json, canonical_sha256, digest  # noqa: E402

# p3_runner (fcntl, process groups) and run_p3.sh run only in the Linux runtime host.
# These tests are reported as skipped on Windows, never as passed.
POSIX_ONLY = pytest.mark.skipif(sys.platform == "win32",
                                reason="POSIX-only: p3_runner/run_p3.sh target the Linux runtime host")

FIXTURES = REPO / "tests/fixtures/p3_mavlink"
ACCEPTED_P2_PACKAGE = "e1857b3fc0a5678c0789b39860ba36cae5193b8daa83a20c246a02f49bb431af"


# P2 preservation -----------------------------------------------------------------------------
def test_p2_controller_package_is_byte_identical_to_accepted_r3():
    assert p2_build.package_hash(p2_build.package_manifest(SIM)) == ACCEPTED_P2_PACKAGE
    evidence = json.loads((REPO / "docs/evidence/v3_sim_p2_summary.json").read_text())
    assert evidence["nominal_smoke"]["package_hash"] == ACCEPTED_P2_PACKAGE


def test_p3_flight_uses_the_unchanged_p2_mission_configuration():
    runner = (SIM / "scripts/p3_runner.py").read_text()
    assert 'SIM / "configs/p2_control.yaml"' in runner and "p3_control" not in runner
    config = json.loads((SIM / "configs/p2_control.yaml").read_text())
    assert (config["height_m"], config["east_m"], config["north_m"], config["yaw_enu_deg"]) == (2.0, 1.0, 1.0, 30.0)
    assert (config["ramp_m_s"], config["yaw_ramp_deg_s"], config["max_sample_gap_sim_s"]) == (0.3, 10.0, 0.2)


def test_mc_benchmark_validates_against_the_accepted_control_config():
    # The archived p3_control.yaml supplied the same two MC limits; the benchmark must
    # use the accepted config the runner binds, with the limits unchanged.
    bench = (SIM / "scripts/benchmark_mc_trace.py").read_text()
    assert "configs/p2_control.yaml" in bench and "p3_control" not in bench
    evidence = (SIM / "validation/mc_trace_evidence.py").read_text()
    assert set(re.findall(r"config\['(\w+)'\]", evidence)) == {"max_sample_gap_sim_s", "position_fresh_sim_s"}
    config = json.loads((SIM / "configs/p2_control.yaml").read_text())
    assert (config["max_sample_gap_sim_s"], config["position_fresh_sim_s"]) == (0.2, 0.25)


@pytest.mark.parametrize("forbidden", ["commander arm", "arm -f", "force", "param set", "COM_ARM", "CBRK_",
                                       "VehicleCommand(", "create_publisher"])
def test_runner_has_no_flight_or_preflight_bypass_path(forbidden):
    import ast
    source = (SIM / "scripts/p3_runner.py").read_text()
    text = source.replace(ast.get_docstring(ast.parse(source)), "")  # code only, not the policy prose
    if forbidden == "force":
        assert "force_arm" not in text and "force-arm" not in text and "force_disarm" not in text
    else:
        assert forbidden not in text


def test_sensor_adapter_creates_publishers_only_through_guard():
    text = (SIM / "ros2_ws/src/gwm_sensor_adapter/gwm_sensor_adapter/node.py").read_text()
    assert text.count("self.create_publisher(") == 1
    assert text.count("self.guarded_publisher(") == 2 and "px4_msgs.msg as px4" in text
    assert "VehicleCommand" not in text and "TrajectorySetpoint" not in text and "OffboardControlMode" not in text


# Cleanup classification ----------------------------------------------------------------------
def record(**override):
    base = {"name": "p", "stop_method": "SIGINT", "grace_s": 15, "graceful_codes": [0], "exit_code": 0,
            "reaped": True, "forced": False, "remaining_group_members": [],
            "stop_requested_monotonic_s": 10.0, "exit_observed_monotonic_s": 10.5}
    base.update(override)
    return base


@pytest.mark.parametrize("override,reason", [
    ({}, None), ({"exit_code": -11}, "terminated_by_SIGSEGV"), ({"exit_code": -15}, "terminated_by_SIGTERM"),
    ({"exit_code": -9, "forced": True}, "forced_kill_after_grace"), ({"exit_code": -9}, "terminated_by_SIGKILL"),
    ({"reaped": False, "exit_code": None}, "not_reaped"), ({"exit_code": 3}, "nonzero_exit_3"),
    ({"stop_requested_monotonic_s": None}, "exited_before_stop_request"),
    ({"exit_observed_monotonic_s": 30.0}, "graceful_deadline_exceeded"),
    ({"remaining_group_members": [44]}, "process_group_members_remained"),
    ({"stop_method": "kill"}, "undeclared_stop_method")])
def test_exit_classification_is_strict(override, reason):
    classification, actual = p3_process_exit.classify(record(**override))
    assert actual == reason and (classification == "graceful") == (reason is None)


def test_controller_that_completes_its_own_work():
    ok = record(stop_method="completes_own_work", stop_requested_monotonic_s=None)
    assert p3_process_exit.classify(ok) == ("graceful", None)
    assert p3_process_exit.classify(dict(ok, exit_code=1))[1] == "nonzero_exit_1"


def test_classify_all_requires_at_least_one_process():
    assert p3_process_exit.classify_all([])["status"] == "failed"


CHILD = {"graceful": "import signal,sys,time\nsignal.signal(signal.SIGINT,lambda *a: sys.exit(0))\n"
                     "print('ready',flush=True)\ntime.sleep(30)",
         "ignores": "import signal,time\nsignal.signal(signal.SIGINT,signal.SIG_IGN)\nprint('ready',flush=True)\ntime.sleep(30)",
         "segfault": "import os,signal,sys,time\nsignal.signal(signal.SIGINT,lambda *a: os.kill(os.getpid(),signal.SIGSEGV))\n"
                     "print('ready',flush=True)\ntime.sleep(30)",
         "default": "import time\nprint('ready',flush=True)\ntime.sleep(30)"}


@POSIX_ONLY
@pytest.mark.parametrize("kind,expected", [("graceful", None), ("ignores", "forced_kill_after_grace"),
                                           ("segfault", "terminated_by_SIGSEGV"), ("default", "nonzero_exit_1")])
def test_owned_process_with_real_children(kind, expected):
    from p3_runner import Owned
    process = subprocess.Popen([sys.executable, "-c", CHILD[kind]], stdout=subprocess.PIPE, start_new_session=True)
    assert process.stdout.readline().strip() == b"ready"
    owned = Owned(kind, process.pid, ["python"], "SIGINT", 1.0, process)
    owned.request_stop(signal.SIGINT)
    if not owned.wait(1.0):
        owned.force()
    # A default-handler Python exits 1 with KeyboardInterrupt? (CPython re-raises SIGINT: -2).
    classification, reason = p3_process_exit.classify(owned.record)
    if kind == "default":
        assert classification == "failed" and reason in ("nonzero_exit_1", "terminated_by_SIGINT")
    else:
        assert reason == expected
    assert owned.record["reaped"] is True


# Native MAVLink -------------------------------------------------------------------------------
def test_decoder_matches_native_header_generated_fixtures():
    identity = json.loads((FIXTURES / "identity.json").read_text())
    for name, expected in identity["fixtures"].items():
        raw = (FIXTURES / name).read_bytes()
        assert digest(FIXTURES / name) == expected["sha256"]
        decoded = p3_mavlink.decode_datagram(raw)
        assert len(decoded) == len(expected["decoded"])
        for got, want in zip(decoded, expected["decoded"]):
            for key in ("message_id", "source_system", "source_component", "raw_hex", "command",
                        "target_system", "target_component"):
                assert got.get(key) == want.get(key), (name, key)


def test_decoder_rejects_corruption_and_truncation():
    raw = bytearray((FIXTURES / "command401-v2.bin").read_bytes())
    raw[-1] ^= 1
    with pytest.raises(ValueError, match="CRC"):
        p3_mavlink.decode_datagram(bytes(raw))
    with pytest.raises(ValueError, match="truncated"):
        p3_mavlink.decode_datagram((FIXTURES / "command401-v2.bin").read_bytes()[:-3])
    with pytest.raises(ValueError, match="unframed"):
        p3_mavlink.decode_datagram(b"\x00\x01")


def frames(*names, stamp_ns=1_000_000_000):
    rows = []
    for name in names:
        for frame in p3_mavlink.decode_datagram((FIXTURES / name).read_bytes()):
            rows.append(dict(frame, monotonic_ns=stamp_ns))
    return {"frames_of_interest": rows, "complete": True}


def policy_file():
    return SIM / "configs/p3_native_command_policy.json"


def test_default_policy_is_strict_zero_and_rejects_stock_401():
    effective = p3_mavlink.load_policy(policy_file(), None)
    assert effective["name"] == "strict_zero"
    result = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin", "command401-v2.bin"), effective, 2_000_000_000)
    assert result["status"] == "failed" and "command_not_permitted_by_policy" in result["failures"]
    clean = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin"), effective, 2_000_000_000)
    assert clean["status"] == "passed" and clean["global_zero_native_commands"]


def test_heartbeat_is_required():
    effective = p3_mavlink.load_policy(policy_file(), None)
    assert "gcs_heartbeat_not_observed" in p3_mavlink.evaluate_policy({"frames_of_interest": [], "complete": True},
                                                                      effective)["failures"]


def test_policy_a_requires_bound_operator_approval(tmp_path):
    with pytest.raises(PermissionError):
        p3_mavlink.load_policy(policy_file(), "A")
    document = json.loads(policy_file().read_text())
    approval = tmp_path / "approval.json"
    approval.write_text(json.dumps({"contract": "p3-native-command-policy-approval-v1", "policy": "A",
                                    "policy_sha256": "0" * 64, "approved": True, "approved_by": "unit-test"}))
    with pytest.raises(PermissionError, match="bind"):
        p3_mavlink.load_policy(policy_file(), "A", approval)
    approval.write_text(json.dumps({"contract": "p3-native-command-policy-approval-v1", "policy": "A",
                                    "policy_sha256": canonical_sha256(document["policies"]["A"]), "approved": True,
                                    "approved_by": "unit-test"}))
    assert p3_mavlink.load_policy(policy_file(), "A", approval)["name"] == "A"


def approved_policy_a(tmp_path):
    document = json.loads(policy_file().read_text())
    approval = tmp_path / "approval.json"
    approval.write_text(json.dumps({"contract": "p3-native-command-policy-approval-v1", "policy": "A",
                                    "policy_sha256": canonical_sha256(document["policies"]["A"]), "approved": True,
                                    "approved_by": "unit-test"}))
    return p3_mavlink.load_policy(policy_file(), "A", approval)


def test_policy_a_bounds(tmp_path):
    effective = approved_policy_a(tmp_path)
    window_end = 2_000_000_000
    ok = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin", "command401-v2.bin", "request512-v1.bin"),
                                    effective, window_end)
    assert ok["status"] == "passed" and ok["allowed_by_command"] == {401: 1, 512: 1}
    twice = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin", "command401-v2.bin", "command401-v2.bin"),
                                       effective, window_end)
    assert "payload_pattern_cap_exceeded" in twice["failures"]
    late = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin", "command401-v2.bin", stamp_ns=3_000_000_000),
                                      effective, window_end)
    assert "command_outside_startup_window" in late["failures"]
    for name in ("waypoint16-v2.bin", "cancel401-v2.bin"):
        result = p3_mavlink.evaluate_policy(frames("heartbeat-v2.bin", name), effective, window_end)
        assert "command_not_permitted_by_policy" in result["failures"], name


def test_incomplete_capture_never_passes():
    effective = p3_mavlink.load_policy(policy_file(), None)
    capture = frames("heartbeat-v2.bin")
    capture["complete"] = False
    assert "capture_incomplete" in p3_mavlink.evaluate_policy(capture, effective)["failures"]


def test_active_control_message_rejected():
    effective = p3_mavlink.load_policy(policy_file(), None)
    capture = frames("heartbeat-v2.bin")
    capture["frames_of_interest"].append({"message_id": 84, "source_system": 255, "source_component": 190,
                                          "active_control_message": True, "monotonic_ns": 1})
    assert "active_control_message_from_non_vehicle" in p3_mavlink.evaluate_policy(capture, effective)["failures"]


def test_udp_payload_extraction():
    payload = (FIXTURES / "heartbeat-v2.bin").read_bytes()
    udp = struct.pack("!HHHH", 18641, 14550, 8 + len(payload), 0) + payload
    ip = bytes([0x45, 0]) + struct.pack("!H", 20 + len(udp)) + b"\0\0\x40\0\x40\x11\0\0" + bytes([127, 0, 0, 1] * 2)
    packet = b"\0" * 12 + b"\x08\x00" + ip + udp
    assert p3_mavlink.udp_payload(packet) == (18641, 14550, payload)


# Authorization and gates ---------------------------------------------------------------------
def test_authorization_binds_gate_scope_and_manifest(tmp_path):
    sha = "a" * 64
    path = tmp_path / "auth.json"
    atomic_json(path, p3_manifest.authorization_template("A", sha, "operator"), exclusive=True)
    assert p3_manifest.require_authorization(path, "A", sha)["authorized_by"] == "operator"
    with pytest.raises(PermissionError):
        p3_manifest.require_authorization(path, "B", sha)
    with pytest.raises(PermissionError):
        p3_manifest.require_authorization(path, "A", "b" * 64)
    value = json.loads(path.read_text())
    value["authorized_by"] = ""
    path.write_text(json.dumps(value))
    with pytest.raises(PermissionError):
        p3_manifest.require_authorization(path, "A", sha)


def test_authorization_is_consumed_exactly_once(tmp_path):
    (tmp_path / "state").mkdir()
    auth = tmp_path / "auth.json"
    auth.write_text("{}")
    p3_manifest.consume(tmp_path, "A", "c" * 64, auth)
    with pytest.raises(FileExistsError):
        p3_manifest.consume(tmp_path, "A", "c" * 64, auth)


def test_gate_plan_and_ordering():
    assert [role for _, _, role in run_p3_gates.PLAN["A"]] == ["readiness"] + [
        "fixture-" + n for n in ("plane2", "plane4", "plane6", "oblique", "asymmetric", "out_of_range", "interruption")]
    assert run_p3_gates.PLAN["B"] == [("flight", None, "smoke")]
    assert [r for _, _, r in run_p3_gates.PLAN["C"]] == ["qualification-1", "qualification-2", "qualification-3"]


def test_gate_b_and_c_require_accepted_predecessor(tmp_path):
    (tmp_path / "state").mkdir()
    sha = "d" * 64
    with pytest.raises(PermissionError):
        run_p3_gates.require_predecessor(tmp_path, "B", sha)
    run = tmp_path / "run"
    run.mkdir()
    (run / "p3-evaluation.json").write_text('{"status":"passed"}')
    atomic_json(run_p3_gates.result_path(tmp_path, "A", sha),
                {"accepted": True, "manifest_sha256": sha, "runs": [
                    {"run": str(run), "evaluation_sha256": digest(run / "p3-evaluation.json")}]}, exclusive=True)
    assert run_p3_gates.require_predecessor(tmp_path, "B", sha)
    (run / "p3-evaluation.json").write_text('{"status":"passed","edited":true}')
    with pytest.raises(PermissionError, match="changed"):
        run_p3_gates.require_predecessor(tmp_path, "B", sha)
    with pytest.raises(PermissionError):
        run_p3_gates.require_predecessor(tmp_path, "C", sha)
    assert run_p3_gates.require_predecessor(tmp_path, "A", sha) is None


def test_gate_sequencer_refuses_without_explicit_gates(monkeypatch, tmp_path):
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH", "GWM_ALLOW_SITL_COMMANDS"):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(SystemExit, match="Missing explicit gates"):
        run_p3_gates.main(["--gate", "A", "--manifest", str(tmp_path / "m"), "--authorization", str(tmp_path / "a")])
    with pytest.raises(SystemExit, match="allow-simulated-flight"):
        run_p3_gates.main(["--gate", "B", "--manifest", str(tmp_path / "m"), "--authorization", str(tmp_path / "a")])


def test_gate_sequencer_requires_frozen_mc_selection_before_consuming(monkeypatch, tmp_path):
    # Gate A of c4b00bed consumed its authorization and then aborted on a missing
    # GWM_PX4_MC_TRACE selection; the sequencer must refuse before consumption.
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.setenv(key, "1")
    monkeypatch.delenv("GWM_PX4_MC_TRACE", raising=False)
    monkeypatch.setenv("GWM_SIM_ROOT", str(tmp_path))
    frozen = {"runtime": {"mc_trace_build": {"binary_sha256": "frozen"}}}
    monkeypatch.setattr(run_p3_gates.p3_manifest, "verify_current", lambda root, path: (frozen, "sha"))
    consumed = []
    monkeypatch.setattr(run_p3_gates.p3_manifest, "consume", lambda *a: consumed.append(a))
    with pytest.raises(SystemExit, match="GWM_PX4_MC_TRACE"):
        run_p3_gates.main(["--gate", "A", "--manifest", str(tmp_path / "m"), "--authorization", str(tmp_path / "a")])
    assert consumed == []


@POSIX_ONLY
def test_runner_refuses_outside_owned_namespace(monkeypatch, tmp_path):
    import p3_runner
    for key in p3_runner.GATES:
        monkeypatch.setenv(key, "1")
    args = p3_runner.argparse.Namespace(scope="readiness", allow_simulated_flight=False, manifest=tmp_path / "m",
                                        authorization=tmp_path / "a", gate="A", fixture=None, role="readiness")
    with pytest.raises(ValueError, match="namespace"):
        p3_runner.Runner(args).preconditions()


@POSIX_ONLY
def test_runner_flight_requires_flight_gates(monkeypatch, tmp_path):
    import p3_runner
    for key in p3_runner.GATES:
        monkeypatch.setenv(key, "1")
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)
    args = p3_runner.argparse.Namespace(scope="flight", allow_simulated_flight=True, manifest=tmp_path / "m",
                                        authorization=tmp_path / "a", gate="B", fixture=None, role="smoke")
    with pytest.raises(ValueError, match="SITL_COMMANDS"):
        p3_runner.Runner(args).preconditions()


@POSIX_ONLY
def test_no_runtime_on_default_invocation():
    completed = subprocess.run(["bash", str(SIM / "scripts/run_p3.sh")], text=True, capture_output=True)
    assert completed.returncode == 0 and "No runtime started" in completed.stdout
    completed = subprocess.run(["bash", str(SIM / "scripts/run_p3.sh"), "--run", "--scope", "readiness"],
                               text=True, capture_output=True, env={"PATH": os.environ["PATH"], "HOME": "/tmp"})
    assert completed.returncode == 2 and "Missing GWM_ALLOW_OPTIONAL_RUNTIME" in completed.stderr


# QGC unattended teardown evidence ------------------------------------------------------------
def teardown_log(tmp_path, mutate=None):
    from qgc_unattended import PREFIX, REQUIRED
    rows = [{"stage": stage} for stage in REQUIRED]
    rows.append({"stage": "native_attempt_audit_final", "command_attempts": "0", "outbound_attempts": "0",
                 "writer_healthy": True})
    if mutate:
        mutate(rows)
    path = tmp_path / "qgc.log"
    path.write_text("\n".join(PREFIX + json.dumps(row) for row in rows) + "\n")
    return path


def test_qgc_normal_teardown_requires_every_stage_and_exit_zero(tmp_path):
    from qgc_unattended import require_teardown
    clean = {"exit_code": 0, "forced": False, "registered_members_after": []}
    assert require_teardown(teardown_log(tmp_path), clean)["status"] == "passed"
    with pytest.raises(ValueError, match="not_graceful"):
        require_teardown(teardown_log(tmp_path), dict(clean, exit_code=-9, forced=True))
    with pytest.raises(ValueError, match="missing"):
        require_teardown(teardown_log(tmp_path, lambda rows: rows.pop(4)), clean)
    with pytest.raises(ValueError, match="protected"):
        require_teardown(teardown_log(tmp_path, lambda rows: rows.append(
            {"stage": "close_check", "detail": "unsaved_mission_blocked"})), clean)


def test_qgc_ownership_receipt_requires_namespace_owner(tmp_path):
    from qgc_unattended import ownership_receipt
    with pytest.raises(ValueError, match="PID_namespace"):
        ownership_receipt(tmp_path, None, None, None, SIM / "scripts/p3_namespace_owner.py", "0")


def test_qgc_overlay_keeps_protected_close_checks():
    from overlay import qml_overlay
    raw = (REPO / "tests/fixtures/qgc_unattended/pinned-close-block.qml").read_text()
    result = qml_overlay(raw)
    assert result.count("_forceClose = true") == 1
    assert result.index("checkForUnsavedMission()") < result.index("checkForPendingParameterWrites()") \
        < result.index("checkForActiveConnections()")
    assert "QGroundControl.linkManager.shutdown()" in result and "QGroundControl.videoManager.stopVideo();" in result


def test_qgc_build_binds_the_stable_namespace_owner():
    text = (SIM / "scripts/build_qgc_unattended.py").read_text()
    assert text.count("scripts/p3_namespace_owner.py") == 2 and "p3_final_owner" not in text


# Native source probe wire parsing ------------------------------------------------------------
def varint(value):
    out = bytearray()
    while True:
        byte = value & 127
        value >>= 7
        out.append(byte | (128 if value else 0))
        if not value:
            return bytes(out)


def field(number, kind, payload):
    if kind == 0:
        return varint(number << 3) + varint(payload)
    return varint(number << 3 | 2) + varint(len(payload)) + payload


def image_bytes(seq=7, width=4, height=2, step=16, sec=12, nsec=5000, data_bytes=None):
    stamp = field(1, 0, sec) + field(2, 0, nsec)
    data = b"".join(field(2, 2, field(1, 2, k.encode()) + field(2, 2, v.encode()))
                    for k, v in (("seq", str(seq)), ("frame_id", "camera_link")))
    header = field(1, 2, stamp) + data
    return (field(1, 2, header) + field(2, 0, width) + field(3, 0, height) + field(4, 0, step)
            + field(5, 2, b"\0" * (step * height if data_bytes is None else data_bytes)) + field(6, 0, 13))


def test_probe_wire_parser_reads_header_without_pixels():
    row = p3_source_probe.decode_image(image_bytes())
    assert (row["stamp_ns"], row["seq"], row["frame_id"], row["width"], row["height"]) == (
        12_000_005_000, 7, "camera_link", 4, 2)


def test_probe_wire_parser_rejects_inconsistent_payload():
    with pytest.raises(ValueError, match="payload_length"):
        p3_source_probe.decode_image(image_bytes(data_bytes=31))
    with pytest.raises(ValueError):
        p3_source_probe.decode_image(image_bytes()[:-5])


@POSIX_ONLY
def test_runner_tail_reads_only_complete_lines(tmp_path):
    from p3_runner import Tail
    path = tmp_path / "x.jsonl"
    path.write_text('{"a": 1}\n{"a": ')
    tail = Tail(path)
    assert tail.read() == [{"a": 1}]
    with path.open("a") as stream:
        stream.write('2}\n')
    assert tail.read() == [{"a": 2}] and tail.last == {"a": 2}


# Ordered shutdown ----------------------------------------------------------------------------
class FakeProcess:
    def __init__(self):
        self.code = None

    def poll(self):
        return self.code


def ordered_runner(monkeypatch, landed=True):
    import p3_runner
    calls = []
    runner = p3_runner.Runner.__new__(p3_runner.Runner)
    runner.scope, runner.validation = "flight", {"graceful_stop_wall_s": 15}
    runner.summary, runner.owned, runner.capture = {"world": "default"}, [], None
    runner.console = type("C", (), {"process": FakeProcess(), "fd": -1,
                                    "command": lambda self, c: calls.append("console:" + c)})()
    runner.px4 = type("P", (), {"mark_console_stop": lambda self: calls.append("px4_stop_marked"),
                                "wait": lambda self, t: True, "force": lambda self: calls.append("px4_forced")})()
    runner.gazebo = object()
    runner.controller = type("K", (), {"poll": lambda self: 1})()
    monkeypatch.setattr(runner, "wait_landed_disarmed", lambda limit: {"status": {}} if landed else None, raising=False)
    monkeypatch.setattr(runner, "stop", lambda name, sig=None: calls.append("stop:" + name), raising=False)
    monkeypatch.setattr(runner, "stop_qgc", lambda ground: calls.append("qgc_unattended"), raising=False)
    monkeypatch.setattr(runner, "drain_mc", lambda: calls.append("mc_drain"), raising=False)
    monkeypatch.setattr(runner, "pause_world", lambda: calls.append("pause_world"), raising=False)
    monkeypatch.setattr(p3_runner.os, "write", lambda fd, data: calls.append("write:" + data.decode().strip()))
    monkeypatch.setattr(p3_runner, "namespace_processes", lambda: [])
    return runner, calls


@POSIX_ONLY
def test_shutdown_order_after_controller_failure_drains_before_px4_exit(monkeypatch):
    runner, calls = ordered_runner(monkeypatch)
    errors = runner.shutdown()
    assert errors == []
    order = [c for c in calls if not c.startswith("console:logger")]
    assert order.index("qgc_unattended") < order.index("stop:sensor-bridge") < order.index("mc_drain")
    assert order.index("mc_drain") < order.index("write:shutdown") < order.index("stop:dds-agent")
    assert order.index("write:shutdown") < order.index("pause_world") < order.index("stop:gazebo")
    assert "console:logger stop" in calls


@POSIX_ONLY
def test_unexpected_exit_during_landed_wait_still_runs_ordered_stop(monkeypatch):
    # Gate A 8eaefbb6: the probe exited, the landed wait re-raised owned_process_exited
    # and no owned process was stopped. The ordered stop must still run.
    import p3_runner
    runner, calls = ordered_runner(monkeypatch)

    def landed(limit):
        raise p3_runner.Abort("owned_process_exited:source-probe")
    monkeypatch.setattr(runner, "wait_landed_disarmed", landed, raising=False)
    errors = runner.shutdown()
    assert errors[0] == "final_landed_disarmed_wait:Abort:owned_process_exited:source-probe"
    assert "final_landed_disarmed_not_independently_observed" in errors
    for name in ("sensor-bridge", "source-probe", "dds-agent", "gazebo"):
        assert "stop:" + name in calls
    assert "write:shutdown" in calls and "qgc_unattended" not in calls and "mc_drain" not in calls


@POSIX_ONLY
def test_px4_group_member_that_is_owned_gazebo_is_judged_by_its_own_record(monkeypatch):
    # Gate A 8feaa076: PX4's group still held Gazebo (pid 1288) at PX4 exit; Gazebo is owned,
    # stopped later and exited 0. An unowned leftover must still fail.
    runner, _ = ordered_runner(monkeypatch)

    def owned(name, pid, members):
        return type("O", (), {"name": name, "pid": pid, "poll": lambda self: 0,
                              "record": {"reaped": True, "remaining_group_members": members}})()
    px4, gazebo = owned("px4", 972, [1288, 4444]), owned("gazebo", 1288, [])
    runner.owned = [px4, gazebo]
    runner.shutdown()
    assert px4.record["remaining_group_members"] == [4444]
    assert px4.record["group_members_at_exit_raw"] == [1288, 4444]
    assert px4.record["group_members_owned_separately"] == [1288]


@POSIX_ONLY
def test_pump_never_aborts_the_ordered_shutdown(monkeypatch):
    import p3_runner
    runner, _ = ordered_runner(monkeypatch)
    exited = type("O", (), {"name": "source-probe", "poll": lambda self: 1,
                            "record": {"stop_requested_monotonic_s": None, "stop_method": "SIGINT"}})()
    runner.owned, runner.console, runner.shutting_down = [exited], None, False
    with pytest.raises(p3_runner.Abort, match="owned_process_exited:source-probe"):
        runner.pump()
    runner.shutting_down = True
    runner.pump()


def test_source_probe_keeps_the_sourced_ros_python_path():
    runner = (SIM / "scripts/p3_runner.py").read_text()
    launch = runner[runner.index('self.launch("source-probe"'):][:200]
    assert '"-s", "-P"' in launch and '"-I"' not in launch


@POSIX_ONLY
def test_unconfirmed_landing_skips_qgc_receipt_and_trace_stop(monkeypatch):
    runner, calls = ordered_runner(monkeypatch, landed=False)
    errors = runner.shutdown()
    assert "final_landed_disarmed_not_independently_observed" in errors
    assert "qgc_unattended" not in calls and "mc_drain" not in calls
