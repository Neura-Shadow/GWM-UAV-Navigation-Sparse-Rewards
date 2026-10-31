"""P5 M4: the real p5_control node under real rclpy, offline (no PX4/Gazebo) — the P4 harness, ported.

A helper node in the same process publishes /clock, PX4 output topics, the local map and adapter
health with BEST_EFFORT QoS. The accepted P2 `verify_timing` reads the node's trace and rosbag. The
node starts only after verifying the exported checkpoint. Requires a ROS-sourced Linux shell with
px4_msgs; skipped elsewhere.
"""
import json
from pathlib import Path
import random
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
P5 = ROOT / "simulation/px4_gazebo_p5"
for path in (ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control",
             ROOT / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
             P5 / "ros2_ws/src/gwm_policy_control", ROOT / "simulation/px4_gazebo/validation"):
    sys.path.insert(0, str(path))

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("rosbag2_py")
messages = pytest.importorskip("px4_msgs.msg")

from gwm_px4_control.contracts import TOPICS  # noqa: E402
from gwm_px4_control.execution_timing import controller_transport_env  # noqa: E402
from gwm_policy_control import contracts as p5_contracts  # noqa: E402

ARTIFACT = next(iter(sorted((P5 / "models").glob("p5-ranker-*.npz"))), None)
ORIGINAL_SPIN = rclpy.spin_once  # a test may run two controllers; never wrap a wrapper
pytestmark = pytest.mark.skipif(ARTIFACT is None, reason="no exported P5 model artifact")


def run_controller(tmp_path, monkeypatch, publish_px4, stop_when, wall_limit_s=8.0, controller="p5"):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from gwm_obstacle_avoidance import control_node as p4_node
    from gwm_policy_control import control_node as p5_node
    configs = ROOT / "simulation/px4_gazebo/configs"
    timing = json.loads((configs / "p2_timing.yaml").read_text())
    p4 = json.loads((ROOT / "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml").read_text())
    p4["enabled"] = True  # run-local copies, as the runner writes them
    sha = ARTIFACT.stem[len("p5-ranker-"):]
    p5 = json.loads((P5 / "configs/p5_policy.yaml").read_text())
    p5["enabled"] = True
    p5["model"].update(path=str(ARTIFACT), sha256=sha)
    schema = P5 / "configs/p5_model_schema.json"
    run = tmp_path / "run"
    run.mkdir()
    (run / "p4_avoidance.yaml").write_text(json.dumps(p4))
    (run / "p5_policy.yaml").write_text(json.dumps(p5))
    (run / "marker").write_text("{}")
    for key, value in {"GWM_ALLOW_OPTIONAL_RUNTIME": "1", "GWM_RUN_GAZEBO_PX4_TESTS": "1", "GWM_ALLOW_PX4_LAUNCH": "1",
                       "GWM_ALLOW_SITL_COMMANDS": "1", "GWM_P4_OWNED_NAMESPACE": "1",
                       "ROS_DOMAIN_ID": str(random.randint(150, 220)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                       **controller_transport_env(timing)}.items():
        monkeypatch.setenv(key, value)
    best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
    original_spin = ORIGINAL_SPIN
    state = {"helper": None, "sim_ns": 1_000_000_000, "start": time.monotonic()}

    def spin_once(node, timeout_sec=None):
        if state["helper"] is None:
            helper = state["helper"] = rclpy.create_node("p5_harness_publisher")
            state["clock"] = helper.create_publisher(Clock, "/clock", best_effort)
            state["map"] = helper.create_publisher(String, p4["topics"]["local_map"], best_effort)
            state["health"] = helper.create_publisher(String, p4["topics"]["adapter_health"], best_effort)
            state["px4"] = {key: (helper.create_publisher(getattr(messages, TOPICS[key][1]), meta["topic"], best_effort),
                                  getattr(messages, TOPICS[key][1]))
                            for key, meta in node.topic_map.items() if TOPICS[key][0] == "out"} if publish_px4 else {}
        state["sim_ns"] += 10_000_000
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(state["sim_ns"], 1_000_000_000)
        state["clock"].publish(clock)
        state["map"].publish(String(data=json.dumps({"usable": False})))
        state["health"].publish(String(data=json.dumps({"status": "fresh", "sim_s": state["sim_ns"] / 1e9})))
        for publisher, cls in state["px4"].values():
            message = cls()
            message.timestamp = state["sim_ns"] // 1000
            publisher.publish(message)
        original_spin(node, timeout_sec=0.01)
        if stop_when(node) or time.monotonic() - state["start"] > wall_limit_s:
            state["helper"].destroy_node()
            raise KeyboardInterrupt

    monkeypatch.setattr(rclpy, "spin_once", spin_once)
    common = ["--config", str(configs / "p2_control.yaml"), "--timing-config", str(configs / "p2_timing.yaml"),
              "--p4-config", str(run / "p4_avoidance.yaml"), "--run-dir", str(run),
              "--authorization-marker", str(run / "marker"), "--ground-diagnostic"]
    if controller == "p5":
        p5_node.main(common + ["--p5-config", str(run / "p5_policy.yaml"), "--schema", str(schema),
                               "--schema-sha256", p5_contracts.schema_sha256(p5_contracts.load_schema(schema)),
                               "--model", str(ARTIFACT), "--model-sha256", sha])
    else:
        p4_node.main(common)
    return run, json.loads((run / "controller-result.json").read_text())


def test_p5_controller_trace_passes_the_accepted_p2_verify_timing(tmp_path, monkeypatch):
    import collect_p2_evidence
    from gwm_px4_control.contracts import load_config
    from timing_evidence import verify_timing
    run, result = run_controller(tmp_path, monkeypatch, publish_px4=True,
                                 stop_when=lambda node: node.cache.stats and sum(
                                     v["unique"] for v in node.cache.stats.values()) > 50)
    streams, counts, events = collect_p2_evidence.read_bag(run)
    assert events and any(streams[key] for key in TOPICS if TOPICS[key][0] == "out")
    traces = [json.loads(line) for line in (run / "timing-trace.jsonl").read_text().splitlines()]
    assert sum(r["operation"].startswith("bag_write:") for r in traces) == sum(counts.values())
    verify_timing(run, result, streams, events, load_config(ROOT / "simulation/px4_gazebo/configs/p2_control.yaml"))
    assert result["controller"] == "p5_control" and result["p5"]["model_sha256"] == ARTIFACT.stem[len("p5-ranker-"):]
    assert result["p5"]["trained"] is True and result["p5"]["ledger_records"] == 0  # ground diagnostic: no TRANSIT
    verified = [e for e in events if e["event"] == "p5_policy_verified"]
    assert len(verified) == 1 and verified[0]["model_sha256"] == result["p5"]["model_sha256"]


def test_p5_controller_receives_best_effort_map_and_health(tmp_path, monkeypatch):
    _, result = run_controller(tmp_path, monkeypatch, publish_px4=False,
                               stop_when=lambda node: node.map_received > 5 and node.health_received > 5)
    assert result["local_map_messages"] > 5 and result["adapter_health_messages"] > 5


def test_p5_controller_traces_the_graph_check_like_the_accepted_p2_node(tmp_path, monkeypatch):
    import collect_p2_evidence
    from gwm_px4_control.contracts import load_config
    from timing_evidence import verify_timing
    run, result = run_controller(tmp_path, monkeypatch, publish_px4=False, stop_when=lambda node: False,
                                 wall_limit_s=2.6)
    traces = [json.loads(line) for line in (run / "timing-trace.jsonl").read_text().splitlines()]
    checks = [r for r in traces if r["operation"] == "graph_check"]
    writes = [r for r in traces if r["operation"] == "graph_json_persistence"]
    callbacks = [r for r in traces if r["operation"] == "control_callback"]
    ticks = [r for r in traces if r["operation"] == "mission_tick"]
    assert len(checks) >= 2 and len(writes) == len(checks)
    for check in checks:
        assert sum(check["start_wall_s"] <= w["start_wall_s"] <= w["end_wall_s"] <= check["end_wall_s"] for w in writes) == 1
        owner = [c for c in callbacks if c["start_wall_s"] <= check["start_wall_s"] <= check["end_wall_s"] <= c["end_wall_s"]]
        assert len(owner) == 1
        assert any(owner[0]["start_wall_s"] <= t["start_wall_s"] <= owner[0]["end_wall_s"]
                   and t["start_wall_s"] >= check["end_wall_s"] for t in ticks)
    starts = [c["start_wall_s"] for c in checks]
    assert all(b - a >= 1.0 for a, b in zip(starts, starts[1:]))
    streams, _, events = collect_p2_evidence.read_bag(run)
    report = verify_timing(run, result, streams, events, load_config(ROOT / "simulation/px4_gazebo/configs/p2_control.yaml"))
    assert report["graph_check_duration_wall_s"] == max(c["duration_wall_s"] for c in checks)


def test_the_policy_adds_no_publisher_to_the_p4_node(tmp_path, monkeypatch):
    seen = {}

    def record(name):
        def stop(node):
            seen[name] = sorted(p.topic_name for p in node.publishers)
            return True
        return stop
    for name in ("p4", "p5"):
        (tmp_path / name).mkdir()
        run_controller(tmp_path / name, monkeypatch, publish_px4=False, stop_when=record(name), controller=name)
    assert seen["p5"] == seen["p4"] and "/gwm/p2/events" in seen["p5"]


def test_ledger_records_are_persisted_after_the_dispatch_guard_in_their_own_span(tmp_path, monkeypatch):
    import collect_p2_evidence
    from gwm_px4_control.contracts import load_config
    from timing_evidence import verify_timing
    injected = [{"ledger": "p5-policy-ledger-v1", "tick": i, "sim_s": float(i), "invoked": False} for i in (1, 2, 3)]
    state = {"spins": 0}

    def inject(node):
        state["spins"] += 1
        if state["spins"] == 5:
            node.mission.ledger.extend(injected)
        return state["spins"] > 5 and not node.mission.ledger  # stop right after a tick persisted them
    run, result = run_controller(tmp_path, monkeypatch, publish_px4=False, stop_when=inject)
    lines = [json.loads(line) for line in (run / "p5/policy-ledger.jsonl").read_text().splitlines()]
    assert lines == injected
    traces = [json.loads(line) for line in (run / "timing-trace.jsonl").read_text().splitlines()]
    spans = [r for r in traces if r["operation"] == "policy_ledger_write"]
    callbacks = [r for r in traces if r["operation"] == "control_callback"]
    assert len(spans) == 1 and spans[0]["records"] == 3
    assert any(c["start_wall_s"] <= spans[0]["start_wall_s"] <= spans[0]["end_wall_s"] <= c["end_wall_s"] for c in callbacks)
    streams, _, events = collect_p2_evidence.read_bag(run)
    verify_timing(run, result, streams, events, load_config(ROOT / "simulation/px4_gazebo/configs/p2_control.yaml"))
