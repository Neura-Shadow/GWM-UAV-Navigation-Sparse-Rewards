"""P4 Gate A 1628fc44 regressions: the real p4_control node under real rclpy, offline (no PX4/Gazebo).

A helper node in the same process publishes /clock, PX4 output topics, the local map and adapter
health with BEST_EFFORT QoS, as the P3 adapter and P4 perception do. The accepted P2
`verify_timing` then reads the node's trace and rosbag. Requires a ROS-sourced Linux shell with
px4_msgs; skipped elsewhere.
"""
import json
import os
from pathlib import Path
import random
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
P2_PACKAGE = ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control"
P4_PACKAGE = ROOT / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance"
for path in (P2_PACKAGE, P4_PACKAGE, ROOT / "simulation/px4_gazebo/validation"):
    sys.path.insert(0, str(path))

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("rosbag2_py")
messages = pytest.importorskip("px4_msgs.msg")

from gwm_px4_control.contracts import TOPICS  # noqa: E402
from gwm_px4_control.execution_timing import controller_transport_env  # noqa: E402


def run_controller(tmp_path, monkeypatch, publish_px4, stop_when, wall_limit_s=8.0):
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from gwm_obstacle_avoidance import control_node
    configs = ROOT / "simulation/px4_gazebo/configs"
    timing = json.loads((configs / "p2_timing.yaml").read_text())
    p4 = json.loads((ROOT / "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml").read_text())
    p4["enabled"] = True  # run-local copy, as the P4 runner writes it
    run = tmp_path / "run"
    run.mkdir()
    (run / "p4_avoidance.yaml").write_text(json.dumps(p4))
    (run / "marker").write_text("{}")
    for key, value in {"GWM_ALLOW_OPTIONAL_RUNTIME": "1", "GWM_RUN_GAZEBO_PX4_TESTS": "1", "GWM_ALLOW_PX4_LAUNCH": "1",
                       "GWM_ALLOW_SITL_COMMANDS": "1", "GWM_P4_OWNED_NAMESPACE": "1",
                       "ROS_DOMAIN_ID": str(random.randint(150, 220)), "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                       **controller_transport_env(timing)}.items():
        monkeypatch.setenv(key, value)
    best_effort = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
    original_spin = rclpy.spin_once
    state = {"helper": None, "sim_ns": 1_000_000_000, "start": time.monotonic()}

    def spin_once(node, timeout_sec=None):
        if state["helper"] is None:
            helper = state["helper"] = rclpy.create_node("p4_harness_publisher")
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
    control_node.main(["--config", str(configs / "p2_control.yaml"), "--timing-config", str(configs / "p2_timing.yaml"),
                       "--p4-config", str(run / "p4_avoidance.yaml"), "--run-dir", str(run),
                       "--authorization-marker", str(run / "marker"), "--ground-diagnostic"])
    return run, json.loads((run / "controller-result.json").read_text())


def test_p4_controller_trace_passes_the_accepted_p2_verify_timing(tmp_path, monkeypatch):
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


def test_p4_controller_receives_best_effort_map_and_health(tmp_path, monkeypatch):
    _, result = run_controller(tmp_path, monkeypatch, publish_px4=False,
                               stop_when=lambda node: node.map_received > 5 and node.health_received > 5)
    assert result["local_map_messages"] > 5 and result["adapter_health_messages"] > 5
