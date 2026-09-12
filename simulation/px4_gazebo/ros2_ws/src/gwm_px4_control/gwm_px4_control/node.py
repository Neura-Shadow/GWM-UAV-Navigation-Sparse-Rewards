"""ROS transport/recording adapter; imported safely without ROS installed."""
import argparse
import json
import math
import os
from pathlib import Path
import time

from .contracts import (TOPICS, encode_wire, gates, json_message, load_config,
                        offboard_mode, position_setpoint, strict_json, topic_name)
from .mission import Mission
from .timing import StateCache


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--flight", action="store_true")
    args = parser.parse_args(argv)
    gates(os.environ, args.flight)
    if os.getpid() == 1 or "GWM_P2_OWNED_NAMESPACE" not in os.environ:
        raise ValueError("Use the owned namespace launcher")
    c = load_config(args.config)
    # Optional dependencies appear only after explicit gates are checked.
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    import rosbag2_py
    import px4_msgs.msg as messages
    from rosgraph_msgs.msg import Clock as ClockMessage
    from std_msgs.msg import String

    class ControlNode(Node):
        def __init__(self):
            super().__init__("p2_control", namespace="/gwm",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.run = args.run_dir
            self.record = (self.run / "ros-events.jsonl").open("x", buffering=1)
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(self.run / "rosbag"), storage_id="sqlite3"),
                             rosbag2_py.ConverterOptions("cdr", "cdr"))
            self.registered = set()
            self.cache = StateCache(c)
            self.mission = Mission(c, self.cache, time.monotonic(), args.flight)
            self.graph_ok = False
            self.graph_checked = 0.0
            self.graph_record = {}
            self.topic_map = {}
            self.publishers_by_key = {}
            self.ros_subscriptions = []
            self.qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                                   reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            for key, (direction, class_name) in TOPICS.items():
                cls = getattr(messages, class_name)
                version = getattr(cls, "MESSAGE_VERSION", 0)
                topic = topic_name(c["ros_prefix"], key, direction, version)
                self.topic_map[key] = {"topic": topic, "type": "px4_msgs/msg/" + class_name,
                                       "version": version, "fields": cls.get_fields_and_field_types()}
                self.register(topic, "px4_msgs/msg/" + class_name)
                if direction == "out":
                    self.ros_subscriptions.append(self.create_subscription(cls, topic,
                        lambda msg, name=key: self.received(name, msg), self.qos))
                elif args.flight:
                    self.publishers_by_key[key] = self.create_publisher(cls, topic, self.qos)
            self.register("/clock", "rosgraph_msgs/msg/Clock")
            self.register("/gwm/p2/events", "std_msgs/msg/String")
            self.event_pub = self.create_publisher(String, "/gwm/p2/events", 10)
            self.ros_subscriptions.append(self.create_subscription(ClockMessage, "/clock", self.clock_received, self.qos))
            (self.run / "topic-contract.json").write_text(json.dumps(self.topic_map, indent=2, allow_nan=False)+"\n")
            self.timer = self.create_timer(1/c["rate_hz"], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.emit({"event": "startup", "flight_commands_enabled": args.flight, "use_sim_time": True,
                       "pid": os.getpid(), "identity": {k: c[k] for k in ("source_system", "source_component", "vehicle_system", "vehicle_component")}})

        def sim(self):
            return self.get_clock().now().nanoseconds/1e9

        def register(self, topic, type_name):
            if topic not in self.registered:
                self.writer.create_topic(rosbag2_py.TopicMetadata(id=len(self.registered), name=topic, type=type_name, serialization_format="cdr"))
                self.registered.add(topic)

        def bag(self, topic, message, sim):
            self.writer.write(topic, serialize_message(message), max(0, int(sim*1e9)))

        def emit(self, record):
            record = {"ros_sim_s": self.sim(), "monotonic_s": time.monotonic(), **record}
            encoded = strict_json(record)
            self.record.write(encoded+"\n")
            msg = String(data=encoded)
            self.event_pub.publish(msg)
            self.bag("/gwm/p2/events", msg, record["ros_sim_s"])

        def clock_received(self, msg):
            sim = msg.clock.sec + msg.clock.nanosec/1e9
            self.bag("/clock", msg, sim)
            try:
                self.cache.clock(sim, time.monotonic())
            except ValueError as exc:
                self.fail(str(exc))

        def received(self, key, msg):
            sim, wall = self.sim(), time.monotonic()
            data = dict(message_to_ordereddict(msg))
            self.bag(self.topic_map[key]["topic"], msg, sim)
            self.record.write(strict_json({"event": "received", "topic_key": key, "ros_sim_s": sim,
                                           "monotonic_s": wall, **json_message(data)})+"\n")
            try:
                if key == "vehicle_command_ack":
                    self.mission.ack(data, sim, wall)
                else:
                    self.cache.update(key, data, wall)
            except (ValueError, KeyError) as exc:
                self.fail(str(exc))

        def inspect_graph(self):
            all_topics = dict(self.get_topic_names_and_types())
            details = {}
            valid = True
            for key, meta in self.topic_map.items():
                topic = meta["topic"]
                pubs = self.get_publishers_info_by_topic(topic)
                subs = self.get_subscriptions_info_by_topic(topic)
                details[key] = {"topic": topic, "types": all_topics.get(topic, []),
                    "publishers": [{"node": info.node_namespace+"/"+info.node_name,
                                    "reliability": str(info.qos_profile.reliability),
                                    "durability": str(info.qos_profile.durability),
                                    "depth": info.qos_profile.depth} for info in pubs],
                    "subscription_count": len(subs)}
                if all_topics.get(topic) != [meta["type"]]:
                    valid = False
                if TOPICS[key][0] == "in":
                    expected = 1 if args.flight else 0
                    if len(pubs) != expected:
                        if len(pubs) > expected:
                            raise ValueError("competing_control_publisher:" + topic)
                        valid = False
                    if len(subs) < 1:
                        valid = False
                elif len(pubs) != 1:
                    valid = False
            self.graph_record, self.graph_ok = details, valid
            (self.run / "ros-graph.json").write_text(json.dumps(details, indent=2, allow_nan=False)+"\n")

        def send(self, key, fields):
            cls = getattr(messages, TOPICS[key][1])
            msg = cls()
            for name, value in fields.items():
                setattr(msg, name, value)
            self.publishers_by_key[key].publish(msg)
            self.bag(self.topic_map[key]["topic"], msg, self.sim())

        def send_command(self, request):
            if not args.flight:
                raise RuntimeError("Read-only stage cannot issue commands")
            fields = {"timestamp": int(self.sim()*1e6), "command": request["command"],
                      "target_system": c["vehicle_system"], "target_component": c["vehicle_component"],
                      "source_system": c["source_system"], "source_component": c["source_component"],
                      "from_external": True, "confirmation": 0}
            fields.update({f"param{i+1}": value for i, value in enumerate(request["params"])})
            self.emit({"event": "command_sent", "phase": self.mission.state, "fields": fields})
            self.send("vehicle_command", fields)

        def fail(self, reason):
            request = self.mission.abort(reason, self.sim(), time.monotonic())
            if request:
                self.send_command(request)

        def tick(self):
            try:
                if time.monotonic()-self.graph_checked >= 1.0:
                    self.inspect_graph()
                    self.graph_checked = time.monotonic()
                action = self.mission.tick(self.sim(), time.monotonic(), self.graph_ok)
                if action["command"]:
                    self.send_command(action["command"])
                if action["heartbeat_only"]:
                    self.send("offboard_control_mode", offboard_mode(int(self.sim()*1e6)))
                    self.emit({"event": "reference_pending_heartbeat", "phase": self.mission.state})
                if action["setpoint"]:
                    mode = action["setpoint"]["mode"]
                    fields = position_setpoint(action["setpoint"]["position"], action["setpoint"]["yaw"], int(self.sim()*1e6), mode)
                    self.send("offboard_control_mode", offboard_mode(fields["timestamp"]))
                    self.send("trajectory_setpoint", fields)
                    record = {"event": "setpoint", "phase": self.mission.state, **encode_wire(fields, mode)}
                    if self.mission.v3:
                        record.update(mode=mode, yaw_phase=action["setpoint"]["yaw_phase"])
                    self.emit(record)
                if action["sample"]:
                    self.emit({"event": "sample", "phase": self.mission.state, "sample": action["sample"]})
            except (Exception, KeyboardInterrupt) as exc:
                self.fail(str(exc))
            while self.mission.events:
                self.emit(self.mission.events.pop(0))

        def finish(self):
            result = {"connectivity": "passed" if self.mission.connectivity_passed else "failed",
                      "flight": "not_run" if not args.flight else ("passed" if self.mission.state == "COMPLETE" and not self.mission.failure else "failed"),
                      "failure": self.mission.failure, "final_state": self.mission.state,
                      "origin_ned": self.mission.origin, "initial_yaw_ned": self.mission.initial_yaw,
                      "windows": self.mission.windows, "transitions": self.mission.transitions,
                      "transactions": self.mission.transactions.records, "topic_rates": {},
                      "graph_valid": self.graph_ok, "qgc_monitor_present": True,
                      "manual_flight_commands": False, "ros_external_control_owner": args.flight}
            result["reference_policy"] = c.get("reference_policy", "p2-estimator-reference-v1")
            result["reference"] = self.mission.reference.summary() if self.mission.reference else None
            if self.mission.v3:
                result["handover"] = self.mission.handover
                result["max_initialization_drift_deg"] = math.degrees(self.mission.max_initialization_drift)
            for key, value in self.cache.stats.items():
                elapsed = (value["last_us"]-value["first_us"])/1e6
                result["topic_rates"][key] = {**value, "unique_rate_hz": (value["unique"]-1)/elapsed if elapsed > 0 else None}
            (self.run / "controller-result.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
            self.record.close()
            del self.writer
            return result

    rclpy.init(args=[])
    node = ControlNode()
    try:
        while rclpy.ok() and not node.mission.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.fail("controller_interrupted")
    finally:
        result = node.finish()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result["connectivity"] == "passed" and result["flight"] in ("passed", "not_run") and not result["failure"] else 1
