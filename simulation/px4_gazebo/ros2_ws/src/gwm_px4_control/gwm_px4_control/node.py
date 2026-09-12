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
from .execution_timing import TraceBuffer, transport_allowed, DispatchGuard, PublicationCoverage, controller_transport_env


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--flight", action="store_true")
    parser.add_argument("--ground-diagnostic", action="store_true")
    parser.add_argument("--timing-config", required=True, type=Path)
    args = parser.parse_args(argv)
    gates(os.environ, args.flight or args.ground_diagnostic)
    if args.flight and args.ground_diagnostic:
        raise ValueError("Ground diagnostic cannot enable flight commands")
    if os.getpid() == 1 or "GWM_P2_OWNED_NAMESPACE" not in os.environ:
        raise ValueError("Use the owned namespace launcher")
    c = load_config(args.config)
    tc = json.loads(args.timing_config.read_text())
    active = args.flight or args.ground_diagnostic
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
            self.trace = TraceBuffer(tc["trace_capacity"])
            self.start_wall = time.monotonic()
            self.guard = DispatchGuard(tc["dispatch_budget_wall_s"],tc["graph_fresh_wall_s"])
            self.coverage = PublicationCoverage(c["max_sample_gap_sim_s"])
            self.recovery_request = None
            self.evidence_error = None
            self.transport_environment = {k:os.environ.get(k) for k in controller_transport_env(tc)}
            if self.transport_environment != controller_transport_env(tc):
                raise ValueError("Controller transport environment mismatch")
            self.record = (self.run / "ros-events.jsonl").open("x", buffering=1)
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(self.run / "rosbag"), storage_id="sqlite3"),
                             rosbag2_py.ConverterOptions("cdr", "cdr"))
            self.registered = set()
            self.cache = StateCache(c)
            self.mission = Mission(c, self.cache, self.start_wall, active,
                                   args.ground_diagnostic, tc["ground_prestream_sim_s"])
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
                with self.trace.span("type_support_prepare:"+key, self.sim()):
                    cls = getattr(messages, class_name)
                version = getattr(cls, "MESSAGE_VERSION", 0)
                topic = topic_name(c["ros_prefix"], key, direction, version)
                self.topic_map[key] = {"topic": topic, "type": "px4_msgs/msg/" + class_name,
                                       "version": version, "fields": cls.get_fields_and_field_types()}
                self.register(topic, "px4_msgs/msg/" + class_name)
                if direction == "out":
                    self.ros_subscriptions.append(self.create_subscription(cls, topic,
                        lambda msg, name=key: self.received(name, msg), self.qos))
                elif active and not (args.ground_diagnostic and key == "vehicle_command"):
                    with self.trace.span("publisher_prepare:"+key, self.sim()):
                        self.publishers_by_key[key] = self.create_publisher(cls, topic, self.qos)
            self.register("/clock", "rosgraph_msgs/msg/Clock")
            self.register("/gwm/p2/events", "std_msgs/msg/String")
            self.event_pub = self.create_publisher(String, "/gwm/p2/events", 10)
            self.ros_subscriptions.append(self.create_subscription(ClockMessage, "/clock", self.clock_received, self.qos))
            (self.run / "topic-contract.json").write_text(json.dumps(self.topic_map, indent=2, allow_nan=False)+"\n")
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1/c["rate_hz"], self.tick, clock=self.steady)
            self.emit({"event": "startup", "flight_commands_enabled": args.flight, "use_sim_time": True,
                       "pid": os.getpid(), "identity": {k: c[k] for k in ("source_system", "source_component", "vehicle_system", "vehicle_component")}})

        def sim(self):
            return self.get_clock().now().nanoseconds/1e9

        def register(self, topic, type_name):
            if topic not in self.registered:
                self.writer.create_topic(rosbag2_py.TopicMetadata(id=len(self.registered), name=topic, type=type_name, serialization_format="cdr"))
                self.registered.add(topic)

        def bag(self, topic, message, sim):
            try:
                with self.trace.span("cdr_serialize:"+topic, sim):
                    serialized = serialize_message(message)
                with self.trace.span("bag_write:"+topic, sim):
                    self.writer.write(topic, serialized, max(0, int(sim*1e9)))
            except Exception:
                self.evidence_error = "bag_write_failed"
                raise

        def emit(self, record):
            record = {"ros_sim_s": self.sim(), "monotonic_s": time.monotonic(), **record}
            with self.trace.span("json_encode:event", record["ros_sim_s"]):
                encoded = strict_json(record)
            with self.trace.span("text_write_flush:event", record["ros_sim_s"]):
                self.record.write(encoded+"\n")
            msg = String(data=encoded)
            with self.trace.span("event_publish", record["ros_sim_s"]):
                self.event_pub.publish(msg)
            self.bag("/gwm/p2/events", msg, record["ros_sim_s"])

        def clock_received(self, msg):
            entry = time.monotonic()
            sim = msg.clock.sec + msg.clock.nanosec/1e9
            self.bag("/clock", msg, sim)
            try:
                self.cache.clock(sim, time.monotonic())
            except ValueError as exc:
                self.fail(str(exc))
            self.trace.append("clock_callback", entry, time.monotonic(), sim)

        def received(self, key, msg):
            sim, wall = self.sim(), time.monotonic()
            with self.trace.span("message_convert:"+key, sim):
                data = dict(message_to_ordereddict(msg))
            self.bag(self.topic_map[key]["topic"], msg, sim)
            with self.trace.span("json_encode:"+key, sim):
                encoded = strict_json({"event": "received", "topic_key": key, "ros_sim_s": sim,
                                       "monotonic_s": wall, **json_message(data)})+"\n"
            with self.trace.span("text_write_flush:"+key, sim):
                self.record.write(encoded)
            try:
                if key == "vehicle_command_ack":
                    self.mission.ack(data, sim, wall)
                else:
                    with self.trace.span("cache_update:"+key, sim, source_timestamp=data["timestamp"], callback_entry_wall_s=wall):
                        self.cache.update(key, data, wall)
            except (ValueError, KeyError) as exc:
                self.fail(str(exc))
            self.trace.append("subscription_callback:"+key, wall, time.monotonic(), sim, source_timestamp=data["timestamp"])

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
                    expected = 1 if active and not (args.ground_diagnostic and key=="vehicle_command") else 0
                    if len(pubs) != expected:
                        if len(pubs) > expected:
                            raise ValueError("competing_control_publisher:" + topic)
                        valid = False
                    if len(subs) < 1:
                        valid = False
                elif len(pubs) != 1:
                    valid = False
            self.graph_record, self.graph_ok = details, valid
            with self.trace.span("graph_json_persistence", self.sim()):
                (self.run / "ros-graph.json").write_text(json.dumps(details, indent=2, allow_nan=False)+"\n")

        def check_dispatch(self, recovery=False):
            self.guard.check(time.monotonic(),self.graph_checked,self.graph_ok)
            if self.evidence_error and not recovery:
                raise ValueError(self.evidence_error)
            with self.trace.span("dispatch_validation",self.sim(),phase=self.mission.state):
                sample=self.cache.validate(self.sim(),time.monotonic())
            return sample

        def send(self, key, fields, recovery=False):
            transport_allowed(key, args.flight, args.ground_diagnostic)
            with self.trace.span("message_construct:"+key, self.sim(), source_timestamp=fields["timestamp"]):
                cls = getattr(messages, TOPICS[key][1])
                msg = cls()
                for name, value in fields.items():
                    setattr(msg, name, value)
            sample=self.check_dispatch(recovery)
            publication = {"entry_sim_s":self.sim(),"entry_wall_s":time.monotonic(),
                "selected_source_timestamp":round(sample["t"]*1e6),
                "source_age_at_dispatch_sim_s":self.sim()-sample["t"]}
            with self.trace.span("publish:"+key, self.sim(), source_timestamp=fields["timestamp"],
                    selected_source_timestamp=publication["selected_source_timestamp"],
                    source_age_at_dispatch_sim_s=publication["source_age_at_dispatch_sim_s"]) as measurement:
                cpu_start = time.thread_time()
                self.publishers_by_key[key].publish(msg)
                measurement["thread_cpu_s"] = time.thread_time()-cpu_start
            publication.update(return_wall_s=time.monotonic(),return_sim_s=self.sim())
            self.bag(self.topic_map[key]["topic"], msg, publication["entry_sim_s"])
            return publication

        def send_command(self, request, recovery=False):
            if not args.flight:
                raise RuntimeError("Read-only stage cannot issue commands")
            fields = {"timestamp": int(self.sim()*1e6), "command": request["command"],
                      "target_system": c["vehicle_system"], "target_component": c["vehicle_component"],
                      "source_system": c["source_system"], "source_component": c["source_component"],
                      "from_external": True, "confirmation": 0}
            fields.update({f"param{i+1}": value for i, value in enumerate(request["params"])})
            self.check_dispatch(recovery)
            if request["command"] == 176:
                self.coverage.require(c["prestream_sim_s"])
            publication = self.send("vehicle_command", fields, recovery)
            self.emit({"event": "command_sent", "phase": self.mission.state, "fields": fields,
                       "publication": publication, "evidence_schema": 2})

        def fail(self, reason):
            request = self.mission.abort(reason, self.sim(), time.monotonic())
            if request:
                self.recovery_request = request  # validate fresh state in next owned callback

        def tick(self):
            entry = time.monotonic()
            steady_entry = self.steady.now().nanoseconds
            expected = steady_entry+self.timer.time_until_next_call()-self.timer.timer_period_ns
            self.guard.begin(entry)
            try:
                if self.trace.overflow:
                    raise ValueError("timing_trace_overflow")
                if args.ground_diagnostic and entry-self.start_wall > tc["ground_wall_s"]:
                    raise TimeoutError("ground_diagnostic_deadline")
                if time.monotonic()-self.graph_checked >= 1.0:
                    with self.trace.span("graph_check", self.sim()):
                        self.inspect_graph()
                    self.graph_checked = time.monotonic()
                with self.trace.span("mission_tick", self.sim(), phase=self.mission.state) as selection:
                    try:
                        action = self.mission.tick(self.sim(), time.monotonic(), self.graph_ok)
                    finally:
                        selected = self.mission.control_selection
                        if selected:
                            selection.update(selected_source_timestamp=round(selected["t"]*1e6),
                                selection_wall_s=selected["selection_monotonic_s"],
                                callback_entry_wall_s=selected["source_callback_entry_monotonic_s"],
                                source_age_at_control_sim_s=selected["ros_sim_s"]-selected["t"])
                if action["heartbeat_only"]:
                    self.send("offboard_control_mode", offboard_mode(int(self.sim()*1e6)))
                    self.emit({"event": "reference_pending_heartbeat", "phase": self.mission.state})
                if action["setpoint"]:
                    mode = action["setpoint"]["mode"]
                    fields = position_setpoint(action["setpoint"]["position"], action["setpoint"]["yaw"], int(self.sim()*1e6), mode)
                    self.send("offboard_control_mode", offboard_mode(fields["timestamp"]))
                    publication=self.send("trajectory_setpoint", fields)
                    if self.mission.state in ("PRESTREAM_SAFE_SETPOINTS","REQUEST_OFFBOARD"):
                        self.coverage.record(publication["entry_sim_s"])
                    record = {"event": "setpoint", "phase": self.mission.state,
                              "evidence_schema":2,"publication":publication,**encode_wire(fields, mode)}
                    if self.mission.v3:
                        record.update(mode=mode, yaw_phase=action["setpoint"]["yaw_phase"])
                    self.emit(record)
                if action["command"]:
                    self.send_command(action["command"])
                if self.recovery_request:
                    request,self.recovery_request=self.recovery_request,None
                    self.send_command(request,recovery=True)
                if action["sample"]:
                    self.emit({"event": "sample", "phase": self.mission.state, "sample": action["sample"]})
                if action["setpoint"] or action["command"]:
                    self.guard.check(time.monotonic(),self.graph_checked,self.graph_ok)
            except (Exception, KeyboardInterrupt) as exc:
                self.fail(str(exc))
            finally:
                self.guard.end()
            while self.mission.events:
                self.emit(self.mission.events.pop(0))
            self.trace.append("control_callback",entry,time.monotonic(),self.sim(),
                actual_entry_steady_ns=steady_entry,derived_expected_steady_ns=expected,phase=self.mission.state)

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
            result["timing_contract"] = tc["contract"]
            result["timing_configuration"] = tc
            result["transport_environment"] = self.transport_environment
            result["prestream_publication_coverage"] = {"first_sim_s":self.coverage.first,
                "last_sim_s":self.coverage.last,"count":self.coverage.count}
            result["evidence_schema"] = 2
            result["evidence_error"] = self.evidence_error
            result["ground_diagnostic"] = ("failed" if self.mission.failure else "passed") if args.ground_diagnostic else "not_run"
            result["trace"] = self.trace.persist(self.run/"timing-trace.jsonl")
            if not result["trace"]["complete"]:
                result["failure"] = result["failure"] or "timing_trace_overflow"
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
    except Exception as exc:
        node.fail("adapter_exception:"+str(exc))
    finally:
        result = node.finish()
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result["connectivity"] == "passed" and result["flight"] in ("passed", "not_run") and not result["failure"] else 1
