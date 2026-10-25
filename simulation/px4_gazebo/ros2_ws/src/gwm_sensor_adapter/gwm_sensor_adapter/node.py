"""ROS 2 depth adapter. Sensing only: no PX4 input publisher can be created.

Safe to import without ROS. ROS modules load only inside `main()` after the
explicit runtime gates and owned-namespace marker are checked.
"""
import argparse
import json
import os
from pathlib import Path
import time

from .contracts import check_publication, load_config, strict_json
from .pipeline import DepthProcessor, summarize_costs
from .recorder import RawRecorder

GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
COST_CAPACITY = 100000


def require_gates(env):
    missing = [key for key in GATES if env.get(key) != "1"]
    if missing:
        raise ValueError("Missing explicit gates: " + ", ".join(missing))
    if os.getpid() == 1 or env.get("GWM_P3_OWNED_NAMESPACE") != "1":
        raise ValueError("Use the owned P3 namespace launcher")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--require-px4-state", action="store_true")
    parser.add_argument("--px4-prefix", default="/px4_71")
    args = parser.parse_args(argv)
    require_gates(os.environ)
    config = load_config(args.config)
    folder = args.run_dir / "sensors"
    folder.mkdir(exist_ok=False)

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMessage
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    def qos(depth):
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                          reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    class DepthAdapter(Node):
        def __init__(self):
            super().__init__("p3_depth_adapter", namespace="/gwm",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.processor = DepthProcessor(config, args.run_dir.name, args.require_px4_state)
            self.recorder = RawRecorder(folder, config["recorder_capacity"], config["recorder_compression"])
            self.events = (folder / "observations.jsonl").open("x", buffering=1)
            self.health_log = (folder / "health.jsonl").open("x", buffering=1)
            self.sim_time = None
            self.clock_backwards = 0
            self.image_costs, self.poll_costs = [], []
            self.last_health_wall = None
            self.publications = {}
            self.error = None
            self.observed_publications = None
            self.last_graph_wall = None
            self.observation_pub = self.guarded_publisher(String, config["observation_topic"], "std_msgs/msg/String")
            self.health_pub = self.guarded_publisher(String, config["health_topic"], "std_msgs/msg/String")
            self.subscriptions_list = [
                self.create_subscription(Image, config["image_topic"], self.image, qos(config["sensor_queue_depth"])),
                self.create_subscription(CameraInfo, config["info_topic"], self.info, qos(config["sensor_queue_depth"])),
                self.create_subscription(ClockMessage, "/clock", self.clock, qos(config["state_queue_depth"]))]
            if args.require_px4_state:
                import px4_msgs.msg as px4
                for base, cls, callback in (("vehicle_local_position", px4.VehicleLocalPosition, self.position),
                                            ("vehicle_attitude", px4.VehicleAttitude, self.attitude)):
                    version = getattr(cls, "MESSAGE_VERSION", 0)
                    topic = f"{args.px4_prefix}/fmu/out/{base}" + (f"_v{version}" if version else "")
                    self.subscriptions_list.append(self.create_subscription(
                        cls, topic, callback, qos(config["state_queue_depth"])))
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(config["poll_wall_s"], self.poll, clock=self.steady)
            self.started_wall = time.monotonic()

        def guarded_publisher(self, cls, topic, type_name):
            check_publication(config, topic, type_name)
            self.publications[topic] = type_name
            return self.create_publisher(cls, topic, qos(10))

        def clock(self, msg):
            value = msg.clock.sec + msg.clock.nanosec / 1e9
            if self.sim_time is not None and value < self.sim_time:
                self.clock_backwards += 1
                return
            self.sim_time = value

        def info(self, msg):
            fields = {"width": msg.width, "height": msg.height, "distortion_model": msg.distortion_model,
                      "d": list(msg.d), "k": list(msg.k), "p": list(msg.p), "r": list(msg.r),
                      "binning_x": msg.binning_x, "binning_y": msg.binning_y,
                      "roi": {"x_offset": msg.roi.x_offset, "y_offset": msg.roi.y_offset, "height": msg.roi.height,
                              "width": msg.roi.width, "do_rectify": msg.roi.do_rectify}}
            header = {"stamp_ns": msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
                      "frame_id": msg.header.frame_id}
            self.processor.on_camera_info(fields, header)

        def image(self, msg):
            entry, cpu = time.monotonic(), time.thread_time()
            payload = memoryview(msg.data).cast("B")
            fields = {"width": msg.width, "height": msg.height, "encoding": msg.encoding,
                      "is_bigendian": msg.is_bigendian, "step": msg.step, "data": payload}
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            accepted, record = self.processor.on_image(fields, stamp, msg.header.frame_id, entry, self.sim_time)
            if not self.recorder.submit(record, payload):
                self.error = self.error or "raw_recorder_overflow"
            if len(self.image_costs) < COST_CAPACITY:
                self.image_costs.append((time.monotonic() - entry, time.thread_time() - cpu))

        def position(self, msg):
            self.processor.on_position({k: getattr(msg, k) for k in (
                "timestamp", "timestamp_sample", "x", "y", "z", "heading", "xy_valid", "z_valid",
                "xy_reset_counter", "z_reset_counter", "heading_reset_counter", "ref_timestamp")})

        def attitude(self, msg):
            self.processor.on_attitude({"timestamp": msg.timestamp, "timestamp_sample": msg.timestamp_sample,
                                        "q": list(msg.q), "quat_reset_counter": msg.quat_reset_counter})

        def poll(self):
            entry = time.monotonic()
            changed, observation = self.processor.poll(self.sim_time, entry)
            if changed or self.last_health_wall is None or entry - self.last_health_wall >= 1.0:
                record = self.processor.health(self.sim_time, entry)
                encoded = strict_json(record)
                self.health_log.write(encoded + "\n")
                self.health_pub.publish(String(data=encoded))
                self.last_health_wall = entry
            if observation is not None:
                encoded = strict_json(observation)
                self.events.write(encoded + "\n")
                self.observation_pub.publish(String(data=encoded))
            if self.last_graph_wall is None or entry - self.last_graph_wall >= 5.0:
                # Graph queries need a live context, so the observed set is sampled during spin.
                self.observed_publications = self.own_publications()
                self.last_graph_wall = entry
            if len(self.poll_costs) < COST_CAPACITY:
                self.poll_costs.append(time.monotonic() - entry)

        def own_publications(self):
            return {name: types for name, types in
                    self.get_publisher_names_and_types_by_node(self.get_name(), self.get_namespace())}

    rclpy.init(args=None)
    node = None
    result = {"contract": "p3-depth-adapter-result-v1", "run_id": args.run_dir.name, "status": "failed",
              "error": None, "require_px4_state": args.require_px4_state, "pid": os.getpid()}
    try:
        node = DepthAdapter()
        result["declared_publications"] = node.publications
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            result["stop"] = "SIGINT"
    except Exception as exc:  # recorded and returned as failure, never hidden
        result["error"] = type(exc).__name__ + ":" + str(exc)
    finally:
        if node is not None:
            result["observed_own_publications"] = node.observed_publications
            recorder = node.recorder.close(config["recorder_close_wall_s"])
            for stream in (node.events, node.health_log):
                stream.flush()
                os.fsync(stream.fileno())
                stream.close()
            image = summarize_costs([cost[0] for cost in node.image_costs])
            result.update(recorder=recorder, accounting=node.processor.accounting.as_dict(),
                          health_transitions=node.processor.transitions,
                          calibration=node.processor.calibration.calibration,
                          calibration_first_header=node.processor.calibration.first_header,
                          calibration_rejected=node.processor.calibration.rejected,
                          calibration_invalid_reason=node.processor.calibration.invalid_reason,
                          association_outcomes=node.processor.associator.outcomes,
                          state_history=dict(position=node.processor.associator.position.counts,
                                             attitude=node.processor.associator.attitude.counts),
                          clock_backwards=node.clock_backwards,
                          costs={"image_callback_wall": image,
                                 "image_callback_thread_cpu": summarize_costs([c[1] for c in node.image_costs]),
                                 "poll_wall": summarize_costs(node.poll_costs),
                                 "cost_samples_truncated": len(node.image_costs) >= COST_CAPACITY},
                          wall_duration_s=time.monotonic() - node.started_wall)
            result["error"] = result["error"] or node.error
            if result["error"] is None and recorder["complete"]:
                result["status"] = "passed"
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with (folder / "adapter-result.json").open("x") as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    return 0 if result["status"] == "passed" else 3


if __name__ == "__main__":
    raise SystemExit(main())
