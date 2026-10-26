"""P4 perception node: second subscriber to the P3 depth image; publishes the local map.

Sensing only. It creates no PX4 input publisher; its single publication is
`/gwm/p4/local_map` (std_msgs/String JSON). Safe to import without ROS.
"""
import argparse
import json
import os
from pathlib import Path
import time

from .contracts import load_config, require_enabled, strict_json

GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
COST_CAPACITY = 100000


def require_gates(env):
    missing = [key for key in GATES if env.get(key) != "1"]
    if missing:
        raise ValueError("Missing explicit gates: " + ", ".join(missing))
    if os.getpid() == 1 or env.get("GWM_P4_OWNED_NAMESPACE") != "1":
        raise ValueError("Use the owned P4 namespace launcher")


def summarize(values):
    if not values:
        return {"count": 0, "max": None, "p50": None, "p95": None, "p99": None}
    ordered = sorted(values)
    pick = lambda q: ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))]
    return {"count": len(ordered), "max": ordered[-1], "p50": pick(0.5), "p95": pick(0.95), "p99": pick(0.99)}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path, help="run-local p4_avoidance.yaml")
    parser.add_argument("--sensors", required=True, type=Path, help="run-local p3_sensors.yaml")
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--authorization-marker", required=True, type=Path)
    parser.add_argument("--px4-prefix", default="/px4_71")
    args = parser.parse_args(argv)
    require_gates(os.environ)
    config = load_config(args.config)
    require_enabled(config, args.authorization_marker.is_file())
    from gwm_sensor_adapter.contracts import load_config as load_sensors
    sensors = load_sensors(args.sensors)
    folder = args.run_dir / "p4"
    folder.mkdir(exist_ok=True)

    import signal
    import threading

    import px4_msgs.msg as px4
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock as ClockMessage
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String

    from .perception import P4Processor

    def qos(depth):
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                          reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    class Perception(Node):
        def __init__(self):
            super().__init__("p4_perception", namespace="/gwm",
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.processor = P4Processor(sensors, config, args.run_dir.name)
            self.ledger = (folder / "local_map.jsonl").open("x", buffering=1)
            self.sim_time = None
            self.compute, self.latency, self.receipt_gaps = [], [], []
            self.last_receipt_wall = None
            self.gap_since_snapshot = 0.0
            self.snapshots = 0
            self.map_pub = self.create_publisher(String, config["topics"]["local_map"], qos(10))
            self.subs = [self.create_subscription(Image, sensors["image_topic"], self.image, qos(sensors["sensor_queue_depth"])),
                         self.create_subscription(CameraInfo, sensors["info_topic"], self.info, qos(sensors["sensor_queue_depth"])),
                         self.create_subscription(ClockMessage, "/clock", self.clock, qos(sensors["state_queue_depth"]))]
            for base, cls, callback in (("vehicle_local_position", px4.VehicleLocalPosition, self.position),
                                        ("vehicle_attitude", px4.VehicleAttitude, self.attitude)):
                version = getattr(cls, "MESSAGE_VERSION", 0)
                topic = f"{args.px4_prefix}/fmu/out/{base}" + (f"_v{version}" if version else "")
                self.subs.append(self.create_subscription(cls, topic, callback, qos(sensors["state_queue_depth"])))
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(sensors["poll_wall_s"], self.poll, clock=self.steady)
            self.started_wall = time.monotonic()

        def clock(self, msg):
            value = msg.clock.sec + msg.clock.nanosec / 1e9
            if self.sim_time is None or value >= self.sim_time:
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
            entry = time.monotonic()
            if self.last_receipt_wall is not None:
                self.gap_since_snapshot = max(self.gap_since_snapshot, entry - self.last_receipt_wall)
                if len(self.receipt_gaps) < COST_CAPACITY:
                    self.receipt_gaps.append(entry - self.last_receipt_wall)
            self.last_receipt_wall = entry
            fields = {"width": msg.width, "height": msg.height, "encoding": msg.encoding,
                      "is_bigendian": msg.is_bigendian, "step": msg.step, "data": memoryview(msg.data).cast("B")}
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            self.processor.on_image(fields, stamp, msg.header.frame_id, entry, self.sim_time)

        def position(self, msg):
            self.processor.on_position({k: getattr(msg, k) for k in (
                "timestamp", "timestamp_sample", "x", "y", "z", "heading", "xy_valid", "z_valid",
                "xy_reset_counter", "z_reset_counter", "heading_reset_counter", "ref_timestamp")})

        def attitude(self, msg):
            self.processor.on_attitude({"timestamp": msg.timestamp, "timestamp_sample": msg.timestamp_sample,
                                        "q": list(msg.q), "quat_reset_counter": msg.quat_reset_counter})

        def poll(self):
            entry = time.monotonic()
            _, snapshot = self.processor.poll(self.sim_time, entry)
            if snapshot is None:
                return
            compute = time.monotonic() - entry  # contract §6.1: per-frame wall time in this thread
            snapshot["compute_wall_s"] = compute
            snapshot["publication_sim_s"] = self.sim_time
            snapshot["latency_sim_s"] = None if self.sim_time is None else self.sim_time - snapshot["acquisition_sim_s"]
            snapshot["publication_wall_s"] = time.monotonic()
            # Largest image receipt gap since the previous snapshot, including the wait for the next frame.
            pending = 0.0 if self.last_receipt_wall is None else entry - self.last_receipt_wall
            snapshot["receipt_gap_max_wall_s"] = max(self.gap_since_snapshot, pending)
            self.gap_since_snapshot = 0.0
            if len(self.compute) < COST_CAPACITY:
                self.compute.append(compute)
                if snapshot["latency_sim_s"] is not None:
                    self.latency.append(snapshot["latency_sim_s"])
            encoded = strict_json(snapshot)
            self.map_pub.publish(String(data=encoded))
            self.ledger.write(encoded + "\n")
            self.snapshots += 1

    stop_requested = threading.Event()
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        signal.signal(stop_signal, lambda *_: stop_requested.set())
    node = executor = None
    result = {"contract": "p4-perception-result-v1", "run_id": args.run_dir.name, "status": "failed",
              "error": None, "pid": os.getpid()}
    try:
        node = Perception()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            while not stop_requested.is_set():
                executor.spin_once(timeout_sec=0.05)
            result["stop"] = "SIGINT"
        except (KeyboardInterrupt, ExternalShutdownException):
            result["stop"] = "SIGINT"
    except Exception as exc:  # recorded and returned as failure, never hidden
        result["error"] = type(exc).__name__ + ":" + str(exc)
    finally:
        if node is not None:
            node.ledger.flush()
            os.fsync(node.ledger.fileno())
            node.ledger.close()
            processor = node.processor
            result.update(snapshots=node.snapshots, usable=processor.usable,
                          unusable_reasons=processor.unusable_reasons,
                          accounting=processor.accounting.as_dict(), health_transitions=processor.transitions,
                          calibration_identity=(processor.calibration.calibration or {}).get("identity"),
                          compute_wall_s=summarize(node.compute), latency_sim_s=summarize(node.latency),
                          receipt_gap_wall_s=summarize(node.receipt_gaps),
                          samples_truncated=len(node.compute) >= COST_CAPACITY,
                          wall_duration_s=time.monotonic() - node.started_wall)
            if result["error"] is None:
                result["status"] = "passed"
            if executor is not None:
                executor.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with (folder / "perception-result.json").open("x") as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
    return 0 if result["status"] == "passed" else 3


if __name__ == "__main__":
    raise SystemExit(main())
