"""P6 passive witness (contract §7.2). Safe to import without ROS. It never creates a publisher.

It records, side by side, each original topic and its relayed copy:
* depth image and camera info: raw bridge output and relay output;
* perception's two PX4 state topics: `/px4_71` and `/gwm/p6/px4_71`.

Each receipt goes to `p6/witness/<stream>.jsonl` with its stamp, receipt sim and wall time, and a
content fingerprint (CRC-32 of the payload). The evaluator proves an injection from these records,
never from the injector's ledger alone.
"""
import argparse
import json
import signal
import threading
import time
import zlib
from pathlib import Path

from .qos import clock_profile

DEPTH = (("raw_image", "/gwm/p6/raw/front_depth/image_raw"), ("relayed_image", "/gwm/sensors/front_depth/image_raw"),
         ("raw_info", "/gwm/p6/raw/front_depth/camera_info"), ("relayed_info", "/gwm/sensors/front_depth/camera_info"))
STATE = (("true_position", "/px4_71", "vehicle_local_position"), ("relayed_position", "/gwm/p6/px4_71", "vehicle_local_position"),
         ("true_attitude", "/px4_71", "vehicle_attitude"), ("relayed_attitude", "/gwm/p6/px4_71", "vehicle_attitude"))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--depth", action="store_true")
    parser.add_argument("--state", action="store_true")
    args = parser.parse_args(argv)
    folder = args.run_dir / "p6" / "witness"
    folder.mkdir(parents=True, exist_ok=True)

    import px4_msgs.msg as px4
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image

    stop = threading.Event()
    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, lambda *_: stop.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)  # matches any publisher, like the accepted consumers

    class Witness(Node):
        def __init__(self):
            super().__init__("p6_witness", namespace="/gwm", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.sim = None
            self.streams = {}
            self.create_subscription(Clock, "/clock", self.clock, clock_profile())
            if args.depth:
                for name, topic in DEPTH:
                    cls = Image if name.endswith("image") else CameraInfo
                    self.watch(name, cls, topic)
            if args.state:
                for name, prefix, base in STATE:
                    cls = px4.VehicleLocalPosition if base == "vehicle_local_position" else px4.VehicleAttitude
                    version = getattr(cls, "MESSAGE_VERSION", 0)
                    self.watch(name, cls, f"{prefix}/fmu/out/{base}" + (f"_v{version}" if version else ""))

        def watch(self, name, cls, topic):
            self.streams[name] = (folder / (name + ".jsonl")).open("x", buffering=1)
            self.create_subscription(cls, topic, lambda msg, key=name: self.record(key, msg), qos)

        def clock(self, msg):
            self.sim = msg.clock.sec + msg.clock.nanosec / 1e9

        def record(self, name, msg):
            header = getattr(msg, "header", None)
            if header is not None:
                stamp = header.stamp.sec * 10**9 + header.stamp.nanosec
                payload = bytes(msg.data) if hasattr(msg, "data") else bytes(json.dumps([list(msg.k), list(msg.p)]), "utf-8")
            else:
                stamp = int(msg.timestamp) * 1000
                payload = repr((msg.timestamp,) + tuple(getattr(msg, f) for f in ("x", "y", "z", "q")
                                                         if hasattr(msg, f))).encode()
            entry = {"stream": name, "stamp_ns": stamp, "receipt_sim_s": self.sim, "receipt_wall_s": time.monotonic(),
                     "crc32": zlib.crc32(payload), "bytes": len(payload)}
            self.streams[name].write(json.dumps(entry, allow_nan=False, separators=(",", ":")) + "\n")

    node = Witness()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for stream in node.streams.values():
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
