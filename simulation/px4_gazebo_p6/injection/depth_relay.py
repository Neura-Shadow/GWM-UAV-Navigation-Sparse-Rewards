"""P6 depth relay (DR, decision D1a). Safe to import without ROS.

The P6 runner moves the bridge output to the raw topics through two declared keys of the run-local
`p3_sensor_bridge.yaml` (rule C11). This node republishes image and camera info on the accepted names
the unchanged adapter and perception subscribe to, with the bridge's own QoS. In pass-through, and for
every frame the fault leaves alone, it republishes the received message object itself.

Activation comes from the runner (SIGUSR1) at the declared trigger; the injector's windows are then
relative to that simulation time (`/clock`).
"""
import argparse
import signal
import threading
import time

from . import declaration as declaration_module
from .depth_faults import DepthInjector
from .fault_ledger import FaultLedger
from .qos import DEPTH_QUEUE, clock_profile, relay_profile

RAW = "/gwm/p6/raw/front_depth"
OUT = "/gwm/sensors/front_depth"
QOS_WAIT_WALL_S = 30.0


def frame_fields(msg):
    return {"width": msg.width, "height": msg.height, "encoding": msg.encoding, "is_bigendian": msg.is_bigendian,
            "step": msg.step, "data": bytes(msg.data), "stamp_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--declaration", required=True)
    parser.add_argument("--ledger", required=True)
    args = parser.parse_args(argv)
    declared = declaration_module.load(args.declaration)
    mode = declaration_module.uses(declared, "DR")
    if mode == "off":
        raise SystemExit("the declaration does not use the depth relay")
    injector = DepthInjector(declared["fault"], declared["parameters"], mode)
    ledger = FaultLedger(args.ledger, declared["row"], declared["fault"], "depth_relay")
    ledger.record("declared", None, declaration=declared)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image

    requested = threading.Event()
    stop = threading.Event()
    signal.signal(signal.SIGUSR1, lambda *_: requested.set())
    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, lambda *_: stop.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    class Relay(Node):
        def __init__(self):
            super().__init__("p6_depth_relay", namespace="/gwm", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.sim = None
            self.counts = {"images_in": 0, "images_out": 0, "infos_in": 0, "infos_out": 0, "rewritten": 0}
            qos = self.bridge_qos(RAW + "/image_raw")
            self.image_pub = self.create_publisher(Image, OUT + "/image_raw", qos)
            self.info_pub = self.create_publisher(CameraInfo, OUT + "/camera_info", qos)
            self.create_subscription(Image, RAW + "/image_raw", self.image, qos)
            self.create_subscription(CameraInfo, RAW + "/camera_info", self.info, qos)
            self.create_subscription(Clock, "/clock", self.clock, clock_profile())
            self.create_timer(0.01, self.release)
            ledger.record("armed", None, mode=mode, qos=str(qos))

        def bridge_qos(self, topic):
            """The bridge's reliability and durability, so delivery to the unchanged consumers stays as accepted."""
            deadline = time.monotonic() + QOS_WAIT_WALL_S
            while time.monotonic() < deadline:
                infos = self.get_publishers_info_by_topic(topic)
                if infos:
                    return relay_profile(infos[0].qos_profile, DEPTH_QUEUE)
                rclpy.spin_once(self, timeout_sec=0.1)
            raise RuntimeError("no bridge publisher on " + topic)

        def clock(self, msg):
            self.sim = msg.clock.sec + msg.clock.nanosec / 1e9
            if requested.is_set() and injector.trigger_sim is None:
                injector.activate(self.sim)
                ledger.record("activated", self.sim)

        def image(self, msg):
            self.counts["images_in"] += 1
            fields = frame_fields(msg) if mode == "active" else None
            outputs = injector.image(fields if fields is not None else msg, self.sim if self.sim is not None else 0.0)
            for item in outputs:
                self.publish_image(msg, fields, item)
            self.drain()

        def publish_image(self, msg, fields, item):
            if item is msg or item is fields:
                self.image_pub.publish(msg)  # untouched: the received object itself
            else:
                out = Image()
                out.header.stamp.sec, out.header.stamp.nanosec = divmod(item["stamp_ns"], 10**9)
                out.header.frame_id = item["frame_id"]
                out.height, out.width, out.encoding = item["height"], item["width"], item["encoding"]
                out.is_bigendian, out.step, out.data = item["is_bigendian"], item["step"], item["data"]
                self.image_pub.publish(out)
                self.counts["rewritten"] += 1
            self.counts["images_out"] += 1

        def info(self, msg):
            self.counts["infos_in"] += 1
            info = {"k": list(msg.k), "p": list(msg.p)} if mode == "active" else None
            changed = injector.camera_info(info, self.sim or 0.0) if info is not None else None
            if changed is None or changed is info:
                self.info_pub.publish(msg)
            else:
                msg.k, msg.p = changed["k"], changed["p"]
                self.info_pub.publish(msg)
                self.counts["rewritten"] += 1
            self.counts["infos_out"] += 1
            self.drain()

        def release(self):
            for item in injector.due(self.sim if self.sim is not None else 0.0):
                self.publish_image(None, None, item)

        def drain(self):
            while injector.affected:
                index, what = injector.affected.pop(0)
                ledger.record("affected", self.sim, item="image", index=index, what=what)

    node = Relay()
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        ledger.record("finished", node.sim, counts=node.counts)
        ledger.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
