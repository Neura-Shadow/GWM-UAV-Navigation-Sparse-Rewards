"""F4.07 competing PX4-input publisher: advertises, never publishes. Safe to import without ROS.

It creates one `TrajectorySetpoint` publisher on the declared PX4 input topic for the declared time,
so the accepted controller's graph check sees a second owner. By construction this module contains
no publish call (a test checks that); the evaluator also proves zero foreign setpoints from the ULog.
"""
import argparse
import signal
import threading
import time

from . import declaration as declaration_module
from .fault_ledger import FaultLedger


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--declaration", required=True)
    parser.add_argument("--ledger", required=True)
    args = parser.parse_args(argv)
    declared = declaration_module.load(args.declaration)
    if declared["fault"] != "F4.07" or declared["mode"] != "active":
        raise SystemExit("the competing publisher runs only for an active F4.07 declaration")
    p = declared["parameters"]
    if p.get("messages_published") != 0:
        raise SystemExit("the F4.07 declaration must state messages_published: 0")
    ledger = FaultLedger(args.ledger, declared["row"], declared["fault"], "competing_publisher")
    ledger.record("declared", None, declaration=declared)

    import px4_msgs.msg as px4
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions

    stop = threading.Event()
    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, lambda *_: stop.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node("p6_competing_publisher", namespace="/gwm")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    advertised = node.create_publisher(px4.TrajectorySetpoint, p["topic"], qos)
    ledger.record("activated", None, topic=p["topic"], advertised_wall_s=time.monotonic())
    end = time.monotonic() + p["advertise_s"]
    try:
        while not stop.is_set() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_publisher(advertised)
        ledger.record("cleared", None, withdrawn_wall_s=time.monotonic())
        ledger.record("finished", None, messages_published=0)
        ledger.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
