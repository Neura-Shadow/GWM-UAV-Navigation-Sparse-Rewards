"""P6 state relay to perception (SR, decision D1a). Safe to import without ROS.

`p4_perception` is started with its existing `--px4-prefix /gwm/p6/px4_71` argument; this node
republishes the two state topics it reads (`vehicle_local_position`, `vehicle_attitude`) from the real
`/px4_71` prefix, with PX4's own QoS. The adapter and the controller keep `/px4_71`, so only
perception's pose association sees the fault (F2.09). Activation by SIGUSR1 from the runner.
"""
import argparse
import signal
import threading
import time

from . import declaration as declaration_module
from .fault_ledger import FaultLedger
from .state_faults import StateInjector

SOURCE, TARGET = "/px4_71", "/gwm/p6/px4_71"
TOPICS = (("vehicle_local_position", "VehicleLocalPosition"), ("vehicle_attitude", "VehicleAttitude"))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--declaration", required=True)
    parser.add_argument("--ledger", required=True)
    args = parser.parse_args(argv)
    declared = declaration_module.load(args.declaration)
    mode = declaration_module.uses(declared, "SR")
    if mode == "off":
        raise SystemExit("the declaration does not use the state relay")
    injector = StateInjector(declared["fault"], declared["parameters"], mode)
    ledger = FaultLedger(args.ledger, declared["row"], declared["fault"], "state_relay")
    ledger.record("declared", None, declaration=declared)

    import px4_msgs.msg as px4
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock

    requested, stop = threading.Event(), threading.Event()
    signal.signal(signal.SIGUSR1, lambda *_: requested.set())
    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, lambda *_: stop.set())
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)

    class Relay(Node):
        def __init__(self):
            super().__init__("p6_state_relay", namespace="/gwm", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.sim = None
            self.counts = {}
            self.pubs = {}
            for base, class_name in TOPICS:
                cls = getattr(px4, class_name)
                version = getattr(cls, "MESSAGE_VERSION", 0)
                suffix = f"/fmu/out/{base}" + (f"_v{version}" if version else "")
                qos = self.source_qos(SOURCE + suffix)
                self.pubs[base] = self.create_publisher(cls, TARGET + suffix, qos)
                self.create_subscription(cls, SOURCE + suffix, lambda msg, key=base: self.state(key, msg), qos)
            self.create_subscription(Clock, "/clock", self.clock, 10)
            ledger.record("armed", None, mode=mode)

        def source_qos(self, topic):
            deadline = time.monotonic() + 30.0
            while time.monotonic() < deadline:
                infos = self.get_publishers_info_by_topic(topic)
                if infos:
                    return infos[0].qos_profile
                rclpy.spin_once(self, timeout_sec=0.1)
            raise RuntimeError("no PX4 publisher on " + topic)

        def clock(self, msg):
            self.sim = msg.clock.sec + msg.clock.nanosec / 1e9
            if requested.is_set() and injector.trigger_sim is None:
                injector.activate(self.sim)
                ledger.record("activated", self.sim)

        def state(self, key, msg):
            self.counts[key] = self.counts.get(key, 0) + 1
            for item in injector.state(key, msg, self.sim if self.sim is not None else 0.0):
                if item is not msg:  # the relay only withholds; a rewritten state is never produced here
                    raise RuntimeError("state relay produced a modified message")
                self.pubs[key].publish(msg)
            while injector.affected:
                topic, sim, what = injector.affected.pop(0)
                ledger.record("affected", sim, item=topic, what=what)

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
