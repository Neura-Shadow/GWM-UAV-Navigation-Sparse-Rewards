"""Evaluator-only Gazebo ground-truth ledger for P4 (contract §2, §6). Never read by the vehicle.

Records the vehicle pose at least every 0.05 s of simulation time
(`p4/truth_poses.jsonl`) and every obstacle contact message (`p4/contacts.jsonl`),
then a close record. It subscribes only to Gazebo transport topics; it publishes nothing.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import threading
import time

GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
MIN_POSE_INTERVAL_NS = 20_000_000  # keep the vehicle pose at <= 50 Hz, well inside the 0.05 s limit


class Ledger:
    def __init__(self, path):
        self.stream = path.open("x", buffering=1)
        self.count = 0
        self.lock = threading.Lock()

    def write(self, row):
        with self.lock:
            self.stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
            self.count += 1

    def close(self):
        self.stream.flush()
        os.fsync(self.stream.fileno())
        self.stream.close()


def pose_row(stamp_ns, poses, model):
    found = {p.name: {"position": [p.position.x, p.position.y, p.position.z],
                      "q_wxyz": [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]}
             for p in poses if p.name == model}
    return {"stamp_ns": stamp_ns, "receipt_monotonic_ns": time.monotonic_ns(), "poses": found} if found else None


def contact_row(stamp_ns, obstacle, contacts):
    pairs = [[c.collision1.name, c.collision2.name, len(c.position)] for c in contacts]
    return {"stamp_ns": stamp_ns, "obstacle": obstacle, "receipt_monotonic_ns": time.monotonic_ns(), "contacts": pairs}


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("world")
    parser.add_argument("model")
    parser.add_argument("--obstacle", action="append", default=[])
    args = parser.parse_args(argv)
    missing = [k for k in GATES if os.environ.get(k) != "1"]
    if missing or os.environ.get("GWM_P4_OWNED_NAMESPACE") != "1" or os.getpid() == 1:
        raise ValueError("Use the owned P4 namespace launcher with explicit gates")
    if os.environ.get("GZ_PARTITION") != args.run.name:
        raise ValueError("truth_observer_partition_identity")
    for value in (args.world, args.model, *args.obstacle):
        if not re.fullmatch(r"[A-Za-z0-9_]+", value):
            raise ValueError("truth_observer_identity")
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    folder = args.run / "p4"
    folder.mkdir(exist_ok=True)
    poses, contacts = Ledger(folder / "truth_poses.jsonl"), Ledger(folder / "contacts.jsonl")
    stopping = threading.Event()
    state = {"error": None, "last_pose_ns": None, "max_pose_gap_ns": 0}

    def fail(exc):
        state["error"] = state["error"] or type(exc).__name__ + ":" + str(exc)
        stopping.set()

    def on_poses(msg):
        try:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
            last = state["last_pose_ns"]
            if last is not None and stamp - last < MIN_POSE_INTERVAL_NS:
                return
            row = pose_row(stamp, msg.pose, args.model)
            if row is not None:
                if last is not None:
                    state["max_pose_gap_ns"] = max(state["max_pose_gap_ns"], stamp - last)
                state["last_pose_ns"] = stamp
                poses.write(row)
        except Exception as exc:
            fail(exc)

    def contact_receiver(name):
        def receive(msg):
            try:
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
                contacts.write(contact_row(stamp, name, msg.contact))
            except Exception as exc:
                fail(exc)
        return receive

    node = Node()
    subscribed = {"poses": bool(node.subscribe(Pose_V, f"/world/{args.world}/dynamic_pose/info", on_poses))}
    for name in args.obstacle:
        subscribed["contact:" + name] = bool(node.subscribe(Contacts, "/gwm/p4/contacts/" + name, contact_receiver(name)))
    if not all(subscribed.values()):
        fail(ValueError("subscription_failed:" + ",".join(k for k, v in subscribed.items() if not v)))
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    started = time.monotonic()
    while not stopping.wait(0.1):
        pass
    for ledger in (poses, contacts):
        ledger.close()
    closed = {"contract": "p4-truth-observer-v1", "subscribed": subscribed, "error": state["error"],
              "pose_rows": poses.count, "contact_rows": contacts.count,
              "max_pose_gap_s": state["max_pose_gap_ns"] / 1e9, "wall_duration_s": time.monotonic() - started,
              "model": args.model, "obstacles": args.obstacle}
    (folder / "truth-closed.json").write_text(json.dumps(closed, indent=2, allow_nan=False) + "\n")
    return 0 if state["error"] is None else 3


if __name__ == "__main__":
    raise SystemExit(main())
