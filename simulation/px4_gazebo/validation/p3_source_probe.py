"""Evaluator-only native Gazebo ledger: depth/RGB headers, CameraInfo, poses, clock.

This process is a passive Gazebo-transport subscriber. It never publishes and
never feeds the ROS adapter. Image payloads are skipped with a bounded wire
parser (pixels are never retained); the first message of each stream is also
decoded with the installed protobuf class and must agree exactly.

Ground-truth poses recorded here are for the offline evaluator only.
Run: p3_source_probe.py RUN_DIR WORLD MODEL [--rgb | --rgb-absence-monitor | --clock-only] [--watch NAME ...]
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import threading
import time

MAX_IMAGE = 8 * 1024 * 1024
MAX_HEADER = 8192
CAPACITY = 200000


def varint(raw, pos):
    value = 0
    for index in range(10):
        if pos >= len(raw):
            raise ValueError("truncated_varint")
        byte = raw[pos]
        pos += 1
        if index == 9 and byte > 1:
            raise ValueError("varint_overflow")
        value |= (byte & 127) << (index * 7)
        if not byte & 128:
            return value, pos
    raise ValueError("varint_unbounded")


def fields(raw, grammar, maximum):
    """Known gz-msgs10 wire grammar {number: (wire_type, repeated)}; views skip pixels."""
    if len(raw) > maximum:
        raise ValueError("message_bound")
    result, pos = {}, 0
    while pos < len(raw):
        key, pos = varint(raw, pos)
        number, kind = key >> 3, key & 7
        if number not in grammar or kind != grammar[number][0]:
            raise ValueError("unexpected_wire_field:" + str(number))
        if number in result and not grammar[number][1]:
            raise ValueError("duplicate_field:" + str(number))
        if kind == 0:
            value, pos = varint(raw, pos)
        else:
            size, pos = varint(raw, pos)
            if size > len(raw) - pos:
                raise ValueError("truncated_field")
            value, pos = raw[pos:pos + size], pos + size
        result.setdefault(number, []).append(value)
        if sum(map(len, result.values())) > 64:
            raise ValueError("field_count_bound")
    return result


def decode_header(raw):
    part = fields(raw, {1: (2, False), 2: (2, True)}, MAX_HEADER)
    if 1 not in part:
        raise ValueError("missing_timestamp")
    stamp = fields(part[1][0], {1: (0, False), 2: (0, False)}, 32)
    sec, nsec = stamp.get(1, [0])[0], stamp.get(2, [0])[0]
    if not 0 <= nsec < 10**9 or sec * 10**9 + nsec <= 0:
        raise ValueError("malformed_timestamp")
    data = {}
    for item in part.get(2, []):
        pair = fields(item, {1: (2, False), 2: (2, True)}, 1024)
        if set(pair) != {1, 2} or len(pair[2]) != 1:
            raise ValueError("ambiguous_header_entry")
        key, value = bytes(pair[1][0]).decode(), bytes(pair[2][0]).decode()
        if key in data:
            raise ValueError("duplicate_header_key")
        data[key] = value
    if not re.fullmatch(r"0|[1-9][0-9]{0,19}", data.get("seq", "")) or "frame_id" not in data:
        raise ValueError("missing_or_malformed_sequence")
    return sec * 10**9 + nsec, data


def decode_image(raw):
    """gz.msgs.Image: 1 header, 2 width, 3 height, 4 step, 5 data, 6 pixel_format_type."""
    part = fields(memoryview(raw), {1: (2, False), 2: (0, False), 3: (0, False), 4: (0, False),
                                    5: (2, False), 6: (0, False)}, MAX_IMAGE)
    if not {1, 2, 3, 4, 5, 6} <= set(part):
        raise ValueError("missing_image_field")
    stamp, header = decode_header(part[1][0])
    width, height, step, pixel = (part[k][0] for k in (2, 3, 4, 6))
    if len(part[5][0]) != step * height:
        raise ValueError("payload_length")
    return {"stamp_ns": stamp, "seq": int(header["seq"]), "frame_id": header["frame_id"], "width": width,
            "height": height, "step": step, "pixel_format": pixel, "payload_bytes": len(part[5][0]),
            "serialized_bytes": len(raw), "header_hex": bytes(part[1][0]).hex()}


class Stream:
    def __init__(self, path):
        self.lock = threading.Lock()
        self.file = Path(path).open("x", buffering=1)
        self.count = 0
        self.first_checked = False

    def write(self, row):
        with self.lock:
            if self.count >= CAPACITY:
                raise ValueError("ledger_capacity:" + self.file.name)
            self.count += 1
            row["id"] = self.count
            self.file.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")

    def close(self):
        with self.lock:
            self.file.flush()
            os.fsync(self.file.fileno())
            self.file.close()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("world")
    parser.add_argument("model")
    parser.add_argument("--rgb", action="store_true")
    parser.add_argument("--watch", nargs="*", default=[])
    # Diagnostic scope only: keep native sim time, drop every image/pose/camera_info subscription.
    parser.add_argument("--clock-only", action="store_true")
    # Contract v1.3 flights: no RGB subscription; sample IMX214 subscriber presence every period.
    parser.add_argument("--rgb-absence-monitor", action="store_true")
    args = parser.parse_args(argv)
    if args.clock_only and (args.rgb or args.watch):
        raise ValueError("source_probe_clock_only_excludes_other_subscriptions")
    if args.rgb_absence_monitor and (args.rgb or args.clock_only):
        raise ValueError("source_probe_rgb_absence_monitor_excludes_rgb")
    if os.environ.get("GZ_PARTITION") != args.run.name:
        raise ValueError("source_probe_partition_identity")
    for value in (args.world, args.model, *args.watch):
        if not re.fullmatch(r"[A-Za-z0-9_]+", value):
            raise ValueError("source_probe_identity")
    from gz.transport13 import Node, SubscribeOptions
    from gz.msgs10.camera_info_pb2 import CameraInfo
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_v_pb2 import Pose_V

    folder = args.run / "source"
    folder.mkdir()
    stopping = threading.Event()
    state = {"error": None, "camera_info": None, "camera_info_count": 0, "camera_info_changed": 0,
             "last_pose_sim": None, "last_clock_wall": None}
    streams = {"depth": Stream(folder / "depth-headers.jsonl"), "rgb": Stream(folder / "rgb-headers.jsonl"),
               "pose": Stream(folder / "poses.jsonl"), "clock": Stream(folder / "clock.jsonl")}
    if args.rgb_absence_monitor:
        streams["imx214"] = Stream(folder / "imx214-subscribers.jsonl")
    costs = {"depth": 0, "rgb": 0}
    watched = {args.model, *args.watch}

    def fail(exc):
        state["error"] = state["error"] or (type(exc).__name__ + ":" + str(exc))
        stopping.set()

    def image_receiver(kind, expected_dims, expected_frame):
        stream = streams[kind]

        def receive(raw, _info):
            begin = time.monotonic_ns()
            try:
                row = decode_image(raw)
                if (row["width"], row["height"]) != expected_dims or row["frame_id"] != expected_frame:
                    raise ValueError(kind + "_identity")
                if not stream.first_checked:
                    typed = Image()
                    typed.ParseFromString(bytes(raw))
                    seq = next(int(d.value[0]) for d in typed.header.data if d.key == "seq")
                    if (typed.width, typed.height, typed.step, typed.header.stamp.sec * 10**9 + typed.header.stamp.nsec,
                            seq) != (row["width"], row["height"], row["step"], row["stamp_ns"], row["seq"]):
                        raise ValueError(kind + "_wire_parser_disagrees_with_protobuf")
                    stream.first_checked = True
                row.pop("header_hex") if kind == "depth" else None
                row["receipt_monotonic_ns"] = begin
                stream.write(row)
            except Exception as exc:
                fail(exc)
            finally:
                costs[kind] = max(costs[kind], time.monotonic_ns() - begin)
        return receive

    def camera_info(msg):
        try:
            static = {"width": msg.width, "height": msg.height, "k": list(msg.intrinsics.k),
                      "p": list(msg.projection.p), "r": list(msg.rectification_matrix),
                      "distortion_k": list(msg.distortion.k), "frame_id": next(
                          (d.value[0] for d in msg.header.data if d.key == "frame_id"), None)}
            state["camera_info_count"] += 1
            if state["camera_info"] is None:
                state["camera_info"] = static
                (folder / "camera-info.json").write_text(json.dumps(static, indent=2, allow_nan=False) + "\n")
            elif static != state["camera_info"]:
                state["camera_info_changed"] += 1
        except Exception as exc:
            fail(exc)

    def poses(msg):
        try:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
            if state["last_pose_sim"] is not None and stamp - state["last_pose_sim"] < 200_000_000:
                return
            state["last_pose_sim"] = stamp
            found = {p.name: {"position": [p.position.x, p.position.y, p.position.z],
                              "q_wxyz": [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]}
                     for p in msg.pose if p.name in watched}
            streams["pose"].write({"stamp_ns": stamp, "receipt_monotonic_ns": time.monotonic_ns(), "poses": found})
        except Exception as exc:
            fail(exc)

    def clock(msg):
        now = time.monotonic_ns()
        state["last_sim"] = msg.sim.sec + msg.sim.nsec / 1e9
        if state["last_clock_wall"] is not None and now - state["last_clock_wall"] < 100_000_000:
            return
        state["last_clock_wall"] = now
        try:
            streams["clock"].write({"sim_s": msg.sim.sec + msg.sim.nsec / 1e9, "receipt_monotonic_ns": now})
        except Exception as exc:
            fail(exc)

    node = Node()
    options = SubscribeOptions()
    rgb_topic = f"/world/{args.world}/model/{args.model}/link/camera_link/sensor/IMX214/image"
    if args.clock_only:
        subscribed = {"clock": bool(node.subscribe(Clock, f"/world/{args.world}/clock", clock))}
    else:
        subscribed = {
            "depth": bool(node.subscribe_raw("/depth_camera", image_receiver("depth", (640, 480), "camera_link"),
                                             "gz.msgs.Image", options)),
            "camera_info": bool(node.subscribe(CameraInfo, "/camera_info", camera_info)),
            "pose": bool(node.subscribe(Pose_V, f"/world/{args.world}/pose/info", poses)),
            "clock": bool(node.subscribe(Clock, f"/world/{args.world}/clock", clock))}
    if args.rgb:
        subscribed["rgb"] = bool(node.subscribe_raw(rgb_topic, image_receiver("rgb", (1920, 1080), "camera_link"),
                                                    "gz.msgs.Image", options))
    if not all(subscribed.values()):
        fail(ValueError("subscription_failed:" + ",".join(k for k, v in subscribed.items() if not v)))
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    started = time.monotonic()
    monitor = None
    if args.rgb_absence_monitor:
        settings = json.loads((args.run / "p3_validation.yaml").read_text())["rgb_absence_monitor"]

        def sample_subscribers():
            # Local discovery-cache lookups only; every failure is recorded in the row, never hidden.
            while not stopping.wait(settings["sample_period_wall_s"]):
                row = {"monotonic_ns": time.monotonic_ns(), "sim_s": state.get("last_sim"), "topics": {},
                       "control_subscribers": None, "error": None}
                try:
                    for topic in node.topic_list():
                        if settings["topic_marker"] in topic:
                            row["topics"][topic] = len(node.topic_info(topic)[1])
                    row["control_subscribers"] = len(node.topic_info(settings["control_topic"])[1])
                except Exception as exc:
                    row["error"] = type(exc).__name__ + ":" + str(exc)
                try:
                    streams["imx214"].write(row)
                except Exception as exc:
                    fail(exc)
        monitor = threading.Thread(target=sample_subscribers, name="imx214-subscribers", daemon=True)
        monitor.start()
    stopping.wait()
    if monitor is not None:
        monitor.join(timeout=5)
    own_subscriptions = sorted(node.subscribed_topics())
    topics = [f"/world/{args.world}/clock"] if args.clock_only else [
        "/depth_camera", "/camera_info", f"/world/{args.world}/pose/info", f"/world/{args.world}/clock"]
    for topic in topics + ([rgb_topic] if args.rgb else []):
        node.unsubscribe(topic)
    time.sleep(0.2)  # let in-flight transport callbacks return before closing ledgers
    for stream in streams.values():
        stream.close()
    closed = {"contract": "p3-native-source-probe-v1", "run_id": args.run.name, "world": args.world,
              "model": args.model, "subscribed": subscribed, "rgb_subscribed": subscribed.get("rgb", False),
              "rgb_topic": rgb_topic if args.rgb else None, "counts": {k: s.count for k, s in streams.items()},
              "camera_info_count": state["camera_info_count"], "camera_info_changed": state["camera_info_changed"],
              "max_callback_ns": costs, "error": state["error"], "wall_s": time.monotonic() - started,
              "pixels_retained": False, "rgb_absence_monitor": args.rgb_absence_monitor,
              "monitor_rows": streams["imx214"].count if "imx214" in streams else 0,
              "subscribed_topics": own_subscriptions}
    with (folder / "probe-closed.json").open("x") as stream:
        stream.write(json.dumps(closed, indent=2, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    return 0 if state["error"] is None and state["camera_info_changed"] == 0 else 4


if __name__ == "__main__":
    raise SystemExit(main())
