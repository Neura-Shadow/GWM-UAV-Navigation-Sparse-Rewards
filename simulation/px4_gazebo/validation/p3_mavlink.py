"""Passive native MAVLink capture and explicit GCS command-policy evaluation.

Zero ROS flight-control publications do not prove zero native commands, so the
owned loopback link is captured read-only (AF_PACKET + per-socket BPF on the
PX4 instance-71 GCS ports). Every frame is CRC-checked against the pinned
MAVLink definitions. No socket is opened at import; nothing is forwarded or
filtered on the data path.

Policies are data (configs/p3_native_command_policy.json). The default is
strict zero. Any other policy applies only with an operator approval receipt
bound to the exact policy hash; nothing is silently whitelisted.
"""
import hashlib
import json
from pathlib import Path
import socket
import struct
from threading import Event, Thread
import time

from p3_provenance import atomic_json, canonical_sha256, strict_json

CRC = json.loads(Path(__file__).with_name("p3_mavlink_crc.json").read_text())["messages"]
PORTS = (14550, 18641)
MAX_BYTES = 256 * 1024 * 1024
MAX_PACKETS = 262144
COMMAND_MESSAGES = {75: "COMMAND_INT", 76: "COMMAND_LONG", 80: "COMMAND_CANCEL"}
# Messages that set modes, missions, parameters, setpoints or actuators.
ACTIVE_CONTROL = {11, 23, 39, 41, 44, 45, 69, 70, 73, 82, 84, 86, 139, 323}
VEHICLE = (72, 1)


def crc_x25(data):
    value = 0xFFFF
    for byte in data:
        tmp = byte ^ (value & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        value = ((value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return value


def decode_datagram(raw):
    """Decode every MAVLink v1/v2 frame in one UDP payload; any defect raises."""
    frames, position = [], 0
    while position < len(raw):
        start, magic = position, raw[position]
        if magic not in (0xFE, 0xFD):
            raise ValueError("MAVLink_unframed_bytes")
        header = 6 if magic == 0xFE else 10
        if len(raw) - position < header:
            raise ValueError("MAVLink_header_truncated")
        size = raw[position + 1]
        if magic == 0xFD and raw[position + 2] & ~1:
            raise ValueError("MAVLink_unknown_incompatibility_flags")
        signed = magic == 0xFD and bool(raw[position + 2] & 1)
        end = position + header + size + 2 + (13 if signed else 0)
        if end > len(raw):
            raise ValueError("MAVLink_frame_truncated")
        msg = raw[position + 5] if magic == 0xFE else int.from_bytes(raw[position + 7:position + 10], "little")
        spec = CRC.get(str(msg))
        if spec is None:
            raise ValueError("MAVLink_unpinned_message:" + str(msg))
        extra, minimum, maximum = spec
        if size > maximum or (magic == 0xFE and size != minimum):
            raise ValueError("MAVLink_payload_length")
        body = raw[position + header:position + header + size]
        if crc_x25(raw[position + 1:position + header + size] + bytes([extra])) != struct.unpack_from(
                "<H", raw, position + header + size)[0]:
            raise ValueError("MAVLink_CRC_mismatch")
        source = raw[position + 3:position + 5] if magic == 0xFE else raw[position + 5:position + 7]
        row = {"message_id": msg, "source_system": source[0], "source_component": source[1],
               "raw_hex": bytes(raw[start:end]).hex(), "signed": signed, "mavlink2": magic == 0xFD}
        if msg in COMMAND_MESSAGES:
            # MAVLink 2 zero-tail truncation is protocol decoding, not repair.
            payload = bytes(body).ljust(maximum, b"\0")
            offset = 0 if msg == 80 else 28
            row.update(command=struct.unpack_from("<H", payload, offset)[0], target_system=payload[offset + 2],
                       target_component=payload[offset + 3], message=COMMAND_MESSAGES[msg])
            if msg == 76:
                row["params_float32_le_hex"] = payload[:28].hex()
        if msg in ACTIVE_CONTROL:
            row["active_control_message"] = True
        frames.append(row)
        position = end
    return frames


def udp_payload(packet):
    if len(packet) < 42 or packet[12:14] != b"\x08\x00":
        raise ValueError("MAVLink_IP_packet_truncated")
    ihl = (packet[14] & 15) * 4
    total = struct.unpack_from("!H", packet, 16)[0]
    if ihl < 20 or packet[23] != 17 or len(packet) < 14 + total:
        raise ValueError("MAVLink_invalid_IP_packet")
    if struct.unpack_from("!H", packet, 20)[0] & 0x3FFF:
        raise ValueError("MAVLink_IP_fragment_unobservable")
    offset = 14 + ihl
    source, destination, length = struct.unpack_from("!HHH", packet, offset)
    if length < 8 or offset + length != 14 + total:
        raise ValueError("MAVLink_UDP_length")
    return source, destination, packet[offset + 8:offset + length]


def attach_link_filter(sock):
    """Per-observer classic BPF (IPv4/UDP, either port in PORTS); not a firewall."""
    import ctypes
    code = [(0x28, 0, 0, 12), (0x15, 0, 10, 0x800), (0x30, 0, 0, 23), (0x15, 0, 8, 17),
            (0xB1, 0, 0, 14), (0x48, 0, 0, 14), (0x15, 4, 0, PORTS[0]), (0x15, 3, 0, PORTS[1]),
            (0x48, 0, 0, 16), (0x15, 1, 0, PORTS[0]), (0x15, 0, 1, PORTS[1]), (0x06, 0, 0, 0xFFFFFFFF),
            (0x06, 0, 0, 0)]

    class Filter(ctypes.Structure):
        _fields_ = [("code", ctypes.c_ushort), ("jt", ctypes.c_ubyte), ("jf", ctypes.c_ubyte), ("k", ctypes.c_uint)]

    class Program(ctypes.Structure):
        _fields_ = [("length", ctypes.c_ushort), ("filters", ctypes.POINTER(Filter))]

    filters = (Filter * len(code))(*(Filter(*row) for row in code))
    program = Program(len(code), filters)
    sock.setsockopt(socket.SOL_SOCKET, 26, bytes(program))  # SO_ATTACH_FILTER


class LinkCapture:
    """Bounded read-only capture thread; raw packets and decoded index retained."""

    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir()
        self.stop_event = Event()
        self.error = None
        self.packets = self.bytes = 0
        self.frames = []
        self.started_ns = time.monotonic_ns()
        self.closed = None
        self.socket = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        try:
            attach_link_filter(self.socket)
            self.socket.bind(("lo", 0))
            self.socket.settimeout(0.05)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        except BaseException:
            self.socket.close()
            raise
        self.thread = Thread(target=self._read, name="p3-mavlink-capture", daemon=True)
        self.thread.start()

    def _read(self):
        try:
            with (self.folder / "packets.bin").open("xb") as raw, (self.folder / "packets.jsonl").open("x") as index:
                while not self.stop_event.is_set():
                    try:
                        packet, address = self.socket.recvfrom(65550)
                    except socket.timeout:
                        continue
                    if address[2] != 0:  # one PACKET_HOST copy per loopback datagram
                        continue
                    observed = time.monotonic_ns()
                    source, destination, payload = udp_payload(packet)
                    if self.bytes + len(packet) > MAX_BYTES or self.packets >= MAX_PACKETS:
                        raise ValueError("MAVLink_capture_capacity")
                    row = {"ordinal": self.packets, "monotonic_ns": observed, "source_port": source,
                           "destination_port": destination, "offset": raw.tell(), "bytes": len(packet),
                           "sha256": hashlib.sha256(packet).hexdigest()}
                    raw.write(packet)
                    self.bytes += len(packet)
                    self.packets += 1
                    try:
                        frames = decode_datagram(payload)
                    except ValueError as exc:
                        row["decode_error"] = str(exc)
                        self.error = str(exc)
                        frames = []
                    for frame in frames:
                        if "command" in frame or frame.get("active_control_message") or frame["message_id"] == 0:
                            self.frames.append(dict(frame, packet_ordinal=row["ordinal"], monotonic_ns=observed,
                                                    destination_port=destination))
                    row["message_ids"] = [frame["message_id"] for frame in frames]
                    index.write(json.dumps(row, allow_nan=False) + "\n")
                    if self.error:
                        break
                raw.flush()
                index.flush()
                import os
                os.fsync(raw.fileno())
                os.fsync(index.fileno())
        except BaseException as exc:
            self.error = self.error or (type(exc).__name__ + ":" + str(exc))

    def close(self, timeout=5.0):
        if self.closed is not None:
            return self.closed
        self.stop_event.set()
        self.thread.join(timeout=timeout)
        alive = self.thread.is_alive()
        packets, drops = struct.unpack("II", self.socket.getsockopt(263, 6, 8))  # PACKET_STATISTICS
        self.socket.close()
        self.closed = {"contract": "p3-native-mavlink-capture-v1", "started_monotonic_ns": self.started_ns,
                       "closed_monotonic_ns": time.monotonic_ns(), "packets": self.packets, "bytes": self.bytes,
                       "kernel_packets": packets, "kernel_drops": drops, "error": self.error,
                       "thread_stopped": not alive, "frames_of_interest": self.frames,
                       "complete": not alive and self.error is None and drops == 0 and self.packets > 0}
        atomic_json(self.folder / "closed.json", self.closed, exclusive=True)
        return self.closed


def load_policy(path, selected, approval_path=None):
    """Return the effective policy. Non-default policies require a bound approval."""
    document = strict_json(path)
    if document.get("contract") != "p3-native-command-policy-v1":
        raise ValueError("Unsupported native command policy document")
    name = selected or document["default_policy"]
    policy = document["policies"].get(name)
    if policy is None:
        raise ValueError("Unknown native command policy: " + str(name))
    identity = canonical_sha256(policy)
    approval = None
    if policy.get("requires_operator_approval"):
        if approval_path is None:
            raise PermissionError("Policy " + name + " requires an operator approval receipt")
        approval = strict_json(approval_path)
        if (approval.get("contract") != "p3-native-command-policy-approval-v1" or approval.get("policy") != name
                or approval.get("policy_sha256") != identity or approval.get("approved") is not True
                or not approval.get("approved_by")):
            raise PermissionError("Approval receipt does not bind this exact policy")
    return {"name": name, "policy": policy, "policy_sha256": identity, "approval": approval,
            "approval_path": str(approval_path) if approval is not None else None}


def landing_self_command(frame, rule, landing_window_ns, counts):
    """Contract v1.4 rule 2: None if this vehicle self-command is an enumerated PX4 landing
    gimbal-neutral frame inside the controller's REQUEST_LAND..COMPLETE window, else the violation reason."""
    if rule is None or landing_window_ns is None:
        return "vehicle_self_command_on_gcs_link"
    match = next((p for p in rule["payload_patterns"] if p["command"] == frame.get("command")
                  and p["seven_float32_le_hex"] == frame.get("params_float32_le_hex")), None)
    begin, end = landing_window_ns
    if (match is None or frame["message_id"] != rule["message_id"]
            or (frame["source_system"], frame["source_component"]) != tuple(rule["source"])
            or (frame["target_system"], frame["target_component"]) != tuple(rule["target"])
            or not begin <= frame["monotonic_ns"] <= end):
        return "vehicle_self_command_on_gcs_link"
    key = str(match["command"]) + ":" + match["seven_float32_le_hex"]
    counts[key] = counts.get(key, 0) + 1
    if counts[key] > rule["max_frames_per_payload"]:
        return "vehicle_landing_self_command_cap_exceeded"
    return None


def evaluate_policy(closed, effective, startup_window_end_ns=None, landing_window_ns=None):
    """Classify every captured command/control frame. Unknown traffic is a violation."""
    policy = effective["policy"]
    violations, allowed, heartbeats = [], [], 0
    pattern_counts, command_counts = {}, {}
    landing_allowed, landing_counts = [], {}
    for frame in closed["frames_of_interest"]:
        origin = (frame["source_system"], frame["source_component"])
        if frame["message_id"] == 0:
            heartbeats += origin == (255, 190)
            continue
        if origin == VEHICLE:
            if "command" in frame and frame["target_system"] == VEHICLE[0]:
                reason = landing_self_command(frame, policy.get("vehicle_landing_self_commands"),
                                              landing_window_ns, landing_counts)
                if reason is None:
                    landing_allowed.append(frame)
                else:
                    violations.append(dict(frame, reason=reason))
            continue
        if frame.get("active_control_message"):
            violations.append(dict(frame, reason="active_control_message_from_non_vehicle"))
            continue
        rule = next((r for r in policy.get("allowed_commands", [])
                     if frame.get("message_id") == r["message_id"] and frame.get("command") == r["command"]
                     and origin == tuple(r["source"]) and (frame["target_system"], frame["target_component"]) == tuple(r["target"])),
                    None)
        if rule is None:
            violations.append(dict(frame, reason="command_not_permitted_by_policy"))
            continue
        if rule.get("startup_window_only") and (startup_window_end_ns is None or frame["monotonic_ns"] > startup_window_end_ns):
            violations.append(dict(frame, reason="command_outside_startup_window"))
            continue
        patterns = rule.get("payload_patterns")
        if patterns is not None:
            match = next((p for p in patterns if p["seven_float32_le_hex"] == frame.get("params_float32_le_hex")), None)
            if match is None:
                violations.append(dict(frame, reason="payload_not_in_frozen_patterns"))
                continue
            key = str(rule["command"]) + ":" + match["seven_float32_le_hex"]
            pattern_counts[key] = pattern_counts.get(key, 0) + 1
            if pattern_counts[key] > match["maximum_frames"]:
                violations.append(dict(frame, reason="payload_pattern_cap_exceeded"))
                continue
        command_counts[rule["command"]] = command_counts.get(rule["command"], 0) + 1
        if command_counts[rule["command"]] > rule["maximum_frames"]:
            violations.append(dict(frame, reason="command_cap_exceeded"))
            continue
        allowed.append(frame)
    failures = [v["reason"] for v in violations]
    if not closed.get("complete"):
        failures.append("capture_incomplete")
    if heartbeats < 1:
        failures.append("gcs_heartbeat_not_observed")
    return {"policy": effective["name"], "policy_sha256": effective["policy_sha256"],
            "approval": effective["approval"], "gcs_heartbeats": heartbeats, "allowed_frames": len(allowed),
            "allowed_by_command": command_counts,
            "allowed_vehicle_landing_self_commands": {"frames": len(landing_allowed), "by_payload": landing_counts,
                                                      "landing_window_ns": list(landing_window_ns) if landing_window_ns else None},
            "violations": violations[:200], "violation_count": len(violations),
            "global_zero_native_commands": not allowed and not landing_allowed and not violations,
            "failures": sorted(set(failures)),
            "status": "passed" if not failures else "failed"}


def ulog_commands(path):
    """All native VehicleCommand publications in the ULog (all instances)."""
    from pyulog import ULog
    log = ULog(str(path), ["vehicle_command"])
    rows = []
    for dataset in log.data_list:
        for i in range(len(dataset.data["timestamp"])):
            rows.append({key: (values[i].item() if hasattr(values[i], "item") else values[i])
                         for key, values in dataset.data.items() if not key.startswith("param")} |
                        {"instance": dataset.multi_id})
    return rows
