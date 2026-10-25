"""Independent P3 sensor stream accounting (evaluator only; imports no adapter code).

Three accounting boundaries are reconciled against the native Gazebo header
ledger written by the evaluator-only source probe:

* source: native publication sequence/timestamps (render gaps vs probe gaps);
* delivery: every in-window native frame must be received by the ROS adapter;
* recording: every adapter receipt must be in the hashed raw recording;
* processing: intentional newest-frame skips are reported, never credited.

Missing, truncated, duplicate or non-monotonic data fails; nothing becomes zero.
"""
import json
import math
from pathlib import Path
import zlib

import numpy as np

from p3_provenance import digest, digest_bytes, strict_json, strict_jsonl


class Failures:
    def __init__(self):
        self.items = []

    def check(self, condition, reason):
        if not condition:
            self.items.append(reason)
        return bool(condition)


def load_sensor_artifacts(run):
    run = Path(run)
    sensors = run / "sensors"
    adapter = strict_json(sensors / "adapter-result.json")
    closed = strict_json(sensors / "depth-recorder-closed.json")
    index = strict_jsonl(sensors / "depth-index.jsonl")
    observations = strict_jsonl(sensors / "observations.jsonl")
    health = strict_jsonl(sensors / "health.jsonl")
    return adapter, closed, index, observations, health


def read_frames(run, index, width=640, height=480):
    """Yield (row, float32 image or None) after verifying continuity and both hashes."""
    offset = 0
    with (Path(run) / "sensors/depth.bin").open("rb") as data:
        for number, row in enumerate(index, 1):
            if row.get("receipt_id") != number or row.get("offset") != offset:
                raise ValueError("raw_index_continuity")
            stored = data.read(row["stored_bytes"])
            if len(stored) != row["stored_bytes"] or digest_bytes(stored) != row["stored_sha256"]:
                raise ValueError("raw_stored_hash")
            raw = zlib.decompress(stored) if row["compression"] == "zlib-1" else stored
            if len(raw) != row["raw_bytes"] or digest_bytes(raw) != row["raw_sha256"]:
                raise ValueError("raw_payload_hash")
            offset += len(stored)
            image = None
            if row.get("accepted"):
                if (row["width"], row["height"], row["encoding"], row["step"], row["is_bigendian"]) != (
                        width, height, "32FC1", width * 4, 0):
                    raise ValueError("raw_layout")
                image = np.frombuffer(raw, dtype="<f4").reshape(height, width)
            yield row, image
        if data.read(1) != b"":
            raise ValueError("raw_trailing_data")


def source_ledger(run, name):
    rows = strict_jsonl(Path(run) / "source" / name)
    for number, row in enumerate(rows, 1):
        if row.get("id") != number:
            raise ValueError("source_ledger_identity:" + name)
    return rows


def evaluate_source(rows, window, limits, label):
    """Native publication continuity inside [begin, end] simulation seconds."""
    f = Failures()
    begin, end = window
    seq = [r["seq"] for r in rows]
    f.check(all(b == a + 1 for a, b in zip(seq, seq[1:])), label + ":probe_sequence_gap_or_duplicate")
    stamps = [r["stamp_ns"] for r in rows]
    f.check(all(b > a for a, b in zip(stamps, stamps[1:])), label + ":source_timestamp_not_increasing")
    chosen = [r for r in rows if begin * 1e9 <= r["stamp_ns"] <= end * 1e9]
    result = {"frames_in_window": len(chosen), "window_sim_s": [begin, end]}
    if not f.check(len(chosen) >= 2, label + ":no_source_frames_in_window"):
        result["failures"] = f.items
        return result, chosen
    times = np.array([r["stamp_ns"] for r in chosen], dtype=np.int64) / 1e9
    gaps = np.diff(times)
    rate = (len(chosen) - 1) / (times[-1] - times[0])
    result.update(source_hz_sim=float(rate), source_gap_max_sim_s=float(gaps.max()),
                  late_or_unrendered_intervals=int(np.count_nonzero(gaps > 1.5 / 30)),
                  first_seq=chosen[0]["seq"], last_seq=chosen[-1]["seq"])
    f.check(times[0] - begin <= limits["source_gap_sim_s"] and end - times[-1] <= limits["source_gap_sim_s"],
            label + ":window_coverage")
    f.check(gaps.max() <= limits["source_gap_sim_s"], label + ":source_gap")
    f.check(rate >= limits["delivered_hz_min"], label + ":source_rate")
    result["failures"] = f.items
    return result, chosen


def evaluate_stream(run, window, limits, sensors_config, require_association=False):
    """Complete depth stream evaluation for one declared window."""
    run = Path(run)
    f = Failures()
    adapter, closed, index, observations, health = load_sensor_artifacts(run)
    out = {"window_sim_s": list(window)}
    f.check(adapter.get("status") == "passed", "adapter_result_not_passed")
    f.check(closed == adapter.get("recorder"), "recorder_close_marker_differs")
    f.check(closed.get("complete") is True and closed.get("overflow") == limits["recorder_overflow_allowed"]
            and closed.get("accepted") == closed.get("written") == len(index), "raw_recorder_incomplete")
    publications = adapter.get("observed_own_publications") or {}
    allowed = {sensors_config["observation_topic"], sensors_config["health_topic"]}
    f.check(bool(publications) and set(publications) <= allowed
            and all(types == ["std_msgs/msg/String"] for types in publications.values()),
            "adapter_publication_boundary")
    source_rows = source_ledger(run, "depth-headers.jsonl")
    source, chosen = evaluate_source(source_rows, window, limits, "depth")
    out["source"] = source
    f.items += source.pop("failures")
    begin, end = window
    accepted = [r for r in index if r["accepted"]]
    received = {r["acquisition_stamp_ns"] for r in index}
    in_window = [r for r in accepted if begin * 1e9 <= r["acquisition_stamp_ns"] <= end * 1e9]
    missing = [r["stamp_ns"] for r in chosen if r["stamp_ns"] not in received]
    out["delivery"] = {"source_frames": len(chosen), "adapter_receipts": len(in_window),
                       "source_frames_missing_in_ros": len(missing),
                       "recorded_fraction": (len(chosen) - len(missing)) / len(chosen) if chosen else None,
                       "rejected_in_window": len([r for r in index if not r["accepted"]
                                                  and begin * 1e9 <= (r["acquisition_stamp_ns"] or 0) <= end * 1e9])}
    f.check(chosen and out["delivery"]["recorded_fraction"] >= limits["recorded_fraction_required"],
            "source_to_ros_delivery_loss")
    f.check(out["delivery"]["rejected_in_window"] == 0, "rejected_frames_in_window")
    f.check(len(in_window) == len(chosen) and not set(r["acquisition_stamp_ns"] for r in in_window) - set(
        r["stamp_ns"] for r in chosen), "ros_receipts_not_in_native_ledger")
    if len(in_window) >= 2:
        stamps = np.array([r["acquisition_stamp_ns"] for r in in_window]) / 1e9
        receipts = np.array([r["receipt_wall_s"] for r in in_window])
        out["delivery"].update(delivered_hz_sim=float((len(stamps) - 1) / (stamps[-1] - stamps[0])),
                               delivered_hz_wall=float((len(receipts) - 1) / (receipts[-1] - receipts[0])),
                               receipt_gap_max_wall_s=float(np.diff(receipts).max()))
        latency = [r["receipt_sim_s"] - r["acquisition_stamp_ns"] / 1e9 for r in in_window
                   if r["receipt_sim_s"] is not None]
        out["delivery"]["source_to_receipt_sim_s"] = {"min": min(latency), "max": max(latency)} if latency else None
        f.check(out["delivery"]["delivered_hz_sim"] >= limits["delivered_hz_min"], "delivered_rate")
        f.check(out["delivery"]["receipt_gap_max_wall_s"] <= limits["stall_wall_s"], "receipt_stall")
        f.check(bool(latency) and max(latency) <= limits["observation_age_sim_s"], "source_to_receipt_age")
    else:
        f.check(False, "insufficient_ros_receipts")
    window_obs = [o for o in observations if begin <= o["acquisition_sim_s"] <= end]
    ids = [o["observation_id"] for o in observations]
    f.check(ids == list(range(1, len(ids) + 1)), "observation_sequence")
    receipts_used = [o["receipt_id"] for o in observations]
    f.check(len(set(receipts_used)) == len(receipts_used), "observation_reused_frame")
    span = end - begin
    out["processing"] = {"observations": len(window_obs), "processed_hz_sim": len(window_obs) / span if span else None,
                         "observation_age_max_sim_s": max((o["observation_age_sim_s"] for o in window_obs), default=None),
                         "skipped_by_processing_total": adapter["accounting"]["skipped_by_processing"]}
    f.check(out["processing"]["processed_hz_sim"] is not None
            and out["processing"]["processed_hz_sim"] >= limits["processed_hz_min"], "processed_rate")
    f.check(window_obs and out["processing"]["observation_age_max_sim_s"] <= limits["observation_age_sim_s"],
            "observation_age")
    f.check(all(o["health"] == "fresh" for o in window_obs), "observation_while_not_fresh")
    window_health = [h for h in health if h["sim_s"] is not None and begin <= h["sim_s"] <= end]
    out["health"] = {"records": len(window_health),
                     "non_fresh": [h for h in window_health if h["status"] != "fresh"][:20]}
    f.check(window_health and not out["health"]["non_fresh"], "health_not_fresh_in_window")
    if require_association:
        statuses = {}
        for o in window_obs:
            statuses[o["association"]["status"]] = statuses.get(o["association"]["status"], 0) + 1
        out["association"] = statuses
        f.check(window_obs and statuses.get("matched", 0) == len(window_obs), "state_association_incomplete")
        deltas = [abs(o["association"]["position"]["delta_s"]) for o in window_obs
                  if o["association"]["status"] == "matched"]
        f.check(not deltas or max(deltas) <= limits["state_mismatch_s"], "state_association_mismatch")
    out["calibration"] = {"identity": (adapter.get("calibration") or {}).get("identity"),
                          "rejected": adapter.get("calibration_rejected"),
                          "invalid_reason": adapter.get("calibration_invalid_reason")}
    f.check(out["calibration"]["identity"] is not None and out["calibration"]["invalid_reason"] is None,
            "calibration_unavailable_or_invalidated")
    out["recorder"] = closed
    out["costs"] = adapter.get("costs")
    out["raw_hashes"] = {name: digest(run / "sensors" / name) for name in
                         ("depth.bin", "depth-index.jsonl", "observations.jsonl", "health.jsonl",
                          "adapter-result.json", "depth-recorder-closed.json")}
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    return out


def evaluate_rgb(run, window, limits, rgb_config):
    """Native RGB production/delivery evidence from the passive metadata observer."""
    run = Path(run)
    f = Failures()
    closed = strict_json(run / "source/probe-closed.json")
    rows = source_ledger(run, "rgb-headers.jsonl")
    f.check(closed.get("rgb_subscribed") is True and closed.get("error") is None, "rgb_observer_inactive")
    for row in rows:
        if (row["width"], row["height"]) != (rgb_config["width"], rgb_config["height"]):
            f.check(False, "rgb_dimensions")
            break
    source, chosen = evaluate_source(rows, window, limits, "rgb")
    f.items += source.pop("failures")
    if len(chosen) >= 2:
        receipts = np.array([r["receipt_monotonic_ns"] for r in chosen]) / 1e9
        source["receipt_gap_max_wall_s"] = float(np.diff(receipts).max())
        source["delivered_hz_wall"] = float((len(receipts) - 1) / (receipts[-1] - receipts[0]))
        f.check(source["receipt_gap_max_wall_s"] <= limits["stall_wall_s"], "rgb_receipt_stall")
    source["failures"] = f.items
    source["status"] = "passed" if not f.items else "failed"
    source["pixels_retained"] = False
    return source


def native_rtf(run):
    """Simulation progress per monotonic second from the probe's native clock samples."""
    rows = strict_jsonl(Path(run) / "source/clock.jsonl")
    positive = [r for r in rows if r["sim_s"] > 0]
    if len(positive) < 2:
        return None
    sim = positive[-1]["sim_s"] - positive[0]["sim_s"]
    wall = (positive[-1]["receipt_monotonic_ns"] - positive[0]["receipt_monotonic_ns"]) / 1e9
    steps = [(b["sim_s"] - a["sim_s"]) / ((b["receipt_monotonic_ns"] - a["receipt_monotonic_ns"]) / 1e9)
             for a, b in zip(positive, positive[1:]) if b["receipt_monotonic_ns"] > a["receipt_monotonic_ns"]]
    return {"sim_s": sim, "wall_s": wall, "rtf": sim / wall if wall > 0 else None,
            "min_interval_rtf": min(steps) if steps else None, "samples": len(positive)}


def finite(value):
    return value is not None and isinstance(value, (int, float)) and math.isfinite(value)


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
