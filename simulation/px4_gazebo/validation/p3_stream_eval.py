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
    publications = dict(adapter.get("observed_own_publications") or {})
    # Every rclpy node publishes these two ROS infrastructure topics; exact types only.
    infrastructure = {"/rosout": ["rcl_interfaces/msg/Log"], "/parameter_events": ["rcl_interfaces/msg/ParameterEvent"]}
    for topic, types in infrastructure.items():
        if publications.get(topic) == types:
            del publications[topic]
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
        out["association_equidistant_ties_earlier"] = {
            part: sum(1 for o in window_obs if o["association"].get(part, {}).get("equidistant_tie_earlier"))
            for part in ("position", "attitude")}
        bracket_rule = (limits.get("state_association") or {}).get("reset_epoch_bracket")
        bracketed = [o for o in window_obs if bracket_rule
                     and o["association"]["status"] in bracket_rule["allowed_statuses"]]
        f.check(window_obs and statuses.get("matched", 0) + len(bracketed) == len(window_obs),
                "state_association_incomplete")
        if bracketed:
            out["reset_bracketed"] = evaluate_reset_brackets(run, bracketed, bracket_rule, limits, f)
        out["reset_bracketed_observations"] = len(bracketed)
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


RESET_EPOCHS = {"position": ("vehicle_local_position",
                             ("xy_reset_counter", "z_reset_counter", "heading_reset_counter", "ref_timestamp")),
                "attitude": ("vehicle_attitude", ("quat_reset_counter",))}


def recorded_resets(run, topic_key, keys):
    """Epoch steps on one PX4 topic, from the controller's own received-message ledger."""
    steps, previous = [], None
    for row in strict_jsonl(Path(run) / "ros-events.jsonl"):
        if row.get("event") != "received" or row.get("topic_key") != topic_key:
            continue
        fields = row["fields"]
        epoch = {key: fields.get(key) for key in keys}
        if previous is not None and epoch != previous:
            steps.append({"timestamp_sample_us": int(fields.get("timestamp_sample") or fields["timestamp"]),
                          "before": previous, "after": epoch})
        previous = epoch
    return steps


def single_reset_step(before, after, keys):
    """The one changed epoch key, or None: counters must step by exactly +1, ref_timestamp may change."""
    if set(before) != set(keys) or set(after) != set(keys):
        return None
    changed = [key for key in keys if before[key] != after[key]]
    if len(changed) != 1:
        return None
    key = changed[0]
    if key != "ref_timestamp":
        b, a = before[key], after[key]
        if not (isinstance(b, int) and isinstance(a, int) and not isinstance(b, bool)
                and not isinstance(a, bool) and a == b + 1):
            return None
    return key


def evaluate_reset_brackets(run, bracketed, rule, limits, f):
    """Contract v1.4 rule 1: a reset_epoch_bracket observation is acceptable only at one recorded,
    single-key, independently corroborated PX4 reset, at most once per (state, reset event)."""
    span_us = int(round(limits[rule["bracket_span_limit_key"]] * 1e6))
    ledgers = {state: recorded_resets(run, topic, keys) for state, (topic, keys) in RESET_EPOCHS.items()}
    per_event, details = {}, []
    for o in bracketed:
        acquisition_us = int(o["acquisition_stamp_ns"] // 1000)
        parts = {state: o["association"].get(state) or {} for state in RESET_EPOCHS}
        record = {"observation_id": o["observation_id"], "acquisition_us": acquisition_us,
                  "status": o["association"]["status"], "resets": []}
        for state, part in parts.items():
            if part.get("status") == "matched":
                f.check(abs(part["delta_s"]) <= limits["state_mismatch_s"], "state_association_mismatch")
                continue
            if part.get("status") != "reset_epoch_bracket":
                f.check(False, "state_association_incomplete")
                continue
            keys = RESET_EPOCHS[state][1]
            before, after = part.get("before") or {}, part.get("after") or {}
            b_us, a_us = before.get("timestamp_us"), after.get("timestamp_us")
            recorded = (isinstance(b_us, int) and isinstance(a_us, int)
                        and b_us <= acquisition_us <= a_us and a_us - b_us <= span_us)
            key = single_reset_step(before.get("epoch") or {}, after.get("epoch") or {}, keys) if recorded else None
            event = None
            if key is not None:
                matches = [step for step in ledgers[state]
                           if b_us <= step["timestamp_sample_us"] <= a_us
                           and step["before"][key] == before["epoch"][key] and step["after"][key] == after["epoch"][key]
                           and single_reset_step(step["before"], step["after"], keys) == key]
                event = (state, matches[0]["timestamp_sample_us"]) if len(matches) == 1 else None
            if f.check(recorded, "reset_bracket_not_recorded") and f.check(key is not None, "reset_bracket_not_single_reset") \
                    and f.check(event is not None, "reset_bracket_uncorroborated"):
                per_event[event] = per_event.get(event, 0) + 1
                record["resets"].append({"state": state, "key": key, "step": [before["epoch"][key], after["epoch"][key]],
                                         "bracket_us": [b_us, a_us], "ledger_timestamp_sample_us": event[1]})
        details.append(record)
    f.check(all(count <= rule["max_per_reset_event"] for count in per_event.values()), "reset_bracket_bound")
    f.check(len(bracketed) <= len(per_event), "reset_bracket_bound")
    return {"observations": details, "corroborated_reset_events": len(per_event),
            "per_event": {"%s@%d" % event: count for event, count in sorted(per_event.items())}}


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


def evaluate_rgb_absence(run, window_monotonic, settings, owned_pids):
    """Contract v1.3 flights: verify that no process consumed IMX214 during the flight window.

    Sampled discovery monitor plus endpoint `gz topic -i` snapshots with a depth positive
    control and a closed process set. Not continuous observation (see the v1.3 proposal).
    """
    run = Path(run)
    f = Failures()
    begin, end = window_monotonic
    marker, control = settings["topic_marker"], settings["control_topic"]
    out = {"rgb_workload": "depth_only_flight", "window_monotonic_s": [begin, end]}
    closed = strict_json(run / "source/probe-closed.json")
    own = closed.get("subscribed_topics")
    f.check(closed.get("rgb_subscribed") is False and closed.get("rgb_topic") is None
            and closed.get("rgb_absence_monitor") is True and closed.get("error") is None, "probe_not_rgb_absent")
    f.check(isinstance(own, list) and not any(marker in t for t in own), "probe_subscribed_to_imx214")
    out["rgb_metadata_rows"] = len(source_ledger(run, "rgb-headers.jsonl"))
    f.check(out["rgb_metadata_rows"] == 0, "rgb_metadata_rows_in_flight")

    path = run / "source/rgb-absence-snapshots.json"
    record = strict_json(path) if path.exists() else {"snapshots": {}, "errors": ["missing"]}
    f.check(not record.get("errors"), "snapshot_errors")
    allowed_groups = set(owned_pids)
    for label, lower, upper in (("window_start", begin, end), ("window_end", end, math.inf)):
        snap = record["snapshots"].get(label)
        if not f.check(snap is not None, "snapshot_missing:" + label):
            continue
        f.check(lower <= snap["begin_monotonic_s"] <= upper, "snapshot_outside_window:" + label)
        topics = snap["imx214_topics"]
        f.check(any(t.endswith("/IMX214/image") for t in topics), "snapshot_image_topic_absent:" + label)
        for topic in topics:
            info = snap["topics"].get(topic, {})
            f.check(info.get("returncode") == 0 and info.get("subscribers") == 0, "snapshot_subscriber:" + label)
        ctrl = snap["topics"].get(control, {})
        f.check(ctrl.get("returncode") == 0 and (ctrl.get("subscribers") or 0) >= 1, "snapshot_control_failed:" + label)
        unknown = [p for p in snap["processes"] if p["pgid"] not in allowed_groups and not (
            p["pgid"] == record.get("runner_pgid") and ("gz topic" in p["cmdline"] or "gz-transport-topic" in p["cmdline"]))]
        f.check(not unknown, "unowned_process:" + label)
        out[label] = {"imx214_topics": topics, "control_subscribers": ctrl.get("subscribers"),
                      "unowned_processes": len(unknown)}

    monitor_path = run / "source/imx214-subscribers.jsonl"
    rows = strict_jsonl(monitor_path) if monitor_path.exists() else []
    if not f.check(len(rows) >= 2, "monitor_missing"):
        out["failures"], out["status"] = f.items, "failed"
        return out
    f.check([r["id"] for r in rows] == list(range(1, len(rows) + 1)), "monitor_ledger_identity")
    times = [r["monotonic_ns"] / 1e9 for r in rows]
    first = max((i for i, t in enumerate(times) if t <= begin), default=None)
    last = min((i for i, t in enumerate(times) if t >= end), default=None)
    if not f.check(first is not None and last is not None, "monitor_coverage"):
        out["failures"], out["status"] = f.items, "failed"
        return out
    span = rows[first:last + 1]
    gaps = [b - a for a, b in zip(times[first:last + 1], times[first + 1:last + 1])]
    out["monitor"] = {"rows_total": len(rows), "rows_in_window_span": len(span),
                      "max_sample_gap_wall_s": max(gaps) if gaps else None,
                      "coverage_monotonic_s": [times[first], times[last]]}
    f.check(not gaps or max(gaps) <= settings["max_sample_gap_wall_s"], "monitor_gap")
    f.check(all(r["error"] is None for r in span), "monitor_error")
    f.check(all(any(t.endswith("/IMX214/image") for t in r["topics"]) for r in span), "monitor_image_topic_absent")
    f.check(all(count == 0 for r in span for count in r["topics"].values()), "unexpected_subscriber")
    f.check(all((r["control_subscribers"] or 0) >= 1 for r in span), "monitor_control_lost")
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    return out


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
