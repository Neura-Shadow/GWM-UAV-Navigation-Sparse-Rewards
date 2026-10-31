"""Independent offline P4 evaluator for one finalized run directory (contract §6, §6.1, §7, §9, §9.1).

Never launches anything; output is exclusive-create. Pass decisions never rely on
P4 self-reports alone: safety comes from Gazebo ground truth (poses and contacts)
against the declared obstacles, stream limits from the unchanged P3 evaluators,
flight integrity from the unchanged P2 evaluator functions, and decision legality
from the recorded ledgers checked against the recorded maps.

* p3_readiness: the unchanged P3 readiness evaluation (P2 controller, RGB consumer).
* p4_readiness / fixture / fault: P3 sensor checks, P4 controller ground-diagnostic integrity,
  §6.1 performance, reconstruction and trigger decisions (fixture), offline supervisor replay (fault).
* flight: P3 checks, P2 flight integrity, §6 safety, §6.1, scenario outcome, §9.1 for S6.
"""
import argparse
import json
import math
from pathlib import Path
import sys

P4 = Path(__file__).resolve().parents[1]
SIM = P4.parents[0] / "px4_gazebo"
for path in (SIM / "validation", SIM / "ros2_ws/src/gwm_px4_control", SIM / "ros2_ws/src/gwm_sensor_adapter",
             P4 / "ros2_ws/src/gwm_obstacle_avoidance", P4 / "validation"):
    if str(path) not in sys.path:
        sys.path.append(str(path))

from p3_provenance import atomic_json, digest, strict_json, strict_jsonl  # noqa: E402
from p3_stream_eval import Failures, evaluate_rgb, evaluate_rgb_absence, evaluate_stream, native_rtf  # noqa: E402

EVALUATOR_FILES = ("p4_evaluate.py", "p4_truth_observer.py")
VEHICLE = "x500_depth_71"


def evaluator_identity():
    files = {name: digest(P4 / "validation" / name) for name in EVALUATOR_FILES}
    import p3_evaluate
    files.update({"p3:" + k: v for k, v in p3_evaluate.evaluator_identity().items()})
    return files


def pick(values, q):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))] if ordered else None


def ledger(run, name):
    path = Path(run) / name
    return strict_jsonl(path) if path.exists() else None


# ---- §6.1 runtime performance ----------------------------------------------------------------------
def performance(run, window, limits, f):
    rows = ledger(run, "p4/local_map.jsonl")
    if rows is None:
        f.check(False, "performance:local_map_ledger_missing")
        return None
    inside = [r for r in rows if r.get("processing_sim_s") is not None and window[0] <= r["processing_sim_s"] <= window[1]]
    span = window[1] - window[0]
    rate = len(inside) / span if span > 0 else 0.0
    latency = [r["latency_sim_s"] for r in inside if r.get("latency_sim_s") is not None]
    compute = [r["compute_wall_s"] for r in inside if r.get("compute_wall_s") is not None]
    gaps = [r["receipt_gap_max_wall_s"] for r in inside if r.get("receipt_gap_max_wall_s") is not None]
    out = {"window_sim_s": list(window), "snapshots": len(inside), "rate_hz": rate,
           "latency_max_sim_s": max(latency, default=None), "latency_p95_sim_s": pick(latency, 0.95),
           "compute_max_s": max(compute, default=None), "compute_p99_s": pick(compute, 0.99),
           "receipt_gap_max_wall_s": max(gaps, default=None),
           "usable_fraction": (sum(1 for r in inside if r.get("usable")) / len(inside)) if inside else None}
    # Diagnostic only (revision 5): thread CPU time and involuntary switches of the slowest frame.
    slowest = max((r for r in inside if r.get("compute_wall_s") is not None), key=lambda r: r["compute_wall_s"], default=None)
    out["slowest_frame"] = None if slowest is None else {
        k: slowest.get(k) for k in ("acquisition_sim_s", "compute_wall_s", "compute_thread_cpu_s", "involuntary_context_switches")}
    f.check(rate >= limits["snapshot_rate_min_hz"], "performance:snapshot_rate")
    f.check(latency and out["latency_max_sim_s"] <= limits["snapshot_latency_max_sim_s"], "performance:snapshot_latency_max")
    f.check(latency and out["latency_p95_sim_s"] <= limits["snapshot_latency_p95_sim_s"], "performance:snapshot_latency_p95")
    f.check(compute and out["compute_p99_s"] <= limits["perception_compute_p99_s"], "performance:compute_p99")
    f.check(compute and out["compute_max_s"] <= limits["perception_compute_max_s"], "performance:compute_max")
    f.check(gaps and out["receipt_gap_max_wall_s"] <= limits["perception_receipt_gap_max_wall_s"],
            "performance:perception_receipt_gap")
    result = strict_json(Path(run) / "p4/perception-result.json") if (Path(run) / "p4/perception-result.json").exists() else {}
    f.check(result.get("status") == "passed", "performance:perception_result_not_passed")
    out["perception_result"] = {k: result.get(k) for k in ("status", "error", "snapshots", "usable", "unusable_reasons")}
    return out


# ---- P2 integrity, function by function, without P2 mission windows ---------------------------------
def p2_integrity(run, summary, f, flight):
    import collect_p2_evidence as p2
    from timing_evidence import verify_timing
    run = Path(run)
    result, config = summary["controller_result"], summary["config"]
    streams, counts, events = p2.read_bag(run)
    json_events = [json.loads(line) for line in (run / "ros-events.jsonl").read_text().splitlines()]
    out = {"bag_message_counts": dict(counts)}
    f.check(events == [r for r in json_events if r["event"] != "received"], "p2:bag_event_ledger_differs")
    for key in ("vehicle_local_position", "vehicle_status", "vehicle_attitude", "vehicle_land_detected",
                "estimator_status_flags", "failsafe_flags"):
        f.check(bool(streams[key]) and len(streams[key]) == sum(r.get("topic_key") == key for r in json_events),
                "p2:bag_received_ledger_count:" + key)
    log, ulog = p2.read_ulog(summary["ulog_files"][0]["path"], flight)
    f.check(not log.dropouts, "p2:ulog_dropout")
    timing = verify_timing(run, result, streams, events, config)
    out["timing"] = timing
    f.check(timing.get("status") == "passed", "p2:timing")
    if not flight:
        f.check(not streams["vehicle_command"], "p2:ground_scope_published_vehicle_command")
        return out, log, streams, events
    sent = streams["vehicle_command"]
    own = [r for r in ulog["vehicle_command"] if r["source_system"] == config["source_system"]
           and r["source_component"] == config["source_component"]]
    out["command_ids"] = [r["command"] for r in sent]
    f.check(out["command_ids"] == [176, 400, 21], "p2:command_sequence")
    f.check([r["command"] for r in own] == out["command_ids"]
            and all(a["timestamp"] == b["timestamp"] for a, b in zip(own, sent)), "p2:ulog_ros_command_mismatch")
    for command in sent:
        matches = [a for a in streams["vehicle_command_ack"] if a["command"] == command["command"]
                   and a["target_system"] == 201 and a["target_component"] == 191 and a["from_external"] is False
                   and 0 <= (a["timestamp"] - command["timestamp"]) / 1e6 <= config["ack_deadline_sim_s"]]
        f.check(any(a["result"] == 0 for a in matches), "p2:missing_accepted_ack:" + str(command["command"]))
    status, land = ulog["vehicle_status"], ulog["vehicle_land_detected"]
    out.update(final_disarmed=status[-1]["arming_state"] == 1, final_landed=bool(land[-1]["landed"]),
               failsafe_observed=any(s["failsafe"] for s in status))
    f.check(out["final_landed"] and out["final_disarmed"], "p2:final_landed_disarmed")
    f.check(not out["failsafe_observed"], "p2:failsafe_observed")
    keyed = {}
    for p in ulog["vehicle_local_position"]:
        keyed.setdefault((p["timestamp"], p["timestamp_sample"]), []).append(p)
    matching = [p for p in streams["vehicle_local_position"] if (p["timestamp"], p["timestamp_sample"]) in keyed]
    ambiguous = any(len({(u["x"], u["y"], u["z"]) for u in keyed[(p["timestamp"], p["timestamp_sample"])]}) > 1
                    for p in matching)
    delta = max((abs(p[k] - keyed[(p["timestamp"], p["timestamp_sample"])][0][k]) for p in matching for k in "xyz"),
                default=None)
    out["estimator_correlation"] = {"pairs": len(matching), "max_position_difference_m": delta}
    f.check(not ambiguous and len(matching) >= 100 and delta is not None and delta < 1e-6, "p2:ros_ulog_estimator")
    from reference_evidence import classify
    from yaw_evidence import verify_yaw_ownership
    reference = classify(run, config)
    out["yaw_ownership"] = verify_yaw_ownership(log, streams, events, result, config, reference)
    f.check((out["yaw_ownership"] or {}).get("status") == "passed", "p2:yaw_ownership")
    return out, log, streams, events


# ---- §6 safety from ground truth ------------------------------------------------------------------
def box_clearance(xy, box, r_body):
    cx, cy = box["center_m"][:2]
    sx, sy = box["size_m"][:2]
    dx = max(abs(xy[0] - cx) - sx / 2, 0.0)
    dy = max(abs(xy[1] - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy) - r_body


def truth_track(run, window_ns=None):
    rows = ledger(run, "p4/truth_poses.jsonl") or []
    track = [(r["stamp_ns"], r["poses"][VEHICLE]["position"]) for r in rows if VEHICLE in r["poses"]]
    if window_ns:
        track = [t for t in track if window_ns[0] <= t[0] <= window_ns[1]]
    return track


def local_to_world(run, track):
    """Offset from PX4 local ENU to Gazebo world at the takeoff origin (both measured, no assumption)."""
    events = [json.loads(line) for line in (Path(run) / "ros-events.jsonl").read_text().splitlines()]
    prestream = next(e for e in events if e.get("event") == "transition" and e.get("to") == "PRESTREAM_SAFE_SETPOINTS")
    n, e, _ = prestream["evidence"]["origin_ned"]
    stamp = int(prestream["sim_s"] * 1e9)
    near = min(track, key=lambda t: abs(t[0] - stamp))
    return (near[1][0] - e, near[1][1] - n), events


def safety(run, declaration, settings, envelope, f):
    boxes = declaration["boxes"]
    track = truth_track(run)
    out = {"truth_samples": len(track)}
    f.check(len(track) > 10, "safety:truth_poses_missing")
    if not track:
        return out, None, []
    gaps = [(b[0] - a[0]) / 1e9 for a, b in zip(track, track[1:])]
    airborne = [t for t in track if t[1][2] > 0.5]
    out["truth_interval_max_s"] = max(((b[0] - a[0]) / 1e9 for a, b in zip(airborne, airborne[1:])), default=None)
    f.check(out["truth_interval_max_s"] is not None and out["truth_interval_max_s"] <= settings["truth_pose_interval_max_s"],
            "safety:truth_pose_interval")
    clearances = [min((box_clearance(p[:2], b, envelope["r_body_m"]) for b in boxes), default=math.inf) for _, p in track]
    out["min_truth_clearance_m"] = min(clearances) - settings["truth_motion_allowance_m"] if boxes else None
    f.check(not boxes or out["min_truth_clearance_m"] >= settings["min_truth_clearance_m"], "safety:truth_clearance")
    contacts = ledger(run, "p4/contacts.jsonl") or []
    hits = [c for c in contacts if any(VEHICLE in pair[0] or VEHICLE in pair[1] for pair in c["contacts"])]
    out["collisions"] = len(hits)
    f.check(len(hits) <= settings["collisions_allowed"], "safety:collision_contact")
    f.check((Path(run) / "p4/truth-closed.json").exists() and strict_json(Path(run) / "p4/truth-closed.json").get("error") is None,
            "safety:truth_observer_error")
    offset, events = local_to_world(run, track)
    out["local_to_world_offset_m"] = list(offset)
    setpoints = [e for e in events if e.get("event") == "setpoint" and e.get("phase") == "TRANSIT"]
    samples = {round(e["ros_sim_s"], 6): e for e in events if e.get("event") == "sample" and e.get("phase") == "TRANSIT"}
    inside, tracking = [], []
    for s in setpoints:
        n, e, _ = s["fields"]["position"]
        world = (e + offset[0], n + offset[1])
        margin = envelope["r_body_m"] + settings["setpoint_obstacle_margin_m"]
        if any(box_clearance(world, b, 0.0) < margin for b in boxes):
            inside.append(round(s["ros_sim_s"], 3))
    for s in events:
        if s.get("event") == "p4_decision" and s.get("target_enu") and s.get("vehicle_enu"):
            tracking.append(math.dist(s["target_enu"], s["vehicle_enu"]))
    out["setpoints_inside_inflated_obstacles"] = inside[:20]
    out["tracking_max_m"] = max(tracking, default=None)
    f.check(not inside, "safety:setpoint_inside_inflated_obstacle")
    f.check(tracking and out["tracking_max_m"] <= settings["tracking_horizontal_m"], "safety:tracking")
    return out, offset, events


def response_latency(events, limit, f):
    decisions = [e for e in events if e.get("event") == "p4_decision"]
    first = next((d for d in decisions if d.get("occupied_in_corridor") and d.get("path_valid") is False), None)
    if first is None:
        return None
    left = next((d for d in decisions if d["sim_s"] >= first["sim_s"] and d["state"] != "NORMAL"), None)
    latency = None if left is None else left["sim_s"] - first["snapshot_acquisition_sim_s"]
    f.check(latency is not None and latency <= limit, "safety:response_latency")
    return latency


def abort_landing(result, limit_wall_s, prefix, f):
    """Revision 5: a supervisor ABORT lands through P2's nominal sequence. Exactly one accepted LAND,
    `COMPLETE` (landed and disarmed) within `limit_wall_s` of it, and no P2 failure recorded."""
    lands = [t for t in result.get("transactions", []) if t["command"] == 21]
    complete = next((t["monotonic_s"] for t in result.get("transitions", []) if t["to"] == "COMPLETE"), None)
    f.check(len(lands) == 1 and lands[0]["status"] == "accepted", prefix + "single_accepted_land")
    f.check(result.get("final_state") == "COMPLETE" and not result.get("failure"), prefix + "abort_not_landed_through_complete")
    seconds = None if not lands or complete is None else complete - lands[0]["sent_monotonic_s"]
    f.check(seconds is not None and 0 <= seconds <= limit_wall_s, prefix + "abort_landing_wall_limit")
    return seconds


# ---- scenario outcome --------------------------------------------------------------------------------
def outcome(run, summary, declaration, events, offset, track, v, f):
    result = summary["controller_result"]
    p4 = result.get("p4", {})
    transitions = [t["to"] for t in p4.get("supervisor_transitions", [])]
    scenario, expected = declaration["scenario"], declaration["outcome"]
    out = {"scenario": scenario, "expected": expected, "supervisor_transitions": transitions,
           "controller_outcome": result.get("p4_outcome"), "abort_reason": p4.get("abort_reason")}
    origin = offset[0] + p4["origin_enu"][0], offset[1] + p4["origin_enu"][1]
    goal = origin[0] + 8.0, origin[1]
    final = track[-1][1] if track else None
    if expected.startswith("goal_reached"):
        out["final_goal_distance_m"] = None if final is None else math.dist(final[:2], goal)
        f.check(result.get("p4_outcome") == "completed" and result.get("final_state") == "COMPLETE", "outcome:not_completed")
        f.check(out["final_goal_distance_m"] is not None and out["final_goal_distance_m"] <= v["goal_final_distance_m"],
                "outcome:goal_distance")
        if expected == "goal_reached":
            lateral = max(abs(p[1] - origin[1]) for _, p in track)
            out["max_lateral_m"] = lateral
            f.check(not transitions, "outcome:clear_path_avoided_or_hovered")
            f.check(lateral <= v["clear_path_max_lateral_m"], "outcome:clear_path_lateral")
        else:
            f.check(transitions[:2] == ["OBSTACLE_DETECTED", "AVOIDING"] and "RESUME" in transitions
                    and transitions[-1] == "NORMAL" and "HOVER" not in transitions, "outcome:detour_sequence")
            if declaration.get("detour_side") == "negative_y":
                pass_y = [p[1] - origin[1] for _, p in track if abs(p[0] - origin[0] - 4.0) <= 0.3 and p[2] > 1.0]
                out["pass_lateral_m"] = pass_y
                f.check(pass_y and max(pass_y) < 0, "outcome:detour_side")
        out["label"] = expected if not f.items else "failed"
    elif expected == "safe_refusal":
        f.check(result.get("p4_outcome") == "supervisor_abort" and p4.get("abort_reason") == "hover_timeout",
                "outcome:not_a_hover_timeout_abort")
        f.check("HOVER" in transitions, "outcome:no_hover")
        decisions = [e for e in events if e.get("event") == "p4_decision"]
        hover_at = next((t["sim_s"] for t in p4.get("supervisor_transitions", []) if t["to"] == "HOVER"), None)
        after = [d["target_enu"] for d in decisions if hover_at is not None and d["sim_s"] >= hover_at]
        out["target_motion_after_hover_m"] = max((math.dist(after[0], a) for a in after), default=None)
        f.check(after and out["target_motion_after_hover_m"] <= v["fault_target_motion_m"], "outcome:progress_after_hover")
        out["abort_landing_wall_s"] = abort_landing(result, summary["config"]["recovery_wall_s"], "outcome:", f)
        out["label"] = "safe_refusal" if not f.items else "failed"  # never goal_reached
    return out


# ---- §9.1 S6 fault window -------------------------------------------------------------------------------
def fault_window(run, summary, validation_p3, sensors, v, f):
    run = Path(run)
    injection_path = run / "p4/fault_injection.json"
    f.check(injection_path.exists(), "s6:injection_record_missing")
    if not injection_path.exists():
        return None
    injection = strict_json(injection_path)
    t_inj = injection["t_inj_sim_s"]
    window = summary["sensor_window_sim_s"]
    pre = [window[0], t_inj - v["fault_pre_window_trim_sim_s"]]
    out = {"t_inj_sim_s": t_inj, "pre_window_sim_s": pre, "injection": injection}
    f.check(injection["process"] == "sensor-bridge" and injection["signal"] == "SIGINT", "s6:wrong_injection")
    f.check(pre[1] - pre[0] >= v["fault_pre_window_min_sim_s"], "s6:pre_window_too_short")
    stream = evaluate_stream(run, pre, validation_p3, sensors, require_association=True)
    out["pre_window_stream"] = {"status": stream["status"], "failures": stream["failures"]}
    f.items += ["s6:pre:" + x for x in stream["failures"]]
    result = summary["controller_result"]
    transitions = result.get("p4", {}).get("supervisor_transitions", [])
    abort = next((t for t in transitions if t["to"] == "ABORT"), None)
    hover = next((t for t in transitions if t["to"] == "HOVER" and str(t["reason"]).startswith("stale_inputs")), None)
    # F1: the native source continued; ROS delivery stopped; only the bridge stopped, gracefully.
    headers = [r["stamp_ns"] / 1e9 for r in strict_jsonl(run / "source/depth-headers.jsonl")]
    after = [s for s in headers if s >= t_inj and (abort is None or s <= abort["sim_s"])]
    native_gap = max((b - a for a, b in zip(after, after[1:])), default=None)
    reaches_abort = bool(after) and (abort is None or after[-1] >= abort["sim_s"] - validation_p3["source_gap_sim_s"])
    out["native_headers_after_injection"] = len(after)
    f.check(after and native_gap is not None and native_gap <= validation_p3["source_gap_sim_s"] and reaches_abort,
            "s6:f1_native_source_stopped")
    index = strict_jsonl(run / "sensors/depth-index.jsonl")
    late = [r for r in index if r["acquisition_stamp_ns"] / 1e9 > t_inj + v["fault_pre_window_trim_sim_s"]]
    f.check(not late, "s6:f1_ros_delivery_continued")
    bridge = next(p for p in summary["processes"] if p["name"] == "sensor-bridge")
    f.check(bridge["exit_code"] == 0 and bridge["stop_requested_monotonic_s"] == injection["request_monotonic_s"],
            "s6:f1_bridge_exit")
    # F2: adapter health left fresh within the P3 detection limit; no observation afterwards.
    health = strict_jsonl(run / "sensors/health.jsonl")
    left = next((h for h in health if h["wall_s"] >= injection["request_monotonic_s"] and h["status"] != "fresh"), None)
    f.check(left is not None and left["wall_s"] - injection["request_monotonic_s"] <= validation_p3["interruption_detect_wall_s"],
            "s6:f2_detection")
    observations = strict_jsonl(run / "sensors/observations.jsonl")
    f.check(left is not None and not [o for o in observations if o["processing_wall_s"] > left["wall_s"]],
            "s6:f2_observation_after_detection")
    adapter = strict_json(run / "sensors/adapter-result.json")
    f.check(adapter["recorder"]["complete"] is True and adapter["recorder"]["overflow"] == 0, "s6:f2_recorder")
    # F3: HOVER (yaw frozen) promptly, no motion while stale, ABORT at 10 s, one accepted LAND, landed.
    snapshots = strict_jsonl(run / "p4/local_map.jsonl")
    t_last = max((s["acquisition_sim_s"] for s in snapshots if s.get("usable")), default=None)
    out.update(t_last_usable_sim_s=t_last, hover_sim_s=hover and hover["sim_s"], abort_sim_s=abort and abort["sim_s"])
    f.check(hover is not None and t_last is not None and hover["sim_s"] <= t_last + v["fault_hover_after_last_usable_sim_s"],
            "s6:f3_late_hover")
    events = [json.loads(line) for line in (run / "ros-events.jsonl").read_text().splitlines()]
    if hover and abort:
        decisions = [e for e in events if e.get("event") == "p4_decision" and hover["sim_s"] <= e["sim_s"] <= abort["sim_s"]]
        setpoints = [e for e in events if e.get("event") == "setpoint" and hover["sim_s"] <= e["ros_sim_s"] <= abort["sim_s"]]
        motion = max((math.dist(decisions[0]["target_enu"], d["target_enu"]) for d in decisions), default=None)
        yaws = [s["fields"]["yaw"] for s in setpoints if s["fields"].get("yaw") is not None]
        yaw_change = max((abs(math.remainder(y - yaws[0], 2 * math.pi)) for y in yaws), default=None)
        out.update(target_motion_m=motion, yaw_change_deg=None if yaw_change is None else math.degrees(yaw_change))
        f.check(motion is not None and motion <= v["fault_target_motion_m"], "s6:f3_motion_while_stale")
        f.check(yaw_change is not None and math.degrees(yaw_change) <= v["fault_target_yaw_deg"], "s6:f3_yaw_while_stale")
        f.check(abs(abort["sim_s"] - hover["sim_s"] - v["fault_abort_after_stale_sim_s"]) <= v["fault_abort_tolerance_sim_s"],
                "s6:f3_abort_timing")
    else:
        f.check(False, "s6:f3_hover_or_abort_missing")
    out["abort_landing_wall_s"] = abort_landing(result, summary["config"]["recovery_wall_s"], "s6:f3_", f)
    f.check(result.get("p4_outcome") == "supervisor_abort", "s6:f3_not_supervisor_abort")
    out["label"] = "safe_stop_on_sensor_loss"
    return out


# ---- ground fixtures and faults -----------------------------------------------------------------------
def fixture(run, declaration, summary, config, v, f):
    from gwm_obstacle_avoidance import local_map, planner
    rows = [r for r in strict_jsonl(Path(run) / "p4/local_map.jsonl") if r.get("usable")]
    window = summary["sensor_window_sim_s"]
    inside = [r for r in rows if window[0] <= r["processing_sim_s"] <= window[1]]
    f.check(bool(inside), "fixture:no_usable_snapshot_in_window")
    if not inside:
        return None
    last = inside[-1]
    grid = local_map.Grid.decode(last["grid"])
    n, e, _ = last["position_ned"]
    vehicle = (e, n)
    q = last["q_wxyz"]
    yaw_ned = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
    heading = math.pi / 2 - yaw_ned
    track = truth_track(run)
    truth_xy = track[-1][1][:2] if track else None
    f.check(truth_xy is not None, "fixture:truth_missing")
    if truth_xy is None:
        return None
    offset = (truth_xy[0] - vehicle[0], truth_xy[1] - vehicle[1])
    nearest_truth = min(box["center_m"][0] - box["size_m"][0] / 2 - truth_xy[0] for box in declaration["boxes"])
    ahead = [r for r in last["scan"] if r[1] == "hit" and abs(math.remainder(r[0] - heading, 2 * math.pi)) < math.radians(5)]
    measured = min((r[2] for r in ahead), default=None)
    out = {"nearest_surface_truth_m": nearest_truth, "nearest_hit_ahead_m": measured, "local_to_world_offset_m": offset}
    f.check(measured is not None and abs(measured - nearest_truth) <= v["reconstruction_tolerance_m"],
            "fixture:reconstruction")
    goal = (vehicle[0] + 8.0 * math.cos(heading), vehicle[1] + 8.0 * math.sin(heading))
    traversed = planner.traversed_mask(grid, [vehicle], config["envelope"]["r_body_m"])
    ctx = planner.Context(vehicle, vehicle, heading, goal, vehicle, 0.0, config["geofence"])
    status = planner.path_status(grid, traversed, ctx, config["planner"], config["perception"]["snapshot_max_age_sim_s"])
    decision = "obstacle_in_trigger_range" if status["occupied_in_corridor"] else "no_obstacle_in_trigger_range"
    out.update(trigger_decision=decision, expected=declaration["expected_trigger"])
    f.check(decision == declaration["expected_trigger"], "fixture:trigger_decision")
    return out


def fault_replay(run, summary, config, f):
    """Gate A faults: the production supervisor's stale rule replayed on the recorded ledgers must
    reach HOVER (yaw frozen) after the injected stop."""
    from gwm_obstacle_avoidance import supervisor
    run = Path(run)
    injection = strict_json(run / "p4/fault_injection.json")
    snapshots = [r for r in strict_jsonl(run / "p4/local_map.jsonl")]
    health = strict_jsonl(run / "sensors/health.jsonl")
    sup = supervisor.Supervisor(config, injection["t_inj_sim_s"])
    t, end = injection["t_inj_sim_s"], max([r["processing_sim_s"] for r in snapshots] + [h["sim_s"] or 0 for h in health])
    hover = None
    while t <= end + 3.0:
        usable = [s for s in snapshots if s.get("usable") and s["processing_sim_s"] <= t]
        record = [h for h in health if h["sim_s"] is not None and h["sim_s"] <= t]
        reason = sup.stale_reason({"sim": t, "snapshot": usable[-1] if usable else None,
                                   "adapter_health": record[-1] if record else None, "consecutive_unusable": 0,
                                   "reset_sim": None})
        if reason:
            hover = {"sim_s": t, "reason": reason}
            break
        t += 0.05
    out = {"injection": injection, "replayed_hover": hover}
    f.check(hover is not None, "fault:supervisor_would_not_hover")
    return out


# ---- entry ----------------------------------------------------------------------------------------------
def evaluate(run):
    import p3_evaluate
    run = Path(run)
    summary = strict_json(run / "summary.json")
    scope = summary.get("p4_scope")
    if scope == "p3_readiness":
        out = p3_evaluate.evaluate(run)
        out.update(contract="p4-offline-evaluation-v1", p4_scope=scope, p3_evaluation_contract="p3-offline-evaluation-v1")
        return out
    validation_p3, sensors = p3_evaluate.configs()
    v4 = strict_json(P4 / "configs/p4_validation.yaml")
    config = json.loads((run / "p4_avoidance.yaml").read_text())
    f = Failures()
    flight = scope == "flight"
    out = {"contract": "p4-offline-evaluation-v1", "run_id": run.name, "p4_scope": scope, "gate": summary.get("gate"),
           "frozen_manifest_sha256": summary.get("frozen_manifest_sha256"), "p3_baseline": summary.get("p3_baseline"),
           "evaluator_sha256": evaluator_identity(), "runtime_status": summary.get("status"),
           "runtime_failure": summary.get("failure")}
    f.check(summary.get("status") == "passed", "runtime_status:" + str(summary.get("failure")))
    p3_evaluate.common(run, summary, f, out)
    p3_evaluate.mandatory_artifacts(run, f, ("controller-result.json", "ros-events.jsonl", "rosbag/metadata.yaml",
                                             "mavlink/closed.json", "source/probe-closed.json", "sensors/adapter-result.json",
                                             "p4/local_map.jsonl", "p4/perception-result.json"))
    f.check(summary.get("p3_baseline", {}).get("sha256") == v4["p3_baseline_manifest_sha256"], "p3_baseline_mismatch")
    declaration = strict_json(run / "p4/scenario.json") if (run / "p4/scenario.json").exists() else None
    is_s6 = bool(declaration and declaration.get("fault") == "bridge_stop_at_distance")
    f.check((run / "p4/fault_injection.json").exists() == (is_s6 or scope == "fault"), "fault_injection_outside_s6_or_fault_scope")
    window = summary["sensor_window_sim_s"]
    try:
        if is_s6:
            out["fault_window"] = fault_window(run, summary, validation_p3, sensors, v4["outcomes"], f)
        else:
            stream = evaluate_stream(run, window, validation_p3, sensors, require_association=True)
            out["depth_stream"] = {"status": stream["status"], "failures": stream["failures"],
                                   "delivery": stream.get("delivery"), "processing": stream.get("processing")}
            f.items += ["depth:" + x for x in stream["failures"]]
        perf_window = [window[0], out["fault_window"]["pre_window_sim_s"][1]] if is_s6 and out.get("fault_window") else window
        out["performance"] = performance(run, perf_window, v4["performance"], f)
        if flight:
            transitions = {t["to"]: t["monotonic_s"] for t in summary["controller_result"]["transitions"]}
            end = transitions.get("COMPLETE") or transitions.get("RECOVERY")
            out["rgb_workload"] = "depth_only_flight"
            out["rgb_absence"] = evaluate_rgb_absence(run, (transitions["PRESTREAM_SAFE_SETPOINTS"], end),
                                                      validation_p3["rgb_absence_monitor"],
                                                      [p["pid"] for p in summary["processes"] if p.get("pid")])
            f.items += ["rgb_absence:" + x for x in out["rgb_absence"]["failures"]]
        else:
            out["rgb_stream"] = evaluate_rgb(run, window, validation_p3, sensors["rgb_sensor"])
            f.items += ["rgb:" + x for x in out["rgb_stream"]["failures"]]
    except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
        f.check(False, "sensor_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
    p2_report, log = None, None
    try:
        p2_report, log, _, _ = p2_integrity(run, summary, f, flight)
        out["p2_integrity"] = {k: v for k, v in p2_report.items() if k != "timing"}
        out["p2_timing_status"] = p2_report["timing"].get("status")
    except (ValueError, KeyError, FileNotFoundError, StopIteration, IndexError) as exc:
        f.check(False, "p2_integrity_error:" + type(exc).__name__ + ":" + str(exc))
    try:
        native_commands(run, summary, f, out)
    except (ValueError, KeyError, FileNotFoundError) as exc:
        f.check(False, "native_command_evaluation_error:" + str(exc))
    mc(run, summary, f, out, flight, p2_report)
    out["rtf"] = native_rtf(run)
    renderer = summary.get("renderer") or {}
    identity = strict_json(SIM / "configs/p3_d3d12_profile.json")["renderer_identity"]
    f.check(renderer.get("identified") is True and renderer.get("software") is False
            and identity in renderer.get("strings", []), "renderer_not_declared_d3d12")
    f.check(summary.get("controller_publishers_only_from_controller") is True, "competing_flight_publisher")
    result = summary.get("controller_result") or {}
    f.check(result.get("controller") == "p4_control", "controller_not_p4")
    try:
        if flight:
            safety_out, offset, events = safety(run, declaration, v4["safety"], config["envelope"], f)
            out["safety"] = safety_out
            out["response_latency_sim_s"] = response_latency(events, v4["safety"]["response_latency_sim_s"], f)
            if not is_s6 and offset is not None:
                out["outcome"] = outcome(run, summary, declaration, events, offset, truth_track(run), v4["outcomes"], f)
            elif is_s6:
                out["outcome"] = {"label": "safe_stop_on_sensor_loss" if not f.items else "failed", "scenario": "S6"}
        elif scope == "fixture":
            out["fixture"] = fixture(run, declaration, summary, config, v4["ground_fixture"], f)
        elif scope == "fault":
            out["fault"] = fault_replay(run, summary, config, f)
    except (ValueError, KeyError, FileNotFoundError, StopIteration) as exc:
        f.check(False, "p4_evaluation_error:" + type(exc).__name__ + ":" + str(exc))
    out["failures"] = f.items
    out["status"] = "passed" if not f.items else "failed"
    out["acceptance_credit"] = out["status"] == "passed"
    return out


def native_commands(run, summary, f, out):
    """Policy A unchanged; the landing window starts at the first controller LAND (normal or ABORT)."""
    from p3_mavlink import evaluate_policy
    closed = strict_json(Path(run) / "mavlink/closed.json")
    effective = summary["native_command_policy"]
    startup_end = summary["startup"]["origin_monotonic_ns"] + int(effective["policy"].get("startup_wall_s", 0) * 1e9)
    result = summary["controller_result"]
    lands = [t for t in result.get("transactions", []) if t["command"] == 21]
    ends = [t["monotonic_s"] for t in result.get("transitions", []) if t["to"] == "COMPLETE"]
    end = ends[0] if ends else (summary.get("final_console_state") or {}).get("end_wall_s")
    landing = (int(lands[0]["sent_monotonic_s"] * 1e9), int(end * 1e9)) if lands and end else None
    out["native_commands"] = evaluate_policy(closed, effective, startup_end, landing)
    f.items += ["native_commands:" + x for x in out["native_commands"]["failures"]]


def mc(run, summary, f, out, flight, p2_report):
    import p3_evaluate
    if flight and p2_report is None:
        f.check(False, "mc_causal_yaw:p2_integrity_missing")
        return
    p3_evaluate.mc_component(run, summary, f, out, flight, p2_report)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args(argv)
    output = args.run / "p4-evaluation.json"
    if output.exists():
        raise SystemExit("Evaluation already exists; the first evaluation is retained")
    try:
        result = evaluate(args.run)
    except Exception as exc:  # incomplete evidence is recorded as failed, never missing
        summary = {}
        try:
            summary = strict_json(args.run / "summary.json")
        except Exception:
            pass
        result = {"contract": "p4-offline-evaluation-v1", "run_id": args.run.name, "gate": summary.get("gate"),
                  "runtime_status": summary.get("status"), "runtime_failure": summary.get("failure"),
                  "failures": ["runtime_status:" + str(summary.get("failure")),
                               "evaluator_incomplete_evidence:" + type(exc).__name__ + ":" + str(exc)],
                  "status": "failed", "acceptance_credit": False}
    atomic_json(output, result, exclusive=True)
    print(result["status"], len(result["failures"]), "failures", flush=True)
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
