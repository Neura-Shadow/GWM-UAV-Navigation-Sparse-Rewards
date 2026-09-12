"""Offline bag/ULog correlation. Never creates ROS nodes or replays messages."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
import re
from pathlib import Path
import sqlite3
import sys


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def asof(rows, timestamp, freshness):
    index = bisect_right([r["timestamp"] for r in rows], timestamp)-1
    require(index >= 0, "Missing earlier state")
    row = rows[index]
    require(0 <= timestamp-row["timestamp"] <= freshness*1e6, "Offline state gap")
    return row


def yaw(q):
    require(len(q) == 4 and all(math.isfinite(x) for x in q), "Invalid quaternion")
    w, x, y, z = q
    require(abs(sum(x*x for x in q)-1) < 0.1, "Non-unit quaternion")
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def joined_samples(streams, config, start, end):
    output = []
    for p in streams["vehicle_local_position"]:
        if not start <= p["timestamp"]/1e6 <= end:
            continue
        t = p["timestamp"]
        a = asof(streams["vehicle_attitude"], t, config["attitude_fresh_sim_s"])
        s = asof(streams["vehicle_status"], t, config["flags_fresh_sim_s"])
        land = asof(streams["vehicle_land_detected"], t, config["flags_fresh_sim_s"])
        require(all(p[k] for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")), "Invalid recorded estimate")
        output.append({"t": t/1e6, "position": [float(p[k]) for k in ("x", "y", "z")],
                       "yaw": yaw(a["q"]), "nav_state": int(s["nav_state"]),
                       "arming_state": int(s["arming_state"]), "landed": bool(land["landed"])})
    return output


def read_bag(run):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    contract = json.loads((run/"topic-contract.json").read_text())
    names = {v["topic"]: k for k, v in contract.items()}
    streams, counts, events, clocks = defaultdict(list), Counter(), [], []
    for database in sorted((run/"rosbag").glob("*.db3")):
        connection = sqlite3.connect(database.as_uri()+"?mode=ro", uri=True)
        require(connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok", "Bag SQLite corruption")
        meta = {row[0]: (row[1], get_message(row[2])) for row in connection.execute("SELECT id,name,type FROM topics")}
        for record_id, topic_id, stamp, blob in connection.execute("SELECT id,topic_id,timestamp,data FROM messages ORDER BY id"):
            name, cls = meta[topic_id]
            message = deserialize_message(blob, cls)
            counts[name] += 1
            if name in names:
                fields = dict(message_to_ordereddict(message))
                fields["bag_sim_ns"] = stamp
                fields["_record_id"] = f"{run.name}:bag:{database.name}:{record_id}"
                fields["_run_id"] = run.name
                fields["_instance"] = 0
                fields["_message_version"] = contract[names[name]]["version"]
                fields["_raw_sha256"] = hashlib.sha256(blob).hexdigest()
                streams[names[name]].append(fields)
            elif name == "/gwm/p2/events":
                events.append(json.loads(message.data))
            elif name == "/clock":
                clocks.append(message.clock.sec+message.clock.nanosec/1e9)
        connection.close()
    require(clocks and all(b >= a for a, b in zip(clocks, clocks[1:])), "Missing/backwards bag clock")
    return streams, counts, events


def read_ulog(path, flight=True, extended=False, run_id=None):
    from pyulog import ULog
    log = ULog(str(path))
    streams = {}
    names = ("vehicle_local_position", "vehicle_attitude", "vehicle_status", "vehicle_land_detected")
    if extended:
        names += ("estimator_status_flags", "failsafe_flags")
    if flight:
        names += ("vehicle_command", "vehicle_command_ack", "trajectory_setpoint", "offboard_control_mode")
    for name in names:
        if name not in {d.name for d in log.data_list} and name in (
                "vehicle_command", "vehicle_command_ack", "trajectory_setpoint", "offboard_control_mode"):
            streams[name] = []  # a ground abort can precede the first command
            continue
        dataset = log.get_dataset(name)
        require({d.multi_id for d in log.data_list if d.name == name} == {0}, "Unsupported ULog instance: "+name)
        data = dataset.data
        rows = []
        for i in range(len(data["timestamp"])):
            row = {key: value[i].item() for key, value in data.items()}
            array_fields = defaultdict(dict)
            for key, value in row.items():
                match = re.fullmatch(r'(.+)\[(\d+)\]', key)
                if match:
                    array_fields[match[1]][int(match[2])] = value
            for field, values in array_fields.items():
                require(sorted(values) == list(range(len(values))), 'Incomplete ULog array:'+field)
                row[field] = [values[index] for index in range(len(values))]
            row["_record_id"] = f"{run_id or Path(path).parent.name}:ulog:{Path(path).name}:{name}:{dataset.multi_id}:{i+1}"
            row["_native_dataset_ordinal"] = i+1
            row['_ulog_msg_id'] = dataset.msg_id
            row["_instance"] = dataset.multi_id
            if run_id is not None:
                row['_run_id'] = run_id
            rows.append(row)
        streams[name] = rows
    return log, streams


def wire_checks(streams, config, reference=None):
    targets, modes = streams["trajectory_setpoint"], streams["offboard_control_mode"]
    v3 = config.get("reference_policy") == "p2-estimator-reference-v3"
    revised = config.get("reference_policy") in ("p2-estimator-reference-v2", "p2-estimator-reference-v3")
    require(len(targets) > 2 and len(modes) >= len(targets), "Missing wire stream")
    if not revised or v3:
        require(len(targets) == len(modes), "Strict-v1 heartbeat/target count differs")
    for target in targets:
        unspecified = v3 and math.isnan(target["yaw"])
        require(all(math.isfinite(v) for v in target["position"]) and (unspecified or math.isfinite(target["yaw"])), "Invalid active target")
        require(all(math.isnan(v) for key in ("velocity", "acceleration", "jerk") for v in target[key])
                and (target["yawspeed"] == 0.0 if unspecified else math.isnan(target["yawspeed"])), "Inactive wire fields not NaN or wrong initialization rate")
    for mode in modes:
        require(mode["position"] is True and all(mode[k] is False for k in
                ("velocity", "acceleration", "attitude", "body_rate", "thrust_and_torque", "direct_actuator")), "Wrong Offboard level")
    speed, yaw_rate, raw_yaw_rate, gaps = [], [], [], []
    accepted = (reference or {}).get("accepted", [])
    for a, b in zip(targets, targets[1:]):
        dt = (b["timestamp"]-a["timestamp"])/1e6
        require(dt > 0, "Repeated/backwards transmitted stamp")
        gaps.append(dt)
        speed.append(math.dist(a["position"], b["position"])/dt)
        if v3:
            require(dt <= config["max_sample_gap_sim_s"], "Trajectory stream gap")
            if math.isnan(a["yaw"]) or math.isnan(b["yaw"]):
                require(math.isnan(a["yaw"]), "Finite yaw reverted to unspecified")
                continue
        dyaw = b["yaw"]-a["yaw"]
        raw_yaw_rate.append(abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw))))/dt)
        corrections = [] if v3 else [e for e in accepted if a["timestamp"]/1e6 < e["accepted_sim_s"] <= b["timestamp"]/1e6+1e-6]
        dyaw -= sum(e["delta_heading"] for e in corrections)
        yaw_rate.append(abs(math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw))))/dt)
        if dt > config["max_sample_gap_sim_s"]:
            require(revised and len(corrections) == 1 and dt <= config["reference_settings"]["pair_sim_s"]+.1
                    and a["timestamp"]/1e6 <= corrections[0]["start_sim_s"], "Unexplained trajectory stream gap")
    # Float32 serialization contributes sub-micrometre rounding only.
    require(max(speed) <= config["ramp_m_s"]+1e-5 and max(yaw_rate,default=0) <= config["yaw_ramp_deg_s"]+1e-3, "Wire ramp exceeded")
    heartbeat_gaps = [(b["timestamp"]-a["timestamp"])/1e6 for a,b in zip(modes,modes[1:])]
    require(min(heartbeat_gaps)>0 and max(heartbeat_gaps) <= config["max_sample_gap_sim_s"], "Wire heartbeat gap")
    return {"samples": len(targets), "rate_per_sim_s": 1/(sum(gaps)/len(gaps)),
            "max_gap_sim_s": max(gaps), "max_position_ramp_m_s": max(speed),
            "max_yaw_ramp_deg_s": max(yaw_rate,default=0), "raw_max_yaw_wire_rate_deg_s": max(raw_yaw_rate,default=0),
            "heartbeat_rate_per_sim_s": 1/(sum(heartbeat_gaps)/len(heartbeat_gaps)),
            "max_heartbeat_gap_sim_s": max(heartbeat_gaps), "inactive_fields_nan": True}


def verify(run, sample_contract=None):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"ros2_ws/src/gwm_px4_control"))
    from gwm_px4_control.acceptance import evaluate_flight, evaluate_window, evaluate_window_v2, fixed_window_us, expected_fixture, PHASES
    summary = json.loads((run/"summary.json").read_text())
    result, config = summary["controller_result"], summary["config"]
    historical = sample_contract is not None
    if historical:
        require(sample_contract == "p3-sample-evidence-v2", "Unsupported historical sample contract")
        config = {**config, "sample_evidence_contract": sample_contract}
    revised = config.get("sample_evidence_contract") == "p3-sample-evidence-v2"
    if revised and not historical:
        from p3_provenance import require_finalized, frozen_inputs
        require_finalized(run, frozen_inputs(Path(__file__).resolve().parents[1]))
    require(result is not None, "Controller never initialized complete recording")
    require(len(summary["ulog_files"]) == 1, "Expected one continuous ULog")
    artifacts = summary["ulog_files"]+summary["rosbag_files"]
    for entry in artifacts:
        require(digest(entry["path"]) == entry["sha256"], "Artifact hash changed: "+entry["path"])
    streams, counts, events = read_bag(run)
    if revised:
        from p3_provenance import strict_jsonl
        json_events = strict_jsonl(run/"ros-events.jsonl")
    else:
        json_events = [json.loads(line) for line in (run/"ros-events.jsonl").read_text().splitlines()]
    ledger_only = {"received", "source_delivery", "source_delivery_rejected"} if revised else {"received"}
    require(events == [r for r in json_events if r["event"] not in ledger_only], "Bag/event ledger differs")
    for key in ("vehicle_local_position", "vehicle_status", "vehicle_attitude", "vehicle_land_detected",
                "estimator_status_flags", "failsafe_flags"):
        require(streams[key], "Discovered but no recorded state: "+key)
        require(len(streams[key]) == sum(r.get("event") == "received" and r.get("topic_key") == key for r in json_events), "Bag/received ledger count mismatch")
        if revised:
            from gwm_px4_control.contracts import json_message
            from sample_evidence import payload
            received = [r for r in json_events if r.get("event") == "received" and r.get("topic_key") == key]
            for bag_row, callback in zip(streams[key], received):
                decoded = json_message(payload(bag_row))
                require(decoded["fields"] == callback["fields"] and sorted(decoded["nonfinite_fields"]) == sorted(callback["nonfinite_fields"]),
                        "Raw callback/bag payload mismatch:"+key)
    log, ulog = (read_ulog(summary["ulog_files"][0]["path"], summary["kind"] == "flight", True, summary['run_id']) if revised else
                 read_ulog(summary["ulog_files"][0]["path"], summary["kind"] == "flight"))
    require(not log.dropouts, "ULog dropout")
    report = {"run_id": summary["run_id"], "recording_integrity": "passed", "flight_acceptance": "not_run",
              "artifacts": artifacts, "bag_message_counts": dict(counts), "ulog_dropouts": len(log.dropouts),
              "clock_mapping": "PX4 microseconds and Gazebo/ROS seconds share simulation boot epoch; no wall-epoch subtraction",
              "raw_jsonl_sha256": digest(run/"ros-events.jsonl"), "bag_event_ledger_equal": True}
    if revised:
        from sample_evidence import (validate_stream, exact_overlap, joined_samples_v2, verify_source_deliveries,
                                     all_record_window_checks, reconstruct_controller_samples)
        report.update(sample_evidence_contract=config["sample_evidence_contract"],
                      frozen_inputs=summary.get("frozen_inputs"), historical_reanalysis=historical,
                      qualification_credit=False if historical else None,
                      sample_evaluator_sha256=digest(Path(__file__).with_name("sample_evidence.py")),
                      acceptance_evaluator_sha256=digest(Path(__file__).resolve().parents[1]/"ros2_ws/src/gwm_px4_control/gwm_px4_control/acceptance.py"))
        report["native_record_classification"] = {
            label: {topic: validate_stream(data[topic], topic, summary["run_id"])["statistics"]
                    for topic in ("vehicle_local_position", "vehicle_attitude", "vehicle_status", "vehicle_land_detected",
                                  "estimator_status_flags", "failsafe_flags")}
            for label, data in (("rosbag", streams), ("ulog", ulog))}
        if not historical:
            require(result.get('run_id') == run.name and result.get('sample_evidence_contract') == config['sample_evidence_contract'], 'Controller result identity')
            report['source_delivery_verification'] = verify_source_deliveries(streams,json_events,run.name)
            reconstructed = reconstruct_controller_samples(json_events,config,run.name)
            report['controller_source_reconstruction'] = reconstructed['statistics']
            require(reconstructed['statistics']['controller_evaluations'] == result['source_identity_statistics']['evaluations'], 'Evaluation result count mismatch')
            require(not reconstructed['statistics']['evaluation_errors'], 'Recorded control evaluation error')
    if result.get("evidence_schema") == 2:
        from timing_evidence import verify_timing
        report["timing"] = verify_timing(run, result, streams, events, config)
        report["timing_evaluator_sha256"] = digest(Path(__file__).with_name("timing_evidence.py"))
    if summary["kind"] != "flight":
        require(not streams["vehicle_command"] and not streams["trajectory_setpoint"], "Read-only stage published flight input")
        report["connectivity"] = result["connectivity"]
        return report
    report["reference_policy"] = config.get("reference_policy", "p2-estimator-reference-v1")
    sent = streams["vehicle_command"]
    own = [r for r in ulog["vehicle_command"] if r["source_system"] == config["source_system"]
           and r["source_component"] == config["source_component"]]
    require([r["command"] for r in own] == [r["command"] for r in sent], "ULog/ROS command sequence differs")
    for a, b in zip(own, sent):
        require(a["timestamp"] == b["timestamp"] and a["target_system"] == 72 and a["target_component"] == 1,
                "Command identity/time mapping mismatch")
    report["command_ids"] = [r["command"] for r in sent]
    report["command_timestamp_exact_ulog_match"] = True
    report["acks"] = []
    if not sent:
        aborts = [r for r in events if r["event"] == "abort"]
        require(aborts and result["final_state"] != "COMPLETE", "Missing nominal commands")
        require(all(s["arming_state"] == 1 for s in ulog["vehicle_status"])
                and all(s["landed"] for s in ulog["vehicle_land_detected"]), "Uncommanded airborne/armed state")
        require(all(t["timestamp"]/1e6 <= aborts[0]["sim_s"] for t in streams["trajectory_setpoint"]),
                "Old targets continued after ground abort")
        report.update(flight_acceptance="failed", controller_failure=aborts[0]["reason"],
                      abort_sim_s=aborts[0]["sim_s"], final_disarmed=True, final_landed=True,
                      observed_recovery="remained_grounded", setpoints_stopped_at_abort=True,
                      wire={"status": "not_completed", "samples": len(streams["trajectory_setpoint"])})
        return report
    report["wire"] = wire_checks(streams, config, result.get("reference"))
    for command in sent:
        matches = [ack for ack in streams["vehicle_command_ack"] if ack["command"] == command["command"]
                   and ack["target_system"] == 201 and ack["target_component"] == 191
                   and ack["from_external"] is False and 0 <= (ack["timestamp"]-command["timestamp"])/1e6 <= config["ack_deadline_sim_s"]]
        require(any(a["result"] == 0 for a in matches), "Missing accepted routed ACK")
        accepted = next(a for a in matches if a["result"] == 0)
        require(any(a["command"] == accepted["command"] and a["timestamp"] == accepted["timestamp"]
                    and a["target_system"] == 201 and a["result"] == 0 for a in ulog["vehicle_command_ack"]), "ACK absent from ULog")
        report["acks"].append({"command": command["command"], "latency_sim_s": (accepted["timestamp"]-command["timestamp"])/1e6,
                                "target_system": 201, "target_component": 191, "ulog_match": True})
    first = sent[0]["timestamp"]
    prest = [r for r in streams["trajectory_setpoint"] if r["timestamp"] <= first]
    require((first-prest[0]["timestamp"])/1e6 >= config["prestream_sim_s"], "Short actual prestream")
    require(all(math.dist(r["position"], result["origin_ned"]) < 1e-5 for r in prest), "Unsafe ground prestream")
    report["prestream_sim_s"] = (first-prest[0]["timestamp"])/1e6
    status, land = ulog["vehicle_status"], ulog["vehicle_land_detected"]
    require(any(s["arming_state"] == 2 for s in status), "No armed ULog state")
    require(any(not s["landed"] for s in land), "No airborne ULog state")
    report["final_disarmed"] = status[-1]["arming_state"] == 1
    report["final_landed"] = bool(land[-1]["landed"])
    report["failsafe_observed"] = any(s["failsafe"] for s in status)
    report["observed_nav_states"] = sorted({s["nav_state"] for s in status})
    # Exact timestamp/float comparison proves shared PX4 epoch independently of controller receipts.
    if revised:
        report["estimator_correlation"] = {topic: exact_overlap(streams[topic], ulog[topic], topic, summary["run_id"])
                                            for topic in ("vehicle_local_position", "vehicle_attitude")}
        require(report["estimator_correlation"]["vehicle_local_position"]["exact_payload_matches"] >= 100,
                "Insufficient overlapping ROS/ULog positions")
        require(report['estimator_correlation']['vehicle_attitude']['exact_payload_matches'] > 0, 'No exact ROS/ULog attitude overlap')
    else:
        # Retained strict evaluator path: lossless grouping rejects unsupported
        # ambiguity instead of overwriting a timestamp collision.
        grouped = defaultdict(list)
        for p in ulog["vehicle_local_position"]:
            grouped[p["timestamp"]].append(p)
        matching = [p for p in streams["vehicle_local_position"] if p["timestamp"] in grouped]
        require(len(matching) >= 100, "Insufficient overlapping ROS/ULog positions")
        require(all(len(grouped[p["timestamp"]]) == 1 for p in matching), "Ambiguous legacy timestamp overlap")
        max_delta = max(abs(p[k]-grouped[p["timestamp"]][0][k]) for p in matching for k in ("x", "y", "z"))
        require(max_delta < 1e-6, "ROS/ULog estimator mismatch")
        report["estimator_correlation"] = {"exact_timestamp_pairs": len(matching), "max_position_difference_m": max_delta}
    offsets = [r["ros_sim_s"]-r["fields"]["timestamp"]/1e6 for r in json_events
               if r.get("event") == "received" and r.get("topic_key") == "vehicle_local_position" and r["ros_sim_s"] > 0]
    report["position_receipt_clock_age_s"] = {"min": min(offsets), "max": max(offsets)}
    aborts = [r for r in events if r["event"] == "abort"]
    if aborts:
        abort = aborts[0]
        require(all(t["timestamp"]/1e6 <= abort["sim_s"] for t in streams["trajectory_setpoint"]), "Old targets continued after abort")
        report.update({"flight_acceptance": "failed", "controller_failure": abort["reason"],
                       "abort_sim_s": abort["sim_s"], "setpoints_stopped_at_abort": True,
                       "observed_recovery": "landed_disarmed" if report["final_disarmed"] and report["final_landed"] else "not_observed"})
        reset_events = []
        previous = None
        for p in ulog["vehicle_local_position"]:
            counters = {k: p[k] for k in ("xy_reset_counter", "z_reset_counter", "heading_reset_counter", "vxy_reset_counter", "vz_reset_counter", "ref_timestamp")}
            if previous is not None and counters != previous and p["timestamp"] >= first:
                reset_events.append({"sim_s": p["timestamp"]/1e6, "before": previous, "after": counters,
                                     "position_ned": [p[k] for k in ("x", "y", "z")], "delta_heading_rad": p["delta_heading"]})
            previous = counters
        report["ulog_reference_resets_after_first_command"] = reset_events
        require(abort["reason"] != "estimator_reference_reset" or reset_events, "Reset failure not independently supported")
        return report
    samples = [r["sample"] for r in events if r["event"] == "sample"]
    if revised:
        if historical:
            reconstructed = reconstruct_controller_samples(json_events, config, summary["run_id"], historical=True)
        samples = reconstructed["samples"]
        report["controller_source_reconstruction"] = reconstructed["statistics"]
    if config.get("reference_policy") in ("p2-estimator-reference-v2", "p2-estimator-reference-v3"):
        from reference_evidence import classify, ulog_reference
        reference_config = {k: v for k, v in config.items() if k != "sample_evidence_contract"} if historical else config
        independent_reference = classify(run, reference_config)
        report["reference_reconciliation"] = independent_reference
        report["reference_ulog"] = ulog_reference(log, result, independent_reference)
        report["reference_evaluator_sha256"] = digest(Path(__file__).with_name("reference_evidence.py"))
        if config["reference_policy"] == "p2-estimator-reference-v3":
            from yaw_evidence import verify_yaw_ownership
            if revised:
                try:
                    report["yaw_ownership"] = verify_yaw_ownership(log, streams, events, result, config, independent_reference)
                except ValueError as exc:
                    report["yaw_ownership"] = {"status": "failed", "reason": str(exc),
                        "limitation": "Internal MC consumed source identity is not serialized; the frozen prior-state consistency check is not exact internal-selection proof."}
                    if getattr(exc, "report", None) is not None:
                        report["yaw_ownership"]["prior_state_consistency"] = exc.report
            else:
                report["yaw_ownership"] = verify_yaw_ownership(log, streams, events, result, config, independent_reference)
            report["yaw_evaluator_sha256"] = digest(Path(__file__).with_name("yaw_evidence.py"))
    report["ros_controller_samples"] = evaluate_flight(result, samples, config, historical=historical)
    fixture = expected_fixture(result["origin_ned"], result["initial_yaw_ned"], config)
    report["independent_windows"] = {}
    for phase in PHASES:
        window = result["windows"][phase]
        start, end = window["start_sim_s"], window["end_sim_s"]
        duration = config["initial_hover_sim_s"] if phase == "INITIAL_HOVER" else config["final_hover_sim_s"] if phase == "FINAL_HOVER" else config["dwell_sim_s"]
        report["independent_windows"][phase] = {}
        for label, data in (("rosbag", streams), ("ulog", ulog)):
            # Bracket the fixed controller window, never search for a better window.
            positions = data["vehicle_local_position"]
            if revised:
                start_us, end_us = fixed_window_us(window,historical)
                left = max(p["timestamp"] for p in positions if p["timestamp"] <= start_us)
                right = min(p["timestamp"] for p in positions if p["timestamp"] >= end_us)
                joined = joined_samples_v2(data, config, left, right, summary["run_id"])
                window_report = evaluate_window_v2(joined, fixture[phase], duration, config)
                try:
                    window_report["all_record_components"] = all_record_window_checks(data, left, right, fixture[phase], config)
                except ValueError as exc:
                    window_report.update(status="failed", all_record_components={"status": "failed", "reason": str(exc)})
                window_report["fixed_controller_window_us"] = [start_us, end_us]
                window_report["bracket_publication_us"] = [left, right]
                report["independent_windows"][phase][label] = window_report
            else:
                left = max(p["timestamp"]/1e6 for p in positions if p["timestamp"]/1e6 <= start)
                right = min(p["timestamp"]/1e6 for p in positions if p["timestamp"]/1e6 >= end)
                joined = joined_samples(data, config, left, right)
                report["independent_windows"][phase][label] = evaluate_window(joined, fixture[phase], duration, config)
    land_command = next(r for r in sent if r["command"] == 21)
    require(all(t["timestamp"] < land_command["timestamp"] for t in streams["trajectory_setpoint"]), "Competing targets after landing handover")
    require(any(s["nav_state"] == 18 and s["timestamp"] >= land_command["timestamp"] for s in status), "No actual landing mode")
    passed = (summary["status"] == "passed" and report["ros_controller_samples"]["status"] == "passed"
              and all(w["status"] == "passed" for phases in report["independent_windows"].values() for w in phases.values())
              and report.get("yaw_ownership", {}).get("status", "passed") == "passed"
              and report["final_disarmed"] and report["final_landed"] and not report["failsafe_observed"])
    report["flight_acceptance"] = "passed" if passed else "failed"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--output-name", default="p2-offline-evaluation.json")
    parser.add_argument("--sample-contract", choices=["p3-sample-evidence-v2"], help="Explicit historical reanalysis; never runtime credit")
    args = parser.parse_args(argv)
    require(Path(args.output_name).name == args.output_name and args.output_name.endswith(".json"), "Output must be a JSON basename")
    output = args.run/args.output_name
    require(not output.exists(), "Offline result already exists; retain the first evaluation")
    try:
        require(not args.sample_contract or args.output_name != "p2-offline-evaluation.json", "Historical reanalysis needs a distinct report name")
        report = verify(args.run.resolve(), args.sample_contract)
    except Exception as exc:
        try:
            from p3_provenance import strict_json
            failed_input = strict_json(args.run/'summary.json')
        except Exception:
            failed_input = {}
        report = {"run_id": args.run.name, "recording_integrity": "failed", "flight_acceptance": "unknown", "error": str(exc),
                  "sample_evidence_contract": args.sample_contract or failed_input.get('config',{}).get('sample_evidence_contract'),
                  "frozen_inputs": failed_input.get('frozen_inputs'), "historical_reanalysis": bool(args.sample_contract)}
    report["evaluator_sha256"] = digest(__file__)
    with output.open("x") as stream:
        stream.write(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k: v for k, v in report.items() if k not in ("artifacts", "independent_windows")}, indent=2, allow_nan=False))
    return 0 if report["recording_integrity"] == "passed" and report["flight_acceptance"] in ("passed", "not_run") else 1


if __name__ == "__main__":
    raise SystemExit(main())
