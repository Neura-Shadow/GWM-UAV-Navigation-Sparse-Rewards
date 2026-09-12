"""Read-only reference classification and independent ULog checks."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM/"ros2_ws/src/gwm_px4_control"))
from gwm_px4_control.estimator_reference import ReferenceManager, V2, V3
from gwm_px4_control.timing import StateCache


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def classify(run, config, historical=False):
    result = json.loads((run/"controller-result.json").read_text())
    cache, manager = StateCache(config), None
    phase = "WAIT_FOR_CLOCK_AND_CONNECTION"
    for line in (run/"ros-events.jsonl").open():
        row = json.loads(line)
        sim, wall = row["ros_sim_s"], row["monotonic_s"]
        if sim > 0:
            cache.clock(sim, wall)
        if row["event"] == "transition":
            phase = row["to"]
            if phase == "PRESTREAM_SAFE_SETPOINTS":
                anchor = (result.get("reference") or {}).get("initial_anchor", result["initial_yaw_ned"])
                manager = ReferenceManager(config, cache.data, result["origin_ned"], anchor)
            if historical and phase == "RECOVERY":
                phase = "TAKEOFF"  # classification-only hypothesis; actual mode/health still required
        if row["event"] == "received" and row["topic_key"] != "vehicle_command_ack":
            cache.update(row["topic_key"], row["fields"], wall)
            if manager:
                sample = cache.validate(sim, wall, terminal_landed=(
                    not historical and phase in ("VERIFY_LANDED_AND_DISARMED", "COMPLETE")))
                manager.inspect(cache.data, sample, sim, wall, phase)
                if historical and manager.accepted:
                    break
        if row["event"] == "reference_locked":
            require(manager is not None, "Reference lock without preparation")
            manager.lock(row["sim_s"])
    require(manager is not None, "Missing reference preparation")
    if not historical:
        actual = result.get("reference")
        require(actual and actual["policy"] == config["reference_policy"] and actual["policy"] in (V2,V3), "Missing revised policy result")
        require(manager.lock_sim_s is not None, "Missing recorded final alignment lock")
        require(len(manager.accepted) == actual["accepted_count"], "Fabricated/missing reset event")
        require(abs(manager.anchor-actual["anchor"]) < 1e-8, "Fabricated anchor correction")
        for verified, recorded in zip(manager.accepted, actual["accepted"]):
            require(verified["new_counters"] == recorded["new_counters"]
                    and abs(verified["delta_heading"]-recorded["delta_heading"]) < 1e-8, "Reset ledger differs from raw observations")
    return manager.summary()


def ulog_reference(log, result, reference):
    import numpy as np
    p = log.get_dataset("vehicle_local_position").data
    a = log.get_dataset("vehicle_attitude").data
    e = log.get_dataset("estimator_status_flags").data
    status = log.get_dataset("estimator_status").data
    control = log.get_dataset("vehicle_local_position_setpoint").data
    start = next(r["sim_s"] for r in result["transitions"] if r["to"] == "PRESTREAM_SAFE_SETPOINTS")*1e6
    active = p["timestamp"] >= start
    require(active.any(), "Missing reference coverage")
    for key in ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter", "vz_reset_counter",
                "dist_bottom_reset_counter", "ref_timestamp", "ref_lat", "ref_lon", "ref_alt"):
        require(np.all(p[key][active] == p[key][active][0]), "ULog unsupported reference change: "+key)
    estimator_instances = {d.multi_id for d in log.data_list if d.name == "estimator_status"}
    require(estimator_instances == {0}, "Unsupported EKF instance set")
    valid = status["timestamp"] >= start
    devices = {}
    for key in ("accel_device_id", "gyro_device_id", "mag_device_id", "baro_device_id"):
        values = set(int(x) for x in status[key][valid])
        require(len(values) == 1 and next(iter(values)) != 0, "Estimator device changed/missing: "+key)
        devices[key] = next(iter(values))
    require(not status["filter_fault_flags"][valid].any(), "ULog filter fault")
    h_changes = np.flatnonzero(np.diff(p["heading_reset_counter"].astype(int)) != 0)+1
    h_changes = [i for i in h_changes if p["timestamp"][i] >= start]
    q_changes = np.flatnonzero(np.diff(a["quat_reset_counter"].astype(int)) != 0)+1
    q_changes = [i for i in q_changes if a["timestamp"][i] >= start]
    require(len(h_changes) == len(q_changes) == reference["accepted_count"], "Unexpected/missing ULog reset count")
    events = []
    for h, q, accepted in zip(h_changes, q_changes, reference["accepted"]):
        require(abs(float(p["delta_heading"][h])-accepted["delta_heading"]) < 1e-6, "ULog heading delta mismatch")
        dq = [float(a[f"delta_q_reset[{n}]"][q]) for n in range(4)]
        require(max(abs(x-y) for x,y in zip(dq,accepted["quaternion"]["delta_q_reset"])) < 1e-6, "ULog quaternion delta mismatch")
        require(abs(int(p["timestamp"][h])-int(a["timestamp"][q])) <= 100000, "ULog reset pair skew")
        after = np.flatnonzero(e["timestamp"] >= p["timestamp"][h])
        require(len(after) > 0, "No estimator flags after reset")
        near = [i for i in after if e["timestamp"][i] <= p["timestamp"][h]+1500000]
        require(any(e["cs_mag_aligned_in_flight"][i] and e["cs_mag_3d"][i] and e["cs_in_air"][i] for i in near), "No ULog final magnetic initialization evidence")
        require(all(e["cs_gnss_pos"][i] and not e["cs_vehicle_at_rest"][i] for i in near), "Ambiguous WMM/no-aiding reset context")
        # Raw internal PX4 output must exhibit the once-corrected anchor.
        window = (control["timestamp"] >= max(p["timestamp"][h],a["timestamp"][q])) & (control["timestamp"] <= (accepted["accepted_sim_s"]+.25)*1e6)
        values = control["yaw"][window]
        if reference.get("policy", V2) != V3:
            require(len(values) and np.all(np.abs(values-accepted["anchor_after"]) < 1e-5), "PX4 cached setpoint correction missing/doubled")
        events.append({"ulog_reset_sim_s": float(p["timestamp"][h])/1e6, "delta_heading": accepted["delta_heading"],
                       "internal_yaw_samples": len(values), "internal_yaw_max_error_rad": float(np.max(np.abs(values-accepted["anchor_after"])))})
    if reference["lock_sim_s"] is not None:
        end = reference["lock_sim_s"]*1e6
        beginning = (reference["lock_sim_s"]-5)*1e6
        for source,key in ((p,"heading_good_for_control"),(e,"cs_mag_aligned_in_flight"),(e,"cs_mag_3d")):
            first = max(0,int(np.searchsorted(source["timestamp"],beginning,side="right"))-1)
            last = int(np.searchsorted(source["timestamp"],end,side="right"))
            require(last > first and source[key][first:last].all(), "ULog final alignment not stable for full five seconds")
    return {"status": "passed", "instance_ids": sorted(estimator_instances), "devices": devices,
            "reset_events": events, "position_velocity_origin_unchanged": True,
            "compensation_ownership": "mode-specific yaw evaluation required" if reference.get("policy", V2) == V3 else "PX4 corrected cached target; ROS preserved one corrected anchor"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--historical", type=Path, required=True)
    args = parser.parse_args()
    from collect_p2_evidence import verify
    from pyulog import ULog
    run = args.historical.resolve()
    output = run.parent/(time.strftime("%Y%m%dT%H%M%SZ",time.gmtime())+"-p2-r1-offline-"+uuid.uuid4().hex[:8])
    output.mkdir()
    config = json.loads((SIM/"configs/p2_control.yaml").read_text())
    try:
        original = verify(run)
        require(original["flight_acceptance"] == "failed", "Historical abort must remain failed")
        reference = classify(run, config, historical=True)
        require(reference["accepted_count"] == 1, "Historical initialization not reconciled")
        summary = json.loads((run/"summary.json").read_text())
        ulog = ULog(summary["ulog_files"][0]["path"])
        independent = ulog_reference(ulog, summary["controller_result"], reference)
        report = {"status": "passed", "classification": "offline reset-classification/reconciliation validation",
                  "historical_full_flight": "failed", "historical_run": run.name,
                  "reference": reference, "independent_ulog": independent,
                  "original_offline_verification": original, "source_config": config}
    except Exception as exc:
        report = {"status": "failed", "historical_full_flight": "failed", "error": str(exc)}
    report["script_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    sys.path.insert(0, str(SIM/"scripts"))
    from p2_build import package_manifest, package_hash
    report["package_hash"] = package_hash(package_manifest(SIM))
    report["config_sha256"] = hashlib.sha256((SIM/"configs/p2_control.yaml").read_bytes()).hexdigest()
    (output/"summary.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(str(output))
    print(json.dumps({k:v for k,v in report.items() if k not in ("original_offline_verification","source_config")},indent=2,allow_nan=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
