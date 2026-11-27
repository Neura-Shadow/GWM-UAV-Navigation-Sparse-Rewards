"""P6 timing metrics (contract §4.1, §6.1–6.2), each in its own clock. Reported; the pass/fail limits
are the response checks'.

* detection latency: first record written by accepted code that names the fault, minus the trigger (sim);
* fault-to-safe-action latency: first held tick (HOVER or policy HOLD) or first LAND, minus the trigger (sim);
* time in degraded mode: first fault label to recovery (progress on fresh inputs) or ABORT (sim);
* landing: LAND command to landed and disarmed (wall, from PX4's own state messages).
"""


def trigger_sim(run):
    for injector, records in run.fault_ledgers().items():
        act = next((r for r in records if r["kind"] == "activated" and r.get("sim_s") is not None), None)
        if act is not None:
            return act["sim_s"]
    injection = run.injection or {}
    return injection.get("t_inj_sim_s")


def metrics(run):
    t = trigger_sim(run)
    out = {"trigger_sim_s": t}
    if t is None:
        return out
    after = [e for e in run.decisions if e["sim_s"] > t]
    failures = [r for r in run.ledger if r.get("invoked") and r.get("failure") is not None and r["sim_s"] > t]
    detected = [e["sim_s"] for e in after if e.get("stale")] + [r["sim_s"] for r in failures]
    p2_abort = next((e for e in run.events if e.get("event") == "abort" and (e.get("sim_s") or 0) >= t), None)
    if p2_abort is not None:
        detected.append(p2_abort["sim_s"])
    out["detection_latency_sim_s"] = min(detected) - t if detected else None
    held = [e["sim_s"] for e in after if e["action"] == "HOLD" and (e.get("stale") or e.get("state") == "HOVER")]
    held += [r["sim_s"] for r in failures if r["action"] == "HOLD"]
    lands = [e["ros_sim_s"] for e in run.commands if e["fields"]["command"] == 21 and e["ros_sim_s"] >= t]
    safe = held + lands
    out["fault_to_safe_action_sim_s"] = min(safe) - t if safe else None
    first_fault = min(detected) if detected else None
    end = next((e["sim_s"] for e in after if first_fault is not None and e["sim_s"] > first_fault
                and not e.get("stale") and e["state"] in ("AVOIDING", "RESUME", "NORMAL") and e["action"] in ("TRANSLATE", "YAW_TO")),
               None)
    abort = next((x["sim_s"] for x in run.transitions if x["to"] == "ABORT"), None)
    stop = min(v for v in (end, abort) if v is not None) if (end is not None or abort is not None) else None
    out["degraded_sim_s"] = None if first_fault is None or stop is None else stop - first_fault
    out["recovered"] = end is not None and (abort is None or end < abort)
    return out
