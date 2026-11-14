"""P6 response checks (contract §5, §6): did the accepted stack respond to the fault as required?

Each check is one outcome code of `p6_faults.yaml` evaluated on the accepted controller's own
records (decisions, policy ledger, events, transactions) with the numbers frozen in the codes. The
supervisor and ledger records are written by accepted code, never by an injector.
"""
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]  # the repository (the accepted P3 validation config)

ACCEPTED = 0


class Checks:
    def __init__(self, codes):
        self.codes = codes
        self.out = {}
        self.failures = []

    def record(self, code, ok, **values):
        self.out[code] = {"passed": bool(ok), **values}
        if not ok:
            self.failures.append("response:" + code)


def stale_window(run):
    """(first stale decision, the reason, the last usable snapshot it followed) or (None, None, None)."""
    first = next((e for e in run.decisions if e.get("stale")), None)
    if first is None:
        return None, None, None
    before = [e["snapshot_acquisition_sim_s"] for e in run.decisions
              if e["sim_s"] < first["sim_s"] and e.get("snapshot_acquisition_sim_s") is not None]
    return first, first["stale"], max(before, default=None)


def h35(run, c):
    code = c.codes["H35"]
    first, reason, t_last = stale_window(run)
    if first is None:
        return c.record("H35", False, error="no_stale_decision")
    limit = code["hover_after_last_usable_sim_s"]
    if reason == "adapter_health_missing":
        fresh = [h["sim_s"] for h in run.health if h.get("status") == "fresh" and h.get("sim_s") is not None
                 and h["sim_s"] < first["sim_s"]]
        anchor, limit = max(fresh, default=None), 2.0 + 0.10
    else:
        anchor = t_last
    held = [e for e in run.decisions if e["sim_s"] >= first["sim_s"] and e.get("stale")]
    motion = max((math.dist(held[0]["target_enu"], e["target_enu"]) for e in held), default=0.0)
    in_time = anchor is not None and first["sim_s"] <= anchor + limit + 1e-9
    ok = (first["state"] == "HOVER" and first.get("frozen_yaw") and in_time
          and all(e["action"] == "HOLD" for e in held) and motion <= code["stale_target_motion_m"])
    yaws = [s["fields"]["yaw"] for s in run.setpoints if held and held[0]["sim_s"] <= s["ros_sim_s"] <= held[-1]["sim_s"]
            and s.get("fields", {}).get("yaw") is not None]
    yaw_change = max((abs(math.degrees(math.remainder(y - yaws[0], 2 * math.pi))) for y in yaws), default=0.0)
    ok = ok and yaw_change <= code["stale_target_yaw_deg"]
    c.record("H35", ok, reason=reason, first_stale_sim_s=first["sim_s"], anchor_sim_s=anchor, limit_s=limit,
             target_motion_m=motion, yaw_change_deg=yaw_change)


def a10(run, c, expected_reason=None):
    code = c.codes["A10"]
    first, reason, _ = stale_window(run)
    abort = next((t for t in run.transitions if t["to"] == "ABORT"), None)
    ok = first is not None and abort is not None and str(abort["reason"]).startswith("stale_inputs:")
    delay = None if not ok else abort["sim_s"] - first["sim_s"]
    ok = ok and abs(delay - code["abort_after_stale_sim_s"]) <= code["tolerance_sim_s"] + 1e-9
    if expected_reason:
        ok = ok and abort["reason"] == "stale_inputs:" + expected_reason
    c.record("A10", ok, abort_reason=abort and abort["reason"], stale_to_abort_sim_s=delay)


def ht20(run, c):
    code = c.codes["HT20"]
    hover = next((t for t in run.transitions if t["to"] == "HOVER"), None)
    abort = next((t for t in run.transitions if t["to"] == "ABORT"), None)
    delay = None if not (hover and abort) else abort["sim_s"] - hover["sim_s"]
    ok = abort is not None and abort["reason"] == "hover_timeout" and abs(delay - code["abort_after_hover_sim_s"]) <= code["tolerance_sim_s"] + 1e-9
    c.record("HT20", ok, hover_to_abort_sim_s=delay)


def ph3(run, c):
    """Hold on every failing tick, abort `policy_unavailable` on the 3rd consecutive one, LAND in that tick."""
    code = c.codes["PH3"]
    records = [r for r in run.ledger if r.get("invoked")]
    failing = [r for r in records if r.get("failure") is not None]
    if len(failing) < code["consecutive_failures_to_abort"]:
        return c.record("PH3", False, error="fewer_than_three_failures", failures=len(failing))
    first, third = failing[0], failing[code["consecutive_failures_to_abort"] - 1]
    consecutive = [r["tick"] for r in failing[:3]] == list(range(first["tick"], first["tick"] + 3))
    holds = all(r["action"] == "HOLD" for r in failing[:3])
    decisions = {round(e["sim_s"], 6): e for e in run.decisions}
    targets = [decisions[round(r["sim_s"], 6)]["target_enu"] for r in failing[:3] if round(r["sim_s"], 6) in decisions]
    motion = max((math.dist(targets[0], t) for t in targets), default=None)
    abort = next((t for t in run.transitions if t["to"] == "ABORT"), None)
    delay = None if abort is None else abort["sim_s"] - first["sim_s"]
    land = next((e for e in run.commands if e["fields"]["command"] == 21), None)
    land_delay = None if land is None or abort is None else land["ros_sim_s"] - abort["sim_s"]
    ok = (consecutive and holds and motion is not None and motion <= code["failing_tick_target_motion_m"]
          and abort is not None and abort["reason"] == code["abort_reason"] and abs(abort["sim_s"] - third["sim_s"]) < 1e-6
          and abs(delay - code["abort_after_first_failure_sim_s"]) <= code["tolerance_sim_s"] + 1e-9
          and land_delay is not None and 0 <= land_delay <= code["land_after_abort_sim_s"] + 1e-9)
    c.record("PH3", ok, failing_ticks=[r["tick"] for r in failing[:3]], target_motion_m=motion,
             first_failure_to_abort_sim_s=delay, abort_to_land_sim_s=land_delay)


def l45(run, c):
    """The accepted P4 revision 5 abort landing (one accepted LAND, COMPLETE within 45 s wall), or a goal landing."""
    import p4_evaluate as p4e
    f = _Failures()
    result = run.result
    if result.get("p4_outcome") == "completed":
        seconds = None
        ok = result.get("final_state") == "COMPLETE" and not result.get("failure")
    else:
        seconds = p4e.abort_landing(result, run.summary.get("config", {}).get("recovery_wall_s", 45.0), "l45:", f)
        ok = not f.items
    c.record("L45", ok, landing_wall_s=seconds, items=f.items)


def l45r(run, c, expected_failure):
    """P2 RECOVERY landing: the declared P2 failure, exactly one LAND, landed and disarmed (PX4 state) within 45 s wall."""
    code = c.codes["L45R"]
    result = run.result
    lands = [t for t in result.get("transactions", []) if t["command"] == 21]
    land = lands[0] if len(lands) == 1 else None
    landed = None
    if land is not None:
        status = {}
        for e in run.events:
            if e.get("event") != "received" or e["monotonic_s"] < land["sent_monotonic_s"]:
                continue
            if e["topic_key"] == "vehicle_land_detected":
                status["landed"] = e["fields"].get("landed")
            elif e["topic_key"] == "vehicle_status":
                status["disarmed"] = e["fields"].get("arming_state") == 1
            if status.get("landed") is True and status.get("disarmed") is True:
                landed = e["monotonic_s"] - land["sent_monotonic_s"]
                break
    ok = (result.get("final_state") == "RECOVERY" and result.get("failure") == expected_failure and land is not None
          and landed is not None and landed <= code["ulog_landed_and_disarmed_within_wall_s_of_land"])
    c.record("L45R", ok, failure=result.get("failure"), land_commands=len(lands), landed_and_disarmed_wall_s=landed)


def r2(run, c, expected_failure):
    result = run.result
    c.record("R2", result.get("final_state") == "RECOVERY" and result.get("failure") == expected_failure,
             failure=result.get("failure"))


RECOVERY_LANDING_ITEMS = ("cleanup:ros-controller:nonzero_exit_1", "rgb_absence:snapshot_missing:window_end",
                          "p2_integrity_error:ValueError:estimator_reference_reset:final_alignment_lost",
                          "mc_causal_yaw:p2_integrity_missing")


def recovery_landing(run, c):
    """M6 decision (operator, 2026-11-14, "Adopt"): the four P5 items every P2 RECOVERY landing produces in accepted code,
    on S-5' only, each replaced by evidence the accepted code did not write for that purpose (plan §16.6):
    1. the designed exit: `main` exits 1 because P4 `outcome()` is `failed` for a RECOVERY end; connectivity had
       passed, the final state is RECOVERY and exactly one LAND was sent;
    2. the `window_end` RGB-absence snapshot is taken on COMPLETE only; the continuous absence monitor (the accepted
       ledger the P5 evaluator reports) covers the whole window with zero IMX214 subscribers, within its gap limit,
       and every other absence check passed;
    3. the accepted offline replay feeds RECOVERY-phase samples to the reference manager, which rejects
       `final_alignment_lost` once `cs_in_air` is false after touchdown; the live controller's own record shows no
       rejection, and `cs_in_air` first went false after the LAND command and after `landed`;
    4. the missing P2 report is the consequence of 3 and nothing else."""
    import json as _json
    result, summary = run.result, run.summary
    p5 = run.p5_evaluation or {}
    items = list(p5.get("failures") or [])
    lands = [t for t in result.get("transactions", []) if t["command"] == 21]
    exit_ok = (summary.get("controller_exit_code") == 1 and result.get("p4_outcome") == "failed"
               and result.get("connectivity") == "passed" and result.get("final_state") == "RECOVERY" and len(lands) == 1)
    absence = p5.get("rgb_absence") or {}
    monitor = absence.get("monitor") or {}
    window = absence.get("window_monotonic_s") or [None, None]
    coverage = monitor.get("coverage_monotonic_s") or [None, None]
    settings = _json.loads((ROOT / "simulation/px4_gazebo/configs/p3_validation.yaml").read_text(encoding="utf-8"))["rgb_absence_monitor"]
    absence_ok = (absence.get("failures") == ["snapshot_missing:window_end"] and None not in window and None not in coverage
                  and coverage[0] <= window[0] and coverage[1] >= window[1]
                  and (monitor.get("max_sample_gap_wall_s") or float("inf")) <= settings["max_sample_gap_wall_s"]
                  and absence.get("rgb_metadata_rows") == 0)
    reference = result.get("reference") or {}
    rejected_events = [e for e in run.events if e.get("event") == "reference_rejected"]
    land_wall = lands[0]["sent_monotonic_s"] if lands else None
    landed_wall = next((e["monotonic_s"] for e in run.events if e.get("event") == "received" and e.get("topic_key") == "vehicle_land_detected"
                        and e["fields"].get("landed") is True and land_wall is not None and e["monotonic_s"] >= land_wall), None)
    in_air_false_wall = next((e["monotonic_s"] for e in run.events if e.get("event") == "received"
                              and e.get("topic_key") == "estimator_status_flags" and e["fields"].get("cs_in_air") is False
                              and land_wall is not None and e["monotonic_s"] >= land_wall), None)
    replay_item = "p2_integrity_error:ValueError:estimator_reference_reset:final_alignment_lost"
    reference_ok = (reference.get("rejected_count") == 0 and reference.get("rejection") is None and not rejected_events
                    and replay_item in items and landed_wall is not None and in_air_false_wall is not None
                    and in_air_false_wall >= landed_wall)
    other = [x for x in items if x.startswith(("p2_integrity_error", "mc_causal", "rgb_absence:", "cleanup:"))
             and x not in RECOVERY_LANDING_ITEMS]
    ok = exit_ok and absence_ok and reference_ok and not other
    c.record("RECOVERY_LANDING", ok, exit_designed=exit_ok, absence_monitor_covers_window=absence_ok,
             reference_rejected_only_in_replay_after_touchdown=reference_ok, other_items=other,
             controller_exit_code=summary.get("controller_exit_code"), p4_outcome=result.get("p4_outcome"),
             land_commands=len(lands), monitor_coverage_monotonic_s=coverage, window_monotonic_s=window,
             monitor_max_gap_wall_s=monitor.get("max_sample_gap_wall_s"), controller_rejected_count=reference.get("rejected_count"),
             land_wall_s=land_wall, landed_wall_s=landed_wall, cs_in_air_false_wall_s=in_air_false_wall)


def goal(run, c):
    result = run.result
    c.record("GOAL", result.get("p4_outcome") == "completed" and result.get("final_state") == "COMPLETE",
             p4_outcome=result.get("p4_outcome"))


def recovery(run, c):
    """Contract §6.5: every return from a stale HOVER to progress happens on a fresh, non-stale decision."""
    age = 0.25
    returns = [t for t in run.transitions if t["from"] == "HOVER" and t["to"] in ("AVOIDING",)]
    decisions = {round(e["sim_s"], 6): e for e in run.decisions}
    bad = []
    for t in returns:
        e = decisions.get(round(t["sim_s"], 6))
        if e is None or e.get("stale") or e["sim_s"] - e["snapshot_acquisition_sim_s"] > age:
            bad.append(t["sim_s"])
    c.record("RECOVERY", bool(returns) and not bad, returns=len(returns), unrevalidated=bad)


def stale_iff_age(run, c):
    """Supervisor legality over the fault window: `snapshot_stale` exactly when the snapshot is older than 0.25 s."""
    wrong = [e["sim_s"] for e in run.decisions if e.get("snapshot_acquisition_sim_s") is not None
             and (e.get("stale") == "snapshot_stale") != (e["sim_s"] - e["snapshot_acquisition_sim_s"] > 0.25)]
    c.record("STALE_IFF_AGE", not wrong, violations=wrong[:10])


def no_invoke_after_stale(run, c):
    first, _, _ = stale_window(run)
    late = [] if first is None else [r["tick"] for r in run.ledger if r.get("invoked") and r["sim_s"] >= first["sim_s"]]
    c.record("NO_INVOKE_AFTER_STALE", not late, invoked_after_stale=late[:10])


def limits_uninjected(run, c, limits, injected_sim):
    """P5 inference limits on every tick the declaration did not fault (contract §4: only those are excluded)."""
    times = sorted(r["inference_s"] for r in run.ledger if r.get("invoked") and round(r["sim_s"], 6) not in injected_sim)
    p99 = times[min(len(times) - 1, int(0.99 * len(times)))] if times else None
    ok = bool(times) and p99 <= limits["inference_p99_s"] and times[-1] <= limits["inference_max_s"]
    c.record("LIMITS_UNINJECTED", ok, ticks=len(times), p99_s=p99, max_s=times[-1] if times else None)


def unsafe_setpoints(run, c):
    """Contract §6.3 (a): no target motion beyond 0.01 m or 0.5 deg on any tick that must hold (stale or policy failure)."""
    code = c.codes["H35"]
    hold_ticks = {round(e["sim_s"], 6) for e in run.decisions if e.get("stale")}
    hold_ticks |= {round(r["sim_s"], 6) for r in run.ledger if r.get("failure") is not None}
    ordered = sorted(run.decisions, key=lambda e: e["sim_s"])
    moves = []
    # A decision event records the target at the start of its tick (P4 `_transit` logs before `apply_action`), so the
    # motion a tick causes is the difference to the *next* decision's target.
    for e, after in zip(ordered, ordered[1:]):
        if round(e["sim_s"], 6) in hold_ticks and math.dist(e["target_enu"], after["target_enu"]) > code["stale_target_motion_m"]:
            moves.append(e["sim_s"])
    c.record("UNSAFE", not moves, hold_ticks=len(hold_ticks), moves=moves[:10])


class _Failures:
    def __init__(self):
        self.items = []

    def check(self, ok, name):
        if not ok:
            self.items.append(name)


# ---- Gate A ground fixtures: the accepted supervisor's stale rule replayed on the recorded ground inputs -----
GROUND_LIMIT_S = {"snapshot_stale": 0.35, "adapter_health_missing": 2.0 + 0.10, "adapter_health_not_fresh": 1.2 + 0.10,
                  "calibration_mismatch": 0.35, "unusable_frames": 0.35}


def ground_replay(run, c, expected, config):
    """From the trigger on, step the accepted `Supervisor.stale_reason` at 20 Hz over perception's recorded
    snapshots and the adapter's recorded health, as the controller received them. The first stale reason must be
    one of `expected`, within that reason's detection limit. (Controller-input faults are not ground fixtures:
    their input change leaves no record outside the injector on the ground, M4 decision.)"""
    from gwm_obstacle_avoidance import supervisor
    injection = run.injection or {}
    t0 = injection.get("t_inj_sim_s")
    if t0 is None:
        from p6_timing_evaluate import trigger_sim
        t0 = trigger_sim(run)
    if t0 is None:
        return c.record("GROUND_REPLAY", False, error="no_trigger_time")
    snapshots = [s for s in run.local_map if s.get("usable")]
    health = [h for h in run.health if h.get("sim_s") is not None]
    sup = supervisor.Supervisor(config, t0)
    end = max([s.get("received_sim_s", s["processing_sim_s"]) for s in snapshots] + [h["sim_s"] for h in health] + [t0]) + 3.0
    t, found = t0, None
    while t <= end and found is None:
        usable = [s for s in snapshots if s.get("received_sim_s", s["processing_sim_s"]) <= t]
        record = [h for h in health if h["sim_s"] <= t]
        reason = sup.stale_reason({"sim": t, "snapshot": usable[-1] if usable else None,
                                   "adapter_health": record[-1] if record else None, "consecutive_unusable": 0, "reset_sim": None})
        if reason:
            found = (t, reason)
        t += 0.05
    if found is None:
        return c.record("GROUND_REPLAY", False, error="supervisor_would_not_hover")
    reason = found[1]
    anchor = t0
    if reason == "adapter_health_missing":
        fresh = [h["sim_s"] for h in health if h["sim_s"] <= found[0]]
        anchor = max(fresh, default=t0)
    elif reason == "snapshot_stale":
        before = [s["acquisition_sim_s"] for s in snapshots if s.get("received_sim_s", s["processing_sim_s"]) <= found[0]]
        anchor = max(before, default=t0)
    limit = GROUND_LIMIT_S.get(reason, 0.35)
    ok = reason in expected and found[0] <= anchor + limit + 1e-9
    c.record("GROUND_REPLAY", ok, reason=reason, hover_sim_s=found[0], anchor_sim_s=anchor, limit_s=limit)
