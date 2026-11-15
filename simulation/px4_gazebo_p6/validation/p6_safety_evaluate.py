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


def recovery_landing(run, c, explained=()):
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
    4. the missing P2 report is the consequence of 3 and nothing else.
    `explained`: exact items another passing proof of the same row explains (C-5: LOAD_RECORDER), which therefore do
    not count as unexplained here; every other cleanup, integrity and absence item still fails this proof."""
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
             and x not in RECOVERY_LANDING_ITEMS and x not in explained]
    ok = exit_ok and absence_ok and reference_ok and not other
    c.record("RECOVERY_LANDING", ok, exit_designed=exit_ok, absence_monitor_covers_window=absence_ok,
             reference_rejected_only_in_replay_after_touchdown=reference_ok, other_items=other,
             controller_exit_code=summary.get("controller_exit_code"), p4_outcome=result.get("p4_outcome"),
             land_commands=len(lands), monitor_coverage_monotonic_s=coverage, window_monotonic_s=window,
             monitor_max_gap_wall_s=monitor.get("max_sample_gap_wall_s"), controller_rejected_count=reference.get("rejected_count"),
             land_wall_s=land_wall, landed_wall_s=landed_wall, cs_in_air_false_wall_s=in_air_false_wall)


def fail_closed_landing(run, c, explained=()):
    """C-3 (M6 decision, operator 2026-11-14, "Adopt both"; contract §8.4: S0 required, outcome reported). When the
    accepted P2 controller fails closed and lands through RECOVERY, that landing is the reported outcome: L45R for the
    reported P2 failure (one LAND, landed and disarmed within the limit) and the RECOVERY_LANDING proof. For any other
    ending nothing is recorded, so no exception keyed to this check can apply. (C-5 uses it for the §5.3 dispatch
    abort, which LOAD matches to its breach.)"""
    result = run.result
    if result.get("final_state") != "RECOVERY" or not result.get("failure"):
        return
    sub = Checks(c.codes)
    l45r(run, sub, result["failure"])
    recovery_landing(run, sub, explained)
    ok = sub.out["L45R"]["passed"] and sub.out["RECOVERY_LANDING"]["passed"]
    c.record("FAIL_CLOSED_LANDING", ok, p2_failure=result["failure"], l45r=sub.out["L45R"],
             recovery_landing=sub.out["RECOVERY_LANDING"])


def landing(run, c, explained=()):
    """C-5 (M6 decision, operator 2026-11-14, "Adopt as recommended", plan §16.11): after H35 the row lands either
    through COMPLETE (recovery and the goal, or a supervisor abort: L45), or, when the accepted P2 controller fails
    closed (contract §5.3: dispatch above the budget -> P2 abort, matched to its breach by LOAD), through RECOVERY
    (FAIL_CLOSED_LANDING). Always recorded; it passes only on the path the run actually took."""
    result = run.result
    sub = Checks(c.codes)
    if result.get("final_state") == "RECOVERY" and result.get("failure"):
        fail_closed_landing(run, sub, explained)
        path = "FAIL_CLOSED_LANDING"
    else:
        l45(run, sub)
        path = "L45"
    c.record("LANDING", sub.out[path]["passed"], path=path, detail=sub.out[path])


# ---- C-5 load window (contract §5.3, D8; M6 decision 2026-11-14, plan §16.11) --------------------------------------
def _control_callbacks(run):
    import p6_run
    rows = [r for r in p6_run.jsonl(run.file("timing-trace.jsonl")) if r.get("operation") == "control_callback"]
    return sorted(rows, key=lambda r: r["start_wall_s"])


def load_window(run, callbacks=None):
    """The load window from the harness's own ledger, in the monotonic wall clock every P2-P6 record shares: its
    activation to its clearing, never past the declared window; with the simulation times of the first and last
    controller callbacks inside it (the window ends with the run when the load outlives the stack)."""
    records = run.fault_ledger("load_harness")
    activated = next((r for r in records if r["kind"] == "activated"), None)
    if activated is None or activated.get("start_wall_s") is None:
        return None
    cleared = next((r for r in records if r["kind"] == "cleared"), None)
    begin = activated["start_wall_s"]
    end = begin + activated["seconds"]
    if cleared is not None and cleared.get("end_wall_s") is not None:
        end = min(end, cleared["end_wall_s"])
    callbacks = _control_callbacks(run) if callbacks is None else callbacks
    inside = [r for r in callbacks if begin <= r["start_wall_s"] <= end]
    return {"wall_s": [begin, end], "sim_s": [inside[0]["sim_s"], inside[-1]["sim_s"]] if inside else None,
            "trigger_sim_s": activated.get("sim_s"), "workers": activated.get("workers"), "seconds": activated.get("seconds"),
            "cleared": cleared is not None, "stopped_early": (cleared or {}).get("stopped_early"), "callbacks": inside}


def load(run, c, limits, dispatch_budget_s):
    """§5.3: inside the declared load window a P4 perception or P5 inference breach is acceptable only if its declared
    response follows, with 0 unsafe commands (UNSAFE, a separate check):
    * dispatch above the P2 budget -> P2 abort. The accepted DispatchGuard measures from callback entry before every
      publication and once more after the last; a breach raises `stale_action` in that callback, so a P2 failure must be
      `stale_action`, inside the window, in a callback whose measured duration exceeds the budget. Callbacks over the
      budget without an abort passed the guard's last check (their excess is after it) and are reported;
    * inference above the P5 failure limit -> HOLD on that tick (the accepted timeout failure);
    * stale snapshot -> H35: every stale onset in the window hovers and holds, a `snapshot_stale` onset within the H35
      limit of the last usable snapshot.
    The P5 inference statistics (p99, max) must still hold on every invoked tick outside the window."""
    window = load_window(run)
    if window is None:
        return c.record("LOAD", False, error="no_load_activation")
    declared = (run.declaration or {}).get("parameters") or {}
    problems = []
    if (window["workers"], window["seconds"]) != (declared.get("load_workers"), declared.get("load_window_s")):
        problems.append("load_not_as_declared")
    begin, end = window["wall_s"]
    inside, sim = window.pop("callbacks"), window["sim_s"]
    durations = [r["duration_wall_s"] for r in inside]
    over = [r for r in inside if r["duration_wall_s"] > dispatch_budget_s]
    result = run.result
    failure = result.get("failure")
    abort = None if failure is None else next(
        (e for e in run.events if e.get("event") == "abort" and e.get("reason") == failure), None)  # the P2 abort event
    dispatch = {"budget_s": dispatch_budget_s, "callbacks": len(inside), "max_s": max(durations, default=None),
                "over_budget": len(over), "p2_failure": failure}
    if failure is not None:
        if failure != "stale_action":
            problems.append("p2_failure_not_a_dispatch_response:" + str(failure))
        elif abort is None or not begin <= abort["monotonic_s"] <= end:
            problems.append("p2_abort_outside_load_window")
        else:
            callback = next((r for r in inside if r["start_wall_s"] <= abort["monotonic_s"] <= r["end_wall_s"]), None)
            dispatch.update(abort_sim_s=abort["sim_s"], abort_callback_s=callback and callback["duration_wall_s"])
            if callback is None or callback["duration_wall_s"] <= dispatch_budget_s:
                problems.append("p2_abort_without_measured_overrun")
    dispatch["over_budget_without_abort_sim_s"] = [r["sim_s"] for r in over if abort is None or not
                                                   r["start_wall_s"] <= abort["monotonic_s"] <= r["end_wall_s"]][:10]

    def in_window(sim_s):
        return sim is not None and sim[0] <= sim_s <= sim[1]
    invoked = [r for r in run.ledger if r.get("invoked")]
    slow = [r for r in invoked if in_window(r["sim_s"]) and r["inference_s"] > limits["failure_wall_s"]]
    unanswered = [r["tick"] for r in slow if not (r.get("failure") == "timeout" and r.get("action") == "HOLD")]
    outside = sorted(r["inference_s"] for r in invoked if not in_window(r["sim_s"]))
    p99 = outside[min(len(outside) - 1, int(0.99 * len(outside)))] if outside else None
    outside_ok = not outside or (p99 <= limits["inference_p99_s"] and outside[-1] <= limits["inference_max_s"])
    inference = {"in_window": len([r for r in invoked if in_window(r["sim_s"])]), "above_failure_limit": len(slow),
                 "without_hold": unanswered[:10], "outside_ticks": len(outside), "outside_p99_s": p99,
                 "outside_max_s": outside[-1] if outside else None}

    limit = c.codes["H35"]["hover_after_last_usable_sim_s"]
    ordered = sorted(run.decisions, key=lambda e: e["sim_s"])
    late, onsets, last_usable, previous = [], 0, None, None
    for e in ordered:
        if e.get("stale") and not (previous and previous.get("stale")) and in_window(e["sim_s"]):
            onsets += 1
            ok = e.get("state") == "HOVER" and e.get("action") == "HOLD"
            if e["stale"] == "snapshot_stale":
                ok = ok and last_usable is not None and e["sim_s"] <= last_usable + limit + 1e-9
            if not ok:
                late.append(e["sim_s"])
        if e.get("snapshot_acquisition_sim_s") is not None:
            last_usable = max(last_usable if last_usable is not None else e["snapshot_acquisition_sim_s"],
                              e["snapshot_acquisition_sim_s"])
        previous = e
    stale = {"onsets": onsets, "without_h35": late[:10]}
    ok = not problems and not unanswered and outside_ok and not late
    c.record("LOAD", ok, problems=problems, window=window, dispatch=dispatch, inference=inference, stale=stale)


LOAD_RECORDER_ITEMS = ("cleanup:sensor-adapter:nonzero_exit_3", "depth:adapter_result_not_passed",
                       "depth:raw_recorder_incomplete",
                       # M6 extension (operator 2026-11-14, "Adopt the extension", plan §16.12-16.13): the accepted P3
                       # stream evaluator judges delivery by the recorder's own index, so frames the adapter received
                       # but could not record count as undelivered when the overflow falls inside the sensor window.
                       "depth:source_to_ros_delivery_loss", "depth:ros_receipts_not_in_native_ledger")
DEPTH_RECORDER_FAILURES = ["adapter_result_not_passed", "raw_recorder_incomplete"]
DEPTH_DELIVERY_FAILURES = ["source_to_ros_delivery_loss", "ros_receipts_not_in_native_ledger"]


def _undelivered_matched(run, index, missing):
    """The accepted P3 delivery check's own computation (`p3_stream_eval`: the native source frames in the accepted
    sensor window, delivered only if their stamp is in the recorder's index). Every native frame it counts as
    undelivered must lie in a gap between two written receipts, and each gap must hold at least as many overflowed
    receipt ids as such frames, so each one is matched to a distinct receipt the adapter got but did not record (the
    gaps are disjoint). No written receipt in the window may lack a native frame."""
    import bisect
    from collections import Counter
    import p3_evaluate
    import p3_stream_eval
    window = run.summary.get("sensor_window_sim_s")
    if not window:
        return False, {"error": "no_sensor_window"}
    begin, end = window
    _, chosen = p3_stream_eval.evaluate_source(p3_stream_eval.source_ledger(run.path, "depth-headers.jsonl"), window,
                                               p3_evaluate.configs()[0], "depth")
    by_stamp = sorted(index, key=lambda r: r["acquisition_stamp_ns"])
    stamps = [r["acquisition_stamp_ns"] for r in by_stamp]
    indexed = set(stamps)
    undelivered = [r["stamp_ns"] for r in chosen if r["stamp_ns"] not in indexed]
    gaps, unmatched = Counter(), []
    for stamp in undelivered:
        k = bisect.bisect_left(stamps, stamp)
        if k == 0 or k == len(stamps):
            unmatched.append(stamp)
            continue
        gaps[(by_stamp[k - 1]["receipt_id"], by_stamp[k]["receipt_id"])] += 1
    short = [gap for gap, n in gaps.items() if sum(1 for i in missing if gap[0] < i < gap[1]) < n]
    in_window = {r["acquisition_stamp_ns"] for r in index if r["accepted"] and begin * 1e9 <= r["acquisition_stamp_ns"] <= end * 1e9}
    foreign = sorted(in_window - {r["stamp_ns"] for r in chosen})
    ok = not unmatched and not short and not foreign
    return ok, {"undelivered_by_index": len(undelivered), "gaps": len(gaps), "unmatched_ns": unmatched[:10],
                "gaps_short_of_overflowed_receipts": [list(g) for g in short[:10]], "receipts_not_native_ns": foreign[:10]}


def load_recorder(run):
    """The accepted P3 raw depth recorder's bounded queue can overflow under the declared load (75289deb C-5: 31 of 2539
    frames, all inside the window). Perception is unaffected; only frames go missing from the evidence recording.
    Returns None when the recorder did not overflow, else the proof (M6 decision 2026-11-14, plan §16.11):
    * the adapter's only error is `raw_recorder_overflow`; the recorder drained and fsynced with no error; its close
      marker equals the result; accepted == written == indexed; the missing receipts are exactly the overflow count;
    * every missing receipt lies inside the load window (its written neighbours do; a trailing one, the adapter's stop);
    * the adapter met every processing period and rejected nothing, and the accepted P5 depth-stream evaluation failed
      only on the two recorder items (delivery, rate, ages and health all passed on the same evidence), or, when the
      overflow fell inside the sensor window, on those plus its two delivery items, each undelivered frame matched to
      an overflowed receipt (`_undelivered_matched`; M6 extension, plan §16.13);
    * the adapter's exit is the designed one for that result (3), not forced and reaped."""
    import bisect
    import p6_run
    from p3_process_exit import classify
    adapter = run.optional("sensors/adapter-result.json") or {}
    if adapter.get("error") != "raw_recorder_overflow":
        return None
    recorder = adapter.get("recorder") or {}
    closed = run.optional("sensors/depth-recorder-closed.json")
    index = p6_run.jsonl(run.file("sensors/depth-index.jsonl"))
    accounting = adapter.get("accounting") or {}
    written = {r["receipt_id"]: r for r in index}
    ids = sorted(written)
    received = accounting.get("received") or 0
    missing = [i for i in range(1, received + 1) if i not in written]
    window = load_window(run)
    process = next((p for p in run.summary.get("processes") or [] if p.get("name") == "sensor-adapter"), {})
    stop = process.get("stop_requested_monotonic_s")
    outside = []
    for i in missing:
        k = bisect.bisect_left(ids, i)
        before = written[ids[k - 1]]["receipt_wall_s"] if k > 0 else None
        after = written[ids[k]]["receipt_wall_s"] if k < len(ids) else stop
        if window is None or before is None or after is None or not (
                window["wall_s"][0] <= before and after <= window["wall_s"][1]):
            outside.append(i)
    recorder_ok = (adapter.get("status") == "failed" and recorder.get("error") is None and recorder.get("drained") is True
                   and recorder.get("fsynced") is True and closed == recorder and (recorder.get("overflow") or 0) > 0
                   and recorder.get("accepted") == recorder.get("written") == len(index)
                   and len(missing) == recorder.get("overflow"))
    depth = ((run.p5_evaluation or {}).get("depth_stream") or {}).get("failures")
    delivery = None
    depth_ok = depth == DEPTH_RECORDER_FAILURES
    if depth == DEPTH_RECORDER_FAILURES + DEPTH_DELIVERY_FAILURES:
        depth_ok, delivery = _undelivered_matched(run, index, set(missing))
    processing_ok = accounting.get("missed_periods") == 0 and not accounting.get("rejected") and depth_ok
    exit_ok = process.get("exit_code") == 3 and classify(process) == ("failed", "nonzero_exit_3")
    ok = recorder_ok and processing_ok and exit_ok and not outside
    return {"passed": ok, "overflow": recorder.get("overflow"), "missing_receipts": len(missing),
            "missing_outside_window": outside[:10], "recorder_ok": recorder_ok, "processing_ok": processing_ok,
            "exit_ok": exit_ok, "p5_depth_failures": depth, "missed_periods": accounting.get("missed_periods"),
            "delivery": delivery, "window_wall_s": window and window["wall_s"]}


def load_recorder_check(run, c):
    """Recorded only when the recorder overflowed, so its exceptions can apply to nothing else."""
    proof = load_recorder(run)
    if proof is not None:
        c.record("LOAD_RECORDER", proof.pop("passed"), **proof)


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
