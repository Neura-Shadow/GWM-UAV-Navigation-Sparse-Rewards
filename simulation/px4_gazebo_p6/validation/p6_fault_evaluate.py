"""P6 fault-occurrence proof (contract C8, §7.2): did the declared fault really happen?

Each proof uses evidence the injector did not write: the accepted controller's ledgers and events,
the adapter's and perception's own records, the native source probe, the process table and the witness.
The injector's ledger only says *where* to look; a fault the ledger reports but the independent
evidence does not show is **not proved**, and the run fails (contract C8).
"""
SOURCE_GAP_SIM_S = 0.12  # P3 source-gap limit, the P4 §9.1 pre-window trim
CLOCK_STALL_WALL_S, TICK_S = 1.0, 0.05  # P2 clock stall (contract §3) and one 20 Hz control tick
GRAPH_PERIOD_WALL_S = 1.0  # the accepted controller re-checks the PX4-input graph every 1.0 s wall
SETTLE_S = 0.2  # relay windows: margin at each end for in-flight messages and per-process /clock views
PRESTREAM = "PRESTREAM_SAFE_SETPOINTS"  # the live ground trigger state (M5 decision, `ground_prestream`)
POLICY_REASON = {"F3.04": "output_not_finite", "F3.05": "output_shape", "F3.06": "selection_outside_mask",
                 "F3.07": "timeout", "F3.08": "timeout", "F3.09": "timeout", "F3.10": "timeout",
                 "F3.11": "exception:RuntimeError", "F3.12": "input_not_finite", "F7.02": "exception:RuntimeError",
                 "F8.01": "timeout"}
FAMILY = {**{f: "policy" for f in POLICY_REASON}, "F3.13": "adversarial",
          "F4.01": "map_drop", "F4.02": "map_delay", "F8.03": "map_drop", "F4.06": "health_drop",
          "F2.09": "pose_to_perception", "F6.03": "tick_delay",
          "F1.01": "bridge_stop", "F1.02": "adapter_stop", "F1.03": "depth_drop", "F1.04": "depth_delay", "F1.06": "depth_counters",
          "F1.07": "depth_counters", "F1.12": "depth_counters", "F1.11": "calibration",
          "F3.01": "refusal", "F3.02": "refusal", "F3.03": "refusal", "F4.07": "competing_publisher",
          "F4.13": "clock_stall"}
REFUSAL = {"F3.01": "checkpoint_missing", "F3.02": "checkpoint_sha256_mismatch",
           "F3.03": "p5_model_schema_differs_from_frozen_hash"}
COUNTER = {"F1.06": {"duplicate_acquisition_stamp": "duplicate", "acquisition_stamp_backwards": "backwards",
                     "missing_acquisition_stamp": "missing"},
           "F1.07": {"dimension_mismatch": "frames"}, "F1.12": {"unexpected_raw_frame_id": None}}


class Proof:
    def __init__(self, fault, family):
        self.out = {"fault": fault, "family": family, "evidence": {}, "failures": []}

    def check(self, ok, name):
        if not ok:
            self.out["failures"].append("proof:" + name)

    def evidence(self, **values):
        self.out["evidence"].update(values)

    def done(self):
        self.out["proved"] = not self.out["failures"]
        return self.out


def affected(run, injector, item=None):
    return [r for r in run.fault_ledger(injector) if r["kind"] == "affected" and (item is None or r.get("item") == item)]


def activation(run, injector):
    return next((r for r in run.fault_ledger(injector) if r["kind"] == "activated"), None)


def prove(run, declaration, fault_window=None):
    """`fault_window` is the accepted P4 §9.1 window output (with its failure items), when the orchestrator ran it."""
    fault = declaration["fault"]
    if fault is None:
        return prove_no_fault(run, declaration)
    family = FAMILY.get(fault)
    p = Proof(fault, family)
    if family is None:
        p.check(False, "no_independent_proof_defined_for_" + fault)
        return p.done()
    globals()["_" + family](run, declaration, p, fault_window)
    return p.done()


def prove_no_fault(run, declaration):
    """Pass-through rows: no injector may report any activation or affected item."""
    p = Proof(None, "pass_through")
    ledgers = run.fault_ledgers()
    active = {k: [r["kind"] for r in v if r["kind"] in ("activated", "affected")] for k, v in ledgers.items()}
    p.evidence(injectors=sorted(ledgers), modes={k: next((r.get("mode") for r in v if r["kind"] == "armed"), None)
                                                 for k, v in ledgers.items()})
    p.check(not any(active.values()), "pass_through_injector_acted")
    p.check(all(m == "pass-through" for m in p.out["evidence"]["modes"].values()), "injector_not_in_pass_through")
    return p.done()


# ---- policy path (accepted policy ledger) ---------------------------------------------------------------
def _policy(run, d, p, _):
    """Failure records in the accepted ledger, each matching a shim-affected invocation at the same tick time."""
    fault, params = d["fault"], d["parameters"]
    expected = POLICY_REASON[fault]
    shim = affected(run, "controller_shim", "policy_invocation")
    act = activation(run, "controller_shim")
    failing = [r for r in run.ledger if r.get("invoked") and r.get("failure") is not None]
    p.evidence(declared_trigger_sim_s=act and act["sim_s"], shim_affected=len(shim), ledger_failures=len(failing),
               ledger_failure_reasons=sorted({r["failure"] for r in failing}))
    p.check(act is not None, "no_activation")
    p.check(bool(failing), "accepted_ledger_shows_no_policy_failure")
    p.check(all(r["failure"] == expected for r in failing), "unexpected_failure_reason")
    p.check(sorted(round(r["sim_s"], 6) for r in failing) == sorted(round(r["sim_s"], 6) for r in shim),
            "ledger_failures_differ_from_declared_invocations")
    if act is not None:
        p.check(all(r["sim_s"] > act["sim_s"] for r in failing), "failure_before_trigger")
    if expected == "timeout":
        seconds = params.get("sleep_s", params.get("block_s"))
        p.check(all(r["inference_s"] >= seconds for r in failing), "timeout_shorter_than_declared_delay")
    count = params.get("consecutive_invocations")
    if count is not None:
        p.check(len(failing) == count, "failure_count_differs_from_declaration")
    offsets = params.get("invocation_offsets_sim_s")
    if offsets is not None:
        p.check(len(failing) == len(offsets), "isolated_failure_count_differs")


def _adversarial(run, d, p, _):
    """F3.13: from the trigger on, every recorded choice is the minimum score among allowed slots."""
    import numpy as np
    act = activation(run, "controller_shim")
    p.check(act is not None, "no_activation")
    records = [r for r in run.ledger if r.get("invoked") and r.get("failure") is None and act and r["sim_s"] > act["sim_s"]]
    worst = 0
    for r in records:
        allowed, scores = np.array(r["allowed"], dtype=bool), np.array(r["scores"], dtype=np.float64)
        worst += int(allowed[r["choice_slot"]] and scores[r["choice_slot"]] <= scores[allowed].min() + 1e-9)
    p.evidence(invocations_after_trigger=len(records), worst_allowed_choices=worst)
    p.check(records and worst == len(records), "choices_not_the_declared_adversary")


# ---- map and health inputs (perception's and the adapter's own records vs the controller) ----------------
def _used_snapshots(run):
    return {round(e["snapshot_acquisition_sim_s"], 6) for e in run.decisions if e.get("snapshot_acquisition_sim_s") is not None}


def _ground_controller_input(run, p):
    """On the ground the controller records no decisions, so a fault in its own map or health input has no
    record outside the injector: it cannot be proved independently (contract C8) and must not pass."""
    if not run.decisions:
        p.check(False, "no_independent_ground_evidence_for_a_controller_input_fault")
        return True
    return False


def _map_drop(run, d, p, _):
    if _ground_controller_input(run, p):
        return
    act = activation(run, "controller_shim")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    window = d["parameters"]["drop_window_sim_s"]
    published = [round(s["acquisition_sim_s"], 6) for s in run.local_map
                 if s.get("usable") and act["sim_s"] < s["processing_sim_s"] < act["sim_s"] + window]
    used = _used_snapshots(run)
    p.evidence(published_in_window=len(published), used_from_window=len(set(published) & used))
    p.check(bool(published), "perception_published_nothing_in_window")  # the source was alive: the drop is the controller's input
    p.check(not set(published) & used, "controller_used_a_dropped_snapshot")


def _map_delay(run, d, p, _):
    if _ground_controller_input(run, p):
        return
    act = activation(run, "controller_shim")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    delay = d["parameters"]["delay_s"]
    window = d["parameters"].get("window_sim_s", 1e9)
    late = [e for e in run.decisions if act["sim_s"] + delay < e["sim_s"] < act["sim_s"] + window
            and e.get("snapshot_acquisition_sim_s") is not None]
    ages = [e["sim_s"] - e["snapshot_acquisition_sim_s"] for e in late]
    p.evidence(decisions_in_window=len(late), min_age_s=min(ages, default=None))
    p.check(ages and min(ages) >= delay - 0.05, "controller_snapshots_not_delayed")


def _health_drop(run, d, p, _):
    if _ground_controller_input(run, p):
        return
    act = activation(run, "controller_shim")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    window = d["parameters"]["window_sim_s"]
    adapter = [h for h in run.health if act["sim_s"] < (h.get("sim_s") or -1) < act["sim_s"] + window and h["status"] == "fresh"]
    detected = [e for e in run.decisions if e.get("stale") == "adapter_health_missing"]
    p.evidence(adapter_fresh_records_in_window=len(adapter), controller_missing_decisions=len(detected))
    p.check(bool(adapter), "adapter_health_itself_stopped")  # the adapter kept publishing fresh health
    p.check(bool(detected), "controller_never_missed_health")


# ---- pose to perception (witness + perception's own result) ------------------------------------------------
def _pose_to_perception(run, d, p, _):
    act = activation(run, "state_relay")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    t = act["sim_s"]
    # The relay withholds on the half-open window [t, t + window) of `schedule.Windows`; a message it releases at the
    # boundary is correct. Both ends get the same settle margin, for messages in flight and for the relay's and the
    # witness's separate /clock views (M5: one attitude message at exactly t + 5.000 s was counted as "continued").
    end = t + d["parameters"].get("window_sim_s", float("inf"))
    inside = lambda w: t + SETTLE_S < (w.get("receipt_sim_s") or 0) < end - SETTLE_S  # noqa: E731
    for kind in ("position", "attitude"):
        true = [w for w in run.witness("true_" + kind) if inside(w)]
        relayed = [w for w in run.witness("relayed_" + kind) if inside(w)]
        p.evidence(**{kind + "_true_in_window": len(true), kind + "_relayed_in_window": len(relayed)})
        p.check(bool(true), "true_%s_stopped_too" % kind)
        p.check(not relayed, "relayed_%s_continued" % kind)
    # Perception's own per-snapshot record, inside the window (start-up association misses do not count).
    unassociated = [s for s in run.local_map if t + SETTLE_S < s["processing_sim_s"] < end - SETTLE_S
                    and str(s.get("reason", "")).startswith("association:")]
    p.evidence(perception_unassociated_in_window=len(unassociated),
               reasons=sorted({s["reason"] for s in unassociated}))
    p.check(bool(unassociated), "perception_never_unassociated_in_window")


# ---- control tick (accepted timing trace and controller failure) ---------------------------------------------
def _tick_delay(run, d, p, _):
    act = activation(run, "controller_shim")
    p.check(act is not None, "no_activation")
    delay = d["parameters"]["mission_tick_delay_s"]
    failure = run.result.get("failure")
    p.evidence(controller_failure=failure)
    p.check(failure == "stale_action", "controller_failure_not_dispatch_budget")
    trace = [] if not run.exists("timing-trace.jsonl") else __import__("p6_run").jsonl(run.file("timing-trace.jsonl"))
    long = [s for s in trace if s.get("operation") == "mission_tick" and (s.get("duration_wall_s") or 0) >= delay]
    p.evidence(long_mission_ticks=len(long))
    p.check(len(long) == 1, "mission_tick_delay_not_seen_exactly_once_in_the_accepted_trace")


# ---- depth bridge and relay (native source, adapter records, witness) -----------------------------------------
def _bridge_stop(run, d, p, fault_window):
    """F1: the unchanged P4 §9.1 F1 checks (native source continued, ROS delivery stopped, bridge exit)."""
    p.check(fault_window is not None, "p4_fault_window_not_evaluated")
    if fault_window is None:
        return
    errors = [x for x in fault_window["failures"] if x.startswith("fault_window_error")]
    p.check(not errors, "p4_fault_window_error")  # a window that could not be evaluated proves nothing
    p.check(bool(fault_window.get("native_headers_after_injection")), "no_native_source_evidence_after_injection")
    f1 = [x for x in fault_window["failures"] if x.startswith("s6:f1_") or x == "s6:injection_record_missing"]
    p.evidence(t_inj_sim_s=fault_window.get("t_inj_sim_s"), native_headers_after=fault_window.get("native_headers_after_injection"))
    for item in f1:
        p.check(False, item)


def _adapter_stop(run, d, p, _):
    """F1.02: the adapter's own process record and health stream end at the requested stop, while the native
    source (the probe, not the adapter) keeps producing frames. The unchanged P4 injection record names it."""
    injection = run.optional("p4/fault_injection.json")
    p.check(injection is not None, "injection_record_missing")
    if injection is None:
        return
    adapter = _process(run, "sensor-adapter") or {}
    exited = adapter.get("exit_observed_monotonic_s")
    late = [h for h in run.health if exited is not None and (h.get("wall_s") or 0) > exited]
    headers = [r for r in __import__("p6_run").jsonl(run.file("source/depth-headers.jsonl"))
               if r["stamp_ns"] / 1e9 > injection["t_inj_sim_s"]]
    p.evidence(adapter_exit_code=adapter.get("exit_code"), health_after_exit=len(late), native_headers_after=len(headers))
    p.check(injection.get("process") == "sensor-adapter" and injection.get("signal") == "SIGINT", "wrong_injection")
    p.check(adapter.get("stop_requested_monotonic_s") == injection["request_monotonic_s"] and adapter.get("exit_code") == 0
            and exited is not None, "adapter_exit")
    p.check(not late, "adapter_health_after_exit")
    p.check(bool(headers), "native_source_stopped_too")


def _relay_pairs(run, d):
    raw = {w["stamp_ns"]: w for w in run.witness("raw_image")}
    relayed = {}
    for w in run.witness("relayed_image"):
        relayed.setdefault(w["crc32"], []).append(w)
    return raw, relayed


def _depth_drop(run, d, p, _):
    act = activation(run, "depth_relay")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    from injection.schedule import Windows
    windows = Windows(d["parameters"]["burst_start_offsets_sim_s"], d["parameters"]["burst_durations_sim_s"])
    raw = run.witness("raw_image")
    relayed = {w["stamp_ns"] for w in run.witness("relayed_image")}
    inside = [w for w in raw if windows.active((w["receipt_sim_s"] or 0) - act["sim_s"])]
    outside = [w for w in raw if not windows.active((w["receipt_sim_s"] or 0) - act["sim_s"])]
    p.evidence(raw_inside=len(inside), relayed_inside=sum(w["stamp_ns"] in relayed for w in inside),
               raw_outside_missing=sum(w["stamp_ns"] not in relayed for w in outside))
    p.check(bool(inside) and not any(w["stamp_ns"] in relayed for w in inside), "frames_in_window_not_dropped")
    p.check(all(w["stamp_ns"] in relayed for w in outside), "frames_outside_window_dropped")


def _depth_delay(run, d, p, _):
    act = activation(run, "depth_relay")
    p.check(act is not None, "no_activation")
    if act is None:
        return
    delay = d["parameters"]["delay_s"]
    end = act["sim_s"] + d["parameters"].get("window_sim_s", float("inf"))  # frames after the window are not delayed
    raw = {w["stamp_ns"]: w["receipt_sim_s"] for w in run.witness("raw_image")}
    lags = [w["receipt_sim_s"] - raw[w["stamp_ns"]] for w in run.witness("relayed_image")
            if w["stamp_ns"] in raw and act["sim_s"] + 0.05 < raw[w["stamp_ns"]] <= end - delay]
    p.evidence(delayed_frames=len(lags), min_lag_s=min(lags, default=None))
    p.check(lags and min(lags) >= delay - 0.05, "relayed_frames_not_delayed")


def _depth_counters(run, d, p, _):
    """The accepted adapter's own rejection counters equal the declared counts."""
    adapter = run.optional("sensors/adapter-result.json") or {}
    rejected = (adapter.get("accounting") or {}).get("rejected") or {}
    expected = {}
    for reason, key in COUNTER[d["fault"]].items():
        expected[reason] = d["parameters"][key] if key else None
    p.evidence(adapter_rejected=rejected, declared=expected)
    for reason, count in expected.items():
        p.check(rejected.get(reason, 0) > 0 if count is None else rejected.get(reason) == count, "adapter_counter:" + reason)


def _calibration(run, d, p, _):
    adapter = run.optional("sensors/adapter-result.json") or {}
    reason = adapter.get("calibration_invalid_reason")
    p.evidence(adapter_calibration_invalid_reason=reason)
    p.check(bool(reason), "adapter_calibration_never_invalidated")


# ---- start-up refusal, ownership and clock -----------------------------------------------------------------
def _refusal(run, d, p, _):
    log = run.file("ros-controller.log").read_text(encoding="utf-8", errors="replace") if run.exists("ros-controller.log") else ""
    p.evidence(refusal_in_log=REFUSAL[d["fault"]] in log, ros_events_created=run.exists("ros-events.jsonl"))
    p.check(REFUSAL[d["fault"]] in log, "declared_refusal_not_in_controller_log")
    p.check(not run.exists("ros-events.jsonl"), "controller_started_before_refusing")


def _abort_event(run, prefix):
    return next((e for e in run.events if e.get("event") == "abort" and str(e.get("reason", "")).startswith(prefix)), None)


def _process(run, name):
    return next((x for x in run.summary.get("processes") or [] if x["name"] == name), None)


def _designed_exit(run, p):
    """The accepted `p5_control` exits 0, or 1 when its connectivity check had not passed when it failed closed
    (`main`: ok = connectivity passed and ...). Any other exit is not the designed fail-closed exit."""
    code, connectivity = run.summary.get("controller_exit_code"), run.result.get("connectivity")
    p.evidence(controller_exit_code=code, connectivity=connectivity)
    p.check(code == 0 or (code == 1 and connectivity == "failed"), "controller_exit_not_the_designed_fail_closed_exit")


def _after_prestream(run, p):
    """M5 decision ("Adopt both"): the live ground faults are injected at the controller's own transition into
    PRESTREAM_SAFE_SETPOINTS (its ros-events.jsonl), when its estimate is valid and its connectivity check has passed;
    the runner's injection request (CLOCK_MONOTONIC, as the controller's events) must not precede it."""
    entered = next((e.get("monotonic_s") for e in run.events
                    if e.get("event") == "transition" and e.get("to") == PRESTREAM), None)
    requested = (run.injection or {}).get("request_monotonic_s")
    p.evidence(prestream_entered_monotonic_s=entered, injection_requested_monotonic_s=requested)
    p.check(entered is not None and requested is not None and entered <= requested, "fault_not_injected_after_prestream")
    p.check(run.result.get("connectivity") == "passed", "connectivity_not_passed_before_the_fault")


def _designed_timing(run, p):
    """M5 decision ("Adopt both"): the accepted P2 timing check (`timing_evidence.trace_metrics`) reports failed
    whenever the controller's result carries a failure, and raises on any reconciliation mismatch, which the P5
    evaluator records as `p2_integrity_error:` without a timing status. So `p2:timing` is the designed consequence of
    the fail-closed detection only when the timing status exists, no integrity error was recorded, and the
    controller's failure (the timing report's `controller_failure`) is the declared detection, which the caller checks."""
    p5 = run.p5_evaluation or {}
    status = p5.get("p2_timing_status")
    errors = [x for x in p5.get("failures") or [] if str(x).startswith("p2_integrity_error")]
    p.evidence(p2_timing_status=status, p2_integrity_errors=errors)
    p.check(status in ("passed", "failed") and not errors, "p2_timing_reconciliation_not_completed")


def _competing_publisher(run, d, p, _):
    """F4.07. The accepted controller raises inside its graph check before it rewrites `ros-graph.json`, so that
    file keeps the previous (clean) check (M5 finding). The proof is the controller's own fail-closed reason
    naming the declared topic, inside the advertiser's lifetime from the runner's process table, and an
    advertiser that ended cleanly having published nothing (it has no publish call, M2 test)."""
    topic = d["parameters"]["topic"]
    failure = str(run.result.get("failure", ""))
    abort = _abort_event(run, "competing_control_publisher")
    advertiser = _process(run, "p6-competing-publisher")
    finished = next((r for r in run.fault_ledger("competing_publisher") if r["kind"] == "finished"), None)
    alive = advertiser is not None and abort is not None and (
        advertiser["launched_monotonic_s"] <= abort["monotonic_s"] <= (advertiser.get("exit_observed_monotonic_s") or float("inf")))
    p.evidence(controller_failure=failure, abort_monotonic_s=abort and abort["monotonic_s"],
               advertiser=advertiser and {k: advertiser.get(k) for k in ("launched_monotonic_s", "exit_observed_monotonic_s", "exit_code")},
               detection_after_advertise_wall_s=(abort["monotonic_s"] - advertiser["launched_monotonic_s"]) if alive else None)
    p.check(failure == "competing_control_publisher:" + topic, "controller_did_not_detect_the_declared_topic")
    _designed_exit(run, p)
    _after_prestream(run, p)
    _designed_timing(run, p)
    p.check(alive, "detection_outside_the_advertiser_lifetime")
    p.check(advertiser is not None and advertiser.get("exit_code") == 0, "advertiser_process_record")
    p.check(finished is not None and finished.get("messages_published") == 0, "advertiser_published")
    # "within 1.0 s wall plus graph query": from the advertisement (the advertiser's own wall record, CLOCK_MONOTONIC)
    # to the abort, at most the 1.0 s graph-check period, one tick and the longest accepted graph-check span.
    act = activation(run, "competing_publisher")
    trace = [] if not run.exists("timing-trace.jsonl") else __import__("p6_run").jsonl(run.file("timing-trace.jsonl"))
    query = max((s.get("duration_wall_s") or 0.0 for s in trace if s.get("operation") == "graph_check"), default=0.0)
    if act is not None and abort is not None and act.get("advertised_wall_s") is not None:
        latency = abort["monotonic_s"] - act["advertised_wall_s"]
        p.evidence(detection_after_advertisement_wall_s=latency, longest_graph_check_wall_s=query)
        p.check(0.0 <= latency <= GRAPH_PERIOD_WALL_S + TICK_S + query + 1e-9, "detection_later_than_1_0_s_plus_graph_query")
    else:
        p.check(False, "no_advertisement_time")


def _clock_stall(run, d, p, _):
    """F4.13: the kernel showed the clock bridge stopped and resumed (`/proc`, read by the process-signal
    injector), and the accepted controller aborted `clock_stalled` while it was stopped, within 1.0 s wall plus
    one 20 Hz tick of the stop request (the last clock receipt can only be earlier, so this is conservative)."""
    failure = str(run.result.get("failure", ""))
    abort = _abort_event(run, "clock_stalled")
    act = activation(run, "process_signal")
    p.evidence(controller_failure=failure, activation=act, abort_monotonic_s=abort and abort["monotonic_s"])
    p.check(failure == "clock_stalled", "controller_did_not_report_clock_stalled")
    _designed_exit(run, p)
    _after_prestream(run, p)
    _designed_timing(run, p)
    p.check(act is not None and act.get("signal") == "SIGSTOP" and act.get("proved") is True, "no_proc_evidence_of_the_stop")
    if act is None or abort is None or not act.get("proved"):
        return
    latency = abort["monotonic_s"] - act["request_wall_s"]
    p.evidence(detection_wall_s=latency)
    p.check(act["stopped_seen_wall_s"] <= abort["monotonic_s"] <= act["resumed_seen_wall_s"], "abort_not_while_stopped")
    p.check(latency <= CLOCK_STALL_WALL_S + TICK_S + 1e-9, "clock_stall_detection_later_than_1_0_s_plus_one_tick")
