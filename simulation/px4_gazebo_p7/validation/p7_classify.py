"""P7 flight classifier (contract §5 with D13/D14, §7.1; analysis plan `validity` and `outcome`). Offline only.

**Inputs.** It reads one finalized P4 or P5 run after the unchanged accepted evaluator has written
`p4-evaluation.json` or `p5-evaluation.json`, and it trusts no self-report alone:
* the run's summary and controller result;
* the accepted evaluation's items and outputs;
* the ULog (arming state, through the accepted P2 reader);
* the ground-truth track (through the accepted P4 function);
* the decision events and the P5 policy ledger.

**What it decides:**
* **Validity:**
  * valid = armed evidence: the ULog shows ARMED at least once, or the P2 controller entered TAKEOFF;
  * a disagreement between the two is an integrity event;
  * no armed evidence = infrastructure-invalid.
* **Classes (D14),** from the accepted evaluator's failure items:
  * **Safety events:** collision, truth clearance, a setpoint inside an inflated obstacle, Policy A, the command
    sequence, a competing publisher, not landed and disarmed; plus the two P6 §6.3 rules restated below.
  * **Safety-unverified:** truth poses missing, the truth-pose interval, a truth-observer error, a missing contact
    ledger, or no usable safety evaluation.
  * **Mission failures:** tracking, response latency.
  * **Integrity events:** everything else.
  * **For a flight that never armed,** only Policy A and a competing publisher apply, since nothing moved.
  * **D15, when the accepted P2 integrity check raises** (for example on the stale state a P2 observation-gap
    RECOVERY leaves). Its safety facts are then read from the ULog with the accepted P2 reader: own commands exactly
    176, 400, 21, landed and disarmed, and no PX4 failsafe. The accepted Policy A check must also have run.
    * If they all hold, the error is an integrity event.
    * A failed fact is a safety event.
    * An unreadable ULog leaves the flight safety-unverified.
* **The restated P6 §6.3 rules.** They use the same rule and threshold as the accepted P6 functions, applied to P7
  runs:
  * **hold-tick motion:** on a tick that must hold (a stale decision, or a P5 policy-failure tick), the next
    decision's target moved more than H35's 0.01 m;
  * **unrevalidated resume:** a HOVER -> AVOIDING transition without a fresh (age <= 0.25 s), non-stale decision at
    that instant. A flight with no such return is not a failure.
* **Goal success:** `goal_reached_sim_s` recorded, `p4_outcome` completed, final state COMPLETE, landed and disarmed,
  final goal distance <= `goal_tolerance_m` (as the accepted `outcome()` measures it), no safety event, safety
  verified, no mission failure.
* **Stop reasons:** every safety event and every safety-unverified item.

**Running it.** `p7_classify.py RUN` runs inside `verify_p7_evidence.sh`, the accepted evaluators' environment with
`venv-px4`, which holds `pyulog`. It writes `p7/flight-record.json` with exclusive create. `classify(run)`, the
sequencer's hook, runs that script and returns the record.
"""
import json
import math
from pathlib import Path
import subprocess
import sys

P7 = Path(__file__).resolve().parents[1]
REPO = P7.parents[1]
SIM, P4, P5, P6 = (REPO / "simulation" / n for n in ("px4_gazebo", "px4_gazebo_p4", "px4_gazebo_p5", "px4_gazebo_p6"))
for _path in (SIM / "validation", SIM / "ros2_ws/src/gwm_px4_control", SIM / "ros2_ws/src/gwm_sensor_adapter",
              P4 / "ros2_ws/src/gwm_obstacle_avoidance", P4 / "validation", P5 / "validation"):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))
from p3_provenance import atomic_json, digest, strict_json, strict_jsonl  # noqa: E402

CONTRACT = "p7-flight-record-v1"
WRAPPER = P7 / "scripts/verify_p7_evidence.sh"
EVALUATIONS = {"p4_control": "p4-evaluation.json", "p5_control": "p5-evaluation.json"}
SAFETY_EVENT_ITEMS = ("safety:collision_contact", "safety:truth_clearance", "safety:setpoint_inside_inflated_obstacle",
                      "p2:command_sequence", "competing_flight_publisher", "p2:final_landed_disarmed")
SAFETY_EVENT_PREFIXES = ("native_commands:",)
UNVERIFIED_ITEMS = ("safety:truth_poses_missing", "safety:truth_pose_interval", "safety:truth_observer_error")
UNVERIFIED_PREFIXES = ("evaluator_incomplete_evidence:", "p2_integrity_error:", "native_command_evaluation_error:",
                       "p4_evaluation_error:", "p5_evaluation_error:")
MISSION_ITEMS = ("safety:tracking", "safety:response_latency")
NEVER_ARMED_STOPS = ("competing_flight_publisher",)
P2_ERROR_PREFIX = "p2_integrity_error:"
P2_COMMANDS = [176, 400, 21]  # the accepted P2 sequence (p4_evaluate p2_integrity: set mode, arm, land)
HOLD_MOTION_M = strict_json(P6 / "configs/p6_faults.yaml")["codes"]["H35"]["stale_target_motion_m"]  # 0.01 m


# ---- evidence ---------------------------------------------------------------------------------------------------
def events(run):
    path = Path(run) / "ros-events.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.exists() else []


def p2_ulog_facts(summary):
    """D15: the safety-relevant P2 facts from the ULog with the accepted P2 reader, or None if it cannot be read."""
    files, config = summary.get("ulog_files") or [], summary.get("config") or {}
    if not files or not Path(files[0]["path"]).exists() or "source_system" not in config:
        return None
    try:
        import collect_p2_evidence as p2
        _, streams = p2.read_ulog(files[0]["path"], flight=True)
        own = [r["command"] for r in streams["vehicle_command"]
               if r["source_system"] == config["source_system"] and r["source_component"] == config["source_component"]]
        status, land = streams["vehicle_status"], streams["vehicle_land_detected"]
        return {"own_commands": own, "final_disarmed": status[-1]["arming_state"] == 1,
                "final_landed": bool(land[-1]["landed"]), "failsafe_observed": any(s["failsafe"] for s in status)}
    except Exception:
        return None


def p2_ulog_failures(facts):
    """D15: the P2 safety facts the accepted P2 integrity check could not establish, as safety events."""
    failures = []
    if facts["own_commands"] != P2_COMMANDS:
        failures.append("p2_ulog:command_sequence")
    if not (facts["final_landed"] and facts["final_disarmed"]):
        failures.append("p2_ulog:final_landed_disarmed")
    if facts["failsafe_observed"]:
        failures.append("p2_ulog:failsafe_observed")
    return failures


def ulog_armed(summary):
    """True/False from the ULog vehicle_status (ARMED = 2); None if there is no readable ULog."""
    files = summary.get("ulog_files") or []
    if not files or not Path(files[0]["path"]).exists():
        return None
    try:
        import collect_p2_evidence as p2
        _, streams = p2.read_ulog(files[0]["path"], flight=False)
    except Exception:
        return None
    return any(row.get("arming_state") == 2 for row in streams.get("vehicle_status", []))


# ---- the restated P6 §6.3 rules -------------------------------------------------------------------------------
def hold_tick_motion(decisions, ledger, limit=HOLD_MOTION_M):
    """P6 `unsafe_setpoints`: a hold tick's motion is the next decision's target (the event logs the tick's start)."""
    hold = {round(e["sim_s"], 6) for e in decisions if e.get("stale")}
    hold |= {round(r["sim_s"], 6) for r in ledger if r.get("failure") is not None}
    ordered = sorted(decisions, key=lambda e: e["sim_s"])
    return [e["sim_s"] for e, after in zip(ordered, ordered[1:])
            if round(e["sim_s"], 6) in hold and math.dist(e["target_enu"], after["target_enu"]) > limit]


def unrevalidated_resumes(transitions, decisions, max_age):
    """P6 `recovery` (§6.5): each HOVER -> AVOIDING return must happen on a fresh, non-stale decision."""
    at = {round(e["sim_s"], 6): e for e in decisions}
    bad = []
    for t in transitions:
        if t.get("from") == "HOVER" and t.get("to") == "AVOIDING":
            e = at.get(round(t["sim_s"], 6))
            if e is None or e.get("stale") or e.get("snapshot_acquisition_sim_s") is None \
                    or e["sim_s"] - e["snapshot_acquisition_sim_s"] > max_age:
                bad.append(t["sim_s"])
    return bad


# ---- classes ------------------------------------------------------------------------------------------------------
def sort_items(failures):
    classes = {"safety_events": [], "safety_unverified": [], "mission_failures": [], "integrity_events": []}
    for item in failures:
        if item in SAFETY_EVENT_ITEMS or item.startswith(SAFETY_EVENT_PREFIXES):
            classes["safety_events"].append(item)
        elif item in UNVERIFIED_ITEMS or item.startswith(UNVERIFIED_PREFIXES):
            classes["safety_unverified"].append(item)
        elif item in MISSION_ITEMS:
            classes["mission_failures"].append(item)
        else:
            classes["integrity_events"].append(item)
    return classes


def hover_time(transitions, end_sim):
    total, entered = 0.0, None
    for t in sorted(transitions, key=lambda t: t["sim_s"]):
        if entered is not None:
            total += t["sim_s"] - entered
            entered = None
        if t.get("to") == "HOVER":
            entered = t["sim_s"]
    if entered is not None and end_sim is not None:
        total += max(0.0, end_sim - entered)
    return total


def final_goal_distance(run, result, evaluation, mission):
    """As the accepted `p4_evaluate.outcome()`: the last truth position against the takeoff origin + the goal."""
    import p4_evaluate
    offset = (evaluation.get("safety") or {}).get("local_to_world_offset_m")
    origin = (result.get("p4") or {}).get("origin_enu")
    track = p4_evaluate.truth_track(run)
    if offset is None or origin is None or not track:
        return None
    goal = (offset[0] + origin[0] + mission["goal_enu_m"][0], offset[1] + origin[1] + mission["goal_enu_m"][1])
    return math.dist(track[-1][1][:2], goal)


def build_record(run, ulog_reader=ulog_armed, p2_facts_reader=p2_ulog_facts):
    run = Path(run)
    summary = strict_json(run / "summary.json")
    result = summary.get("controller_result") or {}
    controller = result.get("controller") or ("p5_control" if summary.get("p5_scope") else "p4_control")
    name = EVALUATIONS[controller]
    evaluation = strict_json(run / name) if (run / name).exists() else None
    config = json.loads((run / "p4_avoidance.yaml").read_text())
    p2_states = [t["to"] for t in result.get("transitions", [])]
    armed_ulog = ulog_reader(summary)
    takeoff = "TAKEOFF" in p2_states
    valid = armed_ulog is True or takeoff
    failures = list((evaluation or {}).get("failures", []))
    classes = sort_items(failures)
    if evaluation is None:
        classes["safety_unverified"].append("evaluation_missing")
    if armed_ulog is not None and armed_ulog != takeoff:
        classes["integrity_events"].append("armed_evidence_disagreement")
    record = {"contract": CONTRACT, "run": run.name, "controller": controller[:2],
              "item": ((summary.get("p7") or {}).get("item") or (summary.get("scenario") or {}).get("scenario")),
              "p7": {k: (summary.get("p7") or {}).get(k) for k in ("scope", "block", "slot", "dry_run")},
              "evaluation": {"file": name, "sha256": digest(run / name) if evaluation else None,
                             "status": (evaluation or {}).get("status"), "failures": failures},
              "armed_evidence": {"ulog_armed": armed_ulog, "takeoff_transition": takeoff},
              "validity": "valid" if valid else "infrastructure_invalid",
              "classifier_sha256": digest(Path(__file__)),
              "inputs": {"summary_sha256": digest(run / "summary.json"),
                         "layout_record_sha256": digest(run / "p7/layout.json") if (run / "p7/layout.json").exists() else None}}
    if not valid:
        stops = [i for i in classes["safety_events"] if i.startswith(SAFETY_EVENT_PREFIXES) or i in NEVER_ARMED_STOPS]
        record.update(outcome=None, classes=classes, stop_reasons=["safety_event:" + i for i in stops],
                      unsafe_rules=None, goal_conditions=None, metrics=None)
        return record
    decisions = [e for e in events(run) if e.get("event") == "p4_decision"]
    ledger_path = run / "p5/policy-ledger.jsonl"
    ledger = strict_jsonl(ledger_path) if ledger_path.exists() else []
    supervisor = (result.get("p4") or {}).get("supervisor_transitions", [])
    rules = {"hold_tick_motion": hold_tick_motion(decisions, ledger),
             "unrevalidated_resumes": unrevalidated_resumes(supervisor, decisions, config["perception"]["snapshot_max_age_sim_s"])}
    classes["safety_events"] += ["unsafe:hold_tick_motion"] * bool(rules["hold_tick_motion"])
    classes["safety_events"] += ["unsafe:unrevalidated_resume"] * bool(rules["unrevalidated_resumes"])
    safety = (evaluation or {}).get("safety") or {}
    if evaluation is not None and "min_truth_clearance_m" not in safety:
        classes["safety_unverified"].append("no_safety_evaluation")
    if not (run / "p4/contacts.jsonl").exists():
        classes["safety_unverified"].append("contact_ledger_missing")
    integrity = (evaluation or {}).get("p2_integrity") or {}
    p2_errors = [i for i in classes["safety_unverified"] if i.startswith(P2_ERROR_PREFIX)]
    p2_check = None
    if p2_errors:  # D15: the accepted P2 integrity check raised; establish its safety facts from the ULog instead
        facts = p2_facts_reader(summary)
        policy_checked = "native_commands" in (evaluation or {})
        p2_check = {"facts": facts, "policy_a_checked": policy_checked, "failures": None}
        if facts is not None and policy_checked:
            p2_check["failures"] = p2_ulog_failures(facts)
            for item in p2_errors:
                classes["safety_unverified"].remove(item)
                classes["integrity_events"].append(item)
            classes["safety_events"] += p2_check["failures"]
            integrity = {"final_landed": facts["final_landed"], "final_disarmed": facts["final_disarmed"]}
    distance = final_goal_distance(run, result, evaluation or {}, config["mission"])
    tolerance = config["mission"]["goal_tolerance_m"]
    conditions = {"goal_reached_recorded": (result.get("p4") or {}).get("goal_reached_sim_s") is not None,
                  "p4_outcome_completed": result.get("p4_outcome") == "completed",
                  "final_state_complete": result.get("final_state") == "COMPLETE",
                  "landed_and_disarmed": integrity.get("final_landed") is True and integrity.get("final_disarmed") is True,
                  "final_goal_distance_within_tolerance": distance is not None and distance <= tolerance,
                  "no_safety_event": not classes["safety_events"], "safety_verified": not classes["safety_unverified"],
                  "no_mission_failure": not classes["mission_failures"]}
    metrics = {"final_goal_distance_m": distance, "goal_tolerance_m": tolerance,
               "min_truth_clearance_m": safety.get("min_truth_clearance_m"), "collisions": safety.get("collisions"),
               "tracking_max_m": safety.get("tracking_max_m"), "abort_reason": (result.get("p4") or {}).get("abort_reason"),
               "p4_outcome": result.get("p4_outcome"), "final_state": result.get("final_state"), "p2_failure": result.get("failure"),
               "hover_time_sim_s": hover_time(supervisor, decisions[-1]["sim_s"] if decisions else None),
               "rtf": (evaluation or {}).get("rtf")}
    try:
        import p5_comparison
        reported = p5_comparison.flight_metrics(run, name)["reported"]
        metrics.update(time_to_goal_sim_s=reported["time_to_goal_sim_s"], path_length_m=reported["path_length_m"],
                       detour_ratio=reported["detour_ratio"])
    except Exception as exc:  # the metrics are reported; their absence never changes the outcome class
        classes["integrity_events"].append("metrics_unavailable:" + type(exc).__name__)
    policy = (evaluation or {}).get("policy") or {}
    metrics["policy"] = {k: policy.get(k) for k in ("invoked", "disagreements", "disagreement_rate", "interventions")} if policy else None
    record.update(outcome="goal_success" if all(conditions.values()) else "failure", goal_conditions=conditions,
                  classes=classes, unsafe_rules=rules, metrics=metrics, p2_ulog_check=p2_check,
                  stop_reasons=["safety_event:" + i for i in classes["safety_events"]]
                  + ["safety_unverified:" + i for i in classes["safety_unverified"]])
    return record


# ---- entry points -------------------------------------------------------------------------------------------------
def classify(run):
    """The sequencer's hook: the classifier in the accepted evaluators' environment, then its record."""
    done = subprocess.run(["bash", str(WRAPPER), str(run)], text=True, capture_output=True)
    path = Path(run) / "p7/flight-record.json"
    if done.returncode != 0 or not path.exists():
        raise RuntimeError("classifier_failed:%d:%s" % (done.returncode, done.stderr.strip()[-400:]))
    return strict_json(path)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        raise SystemExit("Usage: p7_classify.py RUN")
    run = Path(argv[0])
    output = run / "p7/flight-record.json"
    if output.exists():
        raise SystemExit("A flight record exists; the first classification is retained")
    output.parent.mkdir(exist_ok=True)
    record = build_record(run)
    atomic_json(output, record, exclusive=True)
    print(json.dumps({"validity": record["validity"], "outcome": record["outcome"], "stop_reasons": record["stop_reasons"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
