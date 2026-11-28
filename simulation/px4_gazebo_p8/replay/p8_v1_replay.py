"""P8 V1: offline counterfactual replay on the 60 accepted P7-E flights (contract `p8-navigation-recovery-v1` §6 V1;
report plan H11). Read-only.

For every TRANSIT decision of every flight it rebuilds what the recovery layer would have seen, from the flight's own
records, and feeds it to the unchanged `RecoveryManager` and, for P5 flights, the R4 guard:
* the frozen supervisor's output (state, action, stale, frozen yaw), from `p4/decisions.jsonl`;
* the P5 policy record of the same tick, from `p5/policy-ledger.jsonl`;
* the HOVER entry and reason, from the recorded supervisor transitions;
* the snapshot that decision used, from `p4/local_map.jsonl`;
* the vehicle yaw, from the nearest earlier recorded sample;
* the traversed mask, rebuilt from the recorded positions as the accepted mission builds it (frozen
  `planner.traversed_mask`, 0.02 m spacing).

**What is open-loop.** The recorded vehicle does not follow recovery, so V1 judges only what can be judged without a
flight:
1. recovery triggers in exactly the 18 failures and in none of the 42 successes;
2. at each triggering episode, its plan and whether the plan fits the 20 s window;
3. its first action, and whether that action passes the frozen L2 or L5 check on the recorded snapshot;
4. the R4 guard fires only on slot 12.

Closed-loop effects are what V3–V5 are for.
Usage: cd /tmp && /usr/bin/python3 p8_v1_replay.py OUTPUT.json   (exclusive create)
"""
import json
import math
from pathlib import Path
import sys

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
for _path in (REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance", P8 / "ros2_ws/src/gwm_navigation_recovery",
              REPO / "simulation/px4_gazebo/validation"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gwm_obstacle_avoidance import planner  # noqa: E402
from gwm_obstacle_avoidance.local_map import Grid  # noqa: E402
from gwm_navigation_recovery.guard import YawOscillationGuard  # noqa: E402
from gwm_navigation_recovery.recovery import RecoveryManager  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

CONTRACT = "p8-v1-replay-v1"
ROOT = Path.home() / "uav_autonomy"
P7E_SHA = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
RESULTS = {1: "1c8672ab5d3a10df831a78ee34ea11282983fb9c002d51ef728dcc9884faa441",
           2: "62c23933960e256d9fc0d60b3ede008f9b0872752a46bbd4377cdf0783b56064",
           3: "d4da3561bfac6158b4992731d7721de3c2082cd0e5003907586e0e937d1a84de"}
SPECS = {"p8_recovery.json": "dee90d51098210daf714fe0b875b0e98040195362095800dc48836f78371333b"}


def jsonl(path):
    path = Path(path)
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()] if path.exists() else []


def enu(ned):
    return float(ned[1]), float(ned[0])


def yaw_enu(yaw_ned):
    return planner.wrap(math.pi / 2 - yaw_ned)


class Flight:
    def __init__(self, entry, p4):
        self.run = Path(entry["run"])
        self.entry, self.p4 = entry, p4
        self.decisions = sorted((d for d in jsonl(self.run / "p4/decisions.jsonl") if d.get("event") == "p4_decision"),
                                key=lambda d: d["sim_s"])
        self.maps = {round(s["acquisition_sim_s"], 6): s for s in jsonl(self.run / "p4/local_map.jsonl")
                     if s.get("usable") and s.get("grid")}
        self.samples = [(e["sample"]["t"] if "t" in e["sample"] else e["ros_sim_s"], e["sample"]["position"], e["sample"]["yaw"])
                        for e in jsonl(self.run / "ros-events.jsonl") if e.get("event") == "sample" and e.get("sample")]
        self.samples.sort(key=lambda s: s[0])
        result = strict_json(self.run / "controller-result.json")
        self.transitions = result["p4"]["supervisor_transitions"]
        self.goal, self.origin = tuple(result["p4"]["goal_enu"]), tuple(result["p4"]["origin_enu"])
        self.policy = {round(r["sim_s"], 6): r for r in jsonl(self.run / "p5/policy-ledger.jsonl")}
        self.record = strict_json(self.run / "p7/flight-record.json")

    def hover_at(self, sim):
        state, since, reason = "NORMAL", None, None
        for t in self.transitions:
            if t["sim_s"] > sim + 1e-9:
                break
            state = t["to"]
            if t["to"] == "HOVER":
                since, reason = t["sim_s"], t["reason"]
        return (since, reason) if state == "HOVER" else None

    def yaw_at(self, sim):
        best = None
        for t, _, yaw in self.samples:
            if t > sim + 1e-9:
                break
            best = yaw
        return yaw_enu(best) if best is not None else 0.0

    def positions_until(self, sim):
        out = []
        for t, pos, _ in self.samples:
            if t > sim + 1e-9:
                break
            xy = enu(pos)
            if not out or math.dist(out[-1], xy) >= 0.02:
                out.append(xy)
        return out


def replay(flight, params):
    p4 = flight.p4
    manager = RecoveryManager(p4, params)
    guard = YawOscillationGuard(params) if flight.entry["controller"] == "p5" else None
    guard_events, episodes, traversed_cache = [], {}, {}
    for d in flight.decisions:
        sim = d["sim_s"]
        policy = flight.policy.get(round(sim, 6)) or {}
        out = {"state": d["state"], "action": d["action"], "heading": d.get("heading"), "end": d.get("end"),
               "length": None, "frozen_yaw": d.get("frozen_yaw", False), "reason": d.get("reason"), "path": None,
               "plan": d.get("plan"), "stale": d.get("stale"),
               "policy": {"failure": policy.get("failure"), "intervention": policy.get("intervention")} if policy else None}
        if guard is not None:
            event = guard.observe(sim, out, tuple(d["vehicle_enu"]), flight.goal)
            if event in ("start", "end"):
                guard_events.append({"sim_s": sim, "event": event})
        hover = flight.hover_at(sim)
        if hover is None or hover[1] != "no_progress_candidate" or d.get("stale") is not None:
            manager.step(dict(sim=sim, snapshot=None, grid=None, traversed=None, vehicle=tuple(d["vehicle_enu"]),
                              vehicle_yaw=0.0, target=tuple(d["target_enu"]), goal=flight.goal, origin=flight.origin,
                              geofence=p4["geofence"]), out, hover)
            continue
        if hover[0] in episodes and episodes[hover[0]].get("first") is not None:
            continue  # open-loop: only the plan and the first action of an episode can be judged
        snap = flight.maps.get(round(d["snapshot_acquisition_sim_s"], 6)) if d.get("snapshot_acquisition_sim_s") is not None else None
        if snap is None:
            episodes.setdefault(hover[0], {})["missing_snapshot"] = True
            continue
        grid = Grid.decode(snap["grid"])
        key = (hover[0], grid.cells.shape)
        if key not in traversed_cache:
            traversed_cache[key] = planner.traversed_mask(grid, flight.positions_until(sim), p4["envelope"]["r_body_m"])
        inputs = {"sim": sim, "snapshot": snap, "grid": grid, "traversed": traversed_cache[key],
                  "vehicle": tuple(d["vehicle_enu"]), "vehicle_yaw": flight.yaw_at(sim), "target": tuple(d["target_enu"]),
                  "goal": flight.goal, "origin": flight.origin, "geofence": p4["geofence"]}
        before = len(manager.records)
        manager.step(inputs, out, hover)
        ep = episodes.setdefault(hover[0], {"since": hover[0]})
        for r in manager.records[before:]:
            if r["kind"] == "episode_start":
                ep["start"] = {k: r[k] for k in ("r1_applies", "terminal_occupied", "terminal_unknown", "search", "phases",
                                                 "planned_time_s", "window_left_s", "plan_fits_window")}
                ep["start"]["planned_actions"] = len(r["queue"])
            elif r["kind"] in ("action", "refused") and ep.get("first") is None:
                ep["first"] = {"kind": r["kind"], "phase": r["phase"], "action": r["action"], "check": r["check"],
                               "sim_s": r["sim_s"], "after_hover_entry_s": r["sim_s"] - hover[0]}
    return {"slot": flight.entry["slot"], "controller": flight.entry["controller"], "run": flight.run.name,
            "outcome": flight.record["outcome"], "triggered": bool(episodes), "episodes": list(episodes.values()),
            "guard_events": guard_events,
            "inputs_sha256": {name: digest(flight.run / name) for name in ("p4/decisions.jsonl", "p4/local_map.jsonl",
                                                                           "controller-result.json")}}


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    params_path = P8 / "configs/p8_recovery.json"
    from p3_provenance import canonical_sha256
    if canonical_sha256(strict_json(params_path)) != SPECS["p8_recovery.json"]:
        raise SystemExit("p8_recovery.json differs from the approved dee90d51…")
    params = strict_json(params_path)
    flights = []
    for block, sha in RESULTS.items():
        path = ROOT / "state" / ("p7e-engineering-30-block-%02d-%s.result.json" % (block, P7E_SHA))
        if digest(path) != sha:
            raise SystemExit("block %d result differs" % block)
        flights += strict_json(path)["flights"]
    rows = []
    for entry in sorted(flights, key=lambda f: (f["slot"], f["controller"])):
        p4 = strict_json(Path(entry["run"]) / "p4_avoidance.yaml")
        rows.append(replay(Flight(entry, p4), params))
    failures = {(r["slot"], r["controller"]) for r in rows if r["outcome"] != "goal_success"}
    triggered = {(r["slot"], r["controller"]) for r in rows if r["triggered"]}
    eps = [e for r in rows for e in r["episodes"]]
    planned = [e for e in eps if e.get("start", {}).get("planned_actions")]
    firsts = [e["first"] for e in eps if e.get("first")]
    guard_slots = sorted({r["slot"] for r in rows for g in r["guard_events"] if g["event"] == "start"})
    criteria = {
        "triggers_exactly_in_the_failures": triggered == failures,
        "no_trigger_in_a_success": not (triggered - failures),
        "every_first_action_passes_its_frozen_check": all(f["kind"] == "action" and f["check"]["result"][0] is True for f in firsts),
        "every_plan_fits_the_window": all(e.get("start", {}).get("plan_fits_window") for e in eps if e.get("start")),
        "guard_fires_only_on_slot_12": guard_slots == [12],
    }
    summary = {
        "flights": len(rows), "failures": len(failures), "triggered": len(triggered),
        "episodes": len(eps), "episodes_with_a_plan": len(planned), "episodes_without_a_plan": len(eps) - len(planned),
        "first_actions": len(firsts), "first_actions_refused": sum(1 for f in firsts if f["kind"] == "refused"),
        "first_action_phases": {p: sum(1 for f in firsts if f["phase"] == p) for p in ("R1", "R2", "R3")},
        "r1_episodes": sum(1 for e in eps if e.get("start", {}).get("r1_applies")),
        "planned_time_s": sorted(round(e["start"]["planned_time_s"], 2) for e in eps if e.get("start")),
        "search_wall_s_max": max((e["start"]["search"]["wall_s"] for e in eps if e.get("start")), default=None),
        "guard_slots": guard_slots, "criteria": criteria, "passed": all(criteria.values()),
    }
    value = {"contract": CONTRACT, "read_only": True, "p7e_manifest_sha256": P7E_SHA, "recovery_params_canonical_sha256": SPECS["p8_recovery.json"],
             "code_sha256": {"p8_v1_replay.py": digest(Path(__file__)),
                             **{name: digest(P8 / "ros2_ws/src/gwm_navigation_recovery/gwm_navigation_recovery" / name)
                                for name in ("recovery.py", "search.py", "guard.py")}},
             "open_loop_limits": "the recorded vehicle does not follow recovery: only triggers, plans, first actions and the "
                                 "guard are judged; closed-loop effects are V3-V5",
             "summary": summary, "flights": rows}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(summary, indent=1, default=float))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
