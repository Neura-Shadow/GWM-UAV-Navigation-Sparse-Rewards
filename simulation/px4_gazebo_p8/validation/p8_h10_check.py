"""P8 H10: offline re-check of every recovery action (contract `p8-navigation-recovery-v1` §3, §7; FC-5, FC-6).

Read-only. For one P8 run it reads `p8/recovery-ledger.jsonl`, `p4/local_map.jsonl` and `p4/decisions.jsonl`. For
every `action` and `refused` record it:
* decodes the snapshot the record names (`observation_id`, `acquisition_sim_s`) and the logged traversed mask;
* rebuilds the planner context;
* re-runs the frozen check the record logged, `planner.collision_check` or `planner.yaw_permitted`, which must
  reproduce the logged result exactly (S1, S2);
* requires that a translation's heading lies within the frozen L4 cone of the logged vehicle yaw;
* requires that the tick lies inside its episode's window, and that the budgets held (S8);
* requires that no action fell on a stale tick: the decision of that tick has no stale reason (S3);
* requires that every `p4_decision` whose reason is `p8_recovery:*` has its `action` record, and the reverse (S10).

A missing or unparseable ledger counts as a failure (FC-5). The result is one JSON value, `ok` true only if every
check passes.
Usage: p8_h10_check.py RUN [--output PATH]   (exclusive create)
"""
import argparse
import json
import math
from pathlib import Path
import sys

P8 = Path(__file__).resolve().parents[1]
REPO = P8.parents[1]
for _path in (REPO / "simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance",
              P8 / "ros2_ws/src/gwm_navigation_recovery"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gwm_obstacle_avoidance import planner  # noqa: E402
from gwm_obstacle_avoidance.local_map import Grid  # noqa: E402
from gwm_navigation_recovery.recovery import LEDGER, decode_mask  # noqa: E402

CONTRACT = "p8-h10-record-v1"
TOLERANCE = 1e-9


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def check_run(run, p4_config, params):
    run = Path(run)
    out = {"contract": CONTRACT, "run": run.name, "records": 0, "actions": 0, "refusals": 0, "reproduced": 0,
           "violations": [], "ok": False}
    try:
        ledger = jsonl(run / "p8/recovery-ledger.jsonl")
    except (OSError, ValueError) as exc:
        out["violations"].append("ledger_unreadable:" + type(exc).__name__)
        return out
    if any(r.get("ledger") != LEDGER for r in ledger):
        out["violations"].append("ledger_contract")
    snapshots = {}
    for s in jsonl(run / "p4/local_map.jsonl"):
        if s.get("usable") and s.get("grid"):
            snapshots[(s.get("observation_id"), s.get("acquisition_sim_s"))] = s
    decisions = [d for d in jsonl(run / "p4/decisions.jsonl") if d.get("event") == "p4_decision"]
    by_sim = {round(d["sim_s"], 6): d for d in decisions}
    settings, envelope = p4_config["planner"], p4_config["envelope"]
    max_age = p4_config["perception"]["snapshot_max_age_sim_s"]
    half = math.radians(settings["observable_half_angle_deg"])
    ep = params["episode"]
    window, max_rot, max_trans = ep["window_s"], math.radians(ep["max_rotation_deg"]), ep["max_translation_m"]
    rate_step = math.radians(ep["yaw_rate_deg_s"]) * 0.25  # the largest per-tick yaw between two accountings
    action_sims = set()
    out["records"] = len(ledger)
    for k, r in enumerate(ledger):
        if r["kind"] not in ("action", "refused"):
            continue
        out["actions" if r["kind"] == "action" else "refusals"] += 1
        where = "record %d (sim %.3f)" % (k, r["sim_s"])
        snap = snapshots.get((r["snapshot"]["observation_id"], r["snapshot"]["acquisition_sim_s"]))
        if snap is None:
            out["violations"].append(where + ": snapshot not found")
            continue
        grid, traversed = Grid.decode(snap["grid"]), decode_mask(r["traversed"])
        if traversed.shape != grid.cells.shape:
            out["violations"].append(where + ": traversed mask shape")
            continue
        ctx = planner.Context(tuple(r["target"]), tuple(r["vehicle"]), r["vehicle_yaw"], tuple(r["goal"]),
                              tuple(r["origin"]), r["snapshot_age_s"], p4_config["geofence"])
        if r["check"]["function"] == "planner.collision_check":
            got = list(planner.collision_check(grid, traversed, tuple(r["target"]), tuple(r["end"]), ctx, settings, max_age))
            if r["kind"] == "action" and abs(planner.wrap(r["heading"] - r["vehicle_yaw"])) > half + TOLERANCE:
                out["violations"].append(where + ": translation outside the L4 cone")
        elif r["check"]["function"] == "planner.yaw_permitted":
            got = list(planner.yaw_permitted(grid, traversed, ctx, settings, envelope, max_age))
        else:
            out["violations"].append(where + ": unknown check")
            continue
        if got != r["check"]["result"]:
            out["violations"].append(where + ": check does not reproduce (%s vs %s)" % (got, r["check"]["result"]))
        elif r["kind"] == "action" and got[0] is not True:
            out["violations"].append(where + ": an executed action failed its check")
        else:
            out["reproduced"] += 1
        since = r.get("episode_hover_since_sim_s")
        if since is None or not 0.0 <= r["sim_s"] - since < window + TOLERANCE:
            out["violations"].append(where + ": outside its episode window")
        if r.get("rotation_used_rad", 0.0) > max_rot + rate_step or r.get("translation_used_m", 0.0) > max_trans + 0.05:
            out["violations"].append(where + ": budget exceeded")
        d = by_sim.get(round(r["sim_s"], 6))
        if r["kind"] == "action":
            action_sims.add(round(r["sim_s"], 6))
            if d is None:
                out["violations"].append(where + ": no decision at the action's tick")
            elif d.get("stale"):
                out["violations"].append(where + ": action on a stale tick")
            elif d.get("action") != r["action"] or not str(d.get("reason") or "").startswith("p8_recovery:"):
                out["violations"].append(where + ": the executed decision differs from the ledger")
    for d in decisions:
        if str(d.get("reason") or "").startswith("p8_recovery:") and round(d["sim_s"], 6) not in action_sims:
            out["violations"].append("decision at sim %.3f: recovery decision without a ledger action" % d["sim_s"])
    out["ok"] = not out["violations"]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    p4_config = json.loads((args.run / "p4_avoidance.yaml").read_text(encoding="utf-8"))
    params = json.loads((P8 / "configs/p8_recovery.json").read_text(encoding="utf-8"))
    value = check_run(args.run, p4_config, params)
    if args.output:
        with open(args.output, "x", encoding="utf-8") as stream:
            stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: value[k] for k in ("records", "actions", "refusals", "reproduced", "ok")}))
    return 0 if value["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
