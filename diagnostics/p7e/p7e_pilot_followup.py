"""P7-E proposal, pilot follow-up (read-only): the observation-gap events and the repeat-outcome inconsistencies.

Uses existing evidence only: no flight, no threshold change, no evaluator re-run. It reads every P7 flight so far (M4
dry run, pilot attempt 1 on 6f60492f, pilot attempt 2 on 0799c37e). For each flight it takes:
* the classifier record;
* the probe's native clock: interval RTF, and stalls (intervals below 0.5);
* the 1 s resource ledger: per-process CPU fraction, load and memory;
* for P2 observation-gap aborts, the abort instant and the resources and clock around it.

It writes one JSON file with exclusive create.
Usage: python3 p7e_pilot_followup.py <output.json>
"""
import json
from pathlib import Path
import statistics as st
import sys

ROOT = Path.home() / "uav_autonomy"
REPO = Path(__file__).resolve().parents[2]
CANDIDATES = {"ac983564": "M4 dry run", "6f60492f": "pilot attempt 1", "0799c37e": "pilot attempt 2"}
STALL_RTF = 0.5
PROCESSES = ("gazebo", "px4", "ros-controller", "p4-perception", "sensor-bridge", "sensor-adapter", "dds-agent")


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()] if Path(path).exists() else []


def clock(run):
    rows = [r for r in jsonl(run / "source/clock.jsonl") if r["sim_s"] > 0]
    steps = []
    for a, b in zip(rows, rows[1:]):
        wall = (b["receipt_monotonic_ns"] - a["receipt_monotonic_ns"]) / 1e9
        if wall > 0:
            steps.append({"sim_s": a["sim_s"], "monotonic_s": a["receipt_monotonic_ns"] / 1e9, "wall_s": wall,
                          "rtf": (b["sim_s"] - a["sim_s"]) / wall})
    return steps


def cpu_fractions(resources):
    first, last, out = {}, {}, {}
    for r in resources:
        for name, p in r["processes"].items():
            first.setdefault(name, (r["monotonic_s"], p["cpu_s"]))
            last[name] = (r["monotonic_s"], p["cpu_s"])
    for name in PROCESSES:
        if name in first and last[name][0] > first[name][0]:
            out[name] = round((last[name][1] - first[name][1]) / (last[name][0] - first[name][0]), 3)
    return out


def around(resources, t, before=4.0, after=2.0):
    """Per-process CPU fraction in the window around monotonic t, and the 1-minute load."""
    rows = [r for r in resources if t - before <= r["monotonic_s"] <= t + after]
    if len(rows) < 2:
        return None
    a, b = rows[0], rows[-1]
    span = b["monotonic_s"] - a["monotonic_s"]
    cpu = {n: round((b["processes"][n]["cpu_s"] - a["processes"][n]["cpu_s"]) / span, 3)
           for n in PROCESSES if n in a["processes"] and n in b["processes"]}
    return {"window_s": round(span, 2), "cpu_fraction": cpu, "load1": [round(r["loadavg"][0], 2) for r in rows],
            "mem_available_gb_min": round(min(r["mem_available_kb"] for r in rows) / 1e6, 2)}


def flight(run):
    s = json.loads((run / "summary.json").read_text())
    rec_path = run / "p7/flight-record.json"
    rec = json.loads(rec_path.read_text()) if rec_path.exists() else {}
    m = rec.get("metrics") or {}
    steps = clock(run)
    resources = jsonl(run / "resources.jsonl")
    stalls = [x for x in steps if x["rtf"] < STALL_RTF]
    worst = min(steps, key=lambda x: x["rtf"]) if steps else None
    out = {"run": run.name, "set": CANDIDATES.get(s["frozen_manifest_sha256"][:8], "other"),
           "controller": (s.get("p7") or {}).get("controller"), "item": (s.get("p7") or {}).get("item"),
           "validity": rec.get("validity"), "outcome": rec.get("outcome"), "abort_reason": m.get("abort_reason"),
           "p2_failure": m.get("p2_failure"), "time_to_goal_sim_s": m.get("time_to_goal_sim_s"),
           "hover_time_sim_s": m.get("hover_time_sim_s"), "min_truth_clearance_m": m.get("min_truth_clearance_m"),
           "policy": m.get("policy"), "wall_duration_s": s.get("wall_duration_s"),
           "rtf_mean": (steps[-1]["sim_s"] - steps[0]["sim_s"]) / sum(x["wall_s"] for x in steps) if steps else None,
           "rtf_min_interval": worst and round(worst["rtf"], 4), "rtf_min_at_sim_s": worst and round(worst["sim_s"], 3),
           "stalls_below_0_5": len(stalls), "stall_wall_s_total": round(sum(x["wall_s"] for x in stalls), 2),
           "cpu_fraction": cpu_fractions(resources),
           "load1_max": max((r["loadavg"][0] for r in resources), default=None),
           "lineage": [t["to"] for t in ((s.get("controller_result") or {}).get("p4") or {}).get("supervisor_transitions", [])]}
    if m.get("p2_failure") == "observation_gap":
        ev = next(e for e in jsonl(run / "ros-events.jsonl") if e.get("event") == "abort")
        near = [x for x in steps if abs(x["sim_s"] - ev["sim_s"]) <= 3.0]
        out["gap"] = {"abort_sim_s": ev["sim_s"], "abort_monotonic_s": ev["monotonic_s"],
                      "clock_steps_within_3_sim_s": [{"sim_s": round(x["sim_s"], 3), "wall_s": round(x["wall_s"], 3),
                                                       "rtf": round(x["rtf"], 3)} for x in near if x["rtf"] < 0.9],
                      "resources_around": around(resources, ev["monotonic_s"])}
    return out


def summary(rows, key):
    values = [r[key] for r in rows if r[key] is not None]
    return {"n": len(values), "median": st.median(values) if values else None, "max": max(values, default=None),
            "min": min(values, default=None)}


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    runs = sorted(p for p in (ROOT / "runs").iterdir() if ("-p7-" in p.name or "-p7dry-" in p.name) and (p / "summary.json").exists())
    rows = [flight(r) for r in runs]
    by = {c: [r for r in rows if r["controller"] == c] for c in ("p4", "p5")}
    compare = {c: {"flights": len(v), "stalls_below_0_5": summary(v, "stalls_below_0_5"),
                   "rtf_min_interval": summary(v, "rtf_min_interval"), "rtf_mean": summary(v, "rtf_mean"),
                   "controller_cpu_fraction": summary([dict(x, c=x["cpu_fraction"].get("ros-controller")) for x in v], "c"),
                   "gazebo_cpu_fraction": summary([dict(x, c=x["cpu_fraction"].get("gazebo")) for x in v], "c"),
                   "flights_with_a_stall": sum(1 for x in v if x["stalls_below_0_5"]),
                   "observation_gap_aborts": sum(1 for x in v if x["p2_failure"] == "observation_gap")} for c, v in by.items()}
    pilot2 = [r for r in rows if r["set"] == "pilot attempt 2"]
    pairs = {}
    for r in pilot2:
        pairs.setdefault((r["item"], r["controller"]), []).append(r)
    inconsistent = [{"layout": k[0], "controller": k[1], "repeats": [{x: r[x] for x in (
        "run", "outcome", "abort_reason", "p2_failure", "lineage", "hover_time_sim_s", "time_to_goal_sim_s",
        "min_truth_clearance_m", "policy", "stalls_below_0_5", "rtf_min_interval")} for r in v]}
        for k, v in sorted(pairs.items()) if len({r["outcome"] for r in v}) > 1]
    value = {"contract": "p7e-pilot-followup-v1", "read_only": True, "stall_rtf": STALL_RTF, "flights": rows,
             "p4_vs_p5": compare, "gap_events": [r for r in rows if "gap" in r], "repeat_inconsistencies_pilot2": inconsistent}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({"flights": len(rows), "p4_vs_p5": compare}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
