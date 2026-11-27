"""P4 vs P5 comparison (plan §9). Reported evidence, never acceptance evidence. Read-only.

* offline: the paired held-out comparison from the hash-bound readiness report (300 layouts, both
  rankers under the same shield, identical seeds). Safety outcomes (collisions, clearance, aborts by
  reason) are kept apart from decision quality (time to goal, path length, detour ratio); paired
  differences carry fixed-seed bootstrap 95 % intervals; discordant goal outcomes get an exact
  two-sided sign test. P5 claims correct substitution only; superiority needs P7's paired batches.
* sitl: one evaluated P5 flight against the accepted P4 flight of the same scenario from the retained
  `c110bd30` evidence: outcome, truth clearance, tracking, path, time to goal, supervisor lineage,
  resources and RTF, plus the P5 per-decision shadow disagreement.
"""
import argparse
import glob
import json
import math
from pathlib import Path
import sys

import numpy as np

P5 = Path(__file__).resolve().parents[1]
REPO = P5.parents[1]
P4 = REPO / "simulation/px4_gazebo_p4"
SIM = REPO / "simulation/px4_gazebo"
for path in (SIM / "validation", P4 / "validation", P5 / "validation"):
    if str(path) not in sys.path:
        sys.path.append(str(path))

CONTRACT = "p5-comparison-v1"
BOOTSTRAP_SAMPLES = 10_000
BOOTSTRAP_SEED = 20261031
STRAIGHT_PATH_M = 8.0


def mean(values):
    return float(np.mean(values)) if len(values) else None


def interval(values, rng, samples=BOOTSTRAP_SAMPLES):
    """Paired bootstrap 95 % percentile interval of the mean of per-layout differences."""
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return None
    draws = values[rng.integers(0, values.size, size=(samples, values.size))].mean(axis=1)
    low, high = np.percentile(draws, [2.5, 97.5])
    return [float(low), float(high)]


def sign_test(wins, losses):
    """Exact two-sided binomial p-value for discordant pairs (p = 0.5)."""
    n = wins + losses
    if n == 0:
        return None
    tail = sum(math.comb(n, k) for k in range(0, min(wins, losses) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def side_summary(runs):
    reached = [r for r in runs if r["outcome"] == "goal_reached"]
    outcomes = {}
    for r in runs:
        label = r["outcome"] if r["reason"] is None else r["outcome"] + ":" + r["reason"]
        outcomes[label] = outcomes.get(label, 0) + 1
    clearances = [r["min_clearance_m"] for r in runs if math.isfinite(r["min_clearance_m"])]
    return {"episodes": len(runs), "goal_reached": len(reached), "goal_success_pct": 100.0 * len(reached) / len(runs),
            "safety": {"collisions": sum(c < 0.0 for c in clearances), "min_clearance_m": min(clearances, default=None),
                       "mean_min_clearance_m": mean(clearances), "outcomes": dict(sorted(outcomes.items())),
                       "hover_or_abort": len(runs) - len(reached), "mean_hover_s": mean([r["hover_s"] for r in runs])},
            "decision_quality": {"mean_time_to_goal_s": mean([r["transit_s"] for r in reached]),
                                 "mean_path_m": mean([r["path_m"] for r in reached]),
                                 "mean_detour_ratio": mean([r["path_m"] / STRAIGHT_PATH_M for r in reached])}}


def offline(report_path, samples=BOOTSTRAP_SAMPLES, seed=BOOTSTRAP_SEED):
    report_path = Path(report_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    episodes = report["episodes"]
    rng = np.random.default_rng(seed)
    p4 = [e["p4"] for e in episodes]
    p5 = [e["policy"] for e in episodes]
    reached = lambda r: r["outcome"] == "goal_reached"  # noqa: E731
    both = [e for e in episodes if reached(e["p4"]) and reached(e["policy"])]
    p5_only = sum(1 for e in episodes if reached(e["policy"]) and not reached(e["p4"]))
    p4_only = sum(1 for e in episodes if reached(e["p4"]) and not reached(e["policy"]))
    success = [100.0 * (reached(e["policy"]) - reached(e["p4"])) for e in episodes]
    time_diff = [e["policy"]["transit_s"] - e["p4"]["transit_s"] for e in both]
    path_diff = [e["policy"]["path_m"] - e["p4"]["path_m"] for e in both]
    clearance_diff = [e["policy"]["min_clearance_m"] - e["p4"]["min_clearance_m"] for e in episodes
                      if math.isfinite(e["policy"]["min_clearance_m"]) and math.isfinite(e["p4"]["min_clearance_m"])]
    hover_diff = [e["policy"]["hover_s"] - e["p4"]["hover_s"] for e in episodes]
    invocations = sum(r["invocations"] for r in p5)
    families = {}
    for e in episodes:
        row = families.setdefault(e["family"], {"layouts": 0, "p4_goal": 0, "p5_goal": 0})
        row["layouts"] += 1
        row["p4_goal"] += reached(e["p4"])
        row["p5_goal"] += reached(e["policy"])

    def paired(values):
        return {"pairs": len(values), "mean": mean(values), "ci95": interval(values, rng, samples)}
    return {
        "contract": CONTRACT, "kind": "offline_paired_held_out", "acceptance_evidence": False,
        "source": {"readiness_report": report_path.name, "sha256": __import__("hashlib").sha256(report_path.read_bytes()).hexdigest(),
                   "weights_sha256": report.get("weights_sha256"), "layouts": len(episodes)},
        "method": {"bootstrap_samples": samples, "bootstrap_seed": seed, "interval": "paired percentile 95 %",
                   "same_shield": True, "identical_layouts_and_seeds": True},
        "p4": side_summary(p4), "p5": side_summary(p5),
        "paired": {"both_reached": len(both), "p5_only": p5_only, "p4_only": p4_only, "neither": len(episodes) - len(both) - p5_only - p4_only,
                   "goal_success_difference_points": paired(success), "discordant_sign_test_p": sign_test(p5_only, p4_only),
                   "time_to_goal_difference_s": paired(time_diff), "path_difference_m": paired(path_diff),
                   "detour_ratio_difference": paired([d / STRAIGHT_PATH_M for d in path_diff]),
                   "min_clearance_difference_m": paired(clearance_diff), "hover_difference_s": paired(hover_diff)},
        "per_decision": {"invocations": invocations, "failures": sum(r["failures"] for r in p5),
                         "outside_mask": sum(r["outside_mask"] for r in p5),
                         "disagreement_rate": sum(r["disagreements"] for r in p5) / invocations if invocations else None,
                         "heading_disagreement_rate": sum(r["heading_disagreements"] for r in p5) / invocations if invocations else None},
        "by_family": families,
        "claim": "correct substitution only; a superiority claim needs P7's paired batches"}


# ---- SITL: a P5 flight against the accepted P4 flight of the same scenario --------------------------------------
def accepted_p4_runs(root, p4_manifest_sha256):
    """Scenario -> the accepted P4 flight run directories (retained evidence, read-only)."""
    runs = {}
    for gate in ("B", "S", "C"):
        for path in sorted(glob.glob(str(Path(root) / "state" / f"p4-gate-{gate}-{p4_manifest_sha256}.result.json"))):
            result = json.loads(Path(path).read_text())
            if result.get("accepted") is not True:
                continue
            for row in result["runs"]:
                if row.get("scope") == "flight" and row.get("evaluation_status") == "passed":
                    runs.setdefault(row["item"], []).append(row["run"])
    return runs


def flight_metrics(run, evaluation_name):
    import p4_evaluate
    import p5_evaluate
    run = Path(run)
    evaluation = json.loads((run / evaluation_name).read_text())
    summary = json.loads((run / "summary.json").read_text())
    events = p5_evaluate.events_of(run)
    result = summary.get("controller_result") or {}
    declaration = json.loads((run / "p4/scenario.json").read_text())
    safety = evaluation.get("safety") or {}
    p4 = result.get("p4") or {}
    return {"run": run.name, "scenario": declaration["scenario"], "evaluation_status": evaluation.get("status"),
            "controller": result.get("controller"), "outcome": (evaluation.get("outcome") or {}).get("label"),
            "safety": {"min_truth_clearance_m": safety.get("min_truth_clearance_m"), "collisions": safety.get("collisions"),
                       "tracking_max_m": safety.get("tracking_max_m"), "abort_reason": p4.get("abort_reason")},
            "lineage": [t["to"] for t in p4.get("supervisor_transitions", [])],
            "reported": p5_evaluate.reported(run, summary, events, p4_evaluate.truth_track(run)), "rtf": evaluation.get("rtf")}


def difference(a, b):
    return None if a is None or b is None else a - b


def sitl(p5_run, p4_run):
    p5 = flight_metrics(p5_run, "p5-evaluation.json")
    p4 = flight_metrics(p4_run, "p4-evaluation.json")
    if p5["scenario"] != p4["scenario"]:
        raise ValueError("p5_and_p4_flights_are_different_scenarios")
    policy = json.loads((Path(p5_run) / "p5-evaluation.json").read_text()).get("policy") or {}
    r5, r4 = p5["reported"], p4["reported"]
    return {"contract": CONTRACT, "kind": "sitl_paired_flight", "acceptance_evidence": False, "scenario": p5["scenario"],
            "p5": p5, "p4": p4,
            "safety": {"same_outcome": p5["outcome"] == p4["outcome"],
                       "min_truth_clearance_difference_m": difference(p5["safety"]["min_truth_clearance_m"], p4["safety"]["min_truth_clearance_m"]),
                       "tracking_max_difference_m": difference(p5["safety"]["tracking_max_m"], p4["safety"]["tracking_max_m"])},
            "decision_quality": {"path_difference_m": difference(r5["path_length_m"], r4["path_length_m"]),
                                 "detour_ratio_difference": difference(r5["detour_ratio"], r4["detour_ratio"]),
                                 "time_to_goal_difference_s": difference(r5["time_to_goal_sim_s"], r4["time_to_goal_sim_s"]),
                                 "same_lineage": p5["lineage"] == p4["lineage"]},
            "per_decision": {k: policy.get(k) for k in ("invoked", "disagreements", "disagreement_rate", "interventions",
                                                        "participation_fraction", "inference_s")},
            "host": {"p5_controller_cpu_fraction": r5["controller_cpu_fraction"], "p4_controller_cpu_fraction": r4["controller_cpu_fraction"],
                     "p5_rtf": p5["rtf"], "p4_rtf": p4["rtf"]}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    o = sub.add_parser("offline")
    o.add_argument("report", type=Path)
    o.add_argument("--output", type=Path, required=True)
    s = sub.add_parser("sitl")
    s.add_argument("p5_run", type=Path)
    s.add_argument("p4_run", type=Path)
    s.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    value = offline(args.report) if args.command == "offline" else sitl(args.p5_run, args.p4_run)
    if args.output.exists():
        raise SystemExit("Comparison output exists; outputs are never overwritten")
    args.output.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
