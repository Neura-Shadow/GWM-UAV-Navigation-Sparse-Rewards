"""Offline model readiness on held-out layouts (contract §4.2, operator decision D8).

Each held-out layout is flown twice in the synthetic closed loop under the same shield: once by
P4's production `Supervisor` (the baseline) and once by the P5 `PolicySupervisor` with the
production `PolicyRanker` over the candidate weights. Wall time is not measured here (zero clock);
timing is an M2/M4 and SITL measurement. Not acceptance evidence.
"""
import json
import math

import p5_dataset
import p5_episode
import p5_paths
from gwm_obstacle_avoidance import supervisor
from gwm_policy_control import candidate_ranker, policy_runtime, safety_shield
from gwm_policy_control.contracts import ARCHITECTURE


def bar():
    validation = json.loads((p5_paths.P5 / "configs/p5_validation.yaml").read_text(encoding="utf-8"))
    return validation["readiness"]


def _summary(result):
    keys = ("outcome", "reason", "transit_s", "path_m", "min_clearance_m", "track_max_m", "invocations", "failures",
            "outside_mask", "executed_mismatch", "disagreements", "heading_disagreements", "failure_reasons")
    out = {k: result[k] for k in keys}
    out["hover_s"] = round(result["hover_ticks"] * p5_episode.DT, 6)
    return out


def evaluate_layout(job):
    """job: (layout, scan cells, weights, weights sha256, schema, schema sha256)."""
    layout, scan, weights, weights_sha, schema, schema_sha = job
    _, C, _ = p5_episode.configs()
    p4 = p5_episode.run_episode(layout["boxes"], lambda c, s: supervisor.Supervisor(c, s), scan_cells=scan)
    policy = policy_runtime.FrozenPolicy(weights, {"architecture": ARCHITECTURE, "schema_sha256": schema_sha,
                                                   "trained": True}, sha256=weights_sha)
    ranker = candidate_ranker.PolicyRanker(policy, schema, failure_wall_s=0.020, clock=p5_dataset.zero_clock)
    learned = p5_episode.run_episode(layout["boxes"], lambda c, s: safety_shield.PolicySupervisor(c, s, ranker),
                                     scan_cells=scan)
    return {"id": layout["id"], "family": layout["family"], "p4": _summary(p4), "policy": _summary(learned)}


def _mean(values):
    return sum(values) / len(values) if values else None


def _side(episodes, key):
    runs = [e[key] for e in episodes]
    reached = [r for r in runs if r["outcome"] == "goal_reached"]
    outcomes = {}
    for r in runs:
        label = r["outcome"] if r["reason"] is None else r["outcome"] + ":" + r["reason"]
        outcomes[label] = outcomes.get(label, 0) + 1
    clearances = [r["min_clearance_m"] for r in runs if math.isfinite(r["min_clearance_m"])]
    return {"episodes": len(runs), "goal_reached": len(reached), "goal_success_pct": 100.0 * len(reached) / len(runs),
            "mean_time_to_goal_s": _mean([r["transit_s"] for r in reached]),
            "mean_path_m": _mean([r["path_m"] for r in reached]),
            "min_clearance_m": min(clearances) if clearances else None,
            "mean_min_clearance_m": _mean(clearances),
            "collisions": sum(1 for c in clearances if c < 0.0),
            "mean_hover_s": _mean([r["hover_s"] for r in runs]), "outcomes": dict(sorted(outcomes.items()))}


def report(episodes, readiness_bar):
    p4, policy = _side(episodes, "p4"), _side(episodes, "policy")
    both = [e for e in episodes if e["p4"]["outcome"] == "goal_reached" and e["policy"]["outcome"] == "goal_reached"]
    runs = [e["policy"] for e in episodes]
    invocations = sum(r["invocations"] for r in runs)
    shield = {"outside_mask": sum(r["outside_mask"] for r in runs),
              "executed_mismatch": sum(r["executed_mismatch"] for r in runs)}
    violations = shield["outside_mask"] + shield["executed_mismatch"]
    drop = p4["goal_success_pct"] - policy["goal_success_pct"]
    checks = {
        "held_out_layouts": {"value": len(episodes), "limit": readiness_bar["held_out_layouts_min"],
                             "pass": len(episodes) >= readiness_bar["held_out_layouts_min"]},
        "shield_violations": {"value": violations, "limit": readiness_bar["shield_violations_allowed"],
                              "pass": violations <= readiness_bar["shield_violations_allowed"]},
        "goal_success_drop_points": {"value": drop, "limit": readiness_bar["goal_success_max_drop_points"],
                                     "pass": drop <= readiness_bar["goal_success_max_drop_points"]}}
    return {
        "p4": p4, "policy": policy,
        "paired": {"both_reached": len(both),
                   "p4_only": sum(1 for e in episodes if e["p4"]["outcome"] == "goal_reached" and e["policy"]["outcome"] != "goal_reached"),
                   "policy_only": sum(1 for e in episodes if e["policy"]["outcome"] == "goal_reached" and e["p4"]["outcome"] != "goal_reached"),
                   "mean_time_difference_s": _mean([e["policy"]["transit_s"] - e["p4"]["transit_s"] for e in both]),
                   "mean_path_difference_m": _mean([e["policy"]["path_m"] - e["p4"]["path_m"] for e in both])},
        "policy_decisions": {"invocations": invocations, "failures": sum(r["failures"] for r in runs), **shield,
                             "disagreements": sum(r["disagreements"] for r in runs),
                             "heading_disagreements": sum(r["heading_disagreements"] for r in runs),
                             "heading_disagreement_rate": (sum(r["heading_disagreements"] for r in runs) / invocations
                                                           if invocations else None),
                             "disagreement_rate": sum(r["disagreements"] for r in runs) / invocations if invocations else None},
        "checks": checks, "ready": all(c["pass"] for c in checks.values())}
