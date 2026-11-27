"""Replica of P4 `planner.plan` with one change: L3 selection goes through a ranker hook (contract C3).

Everything else (candidates, collision validity, L4 translation authority, L5 yaw permission, L6
SCAN, HOLD) calls the unchanged P4 functions in the same order. With `P4Ranker` the decision equals
P4's field for field (parity tests); a ranker failure yields HOLD and never another ranking.
"""
import math

from gwm_obstacle_avoidance import planner
from gwm_obstacle_avoidance.planner import HOLD, SCAN, TRANSLATE, YAW_TO


def plan_with_ranker(grid, traversed, ctx, settings, envelope, max_age, ranker, state):
    rows = planner.evaluate_candidates(grid, traversed, ctx, settings, max_age)
    reaches_goal = lambda r: math.dist(r["end"], ctx.goal) < 1e-9 and r["progress"] > 0
    progress = [r for r in rows if r["valid"] and (r["progress"] > settings["min_progress_m"] or reaches_goal(r))]
    decision = {"path_heading": ctx.path_heading, "candidates_valid": sum(r["valid"] for r in rows),
                "candidates_progress": len(progress), "best": None, "action": HOLD, "heading": None,
                "length": None, "end": None, "reason": None, "policy": None}
    half = math.radians(settings["observable_half_angle_deg"])
    if progress:
        best, record = ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
        decision["policy"] = record
        if best is None:
            decision["reason"] = "policy_failure:" + record["failure"]
            return decision
        decision["best"] = {k: best[k] for k in ("k", "heading", "length", "end", "score", "progress", "clearance",
                                                 "deviation")}
        if abs(planner.wrap(best["heading"] - ctx.vehicle_yaw)) <= half:
            decision.update(action=TRANSLATE, heading=best["heading"], length=best["length"], end=best["end"])
            return decision
        permitted, reason = planner.yaw_permitted(grid, traversed, ctx, settings, envelope, max_age)
        if permitted:
            decision.update(action=YAW_TO, heading=best["heading"])
        else:
            decision.update(action=HOLD, reason="yaw_not_permitted:" + reason)
        return decision
    blocked = [r for r in rows if r["unknown_only"] and round(r["heading"], 6) not in ctx.scanned]
    if blocked:
        target = min(blocked, key=lambda r: (-r["unknown_cells"], abs(r["k"]), 0 if r["k"] >= 0 else 1, r["nominal_length"]))
        permitted, reason = planner.yaw_permitted(grid, traversed, ctx, settings, envelope, max_age)
        if permitted:
            decision.update(action=SCAN, heading=target["heading"], reason="unknown_blocked")
        else:
            decision.update(action=HOLD, reason="scan_not_permitted:" + reason)
        return decision
    decision["reason"] = "no_collision_valid_progress_candidate"
    return decision
