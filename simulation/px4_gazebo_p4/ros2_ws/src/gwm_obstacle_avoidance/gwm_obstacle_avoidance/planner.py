"""Collision-checked candidate planner with deterministic scoring (contract §5 L1–L7, revision 3).

Collision validity (L2) is independent of the vehicle heading; translation
authority (L4) is a separate check. In-place yaw (L5) is the only way to acquire
new observations, and translation never enters a cell that is not observed FREE.
All functions are pure: identical inputs give identical outputs.
"""
import math

import numpy as np

from .local_map import FREE, OCCUPIED

TRANSLATE, YAW_TO, SCAN, HOLD = "TRANSLATE", "YAW_TO", "SCAN", "HOLD"


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Context:
    """Everything a decision depends on (mission frame: PX4 local ENU)."""

    def __init__(self, target, vehicle, vehicle_yaw, goal, origin, snapshot_age, geofence, scanned=()):
        self.target = (float(target[0]), float(target[1]))
        self.vehicle = (float(vehicle[0]), float(vehicle[1]))
        self.vehicle_yaw = float(vehicle_yaw)
        self.goal = (float(goal[0]), float(goal[1]))
        self.origin = (float(origin[0]), float(origin[1]))
        self.snapshot_age = snapshot_age
        self.geofence = geofence
        self.scanned = frozenset(scanned)

    @property
    def path_heading(self):
        return math.atan2(self.goal[1] - self.target[1], self.goal[0] - self.target[0])


def inside_fence(point, ctx, margin):
    fence = ctx.geofence
    x, y = point[0] - ctx.origin[0], point[1] - ctx.origin[1]
    tolerance = 1e-9
    return (fence["x_min_m"] + margin - tolerance <= x <= fence["x_max_m"] - margin + tolerance
            and abs(y) <= fence["y_abs_max_m"] - margin + tolerance)


def endpoint(start, heading, length):
    return start[0] + length * math.cos(heading), start[1] + length * math.sin(heading)


def collision_check(grid, traversed, start, end, ctx, settings, max_age):
    """L2 (revision 3): every cell within r_infl of the swept segment is FREE or traversed;
    traversed never overrides OCCUPIED. Returns (collision_valid, reason, unknown_only, unknown_cells)."""
    if ctx.snapshot_age is None or ctx.snapshot_age > max_age:
        return False, "stale_snapshot", False, 0
    r = settings["r_infl_m"]
    if not (inside_fence(start, ctx, r) and inside_fence(end, ctx, r)):
        return False, "geofence", False, 0
    ii, jj, states, inside = grid.segment_cells(start, end, r)
    occupied = int(np.count_nonzero(states == OCCUPIED))
    known_free = (states == FREE) | (traversed[ii, jj] & (states != OCCUPIED))
    unknown = int(np.count_nonzero(~known_free & (states != OCCUPIED))) + (0 if inside else 1)
    if occupied:
        return False, "occupied", False, unknown
    if unknown:
        return False, "unknown", True, unknown
    return True, None, False, 0


def deviation(point, ctx):
    ux, uy = ctx.goal[0] - ctx.origin[0], ctx.goal[1] - ctx.origin[1]
    norm = math.hypot(ux, uy)
    if norm == 0:
        return math.hypot(point[0] - ctx.origin[0], point[1] - ctx.origin[1])
    return abs(ux * (point[1] - ctx.origin[1]) - uy * (point[0] - ctx.origin[0])) / norm


def evaluate_candidates(grid, traversed, ctx, settings, max_age):
    rows = []
    step = math.radians(settings["heading_step_deg"])
    weights = settings["weights"]
    search = settings["r_infl_m"] + settings["clearance_cap_m"]
    start_distance = math.dist(ctx.target, ctx.goal)
    for k in range(-settings["heading_steps"], settings["heading_steps"] + 1):
        heading = wrap(ctx.path_heading + k * step)
        for nominal in settings["lengths_m"]:
            # Segments are clipped at the goal distance: a candidate never extends beyond the goal.
            length = min(nominal, max(start_distance, 1e-6))
            end = endpoint(ctx.target, heading, length)
            valid, reason, unknown_only, unknown = collision_check(grid, traversed, ctx.target, end, ctx, settings, max_age)
            row = {"k": k, "heading": heading, "length": length, "nominal_length": nominal, "end": end,
                   "valid": valid, "reason": reason,
                   "unknown_only": unknown_only, "unknown_cells": unknown,
                   "progress": start_distance - math.dist(end, ctx.goal)}
            if valid:
                distance = grid.occupied_distance(ctx.target, end, search)
                # Clearance is only resolved to one grid cell; quantizing keeps mirror-symmetric
                # scenes exactly tied so the L3 tie-break (+y first) decides, not rasterization noise.
                raw = search if distance is None else distance
                clearance = math.floor(raw / grid.resolution + 1e-9) * grid.resolution
                row["clearance"] = clearance
                row["deviation"] = deviation(end, ctx)
                row["turn"] = abs(wrap(heading - ctx.vehicle_yaw))
                row["score"] = (weights["progress"] * row["progress"] - weights["deviation"] * row["deviation"]
                                - weights["turn"] * row["turn"]
                                + weights["clearance"] * min(clearance - settings["r_infl_m"], settings["clearance_cap_m"]))
            rows.append(row)
    return rows


def tie_key(row):
    """L3 tie-break: smaller |k|; then +y (k > 0) before -y; then the shorter L."""
    return (abs(row["k"]), 0 if row["k"] >= 0 else 1, row["nominal_length"])


def score_resolution(settings, resolution):
    """Scores closer than the 0.10 m grid can resolve are ties (clearance quantum plus half a cell
    of deviation): 0.3 * 0.1 + 0.6 * 0.05 = 0.06."""
    weights = settings["weights"]
    return weights["clearance"] * resolution + weights["deviation"] * resolution / 2


def select(rows, settings, resolution):
    """L3: the best score, with scores within the grid's score resolution treated as tied."""
    best = max(r["score"] for r in rows)
    tied = [r for r in rows if r["score"] >= best - score_resolution(settings, resolution) - 1e-12]
    return min(tied, key=tie_key)


def yaw_permitted(grid, traversed, ctx, settings, envelope, max_age):
    """L5: in-place yaw needs a fresh snapshot, the envelope disc FREE or traversed, and no
    OCCUPIED cell within R_body + margin + tracking of the frozen target."""
    if ctx.snapshot_age is None or ctx.snapshot_age > max_age:
        return False, "stale_snapshot"
    r_body = envelope["r_body_m"]
    window, _ = grid._window((ctx.target[0],), (ctx.target[1],), r_body)
    if window is None:
        return False, "outside_grid"
    ii, jj, cx, cy = window
    inner = np.hypot(cx - ctx.target[0], cy - ctx.target[1]) <= r_body
    states = grid.cells[ii[inner], jj[inner]]
    allowed = (states == FREE) | traversed[ii[inner], jj[inner]]
    if not allowed.all():
        return False, "envelope_disc_not_observed"
    outer = r_body + settings["yaw_disc_margin_m"] + settings["tracking_m"]
    distance = grid.occupied_distance(ctx.target, ctx.target, outer)
    if distance is not None:
        return False, "occupied_within_yaw_disc"
    return True, None


def plan(grid, traversed, ctx, settings, envelope, max_age):
    """One deterministic decision: TRANSLATE, YAW_TO, SCAN or HOLD with its evidence."""
    rows = evaluate_candidates(grid, traversed, ctx, settings, max_age)
    reaches_goal = lambda r: math.dist(r["end"], ctx.goal) < 1e-9 and r["progress"] > 0
    progress = [r for r in rows if r["valid"] and (r["progress"] > settings["min_progress_m"] or reaches_goal(r))]
    decision = {"path_heading": ctx.path_heading, "candidates_valid": sum(r["valid"] for r in rows),
                "candidates_progress": len(progress), "best": None, "action": HOLD, "heading": None,
                "length": None, "end": None, "reason": None}
    half = math.radians(settings["observable_half_angle_deg"])
    if progress:
        best = select(progress, settings, grid.resolution)
        decision["best"] = {k: best[k] for k in ("k", "heading", "length", "end", "score", "progress", "clearance",
                                                 "deviation")}
        if abs(wrap(best["heading"] - ctx.vehicle_yaw)) <= half:
            decision.update(action=TRANSLATE, heading=best["heading"], length=best["length"], end=best["end"])
            return decision
        permitted, reason = yaw_permitted(grid, traversed, ctx, settings, envelope, max_age)
        if permitted:
            decision.update(action=YAW_TO, heading=best["heading"])
        else:
            decision.update(action=HOLD, reason="yaw_not_permitted:" + reason)
        return decision
    blocked = [r for r in rows if r["unknown_only"] and round(r["heading"], 6) not in ctx.scanned]
    if blocked:
        target = min(blocked, key=lambda r: (-r["unknown_cells"], abs(r["k"]), 0 if r["k"] >= 0 else 1, r["nominal_length"]))
        permitted, reason = yaw_permitted(grid, traversed, ctx, settings, envelope, max_age)
        if permitted:
            decision.update(action=SCAN, heading=target["heading"], reason="unknown_blocked")
        else:
            decision.update(action=HOLD, reason="scan_not_permitted:" + reason)
        return decision
    decision["reason"] = "no_collision_valid_progress_candidate"
    return decision


def path_status(grid, traversed, ctx, settings, max_age):
    """Straight path segment (k = 0, L = 1.5 m) validity and occupied cells in the 3 m corridor."""
    heading = ctx.path_heading
    length = min(max(settings["lengths_m"]), max(math.dist(ctx.target, ctx.goal), 1e-6))
    end = endpoint(ctx.target, heading, length)
    valid, reason, _, _ = collision_check(grid, traversed, ctx.target, end, ctx, settings, max_age)
    corridor_end = endpoint(ctx.target, heading, settings["trigger_range_m"])
    states = grid.segment_states(ctx.target, corridor_end, settings["r_infl_m"])
    return {"valid": valid, "reason": reason, "end": end, "length": length,
            "occupied_in_corridor": bool(np.count_nonzero(states == OCCUPIED))}


def traversed_mask(grid, positions, r_body):
    """Cells within R_body of any earlier position estimate (the vehicle itself occupied them)."""
    mask = np.zeros(grid.cells.shape, dtype=bool)
    for x, y in positions:
        window, _ = grid._window((x,), (y,), r_body)
        if window is None:
            continue
        ii, jj, cx, cy = window
        inside = np.hypot(cx - x, cy - y) <= r_body
        mask[ii[inside], jj[inside]] = True
    return mask
