"""P4 planner observation -> frozen `(26, 46)` policy tensor and allowed mask (contract §3).

Pure function of the P4 candidate rows, the P4 context, the validated grid, the traversed mask and
the supervisor state; no ground truth. Candidate geometry is exactly P4's and is never changed here.
"""
import math

import numpy as np

from gwm_obstacle_avoidance import planner
from gwm_obstacle_avoidance.local_map import FREE, OCCUPIED

from .contracts import CANDIDATE_FEATURES, FEATURES, SLOTS, STATES


def slot_of(row, settings):
    return 2 * (row["k"] + settings["heading_steps"]) + settings["lengths_m"].index(row["nominal_length"])


def progress_qualified(row, ctx, settings):
    """P4 `plan`'s progress set membership, with its reaches-goal rule."""
    reaches = math.dist(row["end"], ctx.goal) < 1e-9 and row["progress"] > 0
    return bool(row["valid"] and (row["progress"] > settings["min_progress_m"] or reaches)), bool(reaches)


def polar_profile(grid, traversed, start, path_heading, settings, profile):
    """Per candidate heading: distance to the first OCCUPIED cell, then to the first non-FREE cell
    (traversed counts as FREE unless OCCUPIED; outside the grid is non-FREE), capped and scaled."""
    cap, ds = profile["cap_m"], profile["step_m"]
    s = np.arange(1, int(round(cap / ds)) + 1) * ds
    n = settings["heading_steps"]
    step = math.radians(settings["heading_step_deg"])
    occupied, nonfree = np.full(2 * n + 1, cap), np.full(2 * n + 1, cap)
    for idx, k in enumerate(range(-n, n + 1)):
        heading = planner.wrap(path_heading + k * step)
        i = np.floor((start[0] + s * math.cos(heading) - grid.x_min) / grid.resolution).astype(np.int64)
        j = np.floor((start[1] + s * math.sin(heading) - grid.y_min) / grid.resolution).astype(np.int64)
        inside = (i >= 0) & (i < grid.nx) & (j >= 0) & (j < grid.ny)
        states = np.full(s.size, -1, dtype=np.int64)
        states[inside] = grid.cells[i[inside], j[inside]]
        walked = np.zeros(s.size, dtype=bool)
        walked[inside] = traversed[i[inside], j[inside]]
        hit = states == OCCUPIED
        free = (states == FREE) | (walked & inside & ~hit)
        if hit.any():
            occupied[idx] = s[int(np.argmax(hit))]
        if (~free).any():
            nonfree[idx] = s[int(np.argmax(~free))]
    return np.concatenate([occupied, nonfree]) / cap


def build(rows, ctx, settings, grid, traversed, state, schema):
    """Returns (features (26, 46) float64, allowed (26,) bool)."""
    if len(rows) != SLOTS:
        raise ValueError("candidate_count")
    if state not in STATES:
        raise ValueError("state_not_a_planning_state:" + str(state))
    scale = {f["name"]: f.get("scale") for f in schema["candidate_features"] + schema["context_features"]}
    half = math.radians(settings["observable_half_angle_deg"])
    r_infl, cap = settings["r_infl_m"], settings["clearance_cap_m"]
    features = np.zeros((SLOTS, FEATURES), dtype=np.float64)
    allowed = np.zeros(SLOTS, dtype=bool)
    seen = set()
    for row in rows:
        slot = slot_of(row, settings)
        seen.add(slot)
        qualified, reaches = progress_qualified(row, ctx, settings)
        turn = abs(planner.wrap(row["heading"] - ctx.vehicle_yaw))
        clearance_term = min(row["clearance"] - r_infl, cap) / cap if row["valid"] else 0.0
        features[slot, :CANDIDATE_FEATURES] = (
            row["k"] / scale["k"], row["nominal_length"] / scale["nominal_length"], row["length"] / scale["clipped_length"],
            row["progress"] / scale["progress"], planner.deviation(row["end"], ctx) / scale["endpoint_deviation"],
            turn / scale["turn"], clearance_term, float(turn <= half), float(reaches), float(row["valid"]), float(qualified))
        allowed[slot] = qualified
    if seen != set(range(SLOTS)):
        raise ValueError("candidate_slots_incomplete")
    relative = ctx.vehicle_yaw - ctx.path_heading
    one_hot = [1.0 if state == name else 0.0 for name in STATES]
    context = np.concatenate([
        [math.dist(ctx.target, ctx.goal) / scale["goal_distance"], planner.deviation(ctx.target, ctx) / scale["target_deviation"],
         math.sin(relative), math.cos(relative)],
        one_hot,
        polar_profile(grid, traversed, ctx.target, ctx.path_heading, settings, schema["polar_profile"])])
    features[:, CANDIDATE_FEATURES:] = context
    if not np.isfinite(features).all():
        raise ValueError("feature_not_finite")
    return features, allowed
