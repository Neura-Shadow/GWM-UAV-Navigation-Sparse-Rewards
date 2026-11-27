"""P8 proposal: read-only investigation of the P7-E hover-timeout failures. Deletes and changes nothing.

It reads the accepted P7-E batch (candidate e9561f05, 3 block results, 60 flights) and, for every flight, the
run's own records. The 42 goal successes are the contrast group for the 18 `hover_timeout` failures. It uses no
frozen code path at runtime: the accepted P4 avoidance config is read as data, and the planner's rules are
restated here only to measure the recorded situations, never to fly.

Per flight:
* **Supervisor timeline:** time in each state, HOVER entries and exits, and the final HOVER episode.
* **Forward progress:** distance to the goal over time; the last time the best distance improved by 0.10 m; net
  progress, path length and radius of gyration in the 30 s before the final HOVER.
* **Candidate availability:** the share of ticks with no progress candidate, and the actions and reasons
  recorded while hovering.
* **Left/right switching:** sign changes of the chosen candidate side (k), per minute of transit.
* **Timing:** stale decisions, snapshot age, and native-clock stalls (RTF < 0.5) near the final HOVER entry.
* **P5 score margins:** from the policy ledger, the gap between the best and second-best allowed candidate.
* **Replanning opportunity:**
  * the last usable local map before the final HOVER entry is decoded;
  * a C-space with the accepted inflation r_infl is built: a cell is usable only if every cell within r_infl is
    FREE or traversed, as the planner's L2 rule requires, and inside the fence margin;
  * this measures what a multi-step planner on the same map, with the same clearance, could have reached;
  * it also counts in-place out-of-cone moves (outside the planner's +/-60 degrees) that satisfy the same rule.
* **Ground-truth geometric feasibility:**
  * the true box footprints are taken from `p7/layout.json`;
  * the free space is checked for a start-to-goal path at clearance radii 1.00 m (the accepted r_infl), 0.79 m
    (r_body + yaw margin + tracking) and 0.49 m (r_body + the supervisor's minimum margin);
  * the fence margin equals each radius, as the planner applies it.

Usage: cd /tmp && /usr/bin/python3 p8_hover_timeout_analysis.py <output.json>   (exclusive create)
"""
import base64
from collections import Counter, deque
import json
import math
from pathlib import Path
import statistics
import sys
import zlib

import numpy as np

ROOT = Path.home() / "uav_autonomy"
REPO = Path(__file__).resolve().parents[2]
SHA = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
STOPPED_SHA = "38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be"
CONFIG = json.loads((REPO / "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml").read_text())
UNKNOWN, FREE, OCCUPIED = 0, 1, 2
WINDOW_S = 30.0


def jsonl(path):
    path = Path(path)
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def dist(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {"n": len(values), "min": min(values), "median": statistics.median(values), "max": max(values)}


def disk(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if di * di + dj * dj <= radius_cells ** 2 + 1e-9]


def dilate(mask, radius_cells):
    out = np.zeros_like(mask)
    nx, ny = mask.shape
    for di, dj in disk(radius_cells):
        src = mask[max(0, -di):nx - max(0, di), max(0, -dj):ny - max(0, dj)]
        out[max(0, di):nx - max(0, -di), max(0, dj):ny - max(0, -dj)] |= src
    return out


def bfs(ok, start):
    """8-connected reachable set of `ok` cells from `start` (i, j)."""
    seen = np.zeros_like(ok)
    if not (0 <= start[0] < ok.shape[0] and 0 <= start[1] < ok.shape[1]) or not ok[start]:
        return seen
    seen[start] = True
    queue = deque([start])
    while queue:
        i, j = queue.popleft()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                a, b = i + di, j + dj
                if (di or dj) and 0 <= a < ok.shape[0] and 0 <= b < ok.shape[1] and ok[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    queue.append((a, b))
    return seen


class Frame:
    """Cell centres of a grid in the mission frame."""

    def __init__(self, x_min, y_min, nx, ny, res):
        self.x_min, self.y_min, self.nx, self.ny, self.res = x_min, y_min, nx, ny, res
        self.cx = x_min + (np.arange(nx) + 0.5) * res
        self.cy = y_min + (np.arange(ny) + 0.5) * res

    def index(self, x, y):
        return int(math.floor((x - self.x_min) / self.res)), int(math.floor((y - self.y_min) / self.res))

    def fence(self, origin, margin):
        fence = CONFIG["geofence"]
        X, Y = np.meshgrid(self.cx - origin[0], self.cy - origin[1], indexing="ij")
        return (X >= fence["x_min_m"] + margin - 1e-9) & (X <= fence["x_max_m"] - margin + 1e-9) & \
            (np.abs(Y) <= fence["y_abs_max_m"] - margin + 1e-9)


def decode(grid):
    cells = np.frombuffer(zlib.decompress(base64.b64decode(grid["cells"])), dtype=np.uint8)
    return cells.reshape(grid["nx"], grid["ny"])


def traversed_mask(frame, positions):
    mask = np.zeros((frame.nx, frame.ny), dtype=bool)
    r = CONFIG["envelope"]["r_body_m"]
    for x, y in positions:
        i0, j0 = frame.index(x - r, y - r)
        i1, j1 = frame.index(x + r, y + r)
        for i in range(max(0, i0), min(frame.nx, i1 + 1)):
            for j in range(max(0, j0), min(frame.ny, j1 + 1)):
                if math.hypot(frame.cx[i] - x, frame.cy[j] - y) <= r:
                    mask[i, j] = True
    return mask


def segment_ok(ok, frame, start, heading, length):
    steps = max(2, int(length / (frame.res / 2)) + 1)
    for n in range(steps + 1):
        t = n / steps
        i, j = frame.index(start[0] + t * length * math.cos(heading), start[1] + t * length * math.sin(heading))
        if not (0 <= i < frame.nx and 0 <= j < frame.ny) or not ok[i, j]:
            return False
    return True


def replanning(lmap, positions, at, goal, origin, path_heading):
    """What the recorded map allowed at the final HOVER entry, under the accepted clearance rule."""
    g = lmap["grid"]
    frame = Frame(g["x_min_m"], g["y_min_m"], g["nx"], g["ny"], g["resolution_m"])
    cells = decode(g)
    r_cells = CONFIG["planner"]["r_infl_m"] / frame.res
    trav = traversed_mask(frame, positions)
    known_free = (cells == FREE) | (trav & (cells != OCCUPIED))
    fence = frame.fence(origin, CONFIG["planner"]["r_infl_m"])
    strict_ok = ~dilate(~known_free, r_cells) & fence           # L2: everything within r_infl FREE or traversed
    optimistic_ok = ~dilate(cells == OCCUPIED, r_cells) & fence  # unknown treated as free (exploration)
    start = frame.index(*at)
    gcell = frame.index(*goal)
    reach_strict, reach_opt = bfs(strict_ok, start), bfs(optimistic_ok, start)
    X, Y = np.meshgrid(frame.cx, frame.cy, indexing="ij")
    D = np.hypot(X - goal[0], Y - goal[1])
    here = math.dist(at, goal)
    best_strict = float(D[reach_strict].min()) if reach_strict.any() else None
    unknown_near = dilate(cells == UNKNOWN, 1.0)
    frontier = reach_strict & unknown_near
    out_cone, in_cone = 0, 0
    for deg in range(0, 360, 10):
        heading = math.radians(deg)
        rel = abs((heading - path_heading + math.pi) % (2 * math.pi) - math.pi)
        for length in CONFIG["planner"]["lengths_m"]:
            if segment_ok(strict_ok, frame, at, heading, length):
                if rel > math.radians(CONFIG["planner"]["heading_step_deg"] * CONFIG["planner"]["heading_steps"]) + 1e-6:
                    out_cone += 1
                else:
                    in_cone += 1
    inside = lambda c: 0 <= c[0] < frame.nx and 0 <= c[1] < frame.ny
    # The terminal segment (frozen target -> goal): what the L2 rule finds within r_infl of it.
    p0, p1 = np.array(at), np.array(goal)
    seg = p1 - p0
    t = np.clip(((X - p0[0]) * seg[0] + (Y - p0[1]) * seg[1]) / max(float(seg @ seg), 1e-12), 0, 1)
    near = np.hypot(X - (p0[0] + t * seg[0]), Y - (p0[1] + t * seg[1])) <= CONFIG["planner"]["r_infl_m"]
    blocking_unknown = near & ~known_free & (cells != OCCUPIED)
    terminal = {"length_m": float(np.hypot(*seg)), "occupied_cells": int((near & (cells == OCCUPIED)).sum()),
                "unknown_cells": int(blocking_unknown.sum()),
                "unknown_beyond_goal_x": int((blocking_unknown & (X > goal[0])).sum()),
                "unknown_lateral_over_0_39_m": int((blocking_unknown & (np.abs(Y - goal[1]) > CONFIG["envelope"]["r_body_m"])).sum())}
    return {"map_acquisition_sim_s": lmap["acquisition_sim_s"], "terminal_segment": terminal,
            "cells": {"unknown": int((cells == UNKNOWN).sum()), "free": int((cells == FREE).sum()),
                      "occupied": int((cells == OCCUPIED).sum())},
            "start_cell_usable_strict": bool(inside(start) and strict_ok[start]),
            "goal_reachable_strict": bool(inside(gcell) and reach_strict[gcell]),
            "goal_reachable_optimistic": bool(inside(gcell) and reach_opt[gcell]),
            "goal_distance_here_m": here,
            "best_reachable_goal_distance_strict_m": best_strict,
            "strict_gain_m": None if best_strict is None else here - best_strict,
            "reachable_cells_strict": int(reach_strict.sum()),
            "frontier_cells_strict": int(frontier.sum()),
            "frontier_best_goal_distance_m": float(D[frontier].min()) if frontier.any() else None,
            "valid_in_place_moves_in_cone": in_cone, "valid_in_place_moves_out_of_cone": out_cone}


def truth_feasible(layout, origin, radius, res=0.05, fence_margin=None):
    fence = CONFIG["geofence"]
    frame = Frame(origin[0] + fence["x_min_m"] - 0.5, origin[1] - fence["y_abs_max_m"] - 0.5,
                  int((fence["x_max_m"] - fence["x_min_m"] + 1.0) / res), int((2 * fence["y_abs_max_m"] + 1.0) / res), res)
    X, Y = np.meshgrid(frame.cx, frame.cy, indexing="ij")
    occupied = np.zeros((frame.nx, frame.ny), dtype=bool)
    for box in layout["boxes"]:
        (cx, cy), (sx, sy) = box["center"], box["size"]
        occupied |= (np.abs(X - cx) <= sx / 2) & (np.abs(Y - cy) <= sy / 2)
    ok = ~dilate(occupied, radius / res) & frame.fence(origin, radius if fence_margin is None else fence_margin)
    goal = (origin[0] + CONFIG["mission"]["goal_enu_m"][0], origin[1] + CONFIG["mission"]["goal_enu_m"][1])
    start, gcell = frame.index(*origin), frame.index(*goal)
    reach = bfs(ok, start)
    return {"start_usable": bool(ok[start]), "goal_usable": bool(ok[gcell]), "feasible": bool(reach[gcell])}


def clearance_capacity(layout, origin, lo=0.2, hi=2.5, tol=0.01):
    """The largest obstacle clearance radius at which a true start-to-goal path exists. The fence margin is held at
    0.9 m (just under the accepted 1.0 m), because the start lies exactly on the 1.0 m fence margin and would
    otherwise cap every layout at about 1.0 m."""
    fm = CONFIG["planner"]["r_infl_m"] - 0.1
    if not truth_feasible(layout, origin, lo, fence_margin=fm)["feasible"]:
        return None
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if truth_feasible(layout, origin, mid, fence_margin=fm)["feasible"] else (lo, mid)
    return round(lo, 3)


def goal_clearance(layout, goal):
    """Distance from the goal to the nearest true box footprint."""
    out = []
    for box in layout["boxes"]:
        (cx, cy), (sx, sy) = box["center"], box["size"]
        dx, dy = max(abs(goal[0] - cx) - sx / 2, 0.0), max(abs(goal[1] - cy) - sy / 2, 0.0)
        out.append(math.hypot(dx, dy))
    return min(out) if out else None


def clock_stalls(run):
    rows = [r for r in jsonl(Path(run) / "source/clock.jsonl") if r.get("sim_s", 0) > 0]
    out = []
    for a, b in zip(rows, rows[1:]):
        wall = (b["receipt_monotonic_ns"] - a["receipt_monotonic_ns"]) / 1e9
        if wall > 0 and (b["sim_s"] - a["sim_s"]) / wall < 0.5:
            out.append(b["sim_s"])
    return out


def side_switches(decisions):
    sides = [int(math.copysign(1, d["plan"]["best"]["k"])) for d in decisions
             if d.get("action") in ("TRANSLATE", "YAW_TO") and (d.get("plan") or {}).get("best") and d["plan"]["best"]["k"]]
    return sum(1 for a, b in zip(sides, sides[1:]) if a != b), len(sides)


def margins(ledger):
    out = []
    for r in ledger:
        scores, allowed = r.get("scores"), r.get("allowed")
        if not r.get("invoked") or not scores or not allowed:
            continue
        s = sorted((v for v, a in zip(scores, allowed) if a), reverse=True)
        if len(s) >= 2:
            out.append(s[0] - s[1])
    return out


def analyse(entry, block):
    run = Path(entry["run"])
    record = json.loads((run / "p7/flight-record.json").read_text())
    layout = json.loads((run / "p7/layout.json").read_text())
    result = json.loads((run / "controller-result.json").read_text())
    p4 = result["p4"]
    goal, origin = tuple(p4["goal_enu"]), tuple(p4["origin_enu"])
    decisions = sorted((d for d in jsonl(run / "p4/decisions.jsonl") if d.get("event") == "p4_decision" and "state" in d),
                       key=lambda d: d["sim_s"])
    transitions = p4["supervisor_transitions"]
    end = transitions[-1]["sim_s"] if p4.get("abort_reason") else (p4.get("goal_reached_sim_s") or decisions[-1]["sim_s"])
    states = Counter()
    for a, b in zip(decisions, decisions[1:]):
        if a["sim_s"] < end:
            states[a["state"]] += min(b["sim_s"], end) - a["sim_s"]
    hovers = [t for t in transitions if t["to"] == "HOVER"]
    final_hover = hovers[-1] if (hovers and p4.get("abort_reason") == "hover_timeout") else None
    positions = [(d["sim_s"], tuple(d["vehicle_enu"])) for d in decisions if d.get("vehicle_enu")]
    gd = [(t, math.dist(p, goal)) for t, p in positions]
    best, improved = float("inf"), None
    for t, d in gd:
        if d < best - 0.10:
            best, improved = d, t
    row = {"slot": entry["slot"], "controller": entry["controller"], "block": block, "run": run.name,
           "layout": layout["item"], "family": layout["layout"]["family"], "boxes": len(layout["layout"]["boxes"]),
           "outcome": record["outcome"], "abort_reason": record["metrics"]["abort_reason"],
           "time_in_state_sim_s": {k: round(v, 3) for k, v in states.items()},
           "hover_entries": [{"sim_s": t["sim_s"], "from": t["from"], "reason": t["reason"]} for t in hovers],
           "hover_exits": sum(1 for t in transitions if t["from"] == "HOVER" and t["to"] != "ABORT"),
           "min_goal_distance_m": min((d for _, d in gd), default=None),
           "last_best_improvement_sim_s": improved, "end_sim_s": end,
           "no_improvement_before_end_s": None if improved is None else end - improved,
           "ticks": len(decisions),
           "ticks_without_progress_candidate": sum(1 for d in decisions if (d.get("plan") or {}).get("candidates_progress") == 0),
           "stale_decisions": dict(Counter(d["stale"] for d in decisions if d.get("stale"))),
           "snapshot_age_sim_s": dist([d["sim_s"] - d["snapshot_acquisition_sim_s"] for d in decisions
                                       if d.get("snapshot_acquisition_sim_s") is not None]),
           "metrics": {k: record["metrics"].get(k) for k in ("path_length_m", "detour_ratio", "min_truth_clearance_m",
                                                             "hover_time_sim_s", "time_to_goal_sim_s", "final_goal_distance_m")}}
    switches, chosen = side_switches([d for d in decisions if d["sim_s"] < end])
    transit = max(1e-9, end - (decisions[0]["sim_s"] if decisions else end))
    row["side_switches"] = {"switches": switches, "chosen_sided": chosen, "per_min": switches / transit * 60}
    row["yaw_to_decisions"] = sum(1 for d in decisions if d.get("action") == "YAW_TO" and d["sim_s"] < end)
    stalls = clock_stalls(run)
    row["clock_stalls_below_0_5"] = len(stalls)
    if final_hover:
        t0 = final_hover["sim_s"]
        win = [(t, p) for t, p in positions if t0 - WINDOW_S <= t <= t0]
        path = sum(math.dist(a[1], b[1]) for a, b in zip(win, win[1:]))
        net = (math.dist(win[0][1], goal) - math.dist(win[-1][1], goal)) if win else None
        pts = np.array([p for _, p in win]) if win else np.zeros((0, 2))
        gyr = float(np.sqrt(((pts - pts.mean(axis=0)) ** 2).sum(axis=1).mean())) if len(pts) else None
        during = [d for d in decisions if t0 <= d["sim_s"] <= end]
        at = next((tuple(d["target_enu"]) for d in decisions if d["sim_s"] >= t0 and d.get("target_enu")), None)
        lmaps = [m for m in jsonl(run / "p4/local_map.jsonl") if m.get("usable") and m["acquisition_sim_s"] <= t0]
        entry_plan = next(((d.get("plan") or {}) for d in decisions if d["sim_s"] >= t0), {})
        row["final_hover"] = {
            "entry_sim_s": t0, "from": final_hover["from"], "reason": final_hover["reason"],
            "position_enu": at, "goal_distance_m": math.dist(at, goal) if at else None,
            "x_progress_m": (at[0] - origin[0]) if at else None,
            "candidates_at_entry": {k: entry_plan.get(k) for k in ("candidates_valid", "candidates_progress")},
            "actions_while_hovering": dict(Counter("%s:%s" % (d.get("action"), d.get("reason")) for d in during)),
            "window_before_s": WINDOW_S, "net_progress_m": net, "path_length_m": path,
            "straightness": (net / path) if path > 1e-6 and net is not None else None, "radius_of_gyration_m": gyr,
            "stalls_within_5_s_before": sum(1 for s in stalls if t0 - 5 <= s <= t0),
            "nearest_stall_s": min((abs(s - t0) for s in stalls), default=None),
            "replanning": replanning(lmaps[-1], [p for t, p in positions if t <= t0], at, goal, origin,
                                     math.atan2(goal[1] - at[1], goal[0] - at[0])) if (lmaps and at) else None}
    row["truth_feasible"] = {str(r): truth_feasible(layout["layout"], origin, r) for r in (1.0, 0.79, 0.49)}
    row["clearance_capacity_m"] = clearance_capacity(layout["layout"], origin)
    row["goal_clearance_truth_m"] = goal_clearance(layout["layout"], goal)
    if row.get("final_hover"):
        row["final_hover"]["mode"] = "near_goal" if (row["final_hover"]["goal_distance_m"] or 99) < 0.6 else "mid_course"
    if entry["controller"] == "p5":
        ledger = jsonl(run / "p5/policy-ledger.jsonl")
        m = margins(ledger)
        row["p5"] = {"ticks": len(ledger), "margin": dist(m),
                     "margin_below_0_25_fraction": (sum(1 for x in m if x < 0.25) / len(m)) if m else None,
                     "disagreements": sum(1 for r in ledger if r.get("choice_slot") is not None and r.get("choice_slot") != r.get("p4_slot")),
                     "interventions": dict(Counter(r.get("intervention") for r in ledger if r.get("intervention")))}
    return row


def flights(sha):
    out = []
    for block in (1, 2, 3):
        path = ROOT / "state" / ("p7e-engineering-30-block-%02d-%s.result.json" % (block, sha))
        if path.exists():
            out += [(f, block) for f in json.loads(path.read_text())["flights"]]
    return out


def summary(rows):
    fail = [r for r in rows if r["outcome"] != "goal_success"]
    ok = [r for r in rows if r["outcome"] == "goal_success"]
    pairs = {}
    for r in rows:
        pairs.setdefault(r["slot"], {})[r["controller"]] = r
    both = [s for s, p in pairs.items() if all(p[c]["outcome"] != "goal_success" for c in ("p4", "p5"))]
    fh = [r["final_hover"] for r in fail if r.get("final_hover")]
    rp = [h["replanning"] for h in fh if h.get("replanning")]
    same_spot = []
    for s in both:
        a, b = pairs[s]["p4"].get("final_hover"), pairs[s]["p5"].get("final_hover")
        if a and b and a["position_enu"] and b["position_enu"]:
            same_spot.append({"slot": s, "distance_between_final_hover_positions_m": math.dist(a["position_enu"], b["position_enu"])})
    feas = lambda group, r: Counter(x["truth_feasible"][r]["feasible"] for x in group)
    return {
        "flights": len(rows), "failures": len(fail), "failures_by_controller": dict(Counter(r["controller"] for r in fail)),
        "abort_reasons": dict(Counter(r["abort_reason"] for r in fail)),
        "both_failed_slots": sorted(both), "discordant_slots": sorted(s for s in pairs if s not in both and
                                                                       any(pairs[s][c]["outcome"] != "goal_success" for c in pairs[s])),
        "failures_by_family": dict(Counter(r["family"] for r in fail)), "flights_by_family": dict(Counter(r["family"] for r in rows)),
        "final_hover_reason": dict(Counter(h["reason"] for h in fh)),
        "final_hover_from": dict(Counter(h["from"] for h in fh)),
        "final_hover_candidates_at_entry": dict(Counter("%s/%s" % (h["candidates_at_entry"]["candidates_valid"], h["candidates_at_entry"]["candidates_progress"]) for h in fh)),
        "actions_while_hovering": dict(sum((Counter(h["actions_while_hovering"]) for h in fh), Counter())),
        "final_hover_goal_distance_m": dist([h["goal_distance_m"] for h in fh]),
        "final_hover_x_progress_m": dist([h["x_progress_m"] for h in fh]),
        "no_improvement_before_end_s": {"failures": dist([r["no_improvement_before_end_s"] for r in fail])},
        "straightness_30_s_before_hover": dist([h["straightness"] for h in fh]),
        "net_progress_30_s_before_hover_m": dist([h["net_progress_m"] for h in fh]),
        "radius_of_gyration_30_s_before_hover_m": dist([h["radius_of_gyration_m"] for h in fh]),
        "stalls_within_5_s_before_final_hover": dict(Counter(h["stalls_within_5_s_before"] for h in fh)),
        "nearest_stall_to_final_hover_s": dist([h["nearest_stall_s"] for h in fh]),
        "stale_decisions": {"failures": dict(sum((Counter(r["stale_decisions"]) for r in fail), Counter())),
                            "successes": dict(sum((Counter(r["stale_decisions"]) for r in ok), Counter()))},
        "max_snapshot_age_sim_s": {"failures": dist([(r["snapshot_age_sim_s"] or {}).get("max") for r in fail]),
                                   "successes": dist([(r["snapshot_age_sim_s"] or {}).get("max") for r in ok])},
        "side_switches_per_min": {"failures": dist([r["side_switches"]["per_min"] for r in fail]),
                                  "successes": dist([r["side_switches"]["per_min"] for r in ok])},
        "yaw_to_decisions": {"failures": dist([r["yaw_to_decisions"] for r in fail]), "successes": dist([r["yaw_to_decisions"] for r in ok])},
        "hover_exits": {"failures": dist([r["hover_exits"] for r in fail]), "successes": dist([r["hover_exits"] for r in ok])},
        "time_in_state_sim_s": {g: {s: dist([r["time_in_state_sim_s"].get(s, 0.0) for r in grp])
                                    for s in ("NORMAL", "OBSTACLE_DETECTED", "AVOIDING", "RESUME", "HOVER")}
                                for g, grp in (("failures", fail), ("successes", ok))},
        "replanning_at_final_hover": {
            "maps": len(rp),
            "goal_reachable_strict": dict(Counter(x["goal_reachable_strict"] for x in rp)),
            "goal_reachable_optimistic": dict(Counter(x["goal_reachable_optimistic"] for x in rp)),
            "strict_gain_m": dist([x["strict_gain_m"] for x in rp]),
            "strict_gain_over_0_5_m": sum(1 for x in rp if (x["strict_gain_m"] or 0) > 0.5),
            "frontier_cells_strict": dist([x["frontier_cells_strict"] for x in rp]),
            "valid_in_place_moves_out_of_cone": dist([x["valid_in_place_moves_out_of_cone"] for x in rp]),
            "valid_in_place_moves_in_cone": dist([x["valid_in_place_moves_in_cone"] for x in rp])},
        "truth_feasible": {r: {"failures": dict(feas(fail, r)), "successes": dict(feas(ok, r))} for r in ("1.0", "0.79", "0.49")},
        "failure_modes": dict(Counter(h["mode"] for h in fh)),
        "failure_modes_by_controller": dict(Counter("%s:%s" % (r["controller"], r["final_hover"]["mode"]) for r in fail if r.get("final_hover"))),
        "clearance_capacity_m": {"failures": dist([r["clearance_capacity_m"] for r in fail]),
                                 "successes": dist([r["clearance_capacity_m"] for r in ok]),
                                 "near_goal_failures": dist([r["clearance_capacity_m"] for r in fail if (r.get("final_hover") or {}).get("mode") == "near_goal"]),
                                 "mid_course_failures": dist([r["clearance_capacity_m"] for r in fail if (r.get("final_hover") or {}).get("mode") == "mid_course"])},
        "clearance_capacity_below_1_2_m": {"failures": sum(1 for r in fail if (r["clearance_capacity_m"] or 0) < 1.2),
                                           "successes": sum(1 for r in ok if (r["clearance_capacity_m"] or 0) < 1.2)},
        "goal_clearance_truth_m": {"failures": dist([r["goal_clearance_truth_m"] for r in fail]),
                                   "successes": dist([r["goal_clearance_truth_m"] for r in ok])},
        "near_goal_terminal_segment": [dict(slot=r["slot"], controller=r["controller"], goal_distance_m=r["final_hover"]["goal_distance_m"],
                                            **((r["final_hover"].get("replanning") or {}).get("terminal_segment") or {}))
                                       for r in fail if (r.get("final_hover") or {}).get("mode") == "near_goal"],
        "both_failed_final_hover_separation": same_spot,
        "p5_margin_median": {"failures": dist([((r.get("p5") or {}).get("margin") or {}).get("median") for r in fail if r["controller"] == "p5"]),
                             "successes": dist([((r.get("p5") or {}).get("margin") or {}).get("median") for r in ok if r["controller"] == "p5"])},
        "p5_margin_below_0_25_fraction": {"failures": dist([(r.get("p5") or {}).get("margin_below_0_25_fraction") for r in fail if r["controller"] == "p5"]),
                                          "successes": dist([(r.get("p5") or {}).get("margin_below_0_25_fraction") for r in ok if r["controller"] == "p5"])},
    }


def stopped_attempt():
    """Timing facts of the stopped attempt's slot 1 P5 flight (P2 stale_action RECOVERY), for comparison."""
    out = []
    for f, _ in flights(STOPPED_SHA):
        run = Path(f["run"])
        rec = json.loads((run / "p7/flight-record.json").read_text())
        if rec["outcome"] == "goal_success":
            continue
        cr = json.loads((run / "controller-result.json").read_text())
        rec_t = next((t["sim_s"] for t in cr.get("transitions", []) if t.get("to") == "RECOVERY"), None)
        stalls = clock_stalls(run)
        out.append({"run": run.name, "slot": f["slot"], "controller": f["controller"], "p2_failure": rec["metrics"]["p2_failure"],
                    "recovery_sim_s": rec_t, "stalls": len(stalls),
                    "stalls_within_2_s_before_recovery": sum(1 for s in stalls if rec_t is not None and rec_t - 2 <= s <= rec_t),
                    "min_interval_rtf": (rec["metrics"].get("rtf") or {}).get("min_interval_rtf")})
    return out


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    rows = [analyse(f, b) for f, b in flights(SHA)]
    value = {"contract": "p8-hover-timeout-analysis-v1", "read_only": True, "candidate": SHA,
             "config": {k: CONFIG[k] for k in ("planner", "supervisor", "envelope", "geofence")},
             "summary": summary(rows), "stopped_attempt_failures": stopped_attempt(), "flights": rows}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(value["summary"], indent=1, default=float))


if __name__ == "__main__":
    main(sys.argv[1:])
