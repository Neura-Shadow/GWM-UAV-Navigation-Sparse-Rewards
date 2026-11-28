"""P8 layout geometry: true clearance capacity and goal clearance (contract `p8-navigation-recovery-v1` §6).

Pure functions on a layout's box footprints, using the accepted P4 avoidance configuration as data. They restate the
measures of the read-only P8 analysis (`diagnostics/p8/p8_hover_timeout_analysis.py`, evidence `cde7fa52…`), so the
V4 selection rule uses exactly the measure that the P7-E evidence was described with. A test checks that both give
the same value on all 60 P7-E flights.

* **Feasible at radius r:** a start-to-goal path exists on a 0.05 m grid. Every point on it is more than r from
  every box footprint (axis-aligned rectangles) and inside the geofence by the fence margin (r unless given).
* **Clearance capacity:** the largest r, to 0.01 m by bisection in [0.2, 2.5] m, that is feasible. The fence margin is
  held at `r_infl` - 0.1 m (0.9 m), because the start lies exactly on the accepted 1.0 m fence margin.
* **Goal clearance:** the distance from the goal to the nearest box footprint.
"""
from collections import deque
import json
import math
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
CONFIG_PATH = REPO / "simulation/px4_gazebo_p4/configs/p4_avoidance.yaml"
CONFIG = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
RESOLUTION_M = 0.05
CAPACITY_BOUNDS_M = (0.2, 2.5)
CAPACITY_TOLERANCE_M = 0.01


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
    """8-connected reachable set of `ok` cells from `start`."""
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


def goal_of(origin):
    g = CONFIG["mission"]["goal_enu_m"]
    return origin[0] + g[0], origin[1] + g[1]


def truth_feasible(boxes, origin, radius, res=RESOLUTION_M, fence_margin=None):
    fence = CONFIG["geofence"]
    frame = Frame(origin[0] + fence["x_min_m"] - 0.5, origin[1] - fence["y_abs_max_m"] - 0.5,
                  int((fence["x_max_m"] - fence["x_min_m"] + 1.0) / res), int((2 * fence["y_abs_max_m"] + 1.0) / res), res)
    X, Y = np.meshgrid(frame.cx, frame.cy, indexing="ij")
    occupied = np.zeros((frame.nx, frame.ny), dtype=bool)
    for box in boxes:
        (cx, cy), (sx, sy) = box["center"], box["size"]
        occupied |= (np.abs(X - cx) <= sx / 2) & (np.abs(Y - cy) <= sy / 2)
    ok = ~dilate(occupied, radius / res) & frame.fence(origin, radius if fence_margin is None else fence_margin)
    start, gcell = frame.index(*origin), frame.index(*goal_of(origin))
    reach = bfs(ok, start)
    return {"start_usable": bool(ok[start]), "goal_usable": bool(ok[gcell]), "feasible": bool(reach[gcell])}


def clearance_capacity(boxes, origin=(0.0, 0.0), lo=CAPACITY_BOUNDS_M[0], hi=CAPACITY_BOUNDS_M[1], tol=CAPACITY_TOLERANCE_M):
    fm = CONFIG["planner"]["r_infl_m"] - 0.1
    if not truth_feasible(boxes, origin, lo, fence_margin=fm)["feasible"]:
        return None
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if truth_feasible(boxes, origin, mid, fence_margin=fm)["feasible"] else (lo, mid)
    return round(lo, 3)


def goal_clearance(boxes, origin=(0.0, 0.0)):
    goal = goal_of(origin)
    out = []
    for box in boxes:
        (cx, cy), (sx, sy) = box["center"], box["size"]
        dx, dy = max(abs(goal[0] - cx) - sx / 2, 0.0), max(abs(goal[1] - cy) - sy / 2, 0.0)
        out.append(math.hypot(dx, dy))
    return min(out) if out else None
