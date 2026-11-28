"""P8 recovery search on the observed map (contract `p8-navigation-recovery-v1` §4 R2, R3; S7).

Pure functions. The C-space restates the frozen L2 rule cell by cell, with the accepted `r_infl_m`:
* a cell is usable only if every cell whose centre lies within `r_infl_m` of its centre is observed FREE, or
  traversed and not OCCUPIED;
* the cell must also lie inside the geofence by the margin `r_infl_m`, as `planner.inside_fence` requires of a
  segment's ends.

**The search is conservative, never authoritative.** Every segment it proposes is checked again, before it is
executed, by the frozen `planner.collision_check` on the snapshot of that tick (S1). It runs once per HOVER
episode (S7).
"""
from collections import deque
import math
import time

import numpy as np

from gwm_obstacle_avoidance.local_map import FREE, OCCUPIED, UNKNOWN


def disk(radius_cells):
    r = int(math.ceil(radius_cells))
    return [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if di * di + dj * dj <= radius_cells ** 2 + 1e-9]


def dilate(mask, radius_cells):
    """Cells within `radius_cells` (centre to centre) of any True cell."""
    out = np.zeros_like(mask, dtype=bool)
    nx, ny = mask.shape
    for di, dj in disk(radius_cells):
        src = mask[max(0, -di):nx - max(0, di), max(0, -dj):ny - max(0, dj)]
        out[max(0, di):nx - max(0, -di), max(0, dj):ny - max(0, -dj)] |= src
    return out


def known_free(grid, traversed):
    return (grid.cells == FREE) | (traversed & (grid.cells != OCCUPIED))


def fence_mask(grid, origin, geofence, margin):
    cx = grid.x_min + (np.arange(grid.nx) + 0.5) * grid.resolution - origin[0]
    cy = grid.y_min + (np.arange(grid.ny) + 0.5) * grid.resolution - origin[1]
    X, Y = np.meshgrid(cx, cy, indexing="ij")
    return (X >= geofence["x_min_m"] + margin - 1e-9) & (X <= geofence["x_max_m"] - margin + 1e-9) & \
        (np.abs(Y) <= geofence["y_abs_max_m"] - margin + 1e-9)


def c_space(grid, traversed, r_infl, origin, geofence):
    """Usable cells under the L2 rule (see the module docstring)."""
    return ~dilate(~known_free(grid, traversed), r_infl / grid.resolution) & fence_mask(grid, origin, geofence, r_infl)


def bfs(ok, start):
    """8-connected reachable set and parents from `start` (i, j); parents[i, j] is the flat index of the parent."""
    seen = np.zeros(ok.shape, dtype=bool)
    parent = np.full(ok.shape, -1, dtype=np.int64)
    if start is None or not ok[start]:
        return seen, parent
    nx, ny = ok.shape
    seen[start] = True
    queue = deque([start])
    while queue:
        i, j = queue.popleft()
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                a, b = i + di, j + dj
                if (di or dj) and 0 <= a < nx and 0 <= b < ny and ok[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    parent[a, b] = i * ny + j
                    queue.append((a, b))
    return seen, parent


def path_to(parent, start, cell, ny):
    """The cell path start -> cell from `bfs` parents (start included), or None."""
    path = [cell]
    while path[-1] != start:
        p = int(parent[path[-1]])
        if p < 0:
            return None
        path.append((p // ny, p % ny))
    return path[::-1]


def bearing(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def quantize(angle, step):
    return wrap(round(angle / step) * step)


def episode_search(grid, traversed, target, goal, origin, geofence, r_infl, dead_end, clock=time.perf_counter):
    """The once-per-episode search (S7).

    It returns the reachable usable set, the frontier, the escape target and the cell path to it, with its own
    wall time. The escape target is the goal if reachable, else the reachable frontier cell with the least goal
    distance outside the dead-end mask."""
    t0 = clock()
    ok = c_space(grid, traversed, r_infl, origin, geofence)
    start = grid.index(*target)
    reach, parent = bfs(ok, start)
    frontier = reach & dilate(grid.cells == UNKNOWN, 1.0)
    cx = grid.x_min + (np.arange(grid.nx) + 0.5) * grid.resolution
    cy = grid.y_min + (np.arange(grid.ny) + 0.5) * grid.resolution
    X, Y = np.meshgrid(cx, cy, indexing="ij")
    goal_d = np.hypot(X - goal[0], Y - goal[1])
    gcell = grid.index(*goal)
    target_cell, kind = None, None
    if gcell is not None and reach[gcell]:
        target_cell, kind = gcell, "goal"
    else:
        candidates = frontier & ~dead_end
        if candidates.any():
            flat = int(np.argmin(np.where(candidates, goal_d, np.inf)))
            target_cell, kind = (flat // grid.ny, flat % grid.ny), "frontier"
    path = path_to(parent, start, target_cell, grid.ny) if target_cell is not None else None
    return {"reach": reach, "frontier": frontier, "goal_distance": goal_d, "target_cell": target_cell,
            "target_kind": kind, "path": path, "start_cell": start, "start_usable": bool(start is not None and ok[start]),
            "wall_s": clock() - t0}


def waypoints(grid, path, start_xy, max_length, step_check):
    """Greedy segments along a cell path. From each point, take the furthest path cell within `max_length` whose
    straight segment passes `step_check(p, q)`. Returns the list of segment ends; it stops when no segment
    passes."""
    if not path:
        return []
    centres = [grid.center(i, j) for i, j in path[1:]]
    out, here, k = [], start_xy, 0
    while k < len(centres):
        best = None
        for m in range(k, len(centres)):
            if math.dist(here, centres[m]) > max_length + 1e-9:
                break
            if step_check(here, centres[m]):
                best = m
        if best is None:
            break
        out.append(centres[best])
        here, k = centres[best], best + 1
    return out
