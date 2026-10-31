"""Privileged training-time oracle (plan §4.3, operator decision D1a). Ground truth is used here and
in evaluation only; it never enters the onboard features.

* Cost-to-go: Dijkstra on a 0.1 m lattice over the region the P4 planner can place a target in
  (the geofence shrunk by r_infl), 16-connected. Nodes closer than R_body + the supervisor clearance
  margin to a true box are impassable; nodes inside the r_infl inflation band are passable at
  `BAND_MULTIPLIER` times the length, so the field is the shortest path outside the inflated
  obstacles wherever one exists and stays finite for a target that is already inside the band.
* Candidate cost (metres, at transit speed):
  c_i = clipped length + V(endpoint) + LAMBDA * max(0, 0.6 - true segment clearance)
        + (speed / yaw rate) * max(0, |turn| - observable half-angle).
  The last term is the in-place yaw time before P4 may translate along the candidate (L4), so the
  label approximates time-to-goal.
* Target: softmax(-c / TAU) over the allowed (P4 progress-set) slots.
"""
import heapq
import math

import numpy as np

import p5_render

LATTICE_M = 0.1
BAND_MULTIPLIER = 10.0
CLEARANCE_TARGET_M = 0.6
LAMBDA = 10.0
TAU_M = 0.10
UNREACHABLE_COST_M = 50.0
SEGMENT_SAMPLE_M = 0.02
_NEIGHBOURS = [(di, dj) for di in range(-2, 3) for dj in range(-2, 3)
               if (di, dj) != (0, 0) and math.gcd(abs(di), abs(dj)) == 1]  # 16-connected


class CostToGo:
    def __init__(self, boxes, config, strict=False):
        r_infl = config["planner"]["r_infl_m"]
        fence = config["geofence"]
        self.x0, self.y0 = fence["x_min_m"] + r_infl, -(fence["y_abs_max_m"] - r_infl)
        self.nx = int(round((fence["x_max_m"] - r_infl - self.x0) / LATTICE_M)) + 1
        self.ny = int(round((fence["y_abs_max_m"] - r_infl - self.y0) / LATTICE_M)) + 1
        xs = self.x0 + LATTICE_M * np.arange(self.nx)
        ys = self.y0 + LATTICE_M * np.arange(self.ny)
        self.boxes = boxes
        self.r_infl = r_infl
        self.hard_m = config["envelope"]["r_body_m"] + config["supervisor"]["min_clearance_margin_m"]
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        distance = p5_render.surface_distance(gx, gy, boxes)
        self.multiplier = np.where(distance > r_infl, 1.0, np.inf if strict else BAND_MULTIPLIER)
        self.multiplier[distance <= self.hard_m] = np.inf
        self.goal = tuple(config["mission"]["goal_enu_m"])
        self.value = self._dijkstra(self._node(self.goal))

    def _node(self, point):
        i = int(round((point[0] - self.x0) / LATTICE_M))
        j = int(round((point[1] - self.y0) / LATTICE_M))
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def _dijkstra(self, source):
        nx, ny = self.nx, self.ny
        m = self.multiplier.tolist()  # plain lists: this loop is scalar-bound
        value = [[math.inf] * ny for _ in range(nx)]
        if not math.isfinite(m[source[0]][source[1]]):
            return np.array(value)
        steps = [(di, dj, LATTICE_M * math.hypot(di, dj)) for di, dj in _NEIGHBOURS]
        value[source[0]][source[1]] = 0.0
        heap = [(0.0, source[0], source[1])]
        while heap:
            v, i, j = heapq.heappop(heap)
            if v > value[i][j]:
                continue
            here = m[i][j]
            for di, dj, length in steps:
                a, b = i + di, j + dj
                if not (0 <= a < nx and 0 <= b < ny):
                    continue
                worst = max(here, m[a][b])
                # A knight move passes between two intermediate nodes: charge the worst on the way.
                if di in (-2, 2):
                    worst = max(worst, m[i + di // 2][j], m[i + di // 2][b])
                elif dj in (-2, 2):
                    worst = max(worst, m[i][j + dj // 2], m[a][j + dj // 2])
                if worst == math.inf:
                    continue
                cost = v + length * worst
                if cost < value[a][b]:
                    value[a][b] = cost
                    heapq.heappush(heap, (cost, a, b))
        return np.array(value)

    def at(self, point):
        """V at an arbitrary point: the best of the 3x3 lattice nodes around it plus the straight hop."""
        if math.dist(point, self.goal) < 1e-9:
            return 0.0
        ci, cj = self._node(point)
        here = p5_render.surface_distance(point[0], point[1], self.boxes)
        hop = 1.0 if here > self.r_infl else BAND_MULTIPLIER
        best = math.inf
        for i in range(max(ci - 1, 0), min(ci + 2, self.nx)):
            for j in range(max(cj - 1, 0), min(cj + 2, self.ny)):
                if np.isfinite(self.value[i, j]):
                    node = (self.x0 + LATTICE_M * i, self.y0 + LATTICE_M * j)
                    best = min(best, self.value[i, j] + hop * math.dist(point, node))
        return best

    def feasible_from(self, point):
        return math.isfinite(self.at(point))


def segment_clearance(start, end, boxes):
    """True rotor-tip clearance along a straight segment (sampled every 2 cm, both ends included)."""
    if not boxes:
        return math.inf
    n = max(int(math.ceil(math.dist(start, end) / SEGMENT_SAMPLE_M)), 1)
    s = np.linspace(0.0, 1.0, n + 1)
    xs = start[0] + s * (end[0] - start[0])
    ys = start[1] + s * (end[1] - start[1])
    return float(p5_render.surface_distance(xs, ys, boxes).min()) - p5_render.R_BODY


def candidate_costs(rows, allowed, slots, ctx, field, boxes, settings, speed, yaw_rate):
    """Oracle cost per slot (NaN where the slot is not allowed)."""
    half = math.radians(settings["observable_half_angle_deg"])
    yaw_m_per_rad = speed / yaw_rate
    costs = np.full(len(allowed), np.nan)
    for row, slot in zip(rows, slots):
        if not allowed[slot]:
            continue
        value = field.at(row["end"])
        if not math.isfinite(value):
            value = UNREACHABLE_COST_M
        turn = abs(math.atan2(math.sin(row["heading"] - ctx.vehicle_yaw), math.cos(row["heading"] - ctx.vehicle_yaw)))
        clearance = segment_clearance(ctx.target, row["end"], boxes)
        costs[slot] = (row["length"] + value + LAMBDA * max(0.0, CLEARANCE_TARGET_M - clearance)
                       + yaw_m_per_rad * max(0.0, turn - half))
    return costs


def target_distribution(costs, allowed, tau=TAU_M):
    """softmax(-c / tau) over the allowed slots; zeros elsewhere."""
    out = np.zeros(costs.shape)
    c = costs[allowed]
    z = -(c - c.min()) / tau
    p = np.exp(z)
    out[allowed] = p / p.sum()
    return out
