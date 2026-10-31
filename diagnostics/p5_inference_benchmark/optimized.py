"""Experimental CPU-D variant of the P5 decision path (diagnostics only; isolated from the frozen candidate).

Two changes, both meant to be bit-identical to the production code, and checked on every benchmark call:

* `polar_profile`: the production per-heading loop (13 headings x ~15 small NumPy calls) evaluated as one
  2-D array expression. Per element it performs the same IEEE operations in the same order (`math.cos` and
  `math.sin` per heading as in production, then multiply, add, subtract, divide, floor), so the result is
  identical; only the NumPy call overhead changes. The step array is cached per (cap, step).
* `Forward`: the same three float64 dense layers through the same BLAS routine, writing into preallocated
  buffers (`np.matmul(..., out=)`, in-place add and ReLU) instead of allocating six temporaries per call.
"""
import math

import numpy as np

from gwm_obstacle_avoidance import planner
from gwm_obstacle_avoidance.local_map import FREE, OCCUPIED

_STEPS = {}


def polar_profile(grid, traversed, start, path_heading, settings, profile):
    cap, ds = profile["cap_m"], profile["step_m"]
    s = _STEPS.get((cap, ds))
    if s is None:
        s = _STEPS[(cap, ds)] = np.arange(1, int(round(cap / ds)) + 1) * ds
    n = settings["heading_steps"]
    step = math.radians(settings["heading_step_deg"])
    headings = [planner.wrap(path_heading + k * step) for k in range(-n, n + 1)]
    cos = np.array([math.cos(h) for h in headings])[:, None]
    sin = np.array([math.sin(h) for h in headings])[:, None]
    i = np.floor((start[0] + s * cos - grid.x_min) / grid.resolution).astype(np.int64)
    j = np.floor((start[1] + s * sin - grid.y_min) / grid.resolution).astype(np.int64)
    inside = (i >= 0) & (i < grid.nx) & (j >= 0) & (j < grid.ny)
    states = np.full(i.shape, -1, dtype=np.int64)
    states[inside] = grid.cells[i[inside], j[inside]]
    walked = np.zeros(i.shape, dtype=bool)
    walked[inside] = traversed[i[inside], j[inside]]
    hit = states == OCCUPIED
    nonfree = ~((states == FREE) | (walked & inside & ~hit))
    occupied = np.where(hit.any(axis=1), s[np.argmax(hit, axis=1)], cap)
    blocked = np.where(nonfree.any(axis=1), s[np.argmax(nonfree, axis=1)], cap)
    return np.concatenate([occupied, blocked]) / cap


class Forward:
    def __init__(self, policy):
        w = policy._weights
        self.w = w
        rows = 26
        self.h1 = np.empty((rows, w["W1"].shape[1]))
        self.h2 = np.empty((rows, w["W2"].shape[1]))
        self.out = np.empty((rows, w["W3"].shape[1]))

    def __call__(self, policy, x):
        w, h1, h2, out = self.w, self.h1, self.h2, self.out
        np.matmul(x, w["W1"], out=h1)
        np.add(h1, w["b1"], out=h1)
        np.maximum(h1, 0.0, out=h1)
        np.matmul(h1, w["W2"], out=h2)
        np.add(h2, w["b2"], out=h2)
        np.maximum(h2, 0.0, out=h2)
        np.matmul(h2, w["W3"], out=out)
        np.add(out, w["b3"], out=out)
        return out[:, 0].copy()  # never hand out the reused buffer
