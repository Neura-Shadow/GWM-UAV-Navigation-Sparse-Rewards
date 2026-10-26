"""2-D local occupancy grid in PX4 local ENU (x = east, y = north), contract P3 rule.

Cells are UNKNOWN, FREE or OCCUPIED. A hit overrides free; free never clears an
occupied cell (static world); a cell never covered stays UNKNOWN and is never
treated as free by any consumer.
"""
import base64
import math
import zlib

import numpy as np

UNKNOWN, FREE, OCCUPIED = 0, 1, 2


class Grid:
    def __init__(self, x_min, y_min, nx, ny, resolution, cells=None):
        self.x_min, self.y_min, self.nx, self.ny, self.resolution = float(x_min), float(y_min), int(nx), int(ny), float(resolution)
        self.cells = np.zeros((self.nx, self.ny), dtype=np.uint8) if cells is None else cells
        if self.cells.shape != (self.nx, self.ny) or self.cells.dtype != np.uint8:
            raise ValueError("grid_shape_mismatch")

    @classmethod
    def around(cls, anchor_xy, geofence, margin, resolution):
        """Grid covering the geofence (relative to anchor) plus a margin on every side."""
        x_min = anchor_xy[0] + geofence["x_min_m"] - margin
        y_min = anchor_xy[1] - geofence["y_abs_max_m"] - margin
        nx = int(math.ceil((geofence["x_max_m"] - geofence["x_min_m"] + 2 * margin) / resolution))
        ny = int(math.ceil((2 * geofence["y_abs_max_m"] + 2 * margin) / resolution))
        return cls(x_min, y_min, nx, ny, resolution)

    # Coordinates -------------------------------------------------------------------------------
    def index(self, x, y):
        i, j = int(math.floor((x - self.x_min) / self.resolution)), int(math.floor((y - self.y_min) / self.resolution))
        return (i, j) if 0 <= i < self.nx and 0 <= j < self.ny else None

    def center(self, i, j):
        return self.x_min + (i + 0.5) * self.resolution, self.y_min + (j + 0.5) * self.resolution

    def state(self, x, y):
        index = self.index(x, y)
        return UNKNOWN if index is None else int(self.cells[index])

    # Updates ----------------------------------------------------------------------------------
    def mark_ray(self, start, end, hit):
        """FREE along start->end (excluding an occupied cell), OCCUPIED at end when `hit`."""
        self.mark_rays([start], [end], [hit])

    def mark_rays(self, starts, ends, hits):
        """`mark_ray` for a batch, with identical cells. Each ray samples t = n/steps, n = 0..steps,
        steps = ceil(length / (resolution/2)), using the same float operations as a scalar loop.
        States only rise (UNKNOWN < FREE < OCCUPIED; free never clears occupied), so the result
        does not depend on the order of the rays."""
        if not starts:
            return
        half = self.resolution / 2
        steps = np.array([max(1, int(math.ceil(math.hypot(e[0] - s[0], e[1] - s[1]) / half)))
                          for s, e in zip(starts, ends)], dtype=np.int64)
        sx, sy = np.array([s[0] for s in starts], dtype=float), np.array([s[1] for s in starts], dtype=float)
        dx = np.array([e[0] - s[0] for s, e in zip(starts, ends)], dtype=float)
        dy = np.array([e[1] - s[1] for s, e in zip(starts, ends)], dtype=float)
        end_indices = [self.index(*e) for e in ends]
        end_ok = np.array([index is not None for index in end_indices])
        end_i = np.array([index[0] if index else -1 for index in end_indices], dtype=np.int64)
        end_j = np.array([index[1] if index else -1 for index in end_indices], dtype=np.int64)
        hit = np.asarray(hits, dtype=bool)
        ray = np.repeat(np.arange(len(starts)), steps + 1)
        n = np.arange(ray.size) - np.repeat(np.cumsum(steps + 1) - (steps + 1), steps + 1)
        t = n / steps[ray]
        i = np.floor((sx[ray] + t * dx[ray] - self.x_min) / self.resolution).astype(np.int64)
        j = np.floor((sy[ray] + t * dy[ray] - self.y_min) / self.resolution).astype(np.int64)
        keep = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        keep &= ~(hit[ray] & end_ok[ray] & (i == end_i[ray]) & (j == end_j[ray]))
        i, j = i[keep], j[keep]
        self.cells[i, j] = np.maximum(self.cells[i, j], FREE)
        occupied = hit & end_ok
        self.cells[end_i[occupied], end_j[occupied]] = OCCUPIED

    # Queries ----------------------------------------------------------------------------------
    def _window(self, xs, ys, radius):
        lo = self.index(min(xs) - radius, min(ys) - radius)
        hi = self.index(max(xs) + radius, max(ys) + radius)
        i0 = max(0, int(math.floor((min(xs) - radius - self.x_min) / self.resolution)))
        j0 = max(0, int(math.floor((min(ys) - radius - self.y_min) / self.resolution)))
        i1 = min(self.nx - 1, int(math.floor((max(xs) + radius - self.x_min) / self.resolution)))
        j1 = min(self.ny - 1, int(math.floor((max(ys) + radius - self.y_min) / self.resolution)))
        inside = lo is not None and hi is not None
        if i1 < i0 or j1 < j0:
            return None, False
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        cx = self.x_min + (ii + 0.5) * self.resolution
        cy = self.y_min + (jj + 0.5) * self.resolution
        return (ii, jj, cx, cy), inside

    @staticmethod
    def _segment_distance(cx, cy, p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        denominator = dx * dx + dy * dy
        if denominator == 0:
            return np.hypot(cx - p0[0], cy - p0[1])
        t = np.clip(((cx - p0[0]) * dx + (cy - p0[1]) * dy) / denominator, 0.0, 1.0)
        return np.hypot(cx - (p0[0] + t * dx), cy - (p0[1] + t * dy))

    def segment_cells(self, p0, p1, radius):
        """(i, j, states, inside) of every cell whose centre lies within `radius` of segment p0-p1.

        `inside` is False when part of that region lies outside the grid (unknown space)."""
        window, inside = self._window((p0[0], p1[0]), (p0[1], p1[1]), radius)
        if window is None:
            empty = np.array([], dtype=int)
            return empty, empty, np.array([], dtype=np.uint8), False
        ii, jj, cx, cy = window
        mask = self._segment_distance(cx, cy, p0, p1) <= radius
        return ii[mask], jj[mask], self.cells[ii[mask], jj[mask]], inside

    def segment_states(self, p0, p1, radius):
        """States of the cells within `radius` of p0-p1; any part outside the grid counts as UNKNOWN."""
        _, _, states, inside = self.segment_cells(p0, p1, radius)
        return states if inside else np.concatenate([states, np.array([UNKNOWN], dtype=np.uint8)])

    def occupied_distance(self, p0, p1, search):
        """Distance from segment p0-p1 to the nearest OCCUPIED cell centre, or None within `search`."""
        window, _ = self._window((p0[0], p1[0]), (p0[1], p1[1]), search)
        if window is None:
            return None
        ii, jj, cx, cy = window
        distance = self._segment_distance(cx, cy, p0, p1)
        occupied = (self.cells[ii, jj] == OCCUPIED) & (distance <= search)
        return float(distance[occupied].min()) if occupied.any() else None

    def counts(self):
        return {name: int(np.count_nonzero(self.cells == value))
                for name, value in (("unknown", UNKNOWN), ("free", FREE), ("occupied", OCCUPIED))}

    # Serialization ----------------------------------------------------------------------------
    def encode(self):
        payload = base64.b64encode(zlib.compress(self.cells.tobytes(order="C"), 6)).decode("ascii")
        return {"x_min_m": self.x_min, "y_min_m": self.y_min, "nx": self.nx, "ny": self.ny,
                "resolution_m": self.resolution, "encoding": "uint8-c-order-zlib-base64", "cells": payload}

    @classmethod
    def decode(cls, value):
        if value.get("encoding") != "uint8-c-order-zlib-base64":
            raise ValueError("unsupported_grid_encoding")
        raw = zlib.decompress(base64.b64decode(value["cells"]))
        cells = np.frombuffer(raw, dtype=np.uint8).reshape(value["nx"], value["ny"]).copy()
        return cls(value["x_min_m"], value["y_min_m"], value["nx"], value["ny"], value["resolution_m"], cells)
