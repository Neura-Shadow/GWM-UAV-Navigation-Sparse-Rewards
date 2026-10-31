"""Independent closed-form scene for offline P5 training (plan §4.1).

Depth is rendered from first principles (pinhole, optical-axis Z, level vehicle, axis-aligned 4 m
boxes and the ground plane) with the same camera and extrinsic as the P4 offline tests; no
production geometry is imported. Truth (boxes, clearance) is used for the oracle and for evaluation
only and never reaches the onboard features.
"""
import math

import numpy as np

FX = 432.496042035043
CAL = {"fx": FX, "fy": FX, "cx": 320.0, "cy": 240.0, "identity": "synthetic"}
OPTICAL_FLU = (0.13233, 0.0, 0.02078)
FAR = 19.1
R_BODY = 0.39
BOX_HEIGHT, BOX_CENTER_Z = 4.0, 2.0


def boxes_3d(boxes):
    """Layout boxes (centre xy, size xy) -> (min corner, max corner) in world ENU."""
    out = []
    for box in boxes:
        cx, cy = box["center"]
        sx, sy = box["size"]
        out.append((np.array([cx - sx / 2, cy - sy / 2, BOX_CENTER_Z - BOX_HEIGHT / 2]),
                    np.array([cx + sx / 2, cy + sy / 2, BOX_CENTER_Z + BOX_HEIGHT / 2])))
    return out


def camera(vehicle_enu, yaw_enu):
    c, s = math.cos(yaw_enu), math.sin(yaw_enu)
    position = np.array([vehicle_enu[0] + OPTICAL_FLU[0] * c - OPTICAL_FLU[1] * s,
                         vehicle_enu[1] + OPTICAL_FLU[0] * s + OPTICAL_FLU[1] * c,
                         vehicle_enu[2] + OPTICAL_FLU[2]])
    forward, right, down = np.array([c, s, 0.0]), np.array([s, -c, 0.0]), np.array([0.0, 0.0, -1.0])
    return position, forward, right, down


def render(vehicle_enu, yaw_enu, boxes3d, columns=None, ground=True):
    """Optical-axis depth (640x480, float32). Unrendered columns are NaN; no return is +inf.

    The vehicle is level, so a ray's horizontal direction depends only on its column and its
    vertical component only on its row: the slab test is evaluated per column (x, y) and per row
    (z) and broadcast, which is the per-pixel slab test with the same operands."""
    position, forward, right, down = camera(vehicle_enu, yaw_enu)
    cols = np.arange(640) if columns is None else np.asarray(columns)
    a = ((cols - CAL["cx"]) / FX)[None, :]
    b = ((np.arange(480) - CAL["cy"]) / FX)[:, None]
    dx = forward[0] + a * right[0]  # (1, C); down has no horizontal component
    dy = forward[1] + a * right[1]
    dz = b * down[2]                # (R, 1); forward and right are horizontal
    best = np.full((480, cols.size), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        if ground:
            t = -position[2] / dz
            best = np.where((dz < 0) & (t > 0), np.minimum(best, t), best)
        for lo, hi in boxes3d:
            tx1, tx2 = (lo[0] - position[0]) / dx, (hi[0] - position[0]) / dx
            ty1, ty2 = (lo[1] - position[1]) / dy, (hi[1] - position[1]) / dy
            tz1, tz2 = (lo[2] - position[2]) / dz, (hi[2] - position[2]) / dz
            near_xy = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            far_xy = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            near = np.maximum(near_xy, np.minimum(tz1, tz2))
            far = np.minimum(far_xy, np.maximum(tz1, tz2))
            hit = (far >= near) & (near > 0)
            best = np.where(hit, np.minimum(best, near), best)
    best = np.where(best >= FAR, np.inf, best)
    image = np.full((480, 640), np.nan, dtype="<f4")
    image[:, cols] = best.astype("<f4")
    return image


def px4_pose(vehicle_enu, yaw_enu):
    """PX4 local NED position and Hamilton quaternion (body FRD -> NED) for a level vehicle."""
    yaw_ned = math.pi / 2 - yaw_enu
    return ([vehicle_enu[1], vehicle_enu[0], -vehicle_enu[2]],
            [math.cos(yaw_ned / 2), 0.0, 0.0, math.sin(yaw_ned / 2)])


def box_distance(x, y, box):
    """Horizontal distance from points to a box surface (0 inside); x, y scalars or arrays."""
    cx, cy = box["center"]
    sx, sy = box["size"]
    dx = np.maximum(np.abs(np.asarray(x, dtype=float) - cx) - sx / 2, 0.0)
    dy = np.maximum(np.abs(np.asarray(y, dtype=float) - cy) - sy / 2, 0.0)
    return np.hypot(dx, dy)


def surface_distance(x, y, boxes):
    """Distance to the nearest box surface (inf with no boxes)."""
    best = np.full(np.broadcast(np.asarray(x, dtype=float), np.asarray(y, dtype=float)).shape, np.inf)
    for box in boxes:
        best = np.minimum(best, box_distance(x, y, box))
    return best


def truth_clearance(vehicle_xy, boxes, r_body=R_BODY):
    """Rotor-tip clearance: horizontal distance from the body cylinder to the nearest box surface."""
    return float(surface_distance(vehicle_xy[0], vehicle_xy[1], boxes)) - r_body
