"""Independent closed-form scene for P4 offline tests (not a test module).

Depth is rendered from first principles (pinhole, optical-axis Z, level vehicle,
axis-aligned boxes and the ground plane) without importing any production
geometry, so perception is checked against an independent reference.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "simulation/px4_gazebo"
P4 = REPO / "simulation/px4_gazebo_p4"
for path in (P4 / "ros2_ws/src/gwm_obstacle_avoidance", SIM / "ros2_ws/src/gwm_sensor_adapter",
             SIM / "ros2_ws/src/gwm_px4_control", P4 / "scripts", P4 / "validation", SIM / "validation", SIM / "scripts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

FX = 432.496042035043
CAL = {"fx": FX, "fy": FX, "cx": 320.0, "cy": 240.0, "identity": "synthetic"}
OPTICAL_FLU = (0.13233, 0.0, 0.02078)
FAR = 19.1


def sensors():
    return json.loads((SIM / "configs/p3_sensors.yaml").read_text())


def p4_config(**mission):
    config = json.loads((P4 / "configs/p4_avoidance.yaml").read_text())
    config["mission"].update(mission)
    return config


def scenarios():
    return json.loads((P4 / "configs/p4_scenarios.json").read_text())


def boxes_3d(boxes, height=4.0, center_z=2.0):
    """Scenario boxes (centre xy, size xy) -> (min corner, max corner) in world ENU."""
    out = []
    for box in boxes:
        cx, cy = box["center"]
        sx, sy = box["size"]
        out.append((np.array([cx - sx / 2, cy - sy / 2, center_z - height / 2]),
                    np.array([cx + sx / 2, cy + sy / 2, center_z + height / 2])))
    return out


def camera(vehicle_enu, yaw_enu):
    c, s = math.cos(yaw_enu), math.sin(yaw_enu)
    position = np.array([vehicle_enu[0] + OPTICAL_FLU[0] * c - OPTICAL_FLU[1] * s,
                         vehicle_enu[1] + OPTICAL_FLU[0] * s + OPTICAL_FLU[1] * c,
                         vehicle_enu[2] + OPTICAL_FLU[2]])
    forward, right, down = np.array([c, s, 0.0]), np.array([s, -c, 0.0]), np.array([0.0, 0.0, -1.0])
    return position, forward, right, down


def render(vehicle_enu, yaw_enu, boxes, columns=None, ground=True):
    """Optical-axis depth (640x480, float32). Unrendered columns are NaN; no return is +inf."""
    position, forward, right, down = camera(vehicle_enu, yaw_enu)
    cols = np.arange(640) if columns is None else np.asarray(columns)
    uu, vv = np.meshgrid(cols, np.arange(480))
    a, b = (uu - CAL["cx"]) / FX, (vv - CAL["cy"]) / FX
    dirs = forward + a[..., None] * right + b[..., None] * down  # unit optical Z: t is the depth
    best = np.full(a.shape, np.inf)
    if ground:
        dz = dirs[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -position[2] / dz
        best = np.where((dz < 0) & (t > 0), np.minimum(best, t), best)
    for lo, hi in boxes:
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - position) / dirs
            t2 = (hi - position) / dirs
        near = np.max(np.minimum(t1, t2), axis=-1)
        far = np.min(np.maximum(t1, t2), axis=-1)
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
