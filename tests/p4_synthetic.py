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


def truth_clearance(vehicle_xy, boxes, r_body=0.39):
    """Horizontal distance from the rotor-tip cylinder to the nearest box surface (negative = overlap)."""
    best = math.inf
    for box in boxes:
        cx, cy = box["center"]
        sx, sy = box["size"]
        dx = max(abs(vehicle_xy[0] - cx) - sx / 2, 0.0)
        dy = max(abs(vehicle_xy[1] - cy) - sy / 2, 0.0)
        best = min(best, math.hypot(dx, dy) - r_body)
    return best


def closed_loop(boxes, sim_limit=200.0, scan_deg=360, stop_perception_at=None):
    """Offline closed loop with ideal tracking (vehicle = commanded target, height 2 m).

    Independent renderer -> production perception (scan + grid) at 10 Hz -> production supervisor
    and apply_action at 20 Hz. Returns the trajectory, supervisor and minimum truth clearance."""
    from gwm_obstacle_avoidance import local_map, perception, planner, supervisor
    from gwm_sensor_adapter.geometry import Extrinsic
    S, C = sensors(), p4_config()
    P = C["perception"]
    extrinsic = Extrinsic(S["optical_origin_base_flu_m"])
    columns = np.arange(2, 640, 4)
    grid = local_map.Grid.around((0.0, 0.0), C["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    scene = boxes_3d(boxes)

    def perceive(xy, yaw):
        vehicle = (xy[0], xy[1], 2.0)
        position, q = px4_pose(vehicle, yaw)
        image = render(vehicle, yaw, scene, columns=columns)
        cam, rays = perception.extract_scan(image, CAL, extrinsic, position, q, P, S["near_m"], S["far_m"], 0.0)
        perception.integrate(grid, cam, rays, P["max_hit_range_m"], S["near_m"])

    sim, yaw, target = 0.0, 0.0, (0.0, 0.0)
    positions = [target]
    for d in range(0, scan_deg + 1):  # MAP_SCAN: 1 deg per 10 Hz frame
        perceive(target, math.radians(d))
        sim += 0.1
    sup = supervisor.Supervisor(C, sim)
    health = {"status": "fresh", "calibration_identity": "synthetic"}
    last_acq = sim
    trajectory, decisions, min_clearance = [], [], math.inf
    dt, tick = 0.05, 0
    while sim < sim_limit:
        if tick % 2 == 0 and (stop_perception_at is None or sim < stop_perception_at):
            perceive(target, yaw)
            last_acq = sim
        snapshot = {"acquisition_sim_s": last_acq, "calibration_identity": "synthetic"}
        traversed = planner.traversed_mask(grid, positions[-80:], C["envelope"]["r_body_m"])
        decision = sup.step({"sim": sim, "snapshot": snapshot, "grid": grid, "traversed": traversed,
                             "adapter_health": health if (stop_perception_at is None or sim < stop_perception_at + 1.0)
                             else {"status": "stale", "calibration_identity": "synthetic"},
                             "consecutive_unusable": 0, "reset_sim": None, "vehicle": target, "vehicle_yaw": yaw,
                             "target": target, "goal": tuple(C["mission"]["goal_enu_m"]), "origin": (0.0, 0.0),
                             "geofence": C["geofence"]})
        decisions.append(decision)
        if decision["state"] == supervisor.ABORT:
            break
        target, yaw = supervisor.apply_action(target, yaw, decision, 0.3, math.radians(10), dt,
                                              tuple(C["mission"]["goal_enu_m"]))
        if math.dist(positions[-1], target) > 0.02:
            positions.append(target)
        trajectory.append((sim, target, yaw, decision["state"]))
        min_clearance = min(min_clearance, truth_clearance(target, boxes))
        if math.dist(target, C["mission"]["goal_enu_m"]) < 1e-6:
            break
        sim += dt
        tick += 1
    return {"trajectory": trajectory, "supervisor": sup, "decisions": decisions, "min_clearance": min_clearance,
            "final": target, "sim": sim}
