"""Offline synthetic closed loop for P5 training and readiness (plan §4.1); no simulator.

Independent renderer -> P4 production perception (scan + grid) at 10 Hz -> a supervisor (P4's own,
or the P5 replica with a ranker) and P4 `apply_action` at 20 Hz -> a first-order PX4 horizontal
position loop (v = MPC_XY_P * error, MPC_XY_P 0.95). MAP_SCAN is the 360 deg in-place scan at
1 deg per 10 Hz frame of the P4 offline tests. The goal rule is P4Mission's: NORMAL, target at the
goal, within the goal tolerance and settled.
"""
import functools
import json
import math

import numpy as np

import p5_paths
import p5_render

PX4_XY_P = 0.95
SIM_LIMIT_S = 200.0
DT = 0.05
COLUMNS = np.arange(2, 640, 4)


@functools.lru_cache(maxsize=None)
def _configs_text():
    return tuple((path).read_text(encoding="utf-8") for path in (
        p5_paths.SIM / "configs/p3_sensors.yaml", p5_paths.P4 / "configs/p4_avoidance.yaml",
        p5_paths.SIM / "configs/p2_control.yaml"))


def configs():
    """(P3 sensors, P4 avoidance config, P2 control config), fresh copies."""
    return tuple(json.loads(text) for text in _configs_text())


class World:
    """True boxes, the production grid and perception at the vehicle's pose."""

    def __init__(self, boxes, sensors, config):
        from gwm_obstacle_avoidance import local_map, perception
        from gwm_sensor_adapter.geometry import Extrinsic
        self.perception, self.sensors, self.config = perception, sensors, config
        P = config["perception"]
        self.extrinsic = Extrinsic(sensors["optical_origin_base_flu_m"])
        self.grid = local_map.Grid.around((0.0, 0.0), config["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
        self.boxes, self.scene = boxes, p5_render.boxes_3d(boxes)
        self.height = config["mission"]["height_m"]

    def perceive(self, xy, yaw):
        P, S = self.config["perception"], self.sensors
        vehicle = (xy[0], xy[1], self.height)
        position, q = p5_render.px4_pose(vehicle, yaw)
        image = p5_render.render(vehicle, yaw, self.scene, columns=COLUMNS)
        cam, rays = self.perception.extract_scan(image, p5_render.CAL, self.extrinsic, position, q, P,
                                                 S["near_m"], S["far_m"], 0.0)
        self.perception.integrate(self.grid, cam, rays, P["max_hit_range_m"], S["near_m"])


def _scan_frames(config):
    return int(round(config["mission"]["map_scan_deg"])) + 1


def map_scan(boxes):
    """The grid after MAP_SCAN at the origin (deterministic for a layout; cached by callers)."""
    S, C, _ = configs()
    world = World(boxes, S, C)
    for d in range(_scan_frames(C)):
        world.perceive((0.0, 0.0), math.radians(d))
    return world.grid.cells.copy()


def run_episode(boxes, make_supervisor, px4_xy_p=PX4_XY_P, sim_limit=SIM_LIMIT_S, scan_cells=None):
    """One episode; `make_supervisor(config, start_sim)` builds the supervisor at MAP_SCAN -> TRANSIT.
    `scan_cells` is this layout's `map_scan` result, if already computed."""
    from gwm_obstacle_avoidance import planner, supervisor
    S, C, P2 = configs()
    world = World(boxes, S, C)
    m = C["mission"]
    goal, speed, yaw_rate = tuple(m["goal_enu_m"]), m["transit_speed_m_s"], math.radians(P2["yaw_ramp_deg_s"])
    heading_steps = C["planner"]["heading_steps"]
    sim, yaw, target = 0.0, 0.0, (0.0, 0.0)
    vehicle = target
    world.grid.cells[...] = map_scan(boxes) if scan_cells is None else scan_cells
    for _ in range(_scan_frames(C)):
        sim += 0.1
    sup = make_supervisor(C, sim)
    start = sim
    health = {"status": "fresh", "calibration_identity": "synthetic", "sim_s": 1e9}
    positions, last_acq, tick = [vehicle], sim, 0
    traversed, traversed_for = None, None
    stats = {"ticks": 0, "hover_ticks": 0, "actions": {}, "invocations": 0, "failures": 0, "failure_reasons": {},
             "outside_mask": 0, "executed_mismatch": 0, "disagreements": 0, "heading_disagreements": 0}
    path_m, track_max, min_clearance = 0.0, 0.0, math.inf
    outcome, reason = "sim_limit", None
    while sim < sim_limit:
        if tick % 2 == 0:
            world.perceive(vehicle, yaw)
            last_acq = sim
        if traversed_for != len(positions):  # positions only grow: the mask changes only when they do
            traversed = planner.traversed_mask(world.grid, positions[-80:], C["envelope"]["r_body_m"])
            traversed_for = len(positions)
        inputs = {"sim": sim, "snapshot": {"acquisition_sim_s": last_acq, "calibration_identity": "synthetic"},
                  "grid": world.grid, "traversed": traversed,
                  "adapter_health": health, "consecutive_unusable": 0, "reset_sim": None, "vehicle": vehicle,
                  "vehicle_yaw": yaw, "target": target, "goal": goal, "origin": (0.0, 0.0), "geofence": C["geofence"]}
        out = sup.step(inputs)
        stats["ticks"] += 1
        stats["actions"][out["action"]] = stats["actions"].get(out["action"], 0) + 1
        stats["hover_ticks"] += out["state"] == supervisor.HOVER
        record = out.get("policy")
        if record is not None:
            stats["invocations"] += 1
            if record["failure"] is not None:
                stats["failures"] += 1
                stats["failure_reasons"][record["failure"]] = stats["failure_reasons"].get(record["failure"], 0) + 1
            else:
                slot = record["choice_slot"]
                stats["outside_mask"] += not bool(record["allowed"][slot])
                best = (out.get("plan") or {}).get("best") or {}
                stats["executed_mismatch"] += best.get("k") != slot // 2 - heading_steps
                stats["disagreements"] += slot != record["p4_slot"]
                stats["heading_disagreements"] += slot // 2 != record["p4_slot"] // 2
        if out["state"] == supervisor.ABORT:
            outcome, reason = "abort", out["reason"]
            break
        target, yaw = supervisor.apply_action(target, yaw, out, speed, yaw_rate, DT, goal)
        previous = vehicle
        for _ in range(10):  # 5 ms integration of the position loop
            vehicle = (vehicle[0] + px4_xy_p * (target[0] - vehicle[0]) * DT / 10,
                       vehicle[1] + px4_xy_p * (target[1] - vehicle[1]) * DT / 10)
        path_m += math.dist(previous, vehicle)
        track_max = max(track_max, math.dist(target, vehicle))
        if math.dist(positions[-1], vehicle) > 0.02:
            positions.append(vehicle)
        min_clearance = min(min_clearance, p5_render.truth_clearance(vehicle, boxes))
        horizontal = math.dist(vehicle, goal)
        if (out["state"] == supervisor.NORMAL and math.dist(target, goal) < 1e-6 and horizontal <= m["goal_tolerance_m"]
                and px4_xy_p * math.dist(target, vehicle) <= m["settled_speed_m_s"]):
            outcome = "goal_reached"
            break
        sim += DT
        tick += 1
    return {"outcome": outcome, "reason": reason, "transit_s": round(sim - start, 6), "path_m": path_m,
            "min_clearance_m": min_clearance, "track_max_m": track_max, "final_target": list(target), **stats}
