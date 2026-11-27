"""P5 offline helpers (not a test module): import paths, interface-only models, lockstep closed loops.

Interface models use fixed-seed random weights with `trained: false`. They exercise the adapter,
runtime and shield only; they are never a learned policy and the runtime refuses them when a trained
model is required.
"""
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402  (sets the P2/P3/P4 import paths)

P5 = syn.REPO / "simulation/px4_gazebo_p5"
for path in (P5 / "ros2_ws/src/gwm_policy_control", P5 / "scripts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from gwm_policy_control import contracts, policy_runtime  # noqa: E402

SCHEMA = contracts.load_schema(P5 / "configs/p5_model_schema.json")
SCHEMA_SHA = contracts.schema_sha256(SCHEMA)


def interface_arrays(seed=0, scale=0.3):
    rng = np.random.default_rng(seed)
    return {name: rng.normal(scale=scale, size=shape) for name, shape in contracts.LAYER_SHAPES.items()}


def interface_metadata(trained=False, **changes):
    meta = {"architecture": contracts.ARCHITECTURE, "schema_sha256": SCHEMA_SHA, "trained": trained,
            "provenance": "interface test only: fixed-seed random weights, not a learned policy"}
    meta.update(changes)
    return meta


def interface_policy(seed=0):
    return policy_runtime.FrozenPolicy(interface_arrays(seed), interface_metadata(), sha256="interface-test")


def _world(boxes):
    from gwm_obstacle_avoidance import local_map, perception
    from gwm_sensor_adapter.geometry import Extrinsic
    S, C = syn.sensors(), syn.p4_config()
    P = C["perception"]
    extrinsic = Extrinsic(S["optical_origin_base_flu_m"])
    columns = np.arange(2, 640, 4)
    grid = local_map.Grid.around((0.0, 0.0), C["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    scene = syn.boxes_3d(boxes)

    def perceive(xy, yaw):
        vehicle = (xy[0], xy[1], 2.0)
        position, q = syn.px4_pose(vehicle, yaw)
        image = syn.render(vehicle, yaw, scene, columns=columns)
        cam, rays = perception.extract_scan(image, syn.CAL, extrinsic, position, q, P, S["near_m"], S["far_m"], 0.0)
        perception.integrate(grid, cam, rays, P["max_hit_range_m"], S["near_m"])
    return C, grid, perceive


def closed_loop(boxes, make_supervisor, shadow=None, sim_limit=200.0, stop_perception_at=None):
    """The P4 offline closed loop (ideal tracking, MAP_SCAN, 10 Hz perception, 20 Hz decisions) driven by
    `make_supervisor(config, sim)`. With `shadow`, a second supervisor receives identical inputs every
    tick and its outputs are recorded next to the driver's (lockstep parity)."""
    from gwm_obstacle_avoidance import planner, supervisor
    C, grid, perceive = _world(boxes)
    goal = tuple(C["mission"]["goal_enu_m"])
    sim, yaw, target = 0.0, 0.0, (0.0, 0.0)
    positions = [target]
    for d in range(0, 361):
        perceive(target, math.radians(d))
        sim += 0.1
    driver = make_supervisor(C, sim)
    other = shadow(C, sim) if shadow else None
    health = {"status": "fresh", "calibration_identity": "synthetic", "sim_s": 1e9}
    last_acq, tick, dt = sim, 0, 0.05
    outs, shadow_outs, trajectory, min_clearance = [], [], [], math.inf
    while sim < sim_limit:
        if tick % 2 == 0 and (stop_perception_at is None or sim < stop_perception_at):
            perceive(target, yaw)
            last_acq = sim
        inputs = {"sim": sim, "snapshot": {"acquisition_sim_s": last_acq, "calibration_identity": "synthetic"},
                  "grid": grid, "traversed": planner.traversed_mask(grid, positions[-80:], C["envelope"]["r_body_m"]),
                  "adapter_health": health if (stop_perception_at is None or sim < stop_perception_at + 1.0)
                  else {"status": "stale", "calibration_identity": "synthetic", "sim_s": 1e9},
                  "consecutive_unusable": 0, "reset_sim": None, "vehicle": target, "vehicle_yaw": yaw,
                  "target": target, "goal": goal, "origin": (0.0, 0.0), "geofence": C["geofence"]}
        out = driver.step(inputs)
        outs.append(out)
        if other is not None:
            shadow_outs.append(other.step(inputs))
        if out["state"] == supervisor.ABORT:
            break
        target, yaw = supervisor.apply_action(target, yaw, out, C["mission"]["transit_speed_m_s"], math.radians(10), dt, goal)
        if math.dist(positions[-1], target) > 0.02:
            positions.append(target)
        trajectory.append((sim, target, yaw, out["state"]))
        min_clearance = min(min_clearance, syn.truth_clearance(target, boxes))
        if math.dist(target, goal) < 1e-6:
            break
        sim += dt
        tick += 1
    return {"outs": outs, "shadow_outs": shadow_outs, "trajectory": trajectory, "final": target,
            "min_clearance": min_clearance, "driver": driver}


def strip_policy(out):
    return {k: v for k, v in out.items() if k != "policy"}
