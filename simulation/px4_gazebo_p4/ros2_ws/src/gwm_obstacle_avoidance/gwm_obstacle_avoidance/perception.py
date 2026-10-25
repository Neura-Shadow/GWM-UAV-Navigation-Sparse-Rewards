"""Deterministic depth-to-scan obstacle perception (contract §5 P1–P3).

Frame intake, calibration lock, 10 Hz sim-time processing, health and PX4 state
association are the unchanged P3 `DepthProcessor`; this module only adds the
scan extraction and the local grid on frames whose association is `matched`.
Geometry uses the P3 conventions: optical depth Z, R_FLU_FROM_OPTICAL, FLU->FRD,
and PX4 Hamilton quaternions body FRD -> local NED.
"""
import math

import numpy as np

from gwm_sensor_adapter import depth as depth_module
from gwm_sensor_adapter.geometry import FLU_TO_FRD, quaternion_matrix
from gwm_sensor_adapter.pipeline import DepthProcessor

from .contracts import MAP_CONTRACT
from .local_map import Grid

HIT, FREE_RAY, UNKNOWN_RAY, TOO_CLOSE_RAY = "hit", "free", "unknown", "too_close"


def camera_transform(extrinsic, position_ned, q_wxyz):
    """Rotation and translation taking optical points to local ENU."""
    to_ned = quaternion_matrix(q_wxyz) @ FLU_TO_FRD
    rotation_ned = to_ned @ extrinsic.r
    translation_ned = to_ned @ extrinsic.t + np.asarray(position_ned, dtype=float)
    ned_to_enu = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    return ned_to_enu @ rotation_ned, ned_to_enu @ translation_ned


def extract_scan(image, calibration, extrinsic, position_ned, q_wxyz, settings, near_m, far_m, ground_up):
    """Per sampled column: azimuth (ENU, from +x counter-clockwise), kind, and end point.

    * hit: minimum horizontal range of VALID points inside the flight band;
    * too_close: a too-close pixel among the band rows (occupied at the near limit);
    * free: band rows all no-return (free to the cap) or limited by a farther valid return;
    * unknown: any NaN among the band rows, or nothing usable.
    """
    stride, cap = settings["column_stride"], settings["free_cap_m"]
    below, above = settings["band_below_m"], settings["band_above_m"]
    height, width = image.shape
    columns = np.arange(stride // 2, width, stride)
    rows = np.arange(height)
    fx, fy, cx, cy = calibration["fx"], calibration["fy"], calibration["cx"], calibration["cy"]
    rotation, translation = camera_transform(extrinsic, position_ned, q_wxyz)
    vehicle_enu = np.array([position_ned[1], position_ned[0], -position_ned[2]], dtype=float)
    depth = image[:, columns].astype(float)
    codes = depth_module.classify(depth, near_m, far_m)
    uu, vv = np.meshgrid(columns, rows)
    # Unit-depth optical directions for every sampled pixel, then local ENU directions.
    directions = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu, dtype=float)], axis=-1)
    world_dirs = directions @ rotation.T
    horizontal = np.hypot(world_dirs[..., 0], world_dirs[..., 1])
    tan_elevation = world_dirs[..., 2] / np.maximum(horizontal, 1e-12)
    band_rows = (tan_elevation >= -below / cap) & (tan_elevation <= above / cap)
    safe_depth = np.where(codes == depth_module.VALID, depth, 0.0)
    points = translation + world_dirs * safe_depth[..., None]
    rel_up = points[..., 2] - vehicle_enu[2]
    ground_ok = points[..., 2] - ground_up > settings["ground_exclusion_m"]
    in_band = (codes == depth_module.VALID) & (rel_up >= -below) & (rel_up <= above) & ground_ok
    distance = np.hypot(points[..., 0] - vehicle_enu[0], points[..., 1] - vehicle_enu[1])
    camera_xy = (float(translation[0]), float(translation[1]))
    center_row = int(round(cy))
    rays = []
    for k, u in enumerate(columns):
        direction = world_dirs[center_row, k]
        azimuth = math.atan2(direction[1], direction[0])
        unit = (math.cos(azimuth), math.sin(azimuth))
        band = band_rows[:, k]
        hits = in_band[:, k]
        if hits.any():
            index = int(np.argmin(np.where(hits, distance[:, k], np.inf)))
            point = (float(points[index, k, 0]), float(points[index, k, 1]))
            rays.append({"u": int(u), "azimuth": azimuth, "kind": HIT, "range_m": float(distance[index, k]), "end": point})
            continue
        band_codes = codes[band, k]
        if band_codes.size == 0:
            rays.append({"u": int(u), "azimuth": azimuth, "kind": UNKNOWN_RAY, "range_m": None, "end": None})
            continue
        if np.any(band_codes == depth_module.TOO_CLOSE):
            end = (camera_xy[0] + near_m * unit[0], camera_xy[1] + near_m * unit[1])
            rays.append({"u": int(u), "azimuth": azimuth, "kind": TOO_CLOSE_RAY, "range_m": near_m, "end": end})
            continue
        if np.any(band_codes == depth_module.INVALID_NAN):
            rays.append({"u": int(u), "azimuth": azimuth, "kind": UNKNOWN_RAY, "range_m": None, "end": None})
            continue
        limit = cap
        valid = band_codes == depth_module.VALID
        if valid.any():  # returns outside the band (e.g. ground on the ground) bound the free distance
            limit = min(limit, float(distance[band, k][valid].min()))
        if limit < near_m:
            rays.append({"u": int(u), "azimuth": azimuth, "kind": UNKNOWN_RAY, "range_m": None, "end": None})
            continue
        end = (camera_xy[0] + limit * unit[0], camera_xy[1] + limit * unit[1])
        rays.append({"u": int(u), "azimuth": azimuth, "kind": FREE_RAY, "range_m": limit, "end": end})
    return camera_xy, rays


def integrate(grid, camera_xy, rays, max_hit_range, near_m):
    """Free space starts at the camera's near clip: nothing closer than `near_m` is observed
    (a near-clipped object may be invisible rather than reported too close)."""
    for ray in rays:
        if ray["kind"] not in (HIT, FREE_RAY):
            if ray["kind"] == TOO_CLOSE_RAY:
                index = grid.index(*ray["end"])
                if index is not None:
                    grid.cells[index] = 2
            continue
        start = (camera_xy[0] + near_m * math.cos(ray["azimuth"]), camera_xy[1] + near_m * math.sin(ray["azimuth"]))
        if ray["kind"] == HIT:
            if ray["range_m"] <= max_hit_range:
                grid.mark_ray(start, ray["end"], hit=True)
        else:
            grid.mark_ray(start, ray["end"], hit=False)


class P4Processor(DepthProcessor):
    """The unchanged P3 processor plus the P4 scan and grid on matched frames."""

    def __init__(self, sensors_config, p4_config, run_id):
        super().__init__(sensors_config, run_id, require_state=True)
        self.p4 = p4_config
        self.grid = None
        self.anchor = None
        self.ground_up = None
        self.consecutive_unusable = 0
        self.unusable_reasons = {}
        self.usable = 0

    def observe(self, frame, sim_now, wall_now):
        observation = super().observe(frame, sim_now, wall_now)
        association = observation["association"]
        snapshot = {"contract": MAP_CONTRACT, "run_id": self.run_id, "observation_id": observation["observation_id"],
                    "receipt_id": frame["receipt_id"], "acquisition_stamp_ns": frame["stamp_ns"],
                    "acquisition_sim_s": frame["stamp_ns"] / 1e9, "processing_sim_s": sim_now,
                    "association_status": association.get("status"),
                    "calibration_identity": observation["calibration_identity"], "usable": False, "reason": None}
        if association.get("status") != "matched":
            return self._unusable(snapshot, "association:" + str(association.get("status")))
        position, q = association["position_ned"], association["q_wxyz"]
        if self.grid is None:
            # Anchor while landed: grid around the takeoff point; ground from the pinned base height.
            self.anchor = (float(position[1]), float(position[0]))
            self.ground_up = -float(position[2]) - float(self.config["base_origin_model_m"][2])
            perception = self.p4["perception"]
            self.grid = Grid.around(self.anchor, self.p4["geofence"], perception["grid_margin_m"],
                                    perception["grid_resolution_m"])
        camera_xy, rays = extract_scan(frame["image"], self.calibration.calibration, self.extrinsic, position, q,
                                       self.p4["perception"], self.config["near_m"], self.config["far_m"], self.ground_up)
        integrate(self.grid, camera_xy, rays, self.p4["perception"]["max_hit_range_m"], self.config["near_m"])
        self.consecutive_unusable = 0
        self.usable += 1
        kinds = {}
        for ray in rays:
            kinds[ray["kind"]] = kinds.get(ray["kind"], 0) + 1
        snapshot.update(usable=True, position_ned=list(position), q_wxyz=list(q),
                        reset_epoch=association.get("reset_epoch"), anchor_enu=list(self.anchor),
                        ground_up_m=self.ground_up, camera_enu=list(camera_xy), ray_kinds=kinds,
                        scan=[[round(r["azimuth"], 6), r["kind"], None if r["range_m"] is None else round(r["range_m"], 4)]
                              for r in rays],
                        grid=self.grid.encode(), counts=self.grid.counts())
        return snapshot

    def _unusable(self, snapshot, reason):
        self.consecutive_unusable += 1
        self.unusable_reasons[reason] = self.unusable_reasons.get(reason, 0) + 1
        snapshot.update(reason=reason, consecutive_unusable=self.consecutive_unusable,
                        grid=self.grid.encode() if self.grid is not None else None)
        return snapshot
