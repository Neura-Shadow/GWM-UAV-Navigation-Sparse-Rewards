"""P4 Gate A 45031345: vectorized perception must give exactly the outputs of the per-ray/per-column code.

The reference functions below are the implementations frozen in candidate 45031345 (commit 9604a04),
copied verbatim. Scans are compared float for float and grids byte for byte.
"""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
from gwm_obstacle_avoidance import local_map, perception  # noqa: E402
from gwm_obstacle_avoidance.local_map import FREE, OCCUPIED  # noqa: E402
from gwm_sensor_adapter import depth as depth_module  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402

SENSORS = syn.sensors()
EXTRINSIC = Extrinsic(SENSORS["optical_origin_base_flu_m"], identity=SENSORS["extrinsic_id"])
P = syn.p4_config()["perception"]


def reference_mark_ray(grid, start, end, hit):
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(length / (grid.resolution / 2))))
    end_index = grid.index(*end)
    for n in range(steps + 1):
        t = n / steps
        index = grid.index(start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))
        if index is None or (hit and index == end_index):
            continue
        if grid.cells[index] != OCCUPIED:
            grid.cells[index] = FREE
    if hit and end_index is not None:
        grid.cells[end_index] = OCCUPIED


def reference_extract_scan(image, calibration, extrinsic, position_ned, q_wxyz, settings, near_m, far_m, ground_up):
    stride, cap = settings["column_stride"], settings["free_cap_m"]
    below, above = settings["band_below_m"], settings["band_above_m"]
    height, width = image.shape
    columns = np.arange(stride // 2, width, stride)
    rows = np.arange(height)
    fx, fy, cx, cy = calibration["fx"], calibration["fy"], calibration["cx"], calibration["cy"]
    rotation, translation = perception.camera_transform(extrinsic, position_ned, q_wxyz)
    vehicle_enu = np.array([position_ned[1], position_ned[0], -position_ned[2]], dtype=float)
    depth = image[:, columns].astype(float)
    codes = depth_module.classify(depth, near_m, far_m)
    uu, vv = np.meshgrid(columns, rows)
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
            rays.append({"u": int(u), "azimuth": azimuth, "kind": "hit", "range_m": float(distance[index, k]), "end": point})
            continue
        band_codes = codes[band, k]
        if band_codes.size == 0:
            rays.append({"u": int(u), "azimuth": azimuth, "kind": "unknown", "range_m": None, "end": None})
            continue
        if np.any(band_codes == depth_module.TOO_CLOSE):
            end = (camera_xy[0] + near_m * unit[0], camera_xy[1] + near_m * unit[1])
            rays.append({"u": int(u), "azimuth": azimuth, "kind": "too_close", "range_m": near_m, "end": end})
            continue
        if np.any(band_codes == depth_module.INVALID_NAN):
            rays.append({"u": int(u), "azimuth": azimuth, "kind": "unknown", "range_m": None, "end": None})
            continue
        limit = cap
        valid = band_codes == depth_module.VALID
        if valid.any():
            limit = min(limit, float(distance[band, k][valid].min()))
        if limit < near_m:
            rays.append({"u": int(u), "azimuth": azimuth, "kind": "unknown", "range_m": None, "end": None})
            continue
        end = (camera_xy[0] + limit * unit[0], camera_xy[1] + limit * unit[1])
        rays.append({"u": int(u), "azimuth": azimuth, "kind": "free", "range_m": limit, "end": end})
    return camera_xy, rays


def reference_integrate(grid, camera_xy, rays, max_hit_range, near_m):
    for ray in rays:
        if ray["kind"] not in ("hit", "free"):
            if ray["kind"] == "too_close":
                index = grid.index(*ray["end"])
                if index is not None:
                    grid.cells[index] = 2
            continue
        start = (camera_xy[0] + near_m * math.cos(ray["azimuth"]), camera_xy[1] + near_m * math.sin(ray["azimuth"]))
        if ray["kind"] == "hit":
            if ray["range_m"] <= max_hit_range:
                reference_mark_ray(grid, start, ray["end"], hit=True)
        else:
            reference_mark_ray(grid, start, ray["end"], hit=False)


def scenes():
    rng = np.random.default_rng(11)
    layouts = [spec["boxes"] for group in ("scenarios", "ground_fixtures") for spec in syn.scenarios()[group].values()]
    layouts.append([{"center": [0.6, 0.1], "size": [0.3, 0.3]}, {"center": [1.5, -0.8], "size": [0.4, 2.0]}])
    for li, boxes in enumerate(layouts):
        for step in range(4):
            x, y = 0.9 * step + rng.uniform(-0.2, 0.2), rng.uniform(-1.0, 1.0)
            z = 0.24 if step == 0 else 2.0 + rng.uniform(-0.3, 0.3)
            yaw = rng.uniform(-math.pi, math.pi) if step == 3 else rng.uniform(-0.6, 0.6)
            image = syn.render((x, y, z), yaw, syn.boxes_3d(boxes), ground=step != 1).astype(np.float32)
            if (li + step) % 3 == 0:  # NaN and too-close patches
                image[rng.integers(0, 480, 400), rng.integers(0, 640, 400)] = np.nan
                image[200:280, int(rng.integers(0, 600)):][:, :30] = 0.05
            yield li, (x, y, z), yaw, image, 0.0 if step % 2 else -0.05


def new_grid():
    return local_map.Grid.around((0.0, 0.0), syn.p4_config()["geofence"], P["grid_margin_m"], P["grid_resolution_m"])


def test_vectorized_scan_and_grid_equal_the_reference_exactly():
    kinds, grids = set(), {}
    for li, vehicle, yaw, image, ground_up in scenes():
        position, q = syn.px4_pose(vehicle, yaw)
        args = (image, syn.CAL, EXTRINSIC, position, q, P, SENSORS["near_m"], SENSORS["far_m"], ground_up)
        expected = reference_extract_scan(*args)
        got = perception.extract_scan(*args)
        assert got == expected  # floats compared exactly
        kinds |= {r["kind"] for r in got[1]}
        new, old = grids.setdefault(li, (new_grid(), new_grid()))
        perception.integrate(new, *got, P["max_hit_range_m"], SENSORS["near_m"])
        reference_integrate(old, *expected, P["max_hit_range_m"], SENSORS["near_m"])
        assert np.array_equal(new.cells, old.cells)
    assert kinds == {"hit", "free", "unknown", "too_close"}


def test_mark_rays_equals_sequential_mark_ray_including_off_grid_and_degenerate_rays():
    rng = np.random.default_rng(3)
    batch, sequential = local_map.Grid(-1.0, -1.0, 40, 30, 0.1), local_map.Grid(-1.0, -1.0, 40, 30, 0.1)
    batch.cells[5:9, 5:9] = sequential.cells[5:9, 5:9] = OCCUPIED  # pre-existing occupancy is never cleared
    starts, ends, hits = [], [], []
    for _ in range(500):
        s = tuple(rng.uniform(-1.5, 3.5, 2))
        e = tuple(np.array(s) + rng.uniform(-3, 3, 2) * rng.choice([0.0, 0.01, 1.0]))
        starts.append(s), ends.append(e), hits.append(bool(rng.integers(0, 2)))
        reference_mark_ray(sequential, s, e, hits[-1])
    batch.mark_rays(starts, ends, hits)
    assert np.array_equal(batch.cells, sequential.cells)
    assert (batch.cells == OCCUPIED).any() and (batch.cells == FREE).any()


@pytest.mark.parametrize("start,end,hit", [((0.0, 0.0), (2.0, 0.0), True), ((0.0, 0.05), (2.0, 0.05), True),
                                           ((0.0, 0.0), (3.0, 0.0), False), ((0.33, 0.21), (0.33, 0.21), True)])
def test_single_mark_ray_equals_the_reference(start, end, hit):
    a, b = local_map.Grid(-1.0, -1.0, 50, 20, 0.1), local_map.Grid(-1.0, -1.0, 50, 20, 0.1)
    a.mark_ray(start, end, hit)
    reference_mark_ray(b, start, end, hit)
    assert np.array_equal(a.cells, b.cells)
