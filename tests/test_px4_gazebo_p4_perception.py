"""P4 M1/M2 offline verification: configuration contract, local grid and depth perception."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
from gwm_obstacle_avoidance import contracts, local_map, perception  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402

SENSORS = syn.sensors()
EXTRINSIC = Extrinsic(SENSORS["optical_origin_base_flu_m"], identity=SENSORS["extrinsic_id"])
CONFIG = syn.p4_config()
PERCEPTION = CONFIG["perception"]
TOL = 0.10 + 0.07  # contract e_depth + e_grid


# M1: configuration contract --------------------------------------------------------------------
def test_repository_config_is_disabled_and_pins_contract_values(tmp_path):
    config = contracts.load_config(syn.P4 / "configs/p4_avoidance.yaml")
    assert config["enabled"] is False
    with pytest.raises(PermissionError):
        contracts.require_enabled(config, authorization_present=True)
    enabled = dict(config, enabled=True)
    with pytest.raises(PermissionError):
        contracts.require_enabled(enabled, authorization_present=False)
    contracts.require_enabled(enabled, authorization_present=True)
    for section, key, value in (("planner", "r_infl_m", 0.9), ("envelope", "r_body_m", 0.3),
                                ("supervisor", "stale_abort_sim_s", 12.0), ("geofence", "y_abs_max_m", 4.0)):
        changed = json.loads(json.dumps(config))
        changed[section][key] = value
        path = tmp_path / f"{section}_{key}.json"
        path.write_text(json.dumps(changed))
        with pytest.raises(ValueError, match="contract value changed"):
            contracts.load_config(path)


def test_p4_validation_config_names_the_p3_baseline_and_gates():
    validation = json.loads((syn.P4 / "configs/p4_validation.yaml").read_text())
    assert validation["contract"] == "p4-acceptance-v1"
    assert validation["contract_revision"].startswith("revisions 2 and 3")
    assert "d3c06b4" in validation["contract_revision"] and "ea2c9e3" in validation["contract_revision"]
    assert validation["p3_baseline_manifest_sha256"] == "1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c"
    assert [role for _, _, role in validation["gates"]["C"]] == ["qualification-1", "qualification-2", "qualification-3"]
    assert [s for _, s, _ in validation["gates"]["S"]] == ["S1", "S5", "S6"]
    assert validation["predecessor"] == {"B": "A", "S": "B", "C": "S"}
    assert validation["safety"]["min_truth_clearance_m"] == 0.20 and validation["safety"]["collisions_allowed"] == 0


# M2: grid ---------------------------------------------------------------------------------------
def grid():
    return local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], PERCEPTION["grid_margin_m"], PERCEPTION["grid_resolution_m"])


def test_grid_rays_hit_overrides_free_and_free_never_clears_occupied():
    g = grid()
    g.mark_ray((0.0, 0.0), (2.0, 0.0), hit=True)
    assert g.state(1.0, 0.0) == local_map.FREE and g.state(2.0, 0.0) == local_map.OCCUPIED
    g.mark_ray((0.0, 0.0), (3.0, 0.0), hit=False)
    assert g.state(2.0, 0.0) == local_map.OCCUPIED and g.state(2.5, 0.0) == local_map.FREE
    assert g.state(0.0, 2.0) == local_map.UNKNOWN


def test_grid_queries_treat_outside_as_unknown_and_round_trip():
    g = grid()
    g.mark_ray((0.0, 0.05), (2.0, 0.05), hit=True)  # along a row of cell centres
    states = g.segment_states((0.0, 0.05), (0.5, 0.05), 0.04)
    assert set(states.tolist()) == {local_map.FREE}
    assert local_map.UNKNOWN in g.segment_states((-2.9, 0.0), (-3.5, 0.0), 0.2)  # leaves the grid
    assert g.occupied_distance((0.0, 0.05), (1.0, 0.05), 1.5) == pytest.approx(1.0, abs=0.08)
    copy = local_map.Grid.decode(json.loads(json.dumps(g.encode())))
    assert np.array_equal(copy.cells, g.cells) and copy.x_min == g.x_min and copy.resolution == g.resolution


# M2: scan extraction against the independent renderer ------------------------------------------
def scan(vehicle, yaw, boxes, ground_up=0.0):
    position, q = syn.px4_pose(vehicle, yaw)
    image = syn.render(vehicle, yaw, syn.boxes_3d(boxes), columns=np.arange(2, 640, 4))
    return perception.extract_scan(image, syn.CAL, EXTRINSIC, position, q, PERCEPTION,
                                   SENSORS["near_m"], SENSORS["far_m"], ground_up)


@pytest.mark.parametrize("yaw", [0.0, math.radians(30), -math.radians(50), math.pi / 2])
def test_pillar_range_and_bearing_in_flight(yaw):
    pillar = 4.0
    center = (pillar * math.cos(yaw), pillar * math.sin(yaw))
    camera_xy, rays = scan((0.0, 0.0, 2.0), yaw, [{"center": center, "size": [0.4, 0.4]}])
    hits = [r for r in rays if r["kind"] == perception.HIT]
    assert hits, "pillar not detected"
    nearest = min(hits, key=lambda r: r["range_m"])
    # The nearest point of a 0.4 m axis-aligned box seen along its diagonal is within 0.29 m of its centre.
    assert pillar - 0.29 - TOL <= nearest["range_m"] <= pillar - 0.2 + TOL
    assert abs(perception.math.atan2(nearest["end"][1], nearest["end"][0]) - yaw) < math.radians(4)
    free = [r for r in rays if r["kind"] == perception.FREE_RAY]
    assert free and all(r["range_m"] == pytest.approx(PERCEPTION["free_cap_m"]) for r in free)
    assert not [r for r in rays if r["kind"] == perception.UNKNOWN_RAY]
    assert camera_xy == pytest.approx((0.13233 * math.cos(yaw), 0.13233 * math.sin(yaw)), abs=1e-9)


def test_ground_is_excluded_on_the_ground_and_obstacle_still_detected():
    vehicle = (0.0, 0.0, 0.24)
    _, rays = scan(vehicle, 0.0, [{"center": [2.0, 0.0], "size": [0.4, 0.4]}], ground_up=0.0)
    hits = [r for r in rays if r["kind"] == perception.HIT]
    assert hits and min(r["range_m"] for r in hits) == pytest.approx(1.8, abs=TOL)
    assert all(r["range_m"] < 2.3 for r in hits)  # no ground pixel became an obstacle


def test_invalid_pixels_in_the_band_make_the_column_unknown():
    position, q = syn.px4_pose((0.0, 0.0, 2.0), 0.0)
    image = syn.render((0.0, 0.0, 2.0), 0.0, [], columns=np.arange(2, 640, 4))
    image[:, 2] = np.nan
    image[:, 6] = 0.1  # too close
    _, rays = perception.extract_scan(image, syn.CAL, EXTRINSIC, position, q, PERCEPTION,
                                      SENSORS["near_m"], SENSORS["far_m"], 0.0)
    assert rays[0]["kind"] == perception.UNKNOWN_RAY and rays[1]["kind"] == perception.TOO_CLOSE_RAY


# M2: the P4 processor on top of the unchanged P3 DepthProcessor --------------------------------
def info_fields():
    fx = syn.FX
    return {"width": 640, "height": 480, "distortion_model": "plumb_bob", "d": [0.0] * 5,
            "k": [fx, 0, 320, 0, fx, 240, 0, 0, 1], "p": [fx, 0, 320, 0, 0, fx, 240, 0, 0, 0, 1, 0],
            "r": [1, 0, 0, 0, 1, 0, 0, 0, 1], "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}


def feed_state(processor, vehicle, yaw, t_us, **override):
    position, q = syn.px4_pose(vehicle, yaw)
    message = {"timestamp": t_us, "timestamp_sample": t_us, "x": position[0], "y": position[1], "z": position[2],
               "heading": math.pi / 2 - yaw, "xy_valid": True, "z_valid": True, "xy_reset_counter": 0,
               "z_reset_counter": 0, "heading_reset_counter": 0, "ref_timestamp": 5}
    message.update(override)
    processor.on_position(message)
    processor.on_attitude({"timestamp": t_us, "timestamp_sample": t_us, "q": q, "quat_reset_counter": 0})


def run_processor(frames, boxes, extra_state=None):
    processor = perception.P4Processor(SENSORS, CONFIG, "synthetic")
    assert processor.on_camera_info(info_fields())
    snapshots = []
    for n, (vehicle, yaw) in enumerate(frames):
        t = 1.0 + n / 30
        stamp = int(round(t * 1e9))
        for k in range(-3, 4):
            feed_state(processor, vehicle, yaw, int(round((t + k * 0.01) * 1e6)), **(extra_state or {}))
        image = syn.render(vehicle, yaw, syn.boxes_3d(boxes))
        fields = {"width": 640, "height": 480, "encoding": "32FC1", "is_bigendian": 0, "step": 2560,
                  "data": image.tobytes()}
        processor.on_image(fields, stamp, SENSORS["raw_frame"], 100 + t, t + 0.004)
        _, snapshot = processor.poll(t + 0.004, 100 + t + 0.004)
        if snapshot:
            snapshots.append(snapshot)
    return processor, snapshots


def test_processor_builds_a_grid_only_from_matched_frames():
    frames = [((0.0, 0.0, 2.0), 0.0)] * 7
    processor, snapshots = run_processor(frames, [{"center": [3.0, 0.0], "size": [0.4, 0.4]}])
    assert snapshots and all(s["usable"] for s in snapshots)
    grid = local_map.Grid.decode(snapshots[-1]["grid"])
    assert grid.state(2.85, 0.0) == local_map.OCCUPIED
    assert grid.state(1.5, 0.0) == local_map.FREE
    assert grid.state(-1.5, 0.0) == local_map.UNKNOWN  # behind the camera: never observed
    assert snapshots[-1]["ground_up_m"] == pytest.approx(2.0 - 0.24)


def test_invalid_estimate_frames_are_unusable_and_counted():
    frames = [((0.0, 0.0, 2.0), 0.0)] * 4
    processor, snapshots = run_processor(frames, [], extra_state={"xy_valid": False})
    assert snapshots and not any(s["usable"] for s in snapshots)
    assert snapshots[-1]["consecutive_unusable"] == len(snapshots)
    assert all(s["reason"].startswith("association:") for s in snapshots)
