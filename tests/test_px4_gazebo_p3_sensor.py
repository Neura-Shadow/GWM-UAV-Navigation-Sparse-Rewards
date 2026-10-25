"""P3 sensor adapter: decode, calibration, geometry, association, recorder, pipeline.

Runtime-free and deterministic. Synthetic arrays exercise contracts; they are not
native sensor evidence. Expected geometry is computed independently here with
explicit trigonometry, and cross-checked against the accepted P2 frame module.
"""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zlib

import numpy as np
import pytest

SIM = Path(__file__).resolve().parents[1] / "simulation/px4_gazebo"
sys.path.insert(0, str(SIM / "ros2_ws/src/gwm_sensor_adapter"))
sys.path.insert(0, str(SIM / "ros2_ws/src/gwm_px4_control"))
from gwm_sensor_adapter import association, camera_info, contracts, depth, geometry, pipeline, recorder  # noqa: E402
from gwm_px4_control import frames as p2_frames  # noqa: E402

FX = 432.496042035043  # observed native CameraInfo value retained by the archived P3 contract


@pytest.fixture
def config():
    return contracts.load_config(SIM / "configs/p3_sensors.yaml")


def info_fields(fx=FX, **override):
    fields = {"width": 640, "height": 480, "distortion_model": "plumb_bob", "d": [0.0] * 5,
              "k": [fx, 0, 320, 0, fx, 240, 0, 0, 1], "p": [fx, 0, 320, 0, 0, fx, 240, 0, 0, 0, 1, 0],
              "r": [1, 0, 0, 0, 1, 0, 0, 0, 1], "binning_x": 0, "binning_y": 0,
              "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}
    fields.update(override)
    return fields


def image_fields(values, **override):
    values = np.asarray(values, dtype="<f4")
    fields = {"width": values.shape[1], "height": values.shape[0], "encoding": "32FC1", "is_bigendian": 0,
              "step": values.shape[1] * 4, "data": values.tobytes()}
    fields.update(override)
    return fields


CAL = {"fx": FX, "fy": FX, "cx": 320.0, "cy": 240.0}


# Contracts ---------------------------------------------------------------------------------------
def test_config_is_the_frozen_original_sensor_geometry(config):
    assert (config["width"], config["height"], config["sensor_hz"], config["processing_hz"]) == (640, 480, 30, 10)
    assert (config["near_m"], config["far_m"], config["horizontal_fov_rad"]) == (0.2, 19.1, 1.274)
    assert config["optical_origin_base_flu_m"] == [0.13233, 0.0, 0.02078]


@pytest.mark.parametrize("key,value", [("width", 320), ("encoding", "16UC1"), ("far_m", 25.0),
                                       ("recorder_compression", "lossy"), ("processing_hz", 60)])
def test_config_rejects_changed_workload(config, key, value, tmp_path):
    changed = dict(config, **{key: value})
    path = tmp_path / "c.json"
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError):
        contracts.load_config(path)


def test_inconsistent_extrinsic_chain_rejected(config, tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps(dict(config, optical_origin_base_flu_m=[0.13233, 0.0, 0.26078])))
    with pytest.raises(ValueError, match="inconsistent"):
        contracts.load_config(path)


@pytest.mark.parametrize("topic,type_name", [
    ("/px4_71/fmu/in/vehicle_command", "px4_msgs/msg/VehicleCommand"),
    ("/px4_71/fmu/in/trajectory_setpoint", "px4_msgs/msg/TrajectorySetpoint"),
    ("/px4_71/fmu/in/offboard_control_mode", "px4_msgs/msg/OffboardControlMode"),
    ("/gwm/sensors/front_depth/observation", "px4_msgs/msg/TrajectorySetpoint"),
    ("/fmu/in/obstacle_distance", "px4_msgs/msg/ObstacleDistance"),
    ("/gwm/anything", "std_msgs/msg/String")])
def test_sensor_adapter_cannot_publish_flight_input(config, topic, type_name):
    with pytest.raises(PermissionError):
        contracts.check_publication(config, topic, type_name)


def test_only_observation_and_health_publications(config):
    assert contracts.check_publication(config, config["observation_topic"], "std_msgs/msg/String")
    assert set(contracts.allowed_publications(config)) == {config["observation_topic"], config["health_topic"]}


# Depth decoding ----------------------------------------------------------------------------------
def test_decode_exact_native_layout(config):
    values = np.random.default_rng(1).uniform(0.3, 18, (480, 640)).astype("<f4")
    assert np.array_equal(depth.decode(image_fields(values), config), values)


def test_decode_padding_and_bigendian_only_outside_strict_runtime(config):
    values = np.full((480, 640), 2.5, dtype="<f4")
    padded = np.zeros((480, 648), dtype=">f4")
    padded[:, :640] = values
    fields = image_fields(values, step=648 * 4, is_bigendian=1, data=padded.tobytes())
    with pytest.raises(depth.FrameRejected, match="unregistered_native_layout"):
        depth.decode(fields, config)
    assert np.array_equal(depth.decode(fields, config, strict_layout=False), values)


@pytest.mark.parametrize("override,reason", [
    ({"encoding": "16UC1"}, "unsupported_encoding"), ({"width": 320}, "dimension_mismatch"),
    ({"step": 2559}, "invalid_step"), ({"data": b"\0" * 100}, "payload_length_mismatch"),
    ({"is_bigendian": 2}, "invalid_endianness"), ({"step": True}, "invalid_step")])
def test_decode_rejects_incompatible_messages(config, override, reason):
    with pytest.raises(depth.FrameRejected, match=reason):
        depth.decode(image_fields(np.ones((480, 640)), **override), config)


def test_classification_never_fills_unavailable_depth():
    values = np.array([[np.nan, -np.inf, 0.0, 0.1999, 0.2, 5.0, 19.0999, 19.1, np.inf]], dtype="<f4")
    codes = depth.classify(values, 0.2, 19.1)
    assert [depth.REASONS[c] for c in codes[0]] == [
        "invalid_nan", "too_close", "too_close", "too_close", "valid", "valid", "valid", "no_return", "no_return"]
    stats = depth.statistics(np.full((2, 2), np.inf, dtype="<f4"), np.full((2, 2), depth.NO_RETURN, np.uint8),
                             {"all": (0, 0, 2, 2)})
    assert stats["median_m"] is None and stats["regions"]["all"]["median_m"] is None
    assert stats["valid_fraction"] == 0 and stats["reason_counts"]["no_return"] == 4


def test_statistics_regions_are_independent():
    values = np.full((4, 4), 4.0, dtype="<f4")
    values[:, :2] = 2.0
    stats = depth.statistics(values, depth.classify(values, 0.2, 19.1), {"left": (0, 0, 2, 4), "right": (2, 0, 4, 4)})
    assert stats["regions"]["left"]["median_m"] == 2.0 and stats["regions"]["right"]["median_m"] == 4.0


# Calibration -------------------------------------------------------------------------------------
def test_native_calibration_matches_pinned_fov(config):
    focal, bound = camera_info.expected_focal_px(config)
    assert abs(focal - FX) < 1e-6 and 0.1 < bound < 0.5
    result = camera_info.validate(info_fields(), config)
    assert result["fx"] == FX and len(result["identity"]) == 64


@pytest.mark.parametrize("override,reason", [
    ({"d": [0.1, 0, 0, 0, 0]}, "distortion"), ({"k": [FX, 0, 320, 0, FX, 240, 0, 0, 2]}, "intrinsic"),
    ({"p": [FX, 0, 320, 0.5, 0, FX, 240, 0, 0, 0, 1, 0]}, "projection"),
    ({"r": [0, 1, 0, 1, 0, 0, 0, 0, 1]}, "rectification"), ({"width": 1920}, "dimension")])
def test_calibration_rejections(config, override, reason):
    with pytest.raises(camera_info.CalibrationRejected, match=reason):
        camera_info.validate(info_fields(**override), config)


def test_focal_length_outside_declared_precision_rejected(config):
    with pytest.raises(camera_info.CalibrationRejected, match="focal"):
        camera_info.validate(info_fields(fx=FX + 1.0), config)


def test_calibration_change_invalidates_run(config):
    cache = camera_info.CalibrationCache(config)
    assert cache.update(info_fields()) and cache.usable
    assert not cache.update(info_fields(fx=FX + 0.1))
    assert not cache.usable and cache.invalid_reason == "calibration_identity_changed"


# Geometry ----------------------------------------------------------------------------------------
EXT = geometry.Extrinsic([0.13233, 0.0, 0.02078], identity="test")


def test_forward_axis_and_mount_offset():
    centre = geometry.backproject(320, 240, 4.0, CAL)
    assert np.allclose(EXT.optical_to_flu(centre), [4.13233, 0, 0.02078])
    assert np.allclose(EXT.optical_to_flu([0, 0, 0]), [0.13233, 0, 0.02078])
    assert np.allclose(EXT.optical_to_frd(centre), [4.13233, 0, -0.02078])


def test_image_left_right_and_up_down():
    left = EXT.optical_to_flu(geometry.backproject(100, 240, 2.0, CAL))
    right = EXT.optical_to_flu(geometry.backproject(540, 240, 2.0, CAL))
    up = EXT.optical_to_flu(geometry.backproject(320, 40, 2.0, CAL))
    down = EXT.optical_to_flu(geometry.backproject(320, 440, 2.0, CAL))
    assert left[1] > 0 > right[1]  # image left is body +Y (left)
    assert up[2] > 0.02078 > down[2]  # image up is body +Z (up)
    assert math.isclose(left[1], 2.0 * 220 / FX) and math.isclose(up[2] - 0.02078, 2.0 * 200 / FX)


def test_optical_depth_is_not_euclidean_range():
    u, v, z = 600, 50, 3.0
    expected = z * math.sqrt(1 + ((u - 320) / FX) ** 2 + ((v - 240) / FX) ** 2)
    assert math.isclose(geometry.ray_length(u, v, z, CAL), expected)
    assert geometry.ray_length(u, v, z, CAL) > z
    assert math.isclose(geometry.ray_length(320, 240, z, CAL), z)


def yaw_quaternion(psi):
    return [math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2)]


@pytest.mark.parametrize("psi", [0.0, math.pi / 2, -math.pi / 2, math.pi, 1.674781709, -2.3])
def test_vehicle_yaw_translation_and_nonzero_initial_heading(psi):
    position = [5.0, -3.0, -2.0]
    point_frd = [4.0, 1.0, -0.5]
    world = geometry.frd_to_local_ned(point_frd, position, yaw_quaternion(psi))
    expected = [position[0] + math.cos(psi) * 4.0 - math.sin(psi) * 1.0,
                position[1] + math.sin(psi) * 4.0 + math.cos(psi) * 1.0, position[2] - 0.5]
    assert np.allclose(world, expected, atol=1e-12)
    rotated = p2_frames.body_frd_to_ned(point_frd, yaw_quaternion(psi))  # accepted P2 implementation
    assert np.allclose(world - np.array(position), rotated, atol=1e-12)


def test_heading_east_camera_forward_is_ned_east_and_enu_x():
    world = geometry.frd_to_local_ned(EXT.optical_to_frd([0, 0, 3.0]), [0, 0, -2.0], yaw_quaternion(math.pi / 2))
    assert np.allclose(world, [0, 3.13233, -2.02078], atol=1e-12)
    assert np.allclose(geometry.ned_to_enu(world), [3.13233, 0, 2.02078], atol=1e-12)
    assert np.allclose(EXT.optical_to_frd([1, 1, 1]), p2_frames.flu_to_frd(tuple(EXT.optical_to_flu([1, 1, 1]))))


def test_pitched_vehicle_uses_full_rotation():
    pitch = 0.3  # nose up: body forward gains NED -Z (up)
    q = [math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]
    world = geometry.frd_to_local_ned([1, 0, 0], [0, 0, 0], q)
    assert np.allclose(world, [math.cos(pitch), 0, -math.sin(pitch)])


def sdf_loader():
    models = {
        "x500_depth": """<model name="x500_depth"><include merge="true"><uri>x500</uri></include>
            <include merge="true"><uri>model://OakD-Lite</uri><pose>.12 .03 .242 0 0 0</pose></include></model>""",
        "x500": """<model name="x500"><include merge="true"><uri>model://x500_base</uri></include></model>""",
        "x500_base": """<model name="x500_base"><link name="base_link"><pose>0 0 .24 0 0 0</pose></link></model>""",
        "OakD-Lite": """<model name="OakD-Lite"><link name="camera_link">
            <sensor name="IMX214" type="camera"><pose>0.01233 -0.03 0.01878 0 0 0</pose></sensor>
            <sensor name="StereoOV7251" type="depth_camera"><pose>0.01233 -0.03 0.01878 0 0 0</pose></sensor>
            </link></model>"""}
    return lambda uri: ET.fromstring(models[uri.replace("model://", "")])


def test_sdf_chain_rederives_optical_offset():
    chain = geometry.sdf_chain(sdf_loader(), "x500_depth", "base_link", "StereoOV7251")
    assert np.allclose(chain["optical_origin_base_flu_m"], [0.13233, 0, 0.02078])
    assert np.allclose(chain["sensor_origin_model_m"], [0.13233, 0, 0.26078])
    assert chain["all_rotations_identity"] and chain["sensor_link"] == "camera_link"


def test_sdf_chain_rejects_unknown_relative_frame():
    def load(uri):
        return ET.fromstring('<model name="m"><link name="base_link"><pose relative_to="other">0 0 0 0 0 0</pose>'
                             '<sensor name="s"/></link></model>')
    with pytest.raises(ValueError, match="frame"):
        geometry.sdf_chain(load, "m", "base_link", "s")


# Association -------------------------------------------------------------------------------------
def position_message(t, heading_reset=0, **override):
    message = {"timestamp": t, "timestamp_sample": t, "x": 1.0, "y": 2.0, "z": -3.0, "heading": 0.1,
               "xy_valid": True, "z_valid": True, "xy_reset_counter": 0, "z_reset_counter": 0,
               "heading_reset_counter": heading_reset, "ref_timestamp": 5}
    message.update(override)
    return message


def associator_with(stamps, heading_reset=lambda t: 0):
    a = association.Associator(300, 0.05)
    for t in stamps:
        a.add_position(position_message(t, heading_reset(t)))
        a.add_attitude({"timestamp": t, "timestamp_sample": t, "q": yaw_quaternion(0.1), "quat_reset_counter": 0})
    return a


def test_association_nearest_within_tolerance_without_interpolation():
    a = associator_with([1_000_000, 1_020_000, 1_040_000])
    result = a.associate(1_029_000)
    assert result["status"] == "matched" and result["position"]["timestamp_us"] == 1_020_000
    assert result["position_ned"] == [1.0, 2.0, -3.0]


def test_association_reports_missing_ambiguous_and_pending():
    assert association.Associator(300, 0.05).associate(1_000_000)["status"] == "position:missing_history"
    a = associator_with([1_000_000, 1_020_000])
    assert a.associate(1_010_000)["status"] == "position:ambiguous_equidistant"
    assert a.associate(1_200_000)["status"] == "position:pending_newer_state"
    assert a.associate(100_000)["status"] == "position:older_than_history"


def test_association_rejects_reset_epoch_bracket():
    a = associator_with([1_000_000, 1_020_000], heading_reset=lambda t: 0 if t < 1_010_000 else 1)
    assert a.associate(1_012_000)["status"] == "position:reset_epoch_bracket"


def test_association_duplicates_and_reuse_reported():
    a = associator_with([1_000_000, 1_000_000, 1_020_000])
    assert a.position.counts["duplicate"] == 1
    first, second = a.associate(1_019_000), a.associate(1_021_000)
    assert first["reused"] is False and second["reused"] is True


def test_invalid_estimate_never_counts_as_matched():
    a = association.Associator(300, 0.05)
    a.add_position(position_message(1_000_000, xy_valid=False))
    a.add_attitude({"timestamp": 1_000_000, "timestamp_sample": 1_000_000, "q": [1, 0, 0, 0], "quat_reset_counter": 0})
    assert a.associate(1_000_000)["status"] == "estimate_invalid"


def test_history_is_bounded():
    a = association.StateHistory("p", 3, 0.05)
    for t in range(1, 6):
        a.add(t * 1000, {})
    assert len(a.samples) == 3 and a.counts["evicted"] == 2


# Recorder ----------------------------------------------------------------------------------------
def test_recorder_complete_lossless_hashed(tmp_path):
    rec = recorder.RawRecorder(tmp_path, 8)
    payloads = [np.full((480, 640), i, dtype="<f4").tobytes() for i in range(5)]
    for i, payload in enumerate(payloads):
        assert rec.submit({"receipt_id": i + 1}, payload)
    closed = rec.close(10)
    assert closed["complete"] and closed["written"] == 5 and closed["overflow"] == 0
    rows = [json.loads(line) for line in (tmp_path / recorder.INDEX).read_text().splitlines()]
    blob = (tmp_path / recorder.DATA).read_bytes()
    for row, payload in zip(rows, payloads):
        assert zlib.decompress(blob[row["offset"]:row["offset"] + row["stored_bytes"]]) == payload
    assert json.loads((tmp_path / recorder.CLOSED).read_text())["complete"]


def test_recorder_overflow_is_sticky_incomplete(tmp_path):
    import threading
    gate = threading.Event()
    rec = recorder.RawRecorder(tmp_path, 1, clock=lambda: (gate.wait(5), 0.0)[1])
    results = [rec.submit({"receipt_id": i}, b"x" * 10) for i in range(6)]
    assert not all(results) and rec.overflow >= 1
    gate.set()
    closed = rec.close(10)
    assert not closed["complete"] and closed["overflow"] >= 1


# Pipeline ----------------------------------------------------------------------------------------
def frame(value, shape=(480, 640)):
    return image_fields(np.full(shape, value, dtype="<f4"))


def test_health_unavailable_fresh_stale_and_no_republication(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    assert proc.poll(1.0, 10.0) == (False, None) and proc.status == "unavailable"
    proc.on_camera_info(info_fields())
    accepted, record = proc.on_image(frame(4.0), 1_000_000_000, "camera_link", 10.0, 1.01)
    assert accepted and record["accepted"]
    changed, observation = proc.poll(1.02, 10.02)
    assert changed and proc.status == "fresh" and observation is None  # fixed 10 Hz phase grid: not yet due
    observation = proc.poll(1.1, 10.1)[1]
    assert observation["receipt_id"] == 1 and observation["statistics"]["median_m"] == 4.0
    assert math.isclose(observation["observation_age_sim_s"], 0.1)
    # The same frame is never published twice, even when processing is due again.
    assert proc.poll(1.2, 10.2)[1] is None
    proc.poll(1.3, 11.2)
    assert proc.status == "stale" and proc.transitions[-1]["reason"] in ("receipt_stall_wall", "source_age_sim")
    assert proc.poll(1.36, 11.3)[1] is None


def test_stale_by_receipt_stall_even_if_clock_stops(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    proc.on_camera_info(info_fields())
    proc.on_image(frame(2.0), 1_000_000_000, "camera_link", 10.0, 1.0)
    proc.poll(1.0, 10.0)
    proc.poll(1.0, 11.01)
    assert proc.status == "stale" and proc.status_reason == "receipt_stall_wall"


def test_calibration_required_before_fresh(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    proc.on_image(frame(2.0), 1_000_000_000, "camera_link", 10.0, 1.0)
    assert proc.poll(1.0, 10.0)[1] is None and proc.status == "calibration_unavailable"


def test_newest_frame_processing_at_10hz_counts_skips(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    proc.on_camera_info(info_fields())
    published = 0
    for n in range(90):  # three simulation seconds at 30 Hz
        t = 1.0 + n / 30
        proc.on_image(frame(3.0), int(round(t * 1e9)), "camera_link", 100 + t, t + 0.002)
        published += proc.poll(t + 0.004, 100 + t + 0.004)[1] is not None
    a = proc.accounting.as_dict()
    assert 29 <= published <= 31 and a["decoded"] == 90 and a["processed"] == published
    assert a["skipped_by_processing"] == 90 - published and a["overwritten"] >= 55


def test_duplicate_backwards_and_wrong_frame_rejected(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    assert proc.on_image(frame(2.0), 2_000_000_000, "camera_link", 1, 2)[0]
    assert proc.on_image(frame(2.0), 2_000_000_000, "camera_link", 1, 2)[1]["rejection"] == "duplicate_acquisition_stamp"
    assert proc.on_image(frame(2.0), 1_000_000_000, "camera_link", 1, 2)[1]["rejection"] == "acquisition_stamp_backwards"
    assert proc.on_image(frame(2.0), 3_000_000_000, "base_link", 1, 2)[1]["rejection"] == "unexpected_raw_frame_id"
    assert proc.accounting.rejected == {"duplicate_acquisition_stamp": 1, "acquisition_stamp_backwards": 1,
                                        "unexpected_raw_frame_id": 1}


def test_observation_samples_keep_unknown_as_null_and_transform_valid(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=False)
    proc.on_camera_info(info_fields())
    values = np.full((480, 640), np.inf, dtype="<f4")
    values[:, :320] = 3.0
    proc.on_image(image_fields(values), 1_000_000_000, "camera_link", 1.0, 1.0)
    observation = proc.poll(1.0, 1.0)[1]
    for sample in observation["samples"]:
        if sample["u"] < 320:
            assert sample["reason"] == "valid" and sample["depth_m"] == 3.0
            assert math.isclose(sample["body_flu_m"][0], 3.13233)
            assert math.isclose(sample["body_flu_m"][1], -(sample["u"] - 320) * 3.0 / FX)
        else:
            assert sample["reason"] == "no_return"
            assert sample["depth_m"] is None and sample["body_flu_m"] is None and sample["local_ned_m"] is None
    assert len(observation["samples"]) == 35
    json.dumps(observation, allow_nan=False)


def test_flight_observation_associates_contemporaneous_state(config):
    proc = pipeline.DepthProcessor(config, "run", require_state=True)
    proc.on_camera_info(info_fields())
    for t in (999_000, 1_019_000):
        proc.on_position(position_message(t))
        proc.on_attitude({"timestamp": t, "timestamp_sample": t, "q": yaw_quaternion(math.pi / 2),
                          "quat_reset_counter": 0})
    proc.on_image(frame(3.0), 1_000_000_000, "camera_link", 1.0, 1.0)
    observation = proc.poll(1.0, 1.0)[1]
    assert observation["association"]["status"] == "matched"
    centre = next(s for s in observation["samples"] if s["u"] == 320 and s["v"] == 240)
    # Facing NED east from [1, 2, -3]: the optical-axis point is 3.13233 m east of the vehicle.
    assert np.allclose(centre["local_ned_m"], [1 + 0.0, 2 + 3.13233, -3 - 0.02078], atol=1e-9)
