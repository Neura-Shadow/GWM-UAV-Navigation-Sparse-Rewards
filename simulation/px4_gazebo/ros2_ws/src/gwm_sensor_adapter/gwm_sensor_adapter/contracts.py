"""Strict P3 sensor configuration and the sensing-only publication boundary.

Pure module: imports no ROS runtime. The adapter is a sensing component. It may
publish only its observation and health strings; every PX4 input topic and
every flight-control message type is rejected before a publisher is created.
"""
import json
import math
from pathlib import Path

CONTRACT = "p3-depth-observation-v1"
OBSERVATION_SCHEMA = 1

# Any of these, under any topic name, would give the sensor path flight authority.
FORBIDDEN_TYPES = frozenset("px4_msgs/msg/" + name for name in (
    "VehicleCommand", "TrajectorySetpoint", "OffboardControlMode", "VehicleAttitudeSetpoint",
    "VehicleRatesSetpoint", "VehicleThrustSetpoint", "VehicleTorqueSetpoint", "ActuatorMotors",
    "ActuatorServos", "ManualControlSetpoint", "GotoSetpoint", "VehicleLocalPositionSetpoint",
    "ObstacleDistance", "VehicleVisualOdometry", "VehicleOdometry", "VehicleMocapOdometry",
    "SensorOpticalFlow", "DistanceSensor"))

EXPECTED = {"schema_version": 1, "contract": CONTRACT, "width": 640, "height": 480, "sensor_hz": 30,
            "encoding": "32FC1", "model": "x500_depth_71", "airframe": "gz_x500_depth",
            "raw_frame": "camera_link", "optical_frame": "gwm_front_depth_optical",
            "near_m": 0.2, "far_m": 19.1, "horizontal_fov_rad": 1.274,
            "image_topic": "/gwm/sensors/front_depth/image_raw",
            "info_topic": "/gwm/sensors/front_depth/camera_info",
            "observation_topic": "/gwm/sensors/front_depth/observation",
            "health_topic": "/gwm/sensors/front_depth/health"}

POSITIVE = ("processing_hz", "source_age_sim_s", "stall_wall_s", "poll_wall_s", "state_mismatch_s",
            "recorder_close_wall_s", "horizontal_fov_declared_resolution_rad")
COUNTS = ("sensor_queue_depth", "state_queue_depth", "recorder_capacity", "state_capacity")


def load_config(path):
    config = json.loads(Path(path).read_text())
    for key, expected in EXPECTED.items():
        if config.get(key) != expected or isinstance(config.get(key), bool):
            raise ValueError("Unsupported P3 sensor contract: " + key)
    for key in POSITIVE:
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError("Invalid positive setting: " + key)
    for key in COUNTS:
        value = config.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 1000:
            raise ValueError("Invalid bounded count: " + key)
    if config["processing_hz"] > config["sensor_hz"]:
        raise ValueError("Processing cannot exceed native acquisition rate")
    if config["recorder_compression"] not in ("none", "zlib-1"):
        raise ValueError("Unsupported lossless recorder compression")
    for key in ("base_origin_model_m", "optical_origin_model_m", "optical_origin_base_flu_m"):
        vector = config.get(key)
        if not isinstance(vector, list) or len(vector) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in vector):
            raise ValueError("Invalid extrinsic vector: " + key)
    derived = [a - b for a, b in zip(config["optical_origin_model_m"], config["base_origin_model_m"])]
    if any(abs(a - b) > 1e-9 for a, b in zip(derived, config["optical_origin_base_flu_m"])):
        raise ValueError("Optical-to-base translation is inconsistent with the model chain")
    for name, roi in config["report_regions_uv"].items():
        check_roi(roi, config["width"], config["height"], name)
    grid = config["sample_grid"]
    if not all(isinstance(grid.get(k), int) and 1 <= grid[k] <= 16 for k in ("columns", "rows")):
        raise ValueError("Invalid sample grid")
    return config


def check_roi(roi, width, height, name="roi"):
    if (not isinstance(roi, list) or len(roi) != 4 or any(isinstance(v, bool) or not isinstance(v, int) for v in roi)
            or not (0 <= roi[0] < roi[2] <= width and 0 <= roi[1] < roi[3] <= height)):
        raise ValueError("Invalid half-open pixel region: " + name)
    return tuple(roi)


def allowed_publications(config):
    return {config["observation_topic"]: "std_msgs/msg/String", config["health_topic"]: "std_msgs/msg/String"}


def check_publication(config, topic, type_name):
    """Deny-by-default: exact observation/health topics only, never a PX4 input."""
    if type_name in FORBIDDEN_TYPES or "/fmu/in/" in topic or topic.startswith("/fmu/"):
        raise PermissionError("Sensor adapter must not publish flight-control input: " + topic)
    if allowed_publications(config).get(topic) != type_name:
        raise PermissionError("Unregistered sensor publication: " + topic + " " + type_name)
    return True


def strict_json(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))
