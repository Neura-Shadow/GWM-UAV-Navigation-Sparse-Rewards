"""Strict configuration, wire-level activation and evidence contracts."""
import json
import math
from pathlib import Path

from .frames import finite, wrap

TOPICS = {
    "offboard_control_mode": ("in", "OffboardControlMode"),
    "trajectory_setpoint": ("in", "TrajectorySetpoint"),
    "vehicle_command": ("in", "VehicleCommand"),
    "vehicle_command_ack": ("out", "VehicleCommandAck"),
    "vehicle_status": ("out", "VehicleStatus"),
    "vehicle_local_position": ("out", "VehicleLocalPosition"),
    "vehicle_attitude": ("out", "VehicleAttitude"),
    "vehicle_land_detected": ("out", "VehicleLandDetected"),
    "estimator_status_flags": ("out", "EstimatorStatusFlags"),
    "failsafe_flags": ("out", "FailsafeFlags"),
}
GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH")
INITIALIZATION = "POSITION_INITIALIZATION_YAW_UNSPECIFIED"
NOMINAL = "POSITION_NOMINAL_YAW_TARGET"


def gates(env, flight=False):
    missing = [key for key in GATES + (("GWM_ALLOW_SITL_COMMANDS",) if flight else ())
               if env.get(key) != "1"]
    if missing:
        raise ValueError("Missing explicit gates: " + ", ".join(missing))


def load_config(path):
    config = json.loads(Path(path).read_text())
    profiles = {None: ("default", "x500_71"),
                "p3-depth-coexistence-v1": ("gwm_p3_flight", "x500_depth_71")}
    if config.get("simulation_profile") not in profiles:
        raise ValueError("Unsupported simulation profile")
    world, model = profiles[config.get("simulation_profile")]
    for key, expected in {"schema_version": 1, "instance": 71, "vehicle_system": 72,
                          "vehicle_component": 1, "source_system": 201, "source_component": 191,
                          "dds_domain": 71, "dds_key": 72, "dds_port": 8888,
                          "ros_prefix": "/px4_71", "world": world, "model": model,
                          "control_owner": "gwm_px4_control", "max_command_attempts": 1,
                          "repeat_count": 20}.items():
        if config.get(key) != expected or isinstance(config.get(key), bool):
            raise ValueError("Unsupported P2 contract: " + key)
    for key, value in config.items():
        if isinstance(value, bool):
            raise ValueError("Boolean is not a numeric setting: " + key)
        if isinstance(value, (int, float)):
            if not math.isfinite(value) or (key != "min_height_m" and value <= 0):
                raise ValueError("Invalid finite positive setting: " + key)
    if config["rate_hz"] < 10 or config["prestream_sim_s"] < 2:
        raise ValueError("Unsafe prestream/rate")
    if config["horizontal_tolerance_m"] >= min(config["east_m"], config["north_m"]):
        raise ValueError("Axis response must exceed tolerance")
    policy = config.get("reference_policy", "p2-estimator-reference-v1")
    if policy not in ("p2-estimator-reference-v1", "p2-estimator-reference-v2", "p2-estimator-reference-v3"):
        raise ValueError("Unknown estimator reference policy")
    if policy in ("p2-estimator-reference-v2", "p2-estimator-reference-v3"):
        expected = {"max_events": 1, "max_yaw_deg": 5.0, "stable_sim_s": 5.0,
                    "pair_sim_s": 1.5, "pair_wall_s": 2.0, "pair_skew_s": 0.1,
                    "yaw_consistency_rad": 1e-5, "tilt_component_tolerance": 1e-5, "post_event_sim_s": 0.1}
        if config.get("reference_settings") != expected:
            raise ValueError("Unregistered reference policy bounds")
    return config


def topic_name(prefix, base, direction, version):
    return f"{prefix}/fmu/{direction}/{base}" + (f"_v{version}" if version else "")


def position_setpoint(position, yaw, timestamp_us, mode=NOMINAL):
    position = finite(position, 3)
    if isinstance(timestamp_us, bool) or not isinstance(timestamp_us, int) or timestamp_us <= 0:
        raise ValueError("Invalid shared-simulation timestamp")
    if mode not in (INITIALIZATION, NOMINAL):
        raise ValueError("Unknown position yaw mode")
    if mode == INITIALIZATION and yaw is not None:
        raise ValueError("Initialization requires intentionally unspecified yaw")
    return {"timestamp": timestamp_us, "position": list(position),
            "velocity": [math.nan]*3, "acceleration": [math.nan]*3,
            "jerk": [math.nan]*3, "yaw": math.nan if mode == INITIALIZATION else wrap(yaw),
            "yawspeed": 0.0 if mode == INITIALIZATION else math.nan}


def offboard_mode(timestamp_us):
    return {"timestamp": timestamp_us, "position": True, "velocity": False,
            "acceleration": False, "attitude": False, "body_rate": False,
            "thrust_and_torque": False, "direct_actuator": False}


def encode_wire(fields, mode=NOMINAL):
    """Inactive wire NaNs become explicit nulls with a field activation mask."""
    if mode not in (INITIALIZATION, NOMINAL):
        raise ValueError("Unknown position yaw mode")
    finite(fields["position"], 3)
    inactive = {"velocity", "acceleration", "jerk", "yaw" if mode == INITIALIZATION else "yawspeed"}
    for key in inactive:
        values = fields[key] if isinstance(fields[key], list) else [fields[key]]
        if not all(isinstance(v, float) and math.isnan(v) for v in values):
            raise ValueError("Invalid intentionally inactive field:"+key)
    if mode == INITIALIZATION:
        if fields["yawspeed"] != 0.0:
            raise ValueError("Initialization yawspeed must be zero")
    else:
        finite([fields["yaw"]], 1)
    return {"fields": {k: ([None]*3 if isinstance(v, list) else None) if k in inactive else v
                       for k, v in fields.items()},
            "active": {k: k not in inactive for k in fields}}


def strict_json(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))


def json_message(value):
    """Preserve full received fields, flagging non-finite values explicitly."""
    nonfinite = []
    def convert(item, path):
        if hasattr(item, "tolist"):
            item = item.tolist()
        if isinstance(item, dict):
            return {str(k): convert(v, path + "." + str(k)) for k, v in item.items()}
        if isinstance(item, (list, tuple)) or hasattr(item, "typecode"):
            return [convert(v, path + f"[{i}]") for i, v in enumerate(item)]
        if isinstance(item, float) and not math.isfinite(item):
            nonfinite.append(path)
            return None
        return item
    return {"fields": convert(value, "$"), "nonfinite_fields": nonfinite}
