"""Phase 6-E Isaac Sim / PX4 SITL bridge design readiness.

This module is deliberately a design and dry-run validation layer. It defines
the state ownership, data paths, frame policy, timing policy, and refusal rules
for a future Isaac Sim / Isaac Lab <-> PX4 SITL closed loop without launching
Isaac, starting ROS2 nodes, connecting MAVSDK, launching PX4, or touching
hardware.
"""

from __future__ import annotations

import copy
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping

SCHEMA_VERSION = "gwm_phase6_isaac_px4_bridge_design_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/isaac_px4_bridge_design.json"

ISAAC_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_ISAAC_RUNTIME_TESTS",
)
ROS2_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
)
MAVSDK_PX4_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_MAVSDK_SITL_TESTS",
    "GWM_ALLOW_SITL_COMMANDS",
)
OPTIONAL_ENV_GATES = (
    "GWM_ROS2_LIVE_TOPICS",
    "GWM_ALLOW_PX4_LAUNCH",
)


def _pure_sim_deployment() -> dict[str, bool]:
    return {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }


def _default_prior_reports() -> dict[str, str]:
    return {
        "isaac_sensor_runtime": "outputs/runtime_validation/isaac_sensor_runtime.json",
        "ros2_sim_sensor_bridge": "outputs/runtime_validation/ros2_sim_sensor_bridge.json",
        "px4_sitl_command_validation": (
            "outputs/runtime_validation/px4_sitl_command_validation.json"
        ),
    }


@dataclass
class FrameTransformPolicy:
    """Explicit frame policy required before future Isaac/PX4 coupling."""

    project_frame: str = "project_default"
    isaac_world_frame: str = "isaac_z_up"
    px4_world_frame: str = "px4_ned"
    mavsdk_command_frame: str = "px4_body_ned"
    ros2_frames_preserved_from_headers: bool = True
    coordinate_conversion_applied: bool = False
    transforms_defined: bool = False
    blocks_silent_conversion: bool = True
    stale_transform_rejection: bool = True
    max_transform_age_sec: float = 0.1
    required_before_phase6f_live_loop: bool = True


@dataclass
class IsaacPX4BridgeDesignConfig:
    """Configuration for Phase 6-E bridge-design readiness."""

    output_path: str | None = None
    write_output: bool = True
    require_prior_reports: bool = False
    fail_on_not_ready: bool = False
    use_ros2_sensor_path: bool = True
    bridge_strategy: str = "mavsdk_lightweight"
    future_coupled_execution_requested: bool = False
    px4_launch_requested: bool = False
    isaac_step_sec: float = 0.05
    safety_fast_loop_hz: float = 50.0
    gwm_planner_hz: float = 2.0
    mavsdk_command_hz: float = 10.0
    stale_observation_timeout_sec: float = 0.25
    stale_command_timeout_sec: float = 0.2
    deployment: dict[str, Any] = field(default_factory=_pure_sim_deployment)
    prior_reports: dict[str, str] = field(default_factory=_default_prior_reports)
    frame_transform_policy: FrameTransformPolicy = field(default_factory=FrameTransformPolicy)


@dataclass
class IsaacPX4BridgeDesignResult:
    """JSON-safe Phase 6-E bridge-design result."""

    schema_version: str
    status: str
    reason: str | None
    runtime_gates: dict
    required_reports: dict
    report_readiness: dict
    bridge_strategy: dict
    state_ownership: dict
    data_paths: dict
    command_paths: dict
    coordinate_frames: dict
    frame_transform_policy: dict
    timing_policy: dict
    safety_policy: dict
    failure_handling: dict
    refusal_rules: dict
    artifacts: dict
    timings: dict
    errors: list


def build_isaac_px4_bridge_plan(
    config: dict | IsaacPX4BridgeDesignConfig | None = None,
) -> dict:
    """Build the Phase 6-E bridge plan without invoking runtime processes."""
    design_config = _normalize_config(config)
    frame_policy = asdict(design_config.frame_transform_policy)
    return {
        "schema_version": SCHEMA_VERSION,
        "bridge_strategy": _bridge_strategy(design_config),
        "state_ownership": _state_ownership(),
        "data_paths": _data_paths(design_config),
        "command_paths": _command_paths(design_config),
        "coordinate_frames": _coordinate_frames(frame_policy),
        "frame_transform_policy": _frame_transform_policy_summary(
            frame_policy,
            design_config,
        ),
        "timing_policy": _timing_policy(design_config),
        "safety_policy": _safety_policy(design_config),
        "failure_handling": _failure_handling(design_config),
        "refusal_rules": _refusal_rules(),
        "artifacts": _artifact_policy(design_config),
    }


def run_isaac_px4_bridge_design(
    config: dict | IsaacPX4BridgeDesignConfig | None = None,
) -> dict:
    """Run the dry Phase 6-E bridge-design readiness check."""
    design_config = _normalize_config(config)
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []

    plan = build_isaac_px4_bridge_plan(design_config)
    report_readiness = _report_readiness(design_config)
    required_reports = _required_report_paths(design_config)
    result = IsaacPX4BridgeDesignResult(
        schema_version=SCHEMA_VERSION,
        status="ready",
        reason=None,
        runtime_gates=_runtime_gates(),
        required_reports=required_reports,
        report_readiness=report_readiness,
        bridge_strategy=plan["bridge_strategy"],
        state_ownership=plan["state_ownership"],
        data_paths=plan["data_paths"],
        command_paths=plan["command_paths"],
        coordinate_frames=plan["coordinate_frames"],
        frame_transform_policy=plan["frame_transform_policy"],
        timing_policy=plan["timing_policy"],
        safety_policy=plan["safety_policy"],
        failure_handling=plan["failure_handling"],
        refusal_rules=plan["refusal_rules"],
        artifacts=plan["artifacts"],
        timings=timings,
        errors=errors,
    )

    refusal = _configuration_refusal(design_config)
    if refusal is not None:
        result.status = "failed"
        result.reason = refusal
        result.errors.append({"type": "RuntimeError", "message": refusal})
        return _finalize_result(result, design_config, start)

    if design_config.require_prior_reports and not report_readiness["all_ready"]:
        missing = [
            name
            for name, report in report_readiness["reports"].items()
            if not bool(report.get("ready", False))
        ]
        result.status = "not_ready"
        result.reason = f"Missing or unready prior Phase 6 reports: {', '.join(missing)}"
    elif (
        design_config.future_coupled_execution_requested
        and not design_config.frame_transform_policy.transforms_defined
    ):
        result.status = "not_ready"
        result.reason = "Future coupled execution requires explicit Isaac/PX4 frame transforms."

    return _finalize_result(result, design_config, start)


def _normalize_config(
    config: dict | IsaacPX4BridgeDesignConfig | None,
) -> IsaacPX4BridgeDesignConfig:
    if isinstance(config, IsaacPX4BridgeDesignConfig):
        return copy.deepcopy(config)

    source = _bridge_config_section(config or {})
    deployment = dict(source.get("deployment") or _pure_sim_deployment())
    frame_policy = _frame_policy_from_config(source.get("frame_transform_policy"))
    return IsaacPX4BridgeDesignConfig(
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        require_prior_reports=bool(source.get("require_prior_reports", False)),
        fail_on_not_ready=bool(source.get("fail_on_not_ready", False)),
        use_ros2_sensor_path=bool(source.get("use_ros2_sensor_path", True)),
        bridge_strategy=str(source.get("bridge_strategy", "mavsdk_lightweight")),
        future_coupled_execution_requested=bool(
            source.get("future_coupled_execution_requested", False)
        ),
        px4_launch_requested=bool(source.get("px4_launch_requested", False)),
        isaac_step_sec=float(source.get("isaac_step_sec", 0.05)),
        safety_fast_loop_hz=float(source.get("safety_fast_loop_hz", 50.0)),
        gwm_planner_hz=float(source.get("gwm_planner_hz", 2.0)),
        mavsdk_command_hz=float(source.get("mavsdk_command_hz", 10.0)),
        stale_observation_timeout_sec=float(
            source.get("stale_observation_timeout_sec", 0.25)
        ),
        stale_command_timeout_sec=float(source.get("stale_command_timeout_sec", 0.2)),
        deployment=deployment,
        prior_reports=dict(source.get("prior_reports") or _default_prior_reports()),
        frame_transform_policy=frame_policy,
    )


def _bridge_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("isaac_px4_bridge_design") or {})
    if "isaac_px4_bridge_design" in config:
        return dict(config.get("isaac_px4_bridge_design") or {})
    return dict(config)


def _frame_policy_from_config(value: Any) -> FrameTransformPolicy:
    if isinstance(value, FrameTransformPolicy):
        return copy.deepcopy(value)
    if not isinstance(value, Mapping):
        return FrameTransformPolicy()
    defaults = asdict(FrameTransformPolicy())
    defaults.update(dict(value))
    return FrameTransformPolicy(**defaults)


def _configuration_refusal(config: IsaacPX4BridgeDesignConfig) -> str | None:
    deployment = config.deployment
    if bool(deployment.get("real_hardware_enabled", False)):
        return "Phase 6-E refuses real_hardware_enabled=True."
    if bool(deployment.get("autonomous_real_flight_enabled", False)):
        return "Phase 6-E refuses autonomous_real_flight_enabled=True."
    if bool(config.px4_launch_requested):
        return "Phase 6-E refuses PX4 launch requests; PX4 SITL must be started externally."
    if bool(deployment.get("mock", False)):
        return "Phase 6-E live bridge design requires deployment.mock=False for SITL mode."
    if not bool(deployment.get("sitl_enabled", False)):
        return "Phase 6-E live bridge design requires deployment.sitl_enabled=True."
    return None


def _runtime_gates() -> dict:
    return {
        "isaac": _env_group(ISAAC_ENV_GATES),
        "ros2": _env_group(ROS2_ENV_GATES),
        "mavsdk_px4_sitl": _env_group(MAVSDK_PX4_ENV_GATES),
        "optional": _env_group(OPTIONAL_ENV_GATES, required=False),
    }


def _env_group(names: tuple[str, ...], required: bool = True) -> dict:
    return {
        name: {
            "present": name in os.environ,
            "enabled": os.environ.get(name) == "1",
            "required": required,
        }
        for name in names
    }


def _required_report_paths(config: IsaacPX4BridgeDesignConfig) -> dict[str, str]:
    names = ["isaac_sensor_runtime", "px4_sitl_command_validation"]
    if config.use_ros2_sensor_path:
        names.insert(1, "ros2_sim_sensor_bridge")
    return {name: config.prior_reports[name] for name in names if name in config.prior_reports}


def _report_readiness(config: IsaacPX4BridgeDesignConfig) -> dict:
    reports: dict[str, dict[str, Any]] = {}
    for name, path_text in _required_report_paths(config).items():
        path = Path(path_text)
        entry: dict[str, Any] = {
            "path": path_text,
            "exists": path.exists(),
            "status": None,
            "ready": False,
            "reason": None,
        }
        if path.exists():
            try:
                payload = json.loads(path.read_text(encoding="utf-8"))
                status = payload.get("status")
                entry["status"] = status
                entry["ready"] = status in {"passed", "ready"}
                if not entry["ready"]:
                    entry["reason"] = f"report status is {status!r}"
            except Exception as exc:
                entry["reason"] = f"could not read report: {exc}"
        else:
            entry["reason"] = "report missing"
        reports[name] = entry
    return {
        "required": bool(config.require_prior_reports),
        "all_ready": all(bool(report["ready"]) for report in reports.values()),
        "reports": reports,
    }


def _bridge_strategy(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "primary": config.bridge_strategy,
        "primary_description": (
            "MAVSDK-only lightweight command and telemetry path to externally "
            "running PX4 SITL."
        ),
        "future_option": "ros2_micro_xrce_dds",
        "future_option_status": "documented_only_not_implemented_in_phase6e",
        "px4_launch_attempted": False,
        "external_px4_sitl_required": True,
        "full_closed_loop_execution": False,
    }


def _state_ownership() -> dict:
    return {
        "isaac": [
            "simulated_world",
            "scene_step",
            "virtual_rgb_depth_lidar_imu",
            "visual_and_physics_context",
        ],
        "px4_sitl": [
            "autopilot_state",
            "offboard_mode_state",
            "arming_state",
            "command_acceptance",
        ],
        "mavsdk": ["sitl_only_command_transport", "px4_telemetry_view"],
        "gwm_wam": ["future_rollout", "candidate_scoring", "action_selection"],
        "cbf": ["mandatory_command_saturation", "safety_override_before_bridge_write"],
    }


def _data_paths(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "direct_isaac_sensor_path": [
            "IsaacSimRuntime.read_sensors",
            "IsaacSimRuntime.to_sensor_observation",
            "ObservationBuffer",
        ],
        "ros2_sensor_path_enabled": bool(config.use_ros2_sensor_path),
        "ros2_sensor_path": [
            "Isaac simulation sensor publishers",
            "ROS2SensorSynchronizer",
            "SensorObservation",
            "ObservationBuffer",
        ],
        "telemetry_path": [
            "PX4 SITL telemetry",
            "MAVSDK telemetry",
            "bridge health state",
        ],
        "no_runtime_invoked_by_design_runner": True,
    }


def _command_paths(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "planned_command_path": [
            "GWM/WAM selected trajectory",
            "ControlCommand",
            "ControlBarrierFunction",
            "MAVLinkBridge",
            "MAVSDK",
            "PX4 SITL offboard control",
        ],
        "command_frame": config.frame_transform_policy.mavsdk_command_frame,
        "cbf_required_before_mavsdk_write": True,
        "initial_zero_setpoint_required": True,
        "offboard_start_requires_sitl_ready": True,
        "px4_launch_attempted": False,
    }


def _coordinate_frames(frame_policy: Mapping[str, Any]) -> dict:
    return {
        "project_frame": frame_policy["project_frame"],
        "isaac_world_frame": frame_policy["isaac_world_frame"],
        "px4_world_frame": frame_policy["px4_world_frame"],
        "mavsdk_command_frame": frame_policy["mavsdk_command_frame"],
        "ros2_frames_preserved_from_headers": bool(
            frame_policy["ros2_frames_preserved_from_headers"]
        ),
        "coordinate_conversion_applied": bool(frame_policy["coordinate_conversion_applied"]),
    }


def _frame_transform_policy_summary(
    frame_policy: Mapping[str, Any],
    config: IsaacPX4BridgeDesignConfig,
) -> dict:
    return {
        **dict(frame_policy),
        "required_transforms": [
            "project_default_to_isaac_z_up",
            "isaac_z_up_to_px4_ned",
            "project_velocity_to_px4_body_ned",
        ],
        "coupled_loop_execution_allowed": bool(
            config.future_coupled_execution_requested and frame_policy["transforms_defined"]
        ),
        "silent_conversion_allowed": False,
    }


def _timing_policy(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "isaac_step_sec": float(config.isaac_step_sec),
        "isaac_step_hz": 1.0 / float(config.isaac_step_sec),
        "safety_fast_loop_hz": float(config.safety_fast_loop_hz),
        "gwm_planner_hz": float(config.gwm_planner_hz),
        "mavsdk_command_hz": float(config.mavsdk_command_hz),
        "stale_observation_timeout_sec": float(config.stale_observation_timeout_sec),
        "stale_command_timeout_sec": float(config.stale_command_timeout_sec),
        "stale_observation_action": "hold_zero_command_then_emergency_stop_if_needed",
        "stale_command_action": "hold_last_safe_or_zero_command",
    }


def _safety_policy(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "simulation_only": True,
        "real_hardware_enabled": bool(config.deployment.get("real_hardware_enabled", False)),
        "autonomous_real_flight_enabled": bool(
            config.deployment.get("autonomous_real_flight_enabled", False)
        ),
        "control_barrier_function_required": True,
        "cbf_before_mavsdk_write": True,
        "minimum_actions": [
            "saturate_velocity",
            "check_altitude_bounds",
            "check_geofence_placeholder",
            "check_obstacle_barrier_if_available",
        ],
        "fallback_command": "zero_velocity_hold",
        "emergency_stop_on_runtime_failure": True,
    }


def _failure_handling(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "isaac_step_failure": "stop_loop_and_close_owned_runtime",
        "ros2_sync_timeout": "hold_zero_command_and_mark_sensor_stale",
        "px4_telemetry_stale": "hold_zero_command_then_emergency_stop_if_connected",
        "mavsdk_disconnect": "stop_command_loop_and_record_runtime_failure",
        "frame_transform_missing": "block_coupled_loop_execution",
        "planner_stale": "reuse_latest_safe_command_until_stale_command_timeout_sec",
        "stale_command_timeout_sec": float(config.stale_command_timeout_sec),
    }


def _refusal_rules() -> dict:
    return {
        "refuse_real_hardware_enabled": True,
        "refuse_autonomous_real_flight_enabled": True,
        "refuse_mavsdk_when_sitl_disabled": True,
        "refuse_mavsdk_when_mock_enabled": True,
        "refuse_px4_launch_in_phase6e": True,
        "refuse_missing_prior_reports_when_required": True,
        "refuse_future_coupled_execution_without_frame_transform_policy": True,
    }


def _artifact_policy(config: IsaacPX4BridgeDesignConfig) -> dict:
    return {
        "default_output_path": config.output_path or DEFAULT_OUTPUT_PATH,
        "runtime_report_dir": "outputs/runtime_validation",
        "commit_runtime_reports": False,
        "commit_isaac_logs": False,
        "commit_rosbags": False,
        "commit_px4_logs": False,
        "commit_sitl_artifacts": False,
        "commit_credentials": False,
    }


def _finalize_result(
    result: IsaacPX4BridgeDesignResult,
    config: IsaacPX4BridgeDesignConfig,
    start: float,
) -> dict:
    result.timings["total_sec"] = round(time.perf_counter() - start, 6)
    payload = _json_safe(asdict(result))
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return payload


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
