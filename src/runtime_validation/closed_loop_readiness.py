"""Closed-loop mock-to-SITL integration readiness for Phase 5-E.

This module describes and validates the closed-loop wiring from the Phase 4-F
GWM demo to optional runtime backends. The default path is mock-only: it does
not launch Isaac Sim, start ROS2, connect to MAVSDK/PX4 SITL, launch PX4, or
touch real hardware.
"""

from __future__ import annotations

import copy
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping

from src.generated_world_model import run_demo

SCHEMA_VERSION = "gwm_closed_loop_readiness_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/closed_loop_readiness.json"

DEFAULT_PRIOR_SMOKE_REPORTS = {
    "isaac_runtime": "outputs/runtime_validation/isaac_runtime_smoke.json",
    "ros2_sensor_sync": "outputs/runtime_validation/ros2_sensor_sync_smoke.json",
    "mavsdk_px4_sitl": "outputs/runtime_validation/mavsdk_sitl_smoke.json",
}

RUNTIME_GATE_GROUPS = {
    "isaac": (
        "GWM_RUN_ISAAC_RUNTIME_TESTS",
        "GWM_ALLOW_OPTIONAL_RUNTIME",
    ),
    "ros2_sensor_sync": (
        "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
        "GWM_ALLOW_OPTIONAL_RUNTIME",
    ),
    "mavsdk_px4_sitl": (
        "GWM_RUN_MAVSDK_SITL_TESTS",
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_ALLOW_SITL_COMMANDS",
    ),
}

SAFE_DEPLOYMENT_DEFAULTS = {
    "mock": True,
    "sitl_enabled": False,
    "real_hardware_enabled": False,
    "autonomous_real_flight_enabled": False,
}


@dataclass
class ClosedLoopReadinessConfig:
    """Configuration for the Phase 5-E closed-loop readiness check."""

    steps: int = 5
    timeout_sec: float = 60.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    require_prior_smokes: bool = False
    prior_smoke_reports: Dict[str, str] = field(
        default_factory=lambda: dict(DEFAULT_PRIOR_SMOKE_REPORTS)
    )


@dataclass
class ClosedLoopReadinessResult:
    """JSON-safe result for the Phase 5-E readiness check."""

    schema_version: str
    status: str
    reason: str | None
    runtime_gates: dict
    pipeline_summary: dict
    demo_summary: dict
    backend_readiness: dict
    safety_summary: dict
    metrics: dict
    timings: dict
    errors: list


def build_closed_loop_pipeline_plan() -> dict:
    """Return the planned closed-loop architecture as a JSON-safe dict."""
    flow = [
        "Observation backend",
        "ObservationBuffer",
        "Generated World Model rollout",
        "Candidate trajectory sampler",
        "Trajectory scorer",
        "ControlBarrierFunction safety gate",
        "Execution backend",
        "Runtime metrics / failure handling",
    ]
    return {
        "schema_version": "gwm_closed_loop_pipeline_plan_v1",
        "flow": flow,
        "default_observation_backend": "mock",
        "default_execution_backend": "mock",
        "optional_observation_backends": ["isaac", "ros2_sensor_sync"],
        "optional_execution_backends": ["isaac", "mavsdk_px4_sitl"],
        "components": {
            "observation_backend": {
                "default": "synthetic SensorObservation source",
                "optional": ["IsaacSimRuntime", "ROS2SensorSynchronizer"],
            },
            "context": "ObservationBuffer",
            "planner": {
                "rollout": "AutoregressiveRollout",
                "sampler": "CandidateTrajectorySampler",
                "scorer": "TrajectoryScorer",
            },
            "safety_gate": "ControlBarrierFunction",
            "execution_backend": {
                "default": "mock command backend",
                "optional": ["IsaacSimRuntime", "MAVLinkBridge SITL mode"],
            },
            "failure_handling": [
                "record runtime_unavailable",
                "record safety_stop",
                "record emergency_stop when backend supports it",
            ],
        },
        "non_goals": [
            "launch Isaac Sim",
            "start ROS2 nodes",
            "connect to MAVSDK/PX4 SITL",
            "launch PX4",
            "run hardware checks",
            "enable autonomous real flight",
        ],
    }


def run_closed_loop_readiness(
    config: dict | ClosedLoopReadinessConfig | None = None,
) -> dict:
    """Run the mock-first closed-loop readiness check and return a dict."""
    readiness_config = _normalize_config(config)
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []
    runtime_gates = _runtime_gate_status()
    backend_readiness = _backend_readiness(readiness_config, runtime_gates)
    safety_summary = _safety_summary(config)

    result = ClosedLoopReadinessResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        runtime_gates=runtime_gates,
        pipeline_summary=build_closed_loop_pipeline_plan(),
        demo_summary={"executed": False},
        backend_readiness=backend_readiness,
        safety_summary=safety_summary,
        metrics={},
        timings=timings,
        errors=errors,
    )

    unsafe_reason = _unsafe_deployment_reason(safety_summary)
    if unsafe_reason is not None:
        result.status = "failed"
        result.reason = unsafe_reason
        result.errors.append({"type": "UnsafeDeploymentConfig", "message": unsafe_reason})
        return _finalize_result(result, readiness_config, start)

    if readiness_config.require_prior_smokes:
        missing = _missing_prior_smokes(backend_readiness)
        if missing:
            result.status = "failed" if readiness_config.fail_on_unavailable else "skipped"
            result.reason = f"Missing prior smoke reports: {', '.join(missing)}"
            result.metrics = {
                "mock_demo_executed": False,
                "prior_smoke_reports_required": True,
                "prior_smoke_reports_missing": len(missing),
            }
            return _finalize_result(result, readiness_config, start)

    try:
        _check_timeout(start, readiness_config.timeout_sec, "before mock demo")
        demo_config = _mock_demo_config(readiness_config)
        demo_started = time.perf_counter()
        demo_result = run_demo(demo_config)
        timings["mock_demo_sec"] = round(time.perf_counter() - demo_started, 6)
        _check_timeout(start, readiness_config.timeout_sec, "after mock demo")

        result.demo_summary = _demo_summary(demo_result)
        result.metrics = _metrics_summary(demo_result, backend_readiness)
        result.status = "passed"
        result.reason = None
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})

    return _finalize_result(result, readiness_config, start)


def _normalize_config(
    config: dict | ClosedLoopReadinessConfig | None,
) -> ClosedLoopReadinessConfig:
    if isinstance(config, ClosedLoopReadinessConfig):
        return copy.deepcopy(config)

    source = _readiness_config_section(config or {})
    smoke_reports = dict(DEFAULT_PRIOR_SMOKE_REPORTS)
    smoke_reports.update(source.get("prior_smoke_reports") or {})
    return ClosedLoopReadinessConfig(
        steps=int(source.get("steps", 5)),
        timeout_sec=float(source.get("timeout_sec", 60.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        require_prior_smokes=bool(source.get("require_prior_smokes", False)),
        prior_smoke_reports={str(key): str(value) for key, value in smoke_reports.items()},
    )


def _readiness_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("closed_loop_readiness") or {})
    if "closed_loop_readiness" in config:
        return dict(config.get("closed_loop_readiness") or {})
    return dict(config)


def _runtime_gate_status() -> dict:
    groups = {}
    for group, names in RUNTIME_GATE_GROUPS.items():
        gates = {
            name: {
                "present": name in os.environ,
                "enabled": os.environ.get(name) == "1",
            }
            for name in names
        }
        groups[group] = {
            "required_env_gates": list(names),
            "satisfied": all(bool(gate["enabled"]) for gate in gates.values()),
            "gates": gates,
        }
    return groups


def _backend_readiness(
    config: ClosedLoopReadinessConfig,
    runtime_gates: Mapping[str, Any],
) -> dict:
    prior_reports = {
        name: {
            "path": path,
            "exists": Path(path).exists(),
        }
        for name, path in config.prior_smoke_reports.items()
    }
    return {
        "mock": {
            "ready": True,
            "observation_backend": True,
            "execution_backend": True,
            "runtime_required": False,
        },
        "isaac": {
            "planned_connection_point": "IsaacSimRuntime / IsaacSimNavigationEnv",
            "env_gates_satisfied": bool(runtime_gates["isaac"]["satisfied"]),
            "prior_smoke_report": prior_reports.get("isaac_runtime", {}),
            "invoked": False,
        },
        "ros2_sensor_sync": {
            "planned_connection_point": "ROS2SensorSynchronizer",
            "env_gates_satisfied": bool(runtime_gates["ros2_sensor_sync"]["satisfied"]),
            "prior_smoke_report": prior_reports.get("ros2_sensor_sync", {}),
            "invoked": False,
        },
        "mavsdk_px4_sitl": {
            "planned_connection_point": "MAVLinkBridge SITL mode",
            "env_gates_satisfied": bool(runtime_gates["mavsdk_px4_sitl"]["satisfied"]),
            "prior_smoke_report": prior_reports.get("mavsdk_px4_sitl", {}),
            "px4_launch_attempted": False,
            "invoked": False,
        },
    }


def _missing_prior_smokes(backend_readiness: Mapping[str, Any]) -> list[str]:
    missing: list[str] = []
    for name in ("isaac", "ros2_sensor_sync", "mavsdk_px4_sitl"):
        report = backend_readiness.get(name, {}).get("prior_smoke_report", {})
        if not bool(report.get("exists", False)):
            missing.append(str(report.get("path", name)))
    return missing


def _safety_summary(config: dict | ClosedLoopReadinessConfig | None) -> dict:
    deployment = dict(SAFE_DEPLOYMENT_DEFAULTS)
    if isinstance(config, Mapping):
        source_deployment = config.get("deployment")
        if isinstance(source_deployment, Mapping):
            deployment.update(source_deployment)
        section = _readiness_config_section(config)
        section_deployment = section.get("deployment")
        if isinstance(section_deployment, Mapping):
            deployment.update(section_deployment)
    return {
        "deployment": _json_safe(deployment),
        "mock": bool(deployment.get("mock", True)),
        "sitl_enabled": bool(deployment.get("sitl_enabled", False)),
        "real_hardware_enabled": bool(deployment.get("real_hardware_enabled", False)),
        "autonomous_real_flight_enabled": bool(
            deployment.get("autonomous_real_flight_enabled", False)
        ),
        "optional_runtime_invoked": False,
        "isaac_launch_invoked": False,
        "ros2_start_invoked": False,
        "mavsdk_connect_invoked": False,
        "px4_launch_invoked": False,
        "hardware_check_invoked": False,
    }


def _unsafe_deployment_reason(safety_summary: Mapping[str, Any]) -> str | None:
    if bool(safety_summary.get("real_hardware_enabled", False)):
        return "real_hardware_enabled=True is not allowed for closed-loop readiness."
    if bool(safety_summary.get("autonomous_real_flight_enabled", False)):
        return (
            "autonomous_real_flight_enabled=True is not allowed for closed-loop "
            "readiness."
        )
    return None


def _mock_demo_config(config: ClosedLoopReadinessConfig) -> dict:
    steps = max(1, int(config.steps))
    return {
        "demo": {
            "observation_source": "mock",
            "execution_backend": "mock",
            "steps": steps,
            "horizon": 2,
            "num_candidates": 2,
            "seed": 11,
            "device": "cpu",
            "write_output": False,
            "start_pose": [0.0, 0.0, -5.0],
            "goal": [6.0, 0.0, -5.0],
            "control_dt": 0.4,
            "max_speed": 2.0,
            "context_length": 2,
            "image_height": 16,
            "image_width": 16,
            "mock_obstacle_distance": 20.0,
            "min_safe_depth": 0.5,
            "min_obstacle_distance": 4.0,
        },
        "deployment": dict(SAFE_DEPLOYMENT_DEFAULTS),
        "model": {
            "image_height": 16,
            "image_width": 16,
            "context_length": 2,
            "horizon": 2,
            "latent_dim": 12,
            "visual_feature_dim": 12,
            "state_feature_dim": 8,
            "conditioning_dim": 10,
            "hidden_dim": 16,
        },
        "trajectory_scoring": {
            "weights": {
                "goal_progress": 1.0,
                "collision_risk": 2.0,
                "uncertainty": 0.2,
                "energy": 0.01,
                "smoothness": 0.01,
                "altitude_violation": 4.0,
                "geofence_violation": 4.0,
            }
        },
        "safety": {
            "velocity_limits": {
                "max_vx": 4.0,
                "max_vy": 4.0,
                "max_vz": 2.0,
                "max_yaw_rate": 1.0,
            },
            "altitude_bounds": {"min_altitude": 0.5, "max_altitude": 120.0},
            "geofence": {"enabled": False},
            "cbf": {"enabled": True, "min_obstacle_distance": 4.0, "alpha": 1.0},
        },
    }


def _demo_summary(demo_result: Mapping[str, Any]) -> dict:
    metrics = dict(demo_result.get("metrics") or {})
    backend = dict(demo_result.get("backend_summary") or {})
    return {
        "executed": True,
        "schema_version": demo_result.get("schema_version"),
        "final_status": demo_result.get("final_status"),
        "observation_source": backend.get("observation_source"),
        "execution_backend": backend.get("execution_backend"),
        "mock_default": bool(backend.get("mock_default", False)),
        "steps": int(metrics.get("total_steps", 0)),
        "commands_sent": int(metrics.get("commands_sent", 0)),
        "safety_overrides": int(metrics.get("safety_overrides", 0)),
        "emergency_stops": int(metrics.get("emergency_stops", 0)),
        "runtime_unavailable_reason": backend.get("runtime_unavailable_reason"),
    }


def _metrics_summary(
    demo_result: Mapping[str, Any],
    backend_readiness: Mapping[str, Any],
) -> dict:
    demo_metrics = dict(demo_result.get("metrics") or {})
    prior_reports = [
        backend_readiness.get(name, {}).get("prior_smoke_report", {})
        for name in ("isaac", "ros2_sensor_sync", "mavsdk_px4_sitl")
    ]
    return {
        "mock_demo_executed": True,
        "mock_demo_final_status": demo_result.get("final_status"),
        "mock_demo_steps": int(demo_metrics.get("total_steps", 0)),
        "commands_sent": int(demo_metrics.get("commands_sent", 0)),
        "safety_overrides": int(demo_metrics.get("safety_overrides", 0)),
        "emergency_stops": int(demo_metrics.get("emergency_stops", 0)),
        "prior_smoke_reports_present": sum(
            1 for report in prior_reports if bool(report.get("exists", False))
        ),
        "optional_runtime_invocations": 0,
    }


def _check_timeout(start: float, timeout_sec: float, phase: str) -> None:
    elapsed = time.perf_counter() - start
    if elapsed > float(timeout_sec):
        raise TimeoutError(
            f"Closed-loop readiness timed out during {phase} after {elapsed:.2f}s"
        )


def _finalize_result(
    result: ClosedLoopReadinessResult,
    config: ClosedLoopReadinessConfig,
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
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
