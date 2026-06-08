"""Guarded live Cosys-AirSim validation for Phase 7-B.

This module validates an externally started AirSim-family simulator session only
when explicit runtime gates are set. Cosys-AirSim / ``cosysairsim`` is preferred;
legacy AirSim / ``airsim`` is fallback. The backend registry name remains
``airsim``.
"""

from __future__ import annotations

import importlib.util
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Mapping

from src.digital_twin import AirSimRuntime
from src.digital_twin.airsim_runtime import (
    AIRSIM_BACKEND_REGISTRY_NAME,
    AIRSIM_FALLBACK_LABEL,
    AIRSIM_FALLBACK_MODULE,
    AIRSIM_PRIMARY_LABEL,
    AIRSIM_PRIMARY_MODULE,
)

SCHEMA_VERSION = "gwm_phase7_airsim_live_validation_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/airsim_live_validation.json"
BASIC_ENV_GATES = ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_AIRSIM_RUNTIME_TESTS")
API_CONTROL_GATE = "GWM_ALLOW_AIRSIM_API_CONTROL"


@dataclass
class AirSimLiveValidationConfig:
    """Configuration for guarded live AirSim-family validation."""

    frames: int = 3
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    host: str = "127.0.0.1"
    port: int = 41451
    vehicle_name: str = ""
    lidar_name: str = "LidarSensor1"
    rgb_camera_name: str = "0"
    depth_camera_name: str = "0"
    control_dt: float = 0.1
    validate_zero_command: bool = False
    api_control_enabled: bool = False


@dataclass
class AirSimLiveValidationResult:
    """JSON-safe result for guarded live AirSim-family validation."""

    schema_version: str = SCHEMA_VERSION
    status: str = "skipped"
    reason: str | None = None
    runtime_selection: Dict[str, Any] = field(default_factory=dict)
    env_gates: Dict[str, bool] = field(default_factory=dict)
    connection_summary: Dict[str, Any] = field(default_factory=dict)
    vehicle_summary: Dict[str, Any] = field(default_factory=dict)
    settings_summary: Dict[str, Any] = field(default_factory=dict)
    sensor_summary: Dict[str, Any] = field(default_factory=dict)
    sensor_observation_summary: Dict[str, Any] = field(default_factory=dict)
    command_summary: Dict[str, Any] = field(default_factory=dict)
    safety_summary: Dict[str, Any] = field(default_factory=dict)
    timings: Dict[str, Any] = field(default_factory=dict)
    errors: list[dict[str, str]] = field(default_factory=list)
    closed: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def run_airsim_live_validation(
    config: dict | AirSimLiveValidationConfig | None = None,
    *,
    runtime: AirSimRuntime | None = None,
    import_spec: Callable[[str], Any] | None = None,
) -> dict:
    """Run guarded live AirSim-family validation and return a JSON-safe dict."""
    validation_config = _normalize_config(config)
    start = time.perf_counter()
    env_gates = _env_gates()
    result = AirSimLiveValidationResult(
        runtime_selection=_runtime_selection(import_spec),
        env_gates=env_gates,
        connection_summary={
            "host": validation_config.host,
            "port": validation_config.port,
            "connection_attempted": False,
            "external_session_required": True,
            "simulator_launch_attempted": False,
        },
        command_summary={
            "validate_zero_command": bool(validation_config.validate_zero_command),
            "api_control_enabled": bool(validation_config.api_control_enabled),
            "api_control_gate_required": bool(validation_config.validate_zero_command),
            "commands_sent": 0,
        },
        safety_summary={
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "unreal_launch_attempted": False,
            "airsim_launch_attempted": False,
            "zero_velocity_only": True,
            "arms_or_takeoffs": False,
        },
        timings={"started_at_unix": time.time()},
    )
    injected = runtime is not None

    missing_basic = [name for name in BASIC_ENV_GATES if not env_gates[name]]
    if not injected and missing_basic:
        result.status = "skipped"
        result.reason = f"Missing required AirSim-family live validation env gates: {', '.join(missing_basic)}"
        return _finalize(result, validation_config, start)

    if validation_config.validate_zero_command and not env_gates[API_CONTROL_GATE]:
        result.status = "skipped"
        result.reason = f"Missing required API-control env gate: {API_CONTROL_GATE}"
        return _finalize(result, validation_config, start)

    if not injected and result.runtime_selection.get("selected_module") is None:
        result.status = "runtime_unavailable"
        result.reason = (
            "Cosys-AirSim / cosysairsim is unavailable; legacy AirSim / airsim "
            "fallback is also unavailable."
        )
        if validation_config.fail_on_unavailable:
            result.errors.append({"type": "RuntimeError", "message": result.reason})
        return _finalize(result, validation_config, start)

    runtime_obj = runtime or AirSimRuntime(_runtime_config(validation_config))
    try:
        result.connection_summary["connection_attempted"] = True
        runtime_obj.connect()
        result.connection_summary["connected"] = True
        client = runtime_obj.client
        vehicle_summary = _vehicle_summary(client, validation_config.vehicle_name)
        selected_vehicle = vehicle_summary.get("selected_vehicle") or validation_config.vehicle_name
        result.vehicle_summary = vehicle_summary
        result.settings_summary = _settings_summary(client)

        observation = runtime_obj.reset()
        frames_completed = 1
        if validation_config.validate_zero_command:
            runtime_obj.step((0.0, 0.0, 0.0), dt=validation_config.control_dt)
            result.command_summary["commands_sent"] = 1
            observation = runtime_obj.to_sensor_observation(runtime_obj.read_sensors())

        for _ in range(max(0, int(validation_config.frames) - frames_completed)):
            observation = runtime_obj.to_sensor_observation(runtime_obj.read_sensors())
            frames_completed += 1

        result.sensor_summary = {
            "frames_requested": max(1, int(validation_config.frames)),
            "frames_completed": frames_completed,
            "has_image": observation.image is not None,
            "has_depth": observation.depth is not None,
            "has_lidar": observation.lidar is not None,
            "selected_vehicle": selected_vehicle,
        }
        result.sensor_observation_summary = _observation_summary(observation)
        result.status = "passed"
        result.reason = None
    except ConnectionError as exc:
        result.status = "connection_failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    finally:
        try:
            runtime_obj.close()
            result.closed = True
        except Exception as exc:  # pragma: no cover
            result.closed = False
            result.errors.append({"type": exc.__class__.__name__, "message": f"close failed: {exc}"})

    return _finalize(result, validation_config, start)


def _normalize_config(
    config: dict | AirSimLiveValidationConfig | None,
) -> AirSimLiveValidationConfig:
    if isinstance(config, AirSimLiveValidationConfig):
        return config
    source = _config_section(config or {})
    return AirSimLiveValidationConfig(
        frames=int(source.get("frames", 3)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        host=str(source.get("host", "127.0.0.1")),
        port=int(source.get("port", 41451)),
        vehicle_name=str(source.get("vehicle_name", "")),
        lidar_name=str(source.get("lidar_name", "LidarSensor1")),
        rgb_camera_name=str(source.get("rgb_camera_name", "0")),
        depth_camera_name=str(source.get("depth_camera_name", "0")),
        control_dt=float(source.get("control_dt", 0.1)),
        validate_zero_command=bool(source.get("validate_zero_command", False)),
        api_control_enabled=bool(source.get("api_control_enabled", False)),
    )


def _config_section(config: Mapping[str, Any]) -> dict:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, Mapping):
        return dict(runtime_validation.get("airsim_live_validation") or {})
    return dict(config.get("airsim_live_validation") or config)


def _runtime_config(config: AirSimLiveValidationConfig) -> dict:
    return {
        "host": config.host,
        "port": config.port,
        "vehicle_name": config.vehicle_name,
        "lidar_name": config.lidar_name,
        "rgb_camera_name": config.rgb_camera_name,
        "depth_camera_name": config.depth_camera_name,
        "control_dt": config.control_dt,
        "api_control_enabled": bool(config.api_control_enabled and config.validate_zero_command),
        "reset_on_reset": False,
        "arm_on_reset": False,
        "takeoff_on_reset": False,
    }


def _runtime_selection(import_spec: Callable[[str], Any] | None = None) -> Dict[str, Any]:
    finder = import_spec or importlib.util.find_spec
    modules = {
        AIRSIM_PRIMARY_MODULE: _safe_find_spec(finder, AIRSIM_PRIMARY_MODULE),
        AIRSIM_FALLBACK_MODULE: _safe_find_spec(finder, AIRSIM_FALLBACK_MODULE),
    }
    selected = None
    for module_name in (AIRSIM_PRIMARY_MODULE, AIRSIM_FALLBACK_MODULE):
        if modules[module_name]["available"]:
            selected = module_name
            break
    return {
        "backend_registry_name": AIRSIM_BACKEND_REGISTRY_NAME,
        "primary_runtime": AIRSIM_PRIMARY_MODULE,
        "primary_runtime_label": AIRSIM_PRIMARY_LABEL,
        "fallback_runtime": AIRSIM_FALLBACK_MODULE,
        "fallback_runtime_label": AIRSIM_FALLBACK_LABEL,
        "modules": modules,
        "selected_module": selected,
        "selected_runtime_label": _runtime_label(selected),
    }


def _safe_find_spec(finder: Callable[[str], Any], module_name: str) -> Dict[str, Any]:
    try:
        spec = finder(module_name)
        return {
            "available": spec is not None,
            "origin": None if spec is None else getattr(spec, "origin", None),
        }
    except Exception as exc:
        return {"available": False, "origin": None, "error": str(exc)}


def _runtime_label(module_name: str | None) -> str | None:
    if module_name == AIRSIM_PRIMARY_MODULE:
        return AIRSIM_PRIMARY_LABEL
    if module_name == AIRSIM_FALLBACK_MODULE:
        return AIRSIM_FALLBACK_LABEL
    return None


def _vehicle_summary(client: object | None, configured_vehicle: str) -> Dict[str, Any]:
    vehicles = _call_if_present(client, "listVehicles")
    if vehicles is None:
        vehicles = []
    vehicles = [str(vehicle) for vehicle in vehicles]
    selected_vehicle = configured_vehicle or (vehicles[0] if vehicles else "")
    return {
        "configured_vehicle": configured_vehicle,
        "vehicles_available": vehicles,
        "selected_vehicle": selected_vehicle,
        "list_supported": bool(vehicles),
    }


def _settings_summary(client: object | None) -> Dict[str, Any]:
    settings_text = _call_if_present(client, "getSettingsString")
    supported = settings_text is not None
    if settings_text is None:
        settings_text = ""
    settings_text = str(settings_text)
    return {
        "supported": supported,
        "length": len(settings_text),
        "has_sensors": '"Sensors"' in settings_text or "Sensors" in settings_text,
        "has_lidar": "Lidar" in settings_text,
        "truncated_preview": settings_text[:240],
    }


def _observation_summary(observation: Any) -> Dict[str, Any]:
    return {
        "timestamp": float(observation.timestamp),
        "pose": list(observation.pose),
        "velocity": list(observation.velocity),
        "goal_distance": float(observation.goal_distance),
        "obstacle_distance": float(observation.obstacle_distance),
        "has_image": observation.image is not None,
        "has_depth": observation.depth is not None,
        "has_lidar": observation.lidar is not None,
        "metadata": dict(observation.metadata),
    }


def _env_gates() -> Dict[str, bool]:
    return {
        "GWM_ALLOW_OPTIONAL_RUNTIME": os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") == "1",
        "GWM_RUN_AIRSIM_RUNTIME_TESTS": os.environ.get("GWM_RUN_AIRSIM_RUNTIME_TESTS") == "1",
        API_CONTROL_GATE: os.environ.get(API_CONTROL_GATE) == "1",
    }


def _call_if_present(target: object | None, name: str, *args: Any, **kwargs: Any) -> Any:
    if target is None:
        return None
    method = getattr(target, name, None)
    if callable(method):
        return method(*args, **kwargs)
    return None


def _finalize(
    result: AirSimLiveValidationResult,
    config: AirSimLiveValidationConfig,
    start: float,
) -> dict:
    result.timings["total_sec"] = round(time.perf_counter() - start, 6)
    payload = result.to_dict()
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
