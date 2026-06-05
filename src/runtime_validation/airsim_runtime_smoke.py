"""Guarded AirSim / CosysAirSim runtime smoke test."""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping

from src.digital_twin import AirSimRuntime

SCHEMA_VERSION = "gwm_phase7_airsim_runtime_smoke_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/airsim_runtime_smoke.json"
REQUIRED_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_AIRSIM_RUNTIME_TESTS",
    "GWM_ALLOW_AIRSIM_API_CONTROL",
)


@dataclass
class AirSimRuntimeSmokeConfig:
    """Configuration for the guarded AirSim runtime smoke."""

    frames: int = 3
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    host: str = "127.0.0.1"
    port: int = 41451
    vehicle_name: str = ""
    api_control_enabled: bool = True
    reset_on_reset: bool = False
    arm_on_reset: bool = False
    takeoff_on_reset: bool = False
    control_dt: float = 0.1


@dataclass
class AirSimRuntimeSmokeResult:
    """JSON-safe result for a guarded AirSim runtime smoke run."""

    schema_version: str = SCHEMA_VERSION
    status: str = "skipped"
    reason: str | None = None
    env_gates: Dict[str, bool] = field(default_factory=dict)
    availability: Dict[str, Any] = field(default_factory=dict)
    frames_requested: int = 0
    frames_completed: int = 0
    sensor_observation_summary: Dict[str, Any] = field(default_factory=dict)
    command_summary: Dict[str, Any] = field(default_factory=dict)
    timings: Dict[str, Any] = field(default_factory=dict)
    errors: list[dict[str, str]] = field(default_factory=list)
    closed: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def run_airsim_runtime_smoke(
    config: dict | AirSimRuntimeSmokeConfig | None = None,
    *,
    runtime: AirSimRuntime | None = None,
) -> dict:
    """Run the guarded AirSim runtime smoke and return a JSON-safe dict."""
    smoke_config = _normalize_config(config)
    start = time.perf_counter()
    result = AirSimRuntimeSmokeResult(
        env_gates=_env_gates(),
        frames_requested=int(smoke_config.frames),
        timings={"started_at_unix": time.time()},
    )
    injected = runtime is not None and runtime.client is not None

    if not injected:
        missing = [name for name, present in result.env_gates.items() if not present]
        if missing:
            result.status = "skipped"
            result.reason = f"Missing required AirSim runtime env gates: {', '.join(missing)}"
            return _finalize(result, smoke_config, start)

        available = AirSimRuntime.is_available()
        result.availability = {"airsim_available": available, "connection_attempted": False}
        if not available:
            result.status = "runtime_unavailable"
            result.reason = "AirSim / CosysAirSim Python runtime is unavailable."
            if smoke_config.fail_on_unavailable:
                result.errors.append({"type": "RuntimeError", "message": result.reason})
            return _finalize(result, smoke_config, start)

    runtime_obj = runtime or AirSimRuntime(_runtime_config(smoke_config))
    result.availability.setdefault("airsim_available", True)
    try:
        runtime_obj.connect()
        result.availability["connection_attempted"] = True
        observation = runtime_obj.reset()
        for index in range(max(1, int(smoke_config.frames))):
            command = [0.0, 0.0, 0.0]
            if index > 0:
                runtime_obj.step(command, dt=smoke_config.control_dt)
                observation = runtime_obj.to_sensor_observation(runtime_obj.read_sensors())
            result.frames_completed += 1
        result.sensor_observation_summary = {
            "timestamp": observation.timestamp,
            "pose": list(observation.pose),
            "velocity": list(observation.velocity),
            "goal_distance": observation.goal_distance,
            "obstacle_distance": observation.obstacle_distance,
            "has_image": observation.image is not None,
            "has_depth": observation.depth is not None,
            "has_lidar": observation.lidar is not None,
            "metadata": dict(observation.metadata),
        }
        result.command_summary = {
            "api_control_enabled": bool(_runtime_config(smoke_config)["api_control_enabled"]),
            "commands_sent": max(0, result.frames_completed - 1),
            "unreal_launch_attempted": False,
        }
        result.status = "passed"
        result.reason = None
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

    return _finalize(result, smoke_config, start)


def _normalize_config(config: dict | AirSimRuntimeSmokeConfig | None) -> AirSimRuntimeSmokeConfig:
    if isinstance(config, AirSimRuntimeSmokeConfig):
        return config
    source = _config_section(config or {})
    return AirSimRuntimeSmokeConfig(
        frames=int(source.get("frames", 3)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        host=str(source.get("host", "127.0.0.1")),
        port=int(source.get("port", 41451)),
        vehicle_name=str(source.get("vehicle_name", "")),
        api_control_enabled=bool(source.get("api_control_enabled", True)),
        reset_on_reset=bool(source.get("reset_on_reset", False)),
        arm_on_reset=bool(source.get("arm_on_reset", False)),
        takeoff_on_reset=bool(source.get("takeoff_on_reset", False)),
        control_dt=float(source.get("control_dt", 0.1)),
    )


def _config_section(config: Mapping[str, Any]) -> dict:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, Mapping):
        return dict(runtime_validation.get("airsim_runtime_smoke") or {})
    return dict(config.get("airsim_runtime_smoke") or config)


def _runtime_config(config: AirSimRuntimeSmokeConfig) -> dict:
    return {
        "host": config.host,
        "port": config.port,
        "vehicle_name": config.vehicle_name,
        "control_dt": config.control_dt,
        "api_control_enabled": config.api_control_enabled,
        "reset_on_reset": config.reset_on_reset,
        "arm_on_reset": config.arm_on_reset,
        "takeoff_on_reset": config.takeoff_on_reset,
    }


def _env_gates() -> Dict[str, bool]:
    return {name: os.environ.get(name) == "1" for name in REQUIRED_ENV_GATES}


def _finalize(
    result: AirSimRuntimeSmokeResult,
    config: AirSimRuntimeSmokeConfig,
    start: float,
) -> dict:
    result.timings["total_sec"] = round(time.perf_counter() - start, 6)
    payload = result.to_dict()
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
