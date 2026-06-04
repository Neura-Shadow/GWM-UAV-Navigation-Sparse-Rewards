"""Guarded Isaac Sim runtime smoke test for Phase 5-B.

The smoke runner is safe by default: it will not attempt to launch Isaac Sim
unless both runtime opt-in environment gates are set. Normal tests inject a fake
backend through ``IsaacSimRuntime`` and therefore require no Isaac Sim, GPU, ROS2,
MAVSDK, PX4, or hardware.
"""

from __future__ import annotations

import copy
import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping

import numpy as np

from src.digital_twin import IsaacSimRuntime
from src.utils.data_types import SensorObservation

SCHEMA_VERSION = "gwm_isaac_runtime_smoke_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/isaac_runtime_smoke.json"
REQUIRED_ENV_GATES = (
    "GWM_RUN_ISAAC_RUNTIME_TESTS",
    "GWM_ALLOW_OPTIONAL_RUNTIME",
)


@dataclass
class IsaacRuntimeSmokeConfig:
    """Configuration for the guarded Isaac Sim runtime smoke test."""

    frames: int = 3
    headless: bool = True
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False


@dataclass
class IsaacRuntimeSmokeResult:
    """JSON-safe result for an Isaac runtime smoke attempt."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    headless: bool
    frames_requested: int
    frames_completed: int
    descriptor_summary: dict
    sensor_metadata: dict
    sensor_observation_summary: dict
    timings: dict
    errors: list
    closed: bool


def build_tiny_isaac_descriptor() -> dict:
    """Return a tiny descriptor for guarded runtime smoke tests."""
    return {
        "schema_version": "gwm_isaac_runtime_smoke_descriptor_v1",
        "scenario_id": "phase_5b_isaac_runtime_smoke",
        "description": "Tiny guarded Isaac Sim runtime smoke descriptor",
        "backend": "isaac_sim_descriptor",
        "world": {
            "units": "meters",
            "bounds": [-5.0, -5.0, -10.0, 5.0, 5.0, 0.0],
        },
        "vehicle": {
            "name": "UAV",
            "path": "/World/Vehicle/UAV",
            "type": "quadrotor",
            "position": [0.0, 0.0, -5.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "goal": {
            "path": "/World/Goal",
            "position": [2.0, 0.0, -5.0],
            "radius": 0.5,
        },
        "sensors": [
            {
                "name": "DepthCamera",
                "type": "rgbd_camera",
                "path": "/World/Vehicle/UAV/Sensors/DepthCamera",
            },
            {
                "name": "Lidar",
                "type": "lidar",
                "path": "/World/Vehicle/UAV/Sensors/Lidar",
            },
            {
                "name": "Imu",
                "type": "imu",
                "path": "/World/Vehicle/UAV/Sensors/Imu",
            },
        ],
        "metadata": {
            "phase": "5-B",
            "smoke_test": True,
            "source_coordinate_frame": "project_default",
            "target_coordinate_frame": "isaac_z_up_pending",
            "coordinate_conversion_applied": False,
        },
    }


def run_isaac_runtime_smoke(
    config: dict | IsaacRuntimeSmokeConfig | None = None,
    runtime: Any = None,
) -> dict:
    """Run a guarded Isaac Sim runtime smoke test and return a JSON-safe dict."""
    smoke_config, descriptor_path = _normalize_config(config)
    frames_requested = max(1, int(smoke_config.frames))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []

    result = IsaacRuntimeSmokeResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=_env_gate_status(),
        availability={
            "checked": False,
            "isaac_sim_available": None,
            "runtime_injected": runtime is not None,
            "injected_backend": _uses_injected_backend(runtime),
        },
        headless=bool(smoke_config.headless),
        frames_requested=frames_requested,
        frames_completed=0,
        descriptor_summary={},
        sensor_metadata={},
        sensor_observation_summary={},
        timings=timings,
        errors=errors,
        closed=False,
    )

    smoke_runtime = runtime
    uses_injected_backend = _uses_injected_backend(smoke_runtime)
    requires_real_launch_gate = not uses_injected_backend

    if requires_real_launch_gate and not _env_gates_satisfied(result.env_gates):
        missing = [
            name
            for name, gate in result.env_gates.items()
            if not bool(gate.get("enabled", False))
        ]
        result.reason = f"Missing required Isaac runtime env gates: {', '.join(missing)}"
        return _finalize_result(result, smoke_config, start)

    if requires_real_launch_gate:
        result.availability["checked"] = True
        result.availability["isaac_sim_available"] = bool(IsaacSimRuntime.is_available())
        if not result.availability["isaac_sim_available"]:
            result.reason = "Isaac Sim Python runtime is unavailable."
            return _finalize_result(result, smoke_config, start)

    try:
        _check_timeout(start, smoke_config.timeout_sec, "initialization")
        descriptor = _load_descriptor(descriptor_path)
        result.descriptor_summary = _descriptor_summary(descriptor)

        if smoke_runtime is None:
            smoke_runtime = IsaacSimRuntime(
                config={
                    "enabled": True,
                    "headless": smoke_config.headless,
                }
            )

        _time_phase(timings, "launch", lambda: smoke_runtime.launch(headless=smoke_config.headless))
        _check_timeout(start, smoke_config.timeout_sec, "launch")
        _time_phase(timings, "load_descriptor", lambda: smoke_runtime.load_descriptor(descriptor))
        _check_timeout(start, smoke_config.timeout_sec, "descriptor loading")

        latest_snapshot: dict[str, Any] | None = None
        latest_observation: SensorObservation | None = None
        zero_action = [0.0, 0.0, 0.0]
        for _ in range(frames_requested):
            _check_timeout(start, smoke_config.timeout_sec, "stepping")
            smoke_runtime.step(action=zero_action, dt=0.05)
            latest_snapshot = smoke_runtime.read_sensors()
            latest_observation = smoke_runtime.to_sensor_observation(latest_snapshot)
            result.frames_completed += 1

        if latest_snapshot is not None:
            result.sensor_metadata = _sensor_metadata_summary(latest_snapshot)
        if latest_observation is not None:
            result.sensor_observation_summary = _sensor_observation_summary(latest_observation)
        result.status = "passed"
        result.reason = None
    except Exception as exc:  # pragma: no cover - specific branches are tested
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    finally:
        if smoke_runtime is not None:
            try:
                smoke_runtime.close()
                result.closed = True
            except Exception as exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": exc.__class__.__name__,
                        "message": f"close failed: {exc}",
                    }
                )

    return _finalize_result(result, smoke_config, start)


def _normalize_config(
    config: dict | IsaacRuntimeSmokeConfig | None,
) -> tuple[IsaacRuntimeSmokeConfig, str | None]:
    if isinstance(config, IsaacRuntimeSmokeConfig):
        return copy.deepcopy(config), None

    source = _smoke_config_section(config or {})
    descriptor_path = source.get("descriptor_path") or source.get("descriptor")
    return (
        IsaacRuntimeSmokeConfig(
            frames=int(source.get("frames", 3)),
            headless=bool(source.get("headless", True)),
            timeout_sec=float(source.get("timeout_sec", 30.0)),
            output_path=source.get("output_path"),
            write_output=bool(source.get("write_output", True)),
            fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        ),
        str(descriptor_path) if descriptor_path else None,
    )


def _smoke_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("isaac_runtime_smoke") or {})
    if "isaac_runtime_smoke" in config:
        return dict(config.get("isaac_runtime_smoke") or {})
    return dict(config)


def _load_descriptor(descriptor_path: str | None) -> dict:
    if not descriptor_path:
        return build_tiny_isaac_descriptor()

    path = Path(descriptor_path)
    if not path.exists():
        raise RuntimeError(f"Isaac runtime smoke descriptor path does not exist: {path}")
    if path.suffix.lower() != ".json":
        raise RuntimeError("Isaac runtime smoke descriptor must be a JSON descriptor.")
    return json.loads(path.read_text(encoding="utf-8"))


def _env_gate_status() -> dict:
    return {
        name: {
            "present": name in os.environ,
            "enabled": os.environ.get(name) == "1",
        }
        for name in REQUIRED_ENV_GATES
    }


def _env_gates_satisfied(env_gates: Mapping[str, Mapping[str, Any]]) -> bool:
    return all(bool(gate.get("enabled", False)) for gate in env_gates.values())


def _uses_injected_backend(runtime: Any) -> bool:
    if runtime is None:
        return False
    if not isinstance(runtime, IsaacSimRuntime):
        return True
    return getattr(runtime, "_backend", None) is not None


def _time_phase(timings: dict, name: str, callback: Any) -> Any:
    start = time.perf_counter()
    try:
        return callback()
    finally:
        timings[f"{name}_sec"] = round(time.perf_counter() - start, 6)


def _check_timeout(start: float, timeout_sec: float, phase: str) -> None:
    elapsed = time.perf_counter() - start
    if elapsed > float(timeout_sec):
        raise TimeoutError(f"Isaac runtime smoke timed out during {phase} after {elapsed:.2f}s")


def _descriptor_summary(descriptor: Mapping[str, Any]) -> dict:
    metadata = dict(descriptor.get("metadata", {})) if isinstance(descriptor, Mapping) else {}
    sensors = descriptor.get("sensors", []) if isinstance(descriptor, Mapping) else []
    return {
        "schema_version": descriptor.get("schema_version"),
        "scenario_id": descriptor.get("scenario_id"),
        "has_vehicle": "vehicle" in descriptor,
        "has_goal": "goal" in descriptor,
        "sensor_count": len(sensors) if isinstance(sensors, list) else 0,
        "source_coordinate_frame": metadata.get("source_coordinate_frame"),
        "target_coordinate_frame": metadata.get("target_coordinate_frame"),
        "coordinate_conversion_applied": bool(metadata.get("coordinate_conversion_applied", False)),
    }


def _sensor_metadata_summary(snapshot: Mapping[str, Any]) -> dict:
    metadata = snapshot.get("metadata", {})
    return {
        "timestamp": _json_safe(snapshot.get("timestamp")),
        "keys": sorted(str(key) for key in snapshot.keys()),
        "has_rgb": snapshot.get("rgb", snapshot.get("image")) is not None,
        "has_depth": snapshot.get("depth") is not None,
        "has_lidar": snapshot.get("lidar") is not None,
        "has_imu": snapshot.get("imu") is not None,
        "metadata": _json_safe(metadata),
    }


def _sensor_observation_summary(observation: SensorObservation) -> dict:
    return {
        "timestamp": float(observation.timestamp),
        "pose": [float(value) for value in observation.pose],
        "velocity": [float(value) for value in observation.velocity],
        "goal_distance": float(observation.goal_distance),
        "obstacle_distance": float(observation.obstacle_distance),
        "has_image": observation.image is not None,
        "image_shape": _shape(observation.image),
        "has_depth": observation.depth is not None,
        "depth_shape": _shape(observation.depth),
        "has_lidar": observation.lidar is not None,
        "lidar_shape": _shape(observation.lidar),
        "metadata": _json_safe(observation.metadata),
    }


def _shape(value: Any) -> list[int] | None:
    if value is None:
        return None
    return [int(item) for item in np.asarray(value).shape]


def _finalize_result(
    result: IsaacRuntimeSmokeResult,
    config: IsaacRuntimeSmokeConfig,
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
    if isinstance(value, np.ndarray):
        if value.size > 16:
            return {
                "shape": [int(item) for item in value.shape],
                "dtype": str(value.dtype),
            }
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
