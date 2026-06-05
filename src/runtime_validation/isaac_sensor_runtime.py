"""Guarded Isaac Sim / Isaac Lab sensor runtime execution for Phase 6-B.

The runner exercises the real Isaac runtime seam when available and explicitly
gated. Normal tests inject a fake backend through ``IsaacSimRuntime`` and still
exercise ``IsaacSimNavigationEnv``, ``SensorObservation``, and
``ObservationBuffer`` without requiring Isaac Sim, ROS2, MAVSDK, PX4, or
hardware.
"""

from __future__ import annotations

import copy
import json
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

import numpy as np

from src.digital_twin import IsaacSimRuntime
from src.env import IsaacSimNavigationEnv
from src.generated_world_model import ObservationBuffer
from src.runtime_validation.isaac_runtime_smoke import build_tiny_isaac_descriptor
from src.utils.data_types import SensorObservation

SCHEMA_VERSION = "gwm_phase6_isaac_sensor_runtime_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/isaac_sensor_runtime.json"
REQUIRED_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_ISAAC_RUNTIME_TESTS",
)


@dataclass
class IsaacSensorRuntimeConfig:
    """Configuration for the guarded Phase 6-B Isaac sensor runtime run."""

    frames: int = 5
    headless: bool = True
    timeout_sec: float = 60.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    descriptor_path: str | None = None
    context_length: int = 3
    image_height: int = 32
    image_width: int = 32
    timestep: float = 0.05


@dataclass
class IsaacSensorRuntimeResult:
    """JSON-safe result for Phase 6-B Isaac sensor runtime execution."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    setup_instructions: list
    safety_summary: dict
    frames_requested: int
    frames_completed: int
    observations_collected: int
    descriptor_summary: dict
    sensor_summary: dict
    observation_buffer_summary: dict
    execution_summary: dict
    timings: dict
    errors: list
    closed: bool


def run_isaac_sensor_runtime(
    config: dict | IsaacSensorRuntimeConfig | None = None,
    *,
    runtime: IsaacSimRuntime | None = None,
    env: IsaacSimNavigationEnv | None = None,
    observation_buffer: ObservationBuffer | None = None,
) -> dict:
    """Run a guarded Isaac sensor runtime execution and return a dict."""
    sensor_config = _normalize_config(config)
    frames_requested = max(1, int(sensor_config.frames))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []
    env_gates = _env_gate_status()
    uses_injected_runtime = runtime is not None or env is not None

    result = IsaacSensorRuntimeResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=env_gates,
        availability={
            "checked": False,
            "isaac_sim_available": None,
            "runtime_injected": runtime is not None,
            "env_injected": env is not None,
            "injected_backend": _uses_injected_backend(runtime, env),
            "real_runtime_attempt": not uses_injected_runtime,
        },
        setup_instructions=_setup_instructions(),
        safety_summary={
            "simulation_only": True,
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "ros2_started": False,
            "mavsdk_connected": False,
            "px4_launched": False,
            "hardware_check_run": False,
        },
        frames_requested=frames_requested,
        frames_completed=0,
        observations_collected=0,
        descriptor_summary={},
        sensor_summary={},
        observation_buffer_summary={},
        execution_summary={
            "used_isaac_sim_runtime": True,
            "used_isaac_sim_navigation_env": True,
            "used_observation_buffer": True,
            "action": [0.0, 0.0, 0.0],
        },
        timings=timings,
        errors=errors,
        closed=False,
    )

    if not uses_injected_runtime and not _env_gates_satisfied(env_gates):
        missing = [
            name for name, gate in env_gates.items() if not bool(gate.get("enabled", False))
        ]
        result.reason = f"Missing required Isaac sensor runtime env gates: {', '.join(missing)}"
        return _finalize_result(result, sensor_config, start)

    if not uses_injected_runtime:
        result.availability["checked"] = True
        result.availability["isaac_sim_available"] = bool(IsaacSimRuntime.is_available())
        if not result.availability["isaac_sim_available"]:
            result.status = "runtime_unavailable"
            result.reason = "Isaac Sim / Isaac Lab Python runtime is unavailable."
            return _finalize_result(result, sensor_config, start)

    runtime_env = env
    try:
        _check_timeout(start, sensor_config.timeout_sec, "initialization")
        descriptor = _load_descriptor(sensor_config.descriptor_path)
        result.descriptor_summary = _descriptor_summary(descriptor)

        if runtime_env is None:
            runtime_obj = runtime or IsaacSimRuntime(
                config={
                    "enabled": True,
                    "headless": sensor_config.headless,
                    "timestep": sensor_config.timestep,
                }
            )
            runtime_env = IsaacSimNavigationEnv(
                descriptor=descriptor,
                runtime=runtime_obj,
                config={
                    "headless": sensor_config.headless,
                    "launch_on_reset": True,
                    "control_dt": sensor_config.timestep,
                    "timestep": sensor_config.timestep,
                    "max_steps": frames_requested + 1,
                },
            )

        buffer = observation_buffer or ObservationBuffer(
            context_length=int(sensor_config.context_length),
            image_size=(int(sensor_config.image_height), int(sensor_config.image_width)),
        )

        observations: list[SensorObservation] = []
        reset_started = time.perf_counter()
        first_observation = runtime_env.reset()
        timings["reset_sec"] = round(time.perf_counter() - reset_started, 6)
        observations.append(first_observation)
        buffer.append(first_observation)

        action = np.zeros(3, dtype=np.float32)
        step_started = time.perf_counter()
        for _ in range(frames_requested):
            _check_timeout(start, sensor_config.timeout_sec, "stepping")
            observation, _, _, _ = runtime_env.step(action)
            observations.append(observation)
            buffer.append(observation)
            result.frames_completed += 1
        timings["step_loop_sec"] = round(time.perf_counter() - step_started, 6)

        result.observations_collected = len(observations)
        result.sensor_summary = _sensor_observation_summary(observations[-1])
        result.observation_buffer_summary = _buffer_summary(buffer)
        result.status = "passed"
        result.reason = None
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    finally:
        if runtime_env is not None:
            try:
                runtime_env.close()
                result.closed = True
            except Exception as exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": exc.__class__.__name__,
                        "message": f"close failed: {exc}",
                    }
                )

    return _finalize_result(result, sensor_config, start)


def _normalize_config(
    config: dict | IsaacSensorRuntimeConfig | None,
) -> IsaacSensorRuntimeConfig:
    if isinstance(config, IsaacSensorRuntimeConfig):
        return copy.deepcopy(config)

    source = _sensor_runtime_config_section(config or {})
    return IsaacSensorRuntimeConfig(
        frames=int(source.get("frames", 5)),
        headless=bool(source.get("headless", True)),
        timeout_sec=float(source.get("timeout_sec", 60.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        descriptor_path=_optional_str(source.get("descriptor_path")),
        context_length=int(source.get("context_length", 3)),
        image_height=int(source.get("image_height", 32)),
        image_width=int(source.get("image_width", 32)),
        timestep=float(source.get("timestep", 0.05)),
    )


def _sensor_runtime_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("isaac_sensor_runtime") or {})
    if "isaac_sensor_runtime" in config:
        return dict(config.get("isaac_sensor_runtime") or {})
    return dict(config)


def _load_descriptor(descriptor_path: str | None) -> dict:
    if not descriptor_path:
        return build_tiny_isaac_descriptor()
    path = Path(descriptor_path)
    if not path.exists():
        raise RuntimeError(f"Isaac sensor runtime descriptor path does not exist: {path}")
    if path.suffix.lower() != ".json":
        raise RuntimeError("Isaac sensor runtime descriptor must be a JSON descriptor.")
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


def _uses_injected_backend(
    runtime: IsaacSimRuntime | None,
    env: IsaacSimNavigationEnv | None,
) -> bool:
    if env is not None:
        return True
    if runtime is None:
        return False
    return getattr(runtime, "_backend", None) is not None


def _setup_instructions() -> list[str]:
    return [
        "Install NVIDIA Isaac Sim or Isaac Lab with its Python environment.",
        "Run this command from the Isaac-compatible Python environment.",
        "Set GWM_ALLOW_OPTIONAL_RUNTIME=1 and GWM_RUN_ISAAC_RUNTIME_TESTS=1.",
        "Use a JSON descriptor or the built-in tiny UAV descriptor.",
        "Do not enable real_hardware_enabled or autonomous_real_flight_enabled.",
    ]


def _descriptor_summary(descriptor: Mapping[str, Any]) -> dict:
    metadata = dict(descriptor.get("metadata", {}))
    sensors = descriptor.get("sensors", [])
    return {
        "schema_version": descriptor.get("schema_version"),
        "scenario_id": descriptor.get("scenario_id"),
        "has_vehicle": "vehicle" in descriptor,
        "has_goal": "goal" in descriptor,
        "sensor_count": len(sensors) if isinstance(sensors, list) else 0,
        "sensor_names": [
            str(sensor.get("name"))
            for sensor in sensors
            if isinstance(sensor, Mapping) and sensor.get("name") is not None
        ],
        "source_coordinate_frame": metadata.get("source_coordinate_frame"),
        "target_coordinate_frame": metadata.get("target_coordinate_frame"),
        "coordinate_conversion_applied": bool(
            metadata.get("coordinate_conversion_applied", False)
        ),
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
        "has_imu": "imu" in observation.metadata,
        "metadata": _json_safe(observation.metadata),
    }


def _buffer_summary(buffer: ObservationBuffer) -> dict:
    batch_summary: dict[str, Any] = {}
    if buffer.is_ready:
        batch = buffer.as_observation_batch()
        batch_summary = {
            "rgb_shape": list(batch.rgb.shape),
            "depth_shape": list(batch.depth.shape),
            "pose_shape": list(batch.pose.shape),
            "velocity_shape": list(batch.velocity.shape),
        }
    return {
        "context_length": int(buffer.context_length),
        "items": len(getattr(buffer, "_items", [])),
        "is_ready": bool(buffer.is_ready),
        "batch": batch_summary,
    }


def _shape(value: Any) -> list[int] | None:
    if value is None:
        return None
    return [int(item) for item in np.asarray(value).shape]


def _check_timeout(start: float, timeout_sec: float, phase: str) -> None:
    elapsed = time.perf_counter() - start
    if elapsed > float(timeout_sec):
        raise TimeoutError(
            f"Isaac sensor runtime timed out during {phase} after {elapsed:.2f}s"
        )


def _finalize_result(
    result: IsaacSensorRuntimeResult,
    config: IsaacSensorRuntimeConfig,
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


def _optional_str(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None
