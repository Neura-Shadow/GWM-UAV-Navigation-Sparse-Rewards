"""Guarded ROS2 sensor synchronization smoke test for Phase 5-C.

The smoke runner is mock-first. It uses synthetic ROS-like dictionaries for
normal tests and will not start real ROS2 subscriptions unless both optional
runtime gates are set.
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

from src.ros2_bridge import ROS2SensorSynchronizer, SensorSyncConfig
from src.utils.data_types import SensorObservation

SCHEMA_VERSION = "gwm_ros2_sensor_sync_smoke_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/ros2_sensor_sync_smoke.json"
REQUIRED_ENV_GATES = (
    "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
    "GWM_ALLOW_OPTIONAL_RUNTIME",
)


@dataclass
class ROS2SensorSyncSmokeConfig:
    """Configuration for the guarded ROS2 sensor sync smoke test."""

    packets: int = 1
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    slop_sec: float = 0.05
    queue_size: int = 10


@dataclass
class ROS2SensorSyncSmokeResult:
    """JSON-safe result for a ROS2 sensor sync smoke attempt."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    streams_required: list
    packets_requested: int
    packets_completed: int
    sync_summary: dict
    sensor_metadata: dict
    sensor_observation_summary: dict
    timings: dict
    errors: list
    closed: bool


def build_mock_sensor_messages(timestamp: float = 10.0) -> dict:
    """Build synchronized ROS-like sensor payloads for smoke tests."""
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    depth = np.array([[1.5, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    return {
        "rgb": {
            "header": _header(timestamp, "camera_rgb_optical"),
            "height": 2,
            "width": 3,
            "encoding": "rgb8",
            "data": rgb.tobytes(),
        },
        "depth": {
            "header": _header(timestamp + 0.01, "camera_depth_optical"),
            "height": 2,
            "width": 3,
            "encoding": "32FC1",
            "data": depth.tobytes(),
        },
        "lidar": {
            "header": _header(timestamp + 0.02, "lidar"),
            "angle_min": 0.0,
            "angle_increment": float(np.pi / 2.0),
            "ranges": [2.0, 3.0, float("inf")],
        },
        "odom": {
            "header": _header(timestamp + 0.03, "odom"),
            "child_frame_id": "base_link",
            "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": -3.0}}},
            "twist": {"twist": {"linear": {"x": 0.1, "y": 0.2, "z": 0.3}}},
            "goal_distance": 12.0,
            "obstacle_distance": 50.0,
            "metadata": {"source": "odom"},
        },
        "imu": {
            "header": _header(timestamp + 0.025, "imu"),
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
            "linear_acceleration": {"x": 0.0, "y": 0.0, "z": -9.81},
        },
    }


def run_ros2_sensor_sync_smoke(
    config: dict | ROS2SensorSyncSmokeConfig | None = None,
    synchronizer: Any = None,
) -> dict:
    """Run a guarded ROS2 sensor sync smoke test and return a JSON-safe dict."""
    smoke_config = _normalize_config(config)
    packets_requested = max(1, int(smoke_config.packets))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []

    result = ROS2SensorSyncSmokeResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=_env_gate_status(),
        availability={
            "checked": False,
            "ros2_sensor_sync_available": None,
            "synchronizer_injected": synchronizer is not None,
            "real_mode": synchronizer is None,
        },
        streams_required=["rgb", "depth", "lidar", "odom"],
        packets_requested=packets_requested,
        packets_completed=0,
        sync_summary={},
        sensor_metadata={},
        sensor_observation_summary={},
        timings=timings,
        errors=errors,
        closed=False,
    )

    requires_real_runtime_gate = synchronizer is None
    smoke_synchronizer = synchronizer

    if requires_real_runtime_gate and not _env_gates_satisfied(result.env_gates):
        missing = [
            name
            for name, gate in result.env_gates.items()
            if not bool(gate.get("enabled", False))
        ]
        result.reason = f"Missing required ROS2 sensor sync env gates: {', '.join(missing)}"
        return _finalize_result(result, smoke_config, start)

    if requires_real_runtime_gate:
        result.availability["checked"] = True
        result.availability["ros2_sensor_sync_available"] = bool(ROS2SensorSynchronizer.is_available())
        if not result.availability["ros2_sensor_sync_available"]:
            result.reason = "ROS2 sensor synchronization modules are unavailable."
            return _finalize_result(result, smoke_config, start)

    try:
        _check_timeout(start, smoke_config.timeout_sec, "initialization")
        if smoke_synchronizer is None:
            smoke_synchronizer = ROS2SensorSynchronizer(
                SensorSyncConfig(
                    real_mode=True,
                    slop_sec=float(smoke_config.slop_sec),
                    queue_size=int(smoke_config.queue_size),
                )
            )
            _time_phase(timings, "start", smoke_synchronizer.start)
            packet, observation, completed = _wait_for_real_packets(
                smoke_synchronizer,
                packets_requested,
                start,
                smoke_config.timeout_sec,
            )
            result.packets_completed = completed
            if packet is None or observation is None or completed <= 0:
                result.status = "skipped"
                result.reason = "ROS2 sensor sync started, but no synchronized packet was received."
            else:
                result.sync_summary = _packet_summary(packet)
                result.sensor_metadata = _packet_metadata_summary(packet)
                result.sensor_observation_summary = _sensor_observation_summary(observation)
                result.status = "passed"
                result.reason = None

        else:
            result.streams_required = list(getattr(smoke_synchronizer.config, "required_streams", []))
            latest_packet = None
            latest_observation = None
            for index in range(packets_requested):
                _check_timeout(start, smoke_config.timeout_sec, "manual ingest")
                messages = build_mock_sensor_messages(timestamp=10.0 + index)
                latest_packet = _ingest_mock_messages(smoke_synchronizer, messages)
                latest_observation = smoke_synchronizer.latest_observation()
                if latest_packet is None or latest_observation is None:
                    raise RuntimeError("Synthetic ROS2 sensor messages did not synchronize.")
                result.packets_completed += 1

            result.sync_summary = _packet_summary(latest_packet)
            result.sensor_metadata = _packet_metadata_summary(latest_packet)
            result.sensor_observation_summary = _sensor_observation_summary(latest_observation)
            result.status = "passed"
            result.reason = None
    except Exception as exc:  # pragma: no cover - tested through fake synchronizer failure
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    finally:
        if smoke_synchronizer is not None:
            try:
                smoke_synchronizer.shutdown()
                result.closed = True
            except Exception as exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": exc.__class__.__name__,
                        "message": f"shutdown failed: {exc}",
                    }
                )

    return _finalize_result(result, smoke_config, start)


def _normalize_config(
    config: dict | ROS2SensorSyncSmokeConfig | None,
) -> ROS2SensorSyncSmokeConfig:
    if isinstance(config, ROS2SensorSyncSmokeConfig):
        return copy.deepcopy(config)

    source = _smoke_config_section(config or {})
    return ROS2SensorSyncSmokeConfig(
        packets=int(source.get("packets", 1)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        slop_sec=float(source.get("slop_sec", 0.05)),
        queue_size=int(source.get("queue_size", 10)),
    )


def _smoke_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("ros2_sensor_sync_smoke") or {})
    if "ros2_sensor_sync_smoke" in config:
        return dict(config.get("ros2_sensor_sync_smoke") or {})
    return dict(config)


def _ingest_mock_messages(synchronizer: Any, messages: Mapping[str, Any]) -> Any:
    packet = None
    for stream in ("rgb", "depth", "lidar", "imu", "odom"):
        if stream in messages:
            maybe_packet = synchronizer.ingest(stream, messages[stream])
            if maybe_packet is not None:
                packet = maybe_packet
    return packet or synchronizer.try_sync()


def _wait_for_real_packets(
    synchronizer: Any,
    packets_requested: int,
    start: float,
    timeout_sec: float,
) -> tuple[Any | None, SensorObservation | None, int]:
    latest_packet = None
    latest_observation = None
    completed = 0
    last_packet_id = None
    while time.perf_counter() - start <= float(timeout_sec):
        bridge = getattr(synchronizer, "bridge", None)
        spin_once = getattr(bridge, "spin_once", None)
        if callable(spin_once):
            spin_once(timeout_sec=0.05)
        latest_packet = getattr(synchronizer, "_latest_packet", None) or synchronizer.try_sync()
        latest_observation = synchronizer.latest_observation()
        if latest_packet is not None and latest_observation is not None:
            packet_id = id(latest_packet)
            if packet_id != last_packet_id:
                completed += 1
                last_packet_id = packet_id
            if completed >= packets_requested:
                break
        time.sleep(0.05)
    return latest_packet, latest_observation, completed


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


def _time_phase(timings: dict, name: str, callback: Any) -> Any:
    start = time.perf_counter()
    try:
        return callback()
    finally:
        timings[f"{name}_sec"] = round(time.perf_counter() - start, 6)


def _check_timeout(start: float, timeout_sec: float, phase: str) -> None:
    elapsed = time.perf_counter() - start
    if elapsed > float(timeout_sec):
        raise TimeoutError(f"ROS2 sensor sync smoke timed out during {phase} after {elapsed:.2f}s")


def _packet_summary(packet: Any) -> dict:
    frames = getattr(packet, "frames", {})
    return {
        "timestamp": _json_safe(getattr(packet, "timestamp", None)),
        "slop_sec": _json_safe(getattr(packet, "slop_sec", None)),
        "streams": sorted(str(stream) for stream in frames.keys()),
        "frame_count": len(frames),
        "sync_window_sec": _json_safe(getattr(packet, "metadata", {}).get("sync_window_sec")),
    }


def _packet_metadata_summary(packet: Any) -> dict:
    observation = packet.to_sensor_observation()
    metadata = observation.metadata
    return {
        "stream_timestamps": _json_safe(metadata.get("stream_timestamps", {})),
        "frame_ids": _json_safe(metadata.get("frame_ids", {})),
        "timestamp_sources": _json_safe(metadata.get("timestamp_sources", {})),
        "sync_slop_sec": _json_safe(metadata.get("sync_slop_sec")),
        "has_imu": "imu" in metadata,
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
    result: ROS2SensorSyncSmokeResult,
    config: ROS2SensorSyncSmokeConfig,
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


def _header(timestamp: float, frame_id: str) -> dict:
    seconds = int(timestamp)
    nanoseconds = int(round((timestamp - seconds) * 1_000_000_000))
    return {"stamp": {"sec": seconds, "nanosec": nanoseconds}, "frame_id": frame_id}
