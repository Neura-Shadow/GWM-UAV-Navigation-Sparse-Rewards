"""Guarded ROS2 simulation sensor bridge for Phase 6-C.

This module validates the simulation-only ROS2 sensor transport seam. Normal
tests inject fake publisher/synchronizer objects and exercise the full path:

``publish simulated sensor payloads -> ROS2SensorSynchronizer -> SensorObservation
-> ObservationBuffer -> JSON-safe report``.

Real ROS2 publishers/subscribers are attempted only when explicitly gated and
available. The runner never starts Nav2, MAVSDK, PX4, Isaac Sim, or hardware
interfaces.
"""

from __future__ import annotations

import copy
import json
import math
import os
import struct
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

import numpy as np

from src.generated_world_model import ObservationBuffer
from src.ros2_bridge import ROS2SensorSynchronizer, SensorSyncConfig
from src.runtime_validation.ros2_sensor_sync_smoke import build_mock_sensor_messages
from src.utils.data_types import SensorObservation

SCHEMA_VERSION = "gwm_phase6_ros2_sim_sensor_bridge_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/ros2_sim_sensor_bridge.json"
REQUIRED_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
)
OPTIONAL_ENV_GATES = ("GWM_ROS2_LIVE_TOPICS",)
_STREAM_ORDER = ("rgb", "depth", "lidar", "imu", "odom")


@dataclass
class ROS2SimSensorBridgeConfig:
    """Configuration for guarded Phase 6-C ROS2 simulation sensor bridging."""

    frames: int = 3
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    slop_sec: float = 0.05
    queue_size: int = 10
    context_length: int = 3
    image_height: int = 32
    image_width: int = 32
    node_name: str = "gwm_phase6_ros2_sim_sensor_bridge"
    publish_rate_hz: float = 20.0
    include_imu: bool = True
    use_live_topics: bool = False
    allow_headerless: bool = False


@dataclass
class ROS2SimSensorBridgeResult:
    """JSON-safe result for a Phase 6-C simulation sensor bridge attempt."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    setup_instructions: list
    safety_summary: dict
    topics: dict
    frames_requested: int
    frames_published: int
    packets_synchronized: int
    observations_collected: int
    publish_summary: dict
    sync_summary: dict
    sensor_observation_summary: dict
    observation_buffer_summary: dict
    timings: dict
    errors: list
    closed: bool


class ROS2SimulationSensorBridge:
    """Publish simulation-only sensor payloads and synchronize them."""

    def __init__(
        self,
        config: dict | ROS2SimSensorBridgeConfig | None = None,
        *,
        publisher_bridge: Any | None = None,
        synchronizer: ROS2SensorSynchronizer | None = None,
        observation_buffer: ObservationBuffer | None = None,
    ) -> None:
        self.config = _normalize_config(config)
        self.publisher_bridge = publisher_bridge
        self.synchronizer = synchronizer
        self.observation_buffer = observation_buffer
        self.publishers: dict[str, Any] = {}
        self.topic_map = _default_topic_map()
        self.messages_published: dict[str, int] = {stream: 0 for stream in _STREAM_ORDER}
        self._real_mode = publisher_bridge is None and synchronizer is None
        self._started = False
        self._closed = False

    @staticmethod
    def is_available() -> bool:
        """Return whether real ROS2 publisher/synchronizer modules are importable."""
        return bool(
            ROS2SensorSynchronizer.is_available()
            and _has_rclpy()
            and _load_ros2_message_types(raise_on_error=False) is not None
        )

    def start(self) -> None:
        """Start publishers and the synchronizer."""
        if self.observation_buffer is None:
            self.observation_buffer = ObservationBuffer(
                context_length=int(self.config.context_length),
                image_size=(int(self.config.image_height), int(self.config.image_width)),
            )

        if self.synchronizer is None:
            sync_config = SensorSyncConfig(
                topics=self.topic_map,
                required_streams=("rgb", "depth", "lidar", "odom"),
                slop_sec=float(self.config.slop_sec),
                queue_size=int(self.config.queue_size),
                allow_headerless=bool(self.config.allow_headerless),
                real_mode=bool(self._real_mode),
                node_name=str(self.config.node_name),
            )
            self.synchronizer = ROS2SensorSynchronizer(
                sync_config,
                bridge=self.publisher_bridge if self._real_mode else None,
                observation_buffer=self.observation_buffer,
            )

        if self._real_mode and self.publisher_bridge is None:
            from src.ros2_bridge.ros2_bridge import ROS2Bridge

            self.publisher_bridge = ROS2Bridge(str(self.config.node_name))
            self.synchronizer.bridge = self.publisher_bridge

        self.synchronizer.start()
        self._create_publishers()
        self._started = True

    def publish_simulated_frame(self, frame_index: int) -> tuple[Any | None, SensorObservation | None]:
        """Publish one simulated sensor frame and return synchronized output if any."""
        if not self._started:
            raise RuntimeError("ROS2SimulationSensorBridge.start() must be called first.")

        timestamp = 10.0 + (float(frame_index) / max(float(self.config.publish_rate_hz), 1.0))
        messages = build_mock_sensor_messages(timestamp=timestamp)
        if not self.config.include_imu:
            messages.pop("imu", None)

        packet = None
        for stream in _STREAM_ORDER:
            if stream not in messages:
                continue
            payload = messages[stream]
            publisher = self.publishers.get(stream)
            if publisher is not None:
                publisher.publish(self._publish_payload(stream, payload))
                self.messages_published[stream] = self.messages_published.get(stream, 0) + 1
            if self.synchronizer is not None and not self._sync_is_real_mode():
                maybe_packet = self.synchronizer.ingest(stream, payload)
                if maybe_packet is not None:
                    packet = maybe_packet

        if self._sync_is_real_mode():
            packet = self._wait_for_real_sync()
        elif self.synchronizer is not None:
            packet = packet or self.synchronizer.try_sync()

        observation = self.synchronizer.latest_observation() if self.synchronizer else None
        return packet, observation

    def close(self) -> None:
        """Release synchronizer and publisher bridge resources."""
        if self._closed:
            return
        if self.synchronizer is not None:
            self.synchronizer.shutdown()
        if (
            self.publisher_bridge is not None
            and getattr(self.synchronizer, "bridge", None) is not self.publisher_bridge
            and hasattr(self.publisher_bridge, "shutdown")
        ):
            self.publisher_bridge.shutdown()
        self._closed = True
        self._started = False

    def _create_publishers(self) -> None:
        if self.publisher_bridge is None:
            return
        message_types = _message_types_for_bridge(self._real_mode)
        for stream in _STREAM_ORDER:
            if stream == "imu" and not self.config.include_imu:
                continue
            if stream not in self.topic_map:
                continue
            self.publishers[stream] = self.publisher_bridge.create_publisher(
                self.topic_map[stream],
                message_types.get(stream, dict),
            )

    def _publish_payload(self, stream: str, payload: Mapping[str, Any]) -> Any:
        if not self._real_mode:
            return payload
        message_types = _message_types_for_bridge(real_mode=True)
        return _to_ros_message(stream, payload, message_types[stream])

    def _sync_is_real_mode(self) -> bool:
        return bool(getattr(getattr(self.synchronizer, "config", None), "real_mode", False))

    def _wait_for_real_sync(self) -> Any | None:
        deadline = time.perf_counter() + min(float(self.config.timeout_sec), 2.0)
        latest_packet = None
        while time.perf_counter() <= deadline:
            spin_once = getattr(self.publisher_bridge, "spin_once", None)
            if callable(spin_once):
                spin_once(timeout_sec=0.02)
            latest_packet = getattr(self.synchronizer, "_latest_packet", None)
            if latest_packet is not None:
                return latest_packet
            time.sleep(0.02)
        return latest_packet


def run_ros2_sim_sensor_bridge(
    config: dict | ROS2SimSensorBridgeConfig | None = None,
    *,
    bridge: Any | None = None,
    synchronizer: ROS2SensorSynchronizer | None = None,
    observation_buffer: ObservationBuffer | None = None,
) -> dict:
    """Run the guarded Phase 6-C bridge and return a JSON-safe result."""
    bridge_config = _normalize_config(config)
    frames_requested = max(1, int(bridge_config.frames))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []
    env_gates = _env_gate_status()
    injected_runtime = bridge is not None or synchronizer is not None

    result = ROS2SimSensorBridgeResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=env_gates,
        availability={
            "checked": False,
            "ros2_sim_sensor_bridge_available": None,
            "publisher_bridge_injected": bridge is not None,
            "synchronizer_injected": synchronizer is not None,
            "real_runtime_attempt": not injected_runtime,
        },
        setup_instructions=_setup_instructions(),
        safety_summary={
            "simulation_only": True,
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "isaac_launched": False,
            "mavsdk_connected": False,
            "px4_launched": False,
            "nav2_started": False,
            "hardware_check_run": False,
        },
        topics=_default_topic_map(),
        frames_requested=frames_requested,
        frames_published=0,
        packets_synchronized=0,
        observations_collected=0,
        publish_summary={},
        sync_summary={},
        sensor_observation_summary={},
        observation_buffer_summary={},
        timings=timings,
        errors=errors,
        closed=False,
    )

    if not injected_runtime and not _required_env_gates_satisfied(env_gates):
        missing = [
            name
            for name in REQUIRED_ENV_GATES
            if not bool(env_gates.get(name, {}).get("enabled", False))
        ]
        result.reason = f"Missing required ROS2 simulation sensor bridge env gates: {', '.join(missing)}"
        return _finalize_result(result, bridge_config, start)

    if not injected_runtime:
        result.availability["checked"] = True
        result.availability["ros2_sim_sensor_bridge_available"] = bool(
            ROS2SimulationSensorBridge.is_available()
        )
        if not result.availability["ros2_sim_sensor_bridge_available"]:
            result.status = "runtime_unavailable"
            result.reason = "ROS2 publisher/synchronizer runtime is unavailable."
            return _finalize_result(result, bridge_config, start)

    runtime_bridge: ROS2SimulationSensorBridge | None = None
    latest_packet = None
    latest_observation = None
    try:
        _check_timeout(start, bridge_config.timeout_sec, "initialization")
        runtime_bridge = ROS2SimulationSensorBridge(
            bridge_config,
            publisher_bridge=bridge,
            synchronizer=synchronizer,
            observation_buffer=observation_buffer,
        )
        _time_phase(timings, "start", runtime_bridge.start)
        result.topics = dict(runtime_bridge.topic_map)

        publish_started = time.perf_counter()
        for frame_index in range(frames_requested):
            _check_timeout(start, bridge_config.timeout_sec, "publishing")
            latest_packet, latest_observation = runtime_bridge.publish_simulated_frame(frame_index)
            result.frames_published += 1
            if latest_packet is not None and latest_observation is not None:
                result.packets_synchronized += 1
                result.observations_collected += 1
        timings["publish_loop_sec"] = round(time.perf_counter() - publish_started, 6)

        result.publish_summary = _publish_summary(runtime_bridge)
        if latest_packet is not None:
            result.sync_summary = _packet_summary(latest_packet)
        if latest_observation is not None:
            result.sensor_observation_summary = _sensor_observation_summary(latest_observation)
        if runtime_bridge.observation_buffer is not None:
            result.observation_buffer_summary = _buffer_summary(runtime_bridge.observation_buffer)

        if result.packets_synchronized <= 0:
            result.status = "failed"
            result.reason = "Simulation sensor messages were published but did not synchronize."
        else:
            result.status = "passed"
            result.reason = None
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
    finally:
        if runtime_bridge is not None:
            try:
                runtime_bridge.close()
                result.closed = True
            except Exception as exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": exc.__class__.__name__,
                        "message": f"close failed: {exc}",
                    }
                )

    return _finalize_result(result, bridge_config, start)


def _normalize_config(
    config: dict | ROS2SimSensorBridgeConfig | None,
) -> ROS2SimSensorBridgeConfig:
    if isinstance(config, ROS2SimSensorBridgeConfig):
        return copy.deepcopy(config)

    source = _bridge_config_section(config or {})
    return ROS2SimSensorBridgeConfig(
        frames=int(source.get("frames", 3)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        slop_sec=float(source.get("slop_sec", 0.05)),
        queue_size=int(source.get("queue_size", 10)),
        context_length=int(source.get("context_length", 3)),
        image_height=int(source.get("image_height", 32)),
        image_width=int(source.get("image_width", 32)),
        node_name=str(source.get("node_name", "gwm_phase6_ros2_sim_sensor_bridge")),
        publish_rate_hz=float(source.get("publish_rate_hz", 20.0)),
        include_imu=bool(source.get("include_imu", True)),
        use_live_topics=bool(source.get("use_live_topics", False)),
        allow_headerless=bool(source.get("allow_headerless", False)),
    )


def _bridge_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("ros2_sim_sensor_bridge") or {})
    if "ros2_sim_sensor_bridge" in config:
        return dict(config.get("ros2_sim_sensor_bridge") or {})
    return dict(config)


def _env_gate_status() -> dict:
    names = REQUIRED_ENV_GATES + OPTIONAL_ENV_GATES
    return {
        name: {
            "present": name in os.environ,
            "enabled": os.environ.get(name) == "1",
            "required": name in REQUIRED_ENV_GATES,
        }
        for name in names
    }


def _required_env_gates_satisfied(env_gates: Mapping[str, Mapping[str, Any]]) -> bool:
    return all(bool(env_gates.get(name, {}).get("enabled", False)) for name in REQUIRED_ENV_GATES)


def _default_topic_map() -> dict[str, str]:
    return {
        "rgb": "/camera/rgb",
        "depth": "/camera/depth",
        "lidar": "/scan",
        "odom": "/odom",
        "imu": "/imu",
    }


def _setup_instructions() -> list[str]:
    return [
        "Install and source ROS2 so rclpy, message_filters, sensor_msgs, and nav_msgs are importable.",
        "Set GWM_ALLOW_OPTIONAL_RUNTIME=1 and GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1.",
        "Use GWM_ROS2_LIVE_TOPICS=1 only for operator-approved live simulation topics.",
        "Run only simulation topics; do not connect to Nav2, PX4, MAVSDK, or hardware here.",
    ]


def _has_rclpy() -> bool:
    try:
        from src.ros2_bridge import ros2_bridge as ros2_bridge_module

        return bool(getattr(ros2_bridge_module, "_HAS_RCLPY", False))
    except Exception:
        return False


def _load_ros2_message_types(raise_on_error: bool) -> dict[str, Any] | None:
    try:
        sensor_msgs = __import__(
            "sensor_msgs.msg",
            fromlist=["Image", "Imu", "LaserScan", "PointCloud2", "PointField"],
        )
        nav_msgs = __import__("nav_msgs.msg", fromlist=["Odometry"])
    except ImportError as exc:
        if raise_on_error:
            raise RuntimeError(
                "ROS2 simulation sensor bridge requires sensor_msgs and nav_msgs. "
                f"Import error: {exc}"
            ) from exc
        return None
    return {
        "rgb": sensor_msgs.Image,
        "depth": sensor_msgs.Image,
        "lidar": getattr(sensor_msgs, "PointCloud2", sensor_msgs.LaserScan),
        "odom": nav_msgs.Odometry,
        "imu": sensor_msgs.Imu,
    }


def _message_types_for_bridge(real_mode: bool) -> dict[str, Any]:
    if not real_mode:
        return {stream: dict for stream in _STREAM_ORDER}
    message_types = _load_ros2_message_types(raise_on_error=True)
    assert message_types is not None
    return message_types


def _to_ros_message(stream: str, payload: Mapping[str, Any], msg_type: Any) -> Any:
    message = msg_type()
    _set_header(message, payload.get("header") or {})
    if stream in {"rgb", "depth"}:
        message.height = int(payload.get("height", 0))
        message.width = int(payload.get("width", 0))
        message.encoding = str(payload.get("encoding", "rgb8" if stream == "rgb" else "32FC1"))
        message.data = bytes(payload.get("data", b""))
        if hasattr(message, "step"):
            channels = 3 if stream == "rgb" else 4
            message.step = int(message.width * channels)
    elif stream == "lidar":
        if message.__class__.__name__ == "PointCloud2":
            _fill_point_cloud2_message(message, payload)
        else:
            message.angle_min = float(payload.get("angle_min", 0.0))
            message.angle_increment = float(payload.get("angle_increment", 0.0))
            message.ranges = [float(value) for value in payload.get("ranges", [])]
    elif stream == "odom":
        if hasattr(message, "child_frame_id"):
            message.child_frame_id = str(payload.get("child_frame_id", "base_link"))
        position = (
            payload.get("pose", {})
            .get("pose", {})
            .get("position", {})
        )
        linear = (
            payload.get("twist", {})
            .get("twist", {})
            .get("linear", {})
        )
        message.pose.pose.position.x = float(position.get("x", 0.0))
        message.pose.pose.position.y = float(position.get("y", 0.0))
        message.pose.pose.position.z = float(position.get("z", 0.0))
        message.twist.twist.linear.x = float(linear.get("x", 0.0))
        message.twist.twist.linear.y = float(linear.get("y", 0.0))
        message.twist.twist.linear.z = float(linear.get("z", 0.0))
    elif stream == "imu":
        orientation = payload.get("orientation", {})
        angular_velocity = payload.get("angular_velocity", {})
        linear_acceleration = payload.get("linear_acceleration", {})
        message.orientation.x = float(orientation.get("x", 0.0))
        message.orientation.y = float(orientation.get("y", 0.0))
        message.orientation.z = float(orientation.get("z", 0.0))
        message.orientation.w = float(orientation.get("w", 1.0))
        message.angular_velocity.x = float(angular_velocity.get("x", 0.0))
        message.angular_velocity.y = float(angular_velocity.get("y", 0.0))
        message.angular_velocity.z = float(angular_velocity.get("z", 0.0))
        message.linear_acceleration.x = float(linear_acceleration.get("x", 0.0))
        message.linear_acceleration.y = float(linear_acceleration.get("y", 0.0))
        message.linear_acceleration.z = float(linear_acceleration.get("z", 0.0))
    return message


def _fill_point_cloud2_message(message: Any, payload: Mapping[str, Any]) -> None:
    from sensor_msgs.msg import PointField  # type: ignore[import-not-found]

    points = _lidar_payload_to_points(payload)
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * len(points)
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *point) for point in points)


def _lidar_payload_to_points(payload: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    if "points" in payload:
        array = np.asarray(payload["points"], dtype=np.float32).reshape(-1, 3)
        return [tuple(float(value) for value in row) for row in array]

    ranges = payload.get("ranges", [])
    angle_min = float(payload.get("angle_min", 0.0))
    angle_increment = float(payload.get("angle_increment", 0.0))
    points: list[tuple[float, float, float]] = []
    for index, range_value in enumerate(ranges):
        value = float(range_value)
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        points.append((value * math.cos(angle), value * math.sin(angle), 0.0))
    return points


def _set_header(message: Any, header: Mapping[str, Any]) -> None:
    if not hasattr(message, "header"):
        return
    stamp = header.get("stamp", {})
    message.header.frame_id = str(header.get("frame_id", ""))
    message.header.stamp.sec = int(stamp.get("sec", 0))
    message.header.stamp.nanosec = int(stamp.get("nanosec", stamp.get("nsec", 0)))


def _time_phase(timings: dict, name: str, callback: Any) -> Any:
    start = time.perf_counter()
    try:
        return callback()
    finally:
        timings[f"{name}_sec"] = round(time.perf_counter() - start, 6)


def _check_timeout(start: float, timeout_sec: float, phase: str) -> None:
    elapsed = time.perf_counter() - start
    if elapsed > float(timeout_sec):
        raise TimeoutError(
            f"ROS2 simulation sensor bridge timed out during {phase} after {elapsed:.2f}s"
        )


def _publish_summary(bridge: ROS2SimulationSensorBridge) -> dict:
    return {
        "topics": dict(bridge.topic_map),
        "publisher_count": len(bridge.publishers),
        "messages_published": {
            stream: int(count)
            for stream, count in bridge.messages_published.items()
            if int(count) > 0
        },
        "real_mode": bool(bridge._real_mode),
    }


def _packet_summary(packet: Any) -> dict:
    frames = getattr(packet, "frames", {})
    metadata = getattr(packet, "metadata", {})
    return {
        "timestamp": _json_safe(getattr(packet, "timestamp", None)),
        "slop_sec": _json_safe(getattr(packet, "slop_sec", None)),
        "streams": sorted(str(stream) for stream in frames.keys()),
        "frame_count": len(frames),
        "sync_window_sec": _json_safe(metadata.get("sync_window_sec")),
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


def _finalize_result(
    result: ROS2SimSensorBridgeResult,
    config: ROS2SimSensorBridgeConfig,
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
