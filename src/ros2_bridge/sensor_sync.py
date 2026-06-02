"""Mock-first ROS2 sensor synchronization for generated world model inputs."""

from __future__ import annotations

import importlib
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict

import numpy as np

from src.ros2_bridge.qos_config import QoSConfig, qos_from_config
from src.ros2_bridge.sensor_converters import (
    extract_frame_id,
    extract_timestamp,
    image_to_depth_array,
    image_to_rgb_array,
    imu_to_dict,
    lidar_to_array,
    odom_to_observation,
)
from src.utils.data_types import SensorObservation

logger = logging.getLogger(__name__)

_DEFAULT_TOPICS = {
    "rgb": "/camera/rgb",
    "depth": "/camera/depth",
    "lidar": "/scan",
    "odom": "/odom",
    "imu": "/imu",
}
_DEFAULT_REQUIRED_STREAMS = ("rgb", "depth", "lidar", "odom")


@dataclass(frozen=True)
class SensorSyncConfig:
    """Configuration for ROS2/manual sensor synchronization."""

    topics: Dict[str, str] = field(default_factory=lambda: dict(_DEFAULT_TOPICS))
    required_streams: tuple[str, ...] = _DEFAULT_REQUIRED_STREAMS
    slop_sec: float = 0.05
    queue_size: int = 10
    allow_headerless: bool = False
    qos: QoSConfig = field(default_factory=lambda: QoSConfig(reliability="best_effort"))
    real_mode: bool = False
    node_name: str = "gwm_sensor_sync"

    @classmethod
    def from_config(cls, config: Dict[str, Any] | None) -> "SensorSyncConfig":
        """Create config from None, ``ros2.sensor_sync``, or direct sync config."""
        if config is None:
            return cls()

        source = dict(config)
        if "ros2" in source:
            ros2_config = dict(source.get("ros2") or {})
            sync_config = dict(ros2_config.get("sensor_sync") or {})
            if "node_name" not in sync_config and "node_name" in ros2_config:
                sync_config["node_name"] = ros2_config["node_name"]
            source = sync_config
        elif "sensor_sync" in source:
            source = dict(source.get("sensor_sync") or {})

        topics = dict(_DEFAULT_TOPICS)
        topics.update(source.get("topics") or {})
        required = tuple(
            str(stream)
            for stream in source.get("required_streams", _DEFAULT_REQUIRED_STREAMS)
        )
        return cls(
            topics=topics,
            required_streams=required,
            slop_sec=float(source.get("slop_sec", 0.05)),
            queue_size=int(source.get("queue_size", 10)),
            allow_headerless=bool(source.get("allow_headerless", False)),
            qos=qos_from_config(source.get("qos")),
            real_mode=bool(source.get("real_mode", False)),
            node_name=str(source.get("node_name", "gwm_sensor_sync")),
        )


@dataclass
class SensorFrame:
    """One timestamped sensor message in a synchronization queue."""

    stream: str
    message: Any
    timestamp: float
    frame_id: str | None = None
    receipt_time: float | None = None
    timestamp_source: str = "header"


@dataclass
class SynchronizedSensorPacket:
    """Aligned sensor frames selected from per-stream queues."""

    frames: Dict[str, SensorFrame]
    timestamp: float
    slop_sec: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_sensor_observation(self) -> SensorObservation:
        """Convert packet frames into a single ``SensorObservation``."""
        odom_frame = self.frames["odom"]
        observation = odom_to_observation(odom_frame.message)
        observation.timestamp = float(self.timestamp)

        rgb_frame = self.frames.get("rgb")
        if rgb_frame is not None:
            observation.image = image_to_rgb_array(rgb_frame.message)

        depth_frame = self.frames.get("depth")
        if depth_frame is not None:
            observation.depth = image_to_depth_array(depth_frame.message)

        lidar_frame = self.frames.get("lidar")
        if lidar_frame is not None:
            observation.lidar = lidar_to_array(lidar_frame.message)

        obstacle_candidates = []
        if observation.depth is not None:
            valid_depth = observation.depth[np.isfinite(observation.depth)]
            valid_depth = valid_depth[valid_depth > 0.0]
            if valid_depth.size:
                obstacle_candidates.append(float(np.min(valid_depth)))
        if observation.lidar is not None and observation.lidar.size:
            obstacle_candidates.append(float(np.min(np.linalg.norm(observation.lidar, axis=1))))
        if obstacle_candidates:
            observation.obstacle_distance = float(np.clip(min(obstacle_candidates), 0.2, 50.0))

        metadata = dict(observation.metadata)
        metadata.update(self.metadata)
        metadata["stream_timestamps"] = {
            stream: frame.timestamp for stream, frame in self.frames.items()
        }
        metadata["frame_ids"] = {
            stream: frame.frame_id for stream, frame in self.frames.items() if frame.frame_id
        }
        metadata["timestamp_sources"] = {
            stream: frame.timestamp_source for stream, frame in self.frames.items()
        }
        metadata["sync_slop_sec"] = float(self.slop_sec)
        if "imu" in self.frames:
            metadata["imu"] = imu_to_dict(self.frames["imu"].message)
        observation.metadata = metadata
        return observation


class ROS2SensorSynchronizer:
    """Synchronize ROS2-style sensor streams into ``SensorObservation``."""

    def __init__(
        self,
        config: Dict[str, Any] | SensorSyncConfig | None = None,
        bridge: Any | None = None,
        observation_buffer: Any | None = None,
    ) -> None:
        if isinstance(config, SensorSyncConfig):
            self.config = config
        else:
            self.config = SensorSyncConfig.from_config(config)
        self.bridge = bridge
        self.observation_buffer = observation_buffer
        self._queues: Dict[str, Deque[SensorFrame]] = {
            stream: deque(maxlen=self.config.queue_size)
            for stream in set(self.config.required_streams).union(self.config.topics)
        }
        self._subscriptions: list[Any] = []
        self._time_synchronizer: Any | None = None
        self._latest_packet: SynchronizedSensorPacket | None = None
        self._started = False

    @staticmethod
    def is_available() -> bool:
        """Return whether ROS2 message filter modules appear importable."""
        return _load_ros2_sensor_modules(raise_on_error=False) is not None

    def start(self) -> None:
        """Start real ROS2 subscriptions when configured for real mode."""
        if not self.config.real_mode:
            self._started = True
            return

        modules = _load_ros2_sensor_modules(raise_on_error=True)
        if self.bridge is None:
            from src.ros2_bridge.ros2_bridge import ROS2Bridge

            self.bridge = ROS2Bridge(self.config.node_name)
        node = getattr(self.bridge, "node", self.bridge)
        subscribers = []
        for stream in self.config.required_streams:
            msg_type = modules["message_types"][stream]
            subscriber = modules["message_filters"].Subscriber(
                node,
                msg_type,
                self.config.topics[stream],
            )
            subscribers.append(subscriber)
            self._subscriptions.append(subscriber)

        synchronizer = modules["message_filters"].ApproximateTimeSynchronizer(
            subscribers,
            self.config.queue_size,
            self.config.slop_sec,
            allow_headerless=self.config.allow_headerless,
        )
        synchronizer.registerCallback(self._real_sync_callback)
        self._time_synchronizer = synchronizer
        self._started = True

    def ingest(
        self,
        stream: str,
        message: object,
        receipt_time: float | None = None,
    ) -> SynchronizedSensorPacket | None:
        """Add one message to a stream queue and attempt synchronization."""
        stream = str(stream)
        if stream not in self._queues:
            self._queues[stream] = deque(maxlen=self.config.queue_size)
        frame = self._frame_from_message(stream, message, receipt_time)
        self._queues[stream].append(frame)
        return self.try_sync()

    def try_sync(self) -> SynchronizedSensorPacket | None:
        """Return the newest packet with all required streams within slop."""
        if any(not self._queues.get(stream) for stream in self.config.required_streams):
            return None

        candidates = sorted(
            (
                frame
                for stream in self.config.required_streams
                for frame in self._queues.get(stream, [])
            ),
            key=lambda frame: frame.timestamp,
            reverse=True,
        )
        for anchor in candidates:
            selected = self._select_frames_near(anchor.timestamp)
            if selected is None:
                continue
            packet = self._packet_from_frames(selected)
            self._consume_selected_frames(selected)
            self._latest_packet = packet
            observation = packet.to_sensor_observation()
            if self.observation_buffer is not None:
                self.observation_buffer.append(observation)
            return packet
        return None

    def latest_observation(self) -> SensorObservation | None:
        """Return the latest synchronized observation, if any."""
        if self._latest_packet is None:
            packet = self.try_sync()
            if packet is None:
                return None
        return self._latest_packet.to_sensor_observation()

    def shutdown(self) -> None:
        """Release real ROS2 resources owned by the injected bridge."""
        self._subscriptions.clear()
        self._time_synchronizer = None
        if self.bridge is not None and hasattr(self.bridge, "shutdown"):
            self.bridge.shutdown()
        self._started = False

    def _frame_from_message(
        self,
        stream: str,
        message: object,
        receipt_time: float | None,
    ) -> SensorFrame:
        timestamp = extract_timestamp(message)
        timestamp_source = "header"
        if timestamp is None:
            if not self.config.allow_headerless:
                raise ValueError(
                    f"Stream '{stream}' message has no header.stamp; "
                    "set allow_headerless=True to use receipt time."
                )
            timestamp = time.time() if receipt_time is None else float(receipt_time)
            timestamp_source = "receipt_time"

        return SensorFrame(
            stream=stream,
            message=message,
            timestamp=float(timestamp),
            frame_id=extract_frame_id(message),
            receipt_time=receipt_time,
            timestamp_source=timestamp_source,
        )

    def _select_frames_near(self, anchor_timestamp: float) -> Dict[str, SensorFrame] | None:
        selected: Dict[str, SensorFrame] = {}
        for stream in self.config.required_streams:
            queue = list(self._queues.get(stream, []))
            if not queue:
                return None
            frame = min(queue, key=lambda item: abs(item.timestamp - anchor_timestamp))
            if abs(frame.timestamp - anchor_timestamp) > self.config.slop_sec:
                return None
            selected[stream] = frame

        timestamps = [frame.timestamp for frame in selected.values()]
        if max(timestamps) - min(timestamps) > self.config.slop_sec:
            return None

        for stream, queue in self._queues.items():
            if stream in selected or stream in self.config.required_streams:
                continue
            optional = [
                frame
                for frame in queue
                if abs(frame.timestamp - anchor_timestamp) <= self.config.slop_sec
            ]
            if optional:
                selected[stream] = max(optional, key=lambda frame: frame.timestamp)
        return selected

    def _packet_from_frames(self, frames: Dict[str, SensorFrame]) -> SynchronizedSensorPacket:
        timestamps = {stream: frame.timestamp for stream, frame in frames.items()}
        metadata = {
            "source": "ros2_sensor_sync",
            "required_streams": list(self.config.required_streams),
            "sync_window_sec": float(max(timestamps.values()) - min(timestamps.values())),
        }
        return SynchronizedSensorPacket(
            frames=dict(frames),
            timestamp=float(max(timestamps.values())),
            slop_sec=self.config.slop_sec,
            metadata=metadata,
        )

    def _consume_selected_frames(self, selected: Dict[str, SensorFrame]) -> None:
        for stream, selected_frame in selected.items():
            queue = self._queues.get(stream)
            if queue is None:
                continue
            self._queues[stream] = deque(
                (frame for frame in queue if frame is not selected_frame),
                maxlen=self.config.queue_size,
            )

    def _real_sync_callback(self, *messages: Any) -> None:
        streams = list(self.config.required_streams)
        frames = {
            stream: self._frame_from_message(stream, message, receipt_time=None)
            for stream, message in zip(streams, messages)
        }
        packet = self._packet_from_frames(frames)
        self._latest_packet = packet
        observation = packet.to_sensor_observation()
        if self.observation_buffer is not None:
            self.observation_buffer.append(observation)


def _load_ros2_sensor_modules(raise_on_error: bool) -> Dict[str, Any] | None:
    errors: list[str] = []
    try:
        message_filters = importlib.import_module("message_filters")
        sensor_msgs = importlib.import_module("sensor_msgs.msg")
        nav_msgs = importlib.import_module("nav_msgs.msg")
    except ImportError as exc:
        errors.append(str(exc))
        if raise_on_error:
            raise RuntimeError(
                "ROS2 sensor synchronization requires message_filters, sensor_msgs, "
                f"and nav_msgs. Import errors: {'; '.join(errors)}"
            ) from exc
        return None

    return {
        "message_filters": message_filters,
        "message_types": {
            "rgb": sensor_msgs.Image,
            "depth": sensor_msgs.Image,
            "lidar": getattr(sensor_msgs, "PointCloud2", sensor_msgs.LaserScan),
            "odom": nav_msgs.Odometry,
            "imu": sensor_msgs.Imu,
        },
    }
