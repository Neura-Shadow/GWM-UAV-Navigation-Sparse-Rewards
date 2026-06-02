"""Tests for Phase 4-D mock-first ROS2 sensor synchronization."""

from __future__ import annotations

import os
import struct
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import src.ros2_bridge.sensor_sync as sensor_sync_module
from src.generated_world_model.observation_buffer import ObservationBuffer
from src.ros2_bridge import ROS2SensorSynchronizer, SensorSyncConfig, SynchronizedSensorPacket
from src.ros2_bridge.sensor_converters import (
    extract_timestamp,
    image_to_depth_array,
    image_to_rgb_array,
    lidar_to_array,
)
from src.utils.data_types import SensorObservation


def _stamp(sec: int = 10, nanosec: int = 0) -> dict:
    return {"sec": sec, "nanosec": nanosec}


def _header(timestamp: float, frame_id: str) -> dict:
    seconds = int(timestamp)
    nanoseconds = int(round((timestamp - seconds) * 1_000_000_000))
    return {"stamp": _stamp(seconds, nanoseconds), "frame_id": frame_id}


def _rgb_msg(timestamp: float = 10.0) -> dict:
    image = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    return {
        "header": _header(timestamp, "camera_rgb_optical"),
        "height": 2,
        "width": 3,
        "encoding": "rgb8",
        "data": image.tobytes(),
    }


def _depth_msg(timestamp: float = 10.01) -> dict:
    depth = np.array([[1.5, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    return {
        "header": _header(timestamp, "camera_depth_optical"),
        "height": 2,
        "width": 3,
        "encoding": "32FC1",
        "data": depth.tobytes(),
    }


def _scan_msg(timestamp: float = 10.02) -> dict:
    return {
        "header": _header(timestamp, "lidar"),
        "angle_min": 0.0,
        "angle_increment": np.pi / 2,
        "ranges": [2.0, 3.0, float("inf")],
    }


def _odom_msg(timestamp: float = 10.03) -> dict:
    return {
        "header": _header(timestamp, "odom"),
        "child_frame_id": "base_link",
        "pose": {"pose": {"position": {"x": 1.0, "y": 2.0, "z": -3.0}}},
        "twist": {"twist": {"linear": {"x": 0.1, "y": 0.2, "z": 0.3}}},
        "goal_distance": 12.0,
        "obstacle_distance": 50.0,
        "metadata": {"source": "odom"},
    }


def _imu_msg(timestamp: float = 10.025) -> dict:
    return {
        "header": _header(timestamp, "imu"),
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.1},
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": -9.81},
    }


def test_package_exports_without_ros2() -> None:
    assert ROS2SensorSynchronizer is not None
    assert SensorSyncConfig is not None
    assert SynchronizedSensorPacket is not None


def test_config_parsing_defaults_and_nested_ros2_config() -> None:
    config = SensorSyncConfig.from_config(
        {
            "ros2": {
                "node_name": "custom_node",
                "sensor_sync": {
                    "topics": {"rgb": "/front/rgb"},
                    "required_streams": ["rgb", "depth", "odom"],
                    "slop_sec": 0.1,
                    "queue_size": 3,
                    "allow_headerless": True,
                    "qos": {"reliability": "BEST_EFFORT", "history_depth": 4},
                },
            }
        }
    )

    assert config.node_name == "custom_node"
    assert config.topics["rgb"] == "/front/rgb"
    assert config.topics["lidar"] == "/scan"
    assert config.required_streams == ("rgb", "depth", "odom")
    assert config.slop_sec == pytest.approx(0.1)
    assert config.queue_size == 3
    assert config.allow_headerless is True
    assert config.qos.reliability == "best_effort"


def test_timestamp_extraction_from_dict_and_object_headers() -> None:
    assert extract_timestamp({"header": {"stamp": {"sec": 3, "nanosec": 50}}}) == pytest.approx(
        3.00000005
    )
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=4, nanosec=100), frame_id="camera")
    )

    assert extract_timestamp(message) == pytest.approx(4.0000001)


def test_rgb_and_depth_conversion_shapes_and_dtypes() -> None:
    rgb = image_to_rgb_array(_rgb_msg())
    depth = image_to_depth_array(_depth_msg())

    assert rgb.shape == (2, 3, 3)
    assert rgb.dtype == np.uint8
    assert depth.shape == (2, 3)
    assert depth.dtype == np.float32
    assert depth[0, 0] == pytest.approx(1.5)


def test_laserscan_and_pointcloud2_like_conversion_to_lidar_arrays() -> None:
    scan = lidar_to_array(_scan_msg())
    pointcloud_data = b"".join(
        struct.pack("<fff", *point)
        for point in [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    )
    pointcloud = {
        "header": _header(10.0, "lidar"),
        "fields": [
            {"name": "x", "offset": 0},
            {"name": "y", "offset": 4},
            {"name": "z", "offset": 8},
        ],
        "point_step": 12,
        "data": pointcloud_data,
    }
    cloud = lidar_to_array(pointcloud)

    assert scan.shape == (2, 3)
    assert scan[0, 0] == pytest.approx(2.0)
    assert scan[1, 1] == pytest.approx(3.0)
    assert cloud.shape == (2, 3)
    assert cloud[1].tolist() == pytest.approx([4.0, 5.0, 6.0])


def test_exact_and_approximate_sync_returns_sensor_observation_with_metadata() -> None:
    synchronizer = ROS2SensorSynchronizer()

    assert synchronizer.ingest("rgb", _rgb_msg(10.0)) is None
    assert synchronizer.ingest("depth", _depth_msg(10.01)) is None
    assert synchronizer.ingest("lidar", _scan_msg(10.02)) is None
    synchronizer.ingest("imu", _imu_msg(10.025))
    packet = synchronizer.ingest("odom", _odom_msg(10.03))
    obs = synchronizer.latest_observation()

    assert packet is not None
    assert packet.metadata["source"] == "ros2_sensor_sync"
    assert isinstance(obs, SensorObservation)
    assert obs.pose == (1.0, 2.0, -3.0)
    assert obs.velocity == (0.1, 0.2, 0.3)
    assert obs.image.shape == (2, 3, 3)
    assert obs.depth.shape == (2, 3)
    assert obs.lidar.shape == (2, 3)
    assert obs.obstacle_distance == pytest.approx(1.5)
    assert obs.metadata["stream_timestamps"]["odom"] == pytest.approx(10.03)
    assert obs.metadata["frame_ids"]["rgb"] == "camera_rgb_optical"
    assert obs.metadata["timestamp_sources"]["rgb"] == "header"
    assert obs.metadata["sync_slop_sec"] == pytest.approx(0.05)
    assert obs.metadata["imu"]["frame_id"] == "imu"


def test_no_packet_when_required_stream_missing_or_outside_slop() -> None:
    missing = ROS2SensorSynchronizer()

    missing.ingest("rgb", _rgb_msg(10.0))
    missing.ingest("depth", _depth_msg(10.01))
    missing.ingest("odom", _odom_msg(10.02))
    assert missing.try_sync() is None

    outside = ROS2SensorSynchronizer()
    outside.ingest("rgb", _rgb_msg(10.0))
    outside.ingest("depth", _depth_msg(10.2))
    outside.ingest("lidar", _scan_msg(10.0))
    assert outside.ingest("odom", _odom_msg(10.0)) is None
    assert outside.latest_observation() is None


def test_headerless_messages_rejected_by_default_and_allowed_when_configured() -> None:
    synchronizer = ROS2SensorSynchronizer()

    with pytest.raises(ValueError, match="header.stamp"):
        synchronizer.ingest("rgb", {"array": np.zeros((2, 2, 3), dtype=np.uint8)})

    allowed = ROS2SensorSynchronizer({"allow_headerless": True})
    frame_packet = allowed.ingest(
        "rgb",
        {"array": np.zeros((2, 2, 3), dtype=np.uint8)},
        receipt_time=5.0,
    )

    assert frame_packet is None
    assert allowed._queues["rgb"][0].timestamp == pytest.approx(5.0)
    assert allowed._queues["rgb"][0].timestamp_source == "receipt_time"


def test_observation_buffer_integration() -> None:
    buffer = ObservationBuffer(context_length=1, image_size=(2, 3))
    synchronizer = ROS2SensorSynchronizer(observation_buffer=buffer)

    synchronizer.ingest("rgb", _rgb_msg(10.0))
    synchronizer.ingest("depth", _depth_msg(10.01))
    synchronizer.ingest("lidar", _scan_msg(10.02))
    synchronizer.ingest("odom", _odom_msg(10.03))

    assert buffer.is_ready is True
    batch = buffer.as_observation_batch()
    assert batch.rgb.shape == (1, 1, 3, 2, 3)
    assert batch.depth.shape == (1, 1, 1, 2, 3)


def test_real_mode_raises_when_ros2_sensor_modules_unavailable(monkeypatch) -> None:
    def _raise_import_error(_module_name):
        raise ImportError("not installed")

    monkeypatch.setattr(sensor_sync_module.importlib, "import_module", _raise_import_error)
    synchronizer = ROS2SensorSynchronizer(SensorSyncConfig(real_mode=True))

    with pytest.raises(RuntimeError, match="message_filters"):
        synchronizer.start()


def test_config_file_contains_safe_mock_first_sensor_sync_defaults() -> None:
    with open("configs/ros2_control.yaml", "r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    sync_config = config["ros2"]["sensor_sync"]
    assert sync_config["enabled"] is False
    assert sync_config["real_mode"] is False
    assert sync_config["required_streams"] == ["rgb", "depth", "lidar", "odom"]
    assert sync_config["allow_headerless"] is False


@pytest.mark.ros2_runtime
def test_optional_real_ros2_sensor_sync_smoke() -> None:
    if os.environ.get("GWM_RUN_ROS2_SENSOR_SYNC_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1 to run ROS2 sensor sync smoke tests.")
    if not ROS2SensorSynchronizer.is_available():
        pytest.skip("ROS2 sensor synchronization modules are not available.")

    synchronizer = ROS2SensorSynchronizer(SensorSyncConfig(real_mode=True))
    try:
        synchronizer.start()
    finally:
        synchronizer.shutdown()
