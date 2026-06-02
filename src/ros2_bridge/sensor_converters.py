"""Pure-Python converters for ROS2-style sensor payloads.

The helpers accept dictionaries, SimpleNamespace-like objects, and real ROS2
messages when available. They avoid importing ROS2 packages so tests can run in
ordinary Python environments.
"""

from __future__ import annotations

import math
import struct
from typing import Any, Dict, Iterable, Mapping

import numpy as np

from src.ros2_bridge.msg_converters import odometry_dict_to_sensor_observation
from src.utils.data_types import SensorObservation


def extract_timestamp(message: Any) -> float | None:
    """Extract ``header.stamp`` or a direct timestamp from a ROS-like payload."""
    timestamp = _get(message, "timestamp")
    if timestamp is not None:
        return float(timestamp)

    stamp = _get(_get(message, "header"), "stamp")
    if stamp is None:
        return None

    if isinstance(stamp, (int, float)):
        return float(stamp)
    seconds = _get(stamp, "sec", 0.0)
    nanoseconds = _get(stamp, "nanosec", _get(stamp, "nsec", 0.0))
    return float(seconds) + float(nanoseconds) * 1e-9


def extract_frame_id(message: Any) -> str | None:
    """Extract ``header.frame_id`` from a ROS-like payload."""
    frame_id = _get(_get(message, "header"), "frame_id")
    return None if frame_id is None else str(frame_id)


def image_to_rgb_array(message: Any) -> np.ndarray:
    """Convert an Image-like payload to ``uint8`` RGB ``[H, W, 3]``."""
    if _has(message, "array"):
        array = np.asarray(_get(message, "array"))
    elif _has(message, "data") and _has(message, "height") and _has(message, "width"):
        height = int(_get(message, "height"))
        width = int(_get(message, "width"))
        encoding = str(_get(message, "encoding", "rgb8")).lower()
        data = _get(message, "data")
        if encoding in {"rgb8", "bgr8"}:
            array = np.frombuffer(_bytes(data), dtype=np.uint8).reshape(height, width, 3)
            if encoding == "bgr8":
                array = array[..., ::-1]
        elif encoding in {"mono8", "8uc1"}:
            mono = np.frombuffer(_bytes(data), dtype=np.uint8).reshape(height, width)
            array = np.repeat(mono[..., None], 3, axis=2)
        else:
            raise ValueError(f"Unsupported RGB image encoding: {encoding}")
    else:
        array = np.asarray(message)

    if array.ndim == 3 and array.shape[0] == 3:
        array = np.transpose(array, (1, 2, 0))
    if array.ndim == 2:
        array = np.repeat(array[..., None], 3, axis=2)
    if array.ndim != 3 or array.shape[-1] != 3:
        raise ValueError("RGB image must have shape [H, W, 3], [3, H, W], or [H, W].")
    if array.dtype != np.uint8:
        if array.max(initial=0.0) <= 1.0:
            array = array * 255.0
        array = np.clip(array, 0, 255).astype(np.uint8)
    return array


def image_to_depth_array(message: Any) -> np.ndarray:
    """Convert a depth Image-like payload to float32 ``[H, W]`` metres."""
    if _has(message, "array"):
        array = np.asarray(_get(message, "array"), dtype=np.float32)
    elif _has(message, "data") and _has(message, "height") and _has(message, "width"):
        height = int(_get(message, "height"))
        width = int(_get(message, "width"))
        encoding = str(_get(message, "encoding", "32FC1")).lower()
        data = _bytes(_get(message, "data"))
        if encoding in {"32fc1", "float32"}:
            array = np.frombuffer(data, dtype=np.float32).reshape(height, width)
        elif encoding in {"16uc1", "mono16"}:
            array = np.frombuffer(data, dtype=np.uint16).reshape(height, width).astype(np.float32)
            array = array / 1000.0
        else:
            raise ValueError(f"Unsupported depth image encoding: {encoding}")
    else:
        array = np.asarray(message, dtype=np.float32)

    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.ndim == 3 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 2:
        raise ValueError("Depth image must have shape [H, W], [H, W, 1], or [1, H, W].")
    return array.astype(np.float32)


def lidar_to_array(message: Any) -> np.ndarray:
    """Convert LaserScan-like or PointCloud2-like payloads to ``[N, 3]``."""
    if _has(message, "points"):
        return _points_to_array(_get(message, "points"))
    if _has(message, "ranges"):
        return _laser_scan_to_points(message)
    if _has(message, "fields") and _has(message, "data"):
        return _point_cloud2_to_points(message)
    return _points_to_array(message)


def odom_to_observation(message: Any) -> SensorObservation:
    """Convert Odometry-like payloads into ``SensorObservation``."""
    payload = _to_plain(message)
    return odometry_dict_to_sensor_observation(payload)


def imu_to_dict(message: Any) -> Dict[str, Any]:
    """Convert an IMU-like payload into a JSON-safe metadata dictionary."""
    payload = _to_plain(message)
    return {
        "orientation": payload.get("orientation"),
        "angular_velocity": payload.get("angular_velocity"),
        "linear_acceleration": payload.get("linear_acceleration"),
        "frame_id": extract_frame_id(message),
        "timestamp": extract_timestamp(message),
    }


def _laser_scan_to_points(message: Any) -> np.ndarray:
    ranges = np.asarray(_get(message, "ranges"), dtype=np.float32)
    angle_min = float(_get(message, "angle_min", 0.0))
    angle_increment = float(_get(message, "angle_increment", 0.0))
    points = []
    for index, range_value in enumerate(ranges):
        if not math.isfinite(float(range_value)) or float(range_value) <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        points.append([range_value * math.cos(angle), range_value * math.sin(angle), 0.0])
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _point_cloud2_to_points(message: Any) -> np.ndarray:
    fields = _get(message, "fields", [])
    offsets = {str(_get(field, "name")): int(_get(field, "offset", 0)) for field in fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise ValueError("PointCloud2-like payload must include x, y, and z fields.")

    point_step = int(_get(message, "point_step", 12))
    data = _bytes(_get(message, "data"))
    points = []
    for offset in range(0, len(data), point_step):
        chunk = data[offset : offset + point_step]
        if len(chunk) < point_step:
            continue
        point = [
            struct.unpack_from("<f", chunk, offsets["x"])[0],
            struct.unpack_from("<f", chunk, offsets["y"])[0],
            struct.unpack_from("<f", chunk, offsets["z"])[0],
        ]
        if all(math.isfinite(value) for value in point):
            points.append(point)
    if not points:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def _points_to_array(value: Any) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return array.reshape(-1, 3)


def _to_plain(value: Any) -> Dict[str, Any]:
    if isinstance(value, Mapping):
        return {key: _to_plain_value(val) for key, val in value.items()}
    return {
        key: _to_plain_value(getattr(value, key))
        for key in dir(value)
        if not key.startswith("_") and not callable(getattr(value, key))
    }


def _to_plain_value(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {key: _to_plain_value(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_plain_value(item) for item in value]
    if hasattr(value, "__dict__"):
        return _to_plain(value)
    return value


def _has(value: Any, key: str) -> bool:
    if isinstance(value, Mapping):
        return key in value
    return hasattr(value, key)


def _get(value: Any, key: str, default: Any = None) -> Any:
    if value is None:
        return default
    if isinstance(value, Mapping):
        return value.get(key, default)
    return getattr(value, key, default)


def _bytes(value: Any) -> bytes:
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, np.ndarray):
        return value.tobytes()
    if isinstance(value, Iterable):
        return bytes(value)
    raise TypeError("ROS image/point cloud data must be bytes-like.")
