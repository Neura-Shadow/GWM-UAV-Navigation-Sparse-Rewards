"""Guarded Cosys-AirSim primary / legacy AirSim fallback runtime access.

The simulator backend registry key remains ``airsim``. At runtime this wrapper
prefers ``cosysairsim`` (Cosys-AirSim) and falls back to the legacy ``airsim``
package when needed. The module is import-safe without either package installed,
and tests can exercise it with injected fake clients.
"""

from __future__ import annotations

import importlib
import importlib.util
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from src.utils.data_types import SensorObservation

AIRSIM_BACKEND_REGISTRY_NAME = "airsim"
AIRSIM_PRIMARY_MODULE = "cosysairsim"
AIRSIM_PRIMARY_LABEL = "Cosys-AirSim"
AIRSIM_FALLBACK_MODULE = "airsim"
AIRSIM_FALLBACK_LABEL = "legacy AirSim"
AIRSIM_IMPORT_ORDER = (AIRSIM_PRIMARY_MODULE, AIRSIM_FALLBACK_MODULE)

_DEFAULT_CONFIG: Dict[str, Any] = {
    "backend_registry_name": AIRSIM_BACKEND_REGISTRY_NAME,
    "primary_runtime": AIRSIM_PRIMARY_MODULE,
    "primary_runtime_label": AIRSIM_PRIMARY_LABEL,
    "fallback_runtime": AIRSIM_FALLBACK_MODULE,
    "fallback_runtime_label": AIRSIM_FALLBACK_LABEL,
    "host": "127.0.0.1",
    "port": 41451,
    "vehicle_name": "",
    "lidar_name": "LidarSensor1",
    "rgb_camera_name": "0",
    "depth_camera_name": "0",
    "control_dt": 0.4,
    "target_altitude": -8.0,
    "goal": (60.0, 20.0, -8.0),
    "api_control_enabled": False,
    "arm_on_reset": False,
    "takeoff_on_reset": False,
    "reset_on_reset": False,
}


class AirSimRuntime:
    """Optional AirSim-family runtime wrapper.

    ``cosysairsim`` / Cosys-AirSim is the preferred implementation. The legacy
    ``airsim`` package is retained only as a fallback for older installations.

    Parameters
    ----------
    config:
        Runtime configuration. Values override conservative defaults.
    client:
        Optional injected AirSim-like client for tests or externally managed
        connections.
    airsim_module:
        Optional injected AirSim module shim for tests.
    """

    def __init__(
        self,
        config: Dict[str, Any] | None = None,
        client: object | None = None,
        airsim_module: object | None = None,
    ) -> None:
        self.config = {**_DEFAULT_CONFIG, **dict(config or {})}
        self._client = client
        self._external_client = client is not None
        self._airsim = airsim_module
        self._connected = False
        self._step_count = 0
        self._last_action = np.zeros(3, dtype=np.float32)
        self._last_snapshot: Dict[str, Any] | None = None

    @staticmethod
    def is_available() -> bool:
        """Return whether Cosys-AirSim or the legacy AirSim package is importable."""
        return _find_airsim_module_name() is not None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def client(self) -> object | None:
        return self._client

    def connect(self) -> None:
        """Connect to Cosys-AirSim, legacy AirSim, or an injected fake client."""
        if self._client is None:
            self._airsim = self._airsim or _load_airsim_module()
            kwargs: Dict[str, Any] = {"port": int(self.config["port"])}
            host = self.config.get("host")
            if host:
                kwargs["ip"] = str(host)
            self._client = getattr(self._airsim, "MultirotorClient")(**kwargs)

        _call_if_present(self._client, "confirmConnection")
        self._connected = True

    def reset(self) -> SensorObservation:
        """Reset runtime bookkeeping and return an observation.

        AirSim API control, arming, and takeoff are opt-in through config and
        are expected to be gated by the caller.
        """
        self._require_connected()
        vehicle = str(self.config.get("vehicle_name", ""))
        if bool(self.config.get("reset_on_reset", False)):
            _call_if_present(self._client, "reset")
        if bool(self.config.get("api_control_enabled", False)):
            _call_if_present(self._client, "enableApiControl", True, vehicle)
        if bool(self.config.get("arm_on_reset", False)):
            _call_if_present(self._client, "armDisarm", True, vehicle)
        if bool(self.config.get("takeoff_on_reset", False)):
            _join_if_needed(_call_if_present(self._client, "takeoffAsync", vehicle_name=vehicle))
        target_altitude = self.config.get("target_altitude")
        if bool(self.config.get("takeoff_on_reset", False)) and target_altitude is not None:
            _join_if_needed(
                _call_if_present(
                    self._client,
                    "moveToZAsync",
                    float(target_altitude),
                    2.5,
                    vehicle_name=vehicle,
                )
            )
        self._step_count = 0
        return self.to_sensor_observation(self.read_sensors())

    def step(self, action: Sequence[float], dt: float | None = None) -> Dict[str, Any]:
        """Send a velocity command to AirSim after API-control opt-in."""
        self._require_connected()
        if not bool(self.config.get("api_control_enabled", False)):
            raise RuntimeError(
                "AirSim API-control command refused. Set api_control_enabled=True "
                "only after Phase 7 AirSim gates are satisfied."
            )

        action_array = _vector3(action, name="action")
        self._last_action = action_array
        duration = float(self.config["control_dt"] if dt is None else dt)
        airsim_module = self._airsim
        drivetrain = None
        yaw_mode = None
        if airsim_module is not None:
            drivetrain_type = getattr(airsim_module, "DrivetrainType", None)
            drivetrain = getattr(drivetrain_type, "MaxDegreeOfFreedom", None)
            yaw_mode_cls = getattr(airsim_module, "YawMode", None)
            if callable(yaw_mode_cls):
                yaw_mode = yaw_mode_cls(is_rate=False, yaw_or_rate=0.0)

        kwargs = {
            "vx": float(action_array[0]),
            "vy": float(action_array[1]),
            "vz": float(action_array[2]),
            "duration": duration,
            "vehicle_name": str(self.config.get("vehicle_name", "")),
        }
        if drivetrain is not None:
            kwargs["drivetrain"] = drivetrain
        if yaw_mode is not None:
            kwargs["yaw_mode"] = yaw_mode
        _join_if_needed(_call_if_present(self._client, "moveByVelocityAsync", **kwargs))
        self._step_count += 1
        return {
            "step": self._step_count,
            "dt": duration,
            "action": action_array.tolist(),
            "command_sent": True,
            **self._runtime_metadata(),
        }

    def read_sensors(self) -> Dict[str, Any]:
        """Read an AirSim sensor snapshot as a normalized dict."""
        self._require_connected()
        vehicle = str(self.config.get("vehicle_name", ""))
        pose, velocity = self._read_kinematics(vehicle)
        depth = self._read_depth(vehicle)
        rgb = self._read_rgb(vehicle)
        lidar = self._read_lidar(vehicle)
        snapshot = {
            "timestamp": float(self._step_count * float(self.config["control_dt"])),
            "pose": pose,
            "velocity": velocity,
            "rgb": rgb,
            "depth": depth,
            "lidar": lidar,
            "metadata": {
                **self._runtime_metadata(),
                "source_frame": "airsim_ned",
                "target_frame": "project_default",
                "coordinate_conversion_applied": False,
                "vehicle_name": vehicle,
                "sensor_names": {
                    "lidar": self.config.get("lidar_name"),
                    "rgb": self.config.get("rgb_camera_name"),
                    "depth": self.config.get("depth_camera_name"),
                },
            },
        }
        self._last_snapshot = snapshot
        return snapshot

    def to_sensor_observation(self, snapshot: Mapping[str, Any]) -> SensorObservation:
        """Convert an AirSim snapshot dict into ``SensorObservation``."""
        pose = tuple(_vector3(snapshot.get("pose", (0.0, 0.0, 0.0)), name="pose").tolist())
        velocity = tuple(
            _vector3(snapshot.get("velocity", (0.0, 0.0, 0.0)), name="velocity").tolist()
        )
        goal = _vector3(self.config.get("goal", (60.0, 20.0, -8.0)), name="goal")
        pose_array = np.asarray(pose, dtype=np.float32)
        depth = _optional_array(snapshot.get("depth"))
        if depth is not None and depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[..., 0]
        lidar = _optional_lidar(snapshot.get("lidar"))
        obstacle_distance = snapshot.get("obstacle_distance")
        if obstacle_distance is None:
            obstacle_distance = _estimate_obstacle_distance(depth, lidar)
        return SensorObservation(
            timestamp=float(snapshot.get("timestamp", time.time())),
            pose=pose,  # type: ignore[arg-type]
            velocity=velocity,  # type: ignore[arg-type]
            goal_distance=float(snapshot.get("goal_distance", np.linalg.norm(pose_array - goal))),
            obstacle_distance=float(obstacle_distance),
            image=_optional_array(snapshot.get("rgb", snapshot.get("image"))),
            lidar=lidar,
            depth=depth,
            metadata={
                **dict(snapshot.get("metadata") or {}),
                **self._runtime_metadata(),
                "source_frame": "airsim_ned",
                "target_frame": "project_default",
                "coordinate_conversion_applied": False,
            },
        )

    def close(self) -> None:
        """Release API control when configured, then mark disconnected."""
        if self._client is not None and bool(self.config.get("api_control_enabled", False)):
            vehicle = str(self.config.get("vehicle_name", ""))
            _call_if_present(self._client, "hoverAsync", vehicle_name=vehicle)
            if bool(self.config.get("arm_on_reset", False)):
                _call_if_present(self._client, "armDisarm", False, vehicle)
            _call_if_present(self._client, "enableApiControl", False, vehicle)
        self._connected = False
        if not self._external_client:
            self._client = None

    def _read_kinematics(self, vehicle: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        state = _call_if_present(self._client, "getMultirotorState", vehicle_name=vehicle)
        kin = getattr(state, "kinematics_estimated", None)
        if kin is None:
            return (0.0, 0.0, float(self.config.get("target_altitude", -8.0))), (0.0, 0.0, 0.0)
        position = getattr(kin, "position", None)
        velocity = getattr(kin, "linear_velocity", None)
        return _xyz_tuple(position), _xyz_tuple(velocity)

    def _read_lidar(self, vehicle: str) -> np.ndarray | None:
        try:
            lidar = _call_if_present(
                self._client,
                "getLidarData",
                lidar_name=str(self.config.get("lidar_name", "LidarSensor1")),
                vehicle_name=vehicle,
            )
            points = getattr(lidar, "point_cloud", None)
            if points and len(points) >= 3:
                return np.asarray(points, dtype=np.float32).reshape(-1, 3)
        except Exception:
            return None
        return None

    def _read_depth(self, vehicle: str) -> np.ndarray | None:
        return self._read_image(vehicle, image_type_name="DepthPerspective", pixels_as_float=True)

    def _read_rgb(self, vehicle: str) -> np.ndarray | None:
        return self._read_image(vehicle, image_type_name="Scene", pixels_as_float=False)

    def _read_image(
        self,
        vehicle: str,
        *,
        image_type_name: str,
        pixels_as_float: bool,
    ) -> np.ndarray | None:
        if self._airsim is None:
            return None
        image_request = getattr(self._airsim, "ImageRequest", None)
        image_type = getattr(self._airsim, "ImageType", None)
        if image_request is None or image_type is None:
            return None
        type_value = getattr(image_type, image_type_name, None)
        if type_value is None:
            return None
        camera_name = (
            self.config.get("depth_camera_name")
            if pixels_as_float
            else self.config.get("rgb_camera_name")
        )
        try:
            responses = _call_if_present(
                self._client,
                "simGetImages",
                [image_request(str(camera_name), type_value, pixels_as_float, False)],
                vehicle_name=vehicle,
            )
        except Exception:
            return None
        if not responses:
            return None
        response = responses[0]
        width = int(getattr(response, "width", 0))
        height = int(getattr(response, "height", 0))
        if width <= 0 or height <= 0:
            return None
        if pixels_as_float:
            data = getattr(response, "image_data_float", None)
            if not data:
                return None
            return np.asarray(data, dtype=np.float32).reshape(height, width)
        data_uint8 = getattr(response, "image_data_uint8", None)
        if not data_uint8:
            return None
        array = np.frombuffer(bytes(data_uint8), dtype=np.uint8)
        if array.size < width * height * 3:
            return None
        return array[: width * height * 3].reshape(height, width, 3)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("AirSim runtime is not connected. Call connect() first.")

    def _runtime_metadata(self) -> Dict[str, Any]:
        selected_runtime = _selected_runtime_module(self._airsim)
        return {
            "backend": AIRSIM_BACKEND_REGISTRY_NAME,
            "backend_registry_name": AIRSIM_BACKEND_REGISTRY_NAME,
            "primary_runtime": AIRSIM_PRIMARY_MODULE,
            "primary_runtime_label": AIRSIM_PRIMARY_LABEL,
            "fallback_runtime": AIRSIM_FALLBACK_MODULE,
            "fallback_runtime_label": AIRSIM_FALLBACK_LABEL,
            "selected_runtime": selected_runtime,
            "selected_runtime_label": _runtime_label(selected_runtime),
        }


def _find_airsim_module_name() -> str | None:
    for name in AIRSIM_IMPORT_ORDER:
        if importlib.util.find_spec(name) is not None:
            return name
    return None


def _load_airsim_module() -> object:
    module_name = _find_airsim_module_name()
    if module_name is None:
        raise RuntimeError(
            "AirSim-family Python runtime is unavailable. Install Cosys-AirSim "
            "(cosysairsim) or the legacy AirSim fallback package (airsim), or "
            "inject a fake client for tests."
        )
    return importlib.import_module(module_name)


def _selected_runtime_module(airsim_module: object | None) -> str:
    if airsim_module is None:
        return "unresolved"
    module_name = getattr(airsim_module, "__name__", None)
    if module_name in AIRSIM_IMPORT_ORDER:
        return str(module_name)
    return "injected"


def _runtime_label(runtime_module: str) -> str:
    if runtime_module == AIRSIM_PRIMARY_MODULE:
        return AIRSIM_PRIMARY_LABEL
    if runtime_module == AIRSIM_FALLBACK_MODULE:
        return AIRSIM_FALLBACK_LABEL
    if runtime_module == "injected":
        return "injected AirSim-family client"
    return "unresolved AirSim-family runtime"


def _call_if_present(target: object | None, name: str, *args: Any, **kwargs: Any) -> Any:
    if target is None:
        return None
    method = getattr(target, name, None)
    if callable(method):
        return method(*args, **kwargs)
    return None


def _join_if_needed(value: Any) -> None:
    join = getattr(value, "join", None)
    if callable(join):
        join()


def _vector3(value: Any, *, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size < 3:
        raise ValueError(f"AirSim {name} requires at least three values.")
    return array[:3]


def _xyz_tuple(value: Any) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    return (
        float(getattr(value, "x_val", 0.0)),
        float(getattr(value, "y_val", 0.0)),
        float(getattr(value, "z_val", 0.0)),
    )


def _optional_array(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    array = np.asarray(value)
    return array if array.size else None


def _optional_lidar(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    array = np.asarray(value, dtype=np.float32)
    if array.size == 0:
        return None
    return array.reshape(-1, 3)


def _estimate_obstacle_distance(depth: np.ndarray | None, lidar: np.ndarray | None) -> float:
    candidates: list[float] = []
    if depth is not None and depth.size:
        finite = depth[np.isfinite(depth)]
        if finite.size:
            candidates.append(float(np.clip(np.min(finite), 0.2, 100.0)))
    if lidar is not None and lidar.size:
        distances = np.linalg.norm(lidar.reshape(-1, 3), axis=1)
        if distances.size:
            candidates.append(float(np.clip(np.min(distances), 0.2, 100.0)))
    return min(candidates) if candidates else 50.0
