"""Guarded Isaac Sim runtime connection for digital-twin descriptors.

The module is intentionally import-safe on machines without Isaac Sim. Real
Isaac Sim modules are imported only inside lifecycle helpers, and tests can
exercise the runtime through an injected fake backend.
"""

from __future__ import annotations

import copy
import importlib
import json
import logging
import time
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

import numpy as np

from src.utils.data_types import SensorObservation

logger = logging.getLogger(__name__)


_DEFAULT_CONFIG: Dict[str, Any] = {
    "enabled": False,
    "headless": True,
    "launch_mode": "mock_or_connect",
    "timestep": 0.05,
    "vehicle_path": "/World/Vehicle/UAV",
    "goal_path": "/World/Goal",
    "sensor_paths": {
        "rgb": "/World/Vehicle/UAV/Sensors/DepthCamera",
        "depth": "/World/Vehicle/UAV/Sensors/DepthCamera",
        "lidar": "/World/Vehicle/UAV/Sensors/Lidar",
        "imu": "/World/Vehicle/UAV/Sensors/Imu",
    },
}


class IsaacSimRuntime:
    """Optional Isaac Sim runtime wrapper.

    Parameters
    ----------
    config:
        Runtime configuration. Values override conservative defaults.
    backend:
        Optional injected backend used for tests or for connecting to an
        already-managed Isaac Sim application. Backend methods are duck typed:
        ``launch``, ``connect``, ``load_descriptor``, ``step``, ``read_sensors``,
        and ``close`` are called when present.
    """

    def __init__(
        self,
        config: Dict[str, Any] | None = None,
        backend: object | None = None,
    ) -> None:
        self.config = _merge_config(_DEFAULT_CONFIG, config or {})
        self._backend = backend
        self._external_backend = backend is not None
        self._app: Any | None = None
        self._owns_app = False
        self._connected = False
        self._descriptor: Dict[str, Any] | None = None
        self._last_snapshot: Dict[str, Any] | None = None
        self._last_action: np.ndarray = np.zeros(3, dtype=np.float32)
        self._step_count = 0

    @staticmethod
    def is_available() -> bool:
        """Return whether Isaac Sim's Python entrypoint appears importable."""
        return _load_simulation_app_class(raise_on_error=False) is not None

    @property
    def is_connected(self) -> bool:
        """Whether this runtime has launched or connected to a backend."""
        return self._connected

    @property
    def descriptor(self) -> Dict[str, Any] | None:
        """Loaded descriptor, if any."""
        return copy.deepcopy(self._descriptor)

    def launch(self, headless: bool | None = None) -> None:
        """Launch Isaac Sim or the injected backend.

        Without an injected backend, this requires Isaac Sim to be available.
        """
        launch_config = dict(self.config)
        launch_config["headless"] = self.config["headless"] if headless is None else bool(headless)

        if self._backend is not None:
            _call_if_present(self._backend, "launch", launch_config)
            self._connected = True
            return

        simulation_app_class = _load_simulation_app_class(raise_on_error=True)
        self._app = simulation_app_class({"headless": launch_config["headless"]})
        self._backend = _SimulationAppBackend(self._app)
        self._external_backend = False
        self._owns_app = True
        self._connected = True
        logger.info("Isaac Sim runtime launched (headless=%s)", launch_config["headless"])

    def connect(self) -> None:
        """Connect to an injected or already-launched backend."""
        if self._backend is None:
            if self._app is not None:
                self._connected = True
                return
            raise RuntimeError(
                "Isaac Sim runtime is not available. Inject a backend for tests "
                "or call launch() from an Isaac Sim Python environment."
            )

        _call_if_present(self._backend, "connect")
        self._connected = True

    def load_descriptor(self, descriptor: Dict[str, Any] | str) -> Dict[str, Any]:
        """Load a descriptor dict or descriptor path into the runtime."""
        normalized = self._normalize_descriptor(descriptor)
        self._descriptor = copy.deepcopy(normalized)

        if self._backend is not None:
            result = _call_if_present(self._backend, "load_descriptor", copy.deepcopy(normalized))
            if isinstance(result, Mapping):
                self._descriptor = copy.deepcopy(dict(result))

        return copy.deepcopy(self._descriptor)

    def step(self, action: Any = None, dt: float | None = None) -> Dict[str, Any]:
        """Advance the backend one step and return backend diagnostics."""
        self._require_connected()
        dt_value = float(self.config["timestep"] if dt is None else dt)
        if action is not None:
            self._last_action = _vector3(action, name="action")

        result: Any = None
        if self._backend is not None:
            result = _call_if_present(
                self._backend,
                "step",
                self._last_action.tolist(),
                dt_value,
            )
        self._step_count += 1

        diagnostics = dict(result) if isinstance(result, Mapping) else {}
        diagnostics.setdefault("step", self._step_count)
        diagnostics.setdefault("dt", dt_value)
        diagnostics.setdefault("action", self._last_action.tolist())
        return diagnostics

    def read_sensors(self) -> Dict[str, Any]:
        """Read and normalize the latest sensor snapshot."""
        self._require_connected()
        snapshot: Any = None
        if self._backend is not None:
            snapshot = _call_if_present(self._backend, "read_sensors")

        if isinstance(snapshot, Mapping):
            normalized = dict(snapshot)
        else:
            normalized = self._fallback_snapshot()

        normalized.setdefault("timestamp", self._step_count * float(self.config["timestep"]))
        normalized.setdefault("pose", self._descriptor_vehicle_position())
        normalized.setdefault("velocity", self._last_action.tolist())
        normalized.setdefault("metadata", {})
        normalized["metadata"] = self._metadata_with_runtime_fields(normalized["metadata"])
        self._last_snapshot = copy.deepcopy(normalized)
        return normalized

    def to_sensor_observation(self, snapshot: Dict[str, Any]) -> SensorObservation:
        """Convert an Isaac-style snapshot dict to ``SensorObservation``."""
        pose_value = snapshot.get("pose", self._descriptor_vehicle_position())
        velocity_value = snapshot.get("velocity", (0.0, 0.0, 0.0))
        pose = tuple(_vector3(pose_value, name="pose").tolist())
        velocity = tuple(_vector3(velocity_value, name="velocity").tolist())

        image = _optional_array(snapshot.get("rgb", snapshot.get("image")))
        depth = _optional_array(snapshot.get("depth"))
        if depth is not None and depth.ndim == 3 and depth.shape[-1] == 1:
            depth = depth[..., 0]
        lidar = _optional_lidar(snapshot.get("lidar"))

        goal_distance = snapshot.get("goal_distance")
        if goal_distance is None:
            goal_distance = self._compute_goal_distance(pose, snapshot)

        obstacle_distance = snapshot.get("obstacle_distance")
        if obstacle_distance is None:
            obstacle_distance = _estimate_obstacle_distance(depth, lidar)

        metadata = self._metadata_with_runtime_fields(snapshot.get("metadata", {}))
        if "imu" in snapshot:
            metadata["imu"] = snapshot["imu"]
        if "frame_ids" in snapshot:
            metadata["frame_ids"] = snapshot["frame_ids"]

        return SensorObservation(
            timestamp=float(snapshot.get("timestamp", time.time())),
            pose=pose,  # type: ignore[arg-type]
            velocity=velocity,  # type: ignore[arg-type]
            goal_distance=float(goal_distance),
            obstacle_distance=float(obstacle_distance),
            image=image,
            lidar=lidar,
            depth=depth,
            metadata=metadata,
        )

    def close(self) -> None:
        """Close backend resources owned by this runtime."""
        if self._backend is not None:
            _call_if_present(self._backend, "close")
        if self._owns_app and self._app is not None:
            close = getattr(self._app, "close", None)
            if callable(close):
                close()

        self._connected = False
        self._app = None
        if not self._external_backend:
            self._backend = None
        self._owns_app = False

    def _normalize_descriptor(self, descriptor: Dict[str, Any] | str) -> Dict[str, Any]:
        if isinstance(descriptor, Mapping):
            return copy.deepcopy(dict(descriptor))

        path = Path(descriptor)
        if not path.exists():
            raise RuntimeError(f"Isaac Sim descriptor path does not exist: {path}")

        if path.suffix.lower() == ".json":
            with open(path, "r", encoding="utf-8") as fh:
                return json.load(fh)

        if path.suffix.lower() in {".usd", ".usda", ".usdc"}:
            return {
                "schema_version": "gwm_openusd_stage_reference_v1",
                "backend": "isaac_sim_stage_reference",
                "stage_path": str(path),
                "metadata": {
                    "source_coordinate_frame": "project_default",
                    "target_coordinate_frame": "isaac_z_up_pending",
                    "coordinate_conversion_applied": False,
                    "stage_reference_only": True,
                },
            }

        raise RuntimeError(f"Unsupported Isaac Sim descriptor format: {path.suffix}")

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError(
                "Isaac Sim runtime is not connected. Call connect() with an "
                "injected backend or launch() from an Isaac Sim environment."
            )

    def _fallback_snapshot(self) -> Dict[str, Any]:
        return {
            "timestamp": self._step_count * float(self.config["timestep"]),
            "pose": self._descriptor_vehicle_position(),
            "velocity": self._last_action.tolist(),
            "goal": self._descriptor_goal_position(),
            "metadata": {
                "sensor_extraction": "metadata_only",
                "backend": "isaac_sim_runtime",
            },
        }

    def _descriptor_vehicle_position(self) -> list[float]:
        if self._descriptor:
            vehicle = self._descriptor.get("vehicle", {})
            position = vehicle.get("position")
            if position is not None:
                return _vector3(position, name="vehicle.position").tolist()
        return [0.0, 0.0, 0.0]

    def _descriptor_goal_position(self) -> list[float] | None:
        if self._descriptor:
            goal = self._descriptor.get("goal", {})
            position = goal.get("position")
            if position is not None:
                return _vector3(position, name="goal.position").tolist()
        return None

    def _compute_goal_distance(self, pose: Sequence[float], snapshot: Dict[str, Any]) -> float:
        goal = snapshot.get("goal", self._descriptor_goal_position())
        if goal is None:
            return 0.0
        return float(np.linalg.norm(_vector3(pose, name="pose") - _vector3(goal, name="goal")))

    def _metadata_with_runtime_fields(self, metadata: Any) -> Dict[str, Any]:
        result = dict(metadata) if isinstance(metadata, Mapping) else {}
        descriptor_metadata = {}
        if self._descriptor:
            descriptor_metadata = dict(self._descriptor.get("metadata", {}))

        result.setdefault("source", "isaac_sim")
        result.setdefault(
            "source_coordinate_frame",
            descriptor_metadata.get("source_coordinate_frame", "project_default"),
        )
        result.setdefault(
            "target_coordinate_frame",
            descriptor_metadata.get("target_coordinate_frame", "isaac_z_up_pending"),
        )
        result.setdefault(
            "coordinate_conversion_applied",
            bool(descriptor_metadata.get("coordinate_conversion_applied", False)),
        )
        result.setdefault("vehicle_path", self.config["vehicle_path"])
        result.setdefault("sensor_paths", copy.deepcopy(self.config["sensor_paths"]))
        return result


class _SimulationAppBackend:
    """Minimal backend around an owned Isaac Sim ``SimulationApp``."""

    def __init__(self, simulation_app: Any) -> None:
        self._simulation_app = simulation_app
        self._descriptor: Dict[str, Any] | None = None

    def load_descriptor(self, descriptor: Dict[str, Any]) -> Dict[str, Any]:
        self._descriptor = copy.deepcopy(descriptor)
        stage_path = descriptor.get("stage_path")
        if stage_path:
            self._open_stage(str(stage_path))
        return descriptor

    def step(self, action: Any = None, dt: float | None = None) -> Dict[str, Any]:
        update = getattr(self._simulation_app, "update", None)
        if callable(update):
            update()
        return {"action": action, "dt": dt, "backend": "isaac_sim"}

    def read_sensors(self) -> Dict[str, Any]:
        vehicle = {}
        goal = {}
        metadata = {"backend": "isaac_sim", "sensor_extraction": "metadata_only"}
        if self._descriptor:
            vehicle = self._descriptor.get("vehicle", {})
            goal = self._descriptor.get("goal", {})
            metadata.update(self._descriptor.get("metadata", {}))
        return {
            "pose": vehicle.get("position", [0.0, 0.0, 0.0]),
            "velocity": [0.0, 0.0, 0.0],
            "goal": goal.get("position"),
            "metadata": metadata,
        }

    def close(self) -> None:
        return None

    @staticmethod
    def _open_stage(stage_path: str) -> None:
        try:  # pragma: no cover - exercised only in Isaac Sim environments
            omni_usd = importlib.import_module("omni.usd")
            omni_usd.get_context().open_stage(stage_path)
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("Opening USD stages requires Isaac Sim's omni.usd module.") from exc


def _load_simulation_app_class(raise_on_error: bool) -> Any | None:
    errors: list[str] = []
    for module_name in ("isaacsim", "isaacsim.simulation_app"):
        try:
            module = importlib.import_module(module_name)
        except ImportError as exc:
            errors.append(str(exc))
            continue
        simulation_app = getattr(module, "SimulationApp", None)
        if simulation_app is not None:
            return simulation_app

    if raise_on_error:
        raise RuntimeError(
            "Isaac Sim SimulationApp is unavailable. Run from the Isaac Sim "
            "Python environment or inject a fake backend for tests. "
            f"Import errors: {'; '.join(errors)}"
        )
    return None


def _merge_config(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = _merge_config(dict(result[key]), dict(value))
        else:
            result[key] = copy.deepcopy(value)
    return result


def _call_if_present(target: object, method_name: str, *args: Any) -> Any:
    method = getattr(target, method_name, None)
    if callable(method):
        return method(*args)
    return None


def _vector3(value: Any, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32).reshape(-1)
    if array.size < 3:
        raise ValueError(f"{name} must contain at least 3 values")
    return array[:3]


def _optional_array(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value)


def _optional_lidar(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    lidar = np.asarray(value, dtype=np.float32)
    if lidar.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    return lidar.reshape(-1, 3)


def _estimate_obstacle_distance(depth: np.ndarray | None, lidar: np.ndarray | None) -> float:
    candidates: list[float] = []
    if depth is not None:
        finite_depth = depth[np.isfinite(depth)]
        finite_depth = finite_depth[finite_depth > 0.0]
        if finite_depth.size:
            candidates.append(float(np.min(finite_depth)))
    if lidar is not None and lidar.size:
        norms = np.linalg.norm(lidar.reshape(-1, 3), axis=1)
        if norms.size:
            candidates.append(float(np.min(norms)))
    if not candidates:
        return 50.0
    return float(np.clip(min(candidates), 0.2, 50.0))
