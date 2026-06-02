"""Tests for Phase 4-C guarded Isaac Sim runtime interfaces."""

from __future__ import annotations

import json
import os

import numpy as np
import pytest
import yaml

from src.digital_twin import IsaacSimRuntime, IsaacSimSceneBuilder
from src.env import BaseNavigationEnv, IsaacSimNavigationEnv
from src.utils.data_types import ScenarioSpec, SensorObservation


class FakeIsaacBackend:
    def __init__(self) -> None:
        self.launched = False
        self.connected = False
        self.closed = False
        self.loaded_descriptor = None
        self.step_history = []
        self.pose = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)

    def launch(self, config):
        self.launched = True
        self.connected = True
        self.launch_config = config

    def connect(self):
        self.connected = True

    def load_descriptor(self, descriptor):
        self.loaded_descriptor = descriptor
        self.pose = np.asarray(descriptor["vehicle"]["position"], dtype=np.float32)
        return descriptor

    def step(self, action, dt):
        action_array = np.asarray(action, dtype=np.float32)
        self.velocity = action_array
        self.pose = self.pose + action_array * float(dt)
        self.step_history.append({"action": action_array.tolist(), "dt": dt})
        return {"backend": "fake", "step_count": len(self.step_history)}

    def read_sensors(self):
        return {
            "timestamp": len(self.step_history) * 0.05,
            "pose": self.pose.tolist(),
            "velocity": self.velocity.tolist(),
            "goal": [10.0, 0.0, -5.0],
            "rgb": np.zeros((4, 5, 3), dtype=np.uint8),
            "depth": np.ones((4, 5), dtype=np.float32) * 7.5,
            "lidar": np.array([[3.0, 0.0, 0.0], [0.0, 4.0, 0.0]], dtype=np.float32),
            "imu": {"linear_acceleration": [0.0, 0.0, 0.0]},
            "metadata": {"backend": "fake"},
        }

    def close(self):
        self.closed = True
        self.connected = False


def _scenario() -> ScenarioSpec:
    return ScenarioSpec(
        scenario_id="phase_4c_runtime",
        description="Guarded Isaac runtime test",
        start_position=(0.0, 0.0, -5.0),
        goal_position=(10.0, 0.0, -5.0),
        obstacles=[{"type": "sphere", "position": [5.0, 0.0, -5.0], "radius": 1.0}],
    )


def _descriptor():
    return IsaacSimSceneBuilder().build(_scenario())


def test_package_imports_without_isaac_sim() -> None:
    assert IsaacSimRuntime is not None
    assert IsaacSimNavigationEnv is not None
    assert issubclass(IsaacSimNavigationEnv, BaseNavigationEnv)


def test_runtime_availability_check_is_safe() -> None:
    assert isinstance(IsaacSimRuntime.is_available(), bool)


def test_runtime_raises_clear_error_without_backend_or_launch() -> None:
    runtime = IsaacSimRuntime()

    with pytest.raises(RuntimeError, match="not available|not connected"):
        runtime.connect()

    with pytest.raises(RuntimeError, match="not connected"):
        runtime.read_sensors()


def test_runtime_fake_backend_lifecycle_and_descriptor_loading(tmp_path) -> None:
    backend = FakeIsaacBackend()
    runtime = IsaacSimRuntime(config={"headless": True}, backend=backend)
    descriptor = _descriptor()
    descriptor_path = tmp_path / "descriptor.json"
    descriptor_path.write_text(json.dumps(descriptor), encoding="utf-8")

    runtime.launch()
    loaded = runtime.load_descriptor(str(descriptor_path))
    diagnostics = runtime.step([1.0, 0.0, 0.0], dt=0.1)
    snapshot = runtime.read_sensors()
    runtime.close()

    assert backend.launched is True
    assert backend.closed is True
    assert loaded["scenario_id"] == "phase_4c_runtime"
    assert diagnostics["backend"] == "fake"
    assert snapshot["metadata"]["source_coordinate_frame"] == "project_default"
    assert snapshot["metadata"]["target_coordinate_frame"] == "isaac_z_up_pending"
    assert snapshot["metadata"]["coordinate_conversion_applied"] is False


def test_sensor_snapshot_converts_to_sensor_observation() -> None:
    runtime = IsaacSimRuntime(backend=FakeIsaacBackend())
    runtime.connect()
    runtime.load_descriptor(_descriptor())
    snapshot = runtime.read_sensors()

    obs = runtime.to_sensor_observation(snapshot)

    assert isinstance(obs, SensorObservation)
    assert obs.image.shape == (4, 5, 3)
    assert obs.depth.shape == (4, 5)
    assert obs.lidar.shape == (2, 3)
    assert obs.obstacle_distance == pytest.approx(3.0)
    assert obs.goal_distance == pytest.approx(10.0)
    assert obs.metadata["imu"]["linear_acceleration"] == [0.0, 0.0, 0.0]


def test_runtime_loads_usd_stage_reference_without_opening_stage(tmp_path) -> None:
    stage_path = tmp_path / "scene.usda"
    stage_path.write_text("#usda 1.0\n", encoding="utf-8")
    runtime = IsaacSimRuntime()

    descriptor = runtime.load_descriptor(str(stage_path))

    assert descriptor["schema_version"] == "gwm_openusd_stage_reference_v1"
    assert descriptor["stage_path"] == str(stage_path)
    assert descriptor["metadata"]["coordinate_conversion_applied"] is False


def test_navigation_env_matches_base_contract() -> None:
    backend = FakeIsaacBackend()
    runtime = IsaacSimRuntime(backend=backend)
    env = IsaacSimNavigationEnv(
        descriptor=_descriptor(),
        runtime=runtime,
        config={"control_dt": 0.1},
    )

    obs0 = env.reset()
    obs1, reward, done, info = env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    state = env.get_state_vector()
    env.close()

    assert isinstance(obs0, SensorObservation)
    assert isinstance(obs1, SensorObservation)
    assert state.shape == (8,)
    assert reward < 0.0
    assert done is False
    assert info["runtime"]["backend"] == "fake"
    assert backend.closed is True


def test_navigation_env_rejects_invalid_action_shape() -> None:
    env = IsaacSimNavigationEnv(
        descriptor=_descriptor(),
        runtime=IsaacSimRuntime(backend=FakeIsaacBackend()),
    )
    env.reset()

    with pytest.raises(ValueError, match="shape"):
        env.step(np.array([1.0, 2.0], dtype=np.float32))


def test_digital_twin_config_keeps_runtime_disabled_by_default() -> None:
    with open("configs/digital_twin.yaml", "r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    runtime_config = config["isaac_runtime"]
    assert runtime_config["enabled"] is False
    assert runtime_config["headless"] is True
    assert runtime_config["launch_mode"] == "mock_or_connect"
    assert runtime_config["coordinate_frames"]["coordinate_conversion_applied"] is False


@pytest.mark.isaac_runtime
def test_optional_real_isaac_runtime_smoke() -> None:
    if os.environ.get("GWM_RUN_ISAAC_RUNTIME_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ISAAC_RUNTIME_TESTS=1 to run Isaac Sim smoke tests.")
    if not IsaacSimRuntime.is_available():
        pytest.skip("Isaac Sim Python runtime is not available.")

    runtime = IsaacSimRuntime(config={"enabled": True, "headless": True})
    try:
        runtime.launch(headless=True)
        runtime.load_descriptor(_descriptor())
        runtime.step([0.0, 0.0, 0.0], dt=0.05)
        obs = runtime.to_sensor_observation(runtime.read_sensors())
        assert isinstance(obs, SensorObservation)
    finally:
        runtime.close()
