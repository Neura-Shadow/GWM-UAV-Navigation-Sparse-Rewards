"""Tests for guarded AirSim / CosysAirSim runtime integration."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from src.digital_twin import AirSimRuntime
from src.env import AirSimNavigationEnv
from src.runtime_validation import run_airsim_runtime_smoke

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class _FakeFuture:
    def join(self) -> None:
        return None


class _FakeImageType:
    Scene = 0
    DepthPerspective = 1


class _FakeImageRequest:
    def __init__(self, camera_name: str, image_type: int, pixels_as_float: bool, compress: bool = False) -> None:
        self.camera_name = camera_name
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class _FakeAirSimModule:
    ImageType = _FakeImageType
    ImageRequest = _FakeImageRequest
    DrivetrainType = SimpleNamespace(MaxDegreeOfFreedom="max")
    YawMode = lambda self=None, **kwargs: SimpleNamespace(**kwargs)


class _FakeAirSimClient:
    def __init__(self) -> None:
        self.connected = False
        self.api_control = False
        self.armed = False
        self.position = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.commands: list[dict] = []

    def confirmConnection(self) -> None:
        self.connected = True

    def enableApiControl(self, enabled: bool, vehicle_name: str = "") -> None:
        self.api_control = enabled
        self.commands.append({"action": "enableApiControl", "enabled": enabled})

    def armDisarm(self, enabled: bool, vehicle_name: str = "") -> None:
        self.armed = enabled
        self.commands.append({"action": "armDisarm", "enabled": enabled})

    def reset(self) -> None:
        self.position = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)

    def takeoffAsync(self, vehicle_name: str = "") -> _FakeFuture:
        self.commands.append({"action": "takeoff"})
        return _FakeFuture()

    def moveToZAsync(self, z: float, velocity: float, vehicle_name: str = "") -> _FakeFuture:
        self.position[2] = z
        return _FakeFuture()

    def moveByVelocityAsync(self, **kwargs) -> _FakeFuture:
        self.velocity = np.array([kwargs["vx"], kwargs["vy"], kwargs["vz"]], dtype=np.float32)
        self.position = self.position + self.velocity * float(kwargs["duration"])
        self.commands.append({"action": "moveByVelocityAsync", "velocity": self.velocity.tolist()})
        return _FakeFuture()

    def hoverAsync(self, vehicle_name: str = "") -> _FakeFuture:
        self.commands.append({"action": "hover"})
        return _FakeFuture()

    def getMultirotorState(self, vehicle_name: str = ""):
        position = SimpleNamespace(
            x_val=float(self.position[0]),
            y_val=float(self.position[1]),
            z_val=float(self.position[2]),
        )
        velocity = SimpleNamespace(
            x_val=float(self.velocity[0]),
            y_val=float(self.velocity[1]),
            z_val=float(self.velocity[2]),
        )
        return SimpleNamespace(
            kinematics_estimated=SimpleNamespace(
                position=position,
                linear_velocity=velocity,
            )
        )

    def getLidarData(self, lidar_name: str = "LidarSensor1", vehicle_name: str = ""):
        return SimpleNamespace(point_cloud=[4.0, 0.0, 0.0, 6.0, 1.0, 0.0])

    def simGetImages(self, requests, vehicle_name: str = ""):
        request = requests[0]
        if request.pixels_as_float:
            return [SimpleNamespace(width=2, height=2, image_data_float=[5.0, 6.0, 7.0, 8.0])]
        return [
            SimpleNamespace(
                width=2,
                height=2,
                image_data_uint8=bytes([0, 1, 2] * 4),
            )
        ]


def test_airsim_runtime_imports_without_airsim_installed() -> None:
    assert AirSimRuntime is not None
    assert AirSimNavigationEnv is not None


def test_airsim_runtime_fake_client_observation_and_step() -> None:
    client = _FakeAirSimClient()
    runtime = AirSimRuntime(
        {
            "api_control_enabled": True,
            "goal": [10.0, 0.0, -5.0],
            "control_dt": 0.2,
        },
        client=client,
        airsim_module=_FakeAirSimModule(),
    )

    runtime.connect()
    observation = runtime.reset()
    diagnostics = runtime.step([1.0, 0.0, 0.0], dt=0.2)
    snapshot = runtime.read_sensors()
    converted = runtime.to_sensor_observation(snapshot)
    runtime.close()

    assert observation.metadata["backend"] == "airsim"
    assert diagnostics["command_sent"] is True
    assert converted.image is not None
    assert converted.depth is not None
    assert converted.lidar is not None
    assert converted.metadata["source_frame"] == "airsim_ned"
    assert converted.metadata["coordinate_conversion_applied"] is False
    assert client.api_control is False


def test_airsim_runtime_refuses_commands_without_api_control() -> None:
    runtime = AirSimRuntime(client=_FakeAirSimClient())
    runtime.connect()

    with pytest.raises(RuntimeError, match="API-control command refused"):
        runtime.step([0.0, 0.0, 0.0])


def test_airsim_navigation_env_uses_injected_runtime() -> None:
    runtime = AirSimRuntime(
        {"api_control_enabled": True, "control_dt": 0.2},
        client=_FakeAirSimClient(),
        airsim_module=_FakeAirSimModule(),
    )
    env = AirSimNavigationEnv(runtime=runtime, config={"api_control_enabled": True})

    obs = env.reset()
    next_obs, reward, done, info = env.step(np.array([0.5, 0.0, 0.0], dtype=np.float32))
    env.close()

    assert obs.metadata["backend"] == "airsim"
    assert next_obs.velocity[0] == pytest.approx(0.5)
    assert isinstance(reward, float)
    assert done is False
    assert info["runtime"]["backend"] == "airsim"


def test_airsim_runtime_smoke_injected_fake_client_passes(tmp_path: Path) -> None:
    runtime = AirSimRuntime(
        {"api_control_enabled": True, "control_dt": 0.1},
        client=_FakeAirSimClient(),
        airsim_module=_FakeAirSimModule(),
    )
    output_path = tmp_path / "airsim_smoke.json"

    result = run_airsim_runtime_smoke(
        {
            "frames": 2,
            "output_path": str(output_path),
            "write_output": True,
            "api_control_enabled": True,
        },
        runtime=runtime,
    )

    assert result["status"] == "passed"
    assert result["frames_completed"] == 2
    assert output_path.exists()
    assert json.loads(output_path.read_text(encoding="utf-8"))["schema_version"]


def test_airsim_smoke_cli_no_gate_skips_and_writes_nothing(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    for name in (
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_AIRSIM_RUNTIME_TESTS",
        "GWM_ALLOW_AIRSIM_API_CONTROL",
    ):
        env.pop(name, None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_airsim_runtime_smoke.py"),
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "airsim_runtime_smoke status=skipped" in result.stdout
    assert output_path.exists() is False


def test_airsim_smoke_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_airsim_runtime_smoke.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "AirSim runtime smoke" in result.stdout
