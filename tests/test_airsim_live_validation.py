"""Tests for Phase 7-B guarded live Cosys-AirSim validation."""

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
from src.runtime_validation import (
    AirSimLiveValidationConfig,
    run_airsim_live_validation,
)

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


_FAKE_COSYS_MODULE = SimpleNamespace(
    __name__="cosysairsim",
    ImageType=_FakeImageType,
    ImageRequest=_FakeImageRequest,
    DrivetrainType=SimpleNamespace(MaxDegreeOfFreedom="max"),
    YawMode=lambda **kwargs: SimpleNamespace(**kwargs),
)


class _FakeAirSimClient:
    def __init__(self, *, fail_kinematics: bool = False) -> None:
        self.connected = False
        self.api_control = False
        self.position = np.array([1.0, 2.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.commands: list[dict] = []
        self.fail_kinematics = fail_kinematics

    def confirmConnection(self) -> None:
        self.connected = True

    def enableApiControl(self, enabled: bool, vehicle_name: str = "") -> None:
        self.api_control = enabled
        self.commands.append({"action": "enableApiControl", "enabled": enabled})

    def hoverAsync(self, vehicle_name: str = "") -> _FakeFuture:
        self.commands.append({"action": "hover"})
        return _FakeFuture()

    def listVehicles(self) -> list[str]:
        return ["Drone1"]

    def getSettingsString(self) -> str:
        return '{"SettingsVersion": 1.2, "Vehicles": {}, "Sensors": {"LidarSensor1": {"SensorType": 6}}}'

    def getMultirotorState(self, vehicle_name: str = ""):
        if self.fail_kinematics:
            raise RuntimeError("kinematics unavailable")
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
        return SimpleNamespace(point_cloud=[3.0, 0.0, 0.0, 5.0, 1.0, 0.0])

    def simGetImages(self, requests, vehicle_name: str = ""):
        request = requests[0]
        if request.pixels_as_float:
            return [SimpleNamespace(width=2, height=2, image_data_float=[4.0, 5.0, 6.0, 7.0])]
        return [SimpleNamespace(width=2, height=2, image_data_uint8=bytes([3, 2, 1] * 4))]

    def moveByVelocityAsync(self, **kwargs) -> _FakeFuture:
        self.velocity = np.array([kwargs["vx"], kwargs["vy"], kwargs["vz"]], dtype=np.float32)
        self.commands.append({"action": "moveByVelocityAsync", "velocity": self.velocity.tolist()})
        return _FakeFuture()


def _fake_spec(name: str) -> object | None:
    if name in {"cosysairsim", "airsim"}:
        return SimpleNamespace(origin=f"/fake/{name}.py")
    return None


def _missing_spec(_: str) -> None:
    return None


def test_live_validation_imports_without_optional_runtime() -> None:
    assert AirSimLiveValidationConfig is not None
    assert run_airsim_live_validation is not None


def test_missing_gates_skip_without_connection(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_AIRSIM_RUNTIME_TESTS",
        "GWM_ALLOW_AIRSIM_API_CONTROL",
    ):
        monkeypatch.delenv(name, raising=False)

    result = run_airsim_live_validation({"write_output": False}, import_spec=_fake_spec)

    assert result["status"] == "skipped"
    assert result["connection_summary"]["connection_attempted"] is False
    assert "Missing required AirSim-family live validation env gates" in result["reason"]


def test_gated_unavailable_runtime_reports_runtime_unavailable(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")

    result = run_airsim_live_validation({"write_output": False}, import_spec=_missing_spec)

    assert result["status"] == "runtime_unavailable"
    assert result["runtime_selection"]["selected_module"] is None


def test_prefers_cosysairsim_over_legacy_airsim(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")

    runtime = AirSimRuntime(
        {"api_control_enabled": False},
        client=_FakeAirSimClient(),
        airsim_module=_FAKE_COSYS_MODULE,
    )
    result = run_airsim_live_validation(
        {"frames": 1, "write_output": False},
        runtime=runtime,
        import_spec=_fake_spec,
    )

    assert result["status"] == "passed"
    assert result["runtime_selection"]["selected_module"] == "cosysairsim"
    assert result["runtime_selection"]["selected_runtime_label"] == "Cosys-AirSim"
    assert result["sensor_observation_summary"]["metadata"]["backend_registry_name"] == "airsim"


def test_fake_client_validates_settings_sensors_and_observation(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")
    runtime = AirSimRuntime(
        {"api_control_enabled": False},
        client=_FakeAirSimClient(),
        airsim_module=_FAKE_COSYS_MODULE,
    )

    result = run_airsim_live_validation(
        {"frames": 2, "write_output": False},
        runtime=runtime,
        import_spec=_fake_spec,
    )

    assert result["status"] == "passed"
    assert result["vehicle_summary"]["selected_vehicle"] == "Drone1"
    assert result["settings_summary"]["has_sensors"] is True
    assert result["settings_summary"]["has_lidar"] is True
    assert result["sensor_summary"]["has_image"] is True
    assert result["sensor_summary"]["has_depth"] is True
    assert result["sensor_summary"]["has_lidar"] is True
    assert result["sensor_observation_summary"]["metadata"]["source_frame"] == "airsim_ned"
    assert result["sensor_observation_summary"]["metadata"]["coordinate_conversion_applied"] is False


def test_zero_command_validation_requires_api_gate(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")
    monkeypatch.delenv("GWM_ALLOW_AIRSIM_API_CONTROL", raising=False)
    runtime = AirSimRuntime(client=_FakeAirSimClient(), airsim_module=_FAKE_COSYS_MODULE)

    result = run_airsim_live_validation(
        {"validate_zero_command": True, "write_output": False},
        runtime=runtime,
        import_spec=_fake_spec,
    )

    assert result["status"] == "skipped"
    assert result["connection_summary"]["connection_attempted"] is False
    assert "GWM_ALLOW_AIRSIM_API_CONTROL" in result["reason"]


def test_zero_command_validation_sends_only_zero_velocity(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")
    monkeypatch.setenv("GWM_ALLOW_AIRSIM_API_CONTROL", "1")
    client = _FakeAirSimClient()
    runtime = AirSimRuntime(
        {"api_control_enabled": True},
        client=client,
        airsim_module=_FAKE_COSYS_MODULE,
    )

    result = run_airsim_live_validation(
        {
            "validate_zero_command": True,
            "api_control_enabled": True,
            "write_output": False,
        },
        runtime=runtime,
        import_spec=_fake_spec,
    )

    assert result["status"] == "passed"
    assert result["command_summary"]["commands_sent"] == 1
    assert {"action": "moveByVelocityAsync", "velocity": [0.0, 0.0, 0.0]} in client.commands
    assert client.api_control is False


def test_failure_after_connection_still_closes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")
    runtime = AirSimRuntime(
        {"api_control_enabled": False},
        client=_FakeAirSimClient(fail_kinematics=True),
        airsim_module=_FAKE_COSYS_MODULE,
    )

    result = run_airsim_live_validation(
        {"frames": 1, "write_output": False},
        runtime=runtime,
        import_spec=_fake_spec,
    )

    assert result["status"] == "failed"
    assert result["closed"] is True


def test_live_validation_writes_json_to_temp_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_AIRSIM_RUNTIME_TESTS", "1")
    output_path = tmp_path / "airsim_live_validation.json"
    runtime = AirSimRuntime(
        {"api_control_enabled": False},
        client=_FakeAirSimClient(),
        airsim_module=_FAKE_COSYS_MODULE,
    )

    result = run_airsim_live_validation(
        {"frames": 1, "output_path": str(output_path), "write_output": True},
        runtime=runtime,
        import_spec=_fake_spec,
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_phase7_airsim_live_validation_v1"


def test_live_validation_cli_no_gate_skips_and_writes_nothing(tmp_path: Path) -> None:
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
            str(_PROJECT_ROOT / "scripts" / "run_airsim_live_validation.py"),
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
    assert "airsim_live_validation status=skipped" in result.stdout
    assert output_path.exists() is False


def test_live_validation_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_airsim_live_validation.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Cosys-AirSim live validation" in result.stdout


@pytest.mark.airsim_runtime
def test_optional_live_validation_skips_without_runtime_gates(monkeypatch: pytest.MonkeyPatch) -> None:
    if not (
        os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") == "1"
        and os.environ.get("GWM_RUN_AIRSIM_RUNTIME_TESTS") == "1"
    ):
        pytest.skip("optional live AirSim-family validation gates are not enabled")
    monkeypatch.delenv("GWM_ALLOW_AIRSIM_API_CONTROL", raising=False)
    result = run_airsim_live_validation({"write_output": False}, import_spec=_missing_spec)
    assert result["status"] in {"runtime_unavailable", "connection_failed", "passed", "failed"}
