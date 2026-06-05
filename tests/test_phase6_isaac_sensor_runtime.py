"""Tests for Phase 6-B guarded Isaac sensor runtime execution."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

from src.digital_twin import IsaacSimRuntime
from src.generated_world_model import ObservationBuffer
from src.runtime_validation import (
    IsaacSensorRuntimeConfig,
    IsaacSensorRuntimeResult,
    run_isaac_sensor_runtime,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeIsaacSensorBackend:
    def __init__(self, *, fail_on_step: bool = False) -> None:
        self.fail_on_step = fail_on_step
        self.launched = False
        self.closed = False
        self.loaded_descriptor = None
        self.step_history: list[dict] = []
        self.pose = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)

    def launch(self, config: dict) -> None:
        self.launched = True
        self.launch_config = config

    def load_descriptor(self, descriptor: dict) -> dict:
        self.loaded_descriptor = descriptor
        self.pose = np.asarray(descriptor["vehicle"]["position"], dtype=np.float32)
        return descriptor

    def step(self, action, dt: float) -> dict:
        if self.fail_on_step:
            raise RuntimeError("fake phase6 isaac step failure")
        action_array = np.asarray(action, dtype=np.float32)
        self.velocity = action_array
        self.pose = self.pose + action_array * float(dt)
        self.step_history.append({"action": action_array.tolist(), "dt": float(dt)})
        return {"backend": "fake_phase6_isaac", "step_count": len(self.step_history)}

    def read_sensors(self) -> dict:
        return {
            "timestamp": len(self.step_history) * 0.05,
            "pose": self.pose.tolist(),
            "velocity": self.velocity.tolist(),
            "goal": [2.0, 0.0, -5.0],
            "rgb": np.zeros((4, 5, 3), dtype=np.uint8),
            "depth": np.ones((4, 5), dtype=np.float32) * 6.0,
            "lidar": np.array([[3.0, 0.0, 0.0], [0.0, 4.0, 0.0]], dtype=np.float32),
            "imu": {"linear_acceleration": [0.0, 0.0, 0.0]},
            "metadata": {"backend": "fake_phase6_isaac"},
        }

    def close(self) -> None:
        self.closed = True


def _clear_isaac_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_RUN_ISAAC_RUNTIME_TESTS", raising=False)


def test_phase6_isaac_sensor_runtime_exports_without_isaac() -> None:
    assert IsaacSensorRuntimeConfig is not None
    assert IsaacSensorRuntimeResult is not None
    assert run_isaac_sensor_runtime is not None


def test_missing_env_gates_skip_without_runtime_probe(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    monkeypatch.setattr(
        IsaacSimRuntime,
        "is_available",
        staticmethod(lambda: (_ for _ in ()).throw(AssertionError("availability checked"))),
    )

    result = run_isaac_sensor_runtime({"write_output": False})

    assert result["schema_version"] == "gwm_phase6_isaac_sensor_runtime_v1"
    assert result["status"] == "skipped"
    assert "Missing required Isaac sensor runtime env gates" in result["reason"]
    assert result["frames_completed"] == 0
    assert result["availability"]["checked"] is False
    assert result["closed"] is False


def test_gated_unavailable_runtime_reports_setup_instructions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_ISAAC_RUNTIME_TESTS", "1")
    monkeypatch.setattr(IsaacSimRuntime, "is_available", staticmethod(lambda: False))

    result = run_isaac_sensor_runtime({"write_output": False, "fail_on_unavailable": True})

    assert result["status"] == "runtime_unavailable"
    assert result["availability"]["checked"] is True
    assert result["availability"]["isaac_sim_available"] is False
    assert "Isaac Sim / Isaac Lab Python runtime is unavailable" in result["reason"]
    assert any("Install NVIDIA Isaac Sim or Isaac Lab" in item for item in result["setup_instructions"])
    assert result["frames_completed"] == 0


def test_fake_backend_executes_navigation_env_and_observation_buffer(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_isaac_env(monkeypatch)
    backend = FakeIsaacSensorBackend()
    runtime = IsaacSimRuntime(backend=backend)

    result = run_isaac_sensor_runtime(
        IsaacSensorRuntimeConfig(frames=3, context_length=3, write_output=False),
        runtime=runtime,
    )

    buffer = result["observation_buffer_summary"]
    summary = result["sensor_summary"]
    assert result["status"] == "passed"
    assert result["frames_completed"] == 3
    assert result["observations_collected"] == 4
    assert result["availability"]["runtime_injected"] is True
    assert result["availability"]["injected_backend"] is True
    assert result["execution_summary"]["used_isaac_sim_navigation_env"] is True
    assert result["execution_summary"]["used_observation_buffer"] is True
    assert buffer["is_ready"] is True
    assert buffer["items"] == 3
    assert buffer["batch"]["rgb_shape"] == [1, 3, 3, 32, 32]
    assert buffer["batch"]["depth_shape"] == [1, 3, 1, 32, 32]
    assert summary["has_image"] is True
    assert summary["has_depth"] is True
    assert summary["has_lidar"] is True
    assert summary["has_imu"] is True
    assert backend.launched is True
    assert backend.closed is True


def test_descriptor_metadata_preserved_in_phase6_sensor_runtime(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_isaac_env(monkeypatch)
    runtime = IsaacSimRuntime(backend=FakeIsaacSensorBackend())

    result = run_isaac_sensor_runtime(
        {"frames": 1, "write_output": False},
        runtime=runtime,
    )

    descriptor = result["descriptor_summary"]
    assert descriptor["source_coordinate_frame"] == "project_default"
    assert descriptor["target_coordinate_frame"] == "isaac_z_up_pending"
    assert descriptor["coordinate_conversion_applied"] is False
    assert descriptor["sensor_names"] == ["DepthCamera", "Lidar", "Imu"]


def test_injected_observation_buffer_is_filled(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    runtime = IsaacSimRuntime(backend=FakeIsaacSensorBackend())
    buffer = ObservationBuffer(context_length=2, image_size=(16, 16))

    result = run_isaac_sensor_runtime(
        IsaacSensorRuntimeConfig(frames=1, context_length=2, image_height=16, image_width=16, write_output=False),
        runtime=runtime,
        observation_buffer=buffer,
    )

    assert result["status"] == "passed"
    assert buffer.is_ready is True
    assert result["observation_buffer_summary"]["batch"]["rgb_shape"] == [1, 2, 3, 16, 16]


def test_runtime_failure_closes_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    backend = FakeIsaacSensorBackend(fail_on_step=True)
    runtime = IsaacSimRuntime(backend=backend)

    result = run_isaac_sensor_runtime(
        IsaacSensorRuntimeConfig(frames=2, write_output=False),
        runtime=runtime,
    )

    assert result["status"] == "failed"
    assert "fake phase6 isaac step failure" in result["reason"]
    assert result["closed"] is True
    assert backend.closed is True
    assert result["errors"][0]["type"] == "RuntimeError"


def test_result_writes_json_to_temp_path(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _clear_isaac_env(monkeypatch)
    output_path = tmp_path / "isaac_sensor_runtime.json"

    result = run_isaac_sensor_runtime(
        {
            "frames": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        runtime=IsaacSimRuntime(backend=FakeIsaacSensorBackend()),
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_phase6_isaac_sensor_runtime_v1"
    assert payload["sensor_summary"]["image_shape"] == [4, 5, 3]


def test_runtime_validation_config_contains_phase6b_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    phase6b = config["runtime_validation"]["isaac_sensor_runtime"]

    assert phase6b["enabled"] is False
    assert phase6b["frames"] == 5
    assert phase6b["output_path"] == "outputs/runtime_validation/isaac_sensor_runtime.json"
    assert phase6b["required_env_gates"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ISAAC_RUNTIME_TESTS",
    ]


def test_phase6_profile_points_to_isaac_sensor_runtime_command() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["verification"]["phase6b_isaac_sensor_runtime"] == (
        "python scripts/run_isaac_sensor_runtime.py --frames 5"
    )
    assert profile["verification"]["phase6b_report_path"] == (
        "outputs/runtime_validation/isaac_sensor_runtime.json"
    )


def test_no_ros2_mavsdk_or_px4_paths_are_imported(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    runtime = IsaacSimRuntime(backend=FakeIsaacSensorBackend())

    result = run_isaac_sensor_runtime({"frames": 1, "write_output": False}, runtime=runtime)

    assert result["status"] == "passed"
    assert "rclpy" not in sys.modules
    assert "mavsdk" not in sys.modules
    assert result["safety_summary"]["ros2_started"] is False
    assert result["safety_summary"]["mavsdk_connected"] is False
    assert result["safety_summary"]["px4_launched"] is False
    assert result["safety_summary"]["hardware_check_run"] is False


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_isaac_sensor_runtime.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Phase 6-B Isaac Sim / Isaac Lab sensor runtime" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)
    env.pop("GWM_RUN_ISAAC_RUNTIME_TESTS", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_isaac_sensor_runtime.py"),
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
    assert "isaac_sensor_runtime status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.isaac_runtime
def test_optional_real_isaac_sensor_runtime_is_gated() -> None:
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime startup.")
    if os.environ.get("GWM_RUN_ISAAC_RUNTIME_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ISAAC_RUNTIME_TESTS=1 to run Isaac sensor runtime.")
    if not IsaacSimRuntime.is_available():
        pytest.skip("Isaac Sim / Isaac Lab Python runtime is unavailable.")

    result = run_isaac_sensor_runtime({"frames": 3, "write_output": False})

    assert result["status"] == "passed"
    assert result["frames_completed"] == 3
    assert result["observation_buffer_summary"]["is_ready"] is True
