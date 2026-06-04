"""Tests for Phase 5-B guarded Isaac Sim runtime smoke tests."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from src.digital_twin import IsaacSimRuntime
from src.runtime_validation import (
    IsaacRuntimeSmokeConfig,
    IsaacRuntimeSmokeResult,
    build_tiny_isaac_descriptor,
    run_isaac_runtime_smoke,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeIsaacSmokeBackend:
    def __init__(self, *, fail_on_step: bool = False) -> None:
        self.fail_on_step = fail_on_step
        self.launched = False
        self.closed = False
        self.loaded_descriptor = None
        self.step_history = []
        self.pose = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)

    def launch(self, config):
        self.launched = True
        self.launch_config = config

    def load_descriptor(self, descriptor):
        self.loaded_descriptor = descriptor
        self.pose = np.asarray(descriptor["vehicle"]["position"], dtype=np.float32)
        return descriptor

    def step(self, action, dt):
        if self.fail_on_step:
            raise RuntimeError("fake step failure")
        action_array = np.asarray(action, dtype=np.float32)
        self.velocity = action_array
        self.pose = self.pose + action_array * float(dt)
        self.step_history.append({"action": action_array.tolist(), "dt": float(dt)})
        return {"backend": "fake", "step_count": len(self.step_history)}

    def read_sensors(self):
        return {
            "timestamp": len(self.step_history) * 0.05,
            "pose": self.pose.tolist(),
            "velocity": self.velocity.tolist(),
            "goal": [2.0, 0.0, -5.0],
            "rgb": np.zeros((4, 5, 3), dtype=np.uint8),
            "depth": np.ones((4, 5), dtype=np.float32) * 6.0,
            "lidar": np.array([[3.0, 0.0, 0.0], [0.0, 4.0, 0.0]], dtype=np.float32),
            "imu": {"linear_acceleration": [0.0, 0.0, 0.0]},
            "metadata": {"backend": "fake"},
        }

    def close(self):
        self.closed = True


def _clear_isaac_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_RUN_ISAAC_RUNTIME_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)


def test_package_imports_without_isaac_sim() -> None:
    assert IsaacRuntimeSmokeConfig is not None
    assert IsaacRuntimeSmokeResult is not None
    assert build_tiny_isaac_descriptor is not None
    assert run_isaac_runtime_smoke is not None


def test_missing_env_gates_skip_cleanly(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)

    result = run_isaac_runtime_smoke({"write_output": False})

    assert result["schema_version"] == "gwm_isaac_runtime_smoke_v1"
    assert result["status"] == "skipped"
    assert "Missing required Isaac runtime env gates" in result["reason"]
    assert result["frames_completed"] == 0
    assert result["closed"] is False


def test_isaac_unavailable_skips_cleanly(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_RUN_ISAAC_RUNTIME_TESTS", "1")
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setattr(IsaacSimRuntime, "is_available", staticmethod(lambda: False))

    result = run_isaac_runtime_smoke({"write_output": False})

    assert result["status"] == "skipped"
    assert result["availability"]["checked"] is True
    assert result["availability"]["isaac_sim_available"] is False
    assert result["reason"] == "Isaac Sim Python runtime is unavailable."


def test_fake_backend_smoke_passes_without_isaac_env(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    backend = FakeIsaacSmokeBackend()
    runtime = IsaacSimRuntime(backend=backend)

    result = run_isaac_runtime_smoke(
        IsaacRuntimeSmokeConfig(frames=3, write_output=False),
        runtime=runtime,
    )

    assert result["status"] == "passed"
    assert result["frames_completed"] == 3
    assert result["availability"]["checked"] is False
    assert result["availability"]["injected_backend"] is True
    assert result["sensor_observation_summary"]["has_image"] is True
    assert result["sensor_observation_summary"]["has_depth"] is True
    assert result["sensor_observation_summary"]["has_lidar"] is True
    assert backend.launched is True
    assert backend.closed is True


def test_tiny_descriptor_preserves_coordinate_metadata() -> None:
    descriptor = build_tiny_isaac_descriptor()

    assert descriptor["metadata"]["source_coordinate_frame"] == "project_default"
    assert descriptor["metadata"]["target_coordinate_frame"] == "isaac_z_up_pending"
    assert descriptor["metadata"]["coordinate_conversion_applied"] is False
    assert [sensor["name"] for sensor in descriptor["sensors"]] == [
        "DepthCamera",
        "Lidar",
        "Imu",
    ]


def test_sensor_observation_conversion_is_exercised(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    runtime = IsaacSimRuntime(backend=FakeIsaacSmokeBackend())

    result = run_isaac_runtime_smoke(
        {"frames": 1, "write_output": False},
        runtime=runtime,
    )

    summary = result["sensor_observation_summary"]
    assert result["status"] == "passed"
    assert summary["metadata"]["imu"]["linear_acceleration"] == [0.0, 0.0, 0.0]
    assert summary["goal_distance"] == pytest.approx(2.0)
    assert summary["obstacle_distance"] == pytest.approx(3.0)
    assert summary["image_shape"] == [4, 5, 3]


def test_runtime_failure_still_closes_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_isaac_env(monkeypatch)
    backend = FakeIsaacSmokeBackend(fail_on_step=True)
    runtime = IsaacSimRuntime(backend=backend)

    result = run_isaac_runtime_smoke(
        IsaacRuntimeSmokeConfig(frames=2, write_output=False),
        runtime=runtime,
    )

    assert result["status"] == "failed"
    assert "fake step failure" in result["reason"]
    assert result["closed"] is True
    assert backend.closed is True
    assert result["errors"][0]["type"] == "RuntimeError"


def test_result_writes_json_to_temp_path(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _clear_isaac_env(monkeypatch)
    output_path = tmp_path / "isaac_runtime_smoke.json"
    runtime = IsaacSimRuntime(backend=FakeIsaacSmokeBackend())

    result = run_isaac_runtime_smoke(
        {
            "frames": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        runtime=runtime,
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_isaac_runtime_smoke_v1"
    assert payload["sensor_metadata"]["has_rgb"] is True
    assert payload["sensor_observation_summary"]["image_shape"] == [4, 5, 3]


def test_cli_help_works() -> None:
    result = subprocess.run(
        [sys.executable, str(_PROJECT_ROOT / "scripts" / "run_isaac_runtime_smoke.py"), "--help"],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "guarded Isaac Sim runtime smoke test" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_RUN_ISAAC_RUNTIME_TESTS", None)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_isaac_runtime_smoke.py"),
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
    assert "isaac_runtime_smoke status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.isaac_runtime
def test_optional_real_isaac_runtime_smoke_is_gated() -> None:
    if os.environ.get("GWM_RUN_ISAAC_RUNTIME_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ISAAC_RUNTIME_TESTS=1 to run Isaac runtime smoke.")
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime launch.")

    result = run_isaac_runtime_smoke({"frames": 3, "write_output": False})
    if result["status"] == "skipped":
        pytest.skip(result["reason"])

    assert result["status"] == "passed"
    assert result["frames_completed"] == 3
    assert isinstance(result["sensor_observation_summary"], dict)
