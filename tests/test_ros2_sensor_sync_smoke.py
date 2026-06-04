"""Tests for Phase 5-C guarded ROS2 sensor sync smoke tests."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from src.ros2_bridge import ROS2SensorSynchronizer
from src.runtime_validation import (
    ROS2SensorSyncSmokeConfig,
    ROS2SensorSyncSmokeResult,
    build_mock_sensor_messages,
    run_ros2_sensor_sync_smoke,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FailingSynchronizer:
    def __init__(self) -> None:
        self.config = type("Config", (), {"required_streams": ("rgb", "depth", "lidar", "odom")})()
        self.closed = False

    def ingest(self, stream, message):
        raise RuntimeError(f"fake sync failure on {stream}")

    def try_sync(self):
        return None

    def latest_observation(self):
        return None

    def shutdown(self):
        self.closed = True


def _clear_ros2_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)


def test_package_imports_without_ros2() -> None:
    assert ROS2SensorSyncSmokeConfig is not None
    assert ROS2SensorSyncSmokeResult is not None
    assert build_mock_sensor_messages is not None
    assert run_ros2_sensor_sync_smoke is not None


def test_missing_env_gates_skip_cleanly_without_starting_real_mode(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    monkeypatch.setattr(
        ROS2SensorSynchronizer,
        "is_available",
        staticmethod(lambda: (_ for _ in ()).throw(AssertionError("availability checked"))),
    )

    result = run_ros2_sensor_sync_smoke({"write_output": False})

    assert result["schema_version"] == "gwm_ros2_sensor_sync_smoke_v1"
    assert result["status"] == "skipped"
    assert "Missing required ROS2 sensor sync env gates" in result["reason"]
    assert result["availability"]["checked"] is False
    assert result["packets_completed"] == 0
    assert result["closed"] is False


def test_ros2_unavailable_skips_cleanly(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", "1")
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setattr(ROS2SensorSynchronizer, "is_available", staticmethod(lambda: False))

    result = run_ros2_sensor_sync_smoke({"write_output": False, "fail_on_unavailable": True})

    assert result["status"] == "skipped"
    assert result["availability"]["checked"] is True
    assert result["availability"]["ros2_sensor_sync_available"] is False
    assert result["reason"] == "ROS2 sensor synchronization modules are unavailable."


def test_mock_message_builder_creates_required_streams_with_synchronized_timestamps() -> None:
    messages = build_mock_sensor_messages(timestamp=20.0)

    assert set(messages) == {"rgb", "depth", "lidar", "odom", "imu"}
    assert messages["rgb"]["header"]["frame_id"] == "camera_rgb_optical"
    assert messages["odom"]["header"]["stamp"]["sec"] == 20
    assert messages["odom"]["header"]["stamp"]["nanosec"] == 30_000_000
    assert messages["depth"]["encoding"] == "32FC1"


def test_injected_manual_synchronizer_passes_and_produces_sensor_observation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    synchronizer = ROS2SensorSynchronizer()

    result = run_ros2_sensor_sync_smoke(
        ROS2SensorSyncSmokeConfig(packets=1, write_output=False),
        synchronizer=synchronizer,
    )

    assert result["status"] == "passed"
    assert result["packets_completed"] == 1
    assert result["availability"]["checked"] is False
    assert result["availability"]["synchronizer_injected"] is True
    assert result["sync_summary"]["streams"] == ["depth", "imu", "lidar", "odom", "rgb"]
    assert result["sensor_metadata"]["has_imu"] is True
    assert result["sensor_observation_summary"]["has_image"] is True
    assert result["sensor_observation_summary"]["has_depth"] is True
    assert result["sensor_observation_summary"]["has_lidar"] is True
    assert result["sensor_observation_summary"]["goal_distance"] == pytest.approx(12.0)
    assert result["sensor_observation_summary"]["obstacle_distance"] == pytest.approx(1.5)
    assert result["closed"] is True


def test_result_writes_json_to_temp_path(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _clear_ros2_env(monkeypatch)
    output_path = tmp_path / "ros2_sensor_sync_smoke.json"
    synchronizer = ROS2SensorSynchronizer()

    result = run_ros2_sensor_sync_smoke(
        {
            "packets": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        synchronizer=synchronizer,
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_ros2_sensor_sync_smoke_v1"
    assert payload["sensor_observation_summary"]["image_shape"] == [2, 3, 3]
    assert payload["sensor_observation_summary"]["depth_shape"] == [2, 3]


def test_sync_failure_records_failed_and_still_shutdowns(monkeypatch: pytest.MonkeyPatch) -> None:
    _clear_ros2_env(monkeypatch)
    synchronizer = FailingSynchronizer()

    result = run_ros2_sensor_sync_smoke(
        ROS2SensorSyncSmokeConfig(packets=1, write_output=False),
        synchronizer=synchronizer,
    )

    assert result["status"] == "failed"
    assert "fake sync failure" in result["reason"]
    assert result["closed"] is True
    assert synchronizer.closed is True
    assert result["errors"][0]["type"] == "RuntimeError"


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_ros2_sensor_sync_smoke.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "guarded ROS2 sensor synchronization smoke test" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", None)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_ros2_sensor_sync_smoke.py"),
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
    assert "ros2_sensor_sync_smoke status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.ros2_runtime
def test_optional_real_ros2_sensor_sync_smoke_is_gated() -> None:
    if os.environ.get("GWM_RUN_ROS2_SENSOR_SYNC_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1 to run ROS2 sensor sync smoke.")
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime startup.")

    result = run_ros2_sensor_sync_smoke({"packets": 1, "write_output": False})
    if result["status"] == "skipped":
        pytest.skip(result["reason"])

    assert result["status"] == "passed"
    assert result["packets_completed"] == 1
    assert isinstance(result["sensor_observation_summary"], dict)
