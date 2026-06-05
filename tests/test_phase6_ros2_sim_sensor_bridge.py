"""Tests for Phase 6-C guarded ROS2 simulation sensor bridge."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from src.generated_world_model import ObservationBuffer
from src.ros2_bridge import ROS2SensorSynchronizer
from src.runtime_validation import (
    ROS2SimSensorBridgeConfig,
    ROS2SimSensorBridgeResult,
    ROS2SimulationSensorBridge,
    run_ros2_sim_sensor_bridge,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakePublisher:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakePublisherBridge:
    def __init__(self) -> None:
        self.publishers: dict[str, FakePublisher] = {}
        self.closed = False
        self.spin_count = 0

    def create_publisher(self, topic: str, msg_type, qos=None) -> FakePublisher:
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self.spin_count += 1

    def shutdown(self) -> None:
        self.closed = True


class FailingSynchronizer:
    def __init__(self) -> None:
        self.config = type(
            "Config",
            (),
            {"required_streams": ("rgb", "depth", "lidar", "odom"), "real_mode": False},
        )()
        self.closed = False

    def start(self) -> None:
        return None

    def ingest(self, stream, message):
        raise RuntimeError(f"phase6 fake sync failure on {stream}")

    def try_sync(self):
        return None

    def latest_observation(self):
        return None

    def shutdown(self):
        self.closed = True


def _clear_ros2_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", raising=False)
    monkeypatch.delenv("GWM_ROS2_LIVE_TOPICS", raising=False)


def test_phase6_ros2_sim_sensor_bridge_exports_without_ros2() -> None:
    assert ROS2SimSensorBridgeConfig is not None
    assert ROS2SimSensorBridgeResult is not None
    assert ROS2SimulationSensorBridge is not None
    assert run_ros2_sim_sensor_bridge is not None


def test_missing_env_gates_skip_without_availability_probe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    monkeypatch.setattr(
        ROS2SimulationSensorBridge,
        "is_available",
        staticmethod(lambda: (_ for _ in ()).throw(AssertionError("availability checked"))),
    )

    result = run_ros2_sim_sensor_bridge({"write_output": False})

    assert result["schema_version"] == "gwm_phase6_ros2_sim_sensor_bridge_v1"
    assert result["status"] == "skipped"
    assert "Missing required ROS2 simulation sensor bridge env gates" in result["reason"]
    assert result["availability"]["checked"] is False
    assert result["frames_published"] == 0
    assert result["closed"] is False


def test_gated_unavailable_ros2_runtime_reports_setup_instructions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", "1")
    monkeypatch.setattr(ROS2SimulationSensorBridge, "is_available", staticmethod(lambda: False))

    result = run_ros2_sim_sensor_bridge({"write_output": False, "fail_on_unavailable": True})

    assert result["status"] == "runtime_unavailable"
    assert result["availability"]["checked"] is True
    assert result["availability"]["ros2_sim_sensor_bridge_available"] is False
    assert "ROS2 publisher/synchronizer runtime is unavailable" in result["reason"]
    assert any("Install and source ROS2" in item for item in result["setup_instructions"])


def test_fake_publisher_bridge_publishes_syncs_and_fills_observation_buffer(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    fake_bridge = FakePublisherBridge()
    buffer = ObservationBuffer(context_length=3, image_size=(16, 16))

    result = run_ros2_sim_sensor_bridge(
        ROS2SimSensorBridgeConfig(
            frames=3,
            context_length=3,
            image_height=16,
            image_width=16,
            write_output=False,
        ),
        bridge=fake_bridge,
        observation_buffer=buffer,
    )

    publish = result["publish_summary"]
    observation = result["sensor_observation_summary"]
    buffer_summary = result["observation_buffer_summary"]
    assert result["status"] == "passed"
    assert result["frames_published"] == 3
    assert result["packets_synchronized"] == 3
    assert result["observations_collected"] == 3
    assert result["availability"]["publisher_bridge_injected"] is True
    assert publish["messages_published"]["rgb"] == 3
    assert publish["messages_published"]["depth"] == 3
    assert publish["messages_published"]["lidar"] == 3
    assert publish["messages_published"]["odom"] == 3
    assert publish["messages_published"]["imu"] == 3
    assert fake_bridge.publishers["/camera/rgb"].messages
    assert observation["has_image"] is True
    assert observation["has_depth"] is True
    assert observation["has_lidar"] is True
    assert observation["has_imu"] is True
    assert observation["obstacle_distance"] == pytest.approx(1.5)
    assert buffer.is_ready is True
    assert buffer_summary["batch"]["rgb_shape"] == [1, 3, 3, 16, 16]
    assert buffer_summary["batch"]["depth_shape"] == [1, 3, 1, 16, 16]
    assert fake_bridge.closed is True
    assert result["closed"] is True


def test_injected_manual_synchronizer_path_passes_without_fake_publishers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    synchronizer = ROS2SensorSynchronizer()

    result = run_ros2_sim_sensor_bridge(
        {"frames": 2, "write_output": False},
        synchronizer=synchronizer,
    )

    assert result["status"] == "passed"
    assert result["packets_synchronized"] == 2
    assert result["publish_summary"]["publisher_count"] == 0
    assert result["availability"]["synchronizer_injected"] is True
    assert result["closed"] is True


def test_sync_failure_records_failed_and_still_closes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)
    fake_bridge = FakePublisherBridge()
    synchronizer = FailingSynchronizer()

    result = run_ros2_sim_sensor_bridge(
        ROS2SimSensorBridgeConfig(frames=1, write_output=False),
        bridge=fake_bridge,
        synchronizer=synchronizer,
    )

    assert result["status"] == "failed"
    assert "phase6 fake sync failure" in result["reason"]
    assert synchronizer.closed is True
    assert fake_bridge.closed is True
    assert result["closed"] is True
    assert result["errors"][0]["type"] == "RuntimeError"


def test_result_writes_json_to_temp_path(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _clear_ros2_env(monkeypatch)
    output_path = tmp_path / "ros2_sim_sensor_bridge.json"

    result = run_ros2_sim_sensor_bridge(
        {
            "frames": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        bridge=FakePublisherBridge(),
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_phase6_ros2_sim_sensor_bridge_v1"
    assert payload["sensor_observation_summary"]["image_shape"] == [2, 3, 3]
    assert payload["sensor_observation_summary"]["depth_shape"] == [2, 3]


def test_runtime_validation_config_contains_phase6c_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    phase6c = config["runtime_validation"]["ros2_sim_sensor_bridge"]

    assert phase6c["enabled"] is False
    assert phase6c["frames"] == 3
    assert phase6c["output_path"] == "outputs/runtime_validation/ros2_sim_sensor_bridge.json"
    assert phase6c["required_env_gates"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
    ]


def test_phase6_profile_points_to_ros2_sim_sensor_bridge_command() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["verification"]["phase6c_ros2_sim_sensor_bridge"] == (
        "python scripts/run_ros2_sim_sensor_bridge.py --frames 3"
    )
    assert profile["verification"]["phase6c_report_path"] == (
        "outputs/runtime_validation/ros2_sim_sensor_bridge.json"
    )


def test_no_isaac_mavsdk_px4_or_nav2_paths_are_started(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_ros2_env(monkeypatch)

    result = run_ros2_sim_sensor_bridge(
        {"frames": 1, "write_output": False},
        bridge=FakePublisherBridge(),
    )

    safety = result["safety_summary"]
    assert result["status"] == "passed"
    assert "mavsdk" not in sys.modules
    assert safety["isaac_launched"] is False
    assert safety["mavsdk_connected"] is False
    assert safety["px4_launched"] is False
    assert safety["nav2_started"] is False
    assert safety["hardware_check_run"] is False


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_ros2_sim_sensor_bridge.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Phase 6-C ROS2 simulation sensor bridge" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)
    env.pop("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", None)
    env.pop("GWM_ROS2_LIVE_TOPICS", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_ros2_sim_sensor_bridge.py"),
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
    assert "ros2_sim_sensor_bridge status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.ros2_runtime
def test_optional_real_ros2_sim_sensor_bridge_is_gated() -> None:
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime startup.")
    if os.environ.get("GWM_RUN_ROS2_SENSOR_SYNC_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ROS2_SENSOR_SYNC_TESTS=1 to run ROS2 simulation sensor bridge.")
    if not ROS2SimulationSensorBridge.is_available():
        pytest.skip("ROS2 publisher/synchronizer runtime is unavailable.")

    result = run_ros2_sim_sensor_bridge({"frames": 1, "write_output": False})

    assert result["status"] == "passed"
    assert result["packets_synchronized"] >= 1
    assert isinstance(result["sensor_observation_summary"], dict)
