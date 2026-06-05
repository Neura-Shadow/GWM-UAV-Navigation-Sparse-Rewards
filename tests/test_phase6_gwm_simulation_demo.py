"""Tests for Phase 6-F guarded GWM / WAM simulation demo."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.digital_twin import IsaacSimRuntime
from src.generated_world_model import (
    DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH,
    Phase6GWMSimulationDemoConfig,
    Phase6GWMSimulationDemoResult,
    Phase6RuntimeReadiness,
    run_phase6_gwm_simulation_demo,
)
from src.utils.data_types import ControlCommand, SensorObservation

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakePhase6IsaacEnv:
    def __init__(self, *, obstacle_distance: float = 20.0) -> None:
        self.pose = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.goal = np.array([8.0, 0.0, -5.0], dtype=np.float32)
        self.obstacle_distance = float(obstacle_distance)
        self.step_count = 0
        self.closed = False

    def reset(self) -> SensorObservation:
        self.pose = np.array([0.0, 0.0, -5.0], dtype=np.float32)
        self.velocity = np.zeros(3, dtype=np.float32)
        self.step_count = 0
        return self.get_observation()

    def get_observation(self) -> SensorObservation:
        return self._observation()

    def step(self, action: np.ndarray):
        self.velocity = np.asarray(action, dtype=np.float32).reshape(-1)[:3]
        self.pose = self.pose + self.velocity * 0.2
        self.step_count += 1
        observation = self._observation()
        return observation, -observation.goal_distance, False, {"step": self.step_count}

    def close(self) -> None:
        self.closed = True

    def _observation(self) -> SensorObservation:
        image = np.zeros((16, 16, 3), dtype=np.uint8)
        image[..., 0] = int((self.step_count * 29) % 255)
        depth = np.ones((16, 16), dtype=np.float32) * self.obstacle_distance
        return SensorObservation(
            timestamp=self.step_count * 0.2,
            pose=tuple(float(value) for value in self.pose),
            velocity=tuple(float(value) for value in self.velocity),
            goal_distance=float(np.linalg.norm(self.pose - self.goal)),
            obstacle_distance=float(self.obstacle_distance),
            image=image,
            depth=depth,
            metadata={"source": "fake_phase6_isaac", "frame": self.step_count},
        )


class FakeROS2Synchronizer:
    def __init__(self, env: FakePhase6IsaacEnv) -> None:
        self.env = env
        self.closed = False
        self.calls = 0

    def latest_observation(self) -> SensorObservation:
        self.calls += 1
        return self.env.get_observation()

    def shutdown(self) -> None:
        self.closed = True


class FakePhase6MAVLinkBridge:
    def __init__(self, *, fail_on_send: bool = False) -> None:
        self.fail_on_send = fail_on_send
        self.command_history: list[dict] = []
        self._connected = False
        self._offboard = False
        self.disconnect_called = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_offboard(self) -> bool:
        return self._offboard

    async def connect(self) -> bool:
        self._connected = True
        self.command_history.append({"action": "connect"})
        return True

    async def wait_until_ready(self, timeout_sec: float = 10.0) -> bool:
        self.command_history.append({"action": "wait_until_ready", "timeout_sec": timeout_sec})
        return True

    async def start_offboard(self, initial_command: ControlCommand) -> bool:
        self._offboard = True
        self.command_history.append(
            {"action": "send_initial_setpoint", "command": _command_payload(initial_command)}
        )
        self.command_history.append({"action": "start_offboard"})
        return True

    async def send_command(self, command: ControlCommand) -> bool:
        if self.fail_on_send:
            raise RuntimeError("fake phase6 mavsdk send failure")
        self.command_history.append({"action": "send_command", "command": _command_payload(command)})
        return True

    async def emergency_stop(self) -> bool:
        self.command_history.append({"action": "emergency_stop"})
        return True

    async def disconnect(self) -> None:
        self.disconnect_called = True
        self._connected = False
        self._offboard = False
        self.command_history.append({"action": "disconnect"})


def _command_payload(command: ControlCommand) -> dict:
    return {
        "velocity": {"vx": command.vx, "vy": command.vy, "vz": command.vz},
        "mode": command.mode.value,
        "metadata": dict(command.metadata),
    }


def _small_config(**overrides: object) -> dict:
    config = {
        "steps": 2,
        "horizon": 2,
        "num_candidates": 2,
        "seed": 4,
        "device": "cpu",
        "observation_path": "direct_isaac",
        "write_output": False,
        "require_prior_reports": False,
        "planner_interval_steps": 1,
        "control_dt": 0.2,
        "context_length": 2,
        "image_height": 16,
        "image_width": 16,
        "max_speed": 1.0,
        "goal": [8.0, 0.0, -5.0],
        "deployment": {
            "mock": False,
            "sitl_enabled": True,
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
        },
        "model": {
            "image_height": 16,
            "image_width": 16,
            "context_length": 2,
            "horizon": 2,
            "latent_dim": 12,
            "visual_feature_dim": 12,
            "state_feature_dim": 8,
            "conditioning_dim": 10,
            "hidden_dim": 16,
        },
        "trajectory_scoring": {
            "weights": {
                "goal_progress": 1.0,
                "collision_risk": 2.0,
                "uncertainty": 0.2,
                "energy": 0.01,
                "smoothness": 0.01,
                "altitude_violation": 4.0,
                "geofence_violation": 4.0,
            }
        },
        "safety": {
            "min_obstacle_distance": 4.0,
            "limits": {
                "velocity_limits": {
                    "max_vx": 4.0,
                    "max_vy": 4.0,
                    "max_vz": 2.0,
                    "max_yaw_rate": 1.0,
                },
                "altitude_bounds": {"min_altitude": 0.5, "max_altitude": 120.0},
                "geofence": {"enabled": False},
            },
        },
        "mavlink": {"health_timeout_sec": 0.1},
        "frame_transform_policy": {"transforms_defined": False},
    }
    config.update(overrides)
    return config


def test_phase6_gwm_simulation_demo_exports_without_optional_runtimes() -> None:
    assert DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH.endswith(
        "phase6_gwm_simulation_demo.json"
    )
    assert Phase6GWMSimulationDemoConfig is not None
    assert Phase6GWMSimulationDemoResult is not None
    assert Phase6RuntimeReadiness is not None
    assert run_phase6_gwm_simulation_demo is not None


def test_no_gate_run_skips_without_runtime_availability_probe(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_RUN_ISAAC_RUNTIME_TESTS", raising=False)
    monkeypatch.delenv("GWM_RUN_MAVSDK_SITL_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)
    monkeypatch.setattr(
        IsaacSimRuntime,
        "is_available",
        staticmethod(lambda: (_ for _ in ()).throw(AssertionError("availability checked"))),
    )
    monkeypatch.setattr(
        mavlink_module,
        "_load_mavsdk_system",
        lambda: (_ for _ in ()).throw(AssertionError("MAVSDK checked")),
    )

    result = run_phase6_gwm_simulation_demo(_small_config())

    assert result["schema_version"] == "gwm_phase6_simulation_demo_v1"
    assert result["status"] == "skipped"
    assert "Missing required Phase 6-F runtime env gates" in result["reason"]
    assert result["metrics"]["commands_sent"] == 0


def test_unsafe_deployment_flags_are_rejected_before_runtime() -> None:
    config = _small_config(
        deployment={
            "mock": False,
            "sitl_enabled": True,
            "real_hardware_enabled": True,
            "autonomous_real_flight_enabled": False,
        }
    )

    result = run_phase6_gwm_simulation_demo(config)

    assert result["status"] == "failed"
    assert "real_hardware_enabled=True" in result["reason"]


def test_fake_direct_isaac_and_mavsdk_full_loop_passes() -> None:
    env = FakePhase6IsaacEnv()
    bridge = FakePhase6MAVLinkBridge()

    result = run_phase6_gwm_simulation_demo(
        _small_config(),
        isaac_env=env,
        mavlink_bridge=bridge,
    )

    assert result["status"] == "passed"
    assert result["metrics"]["steps"] == 2
    assert result["metrics"]["commands_sent"] == 2
    assert result["metrics"]["planner_updates"] == 2
    assert result["loop_summary"]["state_coupling"] == "command_mirror"
    assert result["loop_summary"]["px4_telemetry_used_for_isaac_state"] is False
    assert result["safety_summary"]["cbf_applied_before_every_mavsdk_write"] is True
    assert bridge.disconnect_called is True
    assert env.closed is True


def test_fake_ros2_observation_path_fills_buffer_and_passes() -> None:
    env = FakePhase6IsaacEnv()
    sync = FakeROS2Synchronizer(env)
    bridge = FakePhase6MAVLinkBridge()

    result = run_phase6_gwm_simulation_demo(
        _small_config(observation_path="ros2"),
        isaac_env=env,
        ros2_synchronizer=sync,
        mavlink_bridge=bridge,
    )

    assert result["status"] == "passed"
    assert result["backend_summary"]["observation_source"]["type"] == "ros2"
    assert result["metrics"]["observations"] == 2
    assert sync.closed is True
    assert env.closed is True


def test_excessive_velocity_is_saturated_before_bridge_write() -> None:
    env = FakePhase6IsaacEnv()
    bridge = FakePhase6MAVLinkBridge()
    config = _small_config(
        max_speed=5.0,
        safety={
            "min_obstacle_distance": 4.0,
            "limits": {
                "velocity_limits": {
                    "max_vx": 0.2,
                    "max_vy": 0.2,
                    "max_vz": 0.2,
                    "max_yaw_rate": 0.1,
                },
                "altitude_bounds": {"min_altitude": 0.5, "max_altitude": 120.0},
                "geofence": {"enabled": False},
            },
        },
    )

    result = run_phase6_gwm_simulation_demo(config, isaac_env=env, mavlink_bridge=bridge)

    safe_command = result["steps"][0]["safe_command"]
    bridge_command = [
        item for item in bridge.command_history if item.get("action") == "send_command"
    ][0]["command"]
    assert result["status"] == "passed"
    assert safe_command["metadata"]["saturated"] is True
    assert abs(safe_command["vx"]) <= 0.2
    assert abs(bridge_command["velocity"]["vx"]) <= 0.2


def test_obstacle_violation_produces_safety_override() -> None:
    env = FakePhase6IsaacEnv(obstacle_distance=1.0)
    bridge = FakePhase6MAVLinkBridge()

    result = run_phase6_gwm_simulation_demo(
        _small_config(),
        isaac_env=env,
        mavlink_bridge=bridge,
    )

    assert result["status"] == "passed"
    assert result["metrics"]["safety_overrides"] == 2
    assert result["steps"][0]["safe_command"]["mode"] == "safety_override"


def test_missing_prior_reports_produce_not_ready(tmp_path: Path) -> None:
    result = run_phase6_gwm_simulation_demo(
        _small_config(
            require_prior_reports=True,
            prior_reports={
                "isaac_sensor_runtime": str(tmp_path / "missing_isaac.json"),
                "px4_sitl_command_validation": str(tmp_path / "missing_px4.json"),
                "isaac_px4_bridge_design": str(tmp_path / "missing_bridge.json"),
            },
        ),
        isaac_env=FakePhase6IsaacEnv(),
        mavlink_bridge=FakePhase6MAVLinkBridge(),
    )

    assert result["status"] == "not_ready"
    assert "Missing or unready prior Phase 6 reports" in result["reason"]


def test_present_prior_reports_allow_fake_full_loop(tmp_path: Path) -> None:
    report_paths = {
        "isaac_sensor_runtime": tmp_path / "isaac.json",
        "px4_sitl_command_validation": tmp_path / "px4.json",
        "isaac_px4_bridge_design": tmp_path / "bridge.json",
    }
    for path in report_paths.values():
        path.write_text(json.dumps({"status": "passed"}), encoding="utf-8")

    result = run_phase6_gwm_simulation_demo(
        _small_config(
            require_prior_reports=True,
            prior_reports={name: str(path) for name, path in report_paths.items()},
        ),
        isaac_env=FakePhase6IsaacEnv(),
        mavlink_bridge=FakePhase6MAVLinkBridge(),
    )

    assert result["status"] == "passed"
    assert result["report_readiness"]["all_ready"] is True


def test_failure_after_mavsdk_connect_attempts_emergency_stop_and_cleanup() -> None:
    bridge = FakePhase6MAVLinkBridge(fail_on_send=True)

    result = run_phase6_gwm_simulation_demo(
        _small_config(),
        isaac_env=FakePhase6IsaacEnv(),
        mavlink_bridge=bridge,
    )

    actions = [item["action"] for item in bridge.command_history]
    assert result["status"] == "failed"
    assert "fake phase6 mavsdk send failure" in result["reason"]
    assert result["safety_summary"]["emergency_stop_attempted"] is True
    assert "emergency_stop" in actions
    assert actions[-1] == "disconnect"


def test_result_writes_json_to_temp_path(tmp_path: Path) -> None:
    output_path = tmp_path / "phase6_demo.json"

    result = run_phase6_gwm_simulation_demo(
        _small_config(output_path=str(output_path), write_output=True),
        isaac_env=FakePhase6IsaacEnv(),
        mavlink_bridge=FakePhase6MAVLinkBridge(),
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_phase6_simulation_demo_v1"
    assert payload["metrics"]["commands_sent"] == 2


def test_runtime_validation_config_contains_phase6f_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    phase6f = config["runtime_validation"]["phase6_gwm_simulation_demo"]

    assert phase6f["enabled"] is False
    assert phase6f["runtime_mode"] == "guarded"
    assert phase6f["observation_path"] == "direct_isaac"
    assert phase6f["output_path"] == "outputs/runtime_validation/phase6_gwm_simulation_demo.json"
    assert phase6f["deployment"] == {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert phase6f["px4_launch_requested"] is False


def test_phase6_profile_points_to_gwm_simulation_demo_command() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["verification"]["phase6f_gwm_simulation_demo"] == (
        "python scripts/run_phase6_gwm_simulation_demo.py --require-prior-reports"
    )
    assert profile["verification"]["phase6f_fake_mode_check"] == (
        "python scripts/run_phase6_gwm_simulation_demo.py --runtime-mode fake --steps 3 "
        "--no-require-prior-reports --no-write-output"
    )
    assert profile["verification"]["phase6f_report_path"] == (
        "outputs/runtime_validation/phase6_gwm_simulation_demo.json"
    )


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_phase6_gwm_simulation_demo.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Phase 6-F GWM / WAM simulation demo" in result.stdout


def test_cli_no_gate_run_skips_and_writes_nothing(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    for name in (
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ISAAC_RUNTIME_TESTS",
        "GWM_RUN_MAVSDK_SITL_TESTS",
        "GWM_ALLOW_SITL_COMMANDS",
    ):
        env.pop(name, None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_phase6_gwm_simulation_demo.py"),
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
    assert "phase6_gwm_simulation_demo status=skipped" in result.stdout
    assert output_path.exists() is False


def test_cli_fake_mode_runs_full_loop_without_runtime_gates() -> None:
    env = dict(os.environ)
    for name in (
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ISAAC_RUNTIME_TESTS",
        "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
        "GWM_RUN_MAVSDK_SITL_TESTS",
        "GWM_ALLOW_SITL_COMMANDS",
    ):
        env.pop(name, None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_phase6_gwm_simulation_demo.py"),
            "--runtime-mode",
            "fake",
            "--steps",
            "3",
            "--no-require-prior-reports",
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "phase6_gwm_simulation_demo status=passed" in result.stdout
    assert "steps=3" in result.stdout
    assert "commands=3" in result.stdout


@pytest.mark.gwm_demo_runtime
def test_optional_runtime_demo_is_gated() -> None:
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime.")
    if os.environ.get("GWM_RUN_ISAAC_RUNTIME_TESTS") != "1":
        pytest.skip("Set GWM_RUN_ISAAC_RUNTIME_TESTS=1 to run Isaac runtime.")
    if os.environ.get("GWM_RUN_MAVSDK_SITL_TESTS") != "1":
        pytest.skip("Set GWM_RUN_MAVSDK_SITL_TESTS=1 to run PX4 SITL runtime.")
    if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1":
        pytest.skip("Set GWM_ALLOW_SITL_COMMANDS=1 to allow SITL commands.")

    result = run_phase6_gwm_simulation_demo(_small_config(write_output=False))

    assert result["status"] in {"passed", "runtime_unavailable", "failed"}
