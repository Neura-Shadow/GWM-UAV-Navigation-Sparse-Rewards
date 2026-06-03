"""Tests for Phase 4-F end-to-end GWM navigation demo."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.generated_world_model import (  # noqa: E402
    GWMDemoConfig,
    GWMDemoResult,
    GWMDemoRunner,
    run_demo,
)


def _small_demo_config(**overrides: object) -> dict:
    demo = {
        "observation_source": "mock",
        "execution_backend": "mock",
        "steps": 2,
        "horizon": 2,
        "num_candidates": 2,
        "seed": 5,
        "device": "cpu",
        "write_output": False,
        "start_pose": [0.0, 0.0, -5.0],
        "goal": [6.0, 0.0, -5.0],
        "control_dt": 0.4,
        "max_speed": 2.0,
        "context_length": 2,
        "image_height": 16,
        "image_width": 16,
        "mock_obstacle_distance": 20.0,
        "min_safe_depth": 0.5,
        "min_obstacle_distance": 4.0,
    }
    demo.update(overrides)
    return {
        "demo": demo,
        "deployment": {
            "mock": True,
            "sitl_enabled": False,
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
            "velocity_limits": {
                "max_vx": 4.0,
                "max_vy": 4.0,
                "max_vz": 2.0,
                "max_yaw_rate": 1.0,
            },
            "altitude_bounds": {"min_altitude": 0.5, "max_altitude": 120.0},
            "geofence": {"enabled": False},
            "cbf": {"enabled": True, "min_obstacle_distance": 4.0, "alpha": 1.0},
        },
    }


def test_demo_package_exports() -> None:
    assert GWMDemoConfig is not None
    assert GWMDemoRunner is not None
    assert GWMDemoResult is not None
    assert run_demo is not None


def test_default_demo_config_preserves_safe_deployment_flags() -> None:
    with open("configs/gwm_navigation_demo.yaml", "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    deployment = config["deployment"]
    demo = config["demo"]

    assert demo["observation_source"] == "mock"
    assert demo["execution_backend"] == "mock"
    assert deployment["mock"] is True
    assert deployment["sitl_enabled"] is False
    assert deployment["real_hardware_enabled"] is False
    assert deployment["autonomous_real_flight_enabled"] is False


def test_mock_demo_runs_full_pipeline_without_optional_dependencies() -> None:
    result = run_demo(_small_demo_config())

    assert result["schema_version"] == "gwm_navigation_demo_v1"
    assert result["final_status"] == "timeout"
    assert result["metrics"]["total_steps"] == 2
    assert result["metrics"]["commands_sent"] == 2
    assert result["backend_summary"]["mock_default"] is True
    assert len(result["steps"]) == 2
    assert result["steps"][0]["candidate_count"] == 2
    assert "total_score" not in result["steps"][0]
    assert "selected_score" in result["steps"][0]
    assert "mavsdk" not in sys.modules
    assert "rclpy" not in sys.modules
    assert "omni.isaac.core" not in sys.modules


def test_demo_writes_json_result_when_requested(tmp_path: Path) -> None:
    output_path = tmp_path / "result.json"
    config = _small_demo_config(write_output=True, output_path=str(output_path))

    result = run_demo(config)
    loaded = json.loads(output_path.read_text(encoding="utf-8"))

    assert loaded["schema_version"] == "gwm_navigation_demo_v1"
    assert loaded["run_id"] == result["run_id"]
    assert loaded["metrics"]["total_steps"] == 2


def test_safety_gate_saturates_unsafe_velocity() -> None:
    config = _small_demo_config()
    config["safety"]["velocity_limits"]["max_vx"] = 0.1
    config["safety"]["velocity_limits"]["max_vy"] = 0.1
    config["safety"]["velocity_limits"]["max_vz"] = 0.1

    result = run_demo(config)
    first_step = result["steps"][0]

    assert abs(first_step["safe_command"]["vx"]) <= 0.1
    assert first_step["safe_command"]["metadata"]["saturated"] is True


def test_obstacle_violation_produces_safety_override() -> None:
    config = _small_demo_config(mock_obstacle_distance=1.0, min_obstacle_distance=4.0)

    result = run_demo(config)

    assert result["metrics"]["safety_overrides"] == 2
    assert result["steps"][0]["safe_command"]["mode"] == "safety_override"
    assert result["steps"][0]["safe_command"]["metadata"]["reason"] == "cbf_obstacle_filter"


def test_real_hardware_flags_are_rejected() -> None:
    config = _small_demo_config()
    config["deployment"]["real_hardware_enabled"] = True

    with pytest.raises(RuntimeError, match="real_hardware_enabled"):
        run_demo(config)


def test_autonomous_real_flight_flag_is_rejected() -> None:
    config = _small_demo_config()
    config["deployment"]["autonomous_real_flight_enabled"] = True

    with pytest.raises(RuntimeError, match="autonomous_real_flight_enabled"):
        run_demo(config)


def test_optional_runtime_without_opt_in_returns_runtime_unavailable() -> None:
    result = run_demo(_small_demo_config(observation_source="isaac"))

    assert result["final_status"] == "runtime_unavailable"
    assert result["metrics"]["total_steps"] == 0
    assert "allow_optional_runtime" in result["backend_summary"]["runtime_unavailable_reason"]


def test_optional_runtime_can_raise_when_requested() -> None:
    config = _small_demo_config(
        observation_source="isaac",
        fail_on_runtime_unavailable=True,
    )

    with pytest.raises(RuntimeError, match="allow_optional_runtime"):
        run_demo(config)


def test_mavsdk_sitl_execution_requires_explicit_sitl_flags() -> None:
    config = _small_demo_config(
        execution_backend="mavsdk_sitl",
        allow_optional_runtime=True,
    )

    with pytest.raises(RuntimeError, match="deployment.mock=False"):
        run_demo(config)


def test_demo_config_from_yaml_path(tmp_path: Path) -> None:
    path = tmp_path / "demo.yaml"
    path.write_text(yaml.safe_dump(_small_demo_config()), encoding="utf-8")

    config = GWMDemoConfig.from_any(path)

    assert config.observation_source == "mock"
    assert config.execution_backend == "mock"
    assert config.image_height == 16


def test_cli_help_and_mock_run(tmp_path: Path) -> None:
    config_path = tmp_path / "demo.yaml"
    config_path.write_text(yaml.safe_dump(_small_demo_config()), encoding="utf-8")

    help_result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "run_gwm_navigation_demo.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert help_result.returncode == 0
    assert "--observation-source" in help_result.stdout

    run_result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "run_gwm_navigation_demo.py"),
            "--config",
            str(config_path),
            "--backend",
            "mock",
            "--steps",
            "3",
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert run_result.returncode == 0, run_result.stderr
    assert "gwm_demo status=timeout" in run_result.stdout
    assert "commands=3" in run_result.stdout


@pytest.mark.gwm_demo_runtime
def test_optional_runtime_smoke_is_gated() -> None:
    import os

    if os.environ.get("GWM_RUN_DEMO_RUNTIME_TESTS") != "1":
        pytest.skip("Set GWM_RUN_DEMO_RUNTIME_TESTS=1 to run guarded runtime smoke tests.")

    result = run_demo(_small_demo_config(allow_optional_runtime=True))

    assert result["schema_version"] == "gwm_navigation_demo_v1"
