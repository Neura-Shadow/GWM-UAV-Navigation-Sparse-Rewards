"""Tests for Phase 5-E closed-loop mock-to-SITL readiness."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.runtime_validation import (  # noqa: E402
    ClosedLoopReadinessConfig,
    ClosedLoopReadinessResult,
    build_closed_loop_pipeline_plan,
    run_closed_loop_readiness,
)


def test_closed_loop_readiness_package_imports() -> None:
    assert ClosedLoopReadinessConfig is not None
    assert ClosedLoopReadinessResult is not None
    assert build_closed_loop_pipeline_plan is not None
    assert run_closed_loop_readiness is not None


def test_pipeline_plan_contains_exact_closed_loop_flow() -> None:
    plan = build_closed_loop_pipeline_plan()

    assert plan["flow"] == [
        "Observation backend",
        "ObservationBuffer",
        "Generated World Model rollout",
        "Candidate trajectory sampler",
        "Trajectory scorer",
        "ControlBarrierFunction safety gate",
        "Execution backend",
        "Runtime metrics / failure handling",
    ]
    assert plan["default_observation_backend"] == "mock"
    assert plan["default_execution_backend"] == "mock"
    assert "launch PX4" in plan["non_goals"]


def test_default_run_executes_mock_demo_and_passes() -> None:
    result = run_closed_loop_readiness(
        ClosedLoopReadinessConfig(steps=2, write_output=False)
    )

    assert result["schema_version"] == "gwm_closed_loop_readiness_v1"
    assert result["status"] == "passed"
    assert result["demo_summary"]["executed"] is True
    assert result["demo_summary"]["final_status"] == "timeout"
    assert result["demo_summary"]["observation_source"] == "mock"
    assert result["demo_summary"]["execution_backend"] == "mock"
    assert result["metrics"]["mock_demo_steps"] == 2
    assert result["metrics"]["commands_sent"] == 2
    assert result["metrics"]["optional_runtime_invocations"] == 0
    assert result["backend_readiness"]["isaac"]["invoked"] is False
    assert result["backend_readiness"]["ros2_sensor_sync"]["invoked"] is False
    assert result["backend_readiness"]["mavsdk_px4_sitl"]["invoked"] is False


def test_safe_deployment_defaults_remain_locked_down() -> None:
    demo_config = yaml.safe_load(
        Path("configs/gwm_navigation_demo.yaml").read_text(encoding="utf-8")
    )
    runtime_config = yaml.safe_load(
        Path("configs/runtime_validation.yaml").read_text(encoding="utf-8")
    )

    demo_deployment = demo_config["deployment"]
    readiness_deployment = runtime_config["runtime_validation"][
        "closed_loop_readiness"
    ]["deployment"]

    assert demo_deployment == {
        "mock": True,
        "sitl_enabled": False,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert readiness_deployment == demo_deployment


def test_real_hardware_flag_is_rejected_before_demo_execution() -> None:
    result = run_closed_loop_readiness(
        {
            "write_output": False,
            "deployment": {"real_hardware_enabled": True},
        }
    )

    assert result["status"] == "failed"
    assert "real_hardware_enabled=True" in result["reason"]
    assert result["demo_summary"]["executed"] is False
    assert result["safety_summary"]["real_hardware_enabled"] is True


def test_autonomous_real_flight_flag_is_rejected_before_demo_execution() -> None:
    result = run_closed_loop_readiness(
        {
            "write_output": False,
            "deployment": {"autonomous_real_flight_enabled": True},
        }
    )

    assert result["status"] == "failed"
    assert "autonomous_real_flight_enabled=True" in result["reason"]
    assert result["demo_summary"]["executed"] is False
    assert result["safety_summary"]["autonomous_real_flight_enabled"] is True


def test_require_prior_smokes_reports_missing_without_running_runtime_smokes(
    tmp_path: Path,
) -> None:
    missing_reports = {
        "isaac_runtime": str(tmp_path / "isaac_runtime_smoke.json"),
        "ros2_sensor_sync": str(tmp_path / "ros2_sensor_sync_smoke.json"),
        "mavsdk_px4_sitl": str(tmp_path / "mavsdk_sitl_smoke.json"),
    }

    result = run_closed_loop_readiness(
        {
            "closed_loop_readiness": {
                "write_output": False,
                "require_prior_smokes": True,
                "prior_smoke_reports": missing_reports,
            }
        }
    )

    assert result["status"] == "skipped"
    assert "Missing prior smoke reports" in result["reason"]
    assert result["demo_summary"]["executed"] is False
    assert result["metrics"]["prior_smoke_reports_missing"] == 3
    assert result["backend_readiness"]["isaac"]["invoked"] is False
    assert result["backend_readiness"]["ros2_sensor_sync"]["invoked"] is False
    assert result["backend_readiness"]["mavsdk_px4_sitl"]["invoked"] is False


def test_result_json_writes_to_temp_path_and_is_serializable(tmp_path: Path) -> None:
    output_path = tmp_path / "closed_loop_readiness.json"

    result = run_closed_loop_readiness(
        ClosedLoopReadinessConfig(
            steps=1,
            output_path=str(output_path),
            write_output=True,
        )
    )
    loaded = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert loaded["schema_version"] == "gwm_closed_loop_readiness_v1"
    assert loaded["metrics"]["commands_sent"] == 1
    json.dumps(result, sort_keys=True)


def test_runtime_gates_are_reported_without_runtime_connections(
    monkeypatch,
) -> None:
    monkeypatch.delenv("GWM_RUN_ISAAC_RUNTIME_TESTS", raising=False)
    monkeypatch.delenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", raising=False)
    monkeypatch.delenv("GWM_RUN_MAVSDK_SITL_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)

    result = run_closed_loop_readiness({"steps": 1, "write_output": False})

    assert result["runtime_gates"]["isaac"]["satisfied"] is False
    assert result["runtime_gates"]["ros2_sensor_sync"]["satisfied"] is False
    assert result["runtime_gates"]["mavsdk_px4_sitl"]["satisfied"] is False
    assert result["safety_summary"]["isaac_launch_invoked"] is False
    assert result["safety_summary"]["ros2_start_invoked"] is False
    assert result["safety_summary"]["mavsdk_connect_invoked"] is False
    assert result["safety_summary"]["px4_launch_invoked"] is False
    assert result["safety_summary"]["hardware_check_invoked"] is False


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_closed_loop_readiness.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "closed-loop GWM runtime readiness check" in result.stdout


def test_cli_no_output_run_works(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_closed_loop_readiness.py"),
            "--steps",
            "3",
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "closed_loop_readiness status=passed" in result.stdout
    assert "steps=3" in result.stdout
    assert "commands=3" in result.stdout
    assert output_path.exists() is False
