"""Tests for Phase 6-E Isaac Sim / PX4 SITL bridge design readiness."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from src.runtime_validation import (
    DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH,
    FrameTransformPolicy,
    IsaacPX4BridgeDesignConfig,
    IsaacPX4BridgeDesignResult,
    build_isaac_px4_bridge_plan,
    run_isaac_px4_bridge_design,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_phase6_isaac_px4_bridge_design_exports_without_optional_runtimes() -> None:
    assert DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH.endswith(
        "isaac_px4_bridge_design.json"
    )
    assert FrameTransformPolicy is not None
    assert IsaacPX4BridgeDesignConfig is not None
    assert IsaacPX4BridgeDesignResult is not None
    assert build_isaac_px4_bridge_plan is not None
    assert run_isaac_px4_bridge_design is not None


def test_bridge_plan_contains_state_ownership_and_bridge_strategy() -> None:
    plan = build_isaac_px4_bridge_plan()

    assert plan["schema_version"] == "gwm_phase6_isaac_px4_bridge_design_v1"
    assert plan["bridge_strategy"]["primary"] == "mavsdk_lightweight"
    assert plan["bridge_strategy"]["future_option"] == "ros2_micro_xrce_dds"
    assert "simulated_world" in plan["state_ownership"]["isaac"]
    assert "offboard_mode_state" in plan["state_ownership"]["px4_sitl"]
    assert plan["command_paths"]["cbf_required_before_mavsdk_write"] is True
    assert plan["bridge_strategy"]["px4_launch_attempted"] is False


def test_default_dry_run_is_ready_and_does_not_invoke_runtime_modules(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_RUN_ISAAC_RUNTIME_TESTS", raising=False)
    monkeypatch.delenv("GWM_RUN_ROS2_SENSOR_SYNC_TESTS", raising=False)
    monkeypatch.delenv("GWM_RUN_MAVSDK_SITL_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)

    result = run_isaac_px4_bridge_design({"write_output": False})

    assert result["status"] == "ready"
    assert result["reason"] is None
    assert result["runtime_gates"]["isaac"]["GWM_ALLOW_OPTIONAL_RUNTIME"]["enabled"] is False
    assert result["data_paths"]["no_runtime_invoked_by_design_runner"] is True
    assert result["safety_policy"]["real_hardware_enabled"] is False
    assert result["safety_policy"]["autonomous_real_flight_enabled"] is False
    assert result["bridge_strategy"]["full_closed_loop_execution"] is False
    assert "isaacsim" not in sys.modules
    assert "rclpy" not in sys.modules
    assert "mavsdk" not in sys.modules


def test_coordinate_frame_policy_blocks_silent_conversion() -> None:
    result = run_isaac_px4_bridge_design({"write_output": False})
    frames = result["coordinate_frames"]
    policy = result["frame_transform_policy"]

    assert frames["project_frame"] == "project_default"
    assert frames["isaac_world_frame"] == "isaac_z_up"
    assert frames["px4_world_frame"] == "px4_ned"
    assert frames["mavsdk_command_frame"] == "px4_body_ned"
    assert frames["coordinate_conversion_applied"] is False
    assert policy["blocks_silent_conversion"] is True
    assert policy["silent_conversion_allowed"] is False
    assert policy["coupled_loop_execution_allowed"] is False


@pytest.mark.parametrize(
    ("deployment", "reason_fragment"),
    [
        (
            {
                "mock": False,
                "sitl_enabled": True,
                "real_hardware_enabled": True,
                "autonomous_real_flight_enabled": False,
            },
            "real_hardware_enabled=True",
        ),
        (
            {
                "mock": False,
                "sitl_enabled": True,
                "real_hardware_enabled": False,
                "autonomous_real_flight_enabled": True,
            },
            "autonomous_real_flight_enabled=True",
        ),
        (
            {
                "mock": True,
                "sitl_enabled": True,
                "real_hardware_enabled": False,
                "autonomous_real_flight_enabled": False,
            },
            "deployment.mock=False",
        ),
        (
            {
                "mock": False,
                "sitl_enabled": False,
                "real_hardware_enabled": False,
                "autonomous_real_flight_enabled": False,
            },
            "deployment.sitl_enabled=True",
        ),
    ],
)
def test_unsafe_deployment_flags_are_rejected(
    deployment: dict,
    reason_fragment: str,
) -> None:
    result = run_isaac_px4_bridge_design(
        {"write_output": False, "deployment": deployment}
    )

    assert result["status"] == "failed"
    assert reason_fragment in result["reason"]


def test_px4_launch_request_is_rejected() -> None:
    result = run_isaac_px4_bridge_design(
        {"write_output": False, "px4_launch_requested": True}
    )

    assert result["status"] == "failed"
    assert "refuses PX4 launch" in result["reason"]


def test_future_coupled_execution_requires_explicit_transforms() -> None:
    result = run_isaac_px4_bridge_design(
        {
            "write_output": False,
            "future_coupled_execution_requested": True,
            "frame_transform_policy": {"transforms_defined": False},
        }
    )

    assert result["status"] == "not_ready"
    assert "requires explicit Isaac/PX4 frame transforms" in result["reason"]


def test_require_prior_reports_missing_produces_not_ready(tmp_path: Path) -> None:
    result = run_isaac_px4_bridge_design(
        {
            "write_output": False,
            "require_prior_reports": True,
            "prior_reports": {
                "isaac_sensor_runtime": str(tmp_path / "missing_isaac.json"),
                "ros2_sim_sensor_bridge": str(tmp_path / "missing_ros2.json"),
                "px4_sitl_command_validation": str(tmp_path / "missing_px4.json"),
            },
        }
    )

    assert result["status"] == "not_ready"
    assert "Missing or unready prior Phase 6 reports" in result["reason"]
    assert result["report_readiness"]["all_ready"] is False
    assert result["report_readiness"]["reports"]["isaac_sensor_runtime"]["exists"] is False


def test_present_prior_reports_produce_ready(tmp_path: Path) -> None:
    report_paths = {
        "isaac_sensor_runtime": tmp_path / "isaac.json",
        "ros2_sim_sensor_bridge": tmp_path / "ros2.json",
        "px4_sitl_command_validation": tmp_path / "px4.json",
    }
    for path in report_paths.values():
        path.write_text(json.dumps({"status": "passed"}), encoding="utf-8")

    result = run_isaac_px4_bridge_design(
        {
            "write_output": False,
            "require_prior_reports": True,
            "prior_reports": {name: str(path) for name, path in report_paths.items()},
        }
    )

    assert result["status"] == "ready"
    assert result["report_readiness"]["all_ready"] is True
    assert all(report["ready"] for report in result["report_readiness"]["reports"].values())


def test_result_writes_json_to_temp_path(tmp_path: Path) -> None:
    output_path = tmp_path / "isaac_px4_bridge_design.json"

    result = run_isaac_px4_bridge_design(
        {"output_path": str(output_path), "write_output": True}
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "ready"
    assert payload["schema_version"] == "gwm_phase6_isaac_px4_bridge_design_v1"
    assert payload["bridge_strategy"]["primary"] == "mavsdk_lightweight"


def test_runtime_validation_config_contains_phase6e_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    phase6e = config["runtime_validation"]["isaac_px4_bridge_design"]

    assert phase6e["enabled"] is False
    assert phase6e["output_path"] == "outputs/runtime_validation/isaac_px4_bridge_design.json"
    assert phase6e["deployment"] == {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert phase6e["frame_transform_policy"]["coordinate_conversion_applied"] is False
    assert phase6e["frame_transform_policy"]["blocks_silent_conversion"] is True


def test_phase6_profile_points_to_isaac_px4_bridge_design_command() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["verification"]["phase6e_isaac_px4_bridge_design"] == (
        "python scripts/run_isaac_px4_bridge_design.py --require-prior-reports"
    )
    assert profile["verification"]["phase6e_report_path"] == (
        "outputs/runtime_validation/isaac_px4_bridge_design.json"
    )


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_isaac_px4_bridge_design.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Phase 6-E Isaac Sim / PX4 SITL bridge design readiness" in result.stdout


def test_cli_no_output_run_is_ready_and_writes_nothing(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_isaac_px4_bridge_design.py"),
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        env=dict(os.environ),
    )

    assert result.returncode == 0, result.stderr
    assert "isaac_px4_bridge_design status=ready" in result.stdout
    assert output_path.exists() is False
