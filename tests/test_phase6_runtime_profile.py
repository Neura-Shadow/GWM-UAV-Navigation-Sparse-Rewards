"""Tests for the Phase 6-A pure-simulation runtime profile."""

from __future__ import annotations

from pathlib import Path

import yaml


PROFILE_PATH = Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml")


def _load_profile() -> dict:
    return yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))


def test_phase6_pure_simulation_profile_exists_and_names_real_stack() -> None:
    profile = _load_profile()
    runtime_stack = profile["runtime_stack"]

    assert profile["schema_version"] == "gwm_phase6_pure_sim_runtime_profile_v1"
    assert profile["profile"]["name"] == "pure_sim_isaac_px4_ros2"
    assert profile["profile"]["mode"] == "pure_simulation_sitl_only"
    assert runtime_stack["isaac"]["required"] is True
    assert runtime_stack["ros2"]["required"] is True
    assert runtime_stack["px4_sitl"]["required"] is True
    assert runtime_stack["mavsdk"]["required"] is True
    assert runtime_stack["gwm_wam_planner"]["required"] is True
    assert runtime_stack["safety_gate"]["implementation"] == "ControlBarrierFunction"


def test_profile_distinguishes_safe_default_from_pure_simulation_deployment() -> None:
    profile = _load_profile()

    assert profile["default_safe_deployment"] == {
        "mock": True,
        "sitl_enabled": False,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert profile["pure_simulation_deployment"] == {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }


def test_existing_default_configs_remain_safe() -> None:
    deployment = yaml.safe_load(Path("configs/deployment.yaml").read_text(encoding="utf-8"))
    demo = yaml.safe_load(Path("configs/gwm_navigation_demo.yaml").read_text(encoding="utf-8"))
    runtime = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))

    expected = {
        "mock": True,
        "sitl_enabled": False,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }

    assert {key: deployment["deployment"][key] for key in expected} == expected
    assert demo["deployment"] == expected
    assert (
        runtime["runtime_validation"]["closed_loop_readiness"]["deployment"]
        == expected
    )


def test_profile_lists_required_runtime_gates() -> None:
    gates = _load_profile()["required_env_gates"]

    assert gates["isaac"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ISAAC_RUNTIME_TESTS",
    ]
    assert gates["ros2"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_ROS2_SENSOR_SYNC_TESTS",
    ]
    assert gates["mavsdk_px4_sitl"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_MAVSDK_SITL_TESTS",
        "GWM_ALLOW_SITL_COMMANDS",
    ]
    assert gates["optional_live_ros2_topics"] == ["GWM_ROS2_LIVE_TOPICS"]
    assert gates["future_px4_launch_approval"] == ["GWM_ALLOW_PX4_LAUNCH"]


def test_profile_refuses_hardware_autonomous_and_automatic_px4_launch() -> None:
    profile = _load_profile()
    refusals = profile["refusal_rules"]
    processes = {
        process["name"]: process
        for process in profile["required_external_processes"]
    }

    assert refusals["refuse_real_hardware_enabled"] is True
    assert refusals["refuse_autonomous_real_flight_enabled"] is True
    assert refusals["refuse_physical_flight_controller_connection"] is True
    assert refusals["refuse_px4_launch_without_later_explicit_approval"] is True
    assert profile["runtime_stack"]["px4_sitl"]["externally_started"] is True
    assert profile["runtime_stack"]["px4_sitl"]["automatic_launch_allowed"] is False
    assert processes["px4_sitl"]["started_by_repo_by_default"] is False


def test_profile_runtime_artifacts_are_not_committable() -> None:
    artifacts = _load_profile()["runtime_artifacts"]

    assert artifacts["output_dir"] == "outputs/runtime_validation"
    assert artifacts["commit_reports"] is False
    assert artifacts["commit_logs"] is False
    assert artifacts["commit_rosbags"] is False
    assert artifacts["commit_sitl_artifacts"] is False
    assert artifacts["commit_isaac_artifacts"] is False


def test_phase6_documentation_states_simulation_only_boundaries() -> None:
    docs = Path("docs/phase6_pure_simulation_runtime.md").read_text(encoding="utf-8")

    assert "NVIDIA Isaac Sim or Isaac Lab" in docs
    assert "PX4 SITL must be started externally" in docs
    assert "real_hardware_enabled: true" in docs
    assert "autonomous_real_flight_enabled: true" in docs
    assert "not real hardware flight validation" in docs
    assert "Production readiness" in docs or "production readiness" in docs
