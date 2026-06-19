"""Tests for the v2-6 C2 benchmark-readiness layer."""

from __future__ import annotations

import json

from src.c2 import C2BenchmarkReadinessBuilder, MissionEvent, build_c2_benchmark_readiness_report
from src.c2.benchmarking import SCHEMA_GROUPS, default_benchmark_events
import src.c2.benchmarking as benchmarking_module


def _profile(report: dict[str, object], name: str) -> dict[str, object]:
    profiles = report["profiles"]
    assert isinstance(profiles, list)
    for profile in profiles:
        if isinstance(profile, dict) and profile.get("name") == name:
            return profile
    raise AssertionError(f"missing profile: {name}")


def test_c2_benchmark_readiness_report_has_required_profiles() -> None:
    report = build_c2_benchmark_readiness_report()

    assert report["schema_version"] == "v2-6-c2-benchmark-readiness"
    assert [profile["name"] for profile in report["profiles"]] == [
        "mock",
        "isaac_readiness",
        "cosys_airsim_family_readiness",
    ]
    assert report["profile_count"] == 3


def test_c2_benchmark_compares_required_schema_groups() -> None:
    report = build_c2_benchmark_readiness_report()

    assert tuple(report["schema_groups"].keys()) == SCHEMA_GROUPS
    for profile in report["profiles"]:
        assert profile["schema_groups"] == list(SCHEMA_GROUPS)
        assert profile["schema_group_count"] == len(SCHEMA_GROUPS)


def test_c2_benchmark_schema_reference_includes_mission_risk_route_replay_dashboard_metrics() -> None:
    schema_groups = build_c2_benchmark_readiness_report()["schema_groups"]

    assert schema_groups["mission"]["schema_classes"] == ["MissionEvent", "MissionTask", "FleetAsset", "UAVState"]
    assert schema_groups["risk"]["schema_classes"] == ["RiskSignal", "ThreatAssessment"]
    assert schema_groups["route"]["schema_classes"] == ["PlannedRoute", "SafetyDecision", "HumanApprovalRecord"]
    assert schema_groups["replay"]["payload_schema_version"] == "v2-5A-dashboard-replay-payload"
    assert schema_groups["dashboard"]["audit_report_schema_version"] == "v2-5B-c2-replay-report"
    assert schema_groups["metrics"]["payload_schema_version"] == "v2-5B-c2-metrics-payload"


def test_c2_benchmark_profiles_are_readiness_only_for_optional_simulators() -> None:
    report = build_c2_benchmark_readiness_report()

    isaac = _profile(report, "isaac_readiness")
    airsim = _profile(report, "cosys_airsim_family_readiness")

    assert isaac["readiness_status"] == "schema_ready_runtime_gated"
    assert isaac["primary_runtime"] == "isaacsim"
    assert isaac["phase6_mainline"] is True
    assert airsim["primary_runtime"] == "cosysairsim"
    assert airsim["fallback_runtime"] == "airsim"
    assert airsim["runtime_family"] == "airsim"


def test_c2_benchmark_does_not_probe_or_launch_runtimes() -> None:
    report = build_c2_benchmark_readiness_report()

    for profile in report["profiles"]:
        assert profile["runtime_required_for_schema_check"] is False
        assert profile["runtime_availability_probed"] is False
        assert profile["runtime_connection_attempted"] is False
        assert profile["simulator_launched"] is False
        assert profile["hardware_connection_attempted"] is False


def test_c2_benchmark_disables_vehicle_command_and_route_upload() -> None:
    report = build_c2_benchmark_readiness_report()

    for profile in report["profiles"]:
        assert profile["vehicle_command_enabled"] is False
        assert profile["route_upload_enabled"] is False
        assert profile["safety_gate_required_for_future_execution"] is True
        assert profile["human_approval_required_for_future_execution"] is True


def test_c2_benchmark_makes_no_simulator_parity_or_production_claim() -> None:
    report = build_c2_benchmark_readiness_report()

    assert report["safety_summary"]["simulator_parity_claimed"] is False
    assert report["safety_summary"]["production_readiness_claimed"] is False
    for profile in report["profiles"]:
        assert profile["performance_parity_claimed"] is False


def test_c2_benchmark_compatibility_matrix_is_schema_only() -> None:
    matrix = build_c2_benchmark_readiness_report()["compatibility_matrix"]

    assert sorted(matrix) == ["cosys_airsim_family_readiness", "isaac_readiness", "mock"]
    for profile in matrix.values():
        assert sorted(profile) == sorted(SCHEMA_GROUPS)
        for group in profile.values():
            assert group == {
                "compatible": True,
                "comparison_scope": "schema_readiness_only",
                "runtime_required": False,
            }


def test_c2_benchmark_report_is_deterministic_and_json_safe() -> None:
    first = build_c2_benchmark_readiness_report()
    second = build_c2_benchmark_readiness_report()

    assert first == second
    json.dumps(first, allow_nan=False, sort_keys=True)


def test_c2_benchmark_accepts_custom_events_without_mutation() -> None:
    events = default_benchmark_events()
    event_dicts = [event.to_dict() for event in events]

    report = C2BenchmarkReadinessBuilder().build_readiness_report(events)

    assert report["source_fixture_summary"]["event_count"] == len(events)
    assert [event.to_dict() for event in events] == event_dicts


def test_c2_benchmark_default_events_are_mission_events() -> None:
    events = default_benchmark_events()

    assert all(isinstance(event, MissionEvent) for event in events)
    assert [event.event_type for event in events] == [
        "mission.task.created",
        "fleet.asset.updated",
        "risk.signal.created",
        "threat.assessment.created",
        "route.planned",
        "safety.decision.created",
        "human_approval.recorded",
    ]


def test_c2_benchmark_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "asyncio",
        "cosysairsim",
        "dash",
        "flask",
        "geopandas",
        "isaacsim",
        "matplotlib",
        "mavsdk",
        "message_filters",
        "numpy",
        "omni",
        "pandas",
        "plotly",
        "pxr",
        "rclpy",
        "shapely",
        "streamlit",
        "threading",
        "torch",
    }

    assert runtime_modules.isdisjoint(benchmarking_module.__dict__)
