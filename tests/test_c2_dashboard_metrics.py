"""Tests for the v2-5B dashboard metrics and audit report builder."""

from __future__ import annotations

import json

from src.c2 import (
    C2MetricsExporter,
    C2ReplayReportBuilder,
    DashboardReplayBuilder,
    FleetAsset,
    HumanApprovalRecord,
    MissionEvent,
    MissionTask,
    PlannedRoute,
    RiskSignal,
    SafetyDecision,
    ThreatAssessment,
)
import src.c2.dashboard_replay as dashboard_module


def _event(event_id: str, event_type: str, payload: dict[str, object], timestamp: float = 1.0) -> MissionEvent:
    return MissionEvent(
        event_id=event_id,
        event_type=event_type,
        timestamp=timestamp,
        source="unit_test",
        payload=payload,
    )


def _task_event(status: str = "assigned") -> MissionEvent:
    task = MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Inspect mock area",
        status=status,
        priority=2,
        assigned_asset_id="uav-001",
    )
    return _event("evt-001", "mission.task.created", task.to_dict(), 1.0)


def _fleet_events() -> list[MissionEvent]:
    available_asset = FleetAsset(
        asset_id="uav-001",
        backend="mock",
        capabilities=["survey"],
        available=True,
    )
    assigned_asset = FleetAsset(
        asset_id="uav-002",
        backend="mock",
        capabilities=["relay"],
        available=True,
        current_task_id="task-001",
    )
    unavailable_asset = FleetAsset(
        asset_id="uav-003",
        backend="mock",
        capabilities=["survey"],
        available=False,
    )
    return [
        _event("evt-002", "fleet.asset.registered", available_asset.to_dict(), 2.0),
        _event("evt-003", "fleet.asset.updated", assigned_asset.to_dict(), 3.0),
        _event("evt-004", "fleet.asset.updated", unavailable_asset.to_dict(), 4.0),
    ]


def _risk_event(category: str = "communication degradation") -> MissionEvent:
    signal = RiskSignal(
        signal_id="risk-001",
        category=category,
        severity=0.4,
        confidence=0.8,
        timestamp=5.0,
    )
    return _event("evt-005", "risk.signal.created", signal.to_dict(), 5.0)


def _risk_unknown_event() -> MissionEvent:
    return _event("evt-006", "risk.signal.created", {"signal_id": "risk-unknown"}, 6.0)


def _threat_event(recommendation: str = "replan") -> MissionEvent:
    assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="req-001",
        total_risk=0.5,
        recommendation=recommendation,
        explanation="Mock defensive assessment.",
        timestamp=7.0,
    )
    return _event("evt-007", "threat.assessment.created", assessment.to_dict(), 7.0)


def _route_event(verdict: str = "blocked") -> MissionEvent:
    route = PlannedRoute(
        route_id="route-001",
        task_id="task-001",
        waypoints=[{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}],
        score=2.0,
        risk_score=0.2,
        constraint_verdict=verdict,
    )
    return _event("evt-008", "route.planned", route.to_dict(), 8.0)


def _route_unknown_event() -> MissionEvent:
    return _event("evt-009", "route.planned", {"route_id": "route-unknown", "task_id": "task-001"}, 9.0)


def _safety_event(status: str = "hold") -> MissionEvent:
    decision = SafetyDecision(
        decision_id="decision-001",
        target_id="route-001",
        status=status,
        reason="Mock safety hold.",
    )
    return _event("evt-010", "safety.decision.created", decision.to_dict(), 10.0)


def _approval_event() -> MissionEvent:
    approval = HumanApprovalRecord(
        approval_id="approval-001",
        operator_id="operator-a",
        target_id="route-001",
        decision="approved",
    )
    return _event("evt-011", "human_approval.recorded", approval.to_dict(), 11.0)


def _approval_boolean_event() -> MissionEvent:
    return _event("evt-012", "human_approval.checked", {"approval_id": "approval-002", "approved": False}, 12.0)


def _unknown_event() -> MissionEvent:
    return _event("evt-999", "operator.note.created", {"note": "preserve"}, 99.0)


def _events() -> list[MissionEvent]:
    return [
        _task_event(),
        *_fleet_events(),
        _risk_event(),
        _risk_unknown_event(),
        _threat_event(),
        _route_event(),
        _route_unknown_event(),
        _safety_event(),
        _approval_event(),
        _approval_boolean_event(),
        _unknown_event(),
    ]


def _report_events() -> list[MissionEvent]:
    return [
        _task_event(),
        *_fleet_events(),
        _risk_event(),
        _threat_event(),
        _route_event(),
        _safety_event(),
        _approval_event(),
        _unknown_event(),
    ]


def _payloads() -> tuple[dict[str, object], dict[str, object]]:
    events = _report_events()
    replay_payload = DashboardReplayBuilder().build_replay_payload(events)
    exporter = C2MetricsExporter()
    metrics_payload = exporter.build_metrics_payload(exporter.summarize_events(events))
    return replay_payload, metrics_payload


def test_dashboard_metrics_counts_event_types() -> None:
    counts = C2MetricsExporter().build_event_type_counts(_events())

    assert counts["fleet.asset.updated"] == 2
    assert counts["route.planned"] == 2
    assert counts["operator.note.created"] == 1


def test_dashboard_metrics_counts_risk_categories() -> None:
    metrics = C2MetricsExporter().build_risk_metrics(_events())

    assert metrics["risk_counts"] == {"communication degradation": 1, "unknown": 1}


def test_dashboard_metrics_counts_route_verdicts() -> None:
    metrics = C2MetricsExporter().build_route_metrics(_events())

    assert metrics["route_count"] == 2
    assert metrics["route_verdict_counts"] == {"blocked": 1, "unknown": 1}


def test_dashboard_metrics_counts_task_statuses() -> None:
    metrics = C2MetricsExporter().build_task_fleet_metrics(_events())

    assert metrics["task_status_counts"] == {"assigned": 1}


def test_dashboard_metrics_counts_fleet_assignments() -> None:
    metrics = C2MetricsExporter().build_task_fleet_metrics(_events())

    assert metrics["fleet_assignment_counts"] == {
        "assigned": 1,
        "available": 1,
        "unavailable": 1,
    }


def test_dashboard_metrics_counts_safety_decisions() -> None:
    metrics = C2MetricsExporter().build_safety_approval_metrics(_events())

    assert metrics["safety_decision_counts"] == {"hold": 1}


def test_dashboard_metrics_counts_human_approvals() -> None:
    metrics = C2MetricsExporter().build_safety_approval_metrics(_events())

    assert metrics["human_approval_counts"] == {"approved": 1, "rejected": 1}


def test_dashboard_metrics_preserves_unknown_events() -> None:
    exporter = C2MetricsExporter()
    payload = exporter.build_metrics_payload(exporter.summarize_events(_events()))

    assert payload["unknown_event_count"] == 1
    assert payload["event_type_counts"]["operator.note.created"] == 1


def test_dashboard_metrics_payload_is_deterministic() -> None:
    exporter = C2MetricsExporter()

    first = exporter.build_metrics_payload(exporter.summarize_events(_events()))
    second = exporter.build_metrics_payload(exporter.summarize_events(_events()))

    assert first == second


def test_dashboard_metrics_payload_is_json_safe() -> None:
    exporter = C2MetricsExporter()
    payload = exporter.build_metrics_payload(exporter.summarize_events(_events()))

    json.dumps(payload, allow_nan=False, sort_keys=True)


def test_dashboard_report_builds_json() -> None:
    replay_payload, metrics_payload = _payloads()

    report = C2ReplayReportBuilder().build_json_report(replay_payload, metrics_payload)

    assert report["schema_version"] == "v2-5B-c2-replay-report"
    assert report["scope_and_safety"]["read_only"] is True
    assert report["scope_and_safety"]["no_vehicle_control"] is True


def test_dashboard_report_builds_markdown() -> None:
    replay_payload, metrics_payload = _payloads()

    markdown = C2ReplayReportBuilder().build_markdown_report(replay_payload, metrics_payload)

    assert markdown.startswith("# Mission Replay Summary")
    assert "event_count" in markdown
    assert "route_verdict_counts" in markdown


def test_dashboard_report_contains_required_sections() -> None:
    replay_payload, metrics_payload = _payloads()
    markdown = C2ReplayReportBuilder().build_markdown_report(replay_payload, metrics_payload)

    for heading in (
        "# Mission Replay Summary",
        "## Event Timeline",
        "## Task and Fleet Status",
        "## Risk Timeline",
        "## Route Timeline",
        "## Safety and Human Approval Timeline",
        "## Metrics Summary",
        "## Scope and Safety Notes",
    ):
        assert heading in markdown


def test_dashboard_report_excludes_credentials() -> None:
    replay_payload, metrics_payload = _payloads()
    replay_payload["credentials"] = {"token": "secret-token-value"}
    metrics_payload["runtime_log"] = "C:/private/runtime.log"

    report = C2ReplayReportBuilder().build_json_report(replay_payload, metrics_payload)
    markdown = C2ReplayReportBuilder().build_markdown_report(replay_payload, metrics_payload)
    encoded = json.dumps(report, sort_keys=True)

    assert "secret-token-value" not in encoded
    assert "C:/private/runtime.log" not in encoded
    assert "secret-token-value" not in markdown
    assert "C:/private/runtime.log" not in markdown
    assert report["replay"]["credentials"] == "<redacted>"
    assert report["metrics"]["runtime_log"] == "<redacted>"


def test_dashboard_report_redacts_sensitive_values_recursively() -> None:
    value = {
        "safe": "visible",
        "nested": [
            {
                "private_key": "secret-key",
                "file_path": "D:/private/file.txt",
                "child": {"private_hostname": "host.local"},
            }
        ],
    }

    redacted = C2ReplayReportBuilder().redact_sensitive_values(value)

    assert redacted["safe"] == "visible"
    assert redacted["nested"][0]["private_key"] == "<redacted>"
    assert redacted["nested"][0]["file_path"] == "<redacted>"
    assert redacted["nested"][0]["child"]["private_hostname"] == "<redacted>"


def test_dashboard_report_is_deterministic() -> None:
    replay_payload, metrics_payload = _payloads()
    builder = C2ReplayReportBuilder()

    first_json = builder.build_json_report(replay_payload, metrics_payload)
    second_json = builder.build_json_report(replay_payload, metrics_payload)
    first_markdown = builder.build_markdown_report(replay_payload, metrics_payload)
    second_markdown = builder.build_markdown_report(replay_payload, metrics_payload)

    assert first_json == second_json
    assert first_markdown == second_markdown


def test_dashboard_metrics_imports_without_runtime_dependencies() -> None:
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

    assert runtime_modules.isdisjoint(dashboard_module.__dict__)
