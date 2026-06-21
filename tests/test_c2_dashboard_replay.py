"""Tests for the v2-5A dashboard replay payload core."""

from __future__ import annotations

import copy
import json

from src.c2 import (
    DashboardReplayBuilder,
    FleetAsset,
    MissionEvent,
    MissionReplayEngine,
    MissionRequest,
    MissionTask,
    PlannedRoute,
    ReplayFrame,
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
        metadata={"fixture": "dashboard_replay"},
    )


def _base_events() -> list[MissionEvent]:
    request = MissionRequest(
        request_id="req-001",
        operator_id="operator-a",
        objective="Inspect mock area",
        priority=2,
        area={"zone": "alpha"},
    )
    task = MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Inspect mock area",
        status="pending",
        priority=2,
    )
    asset = FleetAsset(
        asset_id="uav-001",
        backend="mock",
        capabilities=["survey"],
    )
    return [
        _event("evt-001", "mission.requested", request.to_dict(), 1.0),
        _event("evt-002", "mission.task.created", task.to_dict(), 2.0),
        _event("evt-003", "fleet.asset.registered", asset.to_dict(), 3.0),
    ]


def _risk_event() -> MissionEvent:
    signal = RiskSignal(
        signal_id="risk-001",
        category="communication degradation",
        severity=0.4,
        confidence=0.9,
        timestamp=4.0,
    )
    return _event("evt-004", "risk.signal.created", signal.to_dict(), 4.0)


def _threat_event() -> MissionEvent:
    assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="req-001",
        risk_signals=[{"signal_id": "risk-001", "category": "communication degradation"}],
        total_risk=0.5,
        recommendation="replan",
        explanation="Mock communication degradation asks for review.",
        timestamp=5.0,
    )
    return _event("evt-005", "threat.assessment.created", assessment.to_dict(), 5.0)


def _route_event() -> MissionEvent:
    route = PlannedRoute(
        route_id="route-001",
        task_id="task-001",
        waypoints=[{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}],
        score=2.0,
        risk_score=0.25,
        constraint_verdict="warning",
        metadata={"selected": True, "executable": False},
    )
    return _event("evt-006", "route.planned", route.to_dict(), 6.0)


def _safety_event() -> MissionEvent:
    decision = SafetyDecision(
        decision_id="decision-001",
        target_id="route-001",
        status="hold",
        reason="Mock audit hold.",
    )
    return _event("evt-007", "safety.decision.created", decision.to_dict(), 7.0)


def _events_with_risk_and_route() -> list[MissionEvent]:
    return _base_events() + [_risk_event(), _threat_event(), _route_event(), _safety_event()]


def test_dashboard_replay_builds_timeline() -> None:
    timeline = DashboardReplayBuilder().build_timeline(_base_events())

    assert len(timeline) == 3
    assert timeline[0]["event_id"] == "evt-001"
    assert timeline[0]["event_type"] == "mission.requested"
    assert timeline[0]["family"] == "mission"


def test_dashboard_replay_preserves_event_order() -> None:
    timeline = DashboardReplayBuilder().build_timeline(_base_events())

    assert [entry["event_id"] for entry in timeline] == ["evt-001", "evt-002", "evt-003"]
    assert [entry["index"] for entry in timeline] == [0, 1, 2]


def test_dashboard_replay_preserves_unknown_events() -> None:
    unknown = _event("evt-999", "operator.note.created", {"note": "keep this visible"}, 9.0)

    payload = DashboardReplayBuilder().build_replay_payload([unknown])

    assert payload["timeline"][0]["event_type"] == "operator.note.created"
    assert payload["timeline"][0]["family"] == "unknown"
    assert payload["summary"]["unknown_event_count"] == 1


def test_dashboard_replay_classifies_event_families() -> None:
    builder = DashboardReplayBuilder()

    assert builder.event_family("mission.requested") == "mission"
    assert builder.event_family("fleet.asset.registered") == "fleet"
    assert builder.event_family("uav.state.updated") == "uav"
    assert builder.event_family("risk.signal.created") == "risk"
    assert builder.event_family("threat.assessment.created") == "threat"
    assert builder.event_family("route.planned") == "route"
    assert builder.event_family("safety.decision.created") == "safety"
    assert builder.event_family("human_approval.recorded") == "human_approval"
    assert builder.event_family("operator.note.created") == "unknown"


def test_dashboard_event_payload_summary_includes_ids() -> None:
    summary = DashboardReplayBuilder().payload_summary(_route_event().payload)

    assert summary["route_id"] == "route-001"
    assert summary["task_id"] == "task-001"
    assert summary["constraint_verdict"] == "warning"
    assert summary["risk_score"] == 0.25
    assert "payload_keys" in summary


def test_dashboard_event_payload_summary_redacts_sensitive_keys() -> None:
    payload = {
        "request_id": "req-001",
        "token": "secret-token",
        "nested": {"api_key": "secret-key", "safe": "visible"},
    }

    summary = DashboardReplayBuilder().payload_summary(payload)

    assert summary["request_id"] == "req-001"
    assert summary["token"] == "<redacted>"
    assert "secret-token" not in json.dumps(summary, sort_keys=True)
    assert "secret-key" not in json.dumps(summary, sort_keys=True)


def test_dashboard_snapshot_is_json_safe() -> None:
    replay = MissionReplayEngine().replay(_events_with_risk_and_route())

    snapshot = DashboardReplayBuilder().build_dashboard_snapshot(replay.final_snapshot)

    json.dumps(snapshot, allow_nan=False, sort_keys=True)
    assert "raw_keys" in snapshot


def test_dashboard_snapshot_counts_known_collections() -> None:
    replay = MissionReplayEngine().replay(_events_with_risk_and_route())

    snapshot = DashboardReplayBuilder().build_dashboard_snapshot(replay.final_snapshot)

    assert snapshot["mission_requests"] == {"count": 1, "ids": ["req-001"]}
    assert snapshot["mission_tasks"] == {"count": 1, "ids": ["task-001"]}
    assert snapshot["fleet_assets"] == {"count": 1, "ids": ["uav-001"]}
    assert snapshot["risk_signals"] == {"count": 1, "ids": ["risk-001"]}
    assert snapshot["threat_assessments"] == {"count": 1, "ids": ["assessment-001"]}
    assert snapshot["planned_routes"] == {"count": 1, "ids": ["route-001"]}
    assert snapshot["safety_decisions"] == {"count": 1, "ids": ["decision-001"]}


def test_dashboard_replay_payload_is_deterministic() -> None:
    events = _events_with_risk_and_route()
    builder = DashboardReplayBuilder()

    first = builder.build_replay_payload(events)
    second = builder.build_replay_payload(events)

    assert first == second


def test_dashboard_replay_payload_includes_audit_boundary() -> None:
    payload = DashboardReplayBuilder().build_replay_payload(_base_events())

    assert payload["audit_boundary"] == {
        "read_only": True,
        "command_free": True,
        "runtime_free": True,
    }


def test_dashboard_replay_payload_preserves_route_events() -> None:
    payload = DashboardReplayBuilder().build_replay_payload(_events_with_risk_and_route())

    route_entries = [entry for entry in payload["timeline"] if entry["family"] == "route"]

    assert len(route_entries) == 1
    assert route_entries[0]["payload_summary"]["route_id"] == "route-001"
    assert payload["final_snapshot"]["planned_routes"]["ids"] == ["route-001"]


def test_dashboard_replay_payload_preserves_risk_events() -> None:
    payload = DashboardReplayBuilder().build_replay_payload(_events_with_risk_and_route())

    risk_entries = [entry for entry in payload["timeline"] if entry["family"] == "risk"]

    assert len(risk_entries) == 1
    assert risk_entries[0]["payload_summary"]["signal_id"] == "risk-001"
    assert risk_entries[0]["payload_summary"]["category"] == "communication degradation"


def test_dashboard_filter_timeline_by_event_type() -> None:
    builder = DashboardReplayBuilder()
    timeline = builder.build_timeline(_events_with_risk_and_route())

    filtered = builder.filter_timeline(timeline, ["route.planned"])

    assert [entry["event_id"] for entry in filtered] == ["evt-006"]
    assert builder.filter_timeline(timeline, []) == timeline


def test_dashboard_format_replay_frame() -> None:
    frame = MissionReplayEngine().replay(_base_events()).frames[0]

    formatted = DashboardReplayBuilder().format_replay_frame(frame)

    assert formatted["frame_id"] == "frame-000001"
    assert formatted["event_id"] == "evt-001"
    assert formatted["event_type"] == "mission.requested"
    assert formatted["snapshot_summary"]["mission_requests"]["ids"] == ["req-001"]


def test_dashboard_builder_does_not_mutate_inputs() -> None:
    events = _events_with_risk_and_route()
    event_dicts = [event.to_dict() for event in events]
    replay = MissionReplayEngine().replay(events)
    snapshot = replay.final_snapshot
    snapshot_before = copy.deepcopy(snapshot)
    frame = replay.frames[0]
    frame_before = frame.to_dict()

    builder = DashboardReplayBuilder()
    builder.build_timeline(events)
    builder.build_replay_payload(events)
    builder.build_dashboard_snapshot(snapshot)
    builder.format_replay_frame(frame)

    assert [event.to_dict() for event in events] == event_dicts
    assert snapshot == snapshot_before
    assert frame.to_dict() == frame_before


def test_dashboard_replay_imports_without_runtime_dependencies() -> None:
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
