"""Tests for the v2-1C mock replay and metrics layer."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from src.c2 import (
    FleetAsset,
    HumanApprovalRecord,
    MissionEvent,
    MissionReplayEngine,
    MissionReplayResult,
    MissionRequest,
    MissionTask,
    PlannedRoute,
    RiskSignal,
    SafetyDecision,
    ThreatAssessment,
)


def _event(event_id: str, event_type: str, payload: dict, timestamp: float = 1.0) -> MissionEvent:
    return MissionEvent(
        event_id=event_id,
        event_type=event_type,
        timestamp=timestamp,
        source="unit_test",
        payload=payload,
    )


def _base_events() -> list[MissionEvent]:
    request = MissionRequest(
        request_id="req-001",
        operator_id="operator-a",
        objective="Inspect zone alpha",
        priority=2,
        area={"zone": "alpha"},
    )
    task = MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Inspect zone alpha",
        status="pending",
        priority=2,
    )
    asset = FleetAsset(
        asset_id="uav-1",
        backend="mock",
        capabilities=["survey"],
        available=True,
    )
    return [
        _event("evt-001", "mission.requested", request.to_dict(), 1.0),
        _event("evt-002", "mission.task.created", task.to_dict(), 2.0),
        _event("evt-003", "fleet.asset.registered", asset.to_dict(), 3.0),
    ]


def _metric_events() -> list[MissionEvent]:
    blocked_task = MissionTask(
        task_id="task-002",
        request_id="req-001",
        objective="Hold near boundary",
        status="blocked",
        priority=1,
    )
    risk_one = RiskSignal(
        signal_id="risk-001",
        category="communication degradation",
        severity=0.4,
        confidence=0.8,
    )
    risk_two = RiskSignal(
        signal_id="risk-002",
        category="communication degradation",
        severity=0.3,
        confidence=0.7,
    )
    replan_assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="req-001",
        total_risk=0.5,
        recommendation="replan",
        explanation="Route needs a defensive replan.",
    )
    hold_assessment = ThreatAssessment(
        assessment_id="assessment-002",
        mission_id="req-001",
        total_risk=0.6,
        recommendation="hold",
        explanation="Mock link risk requests hold.",
    )
    blocked_route = PlannedRoute(
        route_id="route-001",
        task_id="task-002",
        waypoints=[{"x": 0.0, "y": 0.0, "z": -5.0}],
        score=0.1,
        risk_score=0.9,
        constraint_verdict="blocked",
    )
    blocked_decision = SafetyDecision(
        decision_id="decision-001",
        target_id="route-001",
        status="blocked",
        reason="Mock geofence violation.",
    )
    approved_decision = SafetyDecision(
        decision_id="decision-002",
        target_id="route-002",
        status="approved",
    )
    approval = HumanApprovalRecord(
        approval_id="approval-001",
        operator_id="operator-a",
        target_id="route-002",
        decision="approved",
    )
    return _base_events() + [
        _event("evt-004", "mission.task.updated", blocked_task.to_dict(), 4.0),
        _event("evt-005", "risk.signal.created", risk_one.to_dict(), 5.0),
        _event("evt-006", "risk.signal.created", risk_two.to_dict(), 6.0),
        _event("evt-007", "threat.assessment.created", replan_assessment.to_dict(), 7.0),
        _event("evt-008", "threat.assessment.created", hold_assessment.to_dict(), 8.0),
        _event("evt-009", "route.planned", blocked_route.to_dict(), 9.0),
        _event("evt-010", "safety.decision.created", blocked_decision.to_dict(), 10.0),
        _event("evt-011", "safety.decision.created", approved_decision.to_dict(), 11.0),
        _event("evt-012", "human.approval.recorded", approval.to_dict(), 12.0),
    ]


def test_replay_generates_frame_per_event() -> None:
    events = _base_events()

    result = MissionReplayEngine().replay(events)

    assert len(result.frames) == len(events)


def test_replay_preserves_event_order() -> None:
    events = _base_events()

    result = MissionReplayEngine().replay(events)

    assert [event.event_id for event in result.events] == ["evt-001", "evt-002", "evt-003"]
    assert [frame.events[0]["event_id"] for frame in result.frames] == ["evt-001", "evt-002", "evt-003"]


def test_replay_frame_ids_are_deterministic() -> None:
    result = MissionReplayEngine().replay(_base_events())

    assert [frame.frame_id for frame in result.frames] == [
        "frame-000001",
        "frame-000002",
        "frame-000003",
    ]


def test_replay_final_snapshot_json_safe() -> None:
    result = MissionReplayEngine().replay(_base_events())

    encoded = json.dumps(result.final_snapshot, allow_nan=False, sort_keys=True)

    assert '"req-001"' in encoded


def test_metric_summary_event_count() -> None:
    result = MissionReplayEngine().replay(_base_events())

    assert result.metrics.event_count == 3
    assert result.metrics.mission_id == "req-001"


def test_metric_summary_risk_counts() -> None:
    result = MissionReplayEngine().replay(_metric_events())

    assert result.metrics.risk_counts == {"communication degradation": 2}
    assert result.frames[-1].risk_summary["risk_signal_count"] == 2


def test_metric_summary_replan_hold_approval_blocked_counts() -> None:
    result = MissionReplayEngine().replay(_metric_events())

    assert result.metrics.replan_count == 1
    assert result.metrics.hold_count == 1
    assert result.metrics.approval_count == 2
    assert result.metrics.blocked_count == 3
    assert result.frames[-1].safety_summary["blocked_count"] == 3


def test_unknown_event_type_preserved() -> None:
    unknown = _event("evt-999", "mission.note.created", {"note": "preserve"})

    result = MissionReplayEngine().replay([unknown])

    assert result.metrics.event_count == 1
    assert result.events[0] == unknown
    assert result.frames[0].events[0]["event_type"] == "mission.note.created"


def test_invalid_known_payload_rejected() -> None:
    engine = MissionReplayEngine()

    with pytest.raises(ValueError, match="MissionTask"):
        engine.replay([_event("evt-bad", "mission.task.created", {"request_id": "req-001"})])


def test_replay_result_json_roundtrip() -> None:
    result = MissionReplayEngine().replay(_metric_events())

    restored = MissionReplayResult.from_dict(result.to_dict())

    assert restored.to_dict() == result.to_dict()


def test_replay_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "cosysairsim",
        "isaacsim",
        "mavsdk",
        "message_filters",
        "omni",
        "pxr",
        "rclpy",
    }

    assert runtime_modules.isdisjoint(sys.modules)


def test_replay_does_not_write_files(tmp_path: Path) -> None:
    MissionReplayEngine().replay(_metric_events())

    assert list(tmp_path.iterdir()) == []
