"""Tests for the v2-1A GWM-UAV-C2 mission dataclasses."""

from __future__ import annotations

import sys

import pytest

from src.c2 import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    AirspaceConstraint,
    FleetAsset,
    HumanApprovalDecision,
    HumanApprovalRecord,
    MetricSummary,
    MissionEvent,
    MissionRequest,
    MissionTask,
    MissionTaskStatus,
    PlannedRoute,
    ReplayFrame,
    RiskSignal,
    SafetyDecision,
    SafetyDecisionStatus,
    ThreatAssessment,
    ThreatRecommendation,
    UAVState,
)


def _sample_request() -> MissionRequest:
    return MissionRequest(
        request_id="req-001",
        operator_id="operator-a",
        objective="Inspect corridor alpha",
        priority=2,
        area={"zone": "alpha"},
        constraints={"max_altitude_m": 40},
        created_at=10.0,
        metadata={"source": "unit_test"},
    )


def test_mission_request_json_roundtrip() -> None:
    request = _sample_request()

    restored = MissionRequest.from_dict(request.to_dict())

    assert restored == request
    assert restored.to_dict()["area"] == {"zone": "alpha"}


def test_mission_task_validation() -> None:
    task = MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Survey safe corridor",
        status=MissionTaskStatus.ASSIGNED.value,
        priority=3,
        assigned_asset_id="uav-1",
    )

    assert task.to_dict()["status"] == MissionTaskStatus.ASSIGNED.value
    with pytest.raises(ValueError, match="status"):
        MissionTask(task_id="task-002", request_id="req-001", objective="x", status="flying")


def test_fleet_asset_availability() -> None:
    asset = FleetAsset(
        asset_id="uav-1",
        backend="mock",
        capabilities=["survey", "relay"],
        available=False,
        current_task_id="task-001",
    )

    assert asset.available is False
    assert asset.to_dict()["capabilities"] == ["survey", "relay"]
    with pytest.raises(ValueError, match="available"):
        FleetAsset(asset_id="uav-2", backend="mock", available="yes")


def test_uav_state_stale_detection() -> None:
    state = UAVState(
        asset_id="uav-1",
        timestamp=100.0,
        position={"x": 0.0, "y": 1.0, "z": -2.0},
        velocity={"vx": 0.0, "vy": 0.0, "vz": 0.0},
        battery=0.9,
        link_quality=0.8,
    )

    assert state.is_stale(now=103.0, max_age=5.0) is False
    assert state.is_stale(now=107.0, max_age=5.0) is True


def test_mission_event_json_safe_payload() -> None:
    event = MissionEvent(
        event_id="evt-001",
        event_type="mission.requested",
        timestamp=12.0,
        source="dispatcher",
        payload=_sample_request().to_dict(),
        correlation_id="req-001",
    )

    assert event.to_dict()["payload"]["request_id"] == "req-001"
    with pytest.raises(ValueError, match="forbidden credential-like key"):
        MissionEvent(
            event_id="evt-002",
            event_type="mission.updated",
            timestamp=13.0,
            source="dispatcher",
            payload={"api_key": "must-not-appear"},
        )


def test_risk_signal_defensive_category_accepted() -> None:
    signal = RiskSignal(
        signal_id="risk-001",
        category="communication degradation",
        severity=0.4,
        confidence=0.8,
        evidence={"link_quality": 0.45},
        timestamp=20.0,
    )

    assert signal.category in ALLOWED_DEFENSIVE_RISK_CATEGORIES
    assert signal.to_dict()["severity"] == 0.4


def test_forbidden_risk_category_rejected() -> None:
    with pytest.raises(ValueError, match="risk category is not allowed"):
        RiskSignal(
            signal_id="risk-002",
            category="non-defensive escalation",
            severity=0.5,
            confidence=0.7,
        )


def test_threat_assessment_requires_explanation() -> None:
    signal = RiskSignal(
        signal_id="risk-003",
        category="collision risk",
        severity=0.6,
        confidence=0.9,
    )

    assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="mission-001",
        risk_signals=[signal.to_dict()],
        total_risk=0.6,
        recommendation=ThreatRecommendation.REPLAN.value,
        explanation="Collision margin is low.",
    )

    assert assessment.to_dict()["recommendation"] == ThreatRecommendation.REPLAN.value
    with pytest.raises(ValueError, match="explanation is required"):
        ThreatAssessment(assessment_id="assessment-002", mission_id="mission-001", total_risk=0.1)


def test_airspace_constraint_altitude_validation() -> None:
    constraint = AirspaceConstraint(
        constraint_id="constraint-001",
        constraint_type="geofence",
        geometry={"type": "polygon", "vertices": [[0, 0], [1, 0], [1, 1]]},
        altitude_min=5.0,
        altitude_max=60.0,
    )

    assert constraint.active is True
    with pytest.raises(ValueError, match="altitude_min"):
        AirspaceConstraint(
            constraint_id="constraint-002",
            constraint_type="altitude",
            altitude_min=100.0,
            altitude_max=50.0,
        )


def test_planned_route_validation() -> None:
    route = PlannedRoute(
        route_id="route-001",
        task_id="task-001",
        waypoints=[{"x": 0.0, "y": 0.0, "z": -5.0}, {"x": 5.0, "y": 2.0, "z": -5.0}],
        score=0.82,
        risk_score=0.2,
        constraint_verdict="valid",
    )

    assert len(route.to_dict()["waypoints"]) == 2
    with pytest.raises(ValueError, match="waypoints must be non-empty"):
        PlannedRoute(route_id="route-002", task_id="task-001", waypoints=[], score=0.0, risk_score=0.0)


def test_safety_decision_requires_reason_when_blocked() -> None:
    decision = SafetyDecision(
        decision_id="safety-001",
        target_id="route-001",
        status=SafetyDecisionStatus.BLOCKED.value,
        reason="Route intersects an active geofence.",
        cbf_metadata={"minimum_margin_m": 3.0},
        requires_human_approval=True,
    )

    assert decision.to_dict()["status"] == SafetyDecisionStatus.BLOCKED.value
    with pytest.raises(ValueError, match="reason is required"):
        SafetyDecision(
            decision_id="safety-002",
            target_id="route-002",
            status=SafetyDecisionStatus.HOLD.value,
        )


def test_human_approval_record_validation() -> None:
    approval = HumanApprovalRecord(
        approval_id="approval-001",
        operator_id="operator-a",
        target_id="route-001",
        decision=HumanApprovalDecision.DEFERRED.value,
        notes="Awaiting updated risk assessment.",
    )

    assert approval.to_dict()["decision"] == HumanApprovalDecision.DEFERRED.value
    with pytest.raises(ValueError, match="decision"):
        HumanApprovalRecord(
            approval_id="approval-002",
            operator_id="operator-a",
            target_id="route-001",
            decision="ignored",
        )


def test_replay_frame_generation_shape() -> None:
    event = MissionEvent(
        event_id="evt-003",
        event_type="mission.assigned",
        timestamp=30.0,
        source="state_store",
        payload={"task_id": "task-001"},
    )
    frame = ReplayFrame(
        frame_id="frame-001",
        timestamp=31.0,
        mission_snapshot={"tasks": {"task-001": {"status": "assigned"}}},
        events=[event.to_dict()],
        risk_summary={"highest_category": "communication degradation"},
        route_summary={"route_count": 1},
        safety_summary={"requires_human_approval": False},
    )

    encoded = frame.to_dict()
    assert encoded["events"][0]["event_id"] == "evt-003"
    assert encoded["mission_snapshot"]["tasks"]["task-001"]["status"] == "assigned"


def test_metric_summary_aggregation_validation() -> None:
    summary = MetricSummary(
        mission_id="mission-001",
        event_count=3,
        risk_counts={"communication degradation": 1},
        replan_count=1,
        hold_count=0,
        approval_count=1,
        blocked_count=0,
    )

    assert summary.to_dict()["risk_counts"] == {"communication degradation": 1}
    with pytest.raises(ValueError, match="risk category is not allowed"):
        MetricSummary(mission_id="mission-002", risk_counts={"non-defensive escalation": 1})


def test_c2_imports_without_runtime_dependencies() -> None:
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
