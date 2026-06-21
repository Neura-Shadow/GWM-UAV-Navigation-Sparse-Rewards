"""Tests for the v2-1B in-memory mission state store."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    AirspaceConstraint,
    FleetAsset,
    HumanApprovalRecord,
    MissionEvent,
    MissionRequest,
    MissionStateStore,
    MissionTask,
    PlannedRoute,
    RiskSignal,
    SafetyDecision,
    ThreatAssessment,
    UAVState,
)


def _event(event_id: str, event_type: str, payload: dict, timestamp: float = 1.0) -> MissionEvent:
    return MissionEvent(
        event_id=event_id,
        event_type=event_type,
        timestamp=timestamp,
        source="unit_test",
        payload=payload,
    )


def _mission_request() -> MissionRequest:
    return MissionRequest(
        request_id="req-001",
        operator_id="operator-a",
        objective="Inspect zone alpha",
        priority=2,
        area={"zone": "alpha"},
    )


def _mission_task() -> MissionTask:
    return MissionTask(
        task_id="task-001",
        request_id="req-001",
        objective="Inspect zone alpha",
        status="pending",
        priority=2,
    )


def _fleet_asset() -> FleetAsset:
    return FleetAsset(
        asset_id="uav-1",
        backend="mock",
        capabilities=["survey"],
        available=True,
        health={"battery": 0.9},
    )


def test_state_store_apply_mission_requested() -> None:
    store = MissionStateStore()
    request = _mission_request()

    store.apply_event(_event("evt-001", "mission.requested", request.to_dict()))

    assert store.get_request("req-001") == request


def test_state_store_apply_task_created() -> None:
    store = MissionStateStore()
    task = _mission_task()

    store.apply_event(_event("evt-002", "mission.task.created", {"task": task.to_dict()}))

    assert store.get_task("task-001") == task


def test_state_store_apply_fleet_asset_registered() -> None:
    store = MissionStateStore()
    asset = _fleet_asset()

    store.apply_event(_event("evt-003", "fleet.asset.registered", asset.to_dict()))

    assert store.get_asset("uav-1") == asset


def test_state_store_apply_uav_state_updated() -> None:
    store = MissionStateStore()
    state = UAVState(
        asset_id="uav-1",
        timestamp=10.0,
        position={"x": 1.0, "y": 2.0, "z": -3.0},
        velocity={"vx": 0.0, "vy": 0.0, "vz": 0.0},
    )

    store.apply_event(_event("evt-004", "uav.state.updated", state.to_dict()))

    assert store.get_uav_state("uav-1") == state


def test_state_store_apply_risk_signal_created() -> None:
    store = MissionStateStore()
    signal = RiskSignal(
        signal_id="risk-001",
        category="communication degradation",
        severity=0.4,
        confidence=0.8,
        evidence={"link_quality": 0.5},
    )

    store.apply_event(_event("evt-005", "risk.signal.created", signal.to_dict()))

    assert store.snapshot()["risk_signals"]["risk-001"]["category"] == "communication degradation"


def test_state_store_unknown_event_preserved() -> None:
    store = MissionStateStore()
    event = _event("evt-006", "mission.note.created", {"note": "preserve me"})

    store.apply_event(event)

    assert store.list_events() == [event]
    assert store.snapshot()["events"][0]["event_type"] == "mission.note.created"


def test_state_store_invalid_known_payload_rejected() -> None:
    store = MissionStateStore()

    with pytest.raises(ValueError, match="MissionTask"):
        store.apply_event(_event("evt-007", "mission.task.created", {"request_id": "req-001"}))

    assert store.list_events() == []


def test_state_store_snapshot_restore() -> None:
    store = MissionStateStore()
    request = _mission_request()
    task = _mission_task()
    asset = _fleet_asset()
    assessment = ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="req-001",
        total_risk=0.2,
        recommendation="replan",
        explanation="Mock risk requires review.",
    )
    constraint = AirspaceConstraint(
        constraint_id="constraint-001",
        constraint_type="geofence",
        geometry={"zone": "alpha"},
    )
    route = PlannedRoute(
        route_id="route-001",
        task_id="task-001",
        waypoints=[{"x": 0.0, "y": 0.0, "z": -5.0}],
        score=0.7,
        risk_score=0.2,
    )
    decision = SafetyDecision(
        decision_id="decision-001",
        target_id="route-001",
        status="blocked",
        reason="Mock geofence check.",
    )
    approval = HumanApprovalRecord(
        approval_id="approval-001",
        operator_id="operator-a",
        target_id="route-001",
        decision="deferred",
    )
    events = [
        _event("evt-001", "mission.requested", request.to_dict(), 1.0),
        _event("evt-002", "mission.task.created", task.to_dict(), 2.0),
        _event("evt-003", "fleet.asset.registered", asset.to_dict(), 3.0),
        _event("evt-004", "threat.assessment.created", assessment.to_dict(), 4.0),
        _event("evt-005", "airspace.constraint.created", constraint.to_dict(), 5.0),
        _event("evt-006", "route.planned", route.to_dict(), 6.0),
        _event("evt-007", "safety.decision.created", decision.to_dict(), 7.0),
        _event("evt-008", "human.approval.recorded", approval.to_dict(), 8.0),
    ]
    for event in events:
        store.apply_event(event)

    restored = MissionStateStore()
    restored.restore(store.snapshot())

    assert restored.snapshot() == store.snapshot()
    assert restored.get_task("task-001") == task
    assert restored.get_asset("uav-1") == asset


def test_state_store_get_task() -> None:
    store = MissionStateStore()
    task = _mission_task()
    store.apply_event(_event("evt-008", "mission.task.created", task.to_dict()))

    returned = store.get_task("task-001")
    assert returned == task
    assert returned is not task


def test_state_store_get_asset() -> None:
    store = MissionStateStore()
    asset = _fleet_asset()
    store.apply_event(_event("evt-009", "fleet.asset.registered", asset.to_dict()))

    returned = store.get_asset("uav-1")
    assert returned == asset
    assert returned is not asset


def test_state_store_list_events_order() -> None:
    store = MissionStateStore()
    store.apply_event(_event("evt-010", "mission.requested", _mission_request().to_dict(), 1.0))
    store.apply_event(_event("evt-011", "mission.task.created", _mission_task().to_dict(), 2.0))

    assert [event.event_id for event in store.list_events()] == ["evt-010", "evt-011"]


def test_state_store_json_safe_snapshot() -> None:
    store = MissionStateStore()
    store.apply_event(_event("evt-012", "mission.requested", _mission_request().to_dict()))

    encoded = json.dumps(store.snapshot(), allow_nan=False, sort_keys=True)

    assert '"req-001"' in encoded


def test_state_store_imports_without_runtime_dependencies() -> None:
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
