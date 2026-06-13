"""Integration tests for v2-3C defensive risk event replay."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    DefensiveRiskPredictor,
    MissionEvent,
    MissionEventBus,
    MissionReplayEngine,
    MissionStateStore,
    RiskSignal,
    ThreatAssessment,
)


def _predictor_stack() -> tuple[DefensiveRiskPredictor, MissionEventBus, MissionStateStore]:
    event_bus = MissionEventBus()
    state_store = MissionStateStore()
    predictor = DefensiveRiskPredictor(event_bus=event_bus, state_store=state_store)
    return predictor, event_bus, state_store


def _risk_signal(
    predictor: DefensiveRiskPredictor,
    category: str = "communication degradation",
    severity: float = 0.5,
    confidence: float = 0.8,
) -> RiskSignal:
    return predictor.create_risk_signal(
        category=category,
        severity=severity,
        confidence=confidence,
        evidence={"source": "integration_test", "rule": "fixture"},
        timestamp=10.0,
    )


def _threat_assessment(
    predictor: DefensiveRiskPredictor,
    signal: RiskSignal,
    mission_id: str = "mission-001",
) -> ThreatAssessment:
    return predictor.create_threat_assessment(
        mission_id=mission_id,
        risk_signals=[signal],
        timestamp=11.0,
    )


def _risk_events() -> list[MissionEvent]:
    predictor, event_bus, _ = _predictor_stack()
    first = _risk_signal(predictor, category="communication degradation")
    second = _risk_signal(predictor, category="collision risk", severity=0.8, confidence=0.8)
    assessment = _threat_assessment(predictor, second)

    predictor.publish_risk_signal(first)
    predictor.publish_risk_signal(second)
    predictor.publish_threat_assessment(assessment)
    return event_bus.history()


def test_risk_signal_event_emission() -> None:
    predictor, _, _ = _predictor_stack()
    signal = _risk_signal(predictor)

    event = predictor.make_risk_signal_event(signal)

    assert event.event_id == "risk-event-000001"
    assert event.event_type == "risk.signal.created"
    assert event.payload == signal.to_dict()
    assert event.metadata["source"] == "defensive_risk_predictor"
    assert event.metadata["risk_category"] == "communication degradation"


def test_threat_assessment_event_emission() -> None:
    predictor, _, _ = _predictor_stack()
    signal = _risk_signal(predictor, category="collision risk")
    assessment = _threat_assessment(predictor, signal)

    event = predictor.make_threat_assessment_event(assessment)

    assert event.event_id == "risk-event-000001"
    assert event.event_type == "threat.assessment.created"
    assert event.payload == assessment.to_dict()
    assert event.metadata["mission_id"] == "mission-001"
    assert event.metadata["recommendation"] == "replan"
    assert event.metadata["total_risk"] == pytest.approx(0.4)


def test_risk_predictor_publish_risk_signal_applies_state_store() -> None:
    predictor, event_bus, state_store = _predictor_stack()
    signal = _risk_signal(predictor)

    event = predictor.publish_risk_signal(signal)

    assert event.event_type == "risk.signal.created"
    assert [stored.event_id for stored in event_bus.history()] == ["risk-event-000001"]
    assert list(state_store.risk_signals) == ["risk-signal-000001"]
    assert state_store.risk_signals[signal.signal_id].category == "communication degradation"


def test_risk_predictor_publish_threat_assessment_applies_state_store() -> None:
    predictor, event_bus, state_store = _predictor_stack()
    signal = _risk_signal(predictor, category="collision risk")
    assessment = _threat_assessment(predictor, signal)

    event = predictor.publish_threat_assessment(assessment)

    assert event.event_type == "threat.assessment.created"
    assert [stored.event_id for stored in event_bus.history()] == ["risk-event-000001"]
    assert list(state_store.threat_assessments) == ["threat-assessment-000001"]
    assert state_store.threat_assessments[assessment.assessment_id].recommendation == "replan"


def test_risk_state_store_snapshot_restore() -> None:
    predictor, _, state_store = _predictor_stack()
    signal = _risk_signal(predictor, category="sensor corruption")
    assessment = _threat_assessment(predictor, signal)

    predictor.publish_risk_signal(signal)
    predictor.publish_threat_assessment(assessment)
    snapshot = state_store.snapshot()
    restored = MissionStateStore()
    restored.restore(snapshot)

    assert restored.snapshot() == snapshot
    assert restored.risk_signals["risk-signal-000001"].category == "sensor corruption"
    assert restored.threat_assessments["threat-assessment-000001"].mission_id == "mission-001"


def test_risk_event_order_is_deterministic() -> None:
    events = _risk_events()

    assert [event.event_id for event in events] == [
        "risk-event-000001",
        "risk-event-000002",
        "risk-event-000003",
    ]
    assert [event.event_type for event in events] == [
        "risk.signal.created",
        "risk.signal.created",
        "threat.assessment.created",
    ]


def test_risk_replay_generates_frame_per_event() -> None:
    result = MissionReplayEngine().replay(_risk_events())

    assert len(result.frames) == 3
    assert [frame.frame_id for frame in result.frames] == [
        "frame-000001",
        "frame-000002",
        "frame-000003",
    ]


def test_risk_replay_final_snapshot_contains_risk_data() -> None:
    result = MissionReplayEngine().replay(_risk_events())
    snapshot = result.final_snapshot

    assert sorted(snapshot["risk_signals"]) == ["risk-signal-000001", "risk-signal-000002"]
    assert sorted(snapshot["threat_assessments"]) == ["threat-assessment-000001"]
    assert snapshot["events"][-1]["event_type"] == "threat.assessment.created"


def test_risk_replay_metric_summary_counts_categories() -> None:
    result = MissionReplayEngine().replay(_risk_events())

    assert result.metrics.event_count == 3
    assert result.metrics.risk_counts == {
        "communication degradation": 1,
        "collision risk": 1,
    }


def test_risk_replay_metric_summary_is_deterministic() -> None:
    events = _risk_events()
    first = MissionReplayEngine().replay(events).metrics.to_dict()
    second = MissionReplayEngine().replay(events).metrics.to_dict()

    assert first == second


def test_risk_replay_rejects_forbidden_category() -> None:
    event = MissionEvent(
        event_id="risk-event-999999",
        event_type="risk.signal.created",
        timestamp=1.0,
        source="unit_test",
        payload={
            "signal_id": "risk-signal-999999",
            "category": "offensive attack targeting",
            "severity": 0.5,
            "confidence": 0.5,
            "evidence": {"source": "unit_test"},
        },
    )

    with pytest.raises(ValueError, match="risk category is not allowed"):
        MissionReplayEngine().replay([event])


def test_risk_integration_outputs_are_json_safe() -> None:
    predictor, _, state_store = _predictor_stack()
    signal = _risk_signal(predictor, category="GPS spoofing risk")
    assessment = _threat_assessment(predictor, signal)

    predictor.publish_risk_signal(signal)
    predictor.publish_threat_assessment(assessment)
    replay = MissionReplayEngine().replay(state_store.list_events())

    json.dumps(signal.to_dict(), allow_nan=False)
    json.dumps(assessment.to_dict(), allow_nan=False)
    json.dumps(state_store.snapshot(), allow_nan=False)
    json.dumps(replay.to_dict(), allow_nan=False)


def test_risk_integration_imports_without_runtime_dependencies() -> None:
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
