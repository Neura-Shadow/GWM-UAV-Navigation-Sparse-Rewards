"""Tests for the v2-3A defensive risk predictor core."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    ALLOWED_RISK_RECOMMENDATIONS,
    DefensiveRiskPredictor,
    MissionEvent,
    RiskSignal,
    ThreatAssessment,
    UAVState,
)
from src.c2.risk_prediction import (
    CRITICAL_BATTERY_THRESHOLD,
    CRITICAL_LINK_QUALITY_THRESHOLD,
    HIGH_WIND_SPEED_THRESHOLD,
    LOW_BATTERY_THRESHOLD,
    LOW_LINK_QUALITY_THRESHOLD,
)


def _predictor() -> DefensiveRiskPredictor:
    return DefensiveRiskPredictor()


def _sample_signal(
    predictor: DefensiveRiskPredictor | None = None,
    category: str = "communication degradation",
    severity: float = 0.5,
    confidence: float = 0.8,
) -> RiskSignal:
    predictor = predictor or _predictor()
    return predictor.create_risk_signal(
        category=category,
        severity=severity,
        confidence=confidence,
        evidence={"source": "unit_test", "sample_count": 3},
        timestamp=12.0,
        metadata={"frame": "project_default"},
    )


def test_risk_predictor_accepts_allowed_categories() -> None:
    predictor = _predictor()

    for category in ALLOWED_DEFENSIVE_RISK_CATEGORIES:
        assert predictor.validate_category(category) == category


def test_risk_predictor_rejects_forbidden_category() -> None:
    predictor = _predictor()

    with pytest.raises(ValueError, match="risk category is not allowed"):
        predictor.validate_category("offensive attack targeting")


def test_risk_predictor_rejects_unknown_category() -> None:
    predictor = _predictor()

    with pytest.raises(ValueError, match="risk category is not allowed"):
        predictor.validate_category("unknown operational concern")


def test_risk_predictor_accepts_allowed_recommendations() -> None:
    predictor = _predictor()

    for recommendation in ALLOWED_RISK_RECOMMENDATIONS:
        assert predictor.validate_recommendation(recommendation) == recommendation


def test_risk_predictor_rejects_offensive_recommendation() -> None:
    predictor = _predictor()

    for recommendation in ("attack", "pursue", "intercept", "disable", "jam", "spoof"):
        with pytest.raises(ValueError, match="recommendation is not allowed"):
            predictor.validate_recommendation(recommendation)


def test_risk_signal_generation_json_safe() -> None:
    signal = _sample_signal()
    encoded = signal.to_dict()

    assert encoded["signal_id"] == "risk-signal-000001"
    assert encoded["category"] == "communication degradation"
    assert encoded["metadata"]["frame"] == "project_default"
    json.dumps(encoded, allow_nan=False)


def test_risk_signal_rejects_invalid_severity() -> None:
    predictor = _predictor()

    with pytest.raises(ValueError, match="severity"):
        predictor.create_risk_signal(
            category="communication degradation",
            severity=1.1,
            confidence=0.8,
            evidence={"source": "unit_test"},
        )


def test_risk_signal_rejects_invalid_confidence() -> None:
    predictor = _predictor()

    with pytest.raises(ValueError, match="confidence"):
        predictor.create_risk_signal(
            category="communication degradation",
            severity=0.4,
            confidence=-0.1,
            evidence={"source": "unit_test"},
        )


def test_risk_signal_ids_are_deterministic() -> None:
    predictor = _predictor()
    first = _sample_signal(predictor)
    second = _sample_signal(predictor, category="collision risk")

    assert first.signal_id == "risk-signal-000001"
    assert second.signal_id == "risk-signal-000002"


def test_threat_assessment_generation_no_risk() -> None:
    predictor = _predictor()

    assessment = predictor.create_threat_assessment(mission_id="mission-001", risk_signals=[])

    assert isinstance(assessment, ThreatAssessment)
    assert assessment.assessment_id == "threat-assessment-000001"
    assert assessment.total_risk == 0.0
    assert assessment.recommendation == "continue"
    assert assessment.explanation == "No defensive risk detected."


def test_threat_assessment_generation_with_risk() -> None:
    predictor = _predictor()
    signal = _sample_signal(predictor, category="collision risk", severity=0.5, confidence=0.8)

    assessment = predictor.create_threat_assessment(
        mission_id="mission-001",
        risk_signals=[signal],
    )

    assert assessment.total_risk == pytest.approx(0.4)
    assert assessment.recommendation == "replan"
    assert "collision risk" in assessment.explanation
    assert assessment.risk_signals[0]["signal_id"] == "risk-signal-000001"


def test_threat_assessment_total_risk_is_bounded() -> None:
    predictor = _predictor()
    low = _sample_signal(predictor, severity=0.2, confidence=0.5)
    high = _sample_signal(predictor, category="telemetry anomaly", severity=1.0, confidence=1.0)

    assert predictor.total_risk([low, high]) == 1.0


def test_threat_assessment_explanation_required_for_nonzero_risk() -> None:
    predictor = _predictor()
    signal = _sample_signal(predictor, category="sensor corruption", severity=0.7, confidence=0.8)

    assessment = predictor.create_threat_assessment("mission-001", [signal])

    assert assessment.explanation
    assert "total_risk=0.560" in assessment.explanation


def test_threat_assessment_rejects_offensive_recommendation() -> None:
    predictor = _predictor()
    signal = _sample_signal(predictor)

    with pytest.raises(ValueError, match="recommendation is not allowed"):
        predictor.create_threat_assessment(
            mission_id="mission-001",
            risk_signals=[signal],
            recommendation="weapon control",
        )


def test_risk_predictor_imports_without_runtime_dependencies() -> None:
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


def test_recommendation_policy_continue_zero() -> None:
    assert _predictor().recommendation_for_risk(0.0) == "continue"


def test_recommendation_policy_continue_low_nonzero() -> None:
    assert _predictor().recommendation_for_risk(0.39) == "continue"


def test_recommendation_policy_replan() -> None:
    assert _predictor().recommendation_for_risk(0.4) == "replan"
    assert _predictor().recommendation_for_risk(0.69) == "replan"


def test_recommendation_policy_hold() -> None:
    assert _predictor().recommendation_for_risk(0.7) == "hold"
    assert _predictor().recommendation_for_risk(0.89) == "hold"


def test_recommendation_policy_request_review() -> None:
    assert _predictor().recommendation_for_risk(0.9) == "request_review"


def test_low_link_quality_maps_to_communication_degradation() -> None:
    state = UAVState(
        asset_id="uav-1",
        timestamp=20.0,
        link_quality=LOW_LINK_QUALITY_THRESHOLD - 0.01,
    )

    signals = _predictor().evaluate_uav_state(state, now=20.0)

    assert len(signals) == 1
    assert signals[0].category == "communication degradation"
    assert signals[0].severity == pytest.approx(0.5)
    assert signals[0].confidence == pytest.approx(0.8)
    assert signals[0].evidence["rule"] == "low_link_quality"


def test_critical_link_quality_increases_risk() -> None:
    state = UAVState(
        asset_id="uav-1",
        timestamp=20.0,
        link_quality=CRITICAL_LINK_QUALITY_THRESHOLD - 0.01,
    )

    signals = _predictor().evaluate_uav_state(state, now=20.0)

    assert signals[0].category == "communication degradation"
    assert signals[0].severity == pytest.approx(0.8)
    assert signals[0].confidence == pytest.approx(0.9)
    assert signals[0].severity * signals[0].confidence > 0.7


def test_low_battery_maps_to_telemetry_anomaly() -> None:
    state = UAVState(
        asset_id="uav-1",
        timestamp=20.0,
        battery=LOW_BATTERY_THRESHOLD - 0.01,
    )

    signals = _predictor().evaluate_uav_state(state, now=20.0)

    assert len(signals) == 1
    assert signals[0].category == "telemetry anomaly"
    assert signals[0].evidence["rule"] == "low_battery"


def test_critical_battery_increases_risk() -> None:
    state = UAVState(
        asset_id="uav-1",
        timestamp=20.0,
        battery=CRITICAL_BATTERY_THRESHOLD - 0.01,
    )

    signals = _predictor().evaluate_uav_state(state, now=20.0)

    assert signals[0].category == "telemetry anomaly"
    assert signals[0].severity == pytest.approx(0.8)
    assert signals[0].confidence == pytest.approx(0.9)


def test_stale_uav_state_maps_to_telemetry_anomaly() -> None:
    state = UAVState(asset_id="uav-1", timestamp=5.0)

    signals = _predictor().evaluate_uav_state(state, now=20.1, max_age=10.0)

    assert len(signals) == 1
    assert signals[0].category == "telemetry anomaly"
    assert signals[0].evidence["rule"] == "stale_uav_state"
    assert signals[0].evidence["state_age"] == pytest.approx(15.1)


def _event(payload: dict[str, object] | None = None, metadata: dict[str, object] | None = None) -> MissionEvent:
    return MissionEvent(
        event_id="event-001",
        event_type="mission.fixture",
        timestamp=30.0,
        source="unit_test",
        payload=payload or {},
        metadata=metadata or {},
    )


def _single_event_signal(payload: dict[str, object] | None = None, metadata: dict[str, object] | None = None) -> RiskSignal:
    signals = _predictor().evaluate_event(_event(payload=payload, metadata=metadata))
    assert len(signals) == 1
    return signals[0]


def test_gps_jump_maps_to_gps_spoofing_risk() -> None:
    signal = _single_event_signal({"gps_jump": True})

    assert signal.category == "GPS spoofing risk"
    assert signal.evidence["rule"] == "gps_jump_fixture"


def test_rf_noise_maps_to_jamming_risk() -> None:
    signal = _single_event_signal({"rf_noise": True})

    assert signal.category == "GPS / RF jamming risk"
    assert signal.evidence["rule"] == "rf_noise_fixture"


def test_sensor_fault_maps_to_sensor_corruption() -> None:
    signal = _single_event_signal({"sensor_fault": True})

    assert signal.category == "sensor corruption"


def test_geofence_violation_maps_to_geofence_risk() -> None:
    signal = _single_event_signal({"geofence_violation": True})

    assert signal.category == "geofence / no-fly-zone violation"


def test_hostile_uav_fixture_maps_to_proximity_risk() -> None:
    signal = _single_event_signal({"hostile_uav_nearby": True})

    assert signal.category == "hostile UAV proximity"


def test_route_conflict_maps_to_collision_risk() -> None:
    signal = _single_event_signal({"route_conflict": True})

    assert signal.category == "collision risk"


def test_high_wind_maps_to_weather_risk() -> None:
    signal = _single_event_signal(metadata={"wind_speed": HIGH_WIND_SPEED_THRESHOLD})

    assert signal.category == "weather or wind disturbance"
    assert signal.evidence["wind_speed"] == pytest.approx(HIGH_WIND_SPEED_THRESHOLD)


def test_invalid_mission_command_maps_to_command_anomaly() -> None:
    signal = _single_event_signal({"invalid_mission_command": True})

    assert signal.category == "mission command anomaly"


def test_numeric_proximity_and_collision_rules() -> None:
    signals = _predictor().evaluate_event(
        _event(payload={"proximity_distance": 20.0, "collision_distance": 8.0})
    )

    assert [signal.category for signal in signals] == ["hostile UAV proximity", "collision risk"]


def test_evaluate_context_aggregates_signals() -> None:
    predictor = _predictor()
    provided_signal = _sample_signal(predictor, category="sensor corruption").to_dict()
    context = {
        "mission_id": "mission-001",
        "risk_signals": [provided_signal],
        "uav_states": [
            {
                "asset_id": "uav-1",
                "timestamp": 5.0,
                "link_quality": 0.3,
            }
        ],
        "events": [_event(payload={"route_conflict": True}).to_dict()],
        "now": 6.0,
    }

    assessment = predictor.evaluate_context(context)

    categories = [signal["category"] for signal in assessment.risk_signals]
    assert categories == ["sensor corruption", "communication degradation", "collision risk"]
    assert assessment.mission_id == "mission-001"


def test_evaluate_context_uses_recommendation_policy() -> None:
    predictor = _predictor()
    context = {
        "mission_id": "mission-001",
        "events": [_event(payload={"gps_jump": True}).to_dict()],
    }

    assessment = predictor.evaluate_context(context)

    assert assessment.total_risk == pytest.approx(0.56)
    assert assessment.recommendation == "replan"


def test_rule_based_outputs_are_json_safe() -> None:
    predictor = _predictor()
    state_signals = predictor.evaluate_uav_state(
        UAVState(asset_id="uav-1", timestamp=1.0, link_quality=0.1, battery=0.05),
        now=20.0,
    )
    event_signals = predictor.evaluate_event(
        _event(
            payload={
                "gps_jump": True,
                "rf_noise": True,
                "sensor_fault": True,
                "geofence_violation": True,
                "hostile_uav_nearby": True,
                "route_conflict": True,
                "invalid_mission_command": True,
            },
            metadata={"wind_speed": 22.0},
        )
    )
    assessment = predictor.create_threat_assessment("mission-001", state_signals + event_signals)

    json.dumps([signal.to_dict() for signal in state_signals + event_signals], allow_nan=False)
    json.dumps(assessment.to_dict(), allow_nan=False)


def test_rule_based_mapping_imports_without_runtime_dependencies() -> None:
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
