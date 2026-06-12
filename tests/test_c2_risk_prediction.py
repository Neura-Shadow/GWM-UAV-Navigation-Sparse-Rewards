"""Tests for the v2-3A defensive risk predictor core."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    ALLOWED_RISK_RECOMMENDATIONS,
    DefensiveRiskPredictor,
    RiskSignal,
    ThreatAssessment,
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
    assert assessment.recommendation == "request_review"
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
