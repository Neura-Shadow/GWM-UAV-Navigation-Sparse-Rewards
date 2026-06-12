"""Defensive risk prediction core for GWM-UAV-C2."""

from __future__ import annotations

import copy
from typing import Dict, List, Optional, Sequence

from src.c2.event_bus import MissionEventBus
from src.c2.mission_types import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    MissionEvent,
    RiskSignal,
    ThreatAssessment,
)
from src.c2.state_store import MissionStateStore


ALLOWED_RISK_RECOMMENDATIONS = (
    "continue",
    "hold",
    "replan",
    "request_review",
)

_FORBIDDEN_RISK_TERMS = (
    "offensive",
    "attack",
    "targeting",
    "payload release",
    "weapon",
    "pursue",
    "intercept",
    "disable",
    "engage",
)


class DefensiveRiskPredictor:
    """Mock-first defensive risk signal and assessment factory."""

    def __init__(
        self,
        event_bus: Optional[MissionEventBus] = None,
        state_store: Optional[MissionStateStore] = None,
    ) -> None:
        self.event_bus = event_bus or MissionEventBus()
        self.state_store = state_store
        self._risk_signal_counter = 0
        self._assessment_counter = 0
        self._event_counter = 0

    def validate_category(self, category: str) -> str:
        if not isinstance(category, str) or not category.strip():
            raise ValueError("risk category must be a non-empty string")
        category = category.strip()
        if category in ALLOWED_DEFENSIVE_RISK_CATEGORIES:
            return category
        lowered = category.lower()
        if any(term in lowered for term in _FORBIDDEN_RISK_TERMS):
            raise ValueError(f"risk category is not allowed: {category!r}")
        raise ValueError(f"risk category is not allowed: {category!r}")

    def validate_recommendation(self, recommendation: str) -> str:
        if not isinstance(recommendation, str) or not recommendation.strip():
            raise ValueError("recommendation must be a non-empty string")
        recommendation = recommendation.strip()
        if recommendation in ALLOWED_RISK_RECOMMENDATIONS:
            return recommendation
        lowered = recommendation.lower()
        if any(term in lowered for term in _FORBIDDEN_RISK_TERMS) or lowered in {"jam", "spoof"}:
            raise ValueError(f"recommendation is not allowed: {recommendation!r}")
        raise ValueError(f"recommendation is not allowed: {recommendation!r}")

    def create_risk_signal(
        self,
        category: str,
        severity: float,
        confidence: float,
        evidence: Dict[str, object],
        timestamp: float = 0.0,
        metadata: Optional[Dict[str, object]] = None,
    ) -> RiskSignal:
        accepted_category = self.validate_category(category)
        self._risk_signal_counter += 1
        return RiskSignal(
            signal_id=f"risk-signal-{self._risk_signal_counter:06d}",
            category=accepted_category,
            severity=severity,
            confidence=confidence,
            evidence=copy.deepcopy(evidence),
            timestamp=timestamp,
            metadata=copy.deepcopy(metadata or {}),
        )

    def create_threat_assessment(
        self,
        mission_id: str,
        risk_signals: Sequence[RiskSignal],
        recommendation: Optional[str] = None,
        timestamp: float = 0.0,
    ) -> ThreatAssessment:
        if not isinstance(mission_id, str) or not mission_id.strip():
            raise ValueError("mission_id must be a non-empty string")
        signals = self._validated_signals(risk_signals)
        total = self.total_risk(signals)
        if recommendation is None:
            accepted_recommendation = "continue" if total == 0.0 else "request_review"
        else:
            accepted_recommendation = self.validate_recommendation(recommendation)
        explanation = self.explain_assessment(signals, total)
        self._assessment_counter += 1
        return ThreatAssessment(
            assessment_id=f"threat-assessment-{self._assessment_counter:06d}",
            mission_id=mission_id.strip(),
            risk_signals=[signal.to_dict() for signal in signals],
            total_risk=total,
            recommendation=accepted_recommendation,
            explanation=explanation,
            timestamp=timestamp,
        )

    def total_risk(self, risk_signals: Sequence[RiskSignal]) -> float:
        signals = self._validated_signals(risk_signals)
        if not signals:
            return 0.0
        total = max(float(signal.severity) * float(signal.confidence) for signal in signals)
        return max(0.0, min(1.0, total))

    def explain_assessment(self, risk_signals: Sequence[RiskSignal], total_risk: float) -> str:
        signals = self._validated_signals(risk_signals)
        if total_risk == 0.0:
            return "No defensive risk detected."
        categories = ", ".join(signal.category for signal in signals)
        return f"Defensive risk categories: {categories}. total_risk={total_risk:.3f}."

    def make_risk_signal_event(self, signal: RiskSignal) -> MissionEvent:
        if not isinstance(signal, RiskSignal):
            raise ValueError("signal must be a RiskSignal")
        signal.validate()
        return self._make_event("risk.signal.created", signal.to_dict())

    def make_threat_assessment_event(self, assessment: ThreatAssessment) -> MissionEvent:
        if not isinstance(assessment, ThreatAssessment):
            raise ValueError("assessment must be a ThreatAssessment")
        assessment.validate()
        return self._make_event("threat.assessment.created", assessment.to_dict())

    def _make_event(self, event_type: str, payload: Dict[str, object]) -> MissionEvent:
        self._event_counter += 1
        return MissionEvent(
            event_id=f"risk-event-{self._event_counter:06d}",
            event_type=event_type,
            timestamp=float(payload.get("timestamp", self._event_counter)),
            source="defensive_risk_predictor",
            payload=copy.deepcopy(payload),
        )

    @staticmethod
    def _validated_signals(risk_signals: Sequence[RiskSignal]) -> List[RiskSignal]:
        if isinstance(risk_signals, (str, bytes)) or not isinstance(risk_signals, Sequence):
            raise ValueError("risk_signals must be a sequence of RiskSignal objects")
        signals: List[RiskSignal] = []
        for index, signal in enumerate(risk_signals):
            if not isinstance(signal, RiskSignal):
                raise ValueError(f"risk_signals[{index}] must be a RiskSignal")
            signal.validate()
            signals.append(copy.deepcopy(signal))
        return signals
