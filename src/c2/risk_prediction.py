"""Defensive risk prediction core for GWM-UAV-C2."""

from __future__ import annotations

import copy
from math import isfinite
from typing import Dict, List, Optional, Sequence

from src.c2.event_bus import MissionEventBus
from src.c2.mission_types import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    MissionEvent,
    RiskSignal,
    ThreatAssessment,
    UAVState,
    ensure_json_safe_dict,
)
from src.c2.state_store import MissionStateStore


# Mock-first research thresholds only; these are not certified safety values.
LOW_LINK_QUALITY_THRESHOLD = 0.4
CRITICAL_LINK_QUALITY_THRESHOLD = 0.2
LOW_BATTERY_THRESHOLD = 0.25
CRITICAL_BATTERY_THRESHOLD = 0.1
STALE_STATE_MAX_AGE = 10.0
HIGH_WIND_SPEED_THRESHOLD = 12.0
CRITICAL_WIND_SPEED_THRESHOLD = 20.0
PROXIMITY_DISTANCE_THRESHOLD = 25.0
COLLISION_DISTANCE_THRESHOLD = 10.0

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
    """Mock-first defensive risk signal, rule mapping, and assessment factory."""

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

    def recommendation_for_risk(self, total_risk: float) -> str:
        """Return the deterministic defensive recommendation for a risk score."""

        risk = self._validated_unit_interval(total_risk, "total_risk")
        if risk == 0.0:
            return "continue"
        if risk < 0.4:
            return "continue"
        if risk < 0.7:
            return "replan"
        if risk < 0.9:
            return "hold"
        return "request_review"

    def evaluate_uav_state(
        self,
        state: UAVState,
        now: float = 0.0,
        max_age: float = STALE_STATE_MAX_AGE,
    ) -> List[RiskSignal]:
        """Map a mock UAV state fixture into defensive risk signals."""

        if not isinstance(state, UAVState):
            raise ValueError("state must be a UAVState")
        state.validate()
        now_value = self._validated_non_negative_number(now, "now")
        max_age_value = self._validated_non_negative_number(max_age, "max_age")
        signals: List[RiskSignal] = []

        if float(state.link_quality) < CRITICAL_LINK_QUALITY_THRESHOLD:
            signals.append(
                self._state_signal(
                    state=state,
                    category="communication degradation",
                    severity=0.8,
                    confidence=0.9,
                    rule="critical_link_quality",
                    values={"link_quality": state.link_quality},
                )
            )
        elif float(state.link_quality) < LOW_LINK_QUALITY_THRESHOLD:
            signals.append(
                self._state_signal(
                    state=state,
                    category="communication degradation",
                    severity=0.5,
                    confidence=0.8,
                    rule="low_link_quality",
                    values={"link_quality": state.link_quality},
                )
            )

        if float(state.battery) < CRITICAL_BATTERY_THRESHOLD:
            signals.append(
                self._state_signal(
                    state=state,
                    category="telemetry anomaly",
                    severity=0.8,
                    confidence=0.9,
                    rule="critical_battery",
                    values={"battery": state.battery},
                )
            )
        elif float(state.battery) < LOW_BATTERY_THRESHOLD:
            signals.append(
                self._state_signal(
                    state=state,
                    category="telemetry anomaly",
                    severity=0.5,
                    confidence=0.8,
                    rule="low_battery",
                    values={"battery": state.battery},
                )
            )

        state_age = now_value - float(state.timestamp)
        if state_age > max_age_value:
            signals.append(
                self.create_risk_signal(
                    category="telemetry anomaly",
                    severity=0.6,
                    confidence=0.8,
                    evidence={
                        "asset_id": state.asset_id,
                        "timestamp": state.timestamp,
                        "now": now_value,
                        "max_age": max_age_value,
                        "state_age": state_age,
                        "rule": "stale_uav_state",
                    },
                    timestamp=now_value,
                    metadata={"source": "evaluate_uav_state"},
                )
            )

        return signals

    def evaluate_event(self, event: MissionEvent) -> List[RiskSignal]:
        """Map a mock mission event fixture into defensive risk signals."""

        if not isinstance(event, MissionEvent):
            raise ValueError("event must be a MissionEvent")
        event.validate()
        signals: List[RiskSignal] = []

        boolean_rules = (
            ("gps_jump", "GPS spoofing risk", 0.7, 0.8, "gps_jump_fixture"),
            ("rf_noise", "GPS / RF jamming risk", 0.7, 0.8, "rf_noise_fixture"),
            ("sensor_fault", "sensor corruption", 0.6, 0.8, "sensor_fault_fixture"),
            (
                "geofence_violation",
                "geofence / no-fly-zone violation",
                0.8,
                0.9,
                "geofence_violation_fixture",
            ),
            (
                "hostile_uav_nearby",
                "hostile UAV proximity",
                0.7,
                0.8,
                "hostile_uav_nearby_fixture",
            ),
            ("route_conflict", "collision risk", 0.8, 0.8, "route_conflict_fixture"),
            ("high_wind", "weather or wind disturbance", 0.6, 0.8, "high_wind_fixture"),
            (
                "invalid_mission_command",
                "mission command anomaly",
                0.7,
                0.9,
                "invalid_mission_command_fixture",
            ),
        )
        for field, category, severity, confidence, rule in boolean_rules:
            value = self._lookup_event_value(event, field)
            if value is True:
                signals.append(
                    self._event_signal(
                        event=event,
                        category=category,
                        severity=severity,
                        confidence=confidence,
                        rule=rule,
                        field=field,
                        value=value,
                    )
                )

        wind_speed = self._numeric_event_value(event, "wind_speed")
        if wind_speed is not None and wind_speed >= CRITICAL_WIND_SPEED_THRESHOLD:
            signals.append(
                self._event_signal(
                    event=event,
                    category="weather or wind disturbance",
                    severity=0.8,
                    confidence=0.9,
                    rule="critical_wind_speed",
                    field="wind_speed",
                    value=wind_speed,
                )
            )
        elif wind_speed is not None and wind_speed >= HIGH_WIND_SPEED_THRESHOLD:
            signals.append(
                self._event_signal(
                    event=event,
                    category="weather or wind disturbance",
                    severity=0.5,
                    confidence=0.8,
                    rule="high_wind_speed",
                    field="wind_speed",
                    value=wind_speed,
                )
            )

        proximity_distance = self._numeric_event_value(event, "proximity_distance")
        if proximity_distance is not None and proximity_distance <= PROXIMITY_DISTANCE_THRESHOLD:
            signals.append(
                self._event_signal(
                    event=event,
                    category="hostile UAV proximity",
                    severity=0.6,
                    confidence=0.8,
                    rule="proximity_distance_threshold",
                    field="proximity_distance",
                    value=proximity_distance,
                )
            )

        collision_distance = self._numeric_event_value(event, "collision_distance")
        if collision_distance is not None and collision_distance <= COLLISION_DISTANCE_THRESHOLD:
            signals.append(
                self._event_signal(
                    event=event,
                    category="collision risk",
                    severity=0.8,
                    confidence=0.9,
                    rule="collision_distance_threshold",
                    field="collision_distance",
                    value=collision_distance,
                )
            )

        return signals

    def evaluate_context(self, context: Dict[str, object]) -> ThreatAssessment:
        """Evaluate a JSON-safe mock context into one threat assessment."""

        ensure_json_safe_dict(context, "context")
        mission_id = context.get("mission_id", "mission-context")
        if not isinstance(mission_id, str) or not mission_id.strip():
            raise ValueError("context.mission_id must be a non-empty string when provided")

        signals: List[RiskSignal] = []
        for signal_data in self._context_list(context, "risk_signals"):
            if not isinstance(signal_data, dict):
                raise ValueError("context.risk_signals entries must be dictionaries")
            signals.append(RiskSignal.from_dict(signal_data))

        for state_data in self._context_list(context, "uav_states"):
            if not isinstance(state_data, dict):
                raise ValueError("context.uav_states entries must be dictionaries")
            now = context.get("now", state_data.get("timestamp", 0.0))
            max_age = context.get("max_age", STALE_STATE_MAX_AGE)
            signals.extend(self.evaluate_uav_state(UAVState.from_dict(state_data), now=now, max_age=max_age))

        for event_data in self._context_list(context, "events"):
            if not isinstance(event_data, dict):
                raise ValueError("context.events entries must be dictionaries")
            signals.extend(self.evaluate_event(MissionEvent.from_dict(event_data)))

        timestamp = self._validated_non_negative_number(context.get("timestamp", 0.0), "context.timestamp")
        return self.create_threat_assessment(
            mission_id=mission_id,
            risk_signals=signals,
            timestamp=timestamp,
        )

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
            accepted_recommendation = self.recommendation_for_risk(total)
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

    def _state_signal(
        self,
        state: UAVState,
        category: str,
        severity: float,
        confidence: float,
        rule: str,
        values: Dict[str, object],
    ) -> RiskSignal:
        return self.create_risk_signal(
            category=category,
            severity=severity,
            confidence=confidence,
            evidence={
                "asset_id": state.asset_id,
                "timestamp": state.timestamp,
                "rule": rule,
                **copy.deepcopy(values),
            },
            timestamp=state.timestamp,
            metadata={"source": "evaluate_uav_state"},
        )

    def _event_signal(
        self,
        event: MissionEvent,
        category: str,
        severity: float,
        confidence: float,
        rule: str,
        field: str,
        value: object,
    ) -> RiskSignal:
        return self.create_risk_signal(
            category=category,
            severity=severity,
            confidence=confidence,
            evidence={
                "event_id": event.event_id,
                "event_type": event.event_type,
                "rule": rule,
                field: copy.deepcopy(value),
            },
            timestamp=event.timestamp,
            metadata={"source": "evaluate_event"},
        )

    @staticmethod
    def _lookup_event_value(event: MissionEvent, field: str) -> object:
        if field in event.payload:
            return event.payload[field]
        return event.metadata.get(field)

    def _numeric_event_value(self, event: MissionEvent, field: str) -> Optional[float]:
        value = self._lookup_event_value(event, field)
        if value is None:
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field} must be numeric when provided")
        if not isfinite(float(value)):
            raise ValueError(f"{field} must be finite")
        return float(value)

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

    @staticmethod
    def _context_list(context: Dict[str, object], key: str) -> List[object]:
        value = context.get(key, [])
        if value is None:
            return []
        if not isinstance(value, list):
            raise ValueError(f"context.{key} must be a list when provided")
        return value

    @staticmethod
    def _validated_non_negative_number(value: object, field_name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{field_name} must be a number")
        if not isfinite(float(value)) or float(value) < 0.0:
            raise ValueError(f"{field_name} must be a non-negative finite number")
        return float(value)

    @classmethod
    def _validated_unit_interval(cls, value: object, field_name: str) -> float:
        number = cls._validated_non_negative_number(value, field_name)
        if number > 1.0:
            raise ValueError(f"{field_name} must be in [0.0, 1.0]")
        return number
