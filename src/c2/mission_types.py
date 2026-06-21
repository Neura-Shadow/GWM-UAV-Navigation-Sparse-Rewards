"""Pure-Python mission dataclasses for the GWM-UAV-C2 extension.

The models in this module are deliberately runtime-free. They use only
standard-library types, validate JSON-safe payloads, and reject offensive or
credential-like data in mission records.
"""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Any, Dict, List, Optional


class MissionTaskStatus(str, Enum):
    """Allowed mission task lifecycle states."""

    PENDING = "pending"
    ASSIGNED = "assigned"
    BLOCKED = "blocked"
    COMPLETED = "completed"
    CANCELLED = "cancelled"


class ThreatRecommendation(str, Enum):
    """Allowed defensive threat assessment recommendations."""

    CONTINUE = "continue"
    HOLD = "hold"
    REPLAN = "replan"
    REQUEST_REVIEW = "request_review"


class RouteConstraintVerdict(str, Enum):
    """Allowed route constraint verdicts."""

    VALID = "valid"
    WARNING = "warning"
    BLOCKED = "blocked"


class SafetyDecisionStatus(str, Enum):
    """Allowed safety decision states."""

    APPROVED = "approved"
    BLOCKED = "blocked"
    HOLD = "hold"
    NEEDS_REVIEW = "needs_review"


class HumanApprovalDecision(str, Enum):
    """Allowed human approval decisions."""

    APPROVED = "approved"
    REJECTED = "rejected"
    DEFERRED = "deferred"


ALLOWED_DEFENSIVE_RISK_CATEGORIES = (
    "GPS spoofing risk",
    "GPS / RF jamming risk",
    "communication degradation",
    "sensor corruption",
    "hostile UAV proximity",
    "collision risk",
    "geofence / no-fly-zone violation",
    "weather or wind disturbance",
    "telemetry anomaly",
    "mission command anomaly",
)

_FORBIDDEN_JSON_KEY_FRAGMENTS = (
    "token",
    "credential",
    "password",
    "secret",
    "api_key",
    "apikey",
    "private_key",
)


def _enum_values(enum_type: type[Enum]) -> List[str]:
    return [item.value for item in enum_type]


def _normalize_value(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {key: _normalize_value(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_normalize_value(child) for child in value]
    return value


def _from_dict(cls: type, data: Dict[str, Any]) -> Any:
    if not isinstance(data, dict):
        raise ValueError(f"{cls.__name__}.from_dict requires a dictionary")
    try:
        return cls(**copy.deepcopy(data))
    except TypeError as exc:
        raise ValueError(f"Invalid {cls.__name__} fields: {exc}") from exc


class SerializableMissionModel:
    """Small serialization mixin for JSON-safe dataclasses."""

    def to_dict(self) -> Dict[str, Any]:
        return _normalize_value(copy.deepcopy(self.__dict__))

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> Any:
        return _from_dict(cls, data)


def ensure_non_empty_string(value: Any, field_name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")


def ensure_non_negative_number(value: Any, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number")
    if not isfinite(float(value)) or float(value) < 0.0:
        raise ValueError(f"{field_name} must be a non-negative finite number")


def ensure_non_negative_int(value: Any, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field_name} must be an integer")
    if value < 0:
        raise ValueError(f"{field_name} must be non-negative")


def ensure_unit_interval(value: Any, field_name: str) -> None:
    ensure_non_negative_number(value, field_name)
    if float(value) > 1.0:
        raise ValueError(f"{field_name} must be in [0.0, 1.0]")


def ensure_finite_number(value: Any, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    if not isfinite(float(value)):
        raise ValueError(f"{field_name} must be finite")


def ensure_allowed_value(value: Any, allowed_values: List[str], field_name: str) -> None:
    if isinstance(value, Enum):
        value = value.value
    if value not in allowed_values:
        allowed = ", ".join(allowed_values)
        raise ValueError(f"{field_name} must be one of: {allowed}")


def ensure_allowed_risk_category(category: str) -> None:
    if category not in ALLOWED_DEFENSIVE_RISK_CATEGORIES:
        raise ValueError(f"risk category is not allowed: {category!r}")


def ensure_json_safe(value: Any, field_name: str) -> None:
    """Validate that a value can be represented as strict JSON."""

    def _check_json(child: Any, path: str) -> None:
        if child is None or isinstance(child, (str, bool)):
            return
        if isinstance(child, int) and not isinstance(child, bool):
            return
        if isinstance(child, float):
            if not isfinite(child):
                raise ValueError(f"{path} contains a non-finite float")
            return
        if isinstance(child, list):
            for index, item in enumerate(child):
                _check_json(item, f"{path}[{index}]")
            return
        if isinstance(child, dict):
            for key, item in child.items():
                if not isinstance(key, str):
                    raise ValueError(f"{path} keys must be strings")
                lowered = key.lower()
                if any(fragment in lowered for fragment in _FORBIDDEN_JSON_KEY_FRAGMENTS):
                    raise ValueError(f"{path} contains forbidden credential-like key: {key}")
                _check_json(item, f"{path}.{key}")
            return
        raise ValueError(f"{path} must be JSON-safe, got {type(child).__name__}")

    _check_json(value, field_name)
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be JSON-safe") from exc


def ensure_json_safe_dict(value: Any, field_name: str) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a dictionary")
    ensure_json_safe(value, field_name)


def ensure_json_safe_dict_list(value: Any, field_name: str) -> None:
    if not isinstance(value, list):
        raise ValueError(f"{field_name} must be a list")
    for index, item in enumerate(value):
        ensure_json_safe_dict(item, f"{field_name}[{index}]")


def ensure_string_list(value: Any, field_name: str) -> None:
    if not isinstance(value, list):
        raise ValueError(f"{field_name} must be a list")
    for index, item in enumerate(value):
        if not isinstance(item, str):
            raise ValueError(f"{field_name}[{index}] must be a string")


def ensure_numeric_mapping(value: Any, field_name: str) -> None:
    ensure_json_safe_dict(value, field_name)
    for key, item in value.items():
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{field_name}.{key} must be numeric")
        if not isfinite(float(item)):
            raise ValueError(f"{field_name}.{key} must be finite")


@dataclass
class MissionRequest(SerializableMissionModel):
    request_id: str
    operator_id: str
    objective: str
    priority: int
    area: Dict[str, Any] = field(default_factory=dict)
    constraints: Dict[str, Any] = field(default_factory=dict)
    created_at: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.request_id, "request_id")
        ensure_non_empty_string(self.operator_id, "operator_id")
        ensure_non_empty_string(self.objective, "objective")
        ensure_non_negative_int(self.priority, "priority")
        if self.priority > 5:
            raise ValueError("priority must be in range 0..5")
        ensure_json_safe_dict(self.area, "area")
        ensure_json_safe_dict(self.constraints, "constraints")
        ensure_non_negative_number(self.created_at, "created_at")
        ensure_json_safe_dict(self.metadata, "metadata")
        if "command" in self.metadata or "command" in self.constraints:
            raise ValueError("MissionRequest must not contain a direct command payload")


@dataclass
class MissionTask(SerializableMissionModel):
    task_id: str
    request_id: str
    objective: str
    status: str = MissionTaskStatus.PENDING.value
    priority: int = 0
    constraints: Dict[str, Any] = field(default_factory=dict)
    assigned_asset_id: Optional[str] = None
    created_at: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.task_id, "task_id")
        ensure_non_empty_string(self.request_id, "request_id")
        ensure_non_empty_string(self.objective, "objective")
        ensure_allowed_value(self.status, _enum_values(MissionTaskStatus), "status")
        ensure_non_negative_int(self.priority, "priority")
        if self.priority > 5:
            raise ValueError("priority must be in range 0..5")
        ensure_json_safe_dict(self.constraints, "constraints")
        if self.assigned_asset_id is not None:
            ensure_non_empty_string(self.assigned_asset_id, "assigned_asset_id")
        ensure_non_negative_number(self.created_at, "created_at")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class FleetAsset(SerializableMissionModel):
    asset_id: str
    backend: str
    capabilities: List[str] = field(default_factory=list)
    available: bool = True
    health: Dict[str, Any] = field(default_factory=dict)
    current_task_id: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.asset_id, "asset_id")
        ensure_non_empty_string(self.backend, "backend")
        ensure_string_list(self.capabilities, "capabilities")
        if not isinstance(self.available, bool):
            raise ValueError("available must be a boolean")
        ensure_json_safe_dict(self.health, "health")
        if self.current_task_id is not None:
            ensure_non_empty_string(self.current_task_id, "current_task_id")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class UAVState(SerializableMissionModel):
    asset_id: str
    timestamp: float
    position: Dict[str, Any] = field(default_factory=dict)
    velocity: Dict[str, Any] = field(default_factory=dict)
    battery: float = 1.0
    link_quality: float = 1.0
    mode: str = "mock"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.asset_id, "asset_id")
        ensure_non_negative_number(self.timestamp, "timestamp")
        ensure_numeric_mapping(self.position, "position")
        ensure_numeric_mapping(self.velocity, "velocity")
        ensure_unit_interval(self.battery, "battery")
        ensure_unit_interval(self.link_quality, "link_quality")
        ensure_non_empty_string(self.mode, "mode")
        ensure_json_safe_dict(self.metadata, "metadata")

    def is_stale(self, now: float, max_age: float) -> bool:
        ensure_non_negative_number(now, "now")
        ensure_non_negative_number(max_age, "max_age")
        return float(now) - float(self.timestamp) > float(max_age)


@dataclass
class MissionEvent(SerializableMissionModel):
    event_id: str
    event_type: str
    timestamp: float
    source: str
    payload: Dict[str, Any] = field(default_factory=dict)
    correlation_id: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.event_id, "event_id")
        ensure_non_empty_string(self.event_type, "event_type")
        ensure_non_negative_number(self.timestamp, "timestamp")
        ensure_non_empty_string(self.source, "source")
        ensure_json_safe_dict(self.payload, "payload")
        if self.correlation_id is not None:
            ensure_non_empty_string(self.correlation_id, "correlation_id")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class RiskSignal(SerializableMissionModel):
    signal_id: str
    category: str
    severity: float
    confidence: float
    evidence: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.signal_id, "signal_id")
        ensure_allowed_risk_category(self.category)
        ensure_unit_interval(self.severity, "severity")
        ensure_unit_interval(self.confidence, "confidence")
        ensure_json_safe_dict(self.evidence, "evidence")
        ensure_non_negative_number(self.timestamp, "timestamp")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class ThreatAssessment(SerializableMissionModel):
    assessment_id: str
    mission_id: str
    risk_signals: List[Dict[str, Any]] = field(default_factory=list)
    total_risk: float = 0.0
    recommendation: str = ThreatRecommendation.CONTINUE.value
    explanation: str = ""
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.assessment_id, "assessment_id")
        ensure_non_empty_string(self.mission_id, "mission_id")
        ensure_json_safe_dict_list(self.risk_signals, "risk_signals")
        ensure_unit_interval(self.total_risk, "total_risk")
        ensure_allowed_value(self.recommendation, _enum_values(ThreatRecommendation), "recommendation")
        if self.total_risk > 0.0 and not self.explanation.strip():
            raise ValueError("explanation is required when total_risk > 0")
        if not isinstance(self.explanation, str):
            raise ValueError("explanation must be a string")
        ensure_non_negative_number(self.timestamp, "timestamp")


@dataclass
class AirspaceConstraint(SerializableMissionModel):
    constraint_id: str
    constraint_type: str
    geometry: Dict[str, Any] = field(default_factory=dict)
    altitude_min: Optional[float] = None
    altitude_max: Optional[float] = None
    active: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.constraint_id, "constraint_id")
        ensure_non_empty_string(self.constraint_type, "constraint_type")
        ensure_json_safe_dict(self.geometry, "geometry")
        if self.altitude_min is not None:
            ensure_finite_number(self.altitude_min, "altitude_min")
        if self.altitude_max is not None:
            ensure_finite_number(self.altitude_max, "altitude_max")
        if self.altitude_min is not None and self.altitude_max is not None:
            if float(self.altitude_min) > float(self.altitude_max):
                raise ValueError("altitude_min must be <= altitude_max")
        if not isinstance(self.active, bool):
            raise ValueError("active must be a boolean")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class PlannedRoute(SerializableMissionModel):
    route_id: str
    task_id: str
    waypoints: List[Dict[str, Any]]
    score: float
    risk_score: float
    constraint_verdict: str = RouteConstraintVerdict.VALID.value
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.route_id, "route_id")
        ensure_non_empty_string(self.task_id, "task_id")
        ensure_json_safe_dict_list(self.waypoints, "waypoints")
        if not self.waypoints:
            raise ValueError("waypoints must be non-empty")
        ensure_finite_number(self.score, "score")
        ensure_unit_interval(self.risk_score, "risk_score")
        ensure_allowed_value(
            self.constraint_verdict,
            _enum_values(RouteConstraintVerdict),
            "constraint_verdict",
        )
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class SafetyDecision(SerializableMissionModel):
    decision_id: str
    target_id: str
    status: str = SafetyDecisionStatus.NEEDS_REVIEW.value
    reason: str = ""
    cbf_metadata: Dict[str, Any] = field(default_factory=dict)
    requires_human_approval: bool = True
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.decision_id, "decision_id")
        ensure_non_empty_string(self.target_id, "target_id")
        ensure_allowed_value(self.status, _enum_values(SafetyDecisionStatus), "status")
        if not isinstance(self.reason, str):
            raise ValueError("reason must be a string")
        if self.status != SafetyDecisionStatus.APPROVED.value and not self.reason.strip():
            raise ValueError("reason is required for non-approved safety decisions")
        ensure_json_safe_dict(self.cbf_metadata, "cbf_metadata")
        if not isinstance(self.requires_human_approval, bool):
            raise ValueError("requires_human_approval must be a boolean")
        ensure_non_negative_number(self.timestamp, "timestamp")


@dataclass
class HumanApprovalRecord(SerializableMissionModel):
    approval_id: str
    operator_id: str
    target_id: str
    decision: str
    notes: str = ""
    timestamp: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.approval_id, "approval_id")
        ensure_non_empty_string(self.operator_id, "operator_id")
        ensure_non_empty_string(self.target_id, "target_id")
        ensure_allowed_value(self.decision, _enum_values(HumanApprovalDecision), "decision")
        if not isinstance(self.notes, str):
            raise ValueError("notes must be a string")
        ensure_non_negative_number(self.timestamp, "timestamp")
        ensure_json_safe_dict(self.metadata, "metadata")


@dataclass
class ReplayFrame(SerializableMissionModel):
    frame_id: str
    timestamp: float
    mission_snapshot: Dict[str, Any] = field(default_factory=dict)
    events: List[Dict[str, Any]] = field(default_factory=list)
    risk_summary: Dict[str, Any] = field(default_factory=dict)
    route_summary: Dict[str, Any] = field(default_factory=dict)
    safety_summary: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.frame_id, "frame_id")
        ensure_non_negative_number(self.timestamp, "timestamp")
        ensure_json_safe_dict(self.mission_snapshot, "mission_snapshot")
        ensure_json_safe_dict_list(self.events, "events")
        ensure_json_safe_dict(self.risk_summary, "risk_summary")
        ensure_json_safe_dict(self.route_summary, "route_summary")
        ensure_json_safe_dict(self.safety_summary, "safety_summary")


@dataclass
class MetricSummary(SerializableMissionModel):
    mission_id: str
    event_count: int = 0
    risk_counts: Dict[str, Any] = field(default_factory=dict)
    replan_count: int = 0
    hold_count: int = 0
    approval_count: int = 0
    blocked_count: int = 0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        ensure_non_empty_string(self.mission_id, "mission_id")
        ensure_non_negative_int(self.event_count, "event_count")
        ensure_json_safe_dict(self.risk_counts, "risk_counts")
        for category, count in self.risk_counts.items():
            ensure_allowed_risk_category(category)
            ensure_non_negative_int(count, f"risk_counts.{category}")
        ensure_non_negative_int(self.replan_count, "replan_count")
        ensure_non_negative_int(self.hold_count, "hold_count")
        ensure_non_negative_int(self.approval_count, "approval_count")
        ensure_non_negative_int(self.blocked_count, "blocked_count")
        ensure_json_safe_dict(self.metadata, "metadata")
