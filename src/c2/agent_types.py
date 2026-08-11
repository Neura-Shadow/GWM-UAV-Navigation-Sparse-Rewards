"""Provider-independent typed contracts for the AgentOps v3 extension.

This module performs structural validation and JSON-safe serialization only.
It does not authorize tools, invoke runtimes, or mutate mission state.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import MISSING, dataclass, field, fields
from enum import Enum, IntEnum
from math import isfinite
from typing import Any, ClassVar, Dict, List, Optional, Type, TypeVar


class AgentContractError(ValueError):
    """Deterministic validation error for an AgentOps contract boundary."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        field_name: Optional[str] = None,
        contract_name: Optional[str] = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.field_name = field_name
        self.contract_name = contract_name

    def to_dict(self) -> Dict[str, Any]:
        return {
            "code": self.code,
            "message": self.message,
            "field_name": self.field_name,
            "contract_name": self.contract_name,
        }


class AgentRole(str, Enum):
    SUPERVISOR = "supervisor"
    MISSION = "mission"
    FLEET = "fleet"
    SITUATION_AWARENESS = "situation_awareness"
    DEFENSIVE_RISK = "defensive_risk"
    PLANNING_AIRSPACE = "planning_airspace"
    SIMULATION_VALIDATION = "simulation_validation"
    SAFETY_REVIEW = "safety_review"
    AUDIT_REPLAY = "audit_replay"


class SideEffectLevel(str, Enum):
    READ_ONLY = "READ_ONLY"
    STATE_PROPOSAL = "STATE_PROPOSAL"
    VALIDATED_STATE_WRITE = "VALIDATED_STATE_WRITE"
    SIMULATOR_READ_ONLY = "SIMULATOR_READ_ONLY"
    SIMULATOR_COMMAND_GATED = "SIMULATOR_COMMAND_GATED"
    SITL_COMMAND_GATED = "SITL_COMMAND_GATED"
    REAL_HARDWARE_PROHIBITED = "REAL_HARDWARE_PROHIBITED"


class ApprovalLevel(IntEnum):
    READ_ONLY = 0
    PROPOSAL = 1
    MOCK_SIMULATION = 2
    EXTERNAL_SIMULATOR = 3
    SITL_COMMAND = 4
    REAL_HARDWARE_PROHIBITED = 5


class AgentTaskStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    TIMED_OUT = "timed_out"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"


class ToolCallStatus(str, Enum):
    PASSED = "passed"
    FAILED = "failed"
    DENIED = "denied"
    BLOCKED = "blocked"
    TIMED_OUT = "timed_out"
    NOT_READY = "not_ready"
    RUNTIME_UNAVAILABLE = "runtime_unavailable"
    SKIPPED = "skipped"


class AgentDecisionType(str, Enum):
    CONTINUE = "continue"
    HOLD = "hold"
    REPLAN = "replan"
    REQUEST_REVIEW = "request_review"
    APPROVE_RECOMMENDATION = "approve_recommendation"
    BLOCK = "block"
    DENY = "deny"


class ApprovalOutcome(str, Enum):
    APPROVED = "approved"
    REJECTED = "rejected"
    DEFERRED = "deferred"
    EXPIRED = "expired"
    REVOKED = "revoked"


class AgentWorkflowStatus(str, Enum):
    CREATED = "created"
    PLANNING = "planning"
    AWAITING_REVIEW = "awaiting_review"
    AWAITING_APPROVAL = "awaiting_approval"
    VALIDATING = "validating"
    COMPLETED = "completed"
    FAILED = "failed"
    TIMED_OUT = "timed_out"
    BLOCKED = "blocked"
    CANCELLED = "cancelled"


_SUPPORTED_ENUM_TYPES = (
    AgentRole,
    SideEffectLevel,
    ApprovalLevel,
    AgentTaskStatus,
    ToolCallStatus,
    AgentDecisionType,
    ApprovalOutcome,
    AgentWorkflowStatus,
)
_SENSITIVE_KEYS = frozenset(
    {
        "password",
        "passwd",
        "secret",
        "token",
        "api_key",
        "apikey",
        "credential",
        "credentials",
        "private_key",
        "hostname",
        "host",
        "runtime_handle",
        "socket",
        "session",
        "shell",
        "command",
        "code_payload",
        "file_handle",
        "chain_of_thought",
        "private_chain_of_thought",
        "reasoning_trace",
        "hidden_reasoning",
    }
)
_MAX_STRING_LENGTH = 16_384
_MAX_COLLECTION_LENGTH = 4_096
_MAX_JSON_DEPTH = 32

ContractT = TypeVar("ContractT", bound="AgentContract")
EnumT = TypeVar("EnumT", bound=Enum)


def _error(
    code: str,
    message: str,
    *,
    field_name: Optional[str] = None,
    contract_name: Optional[str] = None,
) -> AgentContractError:
    return AgentContractError(
        code,
        message,
        field_name=field_name,
        contract_name=contract_name,
    )


def _normalize_key(key: str) -> str:
    return key.strip().lower().replace("-", "_").replace(" ", "_")


def _is_sensitive_key(key: str) -> bool:
    normalized = _normalize_key(key)
    return normalized in _SENSITIVE_KEYS or any(
        normalized.endswith(f"_{sensitive}") for sensitive in _SENSITIVE_KEYS
    )


def _require_non_empty_string(value: Any, field_name: str, contract_name: str) -> str:
    if isinstance(value, Enum) or not isinstance(value, str) or not value.strip():
        raise _error(
            "invalid_contract",
            f"{field_name} must be a non-empty string",
            field_name=field_name,
            contract_name=contract_name,
        )
    if len(value) > _MAX_STRING_LENGTH:
        raise _error(
            "invalid_contract",
            f"{field_name} exceeds the maximum length",
            field_name=field_name,
            contract_name=contract_name,
        )
    return value


def _require_optional_non_empty_string(
    value: Any, field_name: str, contract_name: str
) -> Optional[str]:
    if value is None:
        return None
    return _require_non_empty_string(value, field_name, contract_name)


def _require_bool(value: Any, field_name: str, contract_name: str) -> bool:
    if not isinstance(value, bool):
        raise _error(
            "invalid_contract",
            f"{field_name} must be a boolean",
            field_name=field_name,
            contract_name=contract_name,
        )
    return value


def _require_finite_timestamp(value: Any, field_name: str, contract_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(
            "invalid_contract",
            f"{field_name} must be a non-negative finite number",
            field_name=field_name,
            contract_name=contract_name,
        )
    number = float(value)
    if not isfinite(number) or number < 0.0:
        raise _error(
            "invalid_contract",
            f"{field_name} must be a non-negative finite number",
            field_name=field_name,
            contract_name=contract_name,
        )
    return number


def _require_positive_bounded_number(
    value: Any,
    field_name: str,
    contract_name: str,
    *,
    maximum: float,
    code: str = "invalid_contract",
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise _error(
            code,
            f"{field_name} must be a positive finite number",
            field_name=field_name,
            contract_name=contract_name,
        )
    number = float(value)
    if not isfinite(number) or number <= 0.0 or number > maximum:
        raise _error(
            code,
            f"{field_name} must be greater than zero and at most {maximum:g}",
            field_name=field_name,
            contract_name=contract_name,
        )
    return number


def _require_unique_string_list(value: Any, field_name: str, contract_name: str) -> List[str]:
    if not isinstance(value, list):
        raise _error(
            "invalid_contract",
            f"{field_name} must be a list",
            field_name=field_name,
            contract_name=contract_name,
        )
    if len(value) > _MAX_COLLECTION_LENGTH:
        raise _error(
            "invalid_contract",
            f"{field_name} exceeds the maximum length",
            field_name=field_name,
            contract_name=contract_name,
        )
    result: List[str] = []
    for item in value:
        result.append(_require_non_empty_string(item, field_name, contract_name))
    if len(set(result)) != len(result):
        raise _error(
            "invalid_contract",
            f"{field_name} must contain unique values",
            field_name=field_name,
            contract_name=contract_name,
        )
    return list(result)


def _require_json_safe(
    value: Any,
    field_name: str,
    contract_name: str,
    *,
    depth: int = 0,
) -> None:
    if depth > _MAX_JSON_DEPTH:
        raise _error(
            "invalid_json_payload",
            f"{field_name} exceeds the maximum nesting depth",
            field_name=field_name,
            contract_name=contract_name,
        )
    if isinstance(value, Enum):
        if not isinstance(value, _SUPPORTED_ENUM_TYPES):
            raise _error(
                "invalid_json_payload",
                f"{field_name} contains an unsupported enum value",
                field_name=field_name,
                contract_name=contract_name,
            )
        _require_json_safe(value.value, field_name, contract_name, depth=depth + 1)
        return
    if value is None or isinstance(value, (str, bool)):
        if isinstance(value, str) and len(value) > _MAX_STRING_LENGTH:
            raise _error(
                "invalid_json_payload",
                f"{field_name} contains an oversized string",
                field_name=field_name,
                contract_name=contract_name,
            )
        return
    if isinstance(value, int) and not isinstance(value, bool):
        return
    if isinstance(value, float):
        if not isfinite(value):
            raise _error(
                "invalid_json_payload",
                f"{field_name} contains a non-finite float",
                field_name=field_name,
                contract_name=contract_name,
            )
        return
    if isinstance(value, list):
        if len(value) > _MAX_COLLECTION_LENGTH:
            raise _error(
                "invalid_json_payload",
                f"{field_name} exceeds the maximum collection length",
                field_name=field_name,
                contract_name=contract_name,
            )
        for index, item in enumerate(value):
            _require_json_safe(item, f"{field_name}[{index}]", contract_name, depth=depth + 1)
        return
    if isinstance(value, dict):
        if len(value) > _MAX_COLLECTION_LENGTH:
            raise _error(
                "invalid_json_payload",
                f"{field_name} exceeds the maximum collection length",
                field_name=field_name,
                contract_name=contract_name,
            )
        for key, item in value.items():
            if not isinstance(key, str):
                raise _error(
                    "invalid_json_payload",
                    f"{field_name} keys must be strings",
                    field_name=field_name,
                    contract_name=contract_name,
                )
            if _is_sensitive_key(key):
                raise _error(
                    "sensitive_field_rejected",
                    f"{field_name} contains a prohibited sensitive field",
                    field_name=key,
                    contract_name=contract_name,
                )
            _require_json_safe(item, f"{field_name}.{key}", contract_name, depth=depth + 1)
        return
    raise _error(
        "invalid_json_payload",
        f"{field_name} contains an unsupported value",
        field_name=field_name,
        contract_name=contract_name,
    )


def _copy_json_value(value: Any, field_name: str, contract_name: str) -> Any:
    _require_json_safe(value, field_name, contract_name)
    return deepcopy(value)


def _copy_json_dict(value: Any, field_name: str, contract_name: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise _error(
            "invalid_json_payload",
            f"{field_name} must be a dictionary",
            field_name=field_name,
            contract_name=contract_name,
        )
    return _copy_json_value(value, field_name, contract_name)


def _parse_enum_value(
    value: Any,
    enum_type: Type[EnumT],
    field_name: str,
    contract_name: str,
    *,
    code: str = "invalid_contract",
) -> EnumT:
    if isinstance(value, enum_type):
        return value
    if isinstance(value, Enum) or (
        issubclass(enum_type, IntEnum) and isinstance(value, bool)
    ):
        raise _error(
            code,
            f"{field_name} is not an allowed value",
            field_name=field_name,
            contract_name=contract_name,
        )
    try:
        return enum_type(value)
    except (TypeError, ValueError):
        raise _error(
            code,
            f"{field_name} is not an allowed value",
            field_name=field_name,
            contract_name=contract_name,
        ) from None


def _serialize_contract_value(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, list):
        return [_serialize_contract_value(item) for item in value]
    if isinstance(value, dict):
        return {key: _serialize_contract_value(item) for key, item in value.items()}
    return deepcopy(value)


class AgentContract:
    """Serialization mixin shared by all v3-1A contracts."""

    _contract_name: ClassVar[str]

    def validate(self) -> None:
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            contract_field.name: _serialize_contract_value(getattr(self, contract_field.name))
            for contract_field in fields(self)
        }

    @classmethod
    def from_dict(cls: Type[ContractT], data: Dict[str, Any]) -> ContractT:
        contract_name = cls.__name__
        if not isinstance(data, dict):
            raise _error(
                "invalid_contract",
                f"{contract_name}.from_dict requires a dictionary",
                contract_name=contract_name,
            )
        _require_json_safe(data, contract_name, contract_name)
        contract_fields = {item.name: item for item in fields(cls)}
        unknown = sorted(set(data) - set(contract_fields))
        if unknown:
            raise _error(
                "invalid_contract",
                f"{contract_name} contains unknown fields",
                field_name=unknown[0],
                contract_name=contract_name,
            )
        missing = [
            name
            for name, item in contract_fields.items()
            if item.default is MISSING and item.default_factory is MISSING and name not in data
        ]
        if missing:
            raise _error(
                "invalid_contract",
                f"{contract_name} is missing required fields",
                field_name=missing[0],
                contract_name=contract_name,
            )
        copied = deepcopy(data)
        try:
            return cls(**copied)
        except AgentContractError:
            raise
        except TypeError:
            raise _error(
                "invalid_contract",
                f"{contract_name} could not be constructed",
                contract_name=contract_name,
            ) from None


@dataclass
class AgentIdentity(AgentContract):
    agent_id: str
    role: AgentRole
    version: str
    capability_ids: List[str]
    enabled: bool
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.agent_id, "agent_id", name)
        self.role = _parse_enum_value(
            self.role, AgentRole, "role", name, code="unknown_agent_role"
        )
        _require_non_empty_string(self.version, "version", name)
        self.capability_ids = _require_unique_string_list(
            self.capability_ids, "capability_ids", name
        )
        _require_bool(self.enabled, "enabled", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentCapability(AgentContract):
    capability_id: str
    action: str
    resource_scope: str
    side_effect_level: SideEffectLevel
    max_approval_level: ApprovalLevel
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.capability_id, "capability_id", name)
        _require_non_empty_string(self.action, "action", name)
        _require_non_empty_string(self.resource_scope, "resource_scope", name)
        if "*" in self.action or "*" in self.resource_scope:
            raise _error(
                "unknown_capability",
                "wildcard actions and resource scopes are prohibited",
                field_name="action" if "*" in self.action else "resource_scope",
                contract_name=name,
            )
        self.side_effect_level = _parse_enum_value(
            self.side_effect_level,
            SideEffectLevel,
            "side_effect_level",
            name,
            code="side_effect_not_allowed",
        )
        if self.side_effect_level is SideEffectLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "the real-hardware denial marker is not executable",
                field_name="side_effect_level",
                contract_name=name,
            )
        self.max_approval_level = _parse_enum_value(
            self.max_approval_level,
            ApprovalLevel,
            "max_approval_level",
            name,
            code="approval_invalid",
        )
        if self.max_approval_level is ApprovalLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "approval level 5 is prohibited",
                field_name="max_approval_level",
                contract_name=name,
            )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentTask(AgentContract):
    task_id: str
    workflow_id: str
    assignee_agent_id: str
    objective: str
    input_refs: List[str]
    deadline: Optional[float]
    status: AgentTaskStatus
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.task_id, "task_id", name)
        _require_non_empty_string(self.workflow_id, "workflow_id", name)
        _require_non_empty_string(self.assignee_agent_id, "assignee_agent_id", name)
        _require_non_empty_string(self.objective, "objective", name)
        self.input_refs = _require_unique_string_list(self.input_refs, "input_refs", name)
        if self.deadline is not None:
            self.deadline = _require_finite_timestamp(self.deadline, "deadline", name)
        self.status = _parse_enum_value(
            self.status, AgentTaskStatus, "status", name, code="invalid_contract"
        )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentObservation(AgentContract):
    observation_id: str
    source: str
    timestamp: float
    schema_version: str
    payload_summary: Any
    freshness_status: str
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.observation_id, "observation_id", name)
        _require_non_empty_string(self.source, "source", name)
        self.timestamp = _require_finite_timestamp(self.timestamp, "timestamp", name)
        _require_non_empty_string(self.schema_version, "schema_version", name)
        self.payload_summary = _copy_json_value(
            self.payload_summary, "payload_summary", name
        )
        _require_non_empty_string(self.freshness_status, "freshness_status", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentContext(AgentContract):
    context_id: str
    workflow_id: str
    state_refs: List[str]
    observation_refs: List[str]
    policy_version: str
    freshness: str
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.context_id, "context_id", name)
        _require_non_empty_string(self.workflow_id, "workflow_id", name)
        self.state_refs = _require_unique_string_list(self.state_refs, "state_refs", name)
        self.observation_refs = _require_unique_string_list(
            self.observation_refs, "observation_refs", name
        )
        _require_non_empty_string(self.policy_version, "policy_version", name)
        _require_non_empty_string(self.freshness, "freshness", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentProposal(AgentContract):
    """An untrusted proposal until deterministic validation succeeds.

    Construction does not mutate validated mission state.
    """

    proposal_id: str
    agent_id: str
    task_id: str
    proposal_type: str
    payload: Any
    evidence_refs: List[str]
    created_at: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.proposal_id, "proposal_id", name)
        _require_non_empty_string(self.agent_id, "agent_id", name)
        _require_non_empty_string(self.task_id, "task_id", name)
        _require_non_empty_string(self.proposal_type, "proposal_type", name)
        self.payload = _copy_json_value(self.payload, "payload", name)
        self.evidence_refs = _require_unique_string_list(
            self.evidence_refs, "evidence_refs", name
        )
        self.created_at = _require_finite_timestamp(self.created_at, "created_at", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class ToolCallRequest(AgentContract):
    """A structurally valid request; authorization belongs to v3-1B."""

    request_id: str
    tool_name: str
    tool_version: str
    caller_agent_id: str
    parameters: Any
    approval_ref: Optional[str]
    timeout_sec: float
    runtime_gate_refs: List[str]
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.request_id, "request_id", name)
        _require_non_empty_string(self.tool_name, "tool_name", name)
        _require_non_empty_string(self.tool_version, "tool_version", name)
        _require_non_empty_string(self.caller_agent_id, "caller_agent_id", name)
        self.parameters = _copy_json_value(self.parameters, "parameters", name)
        _require_optional_non_empty_string(self.approval_ref, "approval_ref", name)
        self.timeout_sec = _require_positive_bounded_number(
            self.timeout_sec,
            "timeout_sec",
            name,
            maximum=300.0,
            code="timeout_invalid",
        )
        self.runtime_gate_refs = _require_unique_string_list(
            self.runtime_gate_refs, "runtime_gate_refs", name
        )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class ToolCallResult(AgentContract):
    request_id: str
    tool_name: str
    tool_version: str
    status: ToolCallStatus
    result_summary: Any
    evidence_refs: List[str]
    errors: List[Any]
    started_at: float
    completed_at: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.request_id, "request_id", name)
        _require_non_empty_string(self.tool_name, "tool_name", name)
        _require_non_empty_string(self.tool_version, "tool_version", name)
        self.status = _parse_enum_value(self.status, ToolCallStatus, "status", name)
        self.result_summary = _copy_json_value(
            self.result_summary, "result_summary", name
        )
        self.evidence_refs = _require_unique_string_list(
            self.evidence_refs, "evidence_refs", name
        )
        if not isinstance(self.errors, list):
            raise _error(
                "invalid_contract",
                "errors must be a list",
                field_name="errors",
                contract_name=name,
            )
        self.errors = _copy_json_value(self.errors, "errors", name)
        self.started_at = _require_finite_timestamp(self.started_at, "started_at", name)
        self.completed_at = _require_finite_timestamp(
            self.completed_at, "completed_at", name
        )
        if self.completed_at < self.started_at:
            raise _error(
                "invalid_contract",
                "completed_at must be greater than or equal to started_at",
                field_name="completed_at",
                contract_name=name,
            )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentDecision(AgentContract):
    decision_id: str
    agent_id: str
    proposal_refs: List[str]
    decision: AgentDecisionType
    rationale: str
    evidence_refs: List[str]
    metadata: Dict[str, Any] = field(default_factory=dict)

    _SAFETY_AFFECTING: ClassVar[frozenset[AgentDecisionType]] = frozenset(
        {
            AgentDecisionType.HOLD,
            AgentDecisionType.REPLAN,
            AgentDecisionType.REQUEST_REVIEW,
            AgentDecisionType.APPROVE_RECOMMENDATION,
            AgentDecisionType.BLOCK,
            AgentDecisionType.DENY,
        }
    )

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.decision_id, "decision_id", name)
        _require_non_empty_string(self.agent_id, "agent_id", name)
        self.proposal_refs = _require_unique_string_list(
            self.proposal_refs, "proposal_refs", name
        )
        self.decision = _parse_enum_value(self.decision, AgentDecisionType, "decision", name)
        _require_non_empty_string(self.rationale, "rationale", name)
        self.evidence_refs = _require_unique_string_list(
            self.evidence_refs, "evidence_refs", name
        )
        if self.decision in self._SAFETY_AFFECTING and not self.evidence_refs:
            raise _error(
                "invalid_contract",
                "safety-affecting decisions require at least one evidence reference",
                field_name="evidence_refs",
                contract_name=name,
            )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentConflict(AgentContract):
    conflict_id: str
    workflow_id: str
    proposal_refs: List[str]
    conflict_type: str
    summary: str
    resolution_status: str
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.conflict_id, "conflict_id", name)
        _require_non_empty_string(self.workflow_id, "workflow_id", name)
        self.proposal_refs = _require_unique_string_list(
            self.proposal_refs, "proposal_refs", name
        )
        if len(self.proposal_refs) < 2:
            raise _error(
                "invalid_contract",
                "proposal_refs must contain at least two unique references",
                field_name="proposal_refs",
                contract_name=name,
            )
        _require_non_empty_string(self.conflict_type, "conflict_type", name)
        _require_non_empty_string(self.summary, "summary", name)
        _require_non_empty_string(self.resolution_status, "resolution_status", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class ApprovalRequest(AgentContract):
    approval_request_id: str
    approval_level: ApprovalLevel
    target_refs: List[str]
    rationale: str
    evidence_refs: List[str]
    requested_by: str
    expires_at: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.approval_request_id, "approval_request_id", name)
        self.approval_level = _parse_enum_value(
            self.approval_level,
            ApprovalLevel,
            "approval_level",
            name,
            code="approval_invalid",
        )
        if self.approval_level is ApprovalLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "approval level 5 is prohibited",
                field_name="approval_level",
                contract_name=name,
            )
        if self.approval_level is ApprovalLevel.READ_ONLY:
            raise _error(
                "approval_invalid",
                "approval requests accept levels 1 through 4 only",
                field_name="approval_level",
                contract_name=name,
            )
        self.target_refs = _require_unique_string_list(
            self.target_refs, "target_refs", name
        )
        if not self.target_refs:
            raise _error(
                "invalid_contract",
                "target_refs must contain at least one reference",
                field_name="target_refs",
                contract_name=name,
            )
        _require_non_empty_string(self.rationale, "rationale", name)
        self.evidence_refs = _require_unique_string_list(
            self.evidence_refs, "evidence_refs", name
        )
        _require_non_empty_string(self.requested_by, "requested_by", name)
        self.expires_at = _require_finite_timestamp(self.expires_at, "expires_at", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class ApprovalDecision(AgentContract):
    """Structurally valid human response without authentication claims."""

    decision_id: str
    approval_request_id: str
    operator_id: str
    outcome: ApprovalOutcome
    timestamp: float
    notes: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)

    _PRODUCER_KEYS: ClassVar[frozenset[str]] = frozenset(
        {
            "authoritative_producer",
            "authoritative_producer_role",
            "producer",
            "producer_role",
        }
    )

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.decision_id, "decision_id", name)
        _require_non_empty_string(self.approval_request_id, "approval_request_id", name)
        _require_non_empty_string(self.operator_id, "operator_id", name)
        self.outcome = _parse_enum_value(self.outcome, ApprovalOutcome, "outcome", name)
        self.timestamp = _require_finite_timestamp(self.timestamp, "timestamp", name)
        if not isinstance(self.notes, str):
            raise _error(
                "invalid_contract",
                "notes must be a string",
                field_name="notes",
                contract_name=name,
            )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)
        role_values = {role.value for role in AgentRole}
        for key, value in self.metadata.items():
            producer_value = value.value if isinstance(value, AgentRole) else value
            if _normalize_key(key) in self._PRODUCER_KEYS and producer_value in role_values:
                raise _error(
                    "invalid_agent_identity",
                    "an agent role cannot be the authoritative human approval producer",
                    field_name=key,
                    contract_name=name,
                )


@dataclass
class AgentAuditRecord(AgentContract):
    """One audit record contract; append-only storage belongs to v3-1C."""

    record_id: str
    workflow_id: str
    event_type: str
    actor_id: str
    timestamp: float
    input_refs: List[str] = field(default_factory=list)
    tool_name: Optional[str] = None
    tool_version: Optional[str] = None
    validated_parameters_summary: Any = field(default_factory=dict)
    result_summary: Any = field(default_factory=dict)
    decision: Optional[str] = None
    approval_state: Optional[str] = None
    errors: List[Any] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.record_id, "record_id", name)
        _require_non_empty_string(self.workflow_id, "workflow_id", name)
        _require_non_empty_string(self.event_type, "event_type", name)
        _require_non_empty_string(self.actor_id, "actor_id", name)
        self.timestamp = _require_finite_timestamp(self.timestamp, "timestamp", name)
        self.input_refs = _require_unique_string_list(self.input_refs, "input_refs", name)
        _require_optional_non_empty_string(self.tool_name, "tool_name", name)
        _require_optional_non_empty_string(self.tool_version, "tool_version", name)
        if (self.tool_name is None) != (self.tool_version is None):
            raise _error(
                "invalid_contract",
                "tool_name and tool_version must be both present or both absent",
                field_name="tool_name",
                contract_name=name,
            )
        self.validated_parameters_summary = _copy_json_value(
            self.validated_parameters_summary, "validated_parameters_summary", name
        )
        self.result_summary = _copy_json_value(
            self.result_summary, "result_summary", name
        )
        _require_optional_non_empty_string(self.decision, "decision", name)
        _require_optional_non_empty_string(self.approval_state, "approval_state", name)
        if not isinstance(self.errors, list):
            raise _error(
                "invalid_contract",
                "errors must be a list",
                field_name="errors",
                contract_name=name,
            )
        self.errors = _copy_json_value(self.errors, "errors", name)
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)


@dataclass
class AgentWorkflowState(AgentContract):
    workflow_id: str
    status: AgentWorkflowStatus
    task_refs: List[str]
    proposal_refs: List[str]
    conflict_refs: List[str]
    approval_refs: List[str]
    created_at: float
    updated_at: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        name = type(self).__name__
        _require_non_empty_string(self.workflow_id, "workflow_id", name)
        self.status = _parse_enum_value(self.status, AgentWorkflowStatus, "status", name)
        self.task_refs = _require_unique_string_list(self.task_refs, "task_refs", name)
        self.proposal_refs = _require_unique_string_list(
            self.proposal_refs, "proposal_refs", name
        )
        self.conflict_refs = _require_unique_string_list(
            self.conflict_refs, "conflict_refs", name
        )
        self.approval_refs = _require_unique_string_list(
            self.approval_refs, "approval_refs", name
        )
        self.created_at = _require_finite_timestamp(self.created_at, "created_at", name)
        self.updated_at = _require_finite_timestamp(self.updated_at, "updated_at", name)
        if self.updated_at < self.created_at:
            raise _error(
                "invalid_contract",
                "updated_at must be greater than or equal to created_at",
                field_name="updated_at",
                contract_name=name,
            )
        self.metadata = _copy_json_dict(self.metadata, "metadata", name)
