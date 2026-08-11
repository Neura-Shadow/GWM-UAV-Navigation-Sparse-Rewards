"""Exact-match metadata registry for deny-by-default AgentOps authorization."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import MISSING, dataclass, field, fields
from enum import Enum
from math import isfinite
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from src.c2.agent_audit import AgentAuditLog
from src.c2.agent_permissions import AgentPermissionManifest
from src.c2.agent_types import (
    AgentContractError,
    AgentIdentity,
    AgentRole,
    ApprovalDecision,
    ApprovalLevel,
    ApprovalOutcome,
    ApprovalRequest,
    SideEffectLevel,
    ToolCallRequest,
    ToolCallResult,
    ToolCallStatus,
)


DEFAULT_AGENT_TOOL_SCHEMA_IDS: Tuple[str, ...] = (
    "mission_query_request_v1",
    "mission_snapshot_v1",
    "mission_proposal_request_v1",
    "mission_proposal_v1",
    "fleet_query_request_v1",
    "fleet_snapshot_v1",
    "fleet_assignment_request_v1",
    "fleet_assignment_proposal_v1",
    "situation_query_request_v1",
    "agent_context_v1",
    "risk_evaluation_request_v1",
    "threat_assessment_v1",
    "airspace_validation_request_v1",
    "airspace_verdict_v1",
    "route_scoring_request_v1",
    "planned_route_v1",
    "replay_request_v1",
    "replay_payload_v1",
    "metrics_request_v1",
    "metric_summary_v1",
    "report_request_v1",
    "audit_report_v1",
    "benchmark_readiness_request_v1",
    "benchmark_readiness_result_v1",
    "approval_request_input_v1",
    "approval_request_v1",
)

DEFAULT_AGENT_TOOL_NAMES: Tuple[str, ...] = (
    "mission.query",
    "mission.propose",
    "fleet.query",
    "fleet.propose_assignment",
    "situation.query",
    "risk.evaluate",
    "airspace.validate",
    "route.score",
    "replay.build",
    "metrics.build",
    "report.build",
    "benchmark.readiness",
    "approval.request",
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
        "handler",
        "provider",
    }
)


def _error(
    code: str,
    message: str,
    field_name: Optional[str] = None,
    contract_name: str = "AgentToolDefinition",
) -> AgentContractError:
    return AgentContractError(
        code,
        message,
        field_name=field_name,
        contract_name=contract_name,
    )


def _non_empty(value: Any, field_name: str, *, wildcard_allowed: bool = False) -> str:
    if isinstance(value, Enum) or not isinstance(value, str) or not value.strip():
        raise _error("invalid_contract", f"{field_name} must be a non-empty string", field_name)
    if not wildcard_allowed and "*" in value:
        raise _error("invalid_contract", f"{field_name} must not contain wildcards", field_name)
    return value


def _string_tuple(value: Any, field_name: str, *, require_non_empty: bool = False) -> Tuple[str, ...]:
    if not isinstance(value, (list, tuple)):
        raise _error("invalid_contract", f"{field_name} must be a list or tuple", field_name)
    result = tuple(_non_empty(item, field_name) for item in value)
    if require_non_empty and not result:
        raise _error("invalid_contract", f"{field_name} must not be empty", field_name)
    if len(set(result)) != len(result):
        raise _error("invalid_contract", f"{field_name} must contain unique values", field_name)
    return result


def _parse_role(value: Any) -> AgentRole:
    if isinstance(value, AgentRole):
        return value
    if isinstance(value, Enum):
        raise _error("unknown_agent_role", "agent role is not allowed", "allowed_agent_roles")
    try:
        return AgentRole(value)
    except (TypeError, ValueError):
        raise _error("unknown_agent_role", "agent role is not allowed", "allowed_agent_roles") from None


def _parse_side_effect(value: Any) -> SideEffectLevel:
    if isinstance(value, SideEffectLevel):
        return value
    if isinstance(value, Enum):
        raise _error("side_effect_not_allowed", "side effect is not allowed", "side_effect_level")
    try:
        return SideEffectLevel(value)
    except (TypeError, ValueError):
        raise _error("side_effect_not_allowed", "side effect is not allowed", "side_effect_level") from None


def _parse_approval(value: Any) -> ApprovalLevel:
    if isinstance(value, ApprovalLevel):
        return value
    if isinstance(value, bool) or isinstance(value, Enum):
        raise _error("approval_invalid", "approval level is not allowed", "required_approval_level")
    try:
        return ApprovalLevel(value)
    except (TypeError, ValueError):
        raise _error("approval_invalid", "approval level is not allowed", "required_approval_level") from None


def _normalize_sensitive_key(key: str) -> str:
    return key.strip().lower().replace("-", "_").replace(" ", "_")


def _is_sensitive_key(key: str) -> bool:
    normalized = _normalize_sensitive_key(key)
    return normalized in _SENSITIVE_KEYS or any(
        normalized.endswith(f"_{item}") for item in _SENSITIVE_KEYS
    )


def _validate_json_safe(value: Any, field_name: str, depth: int = 0) -> None:
    if depth > 32:
        raise _error("invalid_json_payload", f"{field_name} is too deeply nested", field_name)
    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int) and not isinstance(value, bool):
        return
    if isinstance(value, float):
        if not isfinite(value):
            raise _error("invalid_json_payload", f"{field_name} contains a non-finite value", field_name)
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_json_safe(item, f"{field_name}[{index}]", depth + 1)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise _error("invalid_json_payload", f"{field_name} keys must be strings", field_name)
            if _is_sensitive_key(key):
                raise _error(
                    "sensitive_field_rejected",
                    f"{field_name} contains a prohibited sensitive field",
                    key,
                )
            _validate_json_safe(item, f"{field_name}.{key}", depth + 1)
        return
    raise _error("invalid_json_payload", f"{field_name} contains an unsupported value", field_name)


def _copy_metadata(value: Any) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise _error("invalid_json_payload", "metadata must be a dictionary", "metadata")
    _validate_json_safe(value, "metadata")
    return deepcopy(value)


@dataclass(frozen=True)
class AgentToolDefinition:
    """Metadata-only definition for one exact tool name and version."""

    tool_name: str
    tool_version: str
    description: str
    allowed_agent_roles: Tuple[AgentRole, ...]
    input_schema_id: str
    output_schema_id: str
    access_class: str
    side_effect_level: SideEffectLevel
    required_approval_level: ApprovalLevel
    runtime_gate_names: Tuple[str, ...]
    timeout_sec: float
    fallback_behavior: str
    audit_field_names: Tuple[str, ...]
    enabled: bool
    metadata: Dict[str, Any] = field(default_factory=dict, compare=True)

    def __post_init__(self) -> None:
        self.validate()

    @property
    def tool_id(self) -> str:
        return f"{self.tool_name}@{self.tool_version}"

    def validate(self) -> None:
        _non_empty(self.tool_name, "tool_name")
        if "@" in self.tool_name:
            raise _error("invalid_contract", "tool_name must not contain @", "tool_name")
        _non_empty(self.tool_version, "tool_version")
        _non_empty(self.description, "description", wildcard_allowed=True)
        if not isinstance(self.allowed_agent_roles, (list, tuple)):
            raise _error("invalid_contract", "allowed_agent_roles must be a list or tuple", "allowed_agent_roles")
        roles = tuple(_parse_role(item) for item in self.allowed_agent_roles)
        if not roles:
            raise _error("invalid_contract", "allowed_agent_roles must not be empty", "allowed_agent_roles")
        if len(set(roles)) != len(roles):
            raise _error("invalid_contract", "allowed_agent_roles must contain unique values", "allowed_agent_roles")
        object.__setattr__(self, "allowed_agent_roles", roles)
        for field_name in ("input_schema_id", "output_schema_id", "access_class"):
            _non_empty(getattr(self, field_name), field_name)
        side_effect = _parse_side_effect(self.side_effect_level)
        if side_effect is SideEffectLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "real-hardware tool definitions are prohibited",
                "side_effect_level",
            )
        object.__setattr__(self, "side_effect_level", side_effect)
        approval = _parse_approval(self.required_approval_level)
        if approval is ApprovalLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "approval level 5 is prohibited",
                "required_approval_level",
            )
        object.__setattr__(self, "required_approval_level", approval)
        gates = _string_tuple(self.runtime_gate_names, "runtime_gate_names")
        object.__setattr__(self, "runtime_gate_names", gates)
        if isinstance(self.timeout_sec, bool) or not isinstance(self.timeout_sec, (int, float)):
            raise _error("timeout_invalid", "timeout_sec must be a positive finite number", "timeout_sec")
        timeout = float(self.timeout_sec)
        if not isfinite(timeout) or timeout <= 0.0 or timeout > 300.0:
            raise _error("timeout_invalid", "timeout_sec must be greater than zero and at most 300", "timeout_sec")
        object.__setattr__(self, "timeout_sec", timeout)
        _non_empty(self.fallback_behavior, "fallback_behavior")
        audit_fields = _string_tuple(
            self.audit_field_names, "audit_field_names", require_non_empty=True
        )
        object.__setattr__(self, "audit_field_names", audit_fields)
        if not isinstance(self.enabled, bool):
            raise _error("invalid_contract", "enabled must be a boolean", "enabled")
        object.__setattr__(self, "metadata", _copy_metadata(self.metadata))
        self._validate_side_effect_consistency()

    def _validate_side_effect_consistency(self) -> None:
        effect = self.side_effect_level
        approval = self.required_approval_level
        if effect is SideEffectLevel.READ_ONLY and approval is not ApprovalLevel.READ_ONLY:
            raise _error("approval_invalid", "READ_ONLY tools require approval level 0", "required_approval_level")
        if effect is SideEffectLevel.STATE_PROPOSAL:
            if approval is not ApprovalLevel.PROPOSAL:
                raise _error("approval_invalid", "STATE_PROPOSAL tools require approval level 1", "required_approval_level")
            if self.runtime_gate_names:
                raise _error("runtime_gate_missing", "STATE_PROPOSAL tools must not declare runtime gates", "runtime_gate_names")
        if effect is SideEffectLevel.VALIDATED_STATE_WRITE and approval not in (
            ApprovalLevel.PROPOSAL,
            ApprovalLevel.MOCK_SIMULATION,
            ApprovalLevel.EXTERNAL_SIMULATOR,
            ApprovalLevel.SITL_COMMAND,
        ):
            raise _error("approval_invalid", "VALIDATED_STATE_WRITE requires approval level 1 through 4", "required_approval_level")
        if effect is SideEffectLevel.SIMULATOR_READ_ONLY and approval not in (
            ApprovalLevel.READ_ONLY,
            ApprovalLevel.EXTERNAL_SIMULATOR,
        ):
            raise _error("approval_invalid", "SIMULATOR_READ_ONLY requires approval level 0 or 3", "required_approval_level")
        if effect is SideEffectLevel.SIMULATOR_READ_ONLY and approval is ApprovalLevel.EXTERNAL_SIMULATOR and not self.runtime_gate_names:
            raise _error("runtime_gate_required", "external simulator reads require runtime gates", "runtime_gate_names")
        if effect is SideEffectLevel.SIMULATOR_COMMAND_GATED:
            if approval is not ApprovalLevel.EXTERNAL_SIMULATOR:
                raise _error("approval_invalid", "SIMULATOR_COMMAND_GATED requires approval level 3", "required_approval_level")
            if not self.runtime_gate_names:
                raise _error("runtime_gate_required", "simulator commands require runtime gates", "runtime_gate_names")
        if effect is SideEffectLevel.SITL_COMMAND_GATED:
            if approval is not ApprovalLevel.SITL_COMMAND:
                raise _error("approval_invalid", "SITL_COMMAND_GATED requires approval level 4", "required_approval_level")
            if not self.runtime_gate_names:
                raise _error("runtime_gate_required", "SITL commands require runtime gates", "runtime_gate_names")

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "tool_name": self.tool_name,
            "tool_version": self.tool_version,
            "description": self.description,
            "allowed_agent_roles": [role.value for role in self.allowed_agent_roles],
            "input_schema_id": self.input_schema_id,
            "output_schema_id": self.output_schema_id,
            "access_class": self.access_class,
            "side_effect_level": self.side_effect_level.value,
            "required_approval_level": int(self.required_approval_level),
            "runtime_gate_names": list(self.runtime_gate_names),
            "timeout_sec": self.timeout_sec,
            "fallback_behavior": self.fallback_behavior,
            "audit_field_names": list(self.audit_field_names),
            "enabled": self.enabled,
            "metadata": deepcopy(self.metadata),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AgentToolDefinition":
        if not isinstance(data, dict):
            raise _error("invalid_contract", "from_dict requires a dictionary")
        expected = {item.name: item for item in fields(cls)}
        unknown = sorted(set(data) - set(expected))
        if unknown:
            raise _error("invalid_contract", "tool definition contains unknown fields", unknown[0])
        missing = [
            name
            for name, item in expected.items()
            if item.default is MISSING and item.default_factory is MISSING and name not in data
        ]
        if missing:
            raise _error("invalid_contract", "tool definition is missing required fields", missing[0])
        _validate_json_safe(data, "tool_definition")
        try:
            return cls(**deepcopy(data))
        except AgentContractError:
            raise
        except TypeError:
            raise _error("invalid_contract", "tool definition could not be constructed") from None


class AgentToolRegistry:
    """Exact registry with deny-by-default authorization and mock-only invocation."""

    def __init__(self, known_schema_ids: Optional[Sequence[str]] = None) -> None:
        schema_source = DEFAULT_AGENT_TOOL_SCHEMA_IDS if known_schema_ids is None else known_schema_ids
        self._known_schema_ids = frozenset(_string_tuple(schema_source, "known_schema_ids", require_non_empty=True))
        self._tools: Dict[Tuple[str, str], AgentToolDefinition] = {}

    def register(self, tool_definition: AgentToolDefinition) -> AgentToolDefinition:
        if not isinstance(tool_definition, AgentToolDefinition):
            raise _error(
                "invalid_contract",
                "register requires an AgentToolDefinition",
                contract_name="AgentToolRegistry",
            )
        tool_definition.validate()
        if tool_definition.input_schema_id not in self._known_schema_ids:
            raise _error(
                "input_schema_mismatch",
                "input schema is not registered",
                "input_schema_id",
                "AgentToolRegistry",
            )
        if tool_definition.output_schema_id not in self._known_schema_ids:
            raise _error(
                "input_schema_mismatch",
                "output schema is not registered",
                "output_schema_id",
                "AgentToolRegistry",
            )
        key = (tool_definition.tool_name, tool_definition.tool_version)
        if key in self._tools:
            raise _error(
                "invalid_contract",
                "tool name and version are already registered",
                "tool_id",
                "AgentToolRegistry",
            )
        stored = AgentToolDefinition.from_dict(tool_definition.to_dict())
        self._tools[key] = stored
        return AgentToolDefinition.from_dict(stored.to_dict())

    def get(self, tool_name: str, tool_version: str) -> AgentToolDefinition:
        _non_empty(tool_name, "tool_name")
        _non_empty(tool_version, "tool_version")
        key = (tool_name, tool_version)
        if key in self._tools:
            return AgentToolDefinition.from_dict(self._tools[key].to_dict())
        if any(name == tool_name for name, _ in self._tools):
            raise _error(
                "tool_version_mismatch",
                "the exact tool version is not registered",
                "tool_version",
                "AgentToolRegistry",
            )
        raise _error(
            "unknown_tool",
            "the exact tool name is not registered",
            "tool_name",
            "AgentToolRegistry",
        )

    def list_tools(self, agent_role: Optional[AgentRole] = None) -> List[AgentToolDefinition]:
        parsed_role = None if agent_role is None else _parse_role(agent_role)
        definitions = []
        for key in sorted(self._tools):
            definition = self._tools[key]
            if parsed_role is None or parsed_role in definition.allowed_agent_roles:
                definitions.append(AgentToolDefinition.from_dict(definition.to_dict()))
        return definitions

    def authorize(
        self,
        request: ToolCallRequest,
        identity: AgentIdentity,
        permission_manifest: AgentPermissionManifest,
        *,
        approval_request: Optional[ApprovalRequest] = None,
        approval_decision: Optional[ApprovalDecision] = None,
        gates: Optional[Mapping[str, bool]] = None,
    ) -> AgentToolDefinition:
        if not isinstance(request, ToolCallRequest):
            raise _error(
                "invalid_contract",
                "authorization requires a ToolCallRequest",
                contract_name="AgentToolRegistry",
            )
        request.validate()
        if not isinstance(identity, AgentIdentity):
            raise _error(
                "invalid_agent_identity",
                "authorization requires an AgentIdentity",
                contract_name="AgentToolRegistry",
            )
        identity.validate()
        if not identity.enabled or request.caller_agent_id != identity.agent_id:
            raise _error(
                "invalid_agent_identity",
                "the caller identity is disabled or does not match the request",
                "caller_agent_id",
                "AgentToolRegistry",
            )
        if permission_manifest is None:
            raise _error(
                "missing_permission_manifest",
                "a permission manifest is required",
                contract_name="AgentToolRegistry",
            )
        if not isinstance(permission_manifest, AgentPermissionManifest):
            raise _error(
                "missing_permission_manifest",
                "authorization requires an AgentPermissionManifest",
                contract_name="AgentToolRegistry",
            )
        permission_manifest.validate()
        if identity.role is not permission_manifest.agent_role:
            raise _error(
                "permission_denied",
                "identity role does not match the permission manifest",
                "agent_role",
                "AgentToolRegistry",
            )
        tool = self.get(request.tool_name, request.tool_version)
        if not tool.enabled:
            raise _error(
                "tool_disabled",
                "the exact tool definition is disabled",
                "enabled",
                "AgentToolRegistry",
            )
        if identity.role not in tool.allowed_agent_roles:
            raise _error(
                "permission_denied",
                "the exact agent role is not allowed for this tool",
                "allowed_agent_roles",
                "AgentToolRegistry",
            )
        if tool.tool_name not in identity.capability_ids:
            raise _error(
                "unknown_capability",
                "the identity does not declare the exact tool capability",
                "capability_ids",
                "AgentToolRegistry",
            )
        if tool.tool_name not in permission_manifest.allowed_capability_ids:
            raise _error(
                "unknown_capability",
                "the manifest does not allow the exact tool capability",
                "allowed_capability_ids",
                "AgentToolRegistry",
            )
        if tool.tool_id not in permission_manifest.allowed_tool_ids:
            raise _error(
                "permission_denied",
                "the manifest does not allow the exact tool identifier",
                "allowed_tool_ids",
                "AgentToolRegistry",
            )
        if permission_manifest.is_explicitly_denied(
            tool_id=tool.tool_id,
            tool_name=tool.tool_name,
            capability_id=tool.tool_name,
            access_class=tool.access_class,
            side_effect_level=tool.side_effect_level,
            approval_level=tool.required_approval_level,
        ):
            raise _error(
                "permission_denied",
                "an exact explicit denial overrides the allowlist",
                "explicit_denials",
                "AgentToolRegistry",
            )
        if tool.side_effect_level not in permission_manifest.allowed_side_effect_levels:
            raise _error(
                "side_effect_not_allowed",
                "the tool side effect is not allowed by the manifest",
                "allowed_side_effect_levels",
                "AgentToolRegistry",
            )
        if not permission_manifest.allows_scope(tool.access_class, tool.side_effect_level):
            raise _error(
                "permission_denied",
                "the exact access class is not allowed for this side effect",
                "access_class",
                "AgentToolRegistry",
            )
        if tool.required_approval_level > permission_manifest.max_requestable_approval_level:
            raise _error(
                "approval_invalid",
                "the tool approval level exceeds the manifest ceiling",
                "required_approval_level",
                "AgentToolRegistry",
            )
        if request.timeout_sec > tool.timeout_sec:
            raise _error(
                "timeout_invalid",
                "the request timeout exceeds the tool timeout",
                "timeout_sec",
                "AgentToolRegistry",
            )
        if tool.required_approval_level in (
            ApprovalLevel.EXTERNAL_SIMULATOR,
            ApprovalLevel.SITL_COMMAND,
        ):
            self._validate_approval(request, tool, approval_request, approval_decision)
        self._validate_gates(request, tool, gates)
        return AgentToolDefinition.from_dict(tool.to_dict())

    def validate_request(
        self,
        request: ToolCallRequest,
        tool_definition: Optional[AgentToolDefinition] = None,
    ) -> ToolCallRequest:
        """Validate a request against one exact registered input schema."""

        if not isinstance(request, ToolCallRequest):
            raise _error(
                "invalid_contract",
                "validation requires a ToolCallRequest",
                contract_name="AgentToolRegistry",
            )
        request.validate()
        tool = (
            self.get(request.tool_name, request.tool_version)
            if tool_definition is None
            else tool_definition
        )
        if not isinstance(tool, AgentToolDefinition):
            raise _error(
                "invalid_contract",
                "validation requires an AgentToolDefinition",
                contract_name="AgentToolRegistry",
            )
        tool.validate()
        if tool.tool_name != request.tool_name or tool.tool_version != request.tool_version:
            raise _error(
                "input_schema_mismatch",
                "request and tool identifiers do not match",
                "tool_name",
                "AgentToolRegistry",
            )
        if tool.input_schema_id not in self._known_schema_ids:
            raise _error(
                "input_schema_mismatch",
                "registered input schema is unknown",
                "input_schema_id",
                "AgentToolRegistry",
            )
        if not isinstance(request.parameters, dict):
            raise _error(
                "input_schema_mismatch",
                "mock tool parameters must be a dictionary",
                "parameters",
                "AgentToolRegistry",
            )
        declared_schema = request.metadata.get("input_schema_id")
        if declared_schema is not None and declared_schema != tool.input_schema_id:
            raise _error(
                "input_schema_mismatch",
                "request input schema does not match the registered tool",
                "input_schema_id",
                "AgentToolRegistry",
            )
        return ToolCallRequest.from_dict(request.to_dict())

    def validate_result(
        self,
        tool_definition: AgentToolDefinition,
        output: Any,
    ) -> Dict[str, Any]:
        """Validate and copy the closed mock-handler output envelope."""

        if not isinstance(tool_definition, AgentToolDefinition):
            raise _error(
                "invalid_contract",
                "result validation requires an AgentToolDefinition",
                contract_name="AgentToolRegistry",
            )
        tool_definition.validate()
        if not isinstance(output, dict):
            raise _error(
                "output_schema_mismatch",
                "mock tool output must be a dictionary",
                "output",
                "AgentToolRegistry",
            )
        if any(not isinstance(key, str) for key in output):
            raise _error(
                "output_schema_mismatch",
                "mock tool output keys must be strings",
                "output",
                "AgentToolRegistry",
            )
        required = {"schema_id", "result_summary"}
        allowed = required | {"evidence_refs", "metadata"}
        unknown = sorted(set(output) - allowed)
        if unknown:
            raise _error(
                "output_schema_mismatch",
                "mock tool output contains unknown fields",
                unknown[0],
                "AgentToolRegistry",
            )
        missing = sorted(required - set(output))
        if missing:
            raise _error(
                "output_schema_mismatch",
                "mock tool output is missing required fields",
                missing[0],
                "AgentToolRegistry",
            )
        if output["schema_id"] != tool_definition.output_schema_id:
            raise _error(
                "output_schema_mismatch",
                "mock tool output schema does not match the registered tool",
                "schema_id",
                "AgentToolRegistry",
            )
        if not isinstance(output["result_summary"], dict):
            raise _error(
                "output_schema_mismatch",
                "result_summary must be a dictionary",
                "result_summary",
                "AgentToolRegistry",
            )
        evidence_refs = output.get("evidence_refs", [])
        if (
            not isinstance(evidence_refs, list)
            or any(not isinstance(item, str) or not item for item in evidence_refs)
            or len(set(evidence_refs)) != len(evidence_refs)
        ):
            raise _error(
                "output_schema_mismatch",
                "evidence_refs must contain unique non-empty strings",
                "evidence_refs",
                "AgentToolRegistry",
            )
        metadata = output.get("metadata", {})
        if not isinstance(metadata, dict):
            raise _error(
                "output_schema_mismatch",
                "metadata must be a dictionary",
                "metadata",
                "AgentToolRegistry",
            )
        _validate_json_safe(output, "output")
        return {
            "schema_id": output["schema_id"],
            "result_summary": deepcopy(output["result_summary"]),
            "evidence_refs": deepcopy(evidence_refs),
            "metadata": deepcopy(metadata),
        }

    def invoke_mock(
        self,
        request: ToolCallRequest,
        identity: AgentIdentity,
        permission_manifest: AgentPermissionManifest,
        *,
        mock_handler: Callable[[Dict[str, Any]], Any],
        audit_log: AgentAuditLog,
        approval_request: Optional[ApprovalRequest] = None,
        approval_decision: Optional[ApprovalDecision] = None,
        gates: Optional[Mapping[str, bool]] = None,
        started_at: float = 0.0,
        mock_elapsed_sec: float = 0.0,
    ) -> ToolCallResult:
        """Run one explicitly injected mock callable after exact authorization."""

        if not isinstance(request, ToolCallRequest):
            raise _error(
                "invalid_contract",
                "mock invocation requires a ToolCallRequest",
                contract_name="AgentToolRegistry",
            )
        request.validate()
        if not callable(mock_handler):
            raise _error(
                "invalid_contract",
                "mock_handler must be an explicitly injected callable",
                "mock_handler",
                "AgentToolRegistry",
            )
        if not isinstance(audit_log, AgentAuditLog):
            raise _error(
                "invalid_contract",
                "audit_log must be an AgentAuditLog",
                "audit_log",
                "AgentToolRegistry",
            )
        workflow_id = self._workflow_id(request)

        try:
            tool = self.authorize(
                request,
                identity,
                permission_manifest,
                approval_request=approval_request,
                approval_decision=approval_decision,
                gates=gates,
            )
        except AgentContractError as exc:
            level = self._required_level_if_registered(request)
            approval_state, approval_summary = self._approval_summary(
                level, approval_request, approval_decision
            )
            metadata = self._invocation_metadata(approval_summary, request, gates)
            errors = [self._safe_error(exc.code, exc.message)]
            audit_time = self._safe_audit_time(started_at)
            self._append_audit(
                audit_log,
                request=request,
                workflow_id=workflow_id,
                event_type="tool_call_denied",
                timestamp=audit_time,
                actor_id=self._actor_id(identity, request),
                errors=errors,
                decision="denied",
                approval_state=approval_state,
                metadata=metadata,
            )
            return self._build_result(
                request,
                status=ToolCallStatus.DENIED,
                result_summary={},
                evidence_refs=[],
                errors=errors,
                started_at=audit_time,
                completed_at=audit_time,
                metadata=metadata,
            )

        start = self._validate_mock_time(started_at, "started_at")
        elapsed = self._validate_mock_time(mock_elapsed_sec, "mock_elapsed_sec")
        completed = start + elapsed
        approval_state, approval_summary = self._approval_summary(
            int(tool.required_approval_level), approval_request, approval_decision
        )
        metadata = self._invocation_metadata(approval_summary, request, gates)

        if elapsed > request.timeout_sec or elapsed > tool.timeout_sec:
            errors = [
                self._safe_error(
                    "tool_call_timed_out",
                    "mock elapsed time exceeds an authorized timeout",
                )
            ]
            self._append_audit(
                audit_log,
                request=request,
                workflow_id=workflow_id,
                event_type="tool_call_timed_out",
                timestamp=completed,
                actor_id=identity.agent_id,
                validated_parameters_summary=request.parameters,
                errors=errors,
                decision="timed_out",
                approval_state=approval_state,
                metadata=metadata,
            )
            return self._build_result(
                request,
                status=ToolCallStatus.TIMED_OUT,
                result_summary={},
                evidence_refs=[],
                errors=errors,
                started_at=start,
                completed_at=completed,
                metadata=metadata,
            )

        try:
            validated_request = self.validate_request(request, tool)
        except AgentContractError as exc:
            errors = [self._safe_error(exc.code, exc.message)]
            self._append_audit(
                audit_log,
                request=request,
                workflow_id=workflow_id,
                event_type="tool_call_failed",
                timestamp=start,
                actor_id=identity.agent_id,
                errors=errors,
                decision="failed",
                approval_state=approval_state,
                metadata=metadata,
            )
            return self._build_result(
                request,
                status=ToolCallStatus.FAILED,
                result_summary={},
                evidence_refs=[],
                errors=errors,
                started_at=start,
                completed_at=start,
                metadata=metadata,
            )

        self._append_audit(
            audit_log,
            request=validated_request,
            workflow_id=workflow_id,
            event_type="tool_call_started",
            timestamp=start,
            actor_id=identity.agent_id,
            validated_parameters_summary=validated_request.parameters,
            decision="started",
            approval_state=approval_state,
            metadata=metadata,
        )
        try:
            raw_output = mock_handler(deepcopy(validated_request.parameters))
        except Exception:
            errors = [
                self._safe_error(
                    "tool_call_failed",
                    "explicitly injected mock handler failed",
                )
            ]
            self._append_audit(
                audit_log,
                request=validated_request,
                workflow_id=workflow_id,
                event_type="tool_call_failed",
                timestamp=completed,
                actor_id=identity.agent_id,
                validated_parameters_summary=validated_request.parameters,
                errors=errors,
                decision="failed",
                approval_state=approval_state,
                metadata=metadata,
            )
            return self._build_result(
                validated_request,
                status=ToolCallStatus.FAILED,
                result_summary={},
                evidence_refs=[],
                errors=errors,
                started_at=start,
                completed_at=completed,
                metadata=metadata,
            )

        try:
            output = self.validate_result(tool, raw_output)
        except AgentContractError as exc:
            errors = [self._safe_error(exc.code, exc.message)]
            self._append_audit(
                audit_log,
                request=validated_request,
                workflow_id=workflow_id,
                event_type="tool_result_invalid",
                timestamp=completed,
                actor_id=identity.agent_id,
                validated_parameters_summary=validated_request.parameters,
                errors=errors,
                decision="failed",
                approval_state=approval_state,
                metadata=metadata,
            )
            return self._build_result(
                validated_request,
                status=ToolCallStatus.FAILED,
                result_summary={},
                evidence_refs=[],
                errors=errors,
                started_at=start,
                completed_at=completed,
                metadata=metadata,
            )

        result_metadata = deepcopy(metadata)
        result_metadata["output_schema_id"] = output["schema_id"]
        result_metadata["mock_output_metadata"] = output["metadata"]
        result = self._build_result(
            validated_request,
            status=ToolCallStatus.PASSED,
            result_summary=output["result_summary"],
            evidence_refs=output["evidence_refs"],
            errors=[],
            started_at=start,
            completed_at=completed,
            metadata=result_metadata,
        )
        self._append_audit(
            audit_log,
            request=validated_request,
            workflow_id=workflow_id,
            event_type="tool_call_completed",
            timestamp=completed,
            actor_id=identity.agent_id,
            validated_parameters_summary=validated_request.parameters,
            result_summary=result.result_summary,
            decision="passed",
            approval_state=approval_state,
            metadata=result_metadata,
        )
        return ToolCallResult.from_dict(result.to_dict())

    @staticmethod
    def _validate_mock_time(value: Any, field_name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise _error(
                "invalid_contract",
                f"{field_name} must be a finite non-negative number",
                field_name,
                "AgentToolRegistry",
            )
        parsed = float(value)
        if not isfinite(parsed) or parsed < 0.0:
            raise _error(
                "invalid_contract",
                f"{field_name} must be a finite non-negative number",
                field_name,
                "AgentToolRegistry",
            )
        return parsed

    @staticmethod
    def _safe_audit_time(value: Any) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return 0.0
        parsed = float(value)
        return parsed if isfinite(parsed) and parsed >= 0.0 else 0.0

    @staticmethod
    def _workflow_id(request: ToolCallRequest) -> str:
        value = request.metadata.get("workflow_id")
        return value if isinstance(value, str) and value else "workflow-unscoped"

    @staticmethod
    def _actor_id(identity: Any, request: ToolCallRequest) -> str:
        return identity.agent_id if isinstance(identity, AgentIdentity) else request.caller_agent_id

    def _required_level_if_registered(self, request: ToolCallRequest) -> Optional[int]:
        try:
            return int(self.get(request.tool_name, request.tool_version).required_approval_level)
        except AgentContractError:
            return None

    @staticmethod
    def _approval_summary(
        required_level: Optional[int],
        approval_request: Optional[ApprovalRequest],
        approval_decision: Optional[ApprovalDecision],
    ) -> Tuple[str, Dict[str, Any]]:
        outcome = None
        if isinstance(approval_decision, ApprovalDecision):
            outcome = approval_decision.outcome.value
        human_decision_required = required_level in (3, 4)
        state = outcome or ("missing" if human_decision_required else "not_required")
        return state, {
            "required_level": required_level,
            "approval_ref_present": approval_request is not None,
            "decision_present": approval_decision is not None,
            "outcome": outcome,
            "human_decision_required": human_decision_required,
        }

    @staticmethod
    def _invocation_metadata(
        approval_summary: Dict[str, Any],
        request: ToolCallRequest,
        gates: Optional[Mapping[str, bool]],
    ) -> Dict[str, Any]:
        supplied = {}
        if isinstance(gates, dict) and all(
            isinstance(name, str) and isinstance(value, bool)
            for name, value in gates.items()
        ):
            supplied = {name: gates[name] for name in sorted(gates)}
        return {
            "mock_only": True,
            "approval_summary": deepcopy(approval_summary),
            "runtime_gate_summary": {
                "request_refs": sorted(request.runtime_gate_refs),
                "supplied": supplied,
            },
        }

    @staticmethod
    def _safe_error(code: str, message: str) -> Dict[str, str]:
        return {"code": code, "message": message}

    @staticmethod
    def _build_result(
        request: ToolCallRequest,
        *,
        status: ToolCallStatus,
        result_summary: Dict[str, Any],
        evidence_refs: Sequence[str],
        errors: Sequence[Dict[str, str]],
        started_at: float,
        completed_at: float,
        metadata: Dict[str, Any],
    ) -> ToolCallResult:
        return ToolCallResult(
            request_id=request.request_id,
            tool_name=request.tool_name,
            tool_version=request.tool_version,
            status=status,
            result_summary=deepcopy(result_summary),
            evidence_refs=list(evidence_refs),
            errors=deepcopy(list(errors)),
            started_at=started_at,
            completed_at=completed_at,
            metadata=deepcopy(metadata),
        )

    @staticmethod
    def _append_audit(
        audit_log: AgentAuditLog,
        *,
        request: ToolCallRequest,
        workflow_id: str,
        event_type: str,
        timestamp: float,
        actor_id: str,
        validated_parameters_summary: Any = None,
        result_summary: Any = None,
        errors: Sequence[Dict[str, str]] = (),
        decision: Optional[str] = None,
        approval_state: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        refs = []
        for value in (
            request.request_id,
            f"{request.tool_name}@{request.tool_version}",
            request.caller_agent_id,
        ):
            if value not in refs:
                refs.append(value)
        audit_log.append(
            audit_log.build_record(
                workflow_id=workflow_id,
                event_type=event_type,
                actor_id=actor_id,
                timestamp=timestamp,
                input_refs=refs,
                tool_name=request.tool_name,
                tool_version=request.tool_version,
                validated_parameters_summary=validated_parameters_summary,
                result_summary=result_summary,
                decision=decision,
                approval_state=approval_state,
                errors=errors,
                metadata=metadata,
            )
        )

    @staticmethod
    def _validate_approval(
        request: ToolCallRequest,
        tool: AgentToolDefinition,
        approval_request: Optional[ApprovalRequest],
        approval_decision: Optional[ApprovalDecision],
    ) -> None:
        if request.approval_ref is None or approval_request is None or approval_decision is None:
            raise _error(
                "approval_required",
                "level 3 and 4 tools require approval request and decision metadata",
                "approval_ref",
                "AgentToolRegistry",
            )
        if not isinstance(approval_request, ApprovalRequest) or not isinstance(
            approval_decision, ApprovalDecision
        ):
            raise _error(
                "approval_invalid",
                "approval metadata contracts are invalid",
                contract_name="AgentToolRegistry",
            )
        approval_request.validate()
        approval_decision.validate()
        if request.approval_ref != approval_request.approval_request_id:
            raise _error(
                "approval_invalid",
                "request approval reference does not match",
                "approval_ref",
                "AgentToolRegistry",
            )
        if approval_request.approval_level is not tool.required_approval_level:
            raise _error(
                "approval_invalid",
                "approval level does not match the tool requirement",
                "approval_level",
                "AgentToolRegistry",
            )
        if approval_decision.approval_request_id != approval_request.approval_request_id:
            raise _error(
                "approval_invalid",
                "approval decision does not match the approval request",
                "approval_request_id",
                "AgentToolRegistry",
            )
        if approval_decision.outcome is not ApprovalOutcome.APPROVED:
            raise _error(
                "approval_invalid",
                "approval outcome is not approved",
                "outcome",
                "AgentToolRegistry",
            )
        if approval_decision.timestamp > approval_request.expires_at:
            raise _error(
                "approval_expired",
                "approval decision timestamp is later than the expiry",
                "timestamp",
                "AgentToolRegistry",
            )
        if request.request_id not in approval_request.target_refs and tool.tool_id not in approval_request.target_refs:
            raise _error(
                "approval_invalid",
                "approval target does not match the request or tool",
                "target_refs",
                "AgentToolRegistry",
            )

    @staticmethod
    def _validate_gates(
        request: ToolCallRequest,
        tool: AgentToolDefinition,
        gates: Optional[Mapping[str, bool]],
    ) -> None:
        if not tool.runtime_gate_names:
            return
        if gates is None:
            raise _error(
                "runtime_gate_required",
                "runtime gate metadata is required",
                "runtime_gate_names",
                "AgentToolRegistry",
            )
        if not isinstance(gates, dict) or any(
            not isinstance(name, str) or not isinstance(value, bool)
            for name, value in gates.items()
        ):
            raise _error(
                "invalid_contract",
                "runtime gate metadata must be a string-to-boolean dictionary",
                "gates",
                "AgentToolRegistry",
            )
        if set(request.runtime_gate_refs) != set(tool.runtime_gate_names):
            raise _error(
                "runtime_gate_missing",
                "request runtime gate references do not exactly match the tool",
                "runtime_gate_refs",
                "AgentToolRegistry",
            )
        if any(gates.get(name) is not True for name in tool.runtime_gate_names):
            raise _error(
                "runtime_gate_missing",
                "a required runtime gate is missing or false",
                "gates",
                "AgentToolRegistry",
            )

    def clear(self) -> None:
        self._tools.clear()


def build_default_agent_tool_catalogue() -> List[AgentToolDefinition]:
    """Return the frozen 13-tool metadata catalogue in declared order."""

    rows = (
        (
            "mission.query",
            "Read a validated mission snapshot.",
            (AgentRole.SUPERVISOR, AgentRole.MISSION, AgentRole.SITUATION_AWARENESS, AgentRole.SAFETY_REVIEW, AgentRole.AUDIT_REPLAY),
            "mission_query_request_v1",
            "mission_snapshot_v1",
            "mission.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            5.0,
            "return_not_ready",
        ),
        (
            "mission.propose",
            "Build a validated mission proposal without mutating state.",
            (AgentRole.MISSION,),
            "mission_proposal_request_v1",
            "mission_proposal_v1",
            "mission.proposal",
            SideEffectLevel.STATE_PROPOSAL,
            ApprovalLevel.PROPOSAL,
            5.0,
            "return_no_change",
        ),
        (
            "fleet.query",
            "Read a validated fleet snapshot.",
            (AgentRole.SUPERVISOR, AgentRole.FLEET, AgentRole.SITUATION_AWARENESS, AgentRole.PLANNING_AIRSPACE, AgentRole.SAFETY_REVIEW, AgentRole.AUDIT_REPLAY),
            "fleet_query_request_v1",
            "fleet_snapshot_v1",
            "fleet.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            5.0,
            "return_not_ready",
        ),
        (
            "fleet.propose_assignment",
            "Build a validated fleet-assignment proposal without dispatch.",
            (AgentRole.FLEET,),
            "fleet_assignment_request_v1",
            "fleet_assignment_proposal_v1",
            "fleet.proposal",
            SideEffectLevel.STATE_PROPOSAL,
            ApprovalLevel.PROPOSAL,
            5.0,
            "return_no_change",
        ),
        (
            "situation.query",
            "Read reference-oriented situation context.",
            (AgentRole.SUPERVISOR, AgentRole.SITUATION_AWARENESS, AgentRole.DEFENSIVE_RISK, AgentRole.PLANNING_AIRSPACE, AgentRole.SAFETY_REVIEW, AgentRole.AUDIT_REPLAY),
            "situation_query_request_v1",
            "agent_context_v1",
            "situation.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            5.0,
            "return_not_ready",
        ),
        (
            "risk.evaluate",
            "Evaluate defensive risk and produce a bounded recommendation.",
            (AgentRole.DEFENSIVE_RISK, AgentRole.SAFETY_REVIEW),
            "risk_evaluation_request_v1",
            "threat_assessment_v1",
            "risk.proposal",
            SideEffectLevel.STATE_PROPOSAL,
            ApprovalLevel.PROPOSAL,
            5.0,
            "return_no_change",
        ),
        (
            "airspace.validate",
            "Validate route metadata against stored airspace constraints.",
            (AgentRole.PLANNING_AIRSPACE, AgentRole.SAFETY_REVIEW),
            "airspace_validation_request_v1",
            "airspace_verdict_v1",
            "airspace.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            5.0,
            "return_not_ready",
        ),
        (
            "route.score",
            "Score route candidates without route execution.",
            (AgentRole.PLANNING_AIRSPACE, AgentRole.SAFETY_REVIEW),
            "route_scoring_request_v1",
            "planned_route_v1",
            "route.proposal",
            SideEffectLevel.STATE_PROPOSAL,
            ApprovalLevel.PROPOSAL,
            5.0,
            "return_no_change",
        ),
        (
            "replay.build",
            "Build a deterministic replay payload from stored events.",
            (AgentRole.SITUATION_AWARENESS, AgentRole.AUDIT_REPLAY),
            "replay_request_v1",
            "replay_payload_v1",
            "replay.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            10.0,
            "return_partial_summary",
        ),
        (
            "metrics.build",
            "Build deterministic mission metrics from stored records.",
            (AgentRole.AUDIT_REPLAY,),
            "metrics_request_v1",
            "metric_summary_v1",
            "metrics.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            10.0,
            "return_partial_summary",
        ),
        (
            "report.build",
            "Build a redaction-ready metadata report.",
            (AgentRole.AUDIT_REPLAY,),
            "report_request_v1",
            "audit_report_v1",
            "report.read",
            SideEffectLevel.READ_ONLY,
            ApprovalLevel.READ_ONLY,
            10.0,
            "return_partial_summary",
        ),
        (
            "benchmark.readiness",
            "Read stored simulator-backend readiness metadata.",
            (AgentRole.SUPERVISOR, AgentRole.SIMULATION_VALIDATION, AgentRole.SAFETY_REVIEW, AgentRole.AUDIT_REPLAY),
            "benchmark_readiness_request_v1",
            "benchmark_readiness_result_v1",
            "simulator.readiness",
            SideEffectLevel.SIMULATOR_READ_ONLY,
            ApprovalLevel.READ_ONLY,
            10.0,
            "return_not_ready",
        ),
        (
            "approval.request",
            "Build a structural human-approval request record.",
            (AgentRole.SUPERVISOR, AgentRole.SAFETY_REVIEW),
            "approval_request_input_v1",
            "approval_request_v1",
            "approval.proposal",
            SideEffectLevel.STATE_PROPOSAL,
            ApprovalLevel.PROPOSAL,
            5.0,
            "return_no_change",
        ),
    )

    audit_fields = ("request_id", "caller_agent_id", "tool_id", "status", "errors")
    return [
        AgentToolDefinition(
            tool_name=name,
            tool_version="1",
            description=description,
            allowed_agent_roles=roles,
            input_schema_id=input_schema,
            output_schema_id=output_schema,
            access_class=access_class,
            side_effect_level=side_effect,
            required_approval_level=approval,
            runtime_gate_names=(),
            timeout_sec=timeout,
            fallback_behavior=fallback,
            audit_field_names=audit_fields,
            enabled=True,
            metadata={"catalogue": "agentops_v3_1b"},
        )
        for (
            name,
            description,
            roles,
            input_schema,
            output_schema,
            access_class,
            side_effect,
            approval,
            timeout,
            fallback,
        ) in rows
    ]


def build_default_agent_tool_registry() -> AgentToolRegistry:
    """Build the metadata-only default registry without invocation behavior."""

    registry = AgentToolRegistry()
    for definition in build_default_agent_tool_catalogue():
        registry.register(definition)
    return registry
