"""Deny-by-default permission manifests for AgentOps metadata authorization."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import MISSING, dataclass, field, fields
from enum import Enum
from math import isfinite
from typing import Any, Dict, Optional, Tuple

from src.c2.agent_types import (
    AgentContractError,
    AgentRole,
    ApprovalLevel,
    SideEffectLevel,
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
    }
)
_DENIAL_PREFIXES = frozenset(
    {"tool", "tool_name", "capability", "scope", "side_effect", "approval_level"}
)


def _error(code: str, message: str, field_name: Optional[str] = None) -> AgentContractError:
    return AgentContractError(
        code,
        message,
        field_name=field_name,
        contract_name="AgentPermissionManifest",
    )


def _non_empty(value: Any, field_name: str) -> str:
    if isinstance(value, Enum) or not isinstance(value, str) or not value.strip():
        raise _error("invalid_contract", f"{field_name} must be a non-empty string", field_name)
    return value


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


def _string_tuple(value: Any, field_name: str, *, reject_wildcard: bool = True) -> Tuple[str, ...]:
    if not isinstance(value, (list, tuple)):
        raise _error("invalid_contract", f"{field_name} must be a list or tuple", field_name)
    result = tuple(_non_empty(item, field_name) for item in value)
    if len(set(result)) != len(result):
        raise _error("invalid_contract", f"{field_name} must contain unique values", field_name)
    if reject_wildcard and any("*" in item for item in result):
        raise _error("permission_denied", f"{field_name} must not contain wildcards", field_name)
    return result


def _parse_role(value: Any) -> AgentRole:
    if isinstance(value, AgentRole):
        return value
    if isinstance(value, Enum):
        raise _error("unknown_agent_role", "agent_role is not allowed", "agent_role")
    try:
        return AgentRole(value)
    except (TypeError, ValueError):
        raise _error("unknown_agent_role", "agent_role is not allowed", "agent_role") from None


def _parse_side_effect(value: Any) -> SideEffectLevel:
    if isinstance(value, SideEffectLevel):
        return value
    if isinstance(value, Enum):
        raise _error("side_effect_not_allowed", "side effect is not allowed", "allowed_side_effect_levels")
    try:
        return SideEffectLevel(value)
    except (TypeError, ValueError):
        raise _error("side_effect_not_allowed", "side effect is not allowed", "allowed_side_effect_levels") from None


def _parse_approval(value: Any) -> ApprovalLevel:
    if isinstance(value, ApprovalLevel):
        return value
    if isinstance(value, bool) or isinstance(value, Enum):
        raise _error("approval_invalid", "approval level is not allowed", "max_requestable_approval_level")
    try:
        return ApprovalLevel(value)
    except (TypeError, ValueError):
        raise _error("approval_invalid", "approval level is not allowed", "max_requestable_approval_level") from None


def _validate_tool_id(tool_id: str, field_name: str = "allowed_tool_ids") -> None:
    if tool_id.count("@") != 1:
        raise _error("invalid_contract", f"{field_name} must use name@version identifiers", field_name)
    name, version = tool_id.split("@", 1)
    _non_empty(name, field_name)
    _non_empty(version, field_name)


def _validate_denial(token: str) -> None:
    if "*" in token:
        raise _error("permission_denied", "explicit denials must not contain wildcards", "explicit_denials")
    if ":" not in token:
        raise _error("invalid_contract", "explicit denial token is malformed", "explicit_denials")
    prefix, value = token.split(":", 1)
    if prefix not in _DENIAL_PREFIXES or not value:
        raise _error("invalid_contract", "explicit denial token is malformed", "explicit_denials")
    _non_empty(value, "explicit_denials")
    if prefix == "tool":
        _validate_tool_id(value, "explicit_denials")
    elif prefix == "side_effect":
        try:
            SideEffectLevel(value)
        except ValueError:
            raise _error("side_effect_not_allowed", "explicit denial side effect is unknown", "explicit_denials") from None
    elif prefix == "approval_level":
        try:
            level = int(value)
        except ValueError:
            raise _error("approval_invalid", "explicit denial approval level is invalid", "explicit_denials") from None
        if str(level) != value or level < 0 or level > 5:
            raise _error("approval_invalid", "explicit denial approval level is invalid", "explicit_denials")


@dataclass(frozen=True)
class AgentPermissionManifest:
    """One exact-match, deny-by-default permission manifest."""

    manifest_id: str
    agent_role: AgentRole
    manifest_version: str
    allowed_capability_ids: Tuple[str, ...]
    allowed_tool_ids: Tuple[str, ...]
    read_scopes: Tuple[str, ...]
    proposal_scopes: Tuple[str, ...]
    validated_write_scopes: Tuple[str, ...]
    allowed_side_effect_levels: Tuple[SideEffectLevel, ...]
    max_requestable_approval_level: ApprovalLevel
    explicit_denials: Tuple[str, ...]
    metadata: Dict[str, Any] = field(default_factory=dict, compare=True)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        _non_empty(self.manifest_id, "manifest_id")
        object.__setattr__(self, "agent_role", _parse_role(self.agent_role))
        _non_empty(self.manifest_version, "manifest_version")
        for field_name in (
            "allowed_capability_ids",
            "allowed_tool_ids",
            "read_scopes",
            "proposal_scopes",
            "validated_write_scopes",
            "explicit_denials",
        ):
            object.__setattr__(self, field_name, _string_tuple(getattr(self, field_name), field_name))
        for tool_id in self.allowed_tool_ids:
            _validate_tool_id(tool_id)
        side_effects = tuple(_parse_side_effect(item) for item in self.allowed_side_effect_levels)
        if len(set(side_effects)) != len(side_effects):
            raise _error(
                "invalid_contract",
                "allowed_side_effect_levels must contain unique values",
                "allowed_side_effect_levels",
            )
        if SideEffectLevel.REAL_HARDWARE_PROHIBITED in side_effects:
            raise _error(
                "real_hardware_prohibited",
                "the real-hardware denial marker cannot be allowed",
                "allowed_side_effect_levels",
            )
        object.__setattr__(self, "allowed_side_effect_levels", side_effects)
        approval = _parse_approval(self.max_requestable_approval_level)
        if approval is ApprovalLevel.REAL_HARDWARE_PROHIBITED:
            raise _error(
                "real_hardware_prohibited",
                "approval level 5 is prohibited",
                "max_requestable_approval_level",
            )
        object.__setattr__(self, "max_requestable_approval_level", approval)
        for denial in self.explicit_denials:
            _validate_denial(denial)
        object.__setattr__(self, "metadata", _copy_metadata(self.metadata))

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return {
            "manifest_id": self.manifest_id,
            "agent_role": self.agent_role.value,
            "manifest_version": self.manifest_version,
            "allowed_capability_ids": list(self.allowed_capability_ids),
            "allowed_tool_ids": list(self.allowed_tool_ids),
            "read_scopes": list(self.read_scopes),
            "proposal_scopes": list(self.proposal_scopes),
            "validated_write_scopes": list(self.validated_write_scopes),
            "allowed_side_effect_levels": [item.value for item in self.allowed_side_effect_levels],
            "max_requestable_approval_level": int(self.max_requestable_approval_level),
            "explicit_denials": list(self.explicit_denials),
            "metadata": deepcopy(self.metadata),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "AgentPermissionManifest":
        if not isinstance(data, dict):
            raise _error("invalid_contract", "from_dict requires a dictionary")
        expected = {item.name: item for item in fields(cls)}
        unknown = sorted(set(data) - set(expected))
        if unknown:
            raise _error("invalid_contract", "manifest contains unknown fields", unknown[0])
        missing = [
            name
            for name, item in expected.items()
            if item.default is MISSING and item.default_factory is MISSING and name not in data
        ]
        if missing:
            raise _error("invalid_contract", "manifest is missing required fields", missing[0])
        _validate_json_safe(data, "manifest")
        try:
            return cls(**deepcopy(data))
        except AgentContractError:
            raise
        except TypeError:
            raise _error("invalid_contract", "manifest could not be constructed") from None

    def is_explicitly_denied(
        self,
        token: Optional[str] = None,
        *,
        tool_id: Optional[str] = None,
        tool_name: Optional[str] = None,
        capability_id: Optional[str] = None,
        access_class: Optional[str] = None,
        side_effect_level: Optional[SideEffectLevel] = None,
        approval_level: Optional[ApprovalLevel] = None,
    ) -> bool:
        candidates = []
        if token is not None:
            candidates.append(token)
        if tool_id is not None:
            candidates.append(f"tool:{tool_id}")
        if tool_name is not None:
            candidates.append(f"tool_name:{tool_name}")
        if capability_id is not None:
            candidates.append(f"capability:{capability_id}")
        if access_class is not None:
            candidates.append(f"scope:{access_class}")
        if side_effect_level is not None:
            parsed_side_effect = _parse_side_effect(side_effect_level)
            candidates.append(f"side_effect:{parsed_side_effect.value}")
        if approval_level is not None:
            parsed_approval = _parse_approval(approval_level)
            candidates.append(f"approval_level:{int(parsed_approval)}")
        return any(candidate in self.explicit_denials for candidate in candidates)

    def allows_tool(self, tool_id: str) -> bool:
        if not isinstance(tool_id, str) or tool_id not in self.allowed_tool_ids:
            return False
        tool_name = tool_id.split("@", 1)[0]
        return not self.is_explicitly_denied(tool_id=tool_id, tool_name=tool_name)

    def allows_capability(self, capability_id: str) -> bool:
        return capability_id in self.allowed_capability_ids and not self.is_explicitly_denied(
            capability_id=capability_id
        )

    def allows_side_effect(self, side_effect_level: SideEffectLevel) -> bool:
        try:
            parsed = _parse_side_effect(side_effect_level)
        except AgentContractError:
            return False
        if parsed is SideEffectLevel.REAL_HARDWARE_PROHIBITED:
            return False
        return parsed in self.allowed_side_effect_levels and not self.is_explicitly_denied(
            side_effect_level=parsed
        )

    def allows_scope(self, access_class: str, side_effect_level: SideEffectLevel) -> bool:
        if not isinstance(access_class, str):
            return False
        try:
            parsed = _parse_side_effect(side_effect_level)
        except AgentContractError:
            return False
        if self.is_explicitly_denied(access_class=access_class, side_effect_level=parsed):
            return False
        if parsed in (SideEffectLevel.READ_ONLY, SideEffectLevel.SIMULATOR_READ_ONLY):
            return access_class in self.read_scopes
        if parsed in (
            SideEffectLevel.STATE_PROPOSAL,
            SideEffectLevel.SIMULATOR_COMMAND_GATED,
            SideEffectLevel.SITL_COMMAND_GATED,
        ):
            return access_class in self.proposal_scopes
        if parsed is SideEffectLevel.VALIDATED_STATE_WRITE:
            return access_class in self.validated_write_scopes
        return False


def build_default_agent_permission_manifests() -> Dict[AgentRole, AgentPermissionManifest]:
    """Build the frozen nine-role manifest set without runtime access."""

    configs = {
        AgentRole.SUPERVISOR: {
            "tools": (
                "mission.query@1",
                "fleet.query@1",
                "situation.query@1",
                "benchmark.readiness@1",
                "approval.request@1",
            ),
            "read": ("mission.read", "fleet.read", "situation.read", "simulator.readiness"),
            "proposal": ("approval.proposal",),
            "effects": (
                SideEffectLevel.READ_ONLY,
                SideEffectLevel.STATE_PROPOSAL,
                SideEffectLevel.SIMULATOR_READ_ONLY,
            ),
            "approval": ApprovalLevel.SITL_COMMAND,
        },
        AgentRole.MISSION: {
            "tools": ("mission.query@1", "mission.propose@1"),
            "read": ("mission.read",),
            "proposal": ("mission.proposal",),
            "effects": (SideEffectLevel.READ_ONLY, SideEffectLevel.STATE_PROPOSAL),
            "approval": ApprovalLevel.PROPOSAL,
        },
        AgentRole.FLEET: {
            "tools": ("fleet.query@1", "fleet.propose_assignment@1"),
            "read": ("fleet.read",),
            "proposal": ("fleet.proposal",),
            "effects": (SideEffectLevel.READ_ONLY, SideEffectLevel.STATE_PROPOSAL),
            "approval": ApprovalLevel.PROPOSAL,
        },
        AgentRole.SITUATION_AWARENESS: {
            "tools": (
                "mission.query@1",
                "fleet.query@1",
                "situation.query@1",
                "replay.build@1",
            ),
            "read": ("mission.read", "fleet.read", "situation.read", "replay.read"),
            "proposal": (),
            "effects": (SideEffectLevel.READ_ONLY,),
            "approval": ApprovalLevel.READ_ONLY,
        },
        AgentRole.DEFENSIVE_RISK: {
            "tools": ("situation.query@1", "risk.evaluate@1"),
            "read": ("situation.read",),
            "proposal": ("risk.proposal",),
            "effects": (SideEffectLevel.READ_ONLY, SideEffectLevel.STATE_PROPOSAL),
            "approval": ApprovalLevel.PROPOSAL,
        },
        AgentRole.PLANNING_AIRSPACE: {
            "tools": (
                "fleet.query@1",
                "situation.query@1",
                "airspace.validate@1",
                "route.score@1",
            ),
            "read": ("fleet.read", "situation.read", "airspace.read"),
            "proposal": ("route.proposal",),
            "effects": (SideEffectLevel.READ_ONLY, SideEffectLevel.STATE_PROPOSAL),
            "approval": ApprovalLevel.PROPOSAL,
        },
        AgentRole.SIMULATION_VALIDATION: {
            "tools": ("benchmark.readiness@1",),
            "read": ("simulator.readiness",),
            "proposal": (),
            "effects": (SideEffectLevel.SIMULATOR_READ_ONLY,),
            "approval": ApprovalLevel.SITL_COMMAND,
        },
        AgentRole.SAFETY_REVIEW: {
            "tools": (
                "mission.query@1",
                "fleet.query@1",
                "situation.query@1",
                "risk.evaluate@1",
                "airspace.validate@1",
                "route.score@1",
                "benchmark.readiness@1",
                "approval.request@1",
            ),
            "read": (
                "mission.read",
                "fleet.read",
                "situation.read",
                "airspace.read",
                "simulator.readiness",
            ),
            "proposal": ("risk.proposal", "route.proposal", "approval.proposal"),
            "effects": (
                SideEffectLevel.READ_ONLY,
                SideEffectLevel.STATE_PROPOSAL,
                SideEffectLevel.SIMULATOR_READ_ONLY,
            ),
            "approval": ApprovalLevel.SITL_COMMAND,
        },
        AgentRole.AUDIT_REPLAY: {
            "tools": (
                "mission.query@1",
                "fleet.query@1",
                "situation.query@1",
                "replay.build@1",
                "metrics.build@1",
                "report.build@1",
                "benchmark.readiness@1",
            ),
            "read": (
                "mission.read",
                "fleet.read",
                "situation.read",
                "replay.read",
                "metrics.read",
                "report.read",
                "simulator.readiness",
            ),
            "proposal": (),
            "effects": (SideEffectLevel.READ_ONLY, SideEffectLevel.SIMULATOR_READ_ONLY),
            "approval": ApprovalLevel.READ_ONLY,
        },
    }

    manifests: Dict[AgentRole, AgentPermissionManifest] = {}
    for role, config in configs.items():
        tool_ids = tuple(config["tools"])
        manifests[role] = AgentPermissionManifest(
            manifest_id=f"manifest-{role.value}-1",
            agent_role=role,
            manifest_version="1",
            allowed_capability_ids=tuple(tool_id.split("@", 1)[0] for tool_id in tool_ids),
            allowed_tool_ids=tool_ids,
            read_scopes=tuple(config["read"]),
            proposal_scopes=tuple(config["proposal"]),
            validated_write_scopes=(),
            allowed_side_effect_levels=tuple(config["effects"]),
            max_requestable_approval_level=config["approval"],
            explicit_denials=(),
            metadata={"source": "v3_0_permission_freeze"},
        )
    return manifests
