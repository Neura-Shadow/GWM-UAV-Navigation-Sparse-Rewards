"""Tests for the exact-match metadata-only AgentOps tool registry."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    AgentContractError,
    AgentIdentity,
    AgentPermissionManifest,
    AgentRole,
    AgentToolDefinition,
    AgentToolRegistry,
    ApprovalLevel,
    ApprovalDecision,
    ApprovalRequest,
    SideEffectLevel,
    ToolCallRequest,
    DEFAULT_AGENT_TOOL_NAMES,
    build_default_agent_permission_manifests,
    build_default_agent_tool_catalogue,
    build_default_agent_tool_registry,
)


def test_tool_definition_json_roundtrip() -> None:
    definition = AgentToolDefinition(
        tool_name="mission.query",
        tool_version="1",
        description="Read a validated mission snapshot.",
        allowed_agent_roles=[AgentRole.MISSION],
        input_schema_id="mission_query_request_v1",
        output_schema_id="mission_snapshot_v1",
        access_class="mission.read",
        side_effect_level=SideEffectLevel.READ_ONLY,
        required_approval_level=ApprovalLevel.READ_ONLY,
        runtime_gate_names=[],
        timeout_sec=5.0,
        fallback_behavior="return_not_ready",
        audit_field_names=["request_id", "caller_agent_id", "status"],
        enabled=True,
        metadata={"catalogue": "default"},
    )

    encoded = definition.to_dict()
    restored = AgentToolDefinition.from_dict(encoded)

    assert restored == definition
    assert restored.tool_id == "mission.query@1"
    assert json.loads(json.dumps(encoded, sort_keys=True)) == encoded


def _definition(**overrides: object) -> AgentToolDefinition:
    values = {
        "tool_name": "mission.query",
        "tool_version": "1",
        "description": "Read a validated mission snapshot.",
        "allowed_agent_roles": ["mission"],
        "input_schema_id": "mission_query_request_v1",
        "output_schema_id": "mission_snapshot_v1",
        "access_class": "mission.read",
        "side_effect_level": "READ_ONLY",
        "required_approval_level": 0,
        "runtime_gate_names": [],
        "timeout_sec": 5.0,
        "fallback_behavior": "return_not_ready",
        "audit_field_names": ["request_id", "caller_agent_id", "status"],
        "enabled": True,
        "metadata": {},
    }
    values.update(overrides)
    return AgentToolDefinition(**values)


def test_tool_definition_rejects_unknown_role() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(allowed_agent_roles=["Mission"])

    assert error.value.code == "unknown_agent_role"


def test_tool_definition_rejects_unknown_schema() -> None:
    registry = AgentToolRegistry()

    with pytest.raises(AgentContractError) as error:
        registry.register(_definition(input_schema_id="unknown_schema_v1"))

    assert error.value.code == "input_schema_mismatch"


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("tool_name", "mission.*"),
        ("tool_version", "*"),
        ("input_schema_id", "mission_*"),
        ("access_class", "mission.*"),
        ("fallback_behavior", "return_*"),
    ],
)
def test_tool_definition_rejects_wildcard(field_name: str, value: str) -> None:
    with pytest.raises(AgentContractError, match="wildcards"):
        _definition(**{field_name: value})


def test_tool_definition_rejects_level_five() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(required_approval_level=5)

    assert error.value.code == "real_hardware_prohibited"


def test_tool_definition_rejects_real_hardware_level() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(side_effect_level="REAL_HARDWARE_PROHIBITED")

    assert error.value.code == "real_hardware_prohibited"


def test_tool_definition_rejects_callable_metadata() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(metadata={"callback": lambda: None})

    assert error.value.code == "invalid_json_payload"


def test_tool_definition_enforces_read_only_level_zero() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(required_approval_level=1)

    assert error.value.code == "approval_invalid"


def test_tool_definition_enforces_state_proposal_level_one() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(side_effect_level="STATE_PROPOSAL", required_approval_level=0)

    assert error.value.code == "approval_invalid"


def test_tool_definition_requires_gates_for_simulator_command() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(side_effect_level="SIMULATOR_COMMAND_GATED", required_approval_level=3)

    assert error.value.code == "runtime_gate_required"


def test_tool_definition_requires_gates_for_sitl_command() -> None:
    with pytest.raises(AgentContractError) as error:
        _definition(side_effect_level="SITL_COMMAND_GATED", required_approval_level=4)

    assert error.value.code == "runtime_gate_required"


def test_tool_registry_registers_allowlisted_tool() -> None:
    registry = AgentToolRegistry()

    registered = registry.register(_definition())

    assert registered.tool_id == "mission.query@1"
    assert registry.get("mission.query", "1") == registered


def test_tool_registry_rejects_duplicate_version() -> None:
    registry = AgentToolRegistry()
    registry.register(_definition())

    with pytest.raises(AgentContractError, match="already registered"):
        registry.register(_definition(description="Duplicate metadata."))


def test_tool_registry_get_requires_exact_version() -> None:
    registry = AgentToolRegistry()
    registry.register(_definition())

    with pytest.raises(AgentContractError) as error:
        registry.get("mission.query", "2")

    assert error.value.code == "tool_version_mismatch"


def test_tool_registry_rejects_unknown_tool() -> None:
    registry = AgentToolRegistry()

    with pytest.raises(AgentContractError) as error:
        registry.get("route.score", "1")

    assert error.value.code == "unknown_tool"


def test_tool_registry_filters_by_exact_agent_role() -> None:
    registry = AgentToolRegistry()
    registry.register(_definition(allowed_agent_roles=["mission", "supervisor"]))

    assert [tool.tool_id for tool in registry.list_tools("mission")] == ["mission.query@1"]
    with pytest.raises(AgentContractError) as error:
        registry.list_tools("Mission")

    assert error.value.code == "unknown_agent_role"


def test_tool_registry_returns_defensive_copies() -> None:
    registry = AgentToolRegistry()
    registered = registry.register(_definition(metadata={"labels": ["frozen"]}))

    registered.metadata["labels"].append("changed")
    fetched = registry.get("mission.query", "1")
    fetched.metadata["labels"].append("also_changed")

    assert registry.get("mission.query", "1").metadata == {"labels": ["frozen"]}


def test_tool_registry_clear_is_test_only_and_deterministic() -> None:
    registry = AgentToolRegistry()
    registry.register(_definition())

    registry.clear()

    assert registry.list_tools() == []


def _identity(**overrides: object) -> AgentIdentity:
    values = {
        "agent_id": "agent-mission-1",
        "role": "mission",
        "version": "1",
        "capability_ids": ["mission.query"],
        "enabled": True,
        "metadata": {},
    }
    values.update(overrides)
    return AgentIdentity(**values)


def _manifest(**overrides: object) -> AgentPermissionManifest:
    values = {
        "manifest_id": "manifest-mission-1",
        "agent_role": "mission",
        "manifest_version": "1",
        "allowed_capability_ids": ["mission.query"],
        "allowed_tool_ids": ["mission.query@1"],
        "read_scopes": ["mission.read"],
        "proposal_scopes": [],
        "validated_write_scopes": [],
        "allowed_side_effect_levels": ["READ_ONLY"],
        "max_requestable_approval_level": 1,
        "explicit_denials": [],
        "metadata": {},
    }
    values.update(overrides)
    return AgentPermissionManifest(**values)


def _request(**overrides: object) -> ToolCallRequest:
    values = {
        "request_id": "request-1",
        "tool_name": "mission.query",
        "tool_version": "1",
        "caller_agent_id": "agent-mission-1",
        "parameters": {"mission_id": "mission-1"},
        "approval_ref": None,
        "timeout_sec": 5.0,
        "runtime_gate_refs": [],
        "metadata": {},
    }
    values.update(overrides)
    return ToolCallRequest(**values)


def _registry_with_query() -> AgentToolRegistry:
    registry = AgentToolRegistry()
    registry.register(_definition())
    return registry


def _gated_definition() -> AgentToolDefinition:
    return _definition(
        tool_name="simulator.validate",
        description="Validate an externally gated simulator session.",
        allowed_agent_roles=["simulation_validation"],
        input_schema_id="benchmark_readiness_request_v1",
        output_schema_id="benchmark_readiness_result_v1",
        access_class="simulator.validation",
        side_effect_level="SIMULATOR_COMMAND_GATED",
        required_approval_level=3,
        runtime_gate_names=["GWM_ALLOW_OPTIONAL_RUNTIME"],
    )


def _gated_authorization_values() -> tuple[
    AgentToolRegistry,
    ToolCallRequest,
    AgentIdentity,
    AgentPermissionManifest,
]:
    registry = AgentToolRegistry()
    registry.register(_gated_definition())
    request = _request(
        tool_name="simulator.validate",
        caller_agent_id="agent-simulation-1",
        approval_ref="approval-1",
        runtime_gate_refs=["GWM_ALLOW_OPTIONAL_RUNTIME"],
    )
    identity = _identity(
        agent_id="agent-simulation-1",
        role="simulation_validation",
        capability_ids=["simulator.validate"],
    )
    manifest = _manifest(
        manifest_id="manifest-simulation-1",
        agent_role="simulation_validation",
        allowed_capability_ids=["simulator.validate"],
        allowed_tool_ids=["simulator.validate@1"],
        read_scopes=[],
        proposal_scopes=["simulator.validation"],
        allowed_side_effect_levels=["SIMULATOR_COMMAND_GATED"],
        max_requestable_approval_level=3,
    )
    return registry, request, identity, manifest


def _approval_pair(
    *,
    request_id: str = "approval-1",
    target_refs: list[str] | None = None,
    decision_timestamp: float = 10.0,
    expires_at: float = 20.0,
    approval_level: int = 3,
) -> tuple[ApprovalRequest, ApprovalDecision]:
    approval_request = ApprovalRequest(
        approval_request_id=request_id,
        approval_level=approval_level,
        target_refs=target_refs or ["simulator.validate@1"],
        rationale="Authorize bounded simulator validation metadata.",
        evidence_refs=["safety-review-1"],
        requested_by="agent-supervisor-1",
        expires_at=expires_at,
    )
    approval_decision = ApprovalDecision(
        decision_id="approval-decision-1",
        approval_request_id=request_id,
        operator_id="operator-1",
        outcome="approved",
        timestamp=decision_timestamp,
        notes="Structurally approved for simulation only.",
    )
    return approval_request, approval_decision


def test_tool_registry_authorizes_exact_allowed_request() -> None:
    authorized = _registry_with_query().authorize(_request(), _identity(), _manifest())

    assert authorized.tool_id == "mission.query@1"


def test_tool_registry_denies_disabled_identity() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(_request(), _identity(enabled=False), _manifest())

    assert error.value.code == "invalid_agent_identity"


def test_tool_registry_denies_request_identity_mismatch() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(caller_agent_id="agent-other"), _identity(), _manifest()
        )

    assert error.value.code == "invalid_agent_identity"


def test_tool_registry_denies_missing_permission_manifest() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(_request(), _identity(), None)

    assert error.value.code == "missing_permission_manifest"


def test_tool_registry_denies_disabled_tool() -> None:
    registry = AgentToolRegistry()
    registry.register(_definition(enabled=False))

    with pytest.raises(AgentContractError) as error:
        registry.authorize(_request(), _identity(), _manifest())

    assert error.value.code == "tool_disabled"


def test_tool_registry_denies_role_manifest_mismatch() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(), _identity(), _manifest(agent_role="supervisor")
        )

    assert error.value.code == "permission_denied"


def test_tool_registry_denies_unlisted_agent() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(caller_agent_id="agent-supervisor-1"),
            _identity(
                agent_id="agent-supervisor-1",
                role="supervisor",
                capability_ids=["mission.query"],
            ),
            _manifest(agent_role="supervisor"),
        )

    assert error.value.code == "permission_denied"


def test_tool_registry_denies_missing_manifest_tool() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(), _identity(), _manifest(allowed_tool_ids=[])
        )

    assert error.value.code == "permission_denied"


def test_tool_registry_denies_missing_identity_capability() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(), _identity(capability_ids=[]), _manifest()
        )

    assert error.value.code == "unknown_capability"


def test_tool_registry_denies_missing_manifest_capability() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(), _identity(), _manifest(allowed_capability_ids=[])
        )

    assert error.value.code == "unknown_capability"


def test_tool_registry_denies_side_effect_escalation() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(),
            _identity(),
            _manifest(allowed_side_effect_levels=["STATE_PROPOSAL"]),
        )

    assert error.value.code == "side_effect_not_allowed"


def test_tool_registry_denies_scope_mismatch() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(), _identity(), _manifest(read_scopes=["fleet.read"])
        )

    assert error.value.code == "permission_denied"


def test_tool_registry_denies_approval_ceiling() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    manifest = AgentPermissionManifest.from_dict(
        {**manifest.to_dict(), "max_requestable_approval_level": 1}
    )

    with pytest.raises(AgentContractError) as error:
        registry.authorize(request, identity, manifest)

    assert error.value.code == "approval_invalid"


def test_tool_registry_denies_request_timeout_above_tool_timeout() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(timeout_sec=6.0), _identity(), _manifest()
        )

    assert error.value.code == "timeout_invalid"


def test_tool_registry_denies_explicit_tool_denial() -> None:
    with pytest.raises(AgentContractError) as error:
        _registry_with_query().authorize(
            _request(),
            _identity(),
            _manifest(explicit_denials=["tool:mission.query@1"]),
        )

    assert error.value.code == "permission_denied"


def test_tool_registry_requires_approval_for_level_three() -> None:
    registry, request, identity, manifest = _gated_authorization_values()

    with pytest.raises(AgentContractError) as error:
        registry.authorize(
            request,
            identity,
            manifest,
            gates={"GWM_ALLOW_OPTIONAL_RUNTIME": True},
        )

    assert error.value.code == "approval_required"


def test_tool_registry_rejects_mismatched_approval() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request, approval_decision = _approval_pair(request_id="approval-other")

    with pytest.raises(AgentContractError) as error:
        registry.authorize(
            request,
            identity,
            manifest,
            approval_request=approval_request,
            approval_decision=approval_decision,
            gates={"GWM_ALLOW_OPTIONAL_RUNTIME": True},
        )

    assert error.value.code == "approval_invalid"


def test_tool_registry_rejects_expired_approval_decision() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request, approval_decision = _approval_pair(
        decision_timestamp=21.0, expires_at=20.0
    )

    with pytest.raises(AgentContractError) as error:
        registry.authorize(
            request,
            identity,
            manifest,
            approval_request=approval_request,
            approval_decision=approval_decision,
            gates={"GWM_ALLOW_OPTIONAL_RUNTIME": True},
        )

    assert error.value.code == "approval_expired"


def test_tool_registry_requires_runtime_gate_metadata() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request, approval_decision = _approval_pair()

    with pytest.raises(AgentContractError) as error:
        registry.authorize(
            request,
            identity,
            manifest,
            approval_request=approval_request,
            approval_decision=approval_decision,
        )

    assert error.value.code == "runtime_gate_required"


def test_tool_registry_rejects_false_runtime_gate() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request, approval_decision = _approval_pair()

    with pytest.raises(AgentContractError) as error:
        registry.authorize(
            request,
            identity,
            manifest,
            approval_request=approval_request,
            approval_decision=approval_decision,
            gates={"GWM_ALLOW_OPTIONAL_RUNTIME": False},
        )

    assert error.value.code == "runtime_gate_missing"


def test_tool_registry_authorizes_structurally_valid_gated_metadata() -> None:
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request, approval_decision = _approval_pair()

    authorized = registry.authorize(
        request,
        identity,
        manifest,
        approval_request=approval_request,
        approval_decision=approval_decision,
        gates={"GWM_ALLOW_OPTIONAL_RUNTIME": True},
    )

    assert authorized.tool_id == "simulator.validate@1"


def test_tool_registry_authorizes_structurally_valid_level_four_metadata() -> None:
    definition = _definition(
        tool_name="sitl.validate",
        description="Validate gated SITL metadata without invoking a command.",
        allowed_agent_roles=["simulation_validation"],
        input_schema_id="benchmark_readiness_request_v1",
        output_schema_id="benchmark_readiness_result_v1",
        access_class="sitl.validation",
        side_effect_level="SITL_COMMAND_GATED",
        required_approval_level=4,
        runtime_gate_names=["GWM_ALLOW_SITL_COMMANDS"],
    )
    registry = AgentToolRegistry()
    registry.register(definition)
    request = _request(
        tool_name="sitl.validate",
        caller_agent_id="agent-simulation-1",
        approval_ref="approval-4",
        runtime_gate_refs=["GWM_ALLOW_SITL_COMMANDS"],
    )
    identity = _identity(
        agent_id="agent-simulation-1",
        role="simulation_validation",
        capability_ids=["sitl.validate"],
    )
    manifest = _manifest(
        manifest_id="manifest-simulation-1",
        agent_role="simulation_validation",
        allowed_capability_ids=["sitl.validate"],
        allowed_tool_ids=["sitl.validate@1"],
        read_scopes=[],
        proposal_scopes=["sitl.validation"],
        allowed_side_effect_levels=["SITL_COMMAND_GATED"],
        max_requestable_approval_level=4,
    )
    approval_request, approval_decision = _approval_pair(
        request_id="approval-4",
        target_refs=["sitl.validate@1"],
        approval_level=4,
    )

    authorized = registry.authorize(
        request,
        identity,
        manifest,
        approval_request=approval_request,
        approval_decision=approval_decision,
        gates={"GWM_ALLOW_SITL_COMMANDS": True},
    )

    assert authorized.tool_id == "sitl.validate@1"


def test_default_tool_catalogue_contains_exact_expected_tools() -> None:
    catalogue = build_default_agent_tool_catalogue()

    assert tuple(tool.tool_name for tool in catalogue) == DEFAULT_AGENT_TOOL_NAMES
    assert len(catalogue) == 13
    assert all(tool.tool_version == "1" for tool in catalogue)


def test_default_tool_catalogue_excludes_prohibited_tools() -> None:
    prohibited = {
        "vehicle.command",
        "vehicle.arm",
        "vehicle.takeoff",
        "vehicle.land",
        "route.execute",
        "mission.upload",
        "simulator.launch",
        "px4.launch",
        "hardware.connect",
        "shell.execute",
        "network.request",
    }

    assert prohibited.isdisjoint({tool.tool_name for tool in build_default_agent_tool_catalogue()})


def test_default_tool_catalogue_has_no_command_gated_tool() -> None:
    prohibited_effects = {
        SideEffectLevel.SIMULATOR_COMMAND_GATED,
        SideEffectLevel.SITL_COMMAND_GATED,
        SideEffectLevel.REAL_HARDWARE_PROHIBITED,
    }

    assert all(
        tool.side_effect_level not in prohibited_effects
        for tool in build_default_agent_tool_catalogue()
    )
    assert all(tool.runtime_gate_names == () for tool in build_default_agent_tool_catalogue())


def test_default_tool_registry_matches_catalogue() -> None:
    registry = build_default_agent_tool_registry()

    assert tuple(tool.tool_name for tool in registry.list_tools()) == tuple(
        sorted(DEFAULT_AGENT_TOOL_NAMES)
    )


def test_default_manifests_reference_registered_exact_tools() -> None:
    registry = build_default_agent_tool_registry()
    manifests = build_default_agent_permission_manifests()

    for role, manifest in manifests.items():
        registered = {tool.tool_id for tool in registry.list_tools(role)}
        assert set(manifest.allowed_tool_ids) <= registered


def test_default_tool_registry_has_no_invocation_api() -> None:
    registry = build_default_agent_tool_registry()

    for method_name in ("invoke", "invoke_mock", "execute", "dispatch", "load_plugin", "discover"):
        assert not hasattr(registry, method_name)


def test_agent_tool_registry_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "cosysairsim",
        "isaacsim",
        "mavsdk",
        "message_filters",
        "omni",
        "openai",
        "pxr",
        "rclpy",
    }

    assert runtime_modules.isdisjoint(sys.modules)
