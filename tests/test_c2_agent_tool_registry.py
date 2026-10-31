"""Tests for the exact-match metadata-only AgentOps tool registry."""

from __future__ import annotations

import builtins
from copy import deepcopy
import json
import sys

import pytest

from src.c2 import (
    AgentAuditLog,
    AgentAuditRecord,
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
    redact_for_audit,
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


def test_default_tool_registry_has_only_explicit_mock_invocation_api() -> None:
    registry = build_default_agent_tool_registry()

    assert hasattr(registry, "invoke_mock")
    for method_name in ("invoke", "execute", "dispatch", "load_plugin", "discover"):
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


def _append_audit_record(
    audit_log: AgentAuditLog,
    *,
    workflow_id: str = "wf-1",
    request_id: str = "request-1",
    event_type: str = "tool_call_started",
    timestamp: float = 1.0,
    result_summary: object | None = None,
) -> AgentAuditRecord:
    record = audit_log.build_record(
        workflow_id=workflow_id,
        event_type=event_type,
        actor_id="agent-1",
        timestamp=timestamp,
        input_refs=[request_id, "mission.query@1", "agent-1"],
        tool_name="mission.query",
        tool_version="1",
        result_summary={} if result_summary is None else result_summary,
        approval_state="not_required",
    )
    return audit_log.append(record)


def test_agent_audit_log_appends_in_order() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log)
    _append_audit_record(
        audit_log,
        request_id="request-2",
        event_type="tool_call_completed",
        timestamp=2.0,
    )

    assert [record.event_type for record in audit_log.list_records()] == [
        "tool_call_started",
        "tool_call_completed",
    ]


def test_agent_audit_log_record_ids_are_deterministic() -> None:
    audit_log = AgentAuditLog()

    assert audit_log.next_record_id() == "audit-000001"
    first = _append_audit_record(audit_log)
    second = _append_audit_record(audit_log, request_id="request-2")

    assert first.record_id == "audit-000001"
    assert second.record_id == "audit-000002"


def test_agent_audit_log_returns_defensive_copies() -> None:
    audit_log = AgentAuditLog()
    appended = _append_audit_record(audit_log, result_summary={"items": [1]})

    appended.result_summary["items"].append(2)
    listed = audit_log.list_records()
    listed[0].result_summary["items"].append(3)

    assert audit_log.list_records()[0].result_summary == {"items": [1]}


def test_agent_audit_log_filters_by_workflow() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log, workflow_id="wf-1")
    _append_audit_record(audit_log, workflow_id="wf-2", request_id="request-2")

    assert [record.workflow_id for record in audit_log.list_records("wf-2")] == ["wf-2"]
    assert audit_log.list_records("WF-2") == []


def test_agent_audit_log_finds_by_request() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log, request_id="request-1")
    _append_audit_record(audit_log, request_id="request-2")

    assert [record.record_id for record in audit_log.find_by_request("request-2")] == [
        "audit-000002"
    ]
    assert audit_log.find_by_request("Request-2") == []


def test_agent_audit_log_snapshot_is_json_safe() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log, result_summary={"ok": True})

    snapshot = audit_log.snapshot()

    assert snapshot["schema_version"] == "agent-audit-log-v1"
    assert snapshot["record_count"] == 1
    assert json.loads(json.dumps(snapshot, sort_keys=True)) == snapshot


def test_agent_audit_log_snapshot_restore() -> None:
    source = AgentAuditLog()
    _append_audit_record(source)
    _append_audit_record(source, request_id="request-2", event_type="tool_call_completed")
    restored = AgentAuditLog()

    restored.restore(source.snapshot())

    assert restored.snapshot() == source.snapshot()
    assert restored.next_record_id() == "audit-000003"


def test_agent_audit_log_restore_is_atomic() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log)
    original = audit_log.snapshot()
    invalid = deepcopy(original)
    invalid["records"][0]["event_type"] = "provider_payload"

    with pytest.raises(AgentContractError):
        audit_log.restore(invalid)

    assert audit_log.snapshot() == original


def test_agent_audit_log_redacts_sensitive_nested_keys() -> None:
    audit_log = AgentAuditLog()
    record = audit_log.build_record(
        workflow_id="wf-1",
        event_type="tool_call_completed",
        actor_id="agent-1",
        timestamp=1.0,
        result_summary={
            "safe": True,
            "nested": {"Token": "secret-value", "private_key": "key", "count": 2},
        },
    )

    stored = audit_log.append(record)

    assert stored.result_summary == {
        "safe": True,
        "nested": {"count": 2, "redacted_fields": ["private_key", "token"]},
    }


def test_agent_audit_log_rejects_private_reasoning() -> None:
    sanitized = redact_for_audit(
        {"answer": "hold", "private_chain_of_thought": "never store this"}
    )

    assert "private_chain_of_thought" not in sanitized
    assert sanitized["redacted_fields"] == ["private_chain_of_thought"]


def test_agent_audit_log_clear_is_test_only_and_deterministic() -> None:
    audit_log = AgentAuditLog()
    _append_audit_record(audit_log)

    audit_log.clear()

    assert audit_log.list_records() == []
    assert audit_log.next_record_id() == "audit-000001"


def test_agent_audit_imports_without_runtime_dependencies() -> None:
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


def _valid_mock_output(**overrides: object) -> dict:
    output = {
        "schema_id": "mission_snapshot_v1",
        "result_summary": {"mission_id": "mission-1", "status": "ready"},
        "evidence_refs": ["mission-1"],
        "metadata": {"fixture": "mock"},
    }
    output.update(overrides)
    return output


def _invoke_query(
    handler,
    *,
    request: ToolCallRequest | None = None,
    identity: AgentIdentity | None = None,
    manifest: AgentPermissionManifest | None = None,
    audit_log: AgentAuditLog | None = None,
    elapsed: float = 0.5,
):
    log = audit_log or AgentAuditLog()
    result = _registry_with_query().invoke_mock(
        request or _request(metadata={"workflow_id": "workflow-1"}),
        identity or _identity(),
        manifest or _manifest(),
        mock_handler=handler,
        audit_log=log,
        started_at=10.0,
        mock_elapsed_sec=elapsed,
    )
    return result, log


def test_mock_tool_call_validates_input() -> None:
    called = False

    def handler(parameters: dict) -> dict:
        nonlocal called
        called = True
        return _valid_mock_output()

    result, audit_log = _invoke_query(
        handler,
        request=_request(
            metadata={"workflow_id": "workflow-1", "input_schema_id": "wrong_v1"}
        ),
    )

    assert result.status.value == "failed"
    assert result.errors[0]["code"] == "input_schema_mismatch"
    assert called is False
    assert audit_log.list_records()[0].event_type == "tool_call_failed"


def test_mock_tool_call_calls_explicit_handler_once() -> None:
    calls = []

    def handler(parameters: dict) -> dict:
        calls.append(parameters)
        return _valid_mock_output()

    result, _ = _invoke_query(handler)

    assert result.status.value == "passed"
    assert len(calls) == 1


def test_mock_tool_call_passes_defensive_parameter_copy() -> None:
    request = _request(
        parameters={"mission_id": "mission-1", "nested": {"items": [1]}},
        metadata={"workflow_id": "workflow-1"},
    )

    def handler(parameters: dict) -> dict:
        parameters["nested"]["items"].append(2)
        return _valid_mock_output()

    result, _ = _invoke_query(handler, request=request)

    assert result.status.value == "passed"
    assert request.parameters["nested"]["items"] == [1]


def test_mock_tool_call_denies_unlisted_agent() -> None:
    called = False

    def handler(parameters: dict) -> dict:
        nonlocal called
        called = True
        return _valid_mock_output()

    result, audit_log = _invoke_query(
        handler,
        manifest=_manifest(allowed_tool_ids=[]),
    )

    assert result.status.value == "denied"
    assert result.errors[0]["code"] == "permission_denied"
    assert called is False
    assert [record.event_type for record in audit_log.list_records()] == [
        "tool_call_denied"
    ]


def test_mock_tool_call_denies_side_effect_escalation() -> None:
    result, _ = _invoke_query(
        lambda parameters: _valid_mock_output(),
        manifest=_manifest(allowed_side_effect_levels=["STATE_PROPOSAL"]),
    )

    assert result.status.value == "denied"
    assert result.errors[0]["code"] == "side_effect_not_allowed"


def test_mock_tool_call_denial_never_calls_handler() -> None:
    calls = []

    result, _ = _invoke_query(
        lambda parameters: calls.append(parameters),
        manifest=_manifest(allowed_tool_ids=[]),
    )

    assert result.status.value == "denied"
    assert calls == []


def _invoke_gated(
    handler,
    *,
    approval: bool,
    gates: dict[str, bool] | None,
):
    registry, request, identity, manifest = _gated_authorization_values()
    approval_request = approval_decision = None
    if approval:
        approval_request, approval_decision = _approval_pair()
    return registry.invoke_mock(
        request,
        identity,
        manifest,
        mock_handler=handler,
        audit_log=AgentAuditLog(),
        approval_request=approval_request,
        approval_decision=approval_decision,
        gates=gates,
        started_at=1.0,
        mock_elapsed_sec=0.1,
    )


def test_mock_tool_call_requires_level_three_approval() -> None:
    calls = []
    result = _invoke_gated(
        lambda parameters: calls.append(parameters),
        approval=False,
        gates={"GWM_ALLOW_OPTIONAL_RUNTIME": True},
    )

    assert result.status.value == "denied"
    assert result.errors[0]["code"] == "approval_required"
    assert calls == []


def test_mock_tool_call_requires_runtime_gate_metadata() -> None:
    result = _invoke_gated(
        lambda parameters: {
            "schema_id": "benchmark_readiness_result_v1",
            "result_summary": {"ready": True},
        },
        approval=True,
        gates=None,
    )

    assert result.status.value == "denied"
    assert result.errors[0]["code"] == "runtime_gate_required"


def test_mock_tool_call_never_reads_environment_gates(monkeypatch) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")

    result = _invoke_gated(
        lambda parameters: {},
        approval=True,
        gates=None,
    )

    assert result.status.value == "denied"
    assert result.errors[0]["code"] == "runtime_gate_required"


def test_mock_tool_call_never_invokes_real_runtime(monkeypatch) -> None:
    real_import = builtins.__import__
    forbidden = {"airsim", "cosysairsim", "isaacsim", "mavsdk", "rclpy"}

    def guarded_import(name, *args, **kwargs):
        if name.split(".", 1)[0] in forbidden:
            raise AssertionError("real runtime must not be imported")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    result, _ = _invoke_query(
        lambda parameters: _valid_mock_output(
            result_summary={"runtime_available": False}
        )
    )

    assert result.status.value == "passed"


def test_mock_tool_call_validates_output_schema() -> None:
    result, audit_log = _invoke_query(
        lambda parameters: _valid_mock_output(schema_id="wrong_v1")
    )

    assert result.status.value == "failed"
    assert result.errors[0]["code"] == "output_schema_mismatch"
    assert audit_log.list_records()[-1].event_type == "tool_result_invalid"


def test_mock_tool_call_rejects_unknown_output_field() -> None:
    result, _ = _invoke_query(
        lambda parameters: _valid_mock_output(unexpected=True)
    )

    assert result.status.value == "failed"
    assert result.errors[0]["code"] == "output_schema_mismatch"


def test_mock_tool_call_rejects_non_dict_output() -> None:
    result, _ = _invoke_query(lambda parameters: None)

    assert result.status.value == "failed"
    assert result.errors[0]["code"] == "output_schema_mismatch"


def test_mock_tool_call_writes_start_and_result_audit_records() -> None:
    result, audit_log = _invoke_query(lambda parameters: _valid_mock_output())

    records = audit_log.find_by_request(result.request_id)
    assert [record.event_type for record in records] == [
        "tool_call_started",
        "tool_call_completed",
    ]
    assert records[0].validated_parameters_summary == {"mission_id": "mission-1"}
    assert records[1].result_summary == result.result_summary


def test_mock_tool_call_writes_redacted_audit_record() -> None:
    audit_log = AgentAuditLog()
    record = audit_log.build_record(
        workflow_id="wf-1",
        event_type="tool_call_failed",
        actor_id="agent-1",
        timestamp=1.0,
        metadata={"safe": True, "Provider_Response": "private"},
    )

    stored = audit_log.append(record)

    assert stored.metadata == {
        "safe": True,
        "redacted_fields": ["provider_response"],
    }


def test_mock_tool_call_failure_is_audited() -> None:
    def handler(parameters: dict) -> dict:
        raise RuntimeError("private failure")

    result, audit_log = _invoke_query(handler)

    assert result.status.value == "failed"
    assert [record.event_type for record in audit_log.list_records()] == [
        "tool_call_started",
        "tool_call_failed",
    ]


def test_mock_tool_call_failure_hides_raw_exception() -> None:
    secret = "internal-provider-trace"

    def handler(parameters: dict) -> dict:
        raise RuntimeError(secret)

    result, audit_log = _invoke_query(handler)
    encoded = json.dumps(result.to_dict()) + json.dumps(audit_log.snapshot())

    assert result.errors == [
        {
            "code": "tool_call_failed",
            "message": "explicitly injected mock handler failed",
        }
    ]
    assert secret not in encoded


def test_mock_tool_timeout_is_deterministic() -> None:
    result, _ = _invoke_query(lambda parameters: _valid_mock_output(), elapsed=5.1)

    assert result.status.value == "timed_out"
    assert result.started_at == 10.0
    assert result.completed_at == 15.1
    assert result.errors[0]["code"] == "tool_call_timed_out"


def test_mock_tool_timeout_does_not_call_handler() -> None:
    calls = []

    result, audit_log = _invoke_query(
        lambda parameters: calls.append(parameters),
        elapsed=5.1,
    )

    assert result.status.value == "timed_out"
    assert calls == []
    assert [record.event_type for record in audit_log.list_records()] == [
        "tool_call_timed_out"
    ]


def test_mock_tool_result_is_json_safe() -> None:
    result, audit_log = _invoke_query(lambda parameters: _valid_mock_output())

    assert json.loads(json.dumps(result.to_dict(), sort_keys=True)) == result.to_dict()
    assert json.loads(json.dumps(audit_log.snapshot(), sort_keys=True)) == audit_log.snapshot()


def test_mock_tool_result_is_defensive_copy() -> None:
    output = _valid_mock_output()
    result, audit_log = _invoke_query(lambda parameters: output)
    output["result_summary"]["status"] = "mutated"
    result.result_summary["status"] = "caller-mutated"

    assert audit_log.list_records()[-1].result_summary["status"] == "ready"
