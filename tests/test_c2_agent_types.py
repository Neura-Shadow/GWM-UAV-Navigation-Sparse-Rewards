"""Tests for provider-independent AgentOps message contracts."""

from __future__ import annotations

import json
from enum import Enum
from math import inf
import sys

import pytest

from src.c2 import (
    AgentCapability,
    AgentContractError,
    AgentIdentity,
    AgentContext,
    AgentConflict,
    AgentDecision,
    AgentDecisionType,
    AgentObservation,
    AgentProposal,
    AgentAuditRecord,
    AgentRole,
    AgentTask,
    AgentTaskStatus,
    AgentWorkflowState,
    AgentWorkflowStatus,
    ApprovalLevel,
    ApprovalDecision,
    ApprovalOutcome,
    ApprovalRequest,
    SideEffectLevel,
    ToolCallRequest,
    ToolCallResult,
    ToolCallStatus,
)


def test_agent_identity_json_roundtrip() -> None:
    identity = AgentIdentity(
        agent_id="agent-supervisor-01",
        role=AgentRole.SUPERVISOR,
        version="1",
        capability_ids=["workflow.read", "proposal.submit"],
        enabled=True,
        metadata={"fixture": {"kind": "unit_test"}},
    )

    encoded = identity.to_dict()
    restored = AgentIdentity.from_dict(encoded)

    assert restored == identity
    assert encoded["role"] == "supervisor"
    assert json.loads(json.dumps(encoded, sort_keys=True)) == encoded


def test_agent_identity_rejects_unknown_role() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentIdentity("agent-1", "Supervisor", "1", [], True)

    assert error.value.code == "unknown_agent_role"


def test_agent_identity_rejects_duplicate_capabilities() -> None:
    with pytest.raises(AgentContractError, match="unique"):
        AgentIdentity("agent-1", "mission", "1", ["mission.read", "mission.read"], True)


def test_agent_capability_validates_side_effect_level() -> None:
    capability = AgentCapability(
        capability_id="mission.query",
        action="mission.query",
        resource_scope="mission",
        side_effect_level=SideEffectLevel.READ_ONLY,
        max_approval_level=ApprovalLevel.READ_ONLY,
    )

    assert capability.to_dict()["side_effect_level"] == "READ_ONLY"
    assert capability.to_dict()["max_approval_level"] == 0


def test_agent_capability_rejects_wildcard() -> None:
    with pytest.raises(AgentContractError, match="wildcard"):
        AgentCapability("cap-1", "mission.*", "mission", "READ_ONLY", 0)


def test_agent_capability_rejects_level_five() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentCapability("cap-1", "mission.query", "mission", "READ_ONLY", 5)

    assert error.value.code == "real_hardware_prohibited"


def test_agent_capability_rejects_boolean_approval_level() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentCapability("cap-1", "mission.query", "mission", "READ_ONLY", True)

    assert error.value.code == "approval_invalid"


def test_agent_capability_rejects_real_hardware_executable_level() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentCapability(
            "cap-1",
            "mission.query",
            "mission",
            "REAL_HARDWARE_PROHIBITED",
            0,
        )

    assert error.value.code == "real_hardware_prohibited"


def test_agent_task_status_validation() -> None:
    task = AgentTask(
        task_id="task-1",
        workflow_id="wf-1",
        assignee_agent_id="agent-1",
        objective="Prepare a bounded proposal",
        input_refs=["event-1"],
        deadline=20.0,
        status=AgentTaskStatus.PENDING,
    )

    assert task.to_dict()["status"] == "pending"
    with pytest.raises(AgentContractError):
        AgentTask("task-2", "wf-1", "agent-1", "x", [], None, "unknown")


def test_agent_task_rejects_non_finite_deadline() -> None:
    with pytest.raises(AgentContractError, match="finite"):
        AgentTask("task-1", "wf-1", "agent-1", "x", [], inf, "pending")


def test_agent_observation_rejects_runtime_object() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentObservation(
            "obs-1",
            "mission_state",
            1.0,
            "1",
            {"runtime": object()},
            "fresh",
        )

    assert error.value.code == "invalid_json_payload"


def test_agent_context_copies_reference_lists() -> None:
    state_refs = ["state-1"]
    context = AgentContext(
        "ctx-1",
        "wf-1",
        state_refs,
        ["obs-1"],
        "v3-0",
        "fresh",
    )

    state_refs.append("state-2")
    encoded = context.to_dict()
    encoded["state_refs"].append("state-3")

    assert context.state_refs == ["state-1"]


def test_agent_proposal_is_json_safe() -> None:
    proposal = AgentProposal(
        "proposal-1",
        "agent-mission-1",
        "task-1",
        "mission_task",
        {"objective": "Inspect corridor alpha", "priority": 2},
        ["event-1"],
        2.0,
    )

    assert json.dumps(proposal.to_dict(), sort_keys=True)


def test_agent_proposal_rejects_sensitive_nested_key() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentProposal(
            "proposal-1",
            "agent-1",
            "task-1",
            "mission_task",
            {"nested": {"Api-Key": "no"}},
            [],
            2.0,
        )

    assert error.value.code == "sensitive_field_rejected"


def test_tool_call_request_json_roundtrip() -> None:
    request = ToolCallRequest(
        "request-1",
        "mission.query",
        "1",
        "agent-mission-1",
        {"mission_id": "mission-1"},
        None,
        10.0,
        [],
    )

    assert ToolCallRequest.from_dict(request.to_dict()) == request


def test_tool_call_request_rejects_sensitive_fields() -> None:
    with pytest.raises(AgentContractError) as error:
        ToolCallRequest(
            "request-1",
            "mission.query",
            "1",
            "agent-1",
            {"session": "not-allowed"},
            None,
            10.0,
            [],
        )

    assert error.value.code == "sensitive_field_rejected"


@pytest.mark.parametrize("timeout", [0.0, -1.0, 300.1, inf])
def test_tool_call_request_rejects_invalid_timeout(timeout: float) -> None:
    with pytest.raises(AgentContractError) as error:
        ToolCallRequest(
            "request-1", "mission.query", "1", "agent-1", {}, None, timeout, []
        )

    assert error.value.code == "timeout_invalid"


def test_tool_call_result_status_validation() -> None:
    result = ToolCallResult(
        "request-1",
        "mission.query",
        "1",
        ToolCallStatus.PASSED,
        {"found": True},
        ["mission-1"],
        [],
        1.0,
        2.0,
    )

    assert result.to_dict()["status"] == "passed"
    with pytest.raises(AgentContractError):
        ToolCallResult("request-1", "tool", "1", "ok", {}, [], [], 1.0, 2.0)


def test_tool_call_result_rejects_reversed_timestamps() -> None:
    with pytest.raises(AgentContractError, match="completed_at"):
        ToolCallResult("request-1", "tool", "1", "failed", {}, [], [], 2.0, 1.0)


def test_tool_call_result_rejects_raw_exception_object() -> None:
    with pytest.raises(AgentContractError) as error:
        ToolCallResult(
            "request-1",
            "tool",
            "1",
            "failed",
            {},
            [],
            [ValueError("internal detail")],
            1.0,
            2.0,
        )

    assert error.value.code == "invalid_json_payload"


def test_agent_decision_requires_evidence_for_block() -> None:
    with pytest.raises(AgentContractError, match="evidence"):
        AgentDecision("decision-1", "agent-safety-1", [], "block", "Unsafe proposal", [])


def test_agent_decision_approve_recommendation_is_not_human_approval() -> None:
    decision = AgentDecision(
        "decision-1",
        "agent-safety-1",
        ["proposal-1"],
        AgentDecisionType.APPROVE_RECOMMENDATION,
        "Deterministic checks passed; human review remains authoritative.",
        ["safety-1"],
    )

    assert decision.to_dict()["decision"] == "approve_recommendation"
    assert "operator_id" not in decision.to_dict()


def test_agent_conflict_requires_two_unique_proposals() -> None:
    with pytest.raises(AgentContractError, match="at least two"):
        AgentConflict(
            "conflict-1",
            "wf-1",
            ["proposal-1"],
            "route_verdict",
            "Candidate routes disagree.",
            "open",
        )


def test_approval_request_rejects_level_five() -> None:
    with pytest.raises(AgentContractError) as error:
        ApprovalRequest(
            "approval-request-1",
            5,
            ["validation-1"],
            "Request simulator validation",
            [],
            "agent-supervisor-1",
            100.0,
        )

    assert error.value.code == "real_hardware_prohibited"


def test_approval_request_requires_target() -> None:
    with pytest.raises(AgentContractError, match="target_refs"):
        ApprovalRequest(
            "approval-request-1",
            ApprovalLevel.PROPOSAL,
            [],
            "Review proposal",
            [],
            "agent-supervisor-1",
            100.0,
        )


def test_approval_decision_requires_operator_id() -> None:
    with pytest.raises(AgentContractError, match="operator_id"):
        ApprovalDecision(
            "decision-1", "approval-request-1", "", "approved", 10.0, ""
        )


def test_approval_decision_rejects_agent_as_authoritative_producer() -> None:
    with pytest.raises(AgentContractError, match="human"):
        ApprovalDecision(
            "decision-1",
            "approval-request-1",
            "operator-1",
            ApprovalOutcome.APPROVED,
            10.0,
            "",
            {"authoritative_producer": "safety_review"},
        )


def test_agent_audit_record_excludes_private_reasoning() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentAuditRecord(
            record_id="audit-1",
            workflow_id="wf-1",
            event_type="proposal.validated",
            actor_id="agent-1",
            timestamp=10.0,
            result_summary={"chain_of_thought": "must not be stored"},
        )

    assert error.value.code == "sensitive_field_rejected"


def test_agent_audit_record_requires_tool_name_version_pair() -> None:
    with pytest.raises(AgentContractError, match="both present"):
        AgentAuditRecord(
            record_id="audit-1",
            workflow_id="wf-1",
            event_type="tool.requested",
            actor_id="agent-1",
            timestamp=10.0,
            tool_name="mission.query",
        )


def test_agent_workflow_state_status_validation() -> None:
    workflow = AgentWorkflowState(
        "wf-1", "planning", ["task-1"], [], [], [], 1.0, 2.0
    )

    assert workflow.status is AgentWorkflowStatus.PLANNING
    with pytest.raises(AgentContractError):
        AgentWorkflowState("wf-2", "active", [], [], [], [], 1.0, 2.0)


def test_agent_workflow_state_timestamp_order() -> None:
    with pytest.raises(AgentContractError, match="updated_at"):
        AgentWorkflowState("wf-1", "created", [], [], [], [], 2.0, 1.0)


def _all_agent_contracts() -> list[object]:
    return [
        AgentIdentity("agent-1", "mission", "1", ["mission.read"], True),
        AgentCapability("cap-1", "mission.query", "mission", "READ_ONLY", 0),
        AgentTask("task-1", "wf-1", "agent-1", "Review", [], None, "pending"),
        AgentObservation("obs-1", "state", 1.0, "1", {"count": 1}, "fresh"),
        AgentContext("ctx-1", "wf-1", ["state-1"], ["obs-1"], "v3-0", "fresh"),
        AgentProposal("proposal-1", "agent-1", "task-1", "mission", {}, [], 1.0),
        ToolCallRequest("request-1", "mission.query", "1", "agent-1", {}, None, 5.0, []),
        ToolCallResult("request-1", "mission.query", "1", "passed", {}, [], [], 1.0, 2.0),
        AgentDecision("decision-1", "agent-1", [], "continue", "Continue analysis", []),
        AgentConflict(
            "conflict-1",
            "wf-1",
            ["proposal-1", "proposal-2"],
            "route",
            "Routes disagree",
            "open",
        ),
        ApprovalRequest(
            "approval-request-1", 1, ["proposal-1"], "Review", [], "agent-1", 5.0
        ),
        ApprovalDecision(
            "approval-decision-1",
            "approval-request-1",
            "operator-1",
            "deferred",
            2.0,
            "More evidence required",
        ),
        AgentAuditRecord(
            "audit-1",
            "wf-1",
            "proposal.created",
            "agent-1",
            2.0,
            input_refs=["proposal-1"],
        ),
        AgentWorkflowState("wf-1", "created", [], [], [], [], 1.0, 1.0),
    ]


def test_all_agent_contracts_json_roundtrip() -> None:
    for contract in _all_agent_contracts():
        encoded = contract.to_dict()
        restored = type(contract).from_dict(encoded)

        assert restored == contract
        assert json.loads(json.dumps(encoded, sort_keys=True)) == encoded


def test_agent_contract_to_dict_returns_deep_copy() -> None:
    proposal = AgentProposal(
        "proposal-1", "agent-1", "task-1", "mission", {"nested": [1]}, [], 1.0
    )

    encoded = proposal.to_dict()
    encoded["payload"]["nested"].append(2)

    assert proposal.payload == {"nested": [1]}


def test_agent_contract_from_dict_copies_input() -> None:
    payload = {
        "proposal_id": "proposal-1",
        "agent_id": "agent-1",
        "task_id": "task-1",
        "proposal_type": "mission",
        "payload": {"nested": [1]},
        "evidence_refs": [],
        "created_at": 1.0,
        "metadata": {},
    }

    proposal = AgentProposal.from_dict(payload)
    payload["payload"]["nested"].append(2)

    assert proposal.payload == {"nested": [1]}


def test_agent_contracts_reject_callable_values() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentProposal("proposal-1", "agent-1", "task-1", "mission", {"fn": lambda: 1}, [], 1.0)

    assert error.value.code == "invalid_json_payload"


def test_agent_contracts_reject_arbitrary_enum_values() -> None:
    class ForeignEnum(str, Enum):
        VALUE = "value"

    with pytest.raises(AgentContractError) as error:
        AgentProposal(
            "proposal-1",
            "agent-1",
            "task-1",
            "mission",
            {"foreign": ForeignEnum.VALUE},
            [],
            1.0,
        )

    assert error.value.code == "invalid_json_payload"


def test_agent_contracts_reject_unknown_top_level_fields() -> None:
    payload = AgentIdentity("agent-1", "mission", "1", [], True).to_dict()
    payload["unexpected"] = True

    with pytest.raises(AgentContractError) as error:
        AgentIdentity.from_dict(payload)

    assert error.value.code == "invalid_contract"


def test_agent_contracts_require_metadata_dictionary() -> None:
    with pytest.raises(AgentContractError) as error:
        AgentIdentity("agent-1", "mission", "1", [], True, metadata=[])

    assert error.value.code == "invalid_json_payload"


def test_agent_contracts_import_without_runtime_dependencies() -> None:
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
