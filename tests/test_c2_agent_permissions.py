"""Tests for deny-by-default AgentOps permission manifests."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    AgentPermissionManifest,
    AgentRole,
    ApprovalLevel,
    AgentContractError,
    SideEffectLevel,
    build_default_agent_permission_manifests,
)


def test_permission_manifest_json_roundtrip() -> None:
    manifest = AgentPermissionManifest(
        manifest_id="manifest-mission-1",
        agent_role=AgentRole.MISSION,
        manifest_version="1",
        allowed_capability_ids=["mission.query", "mission.propose"],
        allowed_tool_ids=["mission.query@1", "mission.propose@1"],
        read_scopes=["mission.read"],
        proposal_scopes=["mission.proposal"],
        validated_write_scopes=[],
        allowed_side_effect_levels=[
            SideEffectLevel.READ_ONLY,
            SideEffectLevel.STATE_PROPOSAL,
        ],
        max_requestable_approval_level=ApprovalLevel.PROPOSAL,
        explicit_denials=[],
        metadata={"source": "frozen_v3_0"},
    )

    encoded = manifest.to_dict()
    restored = AgentPermissionManifest.from_dict(encoded)

    assert restored == manifest
    assert encoded["agent_role"] == "mission"
    assert encoded["allowed_side_effect_levels"] == ["READ_ONLY", "STATE_PROPOSAL"]
    assert json.loads(json.dumps(encoded, sort_keys=True)) == encoded


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


def test_permission_manifest_denies_by_default() -> None:
    manifest = _manifest()

    assert manifest.allows_tool("route.score@1") is False
    assert manifest.allows_capability("route.score") is False
    assert manifest.allows_scope("route.proposal", "STATE_PROPOSAL") is False


def test_permission_manifest_rejects_unknown_role() -> None:
    with pytest.raises(AgentContractError) as error:
        _manifest(agent_role="Mission")

    assert error.value.code == "unknown_agent_role"


def test_permission_manifest_rejects_wildcard_tool() -> None:
    with pytest.raises(AgentContractError, match="wildcards"):
        _manifest(allowed_tool_ids=["mission.*@1"])


def test_permission_manifest_rejects_wildcard_scope() -> None:
    with pytest.raises(AgentContractError, match="wildcards"):
        _manifest(read_scopes=["mission.*"])


def test_permission_manifest_rejects_duplicate_values() -> None:
    with pytest.raises(AgentContractError, match="unique"):
        _manifest(allowed_capability_ids=["mission.query", "mission.query"])


def test_permission_manifest_rejects_level_five() -> None:
    with pytest.raises(AgentContractError) as error:
        _manifest(max_requestable_approval_level=5)

    assert error.value.code == "real_hardware_prohibited"


def test_permission_manifest_rejects_real_hardware_side_effect() -> None:
    with pytest.raises(AgentContractError) as error:
        _manifest(allowed_side_effect_levels=["REAL_HARDWARE_PROHIBITED"])

    assert error.value.code == "real_hardware_prohibited"


def test_permission_manifest_exact_role_matching() -> None:
    manifest = _manifest()

    assert manifest.agent_role is AgentRole.MISSION
    with pytest.raises(AgentContractError):
        AgentPermissionManifest.from_dict({**manifest.to_dict(), "agent_role": "MISSION"})


def test_permission_manifest_exact_tool_matching() -> None:
    manifest = _manifest()

    assert manifest.allows_tool("mission.query@1") is True
    assert manifest.allows_tool("Mission.query@1") is False
    assert manifest.allows_tool("mission.query@2") is False


def test_permission_manifest_exact_scope_matching() -> None:
    manifest = _manifest()

    assert manifest.allows_scope("mission.read", SideEffectLevel.READ_ONLY) is True
    assert manifest.allows_scope("Mission.read", SideEffectLevel.READ_ONLY) is False


def test_permission_manifest_explicit_denial_overrides_allow() -> None:
    manifest = _manifest(explicit_denials=["tool:mission.query@1"])

    assert manifest.allows_tool("mission.query@1") is False
    assert manifest.is_explicitly_denied(tool_id="mission.query@1") is True


def test_permission_manifest_to_dict_returns_deep_copy() -> None:
    manifest = _manifest(metadata={"labels": ["frozen"]})

    encoded = manifest.to_dict()
    encoded["metadata"]["labels"].append("changed")
    encoded["allowed_tool_ids"].append("route.score@1")

    assert manifest.to_dict()["metadata"] == {"labels": ["frozen"]}
    assert manifest.allowed_tool_ids == ("mission.query@1",)


def test_permission_manifest_from_dict_copies_input() -> None:
    payload = _manifest(metadata={"labels": ["frozen"]}).to_dict()

    restored = AgentPermissionManifest.from_dict(payload)
    payload["metadata"]["labels"].append("changed")

    assert restored.to_dict()["metadata"] == {"labels": ["frozen"]}


def test_permission_manifest_rejects_callable_metadata() -> None:
    with pytest.raises(AgentContractError) as error:
        _manifest(metadata={"callback": lambda: None})

    assert error.value.code == "invalid_json_payload"


def test_default_permission_manifests_cover_all_nine_roles() -> None:
    manifests = build_default_agent_permission_manifests()

    assert set(manifests) == set(AgentRole)
    assert len(manifests) == 9
    assert all(manifest.agent_role is role for role, manifest in manifests.items())


def test_default_permission_manifests_have_no_validated_write_scope() -> None:
    manifests = build_default_agent_permission_manifests()

    assert all(manifest.validated_write_scopes == () for manifest in manifests.values())
    assert all(
        SideEffectLevel.REAL_HARDWARE_PROHIBITED not in manifest.allowed_side_effect_levels
        for manifest in manifests.values()
    )


def test_agent_permissions_import_without_runtime_dependencies() -> None:
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
