"""Integration tests for the v2-2 dispatcher/fleet mock workflow."""

from __future__ import annotations

import json
import sys

import pytest

from src.c2 import (
    FleetAsset,
    FleetManager,
    MissionDispatcher,
    MissionEvent,
    MissionEventBus,
    MissionReplayEngine,
    MissionRequest,
    MissionStateStore,
)


def _request(request_id: str = "req-001") -> MissionRequest:
    return MissionRequest(
        request_id=request_id,
        operator_id="operator-a",
        objective="Inspect zone alpha",
        priority=2,
        area={"zone": "alpha"},
        constraints={"required_capability": "survey"},
        created_at=10.0,
    )


def _asset(asset_id: str, capabilities: list[str] | None = None) -> FleetAsset:
    return FleetAsset(
        asset_id=asset_id,
        backend="mock",
        capabilities=capabilities or ["survey"],
        available=True,
        health={"battery": 0.9},
    )


def _shared_stack() -> tuple[MissionEventBus, MissionStateStore, MissionDispatcher, FleetManager]:
    event_bus = MissionEventBus()
    state_store = MissionStateStore()
    dispatcher = MissionDispatcher(event_bus=event_bus, state_store=state_store)
    fleet = FleetManager(event_bus=event_bus, state_store=state_store)
    return event_bus, state_store, dispatcher, fleet


def _assigned_flow() -> tuple[MissionEventBus, MissionStateStore, object, object]:
    event_bus, state_store, dispatcher, fleet = _shared_stack()
    task = dispatcher.submit_request(_request())
    fleet.register_asset(_asset("uav-1", ["survey", "relay"]))
    assigned_task = fleet.assign_task(task, required_capability="survey")
    return event_bus, state_store, assigned_task, fleet.get_asset("uav-1")


def _event_types(events: list[MissionEvent]) -> list[str]:
    return [event.event_type for event in events]


def test_dispatcher_fleet_assignment_flow() -> None:
    event_bus, state_store, assigned_task, assigned_asset = _assigned_flow()

    assert assigned_task.status == "assigned"
    assert assigned_task.assigned_asset_id == "uav-1"
    assert assigned_asset is not None
    assert assigned_asset.current_task_id == assigned_task.task_id
    assert assigned_asset.available is False
    assert state_store.get_task(assigned_task.task_id) == assigned_task
    assert state_store.get_asset("uav-1") == assigned_asset
    assert len(event_bus.history()) == 5


def test_dispatcher_fleet_event_order_is_deterministic() -> None:
    event_bus, _, _, _ = _assigned_flow()
    event_types = _event_types(event_bus.history())

    assert event_types == [
        "mission.requested",
        "mission.task.created",
        "fleet.asset.registered",
        "fleet.asset.updated",
        "mission.task.updated",
    ]
    assert event_types.index("mission.requested") < event_types.index("mission.task.created")
    assert event_types.index("mission.task.created") < event_types.index("fleet.asset.updated")
    assert event_types.index("fleet.asset.updated") < event_types.index("mission.task.updated")


def test_dispatcher_fleet_state_store_snapshot_restore() -> None:
    _, state_store, assigned_task, assigned_asset = _assigned_flow()
    unknown_event = MissionEvent(
        event_id="event-unknown",
        event_type="mission.note.created",
        timestamp=50.0,
        source="integration_test",
        payload={"note": "preserve unknown event"},
    )
    state_store.apply_event(unknown_event)

    restored = MissionStateStore()
    restored.restore(state_store.snapshot())

    assert restored.get_task(assigned_task.task_id) == assigned_task
    assert restored.get_asset(assigned_asset.asset_id) == assigned_asset
    assert restored.list_events()[-1].event_type == "mission.note.created"


def test_dispatcher_fleet_replay_generates_frames() -> None:
    event_bus, _, _, _ = _assigned_flow()

    result = MissionReplayEngine().replay(event_bus.history())

    assert len(result.frames) == len(event_bus.history())
    assert [frame.frame_id for frame in result.frames] == [
        "frame-000001",
        "frame-000002",
        "frame-000003",
        "frame-000004",
        "frame-000005",
    ]
    assert result.final_snapshot["tasks"]["task-req-001"]["status"] == "assigned"


def test_dispatcher_fleet_replay_metrics_are_deterministic() -> None:
    event_bus, _, _, _ = _assigned_flow()
    events = event_bus.history()

    first = MissionReplayEngine().replay(events)
    second = MissionReplayEngine().replay(events)

    assert first.metrics.event_count == len(events)
    assert first.metrics.to_dict() == second.metrics.to_dict()
    assert first.to_dict() == second.to_dict()


def test_dispatcher_fleet_no_available_asset_refusal() -> None:
    event_bus, state_store, dispatcher, fleet = _shared_stack()
    task = dispatcher.submit_request(_request())

    with pytest.raises(ValueError, match="no_available_asset"):
        fleet.assign_task(task)

    assert state_store.get_task(task.task_id).status == "pending"
    assert state_store.get_task(task.task_id).assigned_asset_id is None
    assert _event_types(event_bus.history()) == ["mission.requested", "mission.task.created"]


def test_dispatcher_fleet_required_capability_assignment() -> None:
    _, _, dispatcher, fleet = _shared_stack()
    task = dispatcher.submit_request(_request())
    fleet.register_asset(_asset("uav-a", ["survey"]))
    fleet.register_asset(_asset("uav-b", ["thermal"]))

    assigned = fleet.assign_task(task, required_capability="thermal")

    assert assigned.assigned_asset_id == "uav-b"


def test_dispatcher_fleet_lexicographic_tie_breaking() -> None:
    _, _, dispatcher, fleet = _shared_stack()
    task = dispatcher.submit_request(_request())
    fleet.register_asset(_asset("uav-c", ["survey"]))
    fleet.register_asset(_asset("uav-a", ["survey"]))
    fleet.register_asset(_asset("uav-b", ["survey"]))

    assigned = fleet.assign_task(task, required_capability="survey")

    assert assigned.assigned_asset_id == "uav-a"


def test_dispatcher_fleet_json_safe_outputs() -> None:
    event_bus, state_store, _, _ = _assigned_flow()
    result = MissionReplayEngine().replay(event_bus.history())

    json.dumps([event.to_dict() for event in event_bus.history()], allow_nan=False, sort_keys=True)
    json.dumps(state_store.snapshot(), allow_nan=False, sort_keys=True)
    json.dumps(result.to_dict(), allow_nan=False, sort_keys=True)


def test_dispatcher_fleet_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "cosysairsim",
        "isaacsim",
        "mavsdk",
        "message_filters",
        "omni",
        "pxr",
        "rclpy",
    }

    assert runtime_modules.isdisjoint(sys.modules)
