"""Tests for the v2-2B mock-first fleet manager."""

from __future__ import annotations

import sys

import pytest

from src.c2 import FleetAsset, FleetManager, MissionEventBus, MissionStateStore, MissionTask, UAVState


def _asset(
    asset_id: str = "uav-1",
    capabilities: list[str] | None = None,
    available: bool = True,
    current_task_id: str | None = None,
) -> FleetAsset:
    return FleetAsset(
        asset_id=asset_id,
        backend="mock",
        capabilities=capabilities or ["survey"],
        available=available,
        health={"battery": 0.9},
        current_task_id=current_task_id,
    )


def _task(task_id: str = "task-001", status: str = "pending") -> MissionTask:
    return MissionTask(
        task_id=task_id,
        request_id="req-001",
        objective="Inspect zone alpha",
        status=status,
        priority=2,
        constraints={"required_capability": "survey"},
        created_at=10.0,
    )


def _state(asset_id: str = "uav-1") -> UAVState:
    return UAVState(
        asset_id=asset_id,
        timestamp=20.0,
        position={"x": 1.0, "y": 2.0, "z": -3.0},
        velocity={"vx": 0.0, "vy": 0.0, "vz": 0.0},
    )


def test_fleet_manager_register_asset() -> None:
    event_bus = MissionEventBus()
    manager = FleetManager(event_bus=event_bus)

    asset = manager.register_asset(_asset())

    assert asset.asset_id == "uav-1"
    assert manager.get_asset("uav-1") == asset
    assert event_bus.history()[0].event_type == "fleet.asset.registered"


def test_fleet_manager_rejects_duplicate_asset() -> None:
    manager = FleetManager()
    manager.register_asset(_asset())

    with pytest.raises(ValueError, match="asset_already_assigned"):
        manager.register_asset(_asset())


def test_fleet_manager_update_asset() -> None:
    event_bus = MissionEventBus()
    manager = FleetManager(event_bus=event_bus)
    manager.register_asset(_asset())
    updated = _asset(available=False, current_task_id="task-001")

    returned = manager.update_asset(updated)

    assert returned.available is False
    assert returned.current_task_id == "task-001"
    assert event_bus.history()[-1].event_type == "fleet.asset.updated"
    assert event_bus.history()[-1].metadata["new_available"] is False


def test_fleet_manager_update_missing_asset_rejected() -> None:
    manager = FleetManager()

    with pytest.raises(ValueError, match="asset_not_found"):
        manager.update_asset(_asset())


def test_fleet_manager_update_uav_state() -> None:
    event_bus = MissionEventBus()
    manager = FleetManager(event_bus=event_bus)
    manager.register_asset(_asset())

    state = manager.update_uav_state(_state())

    assert state.asset_id == "uav-1"
    assert event_bus.history()[-1].event_type == "uav.state.updated"


def test_fleet_manager_available_assets() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-b"))
    manager.register_asset(_asset("uav-a"))
    manager.register_asset(_asset("uav-c", available=False))
    manager.register_asset(_asset("uav-d", current_task_id="task-existing"))

    assert [asset.asset_id for asset in manager.available_assets()] == ["uav-a", "uav-b"]


def test_fleet_manager_capability_filter() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-a", capabilities=["survey"]))
    manager.register_asset(_asset("uav-b", capabilities=["thermal", "survey"]))

    assert [asset.asset_id for asset in manager.available_assets("thermal")] == ["uav-b"]


def test_fleet_manager_assigns_available_asset() -> None:
    event_bus = MissionEventBus()
    manager = FleetManager(event_bus=event_bus)
    manager.register_asset(_asset("uav-a"))

    assigned = manager.assign_task(_task(), required_capability="survey")

    assert assigned.status == "assigned"
    assert assigned.assigned_asset_id == "uav-a"
    asset = manager.get_asset("uav-a")
    assert asset is not None
    assert asset.available is False
    assert asset.current_task_id == "task-001"
    assert [event.event_type for event in event_bus.history()[-2:]] == [
        "fleet.asset.updated",
        "mission.task.updated",
    ]


def test_fleet_manager_assignment_is_deterministic() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-z"))
    manager.register_asset(_asset("uav-a"))

    assigned = manager.assign_task(_task())

    assert assigned.assigned_asset_id == "uav-a"


def test_fleet_manager_no_available_asset_refusal() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-a", available=False))

    with pytest.raises(ValueError, match="no_available_asset"):
        manager.assign_task(_task())


def test_fleet_manager_missing_required_capability_refusal() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-a", capabilities=["survey"]))

    with pytest.raises(ValueError, match="missing_required_capability"):
        manager.assign_task(_task(), required_capability="thermal")


def test_fleet_manager_assign_non_pending_task_rejected() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-a"))

    with pytest.raises(ValueError, match="invalid_task_status_transition"):
        manager.assign_task(_task(status="assigned"))


def test_fleet_manager_release_asset() -> None:
    manager = FleetManager()
    manager.register_asset(_asset("uav-a"))
    manager.assign_task(_task(), required_capability="survey")

    released = manager.release_asset("uav-a")

    assert released.available is True
    assert released.current_task_id is None


def test_fleet_manager_state_store_integration() -> None:
    state_store = MissionStateStore()
    manager = FleetManager(state_store=state_store)

    manager.register_asset(_asset("uav-a"))
    manager.update_uav_state(_state("uav-a"))
    manager.assign_task(_task(), required_capability="survey")

    snapshot = state_store.snapshot()
    assert snapshot["assets"]["uav-a"]["current_task_id"] == "task-001"
    assert snapshot["uav_states"]["uav-a"]["timestamp"] == 20.0
    assert snapshot["tasks"]["task-001"]["assigned_asset_id"] == "uav-a"


def test_fleet_manager_event_ids_are_deterministic() -> None:
    event_bus = MissionEventBus()
    manager = FleetManager(event_bus=event_bus)

    manager.register_asset(_asset("uav-a"))
    manager.update_uav_state(_state("uav-a"))
    manager.assign_task(_task(), required_capability="survey")

    assert [event.event_id for event in event_bus.history()] == [
        "fleet-event-000001",
        "fleet-event-000002",
        "fleet-event-000003",
        "fleet-event-000004",
    ]


def test_fleet_manager_imports_without_runtime_dependencies() -> None:
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
