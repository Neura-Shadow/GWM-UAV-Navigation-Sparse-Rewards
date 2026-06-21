"""Tests for the v2-2A mock-first mission dispatcher."""

from __future__ import annotations

import sys

import pytest

from src.c2 import MissionDispatcher, MissionEventBus, MissionRequest, MissionStateStore


def _request(request_id: str = "req-001", created_at: float = 10.0) -> MissionRequest:
    return MissionRequest(
        request_id=request_id,
        operator_id="operator-a",
        objective="Inspect zone alpha",
        priority=2,
        area={"zone": "alpha"},
        constraints={"required_capability": "survey"},
        created_at=created_at,
    )


def test_dispatcher_submit_request_creates_task() -> None:
    dispatcher = MissionDispatcher()

    task = dispatcher.submit_request(_request())

    assert task.task_id == "task-req-001"
    assert task.request_id == "req-001"
    assert task.objective == "Inspect zone alpha"
    assert task.status == "pending"


def test_dispatcher_invalid_request_rejected() -> None:
    dispatcher = MissionDispatcher()
    request = _request()
    request.objective = ""

    with pytest.raises(ValueError, match="missing_objective"):
        dispatcher.submit_request(request)


def test_dispatcher_emits_mission_requested_event() -> None:
    event_bus = MissionEventBus()
    dispatcher = MissionDispatcher(event_bus=event_bus)

    dispatcher.submit_request(_request())

    first_event = event_bus.history()[0]
    assert first_event.event_type == "mission.requested"
    assert first_event.payload["request_id"] == "req-001"


def test_dispatcher_emits_task_created_event() -> None:
    event_bus = MissionEventBus()
    dispatcher = MissionDispatcher(event_bus=event_bus)

    dispatcher.submit_request(_request())

    second_event = event_bus.history()[1]
    assert second_event.event_type == "mission.task.created"
    assert second_event.payload["task_id"] == "task-req-001"


def test_dispatcher_task_status_transition() -> None:
    dispatcher = MissionDispatcher()
    task = dispatcher.submit_request(_request())

    updated = dispatcher.update_task_status(task.task_id, "assigned", reason="mock assignment")

    assert updated.status == "assigned"
    assert updated.metadata["previous_status"] == "pending"
    assert updated.metadata["reason"] == "mock assignment"


def test_dispatcher_invalid_status_transition_rejected() -> None:
    dispatcher = MissionDispatcher()
    task = dispatcher.submit_request(_request())

    with pytest.raises(ValueError, match="invalid_task_status_transition"):
        dispatcher.update_task_status(task.task_id, "completed")


def test_dispatcher_block_task_records_reason() -> None:
    event_bus = MissionEventBus()
    dispatcher = MissionDispatcher(event_bus=event_bus)
    task = dispatcher.submit_request(_request())

    blocked = dispatcher.block_task(task.task_id, "manual safety review")

    assert blocked.status == "blocked"
    assert blocked.metadata["reason"] == "manual safety review"
    assert event_bus.history()[-1].metadata["reason"] == "manual safety review"


def test_dispatcher_cancel_task_records_reason() -> None:
    dispatcher = MissionDispatcher()
    task = dispatcher.submit_request(_request())

    cancelled = dispatcher.cancel_task(task.task_id, "operator cancelled")

    assert cancelled.status == "cancelled"
    assert cancelled.metadata["reason"] == "operator cancelled"


def test_dispatcher_get_task() -> None:
    dispatcher = MissionDispatcher()
    task = dispatcher.submit_request(_request())

    returned = dispatcher.get_task(task.task_id)

    assert returned == task
    assert returned is not task


def test_dispatcher_list_tasks_deterministic() -> None:
    dispatcher = MissionDispatcher()
    dispatcher.submit_request(_request("req-b"))
    dispatcher.submit_request(_request("req-a"))

    assert [task.task_id for task in dispatcher.list_tasks()] == ["task-req-a", "task-req-b"]


def test_dispatcher_state_store_integration() -> None:
    state_store = MissionStateStore()
    dispatcher = MissionDispatcher(state_store=state_store)

    task = dispatcher.submit_request(_request())

    snapshot = state_store.snapshot()
    assert snapshot["requests"]["req-001"]["objective"] == "Inspect zone alpha"
    assert snapshot["tasks"][task.task_id]["status"] == "pending"
    assert [event["event_type"] for event in snapshot["events"]] == [
        "mission.requested",
        "mission.task.created",
    ]


def test_dispatcher_event_ids_are_deterministic() -> None:
    event_bus = MissionEventBus()
    dispatcher = MissionDispatcher(event_bus=event_bus)
    task = dispatcher.submit_request(_request())
    dispatcher.update_task_status(task.task_id, "assigned")

    assert [event.event_id for event in event_bus.history()] == [
        "event-000001",
        "event-000002",
        "event-000003",
    ]


def test_dispatcher_rejects_duplicate_task_id() -> None:
    dispatcher = MissionDispatcher()
    dispatcher.submit_request(_request())

    with pytest.raises(ValueError, match="invalid_request: duplicate task id"):
        dispatcher.submit_request(_request())


def test_dispatcher_imports_without_runtime_dependencies() -> None:
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
