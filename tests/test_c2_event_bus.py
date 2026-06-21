"""Tests for the v2-1B in-memory mission event bus."""

from __future__ import annotations

import sys

import pytest

from src.c2 import MissionEvent, MissionEventBus


def _event(index: int, event_type: str = "mission.test") -> MissionEvent:
    return MissionEvent(
        event_id=f"evt-{index:03d}",
        event_type=event_type,
        timestamp=float(index),
        source="unit_test",
        payload={"index": index},
    )


def test_event_bus_publish_order() -> None:
    bus = MissionEventBus()

    bus.publish(_event(1))
    bus.publish(_event(2))

    assert [event.event_id for event in bus.history()] == ["evt-001", "evt-002"]


def test_event_bus_subscribe_handler() -> None:
    bus = MissionEventBus()
    received: list[str] = []

    bus.subscribe("mission.test", lambda event: received.append(event.event_id))
    bus.publish(_event(1))

    assert received == ["evt-001"]


def test_event_bus_multiple_handlers_preserve_order() -> None:
    bus = MissionEventBus()
    calls: list[str] = []

    bus.subscribe("mission.test", lambda event: calls.append(f"first:{event.event_id}"))
    bus.subscribe("mission.test", lambda event: calls.append(f"second:{event.event_id}"))
    bus.publish(_event(1))

    assert calls == ["first:evt-001", "second:evt-001"]


def test_event_bus_replay_preserves_order() -> None:
    bus = MissionEventBus()

    published = bus.replay([_event(1), _event(2), _event(3)])

    assert [event.event_id for event in published] == ["evt-001", "evt-002", "evt-003"]
    assert [event.event_id for event in bus.history()] == ["evt-001", "evt-002", "evt-003"]


def test_event_bus_drain_does_not_clear_history() -> None:
    bus = MissionEventBus()
    bus.publish(_event(1))

    drained = bus.drain()

    assert [event.event_id for event in drained] == ["evt-001"]
    assert [event.event_id for event in bus.history()] == ["evt-001"]


def test_event_bus_clear_behavior() -> None:
    bus = MissionEventBus()
    received: list[str] = []
    bus.subscribe("mission.test", lambda event: received.append(event.event_id))
    bus.publish(_event(1))

    bus.clear()
    bus.publish(_event(2))

    assert received == ["evt-001"]
    assert [event.event_id for event in bus.history()] == ["evt-002"]


def test_event_bus_handler_exception_is_explicit() -> None:
    bus = MissionEventBus()

    def failing_handler(event: MissionEvent) -> None:
        raise RuntimeError(f"boom:{event.event_id}")

    bus.subscribe("mission.test", failing_handler)
    with pytest.raises(RuntimeError, match="boom:evt-001"):
        bus.publish(_event(1))


def test_event_bus_rejects_invalid_subscription() -> None:
    bus = MissionEventBus()

    with pytest.raises(ValueError, match="event_type"):
        bus.subscribe("", lambda event: None)
    with pytest.raises(ValueError, match="handler"):
        bus.subscribe("mission.test", object())  # type: ignore[arg-type]


def test_event_bus_imports_without_runtime_dependencies() -> None:
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
