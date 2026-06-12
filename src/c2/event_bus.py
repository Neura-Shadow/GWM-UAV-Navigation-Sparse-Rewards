"""Deterministic in-memory event bus for the GWM-UAV-C2 extension."""

from __future__ import annotations

import copy
from typing import Callable, Dict, List

from src.c2.mission_types import MissionEvent, ensure_non_empty_string


class MissionEventBus:
    """Small synchronous event bus with append-only ordered history.

    The bus is intentionally process-local and runtime-free. `clear()` removes
    both event history and subscriptions so tests and later mock workflows can
    reset the bus deterministically.
    """

    def __init__(self) -> None:
        self._history: List[MissionEvent] = []
        self._handlers: Dict[str, List[Callable[[MissionEvent], None]]] = {}

    def publish(self, event: MissionEvent) -> MissionEvent:
        if not isinstance(event, MissionEvent):
            raise ValueError("event must be a MissionEvent")
        event.validate()
        self._history.append(copy.deepcopy(event))
        for handler in self._handlers.get(event.event_type, []):
            handler(copy.deepcopy(event))
        return event

    def subscribe(self, event_type: str, handler: Callable[[MissionEvent], None]) -> None:
        ensure_non_empty_string(event_type, "event_type")
        if not callable(handler):
            raise ValueError("handler must be callable")
        self._handlers.setdefault(event_type, []).append(handler)

    def drain(self) -> List[MissionEvent]:
        return self.history()

    def replay(self, events: List[MissionEvent]) -> List[MissionEvent]:
        if not isinstance(events, list):
            raise ValueError("events must be a list")
        published: List[MissionEvent] = []
        for event in events:
            published.append(self.publish(event))
        return published

    def clear(self) -> None:
        self._history.clear()
        self._handlers.clear()

    def history(self) -> List[MissionEvent]:
        return copy.deepcopy(self._history)
