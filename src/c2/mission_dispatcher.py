"""Mock-first mission dispatcher for GWM-UAV-C2."""

from __future__ import annotations

import copy
from typing import Dict, List, Optional

from src.c2.event_bus import MissionEventBus
from src.c2.mission_types import MissionEvent, MissionRequest, MissionTask
from src.c2.state_store import MissionStateStore


class MissionDispatcher:
    """Create and manage mission tasks without runtime side effects."""

    _VALID_STATUSES = {"pending", "assigned", "blocked", "completed", "cancelled"}
    _ALLOWED_TRANSITIONS = {
        ("pending", "assigned"),
        ("pending", "blocked"),
        ("pending", "cancelled"),
        ("assigned", "completed"),
        ("assigned", "blocked"),
        ("assigned", "cancelled"),
    }

    def __init__(
        self,
        event_bus: Optional[MissionEventBus] = None,
        state_store: Optional[MissionStateStore] = None,
    ) -> None:
        self.event_bus = event_bus or MissionEventBus()
        self.state_store = state_store
        self._tasks: Dict[str, MissionTask] = {}
        self._event_counter = 0

    def submit_request(self, request: MissionRequest) -> MissionTask:
        self.validate_request(request)
        task_id = self.make_task_id(request)
        if task_id in self._tasks:
            raise ValueError("invalid_request: duplicate task id")
        self.publish_event(self.make_event("mission.requested", request.to_dict(), correlation_id=request.request_id))
        task = self.create_task(request)
        self.publish_event(self.make_event("mission.task.created", task.to_dict(), correlation_id=request.request_id))
        return task

    def create_task(self, request: MissionRequest) -> MissionTask:
        self.validate_request(request)
        task_id = self.make_task_id(request)
        if task_id in self._tasks:
            raise ValueError("invalid_request: duplicate task id")
        task = MissionTask(
            task_id=task_id,
            request_id=request.request_id,
            objective=request.objective,
            status="pending",
            priority=request.priority,
            constraints=copy.deepcopy(request.constraints),
            assigned_asset_id=None,
            created_at=request.created_at,
        )
        self._tasks[task.task_id] = copy.deepcopy(task)
        return copy.deepcopy(task)

    def update_task_status(self, task_id: str, status: str, reason: str = "") -> MissionTask:
        if task_id not in self._tasks:
            raise ValueError("task_not_found: task id not found")
        task = self._tasks[task_id]
        if not self.allowed_transition(task.status, status):
            raise ValueError(f"invalid_task_status_transition: {task.status} -> {status} is not allowed")
        metadata = copy.deepcopy(task.metadata)
        metadata["previous_status"] = task.status
        metadata["new_status"] = status
        if reason:
            metadata["reason"] = reason
        updated_task = MissionTask(
            task_id=task.task_id,
            request_id=task.request_id,
            objective=task.objective,
            status=status,
            priority=task.priority,
            constraints=copy.deepcopy(task.constraints),
            assigned_asset_id=task.assigned_asset_id,
            created_at=task.created_at,
            metadata=metadata,
        )
        self._tasks[task_id] = copy.deepcopy(updated_task)
        event_metadata = {"previous_status": task.status, "new_status": status}
        if reason:
            event_metadata["reason"] = reason
        self.publish_event(
            self.make_event(
                "mission.task.updated",
                updated_task.to_dict(),
                correlation_id=updated_task.request_id,
                metadata=event_metadata,
            )
        )
        return copy.deepcopy(updated_task)

    def block_task(self, task_id: str, reason: str) -> MissionTask:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("safety_blocked: reason is required")
        return self.update_task_status(task_id, "blocked", reason=reason)

    def cancel_task(self, task_id: str, reason: str) -> MissionTask:
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("invalid_request: cancellation reason is required")
        return self.update_task_status(task_id, "cancelled", reason=reason)

    def get_task(self, task_id: str) -> Optional[MissionTask]:
        task = self._tasks.get(task_id)
        return copy.deepcopy(task) if task is not None else None

    def list_tasks(self) -> List[MissionTask]:
        return [copy.deepcopy(self._tasks[task_id]) for task_id in sorted(self._tasks)]

    def validate_request(self, request: MissionRequest) -> None:
        if not isinstance(request, MissionRequest):
            raise ValueError("invalid_request: request must be a MissionRequest")
        try:
            request.validate()
        except ValueError as exc:
            message = str(exc)
            if "objective" in message:
                raise ValueError("missing_objective: MissionRequest objective is required") from exc
            if "priority" in message:
                raise ValueError("invalid_priority: priority must be in allowed range") from exc
            raise ValueError(f"invalid_request: {message}") from exc
        if not request.objective.strip():
            raise ValueError("missing_objective: MissionRequest objective is required")
        if request.priority < 0 or request.priority > 5:
            raise ValueError("invalid_priority: priority must be in allowed range")

    def allowed_transition(self, old_status: str, new_status: str) -> bool:
        if old_status not in self._VALID_STATUSES or new_status not in self._VALID_STATUSES:
            return False
        return (old_status, new_status) in self._ALLOWED_TRANSITIONS

    def make_task_id(self, request: MissionRequest) -> str:
        self.validate_request(request)
        return f"task-{request.request_id}"

    def make_event(
        self,
        event_type: str,
        payload: Dict[str, object],
        source: str = "mission_dispatcher",
        correlation_id: Optional[str] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> MissionEvent:
        self._event_counter += 1
        timestamp = float(payload.get("created_at", self._event_counter))
        return MissionEvent(
            event_id=f"event-{self._event_counter:06d}",
            event_type=event_type,
            timestamp=timestamp,
            source=source,
            payload=copy.deepcopy(payload),
            correlation_id=correlation_id,
            metadata=copy.deepcopy(metadata or {}),
        )

    def publish_event(self, event: MissionEvent) -> MissionEvent:
        published = self.event_bus.publish(event)
        if self.state_store is not None:
            self.state_store.apply_event(event)
        return published
