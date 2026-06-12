"""Mock replay and metrics aggregation for GWM-UAV-C2 mission events."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from src.c2.mission_types import (
    ALLOWED_DEFENSIVE_RISK_CATEGORIES,
    MetricSummary,
    MissionEvent,
    ReplayFrame,
    ensure_json_safe_dict,
)
from src.c2.state_store import MissionStateStore


@dataclass
class MissionReplayResult:
    """JSON-safe result returned by the mock replay engine."""

    frames: List[ReplayFrame] = field(default_factory=list)
    metrics: MetricSummary = field(default_factory=lambda: MetricSummary(mission_id="mock-mission"))
    events: List[MissionEvent] = field(default_factory=list)
    final_snapshot: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        for index, frame in enumerate(self.frames):
            if not isinstance(frame, ReplayFrame):
                raise ValueError(f"frames[{index}] must be a ReplayFrame")
            frame.validate()
        if not isinstance(self.metrics, MetricSummary):
            raise ValueError("metrics must be a MetricSummary")
        self.metrics.validate()
        for index, event in enumerate(self.events):
            if not isinstance(event, MissionEvent):
                raise ValueError(f"events[{index}] must be a MissionEvent")
            event.validate()
        ensure_json_safe_dict(self.final_snapshot, "final_snapshot")

    def to_dict(self) -> Dict[str, object]:
        self.validate()
        return {
            "frames": [frame.to_dict() for frame in self.frames],
            "metrics": self.metrics.to_dict(),
            "events": [event.to_dict() for event in self.events],
            "final_snapshot": copy.deepcopy(self.final_snapshot),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "MissionReplayResult":
        ensure_json_safe_dict(data, "MissionReplayResult")
        frames = [ReplayFrame.from_dict(frame) for frame in data.get("frames", [])]
        metrics_data = data.get("metrics", {"mission_id": "mock-mission"})
        if not isinstance(metrics_data, dict):
            raise ValueError("metrics must be a dictionary")
        events = [MissionEvent.from_dict(event) for event in data.get("events", [])]
        final_snapshot = data.get("final_snapshot", {})
        if not isinstance(final_snapshot, dict):
            raise ValueError("final_snapshot must be a dictionary")
        return cls(
            frames=frames,
            metrics=MetricSummary.from_dict(metrics_data),
            events=events,
            final_snapshot=copy.deepcopy(final_snapshot),
        )


class MissionReplayEngine:
    """Deterministic event replay over an in-memory mission state store."""

    def __init__(self, state_store: Optional[MissionStateStore] = None) -> None:
        self.state_store = state_store or MissionStateStore()

    def replay(self, events: Sequence[MissionEvent]) -> MissionReplayResult:
        if isinstance(events, (str, bytes)) or not isinstance(events, Sequence):
            raise ValueError("events must be a sequence of MissionEvent objects")
        self.state_store.clear()
        ordered_events: List[MissionEvent] = []
        frames: List[ReplayFrame] = []
        for index, event in enumerate(events, start=1):
            if not isinstance(event, MissionEvent):
                raise ValueError(f"events[{index - 1}] must be a MissionEvent")
            event.validate()
            self.state_store.apply_event(event)
            ordered_events.append(copy.deepcopy(event))
            frames.append(self.build_frame(f"frame-{index:06d}", event, self.state_store))
        metrics = self.summarize(frames)
        final_snapshot = self.state_store.snapshot()
        return MissionReplayResult(
            frames=frames,
            metrics=metrics,
            events=ordered_events,
            final_snapshot=final_snapshot,
        )

    def build_frame(self, frame_id: str, event: MissionEvent, store: MissionStateStore) -> ReplayFrame:
        snapshot = store.snapshot()
        return ReplayFrame(
            frame_id=frame_id,
            timestamp=event.timestamp,
            mission_snapshot=snapshot,
            events=[event.to_dict()],
            risk_summary=self._risk_summary(snapshot),
            route_summary=self._route_summary(snapshot),
            safety_summary=self._safety_summary(snapshot),
        )

    def summarize(self, frames: Sequence[ReplayFrame]) -> MetricSummary:
        if not frames:
            return MetricSummary(mission_id="mock-mission")
        for index, frame in enumerate(frames):
            if not isinstance(frame, ReplayFrame):
                raise ValueError(f"frames[{index}] must be a ReplayFrame")
            frame.validate()
        snapshot = copy.deepcopy(frames[-1].mission_snapshot)
        ensure_json_safe_dict(snapshot, "mission_snapshot")
        mission_id = self._infer_mission_id(snapshot)
        risk_counts = self._risk_summary(snapshot)["risk_counts"]
        threat_assessments = snapshot.get("threat_assessments", {})
        safety_decisions = snapshot.get("safety_decisions", {})
        human_approvals = snapshot.get("human_approvals", {})
        tasks = snapshot.get("tasks", {})
        planned_routes = snapshot.get("planned_routes", {})
        return MetricSummary(
            mission_id=mission_id,
            event_count=len(frames),
            risk_counts=risk_counts,
            replan_count=self._count_by_field(threat_assessments, "recommendation", "replan"),
            hold_count=(
                self._count_by_field(threat_assessments, "recommendation", "hold")
                + self._count_by_field(safety_decisions, "status", "hold")
            ),
            approval_count=(
                self._count_by_field(human_approvals, "decision", "approved")
                + self._count_by_field(safety_decisions, "status", "approved")
            ),
            blocked_count=(
                self._count_by_field(safety_decisions, "status", "blocked")
                + self._count_by_field(tasks, "status", "blocked")
                + self._count_by_field(planned_routes, "constraint_verdict", "blocked")
            ),
        )

    def _risk_summary(self, snapshot: Dict[str, object]) -> Dict[str, object]:
        risk_counts: Dict[str, int] = {}
        latest_category: Optional[str] = None
        risk_signals = snapshot.get("risk_signals", {})
        if isinstance(risk_signals, dict):
            for signal in risk_signals.values():
                if not isinstance(signal, dict):
                    continue
                category = signal.get("category")
                if category in ALLOWED_DEFENSIVE_RISK_CATEGORIES:
                    latest_category = str(category)
                    risk_counts[latest_category] = risk_counts.get(latest_category, 0) + 1
        return {
            "risk_signal_count": sum(risk_counts.values()),
            "risk_counts": risk_counts,
            "latest_category": latest_category,
        }

    def _route_summary(self, snapshot: Dict[str, object]) -> Dict[str, object]:
        planned_routes = snapshot.get("planned_routes", {})
        if not isinstance(planned_routes, dict) or not planned_routes:
            return {"route_count": 0, "latest_route_id": None, "latest_constraint_verdict": None}
        latest_route_id = list(planned_routes.keys())[-1]
        latest_route = planned_routes[latest_route_id]
        latest_verdict = latest_route.get("constraint_verdict") if isinstance(latest_route, dict) else None
        return {
            "route_count": len(planned_routes),
            "latest_route_id": latest_route_id,
            "latest_constraint_verdict": latest_verdict,
        }

    def _safety_summary(self, snapshot: Dict[str, object]) -> Dict[str, object]:
        threat_assessments = snapshot.get("threat_assessments", {})
        safety_decisions = snapshot.get("safety_decisions", {})
        human_approvals = snapshot.get("human_approvals", {})
        tasks = snapshot.get("tasks", {})
        planned_routes = snapshot.get("planned_routes", {})
        return {
            "replan_count": self._count_by_field(threat_assessments, "recommendation", "replan"),
            "hold_count": (
                self._count_by_field(threat_assessments, "recommendation", "hold")
                + self._count_by_field(safety_decisions, "status", "hold")
            ),
            "approval_count": (
                self._count_by_field(human_approvals, "decision", "approved")
                + self._count_by_field(safety_decisions, "status", "approved")
            ),
            "blocked_count": (
                self._count_by_field(safety_decisions, "status", "blocked")
                + self._count_by_field(tasks, "status", "blocked")
                + self._count_by_field(planned_routes, "constraint_verdict", "blocked")
            ),
        }

    @staticmethod
    def _count_by_field(collection: object, field_name: str, expected_value: str) -> int:
        if not isinstance(collection, dict):
            return 0
        count = 0
        for item in collection.values():
            if isinstance(item, dict) and item.get(field_name) == expected_value:
                count += 1
        return count

    @staticmethod
    def _infer_mission_id(snapshot: Dict[str, object]) -> str:
        requests = snapshot.get("requests", {})
        if isinstance(requests, dict) and requests:
            return str(next(iter(requests)))
        tasks = snapshot.get("tasks", {})
        if isinstance(tasks, dict):
            for task in tasks.values():
                if isinstance(task, dict) and task.get("request_id"):
                    return str(task["request_id"])
        assessments = snapshot.get("threat_assessments", {})
        if isinstance(assessments, dict):
            for assessment in assessments.values():
                if isinstance(assessment, dict) and assessment.get("mission_id"):
                    return str(assessment["mission_id"])
        return "mock-mission"


def replay_events(events: Sequence[MissionEvent]) -> MissionReplayResult:
    return MissionReplayEngine().replay(events)


def build_replay_frames(events: Sequence[MissionEvent]) -> List[ReplayFrame]:
    return replay_events(events).frames


def summarize_metrics(frames: Sequence[ReplayFrame]) -> MetricSummary:
    return MissionReplayEngine().summarize(frames)
