"""Read-only dashboard replay payload assembly for GWM-UAV-C2."""

from __future__ import annotations

import copy
import json
from typing import Dict, List, Optional, Sequence

from src.c2.mission_types import MetricSummary, MissionEvent, ReplayFrame
from src.c2.replay import MissionReplayEngine
from src.c2.state_store import MissionStateStore


SCHEMA_VERSION = "v2-5A-dashboard-replay-payload"
REDACTED_VALUE = "<redacted>"
SUMMARY_KEYS = (
    "mission_id",
    "request_id",
    "task_id",
    "asset_id",
    "state_id",
    "signal_id",
    "assessment_id",
    "route_id",
    "decision_id",
    "approval_id",
    "status",
    "category",
    "recommendation",
    "constraint_verdict",
    "risk_score",
    "total_risk",
    "event_type",
)
SENSITIVE_KEYS = {
    "password",
    "passwd",
    "secret",
    "token",
    "api_key",
    "apikey",
    "credential",
    "credentials",
    "private_key",
    "hostname",
    "host",
    "runtime_log",
    "hardware_log",
    "flight_log",
    "px4_log",
    "rosbag",
    "screenshot",
    "file_path",
    "local_path",
    "absolute_path",
    "private_hostname",
}
SNAPSHOT_COLLECTIONS = (
    ("mission_requests", "requests"),
    ("mission_tasks", "tasks"),
    ("fleet_assets", "assets"),
    ("uav_states", "uav_states"),
    ("risk_signals", "risk_signals"),
    ("threat_assessments", "threat_assessments"),
    ("airspace_constraints", "airspace_constraints"),
    ("planned_routes", "planned_routes"),
    ("safety_decisions", "safety_decisions"),
    ("human_approvals", "human_approvals"),
    ("unknown_events", "unknown_events"),
)


class DashboardReplayBuilder:
    """Build deterministic, JSON-safe replay payloads for audit dashboards.

    This builder is observational only. It formats in-memory mission events,
    replay frames, and snapshots without writing files, starting services,
    connecting to runtimes, or approving command execution.
    """

    def __init__(
        self,
        state_store: Optional[MissionStateStore] = None,
        replay_engine: Optional[MissionReplayEngine] = None,
    ) -> None:
        if state_store is not None and not isinstance(state_store, MissionStateStore):
            raise ValueError("state_store must be a MissionStateStore")
        if replay_engine is not None and not isinstance(replay_engine, MissionReplayEngine):
            raise ValueError("replay_engine must be a MissionReplayEngine")
        self.state_store = state_store
        self.replay_engine = replay_engine or MissionReplayEngine(state_store=state_store)

    def build_timeline(self, events: Sequence[MissionEvent]) -> List[Dict[str, object]]:
        checked_events = self._validate_events(events)
        return [self.format_event(event, index=index) for index, event in enumerate(checked_events)]

    def format_event(self, event: MissionEvent, index: int = 0) -> Dict[str, object]:
        if not isinstance(event, MissionEvent):
            raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
        event.validate()
        entry = {
            "index": self._non_negative_index(index),
            "event_id": event.event_id,
            "event_type": event.event_type,
            "family": self.event_family(event.event_type),
            "timestamp": float(event.timestamp),
            "payload_summary": self.payload_summary(event.payload),
            "metadata": self._redact_sensitive(copy.deepcopy(event.metadata)),
        }
        return self._json_safe_dict(entry)

    def format_replay_frame(self, frame: ReplayFrame, index: int = 0) -> Dict[str, object]:
        if not isinstance(frame, ReplayFrame):
            raise ValueError("invalid_frame: replay frame is required")
        frame.validate()
        event_summary = self._frame_event_summary(frame)
        formatted = {
            "index": self._non_negative_index(index),
            "frame_id": frame.frame_id,
            "event_id": event_summary.get("event_id"),
            "event_type": event_summary.get("event_type"),
            "timestamp": float(frame.timestamp),
            "snapshot_summary": self.build_dashboard_snapshot(frame.mission_snapshot),
            "risk_summary": self._redact_sensitive(copy.deepcopy(frame.risk_summary)),
            "route_summary": self._redact_sensitive(copy.deepcopy(frame.route_summary)),
            "safety_summary": self._redact_sensitive(copy.deepcopy(frame.safety_summary)),
        }
        return self._json_safe_dict(formatted)

    def build_dashboard_snapshot(self, snapshot: Dict[str, object]) -> Dict[str, object]:
        if not isinstance(snapshot, dict):
            raise ValueError("invalid_snapshot: snapshot must be a dict")
        snapshot_copy = copy.deepcopy(snapshot)
        result: Dict[str, object] = {"raw_keys": sorted(str(key) for key in snapshot_copy)}
        for output_key, source_key in SNAPSHOT_COLLECTIONS:
            result[output_key] = self._collection_summary(snapshot_copy.get(source_key, {}))
        return self._json_safe_dict(result)

    def build_replay_payload(self, events: Sequence[MissionEvent]) -> Dict[str, object]:
        checked_events = self._validate_events(events)
        timeline = self.build_timeline(checked_events)
        replay_result = self.replay_engine.replay(checked_events)
        frames = [
            self.format_replay_frame(frame, index=index)
            for index, frame in enumerate(replay_result.frames)
        ]
        payload = {
            "schema_version": SCHEMA_VERSION,
            "event_count": len(checked_events),
            "timeline": timeline,
            "frame_count": len(frames),
            "frames": frames,
            "final_snapshot": self.build_dashboard_snapshot(replay_result.final_snapshot),
            "summary": {
                "families": self._family_counts(timeline),
                "unknown_event_count": sum(1 for item in timeline if item.get("family") == "unknown"),
            },
            "audit_boundary": {
                "read_only": True,
                "command_free": True,
                "runtime_free": True,
            },
        }
        return self._json_safe_dict(payload)

    def filter_timeline(
        self,
        timeline: Sequence[Dict[str, object]],
        event_types: Optional[Sequence[str]] = None,
    ) -> List[Dict[str, object]]:
        if isinstance(timeline, (str, bytes)) or not isinstance(timeline, Sequence):
            raise ValueError("invalid_timeline: timeline must be a sequence of dict entries")
        checked_timeline: List[Dict[str, object]] = []
        for index, entry in enumerate(timeline):
            if not isinstance(entry, dict):
                raise ValueError("invalid_timeline: timeline must be a sequence of dict entries")
            checked_timeline.append(self._json_safe_dict(copy.deepcopy(entry)))
        if not event_types:
            return checked_timeline
        if isinstance(event_types, (str, bytes)) or not isinstance(event_types, Sequence):
            raise ValueError("event_types must be a sequence of strings when provided")
        requested = set()
        for event_type in event_types:
            if not isinstance(event_type, str) or not event_type:
                raise ValueError("event_types must be a sequence of strings when provided")
            requested.add(event_type)
        return [entry for entry in checked_timeline if entry.get("event_type") in requested]

    def event_family(self, event_type: str) -> str:
        if not isinstance(event_type, str) or not event_type:
            raise ValueError("event_type must be a non-empty string")
        if event_type.startswith("mission."):
            return "mission"
        if event_type.startswith("fleet."):
            return "fleet"
        if event_type.startswith("uav."):
            return "uav"
        if event_type == "risk.signal.created":
            return "risk"
        if event_type == "threat.assessment.created":
            return "threat"
        if event_type == "route.planned":
            return "route"
        if event_type.startswith("safety."):
            return "safety"
        if event_type.startswith("human_approval."):
            return "human_approval"
        return "unknown"

    def payload_summary(self, payload: Dict[str, object]) -> Dict[str, object]:
        if not isinstance(payload, dict):
            raise ValueError("payload must be a dict")
        payload_copy = self._redact_sensitive(copy.deepcopy(payload))
        summary: Dict[str, object] = {}
        for key in SUMMARY_KEYS:
            value = self._find_key(payload_copy, key)
            if value is not None:
                summary[key] = value
        summary["payload_keys"] = sorted(str(key) for key in payload_copy)
        for key, value in payload_copy.items():
            if self._is_sensitive_key(str(key)):
                summary[str(key)] = value
        return self._json_safe_dict(summary)

    def _validate_events(self, events: Sequence[MissionEvent]) -> List[MissionEvent]:
        if isinstance(events, (str, bytes)) or not isinstance(events, Sequence):
            raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
        checked_events: List[MissionEvent] = []
        for event in events:
            if not isinstance(event, MissionEvent):
                raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
            event.validate()
            checked_events.append(copy.deepcopy(event))
        return checked_events

    @staticmethod
    def _non_negative_index(index: int) -> int:
        if isinstance(index, bool) or not isinstance(index, int) or index < 0:
            raise ValueError("index must be a non-negative integer")
        return index

    def _frame_event_summary(self, frame: ReplayFrame) -> Dict[str, object]:
        if frame.events and isinstance(frame.events[0], dict):
            event = frame.events[0]
            return {
                "event_id": event.get("event_id"),
                "event_type": event.get("event_type"),
            }
        return {"event_id": None, "event_type": None}

    def _collection_summary(self, collection: object) -> Dict[str, object]:
        if isinstance(collection, dict):
            ids = sorted(str(key) for key in collection)
            return {"count": len(collection), "ids": ids}
        if isinstance(collection, list):
            ids = []
            for index, item in enumerate(collection):
                if isinstance(item, dict):
                    item_id = self._first_identifier(item)
                    ids.append(str(item_id) if item_id is not None else str(index))
                else:
                    ids.append(str(index))
            return {"count": len(collection), "ids": sorted(ids)}
        return {"count": 0, "ids": []}

    def _first_identifier(self, item: Dict[str, object]) -> Optional[object]:
        for key in SUMMARY_KEYS:
            if key in item:
                return item[key]
        return None

    def _family_counts(self, timeline: Sequence[Dict[str, object]]) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for entry in timeline:
            family = entry.get("family")
            if isinstance(family, str):
                counts[family] = counts.get(family, 0) + 1
        return {key: counts[key] for key in sorted(counts)}

    def _find_key(self, value: object, wanted_key: str) -> Optional[object]:
        if isinstance(value, dict):
            if wanted_key in value:
                return copy.deepcopy(value[wanted_key])
            for key in sorted(value):
                found = self._find_key(value[key], wanted_key)
                if found is not None:
                    return found
        if isinstance(value, list):
            for item in value:
                found = self._find_key(item, wanted_key)
                if found is not None:
                    return found
        return None

    def _redact_sensitive(self, value: object) -> object:
        if isinstance(value, dict):
            redacted: Dict[str, object] = {}
            for key in sorted(value):
                key_text = str(key)
                if self._is_sensitive_key(key_text):
                    redacted[key_text] = REDACTED_VALUE
                else:
                    redacted[key_text] = self._redact_sensitive(value[key])
            return redacted
        if isinstance(value, list):
            return [self._redact_sensitive(item) for item in value]
        return value

    @staticmethod
    def _is_sensitive_key(key: str) -> bool:
        return key.lower() in SENSITIVE_KEYS

    @staticmethod
    def _json_safe_dict(value: Dict[str, object]) -> Dict[str, object]:
        try:
            json.dumps(value, allow_nan=False, sort_keys=True)
        except (TypeError, ValueError) as exc:
            raise ValueError("dashboard payload must be JSON-safe") from exc
        return value


class C2MetricsExporter:
    """Build deterministic metrics payloads from in-memory mission events."""

    def __init__(self, replay_engine: Optional[MissionReplayEngine] = None) -> None:
        if replay_engine is not None and not isinstance(replay_engine, MissionReplayEngine):
            raise ValueError("replay_engine must be a MissionReplayEngine")
        self.replay_engine = replay_engine or MissionReplayEngine()
        self.dashboard_builder = DashboardReplayBuilder(replay_engine=self.replay_engine)

    def summarize_events(self, events: Sequence[MissionEvent]) -> MetricSummary:
        checked_events = self._validate_events(events)
        event_type_counts = self.build_event_type_counts(checked_events)
        risk_metrics = self.build_risk_metrics(checked_events)
        route_metrics = self.build_route_metrics(checked_events)
        task_fleet_metrics = self.build_task_fleet_metrics(checked_events)
        safety_approval_metrics = self.build_safety_approval_metrics(checked_events)
        metadata = {
            "event_type_counts": event_type_counts,
            "task_status_counts": task_fleet_metrics["task_status_counts"],
            "fleet_assignment_counts": task_fleet_metrics["fleet_assignment_counts"],
            "risk_counts": risk_metrics["risk_counts"],
            "route_count": route_metrics["route_count"],
            "route_verdict_counts": route_metrics["route_verdict_counts"],
            "safety_decision_counts": safety_approval_metrics["safety_decision_counts"],
            "human_approval_counts": safety_approval_metrics["human_approval_counts"],
            "unknown_event_count": self._unknown_event_count(checked_events),
        }
        return MetricSummary(
            mission_id=self._infer_mission_id(checked_events),
            event_count=len(checked_events),
            risk_counts=self._allowed_metric_risk_counts(risk_metrics["risk_counts"]),
            replan_count=self._count_matching_recommendation(checked_events, "replan"),
            hold_count=self._count_matching_recommendation(checked_events, "hold")
            + int(safety_approval_metrics["safety_decision_counts"].get("hold", 0)),
            approval_count=int(safety_approval_metrics["human_approval_counts"].get("approved", 0))
            + int(safety_approval_metrics["safety_decision_counts"].get("approved", 0)),
            blocked_count=int(route_metrics["route_verdict_counts"].get("blocked", 0))
            + int(task_fleet_metrics["task_status_counts"].get("blocked", 0))
            + int(safety_approval_metrics["safety_decision_counts"].get("blocked", 0)),
            metadata=metadata,
        )

    def build_metrics_payload(self, summary: MetricSummary) -> Dict[str, object]:
        if not isinstance(summary, MetricSummary):
            raise ValueError("invalid_metrics_summary: summary is required")
        summary.validate()
        metadata = copy.deepcopy(summary.metadata)
        payload = {
            "schema_version": "v2-5B-c2-metrics-payload",
            "mission_id": summary.mission_id,
            "event_count": int(summary.event_count),
            "event_type_counts": self._sorted_counts(metadata.get("event_type_counts", {})),
            "task_status_counts": self._sorted_counts(metadata.get("task_status_counts", {})),
            "fleet_assignment_counts": self._sorted_counts(metadata.get("fleet_assignment_counts", {})),
            "risk_counts": self._sorted_counts(metadata.get("risk_counts", summary.risk_counts)),
            "route_count": int(metadata.get("route_count", 0)),
            "route_verdict_counts": self._sorted_counts(metadata.get("route_verdict_counts", {})),
            "safety_decision_counts": self._sorted_counts(metadata.get("safety_decision_counts", {})),
            "human_approval_counts": self._sorted_counts(metadata.get("human_approval_counts", {})),
            "unknown_event_count": int(metadata.get("unknown_event_count", 0)),
            "replay_metric_summary": {
                "replan_count": int(summary.replan_count),
                "hold_count": int(summary.hold_count),
                "approval_count": int(summary.approval_count),
                "blocked_count": int(summary.blocked_count),
            },
        }
        return self._json_safe_dict(payload)

    def build_risk_metrics(self, events: Sequence[MissionEvent]) -> Dict[str, object]:
        checked_events = self._validate_events(events)
        counts: Dict[str, int] = {}
        for event in checked_events:
            if event.event_type != "risk.signal.created":
                continue
            category = self._coerce_label(
                self._first_present(event.payload, ("category", "risk_category")),
                default="unknown",
            )
            counts[category] = counts.get(category, 0) + 1
        return self._json_safe_dict({"risk_counts": self._sorted_counts(counts)})

    def build_route_metrics(self, events: Sequence[MissionEvent]) -> Dict[str, object]:
        checked_events = self._validate_events(events)
        counts: Dict[str, int] = {}
        for event in checked_events:
            if event.event_type != "route.planned":
                continue
            verdict = self._coerce_label(
                self._first_present(event.payload, ("constraint_verdict", "verdict")),
                default="unknown",
            )
            if verdict not in {"valid", "warning", "blocked"}:
                verdict = "unknown"
            counts[verdict] = counts.get(verdict, 0) + 1
        return self._json_safe_dict(
            {
                "route_count": sum(counts.values()),
                "route_verdict_counts": self._sorted_counts(counts),
            }
        )

    def build_task_fleet_metrics(self, events: Sequence[MissionEvent]) -> Dict[str, object]:
        checked_events = self._validate_events(events)
        task_counts: Dict[str, int] = {}
        fleet_counts: Dict[str, int] = {}
        for event in checked_events:
            if event.event_type in {"mission.task.created", "mission.task.updated"}:
                status = self._coerce_label(
                    self._first_present(event.payload, ("status", "task_status")),
                    default="unknown",
                )
                task_counts[status] = task_counts.get(status, 0) + 1
            if event.event_type.startswith("fleet."):
                assignment = self._fleet_assignment_label(event.payload)
                fleet_counts[assignment] = fleet_counts.get(assignment, 0) + 1
        return self._json_safe_dict(
            {
                "task_status_counts": self._sorted_counts(task_counts),
                "fleet_assignment_counts": self._sorted_counts(fleet_counts),
            }
        )

    def build_safety_approval_metrics(self, events: Sequence[MissionEvent]) -> Dict[str, object]:
        checked_events = self._validate_events(events)
        safety_counts: Dict[str, int] = {}
        approval_counts: Dict[str, int] = {}
        for event in checked_events:
            if event.event_type.startswith("safety."):
                status = self._coerce_label(
                    self._first_present(event.payload, ("decision", "status", "verdict")),
                    default="unknown",
                )
                safety_counts[status] = safety_counts.get(status, 0) + 1
            if event.event_type.startswith("human_approval.") or event.event_type.startswith("human.approval."):
                approval = self._approval_label(event.payload)
                approval_counts[approval] = approval_counts.get(approval, 0) + 1
        return self._json_safe_dict(
            {
                "safety_decision_counts": self._sorted_counts(safety_counts),
                "human_approval_counts": self._sorted_counts(approval_counts),
            }
        )

    def build_event_type_counts(self, events: Sequence[MissionEvent]) -> Dict[str, int]:
        checked_events = self._validate_events(events)
        counts: Dict[str, int] = {}
        for event in checked_events:
            counts[event.event_type] = counts.get(event.event_type, 0) + 1
        return self._sorted_counts(counts)

    def _validate_events(self, events: Sequence[MissionEvent]) -> List[MissionEvent]:
        if isinstance(events, (str, bytes)) or not isinstance(events, Sequence):
            raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
        checked_events: List[MissionEvent] = []
        for event in events:
            if not isinstance(event, MissionEvent):
                raise ValueError("invalid_events: events must be a sequence of MissionEvent objects")
            event.validate()
            checked_events.append(copy.deepcopy(event))
        return checked_events

    def _unknown_event_count(self, events: Sequence[MissionEvent]) -> int:
        return sum(1 for event in events if self.dashboard_builder.event_family(event.event_type) == "unknown")

    def _infer_mission_id(self, events: Sequence[MissionEvent]) -> str:
        for key in ("mission_id", "request_id", "task_id"):
            for event in events:
                value = self._find_key(event.payload, key)
                if isinstance(value, str) and value:
                    return value
        return "mock-mission"

    def _count_matching_recommendation(self, events: Sequence[MissionEvent], expected: str) -> int:
        count = 0
        for event in events:
            if event.event_type != "threat.assessment.created":
                continue
            recommendation = self._coerce_label(self._find_key(event.payload, "recommendation"), default="")
            if recommendation == expected:
                count += 1
        return count

    def _fleet_assignment_label(self, payload: Dict[str, object]) -> str:
        if self._first_present(payload, ("assigned_asset_id", "current_task_id")) is not None:
            return "assigned"
        status = self._first_present(payload, ("status", "availability", "available"))
        if isinstance(status, bool):
            return "available" if status else "unavailable"
        label = self._coerce_label(status, default="unknown")
        if label in {"assigned", "available", "unavailable"}:
            return label
        return "unknown"

    def _approval_label(self, payload: Dict[str, object]) -> str:
        approved = self._find_key(payload, "approved")
        if isinstance(approved, bool):
            return "approved" if approved else "rejected"
        decision = self._first_present(payload, ("status", "approval_status", "decision"))
        return self._coerce_label(decision, default="unknown")

    def _first_present(self, value: object, keys: Sequence[str]) -> object:
        for key in keys:
            found = self._find_key(value, key)
            if found is not None:
                return found
        return None

    def _find_key(self, value: object, wanted_key: str) -> object:
        if isinstance(value, dict):
            if wanted_key in value:
                return copy.deepcopy(value[wanted_key])
            for key in sorted(value):
                found = self._find_key(value[key], wanted_key)
                if found is not None:
                    return found
        if isinstance(value, list):
            for item in value:
                found = self._find_key(item, wanted_key)
                if found is not None:
                    return found
        return None

    @staticmethod
    def _coerce_label(value: object, default: str) -> str:
        if isinstance(value, str) and value:
            return value
        if isinstance(value, bool):
            return "true" if value else "false"
        return default

    @staticmethod
    def _allowed_metric_risk_counts(counts: object) -> Dict[str, int]:
        if not isinstance(counts, dict):
            return {}
        return {
            str(key): int(value)
            for key, value in counts.items()
            if key != "unknown" and isinstance(value, int)
        }

    @staticmethod
    def _sorted_counts(counts: object) -> Dict[str, int]:
        if not isinstance(counts, dict):
            return {}
        normalized: Dict[str, int] = {}
        for key, value in counts.items():
            if isinstance(value, bool) or not isinstance(value, int):
                continue
            normalized[str(key)] = int(value)
        return {key: normalized[key] for key in sorted(normalized)}

    @staticmethod
    def _json_safe_dict(value: Dict[str, object]) -> Dict[str, object]:
        try:
            json.dumps(value, allow_nan=False, sort_keys=True)
        except (TypeError, ValueError) as exc:
            raise ValueError("dashboard metrics payload must be JSON-safe") from exc
        return value


class C2ReplayReportBuilder:
    """Build deterministic JSON and Markdown audit reports from replay payloads."""

    def __init__(
        self,
        dashboard_builder: Optional[DashboardReplayBuilder] = None,
        metrics_exporter: Optional[C2MetricsExporter] = None,
    ) -> None:
        if dashboard_builder is not None and not isinstance(dashboard_builder, DashboardReplayBuilder):
            raise ValueError("dashboard_builder must be a DashboardReplayBuilder")
        if metrics_exporter is not None and not isinstance(metrics_exporter, C2MetricsExporter):
            raise ValueError("metrics_exporter must be a C2MetricsExporter")
        self.dashboard_builder = dashboard_builder or DashboardReplayBuilder()
        self.metrics_exporter = metrics_exporter or C2MetricsExporter()

    def build_json_report(self, replay_payload: Dict[str, object], metrics_payload: Dict[str, object]) -> Dict[str, object]:
        if not isinstance(replay_payload, dict):
            raise ValueError("invalid_replay_payload: replay payload must be a dict")
        if not isinstance(metrics_payload, dict):
            raise ValueError("invalid_metrics_payload: metrics payload must be a dict")
        report = {
            "schema_version": "v2-5B-c2-replay-report",
            "replay": self.redact_sensitive_values(copy.deepcopy(replay_payload)),
            "metrics": self.redact_sensitive_values(copy.deepcopy(metrics_payload)),
            "scope_and_safety": {
                "read_only": True,
                "command_free": True,
                "runtime_free": True,
                "no_vehicle_control": True,
            },
        }
        return self._json_safe_dict(report)

    def build_markdown_report(self, replay_payload: Dict[str, object], metrics_payload: Dict[str, object]) -> str:
        report = self.build_json_report(replay_payload, metrics_payload)
        replay = report["replay"]
        metrics = report["metrics"]
        if not isinstance(replay, dict) or not isinstance(metrics, dict):
            raise ValueError("report payloads must be dictionaries")
        timeline = replay.get("timeline", [])
        if not isinstance(timeline, list):
            timeline = []
        lines = [
            "# Mission Replay Summary",
            f"- event_count: {replay.get('event_count', 0)}",
            f"- frame_count: {replay.get('frame_count', 0)}",
            "## Event Timeline",
        ]
        lines.extend(self._timeline_lines(timeline, families=None))
        lines.extend(
            [
                "## Task and Fleet Status",
                f"- task_status_counts: {self._json_inline(metrics.get('task_status_counts', {}))}",
                f"- fleet_assignment_counts: {self._json_inline(metrics.get('fleet_assignment_counts', {}))}",
                "## Risk Timeline",
            ]
        )
        lines.extend(self._timeline_lines(timeline, families={"risk", "threat"}))
        lines.extend(
            [
                "## Route Timeline",
            ]
        )
        lines.extend(self._timeline_lines(timeline, families={"route"}))
        lines.extend(
            [
                "## Safety and Human Approval Timeline",
            ]
        )
        lines.extend(self._timeline_lines(timeline, families={"safety", "human_approval"}))
        lines.extend(
            [
                "## Metrics Summary",
                f"- event_type_counts: {self._json_inline(metrics.get('event_type_counts', {}))}",
                f"- risk_counts: {self._json_inline(metrics.get('risk_counts', {}))}",
                f"- route_verdict_counts: {self._json_inline(metrics.get('route_verdict_counts', {}))}",
                f"- task_status_counts: {self._json_inline(metrics.get('task_status_counts', {}))}",
                f"- safety_decision_counts: {self._json_inline(metrics.get('safety_decision_counts', {}))}",
                f"- human_approval_counts: {self._json_inline(metrics.get('human_approval_counts', {}))}",
                "## Scope and Safety Notes",
                "- read_only: true",
                "- command_free: true",
                "- runtime_free: true",
                "- no_vehicle_control: true",
            ]
        )
        return "\n".join(lines) + "\n"

    def redact_sensitive_values(self, value: object) -> object:
        if isinstance(value, dict):
            redacted: Dict[str, object] = {}
            for key in sorted(value):
                key_text = str(key)
                if DashboardReplayBuilder._is_sensitive_key(key_text):
                    redacted[key_text] = REDACTED_VALUE
                else:
                    redacted[key_text] = self.redact_sensitive_values(value[key])
            return redacted
        if isinstance(value, list):
            return [self.redact_sensitive_values(item) for item in value]
        return value

    def _timeline_lines(self, timeline: Sequence[object], families: Optional[set[str]]) -> List[str]:
        lines: List[str] = []
        for entry in timeline:
            if not isinstance(entry, dict):
                continue
            family = entry.get("family")
            if families is not None and family not in families:
                continue
            lines.append(
                "- "
                f"{entry.get('event_id', '')}: "
                f"{entry.get('event_type', '')} "
                f"({entry.get('family', 'unknown')})"
            )
        return lines or ["- none"]

    @staticmethod
    def _json_inline(value: object) -> str:
        return json.dumps(value, allow_nan=False, sort_keys=True)

    @staticmethod
    def _json_safe_dict(value: Dict[str, object]) -> Dict[str, object]:
        try:
            json.dumps(value, allow_nan=False, sort_keys=True)
        except (TypeError, ValueError) as exc:
            raise ValueError("dashboard report payload must be JSON-safe") from exc
        return value
