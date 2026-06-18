"""Read-only dashboard replay payload assembly for GWM-UAV-C2."""

from __future__ import annotations

import copy
import json
from typing import Dict, List, Optional, Sequence

from src.c2.mission_types import MissionEvent, ReplayFrame
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
