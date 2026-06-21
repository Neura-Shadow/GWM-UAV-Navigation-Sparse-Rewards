"""In-memory mission state store for GWM-UAV-C2 mock workflows."""

from __future__ import annotations

import copy
from typing import Any, Dict, List, Optional

from src.c2.mission_types import (
    AirspaceConstraint,
    FleetAsset,
    HumanApprovalRecord,
    MissionEvent,
    MissionRequest,
    MissionTask,
    PlannedRoute,
    RiskSignal,
    SafetyDecision,
    ThreatAssessment,
    UAVState,
    ensure_json_safe_dict,
    ensure_non_empty_string,
)


_EVENT_PAYLOAD_KEYS: Dict[str, str] = {
    "mission.requested": "request",
    "mission.task.created": "task",
    "mission.task.updated": "task",
    "fleet.asset.registered": "asset",
    "fleet.asset.updated": "asset",
    "uav.state.updated": "state",
    "risk.signal.created": "risk_signal",
    "threat.assessment.created": "assessment",
    "airspace.constraint.created": "constraint",
    "route.planned": "route",
    "safety.decision.created": "decision",
    "human.approval.recorded": "approval",
}


class MissionStateStore:
    """Deterministic, JSON-safe mission state store.

    Unknown events are retained in ordered history. Known events are validated
    and applied atomically: invalid known payloads raise before history changes.
    """

    def __init__(self) -> None:
        self.requests: Dict[str, MissionRequest] = {}
        self.tasks: Dict[str, MissionTask] = {}
        self.assets: Dict[str, FleetAsset] = {}
        self.uav_states: Dict[str, UAVState] = {}
        self.risk_signals: Dict[str, RiskSignal] = {}
        self.threat_assessments: Dict[str, ThreatAssessment] = {}
        self.airspace_constraints: Dict[str, AirspaceConstraint] = {}
        self.planned_routes: Dict[str, PlannedRoute] = {}
        self.safety_decisions: Dict[str, SafetyDecision] = {}
        self.human_approvals: Dict[str, HumanApprovalRecord] = {}
        self.events: List[MissionEvent] = []

    def apply_event(self, event: MissionEvent) -> None:
        if not isinstance(event, MissionEvent):
            raise ValueError("event must be a MissionEvent")
        event.validate()

        if event.event_type == "mission.requested":
            request = MissionRequest.from_dict(self._payload(event))
            self.requests[request.request_id] = request
        elif event.event_type in {"mission.task.created", "mission.task.updated"}:
            task = MissionTask.from_dict(self._payload(event))
            self.tasks[task.task_id] = task
        elif event.event_type in {"fleet.asset.registered", "fleet.asset.updated"}:
            asset = FleetAsset.from_dict(self._payload(event))
            self.assets[asset.asset_id] = asset
        elif event.event_type == "uav.state.updated":
            state = UAVState.from_dict(self._payload(event))
            self.uav_states[state.asset_id] = state
        elif event.event_type == "risk.signal.created":
            signal = RiskSignal.from_dict(self._payload(event))
            self.risk_signals[signal.signal_id] = signal
        elif event.event_type == "threat.assessment.created":
            assessment = ThreatAssessment.from_dict(self._payload(event))
            self.threat_assessments[assessment.assessment_id] = assessment
        elif event.event_type == "airspace.constraint.created":
            constraint = AirspaceConstraint.from_dict(self._payload(event))
            self.airspace_constraints[constraint.constraint_id] = constraint
        elif event.event_type == "route.planned":
            route = PlannedRoute.from_dict(self._payload(event))
            self.planned_routes[route.route_id] = route
        elif event.event_type == "safety.decision.created":
            decision = SafetyDecision.from_dict(self._payload(event))
            self.safety_decisions[decision.decision_id] = decision
        elif event.event_type == "human.approval.recorded":
            approval = HumanApprovalRecord.from_dict(self._payload(event))
            self.human_approvals[approval.approval_id] = approval

        self.events.append(copy.deepcopy(event))

    def snapshot(self) -> Dict[str, Any]:
        snapshot = {
            "requests": self._collection_to_dict(self.requests),
            "tasks": self._collection_to_dict(self.tasks),
            "assets": self._collection_to_dict(self.assets),
            "uav_states": self._collection_to_dict(self.uav_states),
            "risk_signals": self._collection_to_dict(self.risk_signals),
            "threat_assessments": self._collection_to_dict(self.threat_assessments),
            "airspace_constraints": self._collection_to_dict(self.airspace_constraints),
            "planned_routes": self._collection_to_dict(self.planned_routes),
            "safety_decisions": self._collection_to_dict(self.safety_decisions),
            "human_approvals": self._collection_to_dict(self.human_approvals),
            "events": [event.to_dict() for event in self.events],
        }
        ensure_json_safe_dict(snapshot, "snapshot")
        return snapshot

    def restore(self, snapshot: Dict[str, Any]) -> None:
        ensure_json_safe_dict(snapshot, "snapshot")
        restored = MissionStateStore()
        restored.requests = self._restore_collection(snapshot, "requests", MissionRequest, "request_id")
        restored.tasks = self._restore_collection(snapshot, "tasks", MissionTask, "task_id")
        restored.assets = self._restore_collection(snapshot, "assets", FleetAsset, "asset_id")
        restored.uav_states = self._restore_collection(snapshot, "uav_states", UAVState, "asset_id")
        restored.risk_signals = self._restore_collection(snapshot, "risk_signals", RiskSignal, "signal_id")
        restored.threat_assessments = self._restore_collection(
            snapshot,
            "threat_assessments",
            ThreatAssessment,
            "assessment_id",
        )
        restored.airspace_constraints = self._restore_collection(
            snapshot,
            "airspace_constraints",
            AirspaceConstraint,
            "constraint_id",
        )
        restored.planned_routes = self._restore_collection(snapshot, "planned_routes", PlannedRoute, "route_id")
        restored.safety_decisions = self._restore_collection(
            snapshot,
            "safety_decisions",
            SafetyDecision,
            "decision_id",
        )
        restored.human_approvals = self._restore_collection(
            snapshot,
            "human_approvals",
            HumanApprovalRecord,
            "approval_id",
        )
        events = snapshot.get("events", [])
        if not isinstance(events, list):
            raise ValueError("snapshot.events must be a list")
        restored.events = [MissionEvent.from_dict(event) for event in events]
        self.__dict__.update(restored.__dict__)

    def get_request(self, request_id: str) -> Optional[MissionRequest]:
        ensure_non_empty_string(request_id, "request_id")
        request = self.requests.get(request_id)
        return copy.deepcopy(request) if request is not None else None

    def get_task(self, task_id: str) -> Optional[MissionTask]:
        ensure_non_empty_string(task_id, "task_id")
        task = self.tasks.get(task_id)
        return copy.deepcopy(task) if task is not None else None

    def get_asset(self, asset_id: str) -> Optional[FleetAsset]:
        ensure_non_empty_string(asset_id, "asset_id")
        asset = self.assets.get(asset_id)
        return copy.deepcopy(asset) if asset is not None else None

    def get_uav_state(self, asset_id: str) -> Optional[UAVState]:
        ensure_non_empty_string(asset_id, "asset_id")
        state = self.uav_states.get(asset_id)
        return copy.deepcopy(state) if state is not None else None

    def list_tasks(self) -> List[MissionTask]:
        return [copy.deepcopy(task) for task in self.tasks.values()]

    def list_assets(self) -> List[FleetAsset]:
        return [copy.deepcopy(asset) for asset in self.assets.values()]

    def list_events(self) -> List[MissionEvent]:
        return copy.deepcopy(self.events)

    def clear(self) -> None:
        self.__init__()

    def _payload(self, event: MissionEvent) -> Dict[str, Any]:
        payload = copy.deepcopy(event.payload)
        ensure_json_safe_dict(payload, "event.payload")
        nested_key = _EVENT_PAYLOAD_KEYS.get(event.event_type)
        if nested_key and nested_key in payload:
            nested_payload = payload[nested_key]
            if not isinstance(nested_payload, dict):
                raise ValueError(f"{event.event_type} payload.{nested_key} must be a dictionary")
            return copy.deepcopy(nested_payload)
        return payload

    @staticmethod
    def _collection_to_dict(collection: Dict[str, Any]) -> Dict[str, Any]:
        return {key: value.to_dict() for key, value in collection.items()}

    @staticmethod
    def _restore_collection(
        snapshot: Dict[str, Any],
        key: str,
        model_cls: type,
        id_field: str,
    ) -> Dict[str, Any]:
        values = snapshot.get(key, {})
        if not isinstance(values, dict):
            raise ValueError(f"snapshot.{key} must be a dictionary")
        restored: Dict[str, Any] = {}
        for item_key, item in values.items():
            if not isinstance(item_key, str):
                raise ValueError(f"snapshot.{key} keys must be strings")
            model = model_cls.from_dict(item)
            model_id = getattr(model, id_field)
            if model_id != item_key:
                raise ValueError(f"snapshot.{key}.{item_key} id does not match {id_field}")
            restored[item_key] = model
        return restored
