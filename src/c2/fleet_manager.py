"""Mock-first fleet manager for GWM-UAV-C2."""

from __future__ import annotations

import copy
from typing import Dict, List, Optional

from src.c2.event_bus import MissionEventBus
from src.c2.mission_types import FleetAsset, MissionEvent, MissionTask, UAVState
from src.c2.state_store import MissionStateStore


class FleetManager:
    """Track mock fleet assets and deterministic task assignment."""

    def __init__(
        self,
        event_bus: Optional[MissionEventBus] = None,
        state_store: Optional[MissionStateStore] = None,
    ) -> None:
        self.event_bus = event_bus or MissionEventBus()
        self.state_store = state_store
        self._assets: Dict[str, FleetAsset] = {}
        self._uav_states: Dict[str, UAVState] = {}
        self._event_counter = 0

    def register_asset(self, asset: FleetAsset) -> FleetAsset:
        self._validate_asset(asset)
        if asset.asset_id in self._assets:
            raise ValueError("asset_already_assigned: asset id already registered")
        self._assets[asset.asset_id] = copy.deepcopy(asset)
        self.publish_event(self.make_event("fleet.asset.registered", asset.to_dict()))
        return copy.deepcopy(asset)

    def update_asset(self, asset: FleetAsset) -> FleetAsset:
        self._validate_asset(asset)
        if asset.asset_id not in self._assets:
            raise ValueError("asset_not_found: asset id not found")
        previous = self._assets[asset.asset_id]
        self._assets[asset.asset_id] = copy.deepcopy(asset)
        self.publish_event(
            self.make_event(
                "fleet.asset.updated",
                asset.to_dict(),
                metadata={
                    "previous_available": previous.available,
                    "new_available": asset.available,
                    "previous_task_id": previous.current_task_id,
                    "new_task_id": asset.current_task_id,
                },
            )
        )
        return copy.deepcopy(asset)

    def update_uav_state(self, state: UAVState) -> UAVState:
        if not isinstance(state, UAVState):
            raise ValueError("invalid_request: state must be a UAVState")
        state.validate()
        if state.asset_id not in self._assets:
            raise ValueError("asset_not_found: asset id not found")
        self._uav_states[state.asset_id] = copy.deepcopy(state)
        self.publish_event(self.make_event("uav.state.updated", state.to_dict()))
        return copy.deepcopy(state)

    def list_assets(self) -> List[FleetAsset]:
        return [copy.deepcopy(self._assets[asset_id]) for asset_id in sorted(self._assets)]

    def get_asset(self, asset_id: str) -> Optional[FleetAsset]:
        asset = self._assets.get(asset_id)
        return copy.deepcopy(asset) if asset is not None else None

    def available_assets(self, required_capability: Optional[str] = None) -> List[FleetAsset]:
        assets = self.eligible_assets(required_capability)
        return [copy.deepcopy(asset) for asset in assets]

    def assign_task(self, task: MissionTask, required_capability: Optional[str] = None) -> MissionTask:
        self._validate_task(task)
        if task.status != "pending":
            raise ValueError("invalid_task_status_transition: only pending tasks can be assigned")
        selected = self.select_asset(required_capability)
        previous_asset = copy.deepcopy(selected)
        updated_asset = FleetAsset(
            asset_id=selected.asset_id,
            backend=selected.backend,
            capabilities=copy.deepcopy(selected.capabilities),
            available=False,
            health=copy.deepcopy(selected.health),
            current_task_id=task.task_id,
            metadata=copy.deepcopy(selected.metadata),
        )
        updated_task_metadata = copy.deepcopy(task.metadata)
        updated_task_metadata["assigned_asset_id"] = selected.asset_id
        if required_capability is not None:
            updated_task_metadata["required_capability"] = required_capability
        updated_task = MissionTask(
            task_id=task.task_id,
            request_id=task.request_id,
            objective=task.objective,
            status="assigned",
            priority=task.priority,
            constraints=copy.deepcopy(task.constraints),
            assigned_asset_id=selected.asset_id,
            created_at=task.created_at,
            metadata=updated_task_metadata,
        )
        self._assets[selected.asset_id] = copy.deepcopy(updated_asset)
        self.publish_event(
            self.make_event(
                "fleet.asset.updated",
                updated_asset.to_dict(),
                metadata={
                    "previous_available": previous_asset.available,
                    "new_available": updated_asset.available,
                    "previous_task_id": previous_asset.current_task_id,
                    "new_task_id": updated_asset.current_task_id,
                    "assigned_asset_id": selected.asset_id,
                    "required_capability": required_capability,
                },
            )
        )
        self.publish_event(
            self.make_event(
                "mission.task.updated",
                updated_task.to_dict(),
                metadata={
                    "previous_status": task.status,
                    "new_status": updated_task.status,
                    "assigned_asset_id": selected.asset_id,
                    "required_capability": required_capability,
                },
            )
        )
        return copy.deepcopy(updated_task)

    def release_asset(self, asset_id: str) -> FleetAsset:
        if asset_id not in self._assets:
            raise ValueError("asset_not_found: asset id not found")
        previous = self._assets[asset_id]
        released = FleetAsset(
            asset_id=previous.asset_id,
            backend=previous.backend,
            capabilities=copy.deepcopy(previous.capabilities),
            available=True,
            health=copy.deepcopy(previous.health),
            current_task_id=None,
            metadata=copy.deepcopy(previous.metadata),
        )
        self._assets[asset_id] = copy.deepcopy(released)
        self.publish_event(
            self.make_event(
                "fleet.asset.updated",
                released.to_dict(),
                metadata={
                    "previous_available": previous.available,
                    "new_available": released.available,
                    "previous_task_id": previous.current_task_id,
                    "new_task_id": released.current_task_id,
                },
            )
        )
        return copy.deepcopy(released)

    def eligible_assets(self, required_capability: Optional[str] = None) -> List[FleetAsset]:
        eligible = []
        for asset in self._assets.values():
            if not asset.available or asset.current_task_id is not None:
                continue
            if required_capability is not None and required_capability not in asset.capabilities:
                continue
            eligible.append(asset)
        return [copy.deepcopy(asset) for asset in sorted(eligible, key=lambda item: item.asset_id)]

    def select_asset(self, required_capability: Optional[str] = None) -> FleetAsset:
        eligible = self.eligible_assets(required_capability)
        if eligible:
            return eligible[0]
        available_without_capability = [
            asset
            for asset in self._assets.values()
            if asset.available and asset.current_task_id is None
        ]
        if required_capability is not None and available_without_capability:
            raise ValueError("missing_required_capability: no available asset has required capability")
        raise ValueError("no_available_asset: no eligible asset is available")

    def make_event(
        self,
        event_type: str,
        payload: Dict[str, object],
        source: str = "fleet_manager",
        metadata: Optional[Dict[str, object]] = None,
    ) -> MissionEvent:
        self._event_counter += 1
        timestamp = float(payload.get("timestamp", payload.get("created_at", self._event_counter)))
        return MissionEvent(
            event_id=f"fleet-event-{self._event_counter:06d}",
            event_type=event_type,
            timestamp=timestamp,
            source=source,
            payload=copy.deepcopy(payload),
            metadata=copy.deepcopy(metadata or {}),
        )

    def publish_event(self, event: MissionEvent) -> MissionEvent:
        published = self.event_bus.publish(event)
        if self.state_store is not None:
            self.state_store.apply_event(event)
        return published

    @staticmethod
    def _validate_asset(asset: FleetAsset) -> None:
        if not isinstance(asset, FleetAsset):
            raise ValueError("invalid_request: asset must be a FleetAsset")
        asset.validate()

    @staticmethod
    def _validate_task(task: MissionTask) -> None:
        if not isinstance(task, MissionTask):
            raise ValueError("invalid_request: task must be a MissionTask")
        task.validate()
