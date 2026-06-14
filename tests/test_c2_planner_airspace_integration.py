"""Integration tests for v2-4C planner, airspace, store, and replay wiring."""

from __future__ import annotations

import json

from src.c2 import (
    AirspaceConstraint,
    FleetAsset,
    MissionEventBus,
    MissionReplayEngine,
    MissionStateStore,
    MissionTask,
    PlannedRoute,
    RiskAwarePlanner,
    UTMAirspaceLayer,
)
import src.c2.risk_aware_planner as planner_module


def _bbox(min_x: float, max_x: float, min_y: float, max_y: float) -> dict[str, object]:
    return {
        "type": "bbox",
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }


def _task() -> MissionTask:
    return MissionTask(
        task_id="task-001",
        request_id="request-001",
        objective="Inspect mock route",
    )


def _asset() -> FleetAsset:
    return FleetAsset(asset_id="uav-001", backend="mock", capabilities=["survey"])


def _planner_with_airspace(*constraints: AirspaceConstraint) -> tuple[RiskAwarePlanner, MissionEventBus, MissionStateStore]:
    event_bus = MissionEventBus()
    state_store = MissionStateStore()
    airspace = UTMAirspaceLayer(list(constraints))
    planner = RiskAwarePlanner(airspace_layer=airspace, event_bus=event_bus, state_store=state_store)
    return planner, event_bus, state_store


def _valid_constraint() -> AirspaceConstraint:
    return AirspaceConstraint(
        constraint_id="geo-001",
        constraint_type="geofence",
        geometry=_bbox(0.0, 100.0, 0.0, 100.0),
    )


def _warning_constraint() -> AirspaceConstraint:
    return AirspaceConstraint(
        constraint_id="corridor-001",
        constraint_type="corridor",
        geometry=_bbox(0.0, 10.0, 0.0, 10.0),
    )


def _blocked_constraint() -> AirspaceConstraint:
    return AirspaceConstraint(
        constraint_id="nfz-001",
        constraint_type="no_fly_zone",
        geometry=_bbox(4.0, 6.0, 4.0, 6.0),
    )


def _selected_route(planner: RiskAwarePlanner, start: dict[str, object], goal: dict[str, object]) -> PlannedRoute:
    routes = planner.generate_candidate_routes(_task(), _asset(), start, goal)
    return planner.select_route(routes)


def test_planner_route_event_emission() -> None:
    planner, _, _ = _planner_with_airspace(_valid_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0})

    event = planner.make_planned_route_event(route)

    assert event.event_id == "route-event-000001"
    assert event.event_type == "route.planned"
    assert event.payload == route.to_dict()
    assert event.metadata["source"] == "risk_aware_planner"
    assert event.metadata["route_id"] == route.route_id
    assert event.metadata["selected"] is True


def test_planner_publish_planned_route_applies_state_store() -> None:
    planner, event_bus, state_store = _planner_with_airspace(_valid_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0})

    event = planner.publish_planned_route(route)

    assert event.event_type == "route.planned"
    assert [stored.event_id for stored in event_bus.history()] == ["route-event-000001"]
    assert list(state_store.planned_routes) == [route.route_id]
    assert state_store.planned_routes[route.route_id].constraint_verdict == "valid"


def test_planner_state_store_snapshot_restore() -> None:
    planner, _, state_store = _planner_with_airspace(_warning_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 1.0})

    planner.publish_planned_route(route)
    snapshot = state_store.snapshot()
    restored = MissionStateStore()
    restored.restore(snapshot)

    assert restored.snapshot() == snapshot
    assert restored.planned_routes[route.route_id].constraint_verdict == "warning"


def test_planner_replay_generates_frame_per_route_event() -> None:
    planner, event_bus, _ = _planner_with_airspace(_valid_constraint())
    for route in planner.generate_candidate_routes(_task(), _asset(), {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0}):
        planner.publish_planned_route(route)

    result = MissionReplayEngine().replay(event_bus.history())

    assert len(result.frames) == 3
    assert [frame.frame_id for frame in result.frames] == ["frame-000001", "frame-000002", "frame-000003"]


def test_planner_replay_final_snapshot_contains_planned_route() -> None:
    planner, event_bus, _ = _planner_with_airspace(_valid_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0})
    planner.publish_planned_route(route)

    result = MissionReplayEngine().replay(event_bus.history())

    assert sorted(result.final_snapshot["planned_routes"]) == [route.route_id]
    assert result.final_snapshot["planned_routes"][route.route_id]["constraint_verdict"] == "valid"


def test_planner_replay_preserves_warning_route_metadata() -> None:
    planner, event_bus, _ = _planner_with_airspace(_warning_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 1.0})
    planner.publish_planned_route(route)

    result = MissionReplayEngine().replay(event_bus.history())
    planned = result.final_snapshot["planned_routes"][route.route_id]

    assert planned["constraint_verdict"] == "warning"
    assert planned["metadata"]["selected"] is True
    assert result.frames[-1].route_summary["latest_constraint_verdict"] == "warning"


def test_planner_replay_preserves_blocked_route_metadata() -> None:
    planner, event_bus, _ = _planner_with_airspace(_blocked_constraint())
    route = _selected_route(planner, {"x": 5.0, "y": 5.0}, {"x": 5.5, "y": 5.5})
    planner.publish_planned_route(route)

    result = MissionReplayEngine().replay(event_bus.history())
    planned = result.final_snapshot["planned_routes"][route.route_id]

    assert planned["constraint_verdict"] == "blocked"
    assert planned["metadata"]["selection_role"] == "recommendation_only"
    assert result.metrics.blocked_count == 1


def test_planner_airspace_valid_route_flow() -> None:
    planner, _, state_store = _planner_with_airspace(_valid_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0})

    planner.publish_planned_route(route)

    assert route.constraint_verdict == "valid"
    assert state_store.snapshot()["planned_routes"][route.route_id]["constraint_verdict"] == "valid"


def test_planner_airspace_warning_route_flow() -> None:
    planner, _, state_store = _planner_with_airspace(_warning_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 1.0})

    planner.publish_planned_route(route)

    assert route.constraint_verdict == "warning"
    assert state_store.snapshot()["planned_routes"][route.route_id]["constraint_verdict"] == "warning"


def test_planner_airspace_blocked_route_flow() -> None:
    planner, _, state_store = _planner_with_airspace(_blocked_constraint())
    route = _selected_route(planner, {"x": 5.0, "y": 5.0}, {"x": 5.5, "y": 5.5})

    planner.publish_planned_route(route)

    assert route.constraint_verdict == "blocked"
    assert state_store.snapshot()["planned_routes"][route.route_id]["constraint_verdict"] == "blocked"


def test_planner_event_order_is_deterministic() -> None:
    planner, event_bus, _ = _planner_with_airspace(_valid_constraint())
    for route in planner.generate_candidate_routes(_task(), _asset(), {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0}):
        planner.publish_planned_route(route)

    assert [event.event_id for event in event_bus.history()] == [
        "route-event-000001",
        "route-event-000002",
        "route-event-000003",
    ]


def test_planner_replay_metric_summary_is_deterministic() -> None:
    planner, event_bus, _ = _planner_with_airspace(_blocked_constraint())
    for route in planner.generate_candidate_routes(_task(), _asset(), {"x": 5.0, "y": 5.0}, {"x": 5.5, "y": 5.5}):
        planner.publish_planned_route(route)
    events = event_bus.history()

    first = MissionReplayEngine().replay(events).metrics.to_dict()
    second = MissionReplayEngine().replay(events).metrics.to_dict()

    assert first == second
    assert first["blocked_count"] == 3


def test_planner_integration_outputs_are_json_safe() -> None:
    planner, event_bus, state_store = _planner_with_airspace(_warning_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 1.0})
    event = planner.publish_planned_route(route)
    replay = MissionReplayEngine().replay(event_bus.history())

    json.dumps(route.to_dict(), allow_nan=False)
    json.dumps(event.to_dict(), allow_nan=False)
    json.dumps(state_store.snapshot(), allow_nan=False)
    json.dumps(replay.to_dict(), allow_nan=False)


def test_planner_integration_does_not_mark_route_executable() -> None:
    planner, _, _ = _planner_with_airspace(_valid_constraint())
    route = _selected_route(planner, {"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0})
    event = planner.make_planned_route_event(route)

    assert route.metadata["selected"] is True
    assert route.metadata["executable"] is False
    assert route.metadata["selection_role"] == "recommendation_only"
    assert "command" not in event.payload
    assert "upload" not in event.payload


def test_planner_integration_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "asyncio",
        "cosysairsim",
        "geopandas",
        "isaacsim",
        "mavsdk",
        "message_filters",
        "numpy",
        "omni",
        "pxr",
        "rclpy",
        "shapely",
        "threading",
        "torch",
    }

    assert runtime_modules.isdisjoint(planner_module.__dict__)
