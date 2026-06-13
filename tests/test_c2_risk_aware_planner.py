"""Tests for the v2-4B mock risk-aware route scoring core."""

from __future__ import annotations

import json

import pytest

from src.c2 import (
    BLOCKED_CONSTRAINT_PENALTY,
    RISK_WEIGHT,
    SAFE_OFFSET_DISTANCE,
    WARNING_CONSTRAINT_PENALTY,
    AirspaceConstraint,
    FleetAsset,
    MissionTask,
    RiskAwarePlanner,
    ThreatAssessment,
    UTMAirspaceLayer,
)
import src.c2.risk_aware_planner as planner_module


def _task() -> MissionTask:
    return MissionTask(
        task_id="task-001",
        request_id="request-001",
        objective="Inspect mock corridor",
    )


def _asset() -> FleetAsset:
    return FleetAsset(
        asset_id="uav-001",
        backend="mock",
        capabilities=["survey"],
    )


def _risk(total_risk: float = 0.5) -> ThreatAssessment:
    return ThreatAssessment(
        assessment_id="assessment-001",
        mission_id="mission-001",
        total_risk=total_risk,
        recommendation="replan",
        explanation="Mock route risk fixture.",
    )


def _bbox(min_x: float, max_x: float, min_y: float, max_y: float) -> dict[str, object]:
    return {
        "type": "bbox",
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }


def test_planner_generates_deterministic_candidate_routes() -> None:
    planner = RiskAwarePlanner()

    routes = planner.generate_candidate_routes(_task(), _asset(), {"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0})

    assert [route.route_id for route in routes] == ["route-000001", "route-000002", "route-000003"]
    assert [route.metadata["candidate_type"] for route in routes] == ["direct", "midpoint", "safe_offset"]


def test_planner_generates_direct_route() -> None:
    planner = RiskAwarePlanner()

    route = planner.generate_candidate_routes(_task(), _asset(), {"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0})[0]

    assert route.waypoints == [{"x": 0.0, "y": 0.0}, {"x": 10.0, "y": 0.0}]


def test_planner_generates_midpoint_route() -> None:
    planner = RiskAwarePlanner()

    route = planner.generate_candidate_routes(
        _task(),
        _asset(),
        {"x": 0.0, "y": 0.0, "z": 10.0},
        {"x": 10.0, "y": 20.0, "z": 30.0},
    )[1]

    assert route.waypoints[1] == {"x": 5.0, "y": 10.0, "z": 20.0}


def test_planner_generates_safe_offset_route() -> None:
    planner = RiskAwarePlanner()

    route = planner.generate_candidate_routes(
        _task(),
        _asset(),
        {"x": 0.0, "y": 0.0, "altitude": 10.0},
        {"x": 10.0, "y": 20.0, "altitude": 30.0},
    )[2]

    assert route.waypoints[1] == {"x": 5.0, "y": 10.0 + SAFE_OFFSET_DISTANCE, "altitude": 20.0}


def test_planner_route_distance_2d() -> None:
    assert RiskAwarePlanner().route_distance([{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}]) == pytest.approx(5.0)


def test_planner_route_distance_3d() -> None:
    assert RiskAwarePlanner().route_distance(
        [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 3.0, "y": 4.0, "z": 12.0}]
    ) == pytest.approx(13.0)


def test_planner_scores_route_deterministically() -> None:
    planner = RiskAwarePlanner()
    waypoints = [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}]

    first = planner.score_route(waypoints)
    second = planner.score_route(waypoints)

    assert first == second
    assert first["score"] == pytest.approx(5.0)


def test_planner_penalizes_risk_score() -> None:
    score = RiskAwarePlanner().score_route(
        [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}],
        risk_assessment=_risk(0.5),
    )

    assert score["risk_score"] == pytest.approx(0.5)
    assert score["risk_penalty"] == pytest.approx(0.5 * RISK_WEIGHT)
    assert score["score"] == pytest.approx(55.0)


def test_planner_penalizes_warning_constraint() -> None:
    score = RiskAwarePlanner().score_route(
        [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}],
        constraint_verdict="warning",
    )

    assert score["constraint_penalty"] == WARNING_CONSTRAINT_PENALTY
    assert score["score"] == pytest.approx(55.0)


def test_planner_penalizes_blocked_constraint() -> None:
    score = RiskAwarePlanner().score_route(
        [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}],
        constraint_verdict="blocked",
    )

    assert score["constraint_penalty"] == BLOCKED_CONSTRAINT_PENALTY
    assert score["score"] == pytest.approx(1005.0)


def test_planner_uses_airspace_verdict() -> None:
    airspace = UTMAirspaceLayer(
        [
            AirspaceConstraint(
                constraint_id="corridor-001",
                constraint_type="corridor",
                geometry=_bbox(0.0, 10.0, 0.0, 10.0),
            )
        ]
    )
    planner = RiskAwarePlanner(airspace_layer=airspace)

    routes = planner.generate_candidate_routes(_task(), _asset(), {"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 1.0})

    assert [route.constraint_verdict for route in routes] == ["warning", "warning", "warning"]


def test_planner_selects_lowest_score_valid_route() -> None:
    planner = RiskAwarePlanner()
    first = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}], 10.0, 0.0, "valid")
    second = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}], 5.0, 0.0, "valid")

    selected = planner.select_route([first, second])

    assert selected.route_id == "route-000002"


def test_planner_selects_warning_route_over_blocked_route() -> None:
    planner = RiskAwarePlanner()
    warning = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}], 500.0, 0.0, "warning")
    blocked = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}], 1.0, 0.0, "blocked")

    selected = planner.select_route([blocked, warning])

    assert selected.route_id == warning.route_id
    assert selected.constraint_verdict == "warning"


def test_planner_all_blocked_still_marks_selected_blocked() -> None:
    planner = RiskAwarePlanner()
    first = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}], 3.0, 0.0, "blocked")
    second = planner.create_planned_route("task-001", [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}], 1.0, 0.0, "blocked")

    selected = planner.select_route([first, second])

    assert selected.route_id == second.route_id
    assert selected.constraint_verdict == "blocked"


def test_planner_creates_planned_route_json_safe() -> None:
    route = RiskAwarePlanner().create_planned_route(
        "task-001",
        [{"x": 0.0, "y": 0.0}, {"x": 3.0, "y": 4.0}],
        score=5.0,
        risk_score=0.0,
        constraint_verdict="valid",
        metadata={"score_breakdown": {"distance_cost": 5.0}},
    )

    assert route.route_id == "route-000001"
    json.dumps(route.to_dict(), allow_nan=False)


def test_planner_rejects_invalid_waypoint() -> None:
    with pytest.raises(ValueError, match="invalid_waypoint"):
        RiskAwarePlanner().route_distance([{"x": 0.0, "y": 0.0}, {"x": "bad", "y": 1.0}])


def test_planner_rejects_invalid_route_verdict() -> None:
    with pytest.raises(ValueError, match="invalid_route_verdict"):
        RiskAwarePlanner().constraint_penalty("live_airspace")


def test_planner_rejects_empty_route_selection() -> None:
    with pytest.raises(ValueError, match="no_routes_available"):
        RiskAwarePlanner().select_route([])


def test_planner_imports_without_runtime_dependencies() -> None:
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
