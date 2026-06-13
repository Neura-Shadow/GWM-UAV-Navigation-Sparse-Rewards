"""Tests for the v2-4A mock UTM-style airspace constraint core."""

from __future__ import annotations

import json

import pytest

from src.c2 import (
    ALLOWED_AIRSPACE_CONSTRAINT_TYPES,
    ALLOWED_CONSTRAINT_VERDICTS,
    AirspaceConstraint,
    UTMAirspaceLayer,
)
import src.c2.airspace as airspace_module


def _bbox(min_x: float = 0.0, max_x: float = 100.0, min_y: float = 0.0, max_y: float = 100.0) -> dict[str, object]:
    return {
        "type": "bbox",
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }


def _constraint(
    constraint_id: str = "geo-001",
    constraint_type: str = "geofence",
    geometry: dict[str, object] | None = None,
    altitude_min: float | None = None,
    altitude_max: float | None = None,
    active: bool = True,
) -> AirspaceConstraint:
    return AirspaceConstraint(
        constraint_id=constraint_id,
        constraint_type=constraint_type,
        geometry=_bbox() if geometry is None else geometry,
        altitude_min=altitude_min,
        altitude_max=altitude_max,
        active=active,
    )


def test_airspace_accepts_valid_constraint() -> None:
    layer = UTMAirspaceLayer()
    accepted = layer.add_constraint(_constraint())

    assert accepted.constraint_id == "geo-001"
    assert accepted.constraint_type in ALLOWED_AIRSPACE_CONSTRAINT_TYPES
    assert layer.get_constraint("geo-001") == accepted


def test_airspace_rejects_invalid_constraint_type() -> None:
    layer = UTMAirspaceLayer()

    with pytest.raises(ValueError, match="invalid_constraint_type"):
        layer.add_constraint(_constraint(constraint_type="live_airspace"))


def test_airspace_rejects_invalid_bbox_geometry() -> None:
    layer = UTMAirspaceLayer()

    with pytest.raises(ValueError, match="invalid_constraint_geometry"):
        layer.add_constraint(_constraint(geometry={"type": "bbox", "min_x": 0.0}))


def test_airspace_validates_altitude_bounds() -> None:
    with pytest.raises(ValueError, match="altitude_min must be <= altitude_max"):
        AirspaceConstraint(
            constraint_id="alt-001",
            constraint_type="altitude_band",
            geometry={},
            altitude_min=80.0,
            altitude_max=20.0,
        )


def test_airspace_rejects_duplicate_constraint_id() -> None:
    layer = UTMAirspaceLayer([_constraint()])

    with pytest.raises(ValueError, match="duplicate_constraint_id"):
        layer.add_constraint(_constraint())


def test_airspace_remove_constraint() -> None:
    layer = UTMAirspaceLayer([_constraint()])

    removed = layer.remove_constraint("geo-001")

    assert removed.constraint_id == "geo-001"
    assert layer.get_constraint("geo-001") is None
    with pytest.raises(ValueError, match="constraint_not_found"):
        layer.remove_constraint("geo-001")


def test_airspace_list_constraints_deterministic() -> None:
    layer = UTMAirspaceLayer(
        [
            _constraint("zone-c", "no_fly_zone", _bbox(10.0, 20.0, 10.0, 20.0)),
            _constraint("zone-a", "geofence", _bbox()),
            _constraint("zone-b", "corridor", _bbox(0.0, 50.0, 0.0, 50.0)),
        ]
    )

    assert [constraint.constraint_id for constraint in layer.list_constraints()] == [
        "zone-a",
        "zone-b",
        "zone-c",
    ]


def test_airspace_active_only_filter() -> None:
    layer = UTMAirspaceLayer(
        [
            _constraint("active-001"),
            _constraint("inactive-001", active=False),
        ]
    )

    assert [constraint.constraint_id for constraint in layer.list_constraints(active_only=True)] == ["active-001"]


def test_airspace_blocks_no_fly_zone_violation() -> None:
    layer = UTMAirspaceLayer([_constraint("nfz-001", "no_fly_zone", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_waypoint({"x": 5.0, "y": 5.0})

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["nfz-001"]


def test_airspace_blocks_restricted_zone_violation() -> None:
    layer = UTMAirspaceLayer([_constraint("restricted-001", "restricted_zone", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_waypoint({"x": 5.0, "y": 5.0})

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["restricted-001"]


def test_airspace_validates_geofence_fixture() -> None:
    layer = UTMAirspaceLayer([_constraint("geo-001", "geofence", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_waypoint({"x": 5.0, "y": 5.0})

    assert result["verdict"] == "valid"
    assert result["checked_constraints"] == ["geo-001"]


def test_airspace_blocks_outside_geofence() -> None:
    layer = UTMAirspaceLayer([_constraint("geo-001", "geofence", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_waypoint({"x": 15.0, "y": 5.0})

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["geo-001"]


def test_airspace_altitude_band_blocks_low_altitude() -> None:
    layer = UTMAirspaceLayer([
        _constraint("alt-001", "altitude_band", geometry={}, altitude_min=10.0, altitude_max=50.0)
    ])

    result = layer.validate_waypoint({"x": 1.0, "y": 1.0, "z": 5.0})

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["alt-001"]


def test_airspace_altitude_band_blocks_high_altitude() -> None:
    layer = UTMAirspaceLayer([
        _constraint("alt-001", "altitude_band", geometry={}, altitude_min=10.0, altitude_max=50.0)
    ])

    result = layer.validate_waypoint({"x": 1.0, "y": 1.0, "altitude": 55.0})

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["alt-001"]


def test_airspace_altitude_band_warns_missing_altitude() -> None:
    layer = UTMAirspaceLayer([
        _constraint("alt-001", "altitude_band", geometry={}, altitude_min=10.0, altitude_max=50.0)
    ])

    result = layer.validate_waypoint({"x": 1.0, "y": 1.0})

    assert result["verdict"] == "warning"
    assert result["warnings"] == ["alt-001"]


def test_airspace_corridor_warns_outside_corridor() -> None:
    layer = UTMAirspaceLayer([_constraint("corridor-001", "corridor", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_waypoint({"x": 15.0, "y": 5.0})

    assert result["verdict"] == "warning"
    assert result["warnings"] == ["corridor-001"]


def test_airspace_route_verdict_valid() -> None:
    layer = UTMAirspaceLayer([_constraint("geo-001", "geofence", _bbox(0.0, 10.0, 0.0, 10.0))])

    assert layer.constraint_verdict([{"x": 1.0, "y": 1.0}, {"x": 9.0, "y": 9.0}]) == "valid"


def test_airspace_route_verdict_warning() -> None:
    layer = UTMAirspaceLayer([_constraint("corridor-001", "corridor", _bbox(0.0, 10.0, 0.0, 10.0))])

    assert layer.constraint_verdict([{"x": 1.0, "y": 1.0}, {"x": 12.0, "y": 9.0}]) == "warning"


def test_airspace_route_verdict_blocked() -> None:
    layer = UTMAirspaceLayer([_constraint("nfz-001", "no_fly_zone", _bbox(0.0, 10.0, 0.0, 10.0))])

    result = layer.validate_route([{"x": 20.0, "y": 20.0}, {"x": 5.0, "y": 5.0}])

    assert result["verdict"] == "blocked"
    assert result["violations"] == ["nfz-001"]
    assert result["checked_constraints"] == ["nfz-001"]


def test_airspace_inactive_constraints_ignored() -> None:
    layer = UTMAirspaceLayer([_constraint("nfz-001", "no_fly_zone", _bbox(0.0, 10.0, 0.0, 10.0), active=False)])

    result = layer.validate_waypoint({"x": 5.0, "y": 5.0})

    assert result["verdict"] == "valid"
    assert result["checked_constraints"] == []


def test_airspace_outputs_are_json_safe() -> None:
    layer = UTMAirspaceLayer(
        [
            _constraint("geo-001", "geofence", _bbox(0.0, 100.0, 0.0, 100.0)),
            _constraint("corridor-001", "corridor", _bbox(0.0, 50.0, 0.0, 50.0)),
        ]
    )

    result = layer.validate_route([{"x": 10.0, "y": 10.0, "z": 20.0}, {"x": 75.0, "y": 10.0, "z": 20.0}])

    assert result["verdict"] == "warning"
    assert result["warnings"] == ["corridor-001"]
    assert result["waypoint_results"][0]["verdict"] in ALLOWED_CONSTRAINT_VERDICTS
    json.dumps(result, allow_nan=False)


def test_airspace_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
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
        "torch",
    }

    assert runtime_modules.isdisjoint(airspace_module.__dict__)
