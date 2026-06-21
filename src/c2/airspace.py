"""Mock-first UTM-style airspace constraints for GWM-UAV-C2."""

from __future__ import annotations

import copy
import math
from typing import Dict, List, Optional

from src.c2.mission_types import AirspaceConstraint


ALLOWED_AIRSPACE_CONSTRAINT_TYPES = (
    "geofence",
    "no_fly_zone",
    "altitude_band",
    "corridor",
    "restricted_zone",
)

ALLOWED_CONSTRAINT_VERDICTS = (
    "valid",
    "warning",
    "blocked",
)

_BBOX_KEYS = ("min_x", "max_x", "min_y", "max_y")


class UTMAirspaceLayer:
    """Deterministic in-memory airspace constraint layer."""

    def __init__(self, constraints: Optional[List[AirspaceConstraint]] = None) -> None:
        self._constraints: Dict[str, AirspaceConstraint] = {}
        for constraint in constraints or []:
            self.add_constraint(constraint)

    def add_constraint(self, constraint: AirspaceConstraint) -> AirspaceConstraint:
        accepted = self.validate_constraint(constraint)
        if accepted.constraint_id in self._constraints:
            raise ValueError("duplicate_constraint_id: constraint already exists")
        self._constraints[accepted.constraint_id] = copy.deepcopy(accepted)
        return copy.deepcopy(accepted)

    def remove_constraint(self, constraint_id: str) -> AirspaceConstraint:
        if not isinstance(constraint_id, str) or not constraint_id.strip():
            raise ValueError("constraint_id must be a non-empty string")
        if constraint_id not in self._constraints:
            raise ValueError("constraint_not_found: constraint id not found")
        return self._constraints.pop(constraint_id)

    def get_constraint(self, constraint_id: str) -> Optional[AirspaceConstraint]:
        if not isinstance(constraint_id, str) or not constraint_id.strip():
            raise ValueError("constraint_id must be a non-empty string")
        constraint = self._constraints.get(constraint_id)
        return copy.deepcopy(constraint) if constraint is not None else None

    def list_constraints(self, active_only: bool = False) -> List[AirspaceConstraint]:
        if not isinstance(active_only, bool):
            raise ValueError("active_only must be a boolean")
        constraints = [
            constraint
            for constraint in self._constraints.values()
            if constraint.active or not active_only
        ]
        return [copy.deepcopy(constraint) for constraint in sorted(constraints, key=lambda item: item.constraint_id)]

    def validate_constraint(self, constraint: AirspaceConstraint) -> AirspaceConstraint:
        if not isinstance(constraint, AirspaceConstraint):
            raise ValueError("constraint must be an AirspaceConstraint")
        constraint.validate()
        self.validate_constraint_type(constraint.constraint_type)
        self.ensure_json_safe_dict(constraint.geometry, "constraint.geometry")
        self._validate_altitude_bounds(constraint)
        if constraint.constraint_type != "altitude_band" or constraint.geometry:
            self._validate_bbox_geometry(constraint.geometry)
        return copy.deepcopy(constraint)

    @staticmethod
    def validate_constraint_type(constraint_type: str) -> str:
        if not isinstance(constraint_type, str) or not constraint_type.strip():
            raise ValueError("invalid_constraint_type: unsupported airspace constraint type")
        constraint_type = constraint_type.strip()
        if constraint_type not in ALLOWED_AIRSPACE_CONSTRAINT_TYPES:
            raise ValueError("invalid_constraint_type: unsupported airspace constraint type")
        return constraint_type

    def validate_waypoint(self, waypoint: Dict[str, object]) -> Dict[str, object]:
        checked_waypoint = self.validate_waypoint_shape(waypoint)
        violations: List[str] = []
        warnings: List[str] = []
        checked_constraints: List[str] = []

        for constraint in self.list_constraints(active_only=True):
            checked_constraints.append(constraint.constraint_id)
            constraint_type = constraint.constraint_type
            if constraint_type == "geofence" and not self.point_in_bbox(checked_waypoint, constraint.geometry):
                violations.append(constraint.constraint_id)
            elif constraint_type in {"no_fly_zone", "restricted_zone"}:
                if self.point_in_bbox(checked_waypoint, constraint.geometry):
                    violations.append(constraint.constraint_id)
            elif constraint_type == "altitude_band":
                altitude = self.waypoint_altitude(checked_waypoint)
                has_bounds = constraint.altitude_min is not None or constraint.altitude_max is not None
                if altitude is None and has_bounds:
                    warnings.append(constraint.constraint_id)
                elif altitude is not None:
                    if constraint.altitude_min is not None and altitude < float(constraint.altitude_min):
                        violations.append(constraint.constraint_id)
                    elif constraint.altitude_max is not None and altitude > float(constraint.altitude_max):
                        violations.append(constraint.constraint_id)
            elif constraint_type == "corridor" and not self.point_in_bbox(checked_waypoint, constraint.geometry):
                warnings.append(constraint.constraint_id)

        result = {
            "verdict": self._aggregate_verdict(violations, warnings),
            "violations": sorted(set(violations)),
            "warnings": sorted(set(warnings)),
            "checked_constraints": sorted(set(checked_constraints)),
            "metadata": {"waypoint": copy.deepcopy(checked_waypoint)},
        }
        self.ensure_json_safe_dict(result, "waypoint_result")
        return result

    def validate_route(self, waypoints: List[Dict[str, object]]) -> Dict[str, object]:
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError("invalid_route: route must contain at least one waypoint")
        waypoint_results = [self.validate_waypoint(waypoint) for waypoint in waypoints]
        violations: List[str] = []
        warnings: List[str] = []
        checked_constraints: List[str] = []
        for result in waypoint_results:
            violations.extend(result["violations"])
            warnings.extend(result["warnings"])
            checked_constraints.extend(result["checked_constraints"])
        route_result = {
            "verdict": self._aggregate_verdict(violations, warnings),
            "waypoint_results": waypoint_results,
            "violations": sorted(set(violations)),
            "warnings": sorted(set(warnings)),
            "checked_constraints": sorted(set(checked_constraints)),
        }
        self.ensure_json_safe_dict(route_result, "route_result")
        return route_result

    def constraint_verdict(self, waypoints: List[Dict[str, object]]) -> str:
        verdict = self.validate_route(waypoints)["verdict"]
        if verdict not in ALLOWED_CONSTRAINT_VERDICTS:
            raise ValueError("constraint verdict must be valid, warning, or blocked")
        return str(verdict)

    @staticmethod
    def validate_waypoint_shape(waypoint: Dict[str, object]) -> Dict[str, object]:
        if not isinstance(waypoint, dict):
            raise ValueError("invalid_waypoint: waypoint requires numeric x and y")
        UTMAirspaceLayer.ensure_json_safe_dict(waypoint, "waypoint")
        if not UTMAirspaceLayer._is_finite_number(waypoint.get("x")):
            raise ValueError("invalid_waypoint: waypoint requires numeric x and y")
        if not UTMAirspaceLayer._is_finite_number(waypoint.get("y")):
            raise ValueError("invalid_waypoint: waypoint requires numeric x and y")
        altitude = UTMAirspaceLayer.waypoint_altitude(waypoint)
        if altitude is not None and not math.isfinite(altitude):
            raise ValueError("invalid_waypoint: waypoint altitude must be finite when provided")
        return copy.deepcopy(waypoint)

    @staticmethod
    def point_in_bbox(waypoint: Dict[str, object], geometry: Dict[str, object]) -> bool:
        UTMAirspaceLayer._validate_bbox_geometry(geometry)
        checked_waypoint = UTMAirspaceLayer.validate_waypoint_shape(waypoint)
        x = float(checked_waypoint["x"])
        y = float(checked_waypoint["y"])
        return (
            float(geometry["min_x"]) <= x <= float(geometry["max_x"])
            and float(geometry["min_y"]) <= y <= float(geometry["max_y"])
        )

    @staticmethod
    def waypoint_altitude(waypoint: Dict[str, object]) -> Optional[float]:
        if "z" in waypoint:
            value = waypoint["z"]
        elif "altitude" in waypoint:
            value = waypoint["altitude"]
        else:
            return None
        if not UTMAirspaceLayer._is_finite_number(value):
            raise ValueError("invalid_waypoint: waypoint altitude must be finite when provided")
        return float(value)

    @staticmethod
    def ensure_json_safe_dict(value: Dict[str, object], name: str) -> Dict[str, object]:
        if not isinstance(value, dict):
            raise ValueError(f"{name} must be a JSON-safe dictionary")
        UTMAirspaceLayer._ensure_json_safe(value, name)
        return value

    @staticmethod
    def _validate_altitude_bounds(constraint: AirspaceConstraint) -> None:
        if constraint.altitude_min is not None and constraint.altitude_max is not None:
            if float(constraint.altitude_min) > float(constraint.altitude_max):
                raise ValueError("invalid_altitude_bounds: altitude_min must be <= altitude_max")

    @staticmethod
    def _validate_bbox_geometry(geometry: Dict[str, object]) -> None:
        if not isinstance(geometry, dict) or geometry.get("type") != "bbox":
            raise ValueError("invalid_constraint_geometry: bbox geometry requires min_x, max_x, min_y, max_y")
        for key in _BBOX_KEYS:
            if not UTMAirspaceLayer._is_finite_number(geometry.get(key)):
                raise ValueError("invalid_constraint_geometry: bbox geometry requires min_x, max_x, min_y, max_y")
        if float(geometry["min_x"]) > float(geometry["max_x"]):
            raise ValueError("invalid_constraint_geometry: bbox min values must be <= max values")
        if float(geometry["min_y"]) > float(geometry["max_y"]):
            raise ValueError("invalid_constraint_geometry: bbox min values must be <= max values")

    @staticmethod
    def _aggregate_verdict(violations: List[str], warnings: List[str]) -> str:
        if violations:
            return "blocked"
        if warnings:
            return "warning"
        return "valid"

    @staticmethod
    def _is_finite_number(value: object) -> bool:
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(float(value))

    @staticmethod
    def _ensure_json_safe(value: object, name: str) -> None:
        if value is None or isinstance(value, (str, bool)):
            return
        if isinstance(value, int) and not isinstance(value, bool):
            return
        if isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError(f"{name} contains a non-finite float")
            return
        if isinstance(value, list):
            for index, item in enumerate(value):
                UTMAirspaceLayer._ensure_json_safe(item, f"{name}[{index}]")
            return
        if isinstance(value, dict):
            for key, item in value.items():
                if not isinstance(key, str):
                    raise ValueError(f"{name} keys must be strings")
                UTMAirspaceLayer._ensure_json_safe(item, f"{name}.{key}")
            return
        raise ValueError(f"{name} must be JSON-safe")
