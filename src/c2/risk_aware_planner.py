"""Mock-first risk-aware route candidate scoring for GWM-UAV-C2."""

from __future__ import annotations

import copy
import math
from typing import Dict, List, Optional, Sequence

from src.c2.airspace import UTMAirspaceLayer
from src.c2.event_bus import MissionEventBus
from src.c2.mission_types import FleetAsset, MissionEvent, MissionTask, PlannedRoute, ThreatAssessment
from src.c2.state_store import MissionStateStore


# Mock-first research scoring constants only; these are not certified safety values.
RISK_WEIGHT = 100.0
WARNING_CONSTRAINT_PENALTY = 50.0
BLOCKED_CONSTRAINT_PENALTY = 1000.0
SAFE_OFFSET_DISTANCE = 10.0
ALLOWED_ROUTE_VERDICTS = ("valid", "warning", "blocked")


class RiskAwarePlanner:
    """Deterministic route candidate generator and scorer."""

    def __init__(
        self,
        airspace_layer: Optional[UTMAirspaceLayer] = None,
        event_bus: Optional[MissionEventBus] = None,
        state_store: Optional[MissionStateStore] = None,
    ) -> None:
        if airspace_layer is not None and not isinstance(airspace_layer, UTMAirspaceLayer):
            raise ValueError("airspace_layer must be a UTMAirspaceLayer")
        if event_bus is not None and not isinstance(event_bus, MissionEventBus):
            raise ValueError("event_bus must be a MissionEventBus")
        if state_store is not None and not isinstance(state_store, MissionStateStore):
            raise ValueError("state_store must be a MissionStateStore")
        self.airspace_layer = airspace_layer
        self.event_bus = event_bus or MissionEventBus()
        self.state_store = state_store
        self._route_counter = 0
        self._event_counter = 0

    def generate_candidate_routes(
        self,
        task: MissionTask,
        asset: FleetAsset,
        start: Dict[str, object],
        goal: Dict[str, object],
        context: Optional[Dict[str, object]] = None,
    ) -> List[PlannedRoute]:
        if not isinstance(task, MissionTask):
            raise ValueError("task must be a MissionTask")
        if not isinstance(asset, FleetAsset):
            raise ValueError("asset must be a FleetAsset")
        task.validate()
        asset.validate()
        if context is not None and not isinstance(context, dict):
            raise ValueError("context must be a dictionary when provided")

        start_waypoint = self.validate_waypoint(start)
        goal_waypoint = self.validate_waypoint(goal)
        risk_assessment = self._context_risk_assessment(context or {})
        candidates = [
            ("direct", [start_waypoint, goal_waypoint]),
            ("midpoint", [start_waypoint, self.midpoint_waypoint(start_waypoint, goal_waypoint), goal_waypoint]),
            (
                "safe_offset",
                [
                    start_waypoint,
                    self.safe_offset_waypoint(start_waypoint, goal_waypoint),
                    goal_waypoint,
                ],
            ),
        ]

        routes: List[PlannedRoute] = []
        for candidate_type, waypoints in candidates:
            route_result = self.airspace_layer.validate_route(waypoints) if self.airspace_layer else {"verdict": "valid"}
            constraint_verdict = self.validate_route_verdict(str(route_result["verdict"]))
            score_breakdown = self.score_route(
                waypoints,
                risk_assessment=risk_assessment,
                constraint_verdict=constraint_verdict,
            )
            routes.append(
                self.create_planned_route(
                    task_id=task.task_id,
                    waypoints=waypoints,
                    score=float(score_breakdown["score"]),
                    risk_score=float(score_breakdown["risk_score"]),
                    constraint_verdict=constraint_verdict,
                    metadata={
                        "asset_id": asset.asset_id,
                        "candidate_type": candidate_type,
                        "score_breakdown": score_breakdown,
                    },
                )
            )
        return routes

    def score_route(
        self,
        waypoints: List[Dict[str, object]],
        risk_assessment: Optional[ThreatAssessment] = None,
        constraint_verdict: Optional[str] = None,
    ) -> Dict[str, object]:
        checked_waypoints = self.validate_waypoints(waypoints)
        verdict = self.validate_route_verdict(constraint_verdict or "valid")
        risk_score = self.risk_score_from_assessment(risk_assessment)
        distance_cost = self.route_distance(checked_waypoints)
        risk_penalty = risk_score * RISK_WEIGHT
        constraint_penalty = self.constraint_penalty(verdict)
        result = {
            "distance_cost": distance_cost,
            "risk_score": risk_score,
            "risk_penalty": risk_penalty,
            "constraint_verdict": verdict,
            "constraint_penalty": constraint_penalty,
            "score": distance_cost + risk_penalty + constraint_penalty,
        }
        self.ensure_json_safe_dict(result, "score_breakdown")
        return result

    def select_route(self, routes: List[PlannedRoute]) -> PlannedRoute:
        if not isinstance(routes, list) or not routes:
            raise ValueError("no_routes_available: route list is empty")
        for index, route in enumerate(routes):
            if not isinstance(route, PlannedRoute):
                raise ValueError(f"routes[{index}] must be a PlannedRoute")
            route.validate()
        unblocked = [route for route in routes if route.constraint_verdict != "blocked"]
        candidates = unblocked if unblocked else routes
        selected = min(candidates, key=lambda route: (float(route.score), route.route_id))
        selected.metadata = copy.deepcopy(selected.metadata)
        selected.metadata["selected"] = True
        selected.metadata["executable"] = False
        selected.metadata["selection_role"] = "recommendation_only"
        selected.validate()
        return copy.deepcopy(selected)

    def make_planned_route_event(self, route: PlannedRoute) -> MissionEvent:
        if not isinstance(route, PlannedRoute):
            raise ValueError("route must be a PlannedRoute")
        route.validate()
        payload = route.to_dict()
        self._event_counter += 1
        return MissionEvent(
            event_id=f"route-event-{self._event_counter:06d}",
            event_type="route.planned",
            timestamp=float(self._event_counter),
            source="risk_aware_planner",
            payload=payload,
            metadata=self._route_event_metadata(payload),
        )

    def publish_planned_route(self, route: PlannedRoute) -> MissionEvent:
        event = self.make_planned_route_event(route)
        published = self.event_bus.publish(event)
        if self.state_store is not None:
            self.state_store.apply_event(event)
        return published

    def create_planned_route(
        self,
        task_id: str,
        waypoints: List[Dict[str, object]],
        score: float,
        risk_score: float,
        constraint_verdict: str,
        metadata: Optional[Dict[str, object]] = None,
    ) -> PlannedRoute:
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValueError("invalid_task_id: task_id is required")
        checked_waypoints = self.validate_waypoints(waypoints)
        if not self._is_finite_number(score) or float(score) < 0.0:
            raise ValueError("invalid_score: score must be non-negative")
        checked_risk_score = self._bounded_risk_score(risk_score)
        verdict = self.validate_route_verdict(constraint_verdict)
        route_metadata = copy.deepcopy(metadata or {})
        self.ensure_json_safe_dict(route_metadata, "route.metadata")
        self._route_counter += 1
        return PlannedRoute(
            route_id=f"route-{self._route_counter:06d}",
            task_id=task_id.strip(),
            waypoints=checked_waypoints,
            score=float(score),
            risk_score=checked_risk_score,
            constraint_verdict=verdict,
            metadata=route_metadata,
        )

    def route_distance(self, waypoints: List[Dict[str, object]]) -> float:
        checked_waypoints = self.validate_waypoints(waypoints)
        total = 0.0
        for start, goal in zip(checked_waypoints, checked_waypoints[1:]):
            dx = float(goal["x"]) - float(start["x"])
            dy = float(goal["y"]) - float(start["y"])
            start_z = self._waypoint_z(start)
            goal_z = self._waypoint_z(goal)
            dz = 0.0 if start_z is None or goal_z is None else goal_z - start_z
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    def risk_score_from_assessment(self, risk_assessment: Optional[ThreatAssessment]) -> float:
        if risk_assessment is None:
            return 0.0
        if not isinstance(risk_assessment, ThreatAssessment):
            raise ValueError("risk_assessment must be a ThreatAssessment")
        risk_assessment.validate()
        return max(0.0, min(1.0, float(risk_assessment.total_risk)))

    def constraint_penalty(self, constraint_verdict: str) -> float:
        verdict = self.validate_route_verdict(constraint_verdict)
        if verdict == "warning":
            return WARNING_CONSTRAINT_PENALTY
        if verdict == "blocked":
            return BLOCKED_CONSTRAINT_PENALTY
        return 0.0

    def validate_waypoint(self, waypoint: Dict[str, object]) -> Dict[str, object]:
        if not isinstance(waypoint, dict):
            raise ValueError("invalid_waypoint: waypoint requires numeric x and y")
        self.ensure_json_safe_dict(waypoint, "waypoint")
        if not self._is_finite_number(waypoint.get("x")) or not self._is_finite_number(waypoint.get("y")):
            raise ValueError("invalid_waypoint: waypoint requires numeric x and y")
        z_value = self._waypoint_z(waypoint)
        if z_value is not None and not math.isfinite(z_value):
            raise ValueError("invalid_waypoint: waypoint altitude must be finite when provided")
        return copy.deepcopy(waypoint)

    def validate_waypoints(self, waypoints: List[Dict[str, object]]) -> List[Dict[str, object]]:
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ValueError("invalid_route: route must contain at least two waypoints")
        return [self.validate_waypoint(waypoint) for waypoint in waypoints]

    @staticmethod
    def validate_route_verdict(constraint_verdict: str) -> str:
        if not isinstance(constraint_verdict, str) or constraint_verdict not in ALLOWED_ROUTE_VERDICTS:
            raise ValueError("invalid_route_verdict: constraint verdict must be valid, warning, or blocked")
        return constraint_verdict

    def midpoint_waypoint(self, start: Dict[str, object], goal: Dict[str, object]) -> Dict[str, object]:
        start_waypoint = self.validate_waypoint(start)
        goal_waypoint = self.validate_waypoint(goal)
        midpoint: Dict[str, object] = {
            "x": (float(start_waypoint["x"]) + float(goal_waypoint["x"])) / 2.0,
            "y": (float(start_waypoint["y"]) + float(goal_waypoint["y"])) / 2.0,
        }
        if "z" in start_waypoint and "z" in goal_waypoint:
            midpoint["z"] = (float(start_waypoint["z"]) + float(goal_waypoint["z"])) / 2.0
        elif "z" not in start_waypoint and "z" not in goal_waypoint:
            if "altitude" in start_waypoint and "altitude" in goal_waypoint:
                midpoint["altitude"] = (float(start_waypoint["altitude"]) + float(goal_waypoint["altitude"])) / 2.0
        return midpoint

    def safe_offset_waypoint(self, start: Dict[str, object], goal: Dict[str, object]) -> Dict[str, object]:
        waypoint = self.midpoint_waypoint(start, goal)
        waypoint["y"] = float(waypoint["y"]) + SAFE_OFFSET_DISTANCE
        return waypoint

    @staticmethod
    def ensure_json_safe_dict(value: Dict[str, object], name: str) -> Dict[str, object]:
        if not isinstance(value, dict):
            raise ValueError(f"{name} must be a JSON-safe dictionary")
        RiskAwarePlanner._ensure_json_safe(value, name)
        return value

    @staticmethod
    def _route_event_metadata(payload: Dict[str, object]) -> Dict[str, object]:
        route_metadata = payload.get("metadata", {})
        selected = False
        if isinstance(route_metadata, dict):
            selected = route_metadata.get("selected") is True
        metadata = {
            "source": "risk_aware_planner",
            "task_id": payload.get("task_id", ""),
            "route_id": payload.get("route_id", ""),
            "score": payload.get("score", 0.0),
            "risk_score": payload.get("risk_score", 0.0),
            "constraint_verdict": payload.get("constraint_verdict", ""),
            "selected": selected,
        }
        RiskAwarePlanner.ensure_json_safe_dict(metadata, "event.metadata")
        return metadata

    @staticmethod
    def _context_risk_assessment(context: Dict[str, object]) -> Optional[ThreatAssessment]:
        risk_assessment = context.get("risk_assessment")
        if risk_assessment is None:
            return None
        if isinstance(risk_assessment, ThreatAssessment):
            return copy.deepcopy(risk_assessment)
        if isinstance(risk_assessment, dict):
            return ThreatAssessment.from_dict(risk_assessment)
        raise ValueError("context.risk_assessment must be a ThreatAssessment or dictionary")

    @staticmethod
    def _bounded_risk_score(value: object) -> float:
        if not RiskAwarePlanner._is_finite_number(value):
            raise ValueError("invalid_risk_score: risk score must be in [0.0, 1.0]")
        risk_score = float(value)
        if risk_score < 0.0 or risk_score > 1.0:
            raise ValueError("invalid_risk_score: risk score must be in [0.0, 1.0]")
        return risk_score

    @staticmethod
    def _waypoint_z(waypoint: Dict[str, object]) -> Optional[float]:
        if "z" in waypoint:
            value = waypoint["z"]
        elif "altitude" in waypoint:
            value = waypoint["altitude"]
        else:
            return None
        if not RiskAwarePlanner._is_finite_number(value):
            raise ValueError("invalid_waypoint: waypoint altitude must be finite when provided")
        return float(value)

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
                RiskAwarePlanner._ensure_json_safe(item, f"{name}[{index}]")
            return
        if isinstance(value, dict):
            for key, item in value.items():
                if not isinstance(key, str):
                    raise ValueError(f"{name} keys must be strings")
                RiskAwarePlanner._ensure_json_safe(item, f"{name}.{key}")
            return
        raise ValueError(f"{name} must be JSON-safe")


def route_distance(waypoints: Sequence[Dict[str, object]]) -> float:
    return RiskAwarePlanner().route_distance(list(waypoints))
