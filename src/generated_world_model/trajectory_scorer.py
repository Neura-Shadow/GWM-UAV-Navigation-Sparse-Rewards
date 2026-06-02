"""Deterministic trajectory scoring for generated future rollouts."""

from __future__ import annotations

from typing import Any, Dict, Mapping, Optional

import numpy as np
import torch

from src.generated_world_model.types import GeneratedRollout, TrajectoryCandidate, TrajectoryScore


DEFAULT_SCORE_WEIGHTS: Dict[str, float] = {
    "goal_progress": 1.0,
    "collision_risk": 8.0,
    "uncertainty": 2.0,
    "energy": 0.05,
    "smoothness": 0.1,
    "altitude_violation": 10.0,
    "geofence_violation": 10.0,
}


class TrajectoryScorer:
    """Score candidate trajectories using generated observations and safety context."""

    def __init__(self, weights: Optional[Mapping[str, float]] = None) -> None:
        self.weights = dict(DEFAULT_SCORE_WEIGHTS)
        if weights is not None:
            self.weights.update({key: float(value) for key, value in weights.items()})

    def score(
        self,
        generated_rollout: GeneratedRollout | Dict[str, Any],
        candidate_trajectory: TrajectoryCandidate | Any,
        goal: Any,
        safety_context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Return a deterministic score dictionary for one candidate trajectory."""
        rollout = _coerce_rollout(generated_rollout)
        trajectory = _positions_from_candidate(candidate_trajectory)
        goal_vec = _vector3(goal)
        safety = safety_context or {}

        goal_progress = _goal_progress(trajectory, goal_vec)
        collision_risk = _depth_collision_risk(
            rollout.predicted_depth,
            min_safe_depth=float(safety.get("min_safe_depth", 1.0)),
        )
        uncertainty = _tensor_mean(rollout.uncertainty)
        energy = _energy_cost(candidate_trajectory, trajectory)
        smoothness = _smoothness_cost(trajectory)
        altitude_violation = _altitude_violation(
            trajectory,
            safety.get("altitude_bounds", (0.5, 120.0)),
        )
        geofence_violation = _geofence_violation(trajectory, safety.get("geofence"))

        components = {
            "goal_progress": goal_progress,
            "collision_risk": collision_risk,
            "uncertainty": uncertainty,
            "energy": energy,
            "smoothness": smoothness,
            "altitude_violation": altitude_violation,
            "geofence_violation": geofence_violation,
        }
        total = (
            self.weights["goal_progress"] * goal_progress
            - self.weights["collision_risk"] * collision_risk
            - self.weights["uncertainty"] * uncertainty
            - self.weights["energy"] * energy
            - self.weights["smoothness"] * smoothness
            - self.weights["altitude_violation"] * altitude_violation
            - self.weights["geofence_violation"] * geofence_violation
        )
        return TrajectoryScore(
            total_score=float(total),
            components=components,
            metadata={"weights": dict(self.weights), "horizon": int(len(trajectory))},
        ).to_dict()


def _coerce_rollout(value: GeneratedRollout | Dict[str, Any]) -> GeneratedRollout:
    if isinstance(value, GeneratedRollout):
        return value
    return GeneratedRollout(
        predicted_rgb=value["predicted_rgb"],
        predicted_depth=value["predicted_depth"],
        predicted_latent=value["predicted_latent"],
        uncertainty=value["uncertainty"],
        metadata=dict(value.get("metadata", {})),
    )


def _positions_from_candidate(candidate: TrajectoryCandidate | Any) -> np.ndarray:
    if isinstance(candidate, TrajectoryCandidate):
        value = candidate.positions
    elif isinstance(candidate, dict):
        value = candidate["positions"]
    else:
        value = candidate
    array = np.asarray(_detach(value), dtype=np.float64)
    if array.ndim != 2 or array.shape[1] < 3:
        raise ValueError("candidate trajectory positions must have shape [H, 3].")
    return array[:, :3]


def _candidate_actions(candidate: TrajectoryCandidate | Any) -> Any:
    if isinstance(candidate, TrajectoryCandidate):
        return candidate.actions
    if isinstance(candidate, dict):
        return candidate.get("actions")
    return None


def _goal_progress(trajectory: np.ndarray, goal: np.ndarray) -> float:
    start_dist = float(np.linalg.norm(trajectory[0] - goal))
    end_dist = float(np.linalg.norm(trajectory[-1] - goal))
    return start_dist - end_dist


def _depth_collision_risk(depth: torch.Tensor, min_safe_depth: float) -> float:
    depth_tensor = depth.detach().float()
    if min_safe_depth <= 0.0:
        return 0.0
    risk = torch.clamp((min_safe_depth - depth_tensor) / min_safe_depth, min=0.0, max=1.0)
    return float(risk.mean().item())


def _energy_cost(candidate: TrajectoryCandidate | Any, trajectory: np.ndarray) -> float:
    actions = _candidate_actions(candidate)
    if actions is not None:
        action_array = np.asarray(_detach(actions), dtype=np.float64)
        if action_array.size:
            return float(np.linalg.norm(action_array.reshape(-1, action_array.shape[-1]), axis=1).mean())
    if len(trajectory) < 2:
        return 0.0
    deltas = np.diff(trajectory, axis=0)
    return float(np.linalg.norm(deltas, axis=1).mean())


def _smoothness_cost(trajectory: np.ndarray) -> float:
    if len(trajectory) < 3:
        return 0.0
    second_diff = np.diff(trajectory, n=2, axis=0)
    return float(np.linalg.norm(second_diff, axis=1).mean())


def _altitude_violation(trajectory: np.ndarray, bounds: Any) -> float:
    low, high = float(bounds[0]), float(bounds[1])
    altitudes = np.abs(trajectory[:, 2])
    below = np.maximum(low - altitudes, 0.0)
    above = np.maximum(altitudes - high, 0.0)
    return float((below + above).mean())


def _geofence_violation(trajectory: np.ndarray, geofence: Any) -> float:
    if not geofence:
        return 0.0
    penalty = 0.0
    for axis_index, axis in enumerate(("x", "y", "z")):
        bounds = geofence.get(axis)
        if bounds is None:
            continue
        low, high = float(bounds[0]), float(bounds[1])
        values = trajectory[:, axis_index]
        penalty += float((np.maximum(low - values, 0.0) + np.maximum(values - high, 0.0)).mean())
    return penalty


def _tensor_mean(value: torch.Tensor) -> float:
    return float(value.detach().float().mean().item())


def _vector3(value: Any) -> np.ndarray:
    array = np.asarray(_detach(value), dtype=np.float64).reshape(-1)
    if array.shape[0] < 3:
        raise ValueError("goal must contain at least three coordinates.")
    return array[:3]


def _detach(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return value
