"""Simple candidate trajectory sampler for generated-world-model planning."""

from __future__ import annotations

from typing import List, Sequence

import numpy as np

from src.generated_world_model.types import TrajectoryCandidate


class CandidateTrajectorySampler:
    """Sample deterministic straight-line-biased UAV trajectory candidates."""

    def __init__(self, horizon: int = 6, dt: float = 0.4, seed: int | None = 13) -> None:
        if horizon <= 0:
            raise ValueError("horizon must be positive.")
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.seed = seed

    def sample(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        num_candidates: int = 8,
        max_speed: float = 4.0,
    ) -> List[TrajectoryCandidate]:
        """Return candidate trajectories biased toward the goal."""
        if num_candidates <= 0:
            raise ValueError("num_candidates must be positive.")
        rng = np.random.default_rng(self.seed)
        start_vec = _vec3(start)
        goal_vec = _vec3(goal)
        direction = goal_vec - start_vec
        norm = float(np.linalg.norm(direction))
        unit = direction / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0])

        candidates: List[TrajectoryCandidate] = []
        for index in range(num_candidates):
            noise_scale = 0.0 if index == 0 else 0.25
            velocity = unit * max_speed + rng.normal(0.0, noise_scale, size=3)
            positions = []
            actions = []
            current = start_vec.copy()
            for _ in range(self.horizon):
                action = np.clip(velocity, -max_speed, max_speed)
                current = current + action * self.dt
                positions.append(current.copy())
                actions.append(action.copy())
            candidates.append(
                TrajectoryCandidate(
                    positions=np.asarray(positions, dtype=np.float32),
                    actions=np.asarray(actions, dtype=np.float32),
                    metadata={"candidate_index": index},
                )
            )
        return candidates


def _vec3(value: Sequence[float]) -> np.ndarray:
    padded = list(value) + [0.0, 0.0, 0.0]
    return np.asarray(padded[:3], dtype=np.float64)
