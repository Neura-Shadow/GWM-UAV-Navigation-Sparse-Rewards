"""Pure-Python Nav2-style costmap and planner skeletons."""

from __future__ import annotations

from typing import Any, List, Sequence, Tuple

import numpy as np


class WorldModelCostmapLayer:
    """Minimal costmap layer shape for future Nav2 integration."""

    def __init__(self, lethal_cost: int = 254, inflation_radius: float = 2.0) -> None:
        if lethal_cost < 0 or lethal_cost > 255:
            raise ValueError("lethal_cost must be in [0, 255].")
        if inflation_radius < 0.0:
            raise ValueError("inflation_radius must be non-negative.")
        self.lethal_cost = int(lethal_cost)
        self.inflation_radius = float(inflation_radius)

    def update_costs(
        self,
        grid: Any,
        position: Sequence[float],
        radius: float,
        risk_score: float = 1.0,
    ) -> Any:
        """Apply a circular cost update to a 2-D grid-like object."""
        if radius < 0.0:
            raise ValueError("radius must be non-negative.")
        array = np.asarray(grid)
        if array.ndim != 2:
            raise ValueError("costmap grid must be 2-D.")

        x_center = int(round(float(position[0])))
        y_center = int(round(float(position[1])))
        effective_radius = radius + self.inflation_radius
        cost = int(round(self.lethal_cost * _clamp01(risk_score)))

        for y in range(array.shape[0]):
            for x in range(array.shape[1]):
                distance = float(np.linalg.norm(np.asarray([x - x_center, y - y_center])))
                if distance <= effective_radius:
                    array[y, x] = max(int(array[y, x]), cost)
        return grid


class WorldModelPlannerPlugin:
    """Simple straight-line planner skeleton for Nav2-style integration tests."""

    def __init__(self, step_size: float = 1.0) -> None:
        if step_size <= 0.0:
            raise ValueError("step_size must be positive.")
        self.step_size = float(step_size)

    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        costmap: Any = None,
    ) -> List[Tuple[float, float, float]]:
        """Return a deterministic straight-line path from start to goal."""
        del costmap
        start_vec = np.asarray(_pad3(start), dtype=np.float64)
        goal_vec = np.asarray(_pad3(goal), dtype=np.float64)
        distance = float(np.linalg.norm(goal_vec - start_vec))
        if distance == 0.0:
            return [tuple(float(v) for v in start_vec)]

        steps = max(1, int(np.ceil(distance / self.step_size)))
        path = []
        for index in range(steps + 1):
            alpha = index / steps
            point = (1.0 - alpha) * start_vec + alpha * goal_vec
            path.append(tuple(float(v) for v in point))
        return path


def _pad3(value: Sequence[float]) -> Tuple[float, float, float]:
    padded = list(value) + [0.0, 0.0, 0.0]
    return (float(padded[0]), float(padded[1]), float(padded[2]))


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))
