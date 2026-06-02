"""Priority-based task allocation for multi-agent coordination."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from src.multi_agent.agent_state import AgentRegistry
from src.utils.data_types import AgentState, AgentStatus

logger = logging.getLogger(__name__)


DEFAULT_PRIORITY_WEIGHTS: Dict[str, float] = {
    "battery_weight": 1.0,
    "distance_weight": 1.0,
    "capability_weight": 2.0,
    "load_weight": 1.0,
}


@dataclass(frozen=True)
class PriorityAssignment:
    """A computed task assignment candidate."""

    agent_id: str
    task_id: str
    task: Dict[str, Any]
    score: float


class PriorityCoordinator:
    """Scores idle agents and assigns tasks deterministically.

    This coordinator is intentionally policy-light.  It computes a stable
    assignment plan from observable ``AgentState`` fields and metadata, but
    does not mutate the registry.  ``SwarmCoordinator`` remains responsible
    for applying assignments and sending task messages.
    """

    def __init__(
        self,
        registry: AgentRegistry,
        weights: Optional[Dict[str, float]] = None,
    ) -> None:
        self.registry = registry
        self.weights = dict(DEFAULT_PRIORITY_WEIGHTS)
        if weights:
            self.weights.update(weights)

    def assign_tasks(self, tasks: List[Dict[str, Any]]) -> Dict[str, str]:
        """Return a computed ``{agent_id: task_id}`` assignment plan."""
        return {
            assignment.agent_id: assignment.task_id
            for assignment in self.plan_assignments(tasks)
        }

    def plan_assignments(
        self,
        tasks: List[Dict[str, Any]],
    ) -> List[PriorityAssignment]:
        """Return ordered task assignments without mutating agent state."""
        available = sorted(
            self.registry.get_by_status(AgentStatus.IDLE),
            key=lambda agent: agent.agent_id,
        )
        assignments: List[PriorityAssignment] = []

        for index, task in enumerate(tasks):
            if not available:
                logger.warning(
                    "No idle agents available for %d remaining priority task(s).",
                    len(tasks) - index,
                )
                break

            ranked = sorted(
                available,
                key=lambda agent: (-self.score_agent(agent, task), agent.agent_id),
            )
            best_agent = ranked[0]
            task_id = str(task.get("task_id", f"task_{index}"))
            score = self.score_agent(best_agent, task)
            assignments.append(PriorityAssignment(
                agent_id=best_agent.agent_id,
                task_id=task_id,
                task=task,
                score=score,
            ))
            available = [a for a in available if a.agent_id != best_agent.agent_id]

        return assignments

    def score_agent(self, agent: AgentState, task: Dict[str, Any]) -> float:
        """Compute a higher-is-better priority score for one agent and task."""
        if agent.status != AgentStatus.IDLE:
            return -float("inf")

        battery = _clamp01(_metadata_number(agent, "battery_level", 1.0))
        distance_score = self._distance_score(agent, task)
        capability_score = self._capability_score(agent, task)
        load_score = 1.0 - _clamp01(_metadata_number(agent, "current_load", 0.0))

        return (
            self.weights["battery_weight"] * battery
            + self.weights["distance_weight"] * distance_score
            + self.weights["capability_weight"] * capability_score
            + self.weights["load_weight"] * load_score
        )

    def _distance_score(self, agent: AgentState, task: Dict[str, Any]) -> float:
        target = _task_position(task)
        if target is None or agent.observation is None:
            return 1.0

        agent_position = np.asarray(agent.observation.pose, dtype=np.float64)
        target_position = np.asarray(target, dtype=np.float64)
        distance = float(np.linalg.norm(agent_position - target_position))
        return 1.0 / (1.0 + max(distance, 0.0))

    def _capability_score(self, agent: AgentState, task: Dict[str, Any]) -> float:
        required = task.get("required_capability")
        if required is None:
            return 1.0

        capabilities = agent.metadata.get("capabilities", [])
        if isinstance(capabilities, str):
            capabilities = [capabilities]
        return 1.0 if required in capabilities else 0.0


def _metadata_number(agent: AgentState, key: str, default: float) -> float:
    value = agent.metadata.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def _task_position(task: Dict[str, Any]) -> Optional[Tuple[float, float, float]]:
    for key in ("goal", "position", "target_position"):
        value = task.get(key)
        if _is_position(value):
            return (float(value[0]), float(value[1]), float(value[2]))
    return None


def _is_position(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(value, str) and len(value) >= 3
