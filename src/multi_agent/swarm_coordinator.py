"""Swarm coordinator for multi-agent task allocation and collision avoidance.

Ties together the ``AgentRegistry``, ``SharedSpatiotemporalMap``, and
``CommunicationChannel`` to provide fleet-level coordination:  round-robin
task assignment, pairwise conflict detection, and basic avoidance command
generation.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List

import numpy as np

from src.multi_agent.agent_state import AgentRegistry
from src.multi_agent.communication import (
    AgentMessage,
    CommunicationChannel,
    MessageType,
)
from src.multi_agent.shared_map import SharedSpatiotemporalMap
from src.utils.data_types import AgentStatus, ControlCommand

logger = logging.getLogger(__name__)


class SwarmCoordinator:
    """Coordinates multiple agents for task allocation and collision avoidance."""

    def __init__(
        self,
        registry: AgentRegistry,
        shared_map: SharedSpatiotemporalMap,
        channel: CommunicationChannel,
        safety_radius: float = 5.0,
    ) -> None:
        self.registry = registry
        self.shared_map = shared_map
        self.channel = channel
        self.safety_radius = safety_radius
        logger.info(
            "SwarmCoordinator initialised (safety_radius=%.1f).", safety_radius,
        )

    # ------------------------------------------------------------------
    # Task allocation
    # ------------------------------------------------------------------

    def assign_tasks(self, tasks: List[Dict[str, Any]]) -> Dict[str, str]:
        """Assign tasks to available (IDLE) agents using round-robin.

        Args:
            tasks: List of task dicts, each must contain ``"task_id"``.

        Returns:
            Mapping ``{agent_id: task_id}`` for all assignments made.
        """
        idle_agents = self.registry.get_by_status(AgentStatus.IDLE)
        assignments: Dict[str, str] = {}

        for i, task in enumerate(tasks):
            if not idle_agents:
                logger.warning("No more idle agents — %d task(s) unassigned.", len(tasks) - i)
                break

            agent = idle_agents.pop(0)
            task_id = task.get("task_id", f"task_{i}")
            self.registry.update_state(
                agent.agent_id,
                task_id=task_id,
                status=AgentStatus.NAVIGATING,
            )
            assignments[agent.agent_id] = task_id

            # Notify agent via communication channel
            msg = AgentMessage(
                sender_id="coordinator",
                message_type=MessageType.TASK_ASSIGNMENT,
                payload={"task_id": task_id, "task": task},
                timestamp=time.time(),
                priority=5,
            )
            self.channel.send(msg)

        logger.info(
            "Assigned %d task(s) to %d agent(s).",
            len(assignments),
            len(assignments),
        )
        return assignments

    # ------------------------------------------------------------------
    # Conflict detection
    # ------------------------------------------------------------------

    def check_conflicts(self) -> List[Dict[str, Any]]:
        """Check for potential collisions between agents.

        Returns a list of conflict dicts, each with:
        - ``agents``: pair of agent IDs
        - ``distance``: distance between them
        - ``positions``: dict of each agent's position
        """
        positions = self.shared_map.get_agent_positions(max_age=5.0)
        agent_ids = list(positions.keys())
        conflicts: List[Dict[str, Any]] = []

        for i in range(len(agent_ids)):
            for j in range(i + 1, len(agent_ids)):
                aid_a = agent_ids[i]
                aid_b = agent_ids[j]
                pos_a = np.asarray(positions[aid_a])
                pos_b = np.asarray(positions[aid_b])
                dist = float(np.linalg.norm(pos_a - pos_b))

                if dist < self.safety_radius:
                    conflicts.append({
                        "agents": (aid_a, aid_b),
                        "distance": dist,
                        "positions": {aid_a: positions[aid_a], aid_b: positions[aid_b]},
                    })

        if conflicts:
            logger.warning("Detected %d conflict(s).", len(conflicts))
        return conflicts

    # ------------------------------------------------------------------
    # Conflict resolution
    # ------------------------------------------------------------------

    def resolve_conflicts(
        self, conflicts: List[Dict[str, Any]]
    ) -> List[ControlCommand]:
        """Generate avoidance commands for conflicting agents.

        Simple repulsion strategy: each agent in a conflict pair is pushed
        away from the other along the connecting line.
        """
        commands: List[ControlCommand] = []

        for conflict in conflicts:
            aid_a, aid_b = conflict["agents"]
            pos_a = np.asarray(conflict["positions"][aid_a])
            pos_b = np.asarray(conflict["positions"][aid_b])

            direction = pos_a - pos_b
            norm = float(np.linalg.norm(direction))
            if norm < 1e-6:
                direction = np.array([1.0, 0.0, 0.0])
            else:
                direction = direction / norm

            # Push each agent away at a fixed avoidance speed
            avoidance_speed = 2.0
            vel_a = (direction * avoidance_speed).tolist()
            vel_b = (-direction * avoidance_speed).tolist()

            commands.append(ControlCommand(
                vx=vel_a[0], vy=vel_a[1], vz=vel_a[2],
                metadata={"agent_id": aid_a, "reason": "conflict_avoidance"},
            ))
            commands.append(ControlCommand(
                vx=vel_b[0], vy=vel_b[1], vz=vel_b[2],
                metadata={"agent_id": aid_b, "reason": "conflict_avoidance"},
            ))

        logger.info("Generated %d avoidance command(s).", len(commands))
        return commands

    # ------------------------------------------------------------------
    # Broadcasting
    # ------------------------------------------------------------------

    def broadcast_states(self) -> None:
        """Have all agents broadcast their current state."""
        for agent in self.registry.get_all():
            obs = agent.observation
            pose = obs.pose if obs is not None else (0.0, 0.0, 0.0)
            msg = AgentMessage(
                sender_id=agent.agent_id,
                message_type=MessageType.STATE_BROADCAST,
                payload={
                    "agent_id": agent.agent_id,
                    "status": agent.status.value,
                    "position": pose,
                },
                timestamp=time.time(),
            )
            self.channel.send(msg)

        logger.debug("State broadcast completed for %d agent(s).", len(self.registry))
