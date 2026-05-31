"""Agent state registry for multi-agent coordination.

Provides a central registry to track, query, and update the state of all
agents in the fleet.  Thread-safety is *not* guaranteed — callers in
concurrent settings should wrap mutations in their own locks.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Optional

from src.utils.data_types import AgentState, AgentStatus, VehicleType

logger = logging.getLogger(__name__)


class AgentRegistry:
    """Manages a collection of agents in a multi-agent system."""

    def __init__(self) -> None:
        self._agents: Dict[str, AgentState] = {}
        logger.info("AgentRegistry initialised.")

    # ------------------------------------------------------------------
    # CRUD operations
    # ------------------------------------------------------------------

    def register(self, agent: AgentState) -> None:
        """Register an agent.  Overwrites if the ID already exists."""
        self._agents[agent.agent_id] = agent
        logger.info("Registered agent '%s' (type=%s).", agent.agent_id, agent.vehicle_type.value)

    def unregister(self, agent_id: str) -> None:
        """Remove an agent from the registry.

        Raises:
            KeyError: If the agent is not registered.
        """
        if agent_id not in self._agents:
            raise KeyError(f"Agent '{agent_id}' is not registered.")
        del self._agents[agent_id]
        logger.info("Unregistered agent '%s'.", agent_id)

    def get(self, agent_id: str) -> Optional[AgentState]:
        """Return the agent with *agent_id*, or *None* if not found."""
        return self._agents.get(agent_id)

    def get_all(self) -> List[AgentState]:
        """Return a list of all registered agents."""
        return list(self._agents.values())

    # ------------------------------------------------------------------
    # Filtered queries
    # ------------------------------------------------------------------

    def get_by_type(self, vehicle_type: VehicleType) -> List[AgentState]:
        """Return all agents of a given vehicle type."""
        return [a for a in self._agents.values() if a.vehicle_type == vehicle_type]

    def get_by_status(self, status: AgentStatus) -> List[AgentState]:
        """Return all agents with a given status."""
        return [a for a in self._agents.values() if a.status == status]

    # ------------------------------------------------------------------
    # Updates
    # ------------------------------------------------------------------

    def update_state(self, agent_id: str, **kwargs: object) -> None:
        """Update fields on an existing agent.

        Only fields that exist on ``AgentState`` are applied; unknown keys
        are silently ignored.

        Raises:
            KeyError: If the agent is not registered.
        """
        agent = self._agents.get(agent_id)
        if agent is None:
            raise KeyError(f"Agent '{agent_id}' is not registered.")

        for key, value in kwargs.items():
            if hasattr(agent, key):
                setattr(agent, key, value)
            else:
                logger.warning(
                    "Ignoring unknown field '%s' on AgentState for agent '%s'.",
                    key,
                    agent_id,
                )

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._agents)

    def __contains__(self, agent_id: str) -> bool:
        return agent_id in self._agents

    def __repr__(self) -> str:
        return f"AgentRegistry(n_agents={len(self._agents)})"
