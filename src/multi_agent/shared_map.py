"""Shared spatiotemporal map for multi-agent coordination.

Provides a lightweight 4-D (x, y, z, t) grid that all agents read and
write.  Stores agent positions and optional per-cell observation data
(occupancy, obstacle flags, etc.) with automatic expiration of old entries.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class _MapEntry:
    """Internal record for a single agent's last update."""
    agent_id: str
    position: Tuple[float, float, float]
    timestamp: float
    observation_data: Dict[str, Any] = field(default_factory=dict)


class SharedSpatiotemporalMap:
    """4-D spatiotemporal map shared across agents.

    Grid-based representation with (x, y, z, t) dimensions.
    Stores occupancy, agent positions, and obstacle information.
    """

    def __init__(
        self,
        bounds: Tuple[float, float, float, float, float, float] = (
            -100.0, -100.0, -50.0, 100.0, 100.0, 0.0,
        ),
        resolution: float = 1.0,
        time_window: float = 10.0,
    ) -> None:
        """
        Args:
            bounds: (x_min, y_min, z_min, x_max, y_max, z_max).
            resolution: Metres per grid cell.
            time_window: Default seconds of history to keep.
        """
        self.bounds = bounds
        self.resolution = resolution
        self.time_window = time_window

        # Agent-level tracking (agent_id → latest entry)
        self._agent_entries: Dict[str, _MapEntry] = {}

        # History of all entries for spatial queries
        self._history: List[_MapEntry] = []

        logger.info(
            "SharedSpatiotemporalMap initialised — bounds=%s, resolution=%.1f, "
            "time_window=%.1f.",
            bounds,
            resolution,
            time_window,
        )

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def update(
        self,
        agent_id: str,
        position: Tuple[float, float, float],
        timestamp: float,
        observation_data: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Update the map with an agent's observation.

        The agent's latest position is stored and a history record is
        appended for spatiotemporal queries.
        """
        entry = _MapEntry(
            agent_id=agent_id,
            position=position,
            timestamp=timestamp,
            observation_data=observation_data or {},
        )
        self._agent_entries[agent_id] = entry
        self._history.append(entry)
        logger.debug(
            "Map updated: agent=%s pos=%s t=%.2f.", agent_id, position, timestamp,
        )

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def query(
        self,
        position: Tuple[float, float, float],
        radius: float,
        time_window: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Query the map for information near a position.

        Returns a dict with:
        - ``nearby_agents``: list of agent IDs within *radius*.
        - ``entries``: list of history entries within radius and time window.
        - ``count``: number of matching entries.
        """
        tw = time_window if time_window is not None else self.time_window
        pos = np.asarray(position, dtype=np.float64)

        # Determine time cutoff from the most recent timestamp in history
        if self._history:
            latest_t = max(e.timestamp for e in self._history)
            t_cutoff = latest_t - tw
        else:
            t_cutoff = -float("inf")

        nearby_agents: List[str] = []
        matched_entries: List[Dict[str, Any]] = []

        for entry in self._history:
            if entry.timestamp < t_cutoff:
                continue
            dist = float(np.linalg.norm(np.asarray(entry.position) - pos))
            if dist <= radius:
                if entry.agent_id not in nearby_agents:
                    nearby_agents.append(entry.agent_id)
                matched_entries.append({
                    "agent_id": entry.agent_id,
                    "position": entry.position,
                    "timestamp": entry.timestamp,
                    "distance": dist,
                    "observation_data": entry.observation_data,
                })

        return {
            "nearby_agents": nearby_agents,
            "entries": matched_entries,
            "count": len(matched_entries),
        }

    def get_agent_positions(
        self, max_age: float = 5.0
    ) -> Dict[str, Tuple[float, float, float]]:
        """Get recent positions of all agents.

        Args:
            max_age: Only include agents whose last update is within
                *max_age* seconds of the newest update.

        Returns:
            Dict mapping agent_id → (x, y, z).
        """
        if not self._agent_entries:
            return {}

        latest_t = max(e.timestamp for e in self._agent_entries.values())
        t_cutoff = latest_t - max_age

        return {
            aid: entry.position
            for aid, entry in self._agent_entries.items()
            if entry.timestamp >= t_cutoff
        }

    # ------------------------------------------------------------------
    # Maintenance
    # ------------------------------------------------------------------

    def clear_old_entries(self, max_age: float) -> int:
        """Remove history entries older than *max_age* seconds.

        Args:
            max_age: Maximum allowed age in seconds, relative to the
                newest entry in the history.

        Returns:
            Number of entries removed.
        """
        if not self._history:
            return 0

        latest_t = max(e.timestamp for e in self._history)
        t_cutoff = latest_t - max_age

        before = len(self._history)
        self._history = [e for e in self._history if e.timestamp >= t_cutoff]
        removed = before - len(self._history)

        if removed:
            logger.debug("Cleared %d old map entries (cutoff=%.2f).", removed, t_cutoff)
        return removed
