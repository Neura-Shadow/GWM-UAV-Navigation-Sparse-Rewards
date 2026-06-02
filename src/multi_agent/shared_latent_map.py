"""Shared latent map for distributed multi-agent perception."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from src.multi_agent.shared_map import SharedSpatiotemporalMap

logger = logging.getLogger(__name__)


@dataclass
class _LatentMapEntry:
    """Internal record for one latent map update."""

    agent_id: str
    position: Tuple[float, float, float]
    timestamp: float
    latent_vector: np.ndarray
    confidence: float = 1.0
    metadata: Dict[str, Any] = field(default_factory=dict)


class SharedLatentMap(SharedSpatiotemporalMap):
    """Spatiotemporal map extended with latent feature storage and fusion."""

    def __init__(
        self,
        bounds: Tuple[float, float, float, float, float, float] = (
            -100.0, -100.0, -50.0, 100.0, 100.0, 0.0,
        ),
        resolution: float = 1.0,
        time_window: float = 10.0,
        latent_dim: int = 32,
        confidence_decay: float = 0.95,
        merge_strategy: str = "weighted_average",
    ) -> None:
        super().__init__(
            bounds=bounds,
            resolution=resolution,
            time_window=time_window,
        )
        if latent_dim <= 0:
            raise ValueError("latent_dim must be positive.")
        if merge_strategy != "weighted_average":
            raise ValueError("Only weighted_average merge_strategy is supported.")

        self.latent_dim = latent_dim
        self.confidence_decay = confidence_decay
        self.merge_strategy = merge_strategy
        self._latent_entries: Dict[str, _LatentMapEntry] = {}
        self._latent_history: List[_LatentMapEntry] = []

    def update_with_latent(
        self,
        agent_id: str,
        position: Tuple[float, float, float],
        timestamp: float,
        latent_vector: Any,
        confidence: float = 1.0,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Write a latent vector and keep parent spatiotemporal history aligned."""
        vector = _coerce_latent_vector(latent_vector, self.latent_dim)
        entry = _LatentMapEntry(
            agent_id=agent_id,
            position=tuple(float(v) for v in position),
            timestamp=float(timestamp),
            latent_vector=vector,
            confidence=max(0.0, float(confidence)),
            metadata=metadata or {},
        )
        self._latent_entries[agent_id] = entry
        self._latent_history.append(entry)

        super().update(
            agent_id=agent_id,
            position=entry.position,
            timestamp=entry.timestamp,
            observation_data={
                "latent_vector": vector.tolist(),
                "confidence": entry.confidence,
                "metadata": entry.metadata,
            },
        )
        logger.debug(
            "Latent map updated: agent=%s pos=%s t=%.2f.",
            agent_id,
            entry.position,
            entry.timestamp,
        )

    def query_latents(
        self,
        position: Tuple[float, float, float],
        radius: float,
        time_window: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Return latent entries near a position within the time window."""
        window = time_window if time_window is not None else self.time_window
        query_position = np.asarray(position, dtype=np.float64)
        cutoff = self._time_cutoff(window)

        nearby_agents: List[str] = []
        entries: List[Dict[str, Any]] = []

        for entry in self._latent_history:
            if entry.timestamp < cutoff:
                continue
            distance = float(
                np.linalg.norm(np.asarray(entry.position, dtype=np.float64) - query_position)
            )
            if distance <= radius:
                if entry.agent_id not in nearby_agents:
                    nearby_agents.append(entry.agent_id)
                entries.append({
                    "agent_id": entry.agent_id,
                    "position": entry.position,
                    "timestamp": entry.timestamp,
                    "distance": distance,
                    "latent_vector": entry.latent_vector.copy(),
                    "confidence": entry.confidence,
                    "metadata": dict(entry.metadata),
                })

        return {
            "nearby_agents": nearby_agents,
            "entries": entries,
            "count": len(entries),
        }

    def merge_latents(self, latent_entries: List[Dict[str, Any]]) -> np.ndarray:
        """Fuse latent entries using confidence and recency weighting."""
        if not latent_entries:
            return np.zeros(self.latent_dim, dtype=np.float32)

        latest_t = max(float(entry.get("timestamp", 0.0)) for entry in latent_entries)
        vectors = []
        weights = []

        for entry in latent_entries:
            vector = _coerce_latent_vector(entry["latent_vector"], self.latent_dim)
            age = max(0.0, latest_t - float(entry.get("timestamp", latest_t)))
            confidence = max(0.0, float(entry.get("confidence", 1.0)))
            weight = confidence * (self.confidence_decay ** age)
            vectors.append(vector)
            weights.append(weight)

        weight_array = np.asarray(weights, dtype=np.float32)
        if float(weight_array.sum()) <= 0.0:
            weight_array = np.ones(len(vectors), dtype=np.float32)

        stacked = np.vstack(vectors).astype(np.float32)
        return np.average(stacked, axis=0, weights=weight_array).astype(np.float32)

    def clear_old_entries(self, max_age: float) -> int:
        """Remove old spatiotemporal and latent history entries."""
        parent_removed = super().clear_old_entries(max_age=max_age)
        if not self._latent_history:
            return parent_removed

        latest_t = max(entry.timestamp for entry in self._latent_history)
        cutoff = latest_t - max_age
        before = len(self._latent_history)
        self._latent_history = [
            entry for entry in self._latent_history if entry.timestamp >= cutoff
        ]
        self._latent_entries = {
            agent_id: entry
            for agent_id, entry in self._latent_entries.items()
            if entry.timestamp >= cutoff
        }
        latent_removed = before - len(self._latent_history)
        return max(parent_removed, latent_removed)

    def _time_cutoff(self, time_window: float) -> float:
        if not self._latent_history:
            return -float("inf")
        latest_t = max(entry.timestamp for entry in self._latent_history)
        return latest_t - time_window


def _coerce_latent_vector(latent_vector: Any, latent_dim: int) -> np.ndarray:
    vector = np.asarray(latent_vector, dtype=np.float32).reshape(-1)
    if vector.shape[0] != latent_dim:
        raise ValueError(
            f"latent_dim mismatch: expected {latent_dim}, got {vector.shape[0]}."
        )
    return vector
