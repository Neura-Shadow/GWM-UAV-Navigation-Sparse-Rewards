"""ROS2 QoS configuration helpers.

The dataclass is intentionally independent of ``rclpy`` so tests can validate
QoS parsing without requiring a ROS2 installation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


_VALID_RELIABILITY = {"reliable", "best_effort"}


@dataclass(frozen=True)
class QoSConfig:
    """Small transport-neutral QoS profile used by the ROS2 bridge."""

    reliability: str = "reliable"
    history_depth: int = 10
    deadline_ms: float = 100.0
    lifespan_sec: float = 1.0

    def __post_init__(self) -> None:
        normalized = normalize_reliability(self.reliability)
        if self.history_depth < 1:
            raise ValueError("history_depth must be >= 1")
        object.__setattr__(self, "reliability", normalized)

    def to_dict(self) -> Dict[str, Any]:
        """Return a plain dict suitable for YAML serialization."""
        return {
            "reliability": self.reliability,
            "history_depth": self.history_depth,
            "deadline_ms": self.deadline_ms,
            "lifespan_sec": self.lifespan_sec,
        }


def normalize_reliability(value: str) -> str:
    """Normalize ROS-style reliability values to lowercase snake_case."""
    normalized = value.strip().lower().replace("-", "_").replace(" ", "_")
    if normalized not in _VALID_RELIABILITY:
        valid = ", ".join(sorted(_VALID_RELIABILITY))
        raise ValueError(f"Unsupported QoS reliability '{value}'. Expected one of: {valid}")
    return normalized


def qos_from_config(config: Dict[str, Any] | None) -> QoSConfig:
    """Build a :class:`QoSConfig` from a possibly sparse config dictionary."""
    if config is None:
        return QoSConfig()
    return QoSConfig(
        reliability=str(config.get("reliability", "reliable")),
        history_depth=int(config.get("history_depth", 10)),
        deadline_ms=float(config.get("deadline_ms", 100.0)),
        lifespan_sec=float(config.get("lifespan_sec", 1.0)),
    )
