"""Types for Phase 5-A runtime capability detection."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional


@dataclass
class CapabilityStatus:
    """Availability status for one optional runtime capability."""

    name: str
    available: bool
    version: Optional[str] = None
    details: Optional[Dict[str, Any]] = None
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-safe dictionary."""
        return asdict(self)


@dataclass
class RuntimeCapabilityReport:
    """Read-only runtime capability report for real-runtime readiness planning."""

    schema_version: str
    generated_at: str
    platform: Dict[str, Any]
    python: Dict[str, Any]
    cuda: Dict[str, Any]
    gpu: Dict[str, Any]
    isaac_sim: CapabilityStatus
    ros2: CapabilityStatus
    mavsdk: CapabilityStatus
    px4: CapabilityStatus
    github_cli: CapabilityStatus
    environment: Dict[str, Any]
    safety: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        """Return a JSON-safe dictionary."""
        data = asdict(self)
        for key in ("isaac_sim", "ros2", "mavsdk", "px4", "github_cli"):
            value = getattr(self, key)
            data[key] = value.to_dict()
        return data

