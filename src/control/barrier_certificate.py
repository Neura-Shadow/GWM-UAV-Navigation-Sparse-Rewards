"""Mock-first Control Barrier Function safety filter.

This module provides a lightweight, pure-Python baseline for deployment-time
safety filtering. It is not a certification proof; it is a testable interface
for command saturation and simple geometric obstacle checks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np

from src.utils.data_types import ControlCommand, ControlMode, SensorObservation


@dataclass(frozen=True)
class SafetyLimits:
    """Deployment safety limits applied before hardware writes."""

    max_vx: float = 4.0
    max_vy: float = 4.0
    max_vz: float = 2.0
    max_yaw_rate: float = 1.0
    min_altitude: float = 0.5
    max_altitude: float = 120.0
    geofence: Optional[Dict[str, Any]] = None


class ControlBarrierFunction:
    """Conservative CBF-style safety filter for mock deployment flows."""

    def __init__(
        self,
        limits: Optional[SafetyLimits] = None,
        min_obstacle_distance: float = 4.0,
        alpha: float = 1.0,
    ) -> None:
        self.limits = limits or SafetyLimits()
        self.min_obstacle_distance = float(min_obstacle_distance)
        self.alpha = float(alpha)

    def h(self, state: Any, obstacle: Any) -> float:
        """Return positive margin when obstacle distance is safe."""
        state_position = np.asarray(_position_from_state(state), dtype=np.float64)
        obstacle_position = np.asarray(_position_from_state(obstacle), dtype=np.float64)
        distance = float(np.linalg.norm(state_position - obstacle_position))
        return distance - self.min_obstacle_distance

    def h_dot(self, state: Any, obstacle: Any, action: ControlCommand) -> float:
        """Approximate barrier derivative under a desired velocity command."""
        state_position = np.asarray(_position_from_state(state), dtype=np.float64)
        obstacle_position = np.asarray(_position_from_state(obstacle), dtype=np.float64)
        relative = state_position - obstacle_position
        distance = float(np.linalg.norm(relative))
        if distance < 1e-9:
            return -float("inf")
        direction = relative / distance
        velocity = np.asarray((action.vx, action.vy, action.vz), dtype=np.float64)
        return float(np.dot(direction, velocity))

    def is_safe(self, state: Any, obstacle: Any) -> bool:
        """Return True when the state is outside the obstacle safety margin."""
        return self.h(state, obstacle) >= 0.0

    def filter_action(
        self,
        state: Any,
        obstacle: Any,
        desired_action: ControlCommand,
    ) -> ControlCommand:
        """Clamp a command and hover if an obstacle violates the barrier."""
        saturated = self.saturate(desired_action)
        if self.is_safe(state, obstacle):
            return saturated

        metadata = dict(saturated.metadata)
        metadata.update({
            "reason": "cbf_obstacle_filter",
            "barrier_margin": self.h(state, obstacle),
        })
        return ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=saturated.duration,
            mode=ControlMode.SAFETY_OVERRIDE,
            metadata=metadata,
        )

    def saturate(self, command: ControlCommand) -> ControlCommand:
        """Clamp linear velocity and yaw rate to configured limits."""
        vx = _clamp(command.vx, -self.limits.max_vx, self.limits.max_vx)
        vy = _clamp(command.vy, -self.limits.max_vy, self.limits.max_vy)
        vz = _clamp(command.vz, -self.limits.max_vz, self.limits.max_vz)
        yaw_rate = _clamp(
            command.yaw_rate,
            -self.limits.max_yaw_rate,
            self.limits.max_yaw_rate,
        )
        saturated = (
            vx != command.vx
            or vy != command.vy
            or vz != command.vz
            or yaw_rate != command.yaw_rate
        )
        metadata = dict(command.metadata)
        if saturated:
            metadata["saturated"] = True

        return ControlCommand(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate,
            duration=command.duration,
            mode=command.mode,
            metadata=metadata,
        )

    def within_altitude_bounds(self, state: Any) -> bool:
        """Check positive altitude inferred from state metadata or NED z."""
        altitude = _altitude_from_state(state)
        return self.limits.min_altitude <= altitude <= self.limits.max_altitude

    def within_geofence(self, state: Any) -> bool:
        """Check optional axis-aligned geofence bounds."""
        geofence = self.limits.geofence
        if not geofence:
            return True
        x, y, z = _position_from_state(state)
        for axis, value in (("x", x), ("y", y), ("z", z)):
            bounds = geofence.get(axis)
            if bounds is None:
                continue
            if len(bounds) != 2:
                raise ValueError(f"Geofence axis '{axis}' must contain [min, max].")
            if float(value) < float(bounds[0]) or float(value) > float(bounds[1]):
                return False
        return True


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _position_from_state(state: Any) -> Tuple[float, float, float]:
    if isinstance(state, SensorObservation):
        return tuple(float(v) for v in state.pose)
    if isinstance(state, dict):
        value = state.get("position", state.get("pose", (0.0, 0.0, 0.0)))
        return _coerce_position(value)
    if hasattr(state, "position"):
        return _coerce_position(getattr(state, "position"))
    if hasattr(state, "pose"):
        return _coerce_position(getattr(state, "pose"))
    return _coerce_position(state)


def _coerce_position(value: Any) -> Tuple[float, float, float]:
    if isinstance(value, dict):
        return (
            float(value.get("x", 0.0)),
            float(value.get("y", 0.0)),
            float(value.get("z", 0.0)),
        )
    if isinstance(value, Sequence) and not isinstance(value, str):
        padded = list(value) + [0.0, 0.0, 0.0]
        return (float(padded[0]), float(padded[1]), float(padded[2]))
    return (0.0, 0.0, 0.0)


def _altitude_from_state(state: Any) -> float:
    if isinstance(state, dict):
        if "altitude" in state:
            return float(state["altitude"])
        metadata = state.get("metadata", {})
        if isinstance(metadata, dict) and "altitude" in metadata:
            return float(metadata["altitude"])
    if hasattr(state, "metadata"):
        metadata = getattr(state, "metadata")
        if isinstance(metadata, dict) and "altitude" in metadata:
            return float(metadata["altitude"])

    _, _, z = _position_from_state(state)
    return abs(float(z))
