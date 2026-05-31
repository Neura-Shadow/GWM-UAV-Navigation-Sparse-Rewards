"""Deterministic safety controller for emergency situations.

``SafetyController`` is the *cerebellum* — fast, deterministic, and
safety-critical.  It handles collision avoidance (retreat), safe hover,
and emergency stop.  The logic is extracted from the legacy
``AirSimNeuroPlanner._safe_action`` method and generalised so it no
longer depends on AirSim.
"""

from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from src.utils.data_types import ControlCommand, ControlMode, SensorObservation

logger = logging.getLogger(__name__)


class SafetyController:
    """Deterministic safety controller.

    Parameters
    ----------
    min_obstacle_dist:
        Distance (m) below which the safety controller intervenes.
    retreat_speed:
        Backward speed (m/s) used when retreating from an obstacle.
    """

    def __init__(
        self,
        min_obstacle_dist: float = 4.0,
        retreat_speed: float = 2.0,
    ) -> None:
        self.min_obstacle_dist = min_obstacle_dist
        self.retreat_speed = retreat_speed
        logger.info(
            "SafetyController created: min_obs_dist=%.1f, retreat_speed=%.1f",
            min_obstacle_dist,
            retreat_speed,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def check_safety(
        self,
        observation: SensorObservation,
        obstacle_dist: float,
    ) -> bool:
        """Return ``True`` if the situation is safe, ``False`` if intervention is needed."""
        safe = obstacle_dist >= self.min_obstacle_dist
        if not safe:
            logger.warning(
                "Safety violation: obstacle_dist=%.2f < threshold=%.2f",
                obstacle_dist,
                self.min_obstacle_dist,
            )
        return safe

    def get_safe_command(
        self,
        observation: SensorObservation,
        obstacle_dist: float,
        current_yaw: float = 0.0,
    ) -> ControlCommand:
        """Generate a safe control command (retreat, hover, or stop).

        When an obstacle is too close the UAV retreats backward relative
        to its current heading and moves slightly upward.

        Parameters
        ----------
        observation:
            Current sensor snapshot.
        obstacle_dist:
            Estimated distance to the nearest obstacle (m).
        current_yaw:
            Current heading in radians (0 = forward along x-axis).

        Returns
        -------
        ControlCommand
            A safety-override command.
        """
        if obstacle_dist < self.min_obstacle_dist:
            # Retreat backward relative to current heading
            vx = -self.retreat_speed * math.cos(current_yaw)
            vy = -self.retreat_speed * math.sin(current_yaw)
            vz = -0.2  # slight ascent (NED: negative = up)

            cmd = ControlCommand(
                vx=vx,
                vy=vy,
                vz=vz,
                yaw_rate=0.0,
                duration=0.4,
                mode=ControlMode.SAFETY_OVERRIDE,
                metadata={"reason": "obstacle_retreat", "obstacle_dist": obstacle_dist},
            )
            logger.info(
                "Retreat command: vx=%.2f vy=%.2f vz=%.2f (obs_dist=%.2f)",
                vx,
                vy,
                vz,
                obstacle_dist,
            )
            return cmd

        # Default: hover in place
        return ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            mode=ControlMode.SAFETY_OVERRIDE,
            metadata={"reason": "hover"},
        )

    def emergency_stop(self) -> ControlCommand:
        """Generate an immediate stop command."""
        logger.critical("EMERGENCY STOP commanded")
        return ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=0.0,
            mode=ControlMode.EMERGENCY_STOP,
            metadata={"reason": "emergency_stop"},
        )
