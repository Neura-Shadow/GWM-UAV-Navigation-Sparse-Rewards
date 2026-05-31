"""Takeover arbiter: decides which controller (world model vs. safety) is active.

``TakeoverArbiter`` implements the asymmetric-control switching logic.  Based
on real-time uncertainty estimates and obstacle proximity it selects one of
three ``ControlMode`` values and routes the corresponding command to the
actuator layer.
"""

from __future__ import annotations

import logging
from typing import Dict, Optional

from src.utils.data_types import ControlCommand, ControlMode

logger = logging.getLogger(__name__)


class TakeoverArbiter:
    """Decides whether the world model or safety controller should be active.

    Decision boundaries::

        obstacle_dist < emergency_threshold    → EMERGENCY_STOP
        obstacle_dist < obstacle_threshold     → SAFETY_OVERRIDE
        uncertainty   > uncertainty_threshold  → SAFETY_OVERRIDE
        otherwise                              → WORLD_MODEL_GUIDED

    Parameters
    ----------
    uncertainty_threshold:
        Uncertainty above which the safety controller takes over.
    obstacle_threshold:
        Obstacle distance below which the safety controller takes over.
    emergency_obstacle_threshold:
        Obstacle distance below which an emergency stop is issued.
    """

    def __init__(
        self,
        uncertainty_threshold: float = 0.7,
        obstacle_threshold: float = 4.0,
        emergency_obstacle_threshold: float = 2.0,
    ) -> None:
        self.uncertainty_threshold = uncertainty_threshold
        self.obstacle_threshold = obstacle_threshold
        self.emergency_obstacle_threshold = emergency_obstacle_threshold
        logger.info(
            "TakeoverArbiter: unc_thresh=%.2f, obs_thresh=%.1f, emerg_thresh=%.1f",
            uncertainty_threshold,
            obstacle_threshold,
            emergency_obstacle_threshold,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def decide(
        self,
        uncertainty: float,
        obstacle_dist: float,
        additional_flags: Optional[Dict[str, object]] = None,
    ) -> ControlMode:
        """Determine which controller should be active.

        Parameters
        ----------
        uncertainty:
            Current model uncertainty in [0, 1].
        obstacle_dist:
            Estimated distance to nearest obstacle (m).
        additional_flags:
            Reserved for future use (e.g., geofence breach, battery low).

        Returns
        -------
        ControlMode
        """
        if obstacle_dist < self.emergency_obstacle_threshold:
            logger.warning(
                "EMERGENCY_STOP: obstacle_dist=%.2f < %.2f",
                obstacle_dist,
                self.emergency_obstacle_threshold,
            )
            return ControlMode.EMERGENCY_STOP

        if obstacle_dist < self.obstacle_threshold:
            logger.info(
                "SAFETY_OVERRIDE (obstacle): dist=%.2f < %.2f",
                obstacle_dist,
                self.obstacle_threshold,
            )
            return ControlMode.SAFETY_OVERRIDE

        if uncertainty > self.uncertainty_threshold:
            logger.info(
                "SAFETY_OVERRIDE (uncertainty): %.3f > %.3f",
                uncertainty,
                self.uncertainty_threshold,
            )
            return ControlMode.SAFETY_OVERRIDE

        return ControlMode.WORLD_MODEL_GUIDED

    def execute(
        self,
        world_model_command: ControlCommand,
        safety_command: ControlCommand,
        mode: ControlMode,
    ) -> ControlCommand:
        """Select the appropriate command based on the active control mode.

        Parameters
        ----------
        world_model_command:
            Command produced by the world-model-guided planner.
        safety_command:
            Command produced by the safety controller.
        mode:
            The ``ControlMode`` chosen by :meth:`decide`.

        Returns
        -------
        ControlCommand
            The selected command with its ``mode`` field updated.
        """
        if mode == ControlMode.WORLD_MODEL_GUIDED:
            cmd = world_model_command
        else:
            cmd = safety_command

        # Stamp the mode on the outgoing command
        cmd.mode = mode
        logger.debug("TakeoverArbiter executing mode=%s", mode.value)
        return cmd
