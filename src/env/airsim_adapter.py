"""AirSim / CosysAirSim concrete navigation environment.

This module extracts **all** AirSim-specific logic from the original monolithic
``AirSimNeuroPlanner`` class into a clean :class:`BaseNavigationEnv`
implementation.

The import is guarded so that every other module in the project can be imported
and tested without AirSim installed.
"""

from __future__ import annotations

import logging
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np

from src.env.base_env import BaseNavigationEnv
from src.utils.data_types import SensorObservation

logger = logging.getLogger(__name__)

# ---- Import guard – AirSim is never a hard dependency ----
try:
    import cosysairsim as airsim
except ImportError:
    try:
        import airsim  # type: ignore[no-redef]
    except ImportError:
        airsim = None  # type: ignore[assignment]


class AirSimNavigationEnv(BaseNavigationEnv):
    """Navigation environment backed by AirSim / CosysAirSim.

    Parameters
    ----------
    goal:
        3D goal position ``(x, y, z)`` in NED frame.
    target_altitude:
        Desired cruise altitude (NED, negative = up).
    control_dt:
        Duration passed to ``moveByVelocityAsync`` per step.
    min_obstacle_dist:
        Safety threshold for obstacle avoidance override.
    goal_reach_dist:
        Episode terminates when the UAV is closer than this to the goal.
    lidar_name:
        Name of the LiDAR sensor as configured in AirSim settings.
    host:
        IP address of the AirSim host.  ``None`` → localhost.
    port:
        API port of the AirSim host.
    vehicle_name:
        Name of the multirotor vehicle in the simulation.

    Raises
    ------
    ImportError
        If neither ``cosysairsim`` nor ``airsim`` is installed.
    """

    def __init__(
        self,
        goal: Tuple[float, float, float] = (60.0, 20.0, -8.0),
        target_altitude: float = -8.0,
        control_dt: float = 0.4,
        min_obstacle_dist: float = 4.0,
        goal_reach_dist: float = 3.0,
        lidar_name: str = "LidarSensor1",
        host: Optional[str] = None,
        port: int = 41451,
        vehicle_name: str = "",
    ) -> None:
        if airsim is None:
            raise ImportError(
                "AirSim Python package is required for AirSimNavigationEnv.  "
                "Install with: pip install cosysairsim   or   pip install airsim"
            )

        self._goal = np.array(goal, dtype=np.float32)
        self._target_altitude = target_altitude
        self._control_dt = control_dt
        self._min_obstacle_dist = min_obstacle_dist
        self._goal_reach_dist = goal_reach_dist
        self._lidar_name = lidar_name
        self._vehicle_name = vehicle_name

        # Create client
        connect_kwargs: Dict[str, Any] = {"port": port}
        if host is not None:
            connect_kwargs["ip"] = host
        self._client = airsim.MultirotorClient(**connect_kwargs)

        self._connected = False
        self._step_count = 0

    # ------------------------------------------------------------------
    # Connection helpers
    # ------------------------------------------------------------------

    def connect_and_prepare(self) -> None:
        """Confirm connection, arm, take-off, and ascend to cruise altitude."""
        logger.info("Connecting to AirSim…")
        self._client.confirmConnection()
        self._client.enableApiControl(True, self._vehicle_name)
        self._client.armDisarm(True, self._vehicle_name)

        logger.info("Taking off…")
        self._client.takeoffAsync(
            timeout_sec=12, vehicle_name=self._vehicle_name
        ).join()
        self._client.moveToZAsync(
            self._target_altitude, 2.5, vehicle_name=self._vehicle_name
        ).join()

        self._connected = True
        logger.info("UAV airborne – mission ready.")

    def shutdown(self) -> None:
        """Hover, disarm, and release API control."""
        if not self._connected:
            return
        logger.info("Shutting down AirSim environment…")
        self._client.hoverAsync(vehicle_name=self._vehicle_name).join()
        self._client.armDisarm(False, self._vehicle_name)
        self._client.enableApiControl(False, self._vehicle_name)
        self._connected = False

    # ------------------------------------------------------------------
    # BaseNavigationEnv interface
    # ------------------------------------------------------------------

    def reset(self) -> SensorObservation:
        """Reset the AirSim simulation and return the initial observation.

        If not yet connected the method calls :meth:`connect_and_prepare`
        automatically.
        """
        if not self._connected:
            self.connect_and_prepare()
        else:
            # Reset the simulation to initial state
            self._client.reset()
            self.connect_and_prepare()

        self._step_count = 0
        return self.get_observation()

    def step(
        self, action: np.ndarray
    ) -> Tuple[SensorObservation, float, bool, Dict[str, Any]]:
        """Execute a velocity command and return the transition tuple."""
        action = np.asarray(action, dtype=np.float32)

        self._client.moveByVelocityAsync(
            vx=float(action[0]),
            vy=float(action[1]),
            vz=float(action[2]),
            duration=self._control_dt,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
            vehicle_name=self._vehicle_name,
        ).join()

        obs = self.get_observation()
        self._step_count += 1

        done = obs.goal_distance < self._goal_reach_dist
        reward = -obs.goal_distance  # negative-cost convention

        info: Dict[str, Any] = {
            "step": self._step_count,
            "goal_distance": obs.goal_distance,
            "obstacle_distance": obs.obstacle_distance,
        }
        return obs, reward, done, info

    def get_observation(self) -> SensorObservation:
        """Query AirSim for the latest sensor observation."""
        kin = self._client.getMultirotorState(
            vehicle_name=self._vehicle_name
        ).kinematics_estimated

        pos = (kin.position.x_val, kin.position.y_val, kin.position.z_val)
        vel = (
            kin.linear_velocity.x_val,
            kin.linear_velocity.y_val,
            kin.linear_velocity.z_val,
        )

        pos_arr = np.array(pos, dtype=np.float32)
        goal_dist = float(np.linalg.norm(pos_arr - self._goal))
        obs_dist = self._estimate_front_obstacle_dist()

        return SensorObservation(
            timestamp=float(self._step_count * self._control_dt),
            pose=pos,
            velocity=vel,
            goal_distance=goal_dist,
            obstacle_distance=obs_dist,
        )

    def get_state_vector(self) -> np.ndarray:
        """Return the 8-dim state vector ``[px,py,pz, vx,vy,vz, g, o]``."""
        return self.get_observation().to_state_vector()

    def close(self) -> None:
        self.shutdown()

    @property
    def state_dim(self) -> int:
        return 8

    @property
    def action_dim(self) -> int:
        return 3

    # ------------------------------------------------------------------
    # Safety override (ported from original _safe_action)
    # ------------------------------------------------------------------

    def safe_action(
        self, action: np.ndarray, state: np.ndarray
    ) -> np.ndarray:
        """Apply obstacle-avoidance override to the proposed action.

        If the nearest obstacle is closer than ``min_obstacle_dist``, the UAV
        retreats along the negative yaw direction (matching the original
        ``_safe_action`` logic).
        """
        action = action.copy()
        obstacle_dist = state[7]
        if obstacle_dist < self._min_obstacle_dist:
            kin = self._client.getMultirotorState(
                vehicle_name=self._vehicle_name
            ).kinematics_estimated
            yaw = airsim.to_eularian_angles(kin.orientation)[2]
            action[0] = -2.0 * math.cos(yaw)
            action[1] = -2.0 * math.sin(yaw)
            action[2] = -0.2
            logger.warning(
                "Obstacle avoidance override triggered (dist=%.2f m)", obstacle_dist
            )
        return action

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _estimate_front_obstacle_dist(self) -> float:
        """Estimate the distance to the nearest front obstacle.

        Tries LiDAR first; falls back to the depth camera center pixel.
        """
        # --- LiDAR ---
        try:
            lidar = self._client.getLidarData(
                lidar_name=self._lidar_name,
                vehicle_name=self._vehicle_name,
            )
            if lidar.point_cloud and len(lidar.point_cloud) >= 3:
                pts = np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3)
                dists = np.linalg.norm(pts, axis=1)
                return float(np.clip(np.min(dists), 0.2, 50.0))
        except Exception:
            logger.debug("LiDAR query failed; falling back to depth camera.")

        # --- Depth camera fallback ---
        try:
            responses = self._client.simGetImages(
                [airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True)],
                vehicle_name=self._vehicle_name,
            )
            if responses:
                resp = responses[0]
                if resp.width > 0 and resp.height > 0 and resp.image_data_float:
                    depth = np.array(resp.image_data_float, dtype=np.float32)
                    depth = np.clip(depth, 0.2, 100.0)
                    center = depth[(resp.width * resp.height) // 2]
                    return float(center)
        except Exception:
            logger.debug("Depth camera query failed.")

        # Fallback – assume free space
        return 50.0
