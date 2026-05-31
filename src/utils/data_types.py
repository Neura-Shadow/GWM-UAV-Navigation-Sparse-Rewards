"""Shared data types for the UAV Navigation framework.

These dataclasses and enums form the communication contract between
all subsystems: world-model (Axis 1), control (Axis 2), digital twin
(Axis 3), and multi-agent (Axis 4).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class ControlMode(Enum):
    """Operating mode for the asymmetric controller."""

    WORLD_MODEL_GUIDED = "world_model_guided"
    SAFETY_OVERRIDE = "safety_override"
    EMERGENCY_STOP = "emergency_stop"


class VehicleType(Enum):
    """Types of vehicles supported by the framework."""

    UAV = "uav"
    UGV = "ugv"
    AMR = "amr"


class AgentStatus(Enum):
    """Lifecycle states of an individual agent."""

    IDLE = "idle"
    NAVIGATING = "navigating"
    EMERGENCY = "emergency"
    COMPLETED = "completed"


# ---------------------------------------------------------------------------
# Observation / perception layer
# ---------------------------------------------------------------------------

@dataclass
class SensorObservation:
    """A single timestamped sensor snapshot from the UAV.

    The ``to_state_vector`` method converts the observation into the 8-dim
    vector expected by the baseline world model:
    ``[px, py, pz, vx, vy, vz, goal_distance, obstacle_distance]``

    Attributes:
        timestamp: Unix epoch seconds (float).
        pose: (x, y, z) position in world frame.
        velocity: (vx, vy, vz) linear velocity in world frame.
        goal_distance: Euclidean distance to goal [m].
        obstacle_distance: Estimated distance to nearest obstacle [m].
        image: Optional RGB image as numpy array (H, W, 3) uint8.
        lidar: Optional LiDAR point cloud as numpy array (N, 3) float32.
        depth: Optional depth map as numpy array (H, W) float32.
        metadata: Arbitrary key-value bag for extra sensor data.
    """

    timestamp: float = 0.0
    pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal_distance: float = 0.0
    obstacle_distance: float = 50.0
    image: Optional[np.ndarray] = None
    lidar: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_state_vector(self) -> np.ndarray:
        """Flatten observation into the 8-dim state vector used by the world model.

        Layout: ``[px, py, pz, vx, vy, vz, goal_dist, obstacle_dist]``
        """
        return np.array(
            list(self.pose) + list(self.velocity)
            + [self.goal_distance, self.obstacle_distance],
            dtype=np.float32,
        )



# ---------------------------------------------------------------------------
# World-model latent representation
# ---------------------------------------------------------------------------

@dataclass
class LatentState:
    """Latent-space representation produced by a sensor encoder.

    Attributes:
        vector: Latent feature vector (numpy array or torch tensor).
        uncertainty: Scalar uncertainty estimate in [0, 1].
        timestamp: Inherited from the originating observation.
        metadata: Arbitrary key-value bag.
    """

    vector: Any  # np.ndarray or torch.Tensor
    uncertainty: float = 0.0
    timestamp: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Policy / planning outputs
# ---------------------------------------------------------------------------

@dataclass
class PolicyIntent:
    """High-level intent output by the world-model-guided planner.

    Bridges the gap between latent-space predictions and low-level
    velocity commands.

    Attributes:
        target_position: Desired (x, y, z) waypoint.
        desired_velocity: Scalar speed toward target.
        risk_score: Aggregate collision / safety risk in [0, 1].
        horizon: Planning horizon length (number of steps).
        confidence: Planner confidence in [0, 1].
        metadata: Arbitrary key-value bag.
    """

    target_position: Tuple[float, float, float]
    desired_velocity: float = 0.0
    risk_score: float = 0.0
    horizon: int = 1
    confidence: float = 1.0
    metadata: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Low-level control
# ---------------------------------------------------------------------------

@dataclass
class ControlCommand:
    """Low-level velocity command sent to the UAV actuators.

    Attributes:
        vx: Forward velocity (m/s).
        vy: Lateral velocity (m/s).
        vz: Vertical velocity (m/s, negative = up in NED).
        yaw_rate: Yaw rate (rad/s), 0 = hold heading.
        duration: Duration for this command (seconds).
        mode: Active control mode when command was generated.
        metadata: Arbitrary key-value bag.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0
    duration: float = 0.4
    mode: ControlMode = ControlMode.WORLD_MODEL_GUIDED
    metadata: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Multi-agent state
# ---------------------------------------------------------------------------

@dataclass
class AgentState:
    """Full observable state of a single agent in the fleet.

    Attributes:
        agent_id: Unique string identifier for the agent.
        vehicle_type: Physical vehicle class (UAV, UGV, AMR).
        observation: Latest sensor observation, if available.
        latent_state: Latest world-model latent encoding, if available.
        status: Current lifecycle status.
        task_id: Currently assigned task, if any.
        metadata: Arbitrary key-value bag.
    """

    agent_id: str
    vehicle_type: VehicleType = VehicleType.UAV
    observation: Optional[SensorObservation] = None
    latent_state: Optional[LatentState] = None
    status: AgentStatus = AgentStatus.IDLE
    task_id: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Scenario / digital-twin specification
# ---------------------------------------------------------------------------

@dataclass
class ScenarioSpec:
    """Describes a single simulation scenario for the digital-twin pipeline.

    Attributes:
        scenario_id: Unique identifier for this scenario.
        description: Human-readable description of the scenario.
        start_position: (x, y, z) spawn position.
        goal_position: (x, y, z) target position.
        obstacles: List of obstacle descriptors (position, size, type).
        weather: Weather condition parameters (wind, rain, fog, etc.).
        sensor_noise: Per-sensor noise scale factors.
        physics: Physical simulation parameters (friction, drag, etc.).
        metadata: Arbitrary key-value bag.
    """

    scenario_id: str
    description: str = ""
    start_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    goal_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    weather: Dict[str, Any] = field(default_factory=dict)
    sensor_noise: Dict[str, float] = field(default_factory=dict)
    physics: Dict[str, Any] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)
