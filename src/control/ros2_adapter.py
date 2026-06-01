"""ROS2 adapter: abstract interface and mock implementation for ROS2 comms.

``ROS2Adapter`` defines the abstract contract for sending velocity commands
and receiving odometry feedback via ROS2 topics.  ``MockROS2Adapter``
provides a self-contained mock that logs commands and tracks history,
enabling the full control stack to be tested without a ROS2 installation.

Expected ROS2 topics (for future integration)::

    /cmd_vel         geometry_msgs/Twist       velocity commands
    /odom            nav_msgs/Odometry         odometry feedback
    /scan            sensor_msgs/LaserScan     LiDAR data
    /camera/depth    sensor_msgs/Image         depth camera
    /safety/takeover std_msgs/String           takeover notifications
"""

from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from typing import List, Optional

from src.utils.data_types import ControlCommand, SensorObservation

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Abstract interface
# ---------------------------------------------------------------------------

class ROS2Adapter(ABC):
    """Abstract interface for ROS2 communication."""

    def connect(self) -> bool:
        """Connect to the underlying transport.

        Existing adapters are assumed to be connected on construction, so the
        default is a no-op for backward compatibility.
        """
        return True

    def disconnect(self) -> None:
        """Disconnect from the underlying transport."""

    @property
    def is_connected(self) -> bool:
        """Return whether the adapter is ready to exchange messages."""
        return True

    @abstractmethod
    def send_command(self, command: ControlCommand) -> bool:
        """Publish a velocity command.  Return ``True`` on success."""

    @abstractmethod
    def get_odometry(self) -> Optional[SensorObservation]:
        """Subscribe to odometry and return the latest observation, or ``None``."""


# ---------------------------------------------------------------------------
# Mock implementation
# ---------------------------------------------------------------------------

class MockROS2Adapter(ROS2Adapter):
    """Mock ROS2 adapter for testing without a ROS2 installation.

    All commands are logged and stored in :pyattr:`command_history` so
    tests can inspect what was sent.
    """

    def __init__(self) -> None:
        self.command_history: List[ControlCommand] = []
        self._connected: bool = True
        logger.info("MockROS2Adapter created (simulated ROS2 bridge)")

    @property
    def is_connected(self) -> bool:
        """Return whether the mock adapter is connected."""
        return self._connected

    def connect(self) -> bool:
        """Simulate connection or reconnection."""
        self._connected = True
        logger.info("MockROS2Adapter: simulated connect")
        return True

    def send_command(self, command: ControlCommand) -> bool:
        """Log the command and append to history."""
        if not self._connected:
            logger.error("MockROS2Adapter: not connected, command dropped")
            return False

        self.command_history.append(command)
        logger.debug(
            "MockROS2 cmd: vx=%.2f vy=%.2f vz=%.2f mode=%s",
            command.vx,
            command.vy,
            command.vz,
            command.mode.value,
        )
        return True

    def get_odometry(self) -> Optional[SensorObservation]:
        """Return a dummy odometry observation at the origin."""
        if not self._connected:
            return None

        return SensorObservation(
            timestamp=time.time(),
            pose=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            metadata={"source": "mock_ros2"},
        )

    # -- test helpers -------------------------------------------------------

    def disconnect(self) -> None:
        """Simulate a ROS2 connection loss."""
        self._connected = False
        logger.warning("MockROS2Adapter: simulated disconnect")

    def reconnect(self) -> None:
        """Simulate reconnection."""
        self.connect()
