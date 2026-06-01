"""Real ROS2 adapter behind the narrow control adapter contract."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from src.control.ros2_adapter import ROS2Adapter
from src.ros2_bridge.msg_converters import (
    control_command_to_twist_dict,
    odometry_dict_to_sensor_observation,
)
from src.ros2_bridge.qos_config import qos_from_config
from src.ros2_bridge.ros2_bridge import ROS2Bridge, _HAS_RCLPY
from src.utils.data_types import ControlCommand, SensorObservation

logger = logging.getLogger(__name__)

try:  # pragma: no cover - exercised only with ROS2 message packages
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - default in normal test environments
    Twist = None  # type: ignore[assignment]
    Odometry = None  # type: ignore[assignment]


class RealROS2Adapter(ROS2Adapter):
    """ROS2-backed command and odometry adapter.

    The adapter keeps the existing control interface small while delegating
    ROS2 node lifecycle and pub/sub setup to :class:`ROS2Bridge`.
    """

    def __init__(
        self,
        config: Dict[str, Any] | None = None,
        bridge: ROS2Bridge | None = None,
    ) -> None:
        self.config = config or {}
        ros2_config = self.config.get("ros2", self.config)
        topics = ros2_config.get("topics", {})
        qos_profiles = ros2_config.get("qos_profiles", {})

        self.command_topic = topics.get("cmd_vel", "/cmd_vel")
        self.odom_topic = topics.get("odom", topics.get("odometry", "/odom"))
        self._latest_odometry: Optional[SensorObservation] = None
        self._connected = False
        self._owns_bridge = bridge is None

        if self._owns_bridge:
            if not _HAS_RCLPY:
                raise RuntimeError(
                    "RealROS2Adapter requires rclpy. Install ROS2 Humble or inject a mock bridge."
                )
            if Twist is None or Odometry is None:
                raise RuntimeError(
                    "RealROS2Adapter requires geometry_msgs and nav_msgs from ROS2."
                )

        node_name = ros2_config.get("node_name", "gwm_uav_bridge")
        self.bridge = bridge or ROS2Bridge(node_name=node_name, config=ros2_config)

        command_qos = qos_from_config(qos_profiles.get("control_commands"))
        odom_qos = qos_from_config(qos_profiles.get("odometry"))
        self._publisher = self.bridge.create_publisher(
            self.command_topic,
            Twist or dict,
            command_qos,
        )
        self._subscription = self.bridge.create_subscription(
            self.odom_topic,
            Odometry or dict,
            self._on_odometry,
            odom_qos,
        )
        self._connected = True
        logger.info(
            "RealROS2Adapter connected (cmd=%s, odom=%s)",
            self.command_topic,
            self.odom_topic,
        )

    @property
    def is_connected(self) -> bool:
        """Return whether this adapter is accepting commands."""
        return self._connected

    def connect(self) -> bool:
        """Mark the adapter connected.

        Real node creation happens in ``__init__`` for Phase 3-A.
        """
        if self._owns_bridge and getattr(self.bridge, "is_shutdown", False):
            raise RuntimeError("Cannot reconnect a shut down ROS2Bridge; create a new adapter.")
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect and shut down the owned bridge."""
        self._connected = False
        if self._owns_bridge:
            self.bridge.shutdown()

    def send_command(self, command: ControlCommand) -> bool:
        """Publish a velocity command to the configured command topic."""
        if not self._connected:
            logger.error("RealROS2Adapter: not connected, command dropped")
            return False
        msg = self._twist_from_dict(control_command_to_twist_dict(command))
        self._publisher.publish(msg)
        return True

    def get_odometry(self) -> Optional[SensorObservation]:
        """Return the latest odometry received by the subscription callback."""
        return self._latest_odometry

    def _on_odometry(self, message: Any) -> None:
        self._latest_odometry = odometry_dict_to_sensor_observation(self._odom_to_dict(message))

    def _twist_from_dict(self, twist: Dict[str, Any]) -> Any:
        if Twist is None:
            return twist
        msg = Twist()
        msg.linear.x = twist["linear"]["x"]
        msg.linear.y = twist["linear"]["y"]
        msg.linear.z = twist["linear"]["z"]
        msg.angular.z = twist["angular"]["z"]
        return msg

    def _odom_to_dict(self, odom: Any) -> Dict[str, Any]:
        if isinstance(odom, dict):
            return odom

        stamp = getattr(getattr(odom, "header", None), "stamp", None)
        pose = odom.pose.pose
        twist = odom.twist.twist
        return {
            "timestamp": float(getattr(stamp, "sec", 0.0))
            + float(getattr(stamp, "nanosec", 0.0)) * 1e-9,
            "pose": {
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                }
            },
            "twist": {
                "linear": {
                    "x": twist.linear.x,
                    "y": twist.linear.y,
                    "z": twist.linear.z,
                }
            },
            "metadata": {"source": "ros2"},
        }
