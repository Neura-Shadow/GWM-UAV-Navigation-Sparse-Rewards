"""ROS2 bridge package.

The package is importable without ROS2 installed. Real bridge construction is
guarded and raises a clear RuntimeError when ``rclpy`` is unavailable.
"""

from src.ros2_bridge.msg_converters import (
    control_command_to_twist_dict,
    odometry_dict_to_sensor_observation,
    sensor_observation_to_odom_dict,
    twist_dict_to_control_command,
)
from src.ros2_bridge.mavlink_bridge import MAVLinkBridge
from src.ros2_bridge.nav2_costmap_plugin import (
    WorldModelCostmapLayer,
    WorldModelPlannerPlugin,
)
from src.ros2_bridge.qos_config import QoSConfig, qos_from_config
from src.ros2_bridge.real_ros2_adapter import RealROS2Adapter
from src.ros2_bridge.ros2_control_interface import (
    HardwareInterface,
    HardwareState,
    MockHardwareInterface,
    ROS2ControlHardwareInterface,
)
from src.ros2_bridge.ros2_bridge import ROS2Bridge

__all__ = [
    "HardwareInterface",
    "HardwareState",
    "MAVLinkBridge",
    "MockHardwareInterface",
    "QoSConfig",
    "ROS2Bridge",
    "ROS2ControlHardwareInterface",
    "RealROS2Adapter",
    "WorldModelCostmapLayer",
    "WorldModelPlannerPlugin",
    "control_command_to_twist_dict",
    "odometry_dict_to_sensor_observation",
    "qos_from_config",
    "sensor_observation_to_odom_dict",
    "twist_dict_to_control_command",
]
