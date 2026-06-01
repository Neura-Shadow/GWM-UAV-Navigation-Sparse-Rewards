"""Pure-Python conversion helpers for ROS2-style message payloads."""

from __future__ import annotations

from typing import Any, Dict

from src.utils.data_types import ControlCommand, ControlMode, SensorObservation


def control_command_to_twist_dict(cmd: ControlCommand) -> Dict[str, Any]:
    """Convert a control command into a Twist-like dictionary."""
    return {
        "linear": {
            "x": float(cmd.vx),
            "y": float(cmd.vy),
            "z": float(cmd.vz),
        },
        "angular": {
            "x": 0.0,
            "y": 0.0,
            "z": float(cmd.yaw_rate),
        },
        "duration": float(cmd.duration),
        "mode": cmd.mode.value,
        "metadata": dict(cmd.metadata),
    }


def twist_dict_to_control_command(twist: Dict[str, Any]) -> ControlCommand:
    """Convert a Twist-like dictionary into a :class:`ControlCommand`."""
    linear = twist.get("linear", {})
    angular = twist.get("angular", {})
    mode_value = twist.get("mode", ControlMode.WORLD_MODEL_GUIDED.value)
    try:
        mode = ControlMode(mode_value)
    except ValueError:
        mode = ControlMode.WORLD_MODEL_GUIDED

    return ControlCommand(
        vx=float(linear.get("x", 0.0)),
        vy=float(linear.get("y", 0.0)),
        vz=float(linear.get("z", 0.0)),
        yaw_rate=float(angular.get("z", 0.0)),
        duration=float(twist.get("duration", 0.4)),
        mode=mode,
        metadata=dict(twist.get("metadata", {})),
    )


def sensor_observation_to_odom_dict(obs: SensorObservation) -> Dict[str, Any]:
    """Convert a sensor observation into an Odometry-like dictionary."""
    return {
        "timestamp": float(obs.timestamp),
        "pose": {
            "position": {
                "x": float(obs.pose[0]),
                "y": float(obs.pose[1]),
                "z": float(obs.pose[2]),
            }
        },
        "twist": {
            "linear": {
                "x": float(obs.velocity[0]),
                "y": float(obs.velocity[1]),
                "z": float(obs.velocity[2]),
            }
        },
        "goal_distance": float(obs.goal_distance),
        "obstacle_distance": float(obs.obstacle_distance),
        "metadata": dict(obs.metadata),
    }


def odometry_dict_to_sensor_observation(odom: Dict[str, Any]) -> SensorObservation:
    """Convert an Odometry-like dictionary into a :class:`SensorObservation`."""
    pose = odom.get("pose", {})
    if "pose" in pose:
        pose = pose["pose"]
    position = pose.get("position", {})

    twist = odom.get("twist", {})
    if "twist" in twist:
        twist = twist["twist"]
    linear = twist.get("linear", {})

    timestamp = odom.get("timestamp")
    if timestamp is None:
        stamp = odom.get("header", {}).get("stamp", {})
        timestamp = float(stamp.get("sec", 0.0)) + float(stamp.get("nanosec", 0.0)) * 1e-9

    return SensorObservation(
        timestamp=float(timestamp),
        pose=(
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
        ),
        velocity=(
            float(linear.get("x", 0.0)),
            float(linear.get("y", 0.0)),
            float(linear.get("z", 0.0)),
        ),
        goal_distance=float(odom.get("goal_distance", 0.0)),
        obstacle_distance=float(odom.get("obstacle_distance", 50.0)),
        metadata=dict(odom.get("metadata", {})),
    )
