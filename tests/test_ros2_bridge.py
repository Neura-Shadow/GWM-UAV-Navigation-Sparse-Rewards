"""Tests for the Phase 3-A ROS2 bridge layer.

These tests must pass without ROS2, rclpy, or ROS message packages installed.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

import src.ros2_bridge.ros2_bridge as ros2_bridge_module
from src.control.ros2_adapter import MockROS2Adapter
from src.ros2_bridge import QoSConfig, ROS2Bridge, RealROS2Adapter
from src.ros2_bridge.msg_converters import (
    control_command_to_twist_dict,
    odometry_dict_to_sensor_observation,
    sensor_observation_to_odom_dict,
    twist_dict_to_control_command,
)
from src.ros2_bridge.qos_config import qos_from_config
from src.ros2_bridge.ros2_bridge import _HAS_RCLPY
from src.utils.data_types import ControlCommand, ControlMode, SensorObservation


class _FakePublisher:
    def __init__(self) -> None:
        self.published = []

    def publish(self, message) -> None:
        self.published.append(message)


class _FakeBridge:
    def __init__(self) -> None:
        self.publisher = _FakePublisher()
        self.publisher_args = None
        self.subscription_args = None
        self.subscription_callback = None
        self.is_shutdown = False
        self.shutdown_called = False

    def create_publisher(self, topic, msg_type, qos=None):
        self.publisher_args = (topic, msg_type, qos)
        return self.publisher

    def create_subscription(self, topic, msg_type, callback, qos=None):
        self.subscription_args = (topic, msg_type, qos)
        self.subscription_callback = callback
        return object()

    def shutdown(self) -> None:
        self.shutdown_called = True
        self.is_shutdown = True


class _FakeNode:
    def __init__(self, node_name: str) -> None:
        self.node_name = node_name
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


class _FakeRclpy:
    def __init__(self, ok: bool) -> None:
        self._ok = ok
        self.init_calls = 0
        self.shutdown_calls = 0
        self.nodes = []

    def ok(self) -> bool:
        return self._ok

    def init(self, args=None) -> None:
        self.init_calls += 1
        self._ok = True

    def create_node(self, node_name: str) -> _FakeNode:
        node = _FakeNode(node_name)
        self.nodes.append(node)
        return node

    def shutdown(self) -> None:
        self.shutdown_calls += 1
        self._ok = False


class _FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class _FakeQoSProfile:
    def __init__(self, depth: int) -> None:
        self.depth = depth
        self.history = None
        self.reliability = None
        self.deadline = None
        self.lifespan = None


def test_package_exports_public_interfaces() -> None:
    assert ROS2Bridge is not None
    assert RealROS2Adapter is not None
    assert QoSConfig is not None


def test_qos_config_normalizes_reliability() -> None:
    qos = qos_from_config(
        {
            "reliability": "BEST_EFFORT",
            "history_depth": 3,
            "deadline_ms": 25,
            "lifespan_sec": 2,
        }
    )

    assert qos.reliability == "best_effort"
    assert qos.history_depth == 3
    assert qos.deadline_ms == pytest.approx(25.0)
    assert qos.lifespan_sec == pytest.approx(2.0)


def test_qos_config_rejects_invalid_reliability() -> None:
    with pytest.raises(ValueError, match="Unsupported QoS reliability"):
        QoSConfig(reliability="sometimes")


def test_control_command_twist_round_trip() -> None:
    command = ControlCommand(
        vx=1.2,
        vy=-0.3,
        vz=0.5,
        yaw_rate=0.1,
        duration=0.8,
        mode=ControlMode.SAFETY_OVERRIDE,
        metadata={"source": "unit-test"},
    )

    twist = control_command_to_twist_dict(command)
    result = twist_dict_to_control_command(twist)

    assert result.vx == pytest.approx(command.vx)
    assert result.vy == pytest.approx(command.vy)
    assert result.vz == pytest.approx(command.vz)
    assert result.yaw_rate == pytest.approx(command.yaw_rate)
    assert result.duration == pytest.approx(command.duration)
    assert result.mode == ControlMode.SAFETY_OVERRIDE
    assert result.metadata == {"source": "unit-test"}


def test_sensor_observation_odom_round_trip() -> None:
    observation = SensorObservation(
        timestamp=42.5,
        pose=(1.0, 2.0, -3.0),
        velocity=(0.1, 0.2, 0.3),
        goal_distance=12.0,
        obstacle_distance=4.5,
        metadata={"source": "mock"},
    )

    odom = sensor_observation_to_odom_dict(observation)
    result = odometry_dict_to_sensor_observation(odom)

    assert result.timestamp == pytest.approx(observation.timestamp)
    assert result.pose == observation.pose
    assert result.velocity == observation.velocity
    assert result.goal_distance == pytest.approx(observation.goal_distance)
    assert result.obstacle_distance == pytest.approx(observation.obstacle_distance)
    assert result.metadata == {"source": "mock"}


def test_ros2_bridge_fails_gracefully_without_rclpy() -> None:
    if _HAS_RCLPY:
        pytest.skip("rclpy is installed in this environment")

    with pytest.raises(RuntimeError, match="rclpy"):
        ROS2Bridge("test_node")


def test_ros2_bridge_shutdown_stops_owned_rclpy_context(monkeypatch) -> None:
    fake_rclpy = _FakeRclpy(ok=False)
    monkeypatch.setattr(ros2_bridge_module, "_HAS_RCLPY", True)
    monkeypatch.setattr(ros2_bridge_module, "rclpy", fake_rclpy)

    bridge = ROS2Bridge("owned_context")
    bridge.shutdown()

    assert fake_rclpy.init_calls == 1
    assert fake_rclpy.shutdown_calls == 1
    assert fake_rclpy.nodes[0].destroyed is True
    assert bridge.is_shutdown is True


def test_ros2_bridge_shutdown_preserves_external_rclpy_context(monkeypatch) -> None:
    fake_rclpy = _FakeRclpy(ok=True)
    monkeypatch.setattr(ros2_bridge_module, "_HAS_RCLPY", True)
    monkeypatch.setattr(ros2_bridge_module, "rclpy", fake_rclpy)

    bridge = ROS2Bridge("external_context")
    bridge.shutdown()

    assert fake_rclpy.init_calls == 0
    assert fake_rclpy.shutdown_calls == 0
    assert fake_rclpy.nodes[0].destroyed is True
    assert bridge.is_shutdown is True


def test_ros2_bridge_qos_conversion_applies_depth_reliability_and_durations(monkeypatch) -> None:
    monkeypatch.setattr(ros2_bridge_module, "RclpyQoSProfile", _FakeQoSProfile)
    monkeypatch.setattr(
        ros2_bridge_module,
        "HistoryPolicy",
        SimpleNamespace(KEEP_LAST="keep_last"),
    )
    monkeypatch.setattr(
        ros2_bridge_module,
        "ReliabilityPolicy",
        SimpleNamespace(BEST_EFFORT="best_effort", RELIABLE="reliable"),
    )
    monkeypatch.setattr(ros2_bridge_module, "RclpyDuration", _FakeDuration)

    bridge = ROS2Bridge.__new__(ROS2Bridge)
    profile = bridge._to_rclpy_qos(
        QoSConfig(
            reliability="best_effort",
            history_depth=7,
            deadline_ms=125.0,
            lifespan_sec=2.5,
        )
    )

    assert profile.depth == 7
    assert profile.history == "keep_last"
    assert profile.reliability == "best_effort"
    assert profile.deadline.nanoseconds == 125_000_000
    assert profile.lifespan.nanoseconds == 2_500_000_000


def test_real_ros2_adapter_fails_gracefully_without_rclpy() -> None:
    if _HAS_RCLPY:
        pytest.skip("rclpy is installed in this environment")

    with pytest.raises(RuntimeError, match="rclpy"):
        RealROS2Adapter()


def test_mock_ros2_adapter_lifecycle_helpers() -> None:
    adapter = MockROS2Adapter()

    assert adapter.is_connected is True
    adapter.disconnect()
    assert adapter.is_connected is False
    assert adapter.send_command(ControlCommand(vx=1.0)) is False

    assert adapter.connect() is True
    assert adapter.is_connected is True
    assert adapter.send_command(ControlCommand(vx=1.0)) is True


def test_real_ros2_adapter_prefers_new_odom_topic_key() -> None:
    bridge = _FakeBridge()
    adapter = RealROS2Adapter(
        config={
            "ros2": {
                "topics": {
                    "cmd_vel": "/cmd",
                    "odom": "/new_odom",
                    "odometry": "/legacy_odom",
                },
            }
        },
        bridge=bridge,
    )

    assert adapter.odom_topic == "/new_odom"
    assert bridge.subscription_args[0] == "/new_odom"


def test_real_ros2_adapter_supports_legacy_odometry_topic_key() -> None:
    bridge = _FakeBridge()
    adapter = RealROS2Adapter(
        config={"ros2": {"topics": {"cmd_vel": "/cmd", "odometry": "/legacy_odom"}}},
        bridge=bridge,
    )

    assert adapter.odom_topic == "/legacy_odom"
    assert bridge.subscription_args[0] == "/legacy_odom"


def test_real_ros2_adapter_defaults_odom_topic_when_missing() -> None:
    bridge = _FakeBridge()
    adapter = RealROS2Adapter(config={"ros2": {"topics": {"cmd_vel": "/cmd"}}}, bridge=bridge)

    assert adapter.odom_topic == "/odom"
    assert bridge.subscription_args[0] == "/odom"


def test_real_ros2_adapter_with_injected_bridge_uses_configured_topics() -> None:
    bridge = _FakeBridge()
    adapter = RealROS2Adapter(
        config={
            "ros2": {
                "topics": {"cmd_vel": "/uav/cmd_vel", "odom": "/uav/odom"},
                "qos_profiles": {
                    "control_commands": {"reliability": "RELIABLE"},
                    "odometry": {"reliability": "BEST_EFFORT"},
                },
            }
        },
        bridge=bridge,
    )

    assert adapter.is_connected is True
    assert bridge.publisher_args[0] == "/uav/cmd_vel"
    assert bridge.subscription_args[0] == "/uav/odom"

    assert adapter.send_command(ControlCommand(vx=2.0, yaw_rate=0.25)) is True
    published = bridge.publisher.published[0]
    if isinstance(published, dict):
        assert published["linear"]["x"] == pytest.approx(2.0)
        assert published["angular"]["z"] == pytest.approx(0.25)
    else:
        assert published.linear.x == pytest.approx(2.0)
        assert published.angular.z == pytest.approx(0.25)

    bridge.subscription_callback(
        {
            "timestamp": 10.0,
            "pose": {"position": {"x": 1.0, "y": 2.0, "z": -1.0}},
            "twist": {"linear": {"x": 0.1, "y": 0.2, "z": 0.3}},
        }
    )
    odom = adapter.get_odometry()

    assert odom is not None
    assert odom.pose == (1.0, 2.0, -1.0)
    assert odom.velocity == (0.1, 0.2, 0.3)
