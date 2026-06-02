"""Phase 3-C tests for distributed multi-agent coordination."""

from __future__ import annotations

import time
from typing import Any, Dict, List

import numpy as np
import pytest

from src.multi_agent import (
    AgentMessage,
    AgentRegistry,
    MessageType,
    MockDDSChannel,
    PriorityCoordinator,
    ROS2DDSChannel,
    SharedLatentMap,
    SharedSpatiotemporalMap,
    SwarmCoordinator,
)
from src.multi_agent.ros2_dds_channel import (
    deserialize_agent_message,
    serialize_agent_message,
)
from src.utils.data_types import AgentState, AgentStatus, SensorObservation


def _agent(
    agent_id: str,
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
    status: AgentStatus = AgentStatus.IDLE,
    metadata: Dict[str, Any] | None = None,
) -> AgentState:
    return AgentState(
        agent_id=agent_id,
        observation=SensorObservation(timestamp=time.time(), pose=pose),
        status=status,
        metadata=metadata or {},
    )


def _registry_with_agents(agents: List[AgentState]) -> AgentRegistry:
    registry = AgentRegistry()
    for agent in agents:
        registry.register(agent)
    return registry


def test_phase3c_symbols_are_exported() -> None:
    """Phase 3-C public symbols import from src.multi_agent."""
    assert ROS2DDSChannel is not None
    assert PriorityCoordinator is not None
    assert SharedLatentMap is not None


def test_ros2_dds_channel_falls_back_to_mock_transport() -> None:
    """ROS2DDSChannel remains usable without ROS2 by routing through a mock."""
    channel = ROS2DDSChannel()
    channel.subscribe("agent_b", [MessageType.STATE_BROADCAST])

    message = AgentMessage(
        sender_id="agent_a",
        message_type=MessageType.STATE_BROADCAST,
        payload={"position": np.array([1.0, 2.0, -1.0])},
        timestamp=time.time(),
    )

    assert channel.send(message) is True
    inbox = channel.receive("agent_b")

    assert channel.using_mock_backend is True
    assert len(inbox) == 1
    assert inbox[0].sender_id == "agent_a"
    assert inbox[0].payload["position"].tolist() == [1.0, 2.0, -1.0]


def test_ros2_dds_channel_serializes_agent_messages_to_json_safe_dicts() -> None:
    """Agent messages serialize and deserialize through JSON-safe payloads."""
    message = AgentMessage(
        sender_id="agent_a",
        message_type=MessageType.MAP_UPDATE,
        payload={
            "latent": np.array([0.1, 0.2, 0.3], dtype=np.float32),
            "pose": (1.0, 2.0, -3.0),
        },
        timestamp=12.5,
        priority=7,
    )

    data = serialize_agent_message(message)
    round_trip = deserialize_agent_message(data)

    assert data["message_type"] == "map_update"
    assert data["payload"]["latent"] == pytest.approx([0.1, 0.2, 0.3])
    assert data["payload"]["pose"] == [1.0, 2.0, -3.0]
    assert round_trip.message_type == MessageType.MAP_UPDATE
    assert round_trip.priority == 7


def test_ros2_dds_channel_uses_deterministic_topic_mapping() -> None:
    """DDS topics are stable and can be overridden by config."""
    channel = ROS2DDSChannel(
        topics={"task_assignment": "/custom/tasks"},
    )

    assert channel.topic_for(MessageType.STATE_BROADCAST) == "/fleet/agent_state"
    assert channel.topic_for(MessageType.TASK_ASSIGNMENT) == "/custom/tasks"


def test_ros2_dds_channel_can_publish_through_fake_bridge() -> None:
    """Injected bridge path is smoke-tested without importing ROS2."""

    class FakePublisher:
        def __init__(self) -> None:
            self.messages: List[Dict[str, Any]] = []

        def publish(self, message: Dict[str, Any]) -> None:
            self.messages.append(message)

    class FakeBridge:
        def __init__(self) -> None:
            self.publishers: Dict[str, FakePublisher] = {}

        def create_publisher(
            self, topic: str, msg_type: object, qos: object | None = None
        ) -> FakePublisher:
            del msg_type, qos
            publisher = FakePublisher()
            self.publishers[topic] = publisher
            return publisher

    bridge = FakeBridge()
    channel = ROS2DDSChannel(bridge=bridge, prefer_ros2=True)
    message = AgentMessage(
        sender_id="coordinator",
        message_type=MessageType.TASK_ASSIGNMENT,
        payload={"task_id": "task_1"},
        timestamp=1.0,
        priority=5,
    )

    assert channel.send(message) is True
    publisher = bridge.publishers["/fleet/task_assignment"]
    assert publisher.messages[0]["payload"]["task_id"] == "task_1"
    assert channel.using_mock_backend is False


def test_priority_coordinator_selects_best_idle_agent() -> None:
    """Priority strategy prefers capable, nearby, high-battery idle agents."""
    registry = _registry_with_agents([
        _agent(
            "uav_far",
            pose=(100.0, 0.0, 0.0),
            metadata={"battery_level": 1.0, "capabilities": ["lidar"]},
        ),
        _agent(
            "uav_best",
            pose=(5.0, 0.0, 0.0),
            metadata={"battery_level": 0.9, "capabilities": ["lidar"]},
        ),
        _agent(
            "uav_busy",
            pose=(0.0, 0.0, 0.0),
            status=AgentStatus.NAVIGATING,
            metadata={"battery_level": 1.0, "capabilities": ["lidar"]},
        ),
    ])
    coordinator = PriorityCoordinator(registry)

    assignments = coordinator.assign_tasks([
        {"task_id": "inspect", "goal": [0.0, 0.0, 0.0], "required_capability": "lidar"}
    ])

    assert assignments == {"uav_best": "inspect"}
    assert registry.get("uav_best").status == AgentStatus.IDLE


def test_priority_coordinator_tie_breaks_by_agent_id() -> None:
    """Equal scores are deterministic."""
    registry = _registry_with_agents([
        _agent("uav_b", pose=(0.0, 0.0, 0.0)),
        _agent("uav_a", pose=(0.0, 0.0, 0.0)),
    ])
    coordinator = PriorityCoordinator(registry)

    assignments = coordinator.assign_tasks([{"task_id": "survey"}])

    assert assignments == {"uav_a": "survey"}


def test_shared_latent_map_updates_queries_and_merges_latents() -> None:
    """SharedLatentMap stores latent vectors and fuses nearby entries."""
    shared_map = SharedLatentMap(latent_dim=3, confidence_decay=1.0)
    shared_map.update_with_latent(
        "uav_1", (0.0, 0.0, 0.0), 1.0, [1.0, 0.0, 0.0], confidence=1.0
    )
    shared_map.update_with_latent(
        "uav_2", (1.0, 0.0, 0.0), 1.0, [0.0, 1.0, 0.0], confidence=3.0
    )

    result = shared_map.query_latents((0.0, 0.0, 0.0), radius=2.0)
    merged = shared_map.merge_latents(result["entries"])

    assert result["nearby_agents"] == ["uav_1", "uav_2"]
    assert result["count"] == 2
    assert merged.tolist() == pytest.approx([0.25, 0.75, 0.0])
    assert shared_map.query((0.0, 0.0, 0.0), radius=2.0)["count"] == 2


def test_shared_latent_map_rejects_wrong_vector_dimension() -> None:
    """Latent dimension mismatches fail clearly."""
    shared_map = SharedLatentMap(latent_dim=3)

    with pytest.raises(ValueError, match="latent_dim"):
        shared_map.update_with_latent(
            "uav_1", (0.0, 0.0, 0.0), 1.0, [1.0, 0.0], confidence=1.0
        )


def test_swarm_coordinator_preserves_round_robin_default_strategy() -> None:
    """Default task assignment behavior remains round-robin."""
    registry = _registry_with_agents([
        _agent("uav_1", pose=(0.0, 0.0, 0.0)),
        _agent("uav_2", pose=(10.0, 0.0, 0.0)),
    ])
    coordinator = SwarmCoordinator(
        registry,
        SharedSpatiotemporalMap(),
        MockDDSChannel(),
    )

    assignments = coordinator.assign_tasks([
        {"task_id": "task_1"},
        {"task_id": "task_2"},
    ])

    assert assignments == {"uav_1": "task_1", "uav_2": "task_2"}


def test_swarm_coordinator_supports_priority_strategy() -> None:
    """Priority strategy plugs into SwarmCoordinator assignment flow."""
    registry = _registry_with_agents([
        _agent("uav_far", pose=(100.0, 0.0, 0.0), metadata={"battery_level": 1.0}),
        _agent("uav_near", pose=(1.0, 0.0, 0.0), metadata={"battery_level": 0.9}),
    ])
    channel = MockDDSChannel()
    coordinator = SwarmCoordinator(
        registry,
        SharedSpatiotemporalMap(),
        channel,
        strategy="priority",
    )

    assignments = coordinator.assign_tasks([
        {"task_id": "near_goal", "goal": [0.0, 0.0, 0.0]},
    ])

    assert assignments == {"uav_near": "near_goal"}
    assert registry.get("uav_near").status == AgentStatus.NAVIGATING
    assert channel.sent_log[-1].message_type == MessageType.TASK_ASSIGNMENT


def test_swarm_coordinator_rejects_unknown_strategy() -> None:
    """Unknown assignment strategies fail before task mutation."""
    with pytest.raises(ValueError, match="Unknown coordination strategy"):
        SwarmCoordinator(
            AgentRegistry(),
            SharedSpatiotemporalMap(),
            MockDDSChannel(),
            strategy="consensus",
        )
