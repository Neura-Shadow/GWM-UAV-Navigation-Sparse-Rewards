"""Multi-agent module for the World-Model-Guided Digital-Twin UAV Navigation Framework.

Axis 4 — Agent state registry, shared spatiotemporal map, QoS-aware
inter-agent communication, and swarm coordination (task allocation +
collision avoidance).
"""

from src.multi_agent.agent_state import AgentRegistry
from src.multi_agent.communication import (
    AgentMessage,
    CommunicationChannel,
    MessageType,
    MockDDSChannel,
    QoSProfile,
)
from src.multi_agent.shared_map import SharedSpatiotemporalMap
from src.multi_agent.swarm_coordinator import SwarmCoordinator

__all__ = [
    "AgentMessage",
    "AgentRegistry",
    "CommunicationChannel",
    "MessageType",
    "MockDDSChannel",
    "QoSProfile",
    "SharedSpatiotemporalMap",
    "SwarmCoordinator",
]
