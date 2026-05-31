"""Inter-agent communication channel abstraction.

Defines a QoS-aware message passing interface inspired by ROS 2 / DDS,
along with a fully in-memory mock implementation suitable for unit tests
and single-process simulations.
"""

from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Message types and data structures
# ---------------------------------------------------------------------------

class MessageType(Enum):
    """Categories of inter-agent messages."""

    STATE_BROADCAST = "state_broadcast"
    MAP_UPDATE = "map_update"
    TASK_ASSIGNMENT = "task_assignment"
    EMERGENCY_ALERT = "emergency_alert"


@dataclass
class AgentMessage:
    """A single message exchanged between agents."""

    sender_id: str
    message_type: MessageType
    payload: Dict[str, Any]
    timestamp: float
    priority: int = 0  # higher = more important


@dataclass
class QoSProfile:
    """Quality of Service profile inspired by ROS 2 DDS QoS settings."""

    reliability: str = "reliable"  # "reliable" or "best_effort"
    deadline_ms: float = 100.0
    latency_budget_ms: float = 50.0
    history_depth: int = 10
    lifespan_sec: float = 5.0


# ---------------------------------------------------------------------------
# Abstract channel interface
# ---------------------------------------------------------------------------

class CommunicationChannel(ABC):
    """Abstract communication channel for inter-agent messaging."""

    @abstractmethod
    def send(self, message: AgentMessage, qos: Optional[QoSProfile] = None) -> bool:
        """Send a message.  Returns True on success."""
        ...

    @abstractmethod
    def receive(
        self, agent_id: str, timeout: float = 0.0
    ) -> List[AgentMessage]:
        """Receive all pending messages for *agent_id*."""
        ...

    @abstractmethod
    def subscribe(
        self, agent_id: str, message_types: List[MessageType]
    ) -> None:
        """Subscribe *agent_id* to specific message types."""
        ...


# ---------------------------------------------------------------------------
# Mock DDS implementation
# ---------------------------------------------------------------------------

class MockDDSChannel(CommunicationChannel):
    """Mock DDS-like communication channel for testing.

    All messages are routed in-memory.  Supports:
    - Per-agent inboxes with bounded history depth (oldest dropped first).
    - Subscription-based message filtering.
    - Optional simulated latency and random message drops.
    - Lifespan-based expiration of queued messages.
    """

    def __init__(
        self,
        default_qos: Optional[QoSProfile] = None,
        simulated_latency_ms: float = 0.0,
        message_drop_rate: float = 0.0,
    ) -> None:
        self.default_qos = default_qos or QoSProfile()
        self.simulated_latency_ms = simulated_latency_ms
        self.message_drop_rate = message_drop_rate

        # agent_id → inbox (list of messages)
        self._inboxes: Dict[str, List[AgentMessage]] = defaultdict(list)
        # agent_id → subscribed message types
        self._subscriptions: Dict[str, Set[MessageType]] = defaultdict(set)
        # Full log of sent messages (useful for test assertions)
        self.sent_log: List[AgentMessage] = []

        logger.info(
            "MockDDSChannel initialised (latency=%.1fms, drop=%.2f).",
            simulated_latency_ms,
            message_drop_rate,
        )

    # ------------------------------------------------------------------
    # CommunicationChannel interface
    # ------------------------------------------------------------------

    def send(
        self, message: AgentMessage, qos: Optional[QoSProfile] = None
    ) -> bool:
        """Broadcast *message* to all subscribers of its message type.

        Returns ``True`` if the message was delivered to at least one
        subscriber (or if there are no subscribers — we still consider
        the send successful at the channel level).
        """
        effective_qos = qos or self.default_qos
        self.sent_log.append(message)

        # Simulate random drop
        if self.message_drop_rate > 0.0:
            import random
            if random.random() < self.message_drop_rate:
                logger.debug(
                    "Simulated drop for message from '%s'.", message.sender_id,
                )
                return False

        delivered = False
        for agent_id, subs in self._subscriptions.items():
            # Don't deliver to the sender
            if agent_id == message.sender_id:
                continue
            if message.message_type in subs:
                inbox = self._inboxes[agent_id]
                inbox.append(message)
                # Enforce history depth
                if len(inbox) > effective_qos.history_depth:
                    self._inboxes[agent_id] = inbox[-effective_qos.history_depth:]
                delivered = True

        logger.debug(
            "Message from '%s' (type=%s) delivered=%s.",
            message.sender_id,
            message.message_type.value,
            delivered,
        )
        return True  # send itself succeeded even if no subscribers

    def receive(
        self, agent_id: str, timeout: float = 0.0
    ) -> List[AgentMessage]:
        """Return and flush all pending messages for *agent_id*.

        The *timeout* parameter is accepted for API compatibility but is
        ignored in this in-memory mock.
        """
        # Expire messages past their lifespan
        now = time.time()
        lifespan = self.default_qos.lifespan_sec
        inbox = [
            m for m in self._inboxes.get(agent_id, [])
            if (now - m.timestamp) <= lifespan
        ]

        # Flush the inbox
        self._inboxes[agent_id] = []

        # Sort by priority descending, then timestamp ascending
        inbox.sort(key=lambda m: (-m.priority, m.timestamp))
        return inbox

    def subscribe(
        self, agent_id: str, message_types: List[MessageType]
    ) -> None:
        """Subscribe *agent_id* to one or more message types."""
        self._subscriptions[agent_id].update(message_types)
        logger.info(
            "Agent '%s' subscribed to %s.",
            agent_id,
            [mt.value for mt in message_types],
        )
