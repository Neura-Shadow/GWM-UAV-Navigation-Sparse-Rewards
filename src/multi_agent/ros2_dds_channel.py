"""Mock-first ROS2/DDS-style channel for distributed agent messages."""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from src.multi_agent.communication import (
    AgentMessage,
    CommunicationChannel,
    MessageType,
    MockDDSChannel,
    QoSProfile,
)

logger = logging.getLogger(__name__)


DEFAULT_DDS_TOPICS: Dict[MessageType, str] = {
    MessageType.STATE_BROADCAST: "/fleet/agent_state",
    MessageType.MAP_UPDATE: "/fleet/map_update",
    MessageType.TASK_ASSIGNMENT: "/fleet/task_assignment",
    MessageType.EMERGENCY_ALERT: "/fleet/emergency_alert",
}


def serialize_agent_message(message: AgentMessage) -> Dict[str, Any]:
    """Convert an AgentMessage into a JSON-safe dictionary."""
    return {
        "sender_id": message.sender_id,
        "message_type": message.message_type.value,
        "payload": _json_safe(message.payload),
        "timestamp": float(message.timestamp),
        "priority": int(message.priority),
    }


def deserialize_agent_message(raw_message: Any) -> AgentMessage:
    """Convert a JSON string, ROS-like message, or dict into AgentMessage."""
    data = _message_payload(raw_message)
    if isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict):
        raise ValueError("AgentMessage payload must be a dict or JSON object.")

    message_type = data.get("message_type")
    if isinstance(message_type, MessageType):
        parsed_type = message_type
    else:
        parsed_type = MessageType(str(message_type))

    return AgentMessage(
        sender_id=str(data["sender_id"]),
        message_type=parsed_type,
        payload=dict(data.get("payload", {})),
        timestamp=float(data["timestamp"]),
        priority=int(data.get("priority", 0)),
    )


class ROS2DDSChannel(CommunicationChannel):
    """CommunicationChannel with optional ROS2 bridge transport.

    Without an injected bridge, all traffic goes through ``MockDDSChannel``.
    With a bridge, outgoing messages are serialized and published to mapped
    DDS topics.  Incoming bridge subscriptions feed the local mock inbox so
    callers still use the same ``receive(agent_id)`` API.
    """

    def __init__(
        self,
        agent_id: str = "coordinator",
        bridge: Optional[Any] = None,
        fallback_channel: Optional[CommunicationChannel] = None,
        topics: Optional[Dict[Any, str]] = None,
        default_qos: Optional[QoSProfile] = None,
        prefer_ros2: Optional[bool] = None,
    ) -> None:
        self.agent_id = agent_id
        self.bridge = bridge
        self.default_qos = default_qos or QoSProfile()
        self._fallback = fallback_channel or MockDDSChannel(default_qos=self.default_qos)
        self._prefer_ros2 = bridge is not None if prefer_ros2 is None else prefer_ros2
        self._topics = dict(DEFAULT_DDS_TOPICS)
        self._topics.update(_normalize_topics(topics or {}))
        self._publishers: Dict[MessageType, Any] = {}
        self._subscriptions: Dict[MessageType, Any] = {}
        self._ros_message_type = _resolve_ros_message_type()

    @property
    def using_mock_backend(self) -> bool:
        """Return True when sends use the in-memory fallback channel."""
        return not self._can_publish_with_bridge

    def topic_for(self, message_type: MessageType) -> str:
        """Return the DDS topic configured for a message type."""
        return self._topics[message_type]

    def send(
        self,
        message: AgentMessage,
        qos: Optional[QoSProfile] = None,
    ) -> bool:
        """Send a message through ROS2 when available, otherwise mock DDS."""
        if self._can_publish_with_bridge:
            try:
                self._publish_to_bridge(message, qos=qos)
                return True
            except Exception as exc:  # pragma: no cover - defensive fallback
                logger.warning("ROS2DDSChannel bridge publish failed: %s", exc)

        return self._fallback.send(message, qos=qos)

    def receive(self, agent_id: str, timeout: float = 0.0) -> List[AgentMessage]:
        """Receive pending messages for an agent."""
        return self._fallback.receive(agent_id, timeout=timeout)

    def subscribe(
        self,
        agent_id: str,
        message_types: List[MessageType],
    ) -> None:
        """Subscribe an agent to message types on the fallback and bridge."""
        self._fallback.subscribe(agent_id, message_types)
        if not self._can_subscribe_with_bridge:
            return

        for message_type in message_types:
            if message_type in self._subscriptions:
                continue
            topic = self.topic_for(message_type)
            callback = self._make_bridge_callback()
            self._subscriptions[message_type] = self.bridge.create_subscription(
                topic,
                self._ros_message_type,
                callback,
                qos=None,
            )

    @property
    def _can_publish_with_bridge(self) -> bool:
        return bool(
            self._prefer_ros2
            and self.bridge is not None
            and hasattr(self.bridge, "create_publisher")
        )

    @property
    def _can_subscribe_with_bridge(self) -> bool:
        return bool(
            self._prefer_ros2
            and self.bridge is not None
            and hasattr(self.bridge, "create_subscription")
        )

    def _publish_to_bridge(
        self,
        message: AgentMessage,
        qos: Optional[QoSProfile] = None,
    ) -> None:
        message_type = message.message_type
        publisher = self._publishers.get(message_type)
        if publisher is None:
            publisher = self.bridge.create_publisher(
                self.topic_for(message_type),
                self._ros_message_type,
                qos=qos or self.default_qos,
            )
            self._publishers[message_type] = publisher

        payload = serialize_agent_message(message)
        publisher.publish(_to_ros_message(payload, self._ros_message_type))

    def _make_bridge_callback(self) -> Any:
        def _callback(raw_message: Any) -> None:
            message = deserialize_agent_message(raw_message)
            self._fallback.send(message, qos=self.default_qos)

        return _callback


def _normalize_topics(topics: Dict[Any, str]) -> Dict[MessageType, str]:
    normalized: Dict[MessageType, str] = {}
    for key, topic in topics.items():
        if isinstance(key, MessageType):
            message_type = key
        else:
            message_type = MessageType(str(key))
        normalized[message_type] = topic
    return normalized


def _message_payload(raw_message: Any) -> Any:
    if hasattr(raw_message, "data"):
        return raw_message.data
    if isinstance(raw_message, bytes):
        return raw_message.decode("utf-8")
    return raw_message


def _to_ros_message(payload: Dict[str, Any], msg_type: Any) -> Any:
    if msg_type is dict:
        return payload

    json_payload = json.dumps(payload)
    message = msg_type()
    if hasattr(message, "data"):
        message.data = json_payload
        return message
    return payload


def _resolve_ros_message_type() -> Any:
    try:
        from std_msgs.msg import String  # type: ignore

        return String
    except Exception:
        return dict


def _json_safe(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(v) for v in value]
    if hasattr(value, "tolist"):
        return _json_safe(value.tolist())
    if hasattr(value, "item"):
        try:
            return value.item()
        except ValueError:
            pass
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)
