"""Guarded ROS2 bridge lifecycle wrapper."""

from __future__ import annotations

import logging
from typing import Any, Callable, Dict

from src.ros2_bridge.qos_config import QoSConfig, qos_from_config

logger = logging.getLogger(__name__)

try:  # pragma: no cover - exercised only in ROS2 environments
    import rclpy
    from rclpy.duration import Duration as RclpyDuration
    from rclpy.qos import HistoryPolicy, QoSProfile as RclpyQoSProfile, ReliabilityPolicy

    _HAS_RCLPY = True
except ImportError:  # pragma: no cover - default in normal test environments
    rclpy = None  # type: ignore[assignment]
    RclpyDuration = None  # type: ignore[assignment]
    HistoryPolicy = None  # type: ignore[assignment]
    RclpyQoSProfile = None  # type: ignore[assignment]
    ReliabilityPolicy = None  # type: ignore[assignment]
    _HAS_RCLPY = False


class ROS2Bridge:
    """Manage a ROS2 node, publishers, subscriptions, and services.

    Importing this module never requires ROS2. Instantiating the bridge does,
    because it creates a real ``rclpy`` node.
    """

    def __init__(self, node_name: str, config: Dict[str, Any] | None = None) -> None:
        if not _HAS_RCLPY:
            raise RuntimeError(
                "ROS2Bridge requires rclpy. Install ROS2 Humble or inject a mock bridge."
            )

        self.node_name = node_name
        self.config = config or {}
        self._shutdown = False
        self._owns_rclpy_context = False
        self.node = None

        if not rclpy.ok():  # type: ignore[union-attr]
            rclpy.init(args=None)  # type: ignore[union-attr]
            self._owns_rclpy_context = True
        self.node = rclpy.create_node(node_name)  # type: ignore[union-attr]
        logger.info("ROS2Bridge node '%s' created", node_name)

    def create_publisher(
        self,
        topic: str,
        msg_type: Any,
        qos: QoSConfig | Dict[str, Any] | None = None,
    ) -> Any:
        """Create and return an ``rclpy`` publisher handle."""
        return self.node.create_publisher(msg_type, topic, self._to_rclpy_qos(qos))

    def create_subscription(
        self,
        topic: str,
        msg_type: Any,
        callback: Callable[[Any], None],
        qos: QoSConfig | Dict[str, Any] | None = None,
    ) -> Any:
        """Create and return an ``rclpy`` subscription handle."""
        return self.node.create_subscription(msg_type, topic, callback, self._to_rclpy_qos(qos))

    def create_service(
        self,
        service_name: str,
        service_type: Any,
        callback: Callable[..., Any],
    ) -> Any:
        """Create and return an ``rclpy`` service handle."""
        return self.node.create_service(service_type, service_name, callback)

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        """Process one round of ROS2 callbacks."""
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)  # type: ignore[union-attr]

    def shutdown(self) -> None:
        """Destroy this node and shut down ROS2 only if this bridge initialized it."""
        if self._shutdown:
            return
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_rclpy_context and rclpy.ok():  # type: ignore[union-attr]
            rclpy.shutdown()  # type: ignore[union-attr]
        self._shutdown = True
        logger.info("ROS2Bridge node '%s' shut down", self.node_name)

    @property
    def is_shutdown(self) -> bool:
        """Return whether the bridge has been shut down."""
        return self._shutdown

    def _to_rclpy_qos(self, qos: QoSConfig | Dict[str, Any] | None) -> Any:
        qos_config = qos if isinstance(qos, QoSConfig) else qos_from_config(qos)
        profile = RclpyQoSProfile(depth=qos_config.history_depth)
        profile.history = HistoryPolicy.KEEP_LAST
        if qos_config.reliability == "best_effort":
            profile.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            profile.reliability = ReliabilityPolicy.RELIABLE
        self._set_qos_duration(profile, "deadline", qos_config.deadline_ms / 1000.0)
        self._set_qos_duration(profile, "lifespan", qos_config.lifespan_sec)
        return profile

    def _set_qos_duration(self, profile: Any, attr: str, seconds: float) -> None:
        """Set a duration-backed QoS field when the active rclpy supports it."""
        if not hasattr(profile, attr) or RclpyDuration is None:
            return
        duration = self._duration_from_seconds(seconds)
        try:
            setattr(profile, attr, duration)
        except (AttributeError, TypeError, ValueError):  # pragma: no cover - version-specific
            logger.debug("QoSProfile does not support '%s' duration assignment", attr)

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Any:
        """Create an rclpy Duration without exposing rclpy to tests."""
        nanoseconds = int(round(seconds * 1_000_000_000))
        return RclpyDuration(nanoseconds=nanoseconds)
