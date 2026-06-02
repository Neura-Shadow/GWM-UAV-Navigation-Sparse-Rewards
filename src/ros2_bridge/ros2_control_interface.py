"""Mock-first ros2_control-style hardware interface contracts."""

from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from src.control.barrier_certificate import ControlBarrierFunction, SafetyLimits
from src.utils.data_types import ControlCommand, ControlMode

logger = logging.getLogger(__name__)


@dataclass
class HardwareState:
    """Transport-neutral hardware state snapshot."""

    timestamp: float = 0.0
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    armed: bool = False
    mode: str = "mock"
    battery_percent: float = 1.0
    health: str = "nominal"
    metadata: Dict[str, Any] = field(default_factory=dict)


class HardwareInterface(ABC):
    """Abstract read/write contract inspired by ros2_control."""

    def connect(self) -> bool:
        """Connect to the underlying hardware transport."""
        return True

    def disconnect(self) -> None:
        """Disconnect from the underlying hardware transport."""

    @property
    def is_connected(self) -> bool:
        """Return whether the interface can read/write."""
        return True

    @abstractmethod
    def read(self) -> HardwareState:
        """Read the latest hardware state."""

    @abstractmethod
    def write(self, command: ControlCommand) -> bool:
        """Write a low-level command to hardware or mock transport."""

    def emergency_stop(self) -> bool:
        """Write an emergency-stop command."""
        return self.write(ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=0.0,
            mode=ControlMode.EMERGENCY_STOP,
            metadata={"reason": "emergency_stop"},
        ))


class MockHardwareInterface(HardwareInterface):
    """In-memory hardware interface for CI and local development."""

    def __init__(
        self,
        initial_state: Optional[HardwareState] = None,
        limits: Optional[SafetyLimits] = None,
    ) -> None:
        self.limits = limits or SafetyLimits()
        self._cbf = ControlBarrierFunction(limits=self.limits)
        self._state = initial_state or HardwareState(timestamp=time.time())
        self._connected = True
        self.command_history: List[ControlCommand] = []
        self.rejected_commands: List[ControlCommand] = []

    @property
    def is_connected(self) -> bool:
        """Return whether mock writes are accepted."""
        return self._connected

    def connect(self) -> bool:
        """Connect the mock hardware interface."""
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect the mock hardware interface."""
        self._connected = False

    def read(self) -> HardwareState:
        """Return the latest mock hardware state."""
        return self._state

    def write(self, command: ControlCommand) -> bool:
        """Apply safety limits and update mock state."""
        if not self._connected:
            logger.error("MockHardwareInterface: not connected, command rejected.")
            return False

        if not self._command_altitude_allowed(command):
            rejected = _with_reason(command, "altitude_limit")
            self.rejected_commands.append(rejected)
            return False

        state_for_checks = {
            "position": self._state.position,
            "metadata": self._state.metadata,
        }
        if not self._cbf.within_geofence(state_for_checks):
            rejected = _with_reason(command, "geofence_limit")
            self.rejected_commands.append(rejected)
            return False

        safe_command = self._cbf.saturate(command)
        self.command_history.append(safe_command)
        self._state = HardwareState(
            timestamp=time.time(),
            position=self._state.position,
            velocity=(safe_command.vx, safe_command.vy, safe_command.vz),
            armed=self._state.armed,
            mode="mock",
            battery_percent=self._state.battery_percent,
            health=self._state.health,
            metadata=dict(self._state.metadata),
        )
        return True

    def _command_altitude_allowed(self, command: ControlCommand) -> bool:
        if "altitude" not in command.metadata:
            return True
        altitude = float(command.metadata["altitude"])
        return self.limits.min_altitude <= altitude <= self.limits.max_altitude


class ROS2ControlHardwareInterface(HardwareInterface):
    """ROS2 control hardware stub with mock-safe defaults."""

    def __init__(
        self,
        bridge: Optional[Any] = None,
        mock: bool = True,
        config: Optional[Dict[str, Any]] = None,
        real_hardware_enabled: bool = False,
    ) -> None:
        self.bridge = bridge
        self.mock = bool(mock)
        self.config = config or {}
        self.real_hardware_enabled = bool(real_hardware_enabled)
        self._mock_interface = MockHardwareInterface()
        self._connected = False

    @property
    def is_connected(self) -> bool:
        """Return whether this interface is connected."""
        return self._mock_interface.is_connected if self.mock else self._connected

    def connect(self) -> bool:
        """Connect mock mode or guard real hardware mode."""
        if self.mock:
            return self._mock_interface.connect()
        if not self.real_hardware_enabled:
            raise RuntimeError(
                "ROS2ControlHardwareInterface real mode requires "
                "real_hardware_enabled=True."
            )
        if self.bridge is None:
            raise RuntimeError(
                "ROS2ControlHardwareInterface real mode requires an injected ROS2Bridge."
            )
        self._connected = True
        return True

    def disconnect(self) -> None:
        """Disconnect mock or real-stub mode."""
        if self.mock:
            self._mock_interface.disconnect()
        self._connected = False

    def read(self) -> HardwareState:
        """Read hardware state from mock backend in Phase 3-D."""
        if self.mock:
            return self._mock_interface.read()
        if not self._connected:
            raise RuntimeError("ROS2ControlHardwareInterface is not connected.")
        return HardwareState(
            timestamp=time.time(),
            mode="ros2_control_stub",
            metadata={"source": "ros2_control_stub"},
        )

    def write(self, command: ControlCommand) -> bool:
        """Write through mock backend or guarded real stub."""
        if self.mock:
            return self._mock_interface.write(command)
        if not self._connected:
            raise RuntimeError("ROS2ControlHardwareInterface is not connected.")
        logger.info("ROS2 control real write stub received command: %s", command)
        return True


def _with_reason(command: ControlCommand, reason: str) -> ControlCommand:
    metadata = dict(command.metadata)
    metadata["reason"] = reason
    return ControlCommand(
        vx=command.vx,
        vy=command.vy,
        vz=command.vz,
        yaw_rate=command.yaw_rate,
        duration=command.duration,
        mode=command.mode,
        metadata=metadata,
    )
