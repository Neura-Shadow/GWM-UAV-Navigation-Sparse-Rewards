"""Mock-first MAVLink bridge for deployment interface testing."""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

from src.utils.data_types import ControlCommand, ControlMode

logger = logging.getLogger(__name__)

try:  # pragma: no cover - exercised only when MAVSDK is installed
    from mavsdk import System as MAVSDKSystem  # type: ignore

    _HAS_MAVSDK = True
except ImportError:  # pragma: no cover - default test environment
    MAVSDKSystem = None  # type: ignore[assignment]
    _HAS_MAVSDK = False


class MAVLinkBridge:
    """MAVLink-shaped deployment bridge with a CI-safe mock backend."""

    def __init__(
        self,
        connection_url: str = "udp://:14540",
        autopilot: str = "px4",
        mock: bool = True,
        config: Optional[Dict[str, Any]] = None,
        client: Optional[Any] = None,
        real_hardware_enabled: bool = False,
    ) -> None:
        self.connection_url = connection_url
        self.autopilot = _normalize_autopilot(autopilot)
        self.mock = bool(mock)
        self.config = config or {}
        self.client = client
        self.real_hardware_enabled = bool(real_hardware_enabled)
        self.command_history: List[Dict[str, Any]] = []
        self._connected = False
        self._armed = False

    @property
    def is_connected(self) -> bool:
        """Return whether this bridge is connected."""
        return self._connected

    async def connect(self) -> bool:
        """Connect to mock or optional real MAVSDK backend."""
        if self.mock:
            self._connected = True
            self._record("connect", connection_url=self.connection_url)
            return True

        self._ensure_real_mode_allowed()
        if self.client is None:
            if not _HAS_MAVSDK:
                raise RuntimeError(
                    "MAVLinkBridge real mode requires MAVSDK or an injected client."
                )
            self.client = MAVSDKSystem()
        if hasattr(self.client, "connect"):
            result = self.client.connect(system_address=self.connection_url)
            if hasattr(result, "__await__"):
                await result
        self._connected = True
        self._record("connect", connection_url=self.connection_url)
        return True

    async def disconnect(self) -> None:
        """Disconnect the bridge."""
        if not self.mock and self.client is not None and hasattr(self.client, "close"):
            result = self.client.close()
            if hasattr(result, "__await__"):
                await result
        self._connected = False
        self._armed = False
        self._record("disconnect")

    async def arm(self) -> bool:
        """Arm the vehicle in mock mode or through an injected client."""
        self._require_connected()
        if not self.mock and hasattr(self.client, "action"):
            result = self.client.action.arm()
            if hasattr(result, "__await__"):
                await result
        self._armed = True
        self._record("arm")
        return True

    async def takeoff(self, altitude: float) -> bool:
        """Record or request takeoff to the requested altitude."""
        self._require_connected()
        if altitude <= 0.0:
            raise ValueError("takeoff altitude must be positive.")
        if not self.mock and hasattr(self.client, "action"):
            if hasattr(self.client.action, "set_takeoff_altitude"):
                result = self.client.action.set_takeoff_altitude(float(altitude))
                if hasattr(result, "__await__"):
                    await result
            result = self.client.action.takeoff()
            if hasattr(result, "__await__"):
                await result
        self._record("takeoff", altitude=float(altitude))
        return True

    async def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ) -> bool:
        """Send or record a velocity command."""
        self._require_connected()
        command = ControlCommand(
            vx=float(vx),
            vy=float(vy),
            vz=float(vz),
            yaw_rate=float(yaw_rate),
        )
        payload = self.command_to_mavlink(command)
        self._record("send_velocity", command=payload)
        return True

    async def land(self) -> bool:
        """Record or request a safe landing command."""
        self._require_connected()
        if not self.mock and hasattr(self.client, "action"):
            result = self.client.action.land()
            if hasattr(result, "__await__"):
                await result
        self._record("land", command={"command": "land"})
        return True

    async def emergency_stop(self) -> bool:
        """Record an emergency stop command."""
        self._require_connected()
        command = ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=0.0,
            mode=ControlMode.EMERGENCY_STOP,
            metadata={"reason": "emergency_stop"},
        )
        self._record("emergency_stop", command=self.command_to_mavlink(command))
        return True

    def command_to_mavlink(self, command: ControlCommand) -> Dict[str, Any]:
        """Convert a ControlCommand into a MAVLink-like command dictionary."""
        command_name = "set_velocity"
        if command.mode == ControlMode.EMERGENCY_STOP:
            command_name = "emergency_stop"
        elif command.metadata.get("reason") == "safe_land":
            command_name = "land"

        return {
            "autopilot": self.autopilot,
            "command": command_name,
            "frame": "body_ned",
            "velocity": {
                "vx": float(command.vx),
                "vy": float(command.vy),
                "vz": float(command.vz),
            },
            "yaw_rate": float(command.yaw_rate),
            "duration": float(command.duration),
            "mode": command.mode.value,
            "metadata": dict(command.metadata),
        }

    def _ensure_real_mode_allowed(self) -> None:
        if not self.real_hardware_enabled:
            raise RuntimeError(
                "MAVLinkBridge real mode requires real_hardware_enabled=True."
            )

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("MAVLinkBridge is not connected.")

    def _record(self, action: str, **kwargs: Any) -> None:
        entry = {
            "timestamp": time.time(),
            "action": action,
            "mock": self.mock,
            "autopilot": self.autopilot,
        }
        entry.update(kwargs)
        self.command_history.append(entry)
        logger.debug("MAVLinkBridge action recorded: %s", action)


def _normalize_autopilot(value: str) -> str:
    normalized = value.strip().lower()
    aliases = {
        "px4": "px4",
        "ardupilot": "ardupilot",
        "ardu_pilot": "ardupilot",
        "apm": "ardupilot",
    }
    if normalized not in aliases:
        valid = ", ".join(sorted(set(aliases.values())))
        raise ValueError(f"Unsupported autopilot '{value}'. Expected one of: {valid}")
    return aliases[normalized]
