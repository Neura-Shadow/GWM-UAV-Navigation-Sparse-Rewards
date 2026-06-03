"""Mock-first MAVLink / MAVSDK bridge for deployment interface testing."""

from __future__ import annotations

import importlib
import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from src.control.barrier_certificate import ControlBarrierFunction, SafetyLimits
from src.utils.data_types import ControlCommand, ControlMode

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MAVSDKSITLConfig:
    """Configuration for guarded MAVSDK / PX4 SITL connections."""

    connection_url: str = "udp://:14540"
    autopilot: str = "px4"
    takeoff_altitude_m: float = 5.0
    command_frame: str = "body_ned"
    offboard_initial_setpoint_required: bool = True
    health_timeout_sec: float = 10.0
    mock: bool = True
    sitl_enabled: bool = False
    real_hardware_enabled: bool = False
    autonomous_real_flight_enabled: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "autopilot", _normalize_autopilot(self.autopilot))
        object.__setattr__(self, "command_frame", str(self.command_frame).strip().lower())
        if self.command_frame != "body_ned":
            raise ValueError("Phase 4-E supports command_frame='body_ned' only.")
        if self.takeoff_altitude_m <= 0.0:
            raise ValueError("takeoff_altitude_m must be positive.")
        if self.health_timeout_sec <= 0.0:
            raise ValueError("health_timeout_sec must be positive.")

    @classmethod
    def from_config(cls, config: Dict[str, Any] | None) -> "MAVSDKSITLConfig":
        """Create a SITL config from direct or repository deployment config."""
        if config is None:
            return cls()

        source = dict(config)
        deployment = dict(source.get("deployment") or {})
        mavlink = dict(source.get("mavlink") or {})
        if not deployment and not mavlink:
            mavlink = source

        return cls(
            connection_url=str(mavlink.get("connection_url", "udp://:14540")),
            autopilot=str(mavlink.get("autopilot", "px4")),
            takeoff_altitude_m=float(mavlink.get("takeoff_altitude_m", 5.0)),
            command_frame=str(mavlink.get("command_frame", "body_ned")),
            offboard_initial_setpoint_required=bool(
                mavlink.get("offboard_initial_setpoint_required", True)
            ),
            health_timeout_sec=float(mavlink.get("health_timeout_sec", 10.0)),
            mock=bool(deployment.get("mock", mavlink.get("mock", True))),
            sitl_enabled=bool(
                deployment.get("sitl_enabled", mavlink.get("sitl_enabled", False))
            ),
            real_hardware_enabled=bool(
                deployment.get(
                    "real_hardware_enabled",
                    mavlink.get("real_hardware_enabled", False),
                )
            ),
            autonomous_real_flight_enabled=bool(
                deployment.get(
                    "autonomous_real_flight_enabled",
                    mavlink.get("autonomous_real_flight_enabled", False),
                )
            ),
        )


@dataclass
class _VelocityBody:
    """Tiny MAVSDK-like velocity body payload used when MAVSDK types are absent."""

    forward_m_s: float
    right_m_s: float
    down_m_s: float
    yawspeed_deg_s: float


class MAVLinkBridge:
    """MAVLink-shaped deployment bridge with CI-safe mock and SITL paths."""

    def __init__(
        self,
        connection_url: str = "udp://:14540",
        autopilot: str = "px4",
        mock: bool = True,
        config: Optional[Dict[str, Any]] = None,
        client: Optional[Any] = None,
        real_hardware_enabled: bool = False,
        sitl_enabled: bool = False,
        autonomous_real_flight_enabled: bool = False,
        sitl_config: Optional[MAVSDKSITLConfig] = None,
        safety_filter: Optional[ControlBarrierFunction] = None,
        safety_limits: Optional[SafetyLimits] = None,
    ) -> None:
        parsed_config = sitl_config or MAVSDKSITLConfig.from_config(config)
        self.sitl_config = parsed_config
        self.connection_url = connection_url
        if config is not None or sitl_config is not None:
            self.connection_url = parsed_config.connection_url
        if config is not None or sitl_config is not None:
            autopilot_value = parsed_config.autopilot
        else:
            autopilot_value = autopilot
        self.autopilot = _normalize_autopilot(autopilot_value)
        self.mock = bool(parsed_config.mock if config is not None else mock)
        self.config = config or {}
        self.client = client
        self.real_hardware_enabled = bool(
            real_hardware_enabled or parsed_config.real_hardware_enabled
        )
        self.autonomous_real_flight_enabled = bool(
            autonomous_real_flight_enabled or parsed_config.autonomous_real_flight_enabled
        )
        self._sitl_enabled = bool(sitl_enabled or parsed_config.sitl_enabled)
        self.command_frame = parsed_config.command_frame
        self.offboard_initial_setpoint_required = parsed_config.offboard_initial_setpoint_required
        self.takeoff_altitude_m = parsed_config.takeoff_altitude_m
        self.health_timeout_sec = parsed_config.health_timeout_sec
        self._cbf = safety_filter or ControlBarrierFunction(limits=safety_limits or SafetyLimits())
        self.command_history: List[Dict[str, Any]] = []
        self._connected = False
        self._armed = False
        self._offboard = False
        self._initial_setpoint_sent = False
        self._mavsdk_velocity_body_type: Any | None = None

    @property
    def is_connected(self) -> bool:
        """Return whether this bridge is connected."""
        return self._connected

    @property
    def is_sitl_enabled(self) -> bool:
        """Return whether guarded SITL mode is enabled."""
        return self._sitl_enabled

    @property
    def is_offboard(self) -> bool:
        """Return whether offboard mode has been started."""
        return self._offboard

    async def connect(self) -> bool:
        """Connect to mock or optional guarded MAVSDK/PX4 SITL backend."""
        if self.mock:
            self._connected = True
            self._record("connect", connection_url=self.connection_url)
            return True

        self._ensure_sitl_mode_allowed()
        if self.client is None:
            system_cls = _load_mavsdk_system()
            if system_cls is None:
                raise RuntimeError(
                    "MAVLinkBridge SITL mode requires MAVSDK or an injected client."
                )
            self.client = system_cls()
            self._mavsdk_velocity_body_type = _load_mavsdk_velocity_body_type()
        if hasattr(self.client, "connect"):
            result = self.client.connect(system_address=self.connection_url)
            if hasattr(result, "__await__"):
                await result
        self._connected = True
        self._record(
            "connect",
            connection_url=self.connection_url,
            sitl_enabled=self._sitl_enabled,
        )
        return True

    async def disconnect(self) -> None:
        """Disconnect the bridge."""
        if not self.mock and self.client is not None and hasattr(self.client, "close"):
            result = self.client.close()
            if hasattr(result, "__await__"):
                await result
        self._connected = False
        self._armed = False
        self._offboard = False
        self._initial_setpoint_sent = False
        self._record("disconnect")

    async def wait_until_ready(self, timeout_sec: float = 10.0) -> bool:
        """Wait for a fake/real MAVSDK client to report connection and health."""
        self._require_connected()
        if self.mock:
            self._record("wait_until_ready", ready=True, mock=True)
            return True

        ready = await self._client_reports_ready(timeout_sec)
        self._record("wait_until_ready", ready=ready, timeout_sec=float(timeout_sec))
        if not ready:
            raise RuntimeError("MAVSDK/PX4 SITL did not become ready before timeout.")
        return True

    async def arm(self) -> bool:
        """Arm the vehicle in mock mode or through a MAVSDK-like client."""
        self._require_connected()
        if not self.mock and hasattr(self.client, "action"):
            result = self.client.action.arm()
            if hasattr(result, "__await__"):
                await result
        self._armed = True
        self._record("arm")
        return True

    async def takeoff(self, altitude: float | None = None) -> bool:
        """Record or request takeoff to the requested altitude."""
        self._require_connected()
        altitude = self.takeoff_altitude_m if altitude is None else float(altitude)
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

    async def start_offboard(self, initial_command: ControlCommand | None = None) -> bool:
        """Start offboard mode after sending an initial setpoint."""
        self._require_connected()
        if initial_command is not None:
            await self.send_command(
                initial_command,
                history_action="send_initial_setpoint",
                require_offboard=False,
            )
        if self.offboard_initial_setpoint_required and not self._initial_setpoint_sent:
            raise RuntimeError("Cannot start offboard before sending an initial setpoint.")

        if not self.mock and hasattr(self.client, "offboard"):
            result = self.client.offboard.start()
            if hasattr(result, "__await__"):
                await result
        self._offboard = True
        self._record("start_offboard", initial_setpoint_sent=self._initial_setpoint_sent)
        return True

    async def stop_offboard(self) -> bool:
        """Stop offboard mode in mock or MAVSDK-like mode."""
        self._require_connected()
        if not self.mock and hasattr(self.client, "offboard"):
            result = self.client.offboard.stop()
            if hasattr(result, "__await__"):
                await result
        self._offboard = False
        self._record("stop_offboard")
        return True

    async def send_command(
        self,
        command: ControlCommand,
        *,
        history_action: str = "send_command",
        require_offboard: bool = True,
    ) -> bool:
        """Apply the safety filter and send or record a velocity command."""
        self._require_connected()
        if require_offboard and not self.mock and not self._offboard:
            raise RuntimeError("Cannot send MAVSDK velocity command before offboard starts.")

        safe_command = self._cbf.saturate(command)
        payload = self.command_to_mavlink(safe_command)
        velocity_body = self._velocity_body_payload(payload)

        if not self.mock and hasattr(self.client, "offboard"):
            result = self.client.offboard.set_velocity_body(velocity_body)
            if hasattr(result, "__await__"):
                await result
        self._initial_setpoint_sent = True
        self._record(history_action, command=payload, sitl_enabled=self._sitl_enabled)
        return True

    async def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ) -> bool:
        """Send or record a velocity command."""
        command = ControlCommand(
            vx=float(vx),
            vy=float(vy),
            vz=float(vz),
            yaw_rate=float(yaw_rate),
        )
        return await self.send_command(
            command,
            history_action="send_velocity",
            require_offboard=not self.mock,
        )

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
        """Send zero velocity and record an emergency stop command."""
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
        payload = self.command_to_mavlink(command)
        if not self.mock and self._offboard and hasattr(self.client, "offboard"):
            result = self.client.offboard.set_velocity_body(self._velocity_body_payload(payload))
            if hasattr(result, "__await__"):
                await result
        self._record("emergency_stop", command=payload)
        return True

    async def hold(self) -> bool:
        """Hold position when supported, otherwise record a placeholder."""
        self._require_connected()
        called_client = False
        if not self.mock and hasattr(self.client, "action") and hasattr(self.client.action, "hold"):
            result = self.client.action.hold()
            if hasattr(result, "__await__"):
                await result
            called_client = True
        self._record("hold", placeholder=not called_client)
        return True

    async def return_to_launch(self) -> bool:
        """Request RTL when supported, otherwise record a placeholder."""
        self._require_connected()
        called_client = False
        if (
            not self.mock
            and hasattr(self.client, "action")
            and hasattr(self.client.action, "return_to_launch")
        ):
            result = self.client.action.return_to_launch()
            if hasattr(result, "__await__"):
                await result
            called_client = True
        self._record("return_to_launch", placeholder=not called_client)
        return True

    def command_to_mavlink(self, command: ControlCommand) -> Dict[str, Any]:
        """Convert a ControlCommand into a MAVLink-like command dictionary."""
        command_name = "set_velocity"
        if command.mode == ControlMode.EMERGENCY_STOP:
            command_name = "emergency_stop"
        elif command.metadata.get("reason") == "safe_land":
            command_name = "land"

        yaw_rate_deg_s = math.degrees(float(command.yaw_rate))
        return {
            "autopilot": self.autopilot,
            "command": command_name,
            "frame": self.command_frame,
            "velocity": {
                "vx": float(command.vx),
                "vy": float(command.vy),
                "vz": float(command.vz),
            },
            "yaw_rate": float(command.yaw_rate),
            "yaw_rate_deg_s": float(yaw_rate_deg_s),
            "duration": float(command.duration),
            "mode": command.mode.value,
            "metadata": dict(command.metadata),
        }

    async def _client_reports_ready(self, timeout_sec: float) -> bool:
        del timeout_sec
        connected = True
        core = getattr(self.client, "core", None)
        if core is not None and hasattr(core, "connection_state"):
            async for state in core.connection_state():
                connected = bool(getattr(state, "is_connected", True))
                break

        healthy = True
        telemetry = getattr(self.client, "telemetry", None)
        if telemetry is not None and hasattr(telemetry, "health"):
            async for health in telemetry.health():
                healthy = bool(
                    getattr(health, "is_global_position_ok", True)
                    and getattr(health, "is_home_position_ok", True)
                )
                break
        return connected and healthy

    def _velocity_body_payload(self, payload: Dict[str, Any]) -> Any:
        velocity = payload["velocity"]
        values = (
            float(velocity["vx"]),
            float(velocity["vy"]),
            float(velocity["vz"]),
            float(payload["yaw_rate_deg_s"]),
        )
        if self._mavsdk_velocity_body_type is not None:
            return self._mavsdk_velocity_body_type(*values)
        return _VelocityBody(*values)

    def _ensure_sitl_mode_allowed(self) -> None:
        if self.real_hardware_enabled or self.autonomous_real_flight_enabled:
            raise RuntimeError(
                "Phase 4-E rejects real hardware and autonomous real flight flags."
            )
        if not self._sitl_enabled:
            raise RuntimeError(
                "MAVLinkBridge SITL mode requires sitl_enabled=True and mock=False."
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
            "sitl_enabled": self._sitl_enabled,
            "real_hardware_enabled": self.real_hardware_enabled,
            "autonomous_real_flight_enabled": self.autonomous_real_flight_enabled,
        }
        entry.update(kwargs)
        self.command_history.append(entry)
        logger.debug("MAVLinkBridge action recorded: %s", action)


def _load_mavsdk_system() -> Any | None:
    """Lazily load MAVSDK System so mock imports never touch MAVSDK."""
    try:  # pragma: no cover - exercised only when MAVSDK is installed
        mavsdk = importlib.import_module("mavsdk")
    except ImportError:
        return None
    return getattr(mavsdk, "System", None)


def _load_mavsdk_velocity_body_type() -> Any | None:
    try:  # pragma: no cover - exercised only when MAVSDK is installed
        offboard_module = importlib.import_module("mavsdk.offboard")
    except ImportError:
        return None
    return getattr(offboard_module, "VelocityBodyYawspeed", None)


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
