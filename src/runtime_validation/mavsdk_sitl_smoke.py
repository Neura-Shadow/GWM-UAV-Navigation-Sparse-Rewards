"""Guarded MAVSDK / PX4 SITL command-path smoke test for Phase 5-D.

The runner is safe by default: it never launches PX4, never connects to SITL,
and never touches real hardware unless the optional runtime gates are set.
Normal tests inject a fake MAVSDK client through ``MAVLinkBridge``.
"""

from __future__ import annotations

import asyncio
import copy
import json
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.ros2_bridge import MAVLinkBridge
from src.ros2_bridge.mavlink_bridge import MAVSDKSITLConfig
from src.utils.data_types import ControlCommand, ControlMode

SCHEMA_VERSION = "gwm_mavsdk_sitl_smoke_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/mavsdk_sitl_smoke.json"
REQUIRED_ENV_GATES = (
    "GWM_RUN_MAVSDK_SITL_TESTS",
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_ALLOW_SITL_COMMANDS",
)


@dataclass
class MAVSDKSITLSmokeConfig:
    """Configuration for the guarded MAVSDK / PX4 SITL smoke test."""

    commands: int = 1
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    connection_url: str = "udp://:14540"
    autopilot: str = "px4"
    takeoff_altitude_m: float = 5.0
    health_timeout_sec: float = 10.0


@dataclass
class MAVSDKSITLSmokeResult:
    """JSON-safe result for a MAVSDK / PX4 SITL command-path smoke attempt."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    connection_summary: dict
    commands_requested: int
    commands_completed: int
    command_history_summary: dict
    safety_summary: dict
    timings: dict
    errors: list
    closed: bool


def build_safe_sitl_command() -> ControlCommand:
    """Return a safe zero-velocity command for SITL command-path smoke tests."""
    return ControlCommand(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw_rate=0.0,
        duration=0.2,
        mode=ControlMode.WORLD_MODEL_GUIDED,
        metadata={
            "source": "mavsdk_sitl_smoke",
            "safe_zero_velocity": True,
            "sitl_only": True,
        },
    )


def run_mavsdk_sitl_smoke(
    config: dict | MAVSDKSITLSmokeConfig | None = None,
    bridge: Any = None,
) -> dict:
    """Run a guarded MAVSDK / PX4 SITL command-path smoke and return a dict."""
    smoke_config = _normalize_config(config)
    commands_requested = max(1, int(smoke_config.commands))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []

    result = MAVSDKSITLSmokeResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=_env_gate_status(),
        availability={
            "checked": False,
            "mavsdk_available": None,
            "bridge_injected": bridge is not None,
            "real_mode": bridge is None,
        },
        connection_summary={
            "connection_url": smoke_config.connection_url,
            "autopilot": smoke_config.autopilot,
            "px4_launch_attempted": False,
        },
        commands_requested=commands_requested,
        commands_completed=0,
        command_history_summary={},
        safety_summary={
            "sitl_enabled": False,
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "safe_zero_command": True,
            "emergency_stop_attempted": False,
        },
        timings=timings,
        errors=errors,
        closed=False,
    )

    requires_real_runtime_gate = bridge is None
    smoke_bridge = bridge

    if requires_real_runtime_gate and not _env_gates_satisfied(result.env_gates):
        missing = [
            name
            for name, gate in result.env_gates.items()
            if not bool(gate.get("enabled", False))
        ]
        result.reason = f"Missing required MAVSDK/PX4 SITL env gates: {', '.join(missing)}"
        return _finalize_result(result, smoke_config, start)

    if requires_real_runtime_gate:
        result.availability["checked"] = True
        result.availability["mavsdk_available"] = mavlink_module._load_mavsdk_system() is not None
        if not result.availability["mavsdk_available"]:
            result.reason = "MAVSDK Python runtime is unavailable."
            return _finalize_result(result, smoke_config, start)

    try:
        if smoke_bridge is None:
            sitl_config = MAVSDKSITLConfig(
                connection_url=smoke_config.connection_url,
                autopilot=smoke_config.autopilot,
                takeoff_altitude_m=smoke_config.takeoff_altitude_m,
                health_timeout_sec=smoke_config.health_timeout_sec,
                mock=False,
                sitl_enabled=True,
                real_hardware_enabled=False,
                autonomous_real_flight_enabled=False,
            )
            smoke_bridge = MAVLinkBridge(
                mock=False,
                sitl_enabled=True,
                real_hardware_enabled=False,
                autonomous_real_flight_enabled=False,
                sitl_config=sitl_config,
            )

        result.connection_summary.update(_connection_summary(smoke_bridge))
        result.safety_summary.update(_safety_summary(smoke_bridge))
        completed = asyncio.run(
            asyncio.wait_for(
                _run_command_path(smoke_bridge, smoke_config, commands_requested),
                timeout=float(smoke_config.timeout_sec),
            )
        )
        result.commands_completed = completed
        result.command_history_summary = _command_history_summary(smoke_bridge)
        result.connection_summary.update(_connection_summary(smoke_bridge))
        result.safety_summary.update(_safety_summary(smoke_bridge))
        result.status = "passed"
        result.reason = None
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
        if smoke_bridge is not None and getattr(smoke_bridge, "is_connected", False):
            result.safety_summary["emergency_stop_attempted"] = True
            try:
                asyncio.run(smoke_bridge.emergency_stop())
            except Exception as stop_exc:  # pragma: no cover
                result.errors.append(
                    {
                        "type": stop_exc.__class__.__name__,
                        "message": f"emergency_stop failed: {stop_exc}",
                    }
                )
    finally:
        if smoke_bridge is not None:
            try:
                asyncio.run(smoke_bridge.disconnect())
                result.closed = True
            except Exception as close_exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": close_exc.__class__.__name__,
                        "message": f"disconnect failed: {close_exc}",
                    }
                )
            result.command_history_summary = _command_history_summary(smoke_bridge)
            result.connection_summary.update(_connection_summary(smoke_bridge))
            result.safety_summary.update(_safety_summary(smoke_bridge))

    return _finalize_result(result, smoke_config, start)


async def _run_command_path(
    bridge: Any,
    config: MAVSDKSITLSmokeConfig,
    commands_requested: int,
) -> int:
    await bridge.connect()
    await bridge.wait_until_ready(timeout_sec=float(config.health_timeout_sec))
    initial_command = build_safe_sitl_command()
    await bridge.start_offboard(initial_command)

    completed = 0
    for _ in range(commands_requested):
        await bridge.send_command(build_safe_sitl_command())
        completed += 1

    await bridge.stop_offboard()
    return completed


def _normalize_config(
    config: dict | MAVSDKSITLSmokeConfig | None,
) -> MAVSDKSITLSmokeConfig:
    if isinstance(config, MAVSDKSITLSmokeConfig):
        return copy.deepcopy(config)

    source = _smoke_config_section(config or {})
    return MAVSDKSITLSmokeConfig(
        commands=int(source.get("commands", 1)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        connection_url=str(source.get("connection_url", "udp://:14540")),
        autopilot=str(source.get("autopilot", "px4")),
        takeoff_altitude_m=float(source.get("takeoff_altitude_m", 5.0)),
        health_timeout_sec=float(source.get("health_timeout_sec", 10.0)),
    )


def _smoke_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("mavsdk_sitl_smoke") or {})
    if "mavsdk_sitl_smoke" in config:
        return dict(config.get("mavsdk_sitl_smoke") or {})
    return dict(config)


def _env_gate_status() -> dict:
    return {
        name: {
            "present": name in os.environ,
            "enabled": os.environ.get(name) == "1",
        }
        for name in REQUIRED_ENV_GATES
    }


def _env_gates_satisfied(env_gates: Mapping[str, Mapping[str, Any]]) -> bool:
    return all(bool(gate.get("enabled", False)) for gate in env_gates.values())


def _connection_summary(bridge: Any) -> dict:
    return {
        "connection_url": getattr(bridge, "connection_url", None),
        "autopilot": getattr(bridge, "autopilot", None),
        "mock": bool(getattr(bridge, "mock", False)),
        "sitl_enabled": bool(getattr(bridge, "is_sitl_enabled", False)),
        "connected": bool(getattr(bridge, "is_connected", False)),
        "offboard": bool(getattr(bridge, "is_offboard", False)),
        "px4_launch_attempted": False,
    }


def _safety_summary(bridge: Any) -> dict:
    return {
        "sitl_enabled": bool(getattr(bridge, "is_sitl_enabled", False)),
        "real_hardware_enabled": bool(getattr(bridge, "real_hardware_enabled", False)),
        "autonomous_real_flight_enabled": bool(
            getattr(bridge, "autonomous_real_flight_enabled", False)
        ),
        "safe_zero_command": True,
        "px4_launch_attempted": False,
        "hardware_connection_attempted": bool(
            getattr(bridge, "real_hardware_enabled", False)
        ),
    }


def _command_history_summary(bridge: Any) -> dict:
    history = list(getattr(bridge, "command_history", []))
    actions = [str(entry.get("action")) for entry in history]
    commands = [
        _json_safe(entry.get("command"))
        for entry in history
        if isinstance(entry, Mapping) and entry.get("command") is not None
    ]
    return {
        "count": len(history),
        "actions": actions,
        "commands": commands,
        "last_action": actions[-1] if actions else None,
    }


def _finalize_result(
    result: MAVSDKSITLSmokeResult,
    config: MAVSDKSITLSmokeConfig,
    start: float,
) -> dict:
    result.timings["total_sec"] = round(time.perf_counter() - start, 6)
    payload = _json_safe(asdict(result))
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return payload


def _json_safe(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "__dict__"):
        return _json_safe(vars(value))
    return value
