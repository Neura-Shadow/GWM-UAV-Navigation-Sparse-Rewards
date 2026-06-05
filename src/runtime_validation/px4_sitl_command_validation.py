"""Guarded PX4 SITL + MAVSDK command validation for Phase 6-D.

This runner validates the pure-simulation MAVSDK/PX4 SITL command path without
launching PX4 or touching real hardware. Normal tests inject a fake MAVSDK
client through ``MAVLinkBridge``; live SITL attempts require explicit gates and
an externally started PX4 SITL endpoint.
"""

from __future__ import annotations

import asyncio
import copy
import json
import os
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Mapping

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.control.barrier_certificate import ControlBarrierFunction, SafetyLimits
from src.ros2_bridge import MAVLinkBridge
from src.ros2_bridge.mavlink_bridge import MAVSDKSITLConfig
from src.utils.data_types import ControlCommand, ControlMode

SCHEMA_VERSION = "gwm_phase6_px4_sitl_command_validation_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/px4_sitl_command_validation.json"
REQUIRED_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_MAVSDK_SITL_TESTS",
    "GWM_ALLOW_SITL_COMMANDS",
)


def _pure_sim_deployment() -> dict[str, bool]:
    return {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }


@dataclass
class PX4SITLCommandValidationConfig:
    """Configuration for the guarded Phase 6-D command validation run."""

    commands: int = 1
    timeout_sec: float = 30.0
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    connection_url: str = "udp://:14540"
    autopilot: str = "px4"
    health_timeout_sec: float = 10.0
    takeoff_altitude_m: float = 5.0
    command_duration: float = 0.2
    command_vx: float = 0.25
    command_vy: float = 0.0
    command_vz: float = 0.0
    command_yaw_rate: float = 0.0
    allow_arm: bool = False
    land_after_validation: bool = True
    validation_altitude_m: float = 5.0
    validation_position: tuple[float, float, float] = (0.0, 0.0, -5.0)
    obstacle_position: tuple[float, float, float] | None = None
    min_obstacle_distance: float = 4.0
    deployment: dict[str, Any] = field(default_factory=_pure_sim_deployment)
    safety_limits: dict[str, Any] = field(default_factory=dict)


@dataclass
class PX4SITLCommandValidationResult:
    """JSON-safe result for a Phase 6-D command validation attempt."""

    schema_version: str
    status: str
    reason: str | None
    env_gates: dict
    availability: dict
    connection_summary: dict
    deployment_summary: dict
    safety_summary: dict
    commands_requested: int
    commands_completed: int
    command_sequence_summary: dict
    raw_command_summary: dict
    safe_command_summary: dict
    command_history_summary: dict
    timings: dict
    errors: list
    closed: bool


def build_phase6_sitl_command_sequence(
    config: dict | PX4SITLCommandValidationConfig | None = None,
) -> list[ControlCommand]:
    """Build deterministic SITL-only commands for Phase 6-D validation."""
    validation_config = _normalize_config(config)
    commands_requested = max(1, int(validation_config.commands))
    common_metadata = {
        "source": "phase6_px4_sitl_command_validation",
        "sitl_only": True,
        "px4_launch_attempted": False,
    }
    sequence = [
        ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=float(validation_config.command_duration),
            mode=ControlMode.WORLD_MODEL_GUIDED,
            metadata={
                **common_metadata,
                "sequence_role": "initial_zero_setpoint",
                "safe_zero_velocity": True,
            },
        )
    ]
    for index in range(commands_requested):
        sequence.append(
            ControlCommand(
                vx=float(validation_config.command_vx),
                vy=float(validation_config.command_vy),
                vz=float(validation_config.command_vz),
                yaw_rate=float(validation_config.command_yaw_rate),
                duration=float(validation_config.command_duration),
                mode=ControlMode.WORLD_MODEL_GUIDED,
                metadata={
                    **common_metadata,
                    "sequence_role": "validation_velocity_command",
                    "sequence_index": index,
                },
            )
        )
    return sequence


def run_px4_sitl_command_validation(
    config: dict | PX4SITLCommandValidationConfig | None = None,
    bridge: Any = None,
) -> dict:
    """Run guarded PX4 SITL command validation and return a JSON-safe dict."""
    validation_config = _normalize_config(config)
    commands_requested = max(1, int(validation_config.commands))
    start = time.perf_counter()
    timings: Dict[str, Any] = {"started_at_unix": time.time()}
    errors: list[dict[str, str]] = []

    result = PX4SITLCommandValidationResult(
        schema_version=SCHEMA_VERSION,
        status="skipped",
        reason=None,
        env_gates=_env_gate_status(),
        availability={
            "checked": False,
            "mavsdk_available": None,
            "bridge_injected": bridge is not None,
            "real_runtime_attempt": bridge is None,
        },
        connection_summary={
            "connection_url": validation_config.connection_url,
            "autopilot": validation_config.autopilot,
            "px4_launch_attempted": False,
            "px4_launch_allowed": False,
            "px4_started_externally_required": True,
        },
        deployment_summary=_deployment_summary(validation_config, bridge),
        safety_summary={
            "cbf_applied_before_bridge_write": True,
            "saturated_commands": 0,
            "altitude_ok": None,
            "geofence_ok": None,
            "obstacle_filter_applied": False,
            "emergency_stop_attempted": False,
            "land_attempted": False,
            "real_hardware_enabled": _deployment_bool(
                validation_config.deployment, "real_hardware_enabled", False
            ),
            "autonomous_real_flight_enabled": _deployment_bool(
                validation_config.deployment, "autonomous_real_flight_enabled", False
            ),
            "px4_launch_attempted": False,
            "hardware_connection_attempted": False,
        },
        commands_requested=commands_requested,
        commands_completed=0,
        command_sequence_summary={},
        raw_command_summary={},
        safe_command_summary={},
        command_history_summary={},
        timings=timings,
        errors=errors,
        closed=False,
    )

    refusal = _deployment_refusal(validation_config, bridge)
    if refusal is not None:
        result.status = "failed"
        result.reason = refusal
        result.errors.append({"type": "RuntimeError", "message": refusal})
        return _finalize_result(result, validation_config, start)

    requires_real_runtime_gate = bridge is None
    validation_bridge = bridge

    if requires_real_runtime_gate and not _env_gates_satisfied(result.env_gates):
        missing = [
            name
            for name, gate in result.env_gates.items()
            if not bool(gate.get("enabled", False))
        ]
        result.reason = f"Missing required PX4 SITL command validation env gates: {', '.join(missing)}"
        return _finalize_result(result, validation_config, start)

    if requires_real_runtime_gate:
        result.availability["checked"] = True
        result.availability["mavsdk_available"] = mavlink_module._load_mavsdk_system() is not None
        if not result.availability["mavsdk_available"]:
            result.status = "runtime_unavailable"
            result.reason = "MAVSDK Python runtime is unavailable for PX4 SITL validation."
            return _finalize_result(result, validation_config, start)

    command_sequence = build_phase6_sitl_command_sequence(validation_config)
    result.command_sequence_summary = _command_sequence_summary(
        command_sequence,
        validation_config,
    )
    raw_commands: list[ControlCommand] = []
    safe_commands: list[ControlCommand] = []
    safety_decisions: list[dict[str, Any]] = []

    try:
        if validation_bridge is None:
            sitl_config = MAVSDKSITLConfig(
                connection_url=validation_config.connection_url,
                autopilot=validation_config.autopilot,
                takeoff_altitude_m=validation_config.takeoff_altitude_m,
                health_timeout_sec=validation_config.health_timeout_sec,
                mock=False,
                sitl_enabled=True,
                real_hardware_enabled=False,
                autonomous_real_flight_enabled=False,
            )
            validation_bridge = MAVLinkBridge(
                mock=False,
                sitl_enabled=True,
                real_hardware_enabled=False,
                autonomous_real_flight_enabled=False,
                sitl_config=sitl_config,
                safety_filter=_build_cbf(validation_config),
            )

        result.connection_summary.update(_connection_summary(validation_bridge))
        completed = asyncio.run(
            asyncio.wait_for(
                _run_command_path(
                    validation_bridge,
                    validation_config,
                    command_sequence,
                    raw_commands,
                    safe_commands,
                    safety_decisions,
                ),
                timeout=float(validation_config.timeout_sec),
            )
        )
        result.commands_completed = completed
        result.raw_command_summary = _commands_summary(raw_commands)
        result.safe_command_summary = _commands_summary(safe_commands)
        result.command_history_summary = _command_history_summary(validation_bridge)
        result.connection_summary.update(_connection_summary(validation_bridge))
        result.deployment_summary = _deployment_summary(validation_config, validation_bridge)
        result.safety_summary.update(_safety_summary(validation_config, safety_decisions))
        result.status = "passed"
        result.reason = None
    except Exception as exc:
        result.status = "failed"
        result.reason = str(exc)
        result.errors.append({"type": exc.__class__.__name__, "message": str(exc)})
        result.raw_command_summary = _commands_summary(raw_commands)
        result.safe_command_summary = _commands_summary(safe_commands)
        result.safety_summary.update(_safety_summary(validation_config, safety_decisions))
        if validation_bridge is not None and getattr(validation_bridge, "is_connected", False):
            result.safety_summary["emergency_stop_attempted"] = True
            try:
                asyncio.run(validation_bridge.emergency_stop())
            except Exception as stop_exc:  # pragma: no cover
                result.errors.append(
                    {
                        "type": stop_exc.__class__.__name__,
                        "message": f"emergency_stop failed: {stop_exc}",
                    }
                )
    finally:
        if validation_bridge is not None:
            try:
                if getattr(validation_bridge, "is_connected", False):
                    asyncio.run(validation_bridge.disconnect())
                result.closed = True
            except Exception as close_exc:  # pragma: no cover
                result.closed = False
                result.errors.append(
                    {
                        "type": close_exc.__class__.__name__,
                        "message": f"disconnect failed: {close_exc}",
                    }
                )
            result.command_history_summary = _command_history_summary(validation_bridge)
            result.connection_summary.update(_connection_summary(validation_bridge))
            result.deployment_summary = _deployment_summary(validation_config, validation_bridge)

    return _finalize_result(result, validation_config, start)


async def _run_command_path(
    bridge: Any,
    config: PX4SITLCommandValidationConfig,
    command_sequence: list[ControlCommand],
    raw_commands: list[ControlCommand],
    safe_commands: list[ControlCommand],
    safety_decisions: list[dict[str, Any]],
) -> int:
    cbf = _build_cbf(config)
    await bridge.connect()
    await bridge.wait_until_ready(timeout_sec=float(config.health_timeout_sec))

    if config.allow_arm:
        await bridge.arm()

    initial_command, initial_decision = _apply_safety_gate(cbf, config, command_sequence[0])
    raw_commands.append(command_sequence[0])
    safe_commands.append(initial_command)
    safety_decisions.append(initial_decision)
    await bridge.start_offboard(initial_command)

    completed = 0
    for raw_command in command_sequence[1:]:
        safe_command, decision = _apply_safety_gate(cbf, config, raw_command)
        raw_commands.append(raw_command)
        safe_commands.append(safe_command)
        safety_decisions.append(decision)
        await bridge.send_command(safe_command)
        completed += 1

    await bridge.stop_offboard()
    if config.land_after_validation:
        await bridge.land()
    return completed


def _apply_safety_gate(
    cbf: ControlBarrierFunction,
    config: PX4SITLCommandValidationConfig,
    command: ControlCommand,
) -> tuple[ControlCommand, dict[str, Any]]:
    saturated = cbf.saturate(command)
    state = _validation_state(config)
    altitude_ok = cbf.within_altitude_bounds(state)
    geofence_ok = cbf.within_geofence(state)
    obstacle_filter_applied = False
    safe_command = saturated
    barrier_margin = None

    if config.obstacle_position is not None:
        obstacle = {"position": config.obstacle_position}
        barrier_margin = cbf.h(state, obstacle)
        filtered = cbf.filter_action(state, obstacle, saturated)
        obstacle_filter_applied = filtered.mode == ControlMode.SAFETY_OVERRIDE
        safe_command = filtered

    metadata = dict(safe_command.metadata)
    metadata.update(
        {
            "phase6_safety_gate": True,
            "altitude_ok": bool(altitude_ok),
            "geofence_ok": bool(geofence_ok),
        }
    )
    if barrier_margin is not None:
        metadata["barrier_margin"] = float(barrier_margin)
    if not altitude_ok:
        metadata["reason"] = "phase6_altitude_refusal"
        safe_command = ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=safe_command.duration,
            mode=ControlMode.SAFETY_OVERRIDE,
            metadata=metadata,
        )
    elif not geofence_ok:
        metadata["reason"] = "phase6_geofence_refusal"
        safe_command = ControlCommand(
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw_rate=0.0,
            duration=safe_command.duration,
            mode=ControlMode.SAFETY_OVERRIDE,
            metadata=metadata,
        )
    else:
        safe_command = ControlCommand(
            vx=safe_command.vx,
            vy=safe_command.vy,
            vz=safe_command.vz,
            yaw_rate=safe_command.yaw_rate,
            duration=safe_command.duration,
            mode=safe_command.mode,
            metadata=metadata,
        )

    return safe_command, {
        "raw": _command_to_dict(command),
        "safe": _command_to_dict(safe_command),
        "saturated": bool(saturated.metadata.get("saturated", False)),
        "altitude_ok": bool(altitude_ok),
        "geofence_ok": bool(geofence_ok),
        "obstacle_filter_applied": bool(obstacle_filter_applied),
        "barrier_margin": None if barrier_margin is None else float(barrier_margin),
    }


def _normalize_config(
    config: dict | PX4SITLCommandValidationConfig | None,
) -> PX4SITLCommandValidationConfig:
    if isinstance(config, PX4SITLCommandValidationConfig):
        return copy.deepcopy(config)

    source = _validation_config_section(config or {})
    deployment = dict(source.get("deployment") or _pure_sim_deployment())
    safety = dict(source.get("safety") or {})
    command = dict(source.get("command") or {})
    validation_state = dict(source.get("validation_state") or {})
    obstacle_position = validation_state.get("obstacle_position", source.get("obstacle_position"))
    position = validation_state.get("position", source.get("validation_position", (0.0, 0.0, -5.0)))
    return PX4SITLCommandValidationConfig(
        commands=int(source.get("commands", 1)),
        timeout_sec=float(source.get("timeout_sec", 30.0)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        connection_url=str(source.get("connection_url", "udp://:14540")),
        autopilot=str(source.get("autopilot", "px4")),
        health_timeout_sec=float(source.get("health_timeout_sec", 10.0)),
        takeoff_altitude_m=float(source.get("takeoff_altitude_m", 5.0)),
        command_duration=float(command.get("duration", source.get("command_duration", 0.2))),
        command_vx=float(command.get("vx", source.get("command_vx", 0.25))),
        command_vy=float(command.get("vy", source.get("command_vy", 0.0))),
        command_vz=float(command.get("vz", source.get("command_vz", 0.0))),
        command_yaw_rate=float(command.get("yaw_rate", source.get("command_yaw_rate", 0.0))),
        allow_arm=bool(source.get("allow_arm", False)),
        land_after_validation=bool(source.get("land_after_validation", True)),
        validation_altitude_m=float(
            validation_state.get("altitude_m", source.get("validation_altitude_m", 5.0))
        ),
        validation_position=_tuple3(position),
        obstacle_position=None if obstacle_position is None else _tuple3(obstacle_position),
        min_obstacle_distance=float(
            safety.get("min_obstacle_distance", source.get("min_obstacle_distance", 4.0))
        ),
        deployment=deployment,
        safety_limits=dict(safety.get("limits") or source.get("safety_limits") or {}),
    )


def _validation_config_section(config: Mapping[str, Any]) -> Dict[str, Any]:
    if "runtime_validation" in config:
        runtime_validation = config.get("runtime_validation") or {}
        if isinstance(runtime_validation, Mapping):
            return dict(runtime_validation.get("px4_sitl_command_validation") or {})
    if "px4_sitl_command_validation" in config:
        return dict(config.get("px4_sitl_command_validation") or {})
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


def _deployment_refusal(
    config: PX4SITLCommandValidationConfig,
    bridge: Any | None,
) -> str | None:
    deployment = config.deployment
    if _deployment_bool(deployment, "real_hardware_enabled", False) or bool(
        getattr(bridge, "real_hardware_enabled", False)
    ):
        return "Phase 6-D refuses real_hardware_enabled=True."
    if _deployment_bool(deployment, "autonomous_real_flight_enabled", False) or bool(
        getattr(bridge, "autonomous_real_flight_enabled", False)
    ):
        return "Phase 6-D refuses autonomous_real_flight_enabled=True."
    if bridge is None:
        if _deployment_bool(deployment, "mock", False):
            return "Phase 6-D live validation requires deployment.mock=False."
        if not _deployment_bool(deployment, "sitl_enabled", False):
            return "Phase 6-D live validation requires deployment.sitl_enabled=True."
    elif bool(getattr(bridge, "mock", False)):
        return "Phase 6-D injected bridge must use mock=False for SITL command validation."
    elif not bool(getattr(bridge, "is_sitl_enabled", False)):
        return "Phase 6-D injected bridge must have sitl_enabled=True."
    return None


def _build_cbf(config: PX4SITLCommandValidationConfig) -> ControlBarrierFunction:
    limits = config.safety_limits
    velocity = dict(limits.get("velocity_limits") or limits)
    altitude = dict(limits.get("altitude_bounds") or {})
    geofence_config = dict(limits.get("geofence") or {})
    geofence = None
    if geofence_config.get("enabled", False):
        geofence = {
            axis: geofence_config[axis]
            for axis in ("x", "y", "z")
            if axis in geofence_config
        }
    return ControlBarrierFunction(
        limits=SafetyLimits(
            max_vx=float(velocity.get("max_vx", 4.0)),
            max_vy=float(velocity.get("max_vy", 4.0)),
            max_vz=float(velocity.get("max_vz", 2.0)),
            max_yaw_rate=float(velocity.get("max_yaw_rate", 1.0)),
            min_altitude=float(altitude.get("min_altitude", 0.5)),
            max_altitude=float(altitude.get("max_altitude", 120.0)),
            geofence=geofence,
        ),
        min_obstacle_distance=float(config.min_obstacle_distance),
    )


def _validation_state(config: PX4SITLCommandValidationConfig) -> dict[str, Any]:
    return {
        "position": tuple(float(value) for value in config.validation_position),
        "altitude": float(config.validation_altitude_m),
        "metadata": {
            "source": "phase6_px4_sitl_command_validation",
            "sitl_only": True,
        },
    }


def _connection_summary(bridge: Any) -> dict:
    return {
        "connection_url": getattr(bridge, "connection_url", None),
        "autopilot": getattr(bridge, "autopilot", None),
        "mock": bool(getattr(bridge, "mock", False)),
        "sitl_enabled": bool(getattr(bridge, "is_sitl_enabled", False)),
        "connected": bool(getattr(bridge, "is_connected", False)),
        "offboard": bool(getattr(bridge, "is_offboard", False)),
        "px4_launch_attempted": False,
        "px4_started_externally_required": True,
    }


def _deployment_summary(
    config: PX4SITLCommandValidationConfig,
    bridge: Any | None,
) -> dict:
    return {
        "mock": _deployment_bool(config.deployment, "mock", bool(getattr(bridge, "mock", False))),
        "sitl_enabled": _deployment_bool(
            config.deployment,
            "sitl_enabled",
            bool(getattr(bridge, "is_sitl_enabled", False)),
        ),
        "real_hardware_enabled": _deployment_bool(
            config.deployment,
            "real_hardware_enabled",
            bool(getattr(bridge, "real_hardware_enabled", False)),
        ),
        "autonomous_real_flight_enabled": _deployment_bool(
            config.deployment,
            "autonomous_real_flight_enabled",
            bool(getattr(bridge, "autonomous_real_flight_enabled", False)),
        ),
        "px4_launch_attempted": False,
        "px4_launch_allowed": False,
        "external_px4_sitl_required": True,
    }


def _safety_summary(
    config: PX4SITLCommandValidationConfig,
    decisions: list[dict[str, Any]],
) -> dict:
    return {
        "cbf_applied_before_bridge_write": True,
        "saturated_commands": sum(1 for decision in decisions if decision.get("saturated")),
        "altitude_ok": all(decision.get("altitude_ok", True) for decision in decisions)
        if decisions
        else None,
        "geofence_ok": all(decision.get("geofence_ok", True) for decision in decisions)
        if decisions
        else None,
        "obstacle_filter_applied": any(
            decision.get("obstacle_filter_applied", False) for decision in decisions
        ),
        "real_hardware_enabled": _deployment_bool(
            config.deployment, "real_hardware_enabled", False
        ),
        "autonomous_real_flight_enabled": _deployment_bool(
            config.deployment, "autonomous_real_flight_enabled", False
        ),
        "px4_launch_attempted": False,
        "hardware_connection_attempted": False,
        "safety_decisions": _json_safe(decisions),
    }


def _command_sequence_summary(
    commands: list[ControlCommand],
    config: PX4SITLCommandValidationConfig,
) -> dict:
    return {
        "initial_setpoint_included": True,
        "validation_command_count": max(0, len(commands) - 1),
        "sequence_length": len(commands),
        "allow_arm": bool(config.allow_arm),
        "land_after_validation": bool(config.land_after_validation),
        "command_roles": [command.metadata.get("sequence_role") for command in commands],
    }


def _commands_summary(commands: list[ControlCommand]) -> dict:
    return {
        "count": len(commands),
        "commands": [_command_to_dict(command) for command in commands],
    }


def _command_history_summary(bridge: Any) -> dict:
    history = list(getattr(bridge, "command_history", []))
    actions = [str(entry.get("action")) for entry in history if isinstance(entry, Mapping)]
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


def _command_to_dict(command: ControlCommand) -> dict:
    return {
        "vx": float(command.vx),
        "vy": float(command.vy),
        "vz": float(command.vz),
        "yaw_rate": float(command.yaw_rate),
        "duration": float(command.duration),
        "mode": command.mode.value,
        "metadata": _json_safe(command.metadata),
    }


def _finalize_result(
    result: PX4SITLCommandValidationResult,
    config: PX4SITLCommandValidationConfig,
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


def _deployment_bool(deployment: Mapping[str, Any], key: str, default: bool) -> bool:
    return bool(deployment.get(key, default))


def _tuple3(value: Any) -> tuple[float, float, float]:
    values = list(value) if isinstance(value, (list, tuple)) else [0.0, 0.0, 0.0]
    values = values + [0.0, 0.0, 0.0]
    return (float(values[0]), float(values[1]), float(values[2]))


def _json_safe(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if hasattr(value, "__dict__"):
        return _json_safe(vars(value))
    return value
