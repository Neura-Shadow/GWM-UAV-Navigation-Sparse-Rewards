"""Tests for Phase 3-D mock-first deployment interfaces."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest

from src.control import ControlBarrierFunction, SafetyLimits
from src.ros2_bridge import (
    HardwareInterface,
    HardwareState,
    MAVLinkBridge,
    MAVSDKSITLConfig,
    MockHardwareInterface,
    ROS2ControlHardwareInterface,
    WorldModelCostmapLayer,
    WorldModelPlannerPlugin,
)
from src.utils.data_types import ControlCommand, ControlMode, SensorObservation


def test_mavlink_bridge_imports_and_converts_command_without_mavsdk() -> None:
    """MAVLink conversion is pure Python and does not require MAVSDK."""
    bridge = MAVLinkBridge(connection_url="udp://:14540", autopilot="px4", mock=True)
    command = ControlCommand(
        vx=1.0,
        vy=-2.0,
        vz=0.5,
        yaw_rate=0.25,
        duration=0.4,
        mode=ControlMode.WORLD_MODEL_GUIDED,
        metadata={"source": "test"},
    )

    payload = bridge.command_to_mavlink(command)

    assert payload["autopilot"] == "px4"
    assert payload["command"] == "set_velocity"
    assert payload["frame"] == "body_ned"
    assert payload["velocity"] == {"vx": 1.0, "vy": -2.0, "vz": 0.5}
    assert payload["yaw_rate"] == 0.25
    assert payload["mode"] == "world_model_guided"
    assert payload["metadata"] == {"source": "test"}


def test_mavlink_bridge_mock_async_lifecycle_records_history() -> None:
    """Mock lifecycle calls record deterministic command history."""

    async def _run() -> MAVLinkBridge:
        bridge = MAVLinkBridge(mock=True)
        assert await bridge.connect() is True
        assert bridge.is_connected is True
        assert await bridge.arm() is True
        assert await bridge.takeoff(altitude=10.0) is True
        assert await bridge.send_velocity(1.0, 0.0, -0.5, yaw_rate=0.1) is True
        assert await bridge.land() is True
        assert await bridge.emergency_stop() is True
        await bridge.disconnect()
        return bridge

    bridge = asyncio.run(_run())
    actions = [entry["action"] for entry in bridge.command_history]

    assert actions == [
        "connect",
        "arm",
        "takeoff",
        "send_velocity",
        "land",
        "emergency_stop",
        "disconnect",
    ]
    assert bridge.command_history[2]["altitude"] == 10.0
    assert bridge.command_history[-2]["command"]["mode"] == "emergency_stop"
    assert bridge.is_connected is False


def test_mavlink_bridge_real_mode_requires_explicit_hardware_enable() -> None:
    """Non-mock mode remains gated behind explicit SITL opt-in."""
    bridge = MAVLinkBridge(mock=False, real_hardware_enabled=False)

    with pytest.raises(RuntimeError, match="sitl_enabled"):
        asyncio.run(bridge.connect())


def test_mavsdk_sitl_config_imports_without_mavsdk() -> None:
    """The SITL config is pure Python and defaults to safe mock mode."""
    config = MAVSDKSITLConfig()

    assert config.connection_url == "udp://:14540"
    assert config.autopilot == "px4"
    assert config.mock is True
    assert config.sitl_enabled is False
    assert config.real_hardware_enabled is False


def test_mock_hardware_interface_read_write_and_emergency_stop() -> None:
    """Mock hardware exposes a ros2_control-style read/write contract."""
    hardware = MockHardwareInterface(limits=SafetyLimits(max_vx=2.0))
    assert isinstance(hardware, HardwareInterface)
    assert hardware.connect() is True

    ok = hardware.write(ControlCommand(vx=5.0, vy=1.0, vz=0.0, yaw_rate=0.0))
    state = hardware.read()

    assert ok is True
    assert state.mode == "mock"
    assert state.velocity[0] == 2.0
    assert hardware.command_history[-1].vx == 2.0

    assert hardware.emergency_stop() is True
    assert hardware.command_history[-1].mode == ControlMode.EMERGENCY_STOP
    assert hardware.command_history[-1].metadata["reason"] == "emergency_stop"


def test_mock_hardware_interface_rejects_altitude_bounds() -> None:
    """Unsafe altitude metadata prevents command writes."""
    hardware = MockHardwareInterface(
        limits=SafetyLimits(min_altitude=1.0, max_altitude=20.0),
    )
    command = ControlCommand(vx=1.0, metadata={"altitude": 30.0})

    assert hardware.write(command) is False
    assert hardware.command_history == []
    assert hardware.rejected_commands[-1].metadata["reason"] == "altitude_limit"


def test_ros2_control_interface_stub_is_mock_safe_and_real_gated() -> None:
    """The ROS2 control stub imports without ROS2 and refuses real mode by default."""
    mock_iface = ROS2ControlHardwareInterface(mock=True)
    assert mock_iface.connect() is True
    assert mock_iface.write(ControlCommand(vx=0.5)) is True

    real_iface = ROS2ControlHardwareInterface(mock=False, real_hardware_enabled=False)
    with pytest.raises(RuntimeError, match="real_hardware_enabled"):
        real_iface.connect()


def test_control_barrier_function_saturates_and_filters_actions() -> None:
    """CBF baseline clamps commands and overrides unsafe obstacles."""
    limits = SafetyLimits(max_vx=2.0, max_vy=2.0, max_vz=1.0, max_yaw_rate=0.5)
    cbf = ControlBarrierFunction(limits=limits, min_obstacle_distance=4.0)
    state = SensorObservation(pose=(0.0, 0.0, -5.0), velocity=(0.0, 0.0, 0.0))
    desired = ControlCommand(vx=5.0, vy=3.0, vz=-2.0, yaw_rate=1.0)

    safe_far = cbf.filter_action(state, obstacle=(10.0, 0.0, -5.0), desired_action=desired)
    safe_near = cbf.filter_action(state, obstacle=(1.0, 0.0, -5.0), desired_action=desired)

    assert safe_far.vx == 2.0
    assert safe_far.vy == 2.0
    assert safe_far.vz == -1.0
    assert safe_far.yaw_rate == 0.5
    assert safe_far.metadata["saturated"] is True
    assert safe_near.vx == 0.0
    assert safe_near.mode == ControlMode.SAFETY_OVERRIDE
    assert safe_near.metadata["reason"] == "cbf_obstacle_filter"


def test_control_barrier_function_altitude_and_geofence_checks() -> None:
    """Altitude bounds and geofence placeholders are explicit pure-Python checks."""
    limits = SafetyLimits(
        min_altitude=1.0,
        max_altitude=20.0,
        geofence={"x": [-5.0, 5.0], "y": [-5.0, 5.0], "z": [-20.0, 0.0]},
    )
    cbf = ControlBarrierFunction(limits=limits)

    assert cbf.within_altitude_bounds({"position": (0.0, 0.0, -10.0)}) is True
    assert cbf.within_altitude_bounds({"position": (0.0, 0.0, -30.0)}) is False
    assert cbf.within_geofence({"position": (1.0, 1.0, -5.0)}) is True
    assert cbf.within_geofence({"position": (10.0, 1.0, -5.0)}) is False


def test_nav2_costmap_layer_updates_plain_arrays() -> None:
    """Nav2-style costmap skeleton mutates plain NumPy grids without Nav2."""
    grid = np.zeros((5, 5), dtype=np.uint8)
    layer = WorldModelCostmapLayer(lethal_cost=200, inflation_radius=1.5)

    updated = layer.update_costs(grid, position=(2, 2), radius=1.0, risk_score=0.5)

    assert updated is grid
    assert grid[2, 2] == 100
    assert grid[0, 0] == 0


def test_world_model_planner_plugin_returns_simple_path() -> None:
    """Planner skeleton returns a deterministic start-to-goal path."""
    planner = WorldModelPlannerPlugin()

    path = planner.plan((0.0, 0.0, 0.0), (4.0, 0.0, 0.0))

    assert path[0] == (0.0, 0.0, 0.0)
    assert path[-1] == (4.0, 0.0, 0.0)
    assert len(path) >= 2
