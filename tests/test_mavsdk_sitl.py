"""Tests for Phase 4-E guarded MAVSDK / PX4 SITL command path."""

from __future__ import annotations

import asyncio
import os

import pytest
import yaml

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.control import SafetyLimits
from src.ros2_bridge import MAVLinkBridge
from src.ros2_bridge.mavlink_bridge import MAVSDKSITLConfig
from src.utils.data_types import ControlCommand, ControlMode


class _FakeCore:
    def __init__(self, connected: bool = True) -> None:
        self.connected = connected

    async def connection_state(self):
        yield type("ConnectionState", (), {"is_connected": self.connected})()


class _FakeTelemetry:
    def __init__(self, healthy: bool = True) -> None:
        self.healthy = healthy

    async def health(self):
        yield type(
            "Health",
            (),
            {
                "is_global_position_ok": self.healthy,
                "is_home_position_ok": self.healthy,
            },
        )()


class _FakeAction:
    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []

    async def arm(self) -> None:
        self.calls.append(("arm", None))

    async def set_takeoff_altitude(self, altitude: float) -> None:
        self.calls.append(("set_takeoff_altitude", altitude))

    async def takeoff(self) -> None:
        self.calls.append(("takeoff", None))

    async def land(self) -> None:
        self.calls.append(("land", None))

    async def hold(self) -> None:
        self.calls.append(("hold", None))

    async def return_to_launch(self) -> None:
        self.calls.append(("return_to_launch", None))


class _FakeOffboard:
    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []

    async def set_velocity_body(self, command: object) -> None:
        self.calls.append(("set_velocity_body", command))

    async def start(self) -> None:
        self.calls.append(("start", None))

    async def stop(self) -> None:
        self.calls.append(("stop", None))


class _FakeMAVSDKClient:
    def __init__(self, connected: bool = True, healthy: bool = True) -> None:
        self.connected_url = None
        self.closed = False
        self.core = _FakeCore(connected=connected)
        self.telemetry = _FakeTelemetry(healthy=healthy)
        self.action = _FakeAction()
        self.offboard = _FakeOffboard()

    async def connect(self, system_address: str) -> None:
        self.connected_url = system_address

    async def close(self) -> None:
        self.closed = True


def test_mavsdk_sitl_config_parses_nested_deployment_config() -> None:
    config = MAVSDKSITLConfig.from_config(
        {
            "deployment": {
                "mock": False,
                "sitl_enabled": True,
                "real_hardware_enabled": False,
                "autonomous_real_flight_enabled": False,
            },
            "mavlink": {
                "connection_url": "udp://:14541",
                "autopilot": "PX4",
                "takeoff_altitude_m": 7.5,
                "command_frame": "body_ned",
                "health_timeout_sec": 2.0,
            },
        }
    )

    assert config.connection_url == "udp://:14541"
    assert config.autopilot == "px4"
    assert config.takeoff_altitude_m == pytest.approx(7.5)
    assert config.sitl_enabled is True
    assert config.mock is False
    assert config.real_hardware_enabled is False
    assert config.autonomous_real_flight_enabled is False


def test_default_deployment_config_is_locked_down() -> None:
    with open("configs/deployment.yaml", "r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    deployment = config["deployment"]
    mavlink = config["mavlink"]

    assert deployment["mock"] is True
    assert deployment["sitl_enabled"] is False
    assert deployment["real_hardware_enabled"] is False
    assert deployment["autonomous_real_flight_enabled"] is False
    assert mavlink["mock"] is True
    assert mavlink["sitl_enabled"] is False


def test_sitl_connect_requires_explicit_opt_in() -> None:
    bridge = MAVLinkBridge(mock=False, sitl_enabled=False, client=_FakeMAVSDKClient())

    with pytest.raises(RuntimeError, match="sitl_enabled"):
        asyncio.run(bridge.connect())


def test_real_hardware_flags_are_rejected_even_with_fake_client() -> None:
    bridge = MAVLinkBridge(
        mock=False,
        sitl_enabled=True,
        real_hardware_enabled=True,
        autonomous_real_flight_enabled=True,
        client=_FakeMAVSDKClient(),
    )

    with pytest.raises(RuntimeError, match="real hardware"):
        asyncio.run(bridge.connect())


def test_fake_client_connect_wait_ready_arm_takeoff_land_lifecycle() -> None:
    async def _run() -> tuple[MAVLinkBridge, _FakeMAVSDKClient]:
        client = _FakeMAVSDKClient()
        bridge = MAVLinkBridge(
            mock=False,
            sitl_enabled=True,
            client=client,
            sitl_config=MAVSDKSITLConfig(health_timeout_sec=0.1),
        )

        assert await bridge.connect() is True
        assert bridge.is_sitl_enabled is True
        assert await bridge.wait_until_ready(timeout_sec=0.1) is True
        assert await bridge.arm() is True
        assert await bridge.takeoff(altitude=6.0) is True
        assert await bridge.land() is True
        await bridge.disconnect()
        return bridge, client

    bridge, client = asyncio.run(_run())

    assert client.connected_url == "udp://:14540"
    assert client.closed is True
    assert client.action.calls == [
        ("arm", None),
        ("set_takeoff_altitude", 6.0),
        ("takeoff", None),
        ("land", None),
    ]
    assert [entry["action"] for entry in bridge.command_history] == [
        "connect",
        "wait_until_ready",
        "arm",
        "takeoff",
        "land",
        "disconnect",
    ]


def test_offboard_start_requires_initial_setpoint() -> None:
    async def _run() -> None:
        bridge = MAVLinkBridge(mock=False, sitl_enabled=True, client=_FakeMAVSDKClient())
        await bridge.connect()

        with pytest.raises(RuntimeError, match="initial setpoint"):
            await bridge.start_offboard()

        assert await bridge.start_offboard(ControlCommand()) is True
        assert bridge.is_offboard is True
        assert await bridge.stop_offboard() is True
        assert bridge.is_offboard is False

    asyncio.run(_run())


def test_send_command_applies_cbf_saturation_and_records_body_ned_metadata() -> None:
    async def _run() -> tuple[MAVLinkBridge, object]:
        client = _FakeMAVSDKClient()
        bridge = MAVLinkBridge(
            mock=False,
            sitl_enabled=True,
            client=client,
            safety_limits=SafetyLimits(max_vx=1.0, max_vy=1.5, max_vz=0.5, max_yaw_rate=0.25),
        )
        await bridge.connect()
        await bridge.start_offboard(ControlCommand())
        await bridge.send_command(ControlCommand(vx=4.0, vy=-3.0, vz=2.0, yaw_rate=1.0))
        return bridge, client.offboard.calls[-1][1]

    bridge, velocity_body = asyncio.run(_run())
    command_entry = bridge.command_history[-1]["command"]

    assert command_entry["velocity"] == {"vx": 1.0, "vy": -1.5, "vz": 0.5}
    assert command_entry["yaw_rate"] == pytest.approx(0.25)
    assert command_entry["yaw_rate_deg_s"] == pytest.approx(14.32394487827058)
    assert command_entry["frame"] == "body_ned"
    assert command_entry["metadata"]["saturated"] is True
    assert velocity_body.forward_m_s == pytest.approx(1.0)
    assert velocity_body.right_m_s == pytest.approx(-1.5)
    assert velocity_body.down_m_s == pytest.approx(0.5)
    assert velocity_body.yawspeed_deg_s == pytest.approx(14.32394487827058)


def test_send_velocity_delegates_to_send_command() -> None:
    async def _run() -> MAVLinkBridge:
        bridge = MAVLinkBridge(mock=True)
        await bridge.connect()
        await bridge.send_velocity(1.0, 2.0, -0.5, yaw_rate=0.2)
        return bridge

    bridge = asyncio.run(_run())

    assert bridge.command_history[-1]["action"] == "send_velocity"
    assert bridge.command_history[-1]["command"]["velocity"]["vy"] == pytest.approx(2.0)


def test_emergency_stop_sends_zero_velocity_and_stop_metadata() -> None:
    async def _run() -> MAVLinkBridge:
        bridge = MAVLinkBridge(mock=False, sitl_enabled=True, client=_FakeMAVSDKClient())
        await bridge.connect()
        await bridge.start_offboard(ControlCommand())
        assert await bridge.emergency_stop() is True
        return bridge

    bridge = asyncio.run(_run())
    entry = bridge.command_history[-1]

    assert entry["action"] == "emergency_stop"
    assert entry["command"]["velocity"] == {"vx": 0.0, "vy": 0.0, "vz": 0.0}
    assert entry["command"]["mode"] == "emergency_stop"
    assert entry["command"]["metadata"]["reason"] == "emergency_stop"


def test_hold_and_return_to_launch_history_entries_work() -> None:
    async def _run() -> tuple[MAVLinkBridge, _FakeMAVSDKClient]:
        client = _FakeMAVSDKClient()
        bridge = MAVLinkBridge(mock=False, sitl_enabled=True, client=client)
        await bridge.connect()
        assert await bridge.hold() is True
        assert await bridge.return_to_launch() is True
        return bridge, client

    bridge, client = asyncio.run(_run())

    assert ("hold", None) in client.action.calls
    assert ("return_to_launch", None) in client.action.calls
    assert [entry["action"] for entry in bridge.command_history[-2:]] == [
        "hold",
        "return_to_launch",
    ]


def test_mavsdk_missing_without_fake_client_raises_clear_runtime_error(monkeypatch) -> None:
    monkeypatch.setattr(mavlink_module, "_load_mavsdk_system", lambda: None)
    bridge = MAVLinkBridge(mock=False, sitl_enabled=True, client=None)

    with pytest.raises(RuntimeError, match="MAVSDK"):
        asyncio.run(bridge.connect())


def test_mock_mode_does_not_import_mavsdk(monkeypatch) -> None:
    called = False

    def _fail_import():
        nonlocal called
        called = True
        raise AssertionError("MAVSDK should not be loaded in mock mode")

    monkeypatch.setattr(mavlink_module, "_load_mavsdk_system", _fail_import, raising=False)
    bridge = MAVLinkBridge(mock=True)

    assert asyncio.run(bridge.connect()) is True
    assert called is False


@pytest.mark.mavsdk_sitl
def test_optional_real_mavsdk_sitl_smoke() -> None:
    if os.environ.get("GWM_RUN_MAVSDK_SITL_TESTS") != "1":
        pytest.skip("Set GWM_RUN_MAVSDK_SITL_TESTS=1 to run MAVSDK/PX4 SITL smoke tests.")
    if mavlink_module._load_mavsdk_system() is None:
        pytest.skip("MAVSDK is not available.")

    async def _run() -> None:
        bridge = MAVLinkBridge(mock=False, sitl_enabled=True)
        try:
            assert await bridge.connect() is True
        finally:
            await bridge.disconnect()

    asyncio.run(_run())
