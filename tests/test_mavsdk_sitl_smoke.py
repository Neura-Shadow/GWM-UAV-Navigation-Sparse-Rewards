"""Tests for Phase 5-D guarded MAVSDK / PX4 SITL smoke tests."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.ros2_bridge import MAVLinkBridge
from src.ros2_bridge.mavlink_bridge import MAVSDKSITLConfig
from src.runtime_validation import (
    MAVSDKSITLSmokeConfig,
    MAVSDKSITLSmokeResult,
    build_safe_sitl_command,
    run_mavsdk_sitl_smoke,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


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
    async def arm(self) -> None:
        return None

    async def land(self) -> None:
        return None


class _FakeOffboard:
    def __init__(self, fail_on_setpoint: bool = False) -> None:
        self.fail_on_setpoint = fail_on_setpoint
        self.calls: list[tuple[str, object]] = []

    async def set_velocity_body(self, command: object) -> None:
        self.calls.append(("set_velocity_body", command))
        if self.fail_on_setpoint:
            raise RuntimeError("fake offboard setpoint failure")

    async def start(self) -> None:
        self.calls.append(("start", None))

    async def stop(self) -> None:
        self.calls.append(("stop", None))


class _FakeMAVSDKClient:
    def __init__(
        self,
        *,
        connected: bool = True,
        healthy: bool = True,
        fail_on_setpoint: bool = False,
    ) -> None:
        self.connected_url = None
        self.closed = False
        self.core = _FakeCore(connected=connected)
        self.telemetry = _FakeTelemetry(healthy=healthy)
        self.action = _FakeAction()
        self.offboard = _FakeOffboard(fail_on_setpoint=fail_on_setpoint)

    async def connect(self, system_address: str) -> None:
        self.connected_url = system_address

    async def close(self) -> None:
        self.closed = True


def _clear_mavsdk_env(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("GWM_RUN_MAVSDK_SITL_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)


def _fake_bridge(client: _FakeMAVSDKClient | None = None) -> MAVLinkBridge:
    return MAVLinkBridge(
        mock=False,
        sitl_enabled=True,
        client=client or _FakeMAVSDKClient(),
        sitl_config=MAVSDKSITLConfig(health_timeout_sec=0.1),
    )


def test_package_imports_without_mavsdk_or_px4() -> None:
    assert MAVSDKSITLSmokeConfig is not None
    assert MAVSDKSITLSmokeResult is not None
    assert build_safe_sitl_command is not None
    assert run_mavsdk_sitl_smoke is not None


def test_missing_env_gates_skip_cleanly_without_connect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    monkeypatch.setattr(
        mavlink_module,
        "_load_mavsdk_system",
        lambda: (_ for _ in ()).throw(AssertionError("MAVSDK availability checked")),
    )

    result = run_mavsdk_sitl_smoke({"write_output": False})

    assert result["schema_version"] == "gwm_mavsdk_sitl_smoke_v1"
    assert result["status"] == "skipped"
    assert "Missing required MAVSDK/PX4 SITL env gates" in result["reason"]
    assert result["availability"]["checked"] is False
    assert result["commands_completed"] == 0
    assert result["closed"] is False


def test_mavsdk_unavailable_skips_cleanly(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("GWM_RUN_MAVSDK_SITL_TESTS", "1")
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "1")
    monkeypatch.setattr(mavlink_module, "_load_mavsdk_system", lambda: None)

    result = run_mavsdk_sitl_smoke({"write_output": False, "fail_on_unavailable": True})

    assert result["status"] == "skipped"
    assert result["availability"]["checked"] is True
    assert result["availability"]["mavsdk_available"] is False
    assert result["reason"] == "MAVSDK Python runtime is unavailable."


def test_build_safe_sitl_command_returns_zero_velocity_metadata() -> None:
    command = build_safe_sitl_command()

    assert command.vx == 0.0
    assert command.vy == 0.0
    assert command.vz == 0.0
    assert command.metadata["source"] == "mavsdk_sitl_smoke"
    assert command.metadata["safe_zero_velocity"] is True
    assert command.metadata["sitl_only"] is True


def test_fake_client_command_path_passes_without_env_gates(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    client = _FakeMAVSDKClient()
    bridge = _fake_bridge(client)

    result = run_mavsdk_sitl_smoke(
        MAVSDKSITLSmokeConfig(commands=1, write_output=False),
        bridge=bridge,
    )

    actions = result["command_history_summary"]["actions"]
    assert result["status"] == "passed"
    assert result["commands_completed"] == 1
    assert result["availability"]["checked"] is False
    assert result["availability"]["bridge_injected"] is True
    assert result["connection_summary"]["mock"] is False
    assert result["connection_summary"]["sitl_enabled"] is True
    assert result["connection_summary"]["connected"] is False
    assert result["safety_summary"]["real_hardware_enabled"] is False
    assert result["safety_summary"]["autonomous_real_flight_enabled"] is False
    assert result["safety_summary"]["px4_launch_attempted"] is False
    assert actions == [
        "connect",
        "wait_until_ready",
        "send_initial_setpoint",
        "start_offboard",
        "send_command",
        "stop_offboard",
        "disconnect",
    ]
    assert client.connected_url == "udp://:14540"
    assert client.closed is True
    assert [call[0] for call in client.offboard.calls] == [
        "set_velocity_body",
        "start",
        "set_velocity_body",
        "stop",
    ]


def test_result_writes_json_to_temp_path(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    output_path = tmp_path / "mavsdk_sitl_smoke.json"

    result = run_mavsdk_sitl_smoke(
        {
            "commands": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        bridge=_fake_bridge(),
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_mavsdk_sitl_smoke_v1"
    assert payload["command_history_summary"]["commands"][0]["velocity"] == {
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
    }


def test_failure_after_connection_attempts_emergency_stop_and_disconnect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    client = _FakeMAVSDKClient(fail_on_setpoint=True)
    bridge = _fake_bridge(client)

    result = run_mavsdk_sitl_smoke(
        MAVSDKSITLSmokeConfig(commands=1, write_output=False),
        bridge=bridge,
    )

    actions = result["command_history_summary"]["actions"]
    assert result["status"] == "failed"
    assert "fake offboard setpoint failure" in result["reason"]
    assert result["safety_summary"]["emergency_stop_attempted"] is True
    assert "emergency_stop" in actions
    assert actions[-1] == "disconnect"
    assert result["closed"] is True
    assert client.closed is True


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_mavsdk_px4_sitl_smoke.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "guarded MAVSDK / PX4 SITL command-path smoke test" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_RUN_MAVSDK_SITL_TESTS", None)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)
    env.pop("GWM_ALLOW_SITL_COMMANDS", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_mavsdk_px4_sitl_smoke.py"),
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "mavsdk_sitl_smoke status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.mavsdk_sitl
def test_optional_real_mavsdk_sitl_smoke_is_gated() -> None:
    if os.environ.get("GWM_RUN_MAVSDK_SITL_TESTS") != "1":
        pytest.skip("Set GWM_RUN_MAVSDK_SITL_TESTS=1 to run MAVSDK/PX4 SITL smoke.")
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime startup.")
    if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1":
        pytest.skip("Set GWM_ALLOW_SITL_COMMANDS=1 to allow guarded SITL commands.")

    result = run_mavsdk_sitl_smoke({"commands": 1, "write_output": False})
    if result["status"] == "skipped":
        pytest.skip(result["reason"])

    assert result["status"] == "passed"
    assert result["commands_completed"] == 1
    assert result["safety_summary"]["real_hardware_enabled"] is False
