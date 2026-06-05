"""Tests for Phase 6-D guarded PX4 SITL + MAVSDK command validation."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

import src.ros2_bridge.mavlink_bridge as mavlink_module
from src.ros2_bridge import MAVLinkBridge
from src.ros2_bridge.mavlink_bridge import MAVSDKSITLConfig
from src.runtime_validation import (
    PX4SITLCommandValidationConfig,
    PX4SITLCommandValidationResult,
    build_phase6_sitl_command_sequence,
    run_px4_sitl_command_validation,
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
    def __init__(self) -> None:
        self.calls: list[tuple[str, object]] = []

    async def arm(self) -> None:
        self.calls.append(("arm", None))

    async def land(self) -> None:
        self.calls.append(("land", None))


class _FakeOffboard:
    def __init__(self, fail_on_setpoint: bool = False) -> None:
        self.fail_on_setpoint = fail_on_setpoint
        self.calls: list[tuple[str, object]] = []

    async def set_velocity_body(self, command: object) -> None:
        self.calls.append(("set_velocity_body", command))
        if self.fail_on_setpoint:
            raise RuntimeError("phase6 fake PX4 SITL setpoint failure")

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
    monkeypatch.delenv("GWM_ALLOW_OPTIONAL_RUNTIME", raising=False)
    monkeypatch.delenv("GWM_RUN_MAVSDK_SITL_TESTS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_SITL_COMMANDS", raising=False)
    monkeypatch.delenv("GWM_ALLOW_PX4_LAUNCH", raising=False)


def _fake_bridge(client: _FakeMAVSDKClient | None = None) -> MAVLinkBridge:
    return MAVLinkBridge(
        mock=False,
        sitl_enabled=True,
        client=client or _FakeMAVSDKClient(),
        sitl_config=MAVSDKSITLConfig(
            mock=False,
            sitl_enabled=True,
            health_timeout_sec=0.1,
        ),
    )


def test_phase6_px4_sitl_command_validation_exports_without_mavsdk_or_px4() -> None:
    assert PX4SITLCommandValidationConfig is not None
    assert PX4SITLCommandValidationResult is not None
    assert build_phase6_sitl_command_sequence is not None
    assert run_px4_sitl_command_validation is not None


def test_missing_env_gates_skip_without_availability_probe(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    monkeypatch.setattr(
        mavlink_module,
        "_load_mavsdk_system",
        lambda: (_ for _ in ()).throw(AssertionError("MAVSDK availability checked")),
    )

    result = run_px4_sitl_command_validation({"write_output": False})

    assert result["schema_version"] == "gwm_phase6_px4_sitl_command_validation_v1"
    assert result["status"] == "skipped"
    assert "Missing required PX4 SITL command validation env gates" in result["reason"]
    assert result["availability"]["checked"] is False
    assert result["commands_completed"] == 0
    assert result["closed"] is False


def test_gated_unavailable_mavsdk_returns_runtime_unavailable(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("GWM_ALLOW_OPTIONAL_RUNTIME", "1")
    monkeypatch.setenv("GWM_RUN_MAVSDK_SITL_TESTS", "1")
    monkeypatch.setenv("GWM_ALLOW_SITL_COMMANDS", "1")
    monkeypatch.setattr(mavlink_module, "_load_mavsdk_system", lambda: None)

    result = run_px4_sitl_command_validation(
        {"write_output": False, "fail_on_unavailable": True}
    )

    assert result["status"] == "runtime_unavailable"
    assert result["availability"]["checked"] is True
    assert result["availability"]["mavsdk_available"] is False
    assert "MAVSDK Python runtime is unavailable" in result["reason"]


def test_unsafe_deployment_flags_are_rejected_before_connection(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    client = _FakeMAVSDKClient()
    bridge = _fake_bridge(client)

    result = run_px4_sitl_command_validation(
        {
            "write_output": False,
            "deployment": {
                "mock": False,
                "sitl_enabled": True,
                "real_hardware_enabled": True,
                "autonomous_real_flight_enabled": False,
            },
        },
        bridge=bridge,
    )

    assert result["status"] == "failed"
    assert "real_hardware_enabled=True" in result["reason"]
    assert client.connected_url is None
    assert result["closed"] is False


def test_command_sequence_contains_initial_zero_setpoint_and_validation_commands() -> None:
    sequence = build_phase6_sitl_command_sequence({"commands": 2, "command": {"vx": 0.3}})

    assert len(sequence) == 3
    assert sequence[0].metadata["sequence_role"] == "initial_zero_setpoint"
    assert sequence[0].vx == 0.0
    assert sequence[1].metadata["sequence_role"] == "validation_velocity_command"
    assert sequence[1].metadata["sitl_only"] is True
    assert sequence[1].vx == pytest.approx(0.3)


def test_fake_client_command_validation_lifecycle_passes_without_env_gates(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    client = _FakeMAVSDKClient()
    bridge = _fake_bridge(client)

    result = run_px4_sitl_command_validation(
        PX4SITLCommandValidationConfig(
            commands=2,
            allow_arm=True,
            write_output=False,
        ),
        bridge=bridge,
    )

    actions = result["command_history_summary"]["actions"]
    assert result["status"] == "passed"
    assert result["commands_completed"] == 2
    assert result["availability"]["bridge_injected"] is True
    assert result["deployment_summary"]["real_hardware_enabled"] is False
    assert result["deployment_summary"]["autonomous_real_flight_enabled"] is False
    assert result["connection_summary"]["px4_launch_attempted"] is False
    assert actions == [
        "connect",
        "wait_until_ready",
        "arm",
        "send_initial_setpoint",
        "start_offboard",
        "send_command",
        "send_command",
        "stop_offboard",
        "land",
        "disconnect",
    ]
    assert client.connected_url == "udp://:14540"
    assert client.closed is True
    assert [call[0] for call in client.offboard.calls] == [
        "set_velocity_body",
        "start",
        "set_velocity_body",
        "set_velocity_body",
        "stop",
    ]
    assert result["closed"] is True


def test_cbf_saturation_clamps_excessive_command_before_bridge_write(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    bridge = _fake_bridge()

    result = run_px4_sitl_command_validation(
        {
            "commands": 1,
            "write_output": False,
            "command": {"vx": 5.0, "vy": -4.0, "vz": 3.0, "yaw_rate": 2.0},
            "safety": {
                "limits": {
                    "velocity_limits": {
                        "max_vx": 0.5,
                        "max_vy": 0.75,
                        "max_vz": 0.25,
                        "max_yaw_rate": 0.1,
                    }
                }
            },
        },
        bridge=bridge,
    )

    safe_command = result["safe_command_summary"]["commands"][1]
    bridge_command = result["command_history_summary"]["commands"][1]
    assert result["status"] == "passed"
    assert result["safety_summary"]["saturated_commands"] == 1
    assert safe_command["vx"] == pytest.approx(0.5)
    assert safe_command["vy"] == pytest.approx(-0.75)
    assert safe_command["vz"] == pytest.approx(0.25)
    assert safe_command["yaw_rate"] == pytest.approx(0.1)
    assert safe_command["metadata"]["saturated"] is True
    assert bridge_command["velocity"] == {"vx": 0.5, "vy": -0.75, "vz": 0.25}


def test_failure_after_connection_attempts_emergency_stop_and_disconnect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    client = _FakeMAVSDKClient(fail_on_setpoint=True)
    bridge = _fake_bridge(client)

    result = run_px4_sitl_command_validation(
        PX4SITLCommandValidationConfig(commands=1, write_output=False),
        bridge=bridge,
    )

    actions = result["command_history_summary"]["actions"]
    assert result["status"] == "failed"
    assert "phase6 fake PX4 SITL setpoint failure" in result["reason"]
    assert result["safety_summary"]["emergency_stop_attempted"] is True
    assert "emergency_stop" in actions
    assert actions[-1] == "disconnect"
    assert client.closed is True
    assert result["closed"] is True


def test_result_writes_json_to_temp_path(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    _clear_mavsdk_env(monkeypatch)
    output_path = tmp_path / "px4_sitl_command_validation.json"

    result = run_px4_sitl_command_validation(
        {
            "commands": 1,
            "output_path": str(output_path),
            "write_output": True,
        },
        bridge=_fake_bridge(),
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == "passed"
    assert payload["schema_version"] == "gwm_phase6_px4_sitl_command_validation_v1"
    assert payload["command_sequence_summary"]["initial_setpoint_included"] is True
    assert payload["command_history_summary"]["commands"][0]["velocity"] == {
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
    }


def test_runtime_validation_config_contains_phase6d_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    phase6d = config["runtime_validation"]["px4_sitl_command_validation"]

    assert phase6d["enabled"] is False
    assert phase6d["commands"] == 1
    assert phase6d["output_path"] == "outputs/runtime_validation/px4_sitl_command_validation.json"
    assert phase6d["deployment"] == {
        "mock": False,
        "sitl_enabled": True,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert phase6d["required_env_gates"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_MAVSDK_SITL_TESTS",
        "GWM_ALLOW_SITL_COMMANDS",
    ]


def test_phase6_profile_points_to_px4_sitl_command_validation_command() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/pure_sim_isaac_px4_ros2.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["verification"]["phase6d_px4_sitl_command_validation"] == (
        "python scripts/run_px4_sitl_command_validation.py --commands 1"
    )
    assert profile["verification"]["phase6d_report_path"] == (
        "outputs/runtime_validation/px4_sitl_command_validation.json"
    )


def test_cli_help_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_px4_sitl_command_validation.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "Phase 6-D PX4 SITL + MAVSDK command validation" in result.stdout


def test_cli_no_gate_run_skips_and_does_not_write_output(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    env = dict(os.environ)
    env.pop("GWM_ALLOW_OPTIONAL_RUNTIME", None)
    env.pop("GWM_RUN_MAVSDK_SITL_TESTS", None)
    env.pop("GWM_ALLOW_SITL_COMMANDS", None)

    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_px4_sitl_command_validation.py"),
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
    assert "px4_sitl_command_validation status=skipped" in result.stdout
    assert output_path.exists() is False


@pytest.mark.mavsdk_sitl
def test_optional_real_px4_sitl_command_validation_is_gated() -> None:
    if os.environ.get("GWM_ALLOW_OPTIONAL_RUNTIME") != "1":
        pytest.skip("Set GWM_ALLOW_OPTIONAL_RUNTIME=1 to allow optional runtime startup.")
    if os.environ.get("GWM_RUN_MAVSDK_SITL_TESTS") != "1":
        pytest.skip("Set GWM_RUN_MAVSDK_SITL_TESTS=1 to run PX4 SITL validation.")
    if os.environ.get("GWM_ALLOW_SITL_COMMANDS") != "1":
        pytest.skip("Set GWM_ALLOW_SITL_COMMANDS=1 to allow guarded SITL commands.")
    if mavlink_module._load_mavsdk_system() is None:
        pytest.skip("MAVSDK Python runtime is unavailable.")

    result = run_px4_sitl_command_validation({"commands": 1, "write_output": False})

    assert result["status"] == "passed"
    assert result["commands_completed"] == 1
    assert result["deployment_summary"]["real_hardware_enabled"] is False
