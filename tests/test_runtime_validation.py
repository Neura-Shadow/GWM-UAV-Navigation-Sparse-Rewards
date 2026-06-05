"""Tests for Phase 5-A runtime capability detection."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Sequence

import pytest
import yaml

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.runtime_validation import (  # noqa: E402
    CapabilityStatus,
    RuntimeCapabilityDetector,
    RuntimeCapabilityReport,
    report_to_json,
    write_report,
)


def _missing_spec(_: str) -> None:
    return None


def _missing_command(_: str) -> None:
    return None


def _fake_runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(list(command), 0, stdout="", stderr="")


def test_runtime_validation_package_imports() -> None:
    assert CapabilityStatus is not None
    assert RuntimeCapabilityReport is not None
    assert RuntimeCapabilityDetector is not None
    assert write_report is not None


def test_detector_returns_schema_compatible_report() -> None:
    detector = RuntimeCapabilityDetector(
        import_spec=_missing_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )

    report = detector.detect()
    payload = report.to_dict()

    assert payload["schema_version"] == "gwm_runtime_capability_report_v1"
    assert payload["platform"]["system"]
    assert payload["python"]["version"]
    assert payload["isaac_sim"]["available"] is False
    assert payload["airsim"]["available"] is False
    assert payload["ros2"]["available"] is False
    assert payload["mavsdk"]["available"] is False
    assert payload["px4"]["available"] is False
    assert payload["safety"]["read_only_probe"] is True


def test_json_serialization_and_report_write(tmp_path: Path) -> None:
    detector = RuntimeCapabilityDetector(
        import_spec=_missing_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )
    report = detector.detect()
    output_path = tmp_path / "runtime_capability_report.json"

    write_report(report, str(output_path))
    loaded = json.loads(output_path.read_text(encoding="utf-8"))

    assert json.loads(report_to_json(report))["schema_version"] == loaded["schema_version"]
    assert loaded["safety"]["launch_runtimes"] is False


def test_safe_env_allowlist_and_redaction() -> None:
    env = {
        "GWM_RUNTIME_ARTIFACT_DIR": "outputs/runtime_validation",
        "GWM_SITL_CONNECTION_URL": "udp://:14540",
        "GITHUB_TOKEN": "should_not_appear",
        "PATH": "C:\\PX4;C:\\Other",
    }
    detector = RuntimeCapabilityDetector(
        environ=env,
        import_spec=_missing_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )

    payload = detector.detect().to_dict()
    dumped = json.dumps(payload)

    assert payload["environment"]["allowlisted_variables"]["GWM_SITL_CONNECTION_URL"][
        "value"
    ] == "udp://:14540"
    assert "GITHUB_TOKEN" not in dumped
    assert "should_not_appear" not in dumped
    assert payload["environment"]["all_environment_dumped"] is False


def test_missing_optional_runtimes_produce_unavailable_not_failure() -> None:
    detector = RuntimeCapabilityDetector(
        import_spec=_missing_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )

    report = detector.detect()

    assert report.isaac_sim.available is False
    assert report.airsim.available is False
    assert report.ros2.available is False
    assert report.mavsdk.available is False
    assert report.github_cli.available is False


def test_mocked_import_availability_sets_capability_details() -> None:
    available = {"isaacsim", "cosysairsim", "rclpy", "message_filters", "mavsdk"}

    def _spec(name: str) -> object | None:
        if name in available:
            return SimpleNamespace(origin=f"/fake/{name}.py")
        return None

    detector = RuntimeCapabilityDetector(
        environ={"ROS_DISTRO": "humble"},
        import_spec=_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )
    report = detector.detect()

    assert report.isaac_sim.available is True
    assert report.airsim.available is True
    assert report.airsim.version == "cosysairsim"
    assert report.ros2.available is True
    assert report.ros2.version == "humble"
    assert report.mavsdk.available is True
    assert report.isaac_sim.details["simulation_app_instantiated"] is False


def test_mocked_subprocess_command_availability_is_reported() -> None:
    commands = {
        "nvidia-smi": "C:\\Tools\\nvidia-smi.exe",
        "px4": "C:\\PX4\\px4.exe",
        "make": "C:\\Tools\\make.exe",
        "git": "C:\\Tools\\git.exe",
        "gh": "C:\\Tools\\gh.exe",
    }
    seen_commands: list[Sequence[str]] = []

    def _which(name: str) -> str | None:
        return commands.get(name)

    def _runner(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        seen_commands.append(tuple(command))
        if command[0].endswith("nvidia-smi.exe"):
            return subprocess.CompletedProcess(
                list(command),
                0,
                stdout="NVIDIA RTX, 555.55, 8192 MiB\n",
                stderr="",
            )
        if command[0].endswith("gh.exe") and command[1] == "--version":
            return subprocess.CompletedProcess(list(command), 0, stdout="gh version 2.93.0\n", stderr="")
        if command[0].endswith("gh.exe") and command[1:3] == ["auth", "status"]:
            return subprocess.CompletedProcess(
                list(command),
                0,
                stdout="Token: gho_123456\nLogged in\n",
                stderr="",
            )
        return subprocess.CompletedProcess(list(command), 0, stdout="", stderr="")

    detector = RuntimeCapabilityDetector(
        import_spec=_missing_spec,
        command_finder=_which,
        command_runner=_runner,
    )
    report = detector.detect()
    payload = report.to_dict()

    assert payload["gpu"]["nvidia_smi_available"] is True
    assert payload["gpu"]["gpus"][0]["name"] == "NVIDIA RTX"
    assert payload["px4"]["available"] is True
    assert payload["github_cli"]["version"] == "gh version 2.93.0"
    assert "gho_123456" not in json.dumps(payload)
    assert any("nvidia-smi.exe" in command[0] for command in seen_commands)


def test_cli_no_write_output_works(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "check_runtime_capabilities.py"),
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "runtime_capabilities python=ok" in result.stdout
    assert output_path.exists() is False


def test_cli_output_temp_path_writes_json(tmp_path: Path) -> None:
    output_path = tmp_path / "gwm_runtime_capability_report.json"
    result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "check_runtime_capabilities.py"),
            "--output",
            str(output_path),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert output_path.exists()
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "gwm_runtime_capability_report_v1"
    assert payload["safety"]["launch_runtimes"] is False


def test_no_real_runtime_launch_or_connection_flags_are_set() -> None:
    detector = RuntimeCapabilityDetector(
        import_spec=lambda name: SimpleNamespace(origin=name),
        command_finder=lambda name: f"C:\\Tools\\{name}.exe",
        command_runner=_fake_runner,
    )
    payload = detector.detect().to_dict()

    assert payload["isaac_sim"]["details"]["simulation_app_instantiated"] is False
    assert payload["airsim"]["details"]["connection_attempted"] is False
    assert payload["airsim"]["details"]["api_control_enabled"] is False
    assert payload["airsim"]["details"]["unreal_launch_attempted"] is False
    assert payload["ros2"]["details"]["nodes_started"] is False
    assert payload["ros2"]["details"]["live_topics_checked"] is False
    assert payload["mavsdk"]["details"]["connection_attempted"] is False
    assert payload["px4"]["details"]["sitl_launched"] is False
    assert payload["px4"]["details"]["connection_attempted"] is False
    assert payload["safety"]["connect_to_sitl"] is False
    assert payload["safety"]["connect_to_hardware"] is False


def test_default_config_safety_flags_remain_false() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    safety = config["runtime_validation"]["safety"]

    assert safety["launch_runtimes"] is False
    assert safety["connect_to_sitl"] is False
    assert safety["connect_to_hardware"] is False


def test_detector_does_not_touch_codegraph(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)
    detector = RuntimeCapabilityDetector(
        import_spec=_missing_spec,
        command_finder=_missing_command,
        command_runner=_fake_runner,
    )

    detector.detect()

    assert (tmp_path / ".codegraph").exists() is False
