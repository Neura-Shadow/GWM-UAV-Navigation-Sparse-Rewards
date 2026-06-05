"""Tests for Phase 7 simulator backend registry and wrappers."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from src.env import AirSimNavigationEnv, IsaacSimNavigationEnv, MockNavigationEnv
from src.generated_world_model import run_multisim_gwm_demo
from src.runtime_validation import run_simulator_backend_comparison
from src.simulator_backends import (
    SimulatorBackendConfig,
    SimulatorBackendRegistry,
    create_navigation_env,
)

_PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_simulator_backend_package_imports_without_optional_runtimes() -> None:
    registry = SimulatorBackendRegistry()

    assert SimulatorBackendConfig is not None
    assert registry.names() == ("airsim", "isaac", "mock")
    assert create_navigation_env is not None


def test_backend_registry_creates_mock_and_optional_env_wrappers() -> None:
    mock_env = create_navigation_env({"backend": "mock", "env_config": {"max_steps": 2}})
    isaac_env = create_navigation_env({"backend": "isaac", "env_config": {"runtime": {"enabled": False}}})
    airsim_env = create_navigation_env({"backend": "airsim"})

    assert isinstance(mock_env, MockNavigationEnv)
    assert isinstance(isaac_env, IsaacSimNavigationEnv)
    assert isinstance(airsim_env, AirSimNavigationEnv)
    mock_env.close()
    isaac_env.close()
    airsim_env.close()


def test_backend_registry_rejects_unknown_backend() -> None:
    with pytest.raises(ValueError, match="Unsupported simulator backend"):
        create_navigation_env({"backend": "unknown"})


def test_multisim_mock_demo_runs_without_optional_runtime() -> None:
    result = run_multisim_gwm_demo(
        {"simulator_backend": "mock", "steps": 3, "write_output": False}
    )

    assert result["schema_version"] == "gwm_phase7_multisim_demo_v1"
    assert result["simulator_backend"] == "mock"
    assert result["runtime_invocation_summary"]["mock_used"] is True
    assert result["metrics"]["steps"] == 3
    assert result["metrics"]["commands"] == 3


def test_multisim_airsim_no_gate_skips_without_runtime_probe(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_AIRSIM_RUNTIME_TESTS",
        "GWM_ALLOW_AIRSIM_API_CONTROL",
    ):
        monkeypatch.delenv(name, raising=False)

    result = run_multisim_gwm_demo(
        {"simulator_backend": "airsim", "steps": 1, "write_output": False}
    )

    assert result["status"] == "skipped"
    assert "Missing required AirSim demo env gates" in result["reason"]
    assert result["runtime_invocation_summary"]["airsim_runtime_attempted"] is False


def test_simulator_backend_comparison_is_read_only() -> None:
    result = run_simulator_backend_comparison({"write_output": False})

    assert result["schema_version"] == "gwm_phase7_simulator_backend_comparison_v1"
    assert result["status"] == "passed"
    assert result["backend_readiness"]["mock"]["default_backend"] is True
    assert result["backend_readiness"]["airsim"]["phase6_mainline"] is False
    assert result["safety_summary"]["simulators_launched"] is False
    assert result["safety_summary"]["runtime_connections_attempted"] is False


def test_runtime_validation_config_contains_phase7_defaults() -> None:
    config = yaml.safe_load(Path("configs/runtime_validation.yaml").read_text(encoding="utf-8"))
    airsim = config["runtime_validation"]["airsim_runtime_smoke"]
    multisim = config["runtime_validation"]["multisim_gwm_demo"]
    comparison = config["runtime_validation"]["simulator_backend_comparison"]

    assert airsim["enabled"] is False
    assert airsim["output_path"] == "outputs/runtime_validation/airsim_runtime_smoke.json"
    assert airsim["required_env_gates"] == [
        "GWM_ALLOW_OPTIONAL_RUNTIME",
        "GWM_RUN_AIRSIM_RUNTIME_TESTS",
        "GWM_ALLOW_AIRSIM_API_CONTROL",
    ]
    assert multisim["simulator_backend"] == "mock"
    assert multisim["deployment"] == {
        "mock": True,
        "sitl_enabled": False,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert comparison["output_path"] == "outputs/runtime_validation/simulator_backend_comparison.json"


def test_optional_airsim_profile_documents_safe_defaults() -> None:
    profile = yaml.safe_load(
        Path("configs/runtime_profiles/optional_airsim_backend.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert profile["simulator"]["default_backend"] == "mock"
    assert profile["simulator"]["launch_airsim_by_repo"] is False
    assert profile["deployment"] == {
        "mock": True,
        "sitl_enabled": False,
        "real_hardware_enabled": False,
        "autonomous_real_flight_enabled": False,
    }
    assert profile["refusal_rules"]["refuse_airsim_api_control_without_gate"] is True


def test_multisim_cli_mock_runs_and_writes_nothing(tmp_path: Path) -> None:
    output_path = tmp_path / "should_not_exist.json"
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_multisim_gwm_demo.py"),
            "--simulator-backend",
            "mock",
            "--steps",
            "3",
            "--output",
            str(output_path),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "multisim_gwm_demo backend=mock" in result.stdout
    assert "steps=3" in result.stdout
    assert output_path.exists() is False


def test_simulator_backend_comparison_cli_works() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_PROJECT_ROOT / "scripts" / "run_simulator_backend_comparison.py"),
            "--no-write-output",
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
    assert "simulator_backend_comparison status=passed" in result.stdout


def test_multisim_result_writes_json_to_temp_path(tmp_path: Path) -> None:
    output_path = tmp_path / "multisim.json"
    result = run_multisim_gwm_demo(
        {"simulator_backend": "mock", "steps": 1, "output_path": str(output_path)}
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))

    assert result["status"] == payload["status"]
    assert payload["schema_version"] == "gwm_phase7_multisim_demo_v1"
