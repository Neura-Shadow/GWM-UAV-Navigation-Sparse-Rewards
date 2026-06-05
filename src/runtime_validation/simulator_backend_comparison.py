"""Read-only simulator backend comparison report for Phase 7-F."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Mapping

from src.digital_twin import AirSimRuntime, IsaacSimRuntime
from src.simulator_backends import SimulatorBackendRegistry

SCHEMA_VERSION = "gwm_phase7_simulator_backend_comparison_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/simulator_backend_comparison.json"


@dataclass
class SimulatorBackendComparisonConfig:
    """Configuration for the backend comparison report."""

    output_path: str | None = None
    write_output: bool = True
    include_runtime_availability: bool = True


@dataclass
class SimulatorBackendComparisonResult:
    """JSON-safe simulator backend comparison result."""

    schema_version: str = SCHEMA_VERSION
    status: str = "passed"
    reason: str | None = None
    registry_backends: list[str] = field(default_factory=list)
    backend_readiness: Dict[str, Any] = field(default_factory=dict)
    safety_summary: Dict[str, Any] = field(default_factory=dict)
    timings: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def run_simulator_backend_comparison(
    config: dict | SimulatorBackendComparisonConfig | None = None,
) -> dict:
    """Build a read-only comparison report for mock, Isaac, and AirSim."""
    comparison_config = _normalize_config(config)
    start = time.perf_counter()
    registry = SimulatorBackendRegistry()
    result = SimulatorBackendComparisonResult(
        registry_backends=list(registry.names()),
        backend_readiness={
            "mock": {
                "registered": "mock" in registry.names(),
                "normal_tests_require_runtime": False,
                "frame": "project_default",
                "observation_schema": "SensorObservation",
                "default_backend": True,
            },
            "isaac": {
                "registered": "isaac" in registry.names(),
                "normal_tests_require_runtime": False,
                "availability": (
                    IsaacSimRuntime.is_available()
                    if comparison_config.include_runtime_availability
                    else None
                ),
                "frame": "isaac_z_up",
                "coordinate_conversion_applied": False,
                "phase6_mainline": True,
            },
            "airsim": {
                "registered": "airsim" in registry.names(),
                "normal_tests_require_runtime": False,
                "availability": (
                    AirSimRuntime.is_available()
                    if comparison_config.include_runtime_availability
                    else None
                ),
                "frame": "airsim_ned",
                "coordinate_conversion_applied": False,
                "phase6_mainline": False,
            },
        },
        safety_summary={
            "read_only": True,
            "simulators_launched": False,
            "runtime_connections_attempted": False,
            "real_hardware_enabled": False,
            "autonomous_real_flight_enabled": False,
            "performance_parity_claimed": False,
        },
        timings={"started_at_unix": time.time()},
    )
    return _finalize(result, comparison_config, start)


def _normalize_config(
    config: dict | SimulatorBackendComparisonConfig | None,
) -> SimulatorBackendComparisonConfig:
    if isinstance(config, SimulatorBackendComparisonConfig):
        return config
    source = _config_section(config or {})
    return SimulatorBackendComparisonConfig(
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        include_runtime_availability=bool(source.get("include_runtime_availability", True)),
    )


def _config_section(config: Mapping[str, Any]) -> dict:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, Mapping):
        return dict(runtime_validation.get("simulator_backend_comparison") or {})
    return dict(config.get("simulator_backend_comparison") or config)


def _finalize(
    result: SimulatorBackendComparisonResult,
    config: SimulatorBackendComparisonConfig,
    start: float,
) -> dict:
    result.timings["total_sec"] = round(time.perf_counter() - start, 6)
    payload = result.to_dict()
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
