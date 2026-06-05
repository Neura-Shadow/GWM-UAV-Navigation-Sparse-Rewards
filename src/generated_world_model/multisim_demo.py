"""Multi-simulator GWM demo wrapper for Phase 7."""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

from src.digital_twin import AirSimRuntime
from src.generated_world_model.demo import run_demo
from src.generated_world_model.sim_runtime_demo import run_phase6_gwm_simulation_demo
from src.simulator_backends import create_navigation_env

SCHEMA_VERSION = "gwm_phase7_multisim_demo_v1"
DEFAULT_OUTPUT_PATH = "outputs/runtime_validation/multisim_gwm_demo.json"
AIRSIM_ENV_GATES = (
    "GWM_ALLOW_OPTIONAL_RUNTIME",
    "GWM_RUN_AIRSIM_RUNTIME_TESTS",
    "GWM_ALLOW_AIRSIM_API_CONTROL",
)


@dataclass
class MultiSimGWMDemoConfig:
    """Configuration for the Phase 7 multi-simulator wrapper."""

    simulator_backend: str = "mock"
    steps: int = 3
    output_path: str | None = None
    write_output: bool = True
    fail_on_unavailable: bool = False
    airsim: Dict[str, Any] | None = None


def run_multisim_gwm_demo(config: dict | MultiSimGWMDemoConfig | None = None) -> dict:
    """Run a guarded multi-simulator GWM demo wrapper."""
    demo_config = _normalize_config(config)
    start = time.perf_counter()
    payload = {
        "schema_version": SCHEMA_VERSION,
        "status": "skipped",
        "reason": None,
        "simulator_backend": demo_config.simulator_backend,
        "runtime_gates": _airsim_gates(),
        "runtime_invocation_summary": {
            "mock_used": False,
            "phase6_isaac_runner_delegated": False,
            "airsim_runtime_attempted": False,
            "airsim_unreal_launch_attempted": False,
            "hardware_check_run": False,
        },
        "metrics": {"steps": 0, "commands": 0, "safety_overrides": 0},
        "underlying_result": {},
        "timings": {"started_at_unix": time.time()},
    }

    if demo_config.simulator_backend == "mock":
        result = run_demo(
            {
                "demo": {
                    "steps": demo_config.steps,
                    "observation_source": "mock",
                    "execution_backend": "mock",
                    "write_output": False,
                }
            }
        )
        metrics = result.get("metrics", {})
        payload.update(
            {
                "status": str(result.get("final_status", "completed")),
                "runtime_invocation_summary": {
                    **payload["runtime_invocation_summary"],
                    "mock_used": True,
                },
                "metrics": {
                    "steps": int(metrics.get("total_steps", 0)),
                    "commands": int(metrics.get("commands_sent", 0)),
                    "safety_overrides": int(metrics.get("safety_overrides", 0)),
                },
                "underlying_result": _compact_underlying_result(result),
            }
        )
        return _finalize(payload, demo_config, start)

    if demo_config.simulator_backend == "isaac":
        result = run_phase6_gwm_simulation_demo(
            {
                "phase6_gwm_simulation_demo": {
                    "steps": demo_config.steps,
                    "write_output": False,
                }
            }
        )
        metrics = result.get("metrics", {})
        payload.update(
            {
                "status": str(result.get("status", "skipped")),
                "reason": result.get("reason"),
                "runtime_invocation_summary": {
                    **payload["runtime_invocation_summary"],
                    "phase6_isaac_runner_delegated": True,
                },
                "metrics": {
                    "steps": int(metrics.get("steps", 0)),
                    "commands": int(metrics.get("commands_sent", 0)),
                    "safety_overrides": int(metrics.get("safety_overrides", 0)),
                },
                "underlying_result": _compact_underlying_result(result),
            }
        )
        return _finalize(payload, demo_config, start)

    if demo_config.simulator_backend == "airsim":
        missing = [name for name, present in _airsim_gates().items() if not present]
        if missing:
            payload["status"] = "skipped"
            payload["reason"] = f"Missing required AirSim demo env gates: {', '.join(missing)}"
            return _finalize(payload, demo_config, start)
        if not AirSimRuntime.is_available():
            payload["status"] = "runtime_unavailable"
            payload["reason"] = "AirSim / CosysAirSim Python runtime is unavailable."
            return _finalize(payload, demo_config, start)
        payload["runtime_invocation_summary"]["airsim_runtime_attempted"] = True
        env = create_navigation_env(
            {
                "backend": "airsim",
                "env_config": {
                    **dict(demo_config.airsim or {}),
                    "api_control_enabled": True,
                },
            }
        )
        try:
            obs = env.reset()
            commands = 0
            for _ in range(max(1, int(demo_config.steps))):
                obs, _, done, _ = env.step([0.0, 0.0, 0.0])
                commands += 1
                if done:
                    break
            payload["status"] = "passed"
            payload["metrics"] = {
                "steps": commands,
                "commands": commands,
                "safety_overrides": 0,
            }
            payload["underlying_result"] = {
                "final_pose": list(obs.pose),
                "goal_distance": obs.goal_distance,
                "obstacle_distance": obs.obstacle_distance,
                "metadata": dict(obs.metadata),
            }
        finally:
            env.close()
        return _finalize(payload, demo_config, start)

    payload["status"] = "failed"
    payload["reason"] = "simulator_backend must be one of: mock, isaac, airsim."
    return _finalize(payload, demo_config, start)


def _normalize_config(config: dict | MultiSimGWMDemoConfig | None) -> MultiSimGWMDemoConfig:
    if isinstance(config, MultiSimGWMDemoConfig):
        return config
    source = _config_section(config or {})
    return MultiSimGWMDemoConfig(
        simulator_backend=str(source.get("simulator_backend", source.get("backend", "mock"))).lower(),
        steps=int(source.get("steps", 3)),
        output_path=source.get("output_path"),
        write_output=bool(source.get("write_output", True)),
        fail_on_unavailable=bool(source.get("fail_on_unavailable", False)),
        airsim=dict(source.get("airsim") or {}),
    )


def _config_section(config: Mapping[str, Any]) -> dict:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, Mapping):
        return dict(runtime_validation.get("multisim_gwm_demo") or {})
    return dict(config.get("multisim_gwm_demo") or config)


def _airsim_gates() -> Dict[str, bool]:
    return {name: os.environ.get(name) == "1" for name in AIRSIM_ENV_GATES}


def _compact_underlying_result(result: Mapping[str, Any]) -> dict:
    return {
        key: result.get(key)
        for key in ("schema_version", "status", "final_status", "reason", "config_summary")
        if key in result
    }


def _finalize(payload: dict, config: MultiSimGWMDemoConfig, start: float) -> dict:
    payload["timings"]["total_sec"] = round(time.perf_counter() - start, 6)
    if config.write_output:
        output_path = Path(config.output_path or DEFAULT_OUTPUT_PATH)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
