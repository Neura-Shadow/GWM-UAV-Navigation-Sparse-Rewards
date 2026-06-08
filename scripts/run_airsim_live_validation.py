"""CLI for guarded Phase 7-B Cosys-AirSim live validation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

import yaml

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.runtime_validation import (  # noqa: E402
    DEFAULT_AIRSIM_LIVE_VALIDATION_OUTPUT_PATH,
    run_airsim_live_validation,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run guarded Phase 7-B Cosys-AirSim live validation.",
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--vehicle-name", default=None)
    parser.add_argument("--frames", type=int, default=None)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--pretty", action="store_true")
    parser.add_argument("--fail-on-unavailable", action="store_true")
    parser.add_argument("--validate-zero-command", action="store_true")
    parser.add_argument("--no-api-control", action="store_true")
    return parser


def load_config(path: str) -> Dict[str, Any]:
    config_path = Path(path)
    if not config_path.exists():
        return {}
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    validation_config = _airsim_config_section(config)
    for key in ("host", "port", "vehicle_name", "frames", "timeout_sec"):
        value = getattr(args, key)
        if value is not None:
            validation_config[key] = value
    if args.output is not None:
        validation_config["output_path"] = args.output
    validation_config.setdefault("output_path", DEFAULT_AIRSIM_LIVE_VALIDATION_OUTPUT_PATH)
    if args.no_write_output:
        validation_config["write_output"] = False
    if args.fail_on_unavailable:
        validation_config["fail_on_unavailable"] = True
    if args.validate_zero_command:
        validation_config["validate_zero_command"] = True
        validation_config["api_control_enabled"] = True
    if args.no_api_control:
        validation_config["api_control_enabled"] = False
        validation_config["validate_zero_command"] = False

    result = run_airsim_live_validation({"airsim_live_validation": validation_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(result, bool(validation_config.get("fail_on_unavailable", False)))


def _airsim_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("airsim_live_validation") or {})
    return dict(config.get("airsim_live_validation") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    sensor_summary = result.get("sensor_summary") or {}
    command_summary = result.get("command_summary") or {}
    return (
        "airsim_live_validation "
        f"status={result.get('status')} "
        f"frames={sensor_summary.get('frames_completed', 0)}/"
        f"{sensor_summary.get('frames_requested', 0)} "
        f"commands={command_summary.get('commands_sent', 0)} "
        f"closed={str(result.get('closed', False)).lower()}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") in {"failed", "connection_failed"}:
        return 1
    if fail_on_unavailable and result.get("status") == "runtime_unavailable":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
