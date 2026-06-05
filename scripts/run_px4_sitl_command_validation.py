"""CLI for Phase 6-D guarded PX4 SITL + MAVSDK command validation."""

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
    DEFAULT_PX4_SITL_COMMAND_VALIDATION_OUTPUT_PATH,
    run_px4_sitl_command_validation,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run guarded Phase 6-D PX4 SITL + MAVSDK command validation."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--commands", type=int, default=None)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument("--connection-url", default=None)
    parser.add_argument("--autopilot", default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--fail-on-unavailable",
        action="store_true",
        help="Return nonzero if gates are set but MAVSDK is unavailable.",
    )
    parser.add_argument(
        "--allow-arm",
        action="store_true",
        help="Arm the SITL vehicle during validation after readiness checks.",
    )
    parser.add_argument(
        "--no-land",
        action="store_true",
        help="Skip the final land command after stopping offboard mode.",
    )
    return parser


def load_config(path: str) -> Dict[str, Any]:
    """Load YAML config, returning defaults when the file is absent."""
    config_path = Path(path)
    if not config_path.exists():
        return {}
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def main(argv: list[str] | None = None) -> int:
    """Run the guarded Phase 6-D command validation."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    validation_config = _validation_config_section(config)

    if args.commands is not None:
        validation_config["commands"] = args.commands
    if args.timeout_sec is not None:
        validation_config["timeout_sec"] = args.timeout_sec
    if args.connection_url is not None:
        validation_config["connection_url"] = args.connection_url
    if args.autopilot is not None:
        validation_config["autopilot"] = args.autopilot
    if args.output is not None:
        validation_config["output_path"] = args.output
    validation_config.setdefault(
        "output_path",
        DEFAULT_PX4_SITL_COMMAND_VALIDATION_OUTPUT_PATH,
    )
    if args.no_write_output:
        validation_config["write_output"] = False
    if args.fail_on_unavailable:
        validation_config["fail_on_unavailable"] = True
    if args.allow_arm:
        validation_config["allow_arm"] = True
    if args.no_land:
        validation_config["land_after_validation"] = False

    result = run_px4_sitl_command_validation(
        {"px4_sitl_command_validation": validation_config}
    )
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(
        result,
        bool(validation_config.get("fail_on_unavailable", False)),
    )


def _validation_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("px4_sitl_command_validation") or {})
    return dict(config.get("px4_sitl_command_validation") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    return (
        "px4_sitl_command_validation "
        f"status={result.get('status')} "
        f"commands={result.get('commands_completed')}/{result.get('commands_requested')} "
        f"closed={str(result.get('closed')).lower()}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if fail_on_unavailable and result.get("status") == "runtime_unavailable":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
