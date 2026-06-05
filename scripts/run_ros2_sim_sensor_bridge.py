"""CLI for Phase 6-C guarded ROS2 simulation sensor bridge."""

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
    DEFAULT_ROS2_SIM_SENSOR_BRIDGE_OUTPUT_PATH,
    run_ros2_sim_sensor_bridge,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run guarded Phase 6-C ROS2 simulation sensor bridge validation."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--frames", type=int, default=None)
    parser.add_argument("--context-length", type=int, default=None)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--fail-on-unavailable",
        action="store_true",
        help="Return nonzero if gates are set but ROS2 publisher/sync runtime is unavailable.",
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
    """Run the guarded ROS2 simulation sensor bridge."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    bridge_config = _bridge_config_section(config)

    if args.frames is not None:
        bridge_config["frames"] = args.frames
    if args.context_length is not None:
        bridge_config["context_length"] = args.context_length
    if args.timeout_sec is not None:
        bridge_config["timeout_sec"] = args.timeout_sec
    if args.output is not None:
        bridge_config["output_path"] = args.output
    bridge_config.setdefault("output_path", DEFAULT_ROS2_SIM_SENSOR_BRIDGE_OUTPUT_PATH)
    if args.no_write_output:
        bridge_config["write_output"] = False
    if args.fail_on_unavailable:
        bridge_config["fail_on_unavailable"] = True

    result = run_ros2_sim_sensor_bridge({"ros2_sim_sensor_bridge": bridge_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(result, bool(bridge_config.get("fail_on_unavailable", False)))


def _bridge_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("ros2_sim_sensor_bridge") or {})
    return dict(config.get("ros2_sim_sensor_bridge") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    buffer = result.get("observation_buffer_summary", {})
    return (
        "ros2_sim_sensor_bridge "
        f"status={result.get('status')} "
        f"frames={result.get('frames_published')}/{result.get('frames_requested')} "
        f"packets={result.get('packets_synchronized')} "
        f"buffer_ready={str(buffer.get('is_ready', False)).lower()}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if fail_on_unavailable and result.get("status") == "runtime_unavailable":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
