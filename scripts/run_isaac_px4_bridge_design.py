"""CLI for Phase 6-E Isaac Sim / PX4 SITL bridge design readiness."""

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
    DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH,
    run_isaac_px4_bridge_design,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run Phase 6-E Isaac Sim / PX4 SITL bridge design readiness."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--require-prior-reports",
        action="store_true",
        help="Require existing Phase 6-B/C/D report files before reporting ready.",
    )
    parser.add_argument(
        "--fail-on-not-ready",
        action="store_true",
        help="Return nonzero when readiness is not ready or failed.",
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
    """Run the Phase 6-E bridge design readiness check."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    bridge_config = _bridge_config_section(config)

    if args.output is not None:
        bridge_config["output_path"] = args.output
    bridge_config.setdefault("output_path", DEFAULT_ISAAC_PX4_BRIDGE_DESIGN_OUTPUT_PATH)
    if args.no_write_output:
        bridge_config["write_output"] = False
    if args.require_prior_reports:
        bridge_config["require_prior_reports"] = True
    if args.fail_on_not_ready:
        bridge_config["fail_on_not_ready"] = True

    result = run_isaac_px4_bridge_design({"isaac_px4_bridge_design": bridge_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(
        result,
        bool(bridge_config.get("fail_on_not_ready", False)),
    )


def _bridge_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("isaac_px4_bridge_design") or {})
    return dict(config.get("isaac_px4_bridge_design") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    readiness = result.get("report_readiness", {})
    return (
        "isaac_px4_bridge_design "
        f"status={result.get('status')} "
        f"prior_reports_required={str(readiness.get('required')).lower()} "
        f"reports_ready={str(readiness.get('all_ready')).lower()}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_not_ready: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if fail_on_not_ready and result.get("status") == "not_ready":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
