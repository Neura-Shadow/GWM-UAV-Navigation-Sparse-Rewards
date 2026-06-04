"""CLI for the Phase 5-E closed-loop mock-to-SITL readiness check."""

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
    DEFAULT_CLOSED_LOOP_READINESS_OUTPUT_PATH,
    run_closed_loop_readiness,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run the mock-first closed-loop GWM runtime readiness check."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--fail-on-unavailable",
        action="store_true",
        help="Return nonzero when required prior smoke reports are unavailable.",
    )
    parser.add_argument(
        "--require-prior-smokes",
        action="store_true",
        help="Require existing Phase 5-B/C/D smoke report files before the mock demo.",
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
    """Run the closed-loop readiness check."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    readiness_config = _readiness_config_section(config)

    if args.steps is not None:
        readiness_config["steps"] = args.steps
    if args.timeout_sec is not None:
        readiness_config["timeout_sec"] = args.timeout_sec
    if args.output is not None:
        readiness_config["output_path"] = args.output
    readiness_config.setdefault("output_path", DEFAULT_CLOSED_LOOP_READINESS_OUTPUT_PATH)
    if args.no_write_output:
        readiness_config["write_output"] = False
    if args.fail_on_unavailable:
        readiness_config["fail_on_unavailable"] = True
    if args.require_prior_smokes:
        readiness_config["require_prior_smokes"] = True

    result = run_closed_loop_readiness({"closed_loop_readiness": readiness_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(
        result,
        bool(readiness_config.get("fail_on_unavailable", False)),
    )


def _readiness_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("closed_loop_readiness") or {})
    return dict(config.get("closed_loop_readiness") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    demo = result.get("demo_summary", {})
    metrics = result.get("metrics", {})
    return (
        "closed_loop_readiness "
        f"status={result.get('status')} "
        f"demo_status={demo.get('final_status')} "
        f"steps={metrics.get('mock_demo_steps', demo.get('steps', 0))} "
        f"commands={metrics.get('commands_sent', demo.get('commands_sent', 0))}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if (
        fail_on_unavailable
        and result.get("status") == "skipped"
        and "Missing prior smoke reports" in str(result.get("reason", ""))
    ):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
