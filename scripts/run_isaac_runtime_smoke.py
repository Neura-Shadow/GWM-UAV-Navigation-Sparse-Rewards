"""CLI for the guarded Phase 5-B Isaac Sim runtime smoke test."""

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
    DEFAULT_ISAAC_RUNTIME_SMOKE_OUTPUT_PATH,
    run_isaac_runtime_smoke,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run a guarded Isaac Sim runtime smoke test."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--descriptor", default=None)
    parser.add_argument("--frames", type=int, default=None)
    parser.add_argument("--headless", action="store_true", default=None)
    parser.add_argument("--timeout-sec", type=float, default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--fail-on-unavailable",
        action="store_true",
        help="Return nonzero if the env gates are set but Isaac Sim is unavailable.",
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
    """Run the guarded Isaac runtime smoke test."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    smoke_config = _smoke_config_section(config)

    if args.descriptor is not None:
        smoke_config["descriptor_path"] = args.descriptor
    if args.frames is not None:
        smoke_config["frames"] = args.frames
    if args.headless is not None:
        smoke_config["headless"] = bool(args.headless)
    if args.timeout_sec is not None:
        smoke_config["timeout_sec"] = args.timeout_sec
    if args.output is not None:
        smoke_config["output_path"] = args.output
    smoke_config.setdefault("output_path", DEFAULT_ISAAC_RUNTIME_SMOKE_OUTPUT_PATH)
    if args.no_write_output:
        smoke_config["write_output"] = False
    if args.fail_on_unavailable:
        smoke_config["fail_on_unavailable"] = True

    result = run_isaac_runtime_smoke({"isaac_runtime_smoke": smoke_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(result, bool(smoke_config.get("fail_on_unavailable", False)))


def _smoke_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("isaac_runtime_smoke") or {})
    return dict(config.get("isaac_runtime_smoke") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    return (
        "isaac_runtime_smoke "
        f"status={result.get('status')} "
        f"frames={result.get('frames_completed')}/{result.get('frames_requested')} "
        f"closed={str(result.get('closed')).lower()}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if (
        fail_on_unavailable
        and result.get("status") == "skipped"
        and result.get("availability", {}).get("isaac_sim_available") is False
    ):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
