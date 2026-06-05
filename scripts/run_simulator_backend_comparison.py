"""CLI for the Phase 7 simulator backend comparison report."""

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
    DEFAULT_SIMULATOR_BACKEND_COMPARISON_OUTPUT_PATH,
    run_simulator_backend_comparison,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run a read-only simulator backend comparison.")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--pretty", action="store_true")
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
    comparison_config = _comparison_config_section(config)
    if args.output is not None:
        comparison_config["output_path"] = args.output
    comparison_config.setdefault("output_path", DEFAULT_SIMULATOR_BACKEND_COMPARISON_OUTPUT_PATH)
    if args.no_write_output:
        comparison_config["write_output"] = False
    result = run_simulator_backend_comparison(
        {"simulator_backend_comparison": comparison_config}
    )
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return 0 if result.get("status") == "passed" else 1


def _comparison_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("simulator_backend_comparison") or {})
    return dict(config.get("simulator_backend_comparison") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    backends = ",".join(result.get("registry_backends", []))
    return f"simulator_backend_comparison status={result.get('status')} backends={backends}"


if __name__ == "__main__":
    raise SystemExit(main())
