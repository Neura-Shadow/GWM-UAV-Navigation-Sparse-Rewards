"""CLI for the Phase 6-F guarded GWM / WAM simulation demo."""

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

from src.generated_world_model import (  # noqa: E402
    DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH,
    run_phase6_gwm_simulation_demo,
)

DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Run the guarded Phase 6-F GWM / WAM simulation demo."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument(
        "--runtime-mode",
        choices=["guarded", "fake"],
        default=None,
        help="Use guarded runtime checks or explicit local fake runtime objects.",
    )
    parser.add_argument(
        "--observation-path",
        choices=["direct_isaac", "ros2"],
        default=None,
    )
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full result JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    parser.add_argument(
        "--require-prior-reports",
        action="store_true",
        help="Require existing Phase 6-B/C/D/E report files before running.",
    )
    parser.add_argument(
        "--no-require-prior-reports",
        action="store_true",
        help="Disable prior report checks even if the config enables them.",
    )
    parser.add_argument(
        "--fail-on-unavailable",
        action="store_true",
        help="Return nonzero when gated runtime dependencies are unavailable.",
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
    """Run the Phase 6-F simulation demo."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    demo_config = _demo_config_section(config)

    if args.steps is not None:
        demo_config["steps"] = args.steps
    if args.runtime_mode is not None:
        demo_config["runtime_mode"] = args.runtime_mode
    if args.observation_path is not None:
        demo_config["observation_path"] = args.observation_path
    if args.output is not None:
        demo_config["output_path"] = args.output
    demo_config.setdefault("output_path", DEFAULT_PHASE6_GWM_SIMULATION_DEMO_OUTPUT_PATH)
    if args.no_write_output:
        demo_config["write_output"] = False
    if args.require_prior_reports:
        demo_config["require_prior_reports"] = True
    if args.no_require_prior_reports:
        demo_config["require_prior_reports"] = False
    if args.fail_on_unavailable:
        demo_config["fail_on_unavailable"] = True

    result = run_phase6_gwm_simulation_demo({"phase6_gwm_simulation_demo": demo_config})
    if args.json:
        print(json.dumps(result, indent=2 if args.pretty else None, sort_keys=True))
    else:
        print(_summary_line(result))
    return _exit_code_for_result(
        result,
        bool(demo_config.get("fail_on_unavailable", False)),
    )


def _demo_config_section(config: Dict[str, Any]) -> Dict[str, Any]:
    runtime_validation = config.get("runtime_validation")
    if isinstance(runtime_validation, dict):
        return dict(runtime_validation.get("phase6_gwm_simulation_demo") or {})
    return dict(config.get("phase6_gwm_simulation_demo") or {})


def _summary_line(result: Dict[str, Any]) -> str:
    metrics = result.get("metrics", {})
    return (
        "phase6_gwm_simulation_demo "
        f"status={result.get('status')} "
        f"steps={metrics.get('steps', 0)} "
        f"commands={metrics.get('commands_sent', 0)} "
        f"safety_overrides={metrics.get('safety_overrides', 0)}"
    )


def _exit_code_for_result(result: Dict[str, Any], fail_on_unavailable: bool) -> int:
    if result.get("status") == "failed":
        return 1
    if fail_on_unavailable and result.get("status") in {"runtime_unavailable", "not_ready"}:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
