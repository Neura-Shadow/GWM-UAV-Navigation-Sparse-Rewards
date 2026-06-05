"""CLI for Phase 5-A read-only runtime capability detection."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict

import yaml

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.runtime_validation import (  # noqa: E402
    RuntimeCapabilityDetector,
    report_to_json,
    write_report,
)


DEFAULT_CONFIG_PATH = "configs/runtime_validation.yaml"
DEFAULT_OUTPUT_DIR = "outputs/runtime_validation"
REPORT_FILENAME = "runtime_capability_report.json"


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI parser."""
    parser = argparse.ArgumentParser(
        description="Detect runtime capabilities without launching runtimes."
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--output", default=None)
    parser.add_argument("--no-write-output", action="store_true")
    parser.add_argument("--json", action="store_true", help="Print the full report JSON.")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON output.")
    return parser


def load_config(path: str) -> Dict[str, Any]:
    """Load YAML config, returning defaults when the file is absent."""
    config_path = Path(path)
    if not config_path.exists():
        return {}
    with open(config_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def main(argv: list[str] | None = None) -> int:
    """Run runtime capability detection."""
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    config = load_config(args.config)
    detector = RuntimeCapabilityDetector(config)
    report = detector.detect()

    runtime_config = config.get("runtime_validation", {})
    output_path = args.output or str(
        Path(runtime_config.get("output_dir", DEFAULT_OUTPUT_DIR)) / REPORT_FILENAME
    )
    write_enabled = bool(
        runtime_config.get("write_report", True)
    )
    if args.no_write_output:
        write_enabled = False
    if write_enabled:
        write_report(report, output_path)

    if args.json:
        print(report_to_json(report, pretty=args.pretty))
    else:
        print(_summary_line(report))
    return 0


def _summary_line(report: Any) -> str:
    cuda_available = bool(report.cuda.get("torch_cuda_available", False))
    return (
        "runtime_capabilities "
        "python=ok "
        f"cuda={_bool_text(cuda_available)} "
        f"isaac={_bool_text(report.isaac_sim.available)} "
        f"airsim={_bool_text(report.airsim.available)} "
        f"ros2={_bool_text(report.ros2.available)} "
        f"mavsdk={_bool_text(report.mavsdk.available)} "
        f"px4={_bool_text(report.px4.available)}"
    )


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


if __name__ == "__main__":
    raise SystemExit(main())
