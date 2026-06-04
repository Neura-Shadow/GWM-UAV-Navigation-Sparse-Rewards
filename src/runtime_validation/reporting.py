"""Reporting helpers for runtime capability detection."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from src.runtime_validation.types import RuntimeCapabilityReport


def report_to_dict(report: RuntimeCapabilityReport) -> Dict[str, Any]:
    """Return a JSON-safe report dictionary."""
    return report.to_dict()


def report_to_json(report: RuntimeCapabilityReport, *, pretty: bool = True) -> str:
    """Serialize a report to JSON."""
    return json.dumps(
        report_to_dict(report),
        indent=2 if pretty else None,
        sort_keys=True,
    )


def write_report(report: RuntimeCapabilityReport, output_path: str) -> None:
    """Write a runtime capability report to a JSON file."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(report_to_json(report, pretty=True) + "\n", encoding="utf-8")

