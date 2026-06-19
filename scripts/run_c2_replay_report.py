"""CLI for mock-first GWM-UAV-C2 replay reports."""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from src.c2 import (  # noqa: E402
    C2MetricsExporter,
    C2ReplayReportBuilder,
    DashboardReplayBuilder,
    MissionEvent,
)
from src.c2.dashboard_replay import REDACTED_VALUE, SENSITIVE_KEYS  # noqa: E402

DUAL_OUTPUT_DELIMITER = "---MARKDOWN---"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a mock-first C2 replay report.")
    parser.add_argument("--input-json", default=None, help="Optional local JSON file containing events.")
    parser.add_argument("--print-json", action="store_true", help="Print JSON report to stdout.")
    parser.add_argument("--print-markdown", action="store_true", help="Print Markdown report to stdout.")
    parser.add_argument("--output", default=None, help="Optional explicit output path for selected report text.")
    return parser


def main(argv: List[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    try:
        events = load_events(args.input_json)
        output_text = build_selected_output(events, print_json=args.print_json, print_markdown=args.print_markdown)
        if args.output:
            write_explicit_output(args.output, output_text)
        sys.stdout.write(output_text)
        if not output_text.endswith("\n"):
            sys.stdout.write("\n")
        return 0
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2


def load_events(input_json: str | None = None) -> List[MissionEvent]:
    if input_json is None:
        return builtin_mock_events()
    input_path = Path(input_json)
    try:
        with input_path.open("r", encoding="utf-8") as handle:
            raw = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("invalid_input_json: input JSON could not be read or parsed") from exc
    if isinstance(raw, dict):
        events_value = raw.get("events")
    else:
        events_value = raw
    if not isinstance(events_value, list):
        raise ValueError("invalid_input_json: input JSON must contain an events list")
    events: List[MissionEvent] = []
    for item in events_value:
        if not isinstance(item, dict):
            raise ValueError("invalid_event_payload: event payload could not be converted to MissionEvent")
        try:
            events.append(MissionEvent.from_dict(sanitize_event_dict(item)))
        except ValueError as exc:
            raise ValueError("invalid_event_payload: event payload could not be converted to MissionEvent") from exc
    return events


def builtin_mock_events() -> List[MissionEvent]:
    return [
        MissionEvent(
            event_id="evt-000001",
            event_type="mission.task.created",
            timestamp=1.0,
            source="c2_replay_report_cli",
            payload={
                "task_id": "task-001",
                "request_id": "req-001",
                "objective": "Mock dashboard replay task",
                "status": "pending",
                "priority": 1,
            },
        ),
        MissionEvent(
            event_id="evt-000002",
            event_type="risk.signal.created",
            timestamp=2.0,
            source="c2_replay_report_cli",
            payload={
                "signal_id": "risk-001",
                "category": "communication degradation",
                "severity": 0.2,
                "confidence": 0.8,
                "timestamp": 2.0,
            },
        ),
        MissionEvent(
            event_id="evt-000003",
            event_type="route.planned",
            timestamp=3.0,
            source="c2_replay_report_cli",
            payload={
                "route_id": "route-001",
                "task_id": "task-001",
                "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}],
                "score": 2.0,
                "risk_score": 0.2,
                "constraint_verdict": "valid",
                "metadata": {"selected": True, "executable": False},
            },
        ),
    ]


def build_selected_output(events: List[MissionEvent], print_json: bool = False, print_markdown: bool = False) -> str:
    replay_payload, metrics_payload = build_report_payloads(events)
    report_builder = C2ReplayReportBuilder()
    json_report = report_builder.build_json_report(replay_payload, metrics_payload)
    markdown_report = report_builder.build_markdown_report(replay_payload, metrics_payload).rstrip()
    json_text = json.dumps(json_report, indent=2, sort_keys=True)
    wants_json = print_json or not print_markdown
    wants_markdown = print_markdown
    if wants_json and wants_markdown:
        return f"{json_text}\n{DUAL_OUTPUT_DELIMITER}\n{markdown_report}\n"
    if wants_markdown:
        return f"{markdown_report}\n"
    return f"{json_text}\n"


def build_report_payloads(events: List[MissionEvent]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    dashboard_builder = DashboardReplayBuilder()
    metrics_exporter = C2MetricsExporter()
    replay_payload = dashboard_builder.build_replay_payload(events)
    summary = metrics_exporter.summarize_events(events)
    metrics_payload = metrics_exporter.build_metrics_payload(summary)
    return replay_payload, metrics_payload


def write_explicit_output(output: str, text: str) -> None:
    output_path = Path(output)
    if output_path.exists() and output_path.is_dir():
        raise ValueError("invalid_output_path: output path is invalid")
    try:
        if output_path.parent and str(output_path.parent) != ".":
            output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(text, encoding="utf-8")
    except OSError as exc:
        raise ValueError("invalid_output_path: output path is invalid") from exc


def sanitize_event_dict(raw_event: Dict[str, Any]) -> Dict[str, Any]:
    allowed_fields = {"event_id", "event_type", "timestamp", "source", "payload", "correlation_id", "metadata"}
    sanitized: Dict[str, Any] = {}
    metadata = raw_event.get("metadata", {})
    if not isinstance(metadata, dict):
        metadata = {}
    metadata, metadata_redactions = sanitize_sensitive_mapping(metadata, "metadata")
    event_redactions = []
    payload_redactions = []
    for key, value in raw_event.items():
        if key not in allowed_fields:
            if is_sensitive_key(key):
                event_redactions.append({"field": key, "value": REDACTED_VALUE})
            continue
        if key == "payload":
            if not isinstance(value, dict):
                raise ValueError("invalid_event_payload: event payload could not be converted to MissionEvent")
            payload, payload_redactions = sanitize_sensitive_mapping(value, "payload")
            sanitized[key] = payload
        elif key == "metadata":
            sanitized[key] = metadata
        elif is_sensitive_key(key):
            event_redactions.append({"field": key, "value": REDACTED_VALUE})
        else:
            sanitized[key] = copy.deepcopy(value)
    redactions = event_redactions + metadata_redactions + payload_redactions
    if redactions:
        sanitized.setdefault("metadata", metadata)
        existing_redactions = sanitized["metadata"].get("redactions", [])
        if not isinstance(existing_redactions, list):
            existing_redactions = []
        sanitized["metadata"]["redactions"] = existing_redactions + redactions
    sanitized.setdefault("payload", {})
    sanitized.setdefault("metadata", metadata)
    return sanitized


def sanitize_sensitive_mapping(value: Dict[str, Any], path: str) -> tuple[Dict[str, Any], List[Dict[str, str]]]:
    sanitized: Dict[str, Any] = {}
    redactions = []
    for key, item in value.items():
        field_path = f"{path}.{key}"
        if is_sensitive_key(key):
            redactions.append({"field": field_path, "value": REDACTED_VALUE})
            continue
        if isinstance(item, dict):
            child, child_redactions = sanitize_sensitive_mapping(item, field_path)
            sanitized[key] = child
            redactions.extend(child_redactions)
        elif isinstance(item, list):
            children = []
            for index, child_item in enumerate(item):
                child, child_redactions = sanitize_sensitive_item(child_item, f"{field_path}[{index}]")
                children.append(child)
                redactions.extend(child_redactions)
            sanitized[key] = children
        else:
            sanitized[key] = copy.deepcopy(item)
    return sanitized, redactions


def sanitize_sensitive_item(value: Any, path: str) -> tuple[Any, List[Dict[str, str]]]:
    if isinstance(value, dict):
        return sanitize_sensitive_mapping(value, path)
    if isinstance(value, list):
        children = []
        redactions: List[Dict[str, str]] = []
        for index, item in enumerate(value):
            child, child_redactions = sanitize_sensitive_item(item, f"{path}[{index}]")
            children.append(child)
            redactions.extend(child_redactions)
        return children, redactions
    return copy.deepcopy(value), []


def is_sensitive_key(key: str) -> bool:
    return key.lower() in SENSITIVE_KEYS


if __name__ == "__main__":
    raise SystemExit(main())
