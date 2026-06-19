"""Tests for the v2-5C no-write-output C2 replay report CLI."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import scripts.run_c2_replay_report as cli_module


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "run_c2_replay_report.py"


def _run_cli(*args: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        cwd=str(cwd or REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )


def _event(event_id: str, event_type: str, payload: dict[str, object]) -> dict[str, object]:
    return {
        "event_id": event_id,
        "event_type": event_type,
        "timestamp": 1.0,
        "source": "unit_test",
        "payload": payload,
    }


def _input_events() -> list[dict[str, object]]:
    return [
        _event(
            "evt-001",
            "mission.task.created",
            {
                "task_id": "task-001",
                "request_id": "req-001",
                "objective": "Inspect mock area",
                "status": "assigned",
                "priority": 1,
            },
        ),
        _event(
            "evt-002",
            "risk.signal.created",
            {
                "signal_id": "risk-001",
                "category": "communication degradation",
                "severity": 0.2,
                "confidence": 0.8,
            },
        ),
        _event(
            "evt-003",
            "route.planned",
            {
                "route_id": "route-001",
                "task_id": "task-001",
                "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}],
                "score": 1.0,
                "risk_score": 0.2,
                "constraint_verdict": "valid",
            },
        ),
    ]


def test_dashboard_cli_print_json_no_write_by_default(tmp_path: Path) -> None:
    result = _run_cli("--print-json", cwd=tmp_path)

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload["schema_version"] == "v2-5B-c2-replay-report"
    assert list(tmp_path.iterdir()) == []


def test_dashboard_cli_print_markdown_no_write_by_default(tmp_path: Path) -> None:
    result = _run_cli("--print-markdown", cwd=tmp_path)

    assert result.returncode == 0
    assert result.stdout.startswith("# Mission Replay Summary")
    assert "## Metrics Summary" in result.stdout
    assert list(tmp_path.iterdir()) == []


def test_dashboard_cli_dual_print_is_deterministic() -> None:
    first = _run_cli("--print-json", "--print-markdown")
    second = _run_cli("--print-json", "--print-markdown")

    assert first.returncode == 0
    assert first.stdout == second.stdout
    assert "\n---MARKDOWN---\n" in first.stdout


def test_dashboard_cli_uses_builtin_mock_events_without_input_file() -> None:
    result = _run_cli()

    payload = json.loads(result.stdout)

    assert result.returncode == 0
    assert payload["replay"]["event_count"] == 3
    assert payload["metrics"]["event_type_counts"]["mission.task.created"] == 1
    assert payload["metrics"]["event_type_counts"]["route.planned"] == 1


def test_dashboard_cli_reads_input_json_events(tmp_path: Path) -> None:
    input_path = tmp_path / "events.json"
    input_path.write_text(json.dumps({"events": _input_events()}), encoding="utf-8")

    result = _run_cli("--input-json", str(input_path), "--print-json")
    payload = json.loads(result.stdout)

    assert result.returncode == 0
    assert payload["metrics"]["mission_id"] == "req-001"
    assert payload["metrics"]["route_verdict_counts"] == {"valid": 1}


def test_dashboard_cli_writes_only_when_output_is_explicit(tmp_path: Path) -> None:
    output_path = tmp_path / "reports" / "report.md"

    no_output = _run_cli("--print-markdown", cwd=tmp_path)
    assert no_output.returncode == 0
    assert not output_path.exists()

    with_output = _run_cli("--print-markdown", "--output", str(output_path), cwd=tmp_path)
    assert with_output.returncode == 0
    assert output_path.read_text(encoding="utf-8") == with_output.stdout


def test_dashboard_cli_output_json_is_sorted() -> None:
    result = _run_cli("--print-json")

    lines = result.stdout.splitlines()

    assert result.returncode == 0
    assert lines[1].startswith('  "metrics"')


def test_dashboard_cli_output_redacts_credentials(tmp_path: Path) -> None:
    events = _input_events()
    events[0]["payload"]["credentials"] = {"password": "super-secret"}
    input_path = tmp_path / "events.json"
    input_path.write_text(json.dumps(events), encoding="utf-8")

    result = _run_cli("--input-json", str(input_path), "--print-json")

    assert result.returncode == 0
    assert "super-secret" not in result.stdout
    assert "<redacted>" in result.stdout


def test_dashboard_cli_output_redacts_tokens(tmp_path: Path) -> None:
    events = _input_events()
    events[1]["payload"]["token"] = "token-secret"
    input_path = tmp_path / "events.json"
    input_path.write_text(json.dumps({"events": events}), encoding="utf-8")

    result = _run_cli("--input-json", str(input_path), "--print-json")

    assert result.returncode == 0
    assert "token-secret" not in result.stdout
    assert "<redacted>" in result.stdout


def test_dashboard_cli_output_redacts_log_paths(tmp_path: Path) -> None:
    events = _input_events()
    events[2]["metadata"] = {"runtime_log": "C:/private/runtime.log"}
    input_path = tmp_path / "events.json"
    input_path.write_text(json.dumps(events), encoding="utf-8")

    result = _run_cli("--input-json", str(input_path), "--print-json")

    assert result.returncode == 0
    assert "C:/private/runtime.log" not in result.stdout
    assert "<redacted>" in result.stdout


def test_dashboard_cli_rejects_invalid_input_json(tmp_path: Path) -> None:
    input_path = tmp_path / "bad.json"
    input_path.write_text("{not json", encoding="utf-8")

    result = _run_cli("--input-json", str(input_path))

    assert result.returncode != 0
    assert "invalid_input_json" in result.stderr


def test_dashboard_cli_does_not_start_server_or_runtime() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    forbidden = ("flask", "fastapi", "uvicorn", "rclpy", "mavsdk", "airsim", "socket")

    assert all(term not in source.lower() for term in forbidden)


def test_dashboard_cli_imports_without_runtime_dependencies() -> None:
    runtime_modules = {
        "airsim",
        "asyncio",
        "cosysairsim",
        "dash",
        "flask",
        "geopandas",
        "isaacsim",
        "matplotlib",
        "mavsdk",
        "message_filters",
        "numpy",
        "omni",
        "pandas",
        "plotly",
        "pxr",
        "rclpy",
        "shapely",
        "streamlit",
        "threading",
        "torch",
    }

    assert runtime_modules.isdisjoint(cli_module.__dict__)
