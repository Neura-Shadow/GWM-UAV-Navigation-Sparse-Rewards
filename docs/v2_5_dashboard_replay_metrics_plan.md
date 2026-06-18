# GWM-UAV-C2 v2-5 Dashboard Replay and Metrics Plan

## 1. Purpose

v2-5 defines the mock-first dashboard replay, mission audit, and
metrics-export layer for GWM-UAV-C2.

`v1.0.0` remains the completed archive release. v2-0 froze the C2 concept and
boundaries. v2-1 completed the mission data foundation. v2-2 completed the
mock-first mission dispatch and fleet allocation layer. v2-3 completed the
mock-first defensive threat and risk prediction layer. v2-4 completed the
mock-first risk-aware planning and UTM-style airspace layer. v2-5 is the next
implementation-oriented planning slice, but this document is docs-only.

The later v2-5 implementation must remain mock-first, deterministic,
pure-Python by default, JSON-safe, in-memory by default, runtime-free,
simulator-free by default, hardware-free, audit-oriented, read-only /
observational, and command-free.

## 2. Relationship to Existing v2 Foundation

v2-5 must build only on:

- `src/c2/mission_types.py`
- `src/c2/event_bus.py`
- `src/c2/state_store.py`
- `src/c2/replay.py`
- `src/c2/mission_dispatcher.py`
- `src/c2/fleet_manager.py`
- `src/c2/risk_prediction.py`
- `src/c2/airspace.py`
- `src/c2/risk_aware_planner.py`

v2-5 should use the existing dataclasses and interfaces:

- `MissionEvent`
- `MissionRequest`
- `MissionTask`
- `FleetAsset`
- `UAVState`
- `RiskSignal`
- `ThreatAssessment`
- `AirspaceConstraint`
- `PlannedRoute`
- `SafetyDecision`
- `HumanApprovalRecord`
- `ReplayFrame`
- `MetricSummary`
- `MissionStateStore`
- `MissionReplayEngine`

No new runtime dependency is allowed.

## 3. v2-5 Allowed Scope

Allowed later implementation files:

- `src/c2/dashboard_replay.py`
- `tests/test_c2_dashboard_replay.py`
- `tests/test_c2_dashboard_metrics.py`
- `tests/test_c2_dashboard_replay_integration.py`
- `scripts/run_c2_replay_report.py`
- `tests/test_c2_replay_report_cli.py`

Allowed capabilities:

- mission timeline assembly
- dashboard-ready JSON export
- read-only replay frame formatting
- metric summary aggregation
- risk event timeline export
- route event timeline export
- fleet/task status timeline export
- safety decision timeline export
- audit metadata generation
- static Markdown report generation
- optional no-write-output CLI mode
- focused unit tests

## 4. v2-5 Explicit Non-goals

v2-5 must not implement:

- interactive command dashboard
- vehicle control dashboard
- route execution controls
- mission upload controls
- arming controls
- takeoff controls
- landing controls
- payload controls
- offensive targeting
- attack execution
- weapon control
- autonomous attack-decision logic
- real simulator connection
- real MAVSDK/PX4 connection
- ROS2 runtime node
- Nav2 runtime plugin
- hardware interface
- autonomous flight behavior
- production monitoring claim
- certified safety dashboard claim
- network server
- database server
- credentials or tokens
- runtime artifacts by default

The dashboard replay layer is observational and audit-oriented only. It must
not issue commands, approve execution, upload routes, or bypass
`SafetyDecision` / `HumanApprovalRecord` flow.

## 5. Dashboard Replay Design Plan

The later implementation should define a mock-first class such as:

```text
DashboardReplayBuilder
```

Recommended API:

```text
__init__(state_store=None, replay_engine=None)
build_timeline(events: list[MissionEvent]) -> list[dict]
build_dashboard_snapshot(snapshot: dict) -> dict
build_replay_payload(events: list[MissionEvent]) -> dict
format_replay_frame(frame: ReplayFrame) -> dict
filter_timeline(event_types: Optional[list[str]] = None) -> list[dict]
```

Required behavior:

- deterministic output
- JSON-safe payloads
- preserve event order
- preserve replay frame order
- include event_id, event_type, timestamp, payload summary, and metadata
- do not mutate state store
- do not write files by default
- do not start web server
- do not connect to runtime

## 6. Metrics Export Design Plan

The later implementation should define a mock-first class such as:

```text
C2MetricsExporter
```

Recommended API:

```text
__init__(replay_engine=None)
summarize_events(events: list[MissionEvent]) -> MetricSummary
build_metrics_payload(summary: MetricSummary) -> dict
build_risk_metrics(events: list[MissionEvent]) -> dict
build_route_metrics(events: list[MissionEvent]) -> dict
build_task_fleet_metrics(events: list[MissionEvent]) -> dict
```

Expected metric groups:

- `event_count`
- `event_type_counts`
- `task_status_counts`
- `fleet_assignment_counts`
- `risk_counts`
- `route_count`
- `route_verdict_counts`
- `safety_decision_counts`
- `human_approval_counts`

Metrics must be deterministic and JSON-safe.

If existing `MetricSummary` already supports some fields, reuse them. If later
implementation needs new fields, add only minimal backward-compatible fields.

## 7. Report Export Plan

The later implementation should define a mock-first report helper such as:

```text
C2ReplayReportBuilder
```

Recommended API:

```text
build_markdown_report(replay_payload: dict, metrics_payload: dict) -> str
build_json_report(replay_payload: dict, metrics_payload: dict) -> dict
```

Expected report sections:

- Mission Replay Summary
- Event Timeline
- Task and Fleet Status
- Risk Timeline
- Route Timeline
- Safety and Human Approval Timeline
- Metrics Summary
- Scope and Safety Notes

Reports must not include:

- credentials
- tokens
- runtime logs
- hardware logs
- flight logs
- simulator screenshots
- PX4 logs
- ROS bag paths
- private hostnames

## 8. CLI Plan

v2-5 may add this optional CLI only in a later implementation slice:

```text
scripts/run_c2_replay_report.py
```

CLI behavior should be mock-first and no-write-output by default:

- `--input-json` optional
- `--print-json`
- `--print-markdown`
- `--output` optional and explicit

Default behavior:

- no file writes unless `--output` is explicitly provided
- no runtime access
- no simulator access
- no network access
- no database access

Do not implement this CLI in the planning slice.

## 9. Event Flow Plan

Expected event flow:

```text
MissionEvent list or MissionStateStore snapshot input
-> MissionReplayEngine replay
-> DashboardReplayBuilder timeline payload
-> C2MetricsExporter metrics payload
-> optional C2ReplayReportBuilder JSON/Markdown report
```

Expected event families:

- `mission.*`
- `fleet.*`
- `uav.*`
- `risk.signal.created`
- `threat.assessment.created`
- `route.planned`
- `safety.*`
- `human_approval.*`
- unknown event types preserved

Unknown event types must remain visible as audit timeline entries.

## 10. State Store and Replay Integration Plan

v2-5 should not replace storage or replay. It should use:

- `MissionStateStore.snapshot()`
- `MissionReplayEngine.replay()`
- `ReplayFrame`
- `MetricSummary`

Requirements:

- dashboard replay can be built from replay frames
- dashboard replay can be built from final snapshot
- timeline output is deterministic
- metrics output is deterministic
- unknown event types are preserved
- blocked/warning routes remain visible
- risk categories remain visible
- no file writes by default
- no dashboard server
- no runtime connection

## 11. Safety and Audit Boundary

v2-5 is read-only and audit-oriented.

v2-5 does not approve route execution. v2-5 does not upload routes to PX4,
ArduPilot, MAVSDK, ROS2, Nav2, or any simulator. v2-5 does not command
vehicles. v2-5 does not replace `SafetyDecision` or `HumanApprovalRecord`.
v2-5 does not make production readiness, simulator parity, or safety
certification claims.

## 12. Test Plan for v2-5 Implementation

Focused tests:

- `test_dashboard_replay_builds_timeline`
- `test_dashboard_replay_preserves_event_order`
- `test_dashboard_replay_preserves_unknown_events`
- `test_dashboard_snapshot_is_json_safe`
- `test_dashboard_replay_payload_is_deterministic`
- `test_dashboard_metrics_counts_event_types`
- `test_dashboard_metrics_counts_risk_categories`
- `test_dashboard_metrics_counts_route_verdicts`
- `test_dashboard_metrics_counts_task_statuses`
- `test_dashboard_report_builds_markdown`
- `test_dashboard_report_builds_json`
- `test_dashboard_report_excludes_credentials`
- `test_dashboard_cli_print_json_no_write_by_default`
- `test_dashboard_cli_print_markdown_no_write_by_default`
- `test_dashboard_imports_without_runtime_dependencies`

Tests must not require:

- GPU
- Isaac Sim
- Cosys-AirSim
- legacy AirSim
- ROS2
- MAVSDK
- PX4
- Nav2
- hardware
- network
- database
- real dashboard server
- real browser
- real simulator logs

## 13. Verification Commands for Future v2-5 Implementation

Future implementation should run:

```bash
python -m pytest tests/test_c2_dashboard_replay.py tests/test_c2_dashboard_metrics.py tests/test_c2_dashboard_replay_integration.py tests/test_c2_replay_report_cli.py -q
python -m pytest tests/test_c2_mission_types.py tests/test_c2_event_bus.py tests/test_c2_state_store.py tests/test_c2_replay.py tests/test_c2_mission_dispatcher.py tests/test_c2_fleet_manager.py tests/test_c2_dispatcher_fleet_integration.py tests/test_c2_risk_prediction.py tests/test_c2_risk_prediction_integration.py tests/test_c2_airspace.py tests/test_c2_risk_aware_planner.py tests/test_c2_planner_airspace_integration.py -q
python -m compileall -q src tests scripts
git diff --check
rg -n "offensive attack|weapon|targeting|payload release|autonomous attack|production-ready|certified safety|real hardware validation|autonomous real flight|arming|takeoff|landing|mission upload|attack execution|weapon control|pursue|intercept|disable|jam|spoof|production UTM|live airspace|real airspace|command dashboard|vehicle control" src/c2 tests scripts docs || true
```

Expected grep hits are acceptable only in explicit non-goal/safety statements,
defensive observation categories, rejection tests, airspace non-goal
statements, dashboard non-goal statements, or legacy guarded-runtime fixture
names outside this slice.

## 14. v2-5 Completion Criteria

v2-5 implementation will be complete later when:

- `DashboardReplayBuilder` exists
- `C2MetricsExporter` exists
- dashboard timeline tests pass
- metrics export tests pass
- report builder tests pass
- optional no-write-output CLI tests pass
- unknown event preservation tests pass
- all normal tests remain runtime-free
- no simulator/hardware dependencies are introduced
- no production dashboard claim is introduced
- no command dashboard is introduced
- no offensive automation is introduced
- no direct vehicle command behavior is introduced

## 15. Recommended Implementation Split

Recommended implementation slices:

- v2-5A: Dashboard replay payload core
- v2-5B: Metrics exporter and audit report builder
- v2-5C: Optional no-write-output replay report CLI

v2-5A should be the next implementation slice after this planning spec is
reviewed.

## 16. Status Update

v2-5A implements the mock-first dashboard replay payload core in
`src/c2/dashboard_replay.py`. Metrics export, audit report building, and
optional no-write-output CLI remain planned but not implemented.
