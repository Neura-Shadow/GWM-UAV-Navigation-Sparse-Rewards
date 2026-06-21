# GWM-UAV-C2 v2-1 Mission Data Model and Event Bus Plan

## 1. Purpose

v2-1 will define the mock-first mission data foundation for GWM-UAV-C2.

`v1.0.0` remains the completed archive release. v2-0 froze the C2 concept and
boundaries. v2-1 is the first implementation-oriented planning slice, but this
document is docs-only.

Status update: v2-1A implements the mission dataclasses and validation
helpers in `src/c2/mission_types.py`.

Status update: v2-1B now implements the mock-first event bus and in-memory
mission state store in `src/c2/event_bus.py` and `src/c2/state_store.py`.

Status update: v2-1C implements mock replay and metrics aggregation in
`src/c2/replay.py`. With v2-1A, v2-1B, and v2-1C complete, v2-1 provides the
mock-first mission data foundation for later C2 modules.

## 2. Relationship to v2-0 Scope Freeze

v2-0 froze:

- module boundaries
- defensive-risk scope
- data-model direction
- runtime non-goals
- safety boundaries

v2-1 must follow `docs/v2_gwm_uav_c2_scope_freeze.md`. Any implementation
that expands beyond the frozen boundaries needs a separate scope review before
code is added.

## 3. v2-1 Allowed Scope

Allowed later implementation files:

- `src/c2/__init__.py`
- `src/c2/mission_types.py`
- `src/c2/event_bus.py`
- `src/c2/state_store.py`
- `src/c2/replay.py`
- `tests/test_c2_mission_types.py`
- `tests/test_c2_event_bus.py`
- `tests/test_c2_state_store.py`

Allowed capabilities:

- pure-Python dataclasses
- JSON-safe serialization
- deterministic event ordering
- in-memory event bus
- in-memory mission state store
- mock replay frames
- validation helpers
- focused unit tests

## 4. v2-1 Explicit Non-goals

v2-1 must not implement:

- real simulator connection
- real MAVSDK/PX4 connection
- ROS2 runtime node
- Nav2 runtime plugin
- hardware interface
- autonomous flight behavior
- offensive threat automation
- weaponized targeting
- payload release
- autonomous attack-decision logic
- network broker
- database server
- credentials or tokens
- runtime artifacts

## 5. Frozen Data Models for v2-1

All models should use standard Python types only: `str`, `int`, `float`,
`bool`, `list`, `dict`, `Optional`, and `Literal` / `Enum` where useful. No
model should depend on numpy, torch, ROS2, MAVSDK, simulator SDKs, or external
runtime packages.

### MissionRequest

- Purpose: capture operator intent before validation and dispatch.
- Suggested dataclass fields: `request_id: str`, `operator_id: str`,
  `objective: str`, `priority: int`, `area: dict`, `constraints: dict`,
  `created_at: float`, `metadata: dict`.
- Required validation rules: non-empty ids, priority within an agreed small
  integer range, JSON-safe area/constraints, no direct command payload.
- JSON serialization behavior: round-trip with plain dictionaries and stable
  key names.
- Example mock fixture: request for inspection of `area={"zone": "alpha"}` with
  `priority=2` and no runtime target.
- Producer module: Operator Dashboard / C2 Console.
- Consumer module: Mission Dispatcher.

### MissionTask

- Purpose: represent a validated mission unit ready for assignment.
- Suggested dataclass fields: `task_id: str`, `request_id: str`,
  `objective: str`, `status: str`, `priority: int`, `constraints: dict`,
  `assigned_asset_id: Optional[str]`, `created_at: float`, `metadata: dict`.
- Required validation rules: task/request ids required, status is one of
  `pending`, `assigned`, `blocked`, `completed`, or `cancelled`, assigned asset
  may be absent until allocation.
- JSON serialization behavior: serialize `None` assigned asset as JSON `null`.
- Example mock fixture: pending route-inspection task with geofence constraint.
- Producer module: Mission Dispatcher.
- Consumer module: Fleet Manager, Mission State Store.

### FleetAsset

- Purpose: describe a mock or simulator fleet asset.
- Suggested dataclass fields: `asset_id: str`, `backend: str`,
  `capabilities: list[str]`, `available: bool`, `health: dict`,
  `current_task_id: Optional[str]`, `metadata: dict`.
- Required validation rules: asset id required, backend must be known to the
  test fixture, capabilities list must contain strings, unavailable assets
  cannot receive new assignments.
- JSON serialization behavior: no object references; nested health is a plain
  dictionary.
- Example mock fixture: `backend="mock"`, `capabilities=["survey"]`,
  `available=True`.
- Producer module: Fleet Manager.
- Consumer module: Mission Dispatcher, Risk-Aware Global Planner.

### UAVState

- Purpose: summarize vehicle state for planning and stale-state checks.
- Suggested dataclass fields: `asset_id: str`, `timestamp: float`,
  `position: dict`, `velocity: dict`, `battery: float`, `link_quality: float`,
  `mode: str`, `metadata: dict`.
- Required validation rules: timestamp non-negative, battery/link quality in
  `[0.0, 1.0]`, position and velocity contain JSON-safe numeric values.
- JSON serialization behavior: stable dictionaries for position and velocity.
- Example mock fixture: stationary mock UAV with battery `0.9` and link quality
  `1.0`.
- Producer module: Fleet Manager.
- Consumer module: Defensive Threat & Risk Prediction Engine, Local Replanner.

### MissionEvent

- Purpose: provide the append-only mission event record.
- Suggested dataclass fields: `event_id: str`, `event_type: str`,
  `timestamp: float`, `source: str`, `payload: dict`,
  `correlation_id: Optional[str]`, `metadata: dict`.
- Required validation rules: ids/type/source required, timestamp non-negative,
  payload JSON-safe, no credentials or runtime handles in payload.
- JSON serialization behavior: payload and metadata remain plain dictionaries.
- Example mock fixture: `event_type="mission.requested"` with a
  `MissionRequest` payload.
- Producer module: all C2 modules.
- Consumer module: Mission Event Bus, Mission State Store, Dashboard Replay and
  Metrics Layer.

### RiskSignal

- Purpose: encode one defensive risk observation.
- Suggested dataclass fields: `signal_id: str`, `category: str`,
  `severity: float`, `confidence: float`, `evidence: dict`,
  `timestamp: float`, `metadata: dict`.
- Required validation rules: category must be one of the frozen defensive risk
  categories, severity/confidence in `[0.0, 1.0]`, evidence JSON-safe.
- JSON serialization behavior: category serializes as a string, not as runtime
  class state.
- Example mock fixture: `category="communication degradation"`,
  `severity=0.4`, `confidence=0.8`.
- Producer module: Defensive Threat & Risk Prediction Engine.
- Consumer module: Risk-Aware Global Planner, Safety Gate / Human Approval
  Layer, Dashboard Replay and Metrics Layer.

### ThreatAssessment

- Purpose: summarize mission risk state from risk signals.
- Suggested dataclass fields: `assessment_id: str`, `mission_id: str`,
  `risk_signals: list[dict]`, `total_risk: float`, `recommendation: str`,
  `explanation: str`, `timestamp: float`.
- Required validation rules: total risk in `[0.0, 1.0]`, recommendation is one
  of `continue`, `hold`, `replan`, or `request_review`, explanation required
  for non-zero risk.
- JSON serialization behavior: nested risk signals serialize as dictionaries.
- Example mock fixture: assessment recommending `replan` after geofence risk.
- Producer module: Defensive Threat & Risk Prediction Engine.
- Consumer module: Mission Dispatcher, Risk-Aware Global Planner, Safety Gate /
  Human Approval Layer.

### AirspaceConstraint

- Purpose: represent geofence, no-fly-zone, altitude, and corridor constraints.
- Suggested dataclass fields: `constraint_id: str`, `constraint_type: str`,
  `geometry: dict`, `altitude_min: Optional[float]`,
  `altitude_max: Optional[float]`, `active: bool`, `metadata: dict`.
- Required validation rules: known constraint type, geometry JSON-safe,
  altitude min not greater than altitude max when both are present.
- JSON serialization behavior: geometry remains a plain dictionary with no GIS
  runtime objects.
- Example mock fixture: active polygon no-fly-zone with altitude ceiling.
- Producer module: UTM-style Airspace / Geofence Layer.
- Consumer module: Risk-Aware Global Planner, Safety Gate / Human Approval
  Layer.

### PlannedRoute

- Purpose: store global or local route candidate and scoring metadata.
- Suggested dataclass fields: `route_id: str`, `task_id: str`,
  `waypoints: list[dict]`, `score: float`, `risk_score: float`,
  `constraint_verdict: str`, `metadata: dict`.
- Required validation rules: route/task ids required, waypoint list non-empty,
  scores finite and JSON-safe, verdict in `valid`, `warning`, or `blocked`.
- JSON serialization behavior: waypoints serialize as plain dictionaries.
- Example mock fixture: three-waypoint route with risk score `0.2`.
- Producer module: Risk-Aware Global Planner, Local Replanner.
- Consumer module: Safety Gate / Human Approval Layer, Dashboard Replay and
  Metrics Layer.

### SafetyDecision

- Purpose: capture approval, refusal, hold, or review decision.
- Suggested dataclass fields: `decision_id: str`, `target_id: str`,
  `status: str`, `reason: str`, `cbf_metadata: dict`,
  `requires_human_approval: bool`, `timestamp: float`.
- Required validation rules: status in `approved`, `blocked`, `hold`, or
  `needs_review`, reason required for non-approved decisions, CBF metadata
  JSON-safe.
- JSON serialization behavior: booleans and metadata serialize directly.
- Example mock fixture: blocked route due to no-fly-zone violation.
- Producer module: Safety Gate / Human Approval Layer.
- Consumer module: Mission Dispatcher, Dashboard Replay and Metrics Layer.

### HumanApprovalRecord

- Purpose: record operator review state for safety-critical transitions.
- Suggested dataclass fields: `approval_id: str`, `operator_id: str`,
  `target_id: str`, `decision: str`, `notes: str`, `timestamp: float`,
  `metadata: dict`.
- Required validation rules: decision in `approved`, `rejected`, or `deferred`,
  operator id required, notes JSON-safe string.
- JSON serialization behavior: no credential or identity token fields.
- Example mock fixture: operator rejects a high-risk route candidate.
- Producer module: Safety Gate / Human Approval Layer.
- Consumer module: Mission State Store, Dashboard Replay and Metrics Layer.

### ReplayFrame

- Purpose: store one replayable mission timestep.
- Suggested dataclass fields: `frame_id: str`, `timestamp: float`,
  `mission_snapshot: dict`, `events: list[dict]`, `risk_summary: dict`,
  `route_summary: dict`, `safety_summary: dict`.
- Required validation rules: timestamp non-negative, nested fields JSON-safe,
  event order preserved.
- JSON serialization behavior: full frame serializes to a dictionary suitable
  for file output later, but v2-1 writes no files by default.
- Example mock fixture: frame after mission assignment with one risk signal.
- Producer module: Mock replay layer.
- Consumer module: Dashboard Replay and Metrics Layer.

### MetricSummary

- Purpose: summarize replay and mission metrics.
- Suggested dataclass fields: `mission_id: str`, `event_count: int`,
  `risk_counts: dict`, `replan_count: int`, `hold_count: int`,
  `approval_count: int`, `blocked_count: int`, `metadata: dict`.
- Required validation rules: counts non-negative, risk-count keys are allowed
  categories, metadata JSON-safe.
- JSON serialization behavior: stable dictionary output with deterministic key
  names.
- Example mock fixture: summary with one replan, one hold, and zero runtime
  actions.
- Producer module: Dashboard Replay and Metrics Layer.
- Consumer module: reports, dashboard JSON, audit docs.

## 6. Event Bus Design Plan

The mock-first event bus should expose:

```text
publish(event)
subscribe(event_type, handler)
drain()
replay(events)
clear()
```

Requirements:

- deterministic event ordering
- append-only event history
- JSON-safe event payloads
- no threading required
- no async required
- no network broker
- no ROS2/DDS dependency
- no runtime side effects

Implementation direction for later v2-1B: store events in insertion order,
invoke matching handlers synchronously, keep handler errors explicit, and make
`drain()` return ordered events without deleting history unless `clear()` is
called.

## 7. Mission State Store Design Plan

The in-memory mission state store should expose:

```text
apply_event(event)
snapshot()
restore(snapshot)
get_task(task_id)
get_asset(asset_id)
list_events()
```

Requirements:

- deterministic replay
- JSON-safe snapshots
- no database dependency
- no file writes by default
- no credentials
- no network access

Implementation direction for later v2-1B: maintain dictionaries keyed by task
id and asset id, append every applied event to event history, and allow
`restore(snapshot)` to rebuild state from a JSON-safe snapshot.

## 8. Mock Replay Design Plan

Mock replay flow:

```text
MissionEvent list -> MissionStateStore replay -> ReplayFrame list -> MetricSummary
```

Requirements:

- deterministic output
- no runtime connection
- no simulator connection
- no file writes unless explicitly requested later
- no dashboard server

Implementation direction for later v2-1C: convert ordered events into
`ReplayFrame` records after each state transition, then aggregate counts and
risk categories into `MetricSummary`.

## 9. Test Plan for v2-1 Implementation

Focused tests:

- `test_mission_request_json_roundtrip`
- `test_mission_task_validation`
- `test_fleet_asset_availability`
- `test_uav_state_stale_detection`
- `test_event_bus_publish_order`
- `test_event_bus_subscribe_handler`
- `test_state_store_apply_event`
- `test_state_store_snapshot_restore`
- `test_replay_frame_generation`
- `test_metric_summary_aggregation`
- `test_forbidden_risk_category_rejected`
- `test_defensive_risk_category_accepted`

## 10. Verification Commands for Future v2-1 Implementation

Future implementation should run:

```bash
python -m pytest tests/test_c2_mission_types.py tests/test_c2_event_bus.py tests/test_c2_state_store.py -q
python -m compileall -q src tests
git diff --check
rg -n "offensive attack|weapon|targeting|payload release|autonomous attack|production-ready|certified safety|real hardware validation|autonomous real flight" src/c2 tests docs || true
```

Expected grep hits are acceptable only in explicit non-goal/safety statements.

## 11. v2-1 Completion Criteria

v2-1 implementation will be complete later when:

- mission dataclasses exist
- JSON roundtrip tests pass
- event bus deterministic ordering tests pass
- state store snapshot/restore tests pass
- mock replay tests pass
- forbidden risk categories are rejected
- all normal tests remain runtime-free
- no simulator/hardware dependencies are introduced
- no offensive automation is introduced

## 12. Recommended Implementation Split

- v2-1A: Mission dataclasses and validation
- v2-1B: Event bus and state store
- v2-1C: Mock replay and metrics

v2-1A should be the next implementation slice after this planning spec is
reviewed.
