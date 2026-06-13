# GWM-UAV-C2 v2-4 Risk-Aware Planning and UTM-style Airspace Layer Plan

## 1. Purpose

v2-4 defines the mock-first risk-aware route planning and UTM-style airspace
constraint layer for GWM-UAV-C2.

`v1.0.0` remains the completed archive release. v2-0 froze the C2 concept and
boundaries. v2-1 completed the mission data foundation. v2-2 completed the
mock-first mission dispatch and fleet allocation layer. v2-3 completed the
mock-first defensive threat and risk prediction layer. v2-4 is the next
implementation-oriented planning slice, but this document is docs-only.

## 2. Relationship to Existing v2 Foundation

v2-4 must build only on:

- `src/c2/mission_types.py`
- `src/c2/event_bus.py`
- `src/c2/state_store.py`
- `src/c2/replay.py`
- `src/c2/mission_dispatcher.py`
- `src/c2/fleet_manager.py`
- `src/c2/risk_prediction.py`

v2-4 should use the existing dataclasses and interfaces:

- `MissionTask`
- `FleetAsset`
- `UAVState`
- `RiskSignal`
- `ThreatAssessment`
- `AirspaceConstraint`
- `PlannedRoute`
- `SafetyDecision`
- `MissionEvent`
- `MissionStateStore`
- `MissionReplayEngine`
- `MetricSummary`

No new runtime dependency is allowed.

## 3. v2-4 Allowed Scope

Allowed later implementation files:

- `src/c2/airspace.py`
- `src/c2/risk_aware_planner.py`
- `tests/test_c2_airspace.py`
- `tests/test_c2_risk_aware_planner.py`
- `tests/test_c2_planner_airspace_integration.py`

Allowed capabilities:

- mock airspace constraint validation
- geofence and no-fly-zone fixture checks
- altitude bound checks
- mission corridor checks
- simple deterministic waypoint route validation
- mock route candidate generation
- risk-aware route scoring
- constraint verdict generation
- `PlannedRoute` generation
- `MissionEvent` emission
- `MissionStateStore` integration
- `MissionReplayEngine` compatibility
- focused unit tests

## 4. v2-4 Explicit Non-goals

v2-4 must not implement:

- production UTM integration
- real airspace data ingestion
- live flight routing
- real Nav2 integration
- real `ros2_control` integration
- real simulator connection
- real MAVSDK/PX4 connection
- ROS2 runtime node
- Nav2 runtime plugin
- hardware interface
- autonomous flight behavior
- direct vehicle command
- arming
- takeoff
- landing
- mission upload to PX4 or ArduPilot
- offensive targeting
- attack execution
- payload release
- weapon control
- autonomous attack-decision logic
- real-world pursuit/intercept behavior
- network broker
- database server
- credentials or tokens
- runtime artifacts

The planner may generate route candidates and recommendations only. It must
not command, arm, launch, land, upload, or execute routes. Execution remains
outside this mock-first planning layer and must pass through safety and
human-approval gates in later slices.

## 5. Airspace Layer Design Plan

The later implementation should define a mock-first class such as:

```text
UTMAirspaceLayer
```

Recommended API:

```text
__init__(constraints: Optional[list[AirspaceConstraint]] = None)
add_constraint(constraint: AirspaceConstraint) -> AirspaceConstraint
remove_constraint(constraint_id: str) -> AirspaceConstraint
get_constraint(constraint_id: str) -> Optional[AirspaceConstraint]
list_constraints(active_only: bool = False) -> list[AirspaceConstraint]
validate_waypoint(waypoint: dict) -> dict
validate_route(waypoints: list[dict]) -> dict
constraint_verdict(waypoints: list[dict]) -> str
```

Required behavior:

- deterministic output
- JSON-safe geometry
- active/inactive constraint handling
- `altitude_min` / `altitude_max` validation
- constraint verdict limited to `valid`, `warning`, or `blocked`
- no GIS runtime dependency
- no real airspace service
- no network access
- no file writes by default

Supported mock constraint types:

- `geofence`
- `no_fly_zone`
- `altitude_band`
- `corridor`
- `restricted_zone`

## 6. Risk-Aware Planner Design Plan

The later implementation should define a mock-first class such as:

```text
RiskAwarePlanner
```

Recommended API:

```text
__init__(airspace_layer=None, event_bus=None, state_store=None)
generate_candidate_routes(task: MissionTask, asset: FleetAsset, start: dict, goal: dict, context: Optional[dict] = None) -> list[PlannedRoute]
score_route(waypoints: list[dict], risk_assessment: Optional[ThreatAssessment] = None, constraints: Optional[list[AirspaceConstraint]] = None) -> dict
select_route(routes: list[PlannedRoute]) -> PlannedRoute
create_planned_route(task_id: str, waypoints: list[dict], score: float, risk_score: float, constraint_verdict: str, metadata: Optional[dict] = None) -> PlannedRoute
publish_planned_route(route: PlannedRoute) -> MissionEvent
```

Required behavior:

- deterministic route candidates
- deterministic scoring
- JSON-safe waypoints
- bounded `risk_score` in `[0.0, 1.0]`
- `constraint_verdict` in `valid`, `warning`, or `blocked`
- no runtime side effects
- no external connections
- no command output

## 7. Route Candidate and Scoring Plan

Simple deterministic mock route candidates for later implementation:

- direct route: start -> goal
- midpoint route: start -> midpoint -> goal
- safe-offset route: start -> offset waypoint -> goal

Simple deterministic scoring approach:

```text
distance_cost = total Euclidean distance over waypoint x/y/z fields when present
risk_penalty = risk_score * risk_weight
constraint_penalty = 0 for valid, medium for warning, high for blocked
score = distance_cost + risk_penalty + constraint_penalty
```

Suggested constants:

```text
RISK_WEIGHT = 100.0
WARNING_CONSTRAINT_PENALTY = 50.0
BLOCKED_CONSTRAINT_PENALTY = 1000.0
```

These are mock-first research scoring constants, not certified safety values.

## 8. Constraint Verdict Plan

Allowed `PlannedRoute.constraint_verdict` values:

- `valid`
- `warning`
- `blocked`

Suggested policy:

- `valid`: all waypoints satisfy active constraints
- `warning`: route touches a warning or soft corridor boundary fixture
- `blocked`: route violates active `no_fly_zone`, required geofence,
  altitude bound, or `restricted_zone` fixture

Blocked route candidates may be generated for comparison but must be clearly
marked as blocked and must not be treated as executable.

## 9. Event Flow Plan

Expected event flow:

```text
MissionTask + FleetAsset + UAVState + ThreatAssessment input
-> UTMAirspaceLayer validates constraints
-> RiskAwarePlanner generates candidate routes
-> RiskAwarePlanner scores candidate routes
-> route.planned
-> MissionStateStore update
-> optional MissionReplayEngine replay
-> MetricSummary reflects planned route and blocked/warning outcomes when supported
```

Event payload expectations:

```text
route.planned -> PlannedRoute.to_dict()
payloads are JSON-safe
known invalid payloads raise ValueError
unknown event types remain preserved by state store / replay
```

## 10. State Store and Replay Integration Plan

v2-4 should not replace storage or replay. It should use:

- `MissionStateStore.apply_event()`
- `MissionStateStore.snapshot()`
- `MissionReplayEngine.replay()`
- `MetricSummary`

Requirements:

- planned routes persist in state store
- `route.planned` events are replayable
- replay frame generation remains deterministic
- blocked/warning route metadata remains visible in snapshots and replay frames
- no file writes by default
- no dashboard server
- no runtime connection

## 11. Safety Gate Boundary

v2-4 produces route candidates and scoring metadata only. v2-4 does not approve
route execution. v2-4 does not upload routes to PX4, ArduPilot, MAVSDK, ROS2,
Nav2, or any simulator. v2-4 does not command vehicles. Any later
execution-like behavior must pass through `SafetyDecision` and
`HumanApprovalRecord` flow.

## 12. Test Plan for v2-4 Implementation

Focused tests:

- `test_airspace_accepts_valid_constraint`
- `test_airspace_rejects_invalid_constraint_type`
- `test_airspace_validates_altitude_bounds`
- `test_airspace_blocks_no_fly_zone_violation`
- `test_airspace_validates_geofence_fixture`
- `test_airspace_route_verdict_valid`
- `test_airspace_route_verdict_blocked`
- `test_planner_generates_deterministic_candidate_routes`
- `test_planner_scores_route_deterministically`
- `test_planner_penalizes_risk_score`
- `test_planner_penalizes_blocked_constraint`
- `test_planner_selects_lowest_score_valid_route`
- `test_planner_creates_planned_route_json_safe`
- `test_planner_route_event_emission`
- `test_planner_state_store_integration`
- `test_planner_replay_integration`
- `test_planner_imports_without_runtime_dependencies`

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
- real GIS libraries
- real airspace services

## 13. Verification Commands for Future v2-4 Implementation

Future implementation should run:

```bash
python -m pytest tests/test_c2_airspace.py tests/test_c2_risk_aware_planner.py tests/test_c2_planner_airspace_integration.py -q
python -m pytest tests/test_c2_mission_types.py tests/test_c2_event_bus.py tests/test_c2_state_store.py tests/test_c2_replay.py tests/test_c2_mission_dispatcher.py tests/test_c2_fleet_manager.py tests/test_c2_dispatcher_fleet_integration.py tests/test_c2_risk_prediction.py tests/test_c2_risk_prediction_integration.py -q
python -m compileall -q src tests
git diff --check
rg -n "offensive attack|weapon|targeting|payload release|autonomous attack|production-ready|certified safety|real hardware validation|autonomous real flight|arming|takeoff|landing|mission upload|attack execution|weapon control|pursue|intercept|disable|jam|spoof|production UTM|live airspace" src/c2 tests docs || true
```

Expected grep hits are acceptable only in explicit non-goal/safety statements,
defensive observation categories, rejection tests, or legacy guarded-runtime
fixture names outside this slice.

## 14. v2-4 Completion Criteria

v2-4 implementation will be complete later when:

- `UTMAirspaceLayer` exists
- `RiskAwarePlanner` exists
- airspace constraint validation tests pass
- route candidate generation tests pass
- route scoring tests pass
- `route.planned` event tests pass
- state-store integration tests pass
- replay compatibility tests pass
- all normal tests remain runtime-free
- no simulator/hardware dependencies are introduced
- no production UTM claim is introduced
- no offensive automation is introduced
- no direct vehicle command behavior is introduced

## 15. Recommended Implementation Split

Recommended implementation slices:

- v2-4A: UTM-style airspace constraint core
- v2-4B: Risk-aware route candidate scoring
- v2-4C: Planner, airspace, state-store, and replay integration

v2-4A should be the next implementation slice after this planning spec is
reviewed.

## 16. Status Update

v2-4A implements the mock-first UTM-style airspace constraint core in
`src/c2/airspace.py`. Risk-aware route scoring and planner/state-store/replay
integration remain planned but not implemented.
