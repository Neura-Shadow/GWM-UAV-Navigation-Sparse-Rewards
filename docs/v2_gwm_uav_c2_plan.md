# GWM-UAV-C2 Post-v1 Extension Plan

## 1. Purpose

`v1.0.0-research-framework-complete` remains the completed archived research
framework. This document explores a post-v1 UAV command and mission
intelligence extension that can build on the archived framework without
changing its completion claim.

GWM-UAV-C2 is an optional v2 research direction for dispatching, risk-aware
path planning, defensive threat prediction, replay, and mission intelligence.
It is not a production UAV flight stack and does not enable real hardware or
autonomous real flight.

## 2. Why This Is Post-v1

`v1.0.0` completed the safe mock-first, guarded-runtime, pure-simulation
research framework. GWM-UAV-C2 is an optional extension that builds on top of
it. It does not change the `v1.0.0` completion claim, retarget the archive tag,
or reopen v1 as unfinished work.

## 3. Proposed Research Title

GWM-UAV-C2: A Generated World Model-based UAV Command and Mission Intelligence
Framework for Dispatching, Risk-Aware Path Planning, and Defensive Threat
Prediction

The Chinese title is intentionally omitted in this planning draft because the
source text was corrupted/mojibake. A corrected Chinese title can be added in a
later review without changing the scope of this post-v1 plan.

## 4. System Architecture

```text
Operator Dashboard / C2 Console
-> Mission Dispatcher
-> Fleet Manager
-> World Model / Situation Memory
-> Defensive Threat & Risk Prediction
-> Global Path Planner
-> Local Replanner
-> UTM-style Airspace / Geofence Layer
-> Safety Gate / CBF / Human Approval
-> PX4 / ArduPilot / MAVSDK / MAVLink guarded adapter
-> Isaac / Cosys-AirSim / mock simulator backend
-> Telemetry / Replay / Metrics / Database
```

The architecture is command-and-mission intelligence for research simulation
and replay. Runtime paths remain gated, simulator-backed, or mock-first. Any
future safety-critical transition requires human approval.

## 5. Core Modules

### Mission Dispatcher

- Purpose: Convert operator intent into structured mission requests,
  priorities, constraints, and reviewable task packets.
- Inputs: operator mission request, region constraints, vehicle availability,
  mission priority, risk policy.
- Outputs: mission task packet, dispatch status, human-review metadata.
- Mock-first implementation idea: pure-Python dataclasses, in-memory mission
  queue, deterministic validation, and JSON replay fixtures.
- Future optional runtime path: dashboard form/API integration and guarded
  simulator mission injection.
- Safety boundary: never issues direct physical UAV commands; all dispatches
  remain reviewable and pass through safety/human-approval layers.

### Fleet Manager

- Purpose: Track simulated vehicle availability, assignments, health, and
  mission allocation state.
- Inputs: mission task packets, mock or simulator telemetry, vehicle registry,
  health summaries.
- Outputs: assignment plan, vehicle state summary, unavailable-vehicle reasons.
- Mock-first implementation idea: in-memory fleet registry with deterministic
  assignment policies and test fixtures.
- Future optional runtime path: read-only simulator telemetry adapters and
  guarded SITL fleet status ingestion.
- Safety boundary: does not arm, take off, land, or command physical vehicles.

### Situation Memory / World Model

- Purpose: Maintain a mission-aware memory of observations, map context, risk
  events, route candidates, and generated future rollouts.
- Inputs: `SensorObservation` windows, mission state, map/geofence data,
  telemetry, risk events.
- Outputs: situation snapshot, context window, predicted risk hints, replayable
  mission timeline.
- Mock-first implementation idea: append-only event store plus lightweight
  world-model context summaries using existing GWM abstractions.
- Future optional runtime path: simulator-backed observation ingestion from
  Isaac, Cosys-AirSim, ROS2, or SITL reports.
- Safety boundary: predictions inform planning and review; they do not bypass
  CBF, geofence, or human-approval gates.

### Defensive Threat & Risk Prediction Engine

- Purpose: Estimate defensive mission risks and flag anomalous conditions that
  may require replanning, operator review, or mission hold.
- Inputs: telemetry stream, sensor health, communication quality, route state,
  airspace constraints, nearby-vehicle observations.
- Outputs: risk score, risk category labels, explanation metadata, recommended
  defensive action such as hold, replan, or request review.
- Mock-first implementation idea: deterministic rule engine with fixture-based
  anomaly scenarios and generated-world-model risk features.
- Future optional runtime path: simulator-event ingestion and replay evaluation
  against synthetic disruption scenarios.
- Safety boundary: defensive prediction only; no offensive targeting, attack
  execution, payload release, weapon control, or autonomous attack decision
  logic.

### Risk-Aware Global Planner

- Purpose: Generate route candidates that balance mission objective,
  geofence/no-fly-zone constraints, risk scores, uncertainty, and energy.
- Inputs: mission task packet, fleet state, risk map, geofence/no-fly-zone
  layers, world-model predictions.
- Outputs: ranked global route candidates, score breakdowns, route risk
  metadata.
- Mock-first implementation idea: grid/graph planner over synthetic maps with
  deterministic scoring and JSON outputs.
- Future optional runtime path: simulator-map ingestion and SITL dry-run route
  replay.
- Safety boundary: produces recommendations only; execution remains gated by
  local replanning, CBF, and human approval.

### Local Replanner

- Purpose: Adjust near-term route segments when telemetry, obstacles, or risk
  events change.
- Inputs: selected global route, latest observation, local risk events,
  vehicle dynamics envelope, stale-plan timeout.
- Outputs: local route update, hold/replan recommendation, explanation
  metadata.
- Mock-first implementation idea: deterministic local path adjustment around
  fixture obstacles and risk zones.
- Future optional runtime path: simulator-only local replanning with fake or
  gated runtime observations.
- Safety boundary: cannot directly issue unsafe commands; outputs are
  candidate plans reviewed by safety and human-approval layers.

### UTM-style Airspace Layer

- Purpose: Represent airspace constraints, geofences, no-fly zones, mission
  corridors, and deconfliction metadata.
- Inputs: map constraints, mission area, altitude bounds, vehicle routes,
  simulated airspace events.
- Outputs: airspace validity checks, route conflicts, no-fly-zone violations,
  geofence metadata.
- Mock-first implementation idea: pure-Python polygon/altitude fixtures and
  deterministic route intersection checks.
- Future optional runtime path: simulator or replay import of airspace layers.
- Safety boundary: research-grade constraint checking only; no production UTM
  claim and no real airspace integration.

### Safety Gate / Human Approval Layer

- Purpose: Enforce safety policy, CBF-style command filtering, operator review,
  and explicit approval before any future safety-critical transition.
- Inputs: route candidates, local replans, risk scores, vehicle state, CBF
  checks, operator approval state.
- Outputs: approved/blocked decision, safe hold command, refusal reason,
  audit trail.
- Mock-first implementation idea: rule-based approval state machine and
  deterministic CBF/geofence checks over mock commands.
- Future optional runtime path: guarded simulator-only approval workflow and
  no-write-output dry runs.
- Safety boundary: refuses real hardware and autonomous real flight by default;
  human approval is required for future safety-critical transitions.

### Simulator Benchmark Layer

- Purpose: Compare mission/replanning schema compatibility across mock, Isaac,
  and Cosys-AirSim-family simulator backends.
- Inputs: mission scenarios, backend capability reports, replay fixtures,
  simulator availability metadata.
- Outputs: schema compatibility report, observation availability summary,
  frame metadata, safety-gate behavior summary.
- Mock-first implementation idea: read-only benchmark report over mock and
  existing capability summaries.
- Future optional runtime path: gated simulator-only smoke or replay
  comparison; no automatic simulator launch.
- Safety boundary: no simulator performance parity claim and no live runtime
  validation without explicit gates.

### Dashboard Replay and Metrics Layer

- Purpose: Provide operator-facing replay, mission audit, metrics summaries,
  and risk-event timelines.
- Inputs: mission event log, route plans, risk predictions, safety decisions,
  simulator or mock telemetry.
- Outputs: replay timeline, metrics report, dashboard-ready JSON, audit
  artifacts.
- Mock-first implementation idea: static JSON/Markdown reports and deterministic
  replay fixtures.
- Future optional runtime path: local dashboard viewer or hosted artifact fed by
  simulation-only logs.
- Safety boundary: dashboard is observational and review-oriented; it does not
  issue direct vehicle commands.

## 6. Defensive Threat/Risk Scope

Allowed defensive risk categories:

- GPS spoofing risk
- GPS / RF jamming risk
- communication degradation
- sensor corruption
- hostile UAV proximity
- collision risk
- geofence / no-fly-zone violation
- weather or wind disturbance
- telemetry anomaly
- mission command anomaly

This scope is defensive only. The project must not describe or implement
offensive targeting, attack execution, payload release, weapon control, or
autonomous attack-decision logic.

## 7. Proposed v2 Phase Plan

### v2-0: C2 Concept and Scope Freeze

- Goal: Freeze terminology, safety boundaries, module responsibilities, and
  allowed defensive risk categories.
- Files likely touched: `docs/v2_gwm_uav_c2_plan.md`, `docs/roadmap.md`, and a
  future `docs/v2_c2_scope.md`.
- What not to do: do not implement runtime code, live validation, offensive
  automation, hardware paths, simulator launches, or command adapters.
- Verification commands: `git diff --check` and defensive-scope grep over docs.
- Completion criteria: docs clearly separate v1 completion from optional v2
  planning and list all safety/non-goals.

### v2-1: Mission Data Model and Event Bus

- Goal: Define mission, fleet, risk-event, approval, and replay event schemas.
- Files likely touched: future `src/c2/mission_types.py`,
  `src/c2/event_bus.py`, `tests/test_c2_mission_types.py`, and docs.
- What not to do: do not connect to real vehicles, simulators, ROS2, MAVSDK, or
  hardware.
- Verification commands: focused unit tests, `python -m compileall -q src tests`,
  and schema serialization checks.
- Completion criteria: schemas are JSON-safe, deterministic, import-safe, and
  covered by mock-only tests.

### v2-2: Mission Dispatcher and Fleet Manager

- Goal: Implement mock-first mission queueing, task assignment, and fleet state
  tracking.
- Files likely touched: future `src/c2/mission_dispatcher.py`,
  `src/c2/fleet_manager.py`, and focused tests.
- What not to do: do not arm, take off, land, command vehicles, or connect to
  live runtimes.
- Verification commands: mock-only unit tests, compileall, and JSON replay
  snapshot checks.
- Completion criteria: dispatcher and fleet manager produce deterministic
  assignments and refusal reasons.

### v2-3: Defensive Threat & Risk Prediction Engine

- Goal: Add defensive risk scoring and anomaly labels for mission replay and
  planning.
- Files likely touched: future `src/c2/risk_prediction.py`,
  `tests/test_c2_risk_prediction.py`, and risk-scope docs.
- What not to do: do not add offensive targeting, attack execution, payload
  logic, weapon control, or autonomous attack decisions.
- Verification commands: fixture-based risk tests, forbidden-term grep, and
  JSON-safe output checks.
- Completion criteria: only allowed defensive risk categories are emitted and
  every risk has explanation metadata.

### v2-4: Risk-Aware Path Planning and UTM-style Airspace Layer

- Goal: Rank global route candidates and validate airspace/geofence/no-fly-zone
  constraints in mock maps.
- Files likely touched: future `src/c2/risk_aware_planner.py`,
  `src/c2/airspace.py`, and focused tests.
- What not to do: do not implement production UTM integration, real Nav2, real
  `ros2_control`, or live flight routing.
- Verification commands: mock map tests, route scoring tests, geofence/no-fly
  violation tests, and compileall.
- Completion criteria: candidate plans include scores, constraint metadata, and
  safety-gate inputs.

### v2-5: Dashboard Replay and Metrics

- Goal: Export mission timelines, route/risk decisions, and safety outcomes for
  review.
- Files likely touched: future `scripts/run_c2_replay_report.py`,
  `src/c2/replay.py`, docs, and tests.
- What not to do: do not build command controls that bypass human approval or
  safety gates.
- Verification commands: report-generation tests, JSON serialization tests, and
  no-write-output CLI checks.
- Completion criteria: replay artifacts are deterministic and contain no
  credentials, runtime logs, or hardware outputs.

### v2-6: Optional Simulator Benchmark Integration

- Goal: Compare schema compatibility and safety-gate behavior across mock,
  Isaac, and Cosys-AirSim-family simulator backends.
- Files likely touched: future benchmark runner docs, `src/c2/benchmarking.py`,
  and tests.
- What not to do: do not launch simulators automatically, claim simulator
  performance parity, run hardware checks, or connect to real vehicles.
- Verification commands: mock-only benchmark tests and optional-runtime skipped
  tests behind explicit gates.
- Completion criteria: benchmark reports clearly distinguish mock/default
  checks from optional gated simulator checks.

## 8. First Implementation Slice Recommendation

Start with `v2-0: C2 Concept and Scope Freeze`.

This first slice should remain docs-only. It should lock the allowed defensive
risk categories, module vocabulary, data-model boundaries, and non-goals before
any code, runtime path, or dashboard work begins.

## 9. Safety and Non-goals

- No real hardware validation.
- No autonomous real flight.
- No offensive attack automation.
- No weaponized decision-making.
- No certified safety claim.
- No production-readiness claim.
- No simulator performance parity claim.
- No automatic simulator launch.
- No automatic PX4 launch.
- No real Nav2 / `ros2_control` production integration.
- All runtime paths remain gated.
- Human approval is required for any future safety-critical transition.

## 10. Relationship to v1.0.0

`v1.0.0` remains the completed archive release. The post-v1 C2 plan is optional
future work. The `v1.0.0-research-framework-complete` tag and release must not
be moved.
