# GWM-UAV-C2 v2-0 Scope Freeze

## 1. Scope-freeze Purpose

This document freezes the post-v1 GWM-UAV-C2 concept before implementation.
It locks the research title, architecture boundary, module responsibilities,
defensive-risk scope, data-model direction, and first implementation rules.

`v1.0.0-research-framework-complete` remains the completed archived research
framework. GWM-UAV-C2 is an optional post-v1 extension. This scope freeze does
not change the `v1.0.0` completion claim.

## 2. Frozen Research Title

GWM-UAV-C2: A Generated World Model-based UAV Command and Mission Intelligence
Framework for Dispatching, Risk-Aware Path Planning, and Defensive Threat
Prediction

## 3. Frozen Architecture Boundary

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

This is a research architecture boundary, not a production deployment
architecture. The default path remains mock-first and import-safe. Any future
runtime path must be gated, simulator-only unless separately approved, and
blocked from physical UAV control by default.

## 4. Frozen Module List

The following modules are the only v2 core modules for now.

| Module | Responsibility | Allowed inputs | Allowed outputs | Explicit non-goals | Mock-first acceptance criteria |
| --- | --- | --- | --- | --- | --- |
| Mission Dispatcher | Validate operator intent and create mission tasks. | operator request, mission constraints, priority, allowed area | `MissionTask`, dispatch status, refusal reason | no direct vehicle command, no live runtime call | deterministic validation and JSON-safe task output |
| Fleet Manager | Track mock fleet assets, availability, assignment, and health. | `FleetAsset`, `UAVState`, mission task, health summary | assignment proposal, fleet state summary | no arming, takeoff, landing, or physical vehicle control | reproducible assignment tests with unavailable-asset reasons |
| Mission Event Bus | Move mission events between modules in a deterministic order. | `MissionEvent`, module event publications | ordered event stream, replay cursor | no distributed broker, ROS2 node, or live DDS path | in-memory publish/subscribe tests and replay ordering checks |
| Mission State Store | Persist current mission state for replay and planner inputs. | mission events, tasks, route state, risk signals | current mission snapshot, replay frame input | no database server, credentials, or network storage | in-memory store with snapshot and restore tests |
| Situation Memory / World Model | Hold observation context, world-model summaries, and risk context. | replay frame, `SensorObservation` summary, mission state | situation snapshot, context window, prediction hints | no ungated simulator ingestion or hardware telemetry | mock context assembly with deterministic summaries |
| Defensive Threat & Risk Prediction Engine | Classify defensive risk signals and explain mission risk. | telemetry anomaly, route state, communication health, airspace state | `ThreatAssessment`, `RiskSignal`, review recommendation | no offensive targeting, attack execution, payload release, weapon control, or autonomous attack-decision logic | fixture risks map only to allowed defensive categories |
| Risk-Aware Global Planner | Rank mission-level route candidates with risk and constraints. | mission task, risk signals, airspace constraints, fleet limits | `PlannedRoute`, score breakdown, route refusal reason | no production UTM, real Nav2, or live flight route dispatch | mock map route scoring and constraint tests |
| Local Replanner | Adjust near-term route segments from local state changes. | selected route, latest state, risk signal, stale-plan threshold | local route update, hold/replan recommendation | no direct command write, no runtime control loop | deterministic local replan fixtures and stale-plan handling |
| UTM-style Airspace / Geofence Layer | Check airspace, altitude, geofence, and no-fly-zone constraints. | airspace constraints, route, altitude envelope, mission area | constraint verdict, violation metadata | no real UTM integration or production airspace claim | pure-Python constraint checks over mock geometry |
| Safety Gate / Human Approval Layer | Enforce safety policy and require review before risky transitions. | planned route, local replan, risk score, CBF result, approval state | `SafetyDecision`, hold decision, approval record | no autonomous real flight, no bypass of human approval | rule-based approval/refusal tests with audit trail |
| Simulator Benchmark Layer | Compare schema compatibility across mock, Isaac, and AirSim-family paths. | backend capability summaries, replay fixture, mission scenario | benchmark summary, schema compatibility report | no automatic simulator launch or performance parity claim | read-only mock comparison report with gated optional paths |
| Dashboard Replay and Metrics Layer | Render mission timeline, replay, metrics, and audit output. | mission events, replay frames, route/risk/safety decisions | replay report, `MetricSummary`, dashboard-ready JSON | no command controls or safety bypass | deterministic report JSON and no-write-output CLI path |

## 5. Frozen Defensive-risk Scope

Allowed risk categories:

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

Explicitly excluded:

- offensive targeting
- attack execution
- payload release
- weapon control
- autonomous attack-decision logic
- real-world pursuit/intercept behavior

## 6. Frozen Data-model Direction

These are high-level data models only. v2-0 does not implement them.

| Data model | Purpose | Key fields | Producer module | Consumer module | Mock-first validation idea |
| --- | --- | --- | --- | --- | --- |
| `MissionRequest` | Capture operator intent before dispatch. | request id, operator id, objective, area, priority, constraints | C2 console | Mission Dispatcher | required-field and JSON round-trip tests |
| `MissionTask` | Represent validated task ready for assignment. | task id, request id, objective, constraints, status, priority | Mission Dispatcher | Fleet Manager, State Store | deterministic validation and status transition tests |
| `FleetAsset` | Describe a mock or simulator asset. | asset id, backend, capabilities, availability, health | Fleet Manager | Dispatcher, Planner | assignment eligibility tests |
| `UAVState` | Track vehicle state summary. | asset id, pose, velocity, battery, link health, timestamp | Fleet Manager | Risk Engine, Replanner | stale-state and serialization tests |
| `MissionEvent` | Provide append-only event record. | event id, type, timestamp, source, payload, correlation id | all C2 modules | Event Bus, State Store, Replay | deterministic ordering and schema tests |
| `RiskSignal` | Encode one defensive risk observation. | signal id, category, severity, confidence, evidence, timestamp | Risk Engine | Planner, Safety Gate, Replay | allowed-category validation tests |
| `ThreatAssessment` | Summarize mission risk state. | assessment id, risk signals, total risk, recommendation, explanation | Risk Engine | Dispatcher, Planner, Safety Gate | score bounds and explanation tests |
| `AirspaceConstraint` | Represent geofence/no-fly/altitude constraint. | constraint id, geometry, altitude bounds, validity window, source | Airspace Layer | Planner, Safety Gate | mock route intersection tests |
| `PlannedRoute` | Store global or local route candidate. | route id, waypoints, score, risk metadata, constraint verdict | Planner, Replanner | Safety Gate, Replay | deterministic scoring and JSON tests |
| `SafetyDecision` | Capture safety approval/refusal. | decision id, route id, status, reason, CBF metadata, timestamp | Safety Gate | Dispatcher, Replay | approval/refusal state tests |
| `HumanApprovalRecord` | Record operator review state. | approval id, operator id, target id, decision, timestamp, notes | Human Approval Layer | Safety Gate, Replay | required approval and audit tests |
| `ReplayFrame` | Store one replayable mission timestep. | frame id, timestamp, mission state, route, risk, safety state | State Store, Replay Layer | Dashboard, Metrics | stable fixture snapshot tests |
| `MetricSummary` | Summarize mission/replay metrics. | mission id, risk counts, replans, holds, approvals, failures | Metrics Layer | Dashboard, reports | deterministic aggregate tests |

## 7. v2-1 Implementation Readiness

v2-1 may implement:

- mission data model dataclasses
- mission event bus interface
- in-memory mission state store
- mock mission event replay
- unit tests for serialization and validation

The v2-1 planning specification is tracked in
`docs/v2_1_mission_data_model_event_bus_plan.md`.

v2-1 must not implement:

- real simulator connection
- real MAVSDK/PX4 connection
- ROS2 runtime node
- Nav2 runtime plugin
- hardware interface
- autonomous flight behavior
- offensive threat automation

## 8. Verification Rules for All v2 Slices

All v2 slices must keep:

- normal tests runnable without GPU
- normal tests runnable without Isaac Sim
- normal tests runnable without Cosys-AirSim or legacy AirSim
- normal tests runnable without ROS2
- normal tests runnable without MAVSDK or PX4
- normal tests runnable without Nav2
- normal tests runnable without hardware
- no runtime path enabled by default

## 9. Safety and Non-claim Boundaries

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

## 10. v2-0 Completion Criteria

v2-0 is complete when:

- scope-freeze doc exists
- module boundaries are frozen
- defensive-risk scope is frozen
- offensive/non-goal boundaries are explicit
- v2-1 allowed implementation scope is defined
- roadmap marks v2-0 complete
- `v1.0.0` tag/release remain untouched
- no runtime actions were run
