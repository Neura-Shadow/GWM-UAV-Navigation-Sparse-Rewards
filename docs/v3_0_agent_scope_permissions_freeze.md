# GWM-UAV-C2 AgentOps v3-0 Scope and Permission Freeze

## 1. Purpose

This document freezes the AgentOps v3 research boundary before any agent,
model-provider, orchestration, tool-calling, dashboard, or runtime code is
implemented.

`v1.0.0-research-framework-complete` remains the completed archived research
framework. GWM-UAV-C2 v2 remains the completed deterministic, mock-first, and
readiness-oriented C2 foundation. AgentOps v3 is an optional extension above
that foundation and does not reopen v1 or v2 as unfinished work.

Agents may analyze, propose, coordinate, and request validation. Existing v2
modules validate and own mission state, fleet assignments, defensive risk,
airspace verdicts, route scoring, replay, metrics, and benchmark readiness.

## 2. Frozen Research Title

GWM-UAV-C2 AgentOps:
A Human-Supervised Multi-Agent Ground Operations Center for Mission Dispatch,
Defensive Risk Assessment, Risk-Aware Planning, and Simulator-Gated UAV Fleet
Coordination

## 3. Frozen System Boundary

The system boundary is:

```text
Human operator
-> observational and approval-oriented C2 interface
-> proposal-only Supervisor and specialist agents
-> typed agent messages
-> allowlisted tool registry and permission checks
-> deterministic GWM-UAV-C2 v2 authorities
-> deterministic safety policy / CBF / human approval
-> optional explicitly gated simulation and SITL validation
-> redacted telemetry / replay / audit / metrics
```

This is a human-supervised research ground operations center. It is not a
production battle-management system, flight-control system, real-hardware
controller, or autonomous real-flight stack.

The dashboard concept is observational and approval-oriented. It is not a
command dashboard and contains no arm, takeoff, land, execute-route,
mission-upload, or payload controls.

## 4. Frozen Agent Roster

The v3 roster is limited to:

- Supervisor Agent
- Mission Agent
- Fleet Agent
- Situation Awareness Agent
- Defensive Risk Agent
- Planning and Airspace Agent
- Simulation Validation Agent
- Safety Review Agent
- Audit and Replay Agent

Adding an agent role requires a future scope review, an explicit permission
manifest, typed contracts, tool allowlisting, failure behavior, and audit
coverage. No dynamically created role receives permissions by inheritance.

## 5. Agent Permission Matrix

| Agent | Read state | Submit proposal | Validated state write | Simulator readiness | Simulator command request | SITL command request | Human approval authority | Real hardware |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Supervisor Agent | yes | yes | no direct write | yes | request only | request only | no | prohibited |
| Mission Agent | mission scope | yes | validated event only | no | no | no | no | prohibited |
| Fleet Agent | fleet scope | yes | validated event only | no | no | no | no | prohibited |
| Situation Awareness Agent | yes | summary only | no | read-only | no | no | no | prohibited |
| Defensive Risk Agent | risk/context scope | yes | validated risk events only | read-only | no | no | no | prohibited |
| Planning and Airspace Agent | task/fleet/risk/airspace | yes | validated route event only | read-only | no | no | no | prohibited |
| Simulation Validation Agent | approved validation package | validation result | validated simulator result event only | yes | gated request | gated request | no | prohibited |
| Safety Review Agent | yes | approve/block/review recommendation | validated safety event only | yes | no direct command | no direct command | no | prohibited |
| Audit and Replay Agent | yes | report only | no | read-only | no | no | no | prohibited |

Only a human operator may produce the authoritative `HumanApprovalRecord` for
a safety-critical transition. An agent recommendation, including a Safety
Review Agent recommendation, is never a human approval.

## 6. Tool Access Matrix

All entries describe future allowlisted access. They do not grant executable
capabilities in v3-0.

| Agent | Allowlisted deterministic tools | Access class | Approval ceiling | Explicit denials |
| --- | --- | --- | --- | --- |
| Supervisor Agent | workflow state, typed message bus, permission query, approval-request builder | read and proposal coordination | may request Levels 3 and 4; cannot approve | direct store write, shell, network, simulator command, vehicle command |
| Mission Agent | `MissionDispatcher`, `MissionEventBus`, read-only `MissionStateStore` query | state proposal; validated mission event | Level 1 proposal only | fleet override, route execution, mission upload, runtime call |
| Fleet Agent | `FleetManager`, fleet/UAV read queries, `MissionEventBus` | state proposal; validated assignment event | Level 1 proposal only | arming, takeoff, landing, telemetry connection, hardware access |
| Situation Awareness Agent | `MissionStateStore`, `MissionReplayEngine`, GWM/situation-memory read facade, `DashboardReplayBuilder` | `READ_ONLY` | Level 0 only | state mutation, runtime probe, hardware access |
| Defensive Risk Agent | `DefensiveRiskPredictor`, read-only state query, `MissionEventBus` | state proposal; validated defensive-risk event | Level 1 proposal only | offensive recommendation, jamming/spoofing action, pursuit/intercept, weaponized decision |
| Planning and Airspace Agent | `UTMAirspaceLayer`, `RiskAwarePlanner`, task/fleet/risk readers | state proposal; validated route event | Level 1 proposal only | route execution, mission upload, Nav2, MAVSDK, PX4, simulator control |
| Simulation Validation Agent | `C2BenchmarkReadinessBuilder`, simulator backend registry, guarded readiness/validation adapters | simulator read-only or gated request | Level 3 or 4 request with human approval | automatic launch, self-enabled gates, real hardware, parity claim |
| Safety Review Agent | deterministic safety policy, CBF/geofence result readers, risk/route readers, approval-request builder | read, block, or recommendation; validated safety event | recommends only; cannot author human approval | direct command, gate activation, approval fabrication, certification claim |
| Audit and Replay Agent | `DashboardReplayBuilder`, `C2MetricsExporter`, `C2ReplayReportBuilder`, `MissionReplayEngine`, `C2BenchmarkReadinessBuilder` | `READ_ONLY` and report output | Level 0 only | mission mutation, runtime connection, command controls, credential access |

Every future registry entry must declare tool name and version, allowed agents,
input/output schemas, read/write and side-effect classes, required approval
level, runtime gates, timeout, fallback behavior, and audit fields.

The frozen side-effect classes are:

- `READ_ONLY`
- `STATE_PROPOSAL`
- `VALIDATED_STATE_WRITE`
- `SIMULATOR_READ_ONLY`
- `SIMULATOR_COMMAND_GATED`
- `SITL_COMMAND_GATED`
- `REAL_HARDWARE_PROHIBITED`

`REAL_HARDWARE_PROHIBITED` is a denial marker. No agent may receive it as an
executable capability.

## 7. Human Approval Levels

| Level | Scope | Examples | Human approval rule |
| --- | --- | --- | --- |
| 0 | Read-only analysis | state/timeline query, metrics, benchmark readiness | not required |
| 1 | Proposal generation | mission, assignment, risk, route proposal | not required to compute; required before a safety-critical transition |
| 2 | Mock simulation evaluation | mock replay, benchmark, route evaluation | not required unless policy says otherwise |
| 3 | Gated external simulator validation | Isaac, Cosys-AirSim, ROS2 simulation validation | explicit gate, operator approval, safety recommendation, and audit required |
| 4 | PX4 SITL / MAVSDK simulated command validation | approved SITL-only validation request | explicit gates, operator approval, simulator-only flags, deterministic safety/CBF/geofence/staleness checks, and audit required |
| 5 | Real hardware or autonomous real flight | physical UAV or autonomous flight | prohibited and out of scope |

Agents must never self-escalate approval levels. Missing, stale, expired, or
mismatched approval produces `not_ready` or `blocked`; it never produces an
implicit approval.

## 8. Simulator and SITL Gate Boundary

Normal tests remain mock-first and runtime-free. No simulator, ROS2, MAVSDK,
PX4 SITL, Nav2, or hardware process is started by default.

The Simulation Validation Agent may prepare a request and inspect deterministic
readiness metadata. A Level 3 or Level 4 action requires:

1. an allowlisted tool with a matching typed request
2. explicit environment/runtime gates already enabled by the operator
3. an authoritative, unexpired human approval record
4. a Safety Review Agent recommendation
5. simulator-only deployment flags
6. structured audit start, result, error, and cleanup records

Isaac Sim / Isaac Lab remains the guarded full-stack simulator mainline.
Cosys-AirSim remains the preferred AirSim-family runtime, and legacy AirSim is
fallback only. PX4 is SITL-only. MAVSDK is permitted only through the existing
guarded SITL validation boundary.

The agent may not silently launch or connect to a simulator, launch PX4,
self-enable gates, or convert a readiness result into a runtime-validation
claim. No simulator performance parity claim is allowed.

## 9. Prohibited Capabilities

The current v3 design prohibits:

- direct vehicle command behavior
- route execution or mission upload
- arming, takeoff, landing, or payload behavior
- physical UAV or flight-controller connection
- real hardware or autonomous real flight
- automatic simulator launch or automatic PX4 launch
- ungated simulator, ROS2, MAVSDK, PX4, Nav2, or SITL access
- offensive targeting or attack execution
- weapon control or payload release
- autonomous attack-decision logic
- real-world pursuit or intercept behavior
- RF jamming or GPS spoofing actions
- human-approval fabrication or self-approval
- safety-rule bypass or runtime-gate self-enablement
- unrestricted shell, network, file-system, or code-execution access
- dynamic plugin installation or self-modifying permissions
- production-readiness, simulator-parity, or certified-safety claims

GPS spoofing and GPS/RF jamming may appear only as observed defensive risk
categories, never as agent actions.

## 10. Failure and Degraded-mode Policy

| Condition | Frozen behavior |
| --- | --- |
| agent timeout | mark task `timed_out`, preserve evidence, return control to Supervisor |
| invalid schema | reject proposal and audit the validation error |
| tool failure | record failure and return no-op, hold, or `request_review` recommendation |
| stale state | hold or `request_review` |
| conflicting proposals | escalate to Safety Review Agent and human operator |
| simulator unavailable | report `runtime_unavailable`; never fake success |
| missing approval | report `not_ready` |
| unknown tool | deny and audit |
| permission violation | deny and audit |
| unsafe proposal | report `blocked` |
| agent crash | preserve event history and continue in degraded mode when safe |

No failure path may silently fall back to vehicle command execution, route
execution, mission upload, simulator control, or hardware access.

## 11. Audit and Redaction Policy

All future agent and tool boundaries emit typed, JSON-safe audit records.
Sensitive fields are recursively redacted before report output. Unknown event
types are preserved as redacted, inspectable records rather than silently
dropped.

Audit records may contain only:

- task summary
- input references
- tool name and version
- validated parameters
- tool-result summary
- decision
- evidence references
- approval state
- timestamps
- errors

Audit records must not contain private chain-of-thought, credentials, tokens,
API keys, raw sockets, shell sessions, unrestricted code payloads, runtime
handles, or arbitrary Python objects.

## 12. v3-1 Allowed Implementation Scope

Future v3-1 may implement only:

- `src/c2/agent_types.py`
- `src/c2/agent_permissions.py`
- `src/c2/agent_tool_registry.py`
- `src/c2/agent_audit.py`
- `tests/test_c2_agent_types.py`
- `tests/test_c2_agent_permissions.py`
- `tests/test_c2_agent_tool_registry.py`

Allowed capabilities:

- pure-Python dataclasses
- typed agent messages
- permission manifests
- tool metadata
- mock tool-call validation
- audit records
- JSON serialization
- focused runtime-free tests

The first implementation must deny by default, remain provider-independent,
and operate without an LLM, optional runtime, network service, or simulator.

## 13. v3-1 Prohibited Implementation Scope

v3-1 must not add:

- LLM provider integration
- OpenAI API integration
- external model calls
- agent loops
- autonomous orchestration
- simulator launch or simulator connection
- ROS2 connection
- MAVSDK/PX4 connection
- hardware connection
- vehicle command
- route execution
- mission upload
- network broker
- database server
- credentials or tokens

It also must not add dashboard UI, web server, Nav2 plugin, automatic launch,
arming, takeoff, landing, payload behavior, offensive behavior, or runtime
validation.

## 14. v3-0 Completion Criteria

v3-0 is complete when:

- `docs/v3_agentic_ground_operations_center_plan.md` exists
- this scope and permission freeze exists
- the research title and architecture are frozen
- all nine agent roles have explicit permissions and denials
- tool side-effect and human-approval levels are frozen
- simulator/SITL boundaries are explicit and optional
- failure, trust, audit, and redaction policies are frozen
- v3-1 allowed and prohibited scopes are explicit
- `docs/roadmap.md` marks only v3-0 complete
- README links to the v3 plan without changing v1/v2 completion claims
- static documentation checks pass
- no source code, agent runtime, LLM/API integration, runtime action, simulator
  launch, ROS2 node, MAVSDK/PX4 connection, SITL action, or hardware check was
  added or run
- `v1.0.0-research-framework-complete` remains untouched
