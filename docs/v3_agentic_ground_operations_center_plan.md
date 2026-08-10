# GWM-UAV-C2 AgentOps v3 Extension Plan

## 1. Purpose

`v1.0.0-research-framework-complete` remains the completed archived research
framework. GWM-UAV-C2 v2 remains the completed mock-first and
readiness-oriented deterministic C2 extension.

GWM-UAV-C2 AgentOps is a new optional v3 extension layered above the completed
v2 deterministic C2 services. It does not reopen v1 or v2 as unfinished work.

The frozen goal is to build a human-supervised multi-agent ground operations
center in which specialized agents interpret operator intent, coordinate
mission dispatch, manage simulated fleet state, maintain situation awareness,
assess defensive risk, generate risk-aware route proposals, request gated
simulator validation, and produce replayable audit reports.

Agents may analyze, propose, coordinate, and request validation. Agents must
not directly command vehicles, bypass deterministic safety rules,
self-authorize safety-critical transitions, upload routes, arm, take off,
land, or access real hardware.

Existing deterministic v2 C2 modules remain the source of truth for validated
mission state, risk evaluation, airspace constraints, route scoring, replay,
metrics, and benchmark readiness.

## 2. Research Title

GWM-UAV-C2 AgentOps:
A Human-Supervised Multi-Agent Ground Operations Center for Mission Dispatch,
Defensive Risk Assessment, Risk-Aware Planning, and Simulator-Gated UAV Fleet
Coordination

## 3. Architecture

```text
Operator / Commander
        |
        v
Ground Operations Dashboard / C2 Console
        |
        v
Supervisor Agent
        |
        +--> Mission Agent
        +--> Fleet Agent
        +--> Situation Awareness Agent
        +--> Defensive Risk Agent
        +--> Planning and Airspace Agent
        +--> Simulation Validation Agent
        +--> Safety Review Agent
        +--> Audit and Replay Agent
        |
        v
Typed Agent Message Bus
        |
        v
Agent Tool Registry and Permission Layer
        |
        v
Existing Deterministic GWM-UAV-C2 v2 Tools
        |
        +--> MissionDispatcher
        +--> FleetManager
        +--> MissionEventBus
        +--> MissionStateStore
        +--> MissionReplayEngine
        +--> DefensiveRiskPredictor
        +--> UTMAirspaceLayer
        +--> RiskAwarePlanner
        +--> DashboardReplayBuilder
        +--> C2MetricsExporter
        +--> C2ReplayReportBuilder
        +--> C2BenchmarkReadinessBuilder
        |
        v
Safety Policy / CBF / Human Approval
        |
        v
Optional Explicitly Gated Simulation and SITL Paths
        |
        +--> Isaac Sim / Isaac Lab
        +--> Cosys-AirSim
        +--> legacy AirSim fallback
        +--> ROS2 simulation path
        +--> PX4 SITL
        +--> MAVSDK
        |
        v
Telemetry / Replay / Audit / Metrics
```

This architecture is a research ground operations center. It is not a
production battle-management system or flight-control system.

## 4. Agent Roles

### Supervisor Agent

Responsibilities:

- receive operator goals
- decompose high-level requests
- assign bounded tasks to specialist agents
- maintain workflow state
- resolve non-safety-critical coordination conflicts
- collect proposals
- request Safety Review Agent evaluation
- request human approval where required

Explicit non-goals:

- no direct `MissionStateStore` mutation
- no direct vehicle command
- no direct simulator command
- no self-approval
- no safety-rule bypass
- no unrestricted shell or network access

### Mission Agent

Allowed tools:

- `MissionDispatcher`
- `MissionEventBus`
- read-only `MissionStateStore` queries

Responsibilities:

- convert operator intent into `MissionRequest`
- create or update `MissionTask` proposals
- explain mission constraints and priority

Forbidden:

- no vehicle commands
- no fleet-assignment override
- no route execution
- no mission upload

### Fleet Agent

Allowed tools:

- `FleetManager`
- read-only `UAVState` and `FleetAsset` queries
- `MissionEventBus`

Responsibilities:

- inspect simulated asset availability
- propose deterministic asset assignments
- explain refusal reasons
- track mock fleet health and assignments

Forbidden:

- no arm
- no takeoff
- no landing
- no direct telemetry connection
- no real hardware access

### Situation Awareness Agent

Allowed tools:

- `MissionStateStore`
- `MissionReplayEngine`
- existing GWM and situation-memory read interfaces
- `DashboardReplayBuilder`

Responsibilities:

- summarize current mission state
- identify stale or missing information
- assemble a bounded situation context
- prepare replayable evidence for other agents

Forbidden:

- no direct state mutation
- no unvalidated world-state claim
- no runtime or hardware access

### Defensive Risk Agent

Allowed tools:

- `DefensiveRiskPredictor`
- read-only `MissionStateStore` access
- `MissionEventBus`

Responsibilities:

- analyze defensive risk signals
- generate `RiskSignal` proposals
- generate `ThreatAssessment` proposals
- recommend only `continue`, `hold`, `replan`, or `request_review`
- provide evidence and explanation

Allowed defensive categories:

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

Forbidden:

- no attack recommendation
- no target pursuit
- no target interception
- no disabling behavior
- no RF jamming action
- no GPS spoofing action
- no weaponized decision

Jamming and spoofing terms are allowed only as observed defensive risk
categories, never as actions.

### Planning and Airspace Agent

Allowed tools:

- `UTMAirspaceLayer`
- `RiskAwarePlanner`
- `ThreatAssessment` read access
- `MissionTask` and `FleetAsset` read access

Responsibilities:

- generate route candidates
- compare route scores
- explain risk and constraint penalties
- identify valid, warning, and blocked routes
- request human review for blocked or high-risk routes

Forbidden:

- no route execution
- no mission upload
- no Nav2 command
- no MAVSDK call
- no PX4 command
- no simulator control

### Simulation Validation Agent

Allowed future tools:

- `C2BenchmarkReadinessBuilder`
- existing simulator backend registry
- guarded Isaac validation adapters
- guarded Cosys-AirSim validation adapters
- guarded ROS2 / PX4 SITL / MAVSDK validation adapters

Default behavior:

- mock/readiness-only
- no runtime probe by default
- no simulator launch by default
- no simulator connection by default
- no command by default

Responsibilities:

- prepare a simulator validation request
- select a simulator profile
- compare expected schema support
- run explicitly gated validation only after operator approval
- return structured validation evidence

Forbidden:

- no automatic simulator launch
- no automatic PX4 launch
- no self-enabling environment gates
- no real hardware access
- no simulator parity claim

### Safety Review Agent

Allowed tools:

- `SafetyDecision`
- `HumanApprovalRecord`
- deterministic safety policies
- CBF and geofence result readers
- `ThreatAssessment` and `PlannedRoute` readers

Responsibilities:

- check whether proposals comply with frozen policy
- block prohibited actions
- request human approval
- produce concise, replayable safety rationale
- verify stale-data, geofence, route-verdict, and risk conditions

Critical rule: the Safety Review Agent may recommend or block. It must not
self-approve a safety-critical transition on behalf of a human.

Forbidden:

- no vehicle command
- no runtime-gate activation
- no approval fabrication
- no safety-certification claim

### Audit and Replay Agent

Allowed tools:

- `DashboardReplayBuilder`
- `C2MetricsExporter`
- `C2ReplayReportBuilder`
- `MissionReplayEngine`
- `C2BenchmarkReadinessBuilder`

Responsibilities:

- assemble event timelines
- generate metrics
- generate redacted JSON and Markdown reports
- preserve unknown events
- record agent/tool decisions
- produce post-mission audit summaries

Forbidden:

- read-only only
- no command controls
- no mission mutation
- no runtime connection
- no credential exposure

## 5. Existing v2 Tools Remain Deterministic Authorities

Agents do not replace the existing v2 modules. Agent output is an untrusted
proposal until it is validated by typed schemas and deterministic v2 tools.

- `MissionDispatcher` remains authoritative for mission-task construction.
- `FleetManager` remains authoritative for fleet-assignment state.
- `DefensiveRiskPredictor` remains authoritative for allowed risk categories
  and bounded risk outputs.
- `UTMAirspaceLayer` remains authoritative for mock airspace-constraint
  verdicts.
- `RiskAwarePlanner` remains authoritative for deterministic route scoring.
- `MissionStateStore` remains authoritative for validated mission state.
- `MissionReplayEngine` remains authoritative for deterministic replay.
- Dashboard and benchmark builders remain authoritative for redacted,
  deterministic audit output.

## 6. Ground Operations Center Interface Concept

The following layout is a frozen interface concept, not an implementation:

```text
+------------------------------------------------------------------+
| Mission Status | Agent Health | Simulator Mode | Approval Queue   |
+----------------------+-------------------------------------------+
| Fleet / Asset Panel  | 2D / 3D Mission Map                      |
| availability         | route candidates                          |
| battery / link       | geofence / no-fly-zone overlay            |
| current task         | defensive-risk overlay                    |
+----------------------+-------------------------------------------+
| Agent Activity       | Recommendations and Human Decisions        |
| assigned tasks       | Approve simulation validation              |
| tool-call summaries  | Reject / Hold / Replan / Request Review   |
| warnings / timeout   | Safety evidence                           |
+------------------------------------------------------------------+
| Event Timeline | Replay | Metrics | Audit | Benchmark Readiness   |
+------------------------------------------------------------------+
```

The dashboard is observational and approval-oriented. It is not a command
dashboard. It must not contain arm, takeoff, land, execute-route,
mission-upload, or payload controls.

## 7. Example Workflow

Operator request:

> Inspect Zone Alpha using two available simulated UAVs, avoid the northern
> restricted zone, prefer routes with low communication-degradation risk, and
> validate the selected mission in Isaac Sim before approval.

Workflow:

1. Supervisor Agent parses and decomposes the request.
2. Mission Agent creates `MissionRequest` and `MissionTask` proposals.
3. Fleet Agent proposes deterministic asset assignments.
4. Situation Awareness Agent assembles current state and data freshness.
5. Defensive Risk Agent creates risk assessments.
6. Planning and Airspace Agent generates and scores route candidates.
7. Safety Review Agent checks risk, geofence, and approval requirements.
8. Human operator approves or rejects simulator validation.
9. Simulation Validation Agent requests the explicitly gated Isaac path.
10. Audit and Replay Agent records all proposals, tool results, approvals,
    simulator results, and final decisions.

No physical vehicle action occurs.

## 8. Typed Agent Contracts

These future contracts are planned but are not implemented in v3-0. Every
contract must be JSON-safe and schema-validated at creation and consumption.

| Contract | Purpose | Required fields | Producer | Consumer | JSON-safe requirement | Validation requirement | Sensitive-field restrictions |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `AgentIdentity` | Identify an agent and its frozen role. | agent id, role, version, capability ids | registry | supervisor, audit | primitives and arrays only | known role and unique id | no credentials, tokens, runtime handles |
| `AgentCapability` | Describe one bounded permission. | capability id, action, resource scope, side-effect level | permission manifest | registry, safety review | enum-backed values | allowed role/action pair | no implicit wildcard permissions |
| `AgentTask` | Assign bounded work to an agent. | task id, workflow id, assignee, objective, input refs, deadline | supervisor | specialist agent, audit | references instead of object handles | known assignee, bounded objective, valid deadline | no secrets or unrestricted code payload |
| `AgentObservation` | Record validated information available to an agent. | observation id, source, timestamp, schema version, payload summary | tool wrapper, state facade | specialist agent, audit | validated JSON payload only | source schema and freshness checks | redact sensitive nested fields |
| `AgentContext` | Assemble bounded workflow context. | workflow id, state refs, observation refs, policy version, freshness | situation agent | supervisor, specialist agents | references and JSON summaries only | size, freshness, and policy-version checks | no raw sockets, sessions, credentials |
| `AgentProposal` | Represent an untrusted agent recommendation. | proposal id, agent id, task id, proposal type, payload, evidence refs | specialist agent | deterministic tool, safety review | schema-specific JSON payload | proposal schema and permission checks | no executable code or hidden state |
| `ToolCallRequest` | Request an allowlisted deterministic operation. | request id, tool id/version, caller, parameters, approval ref, timeout | authorized agent | tool registry | schema-validated parameters only | caller permission, schema, approval, gate checks | no shell command, credential, token, runtime handle |
| `ToolCallResult` | Capture a deterministic tool outcome. | request id, tool id/version, status, result summary, evidence refs, errors | tool wrapper | caller, audit, supervisor | redacted JSON result only | output schema and status validation | recursively redact sensitive fields |
| `AgentDecision` | Record an agent recommendation or block. | decision id, agent id, proposal refs, decision, rationale, evidence refs | specialist or safety agent | supervisor, human operator, audit | bounded text and references | allowed decision enum and evidence presence | no private reasoning trace |
| `AgentConflict` | Describe incompatible proposals or state. | conflict id, workflow id, proposal refs, conflict type, summary | supervisor, safety review | human operator, audit | references and bounded summary | two or more valid conflicting refs | no raw private context |
| `ApprovalRequest` | Ask a human to authorize a gated transition. | approval request id, level, target refs, rationale, evidence, expiry | supervisor or safety review | human operator | JSON-safe evidence refs | approval-level and expiry checks | no fabricated operator identity |
| `ApprovalDecision` | Record a human response to an approval request. | decision id, request id, operator id, outcome, timestamp, notes | human approval interface | safety policy, tool registry, audit | primitives and bounded notes | authoritative operator identity and request match | redact operator-sensitive metadata as policy requires |
| `AgentAuditRecord` | Preserve a replayable agent/tool event. | record id, workflow id, event type, actor, refs, summary, timestamp, errors | all wrappers and workflow components | audit and replay agent | deterministic redacted JSON | schema version, ordering, redaction checks | never store credentials or private chain-of-thought |
| `AgentWorkflowState` | Track workflow lifecycle without direct mission-state mutation. | workflow id, status, task refs, proposal refs, approval refs, timestamps | supervisor state machine | supervisor, safety review, audit | ids, enums, timestamps only | legal state transitions and referential integrity | no runtime handles or hidden mutable objects |

No contract may contain arbitrary Python objects, runtime handles,
credentials, tokens, raw sockets, shell sessions, or unrestricted code
payloads.

## 9. Tool Registry Model

A future allowlisted Agent Tool Registry mediates every agent/tool boundary.
Every tool definition must include:

- tool name
- tool version
- allowed agents
- input schema
- output schema
- read/write classification
- side-effect classification
- required approval level
- runtime-gate requirements
- timeout
- fallback behavior
- audit fields

Frozen side-effect levels:

- `READ_ONLY`
- `STATE_PROPOSAL`
- `VALIDATED_STATE_WRITE`
- `SIMULATOR_READ_ONLY`
- `SIMULATOR_COMMAND_GATED`
- `SITL_COMMAND_GATED`
- `REAL_HARDWARE_PROHIBITED`

No v3 agent may receive `REAL_HARDWARE_PROHIBITED` permission as an executable
capability. That level is a denial marker, not an action grant.

## 10. Human Approval and Execution Levels

### Level 0: Read-only analysis

Examples: state query, timeline query, metrics generation, and
benchmark-readiness report.

Human approval: not required.

### Level 1: Proposal generation

Examples: mission proposal, fleet-assignment proposal, risk assessment, and
route candidate.

Human approval: not required to compute; required before any safety-critical
transition.

### Level 2: Mock simulation evaluation

Examples: mock replay, mock benchmark, and mock route evaluation.

Human approval: not required unless policy says otherwise.

### Level 3: Gated external simulator validation

Examples: Isaac Sim validation, Cosys-AirSim validation, and ROS2 simulation
validation.

Requirements:

- explicit runtime gate
- explicit operator approval
- Safety Review Agent approval recommendation
- structured audit record
- no automatic simulator launch unless separately approved in a future scope

### Level 4: PX4 SITL / MAVSDK simulated command validation

Requirements:

- explicit runtime gates
- explicit operator approval
- validated simulator-only deployment flags
- deterministic safety policy
- CBF and geofence checks
- stale-command protection
- full audit record

### Level 5: Real hardware or autonomous real flight

Status: explicitly out of scope and prohibited in the current v3 design.

Agents must never self-escalate approval levels.

## 11. Failure and Degradation Rules

- agent timeout -> mark the agent task `timed_out` and return control to the
  Supervisor Agent
- invalid schema -> reject the proposal
- tool failure -> record the failure and use a no-op or hold recommendation
- stale state -> hold or `request_review`
- conflicting proposals -> escalate to the Safety Review Agent and human
  operator
- simulator unavailable -> `runtime_unavailable`; never fake success
- missing approval -> `not_ready`
- unknown tool -> deny
- permission violation -> deny and audit
- unsafe proposal -> `blocked`
- agent crash -> preserve event history and continue in degraded mode

No failure may silently fall back to vehicle command execution.

## 12. Agent Security and Trust Boundaries

Frozen requirements:

- least-privilege tool access
- allowlisted tools only
- typed schemas at every agent/tool boundary
- no shell access by default
- no network access by default
- no credential access
- no arbitrary file-system access
- no dynamic plugin installation
- no self-modifying permissions
- no self-enabling runtime gates
- no hidden state mutation
- all writes occur through validated events or approved tool wrappers
- prompt-injection-like content is treated as untrusted data
- tool output is treated as untrusted until schema validation
- sensitive data is recursively redacted from audit output

Private chain-of-thought must not be stored. Audit records contain only:

- task summary
- input references
- tool name
- validated parameters
- tool result summary
- decision
- evidence
- approval state
- timestamps
- errors

## 13. Simulator Integration Boundary

AgentOps includes simulator integration. Normal tests remain mock-first and
runtime-free. Isaac Sim / Isaac Lab and Cosys-AirSim remain optional gated
simulator paths.

The Simulation Validation Agent may request simulator validation but may not
silently launch or connect to a simulator. The operator starts or explicitly
enables external simulator paths according to the frozen runtime policy.

Isaac remains the guarded full-stack mainline. Cosys-AirSim remains the
preferred AirSim-family runtime. Legacy AirSim remains fallback only. No
simulator performance parity claim is made.

## 14. Proposed v3 Phases

### v3-0: Agentic C2 Scope and Permission Freeze

- Goal: freeze the AgentOps purpose, architecture, agent roster, permissions,
  approval levels, trust boundaries, and next-slice limits.
- Likely files: `docs/v3_agentic_ground_operations_center_plan.md`,
  `docs/v3_0_agent_scope_permissions_freeze.md`, `docs/roadmap.md`, and a
  minimal README link.
- Allowed scope: documentation and static consistency checks only.
- Explicit non-goals: no source, agent runtime, LLM SDK, tool execution,
  dashboard, simulator, ROS2, SITL, or hardware implementation.
- Verification commands: `git diff --check`; targeted `rg` safety-boundary
  audit; `git status`.
- Completion criteria: both v3 planning documents exist, frozen matrices are
  complete, roadmap marks only v3-0 complete, and no runtime action occurred.

### v3-1: Typed Agent Contracts and Tool Registry

- Goal: implement JSON-safe agent contracts, explicit permission manifests,
  an allowlisted mock tool registry, and deterministic audit records.
- Recommended split: v3-1A typed agent message dataclasses; v3-1B tool registry
  and permission manifests; v3-1C mock tool-call validation and audit records.
- Likely files: `src/c2/agent_types.py`, `src/c2/agent_permissions.py`,
  `src/c2/agent_tool_registry.py`, `src/c2/agent_audit.py`, and focused tests.
- Allowed scope: pure-Python dataclasses, validation, JSON serialization,
  permission checks, mock calls, and audit records.
- Explicit non-goals: no LLM/API provider, agent loop, external runtime,
  network broker, database server, simulator connection, or command behavior.
- Verification commands: `python -m pytest tests/test_c2_agent_types.py
  tests/test_c2_agent_permissions.py tests/test_c2_agent_tool_registry.py -q`;
  `python -m compileall -q src/c2 tests`; `git diff --check`.
- Completion criteria: contracts round-trip through JSON, permissions deny by
  default, only allowlisted mock tools execute, and audit output is redacted.

### v3-2: Supervisor Agent and Shared Situation Memory

- Goal: add a deterministic, provider-independent supervisor workflow and a
  read-oriented situation-memory facade.
- Recommended split: v3-2A supervisor workflow state; v3-2B shared situation
  memory facade; v3-2C agent timeout/conflict handling.
- Likely files: future `src/c2/agents/supervisor.py`,
  `src/c2/agent_workflow.py`, `src/c2/situation_memory.py`, and focused tests.
- Allowed scope: deterministic task routing, bounded context assembly,
  timeouts, conflict records, and degraded-mode state transitions.
- Explicit non-goals: no external model call, autonomous orchestration,
  direct `MissionStateStore` mutation, runtime connection, or self-approval.
- Verification commands: focused supervisor/situation-memory tests;
  `python -m compileall -q src/c2 tests`; `git diff --check`.
- Completion criteria: workflows are replayable, conflicts escalate, timeouts
  degrade safely, and no specialist gains extra permissions.

### v3-3: Mission, Fleet, Risk, and Planning Agents

- Goal: wrap completed v2 deterministic tools with proposal-only specialist
  agents.
- Recommended split: v3-3A Mission and Fleet agents; v3-3B Defensive Risk
  agent; v3-3C Planning and Airspace agent.
- Likely files: future `src/c2/agents/mission.py`, `fleet.py`, `risk.py`,
  `planning.py`, and focused tests.
- Allowed scope: validated proposal generation, deterministic v2 tool calls,
  explanations, refusals, and evidence references.
- Explicit non-goals: no vehicle command, route execution, mission upload,
  offensive recommendation, simulator control, or real telemetry.
- Verification commands: focused agent-to-v2 contract tests; existing C2
  regression tests; `git diff --check`.
- Completion criteria: every output validates as a proposal, v2 tools remain
  authoritative, and forbidden requests are denied and audited.

### v3-4: Simulation Validation Agent

- Goal: add a request-and-evidence workflow for mock and explicitly gated
  simulator validation.
- Recommended split: v3-4A mock simulation agent; v3-4B Isaac readiness
  integration; v3-4C Cosys-AirSim readiness integration.
- Likely files: future `src/c2/agents/simulation_validation.py`, adapter
  manifests, runtime-free tests, and optional gated tests.
- Allowed scope: readiness reports, simulator profile selection, approval
  checks, existing guarded adapter requests, and structured result summaries.
- Explicit non-goals: no automatic simulator or PX4 launch, no self-enabled
  gate, no real hardware, and no simulator parity claim.
- Verification commands: mock/readiness tests by default; existing optional
  runtime commands only under explicit operator-approved gates; `git diff
  --check`.
- Completion criteria: no-gate execution is a safe skip, unavailable runtimes
  are reported honestly, and gated requests require a valid human approval.

### v3-5: Safety Review and Human Approval Workflow

- Goal: enforce deterministic policy before any simulator/SITL request and
  preserve authoritative human decisions.
- Recommended split: v3-5A Safety Review Agent; v3-5B human approval queue;
  v3-5C gated simulator/SITL approval policy.
- Likely files: future `src/c2/agents/safety_review.py`,
  `src/c2/approval_queue.py`, `src/c2/approval_policy.py`, and focused tests.
- Allowed scope: recommend/block/review decisions, approval requests,
  deterministic policy checks, expiry, and audit records.
- Explicit non-goals: no fabricated approval, self-approval, direct command,
  runtime-gate activation, real hardware, or certified-safety claim.
- Verification commands: approval state-machine and denial tests; CBF/geofence
  result-reader tests; `git diff --check`.
- Completion criteria: only a human can author the authoritative approval,
  stale or absent approvals deny execution, and all decisions replay.

### v3-6: Ground Operations Dashboard

- Goal: expose read-only operations, agent activity, approvals, replay,
  metrics, audit, and benchmark readiness through deterministic payloads.
- Recommended split: v3-6A read-only operations dashboard payload; v3-6B
  agent activity and approval panels; v3-6C replay, metrics, and audit
  integration.
- Likely files: future dashboard-payload builders, static UI resources, and
  snapshot/accessibility tests after a separate implementation approval.
- Allowed scope: observation, filtering, approval submission, redacted report
  viewing, and replay controls.
- Explicit non-goals: no arm, takeoff, land, execute-route, mission-upload,
  payload, shell, simulator-launch, or hidden command controls.
- Verification commands: deterministic payload tests, UI snapshot/browser
  tests without runtimes, accessibility checks, and `git diff --check`.
- Completion criteria: the interface remains observational and
  approval-oriented, forbidden controls are absent, and sensitive data is
  redacted.

### v3-7: End-to-End Mock and Gated Simulator Demonstration

- Goal: demonstrate the complete human-supervised proposal, validation,
  approval, audit, and replay workflow.
- Recommended split: v3-7A end-to-end mock workflow; v3-7B gated external
  simulator workflow; v3-7C final AgentOps verification and completion
  summary.
- Likely files: future orchestration demo, runtime-free fixtures, gated
  simulator adapters, reports, docs, and focused integration tests.
- Allowed scope: mock-first workflow plus optional operator-approved simulator
  evidence through existing guarded adapters.
- Explicit non-goals: no physical vehicle action, autonomous real flight,
  offensive behavior, automatic launch, production-readiness claim,
  simulator-parity claim, or certified-safety claim.
- Verification commands: full runtime-free test suite, deterministic mock demo,
  static checks, and separately gated simulator validation when explicitly
  approved and available.
- Completion criteria: all v3 completion requirements below are evidenced,
  failures degrade safely, and optional runtime results are never inferred
  from mock evidence.

## 15. Definition of v3 Completion

v3 is complete only when:

- all agent permissions are explicit
- all agent messages are typed and JSON-safe
- all tool calls are allowlisted
- agents cannot directly command vehicles
- human-approval gates are enforced
- simulator paths are optional and explicitly gated
- mock-first tests remain runtime-free
- agent failures degrade safely
- audit and replay are deterministic
- no real hardware or autonomous real flight is enabled
- no offensive or weaponized behavior exists
- no production-readiness claim exists
- no simulator-parity claim exists
- no certified-safety claim exists
