# GWM-UAV-C2 AgentOps v3-1 Typed Agent Contracts and Tool Registry Plan

## 1. Purpose

`v1.0.0-research-framework-complete` remains the completed archived research
framework. GWM-UAV-C2 v2 remains the completed deterministic C2 foundation.

v3-0 froze the AgentOps architecture, agent roster, permission matrix,
approval levels, simulator/SITL gates, failure policy, and security boundary.
v3-1 will establish provider-independent typed contracts, deny-by-default
permissions, an allowlisted mock tool registry, and deterministic audit
records.

This document is planning-only. It does not implement an agent runtime, LLM
integration, external tool execution, simulator access, SITL access, or
vehicle command behavior.

## 2. Relationship to v3-0

The authoritative v3-0 documents are:

- `docs/v3_0_agent_scope_permissions_freeze.md`
- `docs/v3_agentic_ground_operations_center_plan.md`

v3-1 must preserve:

- the frozen nine-agent roster
- the frozen permission matrix
- the frozen tool side-effect levels
- the frozen approval Levels 0 through 5
- the frozen simulator and SITL gate boundary
- the frozen deny-by-default security model
- the frozen audit and redaction policy
- the prohibition on real hardware and autonomous real flight

If a v3-1 implementation choice conflicts with v3-0, v3-0 wins until a
separate scope-review document explicitly changes the freeze.

## 3. Allowed Future Implementation Files

The future v3-1 implementation is limited to:

```text
src/c2/agent_types.py
src/c2/agent_permissions.py
src/c2/agent_tool_registry.py
src/c2/agent_audit.py

tests/test_c2_agent_types.py
tests/test_c2_agent_permissions.py
tests/test_c2_agent_tool_registry.py
```

`src/c2/__init__.py` and the v3 planning documents may receive small export or
status updates in the appropriate implementation slice. No additional file
may be added without documenting why the frozen file set is insufficient.

## 4. Explicit Non-goals

v3-1 must not implement:

- LLM provider integration
- OpenAI API integration
- external model calls
- agent reasoning loops
- autonomous orchestration
- MCP client or server
- shell command execution
- network broker or network access
- database server
- arbitrary file-system access
- dynamic plugin installation
- self-modifying permissions
- simulator launch or simulator connection
- ROS2 connection
- MAVSDK/PX4 connection
- Nav2 connection
- SITL command execution
- hardware connection
- vehicle command
- route execution
- mission upload
- arming
- takeoff
- landing
- payload behavior
- offensive targeting
- attack execution
- weapon control
- autonomous attack-decision logic
- real-world pursuit/intercept behavior
- credentials or token storage
- private chain-of-thought storage

## 5. Frozen Agent-role Identifiers

The only valid role identifiers are:

```text
supervisor
mission
fleet
situation_awareness
defensive_risk
planning_airspace
simulation_validation
safety_review
audit_replay
```

Requirements:

- no dynamically generated role receives permissions
- unknown roles are rejected
- role identifiers are stable and versioned
- role matching is exact and case-sensitive
- no wildcard role exists
- no permission inheritance exists between roles
- a role-version change requires an explicit manifest-version change

The future implementation should expose a fixed enum or equivalent
standard-library-backed validator. Free-form role strings may be accepted at a
deserialization boundary only long enough to validate and reject or normalize
them into the fixed identifier set.

## 6. Typed Contract Plan

All contracts use only standard-library-compatible values: `str`, `int`,
`float`, `bool`, `list`, `dict`, `Optional`, and enum or Literal-like validated
strings. Datetime-like values cross contract boundaries as normalized strings
or finite numeric timestamps, not provider-specific objects.

### Contract field and validation matrix

| Contract | Purpose | Suggested fields | Required fields | Optional fields | Producer | Consumer | Contract-specific validation |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `AgentIdentity` | Identify one enabled agent and its frozen role. | agent_id, role, version, capability_ids, enabled, metadata | agent_id, role, version, capability_ids, enabled | metadata | static identity builder or test fixture | permission layer, registry, audit | non-empty id/version; frozen role; unique capability ids; boolean enabled |
| `AgentCapability` | Describe one bounded action/resource permission. | capability_id, action, resource_scope, side_effect_level, max_approval_level, metadata | capability_id, action, resource_scope, side_effect_level, max_approval_level | metadata | permission manifest builder | permission evaluator, audit | known side effect; approval 0-4; no wildcard; hardware denial is non-executable |
| `AgentTask` | Assign bounded work within a workflow. | task_id, workflow_id, assignee_agent_id, objective, input_refs, deadline, status, metadata | task_id, workflow_id, assignee_agent_id, objective, input_refs, status | deadline, metadata | Supervisor Agent workflow | specialist agent, audit | non-empty ids/objective; unique refs; known status; finite deadline when present |
| `AgentObservation` | Carry a validated, bounded observation summary. | observation_id, source, timestamp, schema_version, payload_summary, freshness_status, metadata | observation_id, source, timestamp, schema_version, payload_summary, freshness_status | metadata | read facade or validated tool result | situation/specialist agents, audit | known freshness status; finite timestamp; bounded JSON payload summary |
| `AgentContext` | Reference the validated state available to one workflow. | context_id, workflow_id, state_refs, observation_refs, policy_version, freshness, metadata | context_id, workflow_id, state_refs, observation_refs, policy_version, freshness | metadata | Situation Awareness Agent | Supervisor and specialist agents | unique refs; known policy version; freshness bounds; context-size limit |
| `AgentProposal` | Represent an untrusted specialist recommendation. | proposal_id, agent_id, task_id, proposal_type, payload, evidence_refs, created_at, metadata | proposal_id, agent_id, task_id, proposal_type, payload, evidence_refs, created_at | metadata | specialist agent | deterministic v2 tool, Safety Review Agent | known proposal type; evidence refs; finite timestamp; schema-specific payload validation |
| `ToolCallRequest` | Request one exact allowlisted deterministic tool/version. | request_id, tool_name, tool_version, caller_agent_id, parameters, approval_ref, timeout_sec, runtime_gate_refs, metadata | request_id, tool_name, tool_version, caller_agent_id, parameters, timeout_sec | approval_ref, runtime_gate_refs, metadata | authorized agent | tool registry, audit | caller/tool/version/schema/permission/side-effect/approval/gate/timeout checks |
| `ToolCallResult` | Capture a validated mock tool outcome. | request_id, tool_name, tool_version, status, result_summary, evidence_refs, errors, started_at, completed_at, metadata | request_id, tool_name, tool_version, status, result_summary, errors, started_at, completed_at | evidence_refs, metadata | mock tool wrapper | caller, Supervisor, audit | known status; matching request/tool/version; ordered finite timestamps; validated output schema |
| `AgentDecision` | Record a bounded recommendation, denial, or block. | decision_id, agent_id, proposal_refs, decision, rationale, evidence_refs, metadata | decision_id, agent_id, decision, rationale | proposal_refs, evidence_refs, metadata | specialist or Safety Review Agent | Supervisor, human operator, audit | known decision; permitted role; evidence for safety-affecting decisions |
| `AgentConflict` | Describe incompatible proposals or state. | conflict_id, workflow_id, proposal_refs, conflict_type, summary, resolution_status, metadata | conflict_id, workflow_id, proposal_refs, conflict_type, summary, resolution_status | metadata | Supervisor or Safety Review Agent | human operator, audit | at least two unique refs; known type/status; bounded summary |
| `ApprovalRequest` | Ask a human to authorize a gated transition. | approval_request_id, approval_level, target_refs, rationale, evidence_refs, requested_by, expires_at, metadata | approval_request_id, approval_level, target_refs, rationale, requested_by, expires_at | evidence_refs, metadata | Supervisor or Safety Review Agent | authoritative human interface, audit | level 1-4; non-empty targets; valid requester; future finite expiry |
| `ApprovalDecision` | Record an authoritative human response. | decision_id, approval_request_id, operator_id, outcome, timestamp, notes, metadata | decision_id, approval_request_id, operator_id, outcome, timestamp | notes, metadata | authoritative human interface only | safety policy, registry, audit | valid request; known outcome; verified human source; finite timestamp |
| `AgentAuditRecord` | Preserve one redacted, replayable agent/tool event. | record_id, workflow_id, event_type, actor_id, input_refs, tool_name, tool_version, validated_parameters_summary, result_summary, decision, approval_state, timestamp, errors, metadata | record_id, workflow_id, event_type, actor_id, timestamp | all summary/ref/tool/decision/error fields not relevant to the event | contract/tool/workflow boundary | Audit and Replay Agent | deterministic id/order; known event class or preserved safe unknown; recursive redaction |
| `AgentWorkflowState` | Track provider-independent workflow lifecycle. | workflow_id, status, task_refs, proposal_refs, conflict_refs, approval_refs, created_at, updated_at, metadata | workflow_id, status, created_at, updated_at | reference lists, metadata | Supervisor state machine | Supervisor, Safety Review, audit | legal transition; unique refs; updated_at not before created_at |

### Serialization, redaction, and mock fixture matrix

| Contract | `to_dict` behavior | `from_dict` behavior | Sensitive-field restrictions | Example mock fixture |
| --- | --- | --- | --- | --- |
| `AgentIdentity` | emit role and capabilities as stable strings/lists | reject extra executable objects and unknown role | no credentials, tokens, runtime handles | `agent-supervisor-01`, role `supervisor`, version `1`, enabled `true` |
| `AgentCapability` | emit enum values as strings | reject wildcard action/scope and unknown side effect | no embedded handler or callable | capability `mission.query`, scope `mission`, `READ_ONLY`, max level 0 |
| `AgentTask` | emit refs and deadline as JSON-safe values | reject unknown status, invalid refs, non-finite deadline | objective and metadata pass sensitive-key scan | task `task-001`, workflow `wf-001`, assignee `agent-mission-01`, status `pending` |
| `AgentObservation` | emit summary, not raw runtime data | reject provider SDK values and invalid timestamp | no raw image handle, socket, session, or SDK object | observation `obs-001`, source `mission_state`, freshness `fresh` |
| `AgentContext` | emit references and bounded summaries only | reject oversized/unvalidated context payload | no raw files, model sessions, or private reasoning | context `ctx-001` with state ref `state-010` and policy `v3-0` |
| `AgentProposal` | emit payload after proposal-schema validation | reject unknown type, sensitive keys, arbitrary objects | no executable code, credentials, or hidden mutable state | proposal `proposal-001`, type `mission_task`, evidence `event-001` |
| `ToolCallRequest` | emit exact tool/version and validated parameters | deny before construction if caller/tool/parameter shape is invalid | no shell command, token, API key, code payload, handle | request `req-001` for `mission.query` version `1` with mission id |
| `ToolCallResult` | emit redacted result summary and deterministic errors | reject request mismatch, unknown status, invalid output | no raw exception object, handle, secret, or private reasoning | request `req-001`, status `passed`, result `mission found` |
| `AgentDecision` | emit bounded rationale and evidence refs | reject unknown decision or unauthorized decision role | rationale is a concise explanation, not chain-of-thought | decision `dec-001`, `request_review`, evidence `risk-002` |
| `AgentConflict` | emit proposal refs and bounded conflict summary | reject fewer than two refs or unknown resolution | no hidden agent context or raw provider trace | conflict `conflict-001`, type `route_verdict`, status `open` |
| `ApprovalRequest` | emit level, targets, evidence, requester, expiry | reject Level 5, invalid expiry, or fabricated requester | no credential or operator secret | request `approval-req-001`, Level 3, target `validation-001` |
| `ApprovalDecision` | emit verified operator id, outcome, time, notes | reject non-human producer, unknown outcome, missing request | redact operator-sensitive metadata as policy requires | decision `approval-dec-001`, outcome `approved`, operator `operator-01` |
| `AgentAuditRecord` | emit recursively redacted deterministic event record | preserve unknown safe fields; reject unsafe values | never store chain-of-thought, secret, token, socket, session, shell, or handle | record `audit-001`, event `tool_denied`, code `permission_denied` |
| `AgentWorkflowState` | emit ids, enums, refs, and timestamps | reject illegal transition or invalid references | no runtime objects or hidden mutable state | workflow `wf-001`, status `awaiting_review`, task `task-001` |

Common `to_dict` rules:

- return a newly allocated JSON-safe dictionary
- serialize enums as stable string values
- copy nested lists/dictionaries before returning
- reject non-finite floats and non-string dictionary keys
- recursively scan for prohibited values and sensitive keys
- include an explicit schema/version field where the contract needs migration

Common `from_dict` rules:

- require a dictionary input
- validate required fields before object construction
- reject unknown enum values and malformed nested values
- reject callable/provider/runtime objects even if nested
- copy accepted input so later caller mutation cannot alter the contract
- produce deterministic validation errors rather than raw exceptions

All contracts reject arbitrary Python objects, runtime handles, open files,
sockets, sessions, credentials, tokens, API keys, shell commands,
unrestricted code payloads, callable objects, provider SDK objects, simulator
SDK objects, ROS2 handles, and MAVSDK/PX4 handles.

## 7. Contract-specific Direction

### `AgentIdentity`

Fields: `agent_id`, `role`, `version`, `capability_ids`, `enabled`, and
`metadata`.

Validation requires a non-empty `agent_id`, a role in the frozen role set, a
non-empty version, unique capability ids, a boolean enabled flag, and JSON-safe
metadata.

### `AgentCapability`

Fields: `capability_id`, `action`, `resource_scope`, `side_effect_level`,
`max_approval_level`, and `metadata`.

Validation requires a known side-effect level, approval Level 0 through 4, no
wildcard action, no wildcard resource scope, and no executable use of
`REAL_HARDWARE_PROHIBITED`.

### `AgentTask`

Fields: `task_id`, `workflow_id`, `assignee_agent_id`, `objective`,
`input_refs`, `deadline`, `status`, and `metadata`.

Allowed statuses:

- `pending`
- `assigned`
- `running`
- `completed`
- `failed`
- `timed_out`
- `blocked`
- `cancelled`

### `AgentObservation`

Fields: `observation_id`, `source`, `timestamp`, `schema_version`,
`payload_summary`, `freshness_status`, and `metadata`. The payload is a bounded
summary, never a runtime handle or unvalidated raw provider object.

### `AgentContext`

Fields: `context_id`, `workflow_id`, `state_refs`, `observation_refs`,
`policy_version`, `freshness`, and `metadata`. Context is reference-oriented,
size-bounded, and validated for freshness before use.

### `AgentProposal`

Fields: `proposal_id`, `agent_id`, `task_id`, `proposal_type`, `payload`,
`evidence_refs`, `created_at`, and `metadata`.

`AgentProposal` is untrusted until deterministic validation succeeds. Proposal
construction does not mutate validated mission state.

### `ToolCallRequest`

Fields: `request_id`, `tool_name`, `tool_version`, `caller_agent_id`,
`parameters`, `approval_ref`, `timeout_sec`, `runtime_gate_refs`, and
`metadata`.

Validation checks the known caller, allowlisted tool, exact version, parameter
schema, permission manifest, side-effect level, approval level, runtime gates,
timeout bounds, and sensitive-field rules.

### `ToolCallResult`

Fields: `request_id`, `tool_name`, `tool_version`, `status`, `result_summary`,
`evidence_refs`, `errors`, `started_at`, `completed_at`, and `metadata`.

Allowed statuses:

- `passed`
- `failed`
- `denied`
- `blocked`
- `timed_out`
- `not_ready`
- `runtime_unavailable`
- `skipped`

### `AgentDecision`

Allowed decisions:

- `continue`
- `hold`
- `replan`
- `request_review`
- `approve_recommendation`
- `block`
- `deny`

`approve_recommendation` is an agent recommendation. It is not an
authoritative human approval.

### `AgentConflict`

Fields: `conflict_id`, `workflow_id`, `proposal_refs`, `conflict_type`,
`summary`, `resolution_status`, and `metadata`. A conflict references at least
two distinct proposals and is escalated rather than resolved by permission
expansion.

### `ApprovalRequest`

Fields: `approval_request_id`, `approval_level`, `target_refs`, `rationale`,
`evidence_refs`, `requested_by`, `expires_at`, and `metadata`. Level 5 requests
are rejected rather than queued.

### `ApprovalDecision`

Fields: `decision_id`, `approval_request_id`, `operator_id`, `outcome`,
`timestamp`, `notes`, and `metadata`.

Allowed outcomes:

- `approved`
- `rejected`
- `deferred`
- `expired`
- `revoked`

Only an authoritative human interface may create `ApprovalDecision`.

### `AgentAuditRecord`

Fields: `record_id`, `workflow_id`, `event_type`, `actor_id`, `input_refs`,
`tool_name`, `tool_version`, `validated_parameters_summary`, `result_summary`,
`decision`, `approval_state`, `timestamp`, `errors`, and `metadata`.

Private chain-of-thought must never be stored.

### `AgentWorkflowState`

Fields: `workflow_id`, `status`, `task_refs`, `proposal_refs`, `conflict_refs`,
`approval_refs`, `created_at`, `updated_at`, and `metadata`.

Allowed statuses:

- `created`
- `planning`
- `awaiting_review`
- `awaiting_approval`
- `validating`
- `completed`
- `failed`
- `timed_out`
- `blocked`
- `cancelled`

## 8. Side-effect Level Plan

The side-effect levels remain exactly:

- `READ_ONLY`
- `STATE_PROPOSAL`
- `VALIDATED_STATE_WRITE`
- `SIMULATOR_READ_ONLY`
- `SIMULATOR_COMMAND_GATED`
- `SITL_COMMAND_GATED`
- `REAL_HARDWARE_PROHIBITED`

`REAL_HARDWARE_PROHIBITED` is a denial marker, never an executable permission.

| Level | Meaning | Permitted behavior | Required approval | Runtime gate | Default timeout | Fallback | Audit requirement |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `READ_ONLY` | Query validated deterministic state. | bounded query or report generation | Level 0; none | none | 5 seconds | empty/redacted result plus error | start/result or denial record |
| `STATE_PROPOSAL` | Produce untrusted proposed state. | construct typed proposal without authoritative mutation | Level 1; none to compute | none | 5 seconds | no-op and proposal failure | proposal source, validation, result |
| `VALIDATED_STATE_WRITE` | Submit a validated event through an authoritative v2 tool. | event-mediated write within exact scope | policy-defined; never agent self-approved | none unless target policy declares one | 5 seconds | reject write and preserve prior state | before/after refs, validator, result |
| `SIMULATOR_READ_ONLY` | Read approved readiness or simulator evidence. | readiness/report query only | Level 0 for stored report; Level 3 for external runtime read | explicit gates for external access | 30 seconds | `runtime_unavailable` or `not_ready` | gates, approval, runtime-invoked flag, cleanup |
| `SIMULATOR_COMMAND_GATED` | Request a bounded simulator-only command validation. | approved request through existing guarded adapter | Level 3 | explicit simulator gates | 30 seconds | deny or safe no-op; no launch | full request, approval, gates, safety, result, cleanup |
| `SITL_COMMAND_GATED` | Request bounded PX4 SITL/MAVSDK validation. | approved simulator-only command request | Level 4 | explicit optional-runtime and SITL gates | 30 seconds | deny/hold and audited cleanup | deployment flags, CBF/geofence/staleness checks, command summary |
| `REAL_HARDWARE_PROHIBITED` | Mark an action as impossible in current v3. | none | Level 5 is always denied | no gate can enable it | 0 seconds | immediate denial | mandatory prohibition audit record |

## 9. Approval-level Plan

The approval levels remain:

| Level | Meaning | Frozen rule |
| --- | --- | --- |
| 0 | read-only analysis | no approval required |
| 1 | proposal generation | no approval to compute; approval required before a safety-critical transition |
| 2 | mock simulation evaluation | no approval unless policy explicitly requires one |
| 3 | gated external simulator validation | explicit operator approval, runtime gates, safety recommendation, and audit required |
| 4 | gated PX4 SITL / MAVSDK simulated command validation | explicit operator approval, gates, simulator-only state, deterministic safety checks, and audit required |
| 5 | real hardware or autonomous real flight | prohibited and always denied |

Agents cannot self-escalate, fabricate approval, author an authoritative human
decision, or convert a recommendation into approval. Level 3 requires explicit
operator approval and gates. Level 4 additionally requires validated
simulator-only deployment state, deterministic safety/CBF/geofence/staleness
checks, and full audit. Level 5 is denied before invocation under every
configuration.

## 10. Permission Manifest Plan

The future `AgentPermissionManifest` should contain:

- `manifest_id`
- `agent_role`
- `manifest_version`
- `allowed_capability_ids`
- `allowed_tool_ids`
- `read_scopes`
- `proposal_scopes`
- `validated_write_scopes`
- `allowed_side_effect_levels`
- `max_requestable_approval_level`
- `explicit_denials`
- `metadata`

Required behavior:

- deny by default
- exact role matching
- exact tool matching
- exact side-effect matching
- no wildcard permissions
- no implicit inheritance
- no self-modification
- unknown capability denied
- unknown scope denied
- missing manifest denied
- disabled agent denied
- approval Level 5 always denied

Manifests are immutable contract values after validation. Updating a manifest
requires constructing and validating a new version; an agent cannot mutate its
own manifest or capability list.

## 11. Tool Definition Plan

The future `AgentToolDefinition` should contain:

- `tool_name`
- `tool_version`
- `description`
- `allowed_agent_roles`
- `input_schema_id`
- `output_schema_id`
- `access_class`
- `side_effect_level`
- `required_approval_level`
- `runtime_gate_names`
- `timeout_sec`
- `fallback_behavior`
- `audit_field_names`
- `enabled`
- `metadata`

Required validation:

- tool name and version are non-empty
- allowed agent roles are known frozen roles
- input and output schemas are known
- side-effect level is known
- required approval level is valid
- timeout is positive and bounded for executable mock tools
- runtime gates are explicitly declared, including an explicit empty list
- fallback behavior is declared
- audit fields are declared
- enabled flag is explicit
- `REAL_HARDWARE_PROHIBITED` cannot be invoked

Tool definitions contain metadata only. They do not contain handlers,
callables, provider objects, runtime clients, sockets, sessions, or command
payloads.

## 12. Tool Registry Design Plan

The future class is `AgentToolRegistry` with this recommended API:

```text
register(tool_definition)
get(tool_name, tool_version)
list_tools(agent_role=None)
authorize(request, identity, permission_manifest, approval=None, gates=None)
validate_request(request)
validate_result(result)
invoke_mock(request, mock_handler=None)
clear()
```

Required behavior:

- deny unknown tools
- deny disabled tools
- deny version mismatch
- deny an unlisted caller
- deny missing permission
- deny excessive side-effect level
- deny missing or invalid approval
- deny missing runtime gates
- deny approval Level 5
- validate input before invocation
- validate output after invocation
- emit an audit result for pass or denial
- never call a real runtime in v3-1

`register` rejects duplicate `(tool_name, tool_version)` pairs. `get` requires
an exact name and version. `list_tools` returns metadata copies and filters by
an exact role when supplied. `clear` is a test-only registry lifecycle helper;
it does not erase the append-only audit log.

`invoke_mock` accepts only an explicitly injected test/mock handler. The
registry does not discover modules, import providers, open a network
connection, create a shell, inspect arbitrary files, or fall back to a real
adapter.

## 13. Initial Mock Tool Catalogue Plan

The initial catalogue contains deterministic metadata only. Actual wrappers
are not implemented in this planning slice.

| Mock tool | Allowed roles | Access class | Side-effect class | Approval | Input schema | Output schema | Timeout | Fallback | Required audit fields |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `mission.query` | supervisor, mission, situation_awareness, safety_review, audit_replay | read mission scope | `READ_ONLY` | Level 0 | mission query ref | redacted mission snapshot | 5s | empty result plus error | caller, mission ref, status, error |
| `mission.propose` | mission | mission proposal | `STATE_PROPOSAL` | Level 1 to compute | operator-intent summary and constraints | validated mission proposal | 5s | reject proposal | caller, task, schema result, evidence |
| `fleet.query` | supervisor, fleet, situation_awareness, planning_airspace, safety_review, audit_replay | read fleet scope | `READ_ONLY` | Level 0 | fleet/asset query ref | redacted fleet snapshot | 5s | empty result plus error | caller, scope, freshness, status |
| `fleet.propose_assignment` | fleet | assignment proposal | `STATE_PROPOSAL` | Level 1 to compute | task and eligible asset refs | validated assignment proposal | 5s | hold/unassigned proposal | caller, refs, refusal/result, evidence |
| `situation.query` | supervisor, situation_awareness, defensive_risk, planning_airspace, safety_review, audit_replay | read context scope | `READ_ONLY` | Level 0 | workflow/state refs | bounded situation context | 5s | stale/empty context plus error | caller, refs, freshness, status |
| `risk.evaluate` | defensive_risk, safety_review | defensive-risk proposal | `STATE_PROPOSAL` | Level 1 to compute | allowed risk signal refs | validated bounded assessment | 5s | `request_review` | caller, categories, evidence, recommendation |
| `airspace.validate` | planning_airspace, safety_review | read/constraint verdict | `READ_ONLY` | Level 0 | route and constraint refs | deterministic airspace verdict | 5s | blocked/unknown verdict | caller, route, constraints, verdict |
| `route.score` | planning_airspace, safety_review | route proposal | `STATE_PROPOSAL` | Level 1 to compute | candidate/risk/airspace refs | ranked route score breakdown | 5s | no selected route | caller, candidates, weights, verdict, score |
| `replay.build` | situation_awareness, audit_replay | read/report | `READ_ONLY` | Level 0 | mission/workflow refs | deterministic replay payload | 10s | partial redacted replay | caller, range, event count, unknown events, errors |
| `metrics.build` | audit_replay | read/report | `READ_ONLY` | Level 0 | replay/mission refs | deterministic metric summary | 10s | empty metrics plus error | caller, refs, metric schema, status |
| `report.build` | audit_replay | read/report | `READ_ONLY` | Level 0 | replay/metrics/audit refs | redacted JSON/Markdown summary | 10s | redacted failure report | caller, refs, redaction summary, status |
| `benchmark.readiness` | supervisor, simulation_validation, safety_review, audit_replay | simulator readiness | `SIMULATOR_READ_ONLY` | Level 0 for stored reports | backend/profile refs | deterministic readiness summary | 10s | `not_ready` | caller, backend, report refs, invoked-runtime false |
| `approval.request` | supervisor, safety_review | approval proposal | `STATE_PROPOSAL` | no approval to request Levels 1-4 | target/evidence/level/expiry | validated approval request | 5s | `not_ready` | caller, level, targets, evidence, expiry |

The catalogue must not contain:

- `vehicle.command`
- `vehicle.arm`
- `vehicle.takeoff`
- `vehicle.land`
- `route.execute`
- `mission.upload`
- `simulator.launch`
- `px4.launch`
- `hardware.connect`
- `shell.execute`
- `network.request`

## 14. Authorization Flow

The future authorization order is frozen as:

```text
ToolCallRequest received
-> validate JSON-safe request
-> validate AgentIdentity
-> load permission manifest
-> resolve exact tool and version
-> check caller role allowlist
-> check capability and resource scope
-> check side-effect level
-> check approval level
-> check approval record when required
-> check runtime gates when required
-> check timeout
-> redact and create audit start record
-> execute mock handler only
-> validate ToolCallResult
-> redact and create audit result record
```

Any failed check denies the request before invocation. Denial still produces a
redacted audit record. Authorization checks are pure and deterministic: the
same validated inputs and registry state produce the same decision and error
code.

## 15. Deterministic Error Model

Planned error codes:

- `invalid_agent_identity`
- `unknown_agent_role`
- `duplicate_agent_id`
- `unknown_capability`
- `invalid_contract`
- `invalid_json_payload`
- `sensitive_field_rejected`
- `missing_permission_manifest`
- `permission_denied`
- `unknown_tool`
- `tool_disabled`
- `tool_version_mismatch`
- `input_schema_mismatch`
- `output_schema_mismatch`
- `side_effect_not_allowed`
- `approval_required`
- `approval_invalid`
- `approval_expired`
- `runtime_gate_required`
- `runtime_gate_missing`
- `timeout_invalid`
- `tool_call_denied`
- `tool_call_failed`
- `tool_call_timed_out`
- `real_hardware_prohibited`

Each error contains:

- `code`
- `message`
- request reference
- actor reference
- tool reference when relevant
- audit reference

Messages are concise, deterministic, and safe to display. Errors never include
secrets, credentials, raw exceptions, private reasoning, provider payloads, or
runtime handles.

## 16. Audit and Redaction Plan

The future `AgentAuditLog` should expose:

```text
append(record)
list_records(workflow_id=None)
find_by_request(request_id)
snapshot()
restore(snapshot)
clear()
```

Required behavior:

- append-only semantics for normal operation
- deterministic ordering
- JSON-safe snapshot and validated restore
- no file writes by default
- no network
- no database
- recursive redaction
- preservation of unknown safe fields
- no private chain-of-thought storage

`clear` is test-only lifecycle behavior. A production-facing workflow cannot
use it to remove history. `restore` validates and copies every record before
replacing an in-memory test instance.

The sensitive denylist includes:

```text
password
passwd
secret
token
api_key
apikey
credential
credentials
private_key
hostname
host
runtime_handle
socket
session
shell
command
code_payload
file_handle
```

Matching is case-insensitive after normalized key conversion and recursive
through dictionaries and lists. A rejected sensitive input produces
`sensitive_field_rejected`; audit-output redaction replaces accepted safe
summaries with a deterministic marker such as `[REDACTED]`.

Audit records contain task summaries, input references, tool identity,
validated parameter summaries, result summaries, decisions, evidence,
approval state, timestamps, and errors only.

## 17. Future Test Plan

### v3-1A contract tests

- `test_agent_identity_json_roundtrip`
- `test_agent_identity_rejects_unknown_role`
- `test_agent_capability_validates_side_effect_level`
- `test_agent_task_status_validation`
- `test_agent_proposal_is_json_safe`
- `test_tool_call_request_rejects_sensitive_fields`
- `test_tool_call_result_status_validation`
- `test_approval_decision_requires_human_operator`
- `test_agent_audit_record_excludes_private_reasoning`
- `test_agent_contracts_import_without_runtime_dependencies`

### v3-1B permission and registry tests

- `test_permission_manifest_denies_by_default`
- `test_permission_manifest_rejects_wildcard`
- `test_permission_manifest_denies_unknown_tool`
- `test_permission_manifest_enforces_max_approval_level`
- `test_permission_manifest_always_denies_level_five`
- `test_tool_definition_validation`
- `test_tool_registry_registers_allowlisted_tool`
- `test_tool_registry_rejects_duplicate_version`
- `test_tool_registry_filters_by_agent_role`
- `test_tool_registry_denies_disabled_tool`

### v3-1C mock invocation and audit tests

- `test_mock_tool_call_validates_input`
- `test_mock_tool_call_denies_unlisted_agent`
- `test_mock_tool_call_denies_side_effect_escalation`
- `test_mock_tool_call_requires_approval`
- `test_mock_tool_call_requires_runtime_gate_metadata`
- `test_mock_tool_call_never_invokes_real_runtime`
- `test_mock_tool_call_validates_output`
- `test_mock_tool_call_writes_redacted_audit_record`
- `test_mock_tool_failure_is_audited`
- `test_mock_tool_timeout_is_deterministic`

Tests must not require an LLM, OpenAI API, network, database, GPU, Isaac Sim,
Cosys-AirSim, legacy AirSim, ROS2, MAVSDK, PX4, Nav2, SITL, hardware, shell, or
browser.

## 18. Verification Commands for Future Implementation

```bash
python -m pytest \
  tests/test_c2_agent_types.py \
  tests/test_c2_agent_permissions.py \
  tests/test_c2_agent_tool_registry.py \
  -q

python -m compileall -q src/c2 tests

git diff --check

rg -n \
  "OpenAI|LLM provider|external model|vehicle command|route execution|mission upload|arming|takeoff|landing|payload|automatic simulator launch|real hardware|autonomous real flight|offensive|weapon" \
  src/c2 tests docs || true
```

Expected matches are acceptable only in explicit rejection, safety,
permission, or non-goal statements.

## 19. Implementation Split

### v3-1A: Typed Agent Message Dataclasses

Allowed scope:

- `src/c2/agent_types.py`
- `tests/test_c2_agent_types.py`
- small exports in `src/c2/__init__.py`
- documentation status updates

This slice implements only standard-library-compatible contracts, fixed
enums/validated strings, JSON conversion, deterministic validation, and
sensitive-value rejection. It does not implement permissions, registry
invocation, agents, providers, or runtimes.

### v3-1B: Tool Registry and Permission Manifests

Allowed scope:

- `src/c2/agent_permissions.py`
- `src/c2/agent_tool_registry.py`
- `tests/test_c2_agent_permissions.py`
- `tests/test_c2_agent_tool_registry.py`
- small exports in `src/c2/__init__.py`
- documentation status updates

This slice implements deny-by-default manifests, metadata-only tool
definitions, exact-match authorization, and registry lifecycle. It does not
invoke external or real tools.

### v3-1C: Mock Tool-call Validation and Audit Records

Allowed scope:

- `src/c2/agent_audit.py`
- small compatibility updates to `src/c2/agent_tool_registry.py`
- focused audit and mock-invocation tests in the frozen test files
- documentation status updates

This slice implements redacted in-memory audit records and explicitly injected
mock-handler invocation only. It does not add provider, network, simulator,
SITL, vehicle, or hardware access.

## 20. v3-1 Completion Criteria

v3-1 implementation will be complete later only when:

- all frozen contracts exist
- contracts validate and round-trip through JSON
- agent roles are fixed and validated
- permissions deny by default
- wildcards and implicit inheritance are rejected
- tool definitions are versioned and allowlisted
- unknown and disabled tools are denied
- side-effect levels are enforced
- approval levels are enforced
- approval Level 5 is always prohibited
- mock tool calls validate input and output
- mock tool calls cannot reach real runtimes
- audit records are deterministic and redacted
- private chain-of-thought is never stored
- normal tests remain runtime-free
- no LLM/API integration is introduced
- no simulator, SITL, vehicle, or hardware behavior is introduced

The planning specification alone does not complete v3-1. v3-1A, v3-1B, and
v3-1C must each be implemented and verified in later, separately approved
slices.
