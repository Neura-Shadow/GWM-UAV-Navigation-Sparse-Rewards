# GWM-UAV-C2 v2-2 Mission Dispatcher and Fleet Manager Plan

## 1. Purpose

v2-2 will define the mock-first mission dispatch and fleet allocation layer for
GWM-UAV-C2.

`v1.0.0` remains the completed archive release. v2-0 froze the C2 concept and
boundaries. v2-1 completed the mission data model, event bus, state store,
replay, and metrics foundation. v2-2 is the next implementation-oriented
planning slice, but this document is docs-only.

Status update: v2-2A implements the mock-first `MissionDispatcher` in
`src/c2/mission_dispatcher.py`.

Status update: v2-2B implements the mock-first `FleetManager` in
`src/c2/fleet_manager.py`. Dispatcher/fleet integration remains planned but not
implemented.

## 2. Relationship to v2-1 Foundation

v2-2 must build only on:

- `src/c2/mission_types.py`
- `src/c2/event_bus.py`
- `src/c2/state_store.py`
- `src/c2/replay.py`

The dispatcher and fleet manager should use the existing dataclasses and
helpers, including:

- `MissionRequest`
- `MissionTask`
- `FleetAsset`
- `UAVState`
- `MissionEvent`
- `SafetyDecision`
- `HumanApprovalRecord`
- `MetricSummary`

No new runtime dependency is allowed. v2-2 should remain pure Python,
mock-first, deterministic, import-safe, and in-memory.

## 3. v2-2 Allowed Scope

Allowed later implementation files:

- `src/c2/mission_dispatcher.py`
- `src/c2/fleet_manager.py`
- `tests/test_c2_mission_dispatcher.py`
- `tests/test_c2_fleet_manager.py`
- `tests/test_c2_dispatcher_fleet_integration.py`

Allowed capabilities:

- mock-first mission request validation
- mission task creation
- mission task status transition
- deterministic fleet asset registration
- fleet asset availability checks
- simple deterministic asset assignment
- refusal reason generation
- `MissionEvent` emission
- `MissionStateStore` integration
- in-memory only behavior
- focused unit tests

## 4. v2-2 Explicit Non-goals

v2-2 must not implement:

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
- direct vehicle command
- arming
- takeoff
- landing
- mission upload to PX4 or ArduPilot

## 5. Mission Dispatcher Design Plan

The mock-first `MissionDispatcher` should validate operator requests, create
mission tasks, emit deterministic mission events, and optionally apply those
events to an injected `MissionStateStore`.

Recommended responsibilities:

- validate `MissionRequest`
- create `MissionTask`
- reject invalid requests with refusal reasons
- publish `mission.requested` event
- publish `mission.task.created` event
- publish `mission.task.updated` event when status changes
- optionally write events into `MissionStateStore`
- keep deterministic behavior

Recommended API:

```text
MissionDispatcher
- __init__(event_bus=None, state_store=None)
- submit_request(request: MissionRequest) -> MissionTask
- create_task(request: MissionRequest) -> MissionTask
- update_task_status(task_id: str, status: str, reason: str = "") -> MissionTask
- block_task(task_id: str, reason: str) -> MissionTask
- cancel_task(task_id: str, reason: str) -> MissionTask
- get_task(task_id: str) -> Optional[MissionTask]
- list_tasks() -> List[MissionTask]
```

Required behavior:

- task ids are deterministic when possible
- invalid `MissionRequest` raises `ValueError` or returns an explicit refusal,
  depending on the chosen implementation design
- all task outputs are JSON-safe
- no direct command payloads are accepted
- no runtime side effects are produced
- events are deterministic and replayable

Task status rules:

```text
pending -> assigned
pending -> blocked
pending -> cancelled
assigned -> completed
assigned -> blocked
assigned -> cancelled
blocked is terminal unless future review explicitly reopens it
completed is terminal
cancelled is terminal
```

## 6. Fleet Manager Design Plan

The mock-first `FleetManager` should track in-memory fleet assets and state,
filter eligible assets, assign tasks deterministically, emit fleet events, and
optionally apply those events to an injected `MissionStateStore`.

Recommended responsibilities:

- register `FleetAsset`
- update `FleetAsset`
- update `UAVState`
- track availability
- filter assets by capability
- filter unavailable assets
- assign task to best available asset deterministically
- produce refusal reasons when no asset is available
- publish `fleet.asset.registered` event
- publish `fleet.asset.updated` event
- publish `uav.state.updated` event
- optionally write events into `MissionStateStore`

Recommended API:

```text
FleetManager
- __init__(event_bus=None, state_store=None)
- register_asset(asset: FleetAsset) -> FleetAsset
- update_asset(asset: FleetAsset) -> FleetAsset
- update_uav_state(state: UAVState) -> UAVState
- list_assets() -> List[FleetAsset]
- get_asset(asset_id: str) -> Optional[FleetAsset]
- available_assets(required_capability: Optional[str] = None) -> List[FleetAsset]
- assign_task(task: MissionTask, required_capability: Optional[str] = None) -> MissionTask
- release_asset(asset_id: str) -> FleetAsset
```

Deterministic assignment policy:

- only assign available assets
- if `required_capability` is provided, the asset must include that capability
- prefer assets with no `current_task_id`
- break ties by lexicographic `asset_id`
- if no asset is eligible, return or raise a clear refusal reason
- do not command, arm, launch, land, or upload missions

## 7. Event Flow Plan

Expected event flow:

```text
MissionRequest submitted
-> mission.requested
-> mission.task.created
-> FleetManager assignment attempt
-> fleet.asset.updated
-> mission.task.updated
-> optional replay through MissionReplayEngine
```

Event payload expectations:

- event payloads are `dataclass.to_dict()`
- payloads must be JSON-safe
- unknown event types remain preserved by state store / replay
- known invalid payloads raise `ValueError`

## 8. State Store Integration Plan

Dispatcher and fleet manager should use `MissionStateStore` by either applying
each `MissionEvent` after publish or publishing through a helper that also
applies the event to the store.

The state store remains:

- in-memory
- JSON-safe for snapshot/restore
- deterministic
- file-write-free by default
- database-free
- network-free

v2-2 should not change the storage backend.

## 9. Refusal Reason Plan

Standard refusal reason categories:

- `invalid_request`
- `missing_objective`
- `invalid_priority`
- `area_not_allowed`
- `no_available_asset`
- `missing_required_capability`
- `asset_unavailable`
- `asset_already_assigned`
- `invalid_task_status_transition`
- `task_not_found`
- `asset_not_found`
- `human_approval_required`
- `safety_blocked`

All refusal reasons must be:

- deterministic
- string-based
- JSON-safe
- replayable in `MissionEvent` metadata or payload

## 10. Test Plan for v2-2 Implementation

Focused tests:

- `test_dispatcher_submit_request_creates_task`
- `test_dispatcher_invalid_request_rejected`
- `test_dispatcher_emits_mission_requested_event`
- `test_dispatcher_emits_task_created_event`
- `test_dispatcher_task_status_transition`
- `test_dispatcher_invalid_status_transition_rejected`
- `test_dispatcher_block_task_records_reason`
- `test_fleet_manager_register_asset`
- `test_fleet_manager_update_asset`
- `test_fleet_manager_update_uav_state`
- `test_fleet_manager_available_assets`
- `test_fleet_manager_assigns_available_asset`
- `test_fleet_manager_capability_filter`
- `test_fleet_manager_no_available_asset_refusal`
- `test_fleet_manager_assignment_is_deterministic`
- `test_dispatcher_fleet_integration_event_replay`
- `test_dispatcher_fleet_imports_without_runtime_dependencies`

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

## 11. Verification Commands for Future v2-2 Implementation

Future implementation should run:

```bash
python -m pytest tests/test_c2_mission_dispatcher.py tests/test_c2_fleet_manager.py tests/test_c2_dispatcher_fleet_integration.py -q
python -m pytest tests/test_c2_mission_types.py tests/test_c2_event_bus.py tests/test_c2_state_store.py tests/test_c2_replay.py -q
python -m compileall -q src tests
git diff --check
rg -n "offensive attack|weapon|targeting|payload release|autonomous attack|production-ready|certified safety|real hardware validation|autonomous real flight" src/c2 tests docs || true
```

Expected grep hits are acceptable only in explicit non-goal/safety statements.

## 12. v2-2 Completion Criteria

v2-2 implementation will be complete later when:

- `MissionDispatcher` exists
- `FleetManager` exists
- dispatcher request-to-task tests pass
- fleet asset registration tests pass
- fleet assignment tests pass
- refusal reason tests pass
- dispatcher/fleet event integration tests pass
- replay remains deterministic
- all normal tests remain runtime-free
- no simulator/hardware dependencies are introduced
- no offensive automation is introduced

## 13. Recommended Implementation Split

Recommended implementation slices:

- v2-2A: Mission Dispatcher
- v2-2B: Fleet Manager
- v2-2C: Dispatcher/Fleet integration and replay validation

v2-2A should be the next implementation slice after this planning spec is
reviewed.
