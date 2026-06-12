# GWM-UAV-C2 v2-3 Defensive Threat and Risk Prediction Plan

## 1. Purpose

v2-3 defines the mock-first defensive risk prediction layer for GWM-UAV-C2.

`v1.0.0` remains the completed archive release. v2-0 froze the C2 concept and
boundaries. v2-1 completed the mission data foundation. v2-2 completed the
mock-first mission dispatch and fleet allocation layer. v2-3 is the next
implementation-oriented planning slice, but this document is docs-only.

## 2. Relationship to Existing v2 Foundation

v2-3 must build only on:

- `src/c2/mission_types.py`
- `src/c2/event_bus.py`
- `src/c2/state_store.py`
- `src/c2/replay.py`
- `src/c2/mission_dispatcher.py`
- `src/c2/fleet_manager.py`

v2-3 should use the existing dataclasses and interfaces:

- `MissionEvent`
- `RiskSignal`
- `ThreatAssessment`
- `UAVState`
- `PlannedRoute`
- `SafetyDecision`
- `MetricSummary`
- `MissionStateStore`
- `MissionReplayEngine`

No new runtime dependency is allowed.

## 3. v2-3 Allowed Scope

Allowed later implementation files:

- `src/c2/risk_prediction.py`
- `tests/test_c2_risk_prediction.py`
- `tests/test_c2_risk_prediction_integration.py`

Allowed capabilities:

- defensive risk category validation
- mock telemetry anomaly interpretation
- rule-based risk scoring
- `RiskSignal` generation
- `ThreatAssessment` generation
- explanation metadata
- recommendation generation
- `MissionEvent` emission
- `MissionStateStore` integration
- `MissionReplayEngine` compatibility
- focused unit tests

## 4. v2-3 Explicit Non-goals

v2-3 must not implement:

- offensive targeting
- attack execution
- payload release
- weapon control
- autonomous attack-decision logic
- real-world pursuit/intercept behavior
- real simulator connection
- real MAVSDK/PX4 connection
- ROS2 runtime node
- Nav2 runtime plugin
- hardware interface
- autonomous flight behavior
- network broker
- database server
- credentials or tokens
- runtime artifacts
- direct vehicle command
- arming
- takeoff
- landing
- mission upload to PX4 or ArduPilot

The risk engine may recommend `continue`, `hold`, `replan`, or
`request_review` only. It must not recommend attack, pursue, intercept,
disable, jam, spoof, or physically engage any target.

## 5. Frozen Defensive Risk Categories

Allowed categories are exactly:

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

Any category outside this list must be rejected with a clear error or mapped to
a safe generic category only if that mapping is explicitly documented.

## 6. Risk Engine Design Plan

The later implementation should define a mock-first class such as:

```text
DefensiveRiskPredictor
```

Recommended API:

```text
__init__(event_bus=None, state_store=None)
evaluate_uav_state(state: UAVState) -> list[RiskSignal]
evaluate_event(event: MissionEvent) -> list[RiskSignal]
evaluate_context(context: dict) -> ThreatAssessment
create_risk_signal(category: str, severity: float, confidence: float, evidence: dict) -> RiskSignal
create_threat_assessment(mission_id: str, risk_signals: list[RiskSignal]) -> ThreatAssessment
publish_risk_signal(signal: RiskSignal) -> MissionEvent
publish_threat_assessment(assessment: ThreatAssessment) -> MissionEvent
```

Required behavior:

- deterministic output
- JSON-safe evidence
- bounded severity in `[0.0, 1.0]`
- bounded confidence in `[0.0, 1.0]`
- bounded total risk in `[0.0, 1.0]`
- clear explanation for non-zero risk
- no runtime side effects
- no external connections

## 7. Initial Rule-based Risk Scoring Plan

Initial implementation should use simple deterministic rules:

- low `link_quality` -> communication degradation
- stale `UAVState.timestamp` -> telemetry anomaly
- battery below threshold -> telemetry anomaly or mission command anomaly
- position outside geofence metadata -> geofence / no-fly-zone violation
- reported `gps_jump` evidence -> GPS spoofing risk
- reported `rf_noise` evidence -> GPS / RF jamming risk
- reported `sensor_fault` evidence -> sensor corruption
- nearby hostile UAV fixture -> hostile UAV proximity
- predicted route conflict fixture -> collision risk
- `wind_speed` above threshold -> weather or wind disturbance
- invalid mission command fixture -> mission command anomaly

Thresholds should be deterministic and documented in code comments or test
fixtures. v2-3 should not use ML models, online learning, simulator streams, or
runtime telemetry feeds.

## 8. Recommendation Policy Plan

Allowed `ThreatAssessment.recommendation` values:

- `continue`
- `hold`
- `replan`
- `request_review`

Suggested deterministic policy:

```text
total_risk == 0.0 -> continue
0.0 < total_risk < 0.4 -> continue with explanation
0.4 <= total_risk < 0.7 -> replan
0.7 <= total_risk < 0.9 -> hold
total_risk >= 0.9 -> request_review
```

This policy is research-grade and mock-first. It is not certified safety logic.

## 9. Event Flow Plan

Expected event flow:

```text
UAVState or MissionEvent input
-> DefensiveRiskPredictor evaluates mock context
-> risk.signal.created
-> threat.assessment.created
-> MissionStateStore update
-> optional MissionReplayEngine replay
-> MetricSummary includes risk counts
```

Event payload expectations:

- `risk.signal.created` -> `RiskSignal.to_dict()`
- `threat.assessment.created` -> `ThreatAssessment.to_dict()`
- payloads are JSON-safe
- known invalid payloads raise `ValueError`
- unknown event types remain preserved by state store / replay

## 10. State Store and Replay Integration Plan

v2-3 should not replace storage or replay. It should use:

- `MissionStateStore.apply_event()`
- `MissionStateStore.snapshot()`
- `MissionReplayEngine.replay()`
- `MetricSummary.risk_counts`

Requirements:

- risk signals persist in state store
- threat assessments persist in state store
- replay frame generation remains deterministic
- risk counts aggregate only allowed defensive categories
- no file writes by default
- no dashboard server
- no runtime connection

## 11. Test Plan for v2-3 Implementation

Focused tests:

- `test_risk_predictor_accepts_allowed_categories`
- `test_risk_predictor_rejects_forbidden_category`
- `test_risk_signal_generation_json_safe`
- `test_threat_assessment_generation`
- `test_threat_assessment_recommendation_continue`
- `test_threat_assessment_recommendation_replan`
- `test_threat_assessment_recommendation_hold`
- `test_threat_assessment_recommendation_request_review`
- `test_low_link_quality_maps_to_communication_degradation`
- `test_stale_uav_state_maps_to_telemetry_anomaly`
- `test_gps_jump_maps_to_gps_spoofing_risk`
- `test_sensor_fault_maps_to_sensor_corruption`
- `test_geofence_violation_maps_to_geofence_risk`
- `test_risk_event_emission`
- `test_risk_state_store_integration`
- `test_risk_replay_metrics_integration`
- `test_risk_predictor_imports_without_runtime_dependencies`

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

## 12. Verification Commands for Future v2-3 Implementation

Future implementation should run:

```bash
python -m pytest tests/test_c2_risk_prediction.py tests/test_c2_risk_prediction_integration.py -q
python -m pytest tests/test_c2_mission_types.py tests/test_c2_event_bus.py tests/test_c2_state_store.py tests/test_c2_replay.py tests/test_c2_mission_dispatcher.py tests/test_c2_fleet_manager.py tests/test_c2_dispatcher_fleet_integration.py -q
python -m compileall -q src tests
git diff --check
rg -n "offensive attack|weapon|targeting|payload release|autonomous attack|production-ready|certified safety|real hardware validation|autonomous real flight|arming|takeoff|landing|mission upload|attack execution|weapon control" src/c2 tests docs || true
```

Expected grep hits are acceptable only in explicit non-goal/safety statements.

## 13. v2-3 Completion Criteria

v2-3 implementation will be complete later when:

- `DefensiveRiskPredictor` exists
- `RiskSignal` generation tests pass
- `ThreatAssessment` generation tests pass
- recommendation policy tests pass
- allowed defensive categories are enforced
- forbidden/offensive categories are rejected
- risk events integrate with `MissionStateStore`
- risk metrics integrate with `MissionReplayEngine`
- all normal tests remain runtime-free
- no simulator/hardware dependencies are introduced
- no offensive automation is introduced

## 14. Recommended Implementation Split

Recommended implementation slices:

- v2-3A: Defensive risk predictor core
- v2-3B: Rule-based risk mapping and recommendation policy
- v2-3C: Risk event, state store, and replay metrics integration

v2-3A should be the next implementation slice after this planning spec is
reviewed.

## 15. Status Update

v2-3A implements the defensive risk predictor core, category validation,
`RiskSignal` factory, and `ThreatAssessment` factory in
`src/c2/risk_prediction.py`. Rule-based risk mapping and state-store/replay
integration remain planned but not implemented.
