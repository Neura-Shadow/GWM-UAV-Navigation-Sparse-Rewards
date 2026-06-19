# GWM-UAV-C2 v2-6 Optional Simulator Benchmark Integration

## Purpose

v2-6 adds a mock-first, runtime-free benchmark-readiness layer for
GWM-UAV-C2. The layer compares mission, risk, route, replay, dashboard, and
metrics schema compatibility across three profiles:

- `mock`
- `isaac_readiness`
- `cosys_airsim_family_readiness`

This is a schema-readiness and audit comparison only. It does not launch
simulators, probe optional runtime availability, connect to hardware, upload
routes, command vehicles, or claim simulator performance parity.

## Implemented Artifact

The implementation lives in:

- `src/c2/benchmarking.py`
- `tests/test_c2_benchmarking.py`

Public interfaces:

```text
C2BenchmarkReadinessBuilder
build_c2_benchmark_readiness_report(events=None) -> dict
```

The default report schema is:

```text
v2-6-c2-benchmark-readiness
```

## Profile Semantics

`mock` is the default in-memory C2 schema fixture. It is ready by default and
requires no optional runtime.

`isaac_readiness` is a readiness-only profile for a future Isaac Sim / Isaac
Lab path. It records Isaac-oriented coordinate metadata and Phase 6 mainline
status, but it does not import, launch, attach to, or query Isaac Sim.

`cosys_airsim_family_readiness` is a readiness-only profile for the
AirSim-family backend. It records Cosys-AirSim / `cosysairsim` as the preferred
runtime and legacy AirSim / `airsim` as fallback, but it does not import,
launch, attach to, or query either runtime.

## Compared Schema Groups

The report compares these schema groups for every profile:

- mission
- risk
- route
- replay
- dashboard
- metrics

Every profile entry explicitly records:

- runtime required for schema check: `false`
- runtime availability probed: `false`
- runtime connection attempted: `false`
- simulator launched: `false`
- hardware connection attempted: `false`
- vehicle command enabled: `false`
- route upload enabled: `false`
- simulator performance parity claimed: `false`

## Safety Boundary

v2-6 is read-only, audit-oriented, and command-free.

It does not approve route execution. It does not upload routes to PX4,
ArduPilot, MAVSDK, ROS2, Nav2, Isaac Sim, Cosys-AirSim, legacy AirSim, or any
simulator. It does not command vehicles. It does not replace safety decisions
or human approval. It does not make production readiness, simulator parity, or
safety certification claims.

## Verification

Focused verification:

```bash
python -m pytest tests/test_c2_benchmarking.py -q
python -m pytest tests/test_c2_dashboard_replay.py tests/test_c2_dashboard_metrics.py tests/test_c2_replay_report_cli.py -q
python -m compileall -q src tests scripts
git diff --check
```

The broader C2 regression suite should remain mock-only and runtime-free.

## Completion Status

v2-6 is complete when the benchmark-readiness report is deterministic,
JSON-safe, mock-first, runtime-free, and covered by tests that confirm no
simulator launch, runtime probing, vehicle command, route upload, hardware
connection, production-readiness claim, or simulator-parity claim is enabled.
