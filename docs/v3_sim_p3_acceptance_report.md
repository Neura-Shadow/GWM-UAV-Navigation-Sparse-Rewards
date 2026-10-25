# v3-SIM P3 consolidated acceptance report

**P3 INCOMPLETE — Gate A has not been authorized or executed (no runtime evidence exists for this implementation) — Qualification 0/3**

Date: 2026-10-25. Scope of this work: Steps 1–7 of the P3 implementation brief (audit,
implementation, offline verification, offline source freeze). Step 8 (Gates A/B/C) needs
separate operator authorization and the Linux runtime in WSL. **No PX4, Gazebo, ROS node,
bridge, QGC or flight was started.** No historical P3 run is counted.

## A. Implementation

### Starting point (verified)

| Item | Value |
|---|---|
| Branch / HEAD | `v3/gwm-uav-c2-agentops-planning` at `6204fedb9ac8a2b554ff825c93159a509a1b91ad` (= `origin`) |
| Accepted P2 | R3 nominal smoke `20260912T034917Z-p2-flight-c74f35ba`; batch `20260912T035329Z-p2-repeat-84bb288d` **20/20** |
| P2 controller package | `e1857b3fc0a5678c0789b39860ba36cae5193b8daa83a20c246a02f49bb431af` — recomputed, unchanged |
| Pins | PX4 v1.17.0 `d6f12ad1…`, px4_msgs `86d8239e…`, DDS Agent v2.4.3 `73622810…`, Jazzy, Harmonic 8.15.0, QGC 5.1.4 |
| Earlier P3 work | 27 unaccepted commits `76f014d..39ba887`, reachable only via reflog/`ORIG_HEAD`; now preserved as branch `archive/p3-history-39ba887`; stash preserved as `archive/p3-stash-1db3574` |
| Untracked work preserved | MC causal-trace overlay sources/tests and P3 MC evidence docs (never committed); `.codegraph/` |

### Reused from the archived P3 history (explicitly, not as evidence)

* Frozen fixture geometry and acceptance thresholds (`p3_validation.yaml` at `76f014d`).
* SHM-64 MiB sensor transport profile (`p3_fastdds.xml`) and bridge config.
* Pinned MAVLink CRC table and native-header-generated MAVLink fixture frames.
* QGC unattended-shutdown overlay (`overlay.py`, `GwmSitlShutdown.h`, teardown verifier, Qt test
  generator), with the compiled owner identity moved to the stable `p3_namespace_owner.py`.
* Policy A payload patterns from the archived decision package (approval does not transfer).
* The untracked MC causal-trace overlay, consumed unchanged; its 23 tests pass with the new
  `p3_provenance` module.

### New components

| Component | Files |
|---|---|
| Sensor adapter (ament_python) | `ros2_ws/src/gwm_sensor_adapter/` — contracts, depth, camera_info, geometry, association, pipeline, recorder, node |
| Configuration | `configs/p3_sensors.yaml`, `p3_validation.yaml`, `p3_sensor_bridge.yaml`, `p3_fastdds.xml`, `p3_native_command_policy.json` |
| Runtime | `scripts/p3_runner.py`, `p3_namespace_owner.py`, `run_p3.sh`, `run_p3_gates.py`, `p3_manifest.py`, `p3_build.py`, `build_p3.sh`, `verify_p3_evidence.sh`, `qgc_unattended.py`, `build_qgc_unattended.py`, `test_qgc_unattended.py` |
| Independent evaluation | `validation/p3_evaluate.py`, `p3_stream_eval.py`, `p3_geometry_eval.py`, `p3_process_exit.py`, `p3_mavlink.py`, `p3_source_probe.py`, `p3_fixtures.py`, `p3_provenance.py` |
| Tests | `tests/test_px4_gazebo_p3_sensor.py`, `test_px4_gazebo_p3_evaluators.py`, `test_px4_gazebo_p3_runtime.py` (+ fixtures) |
| Docs | [contract](v3_sim_p3_contract.md), this report, README P3 section, roadmap, scope amendment |

Tracked files modified: `docs/roadmap.md`, `docs/v3_simulation_scope_amendment.md`,
`simulation/px4_gazebo/README.md`, `simulation/px4_gazebo/.gitattributes` (LF rules for new
file types). No P2 source, config, evaluator or evidence file was modified.

### P2 compatibility guarantees

The P2 controller package, mission, numeric limits, yaw policy, transport, timing contract,
evaluators and runner are byte-identical; P3 flights run the unchanged `p2_control.yaml` and
are judged by the unchanged `collect_p2_evidence.verify()` plus P3 checks. The sensor adapter
cannot create PX4 input publishers, and the P2 graph check rejects a competing publisher.

## B. Verification

### Offline regression (this checkout, Linux VM, Python 3.10, pytest 9.1.1)

| Suite | Result |
|---|---|
| Accepted P2/P1 PX4 contract tests (5 files) | **212 passed** |
| P3 sensor adapter (decode, calibration, geometry, association, recorder, pipeline) | **60 passed** |
| P3 runtime boundary (P2 identity, cleanup, MAVLink policy, gates, QGC, probe, shutdown order) | **54 passed** |
| P3 independent evaluators on synthetic run directories | **26 passed** |
| Untracked MC causal-trace tests, against the new provenance module | **23 passed** |
| Untracked `test_px4_mc_trace_shutdown.py` | **import error** — targets `close_landed_controller`/`finalize_trace_evidence`, which exist only in the stashed P2-runner edit, not in accepted P2 |
| Full ordinary repository suite (`pytest tests`, collection errors continued) | **882 passed, 1 skipped**; 1 failed and 23 files not collectable only because `torch` is absent from this VM (the project's Windows Anaconda environment provides it; none of these files is touched by P3); 1 file is the untracked MC shutdown test above |

Synthetic inputs prove contracts and decision logic only. They are **not** native sensor
evidence. They show that the evaluator: accepts closed-form plane/oblique/out-of-range
renders; rejects a 0.2 m bias, Euclidean-range rendering, mirrored images, fabricated
out-of-range values, zero-filled unknowns, a single delivery loss, a 5 cm vehicle drift,
stale health, a tampered raw frame, a missing ledger or partial JSONL; and fails the
historical Gazebo −11 exit, residual processes, changed artifacts and an unselected MC trace.

### Not verified (requires WSL runtime or build)

* `colcon` build/test of `gwm_sensor_adapter` and the installed-package test.
* The SDF chain re-derivation against the real pinned model files (`p3_build.py`).
* gz-transport probe subscriptions, rclpy adapter callbacks, parameter_bridge behaviour,
  AF_PACKET capture permission, Gazebo world control/pause, QGC rebuild with the new owner.
* Every real sensor measurement, timing, resource, RTF, renderer, MC causality, LAND and
  cleanup result. **No such evidence exists for this implementation.**

### Offline source freeze (repository side)

Canonical SHA-256 of the 96 hashed source/config/evaluator files plus the accepted P2
summary: `6bd621c9152493c5f64ad3015edbe752dee08b994a6019c4c4e6a5cc063d9a29`; of the 8
required MC sources: `05c0f120546b505cff3ee1783324c6eff18549ebef01d81573c8bc300829d532`.
Selected file hashes: `p3_namespace_owner.py` `0d1c2d27…eb74`, `p3_runner.py` `9b4c6db4…de3b`,
`p3_evaluate.py` `5fc4e3da…542e`, `p3_validation.yaml` `b657a393…7965`, `p3_sensors.yaml`
`6c3038fe…a9a4`, `p3_native_command_policy.json` `2e6ab07b…45c3`, `p2_control.yaml`
`e50f824a…7a4d`. The **runtime candidate manifest has not been frozen**: it must be produced in
WSL by `p3_manifest.py freeze` after the builds, because it binds Linux receipts, binaries,
assets and libraries that are not reachable from this session.

## C. Runtime acceptance

| Gate | Result |
|---|---|
| Gate A — readiness + 7 fixtures | **Not run** (not authorized) |
| Gate B — P3 nominal flight smoke | **Not run** (requires accepted A) |
| Gate C — three qualification flights | **Not run** (requires accepted B) |
| Accepted qualification count | **0/3** |
| Run IDs | none |
| Frozen runtime manifest SHA | none yet |

## D. Outstanding issues

1. **First genuine failing invariant:** runtime acceptance has never been attempted for this
   implementation; every runtime invariant is unproven. The first gate to run is Gate A readiness.
2. **Known runtime risks, cause not established:**
   * Gazebo exited −11 (SIGSEGV) after TERM in all archived fixtures. Cause unknown. The new
     shutdown removes sensor consumers, pauses the world and sends SIGINT; this is an untested
     mitigation, and any non-zero exit still fails cleanup.
   * The archived instrumented smoke failed on a 2.008 s wall receipt age for
     `vehicle_land_detected` (limit 2 s) under the combined depth+RGB workload. The limit is
     unchanged. If the measured workload cannot meet it, the honest result is a reported
     bottleneck plus a separately reviewed threshold or workload decision — not a relaxation.
3. **Smallest remaining corrective actions (operator, in WSL):**
   1. `build_p3.sh --build` (also re-derives the camera extrinsic from the pinned SDF).
   2. Rebuild the unattended QGC with `build_qgc_unattended.py` (owner is now `p3_namespace_owner.py`).
   3. Confirm the MC overlay receipt and a passed `px4-mc-overhead-v1` benchmark run exist
      (`benchmark_mc_trace.py` reads `configs/p3_control.yaml`, which exists only in the archive;
      reuse the existing passed benchmark run, or decide how to re-run it).
   4. Decide the native-command policy: `strict_zero` (default; stock QGC's automatic 401/512
      will fail it) or re-approve Policy A for this candidate with a bound approval receipt.
   5. `p3_manifest.py freeze …`, then authorize and run Gate A only.
4. **Another runtime authorization is needed:** yes — one explicit, manifest-bound authorization
   per gate (A, then B, then C).

## E. Commit-time review (2026-10-25)

Repository review before the single local P3 commit; no runtime was started.

* HEAD and P2 checkpoint: `6204fedb9ac8a2b554ff825c93159a509a1b91ad`. `git diff 6204fed` over
  `ros2_ws/src/gwm_px4_control`, `configs`, `scripts` and `validation` is empty: the accepted P2
  controller, configuration, runner and evaluators are byte-identical.
* Preserved, not modified: branches `archive/p3-history-39ba887`, `archive/p3-stash-1db3574`;
  `stash@{0}` (tracked P3 edits made before restoring 6204fed).
* The four tracked edits are documentation/attributes only (roadmap, scope amendment, README P3
  section, LF rules for the new file types).
* The 8 manifest-required MC causal-trace sources, `tests/test_px4_mc_trace.py` and its ABI header
  fixtures were untracked before this cycle; they are committed unchanged because `p3_manifest.py`,
  `p3_runner.py` and `p3_evaluate.py` require them. Left untracked on purpose: `.codegraph/`, the
  MC contract/validation docs, all four historical `docs/evidence/v3_sim_p3_*` JSON files and
  `tests/test_px4_mc_trace_shutdown.py` (it targets functions that exist only in the stash).

### Offline re-run on Windows (Python 3.13.5, no GWM_* runtime variables set)

| Suite | Result |
|---|---|
| Accepted P2/P1 contract tests (5 files) | **212 passed** |
| `test_px4_gazebo_p3_sensor.py` | **60 passed** |
| `test_px4_gazebo_p3_runtime.py` | 43 passed, **11 failed** — all POSIX-only: `fcntl` import (9), `signal_9` vs `SIGKILL` naming (1), Windows path passed to `bash` (1) |
| `test_px4_gazebo_p3_evaluators.py` | 25 passed, **1 failed** — `test_euclidean_range_rendering_is_rejected_for_oblique` passes alone but fails in the full file: the synthetic recorder misses its 10 s close deadline under load, the evaluator rejects the run with `raw_trailing_data` instead of the asserted `p95_depth_error` (still fail-closed) |
| `test_px4_mc_trace.py` | **23 passed** |
| `test_px4_mc_trace_shutdown.py` | **import error** (see above; not committed) |

The Linux results in section B were not re-run in this review. Missing coverage is unchanged:
no `colcon` build/test, no SDF re-derivation against pinned assets, no rclpy/gz-transport/bridge,
QGC or MAVLink-capture execution, and no native sensor, timing, resource, MC-causality, LAND or
cleanup evidence.

### Unresolved runtime prerequisites (all before Gate A)

1. **QGC rebuild** — the unattended QGC must be rebuilt with `build_qgc_unattended.py` because the
   compiled owner identity is now `p3_namespace_owner.py`; the existing binary is not valid.
2. **MC benchmark compatibility** — `benchmark_mc_trace.py` reads `configs/p3_control.yaml`, which
   exists only on `archive/p3-history-39ba887`; reuse of the earlier passed `px4-mc-overhead-v1`
   run or a re-run must be decided before `p3_manifest.py freeze`.
3. **Native command policy approval** — `strict_zero` is the default and stock QGC's automatic
   401/512 traffic will fail it; Policy A needs a fresh operator approval receipt bound to this
   candidate (archived approval does not transfer).
4. **Gazebo shutdown** — the archived −11 (SIGSEGV) exit after TERM is unexplained; the new
   pause + SIGINT shutdown is an untested mitigation and any non-zero exit still fails cleanup.

Final status: **P3 INCOMPLETE — runtime acceptance not authorized/executed (first invariant: Gate A readiness unproven) — Qualification 0/3**
