# v3-SIM P3 consolidated acceptance report

**P3 INCOMPLETE — contract v1.6 candidate `1e205fec`: Gate A and Gate B ACCEPTED (sections AI, AJ); Gate C not run — Qualification 0/3**

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

## F. Offline preflight closure (2026-10-25, on top of `d05e95e`)

No PX4 SITL, Gazebo, ROS node, bridge, QGC application, benchmark run, gate or flight was
started. Accepted P2 sources and thresholds are untouched (P2 package hash test passes).

### Changes

| File | Change | Why |
|---|---|---|
| `validation/p3_process_exit.py` | Signal names come from a fixed Linux x86_64 table instead of the host `signal` module | Exit codes are recorded on Linux; on Windows −9 was named `signal_9`. Still a failure on every host; only the label becomes host-independent |
| `tests/test_px4_gazebo_p3_runtime.py` | `POSIX_ONLY` skip marker (with reason) on 10 tests that import `p3_runner` (`fcntl`, process groups) or run `run_p3.sh` through bash; new benchmark-config test | Windows cannot run them; they are reported as skipped, and all of them pass on Linux |
| `tests/test_px4_gazebo_p3_evaluators.py` | Synthetic harness waits for the recorder writer to drain before `close(10)`, then asserts `complete` | The order-dependent failure was host load: the faster-than-real-time synthetic producer missed the 10 s wall deadline, and the harness never checked `complete`. Production recorder and deadline unchanged |
| `scripts/benchmark_mc_trace.py` | Validates the trace against `configs/p2_control.yaml` instead of the archived `p3_control.yaml` | See MC benchmark below |
| `docs/v3_sim_p3_policy_a_proposal.md` | New: narrowly scoped policy A approval proposal | Proposal only; `strict_zero` remains the default |

### Windows results (Python 3.13.5, pytest 9, no `GWM_*` variables)

| Suite | Result |
|---|---|
| P2/P1 accepted (5 files) | 212 passed |
| P3 sensor | 60 passed |
| P3 evaluators | 26 passed (repeated: stable) |
| P3 runtime | 45 passed, **10 skipped** (POSIX-only, reason printed; not counted as passed) |
| MC causal-trace | 23 passed |
| Full ordinary suite (`--continue-on-collection-errors`, torch available) | 1166 passed, 22 skipped (10 are the P3 POSIX-only tests), 1 collection error: the untracked `test_px4_mc_trace_shutdown.py` |

### Linux results (WSL2 Ubuntu 24.04.3, kernel 6.18.40.1, Python 3.12.3, pytest 7.4.4)

| Suite | Result |
|---|---|
| P2/P1 accepted (5 files) | **212 passed** |
| P3 sensor | **60 passed** |
| P3 evaluators | **26 passed** |
| P3 runtime | **55 passed** (includes the 10 Windows-skipped tests) |
| MC causal-trace | **23 passed** |
| `colcon build` + `colcon test` of `gwm_sensor_adapter` (Jazzy, scratch workspace in `/tmp`) | build OK; **2 tests, 0 failures** (real `sensor_msgs`/`rclpy` serialization; no node) |
| Full ordinary suite (`--continue-on-collection-errors`) | **883 passed, 1 skipped, 1 failed, 24 collection errors** — 21 errors and the failure are `torch` absent in WSL; 3 errors are `src.ros2_bridge` failing to import because of `torch`; 1 is the untracked `test_px4_mc_trace_shutdown.py` (stash-only functions). P3 touches none of these files |
| `py_compile` (38 P3/MC Python files), `bash -n` + `shellcheck -S warning` (3 scripts), JSON/YAML (6), `xmllint` (2) | all clean |
| Owned processes after the runs | none |

Still not verified: every native sensor, timing, resource, RTF, renderer, MC causality, LAND,
MAVLink capture and cleanup result; the SDF re-derivation in `build_p3.sh --build`.

### QGC unattended build (offline; QGC application never executed)

| Item | Value |
|---|---|
| Pinned source | `4568b585648e8c6065cca6c650b52731c1d3baa3` (`~/uav_autonomy/qgc_closure_v1/source`), Qt 6.11.1 gcc_64, Release, `GWM_QGC_SITL_SHUTDOWN_BUILD=ON` |
| Owner bound | `p3_namespace_owner.py` `0d1c2d27c330260475f3f17892f9e4b2612678e9abbdf8301a7a4aa25ce8eb74` (previous build bound `2d765506…`, now invalid) |
| Overlay | re-applied to clean source; only `GwmSitlBuildIdentity.h` differs from the previous overlay; receipt `overlay-owner-0d1c2d27-20261025T050446Z.json` `e2312969…de33` |
| Previous overlay state | preserved: `source-overlay-backups/owner-2d765506-20261025T050446Z.tar` `855c7bdd…01fa` (+ `.diff`); previous ELF `34c39a80…0a71` kept in `bundles/` |
| Offscreen Qt component tests | **passed 8/8** (qml, interactive-default and opt-in guard, signal fixture); receipt `8bfabafb…051f`; `live_graceful_shutdown_proven = false` |
| **New ELF** | **`829942834b71bbd7f30967cb32151e7a32bc1a7fbb9609fd15e2f4dde3e46d03`** (owner hash embedded once); bundle `bundles/829942…6d03`; `ldd -r` clean; 695 native files hashed |
| Receipt for the manifest | `~/uav_autonomy/state/qgc-unattended-built.json` `0c836b52211213a2bddcf549ec62ab29f7982f05d6c42e2241c662e4baf98263`, status `built_offline_not_runtime_verified` |
| Interactive behavior | unchanged: the opt-in CMake option defaults OFF, the guard arms only after an owner-verified TERM, and the stock AppImage `2600af0f…f3ae` is untouched |

### MC benchmark compatibility

`benchmark_mc_trace.py` was written while `configs/p3_control.yaml` existed (archived P3
history). It reads that file only to pass `max_sample_gap_sim_s` and `position_fresh_sim_s`
to `mc_trace_evidence.validate_records`. Those are the only `config[...]` keys that module
reads, and both have identical values in the accepted `p2_control.yaml` (0.2 s and 0.25 s).
`p3_runner` already passes `p2_control.yaml` for flight acquisition. The benchmark now does
the same. The binary, workload (3 pairs × 10 000 iterations), 50 ms wall limit,
output-bit equality, causality checks and the 1e-5 rad yaw tolerance are unchanged.

Reuse decision: both earlier passed runs (`20260912T154212Z-…-0ef339f6`,
`20261024T071853Z-…-e32a9d00`) match the current MC build (instrumented binary `7542919c…`,
overlay `ecdf4eaa…`, benchmark `.cpp` `e713cf13…`, identical executables and libraries).
They were produced by the pre-correction harness, and no harness hash was recorded. The
implementation is therefore **not demonstrably identical**, so they are **not reused**. A
new `benchmark_mc_trace.py --measure` run is required before `p3_manifest.py freeze`. It
is a native microbenchmark with no simulation, but it executes PX4 controller code, so it
was not run here.

**New run (operator-requested, 2026-10-25):** `20261025T051926Z-mc-trace-benchmark-e6000074`,
`summary.json` SHA-256 `1c6af0c76c2d21b4484dfea3d40d0870006e040284c6295c4f6efd1467d2abd1`,
status **passed**. One run with no retries; 0 simulation launches and 0 flight commands.

| Item | Value |
|---|---|
| Harness / benchmark source | `benchmark_mc_trace.py` `0b9b43a1…8ba1` and `mc_trace_benchmark.cpp` `e713cf13…c4dc`, byte-identical to commit `2bb7bd0` |
| Config | `p2_control.yaml` `e50f824a…7a4d` |
| Build | instrumented PX4 `7542919c…a378a`, overlay `ecdf4eaa…0bf9` (= current MC build receipt) |
| Executables | baseline `dbba6f66…5dfa`, instrumented `a46ae781…e900` |
| Pairs 1–3 | output bits equal; trace acquisition `verified`; instrumented wall max 0.288 / 0.059 / 0.151 ms (limit 50 ms); added mean wall 5.4 / 4.8 / 5.3 µs |

This run (not the earlier ones) is the `--mc-benchmark-run` input for `p3_manifest.py freeze`.
It is build evidence only and carries no qualification credit.

### Native command policy

The effective policy is still `strict_zero`, and stock QGC 401/512 start-up traffic is
expected to fail it. Policy A (canonical SHA-256 `ea307bf5…1295`: 1 × 401 + 34 × 512,
startup ≤ 75 s, GCS 255/190 → 72/1 only) is described in the
[approval proposal](v3_sim_p3_policy_a_proposal.md). It is not approved. The receipt
loader binds only the policy hash, not a candidate, so an approval must be tied to one
frozen manifest by the operator.

### Gazebo shutdown risk (offline review only)

* Retained evidence: `docs/evidence/v3_sim_p3_gate_A_e1a674c.json` on
  `archive/p3-history-39ba887`. The `gazebo` process (pid 525, process name `ruby`, the
  `gz` CLI frontend) exited −11 0.67 s after the single requested SIGTERM, not forced,
  with no group members left. Legacy cleanup marked it `passed`. That label is wrong and
  was never reused. The same archive records −11 "in every fixture, beginning with plane2".
* The older WSL run summaries from Sep 12 use a schema with no stop method or forced flag
  (mostly 0, −9 in `20260912T111419Z-p3-plane2` and `20260912T160921Z-p3-plane6`). They
  cannot show what the stop signal was and are not counter-evidence.
* No core dump, backtrace or Gazebo crash log was retained, so the root cause remains
  unknown.
* The new runner pauses the world and sends SIGINT. This is an untested mitigation.
  `p3_process_exit` classifies SIGSEGV, SIGKILL, unexpected SIGTERM, forced kill, a missed
  deadline, leftover group members and an unreaped process as failures, and this is
  covered by tests on both hosts.

### Blockers before Gate A

1. ~~New MC benchmark run~~: done, `20261025T051926Z-mc-trace-benchmark-e6000074` passed.
2. `build_p3.sh --build` (P3 build plus SDF extrinsic re-derivation) and the `p3-built.json`
   receipt for this candidate. `p3_build.py` overwrites `~/uav_autonomy/state/p3-built.json`,
   which currently holds the archived receipt from 2026-09-12, so preserve a copy first.
3. Native command policy: keep `strict_zero` (expected Gate A failure on stock traffic) or
   issue a policy A approval receipt tied to one manifest.
4. `p3_manifest.py freeze` with the new benchmark run and the QGC receipt above, then one
   explicit Gate A authorization for that manifest SHA.
5. Gazebo abnormal-exit risk stays open. A non-zero or signal exit fails Gate A cleanup.

## G. Build, policy approval and frozen candidate (2026-10-25)

* **Historical receipt preserved:** `state/archive/p3-built-20260912-historical.json` `214f1149…f355`.
* **First `build_p3.sh --build`** refused: the SDF-derived base origin was z=0, but the config
  says 0.24. `sdf_chain` ignored an included model's own `<pose>`, which pinned `x500_base`
  uses. This was fixed in `0d38eec`; the config values were correct and are unchanged, and a
  regression test now mirrors the pinned layout.
* **Second build passed:** sensor package `f6605fb7…5944`, colcon tests 2/2, extrinsic
  re-derived from the pinned SDF matches `p3_sensors.yaml`; `state/p3-built.json` `ae3ea317…804a`.
* **Linux regressions on `0d38eec`:** P2 212, P3 sensor 61, evaluators 26, runtime 55, MC 23,
  all passed.
* **Policy A** approved by operator **Neura-Shadow** for this candidate only (policy
  unchanged, `ea307bf5…1295`). Receipt `state/p3-policy-A-approval-0d38eec.json`
  `0b24a8aa…26d4`, bound to commit `0d38eec` and QGC ELF `82994283…6d03`.
* **Frozen candidate:** `state/p3-candidate-0d38eec.json`, canonical manifest SHA-256
  **`c4b00bedf07b4c87867346b95ef3bc6d12ab28ec891e1f96ded99477fd73e3d1`** (file `f789d007…97bc`).
  `p3_manifest.py verify` recomputes all inputs to the same SHA. It binds 96 source files,
  8 MC sources, 5 build receipts, the QGC ELF, MC benchmark
  `20261025T051926Z-mc-trace-benchmark-e6000074`, 44 pinned assets, 4 runtime libraries,
  policy A and 3 qualification flights. Execution is not allowed without authorization.

## H. Gate A of candidate `c4b00bed` — FAILED (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-c4b00bedf07b.json` `07acc83e…72b4`, consumed
  (`p3-gate-A-c4b00bed….consumed.json`). Result `p3-gate-A-c4b00bed….result.json` `1b9edcfb…aac1`,
  `accepted: false`; it is retained and can never be retried or overwritten.
* **First failure:** `readiness:runtime_failed`, run `20261025T053026Z-p3-readiness-49d0ca55`,
  `Abort:mc_trace_build_differs_from_manifest`, 10 s after start. It aborted before PX4,
  Gazebo, ROS or QGC were launched; there were no owned processes and nothing was left
  behind. The six fixture runs never started.
* **Cause:** the invocation lacked `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1`.
  `selected_build()` therefore returned the baseline (by design it never auto-selects),
  which differs from the frozen instrumented build. The gate sequencer did not check this
  before consuming the authorization, and the README command omitted it.
* **Secondary:** the evaluator raised `KeyError: sensor_window_sim_s` on the aborted run and
  wrote no `p3-evaluation.json` (`evaluation_missing`). The gate still failed closed.
* **No sensor, timing, cleanup or Gazebo-shutdown result was measured.** The Gazebo risk is
  untested, not resolved.
* **Corrections (new candidate required):** `run_p3_gates.py` applies the runner's exact MC
  selection check before consuming an authorization. `p3_evaluate.py` records incomplete
  evidence as a `failed` evaluation and never leaves it missing. Both have regression tests.
  The README gate command now sets `GWM_PX4_MC_TRACE`. No threshold, P2 file or sensor
  package changed (package `f6605fb7…` and `p3-built.json` still valid).
* **Linux regressions on `966794b`:** P2 212, sensor 61, evaluators 27, runtime 56, MC 23,
  all passed.
* **New candidate:** Policy A (unchanged) re-approved by Neura-Shadow for commit `966794b`.
  Receipt `state/p3-policy-A-approval-966794b.json` `4787af49…084c`; the `0d38eec` receipt
  stays with the failed candidate. Manifest `state/p3-candidate-966794b.json`, canonical SHA-256
  **`8eaefbb63d3122f6e3373ae245de8c11af8aee029978d76ef111822529b2c11d`**; `verify` reproduces
  it, the explicit MC selection matches the frozen build, and no Gate A artifacts exist for it.

## I. Gate A of candidate `8eaefbb6` — FAILED (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-8eaefbb63d31.json` `31d3f0ea…c898`, consumed.
  Result `p3-gate-A-8eaefbb6….result.json` `d99a6a10…acd9`, `accepted: false`; retained, not retried.
* **First failure:** `readiness:runtime_status:Abort:owned_process_exited:source-probe`, run
  `20261025T053935Z-p3-readiness-113a881c`, 15.8 s. The evaluator recorded a `failed`
  evaluation (the section H fix works). The six fixtures never started. PX4 and the MC
  selection came up (`mc_pos_control running`). No sensor, timing or Gazebo-shutdown result
  was measured.
* **Cause 1 (probe):** the runner launched `p3_source_probe.py` with `python3 -I`, which
  ignores `PYTHONPATH`. The gz bindings exist only in the sourced Jazzy vendor tree
  (`/opt/ros/jazzy/opt/gz_{transport,msgs}_vendor/lib/python`), so the import failed with
  `ModuleNotFoundError: gz`. An import-only check in the runner environment reproduces this
  with `-I` and passes with `-s -P`.
* **Cause 2 (cleanup):** `shutdown()` began with an unguarded `wait_landed_disarmed()`, whose
  `pump()` re-raised the same `owned_process_exited`. No ordered stop ran (`shutdown_exception`),
  and every other owned process ended through the PID-namespace teardown, recorded as
  unreaped. This correctly fails cleanup, but the shutdown it was meant to exercise never
  happened.
* **Corrections:** the probe now launches with `-s -P` (still no user site or cwd), and
  `pump()` no longer aborts once shutdown has started. The landed-wait exception is recorded,
  and every later stop still runs. Three regression tests were added. Linux: P2 212,
  sensor 61, evaluators 27, runtime 59, MC 23. Windows runtime: 47 passed, 12 POSIX-only
  skipped. All other launch targets (gz, parameter_bridge, sensor adapter, QGC AppRun,
  MicroXRCEAgent) resolve in the runner environment. No threshold, P2 file or sensor package
  changed.
* **New candidate:** Policy A (unchanged) re-approved by Neura-Shadow for commit `2f74b2c`.
  Receipt `state/p3-policy-A-approval-2f74b2c.json` `e766373c…69a8`. Manifest
  `state/p3-candidate-2f74b2c.json`, canonical SHA-256
  **`8feaa0764c60ece888123b17af2971a5d52bb0e0aa5a70520ebb94c069eae11e`**; `verify` reproduces
  it, the MC selection matches the frozen build, and no Gate A artifacts exist for it.

## J. Gate A of candidate `8feaa076` — FAILED at readiness evaluation (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-8feaa0764c60.json` `0507a21f…6686`, consumed.
  Result `p3-gate-A-8feaa076….result.json` `7f9a257c…32b4`, `accepted: false`; retained.
* **Readiness run** `20261025T054510Z-p3-readiness-84dd7b19`: the runtime completed (`passed`),
  but the independent evaluation `failed`. The fixtures never started. Nothing was left behind.
* **Measured and passed:** Gazebo exited **0** within 0.14 s of pause + SIGINT (one sample;
  the historical −11 risk is not closed). QGC 0, every bridge, probe and adapter 0. Native
  commands matched policy A exactly (1 × 401, 34 × 512). RGB stream passed. MC causality was
  acquired. RTF 0.57 (minimum interval RTF 0.021).
* **Evaluator failures:**
  1. `cleanup:dds-agent:terminated_by_SIGINT` (exit −2). The pinned MicroXRCEAgent v2.4.3
     (`73622810`) installs no SIGINT/SIGTERM handler; its only clean exit is
     `AgentInstance::stop()`, which no signal reaches. Accepted P2 recorded −15 for it without
     judging it. Under P3's strict rule it can never pass: **operator decision**.
  2. `cleanup:px4:process_group_members_remained` [1288]. That PID is the owned Gazebo, which
     PX4 spawns into its group and which is stopped (exit 0) after PX4 by design. **Fixed:** a
     member that is itself an owned, reaped process is set aside, the raw list is kept, and any
     other leftover still fails.
  3. `depth:adapter_publication_boundary`: the observed `/rosout` and `/parameter_events` are
     created by every rclpy node. **Fixed:** exactly these two topics with their exact types
     are allowed; anything else still fails.
  4. `depth:state_association_incomplete`: 103 matched, 33 position and 14 attitude
     `ambiguous_equidistant` (two state samples exactly equally close to the image stamp,
     structural with quantized sim time). **Operator decision.**
  5. `renderer_unidentified`: Gazebo rendered with Mesa **llvmpipe** (software) although
     `/dev/dxg` and `libd3d12` exist. Archived P3 runs were llvmpipe too; hardware rendering
     has never been active here. The manifest requires `hardware`. **Operator decision.**
* In-memory re-evaluation of the retained run with the fixed evaluator (nothing written)
  leaves failures 1, 2 (raw record), 4 and 5. Linux: P2 212, sensor 61, evaluators 30,
  runtime 60, MC 23.

## K. Operator decisions applied (2026-10-25, Neura-Shadow)

* **DDS agent, patched (decision 1):** `validation/dds_agent_sigstop/microxrce_agent.cpp`
  replaces only the pinned 11-line `main()` (`c3bdc86a…431e`). SIGINT/SIGTERM are blocked
  before the agent thread starts and consumed with `sigwait()`, then the existing
  `AgentInstance::stop()` ends the agent and `main` returns 0. `build_p3_agent.py` compiles
  it with the pinned target's exact defines, includes, flags and link inputs against the
  **unchanged** `dds-install/lib/libmicroxrcedds_agent.so.2.4.3`. P2 keeps the stock binary.
  Binary `~/uav_autonomy/p3_agent/d6bad178d69ddbe5/bin/MicroXRCEAgent`
  `2705cf213f1be55ab5aace3836dde7710624b9e77df07e30ecb3fd17c25b88b0`; receipt
  `state/p3-dds-agent-built.json` `fc2ccb18…9e82`. Stop smoke (agent alone, private
  user/network namespace): SIGINT → **0**, SIGTERM → **0**, "server stopped". The runner
  verifies and launches this binary, and the manifest binds the receipt. The strict graceful
  rule is unchanged.
* **Association ties (decision 2):** an exact equidistant tie now takes the earlier (causal)
  sample. Both candidates have the same |delta|, which must still be ≤ `state_mismatch_s`
  (unchanged). Ties are flagged (`equidistant_tie_earlier`) and counted per window by the
  evaluator. Reset-epoch bracket rejection is unchanged. Sensor package rebuilt:
  `52024487…cdf40`, colcon 2/2, extrinsic matches; `state/p3-built.json` `2af49dea…79ef9`
  (previous receipt kept in `state/archive/p3-built-f6605fb7.json`).
* **Renderer (decision 3, GPU enabled):** an offscreen EGL probe (no simulator), run in the
  runner's environment and namespaces, gives llvmpipe by default and **D3D12 (NVIDIA GeForce
  RTX 4060 Laptop GPU)** with `GALLIUM_DRIVER=d3d12` and
  `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (Intel UHD is used if the adapter is not named).
  The runner sets both for owned processes. The `hardware` requirement and the software
  renderer rejection are unchanged; the actual Gazebo renderer is still verified from
  `ogre2.log` at runtime.
* **Linux:** P2 212, sensor 61, evaluators 30, runtime 62, MC 23, all passed. The manifest
  dependency check binds 6 receipts.
* **New candidate:** Policy A (unchanged) re-approved by Neura-Shadow for commit `1873c23`.
  Receipt `state/p3-policy-A-approval-1873c23.json` `927fd2f3…b112`. Manifest
  `state/p3-candidate-1873c23.json`, canonical SHA-256
  **`123498cd468ddfe0e5fb57854e6d8ba175bd8800b4ce46a1270427728bf9acb0`**; `verify` reproduces
  it, the MC selection matches, and no Gate A artifacts exist for it.

## L. Gate A of candidate `123498cd` — FAILED on Gazebo SIGSEGV (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-123498cd468d.json` `ff5b07ac…c4fa`, consumed.
  Result `p3-gate-A-123498cd….result.json` `31551f65…cc66`, `accepted: false`; retained, not retried.
* **Readiness run** `20261025T060854Z-p3-readiness-3a608df5`: the runtime `passed`; the
  independent evaluation has exactly one failure, **`cleanup:gazebo:terminated_by_SIGSEGV`**
  (exit −11, 0.43 s after the requested SIGINT, not forced, no group members left). The
  fixtures never started.
* **Everything else in readiness passed:**
  * Patched DDS agent exit **0** (3.34 s); PX4 0 (Gazebo correctly set aside as separately
    owned); QGC 0; bridges, probe and adapter 0.
  * Renderer identified as **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)**; RTF 0.82
    (minimum interval 0.0051).
  * Depth stream 0 failures; association 149/149 matched, with ties taken to the earlier
    sample (position 33, attitude 39). RGB passed.
  * Native commands passed policy A (1 × 401, 34 × 512). MC causality `verified`.
* **Crash context (offline, `~/.gz/sim/log/2026-10-25T14:08:59…/server_console.log`):** the
  world pause succeeded (`data: true`), the server received signal 2, `SensorsPrivate::Stop`
  ran, and the render thread stopped after 272 ms. The log then ends before `Shutting down
  gz-sim-server`. In the previous run (llvmpipe, exit 0) the same sequence took 51 ms and
  continued to `Shutting down gz-sim-server` and orderly worker exits. The crash therefore
  happens in server teardown after rendering stops. The archived −11 runs were llvmpipe, so
  the GPU backend is not established as the cause (one sample each). No core dump or
  backtrace exists.
* **Not changed:** SIGSEGV remains a cleanup failure. No check was weakened and no
  diagnostic runtime was started.

## M. Gazebo shutdown diagnostic (operator-authorized, outside the gates, 2026-10-25)

Gazebo only, with no PX4, QGC or flight, in a private user/net/mount/pid namespace. It used
PX4's own server command and `gz_env.sh` (`gz sim --verbose=4 -r -s default.sdf`, spawn
`x500_depth` through `/world/default/create`), subscribed to depth and RGB images, ran
20 s, paused (`data: true`), then sent SIGINT to the server. The `ruby` server process ran
under gdb, which passed SIGINT through and stopped on SIGSEGV. Two earlier harness attempts
(`20261025T061410Z`, `20261025T061422Z`) failed before any Gazebo process existed (shell
`set -u`, then gdb cannot load the Ruby `gz` script); they observed nothing.

| Variant | Run | Renderer | Outcome |
|---|---|---|---|
| D3D12 | `20261025T065412Z-gz-shutdown-diag-d3d12` | D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU) | **SIGSEGV**, thread 8 (LWP 43) |
| llvmpipe | `20261025T065459Z-gz-shutdown-diag-llvmpipe` | llvmpipe (LLVM 20.1.2) | **exited normally**: render thread stopped in 51 ms, then `Shutting down gz-sim-server` |

**Backtrace (D3D12):**

* Thread 1 (main): `SimulationRunner::Step` → `<signal handler>` →
  `gz::common::SignalHandlerPrivate::OnSignal` → `SensorsPrivate::Stop()` →
  `std::thread::join()` on LWP 43.
* Thread 8 (LWP 43, the sensors render thread, already out of its loop):
  `start_thread` → `__nptl_deallocate_tsd` → `0x7fffa1041b10 ??` → SIGSEGV.
* Threads 28–31 (GPU-stack workers) are parked in `pthread_cond_wait`, returning into the
  same unattributed region (`0x7fffa103d74f`, `0x7fffa138862b`). Thread 27 runs in the
  NVIDIA WSL user-mode driver `libnvwgf2umx.so` (still mapped at `0x7fff8f…–0x7fff91…`).

**Root cause:** when the render thread exits after `SensorsPrivate::Stop()`, glibc runs a
pthread thread-specific-data destructor registered on that thread by the D3D12 GPU stack.
The destructor's address is outside every loaded shared object at that moment, so executing
it faults. This happens in the WSL D3D12/NVIDIA driver's thread teardown, not in P3 or P2
code, and not in Gazebo's sensor logic (the render loop had already stopped). The exact
owning module of `0x7fffa1041b10` is not established: no `/proc` maps were captured, and the
region is either code from an unloaded module or anonymous executable memory. With llvmpipe
the same sequence exits 0, consistent with Gate A `8feaa076`. The archived llvmpipe −11
crashes (older runner, SIGTERM) were not reproduced and remain unexplained.

**Implication:** with the D3D12 hardware renderer on this host, Gazebo's own SIGINT
shutdown is expected to end in SIGSEGV, so the strict cleanup rule cannot pass. With
llvmpipe, cleanup can pass but the `hardware` renderer requirement fails. Resolving this
needs an operator decision; nothing was changed.

## N. Renderer decision: llvmpipe evidence assessment (2026-10-25)

The D3D12 crash evidence (section M) is preserved. **Exact module attribution of
`0x7fffa1041b10` is UNCONFIRMED.**

The frozen `p3-acceptance-v1` contract sets no RTF threshold (RTF is measured only). The
only admissible llvmpipe evidence for this implementation is readiness run
`20261025T054510Z-p3-readiness-84dd7b19` (Gate A `8feaa076`; its failures were the since-fixed
agent, group-member, boundary and association defects plus the renderer check):

| Frozen limit | llvmpipe measured | D3D12 measured (`3a608df5`) |
|---|---|---|
| delivered ≥ 25 Hz | 30.31 Hz sim (17.55 Hz wall) | 30.31 Hz sim (24.95 Hz wall) |
| source gap ≤ 0.12 s sim | 0.036 s | 0.036 s |
| processed ≥ 8 Hz | 10.0 Hz | 9.93 Hz |
| observation age ≤ 0.25 s sim | 0.072 s | 0.096 s |
| stall ≤ 1.0 s wall | 0.096 s | 0.080 s |
| recorded 1.0, lost 0 | 1.0, 0 of 455 | 1.0, 0 of 454 |
| P2 ground timing (`p2-timing-v1`) | 20.0 Hz callback, 0.080 s maximum entry gap | 20.0 Hz, 0.060 s |
| RTF (measured only) | 0.57 (minimum interval 0.021) | 0.82 (minimum interval 0.0051) |
| Gazebo CPU | 142 s | 38 s |

**Assessment: insufficient.** One 15 s readiness window passes every sensor and ground
timing limit, but:

1. No fixture geometry, scene-change or interruption result exists under llvmpipe for this
   implementation. Archived llvmpipe fixtures predate it and are not admissible.
2. Flight-phase P2 timing under the combined depth+RGB llvmpipe load is unproven. The
   archived smoke failed at 2.008 s against the unchanged 2 s `vehicle_land_detected` wall
   freshness limit; archived runs were llvmpipe, though that run's renderer was not recorded.
3. The evidence is a single sample with 3.7 × the Gazebo CPU of D3D12 and low wall
   delivery margin (17.55 Hz).

**Selected path:** exactly one separately authorized, ground-only llvmpipe performance
validation (readiness plus the seven fixtures, no flight, no gate credit). All sensor
specifications, timing limits and graceful-cleanup rules stay unchanged. If every
performance and cleanup check passes, a versioned renderer-contract revision allowing the
measured software renderer will be proposed for approval, not applied. Flight timing would
still be judged unchanged at Gate B/C. If any check fails, the recommendation is a native
Linux GPU host, not a relaxation.

## O. Validation V (llvmpipe, ground only, no credit) — PASSED (2026-10-25)

* Gate `V` added in `6e80422`: Gate A's ground plan with no D3D12 variables. The evaluator
  requires the renderer to be llvmpipe for V only. V never gives acceptance credit and can
  never be Gate B's predecessor. Linux: P2 212, sensor 61, evaluators 33, runtime 64, MC 23.
* Policy A (unchanged) re-approved by Neura-Shadow for `6e80422` (receipt `d7cf8acd…b2f`).
  Candidate `state/p3-candidate-6e80422.json`, canonical SHA-256
  `1e439b44caf24bdb4d1a5386046d84c3a9fb34da0a0bcf43d565b52461edc68f`. Authorization
  `p3-auth-V-1e439b44caf2.json` `838ef041…39b8`, consumed.
* Result `p3-gate-V-1e439b44….result.json`: `accepted: true`, `acceptance_credit: false`,
  **8/8 runs passed** (readiness and plane2, plane4, plane6, oblique, asymmetric,
  out_of_range, interruption). Every Gazebo exit was 0 and every process shut down
  gracefully. All frozen sensor, geometry, freshness, recorder, command-policy, MC and P2
  ground-timing checks passed. Measured margins are in the
  [renderer revision proposal](v3_sim_p3_renderer_contract_revision_proposal.md).
* **Proposed, not applied:** `p3-acceptance-v1.1` allowing exactly the measured llvmpipe
  renderer, with every other limit unchanged. Flight timing remains unproven and is judged
  unchanged at Gate B/C.

## P. Contract `p3-acceptance-v1.1` applied (2026-10-25)

* **Approved** by Neura-Shadow as proposed in `8df667d` (llvmpipe only, every other limit
  unchanged).
* **Applied:**
  * `p3_validation.yaml` now has `contract: p3-acceptance-v1.1` and
    `renderer_requirement: software:llvmpipe`; every numeric limit is unchanged and asserted
    by a test.
  * The manifest records the requirement from that file.
  * The runner passes no D3D12 variables for any gate (inherited ones cannot leak through
    the allowlist).
  * The evaluator fails `renderer_not_declared_llvmpipe` for any other renderer.
  * Gate V is unchanged and still never credits. Gazebo clean-shutdown, sensor, timing,
    recorder, policy and MC checks are untouched.
* **Linux:** P2 212, sensor 61, evaluators 34, runtime 64, MC 23. Windows (P3 suites):
  145 passed, 14 POSIX-only skipped.
* **v1.1 candidate:** Policy A (unchanged) re-approved by Neura-Shadow for `7b7d7d8`
  (receipt `ecec49ef…ab75`). Manifest `state/p3-candidate-7b7d7d8.json`, canonical SHA-256
  **`5e14e934f6b15f6d37c769bc43d26b843383ccbdb03aa4b76e16a5f1100a1dac`**,
  `renderer_requirement: software:llvmpipe`. `verify` reproduces it, the MC selection
  matches, and no Gate A artifacts exist for it. Gate A has not been run.

## Q. Gate A of candidate `5e14e934` (v1.1) — FAILED at the last fixture (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-5e14e934f6b1.json` `01231ff1…881d`, consumed.
  Result `p3-gate-A-5e14e934….result.json`, `accepted: false`; retained, not retried.
* **7 of 8 runs passed:** readiness `ac6ccdf6`, plane2, plane4, plane6, oblique, asymmetric
  and out_of_range, all on llvmpipe with every check passed and Gazebo exit 0.
* **First failure:** `fixture-interruption:cleanup:sensor-adapter:nonzero_exit_3`, run
  `20261025T084322Z-p3-interruption-ec038e97` (also `fixture:stream:adapter_result_not_passed`).
  The adapter recorded `RuntimeError: Unable to convert call argument '0' to Python object`
  from inside `rclpy.spin()` and exited 3. Its data path was complete and identical to the
  passing V interruption run: 472/472 frames recorded, 156 processed, recorder complete,
  source gap 0.036 s, maximum age 0.080 s. The bridge interruption exited gracefully (0).
* **Root cause, reproduced offline:** rclpy's default SIGINT handler shuts the context down
  while the adapter's 20 ms `poll` timer callback may still be executing. An adapter-only
  stop smoke (private namespace, no inputs, randomized SIGINT 0.8–7 s) on the Gate A build
  (`52024487…`) failed **4/40** with `RCLError … context is not valid … rcl_shutdown() was
  called` (exit 3). Same defect class, timing-dependent, which is why V's run passed.
* **Fix:** the adapter calls `rclpy.init(signal_handler_options=NO)` and installs its own
  SIGINT/SIGTERM handler that only sets a stop event. It spins with
  `SingleThreadedExecutor.spin_once(0.05)` until that event is set, then shuts down the
  executor, node and context in order, after spinning has ended. Package rebuilt as
  `3a5bd685…2a94` (colcon 2/2; `state/p3-built.json` `f34b6874…0f55`; previous receipt kept
  as `state/archive/p3-built-52024487.json`). Same smoke on the fix: **100/100** passed.
  Regression test added. No threshold, cleanup rule or evaluator check changed.
* **Linux:** P2 212, sensor 61, evaluators 34, runtime 65, MC 23.
* **New candidate:** Policy A (unchanged) re-approved by Neura-Shadow for `4bb06f4`
  (receipt `49c719a5…671f`). Manifest `state/p3-candidate-4bb06f4.json`, canonical SHA-256
  **`764fcfd7c39b8a75042fa0ca9de3721397dc214735eb030e4cc14c72ae8958d5`**
  (contract v1.1, llvmpipe). `verify` reproduces it, the MC selection matches, and no Gate A
  artifacts exist for it.

## R. Gate A of candidate `764fcfd7` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-764fcfd7c39b.json` `ee471580…3e78`, consumed.
  Result `state/p3-gate-A-764fcfd7….result.json` `53e49e31…a8e7`, `accepted: true`,
  `acceptance_credit: true`. Run 08:59:34–09:06:25 UTC; no leftover processes.
* **8/8 independent evaluations passed** on Mesa llvmpipe, with Gazebo exit 0 and sensor
  adapter exit 0 in every run:

| Run | source Hz (≥25) | max gap s (≤0.12) | processed Hz (≥8) | max age s (≤0.25) | max stall s wall (≤1.0) |
|---|---|---|---|---|---|
| readiness `93f53ed2` | 30.31 | 0.036 | 10.0 | 0.072 | 0.073 |
| plane2 `cf862a33` | 30.30 | 0.036 | 10.0 | 0.060 | 0.061 |
| plane4 `c2e7dae4` | 30.30 | 0.036 | 10.0 | 0.056 | 0.050 |
| plane6 `3395c9ad` | 30.30 | 0.036 | 10.0 | 0.056 | 0.055 |
| oblique `00288b55` | 30.30 | 0.036 | 10.0 | 0.056 | 0.050 |
| asymmetric `f6ab7594` | 30.30 | 0.036 | 10.0 | 0.052 | 0.056 |
| out_of_range `f8a109b4` | 30.30 | 0.036 | 10.0 | 0.060 | 0.058 |
| interruption `de425bd9` | 30.31 | 0.036 | 10.0 | 0.052 | 0.049 |

  Also passed in every applicable run: geometry accuracy, valid fraction, out-of-range,
  interruption detection, recorder completeness, state association, policy A native
  commands, MC causality, P2 ground timing and graceful cleanup of every owned process.
  Readiness RTF was 0.554 (wall delivery 17.5 Hz).
* **Not yet established:** flight-phase P2 timing on llvmpipe (Gate B, then Gate C), judged
  against unchanged limits.

## S. Gate B of candidate `764fcfd7` — FAILED on flight timing (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-764fcfd7c39b.json` `b8452179…df17`, consumed.
  Result `state/p3-gate-B-764fcfd7….result.json`, `accepted: false`; retained, never retried.
* **Run** `20261025T091152Z-p3-flight-c287b08f` (SITL, unchanged P2 mission, llvmpipe): arm,
  takeoff, reference lock, yaw handover, initial hover, east test, return, north test and
  return all completed. At sim 78.404 s in **YAW_TEST** the unchanged P2 controller entered
  RECOVERY with **`stale_state:vehicle_land_detected`** and the runtime ended with
  `Abort:controller_failed`. All owned processes, Gazebo included, shut down gracefully (exit 0).
* **Measured cause:** PX4 publishes `vehicle_land_detected` at a steady 1.000 s **sim**
  interval (97 receipts, maximum sim gap 1.004 s). The unchanged P2 limit is a 2 s **wall**
  receipt age. On llvmpipe the wall-clock receipt gap exceeded 1.8 s in 65 of 96 intervals
  and 2.0 s in 4, worst **2.658 s** (sim 77.644 → 78.644, local RTF ≈ 0.38, during the yaw
  test). This is the same invariant the archived smoke missed at 2.008 s. It is a
  real-time-factor limit of software rendering under the full depth+RGB+flight workload,
  not a controller, sensor or command defect.
* **Decision rule applied (operator instruction, section N):** llvmpipe fails a required
  performance gate, so the performance requirement is **not** relaxed. The 2 s limit, the
  P2 controller and every threshold stay unchanged.
* **Recommendation:** run Gates A–C on a **native Linux host with a hardware GPU**. On this
  WSL2 host the two renderers are each blocked: D3D12 gives RTF 0.82 but faults in Gazebo
  shutdown (section M, module unconfirmed), and llvmpipe shuts down cleanly but cannot hold
  the 2 s wall freshness in flight. The implementation, evaluators and P2 controller
  carry over unchanged, but a native host needs its own build receipts, a new frozen
  candidate and a fresh Gate A.

## T. D3D12 shutdown crash — cause confirmed, Gazebo-only repair verified (2026-10-25)

Operator-authorized Gazebo-only diagnostics; no PX4, QGC, flight or gate was run.
* **Cause (confirmed):** `libd3d12.so`'s destructor `dlclose`s and unmaps
  **`/usr/lib/wsl/lib/libd3d12core.so`** on the sensors render thread. At that thread's exit,
  glibc TSD cleanup calls into the unmapped `libd3d12core.so` text (faulting PC was at file
  offset `0x2edb10` of that module before SIGINT). That the entry is a TSD key destructor
  registered by `libd3d12core.so` is inferred, not read directly. This supersedes section M's
  "unconfirmed" attribution. Run `20261025T093020Z-gz-d3d12-trace-baseline`.
* **One repair variant:** `LD_PRELOAD=/usr/lib/wsl/lib/libd3d12core.so` for the Gazebo server
  only. The module is no longer unloaded; Gazebo reaches `Shutting down gz-sim-server` and
  **exits 0** with D3D12 (RTX 4060) active and no residual processes (one sample). Run
  `20261025T093153Z-gz-d3d12-trace-preload-d3d12core`.
* **Not applied:** the [v1.2 hardware-renderer profile proposal](v3_sim_p3_hardware_renderer_profile_proposal.md)
  needs operator approval (contract change and Gazebo-only `gz` shim, because PX4 starts
  Gazebo itself), then Policy A for the new commit, then a fresh Gate A. No environment
  update was made.

## U. Contract `p3-acceptance-v1.2` (D3D12 hardware profile) applied (2026-10-25)

* **Approved** by Neura-Shadow as proposed in `743bbf7`.
* **Applied:**
  * `p3_validation.yaml`: `contract: p3-acceptance-v1.2`, `renderer_requirement: hardware:d3d12`;
    every numeric limit unchanged (asserted by test).
  * New `configs/p3_d3d12_profile.json`: the D3D12 environment, the declared renderer identity,
    the `gz sim` preload `/usr/lib/wsl/lib/libd3d12core.so`, and the five pinned library
    hashes.
  * New `scripts/p3_gz_shim/gz` (LF, mode 755): first on the `PATH` of A/B/C owned processes,
    including PX4, which starts Gazebo. It adds `LD_PRELOAD` only for `gz sim`, so PX4, ROS,
    QGC and other gz subcommands run unchanged and inherited `LD_PRELOAD` never reaches owned
    processes.
  * The runner sets the profile environment for A/B/C; validation V stays llvmpipe with no shim.
  * The manifest re-hashes the pinned libraries and records the profile and shim hashes; any
    change blocks freezing or running.
  * The evaluator requires the exact D3D12 RTX 4060 renderer identity for A/B/C.
* **Unchanged:** P2 controller and mission, sensor rates and RGB workload, pins, freshness
  limits (including the 2 s `vehicle_land_detected` wall limit), MC causality, policy A,
  graceful cleanup, every earlier gate result and budget, and the llvmpipe candidate `764fcfd7`.
* **Verification:** Linux P2 212, sensor 61, evaluators 35, runtime 67, MC 23. Windows (P3
  suites) 148 passed, 15 POSIX-only skipped. Shellcheck is clean. The live WSL library
  hashes match the profile. The shim passes `gz sim --versions` through to Gazebo 8.15.0.
* Flight timing on D3D12 is still unproven; Gate B judges it unchanged.
* **v1.2 candidate:** Policy A (unchanged) re-approved by Neura-Shadow for `f76fbbb`
  (receipt `55709675…b598`). Manifest `state/p3-candidate-f76fbbb.json`, canonical SHA-256
  **`3302379cffc4a67e0df68d14f752cc350228df9fc8ee71d83e1610840a3c892f`**
  (`renderer_requirement: hardware:d3d12`; profile `2e9e9ba1…`, shim `5c3cf88b…`, 5 pinned
  libraries). `verify` reproduces it, the MC selection matches, and no Gate A artifacts
  exist for it. Gate A has not been run.

## V. Gate A of candidate `3302379c` (v1.2, D3D12) — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-3302379cffc4.json` `81d422f2…dbe8`, consumed.
  Result `state/p3-gate-A-3302379c….result.json` `9eba398a…4eca`, `accepted: true`,
  `acceptance_credit: true`. Run 09:48:03–09:55:15 UTC; no leftover processes.
* **8/8 independent evaluations passed.** Renderer `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`
  in every run. **Gazebo exit 0 in all 8 runs**, so the libd3d12core lifetime fix held for 8
  real shutdowns. Sensor adapter exit 0 in all 8.

| Run | source Hz (≥25) | max gap s (≤0.12) | processed Hz (≥8) | max age s (≤0.25) | max stall s wall (≤1.0) |
|---|---|---|---|---|---|
| readiness `adc2a21a` | 30.30 | 0.036 | 10.0 | 0.072 | 0.082 |
| plane2 `35eb96f4` | 30.30 | 0.036 | 10.0 | 0.064 | 0.054 |
| plane4 `13bec8e5` | 30.30 | 0.036 | 10.0 | 0.060 | 0.054 |
| plane6 `38cd08d1` | 30.30 | 0.036 | 10.0 | 0.060 | 0.049 |
| oblique `a413e48a` | 30.30 | 0.036 | 10.0 | 0.060 | 0.053 |
| asymmetric `e5acb438` | 30.30 | 0.036 | 10.0 | 0.056 | 0.049 |
| out_of_range `1e633769` | 30.30 | 0.036 | 10.0 | 0.060 | 0.051 |
| interruption `575b5bdb` | 30.31 | 0.036 | 10.0 | 0.056 | 0.058 |

  Geometry, valid fraction, out-of-range, interruption detection, recorder, association,
  policy A, MC causality, P2 ground timing and graceful cleanup also passed wherever they
  applied.
* **Flight-timing caution:** readiness RTF **0.647** (minimum interval 0.0092; wall delivery
  19.1 Hz), lower than the 0.82 single sample of section L. At that average, PX4's 1.000 s
  sim `vehicle_land_detected` interval is about 1.55 s wall against the unchanged 2 s limit.
  Gate B judges it unchanged.

## W. Gate B of candidate `3302379c` (v1.2, D3D12) — FAILED on flight timing (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-3302379cffc4.json` `77d97b69…51ae`, consumed.
  Result `state/p3-gate-B-3302379c….result.json`, `accepted: false`; retained, never retried.
* **Run** `20261025T100510Z-p3-flight-5a2b42b8` (SITL, unchanged P2 mission, renderer
  `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`): takeoff, reference lock, handover, hover, east
  test and return completed. At sim 64.748 s in **NORTH_TEST** the unchanged controller entered
  RECOVERY with **`stale_state:vehicle_land_detected`** (`Abort:controller_failed`).
* **Graceful cleanup held:** every owned process, Gazebo included, exited 0, so the v1.2
  libd3d12core fix held in flight as well.
* **Measured cause:** PX4's sim interval was 1.000 s (86 receipts). The wall receipt gap
  exceeded 1.8 s in 15 of 85 intervals and 2.0 s in 5, worst **2.725 s** (sim 63.916). That is
  better than llvmpipe (65 over 1.8 s, worst 2.658 s) but still violates the unchanged 2 s
  wall freshness limit. Real-time factor under the full depth+RGB+flight workload on this
  WSL2 host is insufficient on either renderer.
* **Decision rule (operator instruction, section N):** no relaxation. The 2 s limit, the P2
  controller and every threshold stay unchanged.
* **Recommendation:** run P3 Gates A–C on a **native Linux host with a hardware GPU**. On this
  host both renderers now shut down cleanly, but neither sustains the real-time factor the
  unchanged freshness limit needs in flight.

## X. Contract `p3-acceptance-v1.3` applied (2026-10-25)

* **Approved** by Neura-Shadow exactly as written in `e18ed8c`, with no RGB observation-age
  check added.
* **Applied:**
  * `p3_validation.yaml` adds the contract version, `rgb_consumer` {readiness required, fixture
    absent, flight absent_verified} and `rgb_absence_monitor` (marker `/sensor/IMX214/`, control
    `/depth_camera`, 0.5 s sampling, 1.0 s maximum gap). Every numeric limit is identical (test
    asserts the complete set). `p3_sensors.yaml` `6c3038fe…`, the bridge, the D3D12 profile
    `2e9e9ba1…`, the shim and `p2_control.yaml` `e50f824a…` are byte-identical.
  * Runner: the probe gets `--rgb` only at readiness; flights get `--rgb-absence-monitor`. It
    aborts if any bridge mapping carries IMX214. A background snapshotter records `gz topic -i`
    for every IMX214 topic plus the depth positive control, and the namespace process set, at
    the `PRESTREAM_SAFE_SETPOINTS` and `COMPLETE` transitions; shutdown waits for it first.
  * Probe: flight mode samples IMX214 subscriber counts every 0.5 s (`topic_list` +
    `topic_info`) into `source/imx214-subscribers.jsonl` and records `rgb_absence_monitor`,
    `monitor_rows` and its own `subscribed_topics`.
  * Evaluator: readiness keeps the unchanged `evaluate_rgb`. Fixtures add
    `fixture:rgb_consumer_present`. Flights replace RGB throughput with `evaluate_rgb_absence`
    (proposal §4.3, rules 1–6, fail closed) and are labelled `rgb_workload: depth_only_flight`.
* **Tests:** passing RGB-free flight evidence; failures for `rgb_subscribed: true`, an IMX214
  probe subscription, RGB rows in flight, start and end subscribers, a missing monitor, an
  intermediate subscriber, a monitor gap, an unowned process, a lost positive control, missing
  snapshots and a fixture with an RGB consumer; readiness without RGB fails; every numeric
  limit is identical; diagnostic gates stay non-crediting. Linux: P2 212, sensor 61,
  evaluators 49, runtime 78, MC 23. Windows (P3 suites): 170 passed, 18 POSIX-only skipped.
* Nothing was frozen or run. Full-flight timing with RGB absent remains unproven until Gate B.
* **v1.3 candidate:** preflight shows the sensor package receipt matching the current source
  (`3a5bd685…`), the DDS agent binary and the QGC ELF intact with the owner bound, the five WSL
  D3D12 libraries matching their pins, 793.8 GiB free and no simulation processes. Policy A
  (unchanged) was re-approved by Neura-Shadow for `e2a91a6` (receipt `f74cd05d…9e4e`).
  Manifest `state/p3-candidate-e2a91a6.json`, canonical SHA-256
  **`06b3fa54851bc27c843cb125b2358c18adbcb6c5d5717cfbc156cbad9cb29c37`** (file `6c5d027e…a18d`):
  `hardware:d3d12`, D3D12 profile `2e9e9ba1…`, MC build `7542919c…` (selection matches),
  benchmark `20261025T051926Z-mc-trace-benchmark-e6000074`, QGC ELF `82994283…`,
  `p3_validation.yaml` `83d27b26…`. `verify` reproduces it, and no authorization, consumption
  or result artifact exists for it under any gate. Gate A has not been run.

## Y. Gate A of v1.3 candidate `06b3fa54` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-06b3fa54851b.json` `4fc3004a…0bae`, consumed.
  Result `state/p3-gate-A-06b3fa54….result.json` `7e578357…cc14`, `accepted: true`,
  `acceptance_credit: true`. Run 11:26:19–11:33:28 UTC; no leftover processes.
* **Readiness** `20261025T112631Z-p3-readiness-ff8a3401` on `D3D12 (NVIDIA GeForce RTX 4060 Laptop
  GPU)`, with the mandatory RGB observer active (`rgb_subscribed: true`). The unchanged RGB
  checks passed: 1920×1080, source 30.31 Hz sim (≥ 25), maximum source gap 0.036 s (≤ 0.12),
  receipt stall 0.093 s wall (≤ 1.0), 454 frames, 0 late or unrendered. Depth: 30.31 Hz source,
  10.0 Hz processed, maximum age 0.072 s. Native commands passed policy A (1 × 401, 34 × 512);
  MC causality `verified`. Readiness RTF 0.603, with the RGB consumer active as v1.3 requires.
  This is readiness RGB evidence only.
* **Seven depth-only fixtures** (plane2, plane4, plane6, oblique, asymmetric, out_of_range,
  interruption) passed with `rgb_subscribed: false`. Source 30.30 Hz, maximum gap 0.036 s,
  processed 10.0 Hz, maximum age ≤ 0.064 s, maximum stall ≤ 0.064 s.
* Gazebo and the sensor adapter exited 0 in all 8 runs; every owned process shut down gracefully.
* Flight timing with RGB absent is not yet measured; Gate B judges it against unchanged limits.

## Z. Gate B of v1.3 candidate `06b3fa54` — FAILED in evaluation; flight timing passed (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-06b3fa54851b.json` `00cf4480…7064`, consumed.
  Result `p3-gate-B-06b3fa54….result.json`, `accepted: false`; retained, never retried.
* **Run** `20261025T123921Z-p3-flight-624dc8ae` (SITL, unchanged P2 mission, D3D12 RTX 4060,
  depth-only flight): runtime exit 0. The controller completed the whole mission (`COMPLETE`),
  P2 flight acceptance passed, MC causality was `verified`, and every owned process shut
  down gracefully.
* **Landing-state timing (depth-only workload):** 116 `vehicle_land_detected` intervals;
  wall p50 **1.000 s**, p95 **1.003 s**, maximum **1.012 s**; 0 over 1.8 s, 0 over 2.0 s; no
  stale-state event. Native RTF 0.997 (minimum interval 0.026).
* **RGB absence:** the probe had `rgb_subscribed: false` with no IMX214 subscription; 0 RGB
  metadata rows. The monitor recorded 205 rows (188 in the window span), maximum gap 0.50 s,
  zero IMX214 subscribers and the positive control present throughout. The endpoint snapshots
  are **missing** (see failure 1).
* **Evaluator failures (first: `depth:state_association_incomplete`):**
  1. `rgb_absence:snapshot_missing:window_start/window_end`, `snapshot_errors`:
     **implementation defect**. `RgbAbsenceSnapshotter` stored its run directory in an
     attribute named `run`, shadowing `threading.Thread.run`; the thread raised
     `TypeError: 'PosixPath' object is not callable` and never took a snapshot. Fixed offline
     (attribute renamed `run_dir`) with a regression test that executes the thread.
  2. `depth:state_association_incomplete`: 930 of 931 window observations matched. One frame
     (acquisition sim 22.180 s) was bracketed by a PX4 EKF attitude reset
     (`quat_reset_counter` 1 → 2 at sim 22.192 s, during TAKEOFF) and correctly rejected as
     `attitude:reset_epoch_bracket`. The rule that every observation must match makes any
     mid-flight EKF reset fatal. **Operator decision required**; no rule changed.
  3. `native_commands:vehicle_self_command_on_gcs_link`: six PX4-originated COMMAND_LONG frames
     (gimbal manager 1001 ×5, 1000 ×1) from 72/1 to 72/0 on the GCS link at 0.1–2.3 s after
     the P2 controller's `REQUEST_LAND` (phase `VERIFY_LANDED_AND_DISARMED`). They come from
     PX4's land mode, not from QGC or the controller, and were never observed before because
     no earlier P3 flight reached landing. **Operator decision required** on the native
     command policy; no policy changed.
* P3 remains INCOMPLETE; qualification 0/3. No further gate was started.

## AA. Contract `p3-acceptance-v1.4` applied (2026-10-25)

* **Approved** by Neura-Shadow exactly as written in `docs/v3_sim_p3_contract_v1_4_proposal.md`
  (commit `26bf5f7`).
* **Applied (offline only):**
  * `p3_validation.yaml` (`2ce6bf22…`): contract `p3-acceptance-v1.4`, revision text, and
    `state_association.reset_epoch_bracket` {allowed statuses `attitude:`/`position:reset_epoch_bracket`,
    `max_per_reset_event` 1, span limit key `state_mismatch_s`, corroboration
    `controller_ros_events_reset_counter_in_bracket`}. Every numeric limit is identical (test
    asserts the complete set); `p3_sensors.yaml`, the bridge, the D3D12 profile, the shim,
    `p2_control.yaml` and the P2 controller package are byte-identical.
  * `p3_native_command_policy.json` (`7a7be282…`): policy A gains `vehicle_landing_self_commands`
    (message 76, 72/1→72/0, phase `REQUEST_LAND`→`COMPLETE`, ≤4 per payload, rows a/b/c).
    `strict_zero` and the GCS allowance (1×401, 34×512, 75 s) are unchanged. **New policy A
    canonical SHA-256 `bf1ef9b1080a52eab1673ac2a1ac6e20d30cfa7fd85b703cae8433dc55f7b3b0`**; the
    previous approval `ea307bf5…` does not cover it.
  * Adapter (`association.py`): a `reset_epoch_bracket` refusal now also records `before`/`after`
    `{timestamp_us, epoch}`; the decision is unchanged. Sensor package rebuilt:
    **`c5d374137f6556df710c310f8eec994c778c297174b5407866c0d582f820c9b6`** (receipt
    `41bf4f87…`); the previous receipt (`3a5bd685…`, `f34b6874…`) is preserved in
    `state/archive/p3-built-3a5bd685.json`.
  * Evaluators: `p3_stream_eval.evaluate_reset_brackets` implements rule 1 (recorded bracket,
    single +1 reset, `ros-events.jsonl` corroboration, ≤1 per (state, reset), accounting);
    `p3_mavlink.evaluate_policy` implements rule 2 with the landing window that
    `p3_evaluate.native_commands` takes from `controller-result.json`.
* **Tests:** rule 1 — a corroborated attitude or position bracket passes and is counted; an
  uncorroborated bracket, a 0.051 s span, a +2 step, a multi-key change, two frames for one
  reset, `estimate_invalid` and other non-matched statuses fail; a bracket fails without the v1.4
  rule. Rule 2 — the observed landing sequence passes (6 frames); before `REQUEST_LAND`, after
  `COMPLETE`, a wrong payload, a wrong target component, a fifth copy, no landing window and
  `strict_zero` fail. Adapter bracket output; v1.4 config and policy identity. Linux: P2 212,
  sensor 62, evaluators 63, runtime 87, MC 23. Windows (P3 + MC suites): 216 passed, 19
  POSIX-only skipped. The dry candidate dependency check resolves all six receipts.
* **Read-only diagnostic (not credited, run directory verified unchanged):** applied to the
  retained v1.3 Gate B capture, rule 2 classifies the six landing frames as allowed (rows 4/1/1)
  with the GCS allowance unchanged; the controller ledger records the 22.184 s attitude reset as
  a single `quat_reset_counter` 1→2 step. That run's adapter predates the bracket output, so it
  is not re-evaluated under rule 1 and stays failed.
* **v1.4 candidate:** Policy A (`bf1ef9b1…`) was re-approved by Neura-Shadow for `5393103`
  (receipt `state/p3-policy-A-approval-5393103.json`, `2974a296…7a28`). Preflight: sensor package
  receipt matches the current source (`c5d37413…`), DDS agent binary and QGC ELF intact with the
  owner bound, the five WSL D3D12 libraries match their pins, 792.8 GiB free, no simulation
  processes. Manifest `state/p3-candidate-5393103.json`, canonical SHA-256
  **`6c00ecb09f2a2c4bc684e323d2342ab2d33d2d838d2f1196b2b6c065252b70d2`** (file `15f5e16d…a3aa`):
  contract v1.4, `hardware:d3d12`, D3D12 profile `2e9e9ba1…`, MC build `7542919c…` (the runtime
  selection matches the frozen build), benchmark `20261025T051926Z-mc-trace-benchmark-e6000074`,
  QGC ELF `82994283…`, `p3_validation.yaml` `2ce6bf22…`, policy file `7a7be282…`, sensor
  receipt canonical `77f4fa34…`. `verify` and `verify_current` reproduce it, execution without
  authorization is false, and no authorization, consumption, result or run artifact exists for
  it. Gate A has not been run.

## AB. Gate A of v1.4 candidate `6c00ecb0` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-6c00ecb09f2a.json` `ca22473f…2925`, consumed.
  Result `state/p3-gate-A-6c00ecb0….result.json` `3d96d4da…d27b`, `accepted: true`,
  `acceptance_credit: true`. Run 13:29:30–13:36:50 UTC; no leftover processes.
* **Readiness** `20261025T132943Z-p3-readiness-53edf801` on `D3D12 (NVIDIA GeForce RTX 4060 Laptop
  GPU)`, mandatory RGB observer active (`rgb_subscribed: true`). RGB (unchanged checks): source
  30.31 Hz sim, maximum gap 0.036 s, receipt stall 0.075 s wall, 454 frames, 0 late or
  unrendered. Depth: 30.31 Hz source, 10.0 Hz processed, maximum age 0.072 s; association 150/150
  `matched`, 0 reset-bracketed. Native commands passed policy A (1 × 401, 34 × 512; 0 landing
  self-commands, as expected on the ground); MC causality `verified`. Readiness RTF 0.631 with
  the RGB consumer active. This is readiness RGB evidence only.
* **Seven depth-only fixtures** (plane2, plane4, plane6, oblique, asymmetric, out_of_range,
  interruption) passed with `rgb_subscribed: false`: source 30.30–30.31 Hz, maximum gap 0.036 s,
  processed 10.0 Hz, maximum age ≤ 0.064 s, maximum receipt stall ≤ 0.069 s.
* Every owned process (Gazebo, sensor bridge/adapter, probe, clock bridge; at readiness also PX4,
  DDS agent, QGC and the controller) was classified graceful in all 8 runs.
* The v1.4 rules are not exercised on the ground (no EKF reset bracket, no landing). Gate B is
  the first v1.4 flight and needs a separate authorization.

## AC. Gate B of v1.4 candidate `6c00ecb0` — FAILED on raw depth recorder overflow (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-6c00ecb09f2a.json` `c6c000ed…c588`, consumed
  (marker `601334f4…8193`). Result `state/p3-gate-B-6c00ecb0….result.json` `b94f39b1…f121`,
  `accepted: false`, `first_failure: smoke:cleanup:sensor-adapter:nonzero_exit_3`, no automatic
  retry, next gate not started. Run `20261025T133925Z-p3-flight-b4436533`, 13:39:12–13:41:55 UTC;
  runtime exit 0, no leftover processes. Evaluation `08430a15…a781` failed.
* **Flight:** unchanged P2 mission to `COMPLETE` (sim 105.51 s), on D3D12 with RGB absent and
  verified (monitor and snapshots passed). P2 flight acceptance passed; `vehicle_land_detected`
  wall gaps p50 1.000 s, p95 1.006 s, maximum 1.076 s (limit 2 s); RTF 0.995; MC causality
  verified; native commands passed policy A (1 × 401, 34 × 512) with **v1.4 rule 2 exercised
  for the first time**: 6 landing self-commands allowed (rows a 4, b 1, c 1) inside
  `REQUEST_LAND`→`COMPLETE`. No EKF reset bracket occurred in the window (0 bracketed).
* **Primary failure (sensor acquisition):** the adapter's raw recorder (queue capacity 8)
  overflowed for **9 frames at sim 45.180–45.676 s** (`EAST_TEST`). One write took 0.209 s, and
  receipt-to-write latency reached 0.544 s (p50 0.013 s over the run, the same as in v1.3).
  `adapter-result.json`: `failed`, `raw_recorder_overflow`; recorder `overflow 9`, `max_queue 8`,
  `complete: false`. Invariant: `recorder_overflow_allowed = 0`, `recorded_fraction_required = 1.0`
  (measured 0.99681). ROS itself delivered all 3,301 frames (maximum receipt gap 0.139 s wall).
* **Secondary consequences:** the adapter exits 3 when its result failed, by design, so
  `cleanup:sensor-adapter:nonzero_exit_3` appears at shutdown, about 70 s later; it is listed
  first but is not the cause. `adapter_result_not_passed`, `raw_recorder_incomplete`,
  `source_to_ros_delivery_loss` and `ros_receipts_not_in_native_ledger` (2,815 of 2,824 window
  frames) all count the same 9 frames. `state_association_incomplete`: 4 in-window observations
  at sim 45.212–45.476 s were `pending_newer_state`, because the same stall delayed the PX4 state
  messages.
* **Context:** from monotonic 138811.3 to 138814.4 s, every owned process's CPU rate rose
  1.5–1.8× together (Gazebo 1.0→1.8, adapter 0.8→1.65, PX4, controller, bridge, QGC, probe) on
  24 cores, with no saturation and memory steady. This is consistent with a host-level slowdown;
  its cause is not identifiable from run evidence. The v1.3 flight was flat at the same mission
  point (recorder `max_queue` 1).
* **Proposal (not applied):** [v1.5](v3_sim_p3_recorder_capacity_v1_5_proposal.md) raises
  `recorder_capacity` 8 → 32, sized to the existing 1.0 s stall limit at 30.31 Hz, with
  `recorder_overflow_allowed` kept at 0. It would not by itself have passed this run: the four
  `pending_newer_state` observations remain an open operator decision.
* The failed result and all evidence are preserved. This candidate cannot run Gate B again
  (results are never overwritten). P3 remains INCOMPLETE; qualification 0/3.

## AD. Contract `p3-acceptance-v1.5` applied (2026-10-25)

* **Approved** by Neura-Shadow exactly as written in
  `docs/v3_sim_p3_recorder_capacity_v1_5_proposal.md` (commit `3593ae5`).
* **Applied (offline only):**
  * `p3_sensors.yaml` (`6c3038fe…` → `e02ae2a4…`): `recorder_capacity` 8 → 32, the only edit.
    Every other key is identical (a test pins the canonical hash of the file with the old value
    restored).
  * `p3_validation.yaml` (`2ce6bf22…` → `2bb85189…`): contract `p3-acceptance-v1.5` and the
    revision sentence. Every numeric limit is identical, including `recorder_overflow_allowed` 0,
    `recorded_fraction_required` 1.0 and `stall_wall_s` 1.0. The v1.4 rules are unchanged.
  * `p3_native_command_policy.json` is unchanged, so the policy A canonical SHA-256 stays
    `bf1ef9b1…`. Its approval is per candidate, so it still needs re-approval for the v1.5 commit.
  * The sensor package is unchanged and not rebuilt: the receipt `c5d37413…` matches the current
    source.
* **Tests:** v1.5 contract identity; `recorder_capacity == 32 == ⌈1.0 s × 30.31 Hz⌉ + 1`;
  `p3_sensors.yaml` is otherwise identical to v1.4; overflow allowance 0; with the writer blocked
  on one frame, exactly 32 frames queue and the 33rd overflows (sticky, recording incomplete).
  Linux: P2 212, sensor 63, evaluators 63, runtime 88, MC 23. Windows (P3 + MC suites): 218
  passed, 19 POSIX-only skipped. The dry candidate dependency check resolves all six receipts.
* **Still open (unchanged by v1.5):** a host slowdown like the one in section AC can still
  produce `pending_newer_state` associations that fail `state_association_incomplete`.
* **v1.5 candidate:** Policy A (`bf1ef9b1…`, content unchanged) was re-approved by Neura-Shadow
  for `a2d74bd` (receipt `state/p3-policy-A-approval-a2d74bd.json`, `be7dcfbb…c13d`). Preflight:
  the sensor package receipt matches the current source (`c5d37413…`), the DDS agent binary and
  QGC ELF are intact with the owner bound, the five WSL D3D12 libraries match their pins, 791.7 GiB
  is free and no simulation processes are running. Manifest `state/p3-candidate-a2d74bd.json`,
  canonical SHA-256 **`a0deb4d3f1f9624f0a38893303c6984eeae1e20094e186240fdd53f925e3d125`** (file
  `3bf7c7d7…b75b`): contract v1.5, `hardware:d3d12`, D3D12 profile `2e9e9ba1…`, MC build
  `7542919c…` (the runtime selection matches the frozen build), benchmark
  `20261025T051926Z-mc-trace-benchmark-e6000074`, QGC ELF `82994283…`, `p3_sensors.yaml`
  `e02ae2a4…`, `p3_validation.yaml` `2bb85189…`, policy file `7a7be282…`, sensor receipt canonical
  `77f4fa34…`. `verify` and `verify_current` reproduce it, execution without authorization is
  false, and no authorization, consumption, result or run artifact exists for it. Gate A has not
  been run.

## AE. Gate A of v1.5 candidate `a0deb4d3` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-a0deb4d3f1f9.json` `0c8fdaf4…2820`, consumed.
  Result `state/p3-gate-A-a0deb4d3….result.json` `c9a8d91e…5a19`, `accepted: true`,
  `acceptance_credit: true`. Run 15:20:41–15:27:54 UTC; no leftover processes.
* **Readiness** `20261025T152055Z-p3-readiness-3b929fe3` on `D3D12 (NVIDIA GeForce RTX 4060 Laptop
  GPU)`, mandatory RGB observer active (`rgb_subscribed: true`). RGB (unchanged checks): source
  30.31 Hz sim, maximum gap 0.036 s, receipt stall 0.089 s wall, 455 frames, 0 late or
  unrendered. Depth: 30.31 Hz source, 10.0 Hz processed, maximum age 0.076 s, association 150/150
  `matched`. Native commands passed policy A (1 × 401, 34 × 512); MC causality `verified`;
  readiness RTF 0.669 with the RGB consumer active. This is readiness RGB evidence only.
* **Seven depth-only fixtures** (plane2, plane4, plane6, oblique, asymmetric, out_of_range,
  interruption) passed with `rgb_subscribed: false`: source 30.30–30.31 Hz, maximum gap 0.036 s,
  processed 10.0 Hz, maximum age ≤ 0.064 s, maximum receipt stall ≤ 0.070 s.
* **Recorder (v1.5 capacity 32):** complete with 0 overflow in all 8 runs; maximum queue depth 2
  (readiness) or 1 (fixtures); maximum single write 0.030 s.
* Every owned process was classified graceful in all 8 runs.
* The v1.5 capacity and the v1.4 rules are not stressed on the ground. Gate B is the first v1.5
  flight and needs a separate authorization.

## AF. Gate B of v1.5 candidate `a0deb4d3` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-a0deb4d3f1f9.json` `c8e3bae6…61d1`, consumed
  (marker `3d8d6db1…6153`). Result `state/p3-gate-B-a0deb4d3….result.json` `570828bd…6bb2`,
  `accepted: true`, `acceptance_credit: true`, no first failure, no automatic retry, next gate not
  started. Run `20261025T152928Z-p3-flight-6e322ae9`, 15:29:16–15:31:57 UTC; runtime exit 0;
  evaluation `dd6872e8…` passed with no failures; no leftover processes.
* **Flight:** the unchanged P2 mission reached `COMPLETE` (sim 105.94 s) on `D3D12 (NVIDIA GeForce
  RTX 4060 Laptop GPU)`. P2 flight acceptance passed (landed and disarmed, commands
  [176, 400, 21], no failsafe). `vehicle_land_detected` wall gaps p50 1.000 s, p95 1.001 s,
  maximum 1.004 s (limit 2 s, 0 above 1.8 s); RTF 0.998. MC causality verified.
* **Depth (flight window):** source 30.30 Hz, maximum gap 0.036 s; 2,828 of 2,828 frames delivered
  and recorded (fraction 1.0); maximum receipt gap 0.085 s wall; 934 observations at 10.01 Hz,
  maximum age 0.092 s; association 934/934 `matched`, 0 reset-bracketed. Recorder (capacity 32)
  complete, 0 overflow, maximum queue 2, maximum write 0.046 s; adapter `passed`.
* **RGB:** absent and verified (`rgb_workload: depth_only_flight`; the absence monitor and the
  snapshots passed). This is not flight RGB throughput evidence.
* **Native commands:** policy A passed with 0 violations: 1 × 401, 34 × 512, and 6 v1.4 landing
  self-commands (rows a 4, b 1, c 1) inside `REQUEST_LAND`→`COMPLETE`.
* Every owned process (QGC, DDS agent, PX4, Gazebo, both bridges, probe, adapter, controller) was
  classified graceful. Maximum 1-minute load 4.17.
* **Not exercised in this flight:** no host slowdown occurred (recorder queue ≤ 2), so the v1.5
  headroom was not stressed; no EKF reset bracketed an observation, so v1.4 rule 1 was not
  exercised. Gate B is a smoke flight and does not count toward qualification. Gate C (three
  consecutive qualification flights) needs a separate authorization.

## AG. Gate C of v1.5 candidate `a0deb4d3` — FAILED at qualification-2 (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-C-a0deb4d3f1f9.json` `27ac2ac6…cafa`, consumed
  (marker `9c045653…1dc1`). Result `state/p3-gate-C-a0deb4d3….result.json` `601235d9…bf03`,
  `accepted: false`, `first_failure: qualification-2:p2_evaluator_error:ValueError:ROS/ULog
  estimator mismatch`, no automatic retry. Run 15:33:23–15:38:40 UTC; qualification-3 was not
  started; no leftover processes. Qualification stays 0/3, because the three flights must be
  consecutive within one Gate C.
* **qualification-1** `20261025T153335Z-p3-flight-5f89e686`: **passed** (evaluation `5715a8dd…`).
  `COMPLETE` at sim 105.40 s; P2 flight acceptance passed; `vehicle_land_detected` maximum gap
  1.095 s; RTF 0.996; MC verified (13,805 paired updates, yaw error 0.0); depth 2,810/2,810
  recorded, 928/928 matched; recorder overflow 0 (maximum queue 3); policy A with 6 landing
  self-commands; RGB absence verified; all processes graceful.
* **qualification-2** `20261025T153613Z-p3-flight-51e74376`: the flight reached `COMPLETE` (sim
  105.83 s), with `vehicle_land_detected` maximum gap 1.005 s, RTF 0.998, depth 2,821/2,821
  recorded, recorder overflow 0, policy A with 6 landing self-commands, RGB absence verified and
  all processes graceful. **v1.4 rule 1 was exercised for the first time:** one observation
  (acquisition 22.540 s) was `position:reset_epoch_bracket` across `heading_reset_counter` 1→2
  (bracket 22.536–22.560 s), corroborated by the controller ledger at 22.560 s and correctly
  accepted (930 matched + 1 bracketed).
* **Primary failure (evaluation, accepted P2 evaluator):** `collect_p2_evidence.verify` indexes
  ULog `vehicle_local_position` by `timestamp` alone (`u_by_t`) and requires |Δx, Δy, Δz| < 1e-6
  for every ROS sample with the same `timestamp`. PX4 logged **two different estimates with the
  same `timestamp` 15.680000 s** (ULog rows 1356/1357, `timestamp_sample` 15.644000 and
  15.680000 s, during TAKEOFF). The dictionary kept the second one. ROS had received the first,
  and it matches that ULog row bit-exactly (x, y, z and `timestamp_sample`); its difference from
  the second row is 3.95e-4 m. The evaluator raised `ROS/ULog estimator mismatch`, but the
  estimator data agree: this is a key collision in the evaluator. Invariant affected: P2
  recording integrity (exact ROS/ULog epoch match).
* **Secondary:** `mc_causal_yaw:failed` (`KeyError: 'handover_start_sim_s'`). `p3_evaluate`
  takes the yaw-ownership handover window from the P2 report, which was never produced, so the MC
  trace itself was not evaluated.
* **Frequency (read-only check of retained ULogs, not credited):** a duplicate-`timestamp` pair
  appears in 2 of the last 5 P3 flights: qualification-1 at 71.516 s (ROS did not receive that
  sample, so no collision) and qualification-2. It appears in neither v1.4/v1.5 Gate B flight nor
  in the v1.3 Gate B flight. ROS carries about 39 % of the ULog samples, so a collision occurs only
  when the bridged sample is the earlier of the pair.
* **Correction needed (not applied):** the defect is in the accepted P2 evaluator, so fixing it
  requires an operator decision. The smallest change keys the ULog lookup by (`timestamp`,
  `timestamp_sample`), or requires the ROS sample to match one of the ULog rows carrying its
  `timestamp`, keeping the 1e-6 tolerance and the 100-sample minimum. Drafted as
  [v1.6](v3_sim_p3_p2_evaluator_duplicate_timestamp_v1_6_proposal.md) (keyed form, not applied).
  The failed result and all evidence are preserved. This candidate cannot run Gate C again.

## AH. Contract `p3-acceptance-v1.6` applied (2026-10-25)

* **Approved** by Neura-Shadow exactly as written in
  `docs/v3_sim_p3_p2_evaluator_duplicate_timestamp_v1_6_proposal.md` (commit `b9ccf70`).
* **Applied (offline only):**
  * `validation/collect_p2_evidence.py` (`a6acfd2d…` → `3c794001…`): only the ROS/ULog position
    comparison in `verify()` changes. ULog rows are keyed by (`timestamp`, `timestamp_sample`); a
    key carrying different x/y/z fails closed (`Ambiguous ULog position sample`); the 1e-6 m
    tolerance and the 100-pair minimum are unchanged; `estimator_correlation` gains
    `duplicate_ulog_timestamps`.
  * `configs/p3_validation.yaml` (`2bb85189…` → `2d8510f7…`): contract `p3-acceptance-v1.6` and the
    revision sentence. Every numeric limit is identical. The sensor config, command policy
    (`bf1ef9b1…`), sensor package and P2 controller are unchanged.
* **Offline P2 confirmation (read-only; every run directory verified unchanged):** the accepted
  P2-R3 nominal smoke `20260912T034917Z-p2-flight-c74f35ba` and all 20 trials of
  `20260912T035329Z-p2-repeat-84bb288d` were evaluated in memory with the accepted evaluator
  (`a6acfd2d…`, from git) and the v1.6 evaluator. **21/21 reports are identical** apart from the new
  count, which is 0 in every run. Every `estimator_correlation` equals the stored acceptance
  report (4,915–4,950 pairs, maximum difference 0), and every flight acceptance still passes. The
  P2 acceptance record is unchanged. Its batch `frozen_inputs` still pin `a6acfd2d…`, so that
  frozen P2 batch cannot be re-run with the new file; no P2 re-run is planned.
* **Retained v1.5 Gate C runs (read-only, not credited):** qualification-2 — the accepted evaluator
  stops at `ROS/ULog estimator mismatch`; v1.6 completes the full P2 evaluation (5,011 pairs,
  maximum difference 0, 1 duplicate timestamp, flight acceptance passed, yaw ownership passed with
  handover at 28.64 s, so the MC window would be available). qualification-1 — identical reports
  (1 duplicate timestamp counted). The failed Gate C result stays failed.
* **Tests:** a duplicate timestamp with ROS carrying the first or the second row passes; ROS
  matching neither row fails; unpaired `timestamp_sample` values count against the 100-pair
  minimum; an ambiguous key fails closed; without duplicates the correlation is unchanged;
  1.1e-6 m still fails; v1.6 contract identity. Linux: P2 suites 219, sensor 63, evaluators 63,
  runtime 89, MC 23. Windows (P3, MC and reference suites): 272 passed, 19 POSIX-only skipped. The
  dry candidate dependency check resolves all six receipts.
* **v1.6 candidate:** Policy A (`bf1ef9b1…`, content unchanged) was re-approved by Neura-Shadow
  for `087722b` (receipt `state/p3-policy-A-approval-087722b.json`, `75aa0d63…5265`). Preflight:
  the sensor package receipt matches the current source (`c5d37413…`), the DDS agent binary and
  QGC ELF are intact with the owner bound, the five WSL D3D12 libraries match their pins, 788.8 GiB
  is free and no simulation processes are running. Manifest `state/p3-candidate-087722b.json`,
  canonical SHA-256 **`1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c`** (file
  `a38ce477…d3d9`): contract v1.6, `hardware:d3d12`, MC build `7542919c…` (the runtime selection
  matches the frozen build), benchmark `20261025T051926Z-mc-trace-benchmark-e6000074`, QGC ELF
  `82994283…`, `collect_p2_evidence.py` `3c794001…`, `p3_validation.yaml` `2d8510f7…`,
  `p3_sensors.yaml` `e02ae2a4…`, policy file `7a7be282…`, sensor receipt canonical `77f4fa34…`.
  `verify` and `verify_current` reproduce it, execution without authorization is false, and no
  authorization, consumption, result or run artifact exists for it. Gate A has not been run.

## AI. Gate A of v1.6 candidate `1e205fec` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-A-1e205feca482.json` `271a0761…369e`, consumed.
  Result `state/p3-gate-A-1e205fec….result.json` `5d67e8f9…e99d`, `accepted: true`,
  `acceptance_credit: true`. Run 16:00:23–16:07:35 UTC; no leftover processes.
* **Readiness** `20261025T160037Z-p3-readiness-4c6b725c` on `D3D12 (NVIDIA GeForce RTX 4060 Laptop
  GPU)`, mandatory RGB observer active (`rgb_subscribed: true`). RGB (unchanged checks): source
  30.31 Hz sim, maximum gap 0.036 s, receipt stall 0.077 s wall, 454 frames, 0 late or
  unrendered. Depth: 30.31 Hz source, 10.0 Hz processed, maximum age 0.076 s, association 150/150
  `matched`. Native commands passed policy A (1 × 401, 34 × 512); MC causality `verified`;
  readiness RTF 0.803 with the RGB consumer active. This is readiness RGB evidence only.
* **Seven depth-only fixtures** (plane2, plane4, plane6, oblique, asymmetric, out_of_range,
  interruption) passed with `rgb_subscribed: false`: source 30.30–30.31 Hz, maximum gap 0.036 s,
  processed 10.0 Hz, maximum age ≤ 0.060 s, maximum receipt stall ≤ 0.071 s.
* **Recorder (capacity 32):** complete with 0 overflow in all 8 runs, maximum queue 1, maximum
  write 0.014 s. Every owned process was classified graceful in all 8 runs.
* The v1.6 position pairing applies to flights only (the P2 evaluator's readiness path returns
  before that comparison), so Gate B is the first run that exercises it. Gate B needs a separate
  authorization.

## AJ. Gate B of v1.6 candidate `1e205fec` — **ACCEPTED** (2026-10-25)

* **Authorization:** Neura-Shadow, `state/p3-auth-B-1e205feca482.json` `a74c875b…a638`, consumed
  (marker `81c00bda…d0bf`). Result `state/p3-gate-B-1e205fec….result.json` `52cbd099…1e2a`,
  `accepted: true`, `acceptance_credit: true`, no first failure, no automatic retry, next gate not
  started. Run `20261025T160901Z-p3-flight-a8b0f181`, 16:08:46–16:11:30 UTC; runtime exit 0;
  evaluation `58aeb104…` passed with no failures; no leftover processes.
* **Flight:** the unchanged P2 mission reached `COMPLETE` (sim 105.56 s) on `D3D12 (NVIDIA GeForce
  RTX 4060 Laptop GPU)`. P2 flight acceptance passed (landed and disarmed, commands
  [176, 400, 21], no failsafe); yaw ownership passed. **v1.6 pairing exercised:** 5,017 ROS/ULog
  position pairs, maximum difference 0, 0 duplicate timestamps. `vehicle_land_detected` wall gaps
  p50 1.000 s, p95 1.001 s, maximum 1.008 s (0 above 1.8 s); RTF 0.998; MC causality verified.
* **Depth:** source 30.30 Hz, maximum gap 0.036 s; 2,813/2,813 frames delivered and recorded;
  maximum receipt gap 0.057 s wall; 928 observations at 10.00 Hz, maximum age 0.064 s;
  association 928/928 `matched`, 0 reset-bracketed. Recorder complete, 0 overflow, maximum queue
  1. RGB absent and verified (`depth_only_flight`; not RGB throughput evidence).
* **Native commands:** policy A passed with 0 violations: 1 × 401, 34 × 512, and 6 v1.4 landing
  self-commands (rows a 4, b 1, c 1).
* Every owned process was classified graceful. Maximum 1-minute load 7.69 (higher than earlier
  flights, 4.2–4.5), with no effect on any limit (recorder queue ≤ 1, maximum receipt gap 0.057 s).
* Not exercised: no EKF reset bracket, no duplicate ULog timestamp, no host slowdown. Gate B is a
  smoke flight and does not count toward qualification. Gate C (three consecutive qualification
  flights) needs a separate authorization.

Final status: **P3 INCOMPLETE — v1.6 candidate 1e205fec: Gate A and Gate B accepted; Gate C not run — Qualification 0/3**
