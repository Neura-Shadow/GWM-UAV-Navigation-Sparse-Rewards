# v3-SIM P6 — failure injection, fault-tolerant control and safety validation: plan

**Status: PLANNING ONLY (2026-11-06).** Nothing in P6 is implemented, frozen, launched or accepted.
The [P6 contract](v3_sim_p6_contract.md) is a draft. The operator answered decisions D1–D10 on
2026-11-06, all as recommended (contract §10 decision record). **M0 (the read-only audit) is
complete**; its results are in the [M0 audit](v3_sim_p6_m0_audit.md). It raised decisions D1a and
D4a and rule C11; the operator approved all three as recommended on 2026-11-06. The contract itself
is not yet approved. The fault
list is in the [fault matrix](v3_sim_p6_fault_matrix.md), and the preservation of P3–P5 evidence
in the [evidence backup plan](v3_sim_p3_p5_evidence_backup_plan.md). The milestones in §7 are
proposals, not permission to implement or execute them.

**P6 question.** Can the accepted P5 stack detect, contain and safely end or recover from
realistic software, sensor, communication, inference and control failures without violating the
established flight-safety constraints? P6 does not claim formal fault tolerance or safety
certification. Its results hold for PX4 SITL with the declared faults only.

## 0. Baseline verified on 2026-11-06

* **Git:** HEAD `1f06f39e1126a3d9538377347ae06bf13e14536c` on `v3/gwm-uav-c2-agentops-planning`;
  local equals `origin`. The only untracked path is `.codegraph/`.
* **Manifests**, re-verified the same day with `p*_manifest.py verify` (read-only):

  | Phase | Manifest file (`~/uav_autonomy/state/`) | SHA-256 |
  |---|---|---|
  | P3 | `p3-candidate-087722b.json` | `1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c` |
  | P4 | `p4-candidate-50d5002.json` | `c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b` |
  | P5 | `p5-candidate-40a109b.json` | `2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632` |

  The full P5 SHA was read from the manifest file and its `verify` output, not inferred.
* **Accepted gate results:** P3 A/B/C, P4 A/B/S/C and P5 A/B/S/C, all `accepted: true` for the
  manifests above. The historical P5 failures stay on record: `baef37a0…` Gate A and `db4c4779…`
  Gate C (one 10.84 ms inference tick).

## 1. Verified integration interfaces and immutable boundaries

### 1.1 Manifest scopes (read from the manifest scripts)

| Phase | Hashed folders | Also bound |
|---|---|---|
| P3 | `simulation/px4_gazebo/{scripts,validation,configs}`, `ros2_ws/src/{gwm_px4_control,gwm_sensor_adapter}` | `docs/evidence/v3_sim_p2_summary.json`; PX4/QGC/MC runtime receipts, pinned PX4 assets, Policy A |
| P4 | `simulation/px4_gazebo_p4/{configs,scripts,validation,ros2_ws/src/gwm_obstacle_avoidance}` | P3 manifest; `state/p4-built.json` (`c4bd5a55…`) |
| P5 | `simulation/px4_gazebo_p5/{configs,models,training,scripts,validation,ros2_ws/src/gwm_policy_control}` | P3 and P4 manifests; `state/p5-built.json` (`e7a59f35…`); model `ee006a80…`; controller NumPy 1.26.4 |

**Outside every manifest:** `simulation/px4_gazebo_p6/` (proposed), `tests/`, `diagnostics/`, `docs/`
(except the P2 summary) and the READMEs. P6 code placed there cannot invalidate P3, P4 or P5. P6
must never write `state/p3-*`, `state/p4-*` or `state/p5-*`; it gets its own `state/p6-*` files.

### 1.2 Runtime pipeline (accepted, unchanged)

```text
Gazebo depth camera ──gz──> sensor-bridge ──/gwm/sensors/front_depth/image_raw──┬─> sensor-adapter (P3) ──health──┐
       │                                                                        └─> p4_perception (P4) ──/gwm/p4/local_map──┤
       └──native gz headers──> P3 source probe (evaluator witness)                                                         │
PX4 SITL ──uXRCE-DDS──> /px4_71/fmu/out/* ──────────────────────────────────────────────> p4_perception, p5_control <────┘
p5_control (P5Mission → PolicySupervisor → plan_with_ranker → PolicyRanker → FrozenPolicy)
       └──sole publisher──> /px4_71/fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command} ──> PX4
QGC monitor (Policy A) ──MAVLink──> PX4          p4_truth_observer ──> truth poses and contacts (evaluator only)
```

* **Process model.** Every process runs in the owned PID/network namespace launched by the P3
  runner (`p3_namespace_owner.py`). The P4 and P5 runners subclass it. The P5 scheduling diagnostic
  (`diagnostics/p5_sched_diag`) proved a third subclass layer works without changing frozen files:
  `DiagRunner(P5Runner)` with a byte-identical namespace-owner copy.
* **The controller is single-threaded.** `rclpy.spin_once` runs one 20 Hz timer tick:
  `mission.tick` (P2 checks → supervisor → policy), then `send` (dispatch guard → publish).
* **Remapping (M0 Finding A).** No accepted node accepts command-line remaps:
  * the controllers call `rclpy.init(args=[])`;
  * the adapter and `p4_perception` reject `--ros-args` in their strict `argparse` before ROS
    sees it.

  Topics come only from configuration and from the adapter's and perception's `--px4-prefix`
  argument. §4.2 is adjusted accordingly (D1a, approved 2026-11-06).

### 1.3 Existing fault handling, verified from source

**P2 (per tick, in this order, in `Mission.tick` and `P4Mission._p4_tick`):**

1. wall deadline 420 s; clock stall 1.0 s wall; simulation deadline 240 s; phase deadline;
2. ACK transaction deadline (3.0 s sim / 5.0 s wall) and any non-accepted ACK;
3. the control-graph check;
4. `StateCache.validate`:
   * position and attitude freshness 0.25 s sim, flags 1.5 s sim, every receipt 2.0 s wall;
   * clock epoch tolerance 0.1 s;
   * `xy/z/v_xy/v_z_valid`, tilt and yaw alignment, the estimator failsafe flags,
     `vehicle_status.failsafe`, pre-flight checks and vehicle identity;
   * estimator reference resets, through the v3 reference policy (`max_events` 1);
5. observation gap 0.2 s sim; clock backwards; geofence; `offboard_armed_state_lost` (armed, nav 14,
   airborne).

Any of these raises, and `node.fail` then enters P2 `RECOVERY`:

* **One LAND** is sent only if a validated sample shows an armed vehicle in Offboard (nav 14), no
  command is pending and no LAND has been sent.
* **Otherwise the controller only observes.** It stops publishing setpoints, and PX4's own failsafe
  takes over.
* **Never restarted.** There is no automatic restart. A controller started against an armed or
  airborne vehicle refuses (`restart_reject_armed`, `restart_reject_airborne`).

**Dispatch and graph:**

* **Dispatch guard.** It runs from callback entry to publication with a 0.050 s wall budget and
  aborts on a breach.
* **Graph check.** It runs every 1.0 s wall; `graph_fresh_wall_s` is 1.5 s.
* **Exactly one PX4-input owner.** More than one publisher on a PX4 input topic raises
  `competing_control_publisher`.
* **Gap:** the controller does not graph-check its own input topics (`/gwm/p4/local_map`, adapter
  health).

**P3 depth (`gwm_sensor_adapter`):**

* **Rejected frames:** wrong dimensions, encoding, endianness, step or payload length; a missing,
  duplicate or backwards acquisition stamp; an unexpected `frame_id`.
* **Pixel classes:** NaN is `invalid_nan`; below 0.2 m is `too_close`; +inf or ≥ 19.1 m is
  `no_return`.
* **Health:** `fresh`, `stale` (`receipt_stall_wall`, `source_age_sim`), `unavailable` or
  `calibration_unavailable`.
* **Association:** the pose must be matched; a `reset_epoch_bracket` is unusable.

**P4 perception (`P4Processor`):**

* **Usability:** only associated (`matched`) frames update the cumulative grid.
* **Map semantics:** a ray with NaN in the flight band is unknown, and unknown is never free. A
  band of `no_return` is **free to 4.0 m** (`free_cap_m`). Cell states only rise
  (UNKNOWN < FREE < OCCUPIED). The grid is anchored on the first matched frame and **never
  cleared**.

**P4 supervisor (in `PolicySupervisor` order):**

1. ABORT is absorbing; the transit deadline is 150 s sim.
2. **Staleness, with yaw frozen:** no usable snapshot; snapshot older than 0.25 s; adapter health
   not fresh or older than 2.0 s; calibration mismatch; 3 consecutive unusable frames; an
   estimator reset since the last usable frame. Any of these gives HOVER, and ABORT after 10 s
   stale.
3. Clearance below `R_body` + 0.10 m gives ABORT.
4. Planning, then the transition table, then the 20 s hover timeout.

A supervisor ABORT lands through revision 5: exactly one LAND, then `COMPLETE` within 45 s wall.

**P5 policy:**

* **Start-up refusal:** checkpoint missing, SHA mismatch, unreadable, wrong architecture, schema,
  untrained, or non-finite or misshapen weights.
* **Per tick:** checkpoint missing, mask mismatch, input schema, non-finite input, output shape,
  empty mask, non-finite allowed score, selection outside the mask, any exception, or wall time
  above 0.020 s gives **HOLD** that tick. The **3rd consecutive** failure ABORTs with
  `policy_unavailable`. A success resets the counter.
* **No fallback** exists (`fallback: none` is pinned).
* **The 0.020 s timeout is measured after the call returns.** A call that never returns is not
  preempted.

**PX4 (parameters read from an accepted P5 Gate C ULog, `13_24_56.ulg`; their semantics must be
confirmed against the pinned PX4 source in M0):**

* **Offboard loss:** `COM_OF_LOSS_T` 1.0 s, `COM_OBL_RC_ACT` 0, `COM_FAIL_ACT_T` 5.0 s.
* **RC:** `COM_RC_IN_MODE` 1, `NAV_RCL_ACT` 2.
* **Data link:** `COM_DL_LOSS_T` 10 s, `NAV_DLL_ACT` 2.
* **Return:** `RTL_RETURN_ALT` 30 m. The PX4 geofence is disabled (`GF_MAX_HOR_DIST`,
  `GF_MAX_VER_DIST` 0).
* **Landing:** `MPC_LAND_SPEED` 0.7 m/s, `COM_DISARM_LAND` 2.0 s.
* **Failure injection:** `SYS_FAILURE_EN` 1.

### 1.4 What P4/P5 acceptance already covers

| Fault | Covered by | Tier |
|---|---|---|
| Depth bridge stop at 2.0 m in NORMAL | P4 and P5 Gate S S6 (§9.1 F1–F4 window) | SITL |
| Adapter stop, bridge stop | P4 Gate A fault fixtures | ground |
| No progress candidate (wall) | P4 and P5 S5 (`hover_timeout`) | SITL |
| Estimator reset, stale pose | P4/P5 S7 offline replay of retained P3 frames | offline |
| Policy failures (missing, hash, non-finite, timeout, shape, selection) | P5 Gate A item 5, one reproduced TRANSIT tick | offline |
| Checkpoint mismatch at start | P5 Gate A item 5, live refusal | ground |

The P4 contract §8 Gate A(4) also names "stale clock" and "calibration mismatch" fault fixtures. The
accepted P4 Gate A ran only `adapter_stop` and `bridge_stop` (runner `FAULTS`). **M0 Finding B:**
the other two were covered only offline, by unit tests with synthetic inputs
(`tests/test_px4_gazebo_p4_planner.py`, `tests/test_px4_gazebo_p2_contract.py`). They were never
exercised live.

## 2. Highest-priority uncovered failure modes

These are ranked by safety consequence. They come from reading the source; each still needs its
offline confirmation in M3 before any SITL use.

1. **The map is not invalidated after an estimator reset (F2.04).**
   * The supervisor HOVERs on a reset only until the next fresh matched frame. The cumulative grid
     built in the pre-reset frame is then reused unchanged.
   * A real XY reset would shift every old cell. Stale occupancy could be read in the wrong place,
     including old FREE cells over a now-unobserved obstacle.
   * P4 S7 replay tested frame rejection, not map reuse.
2. **Corrupted +inf or far depth becomes free space (F1.09).**
   * The no-return rule marks a band of +inf or ≥ 19.1 m values FREE to 4.0 m.
   * Corruption over a never-observed obstacle would mark its cells FREE until a true hit raises
     them, because states only rise.
3. **Frozen frames go undetected (F1.05).**
   * Identical image content with advancing stamps passes every P3 check, and is projected with
     the new pose.
4. **Internal topics have no integrity check (F4.08).**
   * A second publisher on `/gwm/p4/local_map` (for example a fabricated all-FREE usable
     snapshot) is accepted; the controller graph-checks only PX4 input topics.
   * Ground and offline only; never flown.
5. **Snapshot reordering (F4.03).** `P4Mission.on_snapshot` keeps the newest-received, not the
   newest-acquired, snapshot. An older grid inside the 0.25 s age limit can briefly replace a newer
   one, in which a just-observed OCCUPIED cell is still FREE.
6. **Controller-loss paths end in PX4's failsafe (F2.01, F3.10, F5.01, F5.09, F4.12).**
   * **Why:** P2 `RECOVERY` sends no LAND without a validated Offboard sample. A hung tick, a
     stopped or killed controller, or a stopped DDS agent leaves PX4's offboard-loss failsafe as
     the only safety path.
   * **Established in M0 (Finding D)**, from the pinned source and the one historical failsafe
     ULog: PX4 enters **Return** immediately, with no 5 s Hold.
   * **The Return altitude** is `max(d, 4 m)` for d up to 10 m from home. It flies straight home,
     lands and disarms.
   * **Consequence:** it lands, but always above the P4 3.0 m ceiling, and it can cross 4 m
     obstacle tops with less than 0.20 m vertical gap. Hence decision D4a.
7. **GCS link loss triggers Return to 30 m (F5.12).**
   * With `NAV_DLL_ACT` 2 and `RTL_RETURN_ALT` 30 m, QGC loss would command Return after 10 s,
     climbing far above the P4 3.0 m geofence ceiling.
   * Excluded from flight until analysed (D7).
8. **Alternating policy failures never escalate (F7.02).** Only *consecutive* failures ABORT. A
   fail/succeed pattern HOLDs every other tick and ends only by goal or the 150 s transit deadline.
9. **A perception restart re-anchors the grid (F4.10).** A restarted `p4_perception` anchors a new
   grid at the in-flight pose with the same shape. The controller resets the traversed mask only
   on a shape change.
10. **Bad-but-valid policy behaviour (F3.13).** Nothing detects it as a failure, so only the shield
    contains it. P5 evidence shows the shield with a good policy, not with an adversarial one.

## 3. Fault taxonomy summary

The full matrix has 86 fault IDs in 8 categories (F1 depth 12, F2 estimator 10, F3 policy 14, F4
ROS/DDS 13, F5 PX4 12, F6 timing 10, F7 supervisor 9, F8 combined 6).

**Classes:**
* **R:** required. The behaviour is implemented by the accepted code, so a deterministic pass rule
  exists. Gated.
* **C:** characterization. The accepted code has no detector, or the outcome depends on undecided
  PX4 behaviour. Every safety check is evaluated and reported, but there is no pass/fail gate.
* **D:** deferred. It needs a decision (PX4-native injection, a link-loss Return, or a command
  outside Policy A).
* **Covered:** reused P4/P5 evidence, no new run.

**Rule (decision D3).** P6 never requires a detection the accepted P5 code does not have. A gap
found in characterization becomes a documented limitation. Its remedy would be a new P5 candidate,
outside P6.

## 4. Fault-injection architecture (proposal)

### 4.1 Composition, not modification

* **Runner.** `P6Runner(P5Runner)`, following the proven `DiagRunner` pattern. It overrides only
  run preparation, the process commands it needs, injector launch and the fault ledger.
* **No frozen file is copied, edited or rebuilt.** The P5 install `p5_ws/e7a59f35…` is used as is.
* **Every injector has a pass-through mode,** used in nominal arm-A runs, and an off mode (not
  launched).
* **The injector identity** (source hashes and fault declaration) is bound in the P6 manifest.

### 4.2 Mechanisms

| Mechanism | Injection point | Reaches the production safety path | Timing / ownership effect | Removal | Independent proof the fault occurred |
|---|---|---|---|---|---|
| **M-DR depth relay** | Bridge output moved to `/gwm/p6/raw/front_depth/*` through the run-local `p3_sensor_bridge.yaml` (`ros_topic_name`, 2 keys, declared and checked under C11; M0 Finding A); the relay republishes on the accepted `/gwm/sensors/front_depth/*` names (drop, delay, repeat, stamp, dimension, NaN/inf, `camera_info` faults) | Yes: the unchanged adapter and perception consume it | Adds one copy of about 37 MB/s; latency is measured in pass-through (Gate A row 3, Gate B-0). No PX4 effect | Not launched → the accepted topology | Native Gazebo headers (P3 source probe) versus relay output, recorded by a separate passive witness subscribed to both topics |
| **M-MR map/health input faults** (M0, D1a approved: inside the controller shim, not a DDS relay) | The P6 entry point wraps `P5Mission.on_snapshot`, `on_adapter_health` and `tick` (drop, delay, reorder, burst) | Yes: the accepted mission and supervisor receive the faulted sequence | No topology change and no extra hop | Off = the accepted entry point | Perception's own `p4/local_map.jsonl` versus the controller's `snapshot_acquisition_sim_s` in `p4/decisions.jsonl` |
| **M-SR state relay to perception** (M0, D1a approved) | Perception started with `--px4-prefix /gwm/p6/px4_71` (an accepted argument); the relay republishes the six `fmu/out` state topics (delay, drop, stamp) | Yes: perception's association | One relay; the adapter and controller keep `/px4_71` | Prefix not changed | The bagged true `/px4_71` topics versus the relay output, recorded by the witness |
| **M-CS controller shim** | A P6 entry point imports the accepted `p5_control` and, before `main`, wraps `FrozenPolicy.scores`, `PolicyRanker.clock`, `StateCache.update` inputs or `Mission.ack` (policy faults F3; app-side state and ACK faults F2, F5) | Yes, in the same process, through the accepted classes | No new publisher. Delays are bounded and declared. With the shim pass-through, decisions must equal the P5 replay bit for bit | Off = the accepted entry point | The policy ledger (accepted code: `failure`, `inference_s`), the timing trace and `ros-events.jsonl`; for state faults, the bagged true PX4 messages versus the shim-modified values in the fault ledger |
| **M-PS process signals** | SIGINT, SIGSTOP/SIGCONT (bounded) or SIGKILL to one owned process (bridge, adapter, perception, controller, DDS agent, clock bridge) | Yes | None until injection | Signal not sent | `processes.txt`, exit codes, `/proc` state, P3 cleanup classification, and the PX4 side in the ULog |
| **M-LH load harness** | N bounded CPU or memory workers inside the owned namespace | Indirectly, through scheduling | Intended. No host or WSL setting is changed | Workers exit at their deadline, or are killed in cleanup | `resources.jsonl` (per-process CPU and RSS), `/proc/pressure`, schedstat as in the P5 diagnostic |
| PX4-native failure (`failure`, MAV_CMD 420) | PX4 modules | Yes | A command outside Policy A content | — | ULog `failure_detector`, estimator topics |

The last row is **not proposed** for P6 v1 (decision D5).

**Ownership invariant.** No mechanism publishes on `/px4_71/fmu/in/*`. The F4.07 competing-publisher
fixture *advertises* a publisher and never publishes a message. The evaluator proves this from the
graph record and from ULog `trajectory_setpoint` sources.

### 4.3 Fault ledger and witness

* **Injector ledger, `p6/fault-ledger.jsonl`.** Written by each injector: fault ID, parameters,
  arm/activate/clear sim and wall times, and the affected message IDs or tick IDs.
* **Witness, `p6_witness`.** A separate passive node that records the raw and the injected topic
  side by side. It never publishes.
* **Evaluation rule.** A scenario passes only if the evaluator confirms the injection from evidence
  the injector did not write:
  * the witness;
  * the native source probe;
  * the ULog;
  * the accepted controller's own ledgers;
  * the process table.

## 5. Safety state model (existing states only)

Evaluator labels are derived from existing records. No new controller state is added.

| Label | Derived from |
|---|---|
| NORMAL operation | `TRANSIT` with no stale reason, no policy failure record and no P2 failure event |
| Fault detected | first of: adapter health transition away from `fresh`; supervisor `stale` reason; policy ledger `failure`; P2 `abort` event; PX4 `failsafe` true (ULog) |
| Safe HOLD / HOVER | supervisor `HOVER` with `frozen_yaw`, or a policy HOLD tick (target and yaw unchanged) |
| Recovery permitted | stale reason clears *and* a collision-valid progress candidate exists on a fresh matched snapshot (the P4 transition `HOVER → AVOIDING`) |
| Recovery denied | stale for more than 10 s; 3 consecutive policy failures; any P2 `RECOVERY` (P2 never recovers) |
| ABORT requested | supervisor ABORT (`p4_supervisor_abort`) or P2 `abort` |
| LAND acknowledged | LAND (21) ACK accepted, `VERIFY_LANDING` |
| Landed and disarmed | controller `COMPLETE`, **and** ULog `vehicle_land_detected.landed` with `arming_state` disarmed |
| Terminal failure | P2 `RECOVERY` without LAND, with PX4 failsafe active and the vehicle not landed and disarmed by the run deadline |

**Measured quantities.** Every definition is in contract §6, with the clock it uses:

* detection latency;
* fault-to-safe-action latency;
* unsafe command count;
* time in degraded mode;
* abort initiation delay;
* landing completion;
* recovery eligibility;
* repeated-failure handling.

**Conservative recovery.** The accepted rule already requires a *fresh, matched, usable* snapshot,
fresh health, an unchanged calibration, no reset since the last usable frame and a valid candidate
on that snapshot before any motion. The known exception is F2.04: after a reset, a fresh frame
re-admits a map that was never revalidated.

## 6. Precedence for combined faults (from code order)

1. PX4 failsafe (external; the app only observes);
2. P2 checks → `RECOVERY`;
3. supervisor ABORT (absorbing, transit deadline);
4. staleness → HOVER, then ABORT at 10 s;
5. clearance ABORT;
6. policy failure → HOLD, then ABORT on the 3rd;
7. transitions and hover timeout;
8. dispatch guard at `send`, which can turn a slow tick into a P2 `RECOVERY`.

Two consequences, verified from code and to be confirmed in M3:

* While stale, the policy is **not invoked**, so a combined depth-loss and inference fault (F8.01)
  must show no policy failures after the stale onset.
* An inference above 0.050 s yields **both** a `timeout` ledger record and a dispatch-budget P2
  failure (F3.09).

## 7. Milestones (proposals; each step needs separate authorization)

| M | Content | Exit criterion |
|---|---|---|
| **M0** baseline and contract audit — **COMPLETE 2026-11-06** ([audit](v3_sim_p6_m0_audit.md)) | §1 re-verified; remap support of the adapter and perception; how the P4 "stale clock" and "calibration mismatch" fixtures were covered; PX4 v1.17 offboard-loss, data-link-loss and failsafe-delay semantics from the pinned source and the accepted ULogs (no flight); run-local config checks in the P3/P4/P5 evaluators | written audit; decisions D1–D10 answered; D1a, D4a and C11 raised and approved |
| **M1** taxonomy and scenarios | freeze `p6_faults.yaml` and `p6_scenarios.yaml` from the matrix; precedence and expected outcomes fixed before any runtime | the operator approves the scenario list |
| **M2** injection infrastructure | M-DR, M-MR, M-CS, M-PS, M-LH, the witness and the fault ledger, each with off/pass-through/active modes; unit tests | pass-through bit identity on recorded inputs |
| **M3** offline validation | every O-tier R item: P3 frame decoder and association; P4 perception on retained frames with corruptions; the supervisor and policy on recorded TRANSIT ticks and synthetic inputs; precedence (F8); the F2.04 map-reuse replay (extends P4 S7); F1.09; F4.03 | all R-O items pass; C-O items characterized with a written risk statement |
| **M4** evaluators | `p6_fault_evaluate` (independent injection proof), `p6_safety_evaluate` (composes the unchanged P5/P4 evaluator functions), `p6_timing_evaluate` (clock table), `p6_coverage_report` | tests on synthetic and retained P5 evidence, including negative cases (a fault declared but not observed must fail) |
| **M5** ground and integration | Gate A content dry-run on a development candidate (no credit), then the PX4-managed characterization needed by D4 on the ground only where safe | Gate A content reproducible |
| **M6** SITL fault validation | Gates B, S and C, each separately authorized | gate results |
| **M7** qualification and report | coverage report, A/B/C comparison, baseline re-verification, final report | P6 final acceptance, if earned |

## 8. Research evaluation

* **Arms:**
  * **A:** nominal P5 with the P6 injectors in pass-through. This is a new P6 baseline, not the
    accepted P5 runs.
  * **B:** single faults.
  * **C:** controlled compound faults.
* **Same scenario worlds and seeds across arms.** The accepted P5 flights are a reference, not arm A.
* **Metrics.** Contract §6 defines each:
  * detection rate and false-alarm rate;
  * detection latency and fault-to-safe-action latency;
  * recovery success and safe-abort rate;
  * unsafe command count, collisions and minimum truth clearance;
  * time in HOVER and time to LAND;
  * availability (the TRANSIT fraction not held);
  * timing-deadline violations;
  * fault coverage (by class and tier);
  * recovery stability.
* **Mission success and safety success are separate.** An ABORT and landing can be a successful
  safety validation and an unsuccessful mission; it is never reported as mission completion.
* **What the sample sizes support.**
  * **Gates:** one run per required SITL scenario (12 flights), which is an existence
    demonstration of the specified response under the declared conditions.
  * **Rates:** with 0 unsafe outcomes in 12 flights, the exact one-sided 95 % upper bound on the
    per-flight unsafe-outcome probability is 1 − 0.05^(1/12) ≈ 22 %, for these conditions only.
    No detection-rate, false-alarm-rate or reliability claim is supported. Those need P7 batches.
  * **Arms:** the A/B/C comparison is descriptive (paired by world), with no significance claims.

## 9. Risks and limitations

* **App-side state faults.** The shim changes what the app sees, not PX4's real estimate.
  App-side pose faults are not estimator failures. PX4-native estimator faults are deferred (D5).
* **The shim is part of the running program.** Shim runs execute accepted code plus a declared
  shim. The claim is "accepted code paths under an injected fault", and that is weaker than
  untouched binaries. Bit identity in pass-through limits the difference.
* **The relays add load.** That is about 37 MB/s for depth, in a host already near its limits (the
  P5 history). Pass-through runs must re-establish every P2–P5 limit before fault runs mean
  anything.
* **PX4-managed outcomes.** M0 showed they end in a Return that lands but breaches the P4 ceiling
  and may pass close over obstacle tops (D4a).
* **Load harness risk.** It can push WSL into the class 1 and class 2 stalls seen in P5. It must
  be bounded, and stop before the stack does.
* **No host changes.** No hardware, no host configuration change and no real-flight
  generalization.

## 10. Decisions requiring operator approval

**Answered 2026-11-06, all as recommended.** The record is in contract §10. The next approvals are:

1. the contract as a whole, with its remaining PENDING limits;
2. M0, a read-only audit: source, ULogs and evaluator checks, with no simulator.

The decisions, with the recommendation first for each, are in contract §10:

* **D1:** injection architecture.
* **D2:** shim identity.
* **D3:** the required versus characterization split.
* **D4:** PX4-managed outcomes.
* **D5:** PX4-native injection.
* **D6:** Gate B with a pass-through flight.
* **D7:** GCS link loss.
* **D8:** load-window exceptions.
* **D9:** the backup storage target.
* **D10:** retention.
