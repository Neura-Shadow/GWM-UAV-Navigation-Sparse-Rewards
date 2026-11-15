# v3-SIM P6 — failure injection, fault-tolerant control and safety validation: plan

**Status: CONTRACT APPROVED, NOTHING IMPLEMENTED (2026-11-06).** Nothing in P6 is implemented,
frozen, launched or accepted.
The [P6 contract](v3_sim_p6_contract.md) is approved. The operator answered decisions D1–D10 on
2026-11-06, all as recommended (contract §10 decision record). **M0 (the read-only audit) is
complete**; its results are in the [M0 audit](v3_sim_p6_m0_audit.md). It raised decisions D1a and
D4a and rule C11; the operator approved all three as recommended on 2026-11-06. **The contract
was approved as a whole on 2026-11-06.** No milestone after M0 is authorized yet. The fault
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
| **M1** taxonomy and scenarios — **COMPLETE 2026-11-06** (§11; the scenario list and four corrections approved by the operator) | freeze `p6_faults.yaml` and `p6_scenarios.yaml` from the matrix; precedence and expected outcomes fixed before any runtime | the operator approves the scenario list |
| **M2** injection infrastructure — **COMPLETE 2026-11-06** (§12) | M-DR, M-MR, M-CS, M-PS, M-LH, the witness and the fault ledger, each with off/pass-through/active modes; unit tests | pass-through bit identity on recorded inputs |
| **M3** offline validation — **COMPLETE 2026-11-07** ([report](v3_sim_p6_m3_offline_report.md), §13) | every O-tier R item: P3 frame decoder and association; P4 perception on retained frames with corruptions; the supervisor and policy on recorded TRANSIT ticks and synthetic inputs; precedence (F8); the F2.04 map-reuse replay (extends P4 S7); F1.09; F4.03 | all R-O items pass; C-O items characterized with a written risk statement |
| **M4** evaluators — **COMPLETE 2026-11-07** (§14) | `p6_fault_evaluate` (independent injection proof), `p6_safety_evaluate` (composes the unchanged P5/P4 evaluator functions), `p6_timing_evaluate` (clock table), `p6_coverage_report` | tests on synthetic and retained P5 evidence, including negative cases (a fault declared but not observed must fail) |
| **M5** ground and integration — **COMPLETE 2026-11-13** (§15; the `524f0714` dry run passed all 18 rows, no credit, §15.19) | Gate A content dry-run on a development candidate (no credit), then the PX4-managed characterization needed by D4 on the ground only where safe | Gate A content reproducible |
| **M6** SITL fault validation — **IN PROGRESS** (§16; Gate B dry run passed; Gate S dry run 4 of 5 and the S-5′ decision implemented; Gate C dry run 1 of 5 on `23d513ee`; the fixes and the C-3 rule implemented; candidate `75289deb` awaits a Gate C content rerun authorization, §16.9) | Gates B, S and C, each separately authorized | gate results |
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

**Answered 2026-11-06, all as recommended.** The record is in contract §10. The approvals so far, and the next one:

1. the contract as a whole, with its new limits: **approved 2026-11-06**;
2. M0, a read-only audit: source, ULogs and evaluator checks, with no simulator: **complete
   2026-11-06**;
3. next: M1, freezing the scenario specifications. It needs its own authorization.

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

## 11. M1 record: frozen specifications (2026-11-06, authorized by the operator: "Authorize M1")

* **Files** (outside every P3, P4 and P5 manifest scope; LF bytes pinned by
  `simulation/px4_gazebo_p6/.gitattributes`):

  | File | SHA-256 | Content |
  |---|---|---|
  | `simulation/px4_gazebo_p6/configs/p6_faults.yaml` | `9e585f26526740d3…` (final; first draft `dd7b5ce1…`) | 86 fault IDs with mechanism, detection, expected-behaviour codes, tiers and classes; the code definitions (S0, H35, A10, HT20, L45, L45R, PH3, R2, PX, H12, REF, INJ, LOAD) with their numbers; the precedence order |
  | `simulation/px4_gazebo_p6/configs/p6_scenarios.yaml` | `99b7994f9386c8f5…` (final; first draft `0d7e2d8b…`) | Gate A rows (2 readiness, 2 offline, 14 fault fixtures, 2 evaluation rows), B-0/B-1, S-1/S-2′/S-3/S-4/S-5′, C-1…C-5, with world, trigger, parameters and expected outcome; the characterization registry; the C11 baseline of the runners' own run-local changes; the P3/P4/P5 baselines and the P4 world file SHA-256 |
  | `tests/test_px4_gazebo_p6_specs.py` | `fcc113d7d125d0f4…` (final) | offline consistency test, 7 checks; passed on Windows and Linux |

* **The consistency test checks:**
  * IDs per category;
  * aliases;
  * that every gate row's fault carries that gate's class, and that every required fault is flown,
    directly or inside a compound row;
  * that worlds exist and the world file hash matches;
  * triggers;
  * the P3/P4 baselines against `p5_validation.yaml`;
  * the counts: 14 fixtures, 16 live ground runs, 12 flights, 31 offline items;
  * that the injected timings fall between the accepted limits: 25 ms sits above the 20 ms policy
    timeout and below the 50 ms dispatch budget, and 60 ms sits above the budget;
  * that no gated row expects a PX4-managed outcome.
* **Corrections found while freezing.** The operator approved all four as recommended on
  2026-11-06 ("Approve as recommended"), and they are applied to the specs, the contract and the
  matrix. None changes a limit:
  1. **The F2.09 ground rehearsal.** The matrix row lists R-A, but contract §8.1 has no F2.09
     fixture. **Applied:** `A-5.F2.09` added, giving 14 fixtures and 16 live ground runs (contract
     §8.1).
  2. **`L45R` for P2 `RECOVERY` landings.** S-5′, F3.09 and F4.07 in flight end in `RECOVERY`,
     never `COMPLETE`, so their landing is proven from the ULog only (`L45R`). Contract §8.3 S-5′
     said "L45". **Applied:** it now says `L45R`, and contract §6.4 defines both.
  3. **The offline required items** number **31**, not 30. F3.03 was missed in the matrix count,
     which is now corrected.
  4. **C11 must also allow the accepted runners' own run-local changes:** `p4_avoidance.yaml`
     `enabled`; `p5_policy.yaml` `enabled`, `model.path`, `model.sha256`. Without that, every run
     would fail C11. **Applied:** contract §2 C11.
* **M1 is complete.** M2 (the injection infrastructure) is next and needs its own authorization.

## 12. M2 record: fault-injection infrastructure (2026-11-06, authorized by the operator: "Authorize M2")

Nothing was launched. No simulator, PX4, QGC or ROS node ran, and no P2–P5 file changed. The work
is code and offline tests only, all in `simulation/px4_gazebo_p6/` and `tests/`.

### 12.1 What was built

| Module (`injection/`) | Role |
|---|---|
| `specs.py`, `declaration.py` | Read the frozen M1 specs. A run's fault declaration is built only from a frozen row (with its row digest), and every injector refuses a declaration that differs. The D8 load bounds are checked at declaration time. Modes: `off`, `pass-through`, `active` |
| `fault_ledger.py` | One append-only ledger per injector process (`p6/fault-ledger-<injector>.jsonl`): exclusive create, strict JSON, sequenced kinds (declared, armed, activated, affected, cleared, finished) |
| `schedule.py` | Pure triggers on the controller's own `p4_decision` and `transition` events (`transit_start`, `avoiding_first`, `avoiding_near`, `normal_2m`; `ground_window` fired by the runner), fault windows and invocation plans |
| `depth_faults.py`, `depth_relay.py` | DR: the F1 transforms. The relay node copies the bridge's QoS and republishes the received object itself whenever a frame is untouched. Activated by SIGUSR1 |
| `state_faults.py`, `state_relay.py` | SR: pose withheld from perception, through `--px4-prefix`. CS: app-side state and ACK faults. MR: snapshot and health drop, delay, reorder and burst |
| `policy_faults.py`, `controller_shim.py` | CS and MR: the accepted `p5_control` with wrappers on `P5Mission.tick`, `on_snapshot`, `on_adapter_health` and `ack`, `FrozenPolicy.scores`, `candidate_ranker.policy_choice` and `StateCache.update`. Delays are real sleeps, measured by the accepted `PolicyRanker` clock. Uninstall restores every original |
| `witness.py` | A passive recorder of raw and relayed topics (CRC-32 fingerprints). It contains no publisher |
| `competing_publisher.py` | F4.07: advertises a PX4-input topic and contains no publish call |
| `process_faults.py` | PS: one declared signal. SIGSTOP is bounded and proven from `/proc/<pid>/stat` |
| `load_harness.py` | LH: CPU and memory workers inside the D8 bounds, which stop themselves at their deadline |

### 12.2 Offline tests

`tests/test_px4_gazebo_p6_injection.py` has 29 tests; with the 7 spec checks that is 36 P6 tests.

* **Windows:** 34 passed, 2 skipped (POSIX-only).
* **Linux:** 36 passed.

Every injected fault is checked against the accepted code it targets:
* the P3 decoder and pipeline: `dimension_mismatch`, and the exact 5/5/5 stamp rejections;
* the P3 calibration cache;
* the P2 `StateCache`: `state_time_backwards`, `clock_epoch_mismatch`, `invalid_estimate`,
  `vehicle_health_invalid`;
* the P5 selection: `output_not_finite`, `output_shape`, `input_not_finite`.

Seven rig flights use the accepted P5 mission rig with the trained model and a deterministic policy
clock:
* **B-0:** pass-through is bit-identical to the unwrapped mission, in every event, ledger record,
  action and counter.
* **B-1 and S-3:** HOLD for 3 consecutive ticks, then `policy_unavailable`, then LAND.
* **C-2:** exactly 5 isolated timeouts; the mission completes.
* **C-3:** every choice is the worst allowed slot, inside the mask.
* **S-4:** a stale HOVER, then recovery and the goal.
* **S-5′:** exactly one 60 ms tick delay.

The static checks cover: no `/fmu/in` use outside the competing publisher; no accepted file refers to
P6.

### 12.3 Bug found and fixed while testing

Map faults (mechanism MR) are implemented in the shim, but the shim had enabled itself only for
mechanism CS. The S-4 rig flight dropped nothing, and the test caught it. The shim now serves both
mechanisms, and a test pins that.

### 12.4 Exit criterion: pass-through bit identity on recorded inputs

`validation/p6_passthrough_check.py` ran under the controller interpreter (NumPy 1.26.4), read-only,
on two accepted `2becfbbb` flights:

| Block | Gate C S2 `20261106T132442Z-…-184a5a5e` | Gate S S5 `20261106T130920Z-…-aa867e25` |
|---|---|---|
| Policy-ledger tensors: shim identical; equal to the recorded scores and argmax | 831 / 831 | 649 / 649 |
| Reproduced decision ticks through `PolicyRanker.choose` | 265, identical | 160, identical |
| PX4 state receipts through `StateCache.update` | 19,318, identical | 20,093, identical |
| Local-map snapshots through `P5Mission.on_snapshot` | 1,328, identical | 1,369, identical |
| Recorded raw depth frames (each checked against its recorded SHA-256): pass-through returns the same object | 300 | 300 |
| Active F1.07 / F1.06 on recorded frames | 10 `dimension_mismatch`; 5/5/5 stamp rejections | same |

The reports are `docs/evidence/v3_sim_p6_m2_passthrough_2becfbbb_gate_{c_s2,s_s5}.json`. Both
report `passed: true`.

### 12.5 Not yet exercised

The ROS node wrappers (`depth_relay`, `state_relay`, `witness`, `competing_publisher`) are only
import-tested; they have never run against ROS. Their first live use is M5 (ground integration),
which needs its own authorization. The P6 runner, manifest and evaluators are M4/M5 work.

## 13. M3 record: offline fault validation (2026-11-07, authorized by the operator: "Authorize M3")

* **Run:** the suite runner `validation/p6_offline_suite.py`, at commit `60093c8`, under the
  controller interpreter.
* **Result:** 87 tests passed, 0 failed, 0 skipped; **31 of 31 required offline items; 12 of 12
  flight-row rehearsals**. The result is bound to every P6 source, spec and test hash:
  `docs/evidence/v3_sim_p6_m3_offline_suite_60093c8.json` (SHA-256 `988da4341fff18cd…`).
* **Inputs:** read-only, from one retained accepted flight. The offline perception replay reproduced
  every recorded grid bit for bit before any corruption was measured.
* **Settled:** F5.02, from the rclpy executor order. The first tick after a stall sees pre-stop state
  and pre-stop sim time, so a short stall most likely goes undetected in flight.
* **Measured:** the characterized gaps, with risk statements. The most serious is F2.04: after a 1.0 m
  XY reset, 3 collision-valid candidates enter the real obstacle until the next fresh frame re-marks it;
  0.5 m gives none.
* **Fixed:** the shim no longer invents a selection index (F3.06).
* **Full report:** [M3 report](v3_sim_p6_m3_offline_report.md).
* **Next:** M4, the evaluators, which needs its own authorization.

## 14. M4 record: independent evaluators (2026-11-07, authorized by the operator: "Authorize M4")

Offline only. No simulator ran and no P2–P5 file changed. Everything is in
`simulation/px4_gazebo_p6/validation/`.

### 14.1 What was built

| Module | Role |
|---|---|
| `p6_run.py` | A read-only view of a P6 run: the accepted P5 layout plus `p6/` (declaration, per-injector ledgers, witness, `p6/injection.json`). A depth-bridge stop also writes the accepted `p4/fault_injection.json` |
| `p6_fault_evaluate.py` | The occurrence proof (C8), from evidence the injector did not write (table below) |
| `p6_safety_evaluate.py` | One check per outcome code on the accepted controller's records: H35, A10, HT20, PH3, L45 (through the accepted P4 `abort_landing`), L45R (landed and disarmed from PX4's own state messages), R2, GOAL, RECOVERY (§6.5), STALE_IFF_AGE, NO_INVOKE_AFTER_STALE, LIMITS_UNINJECTED, UNSAFE (§6.3 a), and the Gate A ground replay of the accepted `Supervisor.stale_reason` |
| `p6_timing_evaluate.py` | Detection latency, fault-to-safe-action latency, time degraded, recovered (reported) |
| `p6_evaluate.py` | The orchestrator, described below |
| `p6_coverage_report.py` | Rows, uncovered required faults, latency summaries, recoveries, unsafe-setpoint runs; descriptive only |

**What the orchestrator does:**
1. It requires the declaration to be the frozen row (strict).
2. It composes the unchanged P5 evaluator output. Each failure must match the row's exception table,
   and the replacement check named there must pass.
3. It runs the accepted P4 §9.1 `fault_window` for bridge stops, the proof and the response checks.
4. It writes `p6/p6-evaluation.json`. Characterization rows end `characterized`, never `passed`.

**Never excepted** (pinned by a test): safety, Policy A, ownership, cleanup, and P2's landed-and-disarmed
check.

**Independent proof per family:**

| Family | Evidence |
|---|---|
| Policy | Accepted policy-ledger failures equal the shim-affected invocations; the failure reason is as declared; `inference_s` ≥ the declared delay |
| Map | Perception's own `local_map.jsonl` published in the window, but the controller never used those snapshots |
| Health | The adapter's own health log stayed fresh, while the controller reported it missing |
| Pose | The witness shows true state continuing and relayed state stopped; perception's own unusable reasons |
| Bridge stop | The accepted P4 §9.1 F1 checks |
| Depth relay | The witness, raw versus relayed |
| Depth counters | The adapter's own rejection counters |
| Tick delay | The accepted timing trace, and the controller failure `stale_action` |
| Refusal | The refusal reason in the controller log, and no ROS artifacts |
| Competing publisher | The accepted graph record |
| Clock stall | The controller failure |

### 14.2 Tests

`tests/test_px4_gazebo_p6_evaluate.py` has 17 tests: Windows 11 passed and 6 skipped (retained runs);
WSL 17 passed. The full P6 set in WSL passes 100 of 100.

**On retained accepted runs:**
* The real Gate S S6 bridge stop, evaluated as an S-1-type fault (non-strict, P4 trigger), is
  **proved** and its response passes. Detection is ≤ 0.5 s after the trigger.
* The clean Gate C S2 flight passes as the B-0 pass-through baseline.

**Negative cases, all failing as required:**
* S-1, B-1 or S-4 declared on the clean flight: declared but not observed;
* a non-frozen declaration;
* one policy failure erased from the ledger;
* `safety:collisions` in the P5 output;
* target motion on a held tick;
* S-4 run in pass-through: nothing dropped;
* witness or counter mismatches;
* a controller-input fault on the ground.

**End to end:** B-1 and S-4 rig flights, written in the P6 run layout, pass with exactly their declared
exceptions.

### 14.3 Bugs caught by the tests and fixed

* **Motion attributed to the wrong tick.** The unsafe-motion check attributed the previous moving
  tick's 0.01 m step to the first held tick. A decision event logs the target *before* its tick acts,
  so the motion a tick causes is the next event's target. The accepted S6 flight failed the check until
  this was fixed.
* **A check-name mismatch** (`UNSAFE`) that would have refused C-3's outcome exception.

### 14.4 Decision: two Gate A fixtures removed (operator: "Remove them", 2026-11-07)

A-5.F4.02 (map delay) and A-5.F4.06 (health loss) are faults in the controller's own inputs. On the
ground the controller logs no decisions, so no record outside the injector shows the fault. The
evaluator therefore fails them explicitly (`no_independent_ground_evidence_for_a_controller_input_fault`)
rather than trusting the shim.

* **Already covered offline (M3):** their supervisor response (`adapter_health_missing`,
  `snapshot_stale` timing; F4.11).
* **Already covered in flight:** map input faults, in S-4 and C-5.
* **Recommended:** remove A-5.F4.02 and A-5.F4.06 from Gate A, giving 12 fixtures and 14 live ground
  runs. Keep F4.06 as R-O.
* **Alternative:** keep them, with a replay-only proof. That conflicts with C8.
* **Applied:**
  * `p6_scenarios.yaml`: the rows are gone; counts are 12 fixtures, 14 live ground runs, 32 offline items.
  * `p6_faults.yaml`: F4.02 is now C with tier S; F4.06 is now R-O.
  * Contract §8.1 and the §10 record.
  * The evaluator's ground rules.
  * The offline-suite coverage map: F4.06 is covered by the `adapter_health_missing` stale rule and F4.11.
  * The tests.

  The evaluator still refuses any controller-input fault on the ground.

### 14.5 Not yet exercised

* **Real P6 runs (M5/M6).** Each row's exception table is prefix-based and narrow. The exact P5 item
  names a real faulted run produces will be pinned in the M5 dry run (progress in §15.5).
* **L45R** on real PX4 data, the **load-harness** proof, and the proofs for the characterization
  flights:
  * not yet defined: X-F2.01, X-F5.01, X-F3.10, X-F5.09, X-F4.12, X-F5.02, X-F7.03;
  * they must be defined before each is authorized.

## 15. M5 record: ground integration and Gate A content dry runs (2026-11-07, authorized by the operator: "Authorize M5")

**Status: complete (exit condition met 2026-11-13, §15.19); no credit.**
* **Done:**
  * the offline recovery;
  * a fresh development candidate (`ce73e574…`, §15.7);
  * its authorized confirming dry run: 16 of 18 rows passed (§15.11);
  * the two F4.07/F4.13 decisions, adopted and implemented offline;
  * a new development candidate, `44fa1111…` (§15.12); its dry run was interrupted by a host stop, and its rerun
    passed 15 of 18 rows (§15.13–15.14);
  * the F4.13 detection decision, implemented in a new development candidate, `524f0714…` (§15.15).
  * the `524f0714` dry run: all 18 rows passed under one unchanged candidate (§15.17–15.19).
* **Not done:** M6 and the real P6 freeze, each with its own authorization.
* **No credit:** a dry run earns none. No P6 gate has run, and none is accepted.

Scope as authorized:
* the P6 runner and manifest;
* a ground dry run of Gate A content on development candidates, with no credit.

No run set `GWM_ALLOW_SITL_COMMANDS`, and the vehicle was never armed.

### 15.1 What was built

| File | Role |
|---|---|
| `scripts/p6_runner.py` | `P6Runner(P5Runner)` for the Gate A ground rows: relays, witness, controller shim, process signals, competing publisher, start-up refusals |
| `scripts/p6_manifest.py` | the P6 candidate: P3/P4/P5 re-verified (C1), P6 sources and tests, injectors, specs, owner copy, per-candidate Policy A; development candidates, dry-run approvals, the live-run lock and the drift check |
| `scripts/run_p6_gates.py` | the Gate A sequencer; `--dry-run` for development candidates (every row recorded, no gate result, no credit) |
| `scripts/run_p6.sh`, `p6_namespace_owner.py`, `p3_runner.py`, `p6_node.py` | namespace launch with the byte-identical owner copy; injector entry point |
| `scripts/verify_p6_evidence.sh` | the unchanged P5 evaluation, then the P6 evaluation |
| `validation/p6_integration_evaluate.py` | C11, INJECTORS (A-6), RELAY and GROUND_POLICY (A-3) |
| `injection/qos.py` | relay and `/clock` QoS (§15.4) |

P6 has no build package: its nodes run from the manifest-bound tree under `/usr/bin/python3`.

### 15.2 Operator decisions (2026-11-07)

1. **Policy A for M5 development candidates only.** Answer: "Approve for M5 (Recommended)".
   * Recorded in `state/p6-policy-A-approval-m5-development.json`.
   * The manifest refuses it for a real candidate, and a real candidate refuses it.
2. **F4.07 and F4.13 use the `ground_controller` trigger.** Answer: "Change the trigger (Recommended)".
   * Under `ground_window` the controller had already exited, so nothing could detect either fault.
3. **The designed fail-closed exit of F4.07 and F4.13.** Answer: "Allow that one item (Recommended)".
   * Exactly `cleanup:ros-controller:nonzero_exit_1`, on those two rows only, is replaced by the
     fail-closed proof.
   * Every other cleanup item stays never excepted (§15.5).
4. **F4.07 and F4.13 inject at the `ground_prestream` trigger** (the controller's transition into
   PRESTREAM_SAFE_SETPOINTS). Answer: "Adopt both" (§15.11, open decision 1).
5. **Exactly `p2:timing` on F4.07 and F4.13** is replaced by the fail-closed proof, which also requires
   the timing reconciliation to have completed. Answer: "Adopt both" (§15.11, open decision 2).

### 15.3 Live dry runs (no credit; evidence retained unchanged)

| Candidate | Commit | Rows | Outcome |
|---|---|---|---|
| `f6048036` | `7fe1917` | A-3 | **failed**: the relays could not create publishers, and `/clock` never matched (§15.4, items 1–2) |
| `08e79d05` | `04c1a95` | A-3 | **passed** |
| `ef5ce69e` | `9ce6443` | all 18 Gate A rows | **failed**: 6 passed, 12 failed or not run (below) |
| `ce73e574` | `68ccf30` | all 18 Gate A rows | **failed**: 16 passed; F4.07 and F4.13 failed on P5 items (§15.11) |

A-3 on `08e79d05`:
* the P5 evaluation had 0 failures, so every §3 limit held with three relays, the witness and the
  shim in the loop;
* relayed images arrived after a median of 3.5 ms, p95 9.6 ms, max 23 ms;
* CPU: depth relay 27 %, state relay 22 %, witness 44 %;
* the ground policy replay was bit-exact on 150 snapshots.

Rows of the `ef5ce69e` run:
* **Passed:** A-1, A-2, A-3, A-4 (87/87, 32/32, 12/12), F1.01, F1.06.
* **Failed on defects in M5 code:** F1.02, F1.04, F1.07, F1.11, F2.09; see §15.4.
* **Not run:** F3.01, F3.02, F3.03, F4.07, F4.13. Fixes were committed to bound files while the
  sequence ran, so the runner's manifest check refused each row before launching anything
  (`injectors,p6_source_files` differed). They are neither failures nor passes of their fixtures.
* **Failed through F1.11 only:** A-6 and A-7, whose aggregates include the crashed relay.

### 15.4 Root causes and fixes

Each fix has a regression test in `tests/test_px4_gazebo_p6_runner.py` or the injection tests.

1. **Relay QoS (`f6048036`).** Graph endpoint QoS has no history or depth, so `create_publisher`
   refused it.
   * **Fix:** the relays copy only reliability and durability, with keep-last depth (8, the bridge's
     queue).
2. **`/clock` (`f6048036`).** The P6 nodes subscribed to `/clock` as reliable, but the bridge
   publishes it best-effort, so no relay could ever activate.
   * **Fix:** every P6 node now takes `/clock` best-effort.
3. **F1.02, evaluator.** The P4 §9.1 window is the bridge-stop window. It was applied to the adapter
   stop as well.
   * **Fix:** `p4_window_applies` limits it to `sensor-bridge`.
   * **Re-evaluated on the retained evidence:** passes. The only P5 item allowed is
     `fault_injection_outside_s6`, covered by the adapter-stop proof (own exit, no health after
     exit, 166 native headers after the stop).
4. **F1.04, evaluator.** The delay proof had no upper bound, so it also judged frames after the 5 s
   window, which are correctly undelayed.
   * **Boundaries on the retained evidence:**
     * the relay activated at 31.092;
     * delayed frames run from 31.092 to 36.076, with lags of 0.300–0.304 s;
     * the first undelayed frame is at 36.108, past the window end of 36.092.
   * **Fix:** the proof now judges frames from the trigger to the window end minus the delay.
   * **Re-evaluated:** passes on 141 frames.
5. **F1.07, injector.** The 10-frame budget was counted from the `affected` list, which the relay
   drains into its ledger after each frame, so 161 frames were cut.
   * **Fix:** a separate counter.
   * **Offline in the relay's own pattern:** exactly 10 frames are cut, then every received object is
     relayed untouched.
   * **Still to prove live:** the old evidence correctly still fails.
6. **F1.11, injector.** Images went through the image transform, which had no F1.11 case, and the
   relay exited.
   * **Fix:** images pass untouched; only `camera_info` changes, from the trigger on.
   * **Still to prove live:** relay continuity, witness and cleanup.
7. **F2.09, evaluator boundary.**
   * **The fault reached its consumer:** in the window 0 position and 1 attitude message were
     relayed, against 240 and 481 true ones; perception was unassociated 46 times; the replayed
     supervisor hovered at 31.360 (`snapshot_stale`, within 0.35 s).
   * **The single "continued" attitude message** arrived at 36.080, exactly the trigger (31.080) plus
     5.000 s. The relay's last withheld message was at 36.068.
   * **The relay withholds on the half-open window [t, t+5)** of `schedule.Windows`, so releasing that
     message was correct. The proof's window was closed at the end.
   * **Fix:** both ends get the same 0.2 s settle margin, for messages in flight and for the relay's
     and the witness's separate `/clock` views. The expectation is unchanged: relayed state silent
     while true state continues.
   * **Re-evaluated:** passes.

### 15.5 F3.01–F3.03, F4.07 and F4.13: expected identities from the accepted code

| Row | Accepted controller refusal or failure | P5 evaluator items (unchanged `policy_fault`) | Allowed by |
|---|---|---|---|
| F3.01 | `PolicyError: checkpoint_missing` (`FrozenPolicy.load`, unreadable file) | `policy_fault:not_a_checkpoint_mismatch`, `:mismatched_checkpoint_record`, `:refusal_reason` | PROOF (refusal text, no node started) |
| F3.02 | `PolicyError: checkpoint_sha256_mismatch` (as in accepted P5 Gate A) | none | — |
| F3.03 | `ValueError: p5_model_schema_differs_from_frozen_hash` (`verified_policy`, before any load) | `policy_fault:not_a_checkpoint_mismatch`, `:refusal_reason` | PROOF |
| F4.07 | `competing_control_publisher:/px4_71/fmu/in/trajectory_setpoint` (`inspect_graph`; the topic is unversioned, per `topic-contract.json`) | `cleanup:ros-controller:nonzero_exit_1` (certain) | PROOF (decision 3) |
| F4.13 | `clock_stalled` (P2 mission stall check) | `cleanup:ros-controller:nonzero_exit_1` (certain) | PROOF (decision 3) |

How the items were derived:
* **F3 rows:** with the unchanged P5 check, in a test on the records the runner writes.
* **The two cleanup items:** from the accepted `main` (exit 1 unless connectivity passed). Both
  faults fire during WAIT_FOR_VALID_ESTIMATION, before connectivity passes.

Pinned from live evidence on `ce73e574` (§15.11):
* `p2:timing` on both rows;
* on F4.13 only, four more items, caused by the trigger point rather than by the fault.

The fail-closed proofs require:
* the declared reason;
* the advertiser's lifetime, or the `/proc` stop window;
* detection within 1.0 s plus one tick (plus the graph query for F4.07);
* the designed exit: 0, or 1 with connectivity "failed".

### 15.6 Live-run lifecycle (no manifest drift)

* **The live lock.** The sequencer holds `state/p6-live-run.lock` for the whole of a live sequence.
  While it exists, a freeze and a second sequence are refused.
* **The drift check.** Before every row the sequencer re-checks every hash-bound P6 file. A changed
  candidate ends the sequence; the remaining rows are recorded under `not_run`, never as passed.
* **During a live run:** no bound file changes, no commits, no CPU-heavy tests. If a change is needed,
  the run ends first, its evidence is kept, and a new candidate follows.

### 15.7 Offline regression on `68ccf30` (no simulator)

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests (specs, injection, offline, evaluate, runner) | 135 passed, 0 skipped |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 99 passed, 36 skipped (Linux-only: `fcntl`, ROS, retained WSL runs) |

The Windows skips are not counted as passes.

P3 `1e205fec…`, P4 `c110bd30…` and P5 `2becfbbb…` verify.

**New development candidate:** `ce73e57458fa7f1a743a58f64c8f612b96712829fd83d237a9fe4809daf145e1`
* frozen at `68ccf30` from a clean P6 tree;
* binds 41 files (16 injector modules), faults `172b8219…`, scenarios `699d29f1…`, and the
  development Policy A approval;
* Gate A: 18 rows, 14 of them live.

### 15.8 Authorization status

* Each earlier dry-run approval is bound to its own manifest SHA-256.
* **`ce73e574`.**
  * Authorized by the operator on 2026-11-07, "Authorize dry run", answering: "Authorize the full
    Gate A content dry run on development candidate ce73e574?"
  * Approval: `state/p6-dry-run-approval-ce73e57458fa.json`.
  * It ran once (§15.11).
* **`44fa1111` (§15.12):** no approval has been written. Its dry run needs a new operator
  authorization.
* **M5 exits** only when a complete dry run passes under one unchanged candidate.

### 15.9 PX4-managed characterization on the ground

None is possible without arming. M0 settled the D4 values from source and ULogs. The PX4-managed
outcomes stay separately authorized characterization flights.

### 15.10 Remaining

* The authorized dry run of all 18 rows on `44fa1111`.
  * F4.07 and F4.13 run at `ground_prestream` for the first time.
  * Any further P5 items they produce must be pinned from that evidence.
* M6 and the real P6 freeze, each with its own authorization.

### 15.11 The `ce73e574` dry run (2026-11-07, no credit)

**Run facts:**
* Started 10:50:40Z and took 25 min.
* No simulator processes were running before or after it.
* The live lock was held for the whole sequence and then released.
* The candidate was unchanged at every row and at the end; `not_run` is empty.
* Result: `state/p6-dryrun-A-ce73e57458fa-20261107T105106Z.result.json`; the evidence is retained
  unchanged.

**Passed (16):**
* A-1 and A-2.
* A-3.
* A-4: 87/87, 32/32, 12/12.
* F1.01.
* F1.02:
  * own exit, 0 health records after the exit, 169 native headers after the stop;
  * the P5 items it is allowed are covered by that proof.
* F1.04: 141 delayed frames, minimum lag 0.296 s.
* F1.06.
* **F1.07, live for the first time:** the adapter rejected exactly the 10 declared frames.
* **F1.11, live for the first time:** the adapter invalidated the calibration with
  `calibration_rejected_after_lock:focal_length_inconsistent_with_pinned_fov`, and the relay
  continued.
* F2.09:
  * 0 of 230 position and 0 of 460 attitude messages were relayed in the window;
  * perception was unassociated 46 times.
* **F3.01–F3.03, live for the first time:**
  * each refused with exactly the reason in §15.5, and no node started;
  * the P5 items were exactly those predicted.
* A-6 and A-7.

**F4.07 and F4.13 (failed).** Both fail-closed proofs passed:

| Row | Declared detection | Detected after |
|---|---|---|
| F4.07 | `competing_control_publisher:/px4_71/fmu/in/trajectory_setpoint` | 0.14 s from the advertisement, 1.04 s from the advertiser's launch |
| F4.13 | `clock_stalled` | 1.016 s wall from the stop |

* The controller exited 1 with connectivity "failed".
* Decision 3 replaced the `cleanup:ros-controller:nonzero_exit_1` item.
* The P5 items no exception covers are below.

1. **`p2:timing`, both rows. Certain, from the accepted P2 code.**
   * The P2 timing check (`timing_evidence.trace_metrics`) reports failed whenever the
     controller's result carries a failure, and a fail-closed row always carries one.
   * Every reconciliation check passed: the trace was complete with no overflow, and the counts and
     ordering matched. The evaluator raised no `p2_integrity_error`.
   * No current exception covers the item.
2. **`p2:bag_received_ledger_count:vehicle_attitude`, F4.13. Caused by the trigger point.**
   * PX4 first publishes `vehicle_attitude` at about 5.5 s PX4 time; on this candidate's runs it was
     5.46–5.56 s.
   * F4.13 stalls `/clock` at the controller's first transition (4.37 s). The controller's ledger
     ends at about 4.85 s PX4 time, before PX4's first attitude message.
   * F4.07's controller exited later and holds 393 attitude messages.
3. **`depth:state_association_incomplete`, `depth:reset_bracket_not_single_reset` and
   `depth:reset_bracket_bound`, F4.13. A P6 runner defect: the window placement.**
   * `live_trigger` holds the unchanged ground window from the controller's exit. For F4.13 that was
     6.172 s, inside PX4's estimator start-up.
   * On every run of this candidate the estimate is invalid until about 6.6 s, and start-up
     reset-epoch brackets occur at 5.2–6.7 s.
   * So 3 `estimate_invalid` observations (6.172–6.372) and one start-up reset bracket (6.472) fell
     in the window. The ledger of a controller that has already exited cannot corroborate that reset.
   * F4.07's window began at 9.604 and was clean: 150 of 150 observations matched. The accepted
     ground rows begin their window at about 16 s.

**Open decisions:**
1. **The trigger point.** Recommended: inject at the controller's transition into
   PRESTREAM_SAFE_SETPOINTS.
   * At that transition the estimate has been valid for 5 s (P2 `estimator_stable_sim_s`) and every
     PX4 stream is in the ledger. The window after the exit starts after the estimator start-up,
     so items 2 and 3 cannot arise.
   * The ground diagnostic then holds PRESTREAM for 3.0 s sim, which covers about 1.1 s of
     detection.
   * The accepted `main` exits 0 after connectivity passes (ground `p4_outcome` is `not_run`), so
     the designed exit becomes 0. That is already accepted by the fail-closed proof, and decision 3
     is no longer exercised.
   * The alternative: keep the first transition, hold the window until after the estimator start-up,
     and add a proof-backed exception for item 2.
2. **`p2:timing` on F4.07 and F4.13.** Recommended: allow exactly this item on these two rows,
   replaced by the fail-closed proof. The replacement would also require:
   * the P2 timing reconciliation completed (no `p2_integrity_error`);
   * its `controller_failure` is the declared detection.

   Without this exception no fail-closed row can pass under the accepted P2 rule.

Both decisions were adopted on 2026-11-07 ("Adopt both"); see §15.12.

### 15.12 The two decisions implemented, and candidate `44fa1111` (2026-11-07)

**Code (`f74b71f`):**
* **The trigger.** `injection/schedule.py` and `configs/p6_scenarios.yaml` replace
  `ground_controller` with `ground_prestream` (record `m5_prestream_decision_approved`).
  `p6_runner.live_trigger_due` fires only on the controller's transition into
  PRESTREAM_SAFE_SETPOINTS.
* **The fail-closed proofs** (`validation/p6_fault_evaluate.py`) now also require:
  * the runner's injection request at or after that transition, and connectivity "passed";
  * the P2 timing reconciliation completed: a timing status in the P5 evaluation and no
    `p2_integrity_error`.
* **The evaluator** (`validation/p6_evaluate.py`): on A-5.F4.07 and A-5.F4.13 only, exactly
  `=p2:timing` is replaced by PROOF.
* **What did not change:** the rows' faults, parameters and expected outcomes, and every accepted
  P2–P5 source, threshold and model.

**Read from the accepted sources** (regression tests):
* connectivity passes on the estimator-stability rule just before PRESTREAM;
* the ground diagnostic then holds PRESTREAM for `ground_prestream_sim_s` (3.0 s);
* `main` exits 0 once connectivity has passed (ground `p4_outcome` is `not_run`), so the designed exit
  is 0;
* `trace_metrics` reports failed exactly when the controller's result carries a failure.

**Offline regression on `f74b71f`:**

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 139 passed, 0 skipped |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 103 passed, 36 skipped (Linux-only; not counted as passes) |

**Development candidate:** `44fa111162307b240af152105f266896b96ff5e30c460e08c42c36d3dfb2d5a0`
* frozen at `f74b71f` from a clean P6 tree; it verifies;
* P3 `1e205fec…`, P4 `c110bd30…` and P5 `2becfbbb…` re-verified;
* binds 41 files (16 injector modules), faults `172b8219…`, scenarios `a451bb21…`, and the
  development Policy A approval;
* Gate A: 18 rows, 14 of them live;
* no dry-run approval has been written.

**Expected on its dry run:**
* F4.07 and F4.13 detect at PRESTREAM and exit 0, so no cleanup item arises.
* `p2:timing` is replaced by the proof.
* Any other item is a P6 failure, to be diagnosed from evidence and not excepted.

### 15.13 The `44fa1111` dry run was interrupted by a host stop (2026-11-07)

**Authorization:** the operator, on 2026-11-07, "Authorize dry run", answering: "Authorize the full
Gate A content dry run on development candidate 44fa1111?" Approval:
`state/p6-dry-run-approval-44fa11116230.json`.

**What happened:**
* The sequence started at 12:55:55Z with no other simulator processes running.
* At about 12:57:40Z, during the start-up of its first live row, A-1, the Windows host stopped
  without a clean shutdown.
  * Event Kernel-Power 41 was logged, with bugcheck 0.
  * There was no dump, no sleep and no power-button press.
  * The host rebooted on 2026-11-13.
* WSL lost its unflushed writes:
  * the A-1 run, `20261107T125708Z-p6dry-A-1-b72222af`, has zero-length files and status
    `incomplete`;
  * there is no result file;
  * `state/p6-live-run.lock` is left by the dead sequence (pid 312859, previous boot).
* The cause cannot be settled from the logs. The same launch had run 14 times without fault on
  `ce73e574`.

**Status:**
* No row has a result, so this is neither a pass nor a failure of any row.
* The interrupted run is retained unchanged. Under the approval's cap it counts as 1 of 40 live runs.
* After the reboot the candidate still verifies, including P3, P4 and P5 (C1).
* A rerun is a retry, so it needs the operator's explicit authorization, and so does clearing the
  stale lock.

### 15.14 The `44fa1111` rerun (2026-11-13, no credit)

**Authorization:** the operator, on 2026-11-13, "授權重跑" (authorize the rerun), answering whether to
clear the stale live lock and rerun the full Gate A dry run on `44fa1111`.

**Before the run:**
* The lock's pid (312859, previous boot) was not running, no simulator process was present, and the
  lock named this candidate. Only `state/p6-live-run.lock` was removed.
* The host was on AC power, battery 79 %.

**Run facts:**
* 12:40:48Z to 13:09:11Z.
* The candidate was unchanged at every row and at the end; `not_run` is empty; the lock was released.
* Result: `state/p6-dryrun-A-44fa11116230-20261113T124106Z.result.json`; the evidence is retained
  unchanged.

**Passed (15):**
* A-1, A-2, A-4, A-6, A-7.
* F1.01, F1.02, F1.04 (141 frames), F1.06, F1.07 (10 of 10), F1.11.
* F3.01–F3.03.
* **F4.07, at `ground_prestream` for the first time:**
  * detected 0.686 s after the advertisement;
  * the controller exited 0 with connectivity "passed";
  * the only P5 item, `p2:timing`, was replaced by the proof;
  * no cleanup item arose.

**Failed (3), on two causes.**

1. **A-3 and F2.09: the host ran the simulation too slowly for an accepted P2 rule.**
   * The rule (`timing.validate`) treats estimator status flags as stale after 2.0 s of wall time
     without a new message (`state_fresh_wall_s`). PX4 publishes them every 1.0 s of simulated time.
   * So whenever the simulation runs below half real time for a whole flags interval, the rule
     fires.

   | Dry run | Longest wall gap between flags |
   |---|---|
   | 11-07, every row | 1.67 s |
   | 11-13, A-3 | 2.95 s (22 gaps over 2.0 s) |
   | 11-13, F2.09 | 2.07 s (1 gap over 2.0 s) |
   | 11-13, other rows | up to 1.88 s |

   * **A-3:** the controller never held a valid estimate for 5 s (`ground_diagnostic_deadline`).
   * **F2.09:** the controller aborted `stale_state:estimator_status_flags` during PRESTREAM, before
     the fault was injected. Its fault proof passed; only `p2:timing` was refused.
   * The mean real-time factor was about 0.66, against about 0.76 on 11-07.
   * The Windows power mode on AC was "Best power efficiency" (overlay `961cc777…`). It is the likely
     cause but not proven.
   * No P6 code is involved, and the accepted threshold stays unchanged.
2. **F4.13: a different detection of the same fault. A P6 specification gap caused by moving the
   trigger.**
   * While waiting for a valid estimate, the accepted mission catches a failed state check and only
     restarts the stability timer. So at the old trigger the 1.0 s `clock_stalled` check was the only
     abort path.
   * From PRESTREAM on, a failed state check aborts at once. With `/clock` stopped, PX4's own
     timestamps keep advancing, so `clock_epoch_mismatch:vehicle_local_position` fires as soon as one
     runs more than `clock_offset_tolerance_s` (0.1 s) ahead of the frozen clock. Here PX4 was at
     12.712 against a frozen 12.608.
   * The abort came 0.269 s wall after the stop request, while the clock bridge was stopped.
   * The runner expected only `clock_stalled`, so it ended the row with
     `declared_detection_not_observed`, and the evaluation had no sensor window.

**Open:**
* **An operator decision:** accept `clock_epoch_mismatch:<state topic>` as F4.13's detection at
  `ground_prestream`, besides `clock_stalled`.
  * The proof would be otherwise unchanged:
    * the `/proc` stop window;
    * the abort while stopped, within 1.0 s plus one tick of the stop request;
    * connectivity "passed", exit 0;
    * the timing reconciliation completed.
  * It would also require the named topic's PX4 timestamp in the controller's own ledger to be more
    than 0.1 s past the frozen clock.
* **The host's speed:** to be restored by the operator before the next dry run. That needs a new
  candidate and its own authorization.

### 15.15 The F4.13 detection implemented, and candidate `524f0714` (2026-11-13)

**Decision:** adopted by the operator on 2026-11-13, "接受並準備新候選" (accept and prepare a new
candidate). Recorded as `m5_f413_detection_approved`.

**Code (`e5d7fb5`):**
* **The runner.** `LIVE_DETECTION["F4.13"]` is now `("clock_stalled", "clock_epoch_mismatch:")`.
* **The proof** (`p6_fault_evaluate._clock_stall`) accepts `clock_stalled` or
  `clock_epoch_mismatch:<accepted state topic>`.
  * The epoch reason counts only if the controller's own ledger has a sample of that topic,
    received before the abort, whose PX4 timestamp is past the frozen clock (the abort's simulated
    time) by more than the run-local `clock_offset_tolerance_s`.
  * Unchanged: the `/proc` stop window, the 1.0 s plus one tick bound, connectivity "passed",
    exit 0, and the timing reconciliation.
* **The specifications.** In `p6_faults.yaml` and `p6_scenarios.yaml`, F4.13's detection and expected
  text now name both reasons. Its parameters are unchanged.

**Read from the accepted sources** (regression tests):
* before PRESTREAM, a failed state check only restarts the stability timer;
* the epoch check comes before the staleness check;
* the tolerance is 0.1 s and the stall limit 1.0 s;
* `STATE_TOPICS` matches.

**Replayed in memory on the retained `44fa1111` F4.13 evidence:**
* PX4 was 0.104 s past the frozen clock, against a tolerance of 0.1;
* the detection came 0.269 s after the stop, while stopped;
* exit 0, connectivity "passed".

The only remaining item is the timing reconciliation. That run's P5 evaluation never reached it,
because the runner had ended the row. It is not counted.

**Offline regression on `e5d7fb5`:**

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 141 passed, 0 skipped |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 105 passed, 36 skipped (Linux-only; not counted as passes) |

**Development candidate:** `524f07143f592a5ab7a31f2ad298a31ab879183a4126726ca02c056e47202405`
* frozen at `e5d7fb5` from a clean P6 tree; it verifies;
* P3, P4 and P5 re-verified;
* binds 41 files, faults `97fc5eef…`, scenarios `626fd44f…`, and the development Policy A approval;
* no dry-run approval has been written.

**Before its dry run:**
* **Host speed.** The Windows power mode on AC was still "Best power efficiency" at freezing. A-3 and
  F2.09 need the accepted P2 wall-clock freshness to hold, so the operator should switch it to "Best
  performance" first.
* **Authorization.** The dry run needs its own operator authorization.

### 15.16 Proposed host-stability diagnostic before the next full dry run (2026-11-13)

**Operator instruction (2026-11-13):** proceed without a Windows power-mode change; before another
full 18-row dry run, propose a lightweight, separately authorized, ground-only diagnostic of simulator
RTF, estimator-status publication intervals and controller readiness under the current host settings.
No P2–P5 threshold may be relaxed.

**What the retained evidence already shows (read-only, scratchpad `m5_host_diag.py`):**

| Run | RTF mean | Flags wall gap max / p95 | Gaps > 2.0 s | 1-min load max |
|---|---|---|---|---|
| 11-07 A-1 | 0.80 | 1.61 / 1.50 s | 0 | — |
| 11-07 A-3 | 0.69 | 1.20 / 1.20 s | 0 | 4.5 |
| 11-07 F2.09 | 0.77 | 1.22 / 1.19 s | 0 | — |
| 11-13 A-1 | 0.65 | 1.93 / 1.79 s | 0 | — |
| 11-13 A-3 | 0.46 (ledger) | 2.95 / 2.68 s | 22 | 6.7 |
| 11-13 F2.09 | 0.53 | 2.07 / 1.38 s | 1 | 5.6 |
| 11-13 F4.07 | 0.60 | 1.88 / 1.87 s | 0 | — |

* **Per-process CPU was the same on both days:** Gazebo about 1.3 cores, the adapter 0.43, the witness
  0.4–0.45, perception 0.36, the controller 0.33–0.4, each relay 0.21–0.27; about 4.3 cores in all on
  a 24-thread i7-13700HX.
* **So the P6 processes did not grow.** The host delivered less simulated time per wall second for the
  same demand, and its load was higher. Gazebo's single simulation thread is the limit; nothing in P6
  can speed it up.
* **The flags interval in PX4 time was 1.0 s in every run.** Only the wall interval stretched.

**The proposed diagnostic (needs its own authorization; no credit):**
* **Rows:** on candidate `524f0714`, through the dry-run sequencer with `--rows`: A-1 then A-3 in one
  sequence, then A-3 once more. Three live runs, about six minutes of simulator time.
  * A-1 is the accepted P3 readiness run without any P6 node; A-3 is the P6 ground readiness run with
    the relays and the witness. Their difference isolates P6 overhead from host slowness.
* **Approval:** a separate file, `state/p6-dry-run-approval-524f07143f59-hostdiag.json`, bound to the
  candidate, `live_runs_max: 3`, citing the operator's authorization. The runner counts runs per
  approval file, so these never count against the later full dry run's cap.
* **Host snapshot before each sequence (read-only):** the active power overlay, battery and AC state,
  and the top Windows processes by CPU over a 5 s sample, saved beside the result file.
* **Measured from the retained evidence only**, by the read-only script: RTF mean and minimum interval
  RTF; flags wall gap max, p95 and the count over 2.0 s; time from controller start to PRESTREAM;
  connectivity and failure; per-process CPU; 1-min load.
* **Pre-declared stability criteria**, from the 11-07 envelope and the accepted P2 rule
  (`state_fresh_wall_s` 2.0 s, unchanged):
  1. every run's flags wall gap max ≤ 1.7 s (11-07 maximum over 14 rows was 1.67 s; 0.3 s margin
     under the rule);
  2. both A-3 runs: RTF mean ≥ 0.65, connectivity "passed", 0 P5 failures;
  3. A-1: RTF mean ≥ 0.75.
* **Decision rule:**
  * all criteria hold → request a separate authorization for the complete 18-row dry run on
    `524f0714`;
  * otherwise → report the measured bottleneck with the mitigations below; no retry.
* **Nothing changes:** no bound P6 file, no accepted file, no host setting. The script is a scratchpad
  tool and binds nothing.

**Mitigations to weigh if the criteria fail** (each needs the operator's choice; none relaxes a limit):
1. **Scheduling, no code:** run with the host otherwise idle (no Docker backend, editors or vendor
   utilities active), and add a pre-run host-CPU gate to the launch script so a sequence does not start
   on a busy host.
2. **Software, new candidate:** lower the scheduling priority (`nice`) of the P6 witness and relays.
   Gazebo is the bottleneck and the host is not saturated, so the expected gain is small.
3. **Host, needs authorization:** the power overlay, or the Hyper-V vCPU placement for WSL on the
   performance cores. Both are host configuration and stay the operator's decision.

### 15.17 Host-stability diagnostic: result (2026-11-13, no credit)

**Authorization:** the operator, 2026-11-13, "Authorize diagnostic". Approval
`state/p6-dry-run-approval-524f07143f59-hostdiag.json` (cap 3, bound to `524f0714`).

**Host before the sequence (read-only snapshot):** power overlay "Best power efficiency", AC with
battery 79 %, Windows CPU 7 %, no heavy host process (the top one used 1.45 CPU-seconds in 5 s).

**Sequence 1 (13:53:56Z–13:58:00Z), rows A-1 then A-3; result
`state/p6-dryrun-A-524f07143f59-20261113T135417Z.result.json`:**

| Run | RTF mean | Flags wall gap max / p95 | Gaps > 2.0 s | 1-min load max | Outcome |
|---|---|---|---|---|---|
| A-1 (no P6 node) | 0.58 | 2.31 / 1.85 s | 1 | 2.7 | passed (readiness only; the P3 evaluation has no wall-freshness rule) |
| A-3 (relays, witness) | 0.50 | 2.16 / 2.06 s | 10 | 6.8 | failed: `stale_state:estimator_status_flags` after 51.8 s to PRESTREAM; `p2:timing` |

Per-process CPU was again unchanged: Gazebo 1.17–1.22 cores, adapter 0.4, witness 0.35, perception
0.37, controller 0.3, relays 0.2–0.24.

**Against the pre-declared criteria (§15.16):** all three fail.
1. Flags wall gap max ≤ 1.7 s: A-1 2.31 s, A-3 2.16 s.
2. A-3 RTF ≥ 0.65 and 0 P5 failures: 0.50, and the run failed.
3. A-1 RTF ≥ 0.75: 0.58 (0.80 on 11-07).

**The third run (A-3 again) was not executed.** The decision was already settled, and it would only
have added simulator time. Two of the three authorized runs were used.

**The measured bottleneck:**
* The accepted P3 stack alone (A-1), with no P6 process, already runs at 0.58 real time and breached
  the accepted P2 wall-freshness rule (2.0 s) once. On 11-07 the same row ran at 0.80 with a 1.61 s
  maximum.
* CPU demand per process is identical across days, and the host was idle before the run. So the host
  delivers fewer Gazebo simulation-thread cycles per second than on 11-07: a frequency or core-placement
  cap, consistent with the "Best power efficiency" overlay that has been active since the reboot.
* P6 adds load (A-3 load 6.8 against 2.7) and lengthens the gaps further, but is not the cause: the
  rule fails without it.

**Consequence:** under the current host settings a full 18-row dry run would fail A-3 and every
fixture whose controller must reach PRESTREAM. No request for the full dry run is made.

**Mitigations (the operator's choice):**
1. **Host (needs authorization, not done):** set the AC power overlay to "Best performance" (or
   restore the 11-07 setting), or pin WSL's vCPUs to the performance cores. This is the only measure
   that addresses the measured cause.
2. **Scheduling, no code:** run only on an idle host, with a pre-run host-CPU gate in the launch script.
   The host was already idle today, so this alone will not suffice.
3. **Software, new candidate:** `nice` for the witness and relays. It reduces the A-3 load but cannot
   raise Gazebo's single-thread throughput; A-1 fails without any P6 process.

The accepted P2 freshness timeout and every P3–P5 threshold stay unchanged.

### 15.18 Host-stability diagnostic repeated after the power-mode change (2026-11-13, no credit)

**Host change (the operator's, not recorded in any manifest):** the Windows AC power overlay was set
to "Best performance" (`ded574b5…`); the on-battery overlay stays "Best power efficiency".

**Authorization:** the operator, 2026-11-13: one new capped ground-only sequence on `524f0714`, A-1
then A-3 once each (cap 2), under §15.16; stop and report; no full dry run, no retry. Approval
`state/p6-dry-run-approval-524f07143f59-hostdiag2.json`.

**Verified before starting:** AC power (battery 79 %), the Best-performance overlay, host CPU 2 %, the
manifest and the P3/P4/P5 baselines (C1), no simulator or P6 process, no live lock.

**Sequence (14:13:36Z–14:16:15Z); result
`state/p6-dryrun-A-524f07143f59-20261113T141353Z.result.json`:**

| Run | RTF mean | Flags wall gap max / p95 | Gaps > 2.0 s | 1-min load max | Outcome |
|---|---|---|---|---|---|
| A-1 | 0.80 | 1.23 / 1.21 s | 0 | 1.9 | passed |
| A-3 | 0.69 | 1.59 / 1.45 s | 0 | 2.7 | passed, 0 P5 failures, 11.2 s to PRESTREAM |

Per-process CPU unchanged (Gazebo 1.15–1.19 cores, witness 0.41, adapter 0.4, perception 0.37,
controller 0.35, relays 0.2–0.24). Both runs now match the 11-07 figures (A-1 0.80, A-3 0.69).

**Against the pre-declared criteria (§15.16):** all hold.
1. Flags wall gap max ≤ 1.7 s: 1.23 s and 1.59 s.
2. A-3 RTF ≥ 0.65 and 0 P5 failures: 0.69, passed.
3. A-1 RTF ≥ 0.75: 0.80.

**Conclusion:** the measured bottleneck of §15.17 was the power overlay. With it changed, the host
again meets the accepted P2 wall-freshness rule with margin, and P6's own load has not changed.
The candidate was unchanged throughout and the lock was released. No threshold or source changed; no
re-freeze.

**Stopped here, as authorized.** The full 18-row dry run on `524f0714` needs its own authorization,
with the host kept on AC power in Best-performance mode for the whole run (about 25 min).

### 15.19 The `524f0714` dry run: all 18 rows passed (2026-11-13, no credit)

**Authorization:** the operator, 2026-11-13, "Authorize dry run", answering: "Authorize the full
18-row Gate A content dry run on development candidate 524f0714?" Approval
`state/p6-dry-run-approval-524f07143f59.json`.

**Verified before starting:** AC power, the Best-performance overlay, host CPU 0 %, the manifest and
the P3/P4/P5 baselines, no simulator or P6 process, no live lock.

**Run facts:** 14:19:13Z–14:41:02Z (21.5 min). The candidate was unchanged at every row and at the
end; `not_run` is empty; the lock was released; no residual process. Result:
`state/p6-dryrun-A-524f07143f59-20261113T141932Z.result.json`; the evidence is retained unchanged.

**All 18 rows passed.** The P6 evaluation refused no P5 item on any row.
* A-1, A-2, A-3, A-4 (87/87, 32/32, 12/12), A-6, A-7.
* F1.01, F1.02, F1.04, F1.06, F1.07 (10 of 10 frames rejected), F1.11 (calibration invalidated,
  relay continuous).
* F2.09: 0 of 230 position and 0 of 460 attitude messages relayed in the window; perception
  unassociated 46 times.
* F3.01, F3.02, F3.03: each refused with its declared reason, no node started.
* **F4.07 at `ground_prestream`:** detected 0.080 s after the advertisement; exit 0, connectivity
  "passed"; the only P5 item, `p2:timing`, replaced by the proof.
* **F4.13 at `ground_prestream`, the first pass:** `clock_epoch_mismatch:vehicle_local_position`,
  0.157 s wall after the stop while the clock bridge was stopped; the controller's ledger showed PX4
  0.108 s past the frozen clock against the 0.1 s tolerance; exit 0, connectivity "passed"; the only
  P5 item, `p2:timing`, replaced by the proof.

**Host during the run (read-only metrics over the 10 live rows with a controller):** RTF 0.60–0.76,
flags wall gap maximum 1.92 s, 0 gaps over the accepted 2.0 s rule.

**M5 exit condition met:** a complete dry run passed under one unchanged development candidate. The
dry run earns no credit; no P6 gate has run and none is accepted. M6 and the real P6 freeze each need
their own operator authorization.

## 16. M6 record: SITL fault validation (authorized by the operator 2026-11-13: "Authorize M6")

**Status: in progress.** No flight has been run, no gate has run, and nothing earns credit.

### 16.1 What M6 needs (contract §8)

* Gates B (2 rows), S (5) and C (5), each separately authorized and bound to one non-development P6
  manifest, in the order A → B → S → C: each needs its predecessor's accepted result for the same
  manifest.
* So before any flight gate: a production Policy A approval for the P6 candidate (the development
  approval is refused for a real candidate), the real freeze, and an authorized real Gate A.
* Every flight row is a simulated flight on the row's P4 world (S1–S4) with the vehicle armed in SITL,
  never real hardware.

### 16.2 What was built (`8740815`, offline)

| File | Change |
|---|---|
| `scripts/p6_runner.py` | the `flight` scope: the accepted P5 flight path on the row's world (`--scenario`), under the accepted gating (`GWM_ALLOW_SITL_COMMANDS=1` and `--allow-simulated-flight`, flight rows only; a ground row refuses the flag). The runner feeds the controller's own events to the frozen trigger with the frozen P4 scenario boxes and fires its mechanisms once: relay SIGUSR1 (DR, SR), the accepted P4 bridge SIGINT (PS, position recorded), the bounded load harness at its own `load_start` trigger (LH, C-5). CS and MR fire inside the shim. The accepted flight window (PRESTREAM to COMPLETE or RECOVERY) is written before a fail-closed exit is reported, so the unchanged P5 evaluation runs and the row's rules decide (S-5′). |
| `scripts/run_p6_gates.py` | runs Gates B, S and C; requires the accepted predecessor result for the same manifest; passes the flight gating to flight rows only; dry runs of flight content need a matching approval |
| `scripts/p6_manifest.py` | dry-run approvals carry `gate_content`; Gate B/S/C content needs `flight` and `simulated_flight_only` true, named in the confirmation phrase; Gate A content stays ground-only |
| `injection/load_harness.py` | records its activation and clearing in a fault ledger (`--ledger`, `--row`, `--fault`, `--sim-s`) |
| `validation/p6_integration_evaluate.py` | the injector-process check expects `p6-load-harness` for a load row |

**A bug caught by inspection before any run:** the generated P4 world declaration records boxes as
`center_m` with a height, while the triggers read the scenario spec's 2-D `center`. The runner passes the
frozen `p4_scenarios.json` boxes, and a test pins both shapes.

Per row, what the runner fires: S-1 PS at `avoiding_near`; S-2′ SR at `avoiding_first`; C-1 DR at
`avoiding_first`; C-4 PS at `avoiding_first` (its CS part in the shim); C-5 LH at `transit_start` (its
MR part in the shim); B-1, S-3, S-4, S-5′, C-2, C-3 fire inside the shim only; B-0 fires nothing.

### 16.3 Offline regression on `8740815` (no simulator)

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 149 passed in the full run, 0 skipped; the runner file re-run after the last edit: 46 passed (150 tests in all) |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 106 passed, 44 skipped (Linux-only; not counted as passes) |

P3 `1e205fec…`, P4 `c110bd30…` and P5 `2becfbbb…` verify.

### 16.4 Development candidate and the next step

**Development candidate `33f33c5a22c917a7feaa184a5598cdbc421cc4964a7c04b33226b99f1f160ffb`**, frozen at
`c1b84be` from a clean P6 tree; verifies; 41 files; no dry-run approval written.

**Recommended next step (needs its own authorization):** a flight-content dry run of Gate B on this
development candidate, simulated flight only, no credit: B-0 first (every injector in pass-through, the
first P6 flight at all), then B-1. The M5 experience says the first live use of new runner code finds
defects; a dry run finds them without consuming a gate. The approval would name simulated flight
explicitly (`gate_content` B).

**Then, each separately authorized:** the production Policy A approval, the real freeze, Gate A, Gate B,
Gate S, Gate C.

### 16.5 Gate B content dry run in simulated flight on `33f33c5a` (2026-11-14, no credit)

**Authorization:** the operator, 2026-11-14, "Authorize flight dry run", answering: "Authorize a Gate B content
dry run (B-0, then B-1) in simulated flight on development candidate 33f33c5a?" Approval
`state/p6-dry-run-approval-33f33c5a22c9-gateB-flight.json` (Gate B content, `flight` and
`simulated_flight_only` true, cap 4).

**Verified before each launch:** the manifest and the P3/P4/P5 baselines; no simulator or P6 process; no live lock;
the host on AC in Best-performance mode. The first attempt found the AC overlay reverted to "Best power
efficiency" and stopped before launching anything; the operator switched it back and the runs followed.

**The first P6 flights.** Both rows passed, each as one run (SITL, S2 world, no credit):

| Row | Fault | Result | Window (sim s) | RTF | P5 items (all allowed by a passed check) |
|---|---|---|---|---|---|
| B-0 | none, every injector pass-through | COMPLETE, `completed`; 0 P5 failures; C11, GOAL, INJECTORS, PROOF, UNSAFE passed | 12.66–128.98 | 0.998 | none |
| B-1 | F3.08, three 25 ms inference ticks in AVOIDING (shim) | COMPLETE, `supervisor_abort`; PH3, L45, LIMITS_UNINJECTED, UNSAFE, C11, INJECTORS, PROOF passed | 12.64–88.54 | 0.998 | `outcome:not_completed`, `outcome:goal_distance`, `outcome:detour_sequence` (L45), `policy:failure_ticks`, `policy:participation`, `policy:inference_p99`, `policy:inference_max` (PH3) |

B-1 detail: the shim fired at 81.74 s sim on the first AVOIDING decision; 3 timeout records in the accepted
policy ledger; detection 0.048 s (one tick) after the trigger; the abort followed within 0.10 s; the
vehicle landed through the accepted sequence. No P5 item was refused on either row. The candidate was
unchanged throughout; the lock was released; no residual process.

**What this shows:** the flight path of the runner (world selection, flight gating, the shim in flight, the
relays and witness in pass-through through a complete flight, the flight window, the evaluation and
composition for flight rows) works on the first live use. The runner-fired mechanisms in flight (relay
SIGUSR1 at an in-flight trigger, the bridge SIGINT, the load harness) have not yet been exercised; they
belong to the Gate S and C rows.

**Next (needs its own authorization):** a Gate S content dry run on this candidate (S-1, S-2′, S-3, S-4,
S-5′, in the frozen order), simulated flight only, no credit. It exercises PS at `avoiding_near` (S-1), SR at
`avoiding_first` (S-2′), the shim faults F3.04, F4.01 and F6.03, and the fail-closed L45R path (S-5′).

### 16.6 Gate S content dry run in simulated flight on `33f33c5a` (2026-11-14, no credit)

**Authorization:** the operator, 2026-11-14, "Authorize Gate S dry run". Approval
`state/p6-dry-run-approval-33f33c5a22c9-gateS-flight.json` (Gate S content, simulated flight only, cap 6).
Verified before launch: AC power, Best-performance overlay, host CPU 1 %, the manifest and baselines, no
simulator or P6 process, no lock.

**Run:** 03:36:10Z–03:49:46Z, the five rows in frozen order, one run each; result
`state/p6-dryrun-S-33f33c5a22c9-20261114T033624Z.result.json`. The candidate was unchanged throughout; the
lock was released; no residual process. RTF 0.996–0.998 on every row.

**Passed (4): the runner-fired in-flight mechanisms work.**

| Row | Fault, mechanism, trigger | What happened | P5 items (all allowed by a passed check) |
|---|---|---|---|
| S-1 | F1.01 bridge SIGINT (PS) at `avoiding_near`, fired 89.93 s | FAULT_WINDOW and F1–F4 passed; detection 0.224 s; L45 via `supervisor_abort`; 312 native headers after the stop | 8, under FAULT_WINDOW |
| S-2′ | F2.09 state relay (SR) at `avoiding_first`, fired 81.25 s | 0 of 1329 position and 0 of 2658 attitude messages relayed; perception unassociated 257 times; H35, A10, L45; detection 0.272 s | 3, under L45 |
| S-3 | F3.04 NaN scores (shim) at `avoiding_first`, 81.88 s | 3 `output_not_finite` ledger records; PH3, L45; detection 0.052 s | 5, under PH3/L45 |
| S-4 | F4.01 map loss 1.0 s (shim) at `avoiding_first`, 81.5 s | 10 snapshots published in the window, 0 used; H35, RECOVERY, GOAL; recovered; `completed` | 1, under RECOVERY |

**Failed (1): S-5′, on four P5 items. Its own checks passed:** PROOF (one mission tick delayed 60 ms),
R2 (`stale_action` abort with a valid Offboard sample, detection 0.048 s), L45R (exactly one LAND at 85.764 s,
landed 90.42 s, disarmed by 92.42 s), UNSAFE, C11, INJECTORS. Allowed as predicted: `runtime_status:…`,
`p2:timing`, `outcome:not_completed`, `outcome:goal_distance`. Refused:

1. **`cleanup:ros-controller:nonzero_exit_1`.** The accepted `main` exits 1 whenever `p4_outcome` is
   `failed`, and the accepted P4 `outcome()` returns `failed` for any run ending in P2 RECOVERY. So every
   L45R row exits 1 by design. The same finding as A-5.F4.07/F4.13 (§15.2, decision 3), for a different
   reason.
2. **`rgb_absence:snapshot_missing:window_end`.** The accepted P3 runner's snapshot thread takes the
   `window_end` RGB-absence snapshot on the COMPLETE transition only (`TRIGGERS`). A RECOVERY landing never
   reaches COMPLETE, so the snapshot does not exist. The continuous absence monitor did cover the whole
   window and beyond (148 rows, max gap 0.50 s, zero IMX214 subscribers, control subscriber present), and
   every other absence check passed.
3. **`p2_integrity_error:ValueError:estimator_reference_reset:final_alignment_lost`.** The accepted offline
   replay (`reference_evidence.classify`) feeds every received sample to the reference manager with the
   current phase, including RECOVERY, which is not in the manager's landing exemption
   (REQUEST_LAND … COMPLETE). When the vehicle landed at 90.42 s, `cs_in_air` went false at 90.57 s and the
   replayed manager rejected `final_alignment_lost`. The live controller never rejected (`rejected_count`
   0, no `reference_rejected` event): its RECOVERY tick returns before the reference check. The replay's
   `historical` branch already relabels RECOVERY as TAKEOFF for exactly this reason, but the P4/P5
   evaluators call it with `historical=False`.
4. **`mc_causal_yaw:p2_integrity_missing`.** A consequence of 3: the P2 report was never produced.

**Why these never appeared before:** no accepted P2–P5 flight row ends in P2 RECOVERY; the accepted
abort rows land through the nominal sequence (`supervisor_abort`, COMPLETE). S-5′ is the first L45R flight
evaluated by this stack. The four items are consequences of the RECOVERY landing in accepted code, not of
the fault or of P6.

**Open decision (recommendation first).** On S-5′ only, replace exactly these four items by proofs, with
the accepted evaluators and thresholds unchanged:
* item 1 by L45R plus a designed-exit proof: exit 1, `p4_outcome` `failed`, connectivity passed, final state
  RECOVERY, exactly one LAND;
* item 2 by a proof from the accepted monitor ledger: coverage from before the window start to after the
  window end, max gap within the accepted limit, zero IMX214 subscribers and the control subscriber present
  on every row (the data the P5 evaluator already reports), plus every other absence check passed;
* items 3 and 4 by a proof that the controller's own reference record shows no rejection, the replay's
  rejection reason is `final_alignment_lost`, and `cs_in_air` first went false after the LAND command and
  after `landed` was reported (the landing itself).
The alternative is to leave S-5′ failing under the accepted evaluators, which no change in P6 can satisfy.

**Also to note:** S-1's allowed items include `depth:health_not_fresh_in_window` and `performance:compute_max`
under FAULT_WINDOW, as the accepted P4 §9.1 window intends. No threshold, baseline or source changed.

### 16.7 The S-5′ decision implemented, and candidate `23d513ee` (2026-11-14)

**Decision:** adopted by the operator on 2026-11-14 ("Adopt"), as proposed in §16.6.

**Code (`c274cba`):**
* `validation/p6_safety_evaluate.py`: the `RECOVERY_LANDING` check, with `RECOVERY_LANDING_ITEMS` naming the
  four items. It requires the designed exit (exit 1, `p4_outcome` `failed`, connectivity passed, final state
  RECOVERY, one LAND); the accepted absence-monitor ledger covering the window within its gap limit with every
  other absence check passed and no RGB metadata; the controller's own reference record without a rejection,
  the replay item present, and `cs_in_air` first false after the LAND command and after `landed`; and no other
  item of the cleanup, absence, P2-integrity or MC-causal families.
* `validation/p6_evaluate.py`: S-5′'s response adds `RECOVERY_LANDING`; its exceptions add exactly the four
  items, each replaced by that check. No other row changes.
* Tests: the proof accepts the S-5′ shape and refuses each missing piece (nine negatives); the exception is
  exact and S-5′-only; the retained S-5′ dry run of §16.6 now evaluates as passed with exactly those four
  items allowed and nothing refused (its stored evaluations are unchanged).

**Offline regression on `c274cba`:**

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 153 passed, 0 skipped |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 108 passed, 45 skipped (Linux-only or retained WSL evidence; not counted as passes) |

**Development candidate `23d513ee40290b41327ce705584b4e5cf6948d69612f6b2737182fd2d294eb27`**, frozen at
`c274cba` from a clean P6 tree; verifies; P3/P4/P5 re-verified; 41 files; no dry-run approval written.

**Next (needs its own authorization):** a Gate C content dry run on `23d513ee` in simulated flight (C-1 to
C-5 in frozen order), no credit. It exercises the depth relay's burst faults in flight (C-1), isolated
inference timeouts (C-2), the adversarial policy (C-3), the compound bridge stop plus inference delay (C-4),
and the compound map loss plus the 24-worker load harness, the first in-flight use of LH (C-5). S-5′ itself
has not re-run on the new candidate; its pass is on the retained evidence only, and a complete Gate S dry
run on one unchanged candidate remains to be shown.

### 16.8 Gate C content dry run in simulated flight on `23d513ee` (2026-11-14, no credit)

**Authorization:** the operator, 2026-11-14, "Authorize Gate C". Approval
`state/p6-dry-run-approval-23d513ee4029-gateC-flight.json` (Gate C content, simulated flight only, cap 6). Verified
before launch: AC power, Best-performance overlay, host CPU 1 %, the manifest and baselines, no simulator or P6
process, no lock.

**Run:** 08:33:10Z–08:49:32Z, the five rows in frozen order, one run each; result
`state/p6-dryrun-C-23d513ee4029-20261114T083325Z.result.json`. The candidate was unchanged throughout; the lock was
released; no residual process. RTF 0.996–0.998 on every row.

**Passed (1):** C-2, F3.07 five isolated 25 ms timeouts (shim): 5 timeout ledger records, HOLD on exactly the injected
ticks, LIMITS_UNINJECTED, GOAL; completed.

**Failed (4), on three P6 defects, one host-timing abort and one Gazebo shutdown hang:**

1. **C-1, F1.03 depth bursts (DR at `avoiding_first`, 84.15 s): a P6 evaluator defect.** The relay dropped exactly
   the frames inside the three burst windows (63 frames, none relayed). The proof failed on 3 "outside" frames that
   were the first frames of the run (5.02–5.08 s sim), received by the witness before the relay's subscription had
   matched, about 80 s before the fault was armed. The proof judged every frame from boot; the accepted delivery
   checks judge the sensor window only. **Fix:** judge frames inside the accepted sensor window. Re-evaluated in
   memory on the retained evidence: proved (63 dropped inside, all 3546 outside relayed); the run passes, with the
   four `depth:` items allowed by the proof as the row defines.
2. **C-3, F3.13 adversarial policy (shim): a host-timing abort after 70 s.** The shim chose the worst allowed slot
   on all 1420 invocations from the trigger; the shield held (UNSAFE passed, 0 outside the mask) and the vehicle
   never reached the goal. At 145.5 s one control callback took 51.8 ms against the accepted 50 ms dispatch budget,
   and the accepted P2 controller failed closed (`stale_action`) and landed through RECOVERY. Median mission ticks
   were 5–6 ms throughout, as on B-0, C-2 and S-4 (which had spikes up to 37 ms); C-3's 70 s of TRANSIT gave more
   exposure. The evidence does not attribute the spike to the fault. The row's rules (S0, outcome reported) have no
   check for a P2 fail-closed landing, so its items (`runtime_status`, `p2:timing` and the four RECOVERY-landing
   items of §16.6) were refused.
3. **C-4, F8.01 bridge stop plus 25 ms inference delay: a rule-table defect and a Gazebo shutdown hang.**
   * The M4 rule table gives C-4 the timing exceptions (`policy:inference_p99`, `policy:inference_max`, replaced by
     LIMITS_UNINJECTED) but not the LIMITS_UNINJECTED check itself, so they can never apply. **Fix:** add
     LIMITS_UNINJECTED to C-4's response, as on B-1 and C-2. Re-evaluated in memory: it passes (145 uninjected
     ticks, p99 0.85 ms).
   * At cleanup Gazebo did not exit within its 15 s grace after SIGINT and was killed
     (`cleanup:gazebo:forced_kill_after_grace`, never excepted). In the other 11 flight runs it exited in
     0.44–0.64 s, with the same shutdown order (S-1 also stopped the bridge). No Gazebo-side log records the
     cause. It needs a rerun to show whether it recurs.
   * Its own checks passed: FAULT_WINDOW_F1F2, NO_INVOKE_AFTER_STALE, L45, 3 timeout records, detection 0.052 s.
4. **C-5, F8.03 map loss plus 24 load workers: a P6 runner defect.** The runner launched the load harness at
   `transit_start` (74.67 s) through `p6_node.py`, whose module list does not include `load_harness`; it printed its
   usage and exited 1 at once. No load was applied, so C-5 did not test LH; INJECTORS failed and
   `cleanup:p6-load-harness:nonzero_exit_1` was refused. The MR part passed (H35, STALE_IFF_AGE, L45, recovery).
   **Fix:** add `load_harness` to the node's modules, with a test that every module the runner launches is one the
   node accepts.

**Open decision (recommendation first):**
* **Fix the three P6 defects** (C-1 proof scope, C-4 LIMITS_UNINJECTED, C-5 node module). No accepted code and no
  threshold changes.
* **C-3 outcome:** the contract requires S0 and reports the outcome. Recommended: accept a P2 fail-closed RECOVERY
  landing as a reported C-3 outcome, checked by L45R (one LAND, landed and disarmed within 45 s), the
  RECOVERY_LANDING proof of §16.6 and S0, with the P2 failure reason reported. The alternative is to keep the rule
  and rerun C-3, which passes only if no host spike crosses the budget during a long adversarial flight.
* Then a new development candidate and a rerun of the full Gate C content on it, which also shows whether the C-4
  Gazebo hang recurs.

### 16.9 The Gate C fixes implemented, and candidate `75289deb` (2026-11-14)

**Decision:** adopted by the operator on 2026-11-14 ("Adopt both"), as proposed in §16.8.

**Code (`e3c8b69`):**
* `scripts/p6_node.py`: `load_harness` is a dispatchable module (C-5). A test requires every module the runner
  launches through `self.node` to be one the node accepts, and runs the harness's `--help` through the node.
* `validation/p6_fault_evaluate.py`: the F1.03 drop proof judges frames inside the accepted sensor window, and fails
  without one (C-1). The M4 synthetic test now carries a sensor window.
* `validation/p6_evaluate.py`: C-4 runs LIMITS_UNINJECTED (its timing exceptions' check). C-3 runs
  FAIL_CLOSED_LANDING, whose exceptions are exactly the P2 fail-closed landing's runtime status, `p2:timing` and the
  four RECOVERY-landing items.
* `validation/p6_safety_evaluate.py`: `fail_closed_landing`, recorded only when the run ends in P2 RECOVERY with a
  failure: L45R for the reported failure plus the RECOVERY_LANDING proof.
* A new invariant test: every exception's replacement check is one its row actually records (the defect class of
  C-4).

**Replayed on the retained Gate C evidence** (stored evaluations unchanged): C-1 passes; C-3 passes, reporting
`stale_action`; C-4 fails on `cleanup:gazebo:forced_kill_after_grace` alone; C-5 still fails (its harness never ran).

**Offline regression on `e3c8b69`:**

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 158 passed, 0 skipped |
| WSL | M3 offline suite runner | 87/87, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS sourced, 5 passed |
| Windows | P6 tests | 112 passed, 46 skipped (Linux-only or retained WSL evidence; not counted as passes) |

**Development candidate `75289deb16c90af2f2d11d791706f14209be0af20e41e246b638945005244ef4`**, frozen at `e3c8b69`
from a clean P6 tree; verifies; P3/P4/P5 re-verified; 41 files; no dry-run approval written.

**Next (needs its own authorization):** a rerun of the full Gate C content on `75289deb` in simulated flight (C-1 to
C-5), no credit. It is the first in-flight use of the load harness (C-5) and shows whether the C-4 Gazebo shutdown
hang recurs. Before the real freeze, a complete Gate B, S and C content pass on one unchanged candidate is still to
be shown (S-5′ has not re-run live since its rule changed).

### 16.10 Gate C content rerun in simulated flight on `75289deb` (2026-11-14, no credit)

**Authorization:** the operator, 2026-11-14, "Authorize Gate C rerun". Approval
`state/p6-dry-run-approval-75289deb16c9-gateC-flight.json` (Gate C content, simulated flight only, cap 5: one run per
row, no retry). Verified before launch: AC power, Best-performance overlay `ded574b5`, the manifest, no simulator or
P6 process, no lock.

**Run:** 10:52:15Z–11:08:27Z, the five rows in frozen order, one run each; result
`state/p6-dryrun-C-75289deb16c9-20261114T105229Z.result.json`. The candidate was unchanged throughout; the lock was
released; no residual process. RTF 0.995–1.000 over each flight. Gazebo exited in 0.44–0.52 s on every row: the C-4
shutdown hang of §16.8 did not recur (1 in 17 flight runs so far).

**Passed (4); the §16.9 C-1 and C-4 fixes hold in flight:**
* **C-1**, F1.03 depth bursts: proved on the sensor window (64 frames inside the bursts, none relayed; all 3533
  outside relayed); recovery (2 returns), GOAL; completed.
* **C-2**, F3.07: 5 timeout records, HOLD on exactly the injected ticks, LIMITS_UNINJECTED (832 ticks, p99 0.83 ms),
  GOAL; completed.
* **C-3**, F3.13: the worst allowed slot on all 2999 invocations from the trigger; UNSAFE passed (0 moves outside the
  mask); COMPLETE through a supervisor abort. There was no host spike this time, so FAIL_CLOSED_LANDING was not
  exercised.
* **C-4**, F8.01: detection 0.048 s, NO_INVOKE_AFTER_STALE, L45, FAULT_WINDOW_F1F2, 3 timeout records,
  LIMITS_UNINJECTED (145 ticks, p99 0.56 ms).

**Failed (1): C-5, F8.03 map loss 1.0 s plus 24 CPU workers.** The load harness ran in flight for the first time (the
§16.9 C-5 fix holds): 24 workers started at `transit_start` (74.508 s sim), recorded in the harness ledger.
1. **The load caused a dispatch breach, and the P2 controller aborted, which is §5.3's declared response.**
   * Under the load, the control callback rose from a 1.13 ms median (3.9 ms max) to 20.4 ms (53.8 ms max). The
     mission tick rose from 0.32 ms to 17.1 ms (43.5 ms max).
   * At 76.568 s, 2.06 s after the load started, one callback took 53.8 ms against the accepted 50 ms dispatch budget.
     The accepted P2 controller failed closed (`stale_action`) and landed through RECOVERY.
   * S0 held: UNSAFE passed, with 0 moves.
2. **The map loss never fired.** Its trigger (`avoiding_first`) came 4.3 s after TRANSIT in the §16.8 run, and here
   the abort came first. The shim never activated (0 invocations), so `proof:no_activation`, H35 (`no_stale_decision`)
   and L45 (an abort landed through COMPLETE) failed.
3. **P6 defect: the load harness does not stop gracefully.**
   * The runner stopped it with SIGINT when the stack ended. It died on an unhandled `KeyboardInterrupt` (exit by
     signal), so INJECTORS failed and `cleanup:p6-load-harness:terminated_by_SIGINT` was refused.
   * This is not specific to the abort. Its 60 s window outlives a nominal C-5 too: in the §16.8 run COMPLETE came
     55 s after `transit_start`.
   * The plan's M-LH row (§4.2) already allows the workers to be "killed in cleanup".
4. **P6 defect: §5.3 is not implemented (the C-4 defect class).**
   * The contract gives C-5 "§5.3" and the scenario expects "LOAD", but the rule table runs no LOAD check.
   * §5.3 accepts a P4 perception or P5 inference breach inside a declared load window only when its declared response
     follows, with 0 unsafe commands. The declared responses are:
     * stale snapshot → H35;
     * inference above 0.020 s → HOLD;
     * dispatch above 0.050 s → P2 abort.
5. **A load-induced loss outside §5.3: the accepted P3 raw depth recorder overflowed.**
   * Its 32-slot queue dropped 31 of 2539 frames from the evidence recording. All 31 were inside the load window
     (78.97–82.48 s, during the RECOVERY landing).
   * The adapter reported `raw_recorder_overflow` and exited 3, giving `cleanup:sensor-adapter:nonzero_exit_3`,
     `depth:adapter_result_not_passed` and `depth:raw_recorder_incomplete`.
   * Perception itself met every period: 0 missed periods, a 0.076 s maximum observation age, and complete delivery to
     the adapter.
   * §5.3 covers P4 perception and P5 inference limits, not P3 evidence recording.
6. **Other items:**
   * `policy:inference_p99`: 5.9 ms against the accepted 5 ms. All 47 invocations were inside the load window; the
     maximum was 5.9 ms against 10 ms, and none came near the 0.020 s HOLD threshold.
   * The P2 fail-closed landing's items: the runtime status, `p2:timing` and the four RECOVERY-landing items of §16.6.

**Open decision (recommendation first):**
* **Fix the two P6 defects.** No accepted code and no threshold change.
  * The harness stops its workers on SIGINT or SIGTERM, records `cleared` with `stopped_early`, and exits 0.
  * C-5 runs a LOAD check implementing §5.3: every breach in the window must be matched by its declared response. The
    P5 inference-statistics items are allowed only with that check.
* **How C-5 handles 24 workers on this host, where the P2 abort comes about 2 s into the load:**
  * **Recommended: start the load at the map-loss trigger,** keeping 24 workers and 60 s.
    * The map loss then happens under load from its first moment. In §16.8 the stale decision came 0.144 s after the
      trigger; here the first breach came 2.06 s after the load started.
    * After H35, accept either recovery or A10, or the §5.3 P2 abort checked by FAIL_CLOSED_LANDING.
    * Residual risk: a dispatch breach before the first stale decision fails H35.
  * *Alternative:* keep `transit_start` and accept the §5.3 abort before the map loss as a reported outcome. Map loss
    under load would then likely never be shown on this host.
  * *Alternative:* 8 workers, F6.01's lower level. Probably no breach and no recorder overflow, so no new exception is
    needed, but the load is weaker and the frozen F8.03 row text changes.
* **The recorder overflow at 24 workers is outside §5.3 and needs its own decision.**
  * Recommended, narrowing an accepted P3 evidence check inside the declared load window only: replace the three
    adapter items with a proof. The proof requires every lost frame inside the load window, the adapter meeting every
    processing period, and the recorder drained and fsynced with no error. The loss is reported.
  * Otherwise, C-5 at 24 workers fails whenever the recorder overflows.
* Then a new development candidate. Before the real freeze, one unchanged candidate must still pass Gate B, S and C
  content.

### 16.11 The C-5 load decision implemented, and candidate `0db5c60a` (2026-11-14)

**Decision:** adopted by the operator on 2026-11-14 ("Adopt as recommended"), as proposed in §16.10. It is recorded as
`m6_c5_load_decision_approved` in the scenarios config and in contract §8.4 and §10.

**Code (`4c76455`). No accepted P2–P5 source changes, and no threshold changes:**
* **`injection/load_harness.py`: the harness stops cleanly.**
  * A SIGINT or SIGTERM stop ends the workers, records `cleared` with `stopped_early` naming the signal, and exits 0.
  * The workers ignore SIGINT, so a stop sent to the whole group reaches them only through the harness.
  * Ledger lines are written whole.
* **`configs/p6_scenarios.yaml`: C-5's load starts at the map-loss trigger.** `load_start` changed from `transit_start` to
  `avoiding_first`. The workers (24), window (60 s) and drop window (1.0 s) are unchanged. The expected outcome now
  includes the §5.3 P2 abort landed through RECOVERY.
* **`validation/p6_safety_evaluate.py`: three new checks.**
  * **`LOAD` implements §5.3.** The window comes from the harness's own ledger, on the shared monotonic clock. The
    declared responses are:
    * a P2 failure must be `stale_action`, inside the window, in a callback over the accepted 0.050 s budget;
    * every inference above 0.020 s must hold;
    * every stale onset must hover and hold, a `snapshot_stale` onset within H35.
    
    The P5 inference statistics must hold outside the window.
  * **`LOAD_RECORDER` handles the recorder overflow.** It requires:
    * every lost frame inside the window, with its written neighbours inside it too;
    * the recorder drained and fsynced with no error, and its counts agree;
    * 0 missed processing periods;
    * the accepted P5 depth-stream evaluation failed on its two recorder items alone;
    * the adapter's designed exit 3.
  * **`LANDING` passes on the path the run took:** L45, or FAIL_CLOSED_LANDING. The RECOVERY_LANDING proof now takes
    the items another passing proof of the same row explains.
* **`validation/p6_evaluate.py`: C-5's rules.**
  * Response: H35, STALE_IFF_AGE, LOAD, LOAD_RECORDER, FAIL_CLOSED_LANDING, LANDING, UNSAFE.
  * Exceptions are exact items only. The inference statistics need LOAD, the dispatch abort and its landing items need
    FAIL_CLOSED_LANDING, and the three recorder items need LOAD_RECORDER.

**Tests:**
* The harness, run through `p6_node.py` in its own group, stopped by a SIGINT to the group and by a SIGTERM to the
  harness: exit 0, `cleared` recorded, and no worker survives.
* LOAD: a positive case and each negative (wrong P2 failure, abort outside the window, no measured overrun, a load not
  as declared, an unheld slow inference, a p99 breach outside the window, a late or moving stale onset, no
  activation).
* LOAD_RECORDER: a positive case and each negative, and nothing is recorded without an overflow.
* C-5's composition, and both landing paths.
* The C-5 runner trigger, which now fires on the map loss's own decision.

**Replayed on the retained `75289deb` C-5 run** (non-strict, because its declaration predates the decision):
* LOAD passes, matching the 53.8 ms overrun to the P2 abort.
* LOAD_RECORDER passes, with all 31 lost frames inside the window.
* The fail-closed landing's L45R passes. Its RECOVERY_LANDING proof finds exactly one unexplained item, the old
  harness's `terminated_by_SIGINT`.
* The run still fails, as it should: no H35, and the harness item.

**Offline regression on `4c76455`:**

| Host | Suite | Result |
|---|---|---|
| WSL | P6 tests | 165 passed, 0 skipped |
| WSL | M3 offline suite runner | 89/89, 32/32 required, 12/12 rehearsals |
| WSL | P5 tests | 199 passed; the ROS harness ran separately with ROS and `ros_ws` sourced, 5 passed |
| Windows | P6 tests | 116 passed, 49 skipped (Linux-only or retained WSL evidence; not counted as passes) |

WSL's git, which has no `autocrlf`, lists some accepted P5 files as modified. That is line endings only: the diff
ignoring CRs is empty, Windows git shows them clean, the last commit touching P5 is `40a109b`, and the P5 manifest
verifies.

**Development candidate `0db5c60a20b3c458f8b65a8d508b57a5b99c0b6c477d7ca7c25cfcefc975bcdc`**
(`state/p6-dev-candidate-d25831d.json`), frozen at `d25831d` from a clean P6 tree. Its P6 sources are those of
`4c76455`; only the plan record changed in between. It verifies from any directory, and P3/P4/P5 re-verify. It binds
41 files, faults `97fc5eef…` and scenarios `4e06ab00…`. No dry-run approval has been written.

**Superseded freeze `06d08d98`** (`state/p6-dev-candidate-4c76455.json`, same sources and specs):
* It was frozen with a relative Policy A approval path, so it verifies only from `~/uav_autonomy`. The launchers verify
  from other directories.
* Earlier candidates record the absolute path, and so does `0db5c60a`.
* `06d08d98` was never approved or run, and the file is kept.

**Next (needs its own authorization):**
* A Gate C content rerun on `0db5c60a` in simulated flight, no credit. It is the first run of the load from the map
  loss, and of the new LOAD, LOAD_RECORDER and LANDING checks on live evidence.
* Before the real freeze, a complete Gate B, S and C content pass on one unchanged candidate is still to be shown.
  S-5′ has not re-run live since its rule changed.

### 16.12 Gate C content rerun in simulated flight on `0db5c60a` (2026-11-14, no credit)

**Authorization:** the operator, 2026-11-14, "Authorize Gate C rerun". Approval
`state/p6-dry-run-approval-0db5c60a20b3-gateC-flight.json` (Gate C content, simulated flight only, cap 5: one run per
row, no retry; all 5 used). Verified before launch: AC power, Best-performance overlay `ded574b5`, host CPU 5 %, the
manifest, no simulator or P6 process, no lock.

**Run:** 12:23:05Z–12:39:42Z, the five rows in frozen order, one run each; result
`state/p6-dryrun-C-0db5c60a20b3-20261114T122319Z.result.json`. The candidate was unchanged throughout; the lock was
released; no residual process. RTF 0.987–1.000 over each flight; Gazebo exited in 0.42–0.56 s on every row.

**Passed (4):** C-1, C-2, C-3 (COMPLETE through a supervisor abort) and C-4, as in §16.10.

**Failed (1): C-5, F8.03, on a gap in the adopted LOAD_RECORDER proof.** The §16.11 changes otherwise worked live:
* **The map loss ran under load.** It fired at `avoiding_first` (78.46 s), and the load started on the same decision.
  H35 passed (first stale 0.148 s after the trigger), the vehicle recovered, and STALE_IFF_AGE and UNSAFE passed.
* **The harness stopped cleanly.** Cleanup's SIGINT ended the workers, the harness recorded `cleared` with
  `stopped_early` SIGINT, and INJECTORS passed.
* **LOAD passed.** At 85.66 s, 7.2 s into the load, one callback took 52.2 ms against the 50 ms budget. The accepted
  P2 controller aborted (`stale_action`) and landed through RECOVERY, and LOAD matched that abort to the overrun.
  `policy:inference_p99` was allowed by LOAD (every in-window inference was under 20 ms; outside the window p99 was
  0.69 ms).
* **L45R passed:** one LAND, landed and disarmed in 18.95 s.
* **What failed: LOAD_RECORDER.**
  * The accepted P3 raw recorder overflowed again, losing 43 frames, all inside the load window (82.95–87.34 s). The
    recorder drained and fsynced with no error, the exit was the designed 3, and there were 0 missed periods.
  * This time the overflow began before the sensor window ended at RECOVERY (85.66 s). In the `75289deb` run it came
    after the window.
  * The accepted P3 stream evaluator judges delivery by the recorder's own index, so 32 native frames the adapter had
    received count as undelivered. That added `depth:source_to_ros_delivery_loss` (recorded fraction 0.9856) and
    `depth:ros_receipts_not_in_native_ledger`.
  * The adopted proof requires exactly the two recorder items, so it failed. FAIL_CLOSED_LANDING and LANDING then
    failed too, because the adapter's exit 3 was left unexplained.
* **Cause.** All 32 undelivered native frames lie in the 26 gaps between written receipts. Each gap holds exactly as many
  native frames as overflowed receipt ids (one gap straddles the window end at 85.66 s). No written receipt lacks a
  native frame. The adapter received every frame (receipt ids are assigned on receipt); only the recorder failed to
  write them.
* **In memory, with no file changed:** extending LOAD_RECORDER to also explain those two delivery items, only under
  this one-to-one match, makes the run pass with every item allowed by its own check.

**A plain retry on `0db5c60a` would probably fail the same way.** The recorder overflowed in both 24-worker flights (31
and 43 frames), and whether the two delivery items appear depends only on whether the overflow starts before the P2
abort ends the sensor window.

**Open decision (recommendation first):**
* **Recommended: extend LOAD_RECORDER** to replace `depth:source_to_ros_delivery_loss` and
  `depth:ros_receipts_not_in_native_ledger` too, under all of the following:
  * every native frame counted as undelivered lies in a gap between written receipts;
  * each gap holds no more native frames than overflowed receipt ids;
  * every overflowed receipt is inside the load window;
  * no written receipt lacks a native frame;
  * every other depth check passed.
  
  This narrows two more accepted P3 delivery checks, inside the declared load window only, to frames the adapter
  received but could not record. Then a new development candidate and a C-5 rerun.
* *Alternative:* 8 workers (§16.10).
* *Alternative:* keep the rule. C-5 at 24 workers then fails whenever the overflow starts inside the sensor window.
