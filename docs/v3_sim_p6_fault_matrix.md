# v3-SIM P6 fault matrix — APPROVED with the P6 contract (2026-11-06; nothing implemented)

Companion to the [P6 plan](v3_sim_p6_plan.md) and the draft [P6 contract](v3_sim_p6_contract.md).

**How this matrix was built:**
* Detection mechanisms and expected behaviours were read from the accepted P2–P5 source.
* Items marked *(verify M0/M3)* rest on a code reading that still needs its offline confirmation.
* No item has been executed.

**Thresholds:**
* Numeric limits are reused P2–P5 limits, except the new thresholds of contract §5. Those were approved
  with the contract on 2026-11-06.

## Legend

**Tier:** O = offline (recorded or synthetic inputs, frozen code); G = ground (live stack, vehicle
on the ground, controller in ground-diagnostic mode, supervisor and policy response checked by
replaying the recorded ground inputs through the frozen supervisor); S = SITL flight.

**Class:**
* **R-A / R-B / R-S / R-C:** required in Gate A, B, S or C.
* **R-O:** required in the M3 offline suite, which Gate A binds by hash.
* **C:** characterization. Reported with every safety check, but not gated.
* **D:** deferred; needs a decision.
* **cov:** covered by accepted P4/P5 evidence, reused.

**Injection mechanisms** (plan §4.2):
* DR: depth relay, through the declared run-local bridge config (C11)
* MR: map/health input faults. After M0 (D1a, approved 2026-11-06) they are implemented inside
  the controller shim, not as a DDS relay.
* SR: state relay to perception, through its `--px4-prefix` argument
* CS: controller shim
* PS: process signal
* LH: load harness

**Expected-behaviour and limit codes:**

| Code | Meaning (source) |
|---|---|
| **S0** | 0 collisions; truth clearance ≥ 0.20 m; 0 unsafe commands (contract §6.3); Policy A content unchanged; one PX4-input owner; graceful cleanup (P3/P4/P5) |
| **H35** | supervisor `HOVER`, yaw frozen, by `t_last` + 0.35 s sim (`t_last` = acquisition of the last usable snapshot). While stale, target motion ≤ 0.01 m and yaw ≤ 0.5° (P4 §9.1 F3) |
| **A10** | `ABORT stale_inputs:<reason>` 10.0 ± 0.10 s sim after staleness began (P4 §9.1 F3) |
| **L45** | exactly one LAND (21) accepted; `COMPLETE` within 45 s wall of it; ULog landed and disarmed (P4 rev 5) |
| **PH3** | HOLD on each failing tick (target and yaw unchanged), `ABORT policy_unavailable` on the 3rd consecutive failing tick, i.e. 0.10 s sim after the first at 20 Hz; then L45 (P5 §5) |
| **R2** | P2 `RECOVERY` with the stated reason; one LAND only if the validated sample is armed, nav 14 and airborne (P2 `Mission.abort`) |
| **PX** | PX4-managed: the controller stops publishing, and PX4 enters an immediate **Return** (`AUTO_RTL`, no 5 s Hold; M0 Finding D). It returns at `max(d, 4 m)`, lands at home and disarms. Proposed limits: onset ≤ 1.2 s after the last `offboard_control_mode`; `AUTO_RTL` ≤ 0.1 s after onset; landed and disarmed ≤ 45 s wall; S0 unexempted (contract §5.2, approved 2026-11-06; reported, not gated). It always breaches the P4 3.0 m ceiling, so it is characterization under D4a (approved 2026-11-06) |
| **H12** | adapter health leaves `fresh` within 1.2 s wall (P3 `interruption_detect_wall_s`) |
| **REF** | P3 refusal before any ROS node or publisher exists; exit ≠ 0 accepted only in that declared scope (P5 rev 2 pattern) |
| **INJ** | the independent injection proof required for that mechanism (plan §4.3) |

Columns "Detection limit" and "Safety limit" use these codes. Evidence always includes the fault
ledger, the witness (DR, MR), `ros-events.jsonl`, `p4/decisions.jsonl`, `p5/policy-ledger.jsonl`,
the rosbag, the ULog, truth poses and contacts, and the process table. Only the extra or decisive
channels are listed per row.

## F1 — Depth sensor

| ID | Fault (prior coverage) | Injection | Preconditions | Detection (existing mechanism) | Expected behaviour | Detection limit | Safety limit | Decisive evidence | Evaluation rule | Tier | Class | Risk / cleanup |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F1.01 | Complete interruption, bridge stopped (P4/P5 S6 at 2.0 m in NORMAL; P4 Gate A ground) | PS SIGINT `sensor-bridge` | P6: in `AVOIDING`, ≤ 1.5 m from the S2 pillar | health `receipt_stall_wall`; supervisor `snapshot_stale` | HOVER, then ABORT, then LAND | H12, H35, A10 | S0, L45 | native source probe continues; no ROS receipt after `t_inj` + 0.12 s | P4 §9.1 F1–F4 applied unchanged at the new trigger point | G, S | R-A (ground), R-S (S-1) | low; bridge exits 0 under the P3 graceful-stop limit |
| F1.02 | Adapter stopped (P4 Gate A ground only) | PS SIGINT `sensor-adapter` | TRANSIT | `adapter_health_missing` after 2.0 s sim (perception keeps mapping) | HOVER, then ABORT | HOVER ≤ last health `sim_s` + 2.0 + 0.10 s; A10 | S0, L45 | adapter exit record; last health record | the replayed `stale_reason` equals the recorded reason on every tick | G, S | R-A; S: C | low |
| F1.03 | Intermittent loss (none) | DR drops bursts of 0.1, 0.5 and 1.5 s | TRANSIT, S3 | snapshot age > 0.25 s | HOVER while stale; resume only on a fresh matched snapshot; mission continues | for every tick, recorded stale ⇔ replayed age > 0.25 s (no slack) | S0; goal reached if no stale run lasts 10 s | witness: dropped receipt IDs versus native headers | supervisor-legality replay over the whole fault window | O, S | R-C (C-1) | low |
| F1.04 | Excessive latency (none) | DR delays every frame by 0.30 s, stamps kept | TRANSIT | snapshot age > 0.25 s; P3 observation age | HOVER, then ABORT | H35, A10 | S0, L45 | witness publish − stamp ≥ 0.30 s | as F1.03 | G, S | R-A; S: C | low |
| F1.05 | Frozen frames, content repeated with advancing stamps (none) | DR | TRANSIT | **none** (P3 rejects only duplicate or backwards stamps) | undetected; old content projected with the new pose | — | S0 evaluated and reported | witness content hash repeats | characterization: map error versus truth, clearance | O, then G | C | **high**; never flown before an O-tier risk statement |
| F1.06 | Stamp faults: duplicate, backwards, missing, +0.5 s future (none) | DR | ground / TRANSIT | `FrameRejected` (`duplicate_…`, `…_backwards`, `missing_…`); future gives association `pending_newer_state`, then 3 unusable frames | rejected frames, then HOVER via `snapshot_stale` or `unusable_frames` | H35 | S0 | adapter `rejected` counters; perception `unusable_reasons` | counters equal injected counts; replay legality | O, G | R-O, R-A | low |
| F1.07 | Dimension, encoding, step or payload faults (none) | DR | ground | `FrameRejected` (`dimension_mismatch`, …) | as F1.06 | H35 (replay) | S0 | rejected counters | as F1.06 | O, G | R-O, R-A (`dimension_mismatch`) | low |
| F1.08 | NaN region or partial corruption in the flight band (none) | DR | TRANSIT | NaN band gives an unknown ray; unknown is never free | no motion into the corrupted sector; HOVER if no progress candidate | — | S0 | grid cells versus truth | no cell under a corrupted ray turns FREE because of it | O, S | R-O; S: C | low |
| F1.09 | +inf or ≥ 19.1 m corruption in the band (none) | DR | before the obstacle is first observed | **none**: classified `no_return`, so FREE to 4.0 m (`free_cap_m`) | cells over a never-observed obstacle may turn FREE until a true hit raises them | — | S0 evaluated | grid versus truth; first hit time | characterization: FREE-over-obstacle cell count, duration, clearance | O | C | **high**; O only until approved |
| F1.10 | Sudden coverage loss, ≥ 90 % of pixels NaN (none) | DR | TRANSIT | no progress candidates | HOVER, SCAN where permitted; `hover_timeout` 20 s ABORT | ABORT 20 s ± 0.10 after HOVER | S0, L45 | grid unknown fraction | supervisor-legality replay | O, S | R-O; S: C | low |
| F1.11 | Calibration identity change (named in P4 §8 A(4); coverage to check in M0) | DR on `camera_info` | ground / TRANSIT | `calibration_mismatch` or `calibration_unavailable` | HOVER, then ABORT | H35, A10 | S0 | camera_info witness | reason equality | O, G | R-O, R-A | low |
| F1.12 | Raw `frame_id` mismatch (none) | DR | ground | `unexpected_raw_frame_id` | as F1.06 | H35 (replay) | S0 | counters | as F1.06 | O | R-O | low |

## F2 — Localization and estimator (CS faults are app-side: PX4's real estimate is unchanged)

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F2.01 | Stale `vehicle_local_position` to the controller, 0.5 s (none) | CS withholds updates | NORMAL | `stale_state:vehicle_local_position` (0.25 s sim) | R2 with no LAND (no valid sample), then PX (Return) | P2 failure ≤ 0.25 + 0.05 s sim after the last update; PX limits | S0 unexempted; ceiling breach expected | bag (true messages) versus shim ledger; ULog failsafe and nav_state | P2 reason; ULog failsafe onset and outcome | O, S | C (D4a, approved); before M0 it was R-S (S-2) | **medium**; Return passes close over obstacle tops |
| F2.02 | Position jump +0.5 m, app-side (none) | CS | TRANSIT | none, unless geofence | wrong planning context; PX4 tracks the commanded target | — | S0 evaluated | shim ledger; truth | characterization | O | C | medium |
| F2.03 | Yaw discontinuity 20°, app-side (none) | CS | TRANSIT | possibly the P2 reference manager *(verify M3)* | L4 or L5 gating on a wrong yaw | — | S0 evaluated | shim ledger | characterization | O | C | medium |
| F2.04 | Estimator reset with counters (P4/P5 S7, frame rejection only) | CS (counters plus a shift); O: recorded frames | TRANSIT | P2 reference manager; P3 `reset_epoch_bracket`; supervisor `estimator_reset_since_last_usable_frame` | HOVER until a fresh matched frame; **then the pre-reset grid is reused, with no invalidation** | H35 | S0 | grid frame versus truth after the reset | O: extends S7 to the full perception, map and supervisor path; measures stale-geometry reuse | O | R-O (detection), C (map reuse) | **highest**; no SITL before M3 |
| F2.05 | State stamp backwards or non-finite (none) | CS | TRANSIT | `state_time_backwards`, `invalid_timestamp` | R2 | same tick | S0 | ledger | reason equality | O | R-O | low |
| F2.06 | Future stamp > 0.1 s (none) | CS | TRANSIT | `clock_epoch_mismatch` | R2 | same tick | S0 | ledger | reason equality | O | R-O | low |
| F2.07 | `xy_valid` false or estimator flags (none) | CS | TRANSIT | `invalid_estimate`, `estimator_failsafe_flags` | R2 with no LAND, then PX | same tick | S0; PX limits reported (§5.2) | ULog | reason; PX outcome | O, S | R-O; S: C | medium |
| F2.08 | ENU/NED frame mismatch (none) | CS | TRANSIT | none, unless geofence | — | — | S0 evaluated | — | characterization | O | C | medium |
| F2.09 | Pose-to-depth association failure, pose withheld from perception (none) | SR: perception `--px4-prefix /gwm/p6/px4_71` (M0 Finding A) | TRANSIT | `association:*` unusable frames; then `snapshot_stale` (first in `stale_reason` order) | HOVER, then `ABORT stale_inputs:snapshot_stale`, then LAND | H35, A10 | S0, L45 | perception `unusable_reasons`; witness: true versus relayed state | replay | O, G, S | R-O, R-A; R-S (S-2′, D4a approved) | low; app-managed |
| F2.10 | PX4-native estimator or GPS failure (none) | PX4 `failure` / MAV_CMD 420 (`SYS_FAILURE_EN` 1 observed) | — | PX4 estimator flags, then P2 checks | — | — | — | ULog | — | — | D (D5) | needs a command outside Policy A |

## F3 — Policy and inference (in-process; the policy has no separate worker)

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F3.01 | Checkpoint missing at start (P5 offline) | run-local model path absent | start | `checkpoint_missing` | REF | before any node | no publisher created | controller log, started artifacts | P5 rev 2 refusal rule | G | R-A | low |
| F3.02 | Checkpoint SHA mismatch (P5 Gate A live) | mismatched file | start | `checkpoint_sha256_mismatch` | REF | as above | as above | as above | as above | G | cov; re-run in R-A | low |
| F3.03 | Schema, architecture or untrained (P5 offline) | run-local schema | start | `p5_model_schema_differs_from_frozen_hash`, … | REF | as above | as above | as above | as above | O, G | R-O, R-A (schema) | low |
| F3.04 | Non-finite scores in flight (P5 offline replay) | CS: NaN on allowed slots for 3 ticks | `AVOIDING` | `output_not_finite` | PH3, then L45 | PH3 | S0, L45 | ledger (accepted code) | ledger and decisions replay | O, S | R-S (S-3) | low |
| F3.05 | Output shape (P5 offline) | CS | AVOIDING | `output_shape` | PH3 | PH3 | S0 | ledger | as F3.04 | O | R-O | low |
| F3.06 | Selection outside the mask (P5 offline) | CS around `policy_choice` | AVOIDING | `selection_outside_mask` | PH3 | PH3 | S0 | ledger | as F3.04 | O | R-O | low |
| F3.07 | Isolated slow inference, 25 ms on single ticks, ≥ 5 s apart (none) | CS sleep | TRANSIT, S4 | `timeout` | HOLD that tick; counter resets; mission continues | each injected tick gives `timeout` HOLD, and no ABORT | S0; inference p99/max limits on **all non-injected ticks** | ledger `inference_s` (accepted clock); timing trace | exclusion only of declared tick IDs | O, S | R-C (C-2) | low |
| F3.08 | Repeated slow inference, 3 consecutive 25 ms ticks (P5 offline, one tick) | CS | `AVOIDING`, S2 | `timeout` | PH3, then L45 | PH3 | S0, L45 | ledger; trace | as F3.04 | O, S | R-B (B-1 smoke) | low |
| F3.09 | One inference of 60 ms, above the dispatch budget (none) | CS | TRANSIT | ledger `timeout` **and** P2 dispatch guard (0.050 s) | R2, one LAND (valid Offboard sample) | same tick | S0, L45 | trace span, P2 abort reason | precedence check: both records present | O, S | R-O; S: C | low |
| F3.10 | Policy hang, 3.0 s blocking (none) | CS | NORMAL | no tick completes; PX4 offboard loss at 1.0 s; afterwards the guard trips and nav ≠ 14 | PX | ULog failsafe onset ≤ 1.2 s (§5.2) | S0; PX limits reported (§5.2) | ULog; trace gap | PX outcome | O, S | C (D4a) | **high** |
| F3.11 | Exception in the forward pass (P5 class) | CS raises | AVOIDING | `exception:RuntimeError` | PH3 | PH3 | S0 | ledger | as F3.04 | O | R-O | low |
| F3.12 | Non-finite features (P5 class) | CS | AVOIDING | `input_not_finite` | PH3 | PH3 | S0 | ledger | as F3.04 | O | R-O | low |
| F3.13 | Valid but adversarial choice: the worst allowed slot every tick (none) | CS around `policy_choice` | TRANSIT, S2 | **none, by design** (in the mask) | shield-limited behaviour; goal or deadline ABORT | — | **S0 required**; outcome reported | ledger, decisions, truth | safety only; mission outcome reported, not gated | O, S | R-C (C-3) | medium; tests the shield, not the policy |
| F3.14 | Policy "worker" crash = controller crash | see F5.09 | | | | | | | | | — | |

## F4 — ROS 2 / DDS communication

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F4.01 | `/gwm/p4/local_map` loss for 1.0 s, then restored (none) | MR | `AVOIDING`, S3 | `snapshot_stale` | H35; resume only on a fresh matched snapshot; goal | H35; legality replay | S0; no stale run ≥ 10 s, so no ABORT | witness versus perception ledger | replay over the window; recovery recorded as `HOVER → AVOIDING` on a fresh snapshot | O, S | R-S (S-4) | low |
| F4.02 | Map delay 0.3 s (none) | MR | TRANSIT | age > 0.25 s | HOVER, then ABORT | H35, A10 | S0, L45 | witness | replay | G, S | R-A (ground replay); S: C | low |
| F4.03 | Reordering: two consecutive snapshots swapped (none) | MR | TRANSIT | **none**: `on_snapshot` keeps the newest received | an older grid in use for ≤ 1 cycle | — | S0 evaluated | witness order versus decisions' `snapshot_acquisition_sim_s` | characterization: any OCCUPIED → FREE regression seen by the controller | O | C | **high-medium** |
| F4.04 | Burst delivery: hold 0.2 s, then release (none) | MR | TRANSIT | age rule | as the age rule; the last received wins | legality replay | S0 | witness | replay | O, G | C | low |
| F4.05 | Perception process lost (none) | PS SIGINT `p4_perception` | TRANSIT | `snapshot_stale` (health stays fresh) | HOVER, then ABORT | H35, A10 | S0, L45 | process table | P3 cleanup classification shows only the declared process exited | S | C | low |
| F4.06 | Adapter health topic lost only (none) | MR | ground / TRANSIT | `adapter_health_missing` (2.0 s sim) | HOVER, then ABORT | as F1.02 | S0 | witness | replay | G | R-A | low |
| F4.07 | Competing PX4-input publisher, advertise-only (P2/P4 rule, never live-tested) | foreign node advertises `/px4_71/fmu/in/trajectory_setpoint` and never publishes | ground; S: NORMAL | `competing_control_publisher` at the next graph check | ground: diagnostic fails closed; S: R2 with one LAND | ≤ 1.0 s wall + graph-query time after advertisement | S0; 0 foreign setpoints in the ULog | `ros-graph.json`; ULog setpoint count versus controller sends | ownership proof | G, S | R-A; S: C | low (no message is ever published) |
| F4.08 | Rogue publisher on `/gwm/p4/local_map` (none) | a second publisher with a fabricated usable snapshot | ground only | **none**: internal topics are not graph-checked or authenticated | accepted as input | — | — | — | characterization of the architecture gap | O, G | C | **high**; never flown |
| F4.09 | QoS incompatibility (none) | relay with incompatible QoS | ground | not reachable for the controller's BEST_EFFORT/VOLATILE subscriptions; a RELIABLE probe receives 0 | — | — | — | witness probe | documented N/A with ground proof | G | C | low |
| F4.10 | Perception restarted in flight (none) | PS kill and restart | — | **none**: new grid anchored at the in-flight pose, same shape; traversed mask kept | misaligned traversed cells | — | — | — | characterization; runners never restart processes | O | C | medium |
| F4.11 | Health versus map freshness disagreement (P4 rules) | MR | TRANSIT | independent checks | HOVER on either | replay | S0 | — | replay | O | R-O | low |
| F4.12 | DDS agent stopped (none) | PS SIGINT `MicroXRCEAgent` | NORMAL | `stale_state` gives R2 (no LAND); PX4 offboard loss | PX | as F2.01 | S0; PX limits reported (§5.2) | ULog | PX outcome | S | C (D4a) | **high** |
| F4.13 | `/clock` stall (named in P4 §8 A(4)) | PS SIGSTOP clock bridge (bounded) | ground | `clock_stalled` (1.0 s wall) | ground: fail closed | ≤ 1.0 s wall + 1 tick | S0 | trace | reason equality | G | R-A | low |

## F5 — PX4 Offboard and flight control

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F5.01 | Offboard setpoint interruption: controller SIGSTOP 1.5 s (none) | PS SIGSTOP/SIGCONT `p5_control` | NORMAL, S1 | PX4 offboard loss (1.0 s); after SIGCONT `offboard_armed_state_lost` | PX (Return), then R2 (no LAND) | PX limits | S0 unexempted; ceiling breach expected | ULog nav_state, failsafe, land detected, armed | PX outcome verified only from the ULog | S | C (D4a, approved); before M0 it was R-S (S-5) | **medium-high** |
| F5.02 | Short controller stall 0.3 s (none) | PS SIGSTOP 0.3 s | NORMAL | on resume, **either** `stale_state` (giving R2 with no LAND, then PX) **or** `observation_gap` (giving R2 with one LAND), depending on the callback order after SIGCONT (M0 §7) | to be settled in M3 | — | S0 | trace gap | precedence | O, S | C | low |
| F5.03 | Mode rejection: ACK of 176 rewritten to DENIED (none) | CS on `Mission.ack` | pre-arm (vehicle on the ground) | `command_ack_denied` | R2 before arming | same tick | no arming | ULog: PX4 accepted it in reality | app-view-only fault, declared | O, S (pre-arm) | R-O; S: C | low |
| F5.04 | LAND ACK dropped (none) | CS | after ABORT | ACK deadline 3.0 s sim | R2; **PX4 still lands** (it accepted LAND) | deadline | landed and disarmed **from the ULog** | ULog | "landed" claimed only from the ULog | S | C | low |
| F5.05 | Wrong nav_state reported, app-side (none) | CS | TRANSIT | `offboard_armed_state_lost` | R2 with no LAND; controller stops; PX | same tick | S0; PX limits reported (§5.2) | ULog | PX outcome | O | R-O | medium |
| F5.06 | LAND rejected (none natively) | CS (ACK) | ABORT | `command_ack_denied` | R2 | same tick | ULog truth | ULog | app view only | O | R-O | low |
| F5.07 | Stale `vehicle_status` (none) | CS | TRANSIT | `stale_state:vehicle_status` (1.5 s sim / 2.0 s wall) | R2 | limit + 1 tick | S0 | ledger | reason | O | R-O | low |
| F5.08 | Unexpected disarm (none) | PX4 command | — | — | — | — | — | — | — | — | D (D5) | command outside Policy A |
| F5.09 | Controller killed (none) | PS SIGKILL `p5_control` | NORMAL | PX4 offboard loss; no app response | PX; no restart (runner never relaunches; P2 refuses when armed) | ULog onset ≤ 1.2 s (§5.2) | S0; PX limits reported (§5.2) | ULog | PX outcome; no second controller ever starts | S | C (D4a) | **high** |
| F5.10 | `vehicle_status.failsafe` true, app-side (none) | CS | TRANSIT | `vehicle_health_invalid` | R2 with no LAND, then PX | same tick | PX limits reported (§5.2) | ULog | reason | O | R-O | medium |
| F5.11 | Commander or estimator transitions | merged into F5.05, F5.10, F2.07 | | | | | | | | | — | |
| F5.12 | GCS link loss: QGC stopped (none) | PS SIGTERM `qgc-monitor` | NORMAL | PX4 data-link loss after 10 s; `NAV_DLL_ACT` 2 = Return *(verify M0)* | Return climbs toward `RTL_RETURN_ALT` 30 m, above the 3.0 m P4 ceiling | — | geofence violation expected | ULog | — | O (source analysis) | D (D7) | **high**; never flown before D7 |

## F6 — Timing and computation (no host or WSL setting changes)

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F6.01 | CPU oversubscription: 8 or 24 busy workers (P5 recovery work offline) | LH | TRANSIT | existing freshness, dispatch and stale rules | limits hold, **or** a declared safe response (HOVER, R2 with LAND); never silent | — | S0 | `resources.jsonl`, schedstat | every breached limit is matched by a safe response inside the load window (contract §5.3) | O, S | R-C (inside C-5), C sweep | medium |
| F6.02 | WSL scheduling stall | represented by F5.02 and F6.01 | | | | | | | | | D | no host control |
| F6.03 | Control-loop deadline violation: mission tick +60 ms | CS | NORMAL | dispatch guard (the sample was validated earlier in the same tick) | R2, exactly one LAND | same tick | S0, L45 | trace | as F3.09 | O, S | R-O; R-S (S-5′, D4a approved) | low; app-managed |
| F6.04 | Inference spikes | = F3.07 | | | | | | | | | — | |
| F6.05 | Perception delay under load | LH pinned to perception's CPUs *(affinity only; verify allowed)* | TRANSIT | snapshot latency, then age | HOVER if age > 0.25 s | replay | S0 | perception compute trace | replay | S | C | medium |
| F6.06 | Long perception callbacks | as F6.05 | | | | | | | | | C | |
| F6.07 | Queue backlog | = F4.04 | | | | | | | | | — | |
| F6.08 | Memory pressure: worker at 25 % of free RAM | LH | ground | none expected | no OOM of owned processes | — | owned processes alive | `resources.jsonl`, `dmesg` | characterization | G | C | medium (OOM risk) |
| F6.09 | Logging overhead: a disk writer | LH | ground | P3 recorder overflow 0 | recorder complete | P3 limits | — | recorder | P3 checks | G | C | low |
| F6.10 | Real-time-factor degradation | via F6.01 | | sim-time limits unchanged; wall limits tighten in effect | | | | RTF trace | reported | S | C | |

## F7 — Safety supervisor

| ID | Fault (prior) | Injection | Preconditions | Detection | Expected | Detection limit | Safety limit | Decisive evidence | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| F7.01 | No safe progress candidate (P4/P5 S5) | world S5 | | | `safe_refusal` | | | accepted evidence | reuse | S | cov | |
| F7.02 | Alternating policy failures, fail/ok (none) | CS on every 2nd tick | TRANSIT | counter resets on each success | never `policy_unavailable`; ends by goal or the 150 s transit deadline | — | S0 | ledger | O: property; S: characterization | O, S | R-O; S: C | low; documents the gap |
| F7.03 | Yaw oscillation without progress (known P5 limitation, 4 held-out layouts) | one held-out layout as a new declared world | TRANSIT | `transit_deadline` (150 s) or `hover_timeout` | ABORT, then L45 | ABORT at its rule time ± 0.10 s | S0, L45 | decisions | characterization of the accepted limitation | O, S | C | medium; new world asset |
| F7.04 | Invalid transitions | synthetic and recorded inputs | | | only contract §4 transitions | — | — | — | exhaustive check of `PolicySupervisor` against the table | O | R-O | low |
| F7.05 | Conflicting recovery commands (abort while landing, double abort) | synthetic | | P2 `max_command_attempts` 1; `abort` is a no-op once failed | at most one LAND | — | — | — | property | O | R-O | low |
| F7.06 | Repeated ABORT | synthetic | | ABORT absorbing | reason fixed at the first ABORT | — | — | — | property | O | R-O | low |
| F7.07 | Stale target positions | synthetic | | target moves only via `apply_action`; frozen on HOLD | — | — | — | — | property | O | R-O | low |
| F7.08 | Persistent blocked path | = F7.01 | | | | | | | | | cov | |
| F7.09 | Unsafe recovery after a fault clears | via F1.03, F4.01 | | fresh, matched, usable snapshot; fresh health; same calibration; no reset; valid candidate | motion only after all hold | replay | S0 | | revalidation proof per recovery event; F2.04 is the known exception | O, S | R-C, R-S (through F1.03, F4.01) | |

## F8 — Combined faults (bounded; precedence from plan §6)

| ID | Faults | Injection | Preconditions | Expected (highest-precedence rule) | Limits | Evaluation | Tier | Class | Risk |
|---|---|---|---|---|---|---|---|---|---|
| F8.01 | Bridge stop + 25 ms inference delay on every tick after `t_inj` | PS + CS | AVOIDING, S2 | staleness first: H35, A10, L45; **0 policy records after stale onset** (policy not invoked) | H35, A10, L45, S0 | precedence plus F1.01 rules | O, S | R-C (C-4) | low |
| F8.02 | Estimator reset + stale map | O replay | | = F2.04 | — | characterization | O | C | highest |
| F8.03 | Map loss 1.0 s + 24 CPU workers | MR + LH | AVOIDING, S4 | H35, then recovery or A10; any limit breach has a safe response | S0 | F4.01 rules + contract §5.3 | O, S | R-C (C-5) | medium |
| F8.04 | 3 consecutive policy timeouts on the `OBSTACLE_DETECTED` tick | CS | | the policy rule precedes the transition table: state stays, PH3 | PH3 | precedence | O | R-O | low |
| F8.05 | Controller restarted while armed + stale state | synthetic | | `restart_reject_armed` | — | property | O | R-O | low |
| F8.06 | NaN corruption + map delay 0.2 s | DR + MR | TRANSIT | unknown sector, then age > 0.25 s, then HOVER | S0 | replay | O, G | C | low |

## Counts

* **Fault IDs:** 86 (F1 12, F2 10, F3 14, F4 13, F5 12, F6 10, F7 9, F8 6), including the merged
  and alias rows.
* **Required:**

  | Group | Count | Content |
  |---|---|---|
  | R-O | 30 items, plus an offline rehearsal of each of the 12 flight scenarios | the offline suite, bound by Gate A |
  | R-A | 15 live ground runs | 2 readiness runs + 13 fault fixtures (contract §8.1) |
  | R-B | 2 flights | B-0 pass-through, B-1 F3.08 |
  | R-S | 5 flights | S-1 F1.01, S-2′ F2.09, S-3 F3.04, S-4 F4.01, S-5′ F6.03 |
  | R-C | 5 flights | C-1 F1.03, C-2 F3.07, C-3 F3.13, C-4 F8.01, C-5 F8.03 |

  That is **12 SITL flights**. Under D4a (approved 2026-11-06), Gate S is S-1, S-2′ (F2.09), S-3,
  S-4 and S-5′ (F6.03) (contract §8.3). The PX4-managed F2.01 and F5.01 become separately authorized
  characterization flights. The flight count is unchanged.
* **Characterization:** reported every time, never gated. It needs its own authorization per
  live run.
* **Deferred:** F2.10, F5.08, F5.12 and F6.02.
