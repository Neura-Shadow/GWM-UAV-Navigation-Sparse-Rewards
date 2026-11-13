# v3-SIM P6 M3 — offline fault validation: report

**Status: M3 complete (2026-11-07), authorized by the operator ("Authorize M3").** Offline only. No
simulator, PX4, QGC or ROS node ran, and no P2–P5 file, manifest or evidence changed. Recorded inputs
were read, read-only, from one retained accepted flight: P5 `2becfbbb` Gate C S2,
`20261106T132442Z-p5-flight-S2-184a5a5e`.

* **Contract:** [P6 contract](v3_sim_p6_contract.md) §8.1 row 4, the offline suite that Gate A binds.
* **Matrix:** [fault matrix](v3_sim_p6_fault_matrix.md).
* **Plan record:** [plan](v3_sim_p6_plan.md) §13.

## 1. Result

**The bound run.**

| | |
|---|---|
| Runner | `simulation/px4_gazebo_p6/validation/p6_offline_suite.py` |
| Commit | `60093c8` |
| Interpreter | the controller's (`/usr/bin/python3` 3.12.3, NumPy 1.26.4, `OPENBLAS_NUM_THREADS=1`) |
| Recorded run summary SHA-256 | `1ab57114e925c416…` |
| Tests | **87 passed, 0 failed, 0 skipped** (276 s) |
| Required offline (R-O) items | **31 / 31**, each covered by named tests that all passed |
| Flight-row rehearsals | **12 / 12** (B-0, B-1, S-1, S-2′, S-3, S-4, S-5′, C-1 … C-5) |

The result is bound to the SHA-256 of every P6 source, spec and test file:
`docs/evidence/v3_sim_p6_m3_offline_suite_60093c8.json` (SHA-256 `988da4341fff18cd…`).

**Windows.** The same test files give 37 passed and 14 skipped in the offline file (the skips need the
recorded flight). With the M2 files that is 71 passed and 16 skipped.

**Replay fidelity.** Every recorded-depth corruption starts from a clean replay of the accepted P4
perception. On the recorded frames, at the recorded poses, the replay reproduces every recorded
perception grid **bit for bit**, and this is checked on every step before a corruption is measured. One
lesson came out of building it: the adapter's recorder and perception count frame receipts separately,
so the only common key is the acquisition stamp.

## 2. Required items (R-O)

| Fault | Proven against the accepted code |
|---|---|
| Stale rules (F1.01/04/06/07/11, F2.04/09, F4.02/06) | For each of the 6 stale reasons: the supervisor HOVERs with yaw frozen by `t_last` + 0.35 s (adapter health missing: + 2.0 + 0.10 s), holds the target, and ABORTs with `stale_inputs:<reason>` 10 ± 0.10 s later |
| F1.06 | Recorded frames: exactly 5 duplicate, 5 backwards and 5 missing-stamp rejections in the P3 pipeline; it never accepts an out-of-order frame |
| F1.07, F1.12 | Recorded frames: 10 `dimension_mismatch`, 12 `unexpected_raw_frame_id` |
| F1.08 | 300 recorded snapshots: a NaN band never creates FREE or OCCUPIED space (376 FREE cells lost to unknown) |
| F1.10 | 90 % NaN only removes information; no cell becomes FREE because of it |
| F1.11 | A scaled `fx` invalidates the P3 calibration lock, so health is not fresh and the stale rule applies |
| F2.04 (detection) | A reset bracket makes association `reset_epoch_bracket`; the supervisor HOVERs `estimator_reset_since_last_usable_frame` |
| F2.05, F2.06, F2.07, F5.07, F5.10 | App-side state faults raise `state_time_backwards`, `clock_epoch_mismatch`, `invalid_estimate`, `stale_state:vehicle_status` (only after 1.5 s sim) and `vehicle_health_invalid` |
| F2.09 | On the recorded PX4 messages, withholding pose turns every later frame into `pending_newer_state`, while the frames before the cut still match |
| F3.03 | A one-byte schema change refuses start with `p5_model_schema_differs_from_frozen_hash`, before any node |
| F3.05, F3.06, F3.11, F3.12, F3.09 | The accepted ranker classifies them as `output_shape`, `selection_outside_mask`, `exception:RuntimeError`, `input_not_finite`, and `timeout` (with `inference_s` ≥ 0.060) |
| F3.09, F6.03, F7.05 | A 61 ms callback trips the P2 dispatch guard (`stale_action`). `Mission.abort` sends exactly one LAND, and a second abort sends none |
| F4.11 | Health staleness and map staleness are each detected on their own |
| F5.03, F5.06 | A rewritten ACK gives `command_ack_denied` on the next mission tick (176 before arming; 21 when landing) |
| F5.05 | A wrong nav_state gives `offboard_armed_state_lost`; `abort` then sends no LAND (PX4 Return follows, M0 Finding D) |
| F7.02 | Alternating failures never reach `policy_unavailable` (counter at most 1 over 400 ticks) |
| F7.04, F7.06 | 60 fuzzed episodes plus all 7 accepted `2becfbbb` flights: every supervisor transition is in the contract table, and ABORT is absorbing with its reason fixed |
| F7.07 | 2,000 random decisions: the target moves only through `apply_action`, never while held, at most 0.2 m/s × dt; yaw at most 10°/s × dt |
| F8.04 | The policy-failure rule precedes the transition table: NORMAL, NORMAL, then ABORT `policy_unavailable`, never OBSTACLE_DETECTED |
| F8.05 | A controller started against an armed vehicle refuses (`restart_reject_armed`) |

## 3. Findings that refine the specification

1. **F5.02, the short controller stall, is now settled from source** (the M0 open item).
   * In rclpy Jazzy's executor, ready timers run before ready subscriptions within one wait set, one
     callback per `spin_once`.
   * So the first control tick after SIGCONT uses the pre-stop state *and* the pre-stop simulation
     time (`/clock` is still queued). Its ages look fresh, and no P2 check can fire on it.
   * The queues then drain one message per subscription per pass. `observation_gap` needs two
     consecutive ticks' samples more than 0.2 s apart, so **a stall below `COM_OF_LOSS_T` most likely
     goes undetected in flight**. The post-run P2 timing evaluation is what would see the publication
     gap.
   * The X-F5.02 characterization flight can now be authorized to confirm it.
2. **C-4 precedence, in the rig.** With bridge stop plus 25 ms inference at the same trigger, the
   three policy timeouts (0.15 s) came before staleness (≥ 0.25 s). The run ABORTed with
   `policy_unavailable`, and the policy was never invoked after a stale tick. Both orders are allowed
   by the frozen C-4 row.
3. **F1.10 coverage loss.** The cumulative map keeps earlier observations, so the vehicle continues on
   previously observed free space. HOVER and `hover_timeout` come only where it would need unobserved
   space. The SITL characterization should report where that happens rather than expect an immediate
   HOVER.
4. **F3.06, a minor note.** The accepted re-check `allowed[choice]` would accept a negative index,
   through Python wrap-around.
   * It is unreachable in practice: `policy_choice` returns only allowed indices.
   * Such a value would raise `KeyError` outside the ranker's `try`, which still fails closed (P2
     `RECOVERY`).
   * The P6 injector now never produces an invented index.

## 4. Characterization: measured risk (never gated)

**Hazard metric.** The metric is the number of **collision-valid P4 candidates whose segment passes
within `R_body` (0.39 m) of the true obstacle**, under the accepted L2 test with the traversed mask
the mission keeps. Each such candidate is a move the shield would allow into the real obstacle.

FREE cells inside an obstacle's footprint alone are not a hazard. In the accepted flight all 16
footprint cells of the S2 pillar were marked FREE, by grazing rays and the 4 m free cap, before its
front face was hit. P4's safety comes from the 1.0 m inflation around observed OCCUPIED cells.

| Fault | Measured (recorded flight unless noted) | Risk statement |
|---|---|---|
| **F2.04** map reuse after an estimator reset (synthetic world, real MAP_SCAN and path) | XY shift 0.5 m: **0** unsafe candidates. Shift 1.0 m: **3** unsafe candidates before re-observation, **0** after the first fresh frame | **High, conditional.** Above about the inflation margin, a reset shifts the real obstacle into space the reused map shows as free. The supervisor holds until the next fresh frame, and that frame re-marks any obstacle it sees. The residual risk is obstacles outside the field of view at that moment. No SITL use before a P5 remedy is considered |
| **F1.09** +inf over columns 240–400 around first observation (snapshots 676–736) | 11 FREE cells created that the clean map never had; **0** unsafe candidates at the recorded pose and at 5 approach points | **Medium.** Corruption does create false free space. Here the pillar was still observed OCCUPIED through other columns and frames, and the inflation contained it. A corruption covering the whole obstacle for longer is not bounded by anything in the accepted code |
| **F1.05** frozen frames, same window | 36 FREE cells created, 71 OCCUPIED cells misplaced; **0** unsafe candidates | **Medium.** Undetected, and it distorts the map. Contained here by earlier and later true observations |
| **F4.03** snapshot reordering | 9 regressions in 1,328 snapshots, at most 1 OCCUPIED cell lost, each for about 0.1 s | **Low.** It stays inside the 0.25 s age limit |
| **F4.08** forged usable snapshot on `/gwm/p4/local_map` | An all-FREE forged snapshot is accepted; no check exists | **High (architecture).** Any second publisher can supply the map. Ground and offline only; never flown |
| **F4.10** perception restart (synthetic) | 16 kept traversed cells fall inside the pillar's footprint; 0 unsafe candidates (L2 also needs the ±1 m corridor observed or traversed) | **Low-medium.** The misaligned traversed mask makes unobserved cells count as free. Runners never restart processes |
| **F2.02** app-side position jump +0.5 m | 0 of 120 recorded decisions changed (P4 plans from the target) | It affects only the supervisor's clearance check and the P2 geofence |
| **F2.03** app-side yaw +20° | 61 of 120 decisions changed (the L4 ±25° window and YAW_TO) | **Medium.** It moves where translation is permitted relative to the camera's view |
| **F2.08** ENU/NED swap | 120 of 120 decisions changed | Gross; the geofence is the only backstop |
| **F1.08** NaN band | 0 FREE or OCCUPIED cells created | None: unknown is never free |

## 5. Limits of M3

* **Rehearsals are not transport tests.** Depth, pose and process faults are applied at the mission's
  inputs. The DR, SR and PS transport paths, and the ROS node wrappers, are first exercised in M5.
* **The load part of C-5 cannot be rehearsed offline.**
* **Recorded-input checks use one accepted flight** (Gate C S2). M2 also checked pass-through on Gate S
  S5.
* **Characterization numbers are single measurements, not rates.**

## 6. Next

**M4** is the independent evaluators: `p6_fault_evaluate`, `p6_safety_evaluate`, `p6_timing_evaluate`
and `p6_coverage_report`. They need their own authorization.
