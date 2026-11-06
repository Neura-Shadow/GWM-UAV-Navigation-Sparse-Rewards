# v3-SIM P6 M0 — baseline and contract audit

**Status: M0 complete (2026-11-06), authorized by the operator ("Authorize M0").**

**Read-only.** This audit used:
* source reading;
* the accepted and historical ULogs (read with `pyulog`);
* `p*_manifest.py verify`.

No simulator, PX4, QGC or ROS node was started, and no P2–P5 file, manifest or evidence was
changed. M0 resolves the open audit items of [plan](v3_sim_p6_plan.md) §7. It also raises two
decisions (§7), which do not change P2–P5.

## 1. Baseline (re-verified)

* HEAD `4273215`, two local docs commits ahead of `origin` (`1f06f39`).
* P3 `1e205fec…`, P4 `c110bd30…` and P5
  `2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632` all verify.
* PX4 source `~/uav_autonomy/upstream/PX4-Autopilot` at `v1.17.0`
  (`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`). This equals `ver_sw` in the accepted ULogs.

## 2. Finding A — no accepted node accepts command-line remaps

**Evidence:**
* **Controllers.** `p4_control` and `p5_control` call `rclpy.init(args=[])`.
* **Sensor adapter and `p4_perception`.** Both call `rclpy.init(args=None)`, but first parse
  `sys.argv` with a strict `argparse` `parse_args()`. A `--ros-args -r …` remap is rejected by
  argparse before ROS sees it.
* **Topic sources.** Every topic comes from configuration:
  * `p3_sensor_bridge.yaml` `ros_topic_name`;
  * `p3_sensors.yaml` `image_topic` and `info_topic`;
  * `p4_avoidance.yaml` `topics`;
  * the `--px4-prefix` argument of the adapter and perception (default `/px4_71`).

**Consequence for D1, which assumed relays "where M0 confirms remap support":**

| Mechanism | Feasible without editing frozen files? | How |
|---|---|---|
| Depth relay (DR) | **yes** | The P6 runner writes the run-local `p3_sensor_bridge.yaml` with the two `ros_topic_name` values changed to `/gwm/p6/raw/front_depth/{image_raw,camera_info}`; the relay republishes on the accepted names. The bridge is the stock `ros_gz_bridge`, not frozen code; the adapter and perception are unchanged. This is a **declared run-local config deviation** of 2 keys |
| Map/health relay (MR) | only with a second run-local `p4_avoidance.yaml` for perception | **Recommended instead:** map and health input faults in the controller shim. The P6 entry point wraps `P5Mission.on_snapshot`, `on_adapter_health` and `tick` (to release delayed snapshots). No topology change, no extra DDS hop, and exact control relative to ticks. Independent proof: perception's own `p4/local_map.jsonl` versus the controller's `snapshot_acquisition_sim_s` in `p4/decisions.jsonl` |
| Pose faults toward perception (F2.09) | **yes** | Perception's existing `--px4-prefix` argument pointed at `/gwm/p6/px4_71`; a state relay republishes the six `fmu/out` state topics (delay, drop, stamp faults). An accepted argument, not a config edit. The adapter and controller keep `/px4_71` |
| Controller shim (CS), process signals (PS), load harness (LH) | yes | as planned |

## 3. Finding B — coverage of the P4 "stale clock" and "calibration mismatch" fixtures

* The P4 contract §8 Gate A(4) names four fault fixtures: adapter stopped, bridge stopped, stale
  clock and calibration mismatch.
* The accepted P4 Gate A `c110bd30…` ran **two live**: `adapter_stop` and `bridge_stop` (runner
  `FAULTS`, `p4_validation.yaml` gate rows).
* **Offline coverage only:**
  * All seven supervisor stale reasons, including `calibration_mismatch` and
    `adapter_health_missing`, are covered by `tests/test_px4_gazebo_p4_planner.py`
    (`test_stale_inputs_hover_with_yaw_frozen_then_abort_after_ten_seconds`), with synthetic
    inputs.
  * `clock_stalled` is covered by `tests/test_px4_gazebo_p2_contract.py`.
  * Both test files are outside the manifests.
* **Conclusion.** The two fixtures were never exercised live. The accepted P4 documents are left
  unchanged; this note records the gap. P6 Gate A F1.11 (calibration) and F4.13 (clock stall)
  would be their first live coverage.

## 4. Finding C — the evaluators do not check run-local configs against the frozen ones

* **How runs are built.** The P3/P4/P5 runners copy the frozen configs into each run.
  `p3_runner.finalize` records a SHA-256 of every run file, and `p3_evaluate.common` checks those
  digests. That proves **post-run integrity**, not equality with the frozen configs.
* **The gap.** No P3/P4/P5 evaluator compares a run-local config with its frozen original. It was
  harmless for P3–P5, whose runners copy unchanged.
* **New P6 rule.** It matters for P6 (Finding A), so the proposed rule is **C11**: the P6 evaluator
  requires every run-local config to equal its frozen original byte for byte, except keys declared
  in the run's fault declaration. Each declared key's before and after values are recorded.

## 5. Finding D — what PX4 v1.17 does when the controller stops

### 5.1 Source trace (pinned v1.17.0)

1. `commander/failsafe/failsafe.cpp` `checkModeFallback`. With `offboard_control_signal_lost` set
   while Offboard is the intended mode, `fromOffboardLossActParam(COM_OBL_RC_ACT = 0)` gives
   `FallbackPosCtrl`.
2. Because `manual_control_signal_lost` is set, the action is replaced by
   `fromNavDllOrRclActParam(NAV_RCL_ACT = 2)`, which is **`RTL`**.
3. **No delay.** The `COM_FAIL_ACT_T` (5.0 s) delayed Hold is armed only for actions added through
   `CHECK_FAILSAFE` (`framework.cpp` 351–354, 495). The offboard-loss result comes from the
   mode-fallback path, so RTL starts immediately.
4. **Return altitude.** `navigator/rtl.cpp` `calculate_return_alt_from_cone_half_angle`, with
   `RTL_CONE_ANG` 45°, `RTL_MIN_DIST` 10 m, `NAV_ACC_RAD` 2 m and `RTL_RETURN_ALT` 30 m. At
   horizontal distance d from home it returns:

   | d | Return altitude above home |
   |---|---|
   | d ≤ 2 m | `max(2·d, current)` |
   | 2 m < d ≤ 10 m | `max(d, 4 m, current)` |

   The vehicle climbs in place, flies straight home at that altitude, then lands.
5. **Landing in progress is unaffected.** `AUTO_LAND` does not require the offboard signal. A
   controller loss after an accepted LAND does not trigger this path.
6. **GCS link loss** (`NAV_DLL_ACT` 2, `COM_DLL_EXCEPT` 0) goes through `CHECK_FAILSAFE`. After
   `COM_DL_LOSS_T` 10 s it enters a 5 s delayed Hold, then RTL. This confirms D7.

The parameter values are from an accepted P5 Gate C ULog (`13_24_56.ulg`):
`MPC_Z_V_AUTO_UP` 3.0, `MPC_Z_V_AUTO_DN` 1.5, `MPC_XY_CRUISE` 5.0, `MPC_LAND_SPEED` 0.7,
`COM_DISARM_LAND` 2.0. `manual_control_signal_lost` is **1 for the whole flight** (no RC or
joystick). `gcs_connection_lost` was 1 only before QGC connected (until 3.54 s, disarmed).

### 5.2 Empirical confirmation on this build

**Scan.** All 141 retained ULogs were scanned (read-only). Exactly one has `vehicle_status.failsafe`
set: the historical P2 run `20260911T171449Z-p2-flight-802425c9`, which failed with
`estimator_reference_reset` and entered `RECOVERY` with no LAND.

**Timeline:**

| Event | Time | Detail |
|---|---|---|
| last controller `offboard_control_mode` | 23.46 s | |
| `offboard_control_signal_lost` | 24.51 s | +1.05 s, matching `COM_OF_LOSS_T` 1.0 |
| `nav_state` 5 (`AUTO_RTL`), failsafe 1 | 24.52 s | no Hold state in between |
| vehicle at about 1.7 m, at home | | |
| landed | about 28 s | |
| disarmed | 30.81 s | 6.3 s after failsafe onset |

**Conclusion.** Every fault that stops app setpoints while in Offboard ends in an immediate PX4
**Return**: a hang, a stop, a kill, a stopped DDS agent, or P2 `RECOVERY` without a valid sample.

### 5.3 Consequences for P6

* **PX4 does land.** The Return lands at home and disarms. That is the PX4-managed outcome, and it
  is not a terminal airborne hold.
* **The P4 3.0 m ceiling cannot hold.** For d > 1.5 m the Return altitude is above 3.0 m, by PX4
  design.
* **Obstacle risk.** The return path is straight, at `max(d, 4 m)`. The scenario boxes are 4 m tall
  (top at 4.0 m), and the collision envelope extends 0.23 m below `base_link`. A Return started
  just past an obstacle (4.0 m < d < 4.4 m) crosses its top with a vertical gap of
  `d − 4.23 m` < 0.20 m. The truth clearance limit could then be violated by PX4's own behaviour.

## 6. Proposed §5.2 values (PENDING operator approval)

| Quantity | Proposed limit | Derivation |
|---|---|---|
| Failsafe onset | ULog `offboard_control_signal_lost` within **1.0 + 0.2 s** of the last controller `offboard_control_mode` | `COM_OF_LOSS_T` 1.0; empirical 1.05 s; 0.2 s covers the commander cycle and ULog sampling |
| Action | `nav_state` 5 (`AUTO_RTL`) within **0.1 s** of onset; no other mode before landing | source §5.1; empirical 0.01 s |
| Landed and disarmed | within **45 s wall** of onset, ULog only | reuses P2 `recovery_wall_s` 45 s (empirical 6.3 s from 1.7 m at home) |
| Safety | 0 collisions, truth clearance ≥ 0.20 m | unchanged P4 limits; **no exemption** |
| P4 3.0 m ceiling | **cannot hold** in a PX4-managed window | decision D4a |

## 7. Decisions raised by M0 (recommendation first)

* **D1a — injection details within D1:**
  * **Recommended:**
    * the depth relay through the declared 2-key run-local bridge config deviation, checked under
      rule C11;
    * map and health input faults in the controller shim instead of a map relay;
    * pose faults toward perception through its `--px4-prefix` argument and a state relay.
  * *Alternative:* a second run-local `p4_avoidance.yaml`, so a map relay can sit between
    perception and controller.
* **D4a — PX4-managed scenarios in Gate S:**
  * **Recommended:**
    * S-2 and S-5 become separately authorized characterization flights, with full truth-safety
      evaluation and the §6 values reported. They are never counted as passes, because they would
      need a ceiling exemption and they carry the §5.3 obstacle-top risk.
    * Gate S uses app-managed replacements:
      * **S-2′ = F2.09.** Pose withheld from perception, so its frames become unusable and no new
        usable snapshot arrives. That gives HOVER, then `ABORT stale_inputs:snapshot_stale` after
        10 s, then LAND. `snapshot_stale` precedes `unusable_frames` in `stale_reason`, so it is
        the reason reported.
      * **S-5′ = F6.03.** A 60 ms mission-tick stall makes the P2 dispatch guard abort with a valid
        Offboard sample, so exactly one LAND follows.
  * *Alternative:* keep S-2 and S-5 required, with a narrow exemption. Inside the PX4-managed window
    only, the ceiling becomes "≤ the §5.1 Return altitude + 0.5 m". Clearance and collisions stay
    unexempted.
  * The earlier replacement S-5′ = F5.02 (a 0.3 s stall) is withdrawn. After SIGCONT, the first
    tick may see `stale_state` (no LAND, so PX4 Return) or `observation_gap` (LAND), depending on
    the callback order. M3 must settle which.
* **C11 — run-local config equality** (§4). It is recommended for inclusion in the contract.

The next approvals after these decisions are:
1. the P6 contract as a whole, with §5.1, §5.2 (this section) and §5.4;
2. M1, freezing the scenario specifications.
