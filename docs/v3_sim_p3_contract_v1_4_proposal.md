# P3 contract `p3-acceptance-v1.4` — proposal (EKF reset brackets, PX4 landing gimbal self-commands)

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** Nothing was changed, built, frozen or
run for this proposal. P3 remains INCOMPLETE, qualification 0/3.

Basis: HEAD `5ceb6c7` (v1.3 plus the snapshotter fix) and the retained v1.3 Gate B run
`20261025T123921Z-p3-flight-624dc8ae` (acceptance report section Z). Pinned PX4 v1.17.0
`d6f12ad1` source, read-only.

v1.4 changes exactly two evaluation rules. Everything in v1.3 stays: RGB consumer by scope
with verified absence in flight; the D3D12 renderer and shutdown profile; sensors and
assets; P2 controller, mission and freshness (2.0 s wall `vehicle_land_detected`); every
numeric limit; MC causality at 1e-5 rad; graceful cleanup; gate sequencing.

## Evidence from v1.3 Gate B (the only full flight so far)

The flight completed (`COMPLETE`): land_detected wall p50 1.000 s, p95 1.003 s, maximum
1.012 s; RTF 0.997; P2 and MC passed. Evaluation failed on:
1. the snapshotter defect, already fixed in `5ceb6c7`;
2. one depth observation rejected as `attitude:reset_epoch_bracket`;
3. six `vehicle_self_command_on_gcs_link` frames.

Items 2 and 3 are addressed here.

## Rule 1 — reset-epoch brackets at a recorded PX4 EKF reset: counted and bounded, not fatal

**Facts.** PX4 reset its attitude estimate twice: `quat_reset_counter` 0→1 at
`timestamp_sample` 6.572 s (before the window) and 1→2 at **22.184 s** (during TAKEOFF), each
with a coincident `heading_reset_counter` step at 6.564 s and 22.172 s. Of 931 in-window
observations, 930 matched. The one at acquisition 22.180 s had its position part matched,
while its attitude bracket straddled the 22.184 s reset. The adapter correctly refused it
(no interpolation across an epoch). Attitude samples are at most 0.020 s apart and position
samples at most 0.028 s. Processed depth observations are at least 0.064 s apart. **A
single reset bracket can therefore contain at most one processed observation.**

**Proposed rule (replaces "every window observation must be `matched`").** A window
observation is acceptable if it is `matched`, or if its status is exactly
`attitude:reset_epoch_bracket` or `position:reset_epoch_bracket` and **all** of these hold:
1. **Recorded bracket.** The adapter records the bracketing samples
   `before {timestamp_us, epoch}` and `after {timestamp_us, epoch}` (new output, adapter
   change). The evaluator checks `before ≤ acquisition ≤ after` and
   `after − before ≤ state_mismatch_s` (0.05 s, the existing limit; no new number).
2. **Exactly one PX4 reset.** Between `before.epoch` and `after.epoch` exactly one epoch key
   changes, by +1 for a counter (attitude: `quat_reset_counter`; position: one of
   `xy_reset_counter`, `z_reset_counter`, `heading_reset_counter`, or a `ref_timestamp`
   change).
3. **Independent corroboration.** The controller's own `ros-events.jsonl` records the same
   counter step on the same PX4 topic with a `timestamp_sample` inside
   `[before, after]`.
4. **Bound.** At most **1** bracketed observation per (state, corroborated reset event), and
   the total does not exceed the number of corroborated reset events in the window.
5. **Accounting.** Bracketed observations are excluded from the matched-delta statistics,
   reported individually (acquisition time, reset counter step, bracket), and counted
   (`reset_bracketed_observations`).

Any other non-matched status still fails: `estimate_invalid`, ambiguous ties outside the
v1.1 rule, `no_state_within_tolerance`, `older_than_history`, `pending_newer_state`,
`missing_history`, an uncorroborated or multi-key bracket, or more than one per reset.
`state_mismatch_s`, `observation_age_sim_s` and every depth limit are unchanged.

## Rule 2 — PX4 landing gimbal-neutral self-commands: explicitly enumerated

**Source (pinned PX4 v1.17.0, read-only).** `Land::on_activation()` (`land.cpp:75`) calls
`activate_set_gimbal_neutral_timer()`. After **250 ms**,
`Navigator::neutralize_gimbal_if_control_activated()` (`navigator_main.cpp:1642–1652`) publishes
`acquire_gimbal_control()` → `set_gimbal_neutral()` → `release_gimbal_control()`. These are
`vehicle_command`s with source = target system = own id and target component 0. The MAVLink
`COMMAND_LONG` stream (`streams/COMMAND_LONG.hpp:85–93`) forwards any non-internal command,
and these are not internal because the component does not match. `MavlinkCommandSender`
(`mavlink_command_sender.h:121–122`) retransmits an unacknowledged command up to
`RETRIES = 3` times, every `TIMEOUT_US = 500000`. These are PX4-native frames, not QGC and
not the P2 controller (which publishes over ROS 2 only).

**Observed (Gate B v1.3), all from 72/1 to 72/0, message 76 `COMMAND_LONG`, in phase
`VERIFY_LANDED_AND_DISARMED`:**

| # | Command | params (7 × float32) | `seven_float32_le_hex` | count | t − `REQUEST_LAND` |
|---|---|---|---|---|---|
| a | 1001 `DO_GIMBAL_MANAGER_CONFIGURE` (acquire) | 72, 1, −1, −1, 0, 0, 0 | `000090420000803f000080bf000080bf000000000000000000000000` | 4 | 0.279, 1.286, 1.794, 2.303 s |
| b | 1000 `DO_GIMBAL_MANAGER_PITCHYAW` (neutral) | NaN, NaN, NaN, NaN, 2 (`FLAGS_NEUTRAL`), 0, 0 | `0000c07f0000c07f0000c07f0000c07f000000400000000000000000` | 1 | 0.280 s |
| c | 1001 `DO_GIMBAL_MANAGER_CONFIGURE` (release) | −3, −3, −1, −1, 0, 0, 0 | `000040c0000040c0000080bf000080bf000000000000000000000000` | 1 | 0.280 s |

**Proposed rule.** Add to policy **A** (in `p3_native_command_policy.json`; `strict_zero` is
unchanged) a `vehicle_landing_self_commands` allowance. A vehicle self-command frame is
allowed only if **all** of these hold, and every other vehicle self-command remains
`vehicle_self_command_on_gcs_link`:
1. message 76, source **72/1**, target **72/0**, frame CRC-valid in the capture;
2. command and payload exactly one of rows a, b or c (byte-exact `seven_float32_le_hex`);
3. inside the landing phase: monotonic time from the controller's `REQUEST_LAND` transition
   to its `COMPLETE` transition (from `controller-result.json`); never before landing;
4. per payload at most **1 + RETRIES = 4** frames, so at most **12** in total, for the single
   landing of the unchanged P2 mission;
5. counted and reported separately (`allowed_vehicle_landing_self_commands`) from the GCS
   allowance (1 × 401, 34 × 512), which is unchanged.

Because the policy A object changes, its canonical SHA-256 changes, so **policy A must be
re-approved** for the v1.4 candidate. The existing approval hash `ea307bf5…` does not cover
it.

## Exact proposed changes

| File | Change |
|---|---|
| `configs/p3_validation.yaml` | `contract: p3-acceptance-v1.4`; `contract_revision` text; new `"state_association": {"reset_epoch_bracket": {"allowed_statuses": ["attitude:reset_epoch_bracket", "position:reset_epoch_bracket"], "max_per_reset_event": 1, "bracket_span_limit_key": "state_mismatch_s", "corroboration": "controller_ros_events_reset_counter_in_bracket"}}`. Every numeric key unchanged. |
| `configs/p3_native_command_policy.json` | Policy A gains `vehicle_landing_self_commands`: source [72,1], target [72,0], message 76, phase `REQUEST_LAND`→`COMPLETE`, `max_frames_per_payload` 4, payload rows a/b/c. `strict_zero` unchanged. |
| `gwm_sensor_adapter/association.py` | On `reset_epoch_bracket`, also return `before`/`after` `{timestamp_us, epoch}` (the decision itself is unchanged). The sensor package is rebuilt, giving a new package hash. |
| `validation/p3_stream_eval.py` | Association check implements rule 1 (needs `ros-events.jsonl` for corroboration). |
| `validation/p3_mavlink.py` / `p3_evaluate.py` | `evaluate_policy` implements rule 2 using the controller's landing-phase window. |
| tests | Rule 1 cases: one bracketed frame with corroboration passes; uncorroborated, a 0.051 s span, a multi-key epoch change, two frames for one reset, or `estimate_invalid` fails. Rule 2 cases: rows a/b/c in the landing phase pass; before `REQUEST_LAND`, wrong payload, wrong target component, or a fifth copy of a payload fails; `strict_zero` still fails them. Every numeric limit and the P2 config stay identical. |

## What v1.4 does not change or claim

* No threshold, freshness, sensor, renderer, shutdown, MC, cleanup or RGB rule changes.
  Flight RGB stays depth-only and absence-verified.
* Gate B timing has passed once, on the v1.3 candidate, and that run is not counted. A v1.4
  candidate still needs a fresh Gate A, Gate B and three Gate C flights.
* Resets at other times, or other PX4 self-commands, are not covered and still fail.

## Sequence after approval

1. **Operator approval of this exact v1.4 text** (rules 1 and 2 as written).
2. Implement, rebuild the sensor package, add the tests, regress (Linux and Windows), commit.
3. **Policy A re-approval** for the new policy hash and commit; freeze and verify the
   candidate.
4. Separate authorizations for Gate A, then Gate B, then Gate C (three consecutive), each
   stopping at the first failure.
