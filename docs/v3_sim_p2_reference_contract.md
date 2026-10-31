# P2 estimator reference contracts

## P2-R2 source-backed yaw ownership (policy v3)

This section is frozen before the R2 diagnostic. It changes initialization
yaw semantics explicitly; the R1 sections below retain their original failure
and analysis. PX4 remains `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, px4_msgs
`86d8239e962f6939e05c3737784f60c02fa884db`, and DDS Agent
`73622810d984349b80bbac0ef55fc0b694d62222`. No firmware or parameter change is used.

### Complete selected source path

- `msg/versioned/TrajectorySetpoint.msg` defines float32 yaw and world-NED-z
  yawspeed, with NaN indicating an uncontrolled state. The pinned generated
  `build/px4_sitl_default/uORB/ucdr/trajectory_setpoint.h` copies both float
  fields without replacing NaNs. Only timestamps receive the existing offset/
  current-time handling; ROS continues to send truthful current simulation time.
- `MulticopterPositionControl.cpp:373` obtains yaw from local-position.heading.
  At 440-442 it updates the cached trajectory and applies one-time reset
  compensation. NaN plus a finite delta stays NaN. At 548 it calls
  `setInputSetpoint(_setpoint)` on every position-control update, even when no
  new external target arrives; `setState(states)` follows. The resolved yaw is
  an internal PositionControl value, not written back to the trajectory cache.
- `PositionControl/PositionControl.cpp:92-125` copies the current estimator
  state and incoming yaw/yawspeed, validates translation inputs and resolves
  non-finite yaw to current `_yaw` on each valid update. Finite yawspeed=0 is
  zero feedforward. `getAttitudeSetpoint()` passes the resolved yaw and thrust
  into `ControlMath::thrustToAttitude()` and copies yawspeed to yaw_sp_move_rate.
- `PositionControl/ControlMath.cpp:50-112` constructs a desired quaternion
  from the thrust/body-z vector and desired yaw direction. Position control
  timestamps its internal position and attitude outputs at publication.
- `mc_att_control_main.cpp:306-339` loads newer attitude setpoints and adapts
  them for a quaternion reset only if the attitude-estimate timestamp is newer
  than the last setpoint timestamp. `AttitudeControl/AttitudeControl.cpp:97-106`
  transforms world-z yaw feedforward to body coordinates and adds it to the
  attitude-error-derived rate demand. Zero feedforward is not direct body-rate
  control and does not imply zero attitude-control torque.

This removes the external stale finite-yaw input from the R1 race. It does not
prove that every internal PX4 scheduling discontinuity is impossible, nor is
unspecified yaw equivalent to holding a fixed geographic heading.

### Wire, monitoring and bounded handover

`p2-estimator-reference-v3` publishes finite position, yaw=NaN, yawspeed=0.0
from the first prestream target through TAKEOFF/STABILIZE_REFERENCE. Velocity,
acceleration and jerk remain NaN. Position Offboard mode and 20 Hz heartbeat
remain unchanged. During a pending reset, each fresh target repeats the last
bounded position; upward progression pauses until paired validation succeeds
or the existing 1.5 simulation / 2 wall second deadline fails. There is no
heartbeat-only gap in this v3 trajectory stream.

The R1 classifier retains independent uint8 baselines, one event <=5 degrees,
all source/reference rejection rules and five seconds of final alignment.
Initialization heading drift is checked against the original anchor in each
measurement's own reset generation, using both local heading and attitude
yaw. A candidate delta may normalize a pending drift measurement; it never
authorizes progression without the complete pair. Any residual >5 degrees
fails. Ground position origin, ENU axes and the 2 m target never move.

LOCK_REFERENCE freezes the accepted generation. A fresh local heading,
consistent with fresh attitude within the original 5-degree bound, seeds the
first finite target exactly. Dedicated YAW_HANDOVER then ramps toward the
original measured ground anchor plus accepted correction at <=10 degrees/s.
It completes only after reaching that anchor and observing a subsequent fresh
position sample at least one control period later. Drift/tracking remain
bounded by 5 degrees; reset/freshness/health changes abort. No handover retry or
re-anchoring is allowed. The entire original INITIAL_HOVER and eight motion
windows follow. Phase-D LAND and terminal disarming semantics are unchanged.

Explicit wire modes distinguish initialization from finite handover/nominal
control. JSON uses yaw=null/inactive and yawspeed=0/active during initialization;
finite yaw is active and yawspeed=null/inactive after handover. Invalid active
measurements are rejected, not silently converted to inactive values. Installed
ROS tests deserialize actual CDR for both modes; runtime evaluation compares
the bag messages with every reported mode and activation mask.

### Independent evidence and sequence

The v3 evaluator retains the old finite-cache test for v1/v2 evidence. For v3
it checks every initialization input, the full native-ULog drift interval,
reset classification, pending position hold, original origin/anchor, the first
finite target against its fresh measured heading, and finite-target rate and
handover ordering. Internal resolved yaw is matched to the most recent native
local-position sample, with at most one observed 8 ms estimator period and
1e-5 rad serialization consistency error. This is a sample-matching guard,
not a larger physical tracking tolerance. Internal outputs are logged around
10 Hz; the evaluator reports counts and maximum gaps and makes no claim about
unobserved internal updates. No extra logging or control parameter is changed.

The labelled initialization/handover diagnostic runs the complete bounded
profile to retain identical handover and LAND paths, but receives no nominal
or repeat acceptance credit. Its actual bag/ULog must pass before a new nominal
smoke, and that new smoke must pass before a fresh zero-based 20-trial batch.
If the diagnostic disproves this strategy, stop and retain the actual result.

## Preserved P2-R1 contract and failure

This is an explicit application policy revision from strict-v1 to
`p2-estimator-reference-v2`. PX4, its parameters, flight geometry, tracking
tolerances, freshness limits, envelopes and total deadlines are unchanged.
The historical aborted flight remains failed under its original policy.

**Final R1 status: blocked.** The new smoke passed, but the frozen repeated
sequence failed on trial 2 after one consecutive pass. A reset notification
can arrive after a periodic old-yaw target has already overwritten PX4's
cached correction. The narrow event classifier works for that recording;
the complete compensation-continuity contract does not. See the retained
race evidence below. No further flight or threshold change followed it.

## Pinned source diagnosis

All source references below are to PX4 commit
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.

1. `EKF/aid_sources/magnetometer/mag_control.cpp:362-378` requests the HAGL
   initialization reset when airborne, yaw-aligned, not yet magnetically
   aligned in flight, and above 1.5 m estimated HAGL. The calling branch at
   214-220 additionally requires magnetic heading/3D fusion or manual yaw.
   Altitude and a small angle alone do not establish the cause. The other
   WMM-update branch requires absent NE aiding or vehicle-at-rest. R1 requires
   GNSS position aiding, not-at-rest, no external/manual yaw, healthy magnetic
   fusion and the transition to in-flight magnetic alignment.
2. `resetMagHeading()` calls `resetQuatStateYaw()`. `yaw_fusion.cpp:123-172`
   replaces yaw, preserves tilt and computes `q_new * inverse(q_old)`.
   The output predictor left-multiplies this delta into buffered/current
   attitude only (`output_predictor.cpp:135-145`). It does not rotate local
   position, velocity or geographic origin. EKF2 publishes Euler(delta).yaw
   as `delta_heading`, and the same delta as Hamilton `delta_q_reset`.
3. Final alignment requires `cs_mag_aligned_in_flight`, yaw alignment and
   more than one second since the magnetic reset while magnetic fusion is
   active (`ekf.h:247-258`). `heading_good_for_control` exposes that combined
   result. R1 additionally requires `cs_mag_3d` and five stable simulation
   seconds before locking. A timer without these flags cannot lock.
4. `MulticopterPositionControl.cpp:687-729` corrects cached setpoints only
   when their nonzero timestamp is older than local-position.timestamp and
   the relevant reset counter changed. It adds `delta_heading` once and
   always updates its saved counters. `mc_att_control_main.cpp:323-339`
   likewise adapts an older attitude setpoint, not one generated after the
   current attitude estimate. Future external setpoints remain ROS's
   responsibility; repeatedly applying the delta would be incorrect.
5. `EKF2Selector.cpp:364-529` maintains independent aggregate counters and
   can synthesize deltas when switching estimators. Counter absolute equality
   is not assumed. This installed lane has `SENS_IMU_MODE=1`,
   `SENS_MAG_MODE=1`, `EKF2_MULTI_IMU=0`, `EKF2_MULTI_MAG=0`: one EKF instance.
   Launch verifies these unchanged values. Offline ULog verification also
   checks instance/device identities and filter faults. The fixed DDS map
   has no estimator-selector-status topic; online flags are not presented as
   complete sensor-device identity telemetry.

## Policy and ownership

Ground preparation freezes the original local/geographic origin, counters
and relative-heading anchor. Only TAKEOFF and STABILIZE_REFERENCE may pair
one initialization event, before any nominal dwell window. Ground-phase and
post-lock resets remain rejected. Position/velocity/origin/terrain-reference
changes, tilt deltas, skipped counters, a second event, alternate yaw sources,
faults and ambiguous evidence reject the trial. Each uint8 counter must advance
by exactly one modulo 256 relative to its own baseline.

Frozen project policy bounds are one event, absolute delta <=5 degrees,
heading/quaternion timestamp skew <=0.1 s, pairing/alignment evidence deadline
1.5 simulation / 2 wall seconds, yaw-delta agreement <=1e-5 rad and quaternion
x/y components <=1e-5. The latter are floating-point consistency checks,
not increased physical tracking tolerances. Both notification orders and
duplicates are supported; one paired event updates the anchor once.

Pending validation pauses trajectory progression. ROS continues the 20 Hz
OffboardControlMode heartbeat but temporarily withholds TrajectorySetpoint:
The design expects PX4 to retain its last bounded position target and cached
yaw correction. That expectation is conditional on no old-yaw publication
after the reset and before ROS detects it; trial 2 disproved that guarantee.
This bounded exception to continuous trajectory publication is
part of v2, not a hidden sample-gap relaxation. State sampling, heartbeat and
watchdogs retain their original bounds. Validation requires a subsequent
post-event local-position sample at least 0.1 s later before resuming targets.
The offline evaluator must verify actual PX4 setpoint compensation; time
alone is not claimed to be an acknowledgement from the position controller.

After acceptance, ROS adds the paired delta once to its persistent measured
initial-heading anchor and current yaw target. It publishes normal current
simulation timestamps, without backdating/future-dating them. Position
origin, position target and ENU axes do not change. The first resumed yaw is
the intended corrected anchor, and subsequent periodic sends retain it.
Trial 2 shows PX4 may instead hold an overwritten old anchor until that first
resumed publication. The intended contract preserves physical heading-relative yaw;
the nominal +30 degree ENU maneuver still subtracts 30 degrees in NED.

The entire initialization remains inside all original health, envelope and
deadline checks. After final alignment and five stable simulation seconds,
LOCK_REFERENCE precedes the original complete INITIAL_HOVER and axis/yaw/
normal-LAND sequence. No hidden preflight flight or shortened dwell is used.

### Phase D terminal arming eligibility

The first R1 smoke (`20260912T004810Z-p2-flight-0950fb87`) completed all motion
windows and normal ROS LAND, but failed the terminal check. That failed
attempt remains failed. Pinned `Commander.cpp:1874` publishes
`pre_flight_checks_pass = canArm(current_nav_state)`. Disarm calls
`UserModeIntention::onDisarm()` (`Commander.cpp:673`,
`UserModeIntention.cpp:94`), restoring the prior mode intention. With the
Offboard stream intentionally stopped for LAND, the restored Offboard mode
cannot arm; this is not evidence of an in-flight estimator fault.

Before the next smoke, v2 explicitly distinguishes this terminal condition:
only after accepted normal LAND ACK and observed AUTO_LAND (nav 18), with
fresh landed=true and arming_state=DISARMED, terminal validation no longer
requires eligibility to arm again. Failsafe=false, all estimate validity,
freshness, identity, reference and envelope checks still apply. Every earlier
phase and every armed/airborne sample still requires the original preflight
check. No re-arm or Offboard resumption is issued. Strict-v1 behavior remains
reproducible. Pure tests cover the terminal case and reject airborne, stale
and failsafe variants. This interpretation is frozen before the next run.

## Historical evidence and attribution limits

The original ULog reset is at 23.472 s; recorded DDS messages first expose
heading counter 1->2 at 23.480 s and quaternion counter 1->2 at 23.484 s.
Delta heading is -0.000763416465 rad; delta quaternion is approximately
`[1, -4.657e-10, 3.492e-10, -0.000381708203]`. Position/velocity counters and
origin remain unchanged. `cs_mag_aligned_in_flight` becomes true, followed
by `cs_mag_3d`. The strict-v1 controller aborts at 23.512 s.

Online classification uses the paired counters/deltas, fresh healthy state,
single-instance launch contract and observed magnetic-alignment transition.
It is labelled an allowlisted initialization pattern, not proof of a private
C++ call stack. Offline ULog verifies device/instance continuity, magnetic
flags, estimator events and actual downstream compensation. No ECL text
message identifying the precise call site was recorded in the old ULog.
Offline classification cannot turn its incomplete mission into a passed
flight. New live smoke and twenty new consecutive flights are separate gates.

## Retained asynchronous compensation race

Batch `20260912T010456Z-p2-repeat-7da31865` stopped after trial 2,
`20260912T010708Z-p2-flight-775b24dc`; its count remains **1/20**. The
independent evaluator rejected `PX4 cached setpoint correction missing/doubled`.
The raw sequence is:

| Simulation time | Raw observation |
|---|---|
| 23.408 s | PX4 heading reset, +0.003452183679 rad, counters 1->2 |
| 23.412 s | ROS transmits old anchor 1.673636631380 rad before receiving the reset |
| same ROS clock tick | Heading notification arrives 1.248873 wall milliseconds after that publication |
| 23.496 / 23.600 s | Internal PX4 yaw setpoint remains 1.673636674881 rad |
| 23.612 s | ROS accepts the pair and publishes corrected anchor 1.677088815059 rad |
| 23.704 s | Internal PX4 yaw setpoint is 1.677088856697 rad |

This matches the pinned position-controller semantics: after its reset
counter has advanced, a later external setpoint replaces the cached one and
is not corrected again for the same counter. A 20 Hz ROS callback cannot
assume it has already received every reset that PX4 has processed. Waiting
0.1 s after a *received* event does not protect the preceding publication.
Backdating timestamps, deleting the two uncorrected samples, expanding the
window exemption or counting eventual reconciliation as uninterrupted
compensation would not meet the requested contract.

Read-only diagnostic `20260912T011351Z-p2-r1-diagnostic-b9773624` retains its
script, input hashes, exact receipt order, ULog yaw samples, independently
passed motion windows and final landed/disarmed state. Those successful
subchecks do not reverse the reference rejection or restart the batch.
Internal setpoints are logged at about 10 Hz, so the exact scheduling between
logged samples is not claimed. The fixture
`tests/fixtures/p2_r1_late_notification.json` and two pure regressions preserve
the race and evaluator rejection. A further source-supported strategy for
the pre-notification interval is required before another smoke and streak;
R1 does not claim such a strategy has been proven.
