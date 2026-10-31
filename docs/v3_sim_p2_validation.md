# v3-SIM P2 validation

P2-R2 introduces `p2-estimator-reference-v3`: initialization position control
with intentionally unspecified yaw, bounded heading-drift monitoring, and a
measured handover to the original corrected mission anchor. R3 preserves that
policy and repairs a measured native SHM publication wait. Three post-repair
ground diagnostics, a complete new nominal smoke and a fresh **20/20** batch
passed. Current results are in the R3 section and schema-4 evidence summary;
the complete schema-3 value is preserved under `historical_p2_r2`. Strict-v1,
R1 and R2 failures remain historical failures. P1 flights, diagnostics and
runtime-free tests do not count toward the new P2 streak.

## Preserved strict-v1 implementation and evidence

The following original sections describe strict-v1 and its failed acceptance
state, not the outcome of P2-R1. The original schema-1 summary is preserved
verbatim as `historical_strict_v1` inside the schema-2 evidence document.

## Repository and scope

Work starts from `3bb9bfa09635090991ac42051465793464bbfe04` on
`v3/gwm-uav-c2-agentops-planning`. The v1 archive remains
`v1.0.0-research-framework-complete -> cb304c7a64b2edd57673a3323c0ed2e7d2923453`.
The Windows checkout is the only edited source. Builds, upstream trees,
bags and ULogs remain in `/home/joker0625/uav_autonomy` in Ubuntu-24.04 WSL2.
P0/P1 code, configuration and historical evidence are preserved. AgentOps
permissions, its 13-tool metadata catalogue and `invoke_mock()` are unchanged.
The standalone design report was unavailable; the operator's pasted
source-derived requirements were used instead.

Dependencies remain PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, px4_msgs
`86d8239e962f6939e05c3737784f60c02fa884db`, and DDS Agent
`73622810d984349b80bbac0ef55fc0b694d62222`. Installed versions are ROS Jazzy,
Gazebo Harmonic 8.15.0, PX4 v1.17.0, DDS Agent v2.4.3 and QGC 5.1.4.
The existing installation was used; `clean_rebuild_proven=false`.

## Source contract

Reviewed official versioned references:

- [ROS 2 user guide](https://docs.px4.io/v1.17/en/ros2/user_guide)
- [Offboard requirements](https://docs.px4.io/v1.17/en/flight_modes/offboard)
- [ROS 2 Offboard example](https://docs.px4.io/v1.17/en/ros2/offboard_control)

The local pinned `dds_topics.yaml`, generated message definitions, startup
scripts, Commander ACK routing, EKF2 source and GZBridge source determine the
actual implementation. No dependency was upgraded to match an example.

`gwm_px4_control` is an ament_python package. `frames.py`, `contracts.py`,
`protocol.py`, `timing.py`, `mission.py` and `acceptance.py` import no ROS runtime.
`node.py` loads ROS only after gates and owned-namespace checks. The package is
copied one way into a content-addressed Linux build mirror; source hashes are
checked before use. The source package hash used in both the passed observation
and attempted flight is
`83670f14ba986e9dcd88979b7e749f5c602dd5819c3db95ed380163188f2da7d`.

The exact transport is ROS 2 / Fast DDS -> Micro XRCE-DDS Agent UDP -> PX4
uXRCE-DDS client. The sole external flight owner is `/gwm/p2_control`.
PX4/Gazebo, DDS, ROS, clock bridge, recorder and QGC share a private
user/mount/PID/network namespace with only loopback, plus an exclusive P1/P2
workspace lock. PX4 console access performs boot/parameter/state inspection.
There is no MAVSDK, MAVROS, joystick, console flight fallback or manual QGC
flight command in P2.

| Identity | Actual value |
|---|---|
| PX4 instance / model | 71 / x500_71 |
| Vehicle system / component | 72 / 1 |
| Controller source system / component | 201 / 191 |
| DDS client key / UDP port | 72 (`0x48`) / 8888 |
| DDS domain / ROS prefix | 71 / `/px4_71` |
| Gazebo partition | Unique run ID |
| Initial flight owned PIDs | QGC 6, DDS 7, PX4 21, bridge 775, ROS 776 |

## Topics, QoS and time

All names below are under `/px4_71/fmu/`. Actual graph types are
`px4_msgs/msg/<MessageFamily>`, checked against the generated definition and
the fixed PX4 message schema. Version suffixes are discovered, not assumed.

| Direction / topic | Version | Initial flight unique receive rate |
|---|---:|---:|
| in/offboard_control_mode | 0 | Published 20.0 Hz |
| in/trajectory_setpoint | 0 | Published 20.0 Hz |
| in/vehicle_command | 0 | Two discrete commands |
| out/vehicle_command_ack | 0 | Event-driven, eight recorded ACKs including unrelated QGC requests |
| out/vehicle_status_v1 | 1 | 2.214 Hz |
| out/vehicle_local_position_v1 | 1 | 50.0 Hz |
| out/vehicle_attitude | 0 | 100.009 Hz |
| out/vehicle_land_detected | 0 | 1.682 Hz |
| out/estimator_status_flags | 0 | 1.196 Hz |
| out/failsafe_flags | 0 | 1.895 Hz |

PX4 graph publishers report BEST_EFFORT, TRANSIENT_LOCAL, depth 0. Received
state subscribers explicitly request BEST_EFFORT, VOLATILE, KEEP_LAST depth
50. Read-only validation received every required state family and recorded
zero input command/setpoint publications. The flight graph allowed exactly
one publisher for each of the three control topics; competing publishers fail.

The actual `/world/default/clock` is bridged GZ_TO_ROS to `/clock`.
ROS nodes use simulation time. Run-local startup sets `UXRCE_DDS_SYNCT=0` and
the effective value is recorded. PX4 message timestamps are microseconds in
the simulation boot epoch; ROS clock values are seconds/nanoseconds in that
same epoch. Receipt monotonic seconds are separate wall-watchdog measurements.
No Unix-wall/boot-epoch subtraction is used. In the flight, 1,139 exact PX4
timestamp pairs had identical ROS/ULog positions. Position receipt clock age
ranged from -0.004 to +0.008 s, inside the configured 0.1 s epoch tolerance.

Position/attitude freshness is 0.25 simulation seconds; slower status/flags
use 1.5 seconds. All receipt ages also have a 2 s wall bound. Duplicate PX4
timestamps do not refresh state or enter acceptance as new observations.
Clock stalls have a separate 1 s monotonic watchdog; backwards time and
frozen reference-counter changes abort the active trial. Startup is bounded
at 75 wall seconds, each active phase at 45 simulation / 75 wall seconds,
the mission at 240 simulation / 420 wall seconds, and recovery at 45 wall
seconds. A clock stall is not labelled a proven Gazebo pause or crash.

## Frames, fields and state machine

The pinned default SDF declares ENU orientation. Pinned GZBridge converts
world position as `[world_y, world_x, -world_z]` to NED and body FLU vectors
to FRD as `[x, -y, -z]`. The mission uses ENU offsets from the recorded PX4
estimator origin, not a presumed zero local origin or Gazebo truth feedback.
VehicleAttitude supplies Hamilton `w,x,y,z`, body FRD to NED. Quaternion
rotation is implemented explicitly and tested at multiple headings.
Absolute `yaw_NED=wrap(pi/2-yaw_ENU)` is distinct from a +30 degree ENU
increment, which subtracts 30 degrees from the measured initial NED yaw.

Position Offboard explicitly enables only `position`. Finite NED position
and yaw are sent; unused velocity, acceleration, jerk and yaw rate are NaN
on the PX4 wire. JSON uses nulls plus active/inactive masks and
`allow_nan=false`. CDR inspection verified these semantics in the actual bag.
No velocity-mode flight is claimed.

The nominal state machine waits for clock/discovery and fresh aligned
estimation, freezes origin/reference counters after five stable simulation
seconds, prestreams the ground target for at least two seconds, requests
Offboard and verifies actual nav state 14, requests normal arm and verifies
actual armed state 2, then begins a rate-limited ascent. It implements 2 m
height, 10 s initial settled hover, +1 m east/return, +1 m north/return,
+30 degree ENU yaw/restore, 5 s per settled test and 5 s final hover, followed
by one normal LAND command and separately observed landing mode, landed and
disarmed state. Landing handover stops Offboard targets. No automatic restart,
re-arm, force-arm or force-disarm is provided.

Acceptance settings were frozen before the first flight: 20 Hz; position
ramp <=0.3 m/s; yaw ramp <=10 degrees/s; horizontal/height/yaw tolerances
0.2 m / 0.3 m / 5 degrees; maximum sample gap 0.2 s; horizontal envelope radius
2 m and height envelope -0.5 to 3 m. Entry into a settled window also requires
speed <=0.1 m/s. Windows remain fixed after entry; a failed window is not
restarted to find a better segment. The independent expected-NED fixture does
not call the production frame transform. Missing measurements prevent a pass.

## Command transactions

Command IDs are 176 (mode), 400 (arm), and 21 (land). Exactly one local
transaction is outstanding, with a maximum of one attempt per command ID.
The pinned ACK has no general application UUID. Commander routes ACK target
fields to the command's **source**, hence ACK matching requires target 201/191,
not vehicle 72/1. Matching also requires command ID, phase, fresh timestamp,
internal ACK origin and 3 simulation / 5 wall second deadlines. Accepted,
in-progress, temporary rejection, denial, unsupported, failed and cancelled
results have explicit handling; unrelated/stale ACKs are ignored.

Actual mode ACK was accepted after 0.020 s and normal-arm ACK after 0.004 s.
Both matched ULog exactly. Actual Offboard was separately observed before
arming; actual armed state was separately observed before ascent. No LAND
command was issued in this failed trial because its estimator-reference
contract was invalid. Streamed setpoints have no invented individual ACKs.
Nominal LAND ACK/state completion remains unproven by real P2 flight.

## Failure and independent evidence

The initial origin was NED `[0.002930973, 0.007702557, 0.131673545]` m and yaw
`1.674781709` rad. The control stream contained 189 targets at 20.0 Hz, maximum
gap 0.052001 s, and exactly 2.0 s ground prestream. Maximum serialized position
ramp was 0.300006207 m/s (float32/time quantization, checked with explicit
1e-5 m/s numerical tolerance). This is not a changed flight tracking tolerance.

At PX4 timestamp 23.472 s, the heading counter changed 1 -> 2; ROS observed
heading/quaternion counters 1 -> 2 and aborted at 23.512 s. The ULog reset
position was NED `[0.029551055, -0.002592022, -1.498011351]` m and delta heading
was -0.000763416 rad. Even a small reset invalidates this trial by design.

Pinned `EKF/aid_sources/magnetometer/mag_control.cpp::checkHaglYawResetReq()`
requests a yaw reset once airborne above its fixed 1.5 m HAGL boundary to
recover from ground magnetic interference. `EKF2_MAG_TYPE=0` was effective.
The 2 m target necessarily crosses this boundary. Ground readiness uses
`cs_tilt_align`, `cs_yaw_align`, valid position/velocity, quaternion validity,
native preflight health and failsafe flags. `heading_good_for_control` is
not a ground prerequisite: pinned `isYawFinalAlignComplete()` includes
in-flight magnetic alignment. Correcting that field interpretation allowed
the connection stage to pass; the active reset rejection remains strict.

After invalid estimation, the controller stopped old targets and observed
PX4's configured fallback. It did not assume position hold, RTL or its own
LAND command remained feasible. Effective policies were `COM_OF_LOSS_T=1`,
`COM_OBL_RC_ACT=0`, `NAV_RCL_ACT=2`, `NAV_DLL_ACT=2`, `COM_DL_LOSS_T=10`, and
`COM_FAIL_ACT_T=5`. Pinned failsafe code routes Offboard loss to the RC-loss
action when fallback manual control is unavailable. The actual nav-state
sequence included Offboard 14 and AUTO_RTL 5; console and ULog show landing
and automatic disarming. Final landed/disarmed was independently confirmed.
QGC monitoring/heartbeat remained present (`gcs_connection_lost=false` in
recorded flags). This is one observed failure recovery, not the P6 campaign.

Raw run directories beneath `/home/joker0625/uav_autonomy/runs/`:

| Run ID | Outcome |
|---|---|
| `20260911T170209Z-p2-observe-91527f9f` | Failed before node startup: gate variables absent from child environment; no flight commands |
| `20260911T170320Z-p2-observe-22e9e7c3` | Failed recorder construction: Jazzy TopicMetadata required ID; no flight commands |
| `20260911T170712Z-p2-observe-cabbd9ed` | Failed startup estimation timeout from incorrect ground heading prerequisite; no flight commands |
| `20260911T171351Z-p2-observe-13aa4c5f` | Passed read-only connectivity, recording and offline integrity |
| `20260911T171449Z-p2-flight-802425c9` | Failed initial ROS flight on estimator-reference reset; recording and recovery independently verified |
| `20260911T172201Z-p2-repeat-595bb3f6` | Batch preflight rejected failed smoke; zero flight trials started |
| `20260911T172820Z-p2-observe-7a6faf85` | Passed final-launcher read-only connection and offline recording checks |

The first two startup failures cannot claim complete ROS recording; their
partial artifacts and logs remain preserved. All six runtime attempts remain
in history: two passed observations, three failed observations, one failed
flight. There is no successful P2 smoke or acceptance streak.

The final launcher additionally hashes the QGC profile and records console
state on failed controller exit. That final launcher passed the second
read-only stage above. The earlier failed flight retains its original
launcher identity; it was not relabelled as a flight of the later launcher.

For the attempted flight, `rosbag/metadata.yaml`, `rosbag_0.db3`, full
`ros-events.jsonl`, topic contract/graph, process/endpoint logs, effective
parameters, controller result and summary are retained. ULog is
`rootfs/log/2026-09-11/17_15_00.ulg` (7,732,362 bytes), SHA-256
`ec97ccfcca39c4c4ff8bb9116e3595d010907b122b71007fe3ab6791c700cec0`.
The sanitized summary records all bag/ULog hashes and offline evaluator hash.
Offline verification opens SQLite read-only, deserializes CDR without ROS
nodes, compares the complete event ledger, correlates commands/ACKs and
estimator samples with ULog, checks wire field/ramp semantics, and records
the failure and stop/recovery evidence. ULog dropout count was zero.
No historical command topics were replayed.

## Regression and status

- P2 runtime-free tests: **68 passed**.
- AgentOps/C2 tests: **398 passed**.
- Full Windows Anaconda suite: **903 passed, 12 skipped**, final rerun 97.47 s.
- Actual Linux ament_python colcon build passed; installed-package test:
  **1 passed, 0 errors, 0 failures, 0 skipped**.
- Python compilation, Bash syntax, ShellCheck and Git whitespace checks are
  recorded separately in the sanitized summary. ShellCheck excludes SC1091
  only for generated/sourced ROS workspace files unavailable to static lookup.

| Gate | Status |
|---|---|
| P2 connectivity | passed |
| P2 protocol and frame tests | passed |
| P2 initial ROS flight | failed |
| P2 repeated acceptance | not_run (0/20) |
| P3 sensing | not_implemented |
| P4 avoidance | not_implemented |
| Clean rebuild | not_proven |

P2 remains incomplete. A deliberate estimator/reference strategy is needed
before another nominal flight; the default mandatory reset and strict
reset-invalidates-trial contract cannot both produce a 2 m P2 pass. P3 is
not yet recommended. There is no claim of QGC-free startup, QGC-disconnect
behavior, obstacle avoidance, model decisions, multi-UAV autonomy or hardware
readiness. P3-P7 and v3-2-v3-7 remain unimplemented by this change.

## P2-R1 policy revision and revalidation

R1 starts from `c59cc3246be8a74ea2683aedfa30ff2b4dcd6918` on the same branch,
after an up-to-date fast-forward-only synchronization. All three dependency
pins, the PX4 binary, default `EKF2_MAG_TYPE=0`, original geometry and flight
thresholds remain unchanged. The application reference policy changes to
`p2-estimator-reference-v2`; [the source contract](v3_sim_p2_reference_contract.md)
records the pinned implementation, exact bounds, compensation ownership,
online/offline evidence limits and phase-D terminal interpretation.

The pure reference manager pairs independent heading/quaternion uint8
increments, supports either arrival order and duplicates, and applies one
bounded yaw correction only during TAKEOFF/STABILIZE_REFERENCE. It freezes
position/velocity/terrain/origin and aiding-source state. Pending validation
keeps the Offboard heartbeat while PX4 holds its last position target and
corrects cached yaw. The ROS relative-yaw anchor and future target are then
corrected once; geographic axes and the ground origin never move. A final
alignment interval precedes LOCK_REFERENCE and all original eight windows.
After locking, unsupported resets remain fatal. The package mirror hash is
`1001cc827c1945403b8e7ab1d09cb8b61e6f0f8ab8fb2c75865a7a662c463827`.

### Historical offline classification

`20260912T010200Z-p2-r1-offline-df9cc28c` passed offline reset classification
against the old recording. The original -0.000763416465 rad event advances
each independent counter 1->2, preserves position/velocity/origin, and
coincides with magnetic in-flight alignment. Four internal PX4 setpoint
samples show the once-corrected anchor, with zero measured error. Device IDs
and the single EKF instance remain stable. The historical flight still
**failed**: no offline classifier can establish its unflown nominal windows.
No command topics were replayed and no original result was overwritten.

### New nominal smoke

`20260912T010218Z-p2-flight-9cf2ff9c` passed both the actual controller and
independent bag/ULog verification. It accepted one initialization correction
of -0.001433611149 rad and completed final alignment, reference lock, full
INITIAL_HOVER, east/return, north/return, yaw/restore, FINAL_HOVER and normal
ROS LAND. The matching LAND ACK latency was 0.016 simulation seconds; actual
AUTO_LAND and final landed/disarmed were observed at completion (106.804 s).
No failsafe was observed. Independent checks used each fixed controller
window, with no search for a better segment or restarted dwell.

The recording contains 4,946 exact ROS/ULog position pairs with zero position
difference, zero ULog dropouts, a 2.0 s ground prestream, and a 20.0023 Hz
Offboard heartbeat (maximum interval 0.052001 s). There are 1,727 trajectory
targets and 1,730 heartbeats; the three pending-validation ticks withheld
trajectory targets according to v2. Position ramp and compensated commanded
yaw ramp remain within the original 0.3 m/s and 10 degrees/s bounds plus
the existing serialization-rounding checks. Raw wire yaw rates are retained
separately. The independent evaluator and helper hashes are frozen by the
repeat runner together with controller, configuration, launchers and pins.

### Retained R1 attempts

Every run below remains beneath `/home/joker0625/uav_autonomy/runs/`:

| Run ID | Outcome |
|---|---|
| `20260912T004359Z-p2-r1-offline-dd913691` | Passed historical classification with initial R1 implementation |
| `20260912T004636Z-p2-build-DsIsLc` | Passed initial R1 ROS build/test |
| `20260912T004658Z-p2-observe-3b4dfa98` | Passed read-only connection and offline integrity |
| `20260912T004707Z-p2-r1-offline-489f7187` | Passed historical classification after zero-reset-path hardening |
| `20260912T004810Z-p2-flight-0950fb87` | Failed terminal arming-eligibility check after all motion windows, normal LAND ACK and actual landing; remains failed |
| `20260912T005607Z-p2-r1-offline-ef9e0cda` | Failed offline classification due to a local variable-shadowing defect introduced during cleanup; corrected before the next live flight |
| `20260912T005624Z-p2-r1-offline-1dc99d39` | Passed corrected historical classification |
| `20260912T005630Z-p2-build-orbFx4` | Passed final controller ROS build and installed test |
| `20260912T005646Z-p2-observe-dad84605` | Passed final-controller read-only connection and offline integrity |
| `20260912T005746Z-p2-flight-29957c89` | Failed ground prestream on `observation_gap`, before any flight command; stayed landed/disarmed |
| `20260912T010200Z-p2-r1-offline-df9cc28c` | Passed final historical offline classification |
| `20260912T010218Z-p2-flight-9cf2ff9c` | Passed complete new nominal smoke and independent verification |

The ground-prestream failure exposed missing early-abort handling in the
evaluator. Its original evaluation and two unsuccessful diagnostic
assessments remain alongside `p2-offline-evaluation-r1-ground-abort-v3.json`,
which confirms intact recording and a failed, never-armed trial. First target
publication took about 0.33 wall seconds; the deeper scheduler/serialization
cause was not established. The observation threshold was not increased and
the controller was not changed to bypass this failure. No diagnostic attempt
counts toward the consecutive acceptance streak.

### R1 regressions and consecutive acceptance

- P2/reference tests: **114 passed**, including the original 68 strict-v1 tests and two retained-race regressions.
- AgentOps/C2 regressions: **398 passed**; permissions, 13-tool catalogue and mock invocation unchanged.
- Full Windows Anaconda suite: **949 passed, 12 skipped**, final run 83.22 s.
- Linux ROS build and installed-package test: **passed; 1 test, 0 failures**.
- Python compile: **17 files passed**. Bash syntax/ShellCheck: **3 scripts passed**, SC1091 excluded for sourced ROS paths only.
- Ordinary pytest did not launch simulation. Git whitespace validation passed.

Batch `20260912T010456Z-p2-repeat-7da31865` is a new frozen-input sequence,
separate from the smoke and all P1/strict-v1 history. Its authoritative count
and each trial's raw evidence/hash references are recorded in schema 2 of
[the evidence summary](evidence/v3_sim_p2_summary.json). P2 is complete only
if this batch records twenty consecutive independently verified successes.

**Final batch outcome: failed, 1/20.** Trial 1
(`20260912T010459Z-p2-flight-db751110`) passed controller and independent
verification. Trial 2 (`20260912T010708Z-p2-flight-775b24dc`) completed all
motion and normal LAND but failed the frozen ULog compensation check. No
trial 3 started. No batch input or threshold changed and the batch was not
restarted. The original second-trial evaluator records `recording_integrity:
failed`, `flight_acceptance: unknown` with the semantic reason
`PX4 cached setpoint correction missing/doubled`; this is not a claim of
physical bag/ULog corruption.

The separate read-only diagnostic
`20260912T011351Z-p2-r1-diagnostic-b9773624` confirmed all sixteen fixed
bag/ULog motion windows and final landed/disarmed, but reproduced the
reference-continuity rejection. PX4 reset at 23.408 s; ROS published its old
yaw at 23.412 s, 1.248873 wall milliseconds before receiving the heading
notification. Internal PX4 targets at 23.496 and 23.600 s retained that old
yaw. ROS restored the corrected anchor after paired validation at 23.612 s.
The source's one-time cached-setpoint correction does not automatically
repair later external publications for an already-consumed reset counter.
The full timeline and limits are in the source-contract note. The diagnostic
is not another flight and adds no acceptance credit.

| Final R1 gate | Status |
|---|---|
| P2 reference contract | **blocked** by pre-notification compensation race |
| P2 revised nominal smoke | passed |
| P2 repeated acceptance | failed; incomplete, 1/20 |
| P2 complete | no |
| P3 sensing / P4 avoidance | not_implemented |
| Clean rebuild | not_proven |

P3 is not recommended. The valid bounded-classification implementation and
evidence are retained with this explicit incomplete status. No PX4 patch,
estimator parameter change, fabricated timestamp, increased threshold or
replacement console flight was used to mask the remaining conflict.

## P2-R2 initialization yaw ownership and bounded handover

R2 starts from `0e51fbfff437fc6214cf66ca738d55770bda27a5` on the same branch,
after fast-forward-only synchronization. The unrelated `.codegraph/` tree is
preserved. No upstream revision, binary, estimator parameter, model or
AgentOps permission changed. The source-supported contract and its limitations
were written before the diagnostic in
[the reference document](v3_sim_p2_reference_contract.md).

The actual runtime binary uses `build/px4_sitl_default_linux/bin/px4`. Its
generated `uORB/ucdr/trajectory_setpoint.h` is byte-identical to the initially
inspected `px4_sitl_default` header (SHA-256
`dbb268de1e3fa4a1fb2b0e51a357ceb8ca4ce7c47269b5e10a194de7b5953bab`).
The selected source paths and hashes are recorded in schema 3. Cached NaN yaw
remains unspecified; PositionControl resolves it from its current heading on
each valid update. Zero yawspeed is world-z yaw feedforward, not direct body
rate control. Attitude-setpoint/reset scheduling still has its own timestamp
rules; no claim is made that every downstream discontinuity is impossible.

### Software and wire contract

The package mirror is
`4abf5ce22a3e9ddf723ea9570c90c5d88e64a4e41e461dacbc7c695d5fab404a`.
Initialization begins at the first prestream publication with finite position,
yaw=NaN and yawspeed=0.0. JSON records intentional inactivity as yaw=null and
an inactive mask, while zero yawspeed is active. Finite handover/nominal yaw
uses the opposite yaw/yawspeed activation masks. Actual CDR messages are
checked against those records. Invalid measurements are never silently
converted into valid inactive commands.

Pending reset validation freezes bounded position progression while sending
fresh targets at the original 20 Hz rate. Both estimator heading and attitude
yaw retain the original 5-degree drift bound in their own reset generations.
The independent R1 reset classifier, one-event limit, 5-degree correction
limit, five stable alignment seconds and rejection deadlines are preserved.
After lock, the first finite target equals fresh aligned heading. Handover
ramps at 10 degrees/s to the original ground heading plus accepted correction.
Only then do the full original dwell windows begin. Original position origin,
ENU axes, +30-degree yaw maneuver, LAND and terminal disarming checks remain.
Every configuration value except the policy identifier equals R1, including
all numeric geometry, ramps, tracking, freshness, envelope and deadline limits.

The pure source-faithful scheduler model reproduces finite R1 cache overwrite
and exercises both reset/publication orders, delayed notifications, both
notification orders, duplicates, timestamp ordering, signed corrections and
angle wraparound. The model establishes a publication invariant, not vehicle
dynamics or actual PX4 scheduling. Historical fixtures and v1/v2 evaluator
semantics remain testable; v3 adds explicit mode-specific evaluation.

### Diagnostic and new nominal smoke

| Stage | Run ID | Independent result |
|---|---|---|
| Linux build / installed tests | `20260912T023023Z-p2-build-2btPMr` | passed; 2 installed tests |
| Read-only DDS | `20260912T023340Z-p2-observe-724a0465` | passed; no flight commands |
| Labelled full-profile diagnostic | `20260912T023504Z-p2-flight-8cf8a6ad` | passed; no nominal or repeat credit |
| New nominal smoke | `20260912T023930Z-p2-flight-94486276` | passed; qualifies only the new R2 batch |

The diagnostic contains 309 initialization targets, all yaw-unspecified with
zero yawspeed. Native ULog maximum drift is 0.316696 degrees for local heading
and 0.316784 degrees for attitude yaw. All 134 matched internal resolved-yaw
samples equal their current estimator heading (zero measured error and time
offset). Internal output maximum sample gap is 0.104 s. Handover spans
29.548-29.696 simulation seconds; its first finite yaw is 1.676488638 rad and
its original corrected anchor is 1.678364707 rad. Both complete independent
motion evaluations and normal ROS LAND/ACK, landed/disarmed checks pass.

The new nominal smoke has 311 initialization targets, maximum drift
0.260421 degrees, and another 134 exact internal-heading matches. Handover
spans 29.612-29.760 s from fresh heading 1.676015377 rad to corrected original
anchor 1.676524640 rad. All sixteen fixed rosbag/ULog motion windows pass,
including the full ten-second initial hover. Normal LAND ACK latency is
0.016 simulation seconds and actual landed/disarmed is confirmed. There are
4,953 exact ROS/ULog position pairs, no ULog dropouts, 1,728 trajectory targets
and 1,728 heartbeats. Maximum stream gap is 0.052001 s. No fallback recovery
is counted as success.

The internal-output evidence is approximately 10 Hz. Exact matches establish
the sampled source-defined behavior; intervals between them and downstream
attitude scheduling are not proven at every update. No logging configuration
was added. The complete raw recordings and their hashes remain in WSL.

### R2 regression

- Focused P2/reference tests: **159 passed**, including 45 new R2 cases.
- AgentOps/C2 regression: **398 passed**.
- Full normal Anaconda suite: **994 passed, 12 skipped**, 93.98 s.
- Linux ROS build: passed; **2 installed tests passed**, including CDR round trips.
- Final Python compilation: **20 files passed**.
- Bash syntax and ShellCheck: **3 scripts passed**; only SC1091 excluded for sourced ROS paths.
- Ordinary pytest launched no simulator. Git whitespace checks passed.
- Clean rebuild remains **not_proven**.

### New R2 consecutive acceptance

Batch `20260912T024156Z-p2-repeat-70022b4f` starts at zero after the new smoke.
Its frozen inputs include the controller package, configuration, launchers,
dependency identities and all three independent evaluator source files.
Each flight requires the full motion/reference/recording/LAND checks. The
runner stops on the first failure or interruption. Neither the diagnostic
nor any R1/P1 result contributes a pass. Final status and trial-by-trial
evidence are recorded in schema 3 of
[the sanitized summary](evidence/v3_sim_p2_summary.json).

**Final R2 batch outcome: failed, 5/20.** Five new consecutive trials passed
all independent motion/reference/recording/LAND checks. Their aggregate
initialization drift maximum is 0.765620 degrees; 678 matched internal yaw
samples have zero measured error against current heading. All five landed
and disarmed normally.

Trial 6, `20260912T025246Z-p2-flight-ac58a793`, stopped at 14.180 simulation
seconds for `observation_gap` during PRESTREAM_SAFE_SETPOINTS. The previous
accepted position timestamp was 13.820 s and the next was 14.164 s: a 0.344 s
controller observation gap exceeded the unchanged 0.2 s bound. Only one
initialization target and one heartbeat were transmitted. No external flight
command, arm, takeoff, yaw reset, handover or nominal window occurred. The
independent evaluator reports recording integrity **passed**, flight
acceptance **failed**, zero ULog dropouts and `remained_grounded`, with actual
landed/disarmed. No trial 7 started and the batch was not restarted.

The first target's JSON event was recorded 0.333905 wall seconds after the
sample receipt used by that callback. Received position timestamps themselves
have maximum gap 0.024 s. These facts bracket delayed callback/publication
work, but do not isolate serialization, recorder, middleware or host scheduling
as the root cause. The similar historical R1 ground-prestream failure remains
in the ledger. No callback repair, pre-warming change or threshold relaxation
was applied after this failed batch. The failed trial never exercised the
yaw-reset/handover interval, and supplies no acceptance evidence for it.

| Final R2 gate | Status |
|---|---|
| Publication contract | verified by source, focused tests and sampled runtime evidence |
| Runtime diagnostic | passed; not counted as nominal acceptance |
| New nominal smoke | passed |
| Repeated acceptance | failed; incomplete, **5/20** |
| P2 complete | no |
| P3 sensing / P4 avoidance | not_implemented |
| Clean rebuild | not_proven |

Schema 3 preserves the complete schema-2 JSON value under `historical_p2_r1`,
including nested strict-v1 results. All R2 successful and failed trials retain
manifest, raw recording and evaluator hashes. The next work is bounded
diagnosis of the ground-prestream scheduling/publication delay before a
separately authorized new smoke and fresh streak. P3 is not recommended.

## P2-R3 control-loop latency diagnosis

R3 begins at `a27efab` and preserves `p2-estimator-reference-v3`, all original
`p2_control.yaml` values, pins and estimator parameters. See the separate
[timing contract](v3_sim_p2_timing_contract.md) for measured attribution,
timestamp migration, dispatch guards and remaining synchronous-writer limits.
The historical R2 0.333905 s interval starts at Mission selection, despite
the old `receipt_monotonic_s` name; it is not an isolated publish duration.

Four pre-repair ground starts localized the native first-heartbeat SHM wait.
Two passed, one failed `observation_gap`, and one failed stale attitude after
a measured 335.442 ms publication. The fourth native stack captured an actual
333 ms health-check sleep within Fast DDS SHM transport. Small concurrent
bag/JSON/graph spans do not support blaming evidence persistence for that stall.
The controller alone now uses official UDPv4 built-in transport. No dependency
or host configuration was changed; the recorder remains synchronously owned.

| Post-repair diagnostic | First heartbeat | Maximum publication | Maximum callback | Consumed gap | Result |
|---|---:|---:|---:|---:|---|
| `20260912T034634Z-p2-ground-c8971267` | 0.103 ms | 0.904 ms | 2.794 ms | 0.072 s | passed |
| `20260912T034735Z-p2-ground-2bea0f79` | 0.191 ms | 0.191 ms | 4.525 ms | 0.064 s | passed |
| `20260912T034826Z-p2-ground-20b540c4` | 0.112 ms | 0.312 ms | 2.415 ms | 0.064 s | passed |

All three independently verified zero VehicleCommand, no REQUEST_OFFBOARD or
REQUEST_ARM transition, continuous landed/disarmed observations in ROS/ULog,
correct NaN-yaw/zero-rate wire semantics, complete trace/recording and owned
cleanup. Actual prestream coverage was 2.948, 2.948 and 3.000 simulation seconds.
These diagnostic passes give no credit toward a new 20-consecutive-flight run.

### R3 complete nominal and repeated acceptance

Build `20260912T034318Z-p2-build-wNyi8m` passed with installed package hash
`e1857b3fc0a5678c0789b39860ba36cae5193b8daa83a20c246a02f49bb431af`.
Read-only run `20260912T034509Z-p2-observe-69157ddb` passed for those exact
inputs. Runtime manifests retain the starting Git commit plus the measured
source/config/launcher/library identities of the tested working tree.

New nominal smoke `20260912T034917Z-p2-flight-c74f35ba` passed the controller,
recording integrity, timing, reference reconciliation, yaw ownership, all
16 fixed ROS/ULog windows, command/ACK mapping and LAND. It recorded 311
initialization targets, 0.694315-degree maximum initialization drift and
135 exactly matched internal yaw samples. Actual prestream was 2.000 s;
maximum publish was 1.073 ms, callback 4.767 ms and consumed gap 0.072 s.
The internal-yaw sampling/coverage limitations in the R2 source contract remain.

Fresh batch `20260912T035329Z-p2-repeat-84bb288d` passed **20/20**, from
`20260912T035332Z-p2-flight-d1216632` through
`20260912T043525Z-p2-flight-05a058bd`. It did not resume R2 trial 7 or reuse
any previous success. All 20 controller exits and independent evaluator exits
were zero; all 320 fixed ROS/ULog windows passed. Every trial had accepted
mode/arm/LAND ACKs, final landed/disarmed confirmation and complete recording.
No failed/interrupted trial was omitted, no retry occurred, and frozen source,
config, evaluator, transport, binary and GUI-profile identities remained equal.

| Worst measured batch metric | Value |
|---|---:|
| Controller-consumed source gap | 0.072 s |
| Subscription callback-entry gap | 0.049549 wall s |
| Source age at control / dispatch | 0.032 / 0.032 sim s |
| Heartbeat / trajectory actual publication gap | 0.056 / 0.056 sim s |
| Actual publish call | 3.115 ms |
| Complete control callback | 10.630 ms |
| Graph inspection | 9.475 ms |
| Acquisition-through-persistence callback envelope (descriptive) | 26.498 ms |
| Minimum actual prestream | 2.000 sim s |
| Trace records / configured capacity | 221,632 / 500,000 |
| Trace overflow / ULog dropouts | 0 / 0 |

Post-batch verification confirmed all owned process exit codes, released
exclusive lock, no remaining owned runtime processes and unchanged frozen
inputs. Final console state and ULog independently confirm landed/disarmed.
This is simulation-profile acceptance, not a hard-real-time or clean-rebuild claim.

### R3 retained history and regression

The schema-4 summary lists all 37 R3 build/runtime/batch directories with
manifest, raw-recording and evaluator hashes, including every pre-repair
failure and all 20 new trials. The four pre-repair ground attempts remain
diagnostic outcomes, not warm-up exclusions from acceptance. The immutable
historical reconstruction and native probe evidence remain in WSL.

Additional retained setup runs are builds `20260912T031319Z-p2-build-XV0m7R`
and `20260912T031814Z-p2-build-2g1BDL`; observe runs
`20260912T031354Z-p2-observe-b7c7be32`,
`20260912T031856Z-p2-observe-aa94ea2a`,
`20260912T032305Z-p2-observe-1e71d62b`, and
`20260912T032611Z-p2-observe-d0dec3bf`. All passed their build/connectivity
checks; all observes passed offline recording integrity. None has flight credit.

Regression: **177** focused P2/reference/yaw/timing tests; **398** C2/AgentOps
tests; **1012 passed, 12 skipped** in the full ordinary suite; **2** installed
ROS tests. Python compilation, Bash syntax and whitespace checks passed.
ShellCheck passed with only SC1091 excluded for dynamically sourced ROS setup
paths. The diagnostic interposer compiled with existing gcc and
`-Wall -Wextra -Werror`. Ordinary pytest launched no optional simulator/runtime.
Queue/worker-specific tests are inapplicable because no writer queue/worker
was introduced; synchronous sink failure and bounded trace integrity are tested.

P2 timing attribution is **established**, timing repair **verified**, new
ground diagnostics and nominal smoke **passed**, and repeated acceptance
**passed (20/20)**. P2 is complete for this profile. P3-P7 and v3-2-v3-7 remain
unimplemented/incomplete; P3 is the next eligible slice, not started here.
Clean rebuild remains **not_proven**.
