# v3-SIM P2 validation

P2-R1 adds `p2-estimator-reference-v2`, with a source-verified bounded heading
initialization phase before the original full mission. Its current measured
results are in the P2-R1 section below and the versioned evidence summary.
The original strict-v1 flight remains **failed**. P1 flights and runtime-free
tests do not establish P2 flight acceptance.

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
