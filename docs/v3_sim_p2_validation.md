# v3-SIM P2 validation

P2 control implementation and read-only DDS connectivity are complete. The
first actual ROS-controlled flight **failed** on an estimator-reference reset
during ascent. Repeated acceptance is **not_run, 0/20**. Neither the completed
P1 flights nor runtime-free tests establish full P2 flight acceptance.

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
