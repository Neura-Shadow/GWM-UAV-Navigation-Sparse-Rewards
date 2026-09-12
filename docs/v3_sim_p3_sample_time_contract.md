# P3 sample identity and time contract

`p3-sample-evidence-v2` defines the P3-R1 evidence revision. It keeps fixed
flight windows in the PX4 publication-time domain, preserves every recorded
observation, and separates source coverage from controller execution. It does
not change `p2-estimator-reference-v3`, the physical mission, or the accepted
x500 P2 campaign. A separately labelled historical reanalysis earns no new
flight or qualification credit.

This policy is frozen before P3-R1 runtime qualification. The source findings
below establish its basis; they are not a claim that the revised implementation
or fresh acceptance has passed. Runtime results belong in
[P3 validation](v3_sim_p3_validation.md) and the versioned evidence summary.

## Pinned source and publication semantics

The inspected PX4 checkout is
`/home/joker0625/uav_autonomy/upstream/PX4-Autopilot` at
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. The links below pin that same
revision. They describe the inspected source rather than a current upstream
default.

| Source | Finding |
|---|---|
| [VehicleLocalPosition.msg, lines 4-7](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleLocalPosition.msg#L4-L7) | Message version 1; publication and raw-data timestamps are uint64 microseconds. |
| [VehicleAttitude.msg, lines 4-8](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/versioned/VehicleAttitude.msg#L4-L8) | Message version 0; both timestamps are uint64 microseconds. |
| [EKF2.cpp, lines 673-686](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L673-L686) | Single-EKF input time is `sensor_combined.timestamp`; each update is detected through the uORB subscription. |
| [voted_sensors_update.cpp, lines 165-179](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/voted_sensors_update.cpp#L165-L179), [lines 225-235](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/voted_sensors_update.cpp#L225-L235) | The selected gyro stream supplies `sensor_combined.timestamp` from `vehicle_imu.timestamp_sample`. |
| [VehicleIMU.cpp, lines 653-664](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/sensors/vehicle_imu/VehicleIMU.cpp#L653-L664) | Integrated IMU output carries the latest gyro sample timestamp separately from its publication HRT. |
| [EKF2.cpp, lines 746-751](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L746-L751), [lines 801-809](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L801-L809) | Attitude is published after the latest IMU predictor update; local position is published after an EKF update, using that IMU time. |
| [EKF2.cpp, lines 1036-1046](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1036-L1046), [lines 1557-1567](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1557-L1567), [lines 1675-1677](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1675-L1677) | `timestamp_sample` is the supplied IMU time; normal `timestamp` is `hrt_absolute_time()`. Replay has a different assignment and is outside this runtime profile. |
| [estimator_interface.cpp, lines 78-110](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/estimator_interface.cpp#L78-L110), [output_predictor.cpp, lines 178-198](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/output_predictor/output_predictor.cpp#L178-L198) | The output predictor runs at the latest IMU horizon. The delayed fusion horizon is a separate time, not the local-position/attitude `timestamp_sample`. |
| [estimator_interface.h, lines 252-261](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/estimator_interface.h#L252-L261), [estimator_interface.cpp, lines 571-586](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/estimator_interface.cpp#L571-L586) | Published quaternion, velocity and projected position derive from predictor output. |

The historical smoke records `SENS_IMU_MODE=1`. PX4 selects multiple-EKF mode
only for `SENS_IMU_MODE=0`
([EKF2.cpp, lines 2783-2796](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L2783-L2796)).
The observed topic instance is zero; the historical ULog does not expose a
separate estimator selector stream. A topic instance is not a physical IMU
device identifier.

The selector, when enabled, checks strictly advancing sample times, propagates
reset information, and replaces the publication timestamp with current HRT
([attitude, lines 364-414](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2Selector.cpp#L364-L414),
[local position, lines 510-539](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2Selector.cpp#L510-L539)).
Those selector guards are not evidence that the direct-EKF path enforces the
same guard. P3-R1 explicitly validates progression in recorded data and the
online cache. A violation fails rather than being inferred away from the
intended estimator behavior.

## Why equal publication times do not establish duplication

The inspected SITL build defines `ENABLE_LOCKSTEP_SCHEDULER`. In that build,
[`hrt_absolute_time()`](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/drv_hrt.cpp#L106-L114)
returns the current lockstep value. The
[getter](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/lockstep_scheduler/include/lockstep_scheduler/lockstep_scheduler.h#L50-L53)
does not increment it for a read; the
[setter](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_scheduler.cpp#L50-L56)
assigns simulator time. The
[Gazebo clock callback](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/simulation/gz_bridge/GZBridge.cpp#L331-L345)
synchronizes that clock from Gazebo simulation time. Separate estimator
publications can therefore share an HRT value between simulator clock updates.

This explains why publication time is not guaranteed unique. It does not
establish the scheduling cause of a particular retained collision, attribute
one to rendering or llvmpipe, or establish a controller stall.

## Native order and recording boundaries

uORB increments a generation on every atomic publication independently of
message timestamps
([uORBDeviceNode.cpp, lines 188-203](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/uORBDeviceNode.cpp#L188-L203)).
Subscription updates use that generation
([Subscription.hpp, lines 131-147](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/Subscription.hpp#L131-L147)).
The generated local-position and attitude topics both have queue length one;
their generated `ORB_DEFINE` entries are at line 49 of
`build/px4_sitl_default_linux/msg/topics_sources/vehicle_local_position.cpp`
and `vehicle_attitude.cpp`. A subscriber can consequently skip intermediate
publications. Queue-one copying returns the latest generation
([uORBDeviceNode.hpp, lines 225-255](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/uORBDeviceNode.hpp#L225-L255)).

Logger updates use subscription generations and account for detected gaps in
full-rate subscriptions
([logger.cpp, lines 444-478](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/logger.cpp#L444-L478)).
ULog associates a topic name and `multi_id` with a logger `msg_id`
([lines 1849-1875](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/logger.cpp#L1849-L1875));
data records contain that ID and native topic bytes, not the internal uORB
generation
([messages.h, lines 168-180](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/messages.h#L168-L180)).

Preserve file offset, global record ordinal, topic ordinal, `msg_id` and
`multi_id` where available. Per-topic observed native order is retained;
cross-topic file order is the logger's subscription loop order
([logger.cpp, lines 749-775](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/logger.cpp#L749-L775)).
It is not an established order of publication or controller availability.

DDS local position has a 50 Hz publication limit
([dds_topics.yaml, lines 81-83](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.yaml#L81-L83)).
XRCE applies the subscription interval and copies available uORB data
([dds_topics.h.em, lines 101-103](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em#L101-L103),
[lines 130-140](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/uxrce_dds_client/dds_topics.h.em#L130-L140)).
ROS and ULog counts need not be equal. Unmatched ULog outputs are reported as
unmatched recorded outputs, without a claim of DDS packet loss or complete
one-to-one parity.

XRCE serialization can adjust both `timestamp` and `timestamp_sample`
([ucdr template, lines 138-144](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/Tools/msg/templates/ucdr/msg.h.em#L138-L144)).
The owned profile retains verified `UXRCE_DDS_SYNCT=0`. Exact payload overlap
and run/clock configuration validate the shared epoch; field names alone do
not establish clock equivalence.

## Clock table

Integer native values remain integers internally. Seconds are presentation or
explicit arithmetic outputs. Booleans, fractional values, nonfinite values,
and missing required timestamps cannot silently select another clock.

| Clock or generation | Source, units and epoch | Supported order and uniqueness | Role |
|---|---|---|---|
| PX4 `timestamp` | Message uint64 microseconds; PX4 HRT, synchronized with simulator time in this profile | Nondecreasing order is checked per topic/instance; equal stamps are possible; unrelated topics need no shared uniqueness | Original fixed-window membership, distinct covered publication times, source age, publication/sample latency |
| PX4 `timestamp_sample` | Message uint64 microseconds; latest input IMU horizon for the two predictor outputs | Positive, no later than publication, and strictly advancing for distinct local-position/attitude outputs are checked profile invariants | Validate output identity/order inside and between publication groups; never silently replace the coverage clock |
| `ref_timestamp` and reset counters | PX4 reference establishment time and topic reset generations | Not measurement IDs or general time; counters can wrap and are compared under the existing reference contract | Expose reference/reset changes on every raw observation and control evaluation |
| Bag storage timestamp | Integer nanoseconds supplied by the owned controller writer from its ROS simulation clock reading | Recorder-side timestamp; equal values are possible; database ID/native storage order is retained | Preserve receipt/serialization evidence and correlate storage records; never substitute for acquisition or source freshness |
| ULog file/record ordinal | Byte offset/global ordinal and per-topic ordinal in the original file | File serialization order; no cross-topic causal guarantee | Lossless raw-record identity and supported within-topic observed order |
| Callback entry monotonic time | Process-local monotonic wall clock at message callback entry | Order is local to the run/process; not comparable to PX4 epoch | Callback delay, accepted-source receipt age and delivery diagnostics |
| Selection/evaluation monotonic time | Actual controller evaluation/selection time | Distinct evaluations can inspect the same source observation | Controller consumption deadlines and all-evaluation health/extrema |
| ROS `/clock` | Integer seconds/nanoseconds from Gazebo through the clock bridge | Checked non-regression and wall-clock progress; subscriptions are asynchronous | Simulation progress, source age and clock-stall watchdog |
| Dispatch monotonic time and outgoing wire timestamp | Actual controller publish operation; wire field is integer microseconds | Separately checked outgoing order/ramp and dispatch age; not an observation ID | Existing 50 ms dispatch budget, consumed gap, actual prestream and command/setpoint acceptance |

Historical event records sometimes store display seconds or a supplied bag
timestamp obtained through floating-point conversion. Preserve those bytes and
state their precision limit; recover integer source stamps from the raw
message, not by inventing an epsilon increment. New identity fields carry
native integer timestamps directly.

## Six evidence categories

| Category | Run-scoped identity and retained evidence | Contribution |
|---|---|---|
| A. Raw source records | Recording identity, topic/type/version, instance when available, storage/file ordinal, original timestamps, payload and reset/reference fields | Every recorded value remains available for source-order, validity, reference, tracking and integrity checks |
| B. Application callback deliveries | Unique delivery ID, raw/source reference, callback receipt time, classification and payload fingerprint | Delivery/receipt deadlines; a duplicate does not refresh accepted-source freshness |
| C. Controller evaluations/selections | Unique evaluation/selection ID, actual simulation/monotonic time and separate source reference for each cached component | Check every evaluation's health/mode/reference/extrema and actual consumption/dispatch timing |
| D. Distinct source observations | Topic/version/instance plus publication/sample/reference identity and verified payload, with an observation ID separate from delivery ID | Contribute source values and distinct covered publication instants; a verified reuse contributes no new position measurement |
| E. Outgoing publications | Unique publication ID, selected-source/selection reference, actual publish timing and unchanged wire payload | Actual heartbeat/setpoint/command, prestream, ramp, dispatch and ACK/state confirmation checks |
| F. Fixed-window evaluation records | Contract/evaluator hash, original integer boundaries, every member's raw/source/selection references and classification | Reproducible coverage and all-record extrema without deleting repeated or equal-time records |

Fields unavailable in an old recording are explicitly unavailable. An enclosing
manifest can attribute historical records to a run; it cannot retroactively
claim that old messages carried a correct run ID or a missing callback ID.
New observation and health envelopes must name the unique enclosing owned run,
not the nested directory `sensors`. Cross-run joins and mixed evaluator hashes
cannot form one unchanged acceptance campaign.

## Frozen identity and ordering rules

Publication microseconds are uint64, including zero at the simulation boot
origin. This differs from the positive estimator `timestamp_sample` invariant.
Pinned Commander initializes its status storage and changed-state flag
([Commander.hpp, lines 219 and 281](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/Commander.hpp#L219-L281)),
then stamps an immediate status publication with current HRT
([Commander.cpp, lines 1931-1945](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/Commander.cpp#L1931-L1945)).
Lockstep HRT returns the scheduler clock, whose initial value is zero
([drv_hrt.cpp, lines 106-110](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/drv_hrt.cpp#L106-L110),
[lockstep_scheduler.h, lines 53 and 98](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/lockstep_scheduler/include/lockstep_scheduler/lockstep_scheduler.h#L53-L98)).
A raw boot-origin status is retained with its original invalid preflight health
and receives no readiness or control health credit. Retaining that raw record
does not waive selected-evaluation health, freshness, fixed-window coverage, or
positive estimator sample requirements. Offline publication validation uses the
same uint64 parser as the online identity contract.

1. A timestamp alone is never the complete identity of an estimator output.
   The checked source identity includes run, topic/type/version, available
   topic instance, publication microseconds, sample microseconds and reference
   generation. The payload fingerprint is retained and compared independently.
   Recording ordinals identify raw rows even when every source field repeats.
2. For local position and attitude, publication time cannot regress. Every
   distinct output requires a positive integer sample stamp no later than its
   publication stamp. Distinct outputs must have strictly advancing sample
   stamps in native per-topic/instance order. Missing/invalid stamps,
   unexplained output order, or conflicting identity fail.
3. Equal publication stamps with verified advancing samples remain distinct
   outputs in native order. They share one publication-time coverage instant.
   Their position, attitude, validity and reset fields all remain checked.
4. An exact verified source duplicate is classified as reuse. Its callback and
   each controller evaluation remain recorded. Neither publication coverage
   nor accepted-source freshness advances. Reusing position does not imply
   that attitude/status/flags were also reused.
5. Same claimed source identity with conflicting payload is an error. Equal
   publication time with changed status/flags and no supported distinct-output
   order is ambiguous; it is not silently overwritten. Relevant payload
   equivalence may support a causal projection while preserving all raw rows.
6. Validation precedes cache mutation. The state cache has one owner, records
   per-topic accepted source identity and receipt, and separately retains
   delivery counts/latest delivery information. An identity- or order-invalid
   candidate cannot replace accepted state or refresh its age. Health-invalid
   estimates remain visible to the existing readiness/flight health checks;
   they cannot keep a previous healthy estimate usable. Reference checks remain
   active on equal-time candidates.

These are explicit sample-semantics changes. They are not a global replacement
of a strict-gap comparison with a non-strict comparison.

## Fixed-window coverage and all-record checks

The original fixed boundaries remain in integer publication microseconds.
Window membership is reconstructed from source references and those boundaries,
not from a controller `fresh` assertion, row count, or expected window count.
Where the existing independent measurement brackets a fixed boundary, retain
the same deterministic bracketing rule and report its offsets; do not move the
requested window, select a favorable interval, or borrow future causal state.

Coverage uses sorted-by-observed-order, nondecreasing publication-time groups
after raw order validation. Positive progress between distinct covered times
establishes elapsed duration and maximum uncovered interval. Equal-time groups
add no duration. Repeated rows cannot make a short window long enough or bridge
a missing interval. A genuine time regression is rejected before grouping.

Every recorded estimator output in the interval contributes to tracking,
validity and reference checks. Every control evaluation, including one with
reused position, contributes its actual component values to mode, health,
reference and relevant tracking checks. An unsafe attitude on a reused-position
evaluation or a bad output inside an equal-time group still fails.

Coverage is separate from controller timeliness. Continuous recorded PX4 output
cannot excuse a delayed controller. Preserve the 0.2-second maximum consumed
gap, existing simulation/wall freshness limits, 50 ms dispatch budget, fixed
dwell durations, worst-case tracking limits, drift and flight envelope,
outgoing timestamp/ramp checks, actual prestream, mode/arm/LAND ACKs and state
confirmation. No average replaces a worst-case gate. Normal landed/disarmed
completion and owned cleanup remain required; fallback landing after abort
earns no nominal acceptance.

## Lossless overlap and causal joins

Timestamp-to-single-row maps are forbidden for relevant correlations. Use
ordered multi-value groups and keep all candidate row IDs. ROS/ULog overlap
uses verified source fields and payload identity; never choose the first,
last or nearest-value match because its tracking error is favorable. Report
raw records, distinct observations, verified reuse, matched, unmatched,
ambiguous and reused-match counts. Enforce the declared coverage requirement
without claiming completeness across differently limited streams.

Validate join input order before searching it. For an independent causal
cross-topic projection at publication time `t`, choose the latest publication
group strictly earlier than `t`. A same-time cross-topic message is excluded
because this recording does not establish whether it was already available.
Report that exclusion and the available ordering limitation.

Within the selected earlier local-position/attitude group, use the last record
in verified native/sample order as the latest observed output. This rule is
fixed independently of measured values; all other records still receive their
own checks. For a status/flags group without a supported output order, require
equivalent relevant payload or fail with explicit ambiguity. Apply the unchanged
freshness bound to the selected earlier state. Missing earlier state fails;
there is no future-state fallback.

Check all native attitude/health records inside each fixed window separately,
including equal-time records excluded from a causal projection. The projection
cannot hide a transient violation. Recorded callback/selection references,
rather than ULog file order, determine what the controller actually inspected.

The same limitation applies to internal PX4 yaw-feedback evidence. The direct
EKF executes on
[`INS0`](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L2931),
while multicopter position control executes on
[`nav_and_controllers`](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L46).
Position control copies a local-position update
([MulticopterPositionControl.cpp, lines 392-397](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L392-L397)),
uses its heading, then gives `vehicle_local_position_setpoint` a new publication
HRT
([lines 603-606](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L603-L606)).
That output message carries no selected-input identity or sample timestamp.
Even a unique same-time local-position row does not prove it preceded the
controller's copy on a different work queue.

The frozen internal-yaw test therefore reports **prior-state feedback
consistency**: the latest strictly earlier publication group, final verified
native/sample-ordered local-position output, unchanged maximum time difference
0.008001 seconds and maximum heading difference 0.00001 radians. It never
selects a same-time or nearest-value row to improve the result. Exact internal
MC consumed-source identity remains unproven even when that consistency test
passes. A failed prior-state test remains failed; these logs cannot repair it
by inventing cross-topic causality.

Image-to-PX4 association retains its separate, bounded nearest-observation
meaning and no-interpolation/reset-crossing rules. Retain integer publication
and sample stamps plus full matched identity, reject unresolved collisions,
and report both time differences. A sensor association is not evidence of
causal controller consumption and does not command or change the route.

## Immutable historical comparison and runtime prerequisites

The original smoke `20260912T113430Z-p3-flight-0dc41a18`, its failed independent
reports, raw records and hashes remain immutable. Case A is repeated controller
selection of the same position source during FINAL_HOVER. Case B is distinct
ULog local-position outputs sharing publication time during RESTORE_INITIAL_YAW.
The retained reconstruction must identify exact rows, payload differences,
sample timestamps, callback/storage times and original rejection reasons.

The original evaluator is reproduced separately. Revised analysis names this
contract and its own evaluator hash, reports original and revised gates and
remaining limitations, and uses a new filename. It does not alter historical
statuses or receive new-flight credit. The final run-ID build requires fresh
runtime evidence regardless of historical adjudication.

Before dependent evaluation, require recording closure, bounded writer drain,
final controller result, complete artifact/hash manifest and the required
upstream evaluator result. A partial JSONL line or unfinished manifest yields
explicit incomplete status. Freeze source/build/configuration/model/world/
calibration/transport identities and every evaluator hash before the fresh
readiness, ground matrix, smoke and three-flight qualification. Preserve every
attempt and stop qualification on its first failed or interrupted attempt.

P3 remains incomplete until the declared final-build checks, one new camera
smoke and all three consecutive qualification flights pass. P4-P7 and v3-2
onward remain outside this repair; clean rebuild is not proven by these checks.
