# P3-R2 yaw causal evidence contract

`p3-yaw-causal-evidence-v1` revises the acceptance role of the P3-R1 yaw
proxy. It leaves `p3-sample-evidence-v2`, `p2-estimator-reference-v3`,
`p2-timing-v1`, all flight limits and the PX4 binary unchanged. Outcome C
applies: exact MC consumed-input provenance is unobservable in the retained
recordings and no supported acquisition for it is available in the unchanged
profile. More nominal flights cannot fill that gap. P3 remains incomplete.

## Source and executable audit

The locally inspected upstream checkout is clean at
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. The actual
`build/px4_sitl_default_linux/bin/px4` SHA256 is
`5383767fd3068a8a306660430f10c8852eb4d581a8a7af5f3fc14cea7b9d106d`;
ELF build ID is `d7ac87102e62a15b61586e6623735e2c1c97ada4`. It has DWARF
debug information and a symbol table. These identities match the recorded
profile; this is not a new build or a proof of clean rebuild reproducibility.
All source links below refer to that pinned revision, inspected locally.

1. The stack-local `vehicle_local_position` in
   [MC Run, lines 392-427](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L392-L427)
   is populated by `_local_pos_sub.update(&vehicle_local_position)`.
   `set_vehicle_states` reads that object's `heading` into `states.yaw`
   [at line 373](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L373).
   It does not reread the newest publication when resolving yaw.
2. That copy precedes trajectory update/reset adjustment, `setState`, and
   control calculation. [PositionControl, lines 91-122](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/PositionControl/PositionControl.cpp#L91-L122)
   copies `states.yaw` to `_yaw`; a valid update substitutes `_yaw` for
   nonfinite `_yaw_sp`. The active setpoint can be the normal trajectory,
   last valid fallback, or generated failsafe. The final two paths are
   explicit in [MC lines 574-596](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L574-L596).
   A finite fallback must be compared with that effective finite input.
3. Only after those calculations does `getLocalPositionSetpoint` copy the
   resolved values; [MC lines 603-606](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L603-L606)
   assign a fresh HRT to the output and publish it. This is not an input stamp.
4. Direct EKF2 runs on `INS0` ([line 2931](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L2931));
   MC runs on `nav_and_controllers` ([line 46](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_pos_control/MulticopterPositionControl.cpp#L46)).
   The source does not hold the uORB copy lock across the calculation.
   Another estimator publication can intervene. This is a possible execution,
   not an assertion that a specific historical mismatch followed that sequence.
5. [POSIX HRT, lines 106-110](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/posix/src/px4/common/drv_hrt.cpp#L106-L110)
   returns lockstep scheduler time. Reading the clock does not increment it.
   Copy, input publication and output publication can share an HRT value;
   strict timestamp inequality cannot establish which input was consumed.
6. [VehicleLocalPositionSetpoint.msg](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/msg/VehicleLocalPositionSetpoint.msg)
   has output timestamp, position/velocity, acceleration/thrust, yaw and
   yawspeed. It has no consumed topic generation, input sample timestamp,
   control-update identity, reference generation or fallback-path identity.
7. [Subscription update/copy](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/Subscription.hpp#L139-L173)
   maintains subscriber-local generation. [DeviceNode copy](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/platforms/common/uORB/uORBDeviceNode.hpp#L225-L255)
   copies data and generation atomically for a queue of one. Generation is
   not part of the message payload. [Logger lines 751-775](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/logger.cpp#L751-L775)
   iterate logger subscriptions and serialize a data header/topic ID plus
   payload. File order describes those logger observations, not cross-topic
   producer or MC copy order. Zero ULog dropouts does not prove every source
   publication reached the logger's queue-one subscription.
8. [EKF attitude publication](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1036-L1045)
   and [local position publication](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF2.cpp#L1557-L1676)
   retain predictor input sample time separately from publication HRT.
   `mc_att_control` copies its own attitude at lines 243 onward and uses
   another local-position subscription for heading quality/unaided heading
   at lines 277-282. Those are not the position controller's consumed object.

Both recordings have `SENS_IMU_MODE=1`, `SDLOG_PROFILE=131`. The pinned
[logger profile](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/logger/logged_topics.cpp#L136-L146)
limits internal local-position setpoints to 100 ms; SITL overrides estimator
position/attitude to unrestricted topic logging at lines 257-259. Actual
original/latest datasets contain 1,095/1,100 internal outputs,
13,660/13,726 positions and 27,265/27,421 attitudes. Equal-time distinct
position outputs (1/6) and attitude outputs (200/513) remain present. No
recorded field supplies the missing consumed-input relationship.

## Separate evidence roles

| Role | What can be verified | Acceptance meaning |
|---|---|---|
| A external initialization | Actual CDR NaN yaw, zero yawspeed and ledger/mask parity | Required external publication contract |
| B handover | Recorded position/attitude references, finite seed, reset-corrected original anchor and bounded transition | Required handover provenance; old timestamp-only records retain their stated limitation |
| C physical behavior | Initialization drift, all fixed controller/ROS/ULog windows, all native health/reference/extreme records, timing, mode and LAND/disarm | Required observed physical/mission behavior |
| D exact internal relation | Independently paired consumed input and effective setpoint against resolved output | Required; currently `unobservable_from_recording` |
| E prior-state diagnostic | Frozen latest strictly earlier publication group, last validated sample-ordered output | Retained diagnostic; not D or control latency |

Statuses are `verified`, `contradicted`, `unobservable_from_recording` and
`missing_required_evidence`. Missing provenance cannot become a pass from
good tracking or a matching candidate. A correctly paired contradiction
cannot be suppressed by A/B/C. A failure of E is a contradiction of the
declared proxy, not automatically a physical or control-law failure.

`1e-5 rad` remains the correctly paired angular limit. No tolerance is
enlarged. E retains its original `0.008001 s` prior-record-age bound and
every match/violation. That age is neither actual consumed-input age nor
execution latency. New analyses report overall `flight_acceptance=unknown`
when physical checks pass but required D is unobservable; they still exit
nonzero. Original historical failed reports keep their original status.

The pure `yaw_provenance.exact_relation` model uses explicit source ID,
run/binary, consumed generation, reference generation, effective setpoint,
path and independently captured execution order. It selects before inspecting
output yaw and checks every supplied update, including equal-time updates.
It has **no current ULog trace adapter**. Synthetic verification does not
prove a trace is complete, authentic, approved or available at runtime.
Those properties require a separately qualified acquisition and coverage
contract before any implementation may use it for flight acceptance.

## Observability gate and instrumentation amendment

Outcome A is unavailable: no lossless reconstruction of the MC's private copy
can be derived from these fields. Outcome B is also unavailable in this
profile. Read-only inventory found GDB/debug symbols but no relevant static
tracepoints or SDT notes, no `perf`, `bpftrace` or `trace-cmd`; tracefs was
not readable (`perf_event_paranoid=2`, `ptrace_scope=1`). LTTng 2.13.11 is
installed, but its listing reports no session daemon accessible to the task
user; the existing system daemon is not an MC/uORB UST provider. Neither
starting another session daemon nor increasing logger rates would
create that provider. No debugger attachment, runtime trace, security change,
framework installation, firmware modification or rebuild was performed.

The smallest proposed amendment is an **opt-in diagnostic sidecar in a new
explicitly identified PX4 binary**, without adding a uORB/ROS message ABI or
changing the control law:

- Immediately after the successful MC local-position copy at line 394,
  retain a monotonically increasing control-update ID, subscriber generation
  from `get_last_generation()`, topic/instance, input publication/sample
  times, heading and all relevant reset/reference fields. Retain the values
  in that same `Run` invocation, not a second subscription copy.
- Capture the effective yaw/yawspeed and input timestamp after reset
  adjustment and each `setInputSetpoint`; record initial update return and
  the normal/last-valid/generated-failsafe branch. Record the final attempted
  update's validity. Finite/NaN fields need explicit masks or raw float bits.
- At lines 603-606, pair the resolved float32 yaw/yawspeed and exact output
  timestamp with those retained inputs under the same update ID. Record
  copy/computation/output ordering explicitly; same-HRT values are allowed.
  If wall execution spans are recorded, use a real monotonic clock separate
  from lockstep HRT. Do not infer spans by subtracting source timestamps.
- Write one fixed-size record for **every relevant update**, with sequence,
  first/last/update counts and overflow flag. No rate-limited logging of this
  sidecar is allowed. A preallocated 200,000-record ring (approximately
  32 MB at 160 bytes/record; finalize the actual struct size before approval)
  covers the bounded trial at up to 250 Hz. Flush only after owned control
  shutdown. Overflow, incomplete drain or missing sequences invalidate D.
- Associate the completed sidecar with run ID, binary SHA256/build ID,
  source/patch hash, configuration and recording manifest. An instrumented
  result cannot certify the old uninstrumented binary.

Overhead is **not measured yet**. Approval would need to include a bounded
diagnostic comparison before acceptance: fixed input/window, explicit paired
baseline/instrumented ground acquisitions, per-update wall/thread CPU,
callback gaps, simulation progression, peak memory, ring usage and complete
flush hashes. Keep existing timing limits; reject an acquisition that changes
them or loses records. Do not use breakpoints/stepping in an acceptance flight.
This document prepares the scope; it does not authorize or implement it.

## Historical retention and offline execution

The original smoke and latest R1 smoke remain failed under their original
evaluators. E remains 79/135 and 68/135 failed comparisons respectively;
latest includes 63 angular violations, eight age violations, three overlapping,
maximum `8.08238983154297e-5 rad` and `0.016 s`. No qualification trial exists.
The full schema-2 P3 summary is preserved as `v3_sim_p3_summary_v2.json`;
schema-1 and accepted P2 schema-4 remain unchanged.

New `--historical-analysis` results bind old runtime inputs separately from
current analysis inputs. For R1, the complete finalized artifact manifest is
rechecked and all callback identity checks remain active. The original P3
recording predates R1's runtime seal; its existing ULog/bag/sensor hashes are
checked without fabricating a newer seal. Its explicit `--sample-contract`
path retains the limited historical controller reconstruction. Neither path
earns runtime or qualification credit. Every output uses a new exclusive
filename and atomic finalized publication.

The [offline budget report](v3_sim_p3_offline_budget.md) records the fixed
workflow measurements, performance-only differential comparison and runner
failure tests separately from this deliberate evidence-contract revision.
