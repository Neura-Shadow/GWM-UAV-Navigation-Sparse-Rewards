# P2-R3 control timing and evidence contract

R3 starts from `a27efabf186a9e1e7cc68f4fe0859e76530124c0`. The R2
`p2-estimator-reference-v3` yaw policy, all original numeric flight limits,
dependency pins and estimator parameters remain unchanged. Timing/evidence
is versioned separately as `p2-timing-v1` in `configs/p2_timing.yaml`.

## Observed historical facts

The read-only reconstruction compares the failed R2 trial
`20260912T025246Z-p2-flight-ac58a793`, the passed R2 smoke
`20260912T023930Z-p2-flight-94486276`, and passed batch trials 1 and 5.
Results are retained in WSL as `runs/20260912-p2-r3-historical-timing.json`.
No command topics are replayed. Historical flight outcomes remain unchanged.

| Measurement | Failed trial | Passed smoke |
|---|---:|---:|
| Native ULog position source gap near first target | 0.008 s | 0.008 s |
| ROS received source-stamp maximum gap | 0.024 s | 0.028 s |
| Position application callback-entry gap near first target | 0.343898 s | 0.022120 s |
| Controller-consumed source-stamp maximum gap | 0.344 s | 0.064 s |
| Selection to post-publication target event | 0.333905 s | 0.000817 s |
| Subscription callback entry to target event | 0.342741 s | 0.010370 s |

**Historical naming clarification:** `sample.receipt_monotonic_s` was set
from the `wall` argument of `StateCache.validate()`. At this call site it is
Mission sample-selection time, not subscription receipt. The original R2
summary's 0.333905 s number is preserved, but its attribution to receipt is
corrected here. Actual callback entry is recorded in the `received` event.
Neither is a proven DDS arrival time.

The failed controller attempted to consume position 14.164 s after 13.820 s,
exceeding the original 0.2 s observation-gap limit. It aborted at 14.180 s,
before any VehicleCommand. One heartbeat and one yaw-unspecified target were
recorded. It remained landed/disarmed; recording integrity and cleanup passed.
The original failure is not ignored as warm-up and remains a failed trial.

The old event boundaries cannot isolate construction, publish entry/return,
CDR, bag writing or JSON persistence. Source age at dispatch and individual
operation durations are unknown for that historical interval. A same-process
bag is not independent middleware-arrival evidence. Zero ULog dropouts does
not establish callback or control timeliness.

## Ranked hypotheses and diagnostic budget

Before instrumented runtime, the falsifiable predictions are:

1. Lazy message/type support: first construction or publish has a large span;
   later calls do not, and measured bounded in-memory preparation removes it.
2. Synchronous evidence: serialization, SequentialWriter or text persistence
   spans account for the callback stall; isolation removes that measured work
   from the callback while maintaining complete records.
3. Middleware publish: publish entry/return itself brackets the delay; moving
   disk persistence would not remove it.
4. Graph/host scheduling: graph spans or unexplained callback-entry lateness
   dominate while individual publication/evidence spans remain small.

At most six fresh-process ground diagnostics are allowed before a repair;
at most three after an evidence-supported repair. Every attempt is retained.
No favorable-run search, hidden control warm-up or acceptance credit is used.
If attribution remains unavailable within that budget, retain instrumentation
and report the uncertainty rather than invent a cause or start acceptance.

## Instrumented baseline

The initial `instrumented_synchronous` profile preserves synchronous evidence
processing to measure it before selecting a repair. A bounded 500,000-record
in-memory trace stores monotonic operation spans and simulation time separately.
Each span has a sequence, operation name and call index, distinguishing first
use from later calls. Trace overflow invalidates timing evidence and stops
progression. Trace lines are written only after control stops, not synchronously
inside each measured callback. Missing trace records prevent a complete claim.

Spans cover generated class/publisher preparation, message construction,
publish entry/return, CDR serialization, SequentialWriter.write, JSON/text
writing, graph inspection/persistence, cache updates, clock/subscription
callbacks, Mission.tick and the complete timer callback. The sole executor
owner retains Mission, StateCache, reset pairing, ACKs and command decisions.

The pinned rclpy timer uses a 20 Hz STEADY_TIME timer while mission timestamps
and state freshness use simulation time. This policy is unchanged. This local
rclpy does not expose TimerInfo; trace records actual steady-clock callback
entry and a derived preceding deadline from `time_until_next_call()` minus
one period, with query overhead. These are diagnostic timing fields, not a
new clock policy or proof of hard real-time scheduling. Skipped executor
deadlines cannot be reconstructed as if callbacks actually ran.

`--ground-diagnostic` uses the real readiness and safe first prestream path,
with three simulation seconds of bounded ground prestream and a 75-second
wall bound within existing startup/trial deadlines. The Mission rejects
REQUEST_OFFBOARD/REQUEST_ARM and all command requests. The ROS adapter creates
no VehicleCommand publisher and independently denies it at send(). Normal
landed/disarmed state is required throughout. Finite ground position,
yaw=NaN and yawspeed=0 remain identical to R2 initialization. The independent
ground evaluator checks actual CDR, no commands, state, hashes and full
two-second minimum actual recorded prestream coverage.

## Measured attribution and repair

Four of the allowed six pre-repair starts were used. Two passed, and two
reproduced a first-heartbeat stall. Diagnosis stopped after localization:

| Run suffix (20260912) | Result | First heartbeat publish | Maximum control callback |
|---|---|---:|---:|
| 031516Z-p2-ground-850c231a | observation_gap | 333.361 ms | 334.378 ms |
| 032016Z-p2-ground-b666a058 | passed | 0.360 ms | 3.602 ms |
| 032421Z-p2-ground-40479b0d | passed; native probe v1 | 0.349 ms | 2.220 ms |
| 032703Z-p2-ground-c2bb34ce | stale_state:vehicle_attitude; native probe v2 | 335.442 ms | 336.726 ms |

The fourth run measured only 2.317 ms thread CPU during that 335.442 ms
publish call. A diagnostic-only native wait probe captured a main-thread
`nanosleep` request of 333.000 ms from `SharedMemTransport::find_port`, through
`push_discard`, `send`, RTPSWriter, RMW publish and rclpy. Its monotonic
interval was 7418.925220975 to 7419.260231793. No native trace overflow occurred.
The same run's largest bag-write span was 1.239 ms and graph check 1.616 ms.
The measured stall is inside native Fast DDS shared-memory publication,
not the adjacent JSON event or bag write. The exact listener condition that
triggered the health check is not established.

The installed Fast DDS is 2.14.6. Its pinned source
[SharedMemGlobal.hpp](https://raw.githubusercontent.com/eProsima/Fast-DDS/v2.14.6/src/cpp/rtps/transport/shared_mem/SharedMemGlobal.hpp)
implements the listener health-check wait as one third of a default 1000 ms
timeout. This source and the actual native call stack support the attribution;
the old R2 recordings alone could not establish it.

The repair applies the official
[UDPv4 built-in transport](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html)
only to the owned controller process: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`,
`SKIP_DEFAULT_XML=1`, `RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS`. This removes
the measured SHM transport path. The private namespace contains loopback only.
Agent, PX4, bridge and QGC environments are unchanged. BEST_EFFORT, depth 50,
20 Hz steady timer, GUI profile, simulation speed, binary, dependencies and
all `p2_control.yaml` values remain unchanged. No hidden publication or warm-up
flight is introduced. The first real target is measured and recorded.

## Dispatch and recording limits

One executor thread still owns the Mission, cache, recorder and decisions.
The measured cause does not justify introducing a recorder worker, queue,
new executor or concurrent state mutation. Raw evidence persistence remains
synchronous and is **not guaranteed bounded** by this repair. No queue, enqueue,
worker drain or queue shutdown claim is made; queue-specific tests do not apply.
Future measured disk stalls would require separate evidence-path work.

Before each actual control publication, a guard requires the owner thread,
an active non-reentrant callback, a decision age no greater than 50 ms (one
20 Hz period), a valid graph check no older than 1.5 wall seconds, and fresh
state under the unchanged simulation/wall freshness limits. Graph inspection
still runs every one wall second. A second guard after the action detects
over-budget publication or evidence work. It cannot preempt blocking native
calls. A slow synchronous sink can stall a callback, but cannot silently allow
the following mode/arm command or a passed acceptance result. Pre-arm failure
ends grounded; airborne failure retains state-dependent recovery and freshly
validates any queued LAND request in the next owned callback.

Actual successful trajectory call entries establish a non-restarting prestream
window. Missing two-second coverage, non-advancing entries or an over-limit
gap deny mode publication. The target for the current decision is published
before a mode request, so coverage includes that actual call. The original
controller-consumption maximum remains 0.2 simulation seconds; it is never
replaced by the source-receive rate. The 50 ms action budget is a distinct
wall-time dispatch requirement, not a relaxation of that rule.

Evidence schema 2 adds `publication.entry_wall_s/entry_sim_s/return_wall_s`,
selected source identity and source age at dispatch. `sample.selection_monotonic_s`
names consumption time; `source_callback_entry_monotonic_s` preserves the
original callback time even when a duplicate is received. The legacy
`receipt_monotonic_s` alias remains readable. Historical readers and raw
artifacts are preserved; pre-repair diagnostic traces lack the new selected
sample fields and cannot retroactively gain them.

In the pre-repair diagnostic trace, `mission_tick.source_timestamp` is the
cache candidate at entry, including readiness and failed validation attempts.
Its legacy ground-evaluator field named `control_consumed_sample_gap_sim_s`
must be read as a candidate-gap diagnostic, not proof of successful Mission
consumption. Those first evaluations remain immutable. Repaired traces use
`selected_source_timestamp` only after active control validation and retain
attempted selections that subsequently fail the consumption-gap check.

Trace records are defensive snapshots with bounded capacity, sequence IDs
and per-operation call counts. Persistence occurs after control stops and
incomplete trace persistence or a failed sink cannot produce valid timing evidence.
There is no asynchronous drain to wait for. A sink blocked indefinitely is
bounded only by the existing outer supervisor deadline/owned cleanup, and
its incomplete recording invalidates acceptance. Write return and line-buffer
flush do not prove fsync or physical-disk durability.

The independent evaluator reconciles trace sequence/counts, actual publication
identities with CDR and JSON events, acquisition/publication/event ordering,
prestream coverage, dispatch duration and source age. Existing ROS/ULog
reference, all fixed motion windows, ACK, LAND and cleanup checks remain.
Native wait probing is allowed only in labelled ground diagnostics, never in
the new smoke or acceptance batch. The frozen batch includes timing config,
instrumentation, evaluator, native-probe source and runtime-library hashes.

Runtime qualification results are recorded below after the fixed sequence;
ground success alone does not establish P2 acceptance or hard real-time use.

## Regression boundaries

The focused tests inject clocks and sinks without real sleeps. They preserve
the original 0.344 s consumed-gap failure despite continuous source updates;
distinguish callback, selection and post-publish JSON times; reject stale
actions and missing/stale/competing graph ownership; reject a second owner
or reentrant decision; enforce transport-level ground command denial; and
require actual non-restarting prestream coverage. Existing R1/R2 tests retain
invalid observation versus inactive yaw, reference pairing, duplicate/receive
gaps, backwards time, clock stalls, ACK and terminal LAND rules.

A fake slow synchronous sink advances the injected clock by 333 ms: the
subsequent mode action is denied. Failed trace persistence raises instead of
issuing a completion receipt; overflow invalidates complete timing evidence.
Defensive-copy, sequence/count and persisted-content tests cover the bounded
trace. Independent reconciliation rejects a missing publication/bag record,
incomplete trace and a late native call even when the bag itself is complete.
No writer queue, worker thread or asynchronous shutdown was introduced, so
worker-queue ordering and incomplete-worker-drain tests are inapplicable.
There is no claim that this synchronous recorder can tolerate an indefinitely
blocked sink while maintaining a live control loop.

The reported whole-run simulation/wall progression ratio includes startup
clock acquisition, including the initial jump from zero to PX4 boot time.
It is descriptive and is not a measured physics speed or real-time factor.

## Recorded qualification outcome

All three fixed post-repair ground starts passed with no VehicleCommand,
continuous landed/disarmed evidence and at least 2.948 simulation seconds
of actual recorded prestream. Cold heartbeat publication was 0.103, 0.191
and 0.112 ms; the largest later control publication was 0.904 ms and the
largest ground control callback 4.525 ms.

New nominal `20260912T034917Z-p2-flight-c74f35ba` passed timing, reference,
yaw, all 16 independent motion windows, ACK/LAND, recording and cleanup.
Fresh batch `20260912T035329Z-p2-repeat-84bb288d` then passed **20/20** with
no retries, interruptions, omitted trials or changes to frozen inputs.
All 320 batch ROS/ULog windows passed. Actual prestream was at least 2.000 s
in every trial, and every flight ended landed/disarmed with owned cleanup.

Across the batch, the maximum cold heartbeat call was 0.193 ms; maximum
actual control publish 3.115 ms; control callback 10.630 ms; consumed-source
gap 0.072 s; source age at control/dispatch 0.032 s; publication gap 0.056 s;
and application callback-entry gap 0.049549 wall seconds. Trace overflow
and ULog dropout counts were zero; peak trace usage was 221,632/500,000.
The acquisition-through-persistence callback envelope reached 26.498 ms.
That is a measured same-process envelope, not a bound on physical disk
latency, and reinforces the synchronous-writer limitation described above.

The measured SHM repair is verified for this pinned simulation profile.
P2 acceptance is complete; P3/P4 are not implemented and a clean rebuild
remains not proven. The
[schema-4 evidence summary](evidence/v3_sim_p2_summary.json) lists all 37 R3
directories and preserves the complete historical schema-3 JSON value,
including its nested R1 and strict-v1 outcomes. Historical R2 failure spans
remain unavailable where they were not instrumented.
