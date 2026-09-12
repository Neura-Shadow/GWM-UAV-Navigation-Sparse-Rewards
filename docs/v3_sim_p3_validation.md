# P3 depth sensing and P3-R1 sample/time acceptance

P3 acceptance remains incomplete. The single new smoke,
`20260912T141235Z-p3-flight-778e3f60`, completed normal LAND/disarm and sealed
its recordings, but independent acceptance failed the unchanged prior-state
yaw checks. All sample-window, execution-timing and sensor checks passed.
Qualification was not run and remains 0/3; no further flight was attempted.

P3-R1's sample/time contract is verified by pinned-source review, tests and
the new sample-window evidence. Final v5 readiness and the complete seven-case
ground matrix passed with actual enclosing run IDs and finalized provenance.
Contract verification does not imply flight acceptance. Historical reanalysis
also remains failed on prior-state yaw consistency; all historical and new
failed attempts retain their original results. P3 is not marked complete.

## P3-R1 implementation and current gate

The [sample/time contract](v3_sim_p3_sample_time_contract.md) defines
`p3-sample-evidence-v2`. It preserves `p2-estimator-reference-v3`, controller
transport, mission windows and numeric limits. In particular, the 0.2-second
controller-consumption gap, 50 ms dispatch-age budget, source freshness,
tracking/drift limits, fixed dwell durations and ACK/LAND checks remain gates.
The evaluator semantics have changed explicitly; this is not a claim that the
original strict-timestamp evaluator was unchanged.

The two original failures are separate evidence cases. FINAL_HOVER includes
three controller evaluations reusing position callbacks while evaluation
clocks advance. RESTORE_INITIAL_YAW includes two distinct ULog estimator
outputs with equal publication time and different predictor sample times.
New evidence separates raw source records, callback deliveries, controller
selections, distinct source observations, outgoing publications and fixed
window records. Native timestamps remain integers. Publication-time groups
provide temporal coverage; all records remain available for tracking, mode,
health, reference and order checks. A repeated observation adds no new
position coverage or freshness. Equal-time estimator outputs are not silently
overwritten, and uncertain identity or ordering remains a rejection.

The minimal controller-cache correction uses the same source identity
contract. The sensor history also retains ordered equal-time outputs, records
callback/source references and rejects ambiguous association. A timestamp-only
map cannot choose a favorable ROS/ULog or image/PX4 match. Rendering is not
claimed as the cause of either historical timestamp case.

Every new sensor startup, readiness, observation, health, result, index and
Gazebo source-header record carries the launcher's enclosing run ID, including
recording under a `sensors` child directory. Frozen source/configuration inputs
include both packages, launchers and all evaluator arithmetic. A separate
measured runtime identity verifies exact build mirrors, installed sources,
the sensor build's controller dependency, pinned revisions, model/world assets
and PX4 binary. Ground/readiness/flight prerequisites compare those identities
and calibration while retaining separate predecessor references.

Final offline evaluation requires `runtime-finalized.json`: owned processes
have stopped, the sensor writer has closed, drained and fsynced, controller
results and artifact manifests are complete, and recorded hashes verify.
Dependent P3 evaluation waits for the matching completed P2 evaluation. Strict
JSON readers reject nonstandard numbers, duplicate keys and partial JSONL.
Original reports and old run IDs are never rewritten.

The final builds completed in:

| Package | Build run | Package hash |
|---|---|---|
| controller | `20260912T131520Z-p2-build-vytU8n` | `0d994904f2e43310219e4505c246cc8ecd2f95f2467ed3463a5308c124840456` |
| sensor adapter | `20260912T131525Z-p3-build-NnOzlQ` | `8f0514da5d4e93a3b07efe9983567b8d487f9f1a93cc108676c896794f4778a3` |

Final v5 ordinary verification passed **446 PX4/Gazebo focused tests** in
9.54 s and **1,246 full-suite tests, with 12 skips**, in 121.03 s. The unchanged
398-test AgentOps/C2 result remains valid and those tests are included in the
full suite. Python compilation, Bash syntax, ShellCheck (`-x -e SC1091`) and
`git diff --check` passed. Linux build and installed-package tests passed.
These checks establish source/build verification, not flight acceptance or a
clean rebuild.

The first immutable analysis, `p3-r1-historical-adjudication-v2.json`, failed
controller-attitude reconstruction. A separate read-only diagnostic verified
that normalization reproduces all 2,086 recorded controller yaw values exactly.
The explicitly revised report `p3-r1-historical-adjudication-v2r1.json`, SHA-256
`79b6661b40b0fe1e107c9c3ae15f02aa76dd6a621604435976eb4dfc95ac4ed1`,
passes the eight controller windows and sixteen independent ROS/ULog windows.
It still **fails overall**: 79 of 135 prior-state yaw comparisons exceed the
unchanged 1e-5 rad tolerance; the maximum difference is
7.283687591552734e-5 rad. The exact state consumed by PX4's multicopter
controller is unproven in those historical records. That limitation does not
justify changing the tolerance, selecting favorable matches or awarding
qualification credit. Both analysis versions and the original failed reports
remain immutable; neither analysis represents a new flight.

The runtime freeze used for the first P3-R1 readiness and plane2 attempt is
`state/p3-r1-implementation-20260912T130622Z/frozen-runtime-inputs-v2.json`
under the Linux workspace, SHA-256
`3f1a4f77a9d56f0fb73de81568165385083f0ef6ae721c5066e77050a8b46102`.
Read-only run `20260912T132733Z-p3-observe-1957cc82` passed the owned launcher,
runtime seal and both independent evaluations with these final inputs. Every
record used the enclosing run ID. It recorded 739 raw frames (908,083,200 bytes).
Across the control window [16.072,31.832] s, it delivered 478 frames at
30.297 Hz with a 0.036 s maximum gap, and 158 observations at 9.9949 Hz.
Maximum observation age was 0.068 s and maximum image/state difference 0.012 s.
All 158 associations matched independent PX4 records, with zero ambiguity and
zero missing source frames. These are new read-only results, not flight credit.

Plane2 `20260912T132932Z-p3-plane2-514444e3` completed its fixed collection
window [8.820,38.820] s, with 1,183 accepted/written frames and a closed,
drained/fsynced recorder, zero overflow and no adapter errors. Its five
registered process leaders exited, but sealing found an unregistered sleeping
PID 236. The original status remains **failed**; no sealed ground acceptance
or matrix credit was awarded.

Read-only inspection of the exact installed ROS CLI source established that
`ros2 topic info -v` invokes `NodeStrategy`, which spawns a daemon by default.
That subprocess is outside the launcher's registered child groups. The
recorded QoS output confirms the inspection succeeded, but PID 236's command
line, executable and group were not captured, so its attribution to that
daemon remains a source-backed inference. The installed CLI supports explicit
`--no-daemon`; no `ROS2CLI_NO_DAEMON` implementation reference was found in
ros2cli/ros2topic. Only that flag was added to ground QoS inspection, retaining
the existing timeout and every finalization check.

The immutable diagnosis is
`p3-r1-ground-finalization-daemon-diagnostic-v1.json` within the failed run,
SHA-256 `d1590a16a45ffc5c2682c46264b541220eb7b535f22d97858df610daf2347573`.
It records installed source hashes and the attribution limit. The original
summary remains byte-identical, SHA-256
`c39d78952f23fd64da05fdceb6abf826f62a0428e5f5e722a0078c797643a802`.

The retained v3 freeze is
`state/p3-r1-implementation-20260912T130622Z/frozen-runtime-inputs-v3.json`,
SHA-256 `afe676da5982e35fd1e654d2b14adcb0227bb1d53981186131d69d80e616bb55`.
It records the daemon-free ground inspection, revalidation of each ground
run's raw artifact manifest and complete selected-record health checks with
the existing terminal-state exception. It also binds finite raw/selected
position, velocity and yaw, vehicle identity and raw velocity evidence.
These are launcher/evaluator changes; the controller and sensor package builds
above are unchanged, and no package rebuild is claimed. The disk preflight
recorded 930,577,448,960 free bytes against the declared 64 GiB campaign budget
plus 20 GiB reserve.

Read-only run `20260912T134432Z-p3-observe-113909cf` passed the launcher and
runtime seal but failed independent P2 evaluation with
`invalid_integer_timestamp`. Its initial ULog `vehicle_status` and
`failsafe_flags` records contain true native uint64 publication timestamp zero.
The pinned source and shared uint64 contract allow zero, while the offline
reader had added an unsupported `>0` constraint. The dependent P3 evaluation
also failed; neither result is overwritten or credited as readiness.

The failed run retains
`p3-r1-readiness-invalid-timestamp-diagnostic-v1.json` (SHA-256
`0e83a2f278b2594602a40720804e8ccda7088536d7efe45cdb833ae1e8ad49f7`)
and `p3-r1-readiness-native-zero-publication-diagnostic-v1.json` (SHA-256
`73a68eaf15691af660027e2df0dc546c8186a8acdca080fcbb40e12517497b20`).
The offline correction now uses the shared `stamp_us` validator. Estimator
sample timestamps remain strictly positive; no online cache, scheduling,
flight limit or estimator behavior changed to address this reader mismatch.

The retained v4 freeze is
`state/p3-r1-implementation-20260912T130622Z/frozen-runtime-inputs-v4.json`,
SHA-256 `34532c83229d7364d8f3bce789c33a8c99aae7444d6daf023d6a2ff197eb7bd7`.
Disk free space was 929,706,799,104 bytes. Fresh read-only run
`20260912T135042Z-p3-observe-f4e7bdeb` passed launcher, runtime seal, independent
P2 control/recording and P3 sensor checks under v4. It recorded 675 raw frames
(829,440,000 bytes). The control window [13.548,29.428] s contains 481 frames
at 30.303 Hz with maximum source gap 0.036 s, and 159 observations at
9.994939 Hz. Maximum observation age was 0.064 s and image/state difference
0.012 s. All 159 state associations matched exactly; unmatched, ambiguous,
reused-match and missing-source counts were zero. Run identity and recording
finalization passed; this remains read-only evidence with no flight credit.

Plane2 `20260912T135209Z-p3-plane2-adca8021` passed launcher, runtime seal and
independent evaluation under v4. Matrix
`20260912T135352Z-p3-ground-matrix-2f3837c3` stopped at its first case, plane4
`20260912T135358Z-p3-plane4-f54aa882`, when
`ros2 topic info --no-daemon -v` returned `Unknown topic` for the image stream.
The remaining matrix cases were not run. The adapter recorded 264 raw frames
over [0.008,8.680] s with maximum source gap 0.036 s, including 112 frames
after readiness, and all 32 post-readiness health records were fresh. Its
writer closed, drained and fsynced with no overflow or adapter errors.
The requested fixed collection window had not started.

Installed ROS CLI source shows that each direct inspection creates its own
node and defaults to 0.5 s of graph discovery. Actual sensor flow was present;
incomplete discovery in that new CLI node is a source-backed explanation,
but its graph-cache state was not recorded, so the exact cause remains an
inference. The sole launcher correction adds `--spin-time 3` alongside
`--no-daemon`, retaining the outer 10 s timeout and all runtime, recording,
geometry and flight budgets. The immutable
`p3-r1-ground-direct-discovery-diagnostic-v1.json` in the failed plane4 run
records raw, readiness and installed-source hashes, SHA-256
`c6a61196d8e117b01d475ac810c1a03180a2ccf9309a9b980059c5703bb44f2b`.
The failed run and matrix summaries remain unchanged.

The current v5 freeze is
`state/p3-r1-implementation-20260912T130622Z/frozen-runtime-inputs-v5.json`,
SHA-256 `202e17ebbcf9a3532445ff0a541bc2fc1f06a8a3b9c95c913cafa786b13146b4`.
Disk free space was 927,048,028,160 bytes against the same 64 GiB campaign
budget and 20 GiB reserve. The package builds and evaluator arithmetic are
unchanged from v4. Fresh readiness
`20260912T140041Z-p3-observe-34709c89` passed launcher, runtime seal and both
independent evaluations after final ordinary verification completed. It
recorded 682 raw frames (838,041,600 bytes). The control window
[13.592,29.448] s contains 481 frames at 30.303 Hz with maximum source gap
0.036 s, and 159 observations at 10.01521298 Hz. Maximum observation age was
0.076 s and image/state difference 0.012 s. All 159 associations matched
exactly; unmatched, ambiguous, reused-match and missing-source counts were
zero.

V5 plane2 `20260912T140217Z-p3-plane2-2de3a546` then passed. The matching
matrix `20260912T140437Z-p3-ground-matrix-1e3494df` passed all seven cases,
with sealed recordings and independent evaluation for every constituent run:

| Ground case | Run ID | Result |
|---|---|---|
| plane2 | `20260912T140217Z-p3-plane2-2de3a546` | passed |
| plane4 | `20260912T140443Z-p3-plane4-4a162bb4` | passed, including separate post-window scene change |
| plane6 | `20260912T140557Z-p3-plane6-55a7ff9c` | passed |
| oblique | `20260912T140717Z-p3-oblique-31317a8b` | passed |
| asymmetric | `20260912T140829Z-p3-asymmetric-377736f0` | passed |
| out_of_range | `20260912T140947Z-p3-out_of_range-d2567c2d` | passed |
| interruption | `20260912T141116Z-p3-interruption-d6519c7b` | expected interruption detected; acceptance passed |

The nominal ground source rates were 30.303–30.304 Hz with a maximum source
gap of 0.036 s. The minimum processed rate was 9.990644 Hz and maximum
observation age 0.076 s. The separate scene-change check recorded fourteen
observations at 5.00000095367 m. Interruption acceptance records expected
stream loss separately from nominal geometry results.

With these prerequisites complete, the single new nominal depth smoke
`20260912T141235Z-p3-flight-778e3f60` ran under the same v5 freeze. The launcher
passed, the controller reached `COMPLETE`, normal LAND ended in landed/disarmed
state with no failsafe, owned-process cleanup passed, and the runtime was
finalized. Independent P2 recording validation passed. Independent flight
acceptance **failed solely at prior-state yaw consistency**: 68 of 135
comparisons failed, comprising 63 heading mismatches and eight age violations,
with three comparisons failing both. Maximum heading error was
8.08238983154297e-5 rad against the unchanged 1e-5 rad limit; maximum prior-state
age was 0.016 s against the unchanged 0.008001 s limit. All eight controller
windows, sixteen independent ROS/ULog windows and execution-timing checks
passed. These passing components do not cancel the yaw failure.

The sensor recorder retained 3,297 raw frames (4,051,353,600 bytes). Across
the control window [15.028,113.448] s, 2,982 frames delivered 30.303338 Hz with
a maximum source gap of 0.036 s; 984 observations delivered 9.999593 Hz.
Maximum observation age was 0.084 s and image/state difference 0.012 s.
All 984 associations matched exactly, with zero unmatched, ambiguous,
reused-match or missing-source cases and zero sensor errors. Independent P3
composite acceptance **failed only `control_acceptance`**, reflecting the
independent yaw failure. Final observation identity and sensor provenance
passed; sensor success provides no flight qualification credit.

The full independent P2 evaluator was still CPU-active after more than
180 s of elapsed execution before it completed. The existing qualification
wrapper's 90 s control-evaluation timeout therefore remains unqualified by
runtime evidence. That limit and all frozen inputs were left unchanged after
this sole smoke. Fixing the yaw evidence would not establish that the batch
could complete within its existing evaluator budget.

The smoke failure stopped progression. Qualification was **not_run, 0/3**;
no additional smoke or qualification flight followed, and no P4/P5 work began.
Earlier attempts retain their original inputs and outcomes. The
[operator procedure](../simulation/px4_gazebo/README.md#p3-r1-final-input-readiness-ground-and-flight-procedure)
documents the required chronological prerequisites; the qualification
prerequisite remains unsatisfied by this campaign.

| P3-R1 gate | Current result |
|---|---|
| Sample/time contract | verified by source review, tests and all new sample-window checks; flight acceptance remains failed |
| Final-build observation identity | passed in sealed v5 readiness and the new smoke; exact image/PX4 associations verified |
| Final-input plane2 | passed under v5: `20260912T140217Z-p3-plane2-2de3a546` |
| Final-input ground matrix | passed: `20260912T140437Z-p3-ground-matrix-1e3494df`; all seven cases and scene change |
| New camera-equipped smoke | failed independent prior-state yaw acceptance: `20260912T141235Z-p3-flight-778e3f60`; normal LAND/disarm and finalization passed |
| Coexistence qualification | not_run; 0/3 |
| Qualification evaluator budget | existing 90 s timeout not runtime-qualified; standalone evaluator observed CPU-active beyond 180 s |
| P3 overall | incomplete |
| P4 avoidance / P5 model inference | not_implemented |
| P6/P7 and v3-2 onward | not_implemented |
| Clean rebuild | not_proven |

The [current schema-2 evidence summary](evidence/v3_sim_p3_summary.json) tracks
the repair, final run identities, independent outcomes and incomplete campaign.
The [schema-1 history](evidence/v3_sim_p3_summary_v1.json) remains byte-identical.

## Preserved P3 implementation history

The sections below retain the original P3 component measurements and failed
flight campaign delivered in `76f014d`. Real rendering, one-way bridging,
ground geometry and interruption checks passed for their recorded builds.
The original depth smoke completed normal LAND/disarm but failed independent
acceptance; its qualification count remains 0/3. These are historical
component results, not new final-build measurements or P3-R1 acceptance.

The source base was `6204fedb9ac8a2b554ff825c93159a509a1b91ad` on
`v3/gwm-uav-c2-agentops-planning`; origin matched after fast-forward-only
synchronization. The only pre-existing local item was untracked `.codegraph/`.
The archive still resolves to `cb304c7a64b2edd57673a3323c0ed2e7d2923453`.
No dependency installation, upstream edit, tag, release or main merge occurred.
The standalone design-report attachment was unavailable; the operator's
pasted requirements were used.

The [sensor contract](v3_sim_p3_sensor_contract.md) defines the implementation;
the [preserved schema-1 summary](evidence/v3_sim_p3_summary_v1.json) retains exact metrics,
hashes, source revisions and attempt identities. Raw artifacts remain under
`/home/joker0625/uav_autonomy/runs/` in Ubuntu-24.04. Run IDs below are relative
to that directory. Recorded outcomes are not overwritten by this report.
The schema-1 copy is byte-identical, with SHA-256
`c51b876b836a9efdf8f287e774ecb3a3d9dc09d13160fac2ed6b52123db2d2c0`.

## Sensor/model and bridge

PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` uses models submodule
`b6127f4ec20de867e215fb5f78ae88b80f371909`: merged x500_depth, x500,
x500_base and OakD-Lite. Native camera mass adds 0.061 kg to the
2.0643076923076923 kg x500. No dynamics or sensor overlay was introduced.
Depth remains 640x480 at 30 Hz, horizontal FOV 1.274 rad, near/far 0.2/19.1 m;
the native RGB sensor remains configured but is not bridged.

Actual Ogre2 rendering uses WSLg/Mesa llvmpipe LLVM 20.1.2, OpenGL 4.5.
The selected flight mode is Gazebo server-only with offscreen QGC monitoring.
The initial GUI read-only attempt exposed slower-than-real-time execution;
headless physics alone was never treated as camera proof. The installed
`GstCameraSystem` plugin could not load; actual depth data and the separate
4-to-5 m scene-change test establish the working depth path independently.

Owned `/depth_camera` (`gz.msgs.Image`) and `/camera_info`
(`gz.msgs.CameraInfo`) map GZ_TO_ROS to
`/gwm/sensors/front_depth/image_raw` and `camera_info`, with BEST_EFFORT,
VOLATILE and configured queues of eight. Runtime layout is little-endian
32FC1, step 2560, 1,228,800 bytes/frame. CameraInfo has
fx=fy=432.496042035043, cx=320, cy=240, undistorted P=K, identity R and no
ROI/binning. Its recorded static identity is
`9f0816e89913bd9e8785cb403ce3f13b3f216cf9cc5cee28f9aa01c5538d7e6d`.
In the smoke it arrived 3,282 times from simulation time 7.528 to 115.8 s.

Raw `camera_link` is preserved. Optical XYZ is right/down/forward; normalized
metadata names `gwm_front_depth_optical`. The complete include/link/sensor
composition gives optical-to-base translation `[0.13233,0,0.02078]` m:
body FLU=`[Z+0.13233,-X,-Y+0.02078]`, FRD=`[FLU.x,-FLU.y,-FLU.z]`.
No TF or flight-control publisher exists in the adapter. PX4 state is matched
by acquisition time using a bounded history; Gazebo truth enters only the
offline ground evaluator.

## Fixed ground measurements

Selected plane2: `20260912T111501Z-p3-plane2-707b5913`.
Fixed remaining matrix: `20260912T111718Z-p3-ground-matrix-2c5c064c`.
All six remaining cases ran once under frozen inputs. Each nominal case used
30 simulation seconds and ROI `[160,80,480,200)` (38,400 pixels/frame), fixed
before acquisition. The independent evaluator uses actual settled pose and
analytic visual-surface intersections, without importing production geometry.
The model settled at z=-0.0130001861691 m; optical world z=0.2477798131191 m.
Start/end pose drift was zero at recorded precision.

| Case | Expected median Z, m | Measured median Z, m | Max median / p95 absolute error, m | Result |
|---|---:|---:|---:|---|
| plane2 | 1.999999996 | 2.000000238 | 0.000000243 / 0.000000482 | passed |
| plane4 | 3.999999994 | 4.000000954 | 0.000000957 / 0.000000963 | passed |
| plane6 | 5.999999991 | 6.000001431 | 0.000001438 / 0.000001443 | passed |
| oblique, yaw 0.35 rad | 3.998313409 | 4.000000954 | 0.001687307 / 0.002185245 | passed |
| asymmetric main ROI | 3.999999993 | 4.000000477 | 0.000000722 / 0.000000963 | passed |
| out-of-range plane | 24.999999967 | unavailable (+Inf) | not applicable | correct unknown |

Each in-range ROI had 100% valid coverage. Each nominal source ledger had
909 frames, except the inclusive asymmetric endpoints contained 910. All
Gazebo sequences were contiguous, with zero source-to-ROS/raw loss.
Out-of-range error fields in the original evaluator are accumulator zeros
when no finite pixels exist; they are **not zero-error distance measurements**.
Its ROI valid fraction was zero, and 143,234,766 +Inf pixels were counted
across full selected frames. Live NaN, -Inf and 16UC1 are not claimed.

The asymmetric left/high fixture measured 3.000000715 m against 3 m in
ROI `[195,55,205,65)`; the right/lower fixture measured 2.000000238 m against
2 m in `[455,115,465,125)`. Independent point checks verify image-right to
negative body-FLU Y, image-up to positive body-FLU Z and positive forward X,
including the optical-center translation. The oblique range hypothesis had
0.198687636 m median error, versus 0.001687307 m for optical-axis Z.

The plane4 post-window move measured 5.000000954 m against 5 m over 14
observations. This separate changing-scene proof does not alter the fixed
measurement window. Static-window byte equality is expected in a settled scene.

The interruption run `20260912T112212Z-p3-interruption-f2fd78a7` intentionally
stopped the bridge after readiness. Stale/unavailable health appeared after
0.244895 wall seconds (limit 1.2 s); the old frame was not kept fresh. Its
original status is `passed_expected_failure_diagnostic`, not valid-depth
measurement credit. No airborne fault injection occurred.

## Timing and resource use

The first default-middleware experiments lost native-size frames. The
separately frozen `p3-x500-depth-native-shm64-v1` keeps resolution/rate and all
acceptance limits, and supplies 64 MiB SHM plus UDPv4 discovery only to the
sensor bridge and adapter. The controller retains R3's UDPv4-only transport,
skipped default XML and synchronous publication. There is no global transport
change. Process separation is not a claim of CPU/GPU/disk isolation.

Nominal ground delivery was 30.3000-30.3031 Hz, max source gap 0.036 s,
processing 8.7490-9.8328 Hz and maximum observation age 0.076 s. Worst measured
processing call was 0.028185 wall s. Raw recorder capacity was eight, observed
peak one and overflow zero; newest-frame processing overwrites were intentional
and never counted as recorded-frame losses. The raw image rate is about
37.2 MB per simulation second. Disk reserve is 20 GiB before readiness.

Smoke sensor window: 13.504-112.340 simulation seconds. It contained 2,995
frames at 30.302417 Hz, max gap 0.036 s, 989 observations at 10.002835 Hz,
maximum age 0.076 s, maximum image/PX4 mismatch 0.012 s and 989 independent
PX4 matches. All 3,282 received frames were recorded, totaling 4,032,921,600
raw bytes; source loss, repeated acquisition stamps, queue overflow and
sensor errors were zero. There were 2,198 intentional processing overwrites;
the recorder drained and fsynced successfully. These sensor checks passed.

The unchanged P2 timing evaluator passed: control-consumed source gap 0.068 s,
source age at dispatch 0.032 s, actual publication gap 0.060 s, longest publish
call 1.955 ms and control callback 14.113 ms. The steady control callback rate
was 19.99974 wall Hz; observed simulation progress was 0.89709 sim s/wall s.
Trace capacity was 500,000 records, 226,006 used, zero overflow. These results
do not override the separate strict flight-window failure or prove hard
real-time operation, GPU utilization or a whole-process memory ceiling.

## Flight coexistence and exact failed gate

The final read-only connection `20260912T113246Z-p3-observe-6d982d6b` passed
independent P2 recording and P3 sensor/state checks before flight. The flight
world's closest fixture surface is x=10 m, leaving at least 7.6 m from the
2 m horizontal center envelope plus 0.4 m rotor envelope. No observation
changes the original bounded P2 flight path.

One new smoke ran: `20260912T113430Z-p3-flight-0dc41a18`. Runtime returned
COMPLETE; commands 176/400/21 had accepted, routed ROS/ULog ACKs after
0.016/0.012/0.008 sim s. Actual LAND, landed and disarmed states were observed;
no failsafe occurred. Recording integrity, reference reconciliation, yaw
ownership and timing passed. Owned processes were stopped, with QGC requiring
the existing bounded SIGKILL fallback. No process outside the owned namespace
was targeted.

Independent acceptance still failed the original `min(gaps)>0` condition:

- FINAL_HOVER [100.740,105.760] had 165 control-event samples, including
  repeated position timestamps 102.516, 104.464 and 104.936 s. The live Mission
  window had 162 samples; the independent event selection included all three
  repeats. Position values/source callback identities were repeated while
  processing/ROS clock times advanced.
- RESTORE_INITIAL_YAW [95.700,100.700] had 625 ULog positions. Two adjacent
  records shared publication timestamp 97.568 s but had different sample
  timestamps (97.552 and 97.560 s), positions and headings. This is not simply
  duplicate identical payloads. Reset counters did not change in that pair.

The read-only diagnostic is `p3-smoke-window-diagnostic.json` in that run.
The evidence establishes the precise gate, not a unique scheduler/renderer
root cause. No sample was deleted, deduplicated or shifted; the P2 evaluator
and its numeric limits were not changed. The P3 composite result failed only
its `control_acceptance` check. A completed controller mission is insufficient
for acceptance. Qualification was not started and no additional smoke was run.

## Retained attempts and source versions

All IDs below remain beneath the Linux runs directory. Exact evaluator hashes
and original statuses are included in the schema-1 summary.

| Run ID | Original outcome / interpretation |
|---|---|
| 20260912T110418Z-p3-plane2-c28c38a7 | failed source gap 0.132 s; initial static-frame-change assertion was also unsuitable; no source probe |
| 20260912T110851Z-p3-plane4-495e0dc5 | failed supervisor read of a partial JSONL line |
| 20260912T111048Z-p3-plane4-ccdc6885 | old evaluator said passed; later coverage audit found 96/909 source frames missing; no recording acceptance credit |
| 20260912T111419Z-p3-plane2-fc52d25e | failed namespace interface check before adapter startup; host sysfs was the wrong interface inventory |
| 20260912T111501Z-p3-plane2-707b5913 | passed selected SHM64 plane2 |
| 20260912T111721Z-p3-plane4-83a5b7a8 | passed fixed matrix plane4 plus separate render challenge |
| 20260912T111820Z-p3-plane6-22a4ed22 | passed fixed matrix plane6 |
| 20260912T111919Z-p3-oblique-eed68041 | passed fixed matrix optical-Z check |
| 20260912T112015Z-p3-asymmetric-477a3ec5 | passed fixed matrix signed axes/offsets |
| 20260912T112115Z-p3-out_of_range-aefba15e | passed correct-unknown behavior |
| 20260912T112212Z-p3-interruption-f2fd78a7 | passed expected-failure ground diagnostic |
| 20260912T112516Z-p3-observe-0eb41cf2 | P2 read-only passed; P3 failed 6.3283 Hz processed rate under GUI load |
| 20260912T112834Z-p3-observe-e29ce760 | startup_connection_estimation_timeout; remained grounded/disarmed; recorded flags exceeded 2 wall s freshness under slow simulation |
| 20260912T113246Z-p3-observe-6d982d6b | passed server-only read-only control and sensing |
| 20260912T113430Z-p3-flight-0dc41a18 | normal LAND; independent smoke failed, no qualification |

The GUI-timeout run also retains premature offline results produced before
supervisor finalization: P2 could not see complete controller recording, and
P3 could not see `sensor-artifacts.json`. The smoke retains a premature P3
invocation in `p3-coexistence-prerequisite-failure.json` (P2 offline result
not yet available); after that prerequisite completed, the full independent
P3 evaluation was executed and failed control acceptance. These are offline
invocation failures, not additional flights. Initial unsuccessful SDF
resolution and all build logs remain in Linux state/runs.

Ground measurements used sensor package `9218163a37487797945d8a66581201798158561b1a73d32a06316e7fe29b7ca3`.
Later phase-accumulator scheduling and CameraInfo accounting were built before
the final read-only/smoke package `dad7d3a750d46ffbde787c3b453e24f69601d8be094ac78777f8be0235276db8`.
Geometry, decoder, extrinsics, native sensor workload and numeric limits were
unchanged. This is component evidence across explicitly recorded versions,
not a single final-source ground-and-flight qualification.

Final review corrected the observation envelope's run ID: nested flight
recording previously emitted `sensors`; it now emits the enclosing unique run
ID. Original runtime messages remain unchanged and are attributable through
their hashed enclosing run. The correction has a red/green pure test and a
new installed build (`096bf8e94a58478845b32a4e206713f48930ccf0e2361899273e87bbfe81a3d2`),
but no new runtime or flight acceptance is claimed for that build. A future
authorized repair must establish fresh exact-input readiness and acceptance.

Historical P2 schema 4 remains byte-for-byte unchanged: x500 smoke
`20260912T034917Z-p2-flight-c74f35ba` and batch
`20260912T035329Z-p2-repeat-84bb288d` (20/20). Strict-v1/R1/R2 failures remain
in that history. None of those flights is relabelled x500_depth.

## Historical verification and status at 76f014d

Final Windows Anaconda regression passed: 33 P3 tests, 210 combined
P3/P2/reference/yaw/timing tests, 398 C2/AgentOps tests, and the full ordinary
suite **1045 passed, 12 skipped**. Linux ROS builds passed with two installed
controller tests and one installed sensor test. Python compilation, all 13
Bash scripts' syntax and ShellCheck (26 checks), and whitespace checks passed.
ShellCheck excludes only SC1091 for dynamically sourced installed ROS setup
files. Ordinary pytest does not launch simulation or ROS nodes. No new
dependency was needed. The final namespace/process audit found no active P3
run processes, the exclusive runtime lock was available, and all four upstream
source repositories remained clean.

The first full Windows regression aborted in NumPy matrix multiplication.
A minimal PyTorch-then-NumPy reproduction reported `OMP: Error #15` from
duplicate `libiomp5md.dll` initialization. Fixed three-component ray rotations
now use unoptimized NumPy einsum/elementwise contractions instead of invoking
BLAS. No DLL, package or OpenMP safety setting was changed. The 33 P3 tests
pass after initializing PyTorch, and an offline arithmetic comparison across
all 230,400 fixed fixture ROI rays found identical labels, identical ray
factors and at most 8.89e-16 m difference. Original ground evaluator files
and results are retained; the final evaluator hash differs. This comparison
does not rerun or relabel the failed flight acceptance.

| Requirement | Status |
|---|---|
| P3 sensor rendering | passed, recorded component evidence |
| P3 bridge | passed, selected native SHM64 profile |
| P3 depth geometry | passed, fixed ground matrix |
| P3 observation/health contract | passed component checks; final run-ID correction runtime unproven |
| P3 camera-equipped smoke | failed independent acceptance |
| P3 coexistence qualification | incomplete; not run, 0/3 |
| P4 avoidance | not_implemented |
| P5 model inference | not_implemented |
| P6/P7 and v3-2 onward | not_implemented |
| Clean rebuild | not_proven |
