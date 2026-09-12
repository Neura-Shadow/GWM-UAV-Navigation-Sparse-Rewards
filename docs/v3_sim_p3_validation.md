# P3 depth sensing: measured ground pass, flight acceptance incomplete

P3 sensing is implemented, and real depth rendering, one-way bridging,
independent ground geometry and the bounded interruption diagnostic passed.
The one new camera-equipped smoke completed its maneuver, normal LAND and
disarm, but **failed independent flight acceptance**. No qualification flight
was launched: 0 of the required 3. P3 is incomplete and P4 is not recommended
until the declared P3 acceptance passes.

The source base was `6204fedb9ac8a2b554ff825c93159a509a1b91ad` on
`v3/gwm-uav-c2-agentops-planning`; origin matched after fast-forward-only
synchronization. The only pre-existing local item was untracked `.codegraph/`.
The archive still resolves to `cb304c7a64b2edd57673a3323c0ed2e7d2923453`.
No dependency installation, upstream edit, tag, release or main merge occurred.
The standalone design-report attachment was unavailable; the operator's
pasted requirements were used.

The [sensor contract](v3_sim_p3_sensor_contract.md) defines the implementation;
the [schema-1 summary](evidence/v3_sim_p3_summary.json) retains exact metrics,
hashes, source revisions and attempt identities. Raw artifacts remain under
`/home/joker0625/uav_autonomy/runs/` in Ubuntu-24.04. Run IDs below are relative
to that directory. Recorded outcomes are not overwritten by this report.

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

## Verification and remaining status

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
