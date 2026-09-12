# PX4 / Gazebo operator simulation lane

This directory implements the [operator-directed simulation amendment](../../docs/v3_simulation_scope_amendment.md).
It does not add a handler or capability to AgentOps. All project documentation
and code changes remain in the Windows Git checkout. Raw logs, upstream
checkouts, builds, and Python environments remain in the Linux filesystem.

## Pinned baseline and source decisions

`configs/versions.lock.yaml` pins Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic,
PX4 v1.17.0, px4_msgs release/1.17 at a fixed commit, and DDS Agent v2.4.3.
The version and flight configuration files use the JSON subset of YAML 1.2 to support a
standard-library-only preinstallation parser. Actual versions and hashes are
collected after a successful build; unresolved fields are never evidence of
an installed or built component. A recorded manifest does not prove a clean
rebuild.

Official sources reviewed:

- [PX4 v1.17 release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0)
- [PX4 v1.17 DDS compatibility](https://docs.px4.io/v1.17/en/middleware/uxrce_dds)
- [Jazzy/Harmonic pairing and vendor packages](https://gazebosim.org/docs/harmonic/ros_installation/)
- [PX4 WSL workflow](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl)
- [Gazebo launch and headless mode](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [Versioned PX4 installer](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/Tools/setup/ubuntu.sh)

The pinned installer supports `--no-nuttx --no-sim-tools`. We use those options
and install Harmonic through the ROS Jazzy vendor provider, avoiding OSRF/ROS
provider mixing and unrelated NuttX/hardware toolchains. The installer uses
`--break-system-packages` on Python 3.12; our recorded copy removes that one
override and adds apt `--no-remove`. The upstream checkout is unchanged. The
copy runs inside `venv-px4`; protections on system Python remain enabled.
This is a package-management adaptation, not a major version or architecture
change. The exact diff and hashes are saved in the setup run.

DDS Agent uses Jazzy's Fast DDS/Fast CDR and its upstream default spdlog 1.9.2.
Ubuntu's system spdlog 1.12/fmt 9 failed to compile Agent v2.4.3; the failed
`build/` tree is retained, and `build-jazzy-upstream-logger/` records the
compatible rebuild. The pinned Agent source is unchanged.

The manifest also records PX4's fetched OpticalFlow external-project HEAD
and recursive submodules. The pinned PX4 CMake source requests `master` for
that external project; apt/pip installation also uses live repositories.
Recorded package and source versions describe this build, but do not establish
a reproducible clean build from a frozen dependency mirror.

## Setup and build

Run from a clean Bash shell in the existing `Ubuntu-24.04` WSL distribution:

```bash
cd /mnt/d/GWM-UAV-Navigation-Sparse-Rewards
bash simulation/px4_gazebo/scripts/check_environment.sh
bash simulation/px4_gazebo/scripts/setup_environment.sh
bash simulation/px4_gazebo/scripts/setup_environment.sh --apply
bash simulation/px4_gazebo/scripts/build_baseline.sh --build
```

The default setup command only displays its plan. Apply requires normal sudo
authentication; authenticate in your own WSL terminal if needed. Matching
checkouts are reused, incompatible/dirty checkouts are rejected, and no reset,
rebase, removal, OS upgrade, GPU-driver change, or distro change is performed.
Interrupted runs keep their logs and exit code; rerun the same command to
resume from installed packages and existing checked-out sources. Only
successful setup appends `state/setup-completed.txt`; build records each
completed component and creates `state/p0-built.json` only after all checks.

Default Linux root: `$HOME/uav_autonomy` (`GWM_SIM_ROOT` may name another
dedicated directory beneath Linux HOME). Builds use two jobs. Repository
regressions use `C:\Users\zongx\anaconda3\python.exe`; Linux helper/colcon
uses `/usr/bin/python3`, and PX4 uses `$HOME/uav_autonomy/venv-px4/bin/python`.
No shell startup configuration or default Python is changed.

The process-local Linux PATH and CMake/Python search variables are sanitized
before sourcing Jazzy. Checking only `CONDA_PREFIX` is insufficient on WSL:
an initial configure found Windows Anaconda Protobuf through inherited PATH.
That failed build remains in `build/px4_sitl_default`; the validated lane uses
the upstream Makefile's `BUILD_DIR_SUFFIX=_linux` option and
`build/px4_sitl_default_linux`. No failed build tree is deleted or reset.

## Explicit P1 execution

The default x500 safety policy requires a GCS connection. Install the optional
official QGroundControl v5.1.4 monitor before running this validated lane:

```bash
bash simulation/px4_gazebo/scripts/setup_qgc_monitor.sh --apply
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
bash simulation/px4_gazebo/scripts/run_p1_baseline.sh \
  --run --allow-simulated-flight --qgc-monitor
```

Without arguments no runtime starts. With missing gates execution is blocked.
Omitting `--allow-simulated-flight` performs only boot/health/log checks.
Use `--headless` if GUI rendering is unavailable; that result proves only
physics/state execution, not GUI rendering or depth-image validation.

The documented upstream `make px4_sitl gz_x500` target is checked at build
time. For per-trial evidence isolation, the runner invokes the same built
PX4 binary with `PX4_SIM_MODEL=gz_x500`, the generated `etc` tree, instance 71,
and a fresh run-local rootfs containing the generated `gz_env.sh`. This uses
the pinned checkout's `px4-rc.gzsim` spawn path for one `x500_71` in `default`.
It does not attach to an existing world/model. A unique Gazebo partition and
private network/PID namespaces isolate the session; only loopback exists.
The DDS Agent is built but is not launched for P1. No competing controller,
remote forwarding, physical bridge, or arbitrary endpoint discovery is used.

QGroundControl's official AppImage SHA-256 is verified before extraction and
each launch. It runs as the mapped current user, with fresh run-local config,
cache, data and temporary directories. USB/serial autoconnect, forwarding,
joystick and follow-target are disabled. Only loopback MAVLink telemetry and
the normal GCS heartbeat are enabled; the runner never operates its control
UI. This satisfies normal GCS health checks without disabling a failsafe.
Without `--qgc-monitor`, this baseline can stop at preflight with GCS missing.

The sole control source is the owned PX4 console. The selected v1.17 source
supports `commander check`, `commander takeoff` (normal health checks and
normal arming), and `commander land`. Force arming and health-rule changes
are prohibited. The only trial overrides are `MIS_TAKEOFF_ALT=2.0` and
`SDLOG_MODE=1`; upstream SITL defaults remain visible in the parameter log.

The runner requires advancing local-position timestamps, valid horizontal and
vertical estimates, passing preflight checks, no failsafe, active ULog, and
initial landed/disarmed state. Hover sampling starts in AUTO_LOITER, within
the height band, and with vertical speed at most 0.1 m/s. The fixed window
must span at least 10 simulation seconds with strictly advancing timestamps,
no sample gap greater than 0.5 s, maximum height error 0.3 m, and maximum
horizontal distance from the first hover sample 0.5 m. Height is initial local
NED z minus current local z. The fixed deadlines are 120 simulation seconds
from the first observation and 240 wall seconds from trial launch.
Repeated reads of the same uORB timestamp are retained in raw observations
and waited out under the wall deadline; only fresh observations enter the
hover window. Backwards time fails. The hover sample-gap rule is unchanged.

Missing measurements remain unknown. A failed window is not retried with
looser thresholds. A successful trial observes airborne/armed, hover,
landing, and landed/disarmed states before normal shutdown. A failure stops
further vehicle commands and terminates only the owned process group; exit
of private namespace PID 1 also terminates its remaining descendants. It
does not unconditionally issue RTL or landing when prerequisites are absent.

Each invocation preserves `runs/<run_id>/summary.json`, raw console and
observations, namespace processes/endpoints, configuration hashes, and actual
ULogs. Inspect `summary.json` for the result, never just command exit or an
open window. Failed and interrupted attempts remain in the same evidence
directory. Run the explicit command again for each additional trial; count
only consecutive passed records, resetting the streak after any failure or
interruption. Full P1 acceptance needs 20, separately from the initial smoke.

For an explicit 20-trial sequence, the batch runner stops at the first failure
and rejects changes to the measured inputs within the sequence:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
/usr/bin/python3 simulation/px4_gazebo/scripts/run_p1_repeated.py \
  --run-repeat --allow-simulated-flight --qgc-monitor
```

Use the PX4 venv for an offline ULog cross-check and sanitized evidence export:

```bash
$HOME/uav_autonomy/venv-px4/bin/python \
  simulation/px4_gazebo/validation/collect_evidence.py \
  --root "$HOME/uav_autonomy" --output "$HOME/uav_autonomy/state/p1-evidence.json"
```

This checks ULog hashes, recorded data coverage, the same fixed hover limits,
armed/AUTO_LOITER/airborne state during hover, final landed/disarmed state,
and absence of dropouts or failsafe after arming. It starts no simulator.
Batch order is the ordered trial list in its summary; UTC directory names
are identifiers, not a reliable ordering clock when WSL adjusts wall time.

## Verification categories

`tests/test_px4_gazebo_contract.py` covers pure gates/config/measurement logic
and import safety without Linux, ROS, Gazebo, or PX4. Bash syntax/shellcheck
and ordinary project regression are separate from P0 build logs and P1
observed flight evidence. P0/P1 historical evidence remains unchanged.

## P2 ROS 2 control and revised reference policy

P2 adds `ros2_ws/src/gwm_px4_control`, with pure frame, wire-field, ACK,
state-machine, freshness and acceptance modules and a thin ROS adapter.
The original strict-v1 flight failed on a heading-reference reset and remains
failed. The historical P2-R1 twenty-trial sequence stopped on trial 2 with
**1/20** consecutive passes: a late reset notification allowed an old ROS yaw
target to overwrite PX4's cached correction. P2-R2 introduces
`p2-estimator-reference-v3`, with no finite external yaw during initialization
and a bounded handover after stable alignment. Current measured outcomes are
in [P2 validation](../../docs/v3_sim_p2_validation.md) and
[sanitized evidence](../../docs/evidence/v3_sim_p2_summary.json).

R2 diagnostic and new nominal smoke passed. The fresh R2 batch stopped on
trial 6 for a ground-prestream `observation_gap`, leaving **5/20** consecutive
passes. No mode/arm commands were sent in the failed trial; it remained
landed/disarmed. That R2 batch remains failed. R3 subsequently localized a
native SHM publish wait and completed a separate new smoke and **20/20**
consecutive trials with controller-only UDPv4. P2 acceptance is now complete
for this pinned simulation profile; the R2 streak was never resumed.

The [pinned-source contract](../../docs/v3_sim_p2_reference_contract.md)
allows one verified yaw-only initialization event during bounded takeoff,
followed by five seconds of final alignment and reference lock. The first
finite yaw target uses fresh aligned heading; a dedicated handover ramps at
10 degrees/s toward the original measured ground anchor plus accepted reset
correction before the full mission. Initialization uses yaw=NaN and
yawspeed=0.0, with fresh bounded position holds during pending classification.
This changes yaw semantics and does not promise fixed geographic heading
during initialization. Both heading and attitude drift remain bounded by
5 degrees; invalid or ambiguous reset evidence still fails. It preserves
the ground origin, relative-yaw intent, PX4/default magnetic fusion, mission
height and original flight limits. Unsupported or post-lock resets still
abort. Source/configuration/evaluator changes require a new smoke and streak;
historical failures never become successful flights.
The old failed batch is retained. The v3 evaluator checks explicit mode/mask
agreement in actual CDR, initialization drift, matched internal resolved yaw,
handover and the original full motion/LAND contract. Internal ULog output is
sampled around 10 Hz and does not prove every internal controller update.

From the Windows checkout mounted in a WSL shell, build the one-way,
hash-verified package mirror with Linux system Python and sourced Jazzy:

```bash
bash simulation/px4_gazebo/scripts/build_p2.sh --build
```

Build/install/log directories remain under `$HOME/uav_autonomy/p2_ws/<hash>`.
The launcher rejects changed source mirrors and requires a passed read-only
receipt for the exact controller/config/launcher/dependency identity:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 \
bash simulation/px4_gazebo/scripts/run_p2_control.sh --run --observe
```

Run one labelled initialization/handover diagnostic first by adding
`--diagnostic` to the explicit flight entrypoint below. It exercises the same
complete profile but cannot qualify a repeated batch. Independently verify
that diagnostic before running a new nominal smoke without that flag:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
bash simulation/px4_gazebo/scripts/run_p2_control.sh --run --allow-simulated-flight
```

QGC monitoring/heartbeat is always included. All processes share a private
loopback-only network/PID namespace and the existing exclusive P1/P2 lock.
Only ROS publishes the three PX4 input topics in a flight. Console inspection
does not issue flight commands. Default calls and missing gates start no
simulator or ROS node. `--headless` selects headless physics/offscreen QGC;
the recorded P2 attempts used the normal GUI-requested mode.

After a run, read bags offline; never play command topics into live endpoints:

```bash
bash simulation/px4_gazebo/scripts/verify_p2_evidence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_P2_RUN_ID"
```

This verifies artifact hashes, SQLite/CDR readability, complete event-ledger
agreement, actual wire NaNs/rates/ramps, ACK identity/timing, matched ROS/ULog
positions and phase-specific state. It records a failed flight as failed even
when the recording and abort recovery are sound. Existing offline results
are preserved and cannot be overwritten by this entrypoint.

Only after an initial full flight and its independent evaluator both pass:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
/usr/bin/python3 simulation/px4_gazebo/scripts/run_p2_repeated.py \
  --run-repeat --allow-simulated-flight \
  --smoke-run "$HOME/uav_autonomy/runs/ACTUAL_PASSED_P2_SMOKE_ID"
```

The runner freezes inputs, independently verifies each new trial and stops
at the first failure, interruption or input change. P1's previous 20 passes
never count as P2 evidence. P3-P7, model decisions, obstacle avoidance,
multi-UAV orchestration and hardware integration remain outside this slice.

### P2-R3 timing diagnostics

R3 preserves the R2 yaw policy and flight limits. The
[timing contract](../../docs/v3_sim_p2_timing_contract.md) documents the measured
333 ms Fast DDS SHM wait and the controller-only UDPv4 transport repair.
The pinned middleware, BEST_EFFORT depth, synchronous publication and 20 Hz
steady timer are unchanged. Timing settings are independently versioned in
`configs/p2_timing.yaml`; the raw evidence writer remains synchronous, so this
does not claim bounded disk latency or hard real-time operation.

After a build and exact-input read-only connection pass, an explicitly gated
ground diagnostic exercises the real first safe prestream without creating
a VehicleCommand publisher:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
bash simulation/px4_gazebo/scripts/run_p2_control.sh --run --ground-diagnostic
```

The declared R3 budget is at most six pre-repair and three post-repair starts;
four pre-repair starts established attribution. These are not acceptance
flights. `--native-wait-probe` is an optional ground-only diagnostic interposer
using existing gcc; it never applies to nominal or repeated acceptance.
Every attempt and native-probe version is retained. Ground evaluation uses
`validation/timing_evidence.py --ground RUN --output RUN/p2-timing-evaluation.json`
in the existing sourced ROS/PyULog environment. Outputs are exclusive-create.

Schema 2 events distinguish callback acquisition, Mission selection, actual
publish entry/return and persistence. A bounded in-memory trace is persisted
after control stops. The full offline evaluator also reconciles this trace
against bag/event publication identities and checks actual prestream coverage.
An overflow, missing recording, stale action or over-limit consumed sample
gap invalidates acceptance. The repeated runner freezes the timing evaluator,
instrumentation, transport configuration and measured middleware binaries.
No older R2 pass carries into the new R3 streak.

### P3-R2 offline evidence gate

P3 acceptance is incomplete. Outcome C blocks new nominal flights: existing
ULog fields do not identify the MC position input consumed for an internal
yaw output. Read the [yaw provenance contract](../../docs/v3_sim_p3_yaw_provenance_contract.md)
and its proposed instrumentation amendment before any future runtime sequence.
No PX4 patch or rebuild is authorized by the offline repair. The old proxy's
79/135 and 68/135 failures remain immutable diagnostics; new analyses have
unobservable causal evidence and no flight credit.

Offline historical analysis uses exclusive output names. For a sealed R1 run:

```bash
bash simulation/px4_gazebo/scripts/verify_p2_evidence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_RETAINED_R1_RUN" \
  --historical-analysis --output-name UNIQUE-control.json
```

An unknown mandatory result exits nonzero and **must stop qualification**.
Only explicitly labelled historical diagnostic work may subsequently inspect
the dependent sensor metrics using `--historical-analysis`, a matching
`--control-name` and a distinct `--output-name`. That result remains failed
when control acceptance is unknown. The original pre-R1 smoke additionally
uses `--sample-contract p3-sample-evidence-v2`; it cannot acquire a newer
runtime seal or missing callback identity through reanalysis.

The [offline budget measurements](../../docs/v3_sim_p3_offline_budget.md) use
`measure_p3_offline.py` through these same wrappers/interpreters. Production
qualification uses `p3_offline_budget.json`, requires finalized current-input
control success and verified A/B/C/D before launching sensor evaluation, and
increments progress only after both results pass. Partial/stale files, wrong
run IDs, changed arithmetic, unknown evidence and timeout stop the batch.
Owned evaluator descendants are cleaned up even if their leader has exited.
Flight, dispatch, freshness and ACK deadlines are unchanged.

### P3-R1 final-input readiness, ground and flight procedure

The following records the earlier campaign and future sequence prerequisites.
The P3-R2 observability gate above must be resolved before another campaign.

P3 adds the pinned native x500_depth camera, GZ_TO_ROS Image/CameraInfo bridge,
a separate bounded sensor adapter/recorder and independent calibration tools.
See the [sensor contract](../../docs/v3_sim_p3_sensor_contract.md),
[actual validation and retained failures](../../docs/v3_sim_p3_validation.md)
and [sample/time contract](../../docs/v3_sim_p3_sample_time_contract.md).
The [current schema-2 summary](../../docs/evidence/v3_sim_p3_summary.json) keeps the repair
separate from [preserved schema-1 evidence](../../docs/evidence/v3_sim_p3_summary_v1.json).
No camera input enters P2 control decisions, PX4 fusion or AgentOps.

Historical rendering/bridge/ground checks passed; the original depth smoke
landed normally but failed independent strict timestamp-window acceptance.
Its qualification count remains 0/3. P3-R1 implements
`p3-sample-evidence-v2`, explicit enclosing run IDs, lossless source records
and finalized provenance. The initial historical reconstruction failure is
retained; a separate normalized reanalysis passes the fixed motion windows but
still fails the unchanged prior-state yaw consistency gate, with exact PX4
controller-consumed source identity unproven. No tolerance or matching policy
was relaxed, and the old smoke receives no qualification credit.
Fresh read-only run `20260912T132733Z-p3-observe-1957cc82` now passes the
launcher, sealed finalization and independent control/sensor checks, including
actual enclosing run IDs. The next plane2 attempt
`20260912T132932Z-p3-plane2-514444e3` collected data but failed sealing because
an unregistered sleeping process remained. Installed-source review identified
the default QoS-inspection daemon as a supported explanation, with exact PID
identity unrecorded. Ground inspection now uses explicit `--no-daemon`; no
finalization check was weakened. The failed run and immutable diagnosis are
retained. A v3 freeze now includes the daemon-free launcher, constituent ground
artifact revalidation and complete selected-record health/source binding.
Readiness `20260912T134432Z-p3-observe-113909cf` finalized under that freeze
but failed independent evaluation because the offline reader rejected native
zero-publication timestamps permitted by the pinned uint64 contract. Its
failed results and two read-only diagnostics remain unchanged. A minimal
offline correction uses the shared validator while retaining positive
estimator sample timestamps and all flight limits. Fresh v4 readiness
`20260912T135042Z-p3-observe-f4e7bdeb` passed launcher, seal and both independent
evaluations, with all 159 image/PX4 associations matched exactly and correct
run identities. V4 plane2 `20260912T135209Z-p3-plane2-adca8021` also passed.
Matrix `20260912T135352Z-p3-ground-matrix-2f3837c3` then stopped at plane4
because the daemon-free QoS CLI reported an unknown image topic despite
recorded fresh sensor flow. The remaining cases were not run. Installed-source
review identified the direct CLI node's 0.5 s discovery default; ground
inspection now uses `--no-daemon --spin-time 3` with the existing outer 10 s
timeout. The exact failed CLI cache state is unrecorded; the diagnosis and
original failed summaries are retained.

The current v5 freeze is
`state/p3-r1-implementation-20260912T130622Z/frozen-runtime-inputs-v5.json`,
SHA-256 `202e17ebbcf9a3532445ff0a541bc2fc1f06a8a3b9c95c913cafa786b13146b4`.
It retains the same packages, evaluator arithmetic and physical limits as v4.
Final ordinary verification passed 446 PX4/Gazebo focused tests and 1,246 full
tests with 12 skips, including the unchanged AgentOps/C2 coverage. Python,
Bash, ShellCheck and diff checks passed. Fresh v5 readiness
`20260912T140041Z-p3-observe-34709c89` passed its sealed independent checks,
including all 159 exact image/PX4 associations. Plane2
`20260912T140217Z-p3-plane2-2de3a546` then passed. Matrix
`20260912T140437Z-p3-ground-matrix-1e3494df` passed all seven cases, including
the expected bridge interruption and plane4's separate scene-change check.
The single new smoke `20260912T141235Z-p3-flight-778e3f60` completed normal
LAND/disarm, recording finalization and cleanup. Independent P2 recording,
all eight controller and sixteen ROS/ULog sample windows, execution timing,
and all sensor checks passed. Independent flight acceptance failed prior-state
yaw consistency: 68/135 comparisons failed, including 63 heading mismatches
and eight age violations (three overlap). Maximum error was
8.08238983154297e-5 rad versus 1e-5 rad; maximum age was 0.016 s versus
0.008001 s. P3 composite acceptance failed only `control_acceptance`.
All 984 image/PX4 associations matched exactly without ambiguity, reuse or
missing sources. The sample/time contract and final observation identity are
verified; P3 flight acceptance remains incomplete.

The smoke failure stopped progression: no further flight was attempted and
qualification remains **not_run, 0/3**. The full independent P2 evaluator was
observed CPU-active beyond 180 s before completion, so the qualification
wrapper's existing 90 s control-evaluation timeout is also not runtime-qualified.
No frozen timeout, arithmetic or flight limit was changed after the smoke.
The commands below document the required sequence; qualification requires a
fully passed new smoke, which this campaign did not provide.

From the Windows checkout in a clean WSL shell, build the content-addressed
Linux mirrors using already installed Jazzy dependencies. Build the controller
first: the sensor build verifies and sources that exact installed overlay.

```bash
cd /mnt/d/GWM-UAV-Navigation-Sparse-Rewards
bash simulation/px4_gazebo/scripts/build_p2.sh --build
bash simulation/px4_gazebo/scripts/build_p3.sh --build
```

The final builds recorded for P3-R1 are controller `0d994904f2e43310219e4505c246cc8ecd2f95f2467ed3463a5308c124840456`
and sensor `8f0514da5d4e93a3b07efe9983567b8d487f9f1a93cc108676c896794f4778a3`.
The P3 mirror stays under `$HOME/uav_autonomy/p3_ws/<hash>`. A fresh run verifies
the complete source mirror and installed inventory, evaluator/configuration
hashes and measured pinned runtime assets. Check disk reserve for the declared
ground/flight set before starting; preserve all historical evidence.

Read-only depth/PX4 readiness comes **before** the final-input ground matrix.
It uses Ogre2 server rendering through WSLg, QGC offscreen monitoring and the
existing exclusive private namespace/lock. It publishes no flight inputs:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 \
bash simulation/px4_gazebo/scripts/run_p3_coexistence.sh --run --observe

bash simulation/px4_gazebo/scripts/verify_p2_evidence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_FINAL_READINESS_ID"
bash simulation/px4_gazebo/scripts/verify_p3_coexistence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_FINAL_READINESS_ID"
```

Wait for the launcher process to exit and `runtime-finalized.json` to exist
before the first evaluator; wait for the P2 evaluator to finish before starting
the P3 evaluator. A controller result or a partially written manifest is not
completion. The seal requires stopped owned processes, a closed/drained/fsynced
sensor writer, complete controller records and verified artifact hashes.
Actual startup, health, observations, indices and result records must all carry
the enclosing run ID. Existing evaluator reports are exclusive-create.

After both readiness evaluations pass, start plane2 using that exact readiness
predecessor. This ground-only run creates no PX4 process or flight publisher:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 \
bash simulation/px4_gazebo/scripts/run_p3_sensing.sh --run --case plane2 \
  --readiness-run "$HOME/uav_autonomy/runs/ACTUAL_FINAL_READINESS_ID"

/usr/bin/python3 simulation/px4_gazebo/validation/collect_p3_evidence.py \
  "$HOME/uav_autonomy/runs/ACTUAL_PLANE2_RUN_ID"
```

Again wait for launcher exit and the sealed runtime before evaluation. With
an independently passed plane2 under the final frozen inputs, the matrix
wrapper inherits its readiness predecessor and runs six remaining cases once:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 \
/usr/bin/python3 simulation/px4_gazebo/scripts/run_p3_ground_matrix.py \
  --run-matrix --plane2-run "$HOME/uav_autonomy/runs/ACTUAL_PASSED_PLANE2_ID"
```

It covers plane4, plane6, oblique, asymmetry, out-of-range and a ground-only
bridge interruption. Truth, fixture coordinates and source-header probes
stay evaluator-only. The interruption's expected failure is separate from
nominal depth measurements; plane4 also retains the separately labelled
post-window scene-change proof. Native depth remains 640x480/30 Hz; the selected
sensor-only middleware XML allocates 64 MiB SHM. P2's controller-only UDPv4
transport and all numerical limits stay unchanged. Build/model/configuration/
evaluator identities and calibration must match readiness. Historical ground
measurements do not substitute for this final-input matrix.

After the complete matrix passes, run exactly one new nominal depth smoke:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
bash simulation/px4_gazebo/scripts/run_p3_coexistence.sh --run --allow-simulated-flight \
  --ground-matrix "$HOME/uav_autonomy/runs/ACTUAL_PASSED_GROUND_MATRIX_ID"

bash simulation/px4_gazebo/scripts/verify_p2_evidence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_NEW_DEPTH_SMOKE_ID"
bash simulation/px4_gazebo/scripts/verify_p3_coexistence.sh \
  "$HOME/uav_autonomy/runs/ACTUAL_NEW_DEPTH_SMOKE_ID"
```

Run the evaluators sequentially **after launcher completion and finalization**.
They check the revised sample/time/correlation contract alongside unchanged
physical, timing, ACK, nominal LAND and landed/disarmed requirements. The
controller retains sole external command ownership and its original bounded
mission. A failed or aborted smoke does not authorize qualification. An offline
reanalysis of the historical smoke cannot supply new-runtime credit.

Only after that new smoke and both independent evaluations pass, run the fixed
three-consecutive-flight qualification:

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
GWM_ALLOW_PX4_LAUNCH=1 GWM_ALLOW_SITL_COMMANDS=1 \
/usr/bin/python3 simulation/px4_gazebo/scripts/run_p3_qualification.py \
  --run-qualification --allow-simulated-flight \
  --smoke-run "$HOME/uav_autonomy/runs/ACTUAL_PASSED_DEPTH_SMOKE_ID"
```

It freezes inputs, runs exactly three consecutive trials, evaluates both
control and sensing after finalized recordings, checks unique run IDs and
predecessor hashes, and stops on the first failed or interrupted attempt.
Do not restart selectively to obtain three favorable outcomes. Historical
x500 P2 20/20 results cannot serve as depth-model acceptance. Raw depth binary,
indices, calibration/events, control traces, bags and ULogs remain in the Linux
run directories. Do not replay command bags into a live graph. P4-P7 and
AgentOps v3-2 onward remain incomplete. Clean rebuild remains unproven; P4 is
recommended only after the complete declared P3 acceptance passes.
