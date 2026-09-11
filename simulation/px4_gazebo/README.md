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

## P2 ROS 2 control procedure and current blocker

P2 adds `ros2_ws/src/gwm_px4_control`, with pure frame, wire-field, ACK,
state-machine, freshness and acceptance modules and a thin ROS adapter.
The actual read-only connection passed. The first ROS flight entered Offboard,
armed normally and climbed, then correctly aborted on an estimator reference
reset. Full P2 flight acceptance is **failed**, and repeated acceptance is
**not_run (0/20)**. See [P2 validation](../../docs/v3_sim_p2_validation.md) and
[sanitized evidence](../../docs/evidence/v3_sim_p2_summary.json).

The pinned default magnetometer configuration deliberately resets heading
above approximately 1.5 m HAGL. This conflicts with the requested 2 m mission
and strict reset-invalidates-trial rule. Do not repeatedly run the unchanged
flight expecting a streak. Resolving the estimator/reference contract is the
next implementation decision; no reset exemption, magnetometer-fusion change,
height reduction or threshold relaxation was applied to produce a pass.

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

The explicit flight entrypoint below documents the bounded workflow. Its
current configuration is known to abort at the heading reset described above:

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
