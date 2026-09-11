# v3-SIM P0/P1 validation

## Status

P0, the initial measured P1 flight smoke, and the separate 20-consecutive-trial
acceptance have passed. No ROS 2 autonomous control is implemented.
The [scope amendment](v3_simulation_scope_amendment.md) defines the narrowly
authorized operator workflow; the [README](../simulation/px4_gazebo/README.md)
contains resumable commands and fixed pass/fail rules.

| Gate | Measured status |
| --- | --- |
| P0 environment | passed |
| P1 simulator boot | passed |
| P1 initial flight smoke | passed |
| P1 repeated-flight acceptance | passed; 20 / 20 consecutive successes, each independently cross-checked from ULog |
| ROS 2 autonomous control | not_implemented |
| Obstacle avoidance | not_implemented |

## Verified starting state and environment

- Authoritative checkout: `D:\GWM-UAV-Navigation-Sparse-Rewards`.
- Branch: `v3/gwm-uav-c2-agentops-planning`, initial HEAD `efae1ee`.
- Origin: `https://github.com/Neura-Shadow/GWM-UAV-Navigation-Sparse-Rewards.git`.
- Fetch and fast-forward-only synchronization: already up to date.
- Initial tracked/staged tree: clean; `.codegraph/` was the sole untracked path.
- Preserved tag `v1.0.0-research-framework-complete` remains at
  `cb304c7a64b2edd57673a3323c0ed2e7d2923453`.
- WSL2 `Ubuntu-24.04`, Ubuntu 24.04.3 LTS, user `joker0625`, Linux HOME
  `/home/joker0625`; build/evidence root `/home/joker0625/uav_autonomy`.
- Initial storage: about 899 GiB available; WSL memory about 15 GiB total,
  6.6 GiB available. WSLg `DISPLAY=:0`, `WAYLAND_DISPLAY=wayland-0` existed;
  these fields alone do not prove rendered GUI output.
- Initial process/endpoint inspection found no PX4/Gazebo/DDS/control process.
- Linux system Python: `/usr/bin/python3`, 3.12.3. PX4 build Python:
  `/home/joker0625/uav_autonomy/venv-px4/bin/python`. ROS/colcon uses system
  Python in sourced Jazzy shells; repository regression uses unchanged
  `C:\Users\zongx\anaconda3\python.exe`.

## Pin resolution and setup evidence

Official Git refs were resolved before installation:

| Component | Requested ref | Exact commit |
| --- | --- | --- |
| PX4 | v1.17.0 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | release/1.17 | `86d8239e962f6939e05c3737784f60c02fa884db` |
| Micro XRCE-DDS Agent | v2.4.3 | `73622810d984349b80bbac0ef55fc0b694d62222` |
| PX4 Gazebo models submodule | PX4-recorded gitlink | `b6127f4ec20de867e215fb5f78ae88b80f371909` |
| QGroundControl monitor | v5.1.4 | `4568b585648e8c6065cca6c650b52731c1d3baa3` |

The exact dependency manifest is
[`versions.lock.yaml`](../simulation/px4_gazebo/configs/versions.lock.yaml).
ROS Jazzy installed as `ros-jazzy-ros-base` version
`0.11.0-1noble.20260903.093452`; `ros-jazzy-ros-gz` is
`1.0.24-1noble.20260905.090616`. Sourced `gz sim --versions` reports `8.15.0`
(Harmonic). No system-wide Python default, startup file, GPU driver, WSL
distribution, or unrelated workload was changed.

All following paths are beneath `/home/joker0625/uav_autonomy/runs/`:

- `20260911T152511Z-setup-sLa1Zl/console.log`: first setup installed ROS/Gazebo,
  then failed with exit 127 because the running Bash file was edited while
  awaiting apt; the resumed read treated `libspdlog-dev` as a command. This
  implementation error and attempt are retained, not counted as setup success.
- `20260911T153435Z-setup-GPt5mq/console.log`: rerun reused installed packages
  and clean pinned sources, installed the dedicated PX4 venv, and completed
  with exit 0. Its `installer-adaptation.diff`, `installer-hashes.txt`, and
  `px4-python-freeze.txt` record the precise installer adaptation.
- `20260911T153619Z-build-PUYPhY/console.log`: initial build attempt.
  px4_msgs completed with clock-skew warnings; system spdlog/fmt rejected
  Agent v2.4.3 endpoint formatting. Build exit 2 is preserved.
- `20260911T153846Z-build-c7NUaI/console.log`: px4_msgs recheck completed
  without clock-skew warnings. DDS Agent built and installed using upstream
  spdlog 1.9.2, but the script initially treated its help exit 1 as a failure.
  In v2.4.3, `AgentInstance::create(HELP)` returns false and `main` returns 1
  without running a transport. The script now checks this exact behavior and
  records `dds_help_exit=1` separately from successful compilation/install.
- `20260911T154433Z-build-5SLYcv/console.log`: resumed verified build.
  Agent compilation/install and help behavior passed. PX4 configuration
  selected Windows Anaconda's Protobuf package through WSL's inherited PATH,
  causing a header-version error. The Linux compiler and Python themselves
  were correct. This failed build is preserved.
- `20260911T154644Z-build-rxwym3/console.log`: clean Linux search paths and
  fresh `px4_sitl_default_linux` build directory, using the supported Makefile
  suffix. Windows Anaconda and the failed generated tree remain untouched.
  PX4 SITL and Gazebo plugins built successfully, as did px4_msgs and DDS
  Agent. Overall exit 0; `steps.txt` and `versions.resolved.json` retain the
  checks. `state/p0-built.json` is the successful build receipt.

The built PX4 SHA-256 is
`5383767fd3068a8a306660430f10c8852eb4d581a8a7af5f3fc14cea7b9d106d`.
The version manifest records recursive submodules and the additional fetched
OpticalFlow external project. Upstream requests `master` for that project,
and apt/pip sources are not frozen mirrors. This records the actual build's
inputs; `clean_rebuild_proven` remains false.

The direct HTTPS probe of `packages.ros.org` returned a certificate hostname
mismatch. No TLS verification was disabled. Apt used the official signed HTTP
ROS repository with the Open Robotics key retrieved from a pinned rosdistro
commit; signed package verification succeeded.

## Measured P1 and retained failures

All attempts use one `x500_71` in the default empty Gazebo world, associated
with PX4 instance 71 / system ID 72 in a private loopback-only network/PID
namespace. The owned PX4 console is the sole control source. GUI mode was
requested through WSLg; state/log validation does not certify visual rendering.
Normal safety checks, failsafe parameters and arming remained enabled.

The unchanged x500 policy requires a GCS. The optional official
[QGroundControl v5.1.4 artifact](https://github.com/mavlink/qgroundcontrol/releases/tag/v5.1.4)
was downloaded, hash-verified and extracted in
`20260911T155330Z-qgc-setup-nrV7Zi`, with no launch during installation.
Its SHA-256 is
`1c4ac089abfaac6c6fcd75c7b477ea18da1bc3592cddca5ab1a19c1a13410e65`.
During flights it provides normal local telemetry/heartbeat monitoring only;
there was no manual QGC control interaction. Fresh per-run configuration
disables serial/USB autoconnect, forwarding, joystick and follow-target.

These earlier attempts remain in the Linux evidence tree and are not counted
as complete flights:

| Run ID | Observed outcome |
| --- | --- |
| `20260911T154745Z-p1-11f1c326` | Boot validation failed: the console parser mistook character-redraw prompts for a completed response; a real ULog was retained. |
| `20260911T155120Z-p1-8661c19b` | Boot preflight blocked on missing GCS after 150 state samples; no flight command. |
| `20260911T155532Z-p1-dea27417` | QGC rejected mapped UID 0; no flight command. Namespace mapping now preserves the ordinary user. |
| `20260911T155655Z-p1-bca379c1` | Actual takeoff and 10.040 s hover passed, but a repeated timestamp immediately after the landing command failed the verifier; final landed/disarmed was not established. |
| `20260911T160158Z-p1-ec92e9ab` | QGC startup blocked by its old shared temporary-directory instance lock; no flight command. Each subsequent trial gets a separate TMPDIR. |

No old lock, failed build, or failed run was deleted. Duplicate timestamp reads
are now retained and waited out under the same wall deadline; backwards time
still fails. No height, displacement, sample-gap or duration limit was relaxed.

Initial complete smoke: `20260911T160305Z-p1-f0818342`, process exit 0.

| Measurement | Result |
| --- | --- |
| Hover simulation duration | 10.024 s |
| Height relative to initial local NED ground z | 1.82027 to 2.00859 m |
| Maximum height error / limit | 0.17973 / 0.3 m |
| Maximum horizontal displacement / limit | 0.05938 / 0.5 m |
| Console hover samples / maximum gap | 98 / 0.112 s |
| Independent ULog hover samples / maximum gap | 1,254 / 0.012 s |
| Final landed / disarmed | observed true / arming state 1 |
| ULog dropouts / failsafe after arming | 0 / false |
| Wall duration | 44.921 s |

The 8,727,638-byte ULog is
`20260911T160305Z-p1-f0818342/rootfs/log/2026-09-11/16_03_17.ulg`, SHA-256
`44e713781e67b682e673de98b39c4d99da004ea9debc9c61ed9703d7a85ff2aa`.
Its raw `px4-console.log`, `observations.jsonl`, process/endpoint/parameter
records, configuration copies and derived `ulog-verification.json` exist.

The first sequence, `20260911T160450Z-repeat-251648a4`, stopped after seven
consecutive passes when trial eight (`20260911T161022Z-p1-d3907d3a`) timed out
parsing a console response after the landing command. Its hover passed;
the console later printed landing/disarming, but the required structured
final-state observation did not complete, so the trial remains failed.
PX4 had inserted an asynchronous log after the full command echo and before
its newline. The parser now accepts that inserted text while still requiring
a newline and a subsequent prompt; a regression covers this exact case.
No acceptance threshold changed, and the failed batch was not resumed as
though trial eight had passed. A fresh sequence starts from zero at
`20260911T161204Z-repeat-8970f177`.

Each sequence's ordered `trials` list is authoritative; WSL wall-clock
adjustment made one UTC directory name non-monotonic. Timing gates use PX4
simulation timestamps and Python's monotonic clock, not those directory names.

Flights ran from the implementation working tree based on `efae1ee`.
Each raw summary records exact operator-script hashes as well as configuration,
lock and binary hashes. The batch rejects a change to these inputs mid-sequence.
Raw ULogs and build/runtime artifacts remain outside Git in the Linux workspace.

The fresh sequence completed with exit 0: **20 / 20 consecutive passes**, from
`20260911T161205Z-p1-3c49882b` through `20260911T162712Z-p1-11a6f78a` in the
ordered batch record. All 20 independently passed the ULog cross-check, with
no ULog dropouts or failsafe after arming and observed final landed/disarmed.

| Final sequence aggregate | Measured result |
| --- | --- |
| Hover duration range | 10.016 to 10.084 simulation seconds |
| Worst height error / limit | 0.2910501 / 0.3 m (ULog cross-check) |
| Worst horizontal displacement / limit | 0.095873 / 0.5 m |
| Largest console sample gap / limit | 0.112 / 0.5 simulation seconds |
| Post-batch relevant simulator/control processes | 0 |

The complete retained history contains 34 attempts: 28 completed passed
flights, two failed flight validations, and four blocked flight attempts.
These totals include the initial smoke and earlier failed sequence; only
the final unchanged 20-trial sequence establishes repeated acceptance.

The committed [sanitized evidence summary](evidence/v3_sim_p0_p1_summary.json)
lists every run ID, result, measured hover, ULog reference/hash, both batch
orders and the final batch's common input hashes. The fuller local audit is
`/home/joker0625/uav_autonomy/state/p1-evidence.json`; the batch also retains
`cleanup-verification.txt`. No raw ULogs, simulator logs, downloaded binaries,
build trees or Python environments are committed.

## Regression and acceptance boundaries

The final full runtime-free regression passed `835 passed, 12 skipped` in
117.90 s (`python -m pytest -q -rs` using the Windows interpreter above).
All skips are existing optional-runtime gates: AirSim, demo, Isaac, MAVSDK,
phase-6 runtime and ROS 2 sensor sync. None counts as simulation evidence.
The new focused contract suite passed 35 tests, including missing/NaN
measurements, runtime gates, console redraws, interleaved asynchronous logs
and duplicate timestamp handling.
Existing AgentOps tests passed 144; existing C2 regressions passed 398.
Bash syntax, ShellCheck, and Python compile checks passed for the added files.
Default invocation starts nothing; explicit run without gates exits 2.

Regression results are separate from the P0 build and P1 flight evidence above.
They do not establish ROS 2 autonomous control, obstacle avoidance or flight
certification. P0/P1 have no remaining acceptance blocker in this environment.
P2 through P7 and v3-2 remain open. With P1 acceptance satisfied, the next
simulation implementation slice is P2 ROS 2 control, ACK verification and
coordinate tests; none of that slice is implemented here.
