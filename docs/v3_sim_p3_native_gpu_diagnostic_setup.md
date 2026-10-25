# Native Ubuntu 24.04 + NVIDIA host — two-start stall diagnostic (gate N)

Direction 1 of [`v3_sim_p3_stall_attribution.md`](v3_sim_p3_stall_attribution.md): keep the frozen
workload (depth 640×480 and **RGB 1920×1080 @ 30 Hz unchanged**) and test whether a native GPU
produces the RGB frames at real time. Gate `N` runs exactly the WSL diagnostic pair:

1. `diag-baseline` — full frozen workload;
2. `diag-no_rgb_observer` — the only change is that the source probe does not subscribe to RGB.

Each start uses the unchanged readiness stack (PX4 + Gazebo + sensor bridge, adapter, probe,
controller in observe mode, unattended QGC). The vehicle stays on the ground and is never
armed; the window is a fixed 120 s of wall time. Starts are labelled `diagnostic` with
`acceptance_credit: false`, can never be a Gate B/C predecessor, and are not retried. The set
stops at the first runtime failure, and every start is retained. Gate N refuses to run on WSL;
gates A/B/C/D/V refuse to run on the native host, because contract v1.2 is the WSL D3D12
profile.

Budget: **2 starts**. Running gate N needs the operator's explicit authorization on that host
(step 8).

## Decision rule (fixed before running)

Same metrics as WSL gate D (`p3_stall_diagnostic.py`): the wall time of every 1.000 s sim
interval (the `vehicle_land_detected` 1 Hz family) over the 120 s window.

* **Native GPU sufficient (environment-bound on WSL):** in `diag-baseline`, **zero** 1 Hz
  intervals over 1.8 s **and** a maximum below 2.0 s. Then migrate P3 to this host: its own
  build receipts, a pinned native GPU profile under a new contract version (operator
  approval), a new frozen candidate and a fresh Gate A.
* **Otherwise (workload-bound):** the RGB workload exceeds real time on this host too, and
  direction 2 (an RGB workload contract revision) is the only remaining path.
* `diag-no_rgb_observer` is the control. It must show the 1 Hz family at about 1.000 s; if it
  does not, the host itself is not real-time-capable and the result is inconclusive.

## 0. Host requirements

| Item | Requirement |
|---|---|
| OS | Native **Ubuntu 24.04** (not WSL, no VM), `x86_64`. An external-SSD install on this laptop is fine. |
| GPU driver | NVIDIA proprietary driver from Ubuntu (`ubuntu-drivers`), for example the 570 series; Secure Boot either disabled or the MOK enrolled at the first reboot |
| GPU mode | Discrete GPU available to EGL (PRIME `on-demand` or `nvidia`); AC power connected; firmware/OS power profile "performance" |
| Disk | ≥ 120 GB free on ext4 under `$HOME` (PX4, ROS, Qt 6.11.1, QGC build, runs; the runner also checks a 20 GiB reserve) |
| Network | Needed for setup only (apt, Git, Qt). Runs use loopback only. |
| Account | A normal user with `sudo`. Use a clean shell: no conda, no Python venv active. |

## 1. GPU driver and EGL check

```bash
sudo ubuntu-drivers install
```

Reboot, then confirm:

```bash
nvidia-smi
```

```bash
cat /proc/driver/nvidia/version
```

```bash
ls /usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

## 2. Repository

Clone this repository on ext4 (not a mounted Windows drive) and check out the commit that
contains this document:

```bash
git clone <this repository URL> ~/GWM-UAV-Navigation-Sparse-Rewards
```

```bash
git -C ~/GWM-UAV-Navigation-Sparse-Rewards checkout v3/gwm-uav-c2-agentops-planning
```

All commands below run from the repository root. `GWM_SIM_ROOT` defaults to `~/uav_autonomy`.

## 3. Pinned simulation stack (scripted)

These are the same scripts that built the WSL stack. Pins: PX4 v1.17.0 `d6f12ad1`, px4_msgs
release/1.17 `86d8239e`, DDS Agent v2.4.3 `73622810`, ROS 2 Jazzy plus `ros-jazzy-ros-gz`
(Gazebo Harmonic vendor packages) from the ROS apt source pinned by key commit `984f1f74`.

```bash
bash simulation/px4_gazebo/scripts/setup_environment.sh --apply
```

```bash
bash simulation/px4_gazebo/scripts/check_environment.sh
```

```bash
bash simulation/px4_gazebo/scripts/build_baseline.sh --build
```

```bash
bash simulation/px4_gazebo/scripts/build_p2.sh --build
```

```bash
/usr/bin/python3 simulation/px4_gazebo/scripts/build_mc_trace.py --build
```

```bash
GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 /usr/bin/python3 simulation/px4_gazebo/scripts/benchmark_mc_trace.py --measure
```

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && /usr/bin/python3 simulation/px4_gazebo/scripts/build_p3_agent.py --build'
```

```bash
bash simulation/px4_gazebo/scripts/build_p3.sh --build
```

Expected receipts in `~/uav_autonomy/state/`: `p0-built.json`, `p2-built.json`,
`px4-mc-trace-built.json`, `p3-dds-agent-built.json` (its stop smoke must show SIGINT and
SIGTERM exit 0), `p3-built.json`, and a passed `runs/<ts>-mc-trace-benchmark-*` run.

## 4. QGroundControl (stock monitor + unattended build)

Stock v5.1.4 AppImage, used as the bundle base:

```bash
bash simulation/px4_gazebo/scripts/setup_qgc_monitor.sh --apply
```

Unattended build, the same recipe as WSL `qgc_closure_v1`. The pinned source's
`.github/build-config.json` fixes Qt **6.11.1** and the module list.

```bash
export Q=~/uav_autonomy/qgc_closure_v1 && mkdir -p "$Q/receipts" "$Q/qt-tests" "$Q/bundles"
```

```bash
git clone https://github.com/mavlink/qgroundcontrol.git "$Q/source" && git -C "$Q/source" checkout 4568b585648e8c6065cca6c650b52731c1d3baa3 && git -C "$Q/source" submodule update --init --recursive
```

```bash
cd "$Q/source" && sudo python3 -m tools.setup.install_dependencies; cd -
```

```bash
python3 -m venv "$Q/tools" && "$Q/tools/bin/pip" install aqtinstall==3.3.0 cmake==4.2.0 ninja
```

```bash
"$Q/tools/bin/aqt" install-qt linux desktop 6.11.1 linux_gcc_64 -O "$Q/Qt" -m qtgraphs qtlocation qtpositioning qtspeech qtmultimedia qtserialport qtimageformats qtshadertools qtconnectivity qtquick3d qtsensors qtscxml qtwebsockets qthttpserver
```

Apply the reviewed overlay to the clean pinned source; it binds `p3_namespace_owner.py`:

```bash
/usr/bin/python3 simulation/px4_gazebo/scripts/build_qgc_unattended.py --source "$Q/source" --prepare --output "$Q/receipts/overlay.json"
```

```bash
PATH="$Q/tools/bin:$PATH" cmake -S "$Q/source" -B "$Q/build" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$Q/Qt/6.11.1/gcc_64" -DGWM_QGC_SITL_SHUTDOWN_BUILD=ON -DQGC_BUILD_TESTING=OFF
```

```bash
PATH="$Q/tools/bin:$PATH" cmake --build "$Q/build" --target QGroundControl --parallel
```

```bash
/usr/bin/python3 simulation/px4_gazebo/scripts/test_qgc_unattended.py --source "$Q/source" --qt "$Q/Qt/6.11.1/gcc_64" --output "$Q/qt-tests/native-1"
```

```bash
ELF="$Q/build/Release/QGroundControl"; /usr/bin/python3 simulation/px4_gazebo/scripts/build_qgc_unattended.py --source "$Q/source" --elf "$ELF" --stock ~/uav_autonomy/qgc-v5.1.4 --destination "$Q/bundles/$(sha256sum "$ELF" | cut -c1-64)" --overlay-receipt "$Q/receipts/overlay.json" --qt-tests "$Q/qt-tests/native-1/result.json" --output ~/uav_autonomy/state/qgc-unattended-built.json
```

The Qt test receipt must show `passed` (8 steps). The QGC application itself is never
launched by these steps.

## 5. Headless EGL preflight (no simulator)

```bash
__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json /usr/bin/python3 simulation/px4_gazebo/validation/p3_egl_probe.py
```

Both `surfaceless` and `device0` must report an **NVIDIA** renderer (for example
`NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE2`). `llvmpipe` means EGL is not using the GPU; fix
the driver or PRIME setup before continuing.

## 6. Freeze the diagnostic candidate

Diagnostics are never acceptance-evaluated, so the default `strict_zero` policy is used and no
Policy A approval is involved. Use the passed benchmark run from step 3:

```bash
/usr/bin/python3 simulation/px4_gazebo/scripts/p3_manifest.py freeze --output ~/uav_autonomy/state/p3-candidate-native-diagnostic.json --mc-benchmark-run ~/uav_autonomy/runs/<passed mc-trace-benchmark run>
```

The printed value is the canonical manifest SHA. The manifest records (does not pin) the
native NVIDIA EGL/GLX libraries and the driver version.

## 7. Authorization (operator, on that host)

```bash
/usr/bin/python3 simulation/px4_gazebo/scripts/p3_manifest.py authorize ~/uav_autonomy/state/p3-candidate-native-diagnostic.json --gate N --operator <NAME> --confirm "authorize Gate N <manifest-sha>" --output ~/uav_autonomy/state/p3-auth-N.json
```

## 8. Run the two starts (once)

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 GWM_ALLOW_PX4_LAUNCH=1 GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 /usr/bin/python3 simulation/px4_gazebo/scripts/run_p3_gates.py --gate N --manifest ~/uav_autonomy/state/p3-candidate-native-diagnostic.json --authorization ~/uav_autonomy/state/p3-auth-N.json
```

```bash
/usr/bin/python3 simulation/px4_gazebo/validation/p3_stall_diagnostic.py ~/uav_autonomy/state/p3-gate-N-<manifest-sha>.result.json --output docs/evidence/v3_sim_p3_native_stall_diagnostic.json
```

## 9. What to bring back

* `docs/evidence/v3_sim_p3_native_stall_diagnostic.json`
* `~/uav_autonomy/state/p3-gate-N-<sha>.result.json` and `p3-candidate-native-diagnostic.json`
* The output of `nvidia-smi` and `cat /proc/driver/nvidia/version`
* An archive of the two run folders `~/uav_autonomy/runs/*-p3-diag-*` named in the result file
