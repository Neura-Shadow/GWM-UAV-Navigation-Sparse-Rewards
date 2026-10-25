# v3-SIM P3 — depth sensing and flight-control coexistence (contract and operation)

Status: **P3 COMPLETE in SITL under `p3-acceptance-v1.6` — candidate `1e205fec`, Gates A, B and C
accepted, qualification 3/3 (depth-only flights; no hardware).** See the
[acceptance report](v3_sim_p3_acceptance_report.md), sections AI–AK.

This slice starts from the accepted P2-R3 checkpoint `6204fedb9ac8a2b554ff825c93159a509a1b91ad`
(nominal smoke `20260912T034917Z-p2-flight-c74f35ba`, fresh batch
`20260912T035329Z-p2-repeat-84bb288d`, 20/20). The 27 earlier, unaccepted P3 commits
(`76f014d..39ba887`) and the stash of tracked P3 edits are preserved on
`archive/p3-history-39ba887` and `archive/p3-stash-1db3574`. They are history: no historical
run, report or pass is counted here. Thresholds, fixture geometry and proven design decisions
reused from them are named explicitly below.

## 1. P2 is preserved byte-for-byte

* `ros2_ws/src/gwm_px4_control` is unchanged. Its package hash recomputes to the accepted
  R3 identity `e1857b3fc0a5678c0789b39860ba36cae5193b8daa83a20c246a02f49bb431af`
  (pinned by `test_p2_controller_package_is_byte_identical_to_accepted_r3`).
* P3 flights run the **unchanged** `configs/p2_control.yaml` and `p2_timing.yaml`: same
  mission (2 m, 10 s hover, ±1 m E, ±1 m N, +30° ENU yaw, 5 s dwells, 5 s final hover, LAND),
  20 Hz, 0.3 m/s and 10°/s ramps, 0.2 m / 0.3 m / 5° tolerances, 0.2 s sample gap,
  0.25 s / 1.5 s / 2 s / 1 s freshness, controller-only UDPv4, R2 yaw policy and 75 s startup.
* `p2_runner.py`, P2 evaluators and P2 evidence are untouched, with one operator-approved
  exception: under [v1.6](v3_sim_p3_p2_evaluator_duplicate_timestamp_v1_6_proposal.md),
  `collect_p2_evidence.verify()` pairs ROS/ULog `vehicle_local_position` by
  (`timestamp`, `timestamp_sample`) instead of `timestamp` alone (1e-6 m and 100 pairs unchanged;
  all 21 accepted P2-R3 reports reproduce identically). P3 has its own runner and re-uses that
  `verify()` on every P3 readiness and flight directory.
* The only physical change is the vehicle model, `gz_x500_depth` (+0.061 kg OakD-Lite), which
  is the point of P3. The controller's `model` field is P2 identity metadata; the P3 runner
  independently verifies that PX4 booted `x500_depth_71`.

## 2. Native depth sensing and data path

```
Gazebo OakD-Lite StereoOV7251 (640x480 R_FLOAT32, 30 Hz, hfov 1.274, 0.2-19.1 m)
  └─ gz transport /depth_camera, /camera_info
       ├─ ros_gz_bridge parameter_bridge (GZ_TO_ROS, SENSOR_DATA, queue 8, SHM-64MiB profile)
       │    └─ /gwm/sensors/front_depth/{image_raw,camera_info}
       │         └─ gwm_sensor_adapter (sensing only)
       │              ├─ raw recorder: every receipt, zlib-1 lossless, per-frame SHA-256
       │              ├─ 10 Hz newest-frame processor -> /gwm/sensors/front_depth/observation
       │              └─ health state machine -> /gwm/sensors/front_depth/health
       └─ evaluator-only source probe (gz transport): depth/RGB headers, CameraInfo, poses, clock
```

The adapter (`gwm_sensor_adapter`) is pure-module plus a thin ROS node:

| Module | Contract |
|---|---|
| `contracts.py` | Frozen sensor geometry/topics; deny-by-default publication guard. Any `/fmu/in/` topic or flight-control type (`VehicleCommand`, `TrajectorySetpoint`, `OffboardControlMode`, setpoint/actuator/obstacle types) raises before a publisher exists. |
| `depth.py` | Validates dimensions, `32FC1`, endianness, step and payload length; runtime additionally requires the exact native unpadded little-endian layout. Classifies NaN → invalid, −Inf/<0.2 m → too close, +Inf/≥19.1 m → no return. Nothing is filled; no valid pixels → `null` statistics, never 0. |
| `camera_info.py` | Plumb-bob zero distortion, K/P/R consistency, fx within the bound implied by the SDF FOV's declared precision (±0.0005 rad → ±0.23 px). First valid calibration locks the run; any change invalidates it. |
| `geometry.py` | Explicit optical (X right, Y down, Z forward) → body FLU → FRD → local NED, Hamilton wxyz FRD→NED; depth is optical Z, not range; SDF include-chain resolver. |
| `association.py` | 300-sample position/attitude histories; nearest PX4 `timestamp_sample` within 0.05 s, no interpolation; reports missing, older, pending, ambiguous ties, reset-epoch brackets, duplicates and reuse. |
| `pipeline.py` | Separate accounting for receipt, decode, processing skip and recorder. Simulation-time 10 Hz phase accumulator; never republishes a frame; observations only while fresh. Health: unavailable / calibration_unavailable / stale (1.0 wall s receipt stall or 0.25 sim s source age) / fresh. |
| `recorder.py` | Single-owner bounded queue (`recorder_capacity` 32 since [v1.5](v3_sim_p3_recorder_capacity_v1_5_proposal.md): the 1.0 s stall limit at 30.31 Hz plus the in-flight frame; previously 8); first overflow is sticky and fails the run (`recorder_overflow_allowed` 0); drain + fsync + close marker. |

## 3. Geometry

From the pinned SDF chain (re-derived at build time by `p3_build.py`, which refuses to build on
any difference): base_link `[0,0,0.24]`, OakD camera_link `[0.12,0.03,0.242]`, StereoOV7251 at
`[0.13233,0,0.26078]` in the model; all rotations zero. Hence

```
p_FLU = R · p_opt + [0.13233, 0, 0.02078],   R = [[0,0,1],[-1,0,0],[0,-1,0]]
p_FRD = diag(1,-1,-1) · p_FLU,   p_NED = position_NED + R(q_wxyz) · p_FRD
```

Deterministic tests cover the forward sign, left/right, up/down, optical-Z vs range, the
mount offset, translation, yaw rotation, nonzero initial heading (incl. the recorded P2
heading 1.674781709 rad), pitch, and ENU/NED and FLU/FRD. They are cross-checked against
the accepted P2 `frames.body_frd_to_ned`.

## 4. Ground fixtures (Gate A)

Fixture geometry and thresholds are the repository's original P3 values (`p3_validation.yaml`
at archived commit `76f014d`, frozen before any P3 runtime), unchanged: 30 sim s window after
5 s settle; ROI `[160,80,480,200)`; valid ≥95 %; median ≤ max(0.05 m, 1 %); p95 ≤ max(0.10 m, 2 %);
source gap ≤0.12 s; delivered ≥25 Hz and processed ≥8 Hz per sim s; age ≤0.25 s; zero
source→record loss; pose drift ≤5 mm / 0.005 rad; interruption detected ≤1.2 wall s.

| Fixture | World (static boxes, vehicle at `0 0 0.1`) | Independent check |
|---|---|---|
| plane2/4/6 | front face 2/4/6 m along the optical axis | per-pixel analytic ray-box depth vs every recorded frame |
| oblique | 4 m plane, yaw 0.35 rad | as above, plus the Euclidean-range hypothesis must fail |
| asymmetric | 4 m plane + right_low (2 m) + left_high (3 m) boxes | each declared region hits its own box; mirrored left/right and up/down locations must not show that depth |
| out_of_range | plane at 25 m | zero valid ROI pixels, +Inf present, no NaN, no zeros |
| interruption | plane4; SIGINT to the sensor bridge after 10 sim s | stale within 1.2 wall s, no observation after detection, bridge exit classified graceful, its log kept |
| scene change | after plane4's window, `set_pose` 4 m → 5 m | post-change frames match the 5 m analytic depth under the same thresholds |

Fixtures run Gazebo + bridges + adapter + probe only. No PX4, DDS Agent, QGC or controller
exists in a fixture namespace, so no flight command is possible; "disarmed" is structural and
stationarity is measured from native ground-truth poses (evaluator only).

## 5. Coexistence, timing and causality

* One controller: the unchanged P2 node. The adapter has no flight publisher (guard plus
  observed node graph); the P2 graph check rejects any competing publisher.
* Six PX4 state families keep their P2 freshness rules; P3 adds sensor freshness separately.
* State association: every window observation must be `matched` within `state_mismatch_s`.
  Under [v1.4](v3_sim_p3_contract_v1_4_proposal.md) rule 1 the only exception is a
  `reset_epoch_bracket` observation whose recorded bracket spans ≤ `state_mismatch_s`, crosses
  exactly one PX4 reset (+1 counter step or a `ref_timestamp` change), is corroborated by the
  controller's `ros-events.jsonl` on the same topic inside the bracket, and is at most one per
  (state, reset event). Such observations are counted (`reset_bracketed_observations`),
  reported and excluded from the matched-delta statistics; every other status still fails.
  The independent stream evaluator separates source gaps (native sequence/stamps), delivery
  loss (native ledger vs ROS receipts), recording loss and intentional processing skips.
* RGB: the IMX214 1920×1080 30 Hz workload is kept and measured by a passive native metadata
  observer (no RGB bridge, no pixels retained; ≥25 Hz, ≤0.12 s gap, ≤1 s receipt stall).
* Exact MC yaw causality uses the explicitly selected, separately built and benchmarked
  `px4-mc-causal-trace-v1` overlay (the untracked `mc_trace*` sources, consumed unchanged).
  Flights require `mc_trace_evidence.exact_flight` = verified at 1e-5 rad; readiness requires a
  verified acquisition. The P2 accepted baseline binary is not modified.
* Recorded workload: native positive-clock RTF, Ogre2 GL renderer string (contract
  `p3-acceptance-v1.2`, operator-approved 2026-10-25: Gates A/B/C must render on
  **`D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`**, otherwise `renderer_not_declared_d3d12`.
  `configs/p3_d3d12_profile.json` pins the D3D12/driver library hashes, and the
  `scripts/p3_gz_shim/gz` shim preloads `libd3d12core.so` only into `gz sim`. Validation V
  stays llvmpipe. History: v1 `hardware`, then [v1.1](v3_sim_p3_renderer_contract_revision_proposal.md)
  llvmpipe, then [v1.2](v3_sim_p3_hardware_renderer_profile_proposal.md); under
  [v1.3](v3_sim_p3_contract_v1_3_proposal.md) the RGB consumer is required at readiness,
  absent in fixtures and absent-verified in flights, which are depth-only workload),
  depth/RGB rates, adapter callback/poll costs, MC spans,
  per-process CPU and RSS, disk before/after (20 GiB reserve checked first), run bytes.

## 6. Ground station and native commands

QGC remains the monitor. A passive AF_PACKET capture of the instance-71 GCS ports starts before
QGC; every frame is CRC-checked against the pinned MAVLink tables. Policy is data
(`configs/p3_native_command_policy.json`):

* `strict_zero` (**default**): any command or active-control frame from a non-vehicle source fails.
* `A`: stock QGC management only — COMMAND_LONG 255/190→72/1, ≤1×401 and ≤34×512 inside the
  original 75 s startup window, each payload one of the frozen patterns within its cap.
  Requires an operator approval receipt bound to the policy's canonical SHA-256. The
  historical approval of A belonged to the archived candidate and does **not** transfer.
  v1.4 rule 2 adds `vehicle_landing_self_commands` to A: PX4's own land-mode gimbal-neutral
  COMMAND_LONG frames 72/1→72/0 (1001 acquire, 1000 neutral, 1001 release, byte-exact
  payloads), only between the controller's `REQUEST_LAND` and `COMPLETE` transitions, at most
  4 per payload, reported as `allowed_vehicle_landing_self_commands`. Any other vehicle
  self-command is still `vehicle_self_command_on_gcs_link`; `strict_zero` is unchanged.

Graceful QGC exit uses the restored, reviewed unattended-shutdown overlay
(`validation/qgc_unattended_v1`), which keeps unsaved-mission, pending-write and reentrancy
checks and the normal LinkManager/VideoManager/application teardown; only the active-vehicle
confirmation is approved, and only after a verified TERM ownership receipt. The compiled owner
identity is now the stable `scripts/p3_namespace_owner.py`, so **QGC must be rebuilt once**
(`build_qgc_unattended.py`) before Gate A.

## 7. Cleanup classification

`p3_process_exit.classify` is graceful only for: declared stop method requested (or the
controller completing its own work), reaped, numeric exit in the declared success codes,
within grace, no escalation, empty process group. SIGTERM, SIGKILL, SIGSEGV and every other
signal death fail, including the deliberately interrupted bridge. Shutdown order: confirm
landed/disarmed independently → QGC unattended TERM → sensor bridge, adapter, probe, clock
bridge (SIGINT) → `mc_pos_control stop` + trace drain → `logger stop` → PX4 `shutdown` →
DDS Agent → pause the Gazebo world → SIGINT Gazebo → close capture → scan the namespace.
Pausing and removing sensor consumers before Gazebo exits is a **mitigation hypothesis** for
the archived −11 exits; it is unverified, and the classifier still requires exit 0.

## 8. Runtime ownership and gates

No runtime starts on import, test or default invocation. Every scope needs the three runtime
gates; flights also need `GWM_ALLOW_SITL_COMMANDS=1` and `--allow-simulated-flight`.

1. Build: `build_p2.sh --build` (unchanged), `build_p3.sh --build`, the MC overlay build and
   benchmark (`build_mc_trace.py --build`, `benchmark_mc_trace.py --measure`), and the
   unattended QGC build bound to the new owner.
2. Freeze: `p3_manifest.py freeze --output M --mc-benchmark-run RUN [--policy A --policy-approval F]`.
   The manifest hashes all sources, evaluators, required MC sources, receipts, binaries,
   pinned assets, ROS/DDS libraries and the effective policy.
3. Authorize (operator, once per gate):
   `p3_manifest.py authorize M --gate A --operator NAME --confirm "authorize Gate A <sha>" --output AUTH_A`.
4. Run exactly one gate: `run_p3_gates.py --gate A --manifest M --authorization AUTH_A`
   (Gate B/C add `--allow-simulated-flight`). The sequencer re-verifies the manifest,
   consumes the authorization exclusively, runs each scope in a fresh namespace
   (`run_p3.sh` → `p3_namespace_owner.py` → `p3_runner.py`), evaluates it with
   `verify_p3_evidence.sh`, stops at the first failure and writes an immutable gate result.
   B requires an accepted A and C an accepted B, for the same manifest. Nothing advances
   automatically and nothing is retried.

## 9. Evidence per run

`summary.json` (P2-compatible fields + P3 identity, processes with raw exits, windows,
startup milestones, policy, renderer, resources, artifact hashes), PX4 ULog, controller bag,
event ledger and timing trace, `sensors/` (raw depth, index, observations, health, adapter
result), `source/` (native ledgers), `mavlink/` (raw packets, index, close record),
`mc-trace.bin` + acquisition report, QGC stage log, `p3-evaluation.json` (exclusive).
Raw depth is ~37 MB per sim second before compression.

## 10. Known limits

* No runtime has been executed for this implementation; every live claim is unproven.
* The archived Gazebo −11 shutdown cause is unknown; the mitigation is untested.
* `p3_source_probe.py`, `p3_runner.py`, the adapter node and `p3_build.py` use APIs that
  are available only in the WSL runtime (gz-transport Python, rclpy, colcon); they are covered
  offline only to the extent described in the report.
* The untracked MC sources remain untracked; the manifest requires and hashes them.
  `tests/test_px4_mc_trace_shutdown.py` targets functions from the stashed P2-runner edit and
  fails to import against the accepted P2 runner.
