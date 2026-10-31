# v3-SIM P3 final acceptance — contract `p3-acceptance-v1.6`

**COMPLETE — Contract v1.6 — Qualification 3/3 PASSED.** Gates A, B and C were accepted on
2026-10-25 for candidate `1e205fec`. Scope: PX4 SITL only, with depth-only flights and RGB
validated separately at readiness. No physical UAV was flown.

Design and operation are described in the [P3 contract](v3_sim_p3_contract.md). This report covers
only the accepted v1.6 implementation and its evidence.

## 1. Architecture and runtime environment

| Item | Value |
|---|---|
| Host | Windows 11, WSL2 Ubuntu 24.04, 24 logical CPUs |
| Simulator | PX4 v1.17.0 SITL (`d6f12ad1`), Gazebo Harmonic 8.15.0, ROS 2 Jazzy, Micro XRCE-DDS Agent v2.4.3, QGroundControl 5.1.4 (unattended-shutdown build) |
| Renderer | `D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)` through Mesa d3d12; `libd3d12core.so` preloaded into `gz sim` only (`scripts/p3_gz_shim/gz`); library pins in `configs/p3_d3d12_profile.json` |
| Vehicle | `gz_x500_depth` (`x500_depth_71`), OakD-Lite depth camera 640×480 at 30 Hz; IMX214 RGB 1920×1080 at 30 Hz |
| Control | The accepted P2-R3 controller, mission and configuration, unchanged (package `e1857b3f…`) |
| Depth path | Gazebo depth → ROS bridge → `gwm_sensor_adapter` (no flight publisher): calibration, 10 Hz processing, PX4 state association, lossless raw recorder |
| Independent checks | `p3_evaluate.py` (stream, geometry, RGB, MAVLink policy, cleanup), the P2 evaluator `collect_p2_evidence.verify()`, and the exact MC causal trace (`px4-mc-causal-trace-v1`) |
| Orchestration | `p3_manifest.py` (freeze, verify, authorize) and `run_p3_gates.py` (one gate per call, authorization consumed, first failure stops, immutable result); each scope runs in a private namespace (`run_p3.sh` → `p3_namespace_owner.py` → `p3_runner.py`) |

## 2. Contract v1.6 and candidate identity

| Identity | Value |
|---|---|
| Completion reference | `ce55c49a9c44c0e64097969e917ed570c47f37d2` |
| Candidate source commit | `087722b9d4cfc1deda132404898254e70e219133` |
| Manifest | `~/uav_autonomy/state/p3-candidate-087722b.json`; canonical SHA-256 **`1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c`**; file SHA-256 `a38ce477b5143315f8121aec110b06a192c7dec92616a87899ec42b4a9d5d3d9` |
| Bound inputs | 103 source files, including `p3_validation.yaml` `2d8510f7…`, `p3_sensors.yaml` `e02ae2a4…`, `p3_native_command_policy.json` `7a7be282…` and `collect_p2_evidence.py` `3c7940018941f342e869b927779537ff12ecf1dba1b65a81aa59d7fe7cd9e19c`; 8 MC overlay sources; 6 build receipts; 44 assets; 4 runtime libraries |
| Runtime | sensor package `c5d374137f6556df710c310f8eec994c778c297174b5407866c0d582f820c9b6`; MC build `7542919c732f5739b3563a9cee56955a34f7b5047efee03397249296f6ea378a` (overlay `ecdf4eaa…`, benchmark run `20261025T051926Z-mc-trace-benchmark-e6000074`); QGC ELF `829942834b71bbd7f30967cb32151e7a32bc1a7fbb9609fd15e2f4dde3e46d03`; D3D12 profile `2e9e9ba1b76fd91a992e8d40f13b80a79e6985613b7bfd979beeaa5939f93459` |
| Native-command policy | `A`, canonical SHA-256 `bf1ef9b1080a52eab1673ac2a1ac6e20d30cfa7fd85b703cae8433dc55f7b3b0`; operator approval `state/p3-policy-A-approval-087722b.json` (`75aa0d63a53664aaad8161c82cbd8fda225cff9c8ae98a97b899459e57435265`), Neura-Shadow, bound to `087722b` |

**v1.6 rule set.** Every numeric limit is the frozen P3 threshold set; none was relaxed.

* **Limits.** Ground window 30 sim s, settle 5 s, readiness observation 15 s, startup 75 s wall,
  fixture deadline 240 s. Valid fraction ≥ 0.95; median error ≤ max(0.05 m, 1 %); p95 ≤ max(0.1 m,
  2 %). Source gap ≤ 0.12 s; delivered ≥ 25 Hz; processed ≥ 8 Hz; observation age ≤ 0.25 sim s;
  state mismatch ≤ 0.05 s; receipt stall ≤ 1.0 s wall; interruption detected ≤ 1.2 s. Recorded
  fraction 1.0, recorder overflow 0. Pose drift ≤ 0.005 m / 0.005 rad. Graceful stop ≤ 15 s (QGC
  ≤ 5 s). Disk reserve 20 GiB. Three qualification flights.
* **Renderer (v1.2).** Gates A/B/C must report the D3D12 RTX 4060 renderer; validation gate V stays
  llvmpipe and never credits.
* **RGB by scope (v1.3).** The RGB consumer is required at readiness (unchanged RGB throughput
  checks), absent in fixtures, and absent and verified in flights (subscriber monitor every 0.5 s,
  maximum gap 1.0 s, snapshots at `PRESTREAM_SAFE_SETPOINTS` and `COMPLETE`).
* **EKF reset brackets (v1.4 rule 1).** Every window observation must be `matched`, except a
  `reset_epoch_bracket` whose recorded bracket spans ≤ 0.05 s, crosses exactly one PX4 reset (+1
  counter or a `ref_timestamp` change), is corroborated by the controller's `ros-events.jsonl`
  inside the bracket, and occurs at most once per (state, reset). Such observations are counted and
  excluded from matched deltas.
* **Landing self-commands (v1.4 rule 2).** Policy A allows PX4's land-mode gimbal-neutral
  `COMMAND_LONG` frames 72/1→72/0 (1001 acquire, 1000 neutral, 1001 release; byte-exact payloads),
  only between the controller's `REQUEST_LAND` and `COMPLETE`, at most 4 per payload. The GCS
  allowance stays 1 × 401 and 34 × 512 inside the 75 s startup window. `strict_zero` is unchanged.
* **Recorder capacity (v1.5).** `recorder_capacity` 32 = ⌈1.0 s × 30.31 Hz⌉ + 1, so any writer
  stall longer than the stall limit still overflows and fails.
* **P2 position pairing (v1.6).** `collect_p2_evidence.verify()` pairs ROS/ULog
  `vehicle_local_position` by (`timestamp`, `timestamp_sample`) and fails closed on an ambiguous key;
  tolerance 1e-6 m and minimum 100 pairs unchanged. All 21 accepted P2-R3 reports reproduce
  identically under the change.

## 3. Gate results

| Gate | Scope | Result file SHA-256 | Authorization SHA-256 | Consumption marker SHA-256 | Run (UTC) |
|---|---|---|---|---|---|
| A | readiness + 7 fixtures | `5d67e8f9d7a5822761e59fc69278f074200a116a5f34f900a5941def46ade99d` | `271a076188c6fcc5806a0bc903c5d6669429abd64b3b73c0752d7af4b36f369e` | `e1bf5d71c293c24e43e0ff7054935f123c37111ae44948614ae1cbd74527bbd9` | 16:00:23–16:07:35 |
| B | 1 flight smoke | `52cbd0999c86a6f683cd8df5c678d1ed9d7fa634da7f11967191010300e91e2a` | `a74c875b87957083f49bc78ec66ad22333bdbd83e5649140583521efc5c6a638` | `81c00bdac94e3db927c036c6a6e8993b97260addce30d7391eba041ebf2ad0bf` | 16:08:46–16:11:30 |
| C | 3 consecutive qualification flights | `f4599afb7d1c6485d12a0a7d0fd51b9264ac28eb925670c32a25a8af56fc45e4` | `50cf5f0f9280bfeb46a8d1f89b01bd2cf47600e57ace69b0f9b454087f38e615` | `efea2a9fd3f598a69df21d4c7ad20fe605bf34a4fed5a58feba659d8d45bd33c` | 16:12:42–16:20:41 |

All three are `accepted: true` and `acceptance_credit: true`, with no failure and no automatic
retry. B names A as its predecessor, and C names B. Files are
`~/uav_autonomy/state/p3-gate-{A,B,C}-1e205fec….result.json` and `p3-auth-{A,B,C}-1e205feca482.json`.

## 4. Qualification and smoke flights

The unchanged P2 mission (2 m takeoff, 10 s hover, ±1 m east, ±1 m north, +30° yaw, final hover,
land) ran on D3D12 with RGB absent and verified.

| Flight | Run | `COMPLETE` (sim) | `land_detected` max gap | RTF | Depth recorded · association | P2 position pairs |
|---|---|---|---|---|---|---|
| Gate B smoke | `20261025T160901Z-p3-flight-a8b0f181` | 105.56 s | 1.008 s | 0.998 | 2,813/2,813 · 928/928 matched | 5,017, Δ 0 |
| qualification-1 | `20261025T161256Z-p3-flight-fedc094c` | 105.60 s | 1.040 s | 0.997 | 2,820/2,820 · 931/931 matched | 5,025, Δ 0 (1 duplicate timestamp, paired correctly) |
| qualification-2 | `20261025T161532Z-p3-flight-a10fef87` | 105.68 s | 1.037 s | 0.998 | 2,818/2,818 · 929 matched + 1 reset bracket | 5,019, Δ 0 |
| qualification-3 | `20261025T161811Z-p3-flight-3e3b9ec1` | 105.62 s | 1.006 s | 0.998 | 2,813/2,813 · 928/928 matched | 4,949, Δ 0 |

In every flight: source 30.30 Hz, maximum source gap 0.036 s, processed 10.0 Hz, maximum observation
age ≤ 0.068 s, maximum receipt gap ≤ 0.065 s wall, and recorder complete with 0 overflow (maximum
queue 1). The `vehicle_land_detected` limit is 2 s wall. Qualification-2's bracketed observation
(acquisition 22.212 s) straddled `quat_reset_counter` 1→2 (bracket 22.200–22.216 s) and was
corroborated by the controller ledger at 22.216 s.

## 5. Depth sensing and geometry (Gate A)

Readiness `20261025T160037Z-p3-readiness-4c6b725c`: depth 30.31 Hz source, 10.0 Hz processed,
maximum age 0.076 s, association 150/150 matched.

| Fixture | Run | Frames | Valid fraction | Max median / p95 error (limit) |
|---|---|---|---|---|
| plane2 | `…160130Z-p3-plane2-92fc0be1` | 910 | 1.0 | 0 / 0 m (0.05 / 0.1) |
| plane4 | `…160225Z-p3-plane4-bb0a6a66` | 910 | 1.0 | 0 / 0 m (0.05 / 0.1); scene change 4 → 5 m measured 5.000 m (91 frames) |
| plane6 | `…160328Z-p3-plane6-13238a8b` | 910 | 1.0 | 0 / 0 m (0.06 / 0.12) |
| oblique | `…160423Z-p3-oblique-2ef0617c` | 910 | 1.0 | 0.0017 / 0.0022 m (0.05 / 0.1); the Euclidean-range hypothesis gives 0.199 m and is rejected |
| asymmetric | `…160518Z-p3-asymmetric-a1286654` | 910 | 1.0 | 0 / 0 m (0.05 / 0.1) |
| out_of_range | `…160612Z-p3-out_of_range-d341babb` | — | — | out-of-range classification passed; no values fabricated |
| interruption | `…160706Z-p3-interruption-bc9fad08` | — | — | sensor bridge interrupted; detected after 0.238 s wall (limit 1.2 s), 0 observations afterwards |

The geometry evaluator imports no production geometry, fixture or adapter code. Fixtures have no RGB
consumer. Gazebo and every owned process exited gracefully in all 8 Gate A runs.

## 6. RGB at readiness versus depth-only flights

* **Readiness (RGB evidence):** IMX214 at 30.31 Hz sim, maximum source gap 0.036 s, receipt stall
  0.077 s wall, 454 frames, 0 late or unrendered; readiness RTF 0.803 with the RGB consumer active.
* **Flights (no RGB evidence):** RGB absent and verified in all four flights (`depth_only_flight`).
  No IMX214 subscriber appeared in the monitor or the snapshots, and no RGB frame was recorded.
  These flights make no RGB throughput claim.

## 7. P2 control and landing

All four flights passed the P2 evaluator: flight acceptance, recording integrity, command sequence
[176, 400, 21] matched in ROS and ULog, landed and disarmed at the end, no failsafe, and yaw
ownership passed. Each landing reached `COMPLETE`.

## 8. MC causality and native-command policy

* **MC causality:** `verified` in readiness (acquisition) and in all four flights (exact flight
  check: every MC update paired, yaw error within 1e-5 rad).
* **Policy A:** 0 violations everywhere. Each flight saw 1 × 401 and 34 × 512 from QGC inside the
  startup window, plus the 6 PX4 landing self-commands (rows a × 4, b × 1, c × 1) inside
  `REQUEST_LAND`→`COMPLETE`. The GCS heartbeat was present and the capture complete.

## 9. Graceful cleanup

Every owned process (Gazebo, PX4, DDS Agent, QGC, sensor and clock bridges, source probe, adapter,
controller) was classified `graceful` in all 12 accepted runs, and no residual namespace process
remained after any gate.

## 10. Reproducibility and evidence

Retained evidence (`~/uav_autonomy`): the manifest, its Policy A approval, the six build receipts,
the A/B/C authorizations, consumption markers and results, the 12 accepted run directories (ULog,
rosbag, ROS event ledger, MC trace, raw depth and index, observations, MAVLink capture, summary and
`p3-evaluation.json`), the MC benchmark run, the MC overlay build, the sensor package build and the
pinned runtime (upstream PX4, ROS workspace, DDS install, QGC build, venv).

Read-only verification (no runtime):

```bash
cd /mnt/d/GWM-UAV-Navigation-Sparse-Rewards
python3 simulation/px4_gazebo/scripts/p3_manifest.py verify ~/uav_autonomy/state/p3-candidate-087722b.json
```

```bash
sha256sum ~/uav_autonomy/state/p3-gate-{A,B,C}-1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c.result.json
```

The first command recomputes every bound input and prints the canonical hash above. Each run's
`p3-evaluation.json` is written once by the gate and is never overwritten. Rebuild prerequisites
and the gate procedure are in the [P3 contract](v3_sim_p3_contract.md) (section 8) and
`simulation/px4_gazebo/README.md`. A new run needs a newly frozen manifest, a Policy A approval for
that commit, and separate gate authorizations.

**Limits of the claim.** SITL only. Flights are depth-only. A host-wide slowdown could delay PX4
state messages and fail state association; this did not occur in the accepted runs.
