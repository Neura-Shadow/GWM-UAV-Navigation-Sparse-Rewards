# P3 contract `p3-acceptance-v1.3` — proposal (RGB evidence at readiness, depth-only flights)

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** No source, configuration, build,
manifest or runtime was changed or started for this proposal. P3 remains INCOMPLETE,
qualification 0/3.

Operator direction: Neura-Shadow, 2026-10-25, direction B. Basis: HEAD `e1a207a`; the stall
attribution and gate D evidence in [`v3_sim_p3_stall_attribution.md`](v3_sim_p3_stall_attribution.md);
renderer and shutdown profile v1.2 ([`v3_sim_p3_hardware_renderer_profile_proposal.md`](v3_sim_p3_hardware_renderer_profile_proposal.md)).

## 1. Summary

v1.3 changes **only who consumes the IMX214 RGB stream, by acceptance scope**:

| Scope | RGB consumer | Evidence |
|---|---|---|
| Gate A readiness | **required**: probe RGB metadata observer active | unchanged v1.2 RGB checks (§4.1) |
| Gate A fixtures (7) | **absent** (as today) | unchanged fixture evaluation, plus `rgb_subscribed: false` (§4.2) |
| Gate B / Gate C flights | **absent, verified** | new RGB-absence evidence, evaluated independently (§4.3) |

The sensors are not changed. `x500_depth` / OakD-Lite SDF, IMX214 1920×1080 @ 30 Hz,
StereoOV7251 640×480 @ 30 Hz, FOV, clipping, geometry, extrinsics, `p3_sensors.yaml` and every
pinned model/world asset hash stay byte-identical. In flight the RGB sensor stays configured
in the model; Gazebo does not render it because nothing subscribes.

## 2. Exact configuration change (`simulation/px4_gazebo/configs/p3_validation.yaml`)

Only these keys change or are added; every other key and value is untouched (the file is JSON):

```diff
-  "contract": "p3-acceptance-v1.2",
+  "contract": "p3-acceptance-v1.3",
-  "contract_revision": "v1.2 (operator-approved 2026-10-25, proposal commit 743bbf7): ...",
+  "contract_revision": "v1.3 (operator-approved <date>, proposal commit <this commit>): RGB consumer by scope (readiness required, fixture absent, flight absent_verified); renderer hardware:d3d12 and every limit unchanged",
   "renderer_requirement": "hardware:d3d12",
+  "rgb_consumer": {"readiness": "required", "fixture": "absent", "flight": "absent_verified"},
+  "rgb_absence_monitor": {
+    "topic_marker": "/sensor/IMX214/",
+    "control_topic": "/depth_camera",
+    "sample_period_wall_s": 0.5,
+    "max_sample_gap_wall_s": 1.0
+  },
```

`configs/p3_sensors.yaml` (including `rgb_sensor` and `native_metadata_observer: true`),
`p3_sensor_bridge.yaml` (depth and camera_info only; no RGB mapping), `p3_d3d12_profile.json`,
the `gz` shim, the D3D12 renderer requirement and the Gazebo shutdown profile are **unchanged**.

## 3. Exact runtime behaviour changes

### 3.1 Runner (`scripts/p3_runner.py`)

1. **Probe RGB flag by scope.** The probe gets `--rgb` only when
   `rgb_consumer[scope] == "required"` (readiness). Fixtures already launch without `--rgb`
   (unchanged). Flights launch without `--rgb` and with `--rgb-absence-monitor`. Diagnostic
   gates D/N keep their own declared variants and remain non-crediting.
2. **No other RGB consumer.** No RGB bridge exists (the bridge YAML has none). The runner
   asserts before launch that `p3_sensor_bridge.yaml` contains no topic carrying the
   `topic_marker`, and aborts otherwise.
3. **Window snapshots.** The runner tails `ros-events.jsonl` (already written by the
   controller) and, on the transition to `PRESTREAM_SAFE_SETPOINTS` (flight-window start) and
   to `COMPLETE` (window end), records:
   * `gz topic -i -t <topic>` for every advertised IMX214 topic and for the control topic
     `/depth_camera`, with start/end monotonic time and the latest native sim time, into
     `source/rgb-absence-snapshots.json`;
   * the namespace process list, which must equal the owned process set (closed process set).

   Each `gz topic -i` call takes about 2 s (discovery wait, measured offline). Snapshots are
   taken in a background thread and never block the controller or pump loop.

### 3.2 Source probe (`validation/p3_source_probe.py`)

* **Flight mode** (`--rgb-absence-monitor`, mutually exclusive with `--rgb`): the existing
  depth, camera_info, pose and clock subscriptions, plus a bounded monitor thread. Every
  0.5 s it calls `Node.topic_list()`, selects topics containing `/sensor/IMX214/` (image and
  camera_info), calls `Node.topic_info(t)` → `(publishers, subscribers)` and the same for
  `/depth_camera`. It appends one row to `source/imx214-subscribers.jsonl`:
  `{id, monotonic_ns, sim_s, topics: {name: subscriber_count}, control_subscribers, error}`.
  An exception (the binding raises for an unknown topic) is recorded in `error`, never
  swallowed.
* `probe-closed.json` additionally records `rgb_absence_monitor: true`, the monitor row count,
  and `node.subscribed_topics()` at shutdown.
* Readiness and fixture probe behaviour are unchanged; readiness still gets `--rgb`.

### 3.3 Offline verification of the introspection API (done for this proposal; no simulator)

In a private loopback namespace with a toy gz-transport publisher on `/toy/image`, using the
pinned `gz.transport13` binding:
* `topic_info` returned `([publisher], [])`, then `([publisher], [subscriber])` after a
  subscribe, then `([publisher], [])` after unsubscribe, each within 1.5 s;
* `gz topic -i` printed `No subscribers on topic [...]` and then
  `Subscribers [Address, Message Type]: tcp://127.0.0.1:…`;
* in the same short call, the CLI printed `No publishers` although a publisher existed. An
  empty CLI answer is therefore not proof on its own, hence the positive control in §4.3.

## 4. Exact evaluator rules (`validation/p3_evaluate.py`, `p3_stream_eval.py`)

### 4.1 Readiness — RGB required (unchanged from v1.2)

`evaluate_rgb` stays exactly as in v1.2: the observer is active (`rgb_subscribed: true`, no
probe error); every row is 1920×1080; native sequence has no gaps or duplicates; timestamps
increase; window coverage and source gap ≤ `source_gap_sim_s` (0.12 s); source rate ≥
`delivered_hz_min` (25 Hz sim); receipt stall ≤ `stall_wall_s` (1.0 s wall). Missing, invalid
or incomplete RGB evidence fails readiness.

**Disclosure:** the operator request lists "observation age ≤ 0.25 s" for RGB. v1.2 has no
RGB observation-age check: observation age applies to depth adapter observations, and RGB
is metadata-only. v1.3 adds no new RGB check. If an RGB age check is wanted, it must be
approved as a separate addition.

### 4.2 Fixtures — RGB absent

Unchanged fixture evaluation, plus one assertion: `probe-closed.json` has
`rgb_subscribed: false`. The fixtures already run this way.

### 4.3 Flights — RGB absent, verified (new `evaluate_rgb_absence`, replaces `evaluate_rgb` for flights)

The flight window is the existing one: sim `[PRESTREAM_SAFE_SETPOINTS, COMPLETE]`. Every rule
must hold; any missing record fails closed:

1. `probe-closed.json`: `rgb_subscribed: false`, `rgb_topic: null`,
   `rgb_absence_monitor: true`, `error: null`; `subscribed_topics` contains no
   `topic_marker` topic.
2. `source/rgb-headers.jsonl` exists and has **0 rows**.
3. Snapshot at window start **and** at window end (`rgb-absence-snapshots.json`):
   * every IMX214 topic reports `No subscribers`;
   * the IMX214 image topic is present;
   * **positive control:** `/depth_camera` reports ≥ 1 subscriber in the same snapshot, which
     proves discovery worked;
   * the start snapshot is taken at or after window start and the end snapshot at or after
     window end, each within the snapshot's own recorded duration.
4. Monitor ledger `source/imx214-subscribers.jsonl`:
   * ids contiguous from 1; strict JSONL;
   * coverage: first row `sim_s` ≤ window start, last row `sim_s` ≥ window end;
   * every wall gap between consecutive rows ≤ `max_sample_gap_wall_s` (1.0 s), otherwise
     `rgb_monitor_gap`;
   * in every row: the IMX214 image topic is listed, every IMX214 subscriber count == 0,
     `control_subscribers` ≥ 1 and `error` is null. Any violation gives
     `rgb_unexpected_subscriber`, `rgb_monitor_control_lost` or `rgb_monitor_error`.
5. **Closed process set:** the namespace process lists at window start and end contain only
   owned processes (no unknown process that could subscribe).
6. Reported, not judged: monitor row count, achieved coverage, maximum sample gap.

The flight result is labelled `rgb_workload: "depth_only_flight"` and has **no** `rgb_stream`
throughput section. Flight RGB throughput is never claimed.

### 4.4 Monitoring method and its limitations (disclosed)

* **Method:** periodic discovery sampling at 0.5 s (≤ 1.0 s gap enforced), endpoint CLI
  snapshots with a positive control, a closed process set (private PID and network
  namespace, loopback only, unique `GZ_PARTITION`, every process owned and enumerated), and
  per-process configuration in which no owned process subscribes to IMX214.
* **Not continuous:** discovery is eventually consistent, and subscribe/unsubscribe was
  observed within ≤ 1.5 s offline. A subscriber that attached and detached entirely between
  two samples, faster than discovery reports it, could be missed by sampling alone. That case
  is excluded by the closed process set and the owned configuration, not by the sampling.
  This document does not claim mathematically continuous observation.
* **Overhead:** one `topic_list` and up to 3 `topic_info` local discovery-cache lookups per
  0.5 s inside the existing probe process, plus 4–6 CLI calls per flight off the critical
  path.

## 5. v1.2 vs v1.3

| Aspect | v1.2 | v1.3 |
|---|---|---|
| Renderer / shutdown profile | D3D12 RTX 4060, libd3d12core Gazebo-only preload | **same** |
| Sensors / SDF / assets / `p3_sensors.yaml` | frozen | **same (byte-identical)** |
| Readiness RGB | observer, evaluated | **same** |
| Fixtures RGB | absent | absent, plus `rgb_subscribed: false` assertion |
| Flight RGB | observer active, `rgb_stream` evaluated | **observer absent; RGB-absence evidence required; no RGB throughput claim** |
| Every numeric limit | as listed in §6 | **identical** |

## 6. Numeric limits — identity assertions (to be enforced by test)

`p3_validation.yaml`: `ground_window_sim_s` 30, `settle_sim_s` 5, `readiness_observation_sim_s`
15, `startup_wall_s` 75, `fixture_wall_deadline_s` 240, `valid_fraction_min` 0.95,
`median_error_m` 0.05, `median_error_fraction` 0.01, `p95_error_m` 0.1, `p95_error_fraction`
0.02, `source_gap_sim_s` 0.12, `delivered_hz_min` 25, `processed_hz_min` 8,
`observation_age_sim_s` 0.25, `state_mismatch_s` 0.05, `stall_wall_s` 1.0,
`interruption_detect_wall_s` 1.2, `recorded_fraction_required` 1.0, `recorder_overflow_allowed`
0, `disk_reserve_bytes` 21474836480, `geometry_pose_translation_drift_m` 0.005,
`geometry_pose_rotation_drift_rad` 0.005, `graceful_stop_wall_s` 15.0,
`qgc_graceful_stop_wall_s` 5.0, `qualification_flights` 3, `front_plane_center_height_m` 4,
`scene_change` {plane4, 5 m, 2 s, 3 s}.

Accepted P2 `p2_control.yaml` (unchanged file, unchanged hash): `state_fresh_wall_s` **2.0**
(the `vehicle_land_detected` wall limit), `flags_fresh_sim_s` 1.5, `position_fresh_sim_s` 0.25,
`attitude_fresh_sim_s` 0.25, `max_sample_gap_sim_s` 0.2, `clock_stall_wall_s` 1.0,
`ack_deadline_wall_s` 5.0, `startup_wall_s` 75.0, `phase_deadline_wall_s` 75.0,
`wall_deadline_s` 420.0, `recovery_wall_s` 45.0. MC causality yaw tolerance `1e-5 rad`, policy A
(1 × 401, 34 × 512) and strict graceful cleanup are unchanged.

## 7. Gate D evidence for the workload change (diagnostic, no credit)

Ground-only, D3D12 RTX 4060, 120 s windows (`v3_sim_p3_stall_diagnostic.json`):

| Variant | mean RTF | wall time per 1.000 s sim p50 / max | > 1.8 s / > 2.0 s |
|---|---|---|---|
| baseline (RGB observer active) | 0.572 / 0.559 (repeat) | 1.695 / 2.891 s; 1.761 / 2.610 s | 15 / 5; 24 / 12 |
| **no RGB observer** | **1.000** | **1.000 / 1.010 s** | **0 / 0** |

The only RGB subscriber's presence decides whether Gazebo renders 1080p RGB inside its step.
That is the cost behind both failed Gate B runs.

## 8. What v1.3 does and does not claim

* **Not yet proven:** full-flight timing. Gate D measured ground windows; flight adds
  physics, controller and MC load. v1.3 Gate B judges the unchanged 2.0 s wall limit; if it
  fails with RGB absent, the result is reported and no further relaxation is introduced.
* **Readiness RGB evidence** establishes that the native 1080p RGB stream is produced and
  delivered within the unchanged sim-time limits on the ground at readiness. It is **not**
  flight RGB performance.
* **Flight evidence** is depth-only workload performance, with the RGB sensor configured but
  unconsumed and its absence verified. P3 completion under v1.3 means depth sensing coexisting
  with the unchanged P2 flight, plus readiness-level RGB evidence.

## 9. Candidate and authorization sequence

1. **Operator approval of this exact v1.3 text** (required before any change).
2. Implement §2–§4, add the regression tests listed in the operator plan, run the Linux P2,
   sensor, evaluator, runtime and MC suites and the Windows P3 suites, and commit locally.
3. Verify build receipts and host dependencies (D3D12 profile hashes). **Fresh Policy A
   approval** for the new commit. Freeze and verify a new candidate, confirm the exact MC
   build selection and that no acceptance artifacts exist for it. Stop.
4. Separate manifest-bound **Gate A** authorization: readiness with mandatory RGB, the 7
   depth-only fixtures. One run; stop at the first failure.
5. Separate **Gate B** authorization: one SITL smoke with RGB absent and verified; full
   land_detected interval distribution, RTF, RGB-absence evidence. Stop after Gate B.
6. Separate **Gate C** authorization: three consecutive qualification flights; stop at the
   first failure.

Earlier results stay as recorded. Both failed Gate B runs, every accepted and failed Gate A,
diagnostic gates D, V and N, and all consumed authorizations and budgets are preserved; the
v1.2 Gate A acceptance does not carry over to the v1.3 candidate.
