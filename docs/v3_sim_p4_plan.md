# v3-SIM P4 — deterministic non-learning obstacle avoidance: implementation plan

**Status: contract approved (revisions 2 and 3); this plan records the design.** Nothing was built, launched, frozen or
pushed for this plan. The normative rules (thresholds, scenarios, gates) are in the proposed
[P4 contract](v3_sim_p4_contract.md); this document records the integration facts and the design
an implementer follows.

Baseline: HEAD `f0f9c0c6087afc6c95853bd2d98be6bf33b20c4b` (clean apart from the untracked
`.codegraph/` tool folder), P3 COMPLETE under `p3-acceptance-v1.6`, candidate `1e205fec`
([final acceptance](v3_sim_p3_final_acceptance_v1_6.md)).

## 1. Verified P2/P3 integration points

All facts below were read from the current source and the retained P3 evidence.

### 1.1 The P3 manifest scope decides where P4 code may live

`scripts/p3_manifest.py` `source_files()` hashes **every file** (by `rglob`) under
`simulation/px4_gazebo/{scripts,validation,configs}`, `ros2_ws/src/gwm_px4_control`,
`ros2_ws/src/gwm_sensor_adapter`, and `docs/evidence/v3_sim_p2_summary.json`. `verify_current()`
recomputes this set and compares it to the frozen manifest. **Adding one P4 file to any of those
folders would make the accepted P3 candidate fail verification.** The P2 and P3 builds use
`colcon build --packages-select gwm_px4_control` and `gwm_sensor_adapter`, so they ignore other
packages.

**Consequence:** all P4 code lives in a new sibling tree, `simulation/px4_gazebo_p4/`, outside every
P3 glob. P4 may *import* P2/P3 modules read-only, but never edits them.

### 1.2 How P3 publishes depth

* Gazebo `/depth_camera` 640×480 `32FC1` at 30 Hz → `parameter_bridge` → ROS
  `/gwm/sensors/front_depth/image_raw` and `/camera_info` (`configs/p3_sensors.yaml`).
* `gwm_sensor_adapter` decodes, validates calibration, processes at 10 Hz (sim-time phase), associates
  PX4 state and publishes `std_msgs/String` JSON on `/gwm/sensors/front_depth/observation`, and health
  on `/gwm/sensors/front_depth/health`.
* **An observation carries only a 7×5 sample grid (35 pixels) and five region statistics** (center,
  left, right, upper, lower), plus association, age and provenance. That is too sparse to clear a
  detour corridor. The full frame is only recorded to `sensors/depth.bin`; it is not republished.
* **Consequence:** P4 perception subscribes to `/gwm/sensors/front_depth/image_raw` itself (a second
  subscriber) and reuses the P3 functions, unmodified: `depth.decode`, `depth.classify`,
  `camera_info.validate` / `CalibrationCache`, `geometry.backproject`, `Extrinsic.optical_to_frd`,
  `geometry.frd_to_local_ned`, and `association.Associator`. Changing the adapter's output instead
  would change P3 manifest-bound files.

### 1.3 Sensor geometry (from `p3_sensors.yaml` and the pinned SDF)

| Quantity | Value |
|---|---|
| Depth image | 640×480, 30 Hz; near 0.2 m, far 19.1 m |
| Horizontal FOV | 1.274 rad (73.0°); vertical FOV = 2·atan(tan(0.637)·480/640) = 1.014 rad (58.1°) |
| Optical origin | base FLU (0.13233, 0, 0.02078) m; frames `camera_link` → `gwm_front_depth_optical` (extrinsic id `x500-depth-optical-to-base-v1`) |
| P3-validated accuracy | median error ≤ 0.05 m and p95 ≤ 0.1 m out to 6 m; oblique 0.0017 / 0.0022 m |
| Measured latency (v1.6 flights) | source-to-receipt ≤ 0.092 s sim; processed observation age ≤ 0.068 s (limit 0.25 s) |

### 1.4 Vehicle state, frames and airframe envelope

* PX4 `vehicle_local_position` (NED), `vehicle_attitude` (Hamilton wxyz, body FRD → NED),
  `vehicle_status`, `vehicle_land_detected`, `estimator_status_flags`, `failsafe_flags` through
  `/px4_71/fmu/out/...` (DDS domain 71). The mission frame is ENU offsets from the recorded initial
  NED origin (`gwm_px4_control.frames`).
* **x500 collision envelope** (pinned `x500_base/model.sdf`): rotors at (±0.174, ±0.174) m with
  0.279 m blades, so the rotor-tip radius from `base_link` is √2·0.174 + 0.1396 = **0.386 m**; the
  vertical extent is −0.227 m (gear) to +0.061 m (rotor plane) about `base_link`. The OakD-Lite sits
  inside that radius.
* PX4 defaults: `MPC_ACC_HOR` 3 m/s², `MPC_ACC_HOR_MAX` 5 m/s² (unchanged; P4 sets no parameters).

### 1.5 How P2 controls PX4, and why P4 needs its own controller

* `gwm_px4_control/node.py` defines `ControlNode` **inside `main()`**. At 20 Hz it publishes
  `offboard_control_mode` and `trajectory_setpoint` (position mode) and the only commands 176
  (Offboard), 400 (arm) and 21 (LAND), over ROS 2 only.
* `inspect_graph()` requires **exactly one publisher** on each PX4 input topic and raises
  `competing_control_publisher` otherwise. This is the existing single-owner guard.
* `mission.Mission` is a fixed state machine: initialization (v3 reference policy, yaw handover),
  then `INITIAL_HOVER`, `EAST_TEST` … `FINAL_HOVER`, and LAND. Goals come from `p2_control.yaml`
  offsets; the target ramps at `ramp_m_s` 0.3 m/s and yaw at 10°/s; envelope
  `max_horizontal_radius_m` 2.0 m and height −0.5 to 3.0 m. **There is no upstream
  reference/trajectory input**, so no guarded planner-to-controller interface exists.
* Reusable P2 modules (read-only imports): `frames`, `protocol.AckTracker`, `timing.StateCache`,
  `estimator_reference` (v3 reference and yaw handover), `contracts`, `execution_timing`
  (`DispatchGuard`, `controller_transport_env`), and the setpoint helpers `position_setpoint`,
  `offboard_mode` and `topic_name` used by `node.py`.

### 1.6 Runtime ownership, cleanup and evaluation

* `scripts/p3_runner.py` `P3Runner` launches owned processes in a private namespace, with
  `sensor_stack`, `px4_stack` (PX4, QGC monitor, DDS Agent, clock bridge, P2 controller), the MC
  trace and graceful ordered shutdown classified by `p3_process_exit.py`.
* `p3_evaluate.py` composes independent checks: stream (`p3_stream_eval`), RGB / RGB absence,
  MAVLink policy (`p3_mavlink`), renderer, MC trace (`mc_trace_evidence`), cleanup, and the P2
  evaluator. Its flight path calls `collect_p2_evidence.verify()`, which checks **P2's fixed mission
  windows**, so it cannot judge a detour.
* The P3 source probe records ground-truth poses only for the vehicle, decimated to 5 Hz (200 ms).

## 2. Architecture and control ownership

### 2.1 Decision: an exclusive P4 controller, not a modified P2

The P2 node is **not launched** in P4 runs. A new `p4_control` node is the single owner of
`offboard_control_mode`, `trajectory_setpoint` and `vehicle_command`, and keeps P2's graph guard
semantics (exactly one publisher per PX4 input, `competing_control_publisher` aborts). It reuses the
P2 modules from 1.5 by import, so initialization, reference handling, yaw handover, ACK tracking,
freshness and timing behave exactly as in P2. Accepted P2 sources, configuration and package
identity stay untouched.

```
Gazebo depth ──bridge──► /gwm/sensors/front_depth/image_raw ──┬─► gwm_sensor_adapter (P3, unchanged: recorder, observations, health)
                                                              └─► p4_perception ──► /gwm/p4/local_map (10 Hz, JSON + provenance)
PX4 /fmu/out/* ───────────────────────────────────────────────┬─► p4_perception (state association, P3 rules)
                                                              └─► p4_control: P4Mission + Supervisor + Planner
p4_control ──► /fmu/in/offboard_control_mode, trajectory_setpoint, vehicle_command   (sole publisher)
p4_truth_observer (evaluator-only) ◄── Gazebo dynamic poses (all models) + obstacle contact sensors
```

### 2.2 Source layout (all outside the P3 manifest scope)

```
simulation/px4_gazebo_p4/
  README.md
  configs/p4_avoidance.yaml          perception, planner, supervisor, mission (P4 envelope)
  configs/p4_validation.yaml         acceptance limits, scenarios, gates (contract identity)
  configs/p4_scenarios/*.json        obstacle declarations (box poses and sizes)
  scripts/p4_manifest.py             freeze/verify/authorize; binds P4 sources + the P3 baseline
  scripts/p4_build.py, build_p4.sh   colcon build of gwm_obstacle_avoidance into ~/uav_autonomy/p4_ws/<hash>
  scripts/p4_runner.py               subclass of P3Runner (imported read-only)
  scripts/run_p4.sh, run_p4_gates.py one gate per call; same authorization/consumption rules as P3
  scripts/p4_worlds.py               scenario world generation (P3 fixture pattern)
  validation/p4_evaluate.py          independent evaluator (composes P3 checks + P4 checks)
  validation/p4_truth_observer.py    evaluator-only Gazebo ledger (poses at 20 Hz, contacts)
  validation/p4_trajectory_eval.py   clearance, collisions, latency, progress, ownership
  ros2_ws/src/gwm_obstacle_avoidance/
    gwm_obstacle_avoidance/{perception.py, local_map.py, planner.py, supervisor.py, mission.py,
                            control_node.py, perception_node.py, contracts.py}
    test/                            installed-package tests
tests/test_px4_gazebo_p4_{perception,planner,supervisor,mission,evaluator,runtime}.py
```

### 2.3 Disabled by default

* Nothing in P2 or P3 imports P4, and the P2/P3 runners never launch P4 processes.
* `p4_control` and `p4_perception` refuse to start unless `p4_avoidance.yaml` has `enabled: true`
  **and** the P4 runner passes a consumed P4 authorization bound to the frozen P4 manifest.
* P2/P3 behaviour is therefore unchanged by construction. A regression test asserts that no file
  under the P3 `SOURCE_FOLDERS` changes and that `p3_manifest.verify_current` still returns
  `1e205fec…`.

## 3. Obstacle detection and local planning algorithms

### 3.1 Perception (`perception.py`, `local_map.py`), 10 Hz

1. **Frame intake.** Keep only the latest image (overwrite queue, as in P3). Process on the 10 Hz
   sim-time phase. Validate with `depth.decode` (strict layout), require a usable `CalibrationCache`
   whose identity equals the adapter's calibration identity (from its health/observation stream),
   and require the adapter's health to be `fresh`.
2. **State association.** Use `association.Associator` with the P3 settings (300 samples,
   0.05 s, no interpolation). Plan only on `matched` frames. A `reset_epoch_bracket` frame, which the
   v1.6 evaluator may accept, is **not** used for planning. Any other status marks the frame
   unusable and reports it.
3. **Depth to horizontal scan.** Subsample every 4th column (160 azimuths). For each pixel in the
   column compute the world point (`backproject` → `optical_to_frd` → `frd_to_local_ned` with the
   associated pose). Keep points whose world height lies in the **flight band**: vehicle height
   −0.38 m to +0.22 m, which is the envelope (−0.227, +0.061) plus 0.15 m. Ground and overhead points
   fall outside the band. Per azimuth the range is the minimum horizontal distance of `VALID` band
   points. With no valid band point the result depends on the band rows: all far/no-return gives free
   to the **free cap** (4.0 m); otherwise (NaN, near or invalid) the azimuth is **unknown**.
4. **Local occupancy grid.** A 2-D grid in the mission frame at 0.10 m resolution covers the P4
   geofence. Cells are `UNKNOWN`, `FREE` or `OCCUPIED`. Each azimuth ray marks cells `FREE` up to its
   range (or the cap) and `OCCUPIED` at a valid hit. A hit overrides free, and free never clears an
   occupied cell (static world). Cells never covered stay `UNKNOWN` and are **never** treated as free.
5. **Publication.** `/gwm/p4/local_map` (`std_msgs/String`, JSON) carries the frame stamp,
   association, calibration identity, the zlib+base64 2-bit grid, the scan, and counts of unusable
   frames by reason. Every snapshot is also written to `p4/local_map.jsonl` with the raw frame's
   receipt id.

### 3.2 Planner (`planner.py`): collision-checked candidates with deterministic scoring

Each control tick (20 Hz) runs on the newest snapshot no older than 0.25 sim s:

* **Candidates.** Headings θ = ψ_path + k·10°, k = −6…6 (±60° about the path direction toward the
  current goal), each with length L ∈ {0.75, 1.5} m from the current target position, plus `HOLD`.
* **Collision validity** (contract L2) does not depend on the current heading. Every cell within
  `r_infl` (1.0 m) of the swept segment is `FREE`; the inflated segment lies inside the geofence; the
  first `d_stop` (0.30 m) is clear; and the snapshot is ≤ 0.25 s old. Cells remembered `FREE` from
  earlier frames count (static world).
* **Score** (contract L3, fixed weights in `p4_avoidance.yaml`): `+1.0·progress − 0.6·path_deviation
  − 0.2·|Δθ| + 0.3·min(clearance − r_infl, 1.0)`, over collision-valid candidates only. Ties go to the
  smaller |k|, then +y, then the shorter L. Identical inputs give identical output.
* **Translation authority** (contract L4) is a separate check. The best candidate translates only if
  |θ − ψ_vehicle| ≤ 25°. Otherwise the planner returns `YAW_TO(θ)`, so the camera always faces the
  space being entered.
* **In-place yaw** (contract L5). `YAW_TO`, `SCAN` and the initial `MAP_SCAN` freeze the position
  target and need a fresh snapshot. They also need every cell within `R_body` of the target to be
  `FREE` or traversed (the vehicle occupied it), and no `OCCUPIED` cell within 0.79 m
  (`R_body` + 0.10 + `e_track`). Rotation is at ≤ 10°/s. A cylinder rotating about its axis sweeps
  nothing new, so no unobserved space is entered. Translation authority is re-checked only on a snapshot acquired after the new heading is
  reached.
* **Observation acquisition** (contract L6). When candidates are blocked only by `UNKNOWN` cells,
  `SCAN` yaws toward the most-unknown blocked heading, once per heading per `HOVER` episode.
  Translation never enters `UNKNOWN` space.
* **Execution.** The P4 mission ramps a translating target along the chosen segment at ≤ 0.3 m/s,
  and yaw at ≤ 10°/s (P2 limits), re-validating every tick.

### 3.3 Supervisor (`supervisor.py`)

The states are `NORMAL`, `OBSTACLE_DETECTED`, `AVOIDING`, `HOVER`, `RESUME` and `ABORT`, with the
transition table in the contract (§5). Summary:

* `NORMAL`: the straight path segment (L = 1.5 m, r_infl) is valid.
* If an `OCCUPIED` cell lies in the path corridor within 3.0 m, go to `OBSTACLE_DETECTED`: hold the
  target and pick the best candidate. If valid, go to `AVOIDING`; otherwise `HOVER`.
* `AVOIDING` executes candidates. When the path segment is valid again, go to `RESUME`, which rejoins
  the path line (deviation-penalized candidates). When deviation ≤ 0.10 m and the path is valid, go
  back to `NORMAL`.
* `HOVER` holds the current target and may run L6 `SCAN` yaws when the snapshot is fresh. If no
  collision-valid progress candidate appears within 20 s sim, go to `ABORT`. In a stale-input
  `HOVER`, yaw is frozen as well.
* `ABORT` holds, then sends one LAND (21) in place, using P2's recovery rules.
* **Immediate `HOVER`, never progress:** map snapshot older than 0.25 sim s, adapter health not
  `fresh`, calibration mismatch, unusable association for 3 consecutive frames, an estimator reset
  counter change since the last usable frame (until a fresh matched frame after the reset), or a
  planner output that fails re-validation.
* **Immediate `ABORT`:** any P2 safety error (freshness, offboard lost, envelope, graph or
  competing owner, ACK failure, deadlines); `HOVER` while stale for more than 10 s sim; or measured
  clearance to an `OCCUPIED` cell below `R_body + 0.10` m.

## 4. Safety envelope and threshold derivation

Every value comes from measured or pinned quantities; the contract lists them as proposed limits.

| Symbol | Value | Basis |
|---|---|---|
| `R_body` | 0.39 m | rotor-tip radius 0.386 m (pinned SDF), rounded up |
| `v_max` | 0.30 m/s | P2 `ramp_m_s` (unchanged) |
| `ω_max` | 10°/s | P2 `yaw_ramp_deg_s` (unchanged) |
| `T_lat` | 0.60 s | observation age limit 0.25 + perception period 0.10 + control tick 0.05 + `max_sample_gap_sim_s` 0.20 |
| `a_brake` | 1.0 m/s² | one third of PX4 `MPC_ACC_HOR` 3 m/s² (conservative) |
| `d_stop` | 0.30 m | v·T_lat + v²/(2a) = 0.18 + 0.045 = 0.225 m, rounded up |
| `e_depth` | 0.10 m | P3 p95 depth limit |
| `e_track` | 0.30 m | P4 horizontal tracking bound (P2 height tolerance 0.3; P2 horizontal 0.2 plus ramp lag) |
| `e_grid` | 0.07 m | half-diagonal of a 0.10 m cell |
| margin | 0.10 m | engineering reserve |
| `r_infl` | **1.0 m** | 0.39 + 0.10 + 0.30 + 0.07 + 0.10 = 0.96, rounded up |
| trigger range | 3.0 m | d_stop + longest candidate 1.5 + r_infl ≈ 2.8, rounded up |
| free cap | 4.0 m | beyond the trigger range; no-return space is not trusted further |
| evaluator minimum clearance | **0.20 m** | rotor-tip envelope to obstacle surface, ground truth; leaves 0.40 m of the 0.60 m error budget unused before failure |

Gaps narrower than 2·r_infl (2.0 m between obstacle surfaces) are treated as impassable by design.

## 5. Scenarios and acceptance (summary; normative values in the contract)

P4 worlds are generated from the pinned `default.sdf` with the P3 generator pattern: static boxes,
the `x500_depth` include and the gz `Contact` system, with a contact sensor on each obstacle (the
vehicle SDF is untouched). Mission: takeoff to 2.0 m with P2 initialization and yaw handover, a
deterministic 360° map scan after `INITIAL_HOVER` (revision 3), transit to a goal 8.0 m ahead (ENU +x of the
recorded origin heading), a 5 s goal hover, then LAND. P4 geofence: x ∈ [−1.0, 9.5] m,
|y| ≤ 3.0 m, height 1.0–3.0 m.

| ID | Scenario | Obstacles (centre x, y; size) | Required outcome |
|---|---|---|---|
| S1 | Clear path | none | goal reached, no `AVOIDING`, lateral deviation ≤ 0.30 m |
| S2 | Centre pillar | (4.0, 0.0); 0.4×0.4×4 m | detour, goal reached, landed |
| S3 | Offset pillar | (4.0, +0.5); 0.4×0.4×4 m | detour toward −y (the larger free side), goal reached |
| S4 | Two pillars | (3.5, 0.0) and (6.0, −1.4); 0.4×0.4×4 m | collision-free route, goal reached |
| S5 | Blocked | wall (4.0, 0.0); 0.3×8.0×4 m | `HOVER` before the wall, then `ABORT` + LAND; outcome `safe_refusal` |
| S6 | Depth interruption | S2 geometry; the runner stops the sensor bridge once the vehicle is 2.0 m from origin | full P3 checks before injection; then `HOVER` within 0.35 s sim of the last usable snapshot, no motion while stale, `ABORT` + LAND (contract §9.1) |
| S7 | Reset / stale pose | ground replay only (§6) | input rejected, `HOVER`, recovery on a fresh matched frame |

## 6. Runtime integration and fault handling

* **`P4Runner(P3Runner)`.** It reuses `sensor_stack` (P3 adapter, recorder, probe and RGB-absence
  monitor unchanged), PX4/QGC/DDS/clock bridge and the MC trace. It overrides only the controller
  launch (`p4_control` instead of `p2_control`) and adds `p4_perception` and `p4_truth_observer` as
  owned processes, with SIGINT and the same graceful classification. Shutdown order adds P4
  processes alongside the controller and sensor consumers, before Gazebo pauses.
* **Worlds.** `p4_worlds.py` writes the scenario world into the run directory; the P3 fixture
  generator is not modified.
* **MAVLink policy.** The same `policy A` content as P3 v1.6 (QGC 1×401, 34×512; PX4 landing
  self-commands). An ABORT LAND happens while not in `REQUEST_LAND`, so its landing self-commands
  fall outside the v1.4 window. The P4 contract therefore defines the landing window as the first
  LAND (21) issued by the P4 controller (normal or ABORT) through `COMPLETE`, and the P4 evaluator
  applies it by passing that window to the unchanged `p3_mavlink.evaluate_policy`.
* **MC causality.** Unchanged `mc_trace_evidence.exact_flight`, with the yaw-ownership window from
  the P4 controller's handover record (same v3 phases as P2).
* **Fault injection (S6 only).** `P4Runner` tails `ros-events.jsonl`. On the first `TRANSIT`
  sample ≥ 2.0 m from origin, it sends one SIGINT to the owned `sensor-bridge` and writes
  `p4/fault_injection.json`. The evaluator splits the run at `t_inj`: unchanged P3 stream checks on
  `W_pre` = [window start, `t_inj` − 0.12 s], and the attribution, detection and response checks
  (F1–F4) on `W_fault` (contract §9.1). The native Gazebo depth ledger from the unchanged P3 source
  probe proves that only the bridge was cut. Any other depth loss, or any loss in another scenario,
  still fails.
* **Runtime load.** `p4_perception` is a second subscriber to the 640×480 `32FC1` depth topic
  (≈ 37 MB/s) and does about 19 k point transforms per processed frame (160 azimuths × ≈ 120 band
  rows) at 10 Hz in NumPy. It records per-frame thread wall time, snapshot latency, receipt gaps and
  skipped frames. Gate A part 2 runs the full P4 stack on the ground against every contract §6.1 limit
  (P3 stream limits, P2 freshness and timing, P4 perception rate, latency and compute) before any
  flight is authorized, and the same limits are evaluated in every flight.
* **Faults.**
  * Depth interruption (S6): adapter health stale, then `HOVER` with yaw frozen, then `ABORT`.
  * State reset or stale pose: unusable association, so the supervisor holds `HOVER`. A SITL
    reset cannot be triggered without new PX4 commands, so S7 is verified by offline replay of
    retained P3 frames with injected reset brackets and stale stamps.
  * Competing publisher: P2's graph guard aborts.
  * Planner exception or invalid output: `HOVER`, then `ABORT`.
  * Deadline exceeded: `ABORT`.

## 7. Implementation milestones (after contract approval)

| # | Milestone | Done when |
|---|---|---|
| M1 | Skeleton: `px4_gazebo_p4/`, `gwm_obstacle_avoidance` package, configs, `p4_manifest.py` (P4 sources + P3 baseline binding), guard test that P3 still verifies to `1e205fec` | offline tests pass; P3 `verify_current` unchanged |
| M2 | Perception and local map; tests on synthetic depth (closed-form boxes, P3 test pattern) and read-only replay of retained P3 raw depth | scan and grid match ground truth within `e_depth + e_grid`; unknown is never free |
| M3 | Planner and supervisor (pure Python); table-driven tests per state transition, determinism tests and S1–S7 decision fixtures on synthetic maps | expected decision for each fixture; identical outputs on reruns |
| M4 | `P4Mission` (reuses P2 initialization, reference and handover via imports) and `control_node.py`; offline mission tests with simulated `StateCache` inputs | every transition and abort path covered; only 176/400/21 issued |
| M5 | `p4_worlds.py`, scenario declarations, `p4_truth_observer.py` | generated worlds parse; geometry tests pass |
| M6 | `p4_runner.py` (including the S6 injector), `run_p4_gates.py`, `p4_evaluate.py`, `p4_trajectory_eval.py`; synthetic run directories exercise every pass and fail rule | evaluator rejects collisions, low clearance, stale planning, translation into `UNKNOWN`, competing publishers, P2 safety failures, §6.1 breaches, and S6 cases (missing or duplicate injection, loss before `t_inj`, native-source stop, late `HOVER`, motion while stale, injection outside S6) |
| M7 | `build_p4.sh` receipt and freeze tooling; full Linux and Windows regression | P2, P3, MC and P4 suites green; P3 still `1e205fec` |

Then operator steps: Policy A approval for the P4 candidate, freeze, and Gate A, B, S and C
authorizations, one at a time.

## 8. Gate approval boundaries

Each gate needs an explicit operator authorization bound to the P4 manifest SHA-256. Each consumes
its authorization once, stops at the first failure and never retries. P3 gate results grant no P4
credit. The gate contents are in contract §8; in short:

* **A** — ground: P3 readiness unchanged; P4 readiness under full P4 load against contract §6.1;
  P4 obstacle and fault fixtures.
* **B** — one avoidance smoke (S2).
* **S** — safety flights (S1, S5, S6), if approved.
* **C** — three consecutive qualification flights (S2, S3, S4).

## 9. Compatibility and regression strategy

* No file under the P3 manifest scope changes. A P4 test fails if any does, and the P4 runner refuses
  to start unless `p3_manifest.verify_current` returns `1e205fec…`.
* The P4 manifest records the P3 manifest SHA and all P3 inputs it imports, plus P4 sources, build
  receipt, scenario files and configs.
* Every P4 milestone reruns the existing suites (P2 219, sensor 63, evaluators 63, runtime 89,
  MC 23, Windows P3 set) unchanged, plus P4 suites.
* Retained P3 evidence is read-only input to replay tests and is never modified.

## 10. Decisions requiring operator approval

Listed with recommendations in contract §10:

1. exclusive P4 controller;
2. P4 envelope and mission geometry;
3. safety thresholds;
4. Gate S and the scenario-to-gate mapping;
5. unknown-space and free-cap policy;
6. Policy A reuse and the P4 landing-window definition;
7. the second depth-image subscriber;
8. S7 as replay-only;
9. the `px4_gazebo_p4/` location.

## 11. Implementation status (offline, M1–M7 complete)

All milestones are implemented in `simulation/px4_gazebo_p4/` and verified offline. No P4 runtime
has been launched, no candidate frozen and no gate run. The P3 candidate still verifies to
`1e205fec…`, and no file in its manifest scope changed.

**Implementation choices within the approved contract** (each covered by tests):

* **Near-clip blind zone (revision 3, P3 rule).** Free space is marked only from the camera's near
  clip outward. The cells left unknown around the takeoff point lie inside the traversed body disc
  (L2, revision 3).
* **Goal clipping (L1).** A candidate segment is clipped at the goal distance and never extends past
  the goal. The step that ends exactly at the goal counts as progress.
* **Ties (L3).** Clearance is quantized to the 0.10 m grid. Scores closer than the grid can resolve
  (0.3·0.1 + 0.6·0.05 = 0.06) are ties and go to the L3 tie-break (smaller |k|, then +y, then shorter
  L). Without this, rasterization noise flipped the detour side every tick.
* **Silent adapter (supervisor).** "Adapter health not fresh" includes a missing health stream: a
  record older than 2.0 s sim. That is the adapter's 1.0 s health period plus its 1.0 s P3 stall
  limit.
* **Ground scopes.** The ground-diagnostic controller finishes after its P2 prestream. The runner
  then holds the declared 15 s ground window while perception runs; the fault scope injects its
  stop after that window. Gate A fixture trigger decisions and the fault `HOVER` decision are
  checked by replaying the production planner and supervisor on the recorded ledgers. Stale-clock
  and calibration-mismatch faults are covered by offline tests, since they cannot be induced at
  runtime without changing accepted components.
* **World.** PX4 boots the generated scenario world because the run-local `rootfs/gz_env.sh`
  redirects `PX4_GZ_WORLDS` to the run's world directory. The base world must match the
  P3-pinned `default.sdf` digest.
* **Namespace owner.** `scripts/p4_namespace_owner.py` is a byte-identical copy of the P3 owner, so
  the unattended QGC ownership guard (which pins that file's hash) is unchanged. Its
  `from p3_runner import main` resolves to the P4 entry shim `scripts/p3_runner.py`. The accepted
  P3 runner is loaded from its own path.
* **Evidence channel.** P4 controller events use the existing `/gwm/p2/events` channel, so the
  unchanged P2 timing and yaw-ownership evaluators read them.
* **Landing window.** The native-command landing window runs from the first controller LAND
  (normal or ABORT) through `COMPLETE`, or through the final landed/disarmed observation after an
  ABORT.

**Offline evidence:**

* **Closed loop.** The independent renderer, production perception, supervisor and an
  ideal-tracking vehicle run S1–S5 as the contract requires:
  * S1 reaches the goal with no avoidance or hover;
  * S2 detours on +y and S3 on −y;
  * S4 reaches the goal;
  * S5 hovers and then aborts.

  Minimum truth clearance is at least 0.63 m. S6 hovers with yaw frozen and aborts after 10 s.
* **Mission and controller.** The P4 mission was run on P2 `StateCache` inputs:
  * `MAP_SCAN` is 360° counter-clockwise at ≤ 10°/s;
  * `TRANSIT` reaches `GOAL_HOVER`, which issues exactly one LAND;
  * a supervisor ABORT goes through P2 recovery;
  * geofence, offboard-loss and graph checks abort.
* **Evaluator.** Synthetic run directories exercise every §6, §6.1, §7 and §9.1 rule, in both pass
  and fail cases.

## 12. Frozen P4 candidate (2026-10-26)

* **Policy A** (`bf1ef9b1…`, content unchanged from P3 v1.6) was approved by Neura-Shadow for the P4
  candidate at `6c418336cf425317f5b9972e9f59776a443dde39`. The receipt is
  `state/p4-policy-A-approval-6c41833.json` (`1d63a8de…e67e`).
* **Preflight.**
  * P3 sensor receipt `c5d37413…` and P4 package receipt `3b6aef59…` both match their sources.
  * The DDS agent and QGC ELF are intact, with the owner bound.
  * The five WSL D3D12 libraries match their pins.
  * 871.1 GiB was free and no simulation processes were running.
* **Manifest.** `state/p4-candidate-6c41833.json`, canonical SHA-256
  **`2ddabd14a48a900d66ed2890cc48ecb367d6d3d860a4cb3b85cf0efe77f50a66`** (file `8d97eec5…29c1`).
  * It binds the P3 baseline `1e205fec…` (still verifying), 29 P4 and 103 P3 source files, the MC
    build `7542919c…` (the runtime selection matches), QGC ELF `82994283…`, `hardware:d3d12` with
    D3D12 profile `2e9e9ba1…`, and gates A/B/S/C with 3 qualification flights.
  * `verify` reproduces it. Execution without authorization is false, and no P4 authorization or
    gate artifact exists.
* **Known label defect.** `configs/p4_validation.yaml` `contract_revision` still reads "revision 2".
  The manifest copies that text, although the frozen sources implement revision 3 (`map_scan_deg`
  360 pinned and the traversed L2 rule, both hashed). Correcting the label changes a P4 source, so
  it needs a new commit, a Policy A approval for it, and a re-freeze.
* **Label corrected** in the next commit: `contract_revision` now names revisions 2 and 3, with their
  commits, and a test pins it. Candidate `2ddabd14` stays frozen until it is replaced by a re-freeze
  at that commit, which waits on the operator's Policy A approval.
* Gate A has not been run.

## 13. Re-frozen P4 candidate (supersedes `2ddabd14`)

* **Policy A** (`bf1ef9b1…`) was approved by Neura-Shadow for `8c1bd76a890325003bf7116d925a5498a721f747`
  (the revision-label correction). The receipt is `state/p4-policy-A-approval-8c1bd76.json`
  (`18fd93b7…e9bb`).
* **Preflight** was unchanged from §12: P3 sensor receipt `c5d37413…` and P4 package receipt
  `3b6aef59…` match their sources; DDS agent, QGC ELF (owner bound) and the five D3D12 libraries
  are intact; 871.1 GiB free; no simulation processes.
* **Manifest.** `state/p4-candidate-8c1bd76.json`, canonical SHA-256
  **`3785a54c7f10ae1ce3c6dfcaae95d3126cf9609718e23899b4f9df2b07e37ecf`** (file `32b8c254…f2ed`).
  * Contract `p4-acceptance-v1`, revisions 2 and 3, with the P3 baseline `1e205fec…` still
    verifying.
  * It binds 29 P4 and 103 P3 source files, P4 build `3b6aef59…`, MC build `7542919c…` (the runtime
    selection matches), QGC ELF `82994283…`, `hardware:d3d12` with profile `2e9e9ba1…`, and gates
    A/B/S/C with 3 qualification flights.
  * `verify` reproduces it, execution without authorization is false, and no P4 authorization or
    gate artifact exists.
* **Superseded.** `2ddabd14` no longer verifies, because its sources changed. Its receipt and
  manifest remain in `state/` as history. Gate A has not been run.

## 14. P4 Gate A of candidate `3785a54c` — FAILED at p4-readiness (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-3785a54c7f10.json` (`62021de1…e86f`), consumed.
  Result `state/p4-gate-A-3785a54c….result.json` (`4be2b2a5…21df`): `accepted: false`,
  `first_failure: p4-readiness:runtime_status:Abort:controller_failed`, no automatic retry. Run
  00:35:43–00:37:32 UTC; no leftover processes.
* **p3-readiness** `20261026T003602Z-p4-p3_readiness-c3c4362b`: **passed**. This was the unchanged P3
  readiness evaluation under the P4 runner. Every process was graceful and the QGC unattended
  teardown accepted the P4 namespace-owner copy.
* **p4-readiness** `20261026T003709Z-p4-p4_readiness-fa5e8ad3`: `p4_control` refused to start with
  `Missing explicit gates: GWM_ALLOW_SITL_COMMANDS`. The accepted P2 rule
  (`gwm_px4_control.contracts.gates(env, flight or ground_diagnostic)`) requires that gate for the
  ground-diagnostic mode, because it publishes setpoints. The P4 runner removed the variable from the
  controller environment for every non-flight scope. **This is a P4 runner defect**, not a
  perception, planner or acceptance-rule failure.
  * The other processes were graceful (QGC teardown accepted again).
  * The truth observer recorded 69 poses with a largest gap of 0.036 s.
  * Perception produced no snapshot before the abort, so it was not exercised.
* **Correction (applied offline, §15):** for ground-diagnostic scopes the runner should set
  `GWM_ALLOW_SITL_COMMANDS=1` in the controller environment only, as P2's ground diagnostics require.
  `vehicle_command` stays forbidden by P2's `transport_allowed` and by the graph check (0 expected
  publishers). The fix needs a new commit, a Policy A approval for it, a re-freeze and a new Gate A
  authorization. Candidate `3785a54c` cannot run Gate A again.

## 15. Runner correction after the failed Gate A (offline)

* `p4_runner.p4_controller_command` now sets `GWM_ALLOW_SITL_COMMANDS=1` in the `p4_control`
  environment for every scope (ground diagnostic and flight), and nowhere else. It no longer removes
  the variable. No package, config, threshold, contract or P2/P3 file changed. The P4 package hash is
  unchanged (`3b6aef59…`), so no rebuild is needed.
* Regression test `test_controller_environment_satisfies_the_p2_gate_rule`
  (`tests/test_px4_gazebo_p4_runtime.py`, Linux) passes the controller environment for the
  p4_readiness, fixture, fault and flight scopes through the accepted P2 `gates()` check.
* Regression after the fix:
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 102.
  * Windows: 367 passed, 26 skipped.
  * P3 still verifies to `1e205fec…`.
* **Next:** Policy A approval for this commit, re-freeze, then a new Gate A authorization. Nothing
  has been frozen and no gate has run.
