# v3-SIM P4 — deterministic non-learning obstacle avoidance: implementation plan

**Status (2026-10-26): P4 INCOMPLETE.** Contract approved (revisions 2 and 3). M1–M7 are
implemented and verified offline (§11). Candidate `3785a54c` failed Gate A on a runner defect and
cannot run again (§14). The runner fix is applied offline (§15). Candidate `1628fc44` (§16)
failed Gate A on two `p4_control` defects and cannot run again either (§17). Both are fixed
offline (§18). Candidate `b4db489e` (§19) failed Gate A at P3 readiness cleanup: Gazebo did
not exit on SIGINT within the 15 s grace (§20). The operator treated that as an intermittent
environment event. Candidate `4722a88e` re-froze the same sources (§21). Its Gate A passed P3
and P4 readiness, then failed at fixture G2 on a world-generator defect (§22), now fixed
offline (§23). Candidate `45031345` (§24) failed Gate A at p4-readiness on the perception
compute p99 limit (§25). Perception is now vectorized with identical outputs (§26). Candidate
`22557ad6` (§27) passed P3 readiness, P4 readiness and G2 at Gate A, then failed at G4 cleanup
on a truth-observer shutdown race (§28), now fixed offline (§29). Candidate `c932b212` (§30)
**passed Gate A** (§31). It failed Gate B on the 0.30 m tracking bound (§32), which is infeasible with
position-only setpoints at 0.3 m/s. Contract revision 4 (P4 transit speed 0.20 m/s) was approved and
applied offline (§33, §34). Candidate `e187b475` (§35) **passed Gate A** (§36), then failed Gate B in
`MAP_SCAN` on a one-off 0.48 s stall in the inherited P2 graph check (§37). The graph check is now traced
like P2's (§38). Candidate `1be1896a` (§39) **passed Gate A** (§40) and **Gate B** (§41). It failed Gate S
at S5: an expected supervisor abort lands through P2 `RECOVERY`, which the reused P2/P3 evidence checks
treat as a failed flight (§42). Contract revision 5 (abort lands through P2's nominal sequence to
`COMPLETE`) was approved and applied offline (§43, §44). Candidate `6cc87fb2` (§45) failed Gate A at P3
readiness on one extra QGC `REQUEST_MESSAGE` (35 against the Policy A cap of 34; §46). The operator treated
that as intermittent; candidate `c110bd30` re-freezes the same sources (§47) and **passed Gate A** (§48);
Gates B, S and C await separate authorizations. Gates A, B, S
and C must all pass. Sections 1–10 record the design; §11–§15 record events in date order and
keep the wording of their time. The normative rules (thresholds, scenarios, gates) are in the
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
* **Next (at the time):** Policy A approval, a freeze, then a new Gate A authorization (done in §16
  up to the authorization).

## 16. Frozen P4 candidate `1628fc44` (2026-10-26)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `601eceae76e371a921c6f652413e8a292b0d80d3`. That commit has the same P4 sources as fix commit
  `4804a47`; only docs changed. Receipt: `state/p4-policy-A-approval-601ecea.json` (`11c4cfa7…b82c`).
* Manifest `state/p4-candidate-601ecea.json`: canonical
  **`1628fc44afcffab730c7ad7d69c5047f010f52e64767b07e715f1b90dc122f07`** (file `4bbe9a1f…6d94`).
  `verify` reproduces it.
  * Baselines and build: P3 `1e205fec…`; P4 build `3b6aef59…`; 29 P4 and 103 P3 source files.
  * Runtime: MC build `7542919c…`, and its selection matches; QGC ELF `82994283…`;
    `hardware:d3d12` with profile `2e9e9ba1…`.
  * Gates A/B/S/C; no execution without authorization.
* No gate record exists for `1628fc44`. The failed Gate A of `3785a54c` and its consumed authorization
  stay in `state/` as history. Gate A has not been run.

## 17. P4 Gate A of candidate `1628fc44` — FAILED at p4-readiness (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-1628fc44afcf.json` (`bc6ce22e…1d88`), consumed.
  Result `state/p4-gate-A-1628fc44….result.json` (`234701ff…5862`): `accepted: false`,
  `first_failure: p4-readiness:p2_integrity_error:ValueError:Trace/persisted bag count mismatch:
  offboard_control_mode`, no automatic retry. Run 01:25:35–01:28:06 UTC; no leftover processes.
* **p3-readiness** `20261026T012605Z-p4-p3_readiness-21036f92`: **passed** again.
* **p4-readiness** `20261026T012706Z-p4-p4_readiness-43b0ab68`: the runtime passed. The runner fix
  worked: `p4_control` exited 0 with a valid graph and no flight commands possible. The independent
  evaluator then failed it.
* **Defect 1 (the failing check):** P2's accepted `timing_evidence.verify_timing` requires one
  `bag_write:<topic>` trace span for every persisted message of each topic and of `/gwm/p2/events`.
  P2's `node.bag()` records those spans. P4's `control_node.bag()` writes the bag without them, so the
  trace holds 0 `bag_write` records. No offline test ran `verify_timing` on a P4 controller trace.
* **Defect 2 (found in the same evidence; it would have failed later checks):** `p4_control`
  subscribes to `/gwm/p4/local_map` and to the adapter health topic with the default RELIABLE QoS.
  The perception node and the accepted P3 adapter publish BEST_EFFORT. Both publishers logged
  "incompatible QoS … RELIABILITY", and the controller received `local_map_messages: 0` and
  `adapter_health_messages: 0`. Perception itself worked: `p4/local_map.jsonl` is 1.35 MB.
* **Correction (applied offline, §18):** both are in the `gwm_obstacle_avoidance` package, so the fix
  needs a rebuild and gives a new P4 package hash.
  * Defect 1: add P2's `bag_write:` spans in `control_node.bag()`.
  * Defect 2: subscribe to the map and health topics with the BEST_EFFORT profile the publishers use.
    The accepted P3 adapter is not changed.
  * Offline tests: run `verify_timing` on a P4 controller trace, and check QoS compatibility.
  * Then a new commit, Policy A approval, a freeze and a new Gate A authorization. Candidate
    `1628fc44` cannot run Gate A again.

## 18. `p4_control` corrections after the failed Gate A of `1628fc44` (offline)

* **Trace spans:** `control_node.bag()` now records the accepted P2 node's `cdr_serialize:<topic>` and
  `bag_write:<topic>` spans for every persisted message, including `/gwm/p2/events`.
  * Trace budget: the P2 flights used the same span set. At the 240 s simulation deadline the
    estimate is about 290k records against the 500k capacity.
* **QoS:** `p4_control` subscribes to `/gwm/p4/local_map` and the adapter health topic as
  BEST_EFFORT, KEEP_LAST 10, VOLATILE, matching both publishers. The accepted P3 adapter is unchanged.
* **Tests:** new `tests/test_px4_gazebo_p4_ros_harness.py` runs the real `p4_control` under real
  rclpy and rosbag2, offline, with no PX4 or Gazebo. It is skipped without a ROS-sourced Linux shell.
  * An in-process BEST_EFFORT publisher feeds `/clock`, the PX4 output topics, the map and health.
  * One test checks that the `bag_write` count equals the persisted bag count, then runs the accepted
    P2 `verify_timing` on the trace and bag.
  * The other test checks that the map and health messages arrive.
  * Before the fix both tests failed, reproducing the Gate A evidence: 0 `bag_write` records against
    71 bag messages, and "incompatible QoS … RELIABILITY". After the fix both pass.
* **Rebuild:** new P4 package hash `3dd6106358aac0a920ba31a4a9de84f9950708233694db95ae175d3fe90e140c`;
  `state/p4-built.json` matches the source. The consumed candidate `1628fc44` binds the old build
  (`3b6aef59…`) and stays as history.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 102, ROS harness 2.
  * Windows: 367 passed, 27 skipped.
  * P3 still verifies to `1e205fec…`.
  * Dry P4 candidate: build `3dd61063…`, 29 P4 files.
* **Next (at the time):** Policy A approval, a freeze, then a new Gate A authorization (done in §19
  up to the authorization).

## 19. Frozen P4 candidate `b4db489e` (2026-10-26)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `05f4052c0eec28b8c8a3c85818af5b69df78760d`. Receipt: `state/p4-policy-A-approval-05f4052.json`
  (`8b772de3…486e`).
* Manifest `state/p4-candidate-05f4052.json`: canonical
  **`b4db489e820db723305e7e38e360a3f86f948f938c954b5e0416d6a4d016d8b2`** (file `eeb6a739…4b4c`).
  `verify` reproduces it.
  * Baselines and build: P3 `1e205fec…`; P4 build `3dd61063…`; 29 P4 and 103 P3 source files.
  * Runtime: MC build `7542919c…`, and its selection matches; QGC ELF `82994283…`;
    `hardware:d3d12` with profile `2e9e9ba1…`.
  * Gates A/B/S/C; no execution without authorization.
* No gate record exists for `b4db489e`. The failed Gate A runs of `3785a54c` and `1628fc44` and their
  consumed authorizations stay in `state/` as history. Gate A has not been run.

## 20. P4 Gate A of candidate `b4db489e` — FAILED at p3-readiness cleanup (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-b4db489e820d.json` (`c7b3b908…e827`), consumed.
  Result `state/p4-gate-A-b4db489e….result.json` (`273d356c…9b93`): `accepted: false`,
  `first_failure: p3-readiness:cleanup:gazebo:forced_kill_after_grace`, no automatic retry. Run
  01:49:22–01:50:41 UTC; no leftover processes. P4 readiness, the fixtures and the faults did not run.
* **p3-readiness** `20261026T014941Z-p4-p3_readiness-ea9a39a9`: runtime exit 0, P2 controller
  completed. The shutdown followed the P3 contract §7 order, with the same timings as the passing run
  `…012605Z…21036f92`, up to the DDS Agent (exit 0 at +4.4 s). The Gazebo world pause returned `true`.
  Gazebo then did not exit within 15 s of SIGINT; it was killed with -9 at +20.0 s. In the passing
  runs it exited 0 about 0.5 s after SIGINT.
* **Classification:** the evaluator applied the unchanged P3 rule (exit 0 within grace, no
  escalation) correctly.
  * The step runs only accepted P2/P3 processes (`p2_control`, the P3 bridge, adapter and probe); no
    P4 node runs in it.
  * The `p4_control` corrections of §18 are not involved, and no P4 code defect is identified.
  * Gazebo is adopted from PX4's launch, so its own output is not retained; the cause of the hang is
    unknown.
  * This is the first forced Gazebo stop seen in 15 runs: 12 accepted P3 runs and the three P4 P3
    readiness runs.
* **Status:** candidate `b4db489e` cannot run Gate A again. Whether to treat this as an intermittent
  environment event (new freeze and Gate A authorization on the same sources) or to investigate first
  is an operator decision.

## 21. Re-frozen P4 candidate `4722a88e` (2026-10-26)

* **Operator decision (Neura-Shadow):** the Gazebo forced stop of §20 is treated as an intermittent
  environment event, and the unchanged P4 sources are re-frozen. Policy A (`bf1ef9b1…b3b0`,
  unchanged) was approved for commit `40ad650b349685326685d280ab208cf64ccaffe5`, whose P4 sources are
  identical to `05f4052`. Receipt: `state/p4-policy-A-approval-40ad650.json` (`56ab4e4c…6961`).
* Manifest `state/p4-candidate-40ad650.json`: canonical
  **`4722a88ec20273e72894c2dbac4000122771d74db162491950f027bc6d9b3f50`** (file `78156605…7147`).
  `verify` reproduces it.
  * Same as `b4db489e`: P4 and P3 source files, P4 build `3dd61063…`, and runtime bindings. Only the
    approval receipt and the freeze time differ.
  * P3 baseline `1e205fec…`; MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution
    without authorization.
* No gate record exists for `4722a88e`. The failed Gate A runs of `3785a54c`, `1628fc44` and
  `b4db489e` and their consumed authorizations stay in `state/` as history. Gate A has not been run.

## 22. P4 Gate A of candidate `4722a88e` — FAILED at fixture G2 (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-4722a88ec202.json` (`fc1be449…ee85`), consumed.
  Result `state/p4-gate-A-4722a88e….result.json` (`ac75c3bf…4001`): `accepted: false`,
  `first_failure: fixture-G2:runtime_status:TimeoutError:PX4 prompt/wall deadline exceeded: boot`, no
  automatic retry. Run 01:58:52–02:03:10 UTC; no leftover processes. G4, G6 and the faults did not run.
* **p3-readiness** `20261026T015909Z-p4-p3_readiness-111a3347`: **passed**. Gazebo stopped gracefully.
* **p4-readiness** `20261026T020010Z-p4-p4_readiness-2c12286c`: **passed**. This is the first pass
  under the full P4 workload, and it confirms the §15 and §18 corrections.
* **fixture-G2** `20261026T020125Z-p4-G2-94602e38`: PX4 started Gazebo with the run-local G2 world.
  It then logged "Waiting for Gazebo world…" until "Timed out waiting for Gazebo world", and the
  runner's boot deadline expired.
  * The evaluator also reported `evaluator_incomplete_evidence:KeyError:'sensor_window_sim_s'`. That
    is a consequence of the aborted runtime.
* **Defect (P4 world generator):** `p4_worlds.generate` adds a world-level
  `<plugin filename="gz-sim-contact-system">`. The pinned PX4 `default.sdf` declares no plugins, so
  Gazebo loads its systems from PX4's `server.config`: Physics, UserCommands, SceneBroadcaster,
  Contact, Sensors and the rest. Once a world declares any plugin, Gazebo skips the server-config
  plugins. The G2 world therefore ran without SceneBroadcaster, so `/world/default/scene/info` never
  answered, and without physics or sensors.
  * Both worlds pass `gz sdf -k`.
  * `server.config` already loads the Contact system, so the added plugin is unnecessary.
  * The offline test pinned the wrong expectation: its minimal base world had no server config, and
    it asserted that the plugin was present.
  * Every generated scenario world (G2/G4/G6, S1–S6) has the same defect.
* **Correction (applied offline, §23):** generate worlds without any world-level plugin. Change the
  offline test to assert that the generated world declares no world plugin, and that PX4's
  `server.config` lists `gz-sim-contact-system`. `p4_worlds.py` is in `scripts/`, outside the
  package, so the package build `3dd61063` is unchanged. Then a new commit, Policy A approval, a freeze
  and a new Gate A authorization. Candidate `4722a88e` cannot run Gate A again.

## 23. World-generator correction after the failed Gate A of `4722a88e` (offline)

* `p4_worlds.generate` no longer adds a world-level plugin. It now refuses a base world that declares
  any plugin, because PX4's `server.config` must supply every system, the Contact system included.
* Tests in `tests/test_px4_gazebo_p4_scenarios.py`:
  * Every generated world declares no world plugin.
  * The generator refuses a base world that declares a plugin.
  * Against the real pinned PX4 tree (Linux workspace): `server.config` lists the contact, physics,
    scene-broadcaster, user-commands and sensors systems; the pinned `default.sdf` declares no
    plugin; and all nine generated worlds (G2/G4/G6, S1–S6) stay plugin-free.
* `p4_worlds.py` is outside the package: the build `3dd61063…` is unchanged, and its receipt still
  matches.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 104, ROS harness 2.
  * Windows: 368 passed, 28 skipped.
  * P3 still verifies to `1e205fec…`.
* **Next (at the time):** Policy A approval, a freeze, then a new Gate A authorization (done in §24
  up to the authorization).

## 24. Frozen P4 candidate `45031345` (2026-10-26)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `9604a04d7299cb0cf2cde20ca0e47f12e96a9e5b`. Receipt: `state/p4-policy-A-approval-9604a04.json`
  (`d555ed52…02e5`).
* Manifest `state/p4-candidate-9604a04.json`: canonical
  **`4503134502a51272d9841615f6412b135a44ad37cc1480644e2d4ba89b65ecc1`** (file `f1e5cdc4…7e39`).
  `verify` reproduces it.
  * Only `scripts/p4_worlds.py` differs from `4722a88e`, and the manifest binds the corrected file.
  * Baselines and build: P3 `1e205fec…`; P4 build `3dd61063…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `45031345`. The four failed Gate A runs (`3785a54c`, `1628fc44`,
  `b4db489e`, `4722a88e`) and their consumed authorizations stay in `state/` as history. Gate A has
  not been run.

## 25. P4 Gate A of candidate `45031345` — FAILED at p4-readiness performance (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-4503134502a5.json` (`b84fba28…1aee`), consumed.
  Result `state/p4-gate-A-45031345….result.json` (`ad807744…d509`): `accepted: false`,
  `first_failure: p4-readiness:performance:compute_p99`, no automatic retry. Run 02:18:54–02:21:29
  UTC; no leftover processes. The fixtures and faults did not run.
* **p3-readiness** `20261026T021916Z-p4-p3_readiness-8ceafda7`: **passed**.
* **p4-readiness** `20261026T022023Z-p4-p4_readiness-50f2bba5`: runtime passed and perception
  `passed`, with 244 of 249 snapshots usable. Contract §6.1 limits per frame of thread wall time:
  p99 ≤ 0.050 s and max ≤ 0.090 s.

  | p4-readiness run | p99 | max | Frames over 0.050 s |
  |---|---|---|---|
  | This run (`…50f2bba5`) | **0.0737 s** (fail) | 0.0764 s | 11 of 249 |
  | Passing run of `4722a88e` (`…020010Z…2c12286c`), same package `3dd61063` | 0.0428 s | 0.0500 s | 1 of 266 |

  * The median barely moved (0.034 s against 0.032 s); the tail is heavier.
  * Host contention was higher in this run: every process used 10–15% more CPU, and the RTF was
    0.64 against 0.69.
* **Assessment:** the package code is unchanged, so this is not a regression. The perception step
  has too little margin against the approved budget on this host: even the passing run was within
  6% of the p99 limit.
  * Offline profile (synthetic ground frames, same Python): `poll` p50 is about 0.03 s.
  * About 75% of the time is P4 code. The largest part is plain-Python ray marching
    (`local_map.mark_ray`/`index`, about 300k cell visits per 33 frames, roughly half the time).
    `extract_scan` is about a quarter.
  * The accepted P3 `observe`, about 22%, cannot be changed.
* **Options (operator decision):**
  * Optimize P4 perception without changing outputs: vectorize the ray marching and the scan
    extraction, with offline tests proving identical grids and scans. This needs a package rebuild,
    a new commit, Policy A approval, a freeze and Gate A authorization.
  * Re-freeze the same sources, accepting a low chance of passing given the measured margin.
  * The limit itself is a contract timing requirement and is not to be relaxed.
  * Candidate `45031345` cannot run Gate A again.

## 26. Perception compute optimization after the failed Gate A of `45031345` (offline)

* **Changes, outputs unchanged:**
  * `local_map.Grid.mark_rays` marks a batch of rays with NumPy. It uses the same per-ray step
    count (`math.hypot`), the same `t = n/steps` samples, the same float operations and the same end
    cell, and `mark_ray` delegates to it.
  * Grid states only rise (UNKNOWN < FREE < OCCUPIED, free never clears occupied), so batch marking
    equals sequential marking.
  * `perception.integrate` collects the rays and marks them in one batch.
  * `perception.extract_scan` computes the per-column reductions for all columns at once: hit,
    first-minimum index, band membership, too-close, NaN and the valid minimum. Azimuths still use
    `math.atan2/cos/sin` per column.
  * No configuration, threshold, contract value or P3 code changed.
* **Equivalence proof:** new `tests/test_px4_gazebo_p4_equivalence.py` keeps verbatim copies of the
  implementations frozen in `45031345` (commit `9604a04`). It checks that:
  * scans are equal float for float, and grids byte for byte, over every scenario and fixture layout
    plus a near-obstacle layout, on ground and at altitude, with NaN and too-close patches and all
    four ray kinds present;
  * a 500-ray batch equals sequential marking, including off-grid, zero-length and pre-occupied
    cases;
  * single-ray cases match.
  * A separate 140-frame scratch digest was identical before and after.
* **Rebuild:** new P4 package hash `ada6d608e40855886ea29636ab7df67eb21e3e152319b825c879208ab8d7502b`;
  the receipt matches the source.
* **Offline compute per processed frame** (`P4Processor.poll`, same Python, frozen reference against
  vectorized):

  | Scene | p50 before | p50 after | p99 before | p99 after | max before | max after |
  |---|---|---|---|---|---|---|
  | Ground, empty world | 0.0208 s | 0.0088 s | 0.0310 s | 0.0137 s | 0.0351 s | 0.0153 s |
  | 2 m, S4 obstacles | 0.0218 s | 0.0065 s | 0.0430 s | 0.0095 s | 0.0436 s | 0.0106 s |
  | 2 m, S5 wall | 0.0184 s | 0.0079 s | 0.0229 s | 0.0119 s | 0.0256 s | 0.0132 s |

  * The remaining time is mostly the accepted P3 `DepthProcessor.observe` (about 6 ms).
  * These offline numbers are not runtime evidence. Only a gate run measures the contract limit.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 104, equivalence 6, ROS
    harness 2.
  * Windows: 374 passed, 28 skipped.
  * P3 still verifies to `1e205fec…`.
* **Next (at the time):** Policy A approval, a freeze, then a new Gate A authorization (done in §27
  up to the authorization).

## 27. Frozen P4 candidate `22557ad6` (2026-10-26)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `0379b2275605759a968ffb5f514f258045d1539c`. Receipt: `state/p4-policy-A-approval-0379b22.json`
  (`c80ea928…0ac7`).
* Manifest `state/p4-candidate-0379b22.json`: canonical
  **`22557ad6f223081fba3fc7db08d522188f40dc9f48a61e09bab0cf753574cd6c`** (file `9cc9398a…d5ff`).
  `verify` reproduces it.
  * Only `local_map.py` and `perception.py` differ from `45031345`.
  * Baselines and build: P3 `1e205fec…`; P4 build `ada6d608…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `22557ad6`. The five failed Gate A runs (`3785a54c`, `1628fc44`,
  `b4db489e`, `4722a88e`, `45031345`) and their consumed authorizations stay in `state/` as history.
  Gate A has not been run.

## 28. P4 Gate A of candidate `22557ad6` — FAILED at fixture G4 cleanup (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-22557ad6f223.json` (`efb2887c…b36b`), consumed.
  Result `state/p4-gate-A-22557ad6….result.json` (`3dcc2c0e…d457`): `accepted: false`,
  `first_failure: fixture-G4:cleanup:p4-truth-observer:nonzero_exit_3`, no automatic retry. Run
  02:50:00–02:54:29 UTC; no leftover processes. G6 and the faults did not run.
* **Passed:**
  * **p3-readiness** `…025019Z…f4e263a5`.
  * **p4-readiness** `…025114Z…48f2aea8`: compute p99 0.0201 s and max 0.0222 s, against 0.0737 s
    in §25. The §26 optimization holds at runtime.
  * **fixture-G2** `…025221Z…a6a4f595`: trigger decision `obstacle_in_trigger_range` as expected;
    nearest hit 1.8 m against truth 1.8 m; compute p99 0.0299 s.
* **fixture-G4** `…025332Z…2e1e92c9`: the functional result was correct. The trigger decision was
  `obstacle_in_trigger_range` as expected, the nearest hit was 3.8 m against truth 3.8 m, and compute
  p99 was 0.0332 s. Cleanup then failed: the truth observer exited 3 within 0.02 s of SIGINT, while
  its `truth-closed.json` records `"error": null`.
* **Defect (P4 truth observer shutdown race):** on SIGINT, `p4_truth_observer.main` closes the
  ledgers and builds the close record while its gz-transport subscriptions are still active.
  * Gazebo stops after the observer and keeps publishing poses at 50 Hz.
  * A pose callback that arrives after `Ledger.close()` writes to a closed file. `fail()` then sets
    the error after the close record has been written, so the return code is 3.
  * G2 passed only because no callback landed in that window. Nothing was lost before the stop: G4
    has 1063 pose rows with a largest gap of 0.04 s.
* **Correction (applied offline, §29):** in `p4_truth_observer.py`:
  * On stop, mark the ledgers closed under their lock and unsubscribe, so that later callbacks are
    dropped and are not errors.
  * Derive the exit code from the error recorded in the close record.
  * Add an offline test that writes after close.

  `p4_truth_observer.py` is in `validation/`, outside the package, so build `ada6d608` is unchanged.
  Then a new commit, Policy A approval, a freeze and a new Gate A authorization. Candidate
  `22557ad6` cannot run Gate A again.

## 29. Truth-observer shutdown correction after the failed Gate A of `22557ad6` (offline)

* **`p4_truth_observer.py` changes:**
  * `Ledger.write` returns False and drops the row once the ledger is closed. `close` sets that flag
    under the same lock as the writes.
  * Callbacks return early once a stop is requested.
  * On stop, the observer unsubscribes every topic before closing; the result is recorded as
    `unsubscribed` in `truth-closed.json`.
  * The exit code is derived from the error written in the close record, so the two always agree.
  * The installed gz-transport13 Python `Node` provides `unsubscribe`.
  * The evaluator still requires `truth-closed.json` to have `error: null`, and every pose before the
    stop is still recorded.
* **Tests** in `tests/test_px4_gazebo_p4_scenarios.py`:
  * A ledger write after close is dropped, without an exception.
  * With fake gz modules, a pump thread keeps delivering pose and contact callbacks through SIGINT and
    after the close. The observer then exits 0 with `error: null`, the ledger row count equals the
    recorded count, and both topics are unsubscribed. This test is Linux-only.
  * Against the observer frozen in `22557ad6`, the second test reproduces the Gate A symptom (exit 3)
    and the first fails. Both pass with the fix.
* `p4_truth_observer.py` is outside the package, so build `ada6d608…` is unchanged and its receipt
  still matches.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 106, equivalence 6, ROS
    harness 2.
  * Windows: 375 passed, 29 skipped.
  * P3 still verifies to `1e205fec…`.
* **Next (at the time):** Policy A approval, a freeze, then a new Gate A authorization (done in §30
  up to the authorization).

## 30. Frozen P4 candidate `c932b212` (2026-10-26)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `a3401018254dc6bbb6faf23b61b4489fbb4c6eec`. Receipt: `state/p4-policy-A-approval-a340101.json`
  (`eb687405…0b82`).
* Manifest `state/p4-candidate-a340101.json`: canonical
  **`c932b2125ebb2f11754688b5a44a337be70f4f2b0993e77dbb963ab6e7ee2f9c`** (file `0fb1e426…5047`).
  `verify` reproduces it.
  * Only `validation/p4_truth_observer.py` differs from `22557ad6`, and the manifest binds the
    corrected file.
  * Baselines and build: P3 `1e205fec…`; P4 build `ada6d608…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `c932b212`. The six failed Gate A runs (`3785a54c`, `1628fc44`,
  `b4db489e`, `4722a88e`, `45031345`, `22557ad6`) and their consumed authorizations stay in `state/`
  as history. Gate A has not been run.

## 31. P4 Gate A of candidate `c932b212` — ACCEPTED (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-c932b2125ebb.json` (`dd112d64…4de7`), consumed.
  Result `state/p4-gate-A-c932b212….result.json` (`fdb66af7…0671`): `accepted: true`,
  `first_failure: null`, no automatic retry. Run 03:19:52–03:28:34 UTC; no leftover processes.
* **Every run passed with acceptance credit, and every owned process stopped gracefully in every run.**

  | Run | Result | Compute p99 / max (s) | RTF |
  |---|---|---|---|
  | p3-readiness `…032009Z…1bd9aaac` | passed (unchanged P3 evaluation) | — | 0.78 |
  | p4-readiness `…032108Z…cd131e23` | passed | 0.0283 / 0.0328 | 0.75 |
  | fixture-G2 `…032217Z…fe53a6ec` | `obstacle_in_trigger_range` (expected); hit 1.8 m, truth 1.8 m | 0.0285 / 0.0292 | — |
  | fixture-G4 `…032330Z…805ed9c9` | `obstacle_in_trigger_range` (expected); hit 3.7999 m, truth 3.8 m | 0.0341 / 0.0421 | — |
  | fixture-G6 `…032451Z…c20129ec` | `no_obstacle_in_trigger_range` (expected); hit 5.8501 m, truth 5.85 m | 0.0299 / 0.0318 | — |
  | fault-adapter_stop `…032605Z…6b76b2ac` | adapter SIGINT at t = 30.96 s; supervisor HOVER (`adapter_health_missing`) at 32.56 s | 0.0456 / 0.0465 | — |
  | fault-bridge_stop `…032725Z…abea7443` | bridge SIGINT at t = 30.992 s; HOVER (`snapshot_stale`) at 31.242 s | 0.0227 / 0.0236 | — |

* **Margin note:** the adapter_stop run's compute p99 of 0.0456 s is within the 0.050 s limit, but
  it has the least margin of any run. Gate B, S and C flights run under heavier load.
* **Status:** Gate A is accepted for `c932b212`. Gate B (S2), then Gate S (S1, S5, S6), then Gate C
  (S2, S3, S4) each need their own operator authorization bound to this manifest. P4 is not complete.

## 32. P4 Gate B of candidate `c932b212` — FAILED on tracking (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-B-c932b2125ebb.json` (`93bf622c…1ace`), consumed.
  Result `state/p4-gate-B-c932b212….result.json` (`eb4fc96b…f3d`): `accepted: false`,
  `first_failure: smoke:safety:tracking`, no automatic retry. Run 03:30:56–03:33:57 UTC; no leftover
  processes.
* **S2 flight** `20261026T033114Z-p4-flight-S2-8215374e`: functionally correct.
  * The mission completed: NORMAL → AVOIDING → RESUME → NORMAL, then the goal, with a final goal
    distance of 0.069 m.
  * 0 collisions; minimum truth clearance 0.457 m (limit 0.20 m).
  * Perception compute p99 0.019 s.
  * The only failure is horizontal tracking: maximum 0.342 m against the 0.30 m bound, and 382 of
    620 TRANSIT decisions were above 0.30 m.
* **Cause, physical and not a code fault:** P4 translates the target at P2's `ramp_m_s` of 0.3 m/s
  and sends position-only `trajectory_setpoint`s (the P2 helper sets velocity and acceleration to
  NaN). PX4's horizontal position loop is proportional, with stock `MPC_XY_P` 0.95 (x500 airframe,
  not overridden). A position-only target moving at constant speed `v` therefore leaves a steady lag
  of `v / MPC_XY_P` = 0.3 / 0.95 ≈ **0.316 m**.
  * The flight shows exactly that: the error plateaus at 0.31–0.33 m in every straight segment, peaks
    at 0.342 m in turns, and falls to about 0.03 m when the target holds.
  * The contract's 0.30 m bound is therefore unreachable at 0.3 m/s under the pinned P2 setpoint form
    and PX4 gains.
  * The offline closed loop assumed perfect tracking, so it could not show this.
* **Options (operator decision; nothing applied):**
  * **(a) Recommended. Contract revision 4:** a P4-only transit target speed of 0.20 m/s, as a new
    pinned `p4_avoidance.yaml` value. L7 already allows any speed ≤ 0.3 m/s, P2's `ramp_m_s` stays
    0.3 for takeoff and landing, and every safety threshold is unchanged.
    * Expected steady lag 0.21 m, plus the observed turn overshoot of about 0.03 m, giving about
      0.24 m.
    * Transit stays far inside the 150 s deadline.
  * **(b)** Velocity feedforward in `trajectory_setpoint`. This changes the setpoint wire form that
    P2's accepted evaluators check, so it is riskier.
  * Relaxing the 0.30 m bound or changing PX4 gains is not proposed.

## 33. Contract revision 4 proposed (transit speed 0.20 m/s; awaiting approval)

* [Revision 4 proposal](v3_sim_p4_contract_revision4_proposal.md), drafted at the operator's request
  after §32. A new pinned `mission.transit_speed_m_s` = 0.20 makes the P4 `TRANSIT` target speed
  slower than P2's 0.3 m/s ramp, which stays for every P2 phase. No threshold, scenario, gate or
  P2/P3 file changes.
* **Rationale:** with position-only setpoints, PX4's steady horizontal lag is `v / MPC_XY_P`. At
  0.3 m/s that is 0.316 m, measured in Gate B. At 0.20 m/s it is 0.211 m, about 0.23 m with the
  measured turn overshoot, inside the unchanged 0.30 m bound.
* **Offline study:** production perception and supervisor with a first-order 0.95 position-loop
  model. It reproduces the SITL plateau at 0.3 m/s (0.308 m). At 0.20 m/s, S1–S5 give the same
  outcomes and supervisor transitions, max tracking 0.205 m, equal or larger clearances, and
  `TRANSIT` of at most 55 s against the 150 s deadline.
* **Approved (2026-10-26) and applied in §34.** On approval: the config and pin, `TRANSIT` using the pinned speed in
  `mission.py`, tests, a package rebuild, then a new commit, Policy A approval, a freeze and the gates
  from Gate A onward.

## 34. Contract revision 4 applied (offline)

* **Approval:** operator Neura-Shadow, exactly as written in `65b99e4`. The contract now names
  revisions 2, 3 and 4 in its header, §3, §5 L7 and the §6 table, and the proposal is marked
  approved and applied. `p4_validation.yaml` `contract_revision` names revisions 2, 3 and 4
  (`d3c06b4`, `ea2c9e3`, `65b99e4`).
* **Code:**
  * `p4_avoidance.yaml` has `mission.transit_speed_m_s` 0.2, pinned in `contracts.PINNED`.
  * `mission.py` `TRANSIT` passes the pinned speed to `supervisor.apply_action`. P2 phases keep
    `ramp_m_s` 0.3 from the unchanged `p2_control.yaml`.
* **Tests:**
  * The pin is checked: the repository value is 0.2, and 0.3 is refused.
  * The mission advances the target exactly 0.2 × 0.05 m per 20 Hz `TRANSIT` tick, while P2's ramp
    stays 0.3.
  * The synthetic closed loop takes the pinned speed and an optional first-order PX4 position loop
    (`MPC_XY_P` 0.95). The S2 regression gives max tracking ≤ 0.2 / 0.95 + 0.01 m at the pinned speed,
    above 0.30 m at 0.3 m/s (the Gate B failure), with an unchanged detour and clearance ≥ 0.20 m.
  * `test_s1` allows 50 s for the 8 m transit.
  * The S6 sensor-loss test now uses the exact 10 Hz frame grid. Perception stops between frames,
    and `HOVER` must follow within the snapshot age plus one control tick. The old approximate bound
    failed only because 0.20 m/s put the cutoff exactly on a frame boundary; the behaviour (HOVER
    0.30 s after the last frame, ABORT after 10 s) is unchanged.
* **Rebuild:** new P4 package hash `3adf426bed0d1a57fdfc1fdf97751ce7efe5a21f55654c3f1f46613e4ad7a96f`;
  the receipt matches the source.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 108, equivalence 6, ROS
    harness 2.
  * Windows: full run 376 passed, 29 skipped, with the S6 test failing before its fix; the planner
    file then passed 26 of 26.
  * P3 still verifies to `1e205fec…`.
  * Dry P4 candidate: build `3adf426b…`.
* **Next (at the time):** Policy A approval, a freeze, then the gates from Gate A onward (done in
  §35 up to the Gate A authorization).

## 35. Frozen P4 candidate `e187b475` (2026-10-26, contract revisions 2, 3 and 4)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `add6c225174eb57326cb0dcdd0d3ac8ef688406f`. Receipt: `state/p4-policy-A-approval-add6c22.json`
  (`500413bf…2fa3`), with candidate phase "p4-acceptance-v1 (revisions 2, 3 and 4)".
* Manifest `state/p4-candidate-add6c22.json`: canonical
  **`e187b475eb5f00b6759f13fb1c87a52b6df831b953d13123c0be897aadb7df86`** (file `cfbaaf1b…7c28`).
  `verify` reproduces it.
  * Revision label: "revisions 2, 3 and 4 (… revision 4 commit 65b99e4)".
  * Against `c932b212`, exactly the four revision 4 files differ: `p4_avoidance.yaml` (binds
    `transit_speed_m_s` 0.2), `p4_validation.yaml`, `contracts.py` and `mission.py`.
  * Baselines and build: P3 `1e205fec…`; P4 build `3adf426b…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `e187b475`. Earlier candidates' records stay in `state/` as history,
  including `c932b212`'s accepted Gate A and failed Gate B. A new manifest needs its own Gate A, then
  B, S and C. Gate A has not been run.

## 36. P4 Gate A of candidate `e187b475` — ACCEPTED (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-e187b475eb5f.json` (`494259ea…6485`), consumed.
  Result `state/p4-gate-A-e187b475….result.json` (`2ab4ffec…815e`): `accepted: true`,
  `first_failure: null`, no automatic retry. Run 04:44:00–04:52:59 UTC; no leftover processes.
* **Every run passed with acceptance credit, and every owned process stopped gracefully in every run.**

  | Run | Result | Compute p99 / max (s) | RTF |
  |---|---|---|---|
  | p3-readiness `…044420Z…4bb7a5cc` | passed (unchanged P3 evaluation) | — | 0.71 |
  | p4-readiness `…044522Z…bc63ce22` | passed | 0.0340 / 0.0353 | 0.66 |
  | fixture-G2 `…044638Z…47d04268` | `obstacle_in_trigger_range` (expected); hit 1.8 m, truth 1.8 m | 0.0291 / 0.0349 | 0.68 |
  | fixture-G4 `…044755Z…e117b7cb` | `obstacle_in_trigger_range` (expected); hit 3.8 m, truth 3.8 m | 0.0238 / 0.0272 | 0.68 |
  | fixture-G6 `…044911Z…59e7346a` | `no_obstacle_in_trigger_range` (expected); hit 5.85 m, truth 5.85 m | 0.0341 / 0.0354 | 0.68 |
  | fault-adapter_stop `…045027Z…f7d906b2` | adapter SIGINT at t = 30.892 s; HOVER (`adapter_health_missing`) at 32.342 s | 0.0271 / 0.0283 | 0.72 |
  | fault-bridge_stop `…045148Z…c18bd0c5` | bridge SIGINT at t = 31.004 s; HOVER (`snapshot_stale`) at 31.254 s | 0.0302 / 0.0324 | 0.71 |

* **Margins:** the highest compute p99 is 0.0341 s against the 0.050 s limit. The fault-run p99 is
  lower than on `c932b212` (0.0271 against 0.0456 s).
* **Status:** Gate A is accepted for `e187b475`. Gate B (S2, the first flight at the revision 4
  transit speed), then Gate S (S1, S5, S6), then Gate C (S2, S3, S4) each need their own operator
  authorization bound to this manifest. P4 is not complete.

## 37. P4 Gate B of candidate `e187b475` — FAILED on a dispatch-budget stall in `MAP_SCAN` (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-B-e187b475eb5f.json` (`a449404a…5280`), consumed.
  Result `state/p4-gate-B-e187b475….result.json` (`9b8081b7…104d`): `accepted: false`,
  `first_failure: smoke:runtime_status:Abort:controller_failed`, no automatic retry. Run
  04:54:41–04:56:31 UTC; no leftover processes.
* **S2 flight** `20261026T045500Z-p4-flight-S2-def51241`:
  * P2 initialization, takeoff, reference lock, yaw handover and `INITIAL_HOVER` passed.
  * In `MAP_SCAN`, about 209° into the revolution at t = 59.496 s, `p4_control` raised the accepted
    P2 `DispatchGuard` error `stale_action`: the control callback exceeded the 0.05 s wall-time
    dispatch budget.
  * P2 recovery sent LAND at once (command 21). The vehicle reported landed and at rest from 64.7 s.
  * The truth observer closed cleanly (3171 poses, largest gap 0.04 s); only `ros-controller` exited
    non-zero.
  * `TRANSIT` was never reached, so the revision 4 speed was not exercised.
  * The evaluator's `KeyError:'sensor_window_sim_s'` is a consequence of the abort.
* **Where the time went:** the failing callback lasted 0.4839 s, but its `mission_tick` took
  0.0006 s and started 0.4823 s after the callback entry.
  * Before the mission tick, `tick()` does only trivial checks and the once-per-second
    `inspect_graph()`: ROS graph queries for the PX4 topics, then writing `ros-graph.json`.
  * The callbacks that carry that check recur every 1.05 s with a 1–3 ms pre-tick delay. The failing
    callback falls exactly on that cadence, with a 0.4823 s pre-tick delay.
  * Every `MAP_SCAN` callback before it took at most 0.0058 s.
  * The separate perception process kept publishing every 0.1 s with normal compute through the
    stall, so this was not a host-wide freeze.
* **Assessment:**
  * The structure is inherited unchanged from the accepted P2 node, which runs the same graph check
    inside the same budget window (`node.py` lines 248–251). The stall is not related to revision 4,
    because `MAP_SCAN` is speed-independent, and it is not a P4 logic defect.
  * It is the first such stall seen in any P4 run; `c932b212`'s whole S2 flight had a maximum
    callback of 0.0233 s.
  * Unlike P2, `p4_control` does not trace `graph_check` and `graph_json_persistence`, so whether
    the graph query or the JSON write blocked cannot be told from this evidence.
* **Options (operator chose (b), applied in §38):**
  * **(a)** Treat it as an intermittent environment event and re-freeze the unchanged sources.
  * **(b) Recommended:** add P2's `graph_check` and `graph_json_persistence` spans to `p4_control`.
    This is parity with the accepted P2 instrumentation, with no behaviour change, so a recurrence
    can be attributed. Then rebuild and re-freeze.
  * Either way, candidate `e187b475` cannot run Gate B again, and a new manifest needs Gate A first.

## 38. Graph-check tracing after the failed Gate B of `e187b475` (offline)

* **Change in `p4_control`, the same as the accepted P2 node (`node.py` lines 186 and 249):**
  * The once-per-second `inspect_graph()` call in the control callback is wrapped in a
    `graph_check` span.
  * Its `ros-graph.json` write is wrapped in a `graph_json_persistence` span.
  * There is no behaviour change: the cadence, graph rules and file are the same, and the call is
    still inside the same dispatch-budget window.
  * P2's `verify_timing` now reports `graph_check_duration_wall_s` for P4 runs as it does for P2. A
    recurrence can be attributed to the graph query or to the JSON write.
* **Test:** new `test_p4_controller_traces_the_graph_check_like_the_accepted_p2_node` in
  `tests/test_px4_gazebo_p4_ros_harness.py` runs the real node under real rclpy for about 2.6 s. It
  checks that:
  * there are at least 2 `graph_check` spans, each containing exactly one `graph_json_persistence`;
  * each lies inside one control callback, before that callback's `mission_tick`;
  * the spans are at least 1.0 s apart;
  * P2's `verify_timing` reports their maximum duration.

  Against the `e187b475` controller the test fails (0 spans); with the change it passes.
* **Rebuild:** new P4 package hash `07a6c74d6cb03393b4930d802b4f2d5ab626bad624dd57b47b664a02dac18ed6`;
  the receipt matches the source.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 108, equivalence 6, ROS
    harness 3.
  * Windows: 377 passed, 29 skipped.
  * P3 still verifies to `1e205fec…`.
* **Limit:** tracing makes a stall attributable. It does not prevent one, because the check sits in
  the inherited P2 structure.
* **Next (at the time):** Policy A approval, a freeze, then the gates from Gate A onward (done in
  §39 up to the Gate A authorization).

## 39. Frozen P4 candidate `1be1896a` (2026-10-26, contract revisions 2, 3 and 4)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `e967e4ce414bc77a11de86bf20944683a369bf7c`. Receipt: `state/p4-policy-A-approval-e967e4c.json`
  (`d0a181d3…99c9`), with candidate phase "p4-acceptance-v1 (revisions 2, 3 and 4)".
* Manifest `state/p4-candidate-e967e4c.json`: canonical
  **`1be1896a794200acfd77733b7b788001e4296a66ef00e63a84cf5c158de5137b`** (file `22bb6d4f…277d`).
  `verify` reproduces it.
  * Only `control_node.py` differs from `e187b475`, and the manifest binds the traced version.
  * Baselines and build: P3 `1e205fec…`; P4 build `07a6c74d…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `1be1896a`. Earlier candidates' records stay in `state/` as history,
  including `e187b475`'s accepted Gate A and failed Gate B. Gate A has not been run.

## 40. P4 Gate A of candidate `1be1896a` — ACCEPTED (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-A-1be1896a7942.json` (`cc46bfc0…99ea`), consumed.
  Result `state/p4-gate-A-1be1896a….result.json` (`c73fcce0…046b`): `accepted: true`,
  `first_failure: null`, no automatic retry. Run 05:12:55–05:21:53 UTC; no leftover processes.
* **Every run passed with acceptance credit, and every owned process stopped gracefully in every run.**

  | Run | Result | Compute p99 / max (s) | RTF | `graph_check` max (s) |
  |---|---|---|---|---|
  | p3-readiness `…051315Z…92e596e9` | passed (unchanged P3/P2 run) | — | 0.70 | 0.0028 |
  | p4-readiness `…051420Z…2764e0ad` | passed | 0.0328 / 0.0334 | 0.66 | 0.0049 |
  | fixture-G2 `…051535Z…b79a206c` | `obstacle_in_trigger_range` (expected); hit 1.8 m, truth 1.8 m | 0.0277 / 0.0348 | 0.67 | 0.0052 |
  | fixture-G4 `…051653Z…3614b966` | `obstacle_in_trigger_range` (expected); hit 3.7999 m, truth 3.8 m | 0.0370 / **0.0765** | 0.69 | 0.0045 |
  | fixture-G6 `…051808Z…10292134` | `no_obstacle_in_trigger_range` (expected); hit 5.85 m, truth 5.85 m | 0.0301 / 0.0341 | 0.72 | 0.0036 |
  | fault-adapter_stop `…051921Z…84ceb27c` | adapter SIGINT at t = 31.024 s; HOVER (`adapter_health_missing`) at 32.424 s | 0.0288 / 0.0391 | 0.70 | 0.0023 |
  | fault-bridge_stop `…052043Z…0f49f6d2` | bridge SIGINT at t = 31.024 s; HOVER (`snapshot_stale`) at 31.274 s | 0.0307 / 0.0330 | 0.70 | 0.0045 |

* **New tracing at work:** about 16 graph checks per run, with a p50 of about 1.4 ms, a maximum of
  5.2 ms and JSON writes of at most 2.0 ms. All are far inside the 0.05 s dispatch budget, and the
  stall of §37 did not recur.
* **Margin note:** G4's compute max of 0.0765 s is within the 0.090 s limit, but it is the closest
  value to any limit in this gate; its p99 is 0.037 s.
* **Status:** Gate A is accepted for `1be1896a`. Gate B (S2 at the revision 4 transit speed), then
  Gate S (S1, S5, S6), then Gate C (S2, S3, S4) each need their own operator authorization bound to
  this manifest. P4 is not complete.

## 41. P4 Gate B of candidate `1be1896a` — ACCEPTED (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-B-1be1896a7942.json` (`28619638…be88`), consumed.
  Result `state/p4-gate-B-1be1896a….result.json` (`d46fd9ca…5dcb`): `accepted: true`,
  `first_failure: null`, no automatic retry. Run 05:23:32–05:26:57 UTC; no leftover processes.
* **S2 flight** `20261026T052351Z-p4-flight-S2-80f2bc40`: passed with acceptance credit; every owned
  process stopped gracefully.
  * **Outcome:** `goal_reached_with_detour` as expected. Supervisor NORMAL → OBSTACLE_DETECTED →
    AVOIDING → RESUME → NORMAL; the controller completed through landing (`COMPLETE`); final goal
    distance 0.041 m.
  * **Safety:** 0 collisions; minimum truth clearance 0.403 m (≥ 0.20 m). **Tracking maximum
    0.246 m (≤ 0.30 m)**, against 0.342 m at 0.3 m/s on `c932b212`. The revision 4 prediction was
    about 0.23 m. Truth interval at most 0.04 s.
  * **Performance:** perception compute p99 0.0211 s and max 0.0274 s; latency p95 0.052 s sim;
    10.0 Hz; usable fraction 1.0; RTF 0.99.
  * **Transit:** 44.9 s sim, against the 150 s deadline; the revision 4 study predicted 45.4 s.
  * **Timing:** 120 graph checks, each at most 3.7 ms; slowest control callback 17.4 ms (budget
    0.05 s).
* **Status:** Gates A and B are accepted for `1be1896a`. Gate S (S1, S5, S6), then Gate C (S2, S3,
  S4) each need their own operator authorization bound to this manifest. P4 is not complete.

## 42. P4 Gate S of candidate `1be1896a` — FAILED at S5 (2026-10-26)

* **Authorization:** Neura-Shadow, `state/p4-auth-S-1be1896a7942.json` (`b744ebd0…5cd7`), consumed.
  Result `state/p4-gate-S-1be1896a….result.json` (`3c19e0e1…4005`): `accepted: false`,
  `first_failure: safety-S5:performance:compute_max`, no automatic retry. Run 05:28:56–05:35:01 UTC;
  no leftover processes. S6 did not run.
* **safety-S1** `20261026T052917Z-p4-flight-S1-4735309e`: **passed**. `goal_reached`, final goal
  distance 0.031 m, maximum lateral 0.064 m, tracking 0.256 m, 0 collisions, compute p99 0.030 s,
  RTF 1.00.
* **safety-S5** `20261026T053224Z-p4-flight-S5-e89ee6ef`: **the behaviour was correct.**
  * Supervisor: OBSTACLE_DETECTED and AVOIDING at 81.53 s, HOVER at 90.07 s (`no_progress_candidate`),
    ABORT at 110.12 s (`hover_timeout`). The target moved 0.0 m after HOVER.
  * Minimum truth clearance 0.891 m, 0 collisions, tracking 0.234 m. Commands `[176, 400, 21]`,
    with the single LAND issued through P2 recovery; the vehicle landed.
  * The evaluator failed it on five items.
* **Root cause of four of the five failures:** contract §5 routes `ABORT` into P2 `RECOVERY` ("one
  LAND (21) through P2 recovery rules"). The accepted P2/P3 evidence tools treat `RECOVERY` as a
  failed flight, and the P4 evaluator reuses them unchanged:
  1. **`p2:timing`.** P2 `trace_metrics` sets its status to `failed` whenever the controller result
     has any `failure`, and a supervisor abort records one (`p4_supervisor_abort:hover_timeout`).
     Its full flight timing requirements (prestream, gaps, late dispatch, stale source) are skipped
     in that case, so they were never applied to S5.
  2. **`p2_integrity_error: estimator_reference_reset:final_alignment_lost`.** The P2 reference
     monitor, replayed by `reference_evidence.classify`, rejects any loss of in-air alignment
     outside `REQUEST_LAND`, `VERIFY_LANDING`, `VERIFY_LANDED_AND_DISARMED` and `COMPLETE`. A
     touchdown in `RECOVERY` always trips it.
  3. **`rgb_absence:snapshot_missing:window_end`.** The P3 runner's RGB-absence snapshotter takes
     its `window_end` snapshot only on the `COMPLETE` transition (`p3_runner.py`, `TRIGGERS`), and a
     run ending in `RECOVERY` never reaches it.
  4. **`mc_causal_yaw:p2_integrity_missing`.** This follows from item 2.

  S6 ends the same way (`ABORT` → `RECOVERY`), so as specified **neither S5 nor S6 can pass**
  §8(a)/(b) and §9.1 F4, whatever the behaviour. The offline evaluator tests did not replay P2
  integrity on a real `RECOVERY` landing.
* **Independent fifth failure, `performance:compute_max`:** one perception frame took 0.0948 s
  against the 0.090 s maximum. It was at takeoff (acquisition 19.31 s); the frame before took
  0.0405 s, and only 1 of 974 frames exceeded 0.05 s. The p99 was 0.0266 s. This is a transient
  stall, not sustained cost; the cause (host contention or a Python pause) cannot be told from this
  evidence. The earlier maximum was G4's 0.0765 s (§40).
* **Options (operator decision; nothing applied):**
  * **Contract revision 5 (recommended):** a P4 supervisor `ABORT` lands through P2's nominal
    sequence (`REQUEST_LAND` → `VERIFY_LANDING` → `VERIFY_LANDED_AND_DISARMED` → `COMPLETE`), with
    the same single LAND (21) and P2 landing checks. The abort is recorded as the P4 outcome, not
    as a P2 failure. P2 safety errors still go to `RECOVERY` and fail. The unchanged P2/P3 checks
    (timing, reference, yaw, RGB absence, MC) then apply in full to S5 and S6.
  * **The compute-max spike:** investigate offline (for example Python GC pauses in the perception
    process), or accept it as host variance.
  * Candidate `1be1896a` cannot run Gate S again. Its accepted Gates A and B stay as history; a new
    manifest needs Gates A, B, S and C.

## 43. Contract revision 5 proposed (abort landing; compute spike investigated; awaiting approval)

* [Revision 5 proposal](v3_sim_p4_contract_revision5_proposal.md), drafted at the operator's request
  after §42.
  * A designed supervisor `ABORT` lands through P2's nominal sequence `REQUEST_LAND` →
    `VERIFY_LANDING` → `VERIFY_LANDED_AND_DISARMED` → `COMPLETE`, with exactly one LAND (21).
  * The abort is recorded as the P4 outcome `supervisor_abort`, not as a P2 failure.
  * The 45 s wall LAND-to-landed bound stays as an explicit evaluator check.
  * P2 safety errors and geofence breaches still enter `RECOVERY` and fail.
  * Routing never makes a run pass, because each §7 outcome check still requires its own expected
    outcome and abort reason.
  * The unchanged P2/P3 evidence checks (timing with full flight requirements, reference, yaw, RGB
    absence at `COMPLETE`, MC causality, MAVLink landing window) then apply in full to S5 and S6.
* **Compute-max spike (0.0948 s at S5 takeoff), investigated offline; no contract change:**
  * Not P4 compute: the offline maximum is 0.019 s, and thread CPU equals wall time.
  * Not Python GC: a full collection takes 0.008 s at about 62,000 objects; a 120 s simulated flight
    had no generation-2 collection and only 0.1 ms young-generation pauses; `gc.freeze()` makes no
    difference.
  * Not page faults (0–4 per poll), and not GIL contention (single-threaded executor).
  * `p4_control` was unaffected at that instant (5.4 ms maximum), and the VM was not saturated
    (24 CPUs, 4–5 used).
  * Assessment: host-level preemption of the perception thread. The §6.1 limit stays.
  * Optional diagnostic: record per-frame thread-CPU time and involuntary context switches.
* **Approved (2026-10-30) and applied in §44.** On approval: the mission, controller-outcome and evaluator changes and tests, a
  rebuild, then a new commit, Policy A approval, a freeze and Gates A–C.

## 44. Contract revision 5 applied (offline)

* **Approval:** operator Neura-Shadow, as written in `ae0e037`, including the optional perception
  diagnostic.
  * The contract applies it in its header, the §4 mission note, the split §5 rows plus the abort
    rule, and §9.1 F3.
  * The proposal is marked approved and applied.
  * `p4_validation.yaml` `contract_revision` names revisions 2–5 (`d3c06b4`, `ea2c9e3`, `65b99e4`,
    `ae0e037`).
* **Code:**
  * **`mission.py`:** a `P4Abort` from `MAP_SCAN` or `TRANSIT` is caught inside the mission. It
    records `p4_abort` and a `p4_supervisor_abort` event, enters `REQUEST_LAND` and returns the
    single LAND (21). P2 then runs `VERIFY_LANDING` → `VERIFY_LANDED_AND_DISARMED` → `COMPLETE`
    under its own checks.
    * The P2 order check runs from `GOAL_HOVER`, and the transition record keeps the real source
      state.
    * Phase dispatch is unchanged (one P4 phase handler per tick).
    * Geofence and P2 errors still raise to the node and P2 `RECOVERY`.
  * **`control_node.outcome()`:** `COMPLETE` with a recorded abort gives `supervisor_abort`, and
    `RECOVERY` gives `failed`.
  * **`perception_node.py` (diagnostic only):** each snapshot also records `compute_thread_cpu_s`
    and `involuntary_context_switches`.
  * **`p4_evaluate.py`:**
    * A new `abort_landing` check for S5 (`outcome:`) and S6 (`s6:f3_`): exactly one accepted
      LAND, `COMPLETE` with no P2 failure, and LAND-to-`COMPLETE` ≤ `recovery_wall_s` (45 s wall).
    * `performance` reports the slowest frame's CPU time and context switches; it is not a check.
* **Tests:**
  * A supervisor abort lands through `REQUEST_LAND` → `COMPLETE` with one LAND, a held target and
    outcome `supervisor_abort`.
  * The `MAP_SCAN` stale abort lands the same way.
  * A geofence breach still routes to `RECOVERY` with outcome `failed`.
  * S5 rejects an abort that ends in `RECOVERY` or lands in more than 45 s.
  * S6 rejects a 45.5 s landing and a `RECOVERY` ending.
* **Rebuild:** new P4 package hash `c4bd5a55abd997207f74fc019759f820ef97b8d07dd2ab103c1e54b44faf2486`;
  the receipt matches the source.
* **Regression:**
  * Linux: P2 219, P3 sensor 63, evaluators 63, runtime 89, MC 23, P4 111, equivalence 6, ROS
    harness 3.
  * Windows: 380 passed, 29 skipped.
  * P3 still verifies to `1e205fec…`.
* **Next (at the time):** Policy A approval, a freeze, then Gates A–C (done in §45 up to the Gate A
  authorization).

## 45. Frozen P4 candidate `6cc87fb2` (2026-10-30, contract revisions 2–5)

* Policy A (`bf1ef9b1…b3b0`, unchanged) was approved by Neura-Shadow for commit
  `b7e4de4df96a55de410495f397c89e2aa33da200` ("Approve Policy A", answering the request for that
  commit). Receipt: `state/p4-policy-A-approval-b7e4de4.json` (`903c9294…cdf4`), with candidate phase
  "p4-acceptance-v1 (revisions 2, 3, 4 and 5)".
* Manifest `state/p4-candidate-b7e4de4.json`: canonical
  **`6cc87fb2047d799d3d097104d985c11f18547445a1c7242ece362cec06429a9f`** (file `8815a846…ff2f`).
  `verify` reproduces it. The revision label names revisions 2–5.
  * Against `1be1896a`, exactly the six revision 5 files differ: `p4_validation.yaml`, `contracts.py`,
    `control_node.py`, `mission.py`, `perception_node.py` and `p4_evaluate.py`.
  * Baselines and build: P3 `1e205fec…`; P4 build `c4bd5a55…`; 29 P4 and 103 P3 source files.
  * Runtime: MC selection matches; `hardware:d3d12`; gates A/B/S/C; no execution without
    authorization.
* No gate record exists for `6cc87fb2`. Earlier candidates' records stay in `state/` as history,
  including `1be1896a`'s accepted Gates A and B and its failed Gate S. Gate A has not been run.

## 46. P4 Gate A of candidate `6cc87fb2` — FAILED at p3-readiness native commands (2026-10-30)

* **Authorization:** Neura-Shadow ("Authorize Gate A", answering the request for this manifest),
  `state/p4-auth-A-6cc87fb2047d.json` (`a8ff9c1b…1c5a`), consumed. Result
  `state/p4-gate-A-6cc87fb2….result.json` (`b61ebb6b…f2af`): `accepted: false`,
  `first_failure: p3-readiness:native_commands:payload_pattern_cap_exceeded`, no automatic retry.
  Run 12:58:48–13:00:26 UTC; no leftover processes. The other six runs did not run.
* **p3-readiness** `20261030T125919Z-p4-p3_readiness-958e86ef`:
  * The step runs the unchanged P3 stack with the P2 controller; no P4 node runs in it.
  * The unchanged Policy A evaluation found one violation. QGC (stock unattended build `82994283…`)
    sent a 35th `COMMAND_LONG` 512 (`MAV_CMD_REQUEST_MESSAGE`, param1 435, param2 2) from 255/190
    to 72/1. Policy A allows at most 34 byte-exact 512 frames, and 34 were allowed. There was 1 × 401
    and 37 GCS heartbeats.
  * Everything else passed.
* **History:** in all 43 retained runs with QGC 512 traffic (P3 and P4, ground and flight), QGC sent
  exactly 34. This is the first time it sent 35, most likely one QGC retry. P4 code cannot influence
  QGC's request count.
* **Assessment:** an intermittent behaviour of the unchanged QGC binary against the accepted Policy A
  cap. It is not a P4 defect. Policy A content stays unchanged, as required.
* **Options (operator decision):**
  * **(a)** Treat it as an intermittent event and re-freeze the unchanged sources (a docs-only
    commit), with a new Policy A approval and a new Gate A authorization.
  * **(b)** Investigate the QGC retry first, from the retained MAVLink capture, before re-freezing.
  * Candidate `6cc87fb2` cannot run Gate A again.

## 47. Re-frozen P4 candidate `c110bd30` (2026-10-30, same sources as `6cc87fb2`)

* **Operator decision (Neura-Shadow):** the extra QGC request of §46 is treated as intermittent, and
  the unchanged sources are re-frozen. Policy A (`bf1ef9b1…b3b0`, unchanged) was approved
  ("Approve Policy A", answering the request) for commit `50d5002e9670d70f07c43792415494ae70f4efe0`,
  whose P4 sources are identical to `b7e4de4`. Receipt: `state/p4-policy-A-approval-50d5002.json`
  (`9b2d40c8…3528`).
* Manifest `state/p4-candidate-50d5002.json`: canonical
  **`c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b`** (file `ab1d2af5…cae7`).
  `verify` reproduces it.
  * No P4 file differs from `6cc87fb2`. Build `c4bd5a55…`; revisions 2–5; P3 `1e205fec…`; MC
    selection matches; `hardware:d3d12`.
* No gate record exists for `c110bd30`. Earlier records stay in `state/` as history. Gate A has not
  been run.

## 48. P4 Gate A of candidate `c110bd30` — ACCEPTED (2026-10-30)

* **Authorization:** Neura-Shadow ("Authorize Gate A", answering the request for this manifest),
  `state/p4-auth-A-c110bd30be22.json` (`dd5d7612…b4b0`), consumed. Result
  `state/p4-gate-A-c110bd30….result.json` (`ffbd4ee9…f4fe9`): `accepted: true`,
  `first_failure: null`, no automatic retry. Run 13:39:16–13:48:42 UTC; no leftover processes.
* **Every run passed with acceptance credit, and every owned process stopped gracefully in every run:**
  p3-readiness (QGC 512 count 34, within the cap), p4-readiness, fixtures G2, G4 and G6, and faults
  adapter_stop and bridge_stop.
  * Perception compute p99 was 0.032–0.038 s and the max was 0.035–0.041 s (limits 0.050 / 0.090).
  * Graph checks took at most 10.5 ms. RTF was 0.59–0.70.
* **Status:** Gate A is accepted for `c110bd30`, with contract revisions 2–5. Gate B (S2), then Gate S
  (S1, S5, S6 — the first run of the revision 5 abort landing), then Gate C (S2, S3, S4) each need
  their own operator authorization bound to this manifest. P4 is not complete.
