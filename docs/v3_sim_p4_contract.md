# v3-SIM P4 contract `p4-acceptance-v1` — deterministic non-learning obstacle avoidance

**Status: APPROVED — revision 2 (commit `d3c06b4`) plus revision 3 (commit `ea2c9e3`,
[proposal](v3_sim_p4_contract_revision3_proposal.md)) plus revision 4 (commit `65b99e4`,
[proposal](v3_sim_p4_contract_revision4_proposal.md)) plus revision 5 (commit `ae0e037`,
[proposal](v3_sim_p4_contract_revision5_proposal.md)).** Revision 2 added the S6 fault-injection window
in §9.1, the separation of collision validity from translation authority in §5 L2–L6, and the
runtime-performance limits in §6.1 and Gate A. Revision 3 makes the initial plan feasible: a 360°
`MAP_SCAN`, traversed cells counting as free in L2, and free space observed only beyond the near clip. Revision 4 pins a P4 `TRANSIT` target speed of
0.20 m/s so PX4's steady tracking lag stays inside the unchanged 0.30 m bound. Revision 5 lands a
supervisor `ABORT` through P2's nominal landing sequence to `COMPLETE`; P2 safety errors still use `RECOVERY`. The design rationale
and the verified integration facts are in the [P4 plan](v3_sim_p4_plan.md). Baseline: P3 COMPLETE
under `p3-acceptance-v1.6` (candidate `1e205fec…`), which must remain verifiable unchanged.

## 1. Scope and claims

* **Claimed on acceptance:** in PX4 SITL (v1.17.0, Gazebo Harmonic, ROS 2 Jazzy, WSL2, D3D12), the
  x500 with the forward OakD-Lite depth camera detects **static** box obstacles from simulated depth.
  It avoids them within the P4 geofence using a deterministic candidate planner, resumes and lands.
  When no safe route exists or depth is lost, it refuses to advance safely. All of this holds for
  scenarios S1–S6 at ≤ 0.3 m/s.
* **Not claimed:** moving obstacles; obstacles thinner than one depth pixel at range or outside the
  flight band; space never observed by the camera; speeds above 0.3 m/s; real hardware; RGB use.
  No learning of any kind.

## 2. Software boundary and control authority

| Component | Location | Inputs | Outputs |
|---|---|---|---|
| `p4_perception` | `simulation/px4_gazebo_p4/ros2_ws/src/gwm_obstacle_avoidance` | `/gwm/sensors/front_depth/image_raw`, `/camera_info`, the P3 adapter health stream, PX4 `vehicle_local_position` and `vehicle_attitude` | `/gwm/p4/local_map` (JSON), `p4/local_map.jsonl` |
| `p4_control` (P4Mission, Supervisor, Planner) | same package | PX4 state topics (the P2 set), `/gwm/p4/local_map`, clock | **sole publisher** of `/px4_71/fmu/in/{offboard_control_mode, trajectory_setpoint, vehicle_command}`; `ros-events.jsonl`, `p4/decisions.jsonl` |
| `p4_truth_observer` | `simulation/px4_gazebo_p4/validation` | Gazebo dynamic poses (all models), obstacle contact topics | `p4/truth_poses.jsonl` (≥ 20 Hz), `p4/contacts.jsonl`; evaluator only, never read by the vehicle |
| P3 sensor stack | unchanged P3 code | as in P3 | as in P3 (recorder, observations, health, source probe, RGB-absence monitor) |

**Rules:**

* **C1.** The P2 controller is never launched in a P4 run.
* **C2.** `p4_control` enforces P2's graph rule: exactly one publisher per PX4 input topic, and
  `competing_control_publisher` aborts the run.
* **C3.** The only `vehicle_command`s are 176, 400 and 21, as in P2 (one attempt each, P2 ACK
  rules).
* **C4.** No P4 process starts unless `p4_avoidance.yaml` has `enabled: true` and a consumed P4
  authorization is bound to the frozen P4 manifest.
* **C5.** No file under the P3 manifest scope (`simulation/px4_gazebo/{scripts,validation,configs}`,
  `ros2_ws/src/{gwm_px4_control,gwm_sensor_adapter}`, `docs/evidence/v3_sim_p2_summary.json`) is
  created, changed or deleted. The P4 runner refuses to start unless
  `p3_manifest.verify_current` returns `1e205fec…`.

## 3. Limits carried over from P2/P3 unchanged

* **P2 timing and safety** (`p2_control.yaml` values, read by P4 from that file):
  * rate 20 Hz, prestream 2.0 s, setpoint ramp 0.3 m/s (P2 phases; P4 `TRANSIT` uses the slower
    0.20 m/s of L7), yaw ramp 10°/s;
  * `max_sample_gap_sim_s` 0.2; position and attitude freshness 0.25 s sim; flags 1.5 s;
    state 2.0 s wall; clock stall 1.0 s;
  * ACK deadlines 3.0 s sim / 5.0 s wall, one attempt;
  * startup 75 s, recovery 45 s; v3 reference policy and yaw handover; `yaw_tolerance_deg` 5.
* **P3 sensing** (`p3_validation.yaml`, `p3_sensors.yaml`, read unchanged):
  * all depth stream limits; state association, with the v1.6 reset-bracket rule as an evaluator
    allowance only;
  * recorder overflow 0; RGB absent and verified in flights; renderer D3D12 for credited gates;
  * MC causality at 1e-5 rad; graceful cleanup; Policy A content.
* **Not carried over:** P2 mission waypoints, dwell windows, `horizontal_tolerance_m` against fixed
  waypoints, and `max_horizontal_radius_m` 2.0 m / `max_height_m` 3.0 m. These describe the P2
  mission, not safety of a detour. They are replaced by §4 and §6.

## 4. P4 mission and envelope

* **Mission:**
  1. P2 initialization through `INITIAL_HOVER` at 2.0 m (same states and checks).
  2. `MAP_SCAN`: one full 360° in-place revolution, counter-clockwise in ENU, ending at the initial
     heading, at ≤ 10°/s (≈ 36 s), executed as an L5 yaw (revision 3).
  3. `TRANSIT` under the supervisor to the goal (8.0, 0.0) m ENU from the recorded origin and heading.
  4. `GOAL_HOVER` 5 s (horizontal ≤ 0.30 m from the goal, speed ≤ 0.1 m/s).
     On a supervisor `ABORT` in `MAP_SCAN` or `TRANSIT`, the mission goes from the hold directly to
     step 5 (`REQUEST_LAND`) (revision 5).
  5. `REQUEST_LAND`, then P2 landing verification through `COMPLETE`.
* **Geofence:** x ∈ [−1.0, 9.5] m, |y| ≤ 3.0 m, height 1.0–3.0 m. A breach of the measured position
  aborts.
* **Deadlines:** `TRANSIT` ≤ 150 s sim; `HOVER` ≤ 20 s sim before `ABORT`; stale-depth `HOVER`
  ≤ 10 s sim before `ABORT`; P2 deadlines for every non-P4 phase.

## 5. Perception, planning and supervisor rules (parameters in `p4_avoidance.yaml`)

**Perception:**

* **P1.** Process the newest frame on the 10 Hz sim-time phase only after strict decode; a
  calibration identity equal to the adapter's; adapter health `fresh`; and P3 association `matched`
  (300 samples, 0.05 s, no interpolation). Reset brackets and every other status are unusable for
  planning and are counted by reason.
* **P2.** Flight band: world height of the vehicle −0.38 m to +0.22 m. Use 160 azimuths (every 4th
  column). Per azimuth: minimum horizontal range of `VALID` band points; all no-return gives free to
  4.0 m; otherwise unknown.
* **P3.** Grid at 0.10 m over the geofence with `UNKNOWN` / `FREE` / `OCCUPIED`. A hit overrides free;
  free never clears occupied. `UNKNOWN` is never free. Each ray marks `FREE` only from the camera's
  near clip (`near_m` 0.2 m) outward; a too-close return is `OCCUPIED` at the near clip (revision 3).

**Planner.** Whether a path is collision-free and whether the vehicle may translate along it now
are separate tests. In-place yaw is the only way to acquire new observations; translation never
enters space that is not observed `FREE`.

* **L1.** Translation candidates are θ = ψ_path + k·10° (k = −6…6) × L ∈ {0.75, 1.5} m. The other
  actions are `HOLD`, `YAW_TO(θ)` and `SCAN`.
* **L2 — collision-valid** (independent of the current heading). A translation candidate is
  collision-valid only if:
  * all cells within `r_infl` = 1.0 m of its swept segment are `FREE` or **traversed** (within
    `R_body` of an earlier position estimate, as in L5). Traversed never overrides `OCCUPIED`, and
    `UNKNOWN` cells outside the traversed region still invalidate the candidate (revision 3);
  * the segment and its inflation lie inside the geofence;
  * its first 0.30 m (`d_stop`) is clear;
  * it is evaluated on a snapshot no older than 0.25 s sim.
* **L3 — score.** Only collision-valid candidates are scored:
  `1.0·progress − 0.6·deviation − 0.2·|Δθ| + 0.3·min(clearance − r_infl, 1.0)`. Ties go to the
  smaller |k|, then +y, then the shorter L. Selection is a pure function of (snapshot, state, goal).
* **L4 — translation authority.** The best-scored candidate may be executed as translation only if
  it is collision-valid **and** |θ − ψ_vehicle| ≤ 25°, so the camera keeps observing the space being
  entered. Otherwise the action is `YAW_TO(θ)`.
* **L5 — in-place yaw (`YAW_TO`, `SCAN`).** Yaw without translation is allowed only if:
  * the position target is frozen at the current target (no translation component);
  * the snapshot is no older than 0.25 s sim;
  * every cell within `R_body` (0.39 m) of the current target is `FREE` or **traversed** (within
    `R_body` of an earlier position estimate, so the vehicle itself occupied it);
  * no `OCCUPIED` cell lies within `R_body` + 0.10 m + `e_track` (0.79 m) of the current target.
    The envelope is a cylinder about the yaw axis, so rotation sweeps no volume beyond the disc it
    already occupies; the outer ring covers drift within the tracking bound.
  * the yaw rate is ≤ 10°/s.

  `MAP_SCAN` in `INITIAL_HOVER` is an L5 yaw.

  After a yaw, translation authority (L4) is re-evaluated only on a snapshot acquired after the
  vehicle reached |ψ − θ| ≤ 25°.
* **L6 — observation acquisition.** If no candidate is collision-valid but some are blocked only by
  `UNKNOWN` cells (no `OCCUPIED` cell in their swept inflation), the action is `SCAN`: an L5 yaw
  toward the heading of the blocked candidate with the most `UNKNOWN` cells. Ties go to the smaller
  |k|, then +y. A heading is scanned at most once per `HOVER` episode.
* **L7.** A translating target advances along the selected segment at the pinned `TRANSIT` speed of
  0.20 m/s (`mission.transit_speed_m_s`; never above P2's 0.3 m/s ramp). Any action is re-validated
  each tick, and a failed re-validation yields `HOLD`.

**Supervisor transitions:**

| From | Condition | To |
|---|---|---|
| `NORMAL` | straight path segment (L 1.5 m) invalid, with an occupied cell in the corridor within 3.0 m | `OBSTACLE_DETECTED` |
| `OBSTACLE_DETECTED` | a collision-valid progress candidate exists (progress > 0.05 m); it is executed as translation or as `YAW_TO` per L4 | `AVOIDING` |
| `OBSTACLE_DETECTED` / `AVOIDING` | no collision-valid progress candidate | `HOVER` (L6 `SCAN` allowed) |
| `AVOIDING` | straight path segment collision-valid | `RESUME` |
| `RESUME` | lateral deviation ≤ 0.10 m and path collision-valid | `NORMAL` |
| `HOVER` | a collision-valid progress candidate on a fresh snapshot | `AVOIDING` |
| any | snapshot > 0.25 s sim, adapter health not `fresh`, calibration mismatch, 3 consecutive unusable frames, estimator reset since the last usable frame, or planner output fails re-validation | `HOVER` with yaw frozen too (no translation, no `YAW_TO` or `SCAN`) |
| `HOVER` | 20 s sim without progress, or 10 s sim stale | `ABORT` |
| any | clearance to an occupied cell < `R_body` + 0.10 m, or `TRANSIT` deadline | `ABORT` |
| any | P2 safety error or geofence breach | P2 `RECOVERY` (the run fails; unchanged P2 rules) |
| `ABORT` | hold, then exactly one LAND (21) through P2's nominal landing sequence `REQUEST_LAND` → `VERIFY_LANDING` → `VERIFY_LANDED_AND_DISARMED` → `COMPLETE`, under P2's unchanged landing checks and phase deadlines; landed and disarmed within 45 s wall of the LAND command | (terminal) |

A supervisor `ABORT` (`hover_timeout`, `stale_inputs:*`, `map_scan_stale_inputs:*`, `transit_deadline`,
`clearance_below_body_margin`) is a P4 outcome, not a P2 failure. The controller records `p4_outcome`
= `supervisor_abort` and the abort reason, and leaves the P2 failure field empty. Any other failure
(P2 safety error, geofence breach, timing guard, ACK failure, observation gap, and so on) enters P2
`RECOVERY` exactly as before and fails the run. Routing never makes a run pass: every scenario's §7
outcome check still requires its own expected outcome and abort reason (revision 5).

## 6. Safety envelope (proposed limits; derivation in plan §4)

| Quantity | Limit |
|---|---|
| Collision envelope | cylinder radius `R_body` 0.39 m about `base_link`, z −0.23 to +0.07 m |
| Planning inflation `r_infl` | 1.0 m |
| Stopping allowance `d_stop` | 0.30 m |
| Obstacle trigger range | 3.0 m |
| Free cap for no-return rays | 4.0 m |
| Minimum ground-truth clearance (envelope surface to obstacle surface) | **≥ 0.20 m** at every truth sample |
| Collisions (truth envelope overlap or any obstacle contact message) | **0** |
| Horizontal tracking, vehicle versus commanded target, during `TRANSIT` | ≤ 0.30 m |
| `TRANSIT` target speed | 0.20 m/s (steady PX4 lag 0.20 / `MPC_XY_P` 0.95 ≈ 0.21 m, inside the 0.30 m tracking bound) |
| Response latency (snapshot acquisition that first invalidates the path → supervisor leaves `NORMAL`) | ≤ 0.25 s sim |
| Setpoint validity | no commanded target inside any ground-truth obstacle inflated by `R_body` + 0.10 m |

The truth-pose interval is ≤ 0.05 s. At 0.3 m/s the motion between samples (≤ 0.015 m) is added to
the clearance computed at each sample.

### 6.1 Runtime performance under the added P4 load

P4 adds a second full-frame subscriber to the depth image (640×480 `32FC1`, about 1.2 MB × 30 Hz,
≈ 37 MB/s through the existing Fast DDS profile) and a 10 Hz perception workload. Every P4 run
(Gate A readiness and fixtures, and every flight) is evaluated against all of these limits. None of
the existing limits is relaxed.

| Measurement | Limit | Source |
|---|---|---|
| P3 depth stream with P4 running: delivered rate, source gap, receipt stall, observation age, processed rate, recorded fraction, recorder overflow | ≥ 25 Hz, ≤ 0.12 s, ≤ 1.0 s wall, ≤ 0.25 s sim, ≥ 8 Hz, 1.0, 0 | P3 limits, unchanged |
| P2 state freshness (including `vehicle_land_detected`), clock stall, sample gap | ≤ 2.0 s wall, ≤ 1.0 s wall, ≤ 0.2 s sim | P2 limits, unchanged |
| P4 controller setpoint stream: rate, dispatch budget, publication coverage | the P2 `p2_timing.yaml` values, judged by the unchanged P2 timing evaluator | P2 limits, unchanged |
| P4 perception snapshot rate in the evaluation window | ≥ 8 Hz | equals P3 `processed_hz_min` |
| P4 snapshot latency (depth acquisition → `/gwm/p4/local_map` publication, sim time) | max ≤ 0.25 s, p95 ≤ 0.15 s | max equals P3 `observation_age_sim_s`, which `T_lat` assumes (plan §4) |
| P4 perception compute per processed frame (thread wall time) | p99 ≤ 0.050 s, max ≤ 0.090 s | must finish inside one 0.10 s processing period |
| P4 perception image-receipt gap | ≤ 1.0 s wall | equals P3 `stall_wall_s` |
| Frames skipped by the P4 overwrite queue | counted and reported; no limit (decimation by design), but no processed frame may exceed the latency limit | |
| Per-process CPU and RSS, RTF, host load | recorded every second; informational (the wall-time freshness limits above capture their effect) | |

## 7. Scenarios and pass criteria

Worlds are the pinned `default.sdf` plus static boxes with contact sensors (P3 generator pattern;
vehicle SDF unchanged). Sizes are x×y×z in metres; all boxes are 4 m tall, centred at height 2 m.

| ID | Obstacles | Pass (all P2/P3 carried checks plus §6 must also pass) | Outcome label |
|---|---|---|---|
| S1 | none | goal reached and landed; no `AVOIDING` or `HOVER`; max lateral deviation ≤ 0.30 m | `goal_reached` |
| S2 | pillar 0.4×0.4 at (4.0, 0.0) | `OBSTACLE_DETECTED` → `AVOIDING` → `RESUME` → `NORMAL`; goal reached; landed | `goal_reached_with_detour` |
| S3 | pillar 0.4×0.4 at (4.0, +0.5) | as S2, and the detour passes on −y | `goal_reached_with_detour` |
| S4 | pillars 0.4×0.4 at (3.5, 0.0) and (6.0, −1.4) | as S2 | `goal_reached_with_detour` |
| S5 | wall 0.3×8.0 at (4.0, 0.0) | `HOVER` with clearance ≥ 0.20 m; no progress after `HOVER`; `ABORT` and landed | `safe_refusal` (never `goal_reached`) |
| S6 | as S2; the runner stops the sensor bridge once the vehicle is 2.0 m from origin (§9.1) | full P3 checks before injection; after it: `HOVER` within 0.35 s sim of the last usable snapshot, no translation or yaw while stale, `ABORT` at 10 s stale, landed | `safe_stop_on_sensor_loss` |
| S7 | offline replay of retained P3 frames with injected reset brackets and stale stamps | every injected frame unusable; `HOVER`; resume only on a fresh matched frame | ground fixture only |

## 8. Gates

All gates use a P4 manifest (P4 sources, build receipt, configs, scenarios, the P3 manifest SHA and
the P3 inputs P4 imports). Each needs a separate operator authorization
(`authorize Gate X <p4-sha> — operator NAME`), consumes it once, stops at the first failure, never
retries, and writes an immutable result. A Policy A approval is required per P4 candidate.

| Gate | Content | Credit |
|---|---|---|
| **A** | (1) **P3 readiness unchanged** (P3 scope, RGB consumer, all P3 limits). (2) **P4 readiness under load:** the full P4 stack (P3 sensor stack, `p4_perception`, `p4_control` in ground-diagnostic mode, which streams setpoints but never arms or commands) for the 30 s ground window; every §6.1 limit must pass, proving the extra subscriber and perception keep P3 and P2 freshness before any flight. (3) **P4 ground fixtures:** vehicle on the ground facing S2, S4 and S5 obstacles at 2, 4 and 6 m; reconstructed scan and grid within `e_depth + e_grid` of truth; planner and L5/L6 yaw decisions equal the expected decision tables; §6.1 limits pass. (4) **Fault fixtures** (adapter stopped, bridge stopped, stale clock, calibration mismatch) reach `HOVER` with yaw frozen. (5) A single PX4 input owner in the graph; cleanup graceful | ground |
| **B** | one S2 flight | smoke |
| **S** (proposed) | S1, S5, S6 in fixed order; all must pass | safety behaviour |
| **C** | three consecutive flights S2, S3, S4 in fixed order; 3/3 required | qualification |

P4 is complete only with A, B, S (if approved) and C accepted for the same P4 manifest. No P3 gate
result counts.

## 9. Independent evaluation and evidence

`p4_evaluate.py` never trusts P4 self-reports for pass decisions. Per run it checks:

* **(a)** the P3 checks applied unchanged: stream and association (over the full sensor window,
  except in S6, where §9.1 bounds them), RGB absence, MAVLink policy (with the P4 landing window:
  first controller LAND through `COMPLETE`), renderer, MC causality and cleanup;
* **(a′)** every §6.1 runtime-performance limit;
* **(b)** P2 flight integrity reused function by function from `collect_p2_evidence` (bag/ULog
  integrity, command sequence and ACKs, ROS/ULog position pairing (v1.6), landed and disarmed, no
  failsafe, yaw ownership), without P2 mission windows;
* **(c)** §6 from `p4/truth_poses.jsonl`, `p4/contacts.jsonl` and the declared obstacle boxes;
* **(d)** response latency and supervisor legality from `p4/decisions.jsonl` against
  `p4/local_map.jsonl` and the raw depth index;
* **(e)** the scenario outcome label, progress (final distance to the goal ≤ 0.30 m for goal
  scenarios), unnecessary-detour checks (S1), and runtime cost.

Retained per run: everything P3 retains, plus the P4 ledgers above, the generated world, the
scenario declaration and, in S6 only, `p4/fault_injection.json`.

### 9.1 S6 fault-injection window

Only S6 may contain an intentional sensor fault, and only the one defined here. All other depth
loss, in any scenario, fails under the unchanged P3 checks.

* **Injection.** The P4 runner reads the controller ledger. On the first `TRANSIT` sample with the
  vehicle ≥ 2.0 m (horizontal) from origin, it sends one SIGINT to the owned `sensor-bridge`
  process. It writes `p4/fault_injection.json`: scenario, process name, PID, signal, request
  monotonic time, injection sim time `t_inj` (latest `/clock`) and vehicle position. The run fails
  if S6 has no record or more than one, or if any other scenario has a record.
* **Pre-injection window** `W_pre` = [sensor-window start, `t_inj` − 0.12 s]. It ends one source-gap
  limit early so the frame in flight at injection is not counted. Every P3 stream and association
  check applies **unchanged** to `W_pre`: delivery, source gap, receipt stall, ages, rates, recorded
  fraction, association and recorder. `W_pre` must last ≥ 5 s sim, and §6.1 applies to it.
* **Fault window** `W_fault` = (`t_inj`, `COMPLETE`]. P3 delivery and rate checks are not applied to
  it. These must all hold instead:
  * **F1 — attributable.**
    * Native Gazebo depth headers (P3 source probe, subscribed directly to Gazebo) continue with no
      source gap > 0.12 s until `ABORT`, so the source is alive.
    * No ROS receipt has an acquisition stamp > `t_inj` + 0.12 s, so the bridge is what was cut.
    * `sensor-bridge` exits 0 within the P3 graceful-stop limit after the recorded SIGINT, as in
      the P3 interruption fixture.
    * No other owned process exits before shutdown.
  * **F2 — detected.** Adapter health leaves `fresh` within `interruption_detect_wall_s` (1.2 s
    wall, P3 limit) of the injection request. The adapter publishes no observation after that. The
    recorder stays complete with 0 overflow.
  * **F3 — safe response.**
    * Let `t_last` be the acquisition time of the last usable P4 snapshot. The supervisor enters
      `HOVER` with yaw frozen by `t_last` + 0.25 + 0.10 s sim.
    * From `HOVER` to `ABORT`, the commanded target moves ≤ 0.01 m horizontally and ≤ 0.5° in yaw.
    * `ABORT` comes 10 s sim (±0.10 s) after staleness began.
    * Exactly one LAND (21) is issued through `REQUEST_LAND` and accepted, and the vehicle reaches
      `COMPLETE` (landed and disarmed) within 45 s wall of the LAND command (revision 5).
  * **F4 — everything else unchanged over the whole run:** §6 clearance and collisions, MAVLink
    policy, MC causality, RGB absence, P2 integrity, renderer and cleanup.
* **Unexpected loss still fails.** It is a failure if:
  * a gap or stall starts before `t_inj` − 0.12 s;
  * the native depth source stops;
  * any process other than `sensor-bridge` dies;
  * the adapter or perception stops for any reason other than the recorded injection;
  * any non-S6 run shows depth loss.

## 10. Decisions requiring operator approval (recommendation first)

1. **Control authority:** an exclusive `p4_control` reusing P2 modules by import, with the P2 node
   not launched. The alternative, adding a reference interface to P2, changes accepted P2 and
   P3-manifest code and is not recommended.
2. **P4 envelope and mission:** the §4 geofence (x −1.0…9.5 m, |y| ≤ 3.0 m, height 1.0–3.0 m) and
   goal 8.0 m replace P2's 2.0 m radius and 3.0 m ceiling for P4 runs only. P2 timing, freshness,
   speed and yaw-rate limits stay.
3. **Safety thresholds:** §6 as derived (r_infl 1.0 m, minimum clearance 0.20 m, tracking 0.30 m,
   latency 0.25 s, 0 collisions).
4. **Gate structure:** add Gate S (S1, S5, S6) between B and C, and Gate C = S2, S3, S4. The
   alternatives are 6 flights in Gate C, or S5/S6 as ground replay only (a weaker claim).
5. **Unknown space:** never free. No-return rays count as free only to 4.0 m.
6. **Native commands:** reuse Policy A content unchanged, approved per P4 candidate. The P4 landing
   window starts at the first controller LAND (covers ABORT landings).
7. **Depth access:** P4 perception as a second subscriber to the P3 depth image topic. Its load is
   bounded and verified on the ground (§6.1, Gate A part 2) before any flight. Widening the adapter
   output instead would change P3 manifest-bound code.
8. **S7:** reset and stale-pose handling verified by offline replay only. A SITL reset injection
   would need new PX4 commands outside the policy.
9. **Location:** all P4 code in `simulation/px4_gazebo_p4/`, outside the P3 manifest globs.
