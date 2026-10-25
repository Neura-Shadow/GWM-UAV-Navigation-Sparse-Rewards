# v3-SIM P4 contract `p4-acceptance-v1` — deterministic non-learning obstacle avoidance

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** No P4 code exists yet. The design rationale
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
  * rate 20 Hz, prestream 2.0 s, setpoint ramp 0.3 m/s, yaw ramp 10°/s;
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
  2. `MAP_SCAN`: yaw −30° → +30° → 0° at ≤ 10°/s.
  3. `TRANSIT` under the supervisor to the goal (8.0, 0.0) m ENU from the recorded origin and heading.
  4. `GOAL_HOVER` 5 s (horizontal ≤ 0.30 m from the goal, speed ≤ 0.1 m/s).
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
  free never clears occupied. `UNKNOWN` is never free.

**Planner:**

* **L1.** Candidates are θ = ψ_path + k·10° (k = −6…6) × L ∈ {0.75, 1.5} m, plus `HOLD` and
  `YAW_TO(θ)`.
* **L2.** A candidate is valid only if:
  * all cells within `r_infl` = 1.0 m of its swept segment are `FREE`;
  * it lies inside the geofence, inflated;
  * |θ − ψ_vehicle| ≤ 25°;
  * its first 0.30 m (`d_stop`) is clear;
  * the snapshot age is ≤ 0.25 s sim.
* **L3.** Score: `1.0·progress − 0.6·deviation − 0.2·|Δθ| + 0.3·min(clearance − r_infl, 1.0)`.
  Ties go to the smaller |k|, then +y, then the shorter L. Selection is a pure function of
  (snapshot, state, goal).
* **L4.** The position target advances along the selected segment at ≤ 0.3 m/s, and yaw at
  ≤ 10°/s.

**Supervisor transitions:**

| From | Condition | To |
|---|---|---|
| `NORMAL` | straight path segment (L 1.5 m) invalid, with an occupied cell in the corridor within 3.0 m | `OBSTACLE_DETECTED` |
| `OBSTACLE_DETECTED` | best progress candidate valid (progress > 0.05 m) | `AVOIDING` |
| `OBSTACLE_DETECTED` / `AVOIDING` | no valid progress candidate | `HOVER` |
| `AVOIDING` | straight path segment valid | `RESUME` |
| `RESUME` | lateral deviation ≤ 0.10 m and path valid | `NORMAL` |
| `HOVER` | valid progress candidate after a fresh snapshot | `AVOIDING` |
| any | snapshot > 0.25 s sim, adapter health not `fresh`, calibration mismatch, 3 consecutive unusable frames, estimator reset since the last usable frame, or planner output fails re-validation | `HOVER` (no progress) |
| `HOVER` | 20 s sim without progress, or 10 s sim stale | `ABORT` |
| any | P2 safety error, geofence breach, clearance to an occupied cell < `R_body` + 0.10 m, or `TRANSIT` deadline | `ABORT` |
| `ABORT` | hold, then one LAND (21) through P2 recovery rules | (terminal) |

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
| Response latency (snapshot acquisition that first invalidates the path → supervisor leaves `NORMAL`) | ≤ 0.25 s sim |
| Setpoint validity | no commanded target inside any ground-truth obstacle inflated by `R_body` + 0.10 m |

The truth-pose interval is ≤ 0.05 s. At 0.3 m/s the motion between samples (≤ 0.015 m) is added to
the clearance computed at each sample.

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
| S6 | as S2; sensor bridge stopped once the vehicle is 2.0 m from origin | `HOVER` ≤ 0.35 s sim after the last usable snapshot ages past 0.25 s; no progress while stale; `ABORT` and landed | `safe_stop_on_sensor_loss` |
| S7 | offline replay of retained P3 frames with injected reset brackets and stale stamps | every injected frame unusable; `HOVER`; resume only on a fresh matched frame | ground fixture only |

## 8. Gates

All gates use a P4 manifest (P4 sources, build receipt, configs, scenarios, the P3 manifest SHA and
the P3 inputs P4 imports). Each needs a separate operator authorization
(`authorize Gate X <p4-sha> — operator NAME`), consumes it once, stops at the first failure, never
retries, and writes an immutable result. A Policy A approval is required per P4 candidate.

| Gate | Content | Credit |
|---|---|---|
| **A** | P3 readiness unchanged (all P3 limits); P4 ground fixtures: vehicle on the ground facing S2, S4 and S5 obstacles at 2, 4 and 6 m; reconstructed scan and grid within `e_depth + e_grid` of truth; planner decisions equal the expected decision tables; supervisor fault fixtures (adapter stopped, stale clock, calibration mismatch) reach `HOVER`; graph shows a single PX4 input owner; cleanup graceful | ground |
| **B** | one S2 flight | smoke |
| **S** (proposed) | S1, S5, S6 in fixed order; all must pass | safety behaviour |
| **C** | three consecutive flights S2, S3, S4 in fixed order; 3/3 required | qualification |

P4 is complete only with A, B, S (if approved) and C accepted for the same P4 manifest. No P3 gate
result counts.

## 9. Independent evaluation and evidence

`p4_evaluate.py` never trusts P4 self-reports for pass decisions. Per run it checks:

* **(a)** the P3 checks applied unchanged: stream, association, RGB absence, MAVLink policy (with
  the P4 landing window: first controller LAND through `COMPLETE`), renderer, MC causality and
  cleanup;
* **(b)** P2 flight integrity reused function by function from `collect_p2_evidence` (bag/ULog
  integrity, command sequence and ACKs, ROS/ULog position pairing (v1.6), landed and disarmed, no
  failsafe, yaw ownership), without P2 mission windows;
* **(c)** §6 from `p4/truth_poses.jsonl`, `p4/contacts.jsonl` and the declared obstacle boxes;
* **(d)** response latency and supervisor legality from `p4/decisions.jsonl` against
  `p4/local_map.jsonl` and the raw depth index;
* **(e)** the scenario outcome label, progress (final distance to the goal ≤ 0.30 m for goal
  scenarios), unnecessary-detour checks (S1), and runtime cost.

Retained per run: everything P3 retains, plus the P4 ledgers above, the generated world and the
scenario declaration.

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
7. **Depth access:** P4 perception as a second subscriber to the P3 depth image topic. Widening the
   adapter output instead would change P3 manifest-bound code.
8. **S7:** reset and stale-pose handling verified by offline replay only. A SITL reset injection
   would need new PX4 commands outside the policy.
9. **Location:** all P4 code in `simulation/px4_gazebo_p4/`, outside the P3 manifest globs.
