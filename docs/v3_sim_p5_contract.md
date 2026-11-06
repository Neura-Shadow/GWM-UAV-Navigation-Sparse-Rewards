# v3-SIM P5 contract `p5-acceptance-v1` — fixed-weight policy substitution

**Status: APPROVED (revision 1, commit `673d65e`) by the operator on 2026-10-31, with decisions
D1–D9 as recommended in §11. Revision 2 (Gate A items 4 and 5,
[text](v3_sim_p5_contract_revision2.md)) approved by the operator on 2026-10-31.** Training (plan M3) was authorized and run offline on 2026-10-31.
It produced candidate `p5-ranker-ee006a80…`, which meets §4.2 on 300 held-out layouts (plan §13).
The operator accepted this candidate on 2026-10-31. The offline milestones M4
(`P5Mission`, the policy ledger and `p5_control`) M5 (runner, gates and evaluator), M6 (P4 vs P5 comparison) and M7 (build and freeze tooling) are implemented. Candidate `44ff340` (manifest `baef37a0…`) failed Gate A on an evaluator gap (plan §13.8); the fixed candidate `ab7b0c6` is frozen as manifest `db4c4779…` (plan §13.9); **Gates A, B and S accepted** for `db4c4779…` (plan §13.10–13.12); **Gate C failed** on one S2 inference tick of 10.84 ms against the 10 ms maximum (plan §13.13). **P5 was stopped by the operator on 2026-10-31 and is not qualified** (plan §13.16). **Revision 3** ([text](v3_sim_p5_contract_revision3.md); bit-exact vectorized polar profile, no limit changed) was approved by the operator on 2026-11-06 after the [CPU recovery plan](v3_sim_p5_cpu_recovery_plan.md); candidate `40a109b` has Policy A approval and is frozen as manifest `2becfbbb…` (plan §13.18); it needs fresh Gates A-C. No gate of it has run. The design rationale and verified integration facts are in the
[P5 plan](v3_sim_p5_plan.md).

The baselines must stay independently verifiable and unchanged:

* P3 `p3-acceptance-v1.6`, manifest `1e205fec…`.
* P4 `p4-acceptance-v1` revisions 2–5, manifest `c110bd30…`
  ([P4 final acceptance](v3_sim_p4_final_acceptance.md)).

## 1. Scope and claims

* **Claimed on acceptance:** in the accepted P4 setting (PX4 SITL, x500 with the forward depth
  camera, static box obstacles, the P4 geofence and mission, 0.20 m/s transit), a **frozen,
  fixed-weight learned policy** chooses the executed motion candidate among the P4 collision-valid
  progress candidates.
  * Every executed action passes the unchanged P4 safety rules and supervisor, and the run lands
    safely.
  * The policy participates in every progress decision. No deterministic fallback is counted as a
    policy decision.
* **Research question:** can a fixed-weight learned candidate ranker make effective avoidance
  decisions under the same environment, dynamics and safety constraints as the P4 deterministic
  ranker? P5 establishes correct substitution and validated SITL behaviour. It does not claim that
  P5 outperforms P4; that needs the paired statistics planned for P7.
* **Not claimed:**
  * online learning or any runtime weight change;
  * end-to-end neural control, or neural commands to PX4;
  * moving obstacles, speeds above the P4 transit speed, or scenarios outside S1–S7;
  * real hardware.

## 2. Ownership and architecture (normative)

* **C1. One controller.** `p5_control` is the single owner of the PX4 input topics, enforced by the
  P2 graph check (exactly one publisher). `p4_control` and `p2_control` are never launched with it.
* **C2. No direct publication.** The policy runtime is a pure in-process function. It returns one
  finite score per candidate slot. It has no ROS publisher or subscriber, no PX4 topic access and no
  file output other than the controller's ledgers.
* **C3. Selection only.** The policy replaces only P4's L3 selection: the `select` step among the
  progress candidates in `planner.plan`. The following stay P4's, unchanged:
  * candidate generation (L1);
  * collision validity (L2);
  * translation authority (L4) and in-place yaw (L5);
  * `SCAN` target choice (L6);
  * execution (L7);
  * the supervisor transition table and every P4 threshold.
* **C4. Frozen baselines.** No file under the P2/P3 manifest scope or the P4 manifest scope
  (`simulation/px4_gazebo_p4/{configs,scripts,validation,ros2_ws/src/gwm_obstacle_avoidance}`) is
  created, changed or deleted. The P5 runner refuses to start unless `p3_manifest.verify_current`
  returns `1e205fec…` and `p4_manifest.verify_current` returns `c110bd30…`.
* **C5. Isolation.** All P5 code, configuration, training tooling and model artifacts live under
  `simulation/px4_gazebo_p5/`, outside every P3 and P4 manifest glob.
  * P5 imports P4 and P2 modules read-only from their accepted installs.
  * P4 runs are unaffected when P5 is absent or disabled.
* **C6. Disabled by default.** P5 nodes refuse to run unless an authorized P5 run enables them in its
  run-local configuration and the checkpoint verifies (§4).

## 3. Observation and action contract (normative; `p5_model_schema.json` freezes it)

### 3.1 Candidate slots and order

* There are **26 slots**, in P4 `evaluate_candidates` order: `k` = −6 … +6 (outer loop), then nominal
  length 0.75 m, 1.5 m (inner loop). Slot index = 2·(k + 6) + (0 for 0.75 m, 1 for 1.5 m).
* Geometry is exactly P4's: heading = path heading + k·10°, and each segment is clipped at the goal
  distance. The policy cannot change it.

### 3.2 Per-slot features

Every feature is float64, computed only from the validated onboard grid, the P4 candidate rows, the
P4 context and the supervisor state. No ground truth is used at inference.

* **Candidate features, 11:**
  1. `k / 6`
  2. nominal length / 1.5
  3. clipped length / 1.5
  4. progress / 1.5
  5. endpoint deviation from the path line / 3.0
  6. turn = |wrap(heading − vehicle yaw)| / π
  7. clearance term = min(clearance − r_infl, clearance cap) / clearance cap, with 0 for invalid
     slots
  8. observable flag, turn ≤ 25° (0/1)
  9. reaches-goal flag (0/1)
  10. collision-valid flag (0/1)
  11. progress-qualified flag (0/1)
* **Shared context, broadcast to every slot, 35:**
  * goal distance / 8.0;
  * target deviation from the path line / 3.0;
  * sin and cos of (vehicle yaw − path heading);
  * supervisor state one-hot: NORMAL, OBSTACLE_DETECTED, AVOIDING, RESUME, HOVER (5);
  * a polar occupancy profile: for each of the 13 candidate headings, the distance from the target
    to the first `OCCUPIED` cell and to the first non-`FREE` cell along the ray, capped at 4.0 m and
    divided by 4.0 (26).
* **Totals:** 46 features per slot, input tensor shape `(26, 46)`.
* **Normalization:** the fixed constants above only. There are no learned statistics.
* **Enforcement:** the feature list, order, constants and schema version are hashed into the
  manifest. Any change needs a new candidate.

### 3.3 Output and selection

* **Output:** a `(26,)` float64 score vector, one score per slot. Larger is preferred.
* **Allowed mask:** collision-valid ∧ progress-qualified, exactly P4's `progress` set. Masked slots
  are never selected, whatever their score.
* **Selection:** the argmax of the score over allowed slots. Scores within 1e-9 of the maximum are
  tied, and ties go to P4's L3 tie order (smaller |k|, then +y, then shorter length).
* **Validity:** the output must have shape `(26,)` and be finite on every allowed slot. Otherwise
  the decision is a **policy failure** (§5). Scores on masked slots are ignored, and logged.
* **When the policy runs:** only when the P4 progress set is non-empty, in every supervisor state
  where P4 calls `select`. With no progress candidate the P4 path (L6 `SCAN` / `HOLD` / `HOVER`)
  applies unchanged, and this is not a policy decision.

## 4. Model contract (normative)

* **Architecture `p5-ranker-mlp-v1`:** a shared per-slot MLP, Linear(46→64) → ReLU → Linear(64→64)
  → ReLU → Linear(64→1), about 7.2 k parameters. The canonical forward pass is NumPy float64 on CPU,
  in the controller process.
* **Artifact:** `models/p5-ranker-<sha256>.npz`, with arrays `W1, b1, W2, b2, W3, b3` and a JSON
  metadata record. The metadata names the architecture, the schema hash, `trained: true`, training
  provenance (§4.1), dataset manifest hash, seeds and offline readiness results (§4.2).
  * The SHA-256 of the file is bound in the manifest.
  * The artifact is committed only if it is ≤ 1 MB (operator decision D4).
* **Integrity at start:** `p5_control` verifies the file SHA-256, the architecture, the array shapes,
  the schema hash and `trained: true` against the frozen manifest values before creating any
  publisher. Any mismatch refuses to start.
* **Inference only:** there is no gradient, optimizer or parameter write path; the weight arrays are
  set read-only. No hidden state is carried between ticks or episodes, and the same inputs and
  weights give bit-identical outputs.
* **Untrained models:** a model with `trained: false`, for example fixed-seed random weights, may
  appear only in adapter and interface tests. The controller refuses it in every authorized scope,
  and it never supports a learned-policy claim.

### 4.1 Provenance (no checkpoint exists today; see plan §2)

* **Training method:** decided by operator decision D1. The recommended method is privileged-oracle
  cost-to-go supervision with DAgger aggregation.
* **Environment:** the synthetic closed loop, using the P4 production perception, planner and
  supervisor with an independent renderer and a first-order PX4 position-loop model.
* **Data separation:**
  * Training and held-out layouts are procedurally generated from recorded seeds.
  * They are **disjoint from S1–S6 and G2/G4/G6.** No training or held-out layout may match an
    acceptance layout: an obstacle set with every centre within 0.25 m and every size within 0.10 m
    of an acceptance obstacle set counts as a match. A test enforces this.
  * Acceptance scenarios are never used for training, validation or model selection.
* **Authorization:** training needs separate operator authorization (plan M3). It runs offline only
  (Windows CPU PyTorch or NumPy), with no simulator. The dataset itself is not committed: its
  generator, seeds and SHA-256 manifest are.

### 4.2 Offline model readiness (before any freeze)

Evaluated on at least 200 held-out synthetic layouts, comparing the policy with the P4 ranker under
the same shield:

* **Shield violations:** 0. Executed candidates are always in the allowed set.
* **Goal success:** the policy's goal-reached rate must be no more than 5 percentage points below
  P4's (operator decision D8).
* **Reported:** mean time-to-goal, path length, minimum clearance and the disagreement rate with P4.
* **Binding:** the report and its hash are bound into the manifest.

## 5. Safety shield and failure behaviour (normative)

* **Shield, the unchanged P4 rules on every executed action:**
  * L2 collision validity (r_infl 1.0 m, observed-FREE or traversed only, geofence);
  * L4 translation authority (heading within ±25°);
  * L5 yaw permission;
  * L7 re-validation every tick;
  * `d_stop`;
  * the full supervisor (freshness 0.25 s, adapter health, calibration identity, association,
    estimator reset, clearance below `R_body` + 0.10 m, hover and stale timeouts, transit deadline);
  * the revision 5 abort landing;
  * P2 safety, ACK, command and timing rules.

  The selected slot is re-checked against the allowed mask before execution. A mismatch is a policy
  failure.
* **Policy failure:** any of missing or unverifiable weights, a schema or shape error, a non-finite
  score on an allowed slot, a selection outside the allowed mask, an exception, or inference wall
  time > 0.020 s.
  * Each failure is logged as `policy_failure` with its reason.
  * The failing tick executes **HOLD**: the target and yaw are frozen and no candidate is executed.
  * **3 consecutive** failure ticks (0.15 s) make the supervisor `ABORT` with reason
    `policy_unavailable`, then land through revision 5.
  * A start-up integrity failure refuses to start.
* **No deterministic fallback in acceptance runs (operator decision D2).** The P4 ranker runs only
  in **shadow** each tick, to log the choice P4 would have made. Its output is never executed.
* **Recorded per decision tick:** the raw scores, the allowed mask, the policy choice, the shadow P4
  choice, the final action after the shield, any shield intervention (for example YAW_TO instead of
  TRANSLATE under L4) and the inference time.

## 6. Limits

All P2, P3 and P4 limits apply unchanged, including:

* §6 of the P4 contract: clearance ≥ 0.20 m, 0 collisions, tracking ≤ 0.30 m, response latency
  ≤ 0.25 s, setpoint validity;
* §6.1 of the P4 contract: the perception limits, measured in `p4_perception` as in P4, with compute
  p99 ≤ 0.050 s and max ≤ 0.090 s per frame of `DepthProcessor.poll`;
* the P2 dispatch budget of 0.050 s wall, from control-callback entry to publication, which aborts
  on breach.

P5 adds:

| Quantity | Limit | Notes |
|---|---|---|
| Policy inference per decision tick (feature build + forward + selection, wall, in the control callback) | p99 ≤ 0.005 s, max ≤ 0.010 s | Measured inside the existing 0.050 s P2 dispatch budget. Accepted P4 `TRANSIT` callbacks: p99 0.0113 s, max 0.0206 s |
| Policy failure ticks in Gate B, S and C flights | 0 | A single failure fails the run, even when handled safely |
| Policy participation | 100 % of progress-decision ticks execute the policy's choice (via TRANSLATE or YAW_TO under L4) | Verified from the ledger by recomputing the argmax from the recorded scores |
| Executed choices outside the allowed mask | 0 | Shield invariant |
| Deterministic fallback decisions | 0 | No fallback exists in acceptance builds |
| Checkpoint identity at start | equals the manifest SHA-256 | Otherwise refuse to start |

* **Reported, not limited:**
  * the disagreement rate with the shadow P4 choice;
  * shield interventions;
  * path length and detour ratio (path length / 8.0 m);
  * time to goal;
  * inference p50 and p95;
  * the depth-to-decision latency (snapshot age at decision);
  * control-publication gaps;
  * CPU and memory of `p5_control`;
  * real-time factor.

## 7. Scenarios

These are the P4 scenarios, with the same vehicle, worlds, obstacles, geofence, sensors, mission,
transit speed and outcome rules:

| Scenario | Required outcome |
|---|---|
| S1 clear path | `goal_reached`, no supervisor transitions, maximum lateral ≤ 0.30 m |
| S2 centre pillar | `goal_reached_with_detour`, landed |
| S3 offset pillar | `goal_reached_with_detour`, landed |
| S4 two pillars | `goal_reached_with_detour`, landed |
| S5 wall | `safe_refusal`: HOVER, `ABORT` (`hover_timeout`), landed through `COMPLETE` |
| S6 depth interruption | `safe_stop_on_sensor_loss`: the P4 contract §9.1 F1–F4 |
| S7 reset / stale pose | offline replay only, as in P4 |

Policy participation is required only where progress decisions occur:

* S1: the progress set is the straight path, and the policy must still select it.
* S2–S4: avoidance decisions.
* S5 and S6: decisions before the hover.

A safe refusal is never a goal arrival.

## 8. Independent evaluation

`p5_evaluate.py` composes the unchanged P4 evaluator functions with the P5 checks. Never trust P5
self-reports.

* **P4 checks reused:** P3 stream and association, RGB absence, MAVLink Policy A, renderer, MC
  causality, cleanup, P2 integrity, §6 truth safety, response latency, outcomes, S6 fault window and
  §6.1 performance.
* **P5 checks:** controller identity `p5_control`.
  * Checkpoint SHA-256 at start equals the manifest.
  * The policy ledger is complete for every TRANSIT tick.
  * The argmax recomputed from the recorded scores and mask equals the recorded choice, and the
    executed segment or heading equals that choice.
  * The §6 P5 limits.
  * The ledger scores are reproduced: the evaluator re-runs the frozen model on the recorded feature
    tensors and requires bit-identical scores.

## 9. Gates (operator authorization per gate, bound to the P5 manifest SHA-256)

Each gate is run once with no retries, stops at the first failure and needs an accepted predecessor
for the same manifest. P3 and P4 gate results grant no P5 credit.

| Gate | Content |
|---|---|
| **A** — ground | (1) P3 readiness unchanged. (2) P4 and P3 manifests verify. (3) P5 readiness under load: the full stack with `p5_control` in ground-diagnostic mode for the 30 s window, all §6 limits. (4) **Revision 2:** ground policy evaluation, G2, G4 and G6. On every recorded snapshot: the frozen input and model under the controller interpreter; finite, bit-identical scores; p99 ≤ 5 ms and max ≤ 10 ms; the mask equals P4's progress set; the P5 decision equals P4's (SCAN/HOLD when nothing is allowed); no policy-induced move into unknown or occupied space. (5) **Revision 2:** offline flight-snapshot policy-failure replay on an exactly reproduced TRANSIT tick of the accepted P4 S2 flight evidence. Missing checkpoint, hash mismatch, non-finite scores, timeout, invalid output size and invalid selection each give HOLD, HOLD, then ABORT `policy_unavailable`. One live start with a mismatched checkpoint must refuse before any node exists; its exit 1 is accepted only in that declared scope. (6) A single PX4 input owner; graceful cleanup. Full text: [revision 2](v3_sim_p5_contract_revision2.md) |
| **B** — smoke | S2: genuine frozen-policy participation (§6 limits), shield-safe execution, detour, goal and landing. Replays after Gate B are additional evidence only |
| **S** — safety | S1, S5, S6 in order |
| **C** — qualification | S2, S3, S4 in order, 3/3 |

P5 is complete only with A, B, S and C accepted for the same P5 manifest.

## 10. Manifest

The P5 manifest binds:

* all P5 sources and configs;
* the P3 manifest `1e205fec…` and the P4 manifest `c110bd30…`, both re-verified at freeze and at run
  start;
* the P5 build receipt;
* the model artifact SHA-256, architecture, schema hash and metadata;
* the training-dataset manifest hash and the offline readiness report hash;
* the NumPy version of the controller runtime;
* the scenario and evaluator sources and the shield configuration;
* Policy A content plus a per-candidate approval;
* the renderer, D3D12 profile, QGC ELF, MC build and runtime receipts, as in P4;
* the gate definitions.

Any change to weights, features, normalization or selection after freeze needs a new candidate.

## 11. Decisions requiring operator approval (recommendation first)

1. **D1 — Training method:**
   * **(a) Recommended:** privileged-oracle cost-to-go supervision with DAgger. Labels are
     ground-truth shortest safe path costs, used at training time only.
   * (b) Behaviour cloning of the P4 ranker. This is the simplest option but largely re-learns P4.
   * (c) Rollout-return supervision. Its labels cost tens of hours of synthetic rollouts.
2. **D2 — Failure handling:** fail closed (HOLD, then `ABORT` after 3 failures) with no P4
   fallback in acceptance runs (recommended). The alternative is a declared, separately counted P4
   fallback.
3. **D3 — Policy scope:**
   * **Recommended:** the policy ranks progress candidates in every state where P4 selects,
     `NORMAL` included, so S1 tests that it flies straight when the path is clear. `SCAN` targets and
     L4/L5 authority stay deterministic.
   * **Alternative:** the policy ranks only in the avoidance states (`OBSTACLE_DETECTED`, `AVOIDING`,
     `RESUME`, `HOVER`), and `NORMAL` keeps P4's selection.
4. **D4 — Artifact storage:** commit the content-addressed `.npz` in the repository when it is
   ≤ 1 MB (recommended). The alternative is an external store with only the hash committed.
5. **D5 — Integration:** P5 replicas of `plan`, `Supervisor.step` and the controller node that
   differ only in the selection hook. Each must have bit-identical parity tests against P4 with the
   P4 ranker plugged in (recommended). Runtime patching of P4 modules is rejected.
6. **D6 — Participation and disagreement:** 100 % participation, 0 failures and 0 fallbacks are
   limits; the disagreement rate with P4 is reported only (recommended).
7. **D7 — Inference limits:** p99 ≤ 0.005 s, max ≤ 0.010 s, failure above 0.020 s, inside the
   unchanged 0.050 s P2 budget.
8. **D8 — Offline readiness bar:** ≥ 200 held-out layouts, 0 shield violations, goal success no more
   than 5 points below P4.
9. **D9 — Gate structure:** the P4 structure (A/B/S/C), with Gate A adding the model-integrity and
   policy-failure fixtures.
