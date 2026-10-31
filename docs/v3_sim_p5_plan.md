# v3-SIM P5 — fixed-weight policy substitution: plan and integration facts

**Status: [P5 contract](v3_sim_p5_contract.md) APPROVED (revision 1, 2026-10-31, D1–D9 as recommended).**
Offline milestones M0–M2 are implemented (§13). M3 (training) needs a separate authorization.
Nothing is trained, frozen, launched or pushed.

Baseline, verified on 2026-10-31:

* HEAD `5c1776221e26c0ee09a22e543785c4311cf7aff8` on `v3/gwm-uav-c2-agentops-planning`; local equals
  `origin`.
* P3 COMPLETE (`1e205fec…`) and P4 COMPLETE (`c110bd30…`, Gates A/B/S/C, qualification 3/3), both
  re-verified the same day.

## 1. Verified P4 integration interfaces and manifest boundaries

Everything in this section was read from the accepted sources.

### 1.1 Manifest boundaries

* **P3:** `p3_manifest.source_files()` hashes every file under
  `simulation/px4_gazebo/{scripts,validation,configs}`, `ros2_ws/src/{gwm_px4_control,gwm_sensor_adapter}`
  and `docs/evidence/v3_sim_p2_summary.json`.
* **P4:** `p4_manifest.SOURCE_FOLDERS` = `configs`, `scripts`, `validation`,
  `ros2_ws/src/gwm_obstacle_avoidance`, all relative to `simulation/px4_gazebo_p4/`.
  * The P4 build is bound by `state/p4-built.json` (package `c4bd5a55…`, install
    `~/uav_autonomy/p4_ws/c4bd5a55…/install`).
  * `simulation/px4_gazebo_p4/README.md` is outside the glob.
* **P5:** **`simulation/px4_gazebo_p5/` lies outside both**, so P5 cannot invalidate either manifest.
  P5 must not write `state/p4-built.json`; it gets its own `state/p5-built.json`.

### 1.2 Where the decision is made

* **`planner.plan(grid, traversed, ctx, settings, envelope, max_age)`** is pure.
  1. It calls `evaluate_candidates`, which returns 26 rows (k = −6…6 × L ∈ {0.75, 1.5}), each with
     `valid`, `reason`, `unknown_only`, `unknown_cells`, `progress`, `end`, `heading`, and, when
     valid, `clearance`, `deviation`, `turn` and `score`.
  2. It filters the progress set (valid ∧ (progress > 0.05 m ∨ reaches goal)) and calls
     **`select(progress, settings, resolution)`**: the weighted score, ties within 0.06, then
     `tie_key`. **This is the only step P5 replaces.**
  3. It applies L4 (heading within ±25° → TRANSLATE, otherwise YAW_TO if `yaw_permitted`, otherwise
     HOLD) and, with no progress candidate, L6 `SCAN` / HOLD.
* **`supervisor.Supervisor.step(inputs)`** calls the module-level `planner.plan` on every tick and
  executes the decision in `NORMAL`, `OBSTACLE_DETECTED`, `AVOIDING` and `RESUME`. It owns staleness,
  clearance, hover and transit timeouts.
  * Because the call is module-level, P5 **cannot inject a ranker without changing P4**. P5
    therefore carries replicas of `plan` and `step` that differ only in the selection hook, and
    proves parity with P4 (§6.2).
* **`mission.P4Mission`** creates `supervisor.Supervisor(self.p4, sim)` at the `MAP_SCAN` → `TRANSIT`
  transition. It overrides `transition()` already (revision 5 abort landing). `P5Mission(P4Mission)`
  can install its supervisor in its own `transition()` override, when `next_state == "TRANSIT"`.
* **`control_node.main`** hard-codes `P4Mission` and the controller identity `p4_control`. P5 needs
  its own node, `p5_control`, a replica of the P4 node with `P5Mission`.
  * It must keep the P2 trace spans, `bag_write`, `graph_check`, BEST_EFFORT map and health
    subscriptions and the dispatch guard.
  * It is checked by the existing ROS-harness tests ported to P5.
* **`p4_runner.P4Runner`** subclasses the P3 runner, which it loads with `importlib`.
  * It launches `p4_perception` and `p4_control` from the P4 install, writes the run-local
    `p4_avoidance.yaml` with `enabled: true`, generates worlds, runs the truth observer and injects
    the S6 fault.
  * `P5Runner(P4Runner)`, loaded the same way, overrides only the controller command (`p5_control`,
    the model path and its SHA-256) and adds the P5 configs. Perception stays the unchanged P4 node.
* **`p4_evaluate.evaluate`** checks `controller == "p4_control"`, so P5 cannot call it as a whole.
  `p5_evaluate` composes its functions instead: `performance`, `p2_integrity`, `safety`,
  `response_latency`, `outcome`, `fault_window`, `fixture`, `fault_replay`, `native_commands`, `mc`,
  plus the P3 helpers. It adds the P5 checks.

### 1.3 What the accepted P4 compute limits measure

* **P4 contract §6.1 compute** (p99 ≤ 0.050 s, max ≤ 0.090 s) is `perception_node.poll`'s thread
  wall time in the **`p4_perception` process**: `DepthProcessor.poll`, including P3 `observe`,
  `extract_scan`, `integrate` and `grid.encode`.
  * It does **not** include planning.
  * P5 leaves perception unchanged, so these limits keep applying as in P4.
* **Planning and selection** run inside the controller's 20 Hz **control callback**. They are bounded
  by the P2 `DispatchGuard` (0.050 s wall from callback entry to publication, an abort on breach) and
  by P2 `verify_timing` (late publication).
* **Measured in the accepted P4 flights** (7 flights, 5,772 `TRANSIT` ticks):

  | Measure | p50 | p99 | Max |
  |---|---|---|---|
  | `mission_tick` | 4.3 ms | 9.3 ms | 18.3 ms |
  | Whole control callback | — | 11.3 ms | 20.6 ms |

* **P5 budget:** inference p99 ≤ 5 ms and max ≤ 10 ms keeps the worst callback around 31 ms, inside
  the unchanged 50 ms budget.
* **The critical path:** depth acquisition → perception (bounded by §6.1 and the snapshot age
  ≤ 0.25 s) → map message → control tick (candidates, P5 inference, shield and supervisor) →
  publication (P2 budget). Response latency ≤ 0.25 s stays the end-to-end check.

## 2. Model inventory

| Artifact | Architecture | Format, size, SHA-256 | Provenance | Inputs → outputs | Compatible? |
|---|---|---|---|---|---|
| `checkpoints/world_model_baseline.pt` (gitignored) | `BaselineWorldModel`: MLP 11→128→128→8 (`src/rl/world_model_baseline.py`) | PyTorch dict (model and optimizer state, config, `final_loss` 0.094), 237 KB, `fdf32437…be36` | 2026-05-31, `scripts/train_world_model.py` on the mock environment, 100 updates | mock state (8) + action (3) → state delta | **No.** A dynamics model, not a policy; no occupancy or candidate input; trained on the mock environment |
| `checkpoints/world_model_latent.pt` (gitignored) | latent world model (encoder 8→64→32; dynamics 35→128→128…) | PyTorch dict, 384 KB, `1a042e04…6f38`, `final_loss` 6.44 | as above | mock state → latent dynamics | **No**, for the same reasons |
| `checkpoints/_test*/…pt` | test copies of the two above | `f346575c…`, `754ebe79…` | test runs, 2026-10-25 | — | **No** |
| `src/generated_world_model/trajectory_scorer.py`, `src/rl/mpc_planner.py` | analytic scorers (weighted costs) | code, no weights | mock-first framework | trajectories → scalar cost | **No.** Not learned, and a different observation model |

**There is no compatible pretrained policy checkpoint.** P5 needs a new, purpose-trained model. It
must not reuse these artifacts, use random weights as a "policy", or claim readiness before training
and the offline readiness check (contract §4.2).

**Runtimes:**

| Environment | Python | NumPy | torch | onnxruntime | Role |
|---|---|---|---|---|---|
| WSL controller (ROS Jazzy `/usr/bin/python3`) | 3.12.3 | 1.26.4 | no | no | flight runtime |
| `venv-px4` | 3.12.3 | 2.5.3 | no | no | — |
| Windows | 3.13.5 | 2.3.4 | 2.13.0+cpu (no CUDA) | no | offline training |

The host is an i7-13700HX: WSL sees 24 logical CPUs and 15 GB of memory.

## 3. Recommended model and runtime

* **The model:** a shared per-slot MLP ranker (`p5-ranker-mlp-v1`, contract §4): 46 features per slot
  → 64 → 64 → 1, about 7.2 k parameters.
* **Why this design:**
  * It scores each candidate slot with the same weights, so it ranks a fixed, ordered candidate set
    without changing its geometry.
  * Its inputs are only onboard data that P4 already computes, plus a 13-ray polar occupancy profile
    from the validated grid.
  * Its output can be masked, so it can never select an invalid slot.
* **Runtime:** a pure NumPy float64 forward pass in the controller process, on CPU.
  * It adds no dependency to the flight runtime (NumPy is already there) and is bit-reproducible.
  * It avoids GPU contention with WSL2 D3D12 rendering.
* **Interface benchmark (WSL, random weights, cost only, no policy claim):** forward pass plus
  masked argmax for 26 slots took p50 0.08 ms, p99 0.22 ms, max 2.3 ms. Feature construction is
  measured in M2. Accelerators are not considered unless measurements show a need.
* **Training-to-runtime handoff:** training may use PyTorch on Windows. The exported `.npz` is
  canonical, and a parity test requires the NumPy forward pass to match the training-side forward
  pass within 1e-9 on recorded tensors.

## 4. Offline training procedure (operator decision D1a; needs separate authorization)

1. **Environment:** the synthetic closed loop.
   * The P4 production `perception` (extract_scan / integrate), `planner` and `supervisor`, imported
     read-only.
   * An independent closed-form depth renderer, with the same camera, extrinsic, 0.1 m grid,
     geofence, 8 m goal and 0.20 m/s transit speed.
   * A first-order PX4 horizontal position-loop model (`MPC_XY_P` 0.95). This is the same model
     that predicted the revision 4 tracking result: 0.205 m predicted, 0.23–0.26 m measured in SITL.
2. **Layouts:** procedurally generated from recorded seeds.
   * 1–4 static boxes: pillars 0.2–0.6 m, walls with and without gaps.
   * The start and goal discs are kept free.
   * The **acceptance-layout exclusion** of contract §4.1 is applied and tested.
   * Splits: train ≥ 1,000 layouts and held-out ≥ 200, from disjoint seed ranges.
3. **Privileged oracle (training time only):**
   * Per layout: a Dijkstra cost-to-go field to the goal on the **true** occupancy, inflated by
     r_infl 1.0 m, on the 0.1 m grid.
   * The label for allowed slot *i* is c_i = segment length + cost-to-go(endpoint) + λ·max(0, 0.6 −
     true clearance).
   * The target is softmax(−c/τ) over the allowed slots.
   * Ground truth never enters the onboard features.
4. **Learning:** listwise cross-entropy between softmax(score) and the target over the allowed slots,
   with weight decay, fixed seeds and deterministic data order.
5. **DAgger:**
   * Round 0 collects states by rolling out the P4 ranker.
   * Rounds 1–2 roll out the current learned policy. The shield is always on, and labels come from
     the oracle.
   * Rounds aggregate. 3 rounds is the planned budget.
   * The estimated cost is a few CPU-hours: the oracle field is computed once per layout, and labels
     are O(1) lookups.
6. **Readiness (contract §4.2):** ≥ 200 held-out layouts, 0 shield violations, goal success no more
   than 5 points below P4. The report covers time-to-goal, path length, clearance and disagreement.
7. **Export:** `models/p5-ranker-<sha256>.npz` with metadata covering the architecture, schema hash,
   `trained: true`, dataset manifest hash, seeds, code commit and readiness report hash.
   * The dataset is reproducible from the generator and seeds, and only its manifest (per-file
     SHA-256) is kept.

## 5. Feature construction (implementation notes for contract §3)

* **Candidate features** come directly from the P4 rows: `k`, the lengths, `progress`, `deviation`,
  `turn`, quantized `clearance`, validity and progress flags. The observable flag and reaches-goal use
  P4's own formulas.
* **The polar profile:** for each of the 13 headings, march the 0.1 m grid from the target in 0.05 m
  steps to 4.0 m, recording the first `OCCUPIED` and the first non-`FREE` distance. This uses
  `Grid.index` semantics, with traversed cells counting as FREE as in L2.
* **Validation:** the feature builder is a pure function, so identical inputs give identical
  tensors. Each run records the tensor so the evaluator can re-run the model (contract §8).

## 6. Safety shield and controller ownership

### 6.1 Runtime structure (`gwm_policy_control` package)

| Module | Role |
|---|---|
| `policy_runtime.py` | load, verify (SHA-256, architecture, shapes, schema, `trained`); read-only weights; NumPy forward; timing |
| `observation_adapter.py` | P4 rows and context → `(26, 46)` tensor, plus the allowed mask |
| `candidate_ranker.py` | masked argmax, P4 tie order, failure classification; a shadow P4 `select` |
| `decision.py` | `plan_with_ranker`: a replica of P4 `plan` with the selection hook |
| `safety_shield.py` | P5 supervisor: a replica of P4 `Supervisor.step` calling `plan_with_ranker`, plus the policy-failure counter (3 consecutive → `ABORT` `policy_unavailable`) |
| `mission.py` | `P5Mission(P4Mission)`, which installs the P5 supervisor at `TRANSIT` |
| `control_node.py` | `p5_control`, a replica of the P4 node with `P5Mission`, the policy ledger, and the model path and SHA-256 arguments |

### 6.2 Parity obligations (operator decision D5)

* **Planner parity:** with the P4 ranker plugged in, `plan_with_ranker` must equal P4 `plan`, field by
  field, on every synthetic closed-loop state for S1–S6 and on randomized grids.
* **Supervisor parity:** with the P4 ranker plugged in, the P5 supervisor must reproduce P4
  `Supervisor` transitions and decisions bit for bit, over the closed-loop scenarios and the recorded
  decision inputs.
* **Controller parity:** `p5_control` must pass the ROS-harness tests (`verify_timing`, BEST_EFFORT
  delivery, graph-check spans), plus a check that the policy runtime has no publisher.

### 6.3 Failure behaviour

* **Fail closed:** HOLD on the failing tick (the target and yaw are frozen), and `ABORT` after 3
  consecutive failures, landing through revision 5. A start-up integrity failure refuses to start.
* **No fallback:** acceptance builds have no P4 fallback path. The shadow P4 choice is logged only.

## 7. Repository layout (adapted to the actual tree)

```text
simulation/px4_gazebo_p5/
├── README.md                          (outside the P5 manifest, like P4)
├── configs/  p5_policy.yaml  p5_validation.yaml  p5_model_schema.json
├── models/   p5-ranker-<sha256>.npz  (committed only if ≤ 1 MB, decision D4)
├── training/ layouts.py  oracle.py  dataset.py  train.py  readiness.py  synthetic_render.py
├── ros2_ws/src/gwm_policy_control/gwm_policy_control/
│             policy_runtime.py  observation_adapter.py  candidate_ranker.py  decision.py
│             safety_shield.py  mission.py  control_node.py  contracts.py
├── scripts/  build_p5.sh  p5_build.py  p5_manifest.py  p5_runner.py  run_p5_gates.py  verify_p5_evidence.sh
└── validation/ p5_evaluate.py  p5_policy_eval.py  p5_comparison.py
tests/        test_px4_gazebo_p5_*.py   (the repository test suite, outside the manifests)
```

The P5 manifest folders are `configs`, `models`, `training`, `scripts`, `validation` and
`ros2_ws/src/gwm_policy_control`.

## 8. Milestones (offline; each reruns the P2, P3, P4 and P5 suites, and P3/P4 must still verify)

| # | Milestone | Done when |
|---|---|---|
| M0 | Skeleton, configs, schema, `p5_manifest` (binding P3 and P4 and refusing if either does not verify), guard tests that P3/P4 sources are unchanged | P3 still verifies to `1e205fec` and P4 to `c110bd30` |
| M1 | `policy_runtime` and `observation_adapter`, using an **untrained** interface model (`trained: false`, test-only) | load, hash-mismatch rejection, schema, finiteness, determinism, read-only weights, no hidden state, timing records; the controller refuses `trained: false` in authorized scopes |
| M2 | `candidate_ranker`, `decision` and the P5 supervisor | planner and supervisor parity with the P4 ranker (§6.2); masking; fail-closed paths; feature-build timing measured |
| M3 | **Separate authorization:** layouts, oracle, dataset, training, DAgger, readiness report, export | contract §4.2 met; artifact and metadata hashed; acceptance-layout exclusion test passes |
| M4 | `P5Mission` and `p5_control`; ROS-harness parity | `verify_timing` passes on a P5 trace; no publisher in the runtime; policy ledger complete |
| M5 | `p5_runner`, `run_p5_gates`, `p5_evaluate`, policy-failure replay fixtures | synthetic runs exercise every P5 pass and fail rule; reused P4 checks still applied |
| M6 | Offline paired comparison on held-out layouts (`p5_comparison.py`) | the report described in §9 |
| M7 | `build_p5.sh` receipt, freeze tooling, full Linux and Windows regression | all suites pass; P3/P4 verify; dry P5 candidate binds the model SHA-256 |

Then the operator steps: Policy A approval for the P5 commit, the freeze, and Gates A, B, S and C,
one authorization at a time.

## 9. P4 vs P5 comparison methodology

* **Held constant:** the simulator and vehicle, worlds and obstacles, perception, candidate
  generation, shield and supervisor, mission and geofence, controller, PX4 configuration and Policy A.
  Only the selection step differs.
* **Offline, large sample (M6):** both rankers run over the same ≥ 200 held-out layouts, with
  identical seeds and the same shield. The comparison reports:
  * paired goal success, time-to-goal, path length and detour ratio, minimum clearance,
    hover/abort counts;
  * per-decision disagreement;
  * paired differences with bootstrap intervals.
  It is not acceptance evidence.
* **SITL, small sample (Gates B/S/C):** each P5 flight is compared with the accepted P4 flight of the
  same scenario, using the retained `c110bd30` evidence: outcome, clearance, tracking, path length,
  time-to-goal and decision lineage.
  * The shadow P4 choice logged at every P5 tick gives a per-decision comparison under identical
    inputs.
  * The host load (CPU, RTF) and scenario identity are recorded.
* **Separation:** safety outcomes (collisions, clearance, refusals) are reported separately from
  decision quality (path length, time). P5 claims correct substitution only. A superiority claim
  needs P7's paired batches.

## 10. Risks

* **Mostly agreeing with P4.** The learned policy may agree with P4 nearly everywhere, which makes
  participation real but differences small. Disagreement is reported (D6). A low rate is a finding,
  not a failure.
* **Sim-to-SITL gap.** The synthetic training loop omits PX4 dynamics beyond the first-order model.
  The shield bounds the risk, because any selected candidate is P4-valid.
* **Replica drift.** The `plan` and `step` replicas could drift from P4. The bit-parity tests (§6.2)
  are mandatory before any freeze.
* **Timing.** Feature construction adds work to the control tick. It is measured in M2 against the
  5 ms budget before any runtime.

## 11. Gate authorization boundaries

These are as in contract §9:

* each gate needs an explicit operator authorization bound to the P5 manifest SHA-256;
* each gate runs once, stops at the first failure and is never retried;
* predecessors: B needs A, S needs B, and C needs S;
* P4 credit does not transfer.

No simulator runs before Gate A is authorized.

## 12. Decisions requiring operator approval

These are listed with recommendations in contract §11, D1–D9:

1. the training method;
2. failure handling and fallback;
3. the policy scope;
4. artifact storage;
5. the replica-and-parity integration;
6. the participation and disagreement limits;
7. the inference limits;
8. the offline readiness bar;
9. the gate structure.

M3 (training) additionally needs its own authorization after the contract is approved.

## 13. Implementation status: M0–M2 (offline, 2026-10-31)

* **M0:** the `simulation/px4_gazebo_p5/` skeleton.
  * Configs: `p5_policy.yaml` (disabled, no fallback, no bound model), `p5_validation.yaml` (the
    P3/P4 baselines, the §6 limits, the gates) and the frozen `p5_model_schema.json`.
  * The `gwm_policy_control` package metadata.
  * `p5_manifest.py`: P5 source identity, schema hash and live P3/P4 baseline verification.
  * Guard tests: the P3/P4 manifest-bound sources are unchanged since the accepted P4 commit, and
    P3/P4 still verify to `1e205fec…` / `c110bd30…`.
* **M1:**
  * `policy_runtime.py`: a deterministic `.npz` artifact; loading checks the SHA-256, architecture,
    schema hash, `trained`, shapes and finite weights; the weights are read-only; the float64 forward
    pass is stateless.
  * `observation_adapter.py`: the `(26, 46)` tensor and allowed mask, plus the polar occupancy
    profile.
  * Interface models (`trained: false`, fixed-seed random weights) are used in tests only and are
    refused when a trained model is required.
* **M2:**
  * `candidate_ranker.py`: masked argmax with the P4 tie order, classified fail-closed records and the
    shadow P4 choice.
  * `decision.py`: the `plan` replica with the selection hook.
  * `safety_shield.py`: the P5 supervisor (the P4 `step` replica plus the 3-failure
    `policy_unavailable` rule).
* **Parity:**
  * The replica equals P4 `plan` on 40 structured random worlds, covering TRANSLATE, YAW_TO, SCAN and
    HOLD.
  * A source-level guard confirms the replica differs from P4 only in the hook.
  * The P5 supervisor with the P4 ranker matches P4 tick for tick in lockstep closed loops for S2,
    S4, S5 and S6.
  * An untrained interface policy driving S2 still executes only P4-valid candidates, with truth
    clearance ≥ 0.20 m.
* **Offline per-tick policy cost** (feature build, forward pass and selection; interface model;
  closed-loop S2/S4 states, about 1,066 ticks each; no simulator load):

  | Scenario | p50 | p95 | p99 | Max |
  |---|---|---|---|---|
  | S2 | 0.48 ms | 0.94 ms | 1.42 ms | **7.25 ms** |
  | S4 | 0.47 ms | 0.99 ms | 1.39 ms | 2.08 ms |

  The limits are p99 ≤ 5 ms and max ≤ 10 ms. The 7.25 ms S2 outlier is inside the maximum but is
  the margin to watch under SITL load.
* **Next:** M3 (training) awaits a separate operator authorization. M4–M7 (mission, controller,
  runner, evaluator, comparison, build) can be built against interface models first, but nothing can
  be frozen without a trained model.
