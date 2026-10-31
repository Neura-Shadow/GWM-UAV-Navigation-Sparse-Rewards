# v3-SIM P5 — fixed-weight policy substitution: plan and integration facts

**Status: [P5 contract](v3_sim_p5_contract.md) APPROVED (revision 1, 2026-10-31, D1–D9 as recommended).**
Offline milestones M0–M7 are complete (§13). M3 was authorized and run offline on 2026-10-31.
It produced candidate artifact `p5-ranker-ee006a80…`, which meets the contract §4.2 readiness bar.
The operator accepted that candidate as is, with its known yaw-oscillation limitation (§13.2).
M4 (`P5Mission`, the policy ledger and `p5_control`) and M5 (runner, gates, evaluator; contract
revision 2) followed the same day (§13.3, §13.4).
Nothing is frozen, launched or pushed.

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
├── training/ p5_paths.py  p5_render.py  p5_episode.py  p5_layouts.py  p5_oracle.py  p5_dataset.py
│             p5_train.py  p5_readiness.py  p5_m3.py
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

## 13. Implementation status: M0–M7 (offline, 2026-10-31)

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
* **M3** (authorized 2026-10-31; offline, CPU only, no simulator): see §13.1–13.2.
* **M4** (authorized 2026-10-31; offline, no simulator): see §13.3.
* **M5** (authorized 2026-10-31; offline, no simulator): see §13.4.
* **M6** (authorized 2026-10-31; offline): see §13.5.
* **M7** (authorized 2026-10-31; offline): see §13.6.
* **Next:** operator steps only (§13.6): Policy A approval, freeze, then Gates A, B, S and C.
  Freezing, Policy A approval and the gates come after M7.

### 13.1 M3 tooling (`training/`, commit `13c7dde`)

| Module | Role |
|---|---|
| `p5_render.py` | Independent depth renderer. It is bit-equal to the test renderer and also provides the true surface distance and clearance |
| `p5_episode.py` | Synthetic closed loop: P4 production perception, planner and supervisor, MAP_SCAN, the first-order PX4 loop (`MPC_XY_P` 0.95) at 20 Hz, and the P4Mission goal rule |
| `p5_layouts.py` | Seeded layout families (pillars 0.40, wall with gap 0.20, partial wall 0.15, mixed 0.25), the start/goal discs, strict inflated feasibility, and the contract §4.1 acceptance exclusion |
| `p5_oracle.py` | Privileged cost-to-go field and slot labels (training time only) |
| `p5_dataset.py` | DAgger collection through the production `PolicyRanker` (or P4 L3 in round 0), with the shield always on |
| `p5_train.py` | NumPy listwise cross-entropy training (AdamW with a cosine learning rate). The epoch is selected by validation regret |
| `p5_readiness.py` | Contract §4.2 report: P4 `Supervisor` vs `PolicySupervisor` + `PolicyRanker` on each held-out layout |
| `p5_m3.py` | Run driver and export. It refuses a dirty P5 tree, and export refuses a run that is not ready or whose weights hash differs |

**As-built refinements of §4 (all training time only; ground truth never enters the features):**

* **Oracle field:** 16-connected Dijkstra on the 0.1 m lattice, over the geofence shrunk by r_infl
  (x 0–8.5 m, |y| ≤ 2 m).
  * Cells inside the r_infl band cost ×10, so they are finite but penalized.
  * Cells within R_body + 0.10 m of a true box are impassable.
  * Layout admission uses the strict field, which has no band.
* **Label:** c = segment length (clipped at the goal) + V(endpoint) + 10 · max(0, 0.6 − true segment
  clearance) + (speed / yaw rate) · max(0, |turn| − 25°). The last term is the L4 yaw time, as
  metres at transit speed. τ = 0.10 m.
* **Sampling:** every 8th policy tick, with a phase that changes per layout and round.
* **Numerical environment:**
  * Collection, readiness and export use the controller runtime interpreter (`/usr/bin/python3`,
    NumPy 1.26.4, `OPENBLAS_NUM_THREADS=1`).
  * Gradient training runs in a separate NumPy 2.5.3 / OpenBLAS interpreter for speed.
  * Each round checks forward parity between the two on 256 recorded tensors (limit 1e-9).
  * Platform: WSL Ubuntu 24.04 on the operator workstation, CPU only. Contract §4.1 names Windows
    CPU PyTorch or NumPy; this run used NumPy under WSL on the same host.

### 13.2 M3 run (`p5_m3.py run`, clean commit `13c7dde`, 20 workers, 1 h 55 min)

**Layouts** (disjoint seed ranges of 1,000,000 per split):

| Split | Seed base | Tried | Admitted | Rejected: start/goal disc | Rejected: unreachable when inflated | Rejected: acceptance match |
|---|---|---|---|---|---|---|
| train | 1,000,000 | 1,609 | 1,200 | 300 | 108 | 1 |
| validation | 2,000,000 | 204 | 150 | 44 | 10 | 0 |
| held-out | 3,000,000 | 411 | 300 | 80 | 30 | 1 |

**DAgger** (0 policy failures and 0 executions outside the mask in every collection):

| Round | Driver | Train goal reached | New samples (train / validation) | Aggregated train states | Selected epoch | Validation regret | Oracle top-1 | Forward parity |
|---|---|---|---|---|---|---|---|---|
| 0 | P4 L3 | 895 / 1,200 | 116,041 / 14,524 | 116,041 | 34 | 0.0246 m | 0.709 | 1.78e-14 |
| 1 | policy, round 0 | 986 / 1,200 | 119,366 / 15,142 | 235,407 | 30 | 0.0204 m | 0.701 | 1.42e-14 |
| 2 | policy, round 1 | 972 / 1,200 | 122,630 / 15,048 | 358,037 | 7 | 0.0204 m | 0.666 | 1.07e-14 |

**Readiness on 300 held-out layouts (contract §4.2: READY):**

| Metric | P4 ranker | Policy |
|---|---|---|
| Goal reached | 215 (71.7 %) | **236 (78.7 %)**; the drop is −7.0 points against a limit of ≤ 5 |
| Shield violations (outside mask + executed mismatch) | — | **0** |
| Collisions (truth) | 0 | 0 |
| Minimum / mean minimum truth clearance | 0.544 / 0.698 m | 0.547 / 0.685 m |
| Mean time to goal / path (reached) | 42.0 s / 8.06 m | 41.7 s / 8.07 m |
| Other outcomes | 85 `hover_timeout` | 60 `hover_timeout`, 4 `transit_deadline` |

* **Paired:** both reached 210; P4 only 5; policy only 26. On the layouts both reached, the policy
  took 0.51 s less and its path was 0.02 m shorter.
* **By family (P4 / policy):**
  * pillars 67 / 76 of 100;
  * partial wall 41 / 49 of 54;
  * wall with gap 80 / 82 of 85;
  * mixed 27 / 29 of 61.
* **Decisions:**
  * 241,224 policy invocations; 0 failures.
  * Disagreement with the shadow P4 choice: 0.694, mostly segment length 0.75 m vs 1.5 m.
  * Heading disagreement: 0.246.
* Most P4 failures in both training and held-out runs are hover timeouts at wall local minima, a
  structural limit of the P4 ranker.

**Known limitation (found after readiness and not tuned; held-out results are never used for
tuning):**

* All 4 `transit_deadline` aborts are yaw-oscillation livelocks.
  * The policy keeps preferring candidates outside the ±25° cone, so L4 turns each choice into
    YAW_TO: 2,566–2,998 of 3,000 ticks.
  * In one case (`held_out-3000212`, two pillars with a 1.3 m gap) the vehicle moved only 0.02 m.
* These aborts are safe: truth clearance was ≥ 0.65 m and the existing 150 s transit deadline ended
  them. P4 also failed all 4 layouts, by hover timeout.
* The cost is a different abort path, and a much later one, than P4's 20 s hover timeout.
* This matters for S5, which requires `ABORT` (`hover_timeout`), wherever progress candidates exist
  before the hover.
* Any change, such as anti-oscillation training or a yaw hysteresis, would be a new candidate.
  Evaluating it would need a fresh held-out seed range.
* **Operator decision (2026-10-31): candidate accepted as is.**
  * The limitation is carried as a known risk, mainly for S5 in Gate S.
  * The S5 outcome rule is unchanged: a `transit_deadline` abort there fails the gate.
  * No retraining or tuning was done.

**Artifact (operator decision D4: committed, ≤ 1 MB):**

* `models/p5-ranker-ee006a805ba9c327a29f8f97ef6a13d0608b8459b37f0ca873a5155c59eaf73c.npz`
  (60,773 bytes).
* Weights SHA-256: `95a9c9b3…`; schema `9a651447…`; `trained: true`.
* Three companion files: `.dataset_manifest.json` (per-file SHA-256 of the 52 regenerable dataset
  files, which are not committed), `.training_report.json` and `.readiness_report.json`. The SHA-256
  of each is embedded in the artifact metadata.
* `FrozenPolicy.load(…, require_trained=True)` accepts the artifact.
* The repository tests check:
  * content addressing and size;
  * report hashes;
  * readiness;
  * parity;
  * the train count;
  * that the held-out layouts are disjoint and match no acceptance layout.

**Informational inference cost with the exported artifact** (runtime interpreter, synthetic closed
loop, 10 held-out layouts, 8,097 invocations, no SITL load): p50 0.49 ms, p95 1.02 ms, p99 1.55 ms,
max 5.84 ms. The limits are p99 ≤ 5 ms and max ≤ 10 ms. This is not acceptance evidence.

### 13.3 M4: `P5Mission`, the policy ledger and `p5_control` (offline, 2026-10-31)

| Module | Role |
|---|---|
| `mission.py` | `P5Mission(P4Mission)` overrides only `transition` and `_transit` and adds the ledger bookkeeping. At MAP_SCAN → TRANSIT it replaces P4's `Supervisor`, at the same simulation time and before its first step, with `LedgerSupervisor`. That class is the unchanged `PolicySupervisor`; it only keeps its last output |
| `policy_ledger.py` | One record per TRANSIT supervisor step, `p5-policy-ledger-v1`. An invoked record holds the exact `(26, 46)` feature tensor (little-endian float64, base64), the raw scores, the allowed mask, the policy and shadow P4 slots, the failure, the inference time, the executed action and the shield intervention (`yaw_to_l4`, `hold:<reason>`, `policy_failure_hold`) |
| `control_node.py` | `p5_control`, a replica of `p4_control`. It reuses P4's `outcome` and event topic |

**How `p5_control` differs from `p4_control`:**

* **Start-up integrity (contract §4):** before ROS is imported, it checks:
  * the run configuration's checkpoint SHA-256 against `--model-sha256`;
  * the frozen schema hash against `--schema-sha256`;
  * `FrozenPolicy.load` (file SHA-256, architecture, shapes, schema hash, `trained: true`).

  Any mismatch refuses to start, before a node, publisher or run file exists.
* **Mission:** it runs `P5Mission`.
* **Evidence:**
  * a `p5_policy_verified` event;
  * the ledger in `p5/policy-ledger.jsonl`;
  * a `p5` block in `controller-result.json`.
* **Ledger write:** after the dispatch guard, in a `policy_ledger_write` trace span. A write error
  becomes an evidence error.
* **Unchanged from P4:** the `p4_decision` events, `p4/decisions.jsonl` and the result keys that
  P4's evaluator functions read.

**Tests** (`test_px4_gazebo_p5_mission.py` offline; `test_px4_gazebo_p5_ros_harness.py` under real
rclpy):

* **Mission parity:** with P4's ranker plugged in, `P5Mission` equals `P4Mission` tick for tick
  (actions, events, transitions) from MAP_SCAN to GOAL_HOVER.
* **Trained policy:** `held_out-3000002` reaches GOAL_HOVER with ideal tracking. No acceptance
  scenario is used.
  * 801 ledger records for 801 `p4_decision` events, matched by time, action and state.
  * Every invoked record's scores are reproduced bit for bit from its stored features.
  * The masked argmax equals the recorded choice.
  * The executed segment or heading equals the choice.
* **Policy failures:** three consecutive failures give HOLD, HOLD, then `policy_unavailable` and one
  LAND through revision 5, with the target and yaw frozen throughout.
* **Start-up refusals,** all before ROS import: the configuration's SHA-256 differs; the schema hash
  differs; the file differs; the file is missing; the model is untrained; P5 is disabled. The
  exported checkpoint passes every check.
* **Source drift guard:** `p5_control.main` differs from `p4_control.main` only in the declared
  lines.
* **ROS harness, P5:**
  * the P2 `verify_timing` passes on a `p5_control` trace;
  * BEST_EFFORT map and health are received;
  * the graph check is traced as in P2;
  * the node's publisher set equals `p4_control`'s;
  * ledger records are written inside a control callback in their own span, and `verify_timing`
    still passes.
* **Limit:** the harness runs in ground-diagnostic mode, which never reaches TRANSIT. The live
  TRANSIT ledger path is first exercised in SITL (Gate B).
* **Evidence size:** about 14 KB per invoked record, about 11 MB for a 40 s transit, and at most
  about 42 MB at the 150 s deadline.
* **Informational inference cost in the rig** (801 calls, no load): p50 0.56 ms, p99 1.72 ms, max
  3.43 ms.

### 13.4 M5: runner, gate sequencer, evaluator and contract revision 2 (offline, 2026-10-31)

* **Built:**
  * `p5_runner` (a subclass of the accepted P4 runner: P5 manifest preconditions, `p5_control`
    command with single-threaded BLAS, the `policy_fault` scope, P5 run-local configs);
  * `run_p5_gates`, `run_p5.sh` and `verify_p5_evidence.sh`;
  * the `p5_manifest` candidate identity, authorization and consumption bookkeeping (freeze and
    build receipt in M7);
  * `p5_evaluate`, which composes the unchanged P4 evaluator functions with the P5 checks;
  * `p5_policy_eval`, the policy replays, run under the controller interpreter so scores reproduce
    bit for bit.
* **Gate A feasibility finding:** all 600 accepted P4 ground snapshots have no progress candidate,
  so revision 1's ground-fixture replay could never invoke the policy. **Contract revision 2**
  ([text](v3_sim_p5_contract_revision2.md)) was approved by the operator on 2026-10-31.
* **Item 4, ground policy evaluation, on the accepted P4 G2/G4/G6 snapshots (read-only check):**
  * all 450 snapshots: SCAN, policy not invoked;
  * the mask equals P4's progress set;
  * scores finite and bit-exact on repeat;
  * p99 1.1–1.7 ms, max at most 1.9 ms.
* **Item 5, flight-snapshot fault replay (offline Gate A row `flight_snapshot_replay`):**
  * Both accepted P4 S2 sources pass integrity (gate-result row passed, evaluation hash unchanged).
  * The rebuilt inputs reproduce P4's recorded plan exactly on 793 of 927 (Gate B) and 779 of 859
    (Gate C) ticks.
  * The selected tick is Gate B S2 at 81.152 s, AVOIDING, with 25 progress candidates and 1 slot
    masked.
  * The mask equals P4's, and the true policy chooses slot 14 (P4: 15), finite and inside the mask.
  * All six faults (missing checkpoint, hash mismatch, NaN, timeout, output size, selection
    outside the mask) give HOLD, HOLD, ABORT `policy_unavailable`.
  * The full evaluator chain passes end to end on this real evidence, through a scratch root with
    read-only links.
* **Live checkpoint-mismatch start:** the controller's exit 1 is accepted only in the
  `policy_fault` scope. The same exit in any other scope, and any other cleanup failure, still
  fails (tested).
* **Tests:** `test_px4_gazebo_p5_runtime.py` covers:
  * every ledger and limit rule on a ledger the trained policy produced on a held-out layout;
  * the ground and flight-snapshot rules, the latter on a recorded P4 rig flight of a held-out
    avoidance layout, plus selection, limitation and integrity failures;
  * checkpoint identity, the policy-fault refusal and the composition with the reused P4 checks;
  * runner, gate and manifest bookkeeping, with source drift guards against P4's runner and gate
    sequencer.

### 13.5 M6: P4 vs P5 comparison (offline, 2026-10-31; reported, not acceptance evidence)

* **Tool:** `validation/p5_comparison.py`.
  * `offline` computes the paired held-out comparison from the hash-bound readiness report: the
    same 300 layouts and seeds, under the same shield.
  * `sitl` compares one evaluated P5 flight with the accepted P4 flight of the same scenario, from
    the retained `c110bd30` evidence. It covers outcome, truth clearance, tracking, airborne
    horizontal truth path, time to goal, supervisor lineage, controller CPU and RTF, and the P5
    shadow disagreement.
  * `accepted_p4_runs` maps S1–S6 to those flights.
* **Offline report:** `docs/evidence/v3_sim_p5_offline_comparison.json`, with fixed-seed paired
  bootstrap 95 % intervals (10,000 draws).

  | Paired measure (P5 − P4) | Pairs | Mean | 95 % interval |
  |---|---|---|---|
  | Goal success (points) | 300 | +7.0 | +3.7 to +10.7 |
  | Time to goal, both reached (s) | 210 | −0.51 | −0.84 to −0.23 |
  | Path, both reached (m) | 210 | −0.025 | −0.036 to −0.014 |
  | Minimum truth clearance (m) | 300 | −0.013 | −0.024 to −0.002 |
  | Hover time (s) | 300 | −1.67 | −2.41 to −0.94 |

  * **Discordant goal outcomes:** P5 only 26, P4 only 5; exact sign test p = 0.0002.
  * **Safety, reported separately:** 0 collisions on both sides. Minimum clearance 0.544 m (P4)
    and 0.547 m (P5). Aborts: P4 85 hover timeouts; P5 60 hover timeouts and 4 transit-deadline
    aborts (the §13.2 yaw-oscillation limitation).
  * **Per decision:** 241,224 policy calls, 0 failures, 0 executions outside the mask.
    Disagreement with the shadow P4 choice is 0.694, and heading disagreement 0.246.
* **Claim:** correct substitution only. A superiority claim needs P7's paired batches. These are
  synthetic closed-loop figures, not SITL.

### 13.6 M7: build receipt, freeze tooling, full regression (offline, 2026-10-31)

* **Build:** `scripts/p5_build.py` and `build_p5.sh` follow the P4 build pattern. They mirror
  `gwm_policy_control` into `p5_ws/<package-hash>`, run colcon build plus the installed tests
  (`test/test_installed.py`), and write `state/p5-built.json`.
  * Built: package `35eaccd0566d7413b2a41de27e4eac964709ee62d0f3df33dfab4d35b9bdae0e`, 3/3
    installed tests passing. The `p5_control` entry point is installed.
* **Freeze tooling (`p5_manifest`):**
  * the candidate binds the P5 build receipt;
  * the `freeze`, `verify` and `authorize` commands work as in P4;
  * `authorize` requires the exact phrase `authorize P5 Gate <G> <manifest-sha>`;
  * the output is exclusive-create, and every runner and gate re-verifies.
* **Dry candidate** (computed in memory, not frozen or written):
  * P3 `1e205fec…` and P4 `c110bd30…` verify;
  * model `ee006a80…` (trained, schema `9a651447…`, readiness report `378b7788…`);
  * P5 build `35eaccd0…`, receipt `7c855db2…`;
  * controller runtime `/usr/bin/python3` with NumPy 1.26.4;
  * 45 P5 source files, gates A/B/S/C, execution without authorization false.
* **Operator steps, each separately authorized:**
  1. Approve Policy A for the P5 commit.
  2. Run `p5_manifest.py freeze --model-sha256 ee006a80… --policy-approval <approval> --mc-benchmark-run <run>`.
  3. Authorize Gate A for that manifest SHA-256 (`p5_manifest.py authorize`).
  4. Run Gate A.
  5. Then Gates B, S and C, one at a time.

  No simulator has been run for P5.

### 13.7 Operator steps: Policy A approval and freeze (2026-10-31)

* **Policy A approval:** the operator approved Policy A (`bf1ef9b1…`, unchanged from P4) for P5
  commit `44ff340`. The receipt is `state/p5-policy-A-approval-44ff340.json`, scoped to this
  candidate's Gates A/B/S/C. The P3 policy loader accepts it, and the QGC unattended ELF
  `829942834b71…` matches.
* **Freeze:** `state/p5-candidate-44ff340.json`, manifest SHA-256
  **`baef37a0bc98ab72de2bd6d2b078d4b22f67eb476662eeaa3bafa4a4dc16d27b`**. `p5_manifest.py verify`
  reproduces it. It binds:
  * P3 `1e205fec…` and P4 `c110bd30…`;
  * model `ee006a80…`;
  * build `35eaccd0…`;
  * the controller runtime (`/usr/bin/python3`, NumPy 1.26.4);
  * Policy A with this approval;
  * MC benchmark run `20261025T051926Z-mc-trace-benchmark-e6000074`;
  * 45 P5 source files;
  * gates A/B/S/C.
* **Working-copy bytes:** the manifest binds the files as they are in this checkout. A few P5
  files have CRLF line endings there. If git rewrites them, verification fails closed and a
  re-freeze is needed.
* **Next:** Gate A needs an operator authorization bound to `baef37a0…` (`p5_manifest.py
  authorize --gate A`). Nothing has run in the simulator.

### 13.8 P5 Gate A for manifest `baef37a0…`: FAILED (2026-10-31, one attempt, no retry)

* **Authorization:** `state/p5-auth-A-baef37a0bc98.json`. Result:
  `state/p5-gate-A-baef37a0….result.json`, `accepted: false`. No residual simulator processes.
* **Passed, with independent evaluations:**
  * `p3-readiness`;
  * `p5-readiness`;
  * `fixture-G2`, `fixture-G4` and `fixture-G6` (revision 2 item 4);
  * the offline `policy-failure-replay` (revision 2 item 5).
* **First failure, `policy-fault-checkpoint`:**
  * The runtime behaved as designed: the bound checkpoint plus one byte was refused with
    `checkpoint_sha256_mismatch`, no node or run file existed, the controller exited with 1, and
    status was passed.
  * The evaluator then raised `FileNotFoundError: ros-events.jsonl`. The P3 depth-stream
    association check proves estimator-reset brackets against the controller's own ledger of
    received PX4 messages, which a refused controller never creates.
  * This is an evaluator design gap in the P5 `policy_fault` scope, not a runtime failure.
* **Informational only (in memory; the recorded evaluation and the gate result stand):** with stream
  association skipped in that scope, the same run passes every other check. That covers depth and
  RGB streams, §6.1 performance (compute p99 31.8 ms), the refusal, Policy A and cleanup.
* **Proposed fix (needs operator approval and a new candidate):** in the `policy_fault` scope,
  evaluate the depth stream without controller-ledger association. Association stays required in
  P3 readiness, P5 readiness and G2/G4/G6 of the same gate. No threshold changes.
* **Fix approved and implemented (operator, 2026-10-31):** `p5_evaluate` evaluates the depth stream
  without state association only in the `policy_fault` scope. A test proves association stays on in
  P5 readiness, fixtures and flights. The package build `35eaccd0…` is unchanged. This needs a new
  candidate: Policy A re-approval for the new commit, a re-freeze, and a new Gate A authorization.
