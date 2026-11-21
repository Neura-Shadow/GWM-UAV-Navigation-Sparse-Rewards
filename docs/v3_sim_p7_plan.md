# v3-SIM P7 — batch evaluation: plan

**Status: PLANNING. Decisions D1–D14 APPROVED (2026-11-21), all as recommended.**
* The operator's answers were "照建議核准" for D1–D12 and "Approve as recommended" for D13–D14, which the M0 audit
  raised. The record is in contract §10.
* Nothing in P7 is implemented, frozen, launched or authorized.
* The [`p7-evaluation-v1`](v3_sim_p7_contract.md) contract is still a draft until the operator approves it as a whole.
* **The Selective Dependency Backup (D5, revised 2026-11-21) is DONE and verified (2026-11-21).**
  * The operator approved the minimal set and the destination, and authorized the execution and a re-run.
  * 14 uncompressed tars (12.79 GB) are in `C:\GWM-UAV-Backup\p7-selective-20261121\`.
  * The restore check covered 137,925 entries with 0 problems.
  * Receipt SHA-256 `dce92626…`. The record is in [report §8](v3_sim_p7_m0_dependency_inventory.md).
  * This meets D5's precondition for P7 runtime.
* **The M0 read-only audit is DONE (2026-11-21, on "現在繼續p7"):** [audit report](v3_sim_p7_m0_audit.md).
  * Nothing was launched, and no P2–P6 file changed.
  * It raised two new decisions, **D13** (how the accepted evaluators run on generated layouts) and **D14** (which
    evaluator results decide what). Both were approved as recommended on 2026-11-21 and are encoded in contract
    §5 and §7.3.
  * It corrects two planning facts: the power at 200 pairs on the sampled population is 0.83 (§4.5), and the
    population statement (§4.1).
* **Next, needing its own authorization:** M1 (specs: layout sets, schedule, analysis plan).
* Each milestone, freeze and batch block needs its own operator authorization.
* This plan changes no P2–P6 file. The P3, P4, P5 and P6 manifests still verify.

P7 turns the existence demonstrations of P4–P6 into paired, pre-registered statistics. P4, P5 and P6 were accepted
on one flight per scenario. Their reports say what that cannot support:
* superiority of P5 over P4 (P5 contract §1);
* detection, false-alarm or reliability rates (P6 contract §1).

P7 answers those questions in SITL, under a fixed analysis plan, without changing anything that was accepted.

## 0. Baseline (unchanged by P7)

| Item | Identity |
|---|---|
| P3 | `1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c` (`state/p3-candidate-087722b.json`) |
| P4 | `c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b` (`state/p4-candidate-50d5002.json`) |
| P5 | `2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632` (`state/p5-candidate-40a109b.json`), model `ee006a80…` |
| P6 | `222927752a507fdb2fb0ef13f44d6ce8163d50b7d22f405ebbf27df6c21bfa41` (`state/p6-candidate-8e29f3f.json`) |
| Host | the same WSL2 workstation (24 vCPU). The Windows AC overlay must be Best performance (`ded574b5`): the P6 M5 record shows that the accepted stack alone breaks the P2 2.0 s freshness rule on "Best power efficiency". |

**Prior evidence P7 builds on:**
* **P5 offline paired comparison** (plan §13.2): 300 held-out synthetic layouts, seed base 3,000,000.
  * Goal reached: P4 215, P5 236.
  * Discordant pairs: 26 where only P5 reached the goal, 5 where only P4 did (exact sign test p = 0.0002).
  * On the 210 layouts both reached, P5 was 0.51 s faster.
  * Minimum clearance: −0.013 m for P5.

  These are synthetic closed-loop figures. They set the planning effect size and are not P7 evidence.
* **SITL single flights:** the 7 accepted P5 flights against P4 (`docs/evidence/v3_sim_p5_2becfbbb_*_vs_p4.json`),
  and the 12 accepted P6 flights (`docs/evidence/v3_sim_p6_22292775_m7_comparison.json`).

## 1. Research questions and what each can claim

| RQ | Question | Unit | Claim if supported |
|---|---|---|---|
| **RQ1** (primary, P7-A) | Does the P5 learned ranker reach the goal more often than the P4 deterministic ranker, on new layouts from the P5 layout distribution, under identical SITL conditions? | one layout flown once by each controller (a pair) | a paired difference in goal success with a 95 % CI, and an exact McNemar test |
| **RQ2** (P7-A strata, P7-C) | How do layout family, run-to-run variation and CPU load change success, time to goal and clearance? | strata of RQ1 pairs; the pilot replicates; the load sub-study | descriptive effects with CIs; no confirmatory claim |
| **RQ3** (P7-B) | Under repeated injection of the accepted P6 faults, how often is each fault detected and safely handled, and how fast? | one repeated P6 row flight | proportions with Clopper–Pearson 95 % bounds; latency distributions; false alarms per flight-hour in pass-through flights |

**Not claimed:**
* real-hardware behaviour, certification, or generalization beyond the declared layout distribution, speed, static
  box obstacles and this host;
* any change to P4, P5 or P6 acceptance. P7 is evaluation, not a gate on the accepted phases. A P7 result in either
  direction is a valid P7 outcome.

## 2. Facts from the code and evidence that shape the design (read-only audit, 2026-11-21)

1. **New layouts can be flown without touching frozen files.**
   * The accepted `P4Runner` reads its scenarios once, as `self.scenarios = strict_json(P4 / "configs/p4_scenarios.json")`,
     and `prepare_world()` passes them to `p4_worlds.generate()`. The P5 runner inherits both.
   * A P7 runner subclass in a new tree can supply a P7 layout declaration in the same schema
     (`boxes: [{name, center, size}]`). P6 already extended the P5 runner the same way.
   * **Required check (M2):** for every acceptance scenario, the P7 path must produce a byte-identical generated world
     (`generated_world_sha256`) to the accepted path.
2. **The layout distribution already exists, seeded and admission-filtered.** `simulation/px4_gazebo_p5/training/p5_layouts.py`
   (bound, read-only) generates 1–4 static 4 m boxes from one seed, in four families:
   * pillars, 0.40;
   * wall with gap, 0.20;
   * partial wall, 0.15;
   * mixed, 0.25.

   Its admission rejects layouts that block the 1.5 m start and goal discs, are unreachable when inflated, or match
   an acceptance obstacle set. The box format is the one the P4 world generator takes. P5 used seed bases 1,000,000
   (train), 2,000,000 (validation) and 3,000,000 (held-out).
3. **The accepted evaluators label outcomes against the declared expectation.** `p4_evaluate.outcome()` compares the
   run's `p4/scenario.json` `outcome` with what happened. For a generated layout there is no correct expectation in
   advance. So P7 runs the accepted P4/P5 evaluators unchanged, for safety and integrity, and classifies the mission
   outcome itself from the raw records, as P6 composed its evaluator.
4. **Paired metrics are already implemented.** `p5_comparison.flight_metrics()` (bound, read-only) computes the
   following for any P4 or P5 run, through `p5_evaluate.reported`:
   * outcome, clearance, tracking, abort reason;
   * lineage;
   * path length, detour ratio, time to goal.
5. **Run-to-run variation on one world is small.** Five P5-stack flights of S2 (Gates B and C for `2becfbbb` and
   `db4c4779`, and P6 B-0):

   | Metric | Range |
   |---|---|
   | Time to goal | 42.72–42.92 s |
   | Minimum clearance | 0.356–0.453 m |
   | Path length | 9.57–9.92 m |

   Differences between layouts will dominate. A pilot measures the variation properly (§4.3).
6. **Cost per flight:**
   * **Wall time:** about 3.1 min including start and teardown, from consecutive gate runs. A 150 s transit-deadline
     run takes about 5 min.
   * **Disk:** about 1.2 GB, of which 970 MB is the accepted P3 raw depth recording (`sensors/depth.bin`). The
     accepted evaluator needs it.
   * **Compression:** it barely compresses (zlib 1.04:1 on a 64 MB sample), so compression is not a retention
     option.
   * **Free space:** WSL has 783 GB free, with 95 GB already in `runs/`; `D:` has 892 GB free.
7. **The evidence has no backup.** The P3–P5 evidence (about 52 GB with the runtime trees) and the P6 runs and
   state exist only on this machine. The [backup plan](v3_sim_p3_p5_evidence_backup_plan.md) was written but never
   executed. Under the revised D5, only the dependencies P7-A needs are backed up.
   * **Essential:** 6.91 GB.
   * **Required reference runs:** 4.91 GB.
   * **System package cache:** 0.72 GB.

   The other 108 GB stay in place: 85.7 GB Optional historical and 22.6 GB Future-only. See the
   [M0 dependency inventory](v3_sim_p7_m0_dependency_inventory.md).

## 3. Structure

| Part | Content | Order |
|---|---|---|
| **P7-A** | paired P4 vs P5 flights on fresh layouts (RQ1, and RQ2 by family) | first; the only confirmatory part |
| **P7-B** | repeated P6 fault rows on the P5 stack (RQ3) | after P7-A, its own decision and authorization |
| **P7-C** | CPU-load sensitivity (RQ2 load) | optional, descriptive, its own decision |

## 4. P7-A design (the confirmatory study)

### 4.1 Layouts

* Generated by the bound `p5_layouts.generate(seed)` with its unchanged admission filter.
* **Fresh seed ranges, never used before:**
  * pilot: base 5,000,000;
  * confirmatory: base 4,000,000.
* **Extra exclusion:** besides the acceptance sets, a layout is rejected if it matches any P5 train, validation or
  held-out layout under the same §4.1 match rule. This keeps the test in-distribution but on unseen layouts.
  * **M0 audit ([§4.3](v3_sim_p7_m0_audit.md)):** the exclusion removes about 15 % of admissible layouts, mostly
    wall-with-gap (48 % of that family) and one-box layouts, which are almost all concordant. So the confirmatory
    population is "generator layouts that are not near-duplicates of a P5 layout". Every report states it with the
    family mix.
* **Frozen before any confirmatory flight:** the admitted list is generated offline, and its seeds, boxes, families
  and SHA-256 are bound in the P7 manifest. Layouts are taken in seed order until the pre-registered count is
  reached. No layout is added, removed or replaced afterwards.
* **Distribution caveat (stated in every report):** P5 was trained on this generator's distribution, so P7-A
  measures in-distribution generalization to unseen layouts, which favours P5 by construction. An out-of-distribution
  family would be a separate, later question.

### 4.2 Pairing, order and blocks

* Each confirmatory layout is flown once with P4 (manifest `c110bd30`) and once with P5 (manifest `2becfbbb`). The
  two flights of a pair run back to back.
* **Order is randomized from a pre-registered seed:**
  * the layout order across the batch;
  * which controller flies first within each pair, balanced 50/50.
* **Blocks:** 20 pairs (40 flights, about 2.2 h and about 48 GB). Each block is separately authorized and preceded
  by the read-only host check: AC overlay, AC power, CPU, manifest verification, no lock, no simulator process,
  disk floor.
* Per-flight host records (RTF, CPU, flag gaps) are kept as covariates. A drift check across blocks is reported.

### 4.3 Pilot (not confirmatory)

* 10 layouts from the pilot range, each flown twice by each controller: 40 flights, about 2.2 h, about 48 GB.
* **It measures:**
  * the variation between repeats, within a layout, for each metric;
  * the infrastructure-invalid rate;
  * the real time and disk per flight;
  * the SITL discordance rate.
* **What it may change:** only nuisance parameters, by a pre-declared rule. The final layout count may go up if the
  measured discordance is lower than planned, within the approved budget. The pilot's outcome comparison never
  enters the confirmatory analysis.

### 4.4 Endpoints and analysis (summary; normative in contract §5–§6)

* **Primary (RQ1):** goal success, paired by layout.
  * **Definition:** the goal was reached (the P4 `goal_reached_sim_s` record, final goal distance within the mission
    tolerance), the vehicle landed and disarmed, and there was no safety violation.
  * **D13/D14 (contract §5):**
    * The accepted evaluators run unchanged on the declaration label `p7_not_predeclared`, and the P7 classifier
      decides the outcome.
    * A safety event or a safety-unverified flight is a failure and stops the batch.
    * Tracking > 0.30 m or response latency > 0.25 s is a failure without a stop.
    * Every other evaluator item is an integrity event.
  * **Test:** exact two-sided McNemar on the discordant pairs, α = 0.05.
  * **Estimate:** the paired difference in success proportion, with a 95 % CI.
* **Key secondaries, in a fixed sequence (each tested only if the one before passes, which controls the
  family-wise error):**
  1. **Minimum truth clearance, non-inferiority:** the 95 % CI lower bound of (P5 − P4) must lie above −margin
     (decision D7; recommended 0.05 m).
  2. **Time to goal on pairs where both reached the goal:** Wilcoxon signed-rank, with a Hodges–Lehmann estimate and
     95 % CI.
  3. **Path length on the same pairs:** as above.
* **Safety, reported and never traded:** collisions, unsafe commands and landing failures. The required count is
  0; any one stops the batch (§4.6). Zero unsafe outcomes in N flights gives a 95 % upper bound of 1 − 0.05^(1/N).
* **Exploratory, no claims:**
  * results by family;
  * abort types: P4 hover timeouts, and P5 transit-deadline livelocks (the known P5 yaw-oscillation limitation,
    P5 plan §13.2);
  * hover time, per-decision disagreement, and RTF as a covariate.
* **The analysis code is frozen before the first confirmatory flight.** It is hash-bound in the P7 manifest and
  runs unchanged on the final data.

### 4.5 Sample size and budget

**Exact McNemar power, α = 0.05 two-sided:**

| Pairs | Offline effect (26 vs 5 of 300) | Half the excess | One third |
|---|---|---|---|
| 100 | 0.49 | 0.11 | 0.05 |
| 150 | 0.73 | 0.19 | 0.08 |
| **200** | **0.87** | 0.26 | 0.10 |
| 250 | 0.94 | 0.33 | 0.12 |
| 300 | 0.97 | 0.40 | 0.14 |

**M0 audit correction:** these powers use all 300 held-out layouts. On the population P7 actually samples (P5
near-duplicates excluded: 11.1 % discordant, P5-only share 0.81) they are 0.69 / **0.83** / 0.91 at 150 / 200 / 250
pairs ([audit §4.4](v3_sim_p7_m0_audit.md)). N stays 200 (D3). The pilot rule may raise it to 250.

**Other endpoints, at the offline variances:**
* **Time to goal:** 0.5 s needs about 160 both-reached pairs. Roughly 70 % of layouts are both-reached offline.
* **Clearance non-inferiority:**
  * margin 0.05 m needs about 45 pairs;
  * margin 0.03 m needs about 200 pairs.

**Budget:**

| Plan | Flights | Wall time | Disk |
|---|---|---|---|
| Pilot | 40 | ≈ 2.2 h | ≈ 48 GB |
| P7-A, 200 pairs (recommended) | 400 | ≈ 22 h in 10 blocks | ≈ 480 GB |
| P7-A, 250 pairs | 500 | ≈ 27 h | ≈ 600 GB |

* **Recommendation:** 200 pairs. That's 87 % power if SITL reproduces the offline effect. It fits the disk with a
  150 GB floor, once the earlier evidence is backed up.
* A smaller true effect would not be detected at any affordable size. The CI is reported either way, and "no
  evidence of a difference" is a valid result.

### 4.6 Validity, replacement and stop rules (no outcome-dependent exclusion)

* **Before a block:** a failed host check means nothing is flown.
* **Before arming** (start-up failure, a readiness failure, a host-timing abort in PRESTREAM): the flight is
  infrastructure-invalid. That flight alone is re-flown once, at the end of the block, and the pair's valid flight
  is kept. Replacements are capped at 10 % of a block's flights. Exceeding the cap stops the batch.
* **After arming, every outcome is data.** Goal, hover timeout, transit deadline and P2 abort are all outcomes, and
  are never re-flown. Teardown crashes after landing (for example the DDS agent SIGSEGV) are counted as integrity
  events and do not change the outcome.
* **Safety stop:** any of the following stops the batch at once, with nothing more flown until the operator decides:
  * a collision;
  * truth clearance below 0.20 m;
  * an unsafe command (P6 contract §6.3);
  * a Policy A breach;
  * a failure to land and disarm;
  * a safety-unverified flight (missing or broken truth records, D14).
* **Resource stop:** the batch stops if WSL free space would fall below 150 GB.
* **No interim efficacy analysis.** The confirmatory analysis runs once, on all pre-registered pairs.

### 4.7 Retention (decision D4)

* **Every P7 flight keeps its full evidence,** including `depth.bin`, because the accepted evaluator needs it and it
  does not compress.
* Completed and verified blocks may be copied to `D:` or to external media only under the backup procedure, with a
  hash manifest and a verified restore. Nothing is deleted.

## 5. P7-B and P7-C (later, each with its own decision)

**P7-B, fault rates (RQ3):**
* **What runs:** the accepted P6 rows, repeated on the P5 stack through the accepted P6 runner path, on their own
  accepted worlds. P6 triggers read the frozen `p4_scenarios` boxes, so generated layouts are out of scope here.
* **Rows:**
  * B-0 (pass-through; false alarms);
  * B-1 (F3.08), S-1 (F1.01), S-4 (F4.01), S-5′ (F6.03);
  * C-1 (F1.03), C-2 (F3.07).
* **Repeats per row:**
  * 30 repeats: zero misses gives a 95 % lower bound of 90.5 %;
  * 59 repeats: zero misses gives 95 %.
* **Budget:** 7 rows × 30 = 210 flights, about 11 h and about 250 GB. That does not fit beside P7-A without moving
  verified evidence off the WSL disk.

**P7-C, load sensitivity (RQ2):**
* paired P4/P5 flights on a small layout subset at harness levels 0 and 8 workers;
* descriptive only;
* 24 workers is excluded, because P6 C-5 showed it drives the accepted P2 controller into its dispatch abort.

## 6. Architecture (new tree; every accepted tree untouched)

```text
simulation/px4_gazebo_p7/
├── configs/      p7_layouts.json (frozen layout sets), p7_schedule.json (randomized order), p7_analysis_plan.yaml
├── scripts/      p7_runner.py (P4/P5 runner subclasses supplying the P7 layout), p7_manifest.py, run_p7_block.py
├── validation/   p7_classify.py (validity and outcome classes), p7_metrics.py (wraps p5_comparison.flight_metrics)
├── analysis/     p7_analysis.py (McNemar, CIs, non-inferiority, Wilcoxon/HL, strata), frozen before confirmatory data
└── tests/        offline tests on retained P4/P5/P6 runs and synthetic ledgers
```

* **The P7 manifest binds:**
  * the P3–P6 manifest SHA-256s, re-verified at freeze and before every flight;
  * the P7 sources and the layout set;
  * the randomization schedule and the analysis plan and code;
  * the host-check definition.
* **Authorization:** a production Policy A approval for the P7 candidate, the P6 pattern. Each block has its own
  authorization (`max_attempts` 1, consumed by a permanent marker). Replacements happen inside the same block, under
  the rules of §4.6 only.
* **The runner owns no new safety logic.** Flight safety stays entirely in the accepted P2/P4/P5 code and PX4.

## 7. Milestones (each needs separate authorization)

| M | Content | Exit criterion |
|---|---|---|
| **M0** read-only audit | runner override point, evaluator behaviour on non-acceptance layouts, metric reuse, time/disk budget, backup status (§2 is its first pass); **dependency inventory for the revised D5: done 2026-11-21** ([report](v3_sim_p7_m0_dependency_inventory.md)); **selective backup executed and verified 2026-11-21** (report §8); **read-only audit done 2026-11-21** ([audit](v3_sim_p7_m0_audit.md)), raising D13 and D14 (approved 2026-11-21); **M0 complete** | written audit; D1–D14 answered (done) |
| **M1** specs | P7 layout sets (pilot, confirmatory), randomization schedule, analysis plan | operator approves; hashes recorded |
| **M2** runner and orchestrator (offline) | P7 runner subclasses, block runner, host checks, disk floor, ledger | byte-identical worlds for S1–S6 through the P7 path; offline tests; P3–P6 still verify |
| **M3** classifier and analysis (offline) | validity and outcome classes, metrics, statistics | tests on retained runs and synthetic data, including negative cases; analysis code frozen |
| **M4** parity dry run | development candidate; S2 flown through the P7 path with P4 and P5, compared with the accepted flights | the P7 path reproduces the accepted outcomes within run-to-run variation; no credit |
| **M5** pilot | 10 pilot layouts × 2 controllers × 2 repeats | pilot report; the nuisance-parameter update by the declared rule |
| **M6** confirmatory P7-A | real freeze; blocks of 20 pairs, each authorized | all pre-registered pairs flown, or a stop rule met |
| **M7** P7-B / P7-C | only if decided | their own records |
| **M8** analysis and report | the frozen analysis on the final data; final report | P7 report, whatever the result |

## 8. Risks and limitations

* **In-distribution bias:** the layouts come from P5's training distribution (§4.1).
* **One host:** results hold for this workstation and its power settings. The overlay is the operator's setting and
  is checked before every block.
* **Long batches:** about 22 h of simulator time raises the chance of rare stack faults. These include the DDS-agent
  teardown SIGSEGV, the Gazebo shutdown hang and the recorder overflow seen in P3–P6. §4.6 classifies them without
  looking at outcomes.
* **The effect may be smaller in SITL than offline:** the power table shows that 200 pairs cannot detect half the
  offline effect. The report then gives the CI and no superiority claim.
* **Disk:** P7-A uses most of the free WSL space. The Selective Dependency Backup came first (D5, revised); it was done and
  verified on 2026-11-21. It holds only the minimal set, as one copy on C:.
* **Host packages:** no apt package is held. An `apt upgrade` of the pinned ROS or mesa packages, or a Windows
  GPU-driver update, would break verification of all four manifests. These are the operator's host settings; the
  M0 inventory records the exact versions, and the apt cache holds every installed ros-jazzy `.deb`.

## 9. Decisions requiring operator approval (recommendation first; detail in contract §10)

**All fourteen were approved as recommended on 2026-11-21.** D1–D12 were approved on "照建議核准". D13 and D14,
which the M0 audit raised, were approved on "Approve as recommended".

| ID | Decision | Recommendation (approved) |
|---|---|---|
| D1 | scope and order | P7-A first and confirmatory; P7-B and P7-C later, each decided separately |
| D2 | primary endpoint | paired goal success, exact McNemar, two-sided α = 0.05 |
| D3 | sample size | 200 confirmatory pairs, plus a 40-flight pilot |
| D4 | retention | full evidence for every P7 flight; no deletion; verified copies only |
| D5 | backup first | **revised 2026-11-21:** a Selective Dependency Backup of the M0-inventoried minimal set (Essential, Required reference runs, system package cache; ≈ 12.6 GB) before any P7 runtime; Optional evidence stays in place, and Future-only evidence is backed up only if P7-B is approved |
| D6 | validity and replacement | §4.6: a pre-arming failure is re-flown once, capped at 10 % of a block's flights; post-arming outcomes are data |
| D7 | clearance non-inferiority margin | 0.05 m |
| D8 | layout source | the bound P5 generator; fresh seed bases 4,000,000 / 5,000,000; P5 split-match exclusion |
| D9 | fixed-sequence secondaries | clearance NI, then time to goal, then path length |
| D10 | host conditions | this host only, Best performance overlay, checked before every block |
| D11 | architecture | a new `simulation/px4_gazebo_p7/` tree subclassing the accepted runners; no bound file changes |
| D12 | approvals | a production Policy A approval for the P7 candidate; per-block authorizations |
| D13 | evaluators on generated layouts (M0 audit) | the declaration label `p7_not_predeclared`; the accepted evaluators run unchanged; the P7 classifier decides the outcome |
| D14 | evaluator result classes (M0 audit) | safety event and safety-unverified: failure and batch stop; tracking or response-latency failure: failure, no stop; everything else: integrity event |
