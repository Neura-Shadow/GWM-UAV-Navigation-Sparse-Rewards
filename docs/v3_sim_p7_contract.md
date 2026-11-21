# v3-SIM P7 contract `p7-evaluation-v1` — batch evaluation (DRAFT)

**Status: DRAFT; DECISIONS D1–D14 APPROVED (2026-11-21).** The normative text below encodes every choice.
* **D1–D12:** the operator answered all twelve decisions of §10 in chat on 2026-11-21 ("照建議核准", approve as
  recommended), each as recommended.
* **D13 and D14:** raised by the M0 audit ([audit §8](v3_sim_p7_m0_audit.md)) and approved as recommended on
  2026-11-21 ("Approve as recommended"). They are encoded in §5 and §7.3.

**What this approval does not cover:**
* **The contract as a whole:** it stays a draft until the operator approves it as a whole. The M0 audit may raise
  new decisions first, as P6's M0 did.
* **Implementation and the M0 audit.**
* **The backup:** D5, revised on 2026-11-21 to a Selective Dependency Backup, makes it a precondition. It was
  approved and authorized separately, and executed and verified on 2026-11-21
  ([inventory report §8](v3_sim_p7_m0_dependency_inventory.md)).
* **Any freeze, Policy A approval, pilot or batch block.**

**M1 specifications approved (operator, 2026-11-21, "Approve specs").** These are the pre-registered items C7
freezes. Their canonical SHA-256s are:
* layout sets `d742b7af47c6339fb6d8478870d0cf767b6f09cf41d0ef23d6661bc0b1d2258d`;
* schedule `ef74c71d4242409793c6de759454a498481564c75a3dd13e3402294277ba7c88`;
* analysis plan `c9c7a6f3781d3d90cd858de4f2ea9ef8e4bdebcd1b0a8e6ab84918a4f46d9548`.

See the [M1 report](v3_sim_p7_m1_specs.md). The validity rule in the analysis plan makes §7.1's "the vehicle armed"
exact.

Each milestone, freeze and block still needs its own operator authorization. See the [P7 plan](v3_sim_p7_plan.md).
Every limit not defined here is reused unchanged from P2–P6.

## 1. Scope and claims

* **Claimed on completion, whatever the direction of the result:**
  * **P7-A:** a paired, pre-registered comparison of the accepted P4 (`c110bd30`) and P5 (`2becfbbb`) stacks in PX4
    SITL. The comparison covers goal success (primary), minimum truth clearance, time to goal and path length, on N
    fresh layouts drawn from the declared distribution. It reports an exact McNemar test, effect estimates with 95 %
    confidence intervals, and every safety event.
  * **P7-B (if decided):** for each repeated accepted P6 fault row, the proportions detected, safely handled and
    recovered, with exact 95 % bounds, the latency distributions, and the false alarms per flight-hour in
    pass-through flights.
* **Not claimed:**
  * real-hardware behaviour, certification or formal fault tolerance;
  * results outside the declared layout distribution, the P4 mission and geofence, static boxes, the 0.20 m/s
    transit speed, or this host;
  * any change to the acceptance of P2–P6. P7 is not a gate on them.
* **Mission and safety outcomes are reported separately.** A safe abort is a safety success and a mission failure.

## 2. Boundaries (normative)

* **C1.** No file in the P3, P4, P5 or P6 manifest scopes is created, changed or deleted. Every P7 flight refuses to
  start unless all four manifests verify to the SHA-256s in plan §0.
* **C2.** The models, builds, configs, thresholds and controller interpreters are used unchanged. No P2–P6 rebuild
  happens.
* **C3.** P7 adds no safety logic, fallback or control path. Flight safety stays in the accepted P2/P4/P5 code and
  PX4.
* **C4.** Simulation only. No hardware, and no host-setting change. The AC power overlay is the operator's setting;
  P7 only checks it, read-only.
* **C5.** P7 code lives in `simulation/px4_gazebo_p7/` and `tests/test_px4_gazebo_p7_*.py`. It reaches the accepted
  runners and evaluators only by import or subclass.
* **C6.** World identity: for every acceptance scenario S1–S6, the P7 layout path must generate a world byte-identical
  to the accepted path (`generated_world_sha256`).
* **C7.** Pre-registration. Before the first confirmatory flight, the following are frozen and hash-bound in the P7
  manifest:
  * the layout sets;
  * the randomization schedule;
  * the analysis plan and analysis code;
  * the validity rules.

  None of them changes after the first confirmatory flight.
* **C8.** No outcome-dependent exclusion or re-flight. Validity is decided only by the §7 rules, from records that
  exist before arming or that are independent of the compared outcome.
* **C9.** Each block runs once under its own authorization. A block never retries a valid flight. It stops at the
  first stop condition (§7.3).
* **C10.** No Git push, merge or evidence deletion without explicit operator words.

## 3. Limits carried over unchanged

Every P2, P3, P4, P5 and P6 limit applies as accepted: P2 timing and freshness, P3 stream limits, P4 safety and
supervisor rules, P5 inference limits and shield, and the P6 fault-window rules for P7-B. P7 sets no new safety
limit. Its new numbers are statistical design choices (§6) and resource rules (§7.4).

## 4. Units, layouts and randomization

### 4.1 Layouts

* **Generator:** the bound `p5_layouts.generate(seed)`, with its unchanged admission filter:
  * start and goal discs of 1.5 m;
  * reachable when inflated;
  * no match with an acceptance set.
* **Seed bases:**
  * pilot: 5,000,000;
  * confirmatory: 4,000,000.

  P5 used 1,000,000 (train), 2,000,000 (validation) and 3,000,000 (held-out).
* **Extra exclusion:** a layout that matches any P5 train, validation or held-out layout under the P5 contract §4.1
  rule (same box count, centres within 0.25 m, sizes within 0.10 m) is rejected.
* **Population (M0 audit §4.3):** the confirmatory population is the generator's admitted layouts that are not
  near-duplicates of a P5 layout.
  * The exclusion removes about 15 % of admissible layouts, mostly wall-with-gap and one-box layouts.
  * Every P7 report states this population and its family mix.
* **Selection:** admitted layouts are taken in seed order until the pre-registered count. The list, with seeds,
  family, boxes and SHA-256, is frozen.
  * The generator's boxes carry no names, so each box gets a name under a rule frozen in M1.
  * The list is produced under the interpreter and NumPy version that reproduce the P5 splits (M0 audit §4.1).

### 4.2 Pairs and order

* A **pair** is one confirmatory layout flown once with P4 and once with P5, back to back.
* **From a frozen seed:** the layout order is a random permutation, and the first controller within each pair is
  balanced 50/50.
* A **block** is 20 consecutive pairs of the schedule.

### 4.3 Pilot

* 10 pilot layouts, each flown twice per controller.
* **It estimates only nuisance parameters:**
  * the variation between repeats within a layout;
  * the infrastructure-invalid rate;
  * the time and disk per flight;
  * the discordance rate.
* **The only allowed update:** if the observed discordance proportion is below the planning value, N may be raised,
  to at most the approved budget ceiling. That ceiling is decision D3.
* Pilot outcomes never enter the confirmatory analysis.

## 5. Endpoint definitions (normative)

* **How flights are evaluated (D13).**
  * Every P7 layout declaration (`p4/scenario.json`) carries `outcome: "p7_not_predeclared"`. That is a non-null
    label outside the accepted set: the accepted `p4_evaluate.outcome()` then runs no outcome check and adds no
    failure. A `null` outcome would crash it (M0 audit §3).
  * The accepted evaluators run end to end and unchanged, through their own `verify_p4_evidence.sh` or
    `verify_p5_evidence.sh`. They record every safety, P2, Policy A, stream and policy item.
  * The P7 classifier then decides the mission outcome from the raw records and those items, as below.
  * The accepted evaluator's overall `status` is an acceptance verdict, not a mission outcome, and P7 never uses it
    as one.
* **Classes of evaluator results (D14).**

  | Class | Items | Effect |
  |---|---|---|
  | **Safety event** | `safety:collision_contact`; `safety:truth_clearance` (< 0.20 m); `safety:setpoint_inside_inflated_obstacle`; any `native_commands:*` (a Policy A breach); `p2:command_sequence` other than 176, 400, 21, or more than one LAND; `competing_flight_publisher`; `p2:final_landed_disarmed` failed; the two unsafe-command rules P7 restates (below) | the flight is a failure; the batch stops (§7.3) |
  | **Safety-unverified** | `safety:truth_poses_missing`, `safety:truth_pose_interval` (> 0.05 s), `safety:truth_observer_error`, or a missing contact ledger | the flight is a failure; the batch stops (§7.3) |
  | **Mission failure, no stop** | `safety:tracking` (> 0.30 m), `safety:response_latency` (> 0.25 s) | the flight is not a goal success; reported per flight |
  | **Integrity event** | every other item: depth stream, perception performance, RGB absence, P2 timing, ULog dropout, MC causality, renderer, cleanup and shutdown, recorder overflow, P5 policy-ledger limits | reported per flight; the outcome class does not change |

* **D15: when the accepted P2 integrity check raises** (`p2_integrity_error:*`). This happens, for example, on the
  stale state that a P2 observation-gap RECOVERY leaves. For an armed flight:
  * The classifier reads its safety facts from the ULog with the accepted P2 reader: own commands exactly 176, 400,
    21; landed and disarmed; no PX4 failsafe. The accepted Policy A check must also have run.
  * If they all hold, the item is an integrity event. Each failed fact is a safety event (`p2_ulog:*`). An unreadable
    ULog keeps the flight safety-unverified.
  * The runner records the accepted flight window (PRESTREAM to COMPLETE, or to RECOVERY) before it reports a
    fail-closed exit, as the P6 runner does, so the unchanged evaluator always runs.
* **Unsafe commands (P6 contract §6.3).**
  * Three of its parts are the accepted checks listed as safety events: a setpoint inside an inflated obstacle,
    Policy A or the LAND count, and a setpoint not from the owner.
  * P7's classifier restates the other two, because they exist only as P6 functions over P6's run object:
    * a target motion > 0.01 m or > 0.5° on a tick that must hold (a stale decision, or a P5 policy-failure tick);
    * a return from a stale HOVER to progress without a fresh, non-stale decision (§6.5 there). A flight with no
      such return is not a failure.
  * Both are tested on retained P6 runs, including negative cases (M3).
* **Goal success (primary).** A flight succeeds when all of the following hold:
  * the P4 supervisor recorded `goal_reached_sim_s`;
  * the final goal distance is ≤ the P4 `goal_tolerance_m` (0.30 m). It is measured as in the accepted
    `outcome()`: the last truth position against the takeoff origin + 8 m along +x;
  * the P2 controller reached `COMPLETE`, and the ULog shows landed and disarmed;
  * there was no safety event, and safety was verified;
  * there was no tracking or response-latency failure.

  Any other ending is a failure: a supervisor ABORT for any reason, a P2 RECOVERY, a geofence, a safety event, a
  safety-unverified flight, or a mission failure of the table above.
* **Minimum truth clearance:** the accepted evaluator's `min_truth_clearance_m` over the flight.
* **Time to goal and path length:** `p5_evaluate.reported` (`time_to_goal_sim_s`, `path_length_m`), through
  `p5_comparison.flight_metrics`. Defined only for flights that reached the goal.
* **Safety events:** as in the table above. These comprise collisions, truth clearance < 0.20 m, unsafe commands,
  Policy A breaches and landing failures.
* **Abort type:** the P4 `abort_reason`, or the P2 failure. Reported, not tested.
* **Integrity events:** as in the table above. Reported per flight. They do not change the outcome class.

## 6. Hypotheses and analysis (normative; the code is frozen under C7)

* **Primary, H1:** P(only P5 succeeds) ≠ P(only P4 succeeds).
  * **Test:** exact two-sided McNemar (a binomial test on the discordant pairs, p = 0.5), α = 0.05.
  * **Estimate:** the difference in success proportions, with a 95 % CI (exact or Tango).
* **Key secondaries, in a fixed sequence; each is tested only if the one before passes:**
  1. **S1, clearance non-inferiority:** the lower 95 % confidence bound of the mean paired difference (P5 − P4)
     must be > −δ, with δ from decision D7 (recommended 0.05 m). The bound is computed by paired bootstrap with a
     fixed seed.
  2. **S2, time to goal:** over pairs where both succeeded. Two-sided Wilcoxon signed-rank, α = 0.05, with a
     Hodges–Lehmann estimate and 95 % CI.
  3. **S3, path length:** as S2.
* **Safety:**
  * the counts are reported;
  * zero events in M flights gives the exact 95 % upper bound 1 − 0.05^(1/M);
  * no safety result is traded against efficiency.
* **Exploratory, labelled as such:**
  * results by layout family;
  * abort types;
  * hover time;
  * per-decision disagreement;
  * RTF and host covariates;
  * the pilot's variation between repeats.
* **Missing pairs:** a pair that is still infrastructure-invalid after its one replacement is excluded and listed.
  A sensitivity analysis counts each such pair as a P5-only failure.
* **P7-B, if decided:**
  * per row: Clopper–Pearson 95 % intervals for detected, safely handled and recovered;
  * latency medians, p95 and maxima against the P6 limits;
  * false alarms in pass-through flights, per flight-hour, with an exact Poisson upper bound.

## 7. Validity, replacement and stop rules (normative)

### 7.1 Validity classes

| Class | Determined by | Effect |
|---|---|---|
| not flown | the pre-block host check failed (overlay, AC, CPU, manifests, lock, processes, disk floor) | the block does not start |
| infrastructure-invalid | before arming: start-up or readiness failure, or a P2 abort before `REQUEST_ARM` | that flight alone is re-flown once, at the end of its block; the pair's valid flight is kept |
| valid | the vehicle armed | every outcome is data |

### 7.2 Replacement

* At most one replacement per flight.
* At most 10 % of a block's flights.
* Exceeding either limit stops the batch.
* Valid flights are never re-flown.

### 7.3 Stop conditions

Any of these stops the batch at once. Nothing more is flown until the operator decides.
* a safety event (§5);
* a safety-unverified flight (§5, D14): its safety cannot be told apart from a safety event;
* a manifest drift;
* a simulator process outliving its flight;
* the replacement cap;
* the disk floor (§7.4).

### 7.4 Resources

* A block starts only if WSL free space minus the block's planned use stays ≥ 150 GB.
* Every flight's full evidence is retained.

## 8. Evidence

* **Per flight:** everything the accepted runner retains, plus `p7/layout.json`, `p7/flight-record.json` (validity
  class, outcome class, metrics) and the host record.
* **Per block:** the authorization, the consumption marker, the ledger in schedule order, and the block result.
* **Final:**
  * the frozen analysis output;
  * the P7 report;
  * a JSON evidence summary under `docs/evidence/`.
* **Retention (D4):** full evidence for every flight, including the raw depth recording. Copies off the WSL disk only
  through the backup procedure, with a hash manifest and a verified restore. No deletion.

## 9. Manifest and authorization

**The P7 manifest binds:**
* the P3–P6 manifest SHA-256s;
* the P7 sources and tests;
* the layout sets, schedule, analysis plan and code;
* the host-check definition;
* the runtime receipts, as in P6.

Any change after freeze is a new P7 candidate.

**Approvals and authorizations:**
* a production Policy A approval per P7 candidate commit;
* one authorization per block, `max_attempts` 1, consumed by a permanent marker;
* a separate authorization for the pilot and for any P7-B or P7-C batch.

## 10. Decisions requiring operator approval (recommendation first)

**Decision record.** The operator answered all twelve in chat on 2026-11-21 ("照建議核准"), each as recommended:

| Decision | Answer | Effect |
|---|---|---|
| D1 | P7-A first; P7-B and P7-C later, each decided separately | only P7-A is confirmatory |
| D2 | paired goal success, exact McNemar, α = 0.05 two-sided | §5, §6 primary |
| D3 | N = 200 pairs, ceiling 250 after the pilot; a 40-flight pilot | plan §4.3, §4.5 |
| D4 | full evidence for every flight; no deletion; verified copies only | §8 |
| D5 (revised 2026-11-21) | **Selective Dependency Backup** (see below); originally "the P3–P6 evidence backup runs before any P7 runtime" | no pilot or block before a verified backup of the approved minimal set |
| D6 | a pre-arming failure is re-flown once, capped at 10 % of a block's flights; post-arming outcomes are data | §7.1, §7.2 |
| D7 | clearance non-inferiority margin δ = 0.05 m | §6 S1 |
| D8 | the bound P5 generator; seed bases 5,000,000 (pilot) and 4,000,000 (confirmatory); P5 split-match exclusion | §4.1 |
| D9 | secondaries in the order clearance NI, time to goal, path length | §6 |
| D10 | this host only; Best performance overlay and AC checked before every block | §7.1 |
| D11 | a new `simulation/px4_gazebo_p7/` subclassing the accepted runners; the C6 world-identity check | §2 C5, C6 |
| D12 | a production Policy A per P7 candidate; one authorization per block; a separate pilot authorization | §9 |
| D13 (M0 audit) | P7 layout declarations carry `outcome: "p7_not_predeclared"`; the accepted P4/P5 evaluators run end to end and unchanged; the P7 classifier decides the mission outcome | §5 |
| D14 (M0 audit) | evaluator results fall into four classes: safety event and safety-unverified (both a failure and a batch stop), mission failure (tracking, response latency), and integrity event | §5, §7.3 |
| D15 (M5 pilot stop, 2026-11-21, "Option A") | the runner writes the accepted flight window before a fail-closed exit (the P6 pattern). When the accepted P2 integrity check raises, the classifier establishes the P2 safety facts from the ULog (176/400/21, landed and disarmed, no failsafe, and Policy A checked); then a new candidate, Policy A approval and pilot authorization | §5 |

**M0 audit decisions (operator, 2026-11-21).** The audit ([§8](v3_sim_p7_m0_audit.md)) raised D13 and D14. The
operator approved both as recommended in chat on 2026-11-21 ("Approve as recommended"). D8 stands unchanged. The
audit corrected two planning facts without changing a decision:
* the power at 200 pairs on the sampled population is 0.83;
* the population statement (§4.1).

**D5 revision (operator, 2026-11-21).** D5 is changed from a full historical evidence backup to a **Selective
Dependency Backup**. It revises D5 only; every other decision stands as recorded.
* **What is backed up:** before any P7 runtime, only what P7-A needs, as identified by the M0 dependency inventory
  ([report](v3_sim_p7_m0_dependency_inventory.md)):
  * the Essential set (accepted manifests, build receipts, accepted approvals and gate records, and the runtime
    trees and builds the manifests pin);
  * the Required reference runs;
  * the system package cache.

  It goes to a destination the operator approves.
* **Optional historical evidence** stays in place, untouched, and is not backed up.
* **Future-only P6 evidence** is backed up only if P7-B is approved.
* **Nothing existing** is deleted, moved, overwritten, compressed or uploaded. No runtime build is treated as
  disposable unless its reproducibility or replacement has been verified.
* **P7-A independence:** P7-A code and tests may depend only on the Essential set and the Required reference runs.
* **Approvals:** the operator approved the minimal set (≈ 12.6 GB) and the destination
  `C:\GWM-UAV-Backup\p7-selective-20261121\` on 2026-11-21 ("Approve set and C:").
* **Execution:** authorized on 2026-11-21 ("Authorize the backup"). Attempt 1 stopped on a tool bug, and its folder
  was kept, renamed. The re-run was authorized ("Authorize re-run").
* **Result:** executed and verified on 2026-11-21.
  * 14 tars, 12.79 GB.
  * Restore check: 137,925 entries, 0 problems.
  * Receipt SHA-256 `dce926264a0320cf060eb0af6c0d5b81beacbaf5f434d3978cb21e952b2e8184`.

  D5's precondition for P7 runtime is met.
* **Supplement (2026-11-21):** M3 found that `venv-px4`, which the accepted evaluators run under, had been
  misclassified as Optional (inventory erratum §9). On "Authorize venv backup" it was added as Part E:
  * `E10-venv-px4.tar`;
  * restore check: 16,387 entries, 0 problems;
  * receipt `39bdd616…`;
  * the main receipt unchanged.

The options below are kept for the record.

| ID | Decision | Recommended | Alternatives |
|---|---|---|---|
| D1 | scope and order | P7-A first and the only confirmatory part; P7-B and P7-C decided later | all three together; P7-B first |
| D2 | primary endpoint | paired goal success, exact McNemar, α = 0.05 two-sided | time to goal as primary (needs about 160 both-reached pairs); co-primaries with Holm |
| D3 | sample size and ceiling | N = 200 pairs (87 % power at the offline effect); ceiling 250 after the pilot | 150 (73 %); 300 (97 %, about 720 GB, needs off-disk storage) |
| D4 | retention | full evidence for every flight; no deletion; verified copies only | full for safety and failed flights and a 10 % sample, others hash-and-index only (narrows re-evaluation; not recommended) |
| D5 | backup first | (revised) a Selective Dependency Backup of the M0-inventoried minimal set before any P7 runtime; originally a full P3–P6 evidence backup | back up after P7-A |
| D6 | validity and replacement | §7: pre-arming failures re-flown once, capped at 10 %; post-arming outcomes are data | no replacement at all |
| D7 | clearance non-inferiority margin δ | 0.05 m | 0.03 m (needs about 200 pairs); report only |
| D8 | layout source | the bound P5 generator; seed bases 5,000,000 (pilot) and 4,000,000 (confirmatory); P5 split-match exclusion | the P5 held-out list (already used for readiness); a new out-of-distribution family (a separate study) |
| D9 | secondary order | clearance NI, then time to goal, then path length | time to goal first |
| D10 | host | this host only; Best performance overlay and AC checked before every block | — |
| D11 | architecture | a new `simulation/px4_gazebo_p7/` subclassing the accepted runners; C6 world-identity check | changing the accepted runners (not allowed under C1) |
| D12 | approvals | production Policy A per P7 candidate; one authorization per block; a separate pilot authorization | one authorization for all of P7-A |
| D13 | running the accepted evaluators on generated layouts | the declaration label `p7_not_predeclared`; the accepted evaluators run unchanged, end to end | `outcome: null`, with P7 calling the accepted evaluator functions one by one (more P7 code mirroring the accepted order and exception handling) |
| D14 | which evaluator results decide what | four classes; tracking and response latency count against goal success, without a stop | tracking and response latency as integrity events (no effect on success) |
