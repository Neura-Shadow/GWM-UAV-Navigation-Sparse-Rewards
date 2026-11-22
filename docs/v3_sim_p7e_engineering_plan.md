# v3-SIM P7-E — engineering validation: proposed revision of the P7 next stage

**Status: DIRECTION AND E1–E11 APPROVED IN PRINCIPLE (operator, 2026-11-22), subject to clarifications 1–6.**
* The clarifications are addressed in §12.
* The **contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) was APPROVED by the operator
  (2026-11-22, "Approve contract").**
* **E1 specifications generated and tested (2026-11-22, "Authorize E1"):** [E1 report](v3_sim_p7e_e1_specs.md).
  * Schedule `9b3deacb…`; report plan `15e7f7b4…`.
  * **APPROVED by the operator on 2026-11-22 ("Approve specs"), so E1 is COMPLETE.**
* **Next, each needing its own explicit authorization:** the retention-policy approval (separate from any deletion),
  then E2.
* Nothing is implemented, frozen or flown, and every later step needs its own explicit authorization (contract §13).

It was prepared under the operator's request "P7 — Engineering Validation Revision Plan" (read-only review and
proposal preparation only).

**What was not done:**
* No existing contract, plan, source, test, manifest, frozen candidate or evidence file was changed.
* The 200-pair schedule selection was **not** confirmed or written.
* No candidate was created, no SITL ran, nothing was deleted, nothing was pushed.

**What was added (new files only):**
* this plan;
* the [draft contract revision](v3_sim_p7e_contract_draft.md);
* the [retention and cleanup proposal](v3_sim_p7e_retention_proposal.md);
* two read-only tools in `diagnostics/p7e/`, with their outputs in `docs/evidence/v3_sim_p7e_*.json`. The cleanup
  manifest is now version 2, reconciled (§12).

## 1. Where P7 stands, preserved unchanged

| Item | State |
|---|---|
| P3–P6 baselines | `1e205fec…`, `c110bd30…`, `2becfbbb…`, `22292775…`; all verify |
| Frozen P7 candidate | `0799c37e800c06c278acbc45fdec90b84159ee9d41eb7aaf868cb65a0b950aa5` (`state/p7-candidate-a8bd50e.json`), with its production Policy A approval |
| P7 pilot evidence | attempt 2: 40/40 flights, no stop; attempt 1: stopped after 6; and the M4 dry run. All 48 P7 runs are kept and classified PROTECTED. |
| Original P7-A design | contract `p7-evaluation-v1` (D1–D15), the M1 specifications (`d742b7af…`, `ef74c71d…`, `c9c7a6f3…`) and the frozen analysis code `bd036a2b…`. All unchanged. |
| Pilot rule | record `state/p7-pilot-rule-0799c37e800c.json`: d = 0.15, so 200 pairs. The **selection record is not written**. |
| Authorizations | every existing approval and authorization stays as recorded. The pilot authorization is consumed, and no confirmatory block was authorized. |

**Proposed status of P7-A:** suspended, not cancelled. If it is ever resumed, its 200-pair schedule is still unflown
(§3.1).

## 2. Purpose and claims

**P7-E is an engineering system validation.** It asks one question: on previously unseen layouts, under identical
conditions, do the accepted P4 and P5 stacks run end to end through the P7 infrastructure safely, reliably and
reproducibly? It also measures what each achieves.

**Claimed on completion:**
* every safety, integrity, reliability and evidence criterion of §5, with P4 and P5 reported separately and as
  paired comparisons;
* the host and pipeline costs;
* a risk register.

**Not claimed:**
* that P5 is superior to P4, or the reverse;
* any statistically significant difference;
* any rate claim beyond the stated intervals;
* anything about real hardware.

**Engineering success is a different statement from evidence that P5 is better.** A passed P7-E says the system
works safely as specified on 60 flights. It says nothing reliable about which ranker is better (§7).

## 3. Design (provisional, as requested)

### 3.1 Layouts

**Thirty pairs from the existing frozen confirmatory layout pool**, taken by a rule fixed before any outcome, with no
selection by pilot results.

**Recommended: pool indices 200–229 in seed order** (seeds 4,000,349–4,000,398):
* none of these 30 layouts appears in the 200-pair confirmatory schedule;
* none was flown in the pilot;
* so P7-A keeps an unflown 200-pair set if it is ever resumed.

The alternative, indices 0–29, would use layouts that all belong to the 200-pair schedule and would contaminate
P7-A.

| Indices | Families | Boxes per layout |
|---|---|---|
| 200–229 (recommended) | 9 mixed, 9 pillars, 9 wall with gap, 3 partial wall | 7 × 1, 17 × 2, 6 × 3 |
| 0–29 (alternative) | 11 pillars, 11 mixed, 5 partial wall, 3 wall with gap | 8 × 1, 12 × 2, 10 × 3 |

**Every layout keeps its frozen world SHA-256.** The runner's C6 check stops a flight before PX4 starts if the world
differs.

### 3.2 Pairs, order and blocks

* **Pairs:** each layout is flown once by P4 and once by P5, back to back, under identical conditions.
* **Volume:** 60 SITL flights in **3 blocks of 10 pairs** (20 flights each).
* **Pre-registered order:** the M1 method, with new labels and no hand-chosen seed.
  * The SHA-256 counter stream is seeded by the frozen layouts' canonical hash, with labels `p7e-30-order` and
    `p7e-30-block-<b>-first`.
  * Within every block, 5 pairs fly P4 first and 5 fly P5 first.
* **The schedule belongs to the new P7-E candidate** and is approved and hash-bound before any P7-E flight.

### 3.3 Unchanged rules

These are inherited from `p7-evaluation-v1`:
* No outcome-dependent layout exclusion, and no post-arming retry (C8).
* A pre-arming failure is re-flown once at the end of its block, with at most 2 replacements per 20-flight block (D6).
* Every contract §7.3 stop condition and the D14/D15 classes apply unchanged:
  * a safety event or a safety-unverified flight;
  * manifest drift;
  * a leftover simulator;
  * the replacement cap;
  * the 150 GB disk floor.
* The accepted P2–P6 safety logic and every threshold are unchanged. There is no new safety logic.

## 4. Architecture (recommended: a separate tree)

**`simulation/px4_gazebo_p7e/` binds the frozen P7 candidate `0799c37e…` as its baseline**, the way P6 bound P5.
It reuses the accepted and frozen P7 code by import:
* `p7_runner` (`P7P4Runner`, `P7P5Runner`), with its D15 flight window;
* `p7_classify`, through `verify_p7_evidence.sh`;
* the sequencer's `BlockRules`, `run_sequence` and `Ledger`;
* `p7_specs.scenario_mapping`.

**New P7-E files:**

| File | Role |
|---|---|
| `p7e_manifest.py` | binds P3–P6 through the P7 verification, the P7 candidate, the P7-E sources, specs and host check, and a P7-E Policy A approval; block authorizations for scope `engineering-30`, blocks 1–3 |
| `p7e_runner.py` | subclasses the two P7 runners; overrides only `preconditions` (the P7-E manifest, authorization and slot) and the run naming (`…-p7e-…`) |
| `run_p7e_block.py` | reuses the P7 rules; runs the P7-E launcher |
| `run_p7e.sh`, `p7e_namespace_owner.py`, `p3_runner.py` shim | the P6/P7 launcher pattern, with the owner byte-identical to P3's |
| `p7e_specs.py`, `configs/p7e_schedule.json` | the 30-pair schedule (§3.2) |
| `configs/p7e_report_plan.yaml` | the frozen engineering report definition (§5, §6) |
| `p7e_report.py` | the engineering report: descriptive; reuses the frozen P7 interval functions |

**Why a separate tree:** the P7 tree stays byte-identical, so the frozen candidate `0799c37e…` and the pilot evidence
keep verifying against the working tree.

**The alternative**, extending the P7 tree, is simpler. But `0799c37e…` would then verify only at its commit
`a8bd50e`, no longer in the working tree.

## 5. Engineering acceptance criteria and test matrix

**What the criteria use.** Every numerical limit below is an existing accepted limit, with its source given. Nothing
is invented or relaxed.

**Hard criteria (H)** decide engineering completion. Each is also a §7.3 stop condition, or a completeness check of
the record.

| ID | Criterion | Measure (source of the value) | Limit (source) | Scope |
|---|---|---|---|---|
| H1 | Collision count | `safety:collision_contact` (accepted P4 evaluator, Gazebo contacts) | 0 (P4 `collisions_allowed`) | per flight; batch stop |
| H2 | Minimum truth clearance | `min_truth_clearance_m` (accepted P4 evaluator) | ≥ 0.20 m (P4 `min_truth_clearance_m`) | per flight; batch stop |
| H3 | Unsafe commands | `safety:setpoint_inside_inflated_obstacle`, `competing_flight_publisher`, the restated P6 §6.3 hold-tick and revalidation rules (classifier) | 0 (P6 §6.3, D14) | per flight; batch stop |
| H4 | Policy A | `native_commands:*` (accepted check, Policy A `bf1ef9b1…`) | 0 breaches | per flight; batch stop |
| H5 | Landing and disarm integrity | accepted P2 items, or D15's ULog facts: own commands 176/400/21, landed and disarmed, no failsafe | every valid flight | per flight; batch stop |
| H6 | Safety verifiable | no safety-unverified flight (D14) | 0 | per flight; batch stop |
| H7 | Evidence completeness | each flown flight has its summary, the accepted evaluation, the flight record and a hash-bound ledger line (contract §8) | 100 % | batch |
| H8 | Batch execution reliability | flights flown against scheduled; replacements; stops | all 60 flown, or every gap explained by a §7 rule; replacements ≤ 2 per block (D6) | batch |
| H9 | Reproducibility | re-running the frozen classifier on every retained flight reproduces its recorded flight record; every manifest and world hash re-verifies | identical | batch, after the last block |

**Reported criteria (R)** are measured against their accepted limits. Each breach is an integrity event and is
entered in the risk register; it does not by itself fail the engineering evaluation (D14).

| ID | Quantity | Measure (source) | Accepted limit it is reported against |
|---|---|---|---|
| R1 | Mission success rate | classifier `goal_success`, per controller | none; reported with a Clopper–Pearson 95 % interval |
| R2 | Paired outcomes | both, P5-only, P4-only, neither; the difference with a Tango 95 % interval (frozen P7 functions, labelled descriptive) | none; no test |
| R3 | P2 timing and freshness | `p2:timing`; P2 aborts by reason (e.g. `observation_gap`) | the accepted P2 timing contract |
| R4 | P3 stream and P4 perception timing | `depth:*`, `performance:*` items | snapshot ≥ 8 Hz; latency max 0.25 s / p95 0.15 s; compute p99 50 ms / max 90 ms (P4) |
| R5 | P5 inference | policy-ledger inference p99 and max, failure ticks, fallback, participation | p99 ≤ 5 ms, max ≤ 10 ms, 0, 0, 1.0 (P5) |
| R6 | Tracking and response latency | `safety:tracking`, `safety:response_latency` (D14 mission failures) | 0.30 m, 0.25 s (P4) |
| R7 | Time to goal; path length | `p5_comparison.flight_metrics` on both-success pairs; per-controller medians; paired Hodges–Lehmann difference with interval (descriptive) | none |
| R8 | Host | pre-block host check; per-flight process CPU, memory available, mean RTF, stall count (RTF < 0.5), disk per flight | host check: overlay, AC, CPU ≤ 15 %, 150 GB floor (D10, M2) |
| R9 | Abort types | `hover_timeout`, P2 failures and others, per controller | none |
| R10 | Integrity events | every other evaluator item, per flight | the accepted item limits |

**Engineering completion** means H1–H9 are all met and the report of R1–R10 is complete:
* **Passed:** P7-E then records "engineering validation passed".
* **A failed H criterion** stops the batch under §7.3. P7-E is then "not passed", pending the operator's decision.

**Test matrix:**

| Stage | Covers | How |
|---|---|---|
| E2, offline | P7-E manifest bindings; authorization; schedule (30 unique pool layouts, indices 200–229; 3 blocks; 5/5 balance; derived only from the frozen hash) | unit tests, like M1/M2 |
| E2, offline | C6: P7-E path worlds equal the frozen world hashes for all 30 layouts and the accepted S1–S6 | offline generation against `p7_layouts.json` and the accepted flights |
| E2, offline | the reused P7 runner, classifier and sequencer are unchanged (byte identity with the frozen candidate) | hash test against `0799c37e`'s bound files |
| E2, offline | the report: every H and R criterion computed from records; negative cases (a collision, safety-unverified, missing evidence, a replacement over the cap) | synthetic flight records and retained P7 pilot records (read-only) |
| E3, live (optional, decision E8) | the new launch path end to end | a 2-flight development dry run of S2 (P4, P5), as M4 |
| E4, live | H1–H8 per flight and block | the three authorized blocks |
| E5, offline | H9, and the report | re-classification of all 60 flights on read-only views; the report |

## 6. Reporting

* **Per controller:** success rate with a Clopper–Pearson interval, safety counts with their zero-event bounds, the
  integrity and timing items, inference latency and host costs.
* **Paired:** the outcome table, the success difference (Tango interval), and the time-to-goal and path differences
  on both-success pairs (Hodges–Lehmann interval). Clearance is reported per pair; there is no non-inferiority test.
  All of it is labelled descriptive.
* **No p-values** (recommended, decision E7), so that nothing reads as a significance claim.

## 7. What 30 pairs can and cannot show

Computed with the frozen P7 functions:

| Quantity | Value at 30 pairs |
|---|---|
| Power of an exact McNemar test at the planning effect (11 % discordant, 81 % P5-only) | **3 %** (8 % at the pilot's 15 % discordance); a superiority difference is essentially undetectable |
| Width of a 95 % Tango interval for the success difference | about 0.29–0.32 (b = 3, c = 1 gives [−0.09, 0.23]) |
| 95 % interval for a success rate of 15/30 | [0.31, 0.69] (25/30: [0.65, 0.94]) |
| Zero safety events, one-sided 95 % upper bound per flight | 9.5 % per controller (30 flights); 4.9 % over 60 |

These numbers are why §2 separates engineering success from superiority. P7-E can show that the system runs safely
and completely. It cannot rank the controllers.

## 8. Pilot follow-up and engineering risk register

Investigated with existing evidence only: [`v3_sim_p7e_pilot_followup.json`](evidence/v3_sim_p7e_pilot_followup.json),
covering the 48 P7 flights, read-only.

| ID | Risk | Evidence | Treatment |
|---|---|---|---|
| RR1 (open) | **P2 observation-gap RECOVERY** | 2 of 46 pilot flights, both P5. Each followed a brief simulator slowdown just before the abort (RTF ≈ 0.20 for 0.36 s; 0.23 for 0.31 s). Process CPU (Gazebo 0.82, controller 0.37–0.39, perception 0.33–0.37), load (≈ 2.9) and memory (≈ 7 GB free) were normal. Stalls below RTF 0.5 occur in **all** 48 P7 flights, mostly at start-up; median 3 for both controllers; controller CPU 0.41 (P4) vs 0.42 (P5). P4 0/24 vs P5 2/24: Fisher exact p ≈ 0.49. | Safe in both cases (D15). No threshold change. Counted as an outcome (a failure), reported per controller (R3, R9), monitored in P7-E. The slowdown's source (Gazebo or WSL scheduling) is not visible at the 1 s resource resolution; finding it would need new instrumentation, which is not proposed now. |
| RR2 (open) | **P5 repeat-outcome inconsistency** | 3 of 20 controller repeats disagree, all P5; P4 is 10/10 consistent. Two are behavioural: on `p7p-5000002` and `p7p-5000008` one repeat avoided the obstacle and the other ended in HOVER → `hover_timeout`, with similar clocks. One is RR1 (`p7p-5000010`). | P5's choice at a decision point is sensitive to small timing differences, so a single flight per layout carries run-to-run variance in P5's success. Reported as a limitation of R1/R2. P7-E (one flight per controller per layout) cannot measure it; the pilot's 17/20 is the available estimate. |
| RR3 | Hover-timeout prevalence | about half of the pilot pairs ended in `hover_timeout` for both controllers | An outcome, not a safety issue. Expect many "neither" pairs (R2, R9). |
| RR4 | Disk per flight above the planning value | mean 1.33 GB, max 1.82 GB, against the host check's 1.3 GB | 60 flights need about 80 GB of 776 GB free; no change proposed |
| RR5 | Line-ending fragility | `core.autocrlf` on, no `.gitattributes`; the P3–P6 manifests hash CRLF working copies | P7 specs use canonical hashes. Do not re-checkout the working tree. A `.gitattributes` decision is left for later. |
| RR6 | `venv-px4` built from unpinned `pip` requirements | NumPy 2.5.3, pyulog 1.2.4 | Backed up and verified as Part E |
| RR7 | The accepted P2 integrity function raises on a stale state | the pilot stop | D15 (runner window and ULog facts) |

## 9. Milestones and block schedule

| Milestone | Content | Needs |
|---|---|---|
| E0 | this proposal: decisions E1–E11 (§11) | operator approval |
| E1 | specs: the 30-pair schedule, the report plan, contract `p7-engineering-validation-v1` | operator approval of the hashes |
| E2 | code offline: the `px4_gazebo_p7e` tree, tests (§5 matrix) | authorization; operator acceptance |
| E3 | candidate: optional development dry run (E8), P7-E Policy A approval, real freeze | each explicit |
| E4 | **blocks 1, 2, 3** | one authorization each |
| E5 | report: H9 re-classification, the engineering report, the risk register | authorization; operator acceptance |
| R | retention: the D4-B policy now; a cleanup procedure later, designed, tested and approved separately | see the [retention proposal](v3_sim_p7e_retention_proposal.md) |

**Each block** is 10 pairs and 20 flights, about 64 min (pilot cycle median 191 s, max 218 s, so at most about
73 min). Planned disk is about 27 GB (26 GB in the host check).

## 10. Storage and execution cost

| Item | Estimate |
|---|---|
| Flight time | 60 × about 191 s, about **3.2 h** in 3 blocks of about 64 min (worst case about 3.6 h) |
| Disk for P7-E runs | 60 × 1.33 GB, about **80 GB** (range 62–109 GB at the pilot's minimum and maximum). WSL has 776.4 GB free, leaving about 696 GB. D: grows by the same amount. |
| Optional dry run (E8) | 2 flights, about 7 min, about 2.5 GB |
| Offline work (E1, E2, E5) | no simulator; re-classifying 60 flights takes minutes |
| Space a future cleanup could reclaim | 39.5 GB of raw depth from 107 P6 development dry runs (retention proposal). Under D4-B, P7-E's own raw depth (about 63 GB) becomes eligible only after its report is accepted and integrity-verified, and only by a separate approval. |

## 11. Decisions requiring operator approval

**Approved in principle as recommended (operator, 2026-11-22), subject to clarifications 1–6 (§12).** The final
contract, which incorporates them, was approved on 2026-11-22 ("Approve contract").

| ID | Decision | Recommendation |
|---|---|---|
| E1 | the next stage | adopt P7-E; P7-A is suspended, not cancelled; the 200-pair selection is not written; everything existing is preserved |
| E2 | sample and blocks | 30 pairs, 60 flights, 3 blocks × 10 pairs |
| E3 | layouts | confirmatory-pool indices 200–229 (keeps P7-A's 200-pair set unflown); alternative 0–29 |
| E4 | order | the M1 deterministic method with new labels; 5/5 first-controller balance per block |
| E5 | architecture | the separate `px4_gazebo_p7e` tree binding `0799c37e` (P7 tree unchanged); alternative: extend the P7 tree |
| E6 | acceptance | hard criteria H1–H9 from existing limits; R1–R10 reported against accepted limits |
| E7 | reporting | descriptive intervals, no p-values; alternative: exploratory McNemar p, clearly labelled |
| E8 | dry run | yes: a 2-flight S2 development dry run of the new launch path before the real freeze |
| E9 | retention | D4 revised into A (full evidence while flying and evaluating) and B (long-term retention after evaluation and integrity verification), as in the retention proposal; no cleanup now |
| E10 | P7-E runs under D4-B | kept complete until the P7-E report is accepted; then their raw depth is eligible for a separately approved cleanup, except safety incidents, unresolved failures and runs named by tests or evidence |
| E11 | risk register | adopt RR1–RR7 with the stated treatments; no threshold change |

## 12. Clarifications 1–6 (operator, 2026-11-22) and how they are met

| # | Clarification | Resolution |
|---|---|---|
| 1 | Reconcile 377 reported runs against 376 classified, and mark nothing eligible until every run is accounted for | `runs/` holds 377 entries: 376 run directories and **one file**, `20260912-p2-r3-historical-timing.json` (64 KB, a P2-era read-only timing reconstruction named by `v3_sim_p2_timing_contract.md`). Version 1 of the manifest read directories only and skipped it silently. **Version 2** accounts for every entry and marks nothing eligible unless the count reconciles. The file is **PROTECTED**. Result: `reconciled: true`, 377 of 377 ([`v3_sim_p7e_cleanup_manifest_v2.json`](evidence/v3_sim_p7e_cleanup_manifest_v2.json), `8e04e6be…`; v1 `e87ec4c5…` is superseded). |
| 2 | Layouts 200–229 must not contaminate or alter the 200-pair P7-A schedule; document any overlap with a future expansion | Verified: the 200-pair schedule is exactly pool indices 0–199, with **0** overlap. `p7_schedule.json` and `p7_layouts.json` are untouched. **Limitation:** the unselected 250-pair alternative (indices 0–249) includes all 30 layouts, so a future P7-A expansion beyond 200 must exclude them by a pre-fixed rule, or state they are no longer unseen (contract §3.2). |
| 3 | Distinguish engineering acceptance from evidence that P5 is superior | Contract §7: what a pass states, what must not be stated or implied, a mandatory verbatim statement in the report, and that superiority needs a separately approved confirmatory study |
| 4 | Keep the observation-gap incidents and the P5 repeat inconsistencies in the risk register | RR1 and RR2 are kept **open** (§8, contract §10). P7-E reports what bears on them, closes neither, and changes no threshold. |
| 5 | Separate approval of the retention policy from authorization to delete; no P3–P7 evidence may be deleted under this approval | Contract §11 and the retention proposal §2 and §5: approving the policy authorizes no deletion, move or compression. Each deletion needs its own authorization naming the tested procedure, the manifest version and the file set. No P3–P7 evidence, including the 107 eligible P6 development dry runs, may be deleted under the policy approval. |
| 6 | Every safety event, safety-unverified result and stop condition stays fail-closed | Contract §8. The audit of the reused P7 code found **three fail-open gaps**: FC-1, a sequencer exception recorded as an unstopped block; FC-2, a `pgrep` failure read as "no process"; FC-3, an unknown arm state classed as infrastructure-invalid. P7-E closes each in its own code (E2). The frozen P7 code is untouched, and the same gaps are recorded for any P7-A resumption. |
