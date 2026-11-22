# v3-SIM P7 contract revision `p7-engineering-validation-v1` — APPROVED

**Status: APPROVED by the operator (2026-11-22, "Approve contract"). In force for the P7-E stage.**
* It governs the next P7 stage, P7-E. P7-A (`p7-evaluation-v1`) is suspended, not cancelled.
* **This approval authorizes nothing to run, freeze or delete.** Each step of §13 (2–10) still needs its own
  explicit authorization.
* The retention policy (§13, step 2) is approved separately.

**The basis:**
* The operator approved the P7-E direction and the recommended E1–E11 design **in principle** (2026-11-22), subject
  to clarifications 1–6. This text incorporates them (§3.2, §7, §8, §10, §11, §12).
* The design rationale is in the [P7-E plan](v3_sim_p7e_engineering_plan.md).

**Approving this contract authorizes nothing to run.** Every implementation milestone, Policy A approval, candidate
freeze, dry run and benchmark block needs its own explicit authorization (§13).

**Relation to `p7-evaluation-v1`.** The existing contract `p7-evaluation-v1` ([contract](v3_sim_p7_contract.md))
stays unchanged and in the record. Its confirmatory study P7-A is suspended, not cancelled.

## 1. Scope and claims

* **Claimed on completion:** an engineering system validation of the accepted P4 (`c110bd30`) and P5 (`2becfbbb`)
  stacks in PX4 SITL, through the P7 infrastructure. It covers 30 paired, previously unseen layouts (60 flights).
  Safety, integrity, timing, reliability, cost and evidence are reported per controller and as paired comparisons.
* **Not claimed:**
  * that either ranker is superior;
  * any statistically significant difference;
  * real-hardware behaviour;
  * results outside the declared layouts, mission, speed, static boxes or this host;
  * any change to P2–P6 acceptance.

## 2. Boundaries (normative)

* **C1–C10 of `p7-evaluation-v1` apply unchanged,** except as stated here:
  * no P2–P6 file changes;
  * no new safety logic;
  * simulation only;
  * pre-registration before the first P7-E flight;
  * no outcome-dependent exclusion or post-arming re-flight;
  * one run per block authorization;
  * no push, merge or deletion without explicit words.
* **E-C1, preservation.** These are preserved unchanged:
  * the frozen P7 candidate `0799c37e…`;
  * the P7 pilot evidence (both attempts) and the M4 dry run;
  * the M1 specifications and the original analysis plan and code;
  * every existing approval and authorization.

  The 200-pair schedule-selection record is not written. P7-E code reaches the P7 code only by import, from a
  separate tree, so the P7 tree and its frozen candidate keep verifying.
* **E-C2, binding.** The P7-E candidate binds:
  * the P3–P6 baselines, through the P7 candidate's verification;
  * the P7 candidate;
  * the P7-E sources and tests;
  * the P7-E schedule and report plan;
  * the host check;
  * a P7-E Policy A approval.

## 3. Layouts, pairs, order

### 3.1 Design

* **Layouts:** 30 from the frozen confirmatory pool of `p7_layouts.json` (`d742b7af…`), **indices 200–229** in seed
  order. Each keeps its frozen world SHA-256 (C6).
* **Pairs:** each layout is flown once by P4 and once by P5, back to back.
* **Order:** a permutation from the SHA-256 counter stream seeded by the layouts' canonical hash, with new labels.
  Within each block of 10 pairs, 5 fly P4 first.
* **Blocks:** 3 blocks of 10 pairs (20 flights each).

### 3.2 Non-contamination of P7-A (clarification 2)

**Verified against the frozen files on 2026-11-22.**
* The 200-pair P7-A schedule (`ef74c71d…`) contains exactly pool indices 0–199. **None** of the 30 P7-E layouts is in
  it, and none was flown in the pilot.
* P7-E uses its own schedule file and changes neither `p7_schedule.json` nor `p7_layouts.json`.

**Limitation, recorded:**
* The pre-registered but unselected 250-pair alternative contains pool indices 0–249, so it includes all 30 P7-E
  layouts (in its blocks 1–10, 12 and 13).
* Any future P7-A expansion beyond 200 pairs in pool order would therefore include layouts that P7-E has already
  flown. Such an expansion must either exclude them by a rule fixed before its first flight, or state that those
  layouts are no longer unseen.

## 4. Classification (normative)

* **Inherited unchanged:** validity, outcomes, the D14 classes and D15, as in `p7-evaluation-v1` §5 and the frozen
  classifier of candidate `0799c37e`. The P7-E rules of §8 are added on top.
* **Infrastructure-invalid flights:** re-flown once at the end of the block, capped at 10 % of the block's flights
  (2 of 20).

## 5. Engineering acceptance (normative)

**Hard criteria.** These decide engineering completion. Every limit is an existing accepted limit:

| ID | Criterion | Limit (source) |
|---|---|---|
| H1 | collisions | 0 (P4 `collisions_allowed`) |
| H2 | minimum truth clearance | ≥ 0.20 m (P4) |
| H3 | unsafe commands | 0 (P6 §6.3, D14) |
| H4 | Policy A breaches | 0 (Policy A `bf1ef9b1…`) |
| H5 | landing and disarm | every valid flight lands and disarms, with the command sequence 176/400/21 (P2, D15) |
| H6 | safety-unverified flights | 0 (D14, §8) |
| H7 | evidence | complete for every flown flight (contract §8) |
| H8 | batch execution | all scheduled flights flown, or every gap explained by a stop rule; replacements within D6 |
| H9 | reproducibility | re-classification and every hash reproduce |

**Reported criteria R1–R10** are measured against their accepted limits. A breach is an integrity event entered in
the risk register (§10):
* success rate;
* paired outcomes;
* P2 timing and freshness;
* P3 and P4 timing;
* P5 inference;
* tracking and response latency;
* time to goal and path length;
* host;
* abort types;
* integrity events.

**Statistical significance is not required and not tested.**

## 6. Reporting (normative)

* P4 and P5 are reported separately and as paired comparisons, with descriptive intervals:
  * Clopper–Pearson for rates;
  * Tango for the paired success difference;
  * Hodges–Lehmann for time to goal and path length;
  * zero-event bounds for safety.
* No p-value.
* The report plan and code are frozen and hash-bound before the first P7-E flight.

## 7. Engineering acceptance is not evidence of superiority (clarification 3, normative)

1. **What a passed P7-E states:** "the accepted P4 and P5 stacks completed the declared 60-flight engineering
   validation through the P7 infrastructure, with every hard criterion H1–H9 met". Nothing more.
2. **What the report must not state or imply:** that P5 is better, worse or equivalent to P4. That includes success
   rate, time, path, clearance and safety. A difference in observed counts is reported as an observation with its
   interval, never as a finding.
3. **The required statement.** The report carries this statement verbatim:

   > "This engineering validation was not designed to compare P5 with P4. With 30 pairs, a test of the planning
   > effect would have about 3 % power. Observed differences between the controllers are descriptive and do not
   > show that either is superior."

   It also carries the 30-pair limits of plan §7.
4. **Where superiority belongs:** any superiority claim would need a separately approved confirmatory study, such
   as a resumed P7-A.

## 8. Fail-closed rules (clarification 6, normative)

**Inherited, and kept fail-closed:**

| Condition | Behaviour |
|---|---|
| safety event (H1–H5, D14) | the flight is a failure; **the batch stops**; nothing more flies until the operator decides |
| safety-unverified (D14, D15): missing or broken truth records, a missing contact ledger, a missing or crashed evaluation, an unreadable ULog under D15, a missing Policy A check | **the batch stops** |
| an unclassifiable flight, or a classifier exception | **the batch stops** |
| a runtime that created no evidence | **the batch stops** |
| manifest drift before a flight | **the batch stops**; the remaining flights are listed as not flown |
| a simulator process outliving its flight | **the batch stops** |
| replacements over the per-flight or per-block limit | **the batch stops** |
| a failed host check (overlay, AC, CPU, process, lock, disk floor) | **not flown**; the authorization is not consumed |
| a block result missing, stopped, or not accounting for every planned flight | the next block refuses to start |

**Added by P7-E.** These close three gaps the audit found in the reused P7 code, and are implemented in P7-E's own
code:

| ID | Gap found | P7-E rule |
|---|---|---|
| FC-1 | An exception inside the P7 sequencer's block loop wrote a result with `stopped: false` and no flights (`run_p7_block.py:268–279`), which the next block's check would accept | Any exception during a block is a **stop** (`sequencer_exception:<type>`). Every unflown flight is listed, and the flown flights are recovered from the fsynced ledger. A block counts as complete only if its result is unstopped **and** accounts for every planned flight. |
| FC-2 | A failure of `pgrep` itself (exit code 2 or 3) read as "no process" (`p7_host.py:66`), for both the host check and the residual check | A process check that cannot run is a **failed** check: before a block, not flown; after a flight, a stop. |
| FC-3 | With the ULog unreadable, a flight whose P2 controller reached `REQUEST_ARM` but not `TAKEOFF` was classed infrastructure-invalid and re-flown, so its safety record was never examined | An unknown arm state is **safety-unverified**, and the batch stops. A flight is infrastructure-invalid only when its records show it never reached `REQUEST_ARM`, or the ULog shows it never armed. |

**Note for P7-A.** The same three gaps exist in the frozen P7 code. If P7-A is ever resumed, they need a new P7
candidate.

## 9. Evidence

As `p7-evaluation-v1` §8.

## 10. Engineering risk register (clarification 4, normative)

* The register of plan §8 is part of this contract and is reported with P7-E.
* **RR1, P2 observation-gap RECOVERY,** and **RR2, P5 repeat-behaviour inconsistency,** are kept **open**. P7-E
  reports what bears on them:
  * the observation-gap count per controller (R3, R9);
  * the stall statistics (R8).

  P7-E closes neither. Its single flight per layout cannot measure repeat behaviour, so RR2's evidence stays the
  pilot's (3 of 20 repeats inconsistent, all P5).
* No threshold is changed in response to any risk.

## 11. Retention, separate from deletion (clarification 5, normative)

* **Policy only.** Approving the retention policy D4-A/D4-B
  ([retention proposal](v3_sim_p7e_retention_proposal.md)) **authorizes no deletion**, no move and no compression.
* **What a deletion needs.** Its own explicit authorization, naming:
  * the procedure, which is tested beforehand;
  * the manifest version;
  * the exact file set.
* **Protected evidence.** No existing P3–P7 evidence may be deleted under the policy approval or under this
  contract's approval. That includes the 107 P6 development dry runs that the read-only manifest lists as eligible.
* **Inventory.** The manifest must be reconciled (every entry of `runs/` accounted for) before anything is listed as
  eligible.

## 12. Mapping from the existing decisions

| Decision | In `p7-evaluation-v1` | In this revision |
|---|---|---|
| D1 scope | P7-A confirmatory first | **changed:** P7-E next; P7-A suspended, not cancelled; P7-B and P7-C unchanged |
| D2 primary endpoint | paired goal success, exact McNemar | **replaced** by H1–H9 and R1–R10; no test (§5–§7) |
| D3 sample | 200 pairs (ceiling 250) after a pilot | **changed:** 30 pairs; the pilot is complete; its selection (200) is not written |
| D4 retention | full evidence; no deletion | **revised** to D4-A/D4-B; still no deletion without a separate authorization (§11) |
| D5 backup | Selective Dependency Backup | **unchanged; done** (Parts A–E, verified) |
| D6 replacement | once; cap 10 % | **unchanged** (2 of 20 per block) |
| D7 NI margin | 0.05 m | **not applicable**; clearance is judged against 0.20 m (H2) |
| D8 layout source | the frozen generator pool | **unchanged source:** indices 200–229 (§3.2) |
| D9 secondary order | fixed sequence | **not applicable** |
| D10 host | this host; overlay and AC | **unchanged**, with FC-2 |
| D11 architecture | a P7 tree subclassing the accepted runners | **extended:** a separate P7-E tree reusing P7 by import (E-C1) |
| D12 approvals | Policy A per candidate; per block | **unchanged**, and §13 |
| D13 label | `p7_not_predeclared` | **unchanged** |
| D14 classes and stops | four classes | **unchanged**, with FC-1 to FC-3 added (§8) |
| D15 fail-closed window and ULog facts | — | **unchanged** |

## 13. Authorizations required, each separate and explicit

1. Approval of this contract.
2. Approval of the retention policy D4-A/D4-B. It is separate from any deletion (§11). **Done on 2026-11-22
   ("Approve retention"):** policy only; no deletion authorized or made.
3. **E1** specifications: the 30-pair schedule and the report plan, approved by hash. **Done on 2026-11-22
   ("Approve specs"):** schedule `9b3deacb…` and report plan `15e7f7b4…`.
4. **E2** implementation (offline): the P7-E tree, FC-1 to FC-3 and the tests. Its acceptance is separate.
   **Done on 2026-11-22:** authorized ("Authorize E2") and accepted ("Accept E2"); code commit `2b6ee5d`.
5. The **development dry run** of the new launch path (2 flights, S2), with its own dry-run approval. **Done on
   2026-11-22:** development candidate `2cc7a579…`; exit criterion met; accepted ("Accept E8").
6. The **production Policy A approval** for the P7-E candidate. **Done on 2026-11-22:**
   `state/p7e-policy-A-approval-2b6ee5d.json` ("Approve (Recommended)").
7. The **real freeze** of the P7-E candidate. **Done on 2026-11-22:** `state/p7e-candidate-2b6ee5d.json`, manifest
   `38e23bad…` ("Freeze it (Recommended)").
8. **Block 1**, **block 2** and **block 3**, each with its own authorization.
9. **E5**: the engineering report and its acceptance.
10. Any future cleanup procedure, tested and then approved, and each execution of it.
