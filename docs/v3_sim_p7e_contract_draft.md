# v3-SIM P7 contract revision `p7-engineering-validation-v1` — DRAFT for operator review

**Status: DRAFT (2026-11-22). Not approved; nothing in it is in force.** It is proposed in the
[P7-E engineering plan](v3_sim_p7e_engineering_plan.md).

**Relation to `p7-evaluation-v1`.** If approved, it governs the next P7 stage. The existing contract
`p7-evaluation-v1` ([contract](v3_sim_p7_contract.md)) stays unchanged and in the record, with P7-A suspended (§9).

## 1. Scope and claims

* **Claimed:** an engineering system validation of the accepted P4 (`c110bd30`) and P5 (`2becfbbb`) stacks in PX4
  SITL, through the P7 infrastructure. It covers 30 paired, previously unseen layouts (60 flights). Safety,
  integrity, timing, reliability, cost and evidence criteria are reported per controller and as paired comparisons.
* **Not claimed:**
  * that either ranker is superior;
  * any statistically significant difference;
  * real-hardware behaviour;
  * results outside the declared layouts, mission, speed, static boxes or this host;
  * any change to P2–P6 acceptance.
* **Engineering success is not evidence of superiority** (plan §2, §7).

## 2. Boundaries (normative)

* **C1–C10 of `p7-evaluation-v1` apply unchanged,** except as stated here. In particular:
  * no P2–P6 file changes;
  * no new safety logic;
  * simulation only;
  * pre-registration before the first P7-E flight;
  * no outcome-dependent exclusion or re-flight;
  * one run per block authorization;
  * no push, merge or deletion without explicit words.
* **E-C1.** The frozen P7 candidate `0799c37e…`, the P7 pilot evidence (both attempts and M4), the original
  analysis plan and code, and every existing authorization are preserved unchanged. P7-E code reaches the P7 code
  only by import (plan §4, decision E5).
* **E-C2.** The P7-E candidate binds the P3–P6 baselines through the P7 candidate's verification, plus:
  * the P7 candidate;
  * the P7-E sources and tests;
  * the P7-E schedule and report plan;
  * the host check;
  * a P7-E Policy A approval.

## 3. Layouts, pairs, order

* **Layouts:** 30, from the frozen confirmatory pool of `p7_layouts.json` (`d742b7af…`), indices 200–229 in seed
  order (decision E3). Each keeps its frozen world SHA-256 (C6).
* **Pairs:** each layout is flown once by P4 and once by P5, back to back.
* **Order:** a permutation from the SHA-256 counter stream seeded by the layouts' canonical hash, with new labels.
  Within each block of 10 pairs, 5 fly P4 first.
* **Blocks:** 3 blocks of 10 pairs.

## 4. Classification (normative; unchanged)

* **Validity:** valid if there is armed evidence. An infrastructure-invalid flight is re-flown once, at the end of
  its block. Replacements are capped at 10 % of the block's flights (2 of 20).
* **Outcomes, classes and stops:**
  * contract `p7-evaluation-v1` §5, with D13, D14 and D15;
  * the classifier `p7_classify.py` of candidate `0799c37e`, unchanged;
  * every §7.3 stop condition and the §7.4 disk floor, unchanged.

## 5. Engineering acceptance (normative)

* **Hard criteria H1–H9** (plan §5) decide engineering completion. All their limits are existing accepted limits.
* **Reported criteria R1–R10** are measured against their accepted limits. A breach is an integrity event entered in
  the risk register.
* **Statistical significance is not required** and is not tested.

## 6. Reporting (normative)

* P4 and P5 are reported separately and as paired comparisons, with descriptive intervals:
  * Clopper–Pearson for rates;
  * Tango for the paired success difference;
  * Hodges–Lehmann for time to goal and path length;
  * zero-event bounds for safety.
* No p-value (decision E7).
* The report states the 30-pair limits of plan §7.
* The report code and plan are frozen and hash-bound before the first P7-E flight.

## 7. Evidence and retention

* **Per flight:** as `p7-evaluation-v1` §8.
* **Retention:** decision D4 revised as D4-A and D4-B ([retention proposal](v3_sim_p7e_retention_proposal.md)). No
  deletion happens without a separately tested and separately approved procedure.

## 8. Approvals and authorization

As D12:
* a production Policy A approval for the P7-E candidate;
* one authorization per block, with `max_attempts` 1 and a permanent marker;
* an optional development dry run under its own approval (decision E8).

## 9. Mapping from the existing decisions

| Decision | In `p7-evaluation-v1` | In this revision |
|---|---|---|
| D1 scope and order | P7-A confirmatory first; P7-B and P7-C later | **changed:** P7-E engineering validation is the next stage. P7-A is suspended, not cancelled, and keeps its unflown 200-pair set (E3). P7-B and P7-C are unchanged (later, each decided separately). |
| D2 primary endpoint | paired goal success, exact McNemar, α 0.05 | **replaced** by the engineering criteria H1–H9 and R1–R10; goal success is reported (R1, R2); no test |
| D3 sample size | 200 pairs (ceiling 250) after a pilot | **changed:** 30 pairs; the pilot is complete and its rule's selection (200) is not written |
| D4 retention | full evidence for every flight; no deletion | **revised** into D4-A and D4-B (E9, E10); still no deletion without a separate approval |
| D5 backup first | Selective Dependency Backup | **unchanged; done** (Parts A–D and E, verified) |
| D6 validity and replacement | pre-arming failures re-flown once; cap 10 % | **unchanged** (2 of 20 per block) |
| D7 clearance margin | 0.05 m, for non-inferiority | **not applicable** (no non-inferiority test); clearance is judged against the 0.20 m safety limit (H2) and reported per pair |
| D8 layout source | the bound P5 generator; seeds 4,000,000 and 5,000,000; split-match exclusion | **unchanged source:** the same frozen pool (`d742b7af…`), indices 200–229 (E3) |
| D9 secondary order | clearance NI, then time to goal, then path | **not applicable** (no fixed-sequence testing); all are reported descriptively |
| D10 host | this host; overlay and AC before every block | **unchanged** |
| D11 architecture | a new P7 tree subclassing the accepted runners | **extended:** the `px4_gazebo_p7e` tree reuses the frozen P7 code by import (E5) |
| D12 approvals | Policy A per candidate; per-block authorization; pilot authorization | **unchanged** for the P7-E candidate and its 3 blocks; an optional dry-run approval (E8) |
| D13 declaration label | `p7_not_predeclared` | **unchanged** |
| D14 result classes | four classes; stops | **unchanged** |
| D15 fail-closed window and ULog P2 facts | — | **unchanged** |
| M1 specs (`d742b7af`, `ef74c71d`, `c9c7a6f3`) | approved | the layouts are reused; the 200/250 schedules and the analysis plan stay unchanged and unused by P7-E |
| Frozen analysis code `bd036a2b` | for P7-A | unchanged; P7-E's report reuses its interval functions by import, labelled descriptive |
