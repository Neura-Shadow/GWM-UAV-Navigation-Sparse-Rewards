# v3-SIM P7-E release closure: documentation audit, integrity audit, release summary and proposed tag

**Status: PREPARED FOR THE OPERATOR'S REVIEW (2026-11-28).**
* **RC-1 ACCEPTED** (operator, 2026-11-28, "Accept RC-1"): the documentation corrections D1–D12 of §1.1 stand, as
  committed in `7156017`.
* **RC-2 DONE** (operator, 2026-11-28, "Write RC-2"): the clarification record of §1.2 is written (`073e4646…`).
* **RC-3 DONE** (operator, 2026-11-28, "Create the tag"): the annotated tag `v3.7.0-p7e-engineering-validation` exists
  locally.
  * Tag object `441ec508101f94578e1892c7032138414e8e0417`, on commit
    `59a70a142de805d3658f0083182440f2c985e1f0`. That is the closure commit, with RC-1 and RC-2.
  * Its message is the §3 summary with the identities.
  * This note is recorded in the commit after the tagged one.
* **RC-4 DONE** (operator, 2026-11-28, "Push both"): origin has the branch at `66613f9` (fast-forward from
  `a5a0df4`, 50 commits) and the tag. Both were checked with `git ls-remote`. This record is a later local commit.
* RC-5 to RC-7 are still open (§5). No PR is open, and nothing is merged into `main`.
* **Authority:** the operator's request "P7-E Release Closure and Engineering Next-Stage Preparation". It authorizes
  read-only auditing and proposal preparation only.
* **What changed:**
  * documentation corrections and annotations (§1);
  * new read-only diagnostics tools and their evidence outputs, listed below.
* **What did not change:** frozen sources, tests, manifests, accepted reports and run evidence. Nothing was deleted,
  moved or compressed. No SITL ran, no candidate was created, nothing was pushed, merged or tagged, and P7-A was
  not resumed.

**Companion proposals:**
* the [cleanup proposal v3](v3_sim_p7e_cleanup_proposal_v3.md);
* the [P8 proposal](v3_sim_p8_proposal.md) (Navigation Robustness and Recovery).

**New files** (outside every manifest scope):
* tools:
  * `diagnostics/p7e/p7e_cleanup_manifest_v3.py`;
  * its 11 offline tests, `diagnostics/p7e/test_p7e_cleanup_manifest_v3.py`;
  * `diagnostics/p8/p8_hover_timeout_analysis.py`;
* evidence:
  * `docs/evidence/v3_sim_p7e_cleanup_manifest_v3.json` (`ec5b2205…`);
  * `docs/evidence/v3_sim_p8_hover_timeout_analysis.json` (`cde7fa52…`).

## 1. Documentation audit

### 1.1 Findings and actions

| ID | Document | Statement found | Problem | Action |
|---|---|---|---|---|
| D1 | E5 §4 | "0 safety events and 0 mission-failure flights per controller" | Misleading. 18 missions failed (P4 10, P5 8), all safe `hover_timeout` aborts. The sentence repeated a report field that counts D14 mission-failure *class items* (§1.2), without saying so. | **Corrected** in place, with the field's definition |
| D2 | E5 §1, §5 | "I have not timed…"; "What acceptance would mean" | Written before the run and before the acceptance | **Corrected** (marked as pre-run; past tense) |
| D3 | Engineering plan, status | "DIRECTION AND E1–E11 APPROVED IN PRINCIPLE…" as the headline | Stale headline: P7-E is complete | **Corrected**: "P7-E COMPLETE (2026-11-28)"; the history is kept |
| D4 | Engineering plan, status | "Nothing is frozen or flown…" | False after E3–E5 | **Corrected** |
| D5 | Engineering plan, status | "The real P7-E candidate was FROZEN… `38e23bad…`. No bound file may change until E5." | Names only the first candidate; "until E5" reads as if the binding ended | **Corrected**: both candidates, and bound files release-bound (H9 depends on them) |
| D6 | Engineering plan, after the status | "What was not done: … No candidate was created, no SITL ran…" | Reads as the current state; it describes E0 | **Labelled** "at E0 (2026-11-22)" |
| D7 | Engineering plan §10 | "P7-E's own raw depth (about 63 GB)" | A planning estimate | **Annotated** with the actual: 78.4 GB over 65 P7-E runs; 40.0 GB ELIGIBLE under v3 |
| D8 | Contract §13 items 6–7 | only Policy A `26595bba…` and the freeze `38e23bad…` | The second candidate's production Policy A and freeze were not recorded | **Added**: Policy A r2 `56c264e0…` ("Approve (Recommended)") and freeze `e9561f05…` ("Freeze it (Recommended)"), plus the first candidate's stop and recovery records |
| D9 | Contract, status | "In force for the P7-E stage" | Does not say that the stage is complete | **Annotated**: complete on 2026-11-28 |
| D10 | E3, status | "Nothing has been flown on it." | False after E4 | **Annotated** "at E3", with a release note: stopped, retired, protected |
| D11 | E4, header and Block 1 status | names only `38e23bad…`; "nothing can proceed … until the operator decides" | Stale: the operator decided (recovery, then the re-fly) | **Annotated**: both candidates; block 1 resolved |
| D12 | Retention proposal, status | manifest v2 numbers (107 ELIGIBLE) | Superseded for planning by v3 | **Annotated**: v2 is kept, v3 supersedes it for planning |
| D13 | E1 §4, E2 §7, E8 "Next" | "Next, each needing…" | Dated milestone records | **Left unchanged** (history) |

**The stopped first candidate and its recovery records are preserved.** §2 re-verifies them by hash: the
candidate, its Policy A, the stop record, the ledger, the authorization, the marker and the recovery receipt.

### 1.2 The accepted report JSON

**It does not contain the inconsistency.** In `state/p7e-engineering-report-e9561f05de28.json` (`e2fb9607…`) and
its byte-identical evidence copy:
* `safety_per_controller.<c>.mission_failure_flights` is computed by the frozen
  `p7_analysis.safety` (`bd036a2b…`). It counts valid flights whose frozen classification lists a D14
  **mission-failure class item**. `p7_classify.MISSION_ITEMS` defines those items as exactly `safety:tracking` and
  `safety:response_latency`. None occurred, so the value 0 is correct.
* The mission outcomes are where they belong:
  * R1: P4 20 of 30 and P5 22 of 30 goal successes, so 10 and 8 failures;
  * R2: 8 pairs where neither succeeded;
  * R9: `hover_timeout` 10 and 8.
* The weakness is only the field's name, which a reader can take for "failed missions". The E5 document repeated
  it (D1, now corrected).

**Nothing is overwritten.** The accepted report stays byte-identical; its hash is bound in the E5 records and in
this audit.

**A separate clarification record: WRITTEN on the operator's decision (2026-11-28, "Write RC-2").** It is a
clarification rather than an erratum, because no value is wrong.
* The record: [`v3_sim_p7e_engineering_report_clarification_1.json`](evidence/v3_sim_p7e_engineering_report_clarification_1.json),
  SHA-256 `073e46467579700d1818a386a1d7750bd087ebc294711324c3042423041b0094`, written with exclusive create.
* Every value was checked against the report before writing:
  * the field is 0 for both controllers;
  * the R1 failures are P4 10 and P5 8, equal to the R9 `hover_timeout` counts;
  * there are 0 safety events.
* It also binds:
  * `p7_classify.MISSION_ITEMS` = `("safety:tracking", "safety:response_latency")`;
  * the frozen classifier, `7a46258f…`;
  * `p7_analysis`, `bd036a2b…`.
* Both copies of the report are unchanged (`e2fb9607…`).

As proposed, its core content:
```json
{"contract": "p7e-report-clarification-v1", "report_sha256": "e2fb96071edb77fdaba7da4f56351b26d1959f850c6e50725e95b0dccc12d09c",
 "field": "safety_per_controller.<controller>.mission_failure_flights",
 "meaning": "valid flights whose frozen P7 classification carries a D14 mission-failure class item (safety:tracking or safety:response_latency; p7_classify.MISSION_ITEMS); not the count of failed missions",
 "mission_outcome_failures": {"p4": 10, "p5": 8, "source": "reported_criteria.R1 (valid_flights - goal_success) and R9 abort_reason hover_timeout"},
 "values_changed": false}
```
The file is `docs/evidence/v3_sim_p7e_engineering_report_clarification_1.json`, cited from the E5 document
(decision **RC-2**, §5).

## 2. Release integrity audit (read-only, 2026-11-28)

Every verifier ran from `/tmp` with `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1` and `PYTHONDONTWRITEBYTECODE=1`.

| Item | Check | Result |
|---|---|---|
| P3 manifest | `p3_manifest.py verify state/p3-candidate-087722b.json` | `1e205fec…` ✓ |
| P4 manifest | `p4_manifest.py verify state/p4-candidate-50d5002.json` | `c110bd30…` ✓ |
| P5 manifest | `p5_manifest.py verify state/p5-candidate-40a109b.json` | `2becfbbb…` ✓ |
| P6 manifest | `p6_manifest.py verify state/p6-candidate-8e29f3f.json` | `22292775…` ✓ |
| Frozen P7 candidate | `p7_manifest.py verify state/p7-candidate-a8bd50e.json`; `source_drift` | `0799c37e…` ✓; drift none |
| P7-E candidate 1 (stopped) | `p7e_manifest.py verify state/p7e-candidate-2b6ee5d.json`; drift; Policy A | `38e23bad…` ✓; drift none; `26595bba…` |
| P7-E candidate 2 (accepted) | `p7e_manifest.py verify state/p7e-candidate-2b6ee5d-r2.json`; drift; Policy A | `e9561f05…` ✓; drift none; `56c264e0…` |
| Final report and copy | SHA-256 of `state/p7e-engineering-report-e9561f05de28.json` and of `docs/evidence/v3_sim_p7e_engineering_report.json` | both `e2fb9607…`; identical; status "engineering validation passed"; binds manifest `e9561f05…`, code `8734bbe9…`, plan `15e7f7b4…` |
| Block 1 (accepted) | result; ledger hash recorded in the result; authorization; marker binds it | result `1c8672ab…`, ledger `e7bf02ae…` ✓, authorization `c85d471d…` ✓, marker ✓; unstopped |
| Block 2 (accepted) | same | `62c23933…`, `1723d124…` ✓, `33256d22…` ✓, marker ✓; unstopped |
| Block 3 (accepted) | same; FC-1 view after block 3 | `d4da3561…`, `7a6b3a79…` ✓, `87db8fe1…` ✓, marker ✓; unstopped; complete |
| Stopped attempt (preserved) | block 1 of `38e23bad…`; the recovery receipt | stop record `5f7c2326…` (`sequencer_terminated_externally`), ledger `1e0a0ef4…` ✓, authorization `71e81930…` ✓, marker ✓; receipt `0c7c3c77…`; no result for blocks 2 and 3 |
| Source drift | `git diff 2b6ee5d HEAD` over the P3–P7-E trees and `tests/`; Windows `git status` | no difference; clean apart from the untracked `.codegraph/` |
| Unselected P7-A schedule | `state/p7-schedule-selection-0799c37e….json`; P7 confirmatory authorizations; P7 specs | **absent** (not selected); only the two pilot authorizations exist; layouts `d742b7af…`, schedule `ef74c71d…`, analysis plan `c9c7a6f3…` equal `APPROVED_SPECS`; pilot rule `52c8c880…` kept |
| Live state | P7/P7-E live lock; simulator processes | none; none |
| Tests | the 76 P7-E offline tests (E5 pre-check, 2026-11-28) | 76 passed |

**Git state.**
* Branch `v3/gwm-uav-c2-agentops-planning`, HEAD `4c51c2f` before this closure's commit.
* It is 46 commits ahead of `origin/…` (`a5a0df4`, the P6 release). None of the P7 or P7-E work is pushed, and
  the branch is not merged into `main`.

## 3. Release summary (proposed release notes)

**v3-SIM P7-E: engineering validation of the accepted P4 and P5 stacks through the P7 infrastructure.**
* **Contract:** `p7-engineering-validation-v1` (approved 2026-11-22). Accepted on 2026-11-28.
* **Scope:**
  * 30 fresh layouts (confirmatory pool indices 200–229, disjoint from P7-A's 200);
  * 60 SITL flights in 3 blocks of 10 pairs, PX4 SITL and Gazebo only.
* **Candidate:** `e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608`. It binds:
  * code `2b6ee5d` (16 files);
  * P7 `0799c37e…`;
  * P3 `1e205fec…`, P4 `c110bd30…`, P5 `2becfbbb…`, P6 `22292775…`.
* **Result: "engineering validation passed"** (report `e2fb9607…`). H1–H9 are all met:
  * 0 collisions, 0 unsafe commands, 0 Policy A breaches, 0 safety-unverified flights;
  * minimum truth clearance 0.310 m against the 0.20 m limit;
  * 60 of 60 flights landed and disarmed with own commands 176/400/21;
  * evidence complete;
  * the frozen classifier reproduces all 60 records.
* **Reported (descriptive, no superiority claim):**
  * goal success P4 20 of 30, P5 22 of 30;
  * all 18 failures are safe `hover_timeout` aborts;
  * one R4 breach (slot 27 P4, a 0.640 s perception frame);
  * two R10 integrity events.
* **History kept:** the first candidate `38e23bad…` stopped in block 1 after an external termination. It was
  recovered into a recorded stop and is protected. The batch was re-flown with the same code.
* **Known limitations:**
  * RR1 (the P2 observation-gap RECOVERY) and RR2 (P5 repeat inconsistency) stay open;
  * the 18 hover-timeout failures are the motivation for P8;
  * 30 pairs cannot rank the controllers (McNemar power 3 %).

## 4. Git tag (created locally on 2026-11-28 under RC-3; not pushed)

| Field | Proposal |
|---|---|
| Name | `v3.7.0-p7e-engineering-validation` (the repository's `vX.Y.Z-name` pattern; v3 phase P7) |
| Type | annotated, so that it carries the identities |
| Target | the commit that records this closure, once the operator accepts it. The commit records the doc corrections. `4c51c2f` (the acceptance commit) is the alternative. |
| Message | the release summary of §3, plus the manifest and report hashes |

The commands, for later only, each needing its own authorization (tag RC-3, push RC-4):
```
git tag -a v3.7.0-p7e-engineering-validation <commit> -F <message-file>
git push origin v3/gwm-uav-c2-agentops-planning
git push origin v3.7.0-p7e-engineering-validation
```

## 5. Release-closure plan: decisions for the operator

| ID | Decision | Recommendation |
|---|---|---|
| RC-1 | Accept the documentation corrections D1–D12 (§1.1), as committed locally | **ACCEPTED by the operator on 2026-11-28 ("Accept RC-1")**; the corrections are in commit `7156017` |
| RC-2 | Write the clarification record of §1.2 (no value changes; the report untouched) | **DONE (operator, 2026-11-28, "Write RC-2")**: `v3_sim_p7e_engineering_report_clarification_1.json` (`073e4646…`) |
| RC-3 | Create the annotated tag of §4, locally | **DONE (operator, 2026-11-28, "Create the tag")**: `v3.7.0-p7e-engineering-validation`, tag object `441ec508…` on commit `59a70a1`; local only, not pushed |
| RC-4 | Push the branch (46 + closure commits) and the tag | **DONE (operator, 2026-11-28, "Push both")**: branch `a5a0df4..66613f9` (fast-forward, 50 commits) and tag `v3.7.0-p7e-engineering-validation` (`441ec508…` → `59a70a1`) on origin; no PR or merge |
| RC-5 | Open a PR or merge into `main` | separate; not proposed now |
| RC-6 | Cleanup | see the [cleanup proposal v3](v3_sim_p7e_cleanup_proposal_v3.md) (decisions C-1 to C-8) |
| RC-7 | P8 | see the [P8 proposal](v3_sim_p8_proposal.md) (decisions P8-D1 to P8-D8) |
