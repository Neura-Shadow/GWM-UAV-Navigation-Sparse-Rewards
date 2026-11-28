# v3-SIM P8 release `v3.8.0-p8-navigation-recovery`

**What this release is:** the completed P8 engineering phase (Navigation Robustness and Recovery, contract
`p8-navigation-recovery-v1`) and the Evidence Retention Manager with its executed cleanup C-7. The annotated tag
`v3.8.0-p8-navigation-recovery` points to the commit that adds this note.

## 1. The basis of the acceptance

**The operator accepted P8 on 2026-11-28 ("Accept P8") on two records together:**

| Record | SHA-256 | Status |
|---|---|---|
| The engineering report (`p8_report.py`, run once) | `f03253d064be761079a319b024a3540a0eb831a849fc2c76c476125f5ea477ec` | **"engineering validation not passed"**, single reason "H9 not met" |
| The independent H9 addendum (`diagnostics/p8/p8_h9_addendum.py`, outside the candidate) | `f2903a17211a7b39f8ba98ffb7c406c48847f8eefc8795904251c33d2b0a0783` | **"explained"** on all 60 flights |

* **The report keeps its status.** It is the first and only report. It was not rerun, and no threshold changed. The
  acceptance is the operator's judgement on the report together with finding F-R1 and the addendum. It is not a pass
  statement by the report.
* **Finding F-R1:**
  * H9 compares each P8 assessment with a fresh P7-E assessment on every key except `stop_reasons`.
  * That includes the `contract` label, which the P8 sequencer rewrites by design. So the comparison fails on every
    flight.
  * Every other H9 check passed 60 of 60 in the report itself: the manifest, the worlds, the classifier re-run, H10
    and the unchanged run files.
* **The addendum:**
  * Rule O, the report's comparison as it is, reproduces the report on 60 of 60 and differs on `contract` alone.
  * Rule C is stricter: the exact relabel, the stop reasons derived from H10, and the `h10` summary equal to its
    record. It holds on 60 of 60.
  * No run file changed.
* **The defect is not fixed in the P8 candidate**, because its 37 bound files must not change. Any later phase that
  reuses `p8_report.py` must fix it there and test `h9` on a P8-format assessment.
* The full write-up is [the report note](v3_sim_p8_report.md) §4–§7. The byte-identical copies are
  [`v3_sim_p8_engineering_report.json`](evidence/v3_sim_p8_engineering_report.json) and
  [`v3_sim_p8_h9_addendum.json`](evidence/v3_sim_p8_h9_addendum.json).

## 2. Identities

| Item | Identity |
|---|---|
| Candidate | manifest `fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`; 37 bound files (canonical `10eb4660…`); production Policy A `befd23a3…`; build `1a78c6ba…` |
| Bound baselines | P7-E `e9561f05`, P7 `0799c37e`, P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775` |
| Specs | layouts `62ae950c…`, schedule `df90cf7b…`, recovery `dee90d51…`, report plan `a313a034…` |
| Offline records | V1 `1d6cc0e2…`, V2 `912297fb…` |
| Block results | V3 `8d768d01…`, V4 `e1d48d76…`, V5 `43bf1db8…`; ledgers `34c67ef8…`, `f6de2129…`, `f40d23af…` |
| Report code | `815b0f7c…` (bound); addendum tool `5ef9d5e9…` (not bound) |

## 3. Results (the report; outcomes descriptive only)

* **Safety and integrity:**
  * 3 blocks, none stopped; 60 of 60 flights valid, no replacement;
  * 0 safety events, 0 collisions, 0 unsafe commands, 0 Policy A breaches, 0 safety-unverified flights;
  * minimum truth clearance 0.342 m (limit 0.20 m);
  * every flight landed and disarmed with its own commands 176/400/21.
* **Hard criteria:** H1–H8, H10 and H11 met. H9 is met in every substantive part, with its assessment comparison
  explained by the addendum.
* **Recovery:** 14,647 recovery actions, including 176 recovery translations, each reproduced by H10; 3 FC-4 refusals.
* **Goal success** (Clopper–Pearson 95 %):

  | Stage | P8-P4R | P8-P5R |
  |---|---|---|
  | V3 (P7-E hover-timeout layouts) | 4/10 [0.12, 0.74] | 4/10 [0.12, 0.74] |
  | V4 (10 held-out layouts) | 4/10 [0.12, 0.74] | 4/10 [0.12, 0.74] |
  | V5 (P7-E both-success layouts) | 10/10 [0.69, 1.00] | 10/10 [0.69, 1.00] |

* **Against the P7-E baseline:** on V3, 6 flights went from failure to goal success. On V5 there was no regression.
* **Other findings:**
  * F-B2-1: block 2 slot 14 P8-P5R was a P2 `stale_action` fail-closed landing after a 0.79 s stall in the accepted
    controller. Recovery never acted. It bears on RR1 and RR-P8-5.
  * R10: 14 `policy:ledger:decision_mismatch` items, all reconciled exactly by D-M3-1; 2
    `cleanup:gazebo:forced_kill_after_grace` items.
* **Required statement (contract §12):** *"P8 is an engineering validation of a recovery layer. Its per-layout
  outcomes are descriptive; with 10 layouts per stage they do not establish a rate, and they do not show that any
  controller or variant is superior."*
* **Not claimed:** superiority of any variant or stack, statistical significance, real-hardware behaviour, results
  outside the declared layouts, mission, speed, static boxes or this host, that recovery resolves RR1 or RR2, and any
  change to P2–P7-E acceptance.
* **Risk register:** RR1, RR2 and RR-P8-1 to RR-P8-5 stay open.

## 4. Evidence retention (same release)

* `tools/evidence_retention/` holds the dependency-aware executor and its 29 offline tests. The
  [design](v3_sim_evidence_retention_design.md) covers profiles, triggers, measurements and the open decisions
  R-1 to R-4 and C-8 to C-10.
* Cleanup C-7 was executed under its two file-level authorizations:
  * the P6 development raw depth: 624 files, 26.68 GB, receipt `ce326ddf…`;
  * the raw depth of 31 P7-E goal successes: 248 files, 34.94 GB, receipt `68bb51b1…`.
* The flight records themselves are kept. After the P7-E set, the frozen classifier reproduced 3 of 3 sampled
  records.
* All P8 runs keep their complete raw evidence (D4-A).

## 5. Commit hashes quoted in records

On 2026-11-28 the repository history was rewritten:
* the automatically inserted AI co-author trailers were removed;
* the root commit's author was corrected;
* the v3 line was consolidated into milestone commits.

Every source tree at a milestone boundary is unchanged.

**Records keep the original hashes by design.** That covers manifests, Policy A approvals, run summaries,
evidence files and the documentation. Each commit that carries provenance has a published commit with an identical
tree:

| Original | Published | Meaning |
|---|---|---|
| `087722b` | `feaba7b` | accepted P3 candidate source |
| `50d5002` | `5f9811a` | accepted P4 candidate source |
| `40a109b` | `ffbfc58` | accepted P5 candidate source |
| `13c7dde` | `332e8b1` | P5 model checkpoint `code_commit` |
| `c08bfc8` | `356e6af` | accepted P6 candidate source |
| `8e29f3f` | `13a65e7` | accepted P6 frozen head |
| `a8bd50e` | `6534106` | P7 candidate `0799c37e` |
| `2b6ee5d` | `9cfd318` | P7-E candidate source |
| `66678f2` | `a5f3755` | accepted P7-E frozen head (`e9561f05`) |
| `2a0ae57` | `49e504b` | accepted P8 candidate source and frozen head |

The complete original-to-published mapping and the original history are kept by the operator.

## 6. Not in Git

The following stay outside the repository:
* runtime data and WSL state;
* raw simulation recordings;
* local backups;
* local tool caches (`.codegraph/`, now ignored).
