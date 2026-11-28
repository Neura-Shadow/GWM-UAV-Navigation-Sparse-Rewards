# v3-SIM P8: the engineering report

**Status: RUN ONCE (2026-11-28, operator "Authorize the report"). The report records "engineering validation not
passed", with one reason: "H9 not met" (§4).**
* **The cause is a defect in the bound report code, not in the flights** (finding F-R1, §5):
  * H9 compares each P8 assessment with a fresh P7-E assessment on every key except `stop_reasons`;
  * that includes the `contract` label, which the P8 sequencer rewrites by design;
  * so all 60 comparisons differ on that label alone. Every substantive field reproduces 60 of 60.
* The report is kept as written. There is no rerun and no change to any threshold. **The operator decides** what
  follows (§5.3).
* **Option (A), an out-of-tree addendum, was run once: status "explained"** (§6; operator "Prepare addendum (A)",
  then "Authorize the addendum"). On all 60 flights, the only difference is the `contract` label, and the stricter
  corrected comparison holds. The report itself is unchanged and still reads "not passed".
* **P8's acceptance is not decided.** Whether it may rest on the report and the addendum is the operator's decision.
* It was prepared under "Prepare the P8 report", and nothing else is authorized: no push, merge, tag or cleanup, and
  no change to any bound file.

**Candidate:** `state/p8-candidate-2a0ae57.json`, manifest
`fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a` ([M3 record](v3_sim_p8_m3_candidate.md) §8.3).

**Rules:**
* contract [`p8-navigation-recovery-v1`](v3_sim_p8_contract_draft.md) §6, §7, §10 and §12;
* the approved report plan `p8-report-plan-v1` (canonical `a313a034…`);
* the report code as accepted in P8-M2, with the D-M3-1 (a) reconciliation ([M3 record](v3_sim_p8_m3_candidate.md) §7).

**Flights:** the 60 flights of `p8-validation-60` ([validation blocks](v3_sim_p8_validation_blocks.md)): 3 blocks,
none stopped, 60 of 60 valid.

## 1. What the run does

It is offline only: no simulator, no flight, and no change to any bound file or run file. It runs the bound
`simulation/px4_gazebo_p8/analysis/p8_report.py` (`815b0f7c…`) **once**:

```
cd /tmp && GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  /mnt/d/GWM-UAV-Navigation-Sparse-Rewards/simulation/px4_gazebo_p8/analysis/p8_report.py \
  --manifest /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json \
  --output   /home/joker0625/uav_autonomy/state/p8-engineering-report-fe2a0c6e9052.json
```

**Inputs:**
* the 3 block results and their ledgers;
* for each of the 60 flights, accepted only if its hash matches the block result:
  * the summary;
  * the accepted P4 or P5 evaluation;
  * the frozen P7 flight record;
  * the P8 assessment and the H10 record;
  * the recovery ledger;
* the approved P8 specs. The V3 and V5 baselines are the frozen layout spec's `p7e_outcomes`, taken from the
  accepted P7-E report `e2fb9607…`. **No P7-E run is read,** and no baseline is re-flown. V4 has no baseline.

**Hard criteria:**
* **H1–H7** use the accepted P7-E report functions unchanged, imported by path:
  * collisions 0; truth clearance ≥ 0.20 m; unsafe commands 0; Policy A breaches 0;
  * landing and disarm with 176/400/21; safety-unverified 0; evidence complete.
* **H8:** the block accounting against the approved P8 plan (none stopped, every flight accounted for, replacements
  within the cap of 2).
* **H9:**
  * The frozen P7 classifier is re-run on each of the 60 flights, through its own wrapper (`verify_p7_evidence.sh`,
    `99359ade…`, bound by P7 `0799c37e`). Each run gets a temporary symlink view in `/tmp` (`p8-h9-…`), and the
    classifier writes its record only into that view.
  * Each re-run must reproduce the flight record exactly. FC-3 and H10 must reproduce each assessment and H10 record.
  * No run file may change (size and mtime).
  * Every world must equal its approved hash, and the P8 manifest (with P7-E, P7 and P3–P6 inside it) must
    re-verify.
* **H10:** every flight's H10 record passed.
* **H11:** the offline records V1 and V2 are bound by the manifest and record a pass.

**Reported criteria R-P8-1 to R-P8-10:** descriptive only.
* Goal success uses Clopper–Pearson 95 % intervals through the frozen `p7_analysis.clopper_pearson`.
* There are no p-values and no superiority test. The effectiveness target is none (P8-C1).
* A breach of an accepted limit is the accepted evaluator's own failure item. It goes into the risk register, and is
  not by itself an engineering failure (report plan, `engineering_completion`).

**"Engineering validation passed"** requires H1–H11 all met, the declared 60 flights completed, and R-P8-1..10 all
computed. The report then carries the pass statement and, in every case, the contract's §12 statement verbatim.

**Output:** written once (exclusive create). The tool refuses to run if a report exists. It prints the status and
exits 0 only for "engineering validation passed".

**Launch:** detached, as the blocks were (`setsid nohup`, log `~/uav_autonomy/logs/p8-report-fe2a0c6e9052.log`).
* This way the end of a session cannot kill it.
* If it dies before writing, there is no report and nothing changes. A second run then needs a new authorization.

**Duration:** minutes. The P7-E report took 3.4 min for its 60 flights; P8 adds the H10 reproduction.

**After the run** (part of the same authorization):
* copy the report byte for byte to `docs/evidence/v3_sim_p8_engineering_report.json`, and record its hash;
* write the results here: the status, H1–H11, R-P8-1..10, the risk register, the required statement and the
  not-claimed list;
* commit this note and the copy together, locally. Nothing is pushed.

**If the status is "not passed":**
* the report is kept as written, with no rerun and no change to any threshold;
* the operator decides (report plan, `engineering_completion`).

## 2. Known before the run (from the block records; not computed here)

These items are already recorded in the [validation blocks](v3_sim_p8_validation_blocks.md) note. The report will
compute its own values.
* **F-B2-1** (block 2, slot 14 P8-P5R): a P2 `stale_action` fail-closed landing after a 0.787 s control callback.
  * It carries `p2:timing` and policy inference items, which fall under R-P8-5, R-P8-7 and R-P8-10.
  * It bears on RR1 and RR-P8-5.
* **R-P8-10:**
  * 14 `policy:ledger:decision_mismatch` items, each reconciled exactly by D-M3-1 in a read-only check;
  * 2 `cleanup:gazebo:forced_kill_after_grace` items.
* **V5:** all 20 flights reached the goal, as their baselines did, and recovery never triggered.

## 3. Pre-check (read-only, 2026-11-28)

| Check | Result |
|---|---|
| Manifest `fe2a0c6e…` verifies from `/tmp` (P7-E `e9561f05`, P7 `0799c37e`, P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775` inside), development false | yes |
| Source drift | none |
| Specs identity equals the approved specs | yes |
| Report code `815b0f7c…`, equal to its entry among the 37 bound P8 files | yes |
| Report plan canonical `a313a034…` (checked by the tool's own `plan()`); schedule canonical `df90cf7b…` | both match |
| Offline records V1, V2 equal the manifest's | yes |
| Block results 1, 2, 3 present, for this manifest, none stopped | yes |
| Ledgers: hashes equal the results; 20 lines each | yes, 60 flights |
| Evidence complete and hash-matched (summary, evaluation, flight record, assessment, H10 record, recovery ledger) | 60 of 60, nothing missing |
| `p7/layout.json` and `p4_avoidance.yaml` present (H9 and H10 read them) | 60 of 60 |
| Classifier wrapper `99359ade…` present | yes |
| Classifier environment: ROS Jazzy, `venv-px4`, the P2–P5 installs | all present |
| Report output and evidence copy | neither exists |
| Live locks; simulator processes; H9 view directories left in `/tmp` | none; none; none |
| Free space (`/tmp` and the evidence root) | 649.9 GB |
| The 74 P8 offline tests (`tests/test_px4_gazebo_p8_*.py`) | 74 passed (33 s) |
| Working tree (Windows git) | HEAD `71aa3e5`; clean apart from the untracked `.codegraph/` |

**No H or R value was computed in the pre-check.** The D-M3-1 reconciliations in §2 were run read-only during the
block checks, through the report's own `ledger_reconciliation`, which only reads.

## 4. Result (run once, 2026-11-28)

**Records:**
* the report: `state/p8-engineering-report-fe2a0c6e9052.json`, 124 646 bytes, SHA-256
  `f03253d064be761079a319b024a3540a0eb831a849fc2c76c476125f5ea477ec`;
* its byte-identical copy: [`evidence/v3_sim_p8_engineering_report.json`](evidence/v3_sim_p8_engineering_report.json)
  (same hash, compared with `cmp`).

**Run:**
* detached (PID 1001279), 3.9 min (12:35:18 to 12:39:13 UTC); log `~/uav_autonomy/logs/p8-report-fe2a0c6e9052.log`;
* it printed `{"status": "engineering validation not passed", "not_passed_reasons": ["H9 not met"]}`.

**Bindings in the report:**
* report code `815b0f7c…`, report plan canonical `a313a034…`;
* manifest `fe2a0c6e…`, with P7-E `e9561f05…` and the P3–P7 baselines.

**After the run:**
* the manifest still verifies, with no drift;
* no H9 view directory is left in `/tmp`, and no lock is held;
* H9 itself shows that no run file changed.

**Status: "engineering validation not passed".** The pass statement is absent (`null`). The report carries the
contract's §12 statement verbatim (below).

### Hard criteria (10 of 11 met)

| ID | Criterion | Limit | Value | Met |
|---|---|---|---|---|
| H1 | collisions | 0 | 0 | yes |
| H2 | minimum truth clearance | ≥ 0.20 m | 0.342 m (60 valid flights, none unmeasured) | yes |
| H3 | unsafe commands | 0 | 0 | yes |
| H4 | Policy A breaches | 0 | 0 (60 flights checked) | yes |
| H5 | landed and disarmed with own commands 176/400/21 | every valid flight | 60 of 60 | yes |
| H6 | safety-unverified flights | 0 | 0 | yes |
| H7 | evidence complete; ledgers present, hash-matched, lines equal to the results | 100 % | 60/60; 3 ledgers | yes |
| H8 | batch accounting | all 60 flown, or every unflown flight explained by a stop rule | completed | yes |
| H9 | reproducibility | identical | **60 checks differ** (§5) | **no** |
| H10 | recovery invariants re-checked offline | 100 % reproduce; 0 violations | 60/60 (14 647 actions) | yes |
| H11 | offline records V1 (`1d6cc0e2…`), V2 (`912297fb…`) bound and passed | all passed | bound and passed | yes |

**H9 in detail:**

| H9 check | Reproduced |
|---|---|
| The P8 manifest re-verifies; the 3 results bind it | yes; yes |
| Worlds equal their approved hashes | 60 of 60 |
| The frozen P7 classifier, re-run through its wrapper, reproduces each flight record | 60 of 60 |
| FC-3 reproduces each P8 assessment (the report's comparison) | **0 of 60** |
| H10 reproduces each H10 record | 60 of 60 |
| No run file changed | 60 of 60 |

### Reported criteria (descriptive; no p-value; all ten computed, no reported error)

**R-P8-1, goal success** (Clopper–Pearson 95 %):

| Stage | P8-P4R | P8-P5R |
|---|---|---|
| V3 | 4/10 [0.12, 0.74] | 4/10 [0.12, 0.74] |
| V4 | 4/10 [0.12, 0.74] | 4/10 [0.12, 0.74] |
| V5 | 10/10 [0.69, 1.00] | 10/10 [0.69, 1.00] |

**R-P8-2, against the P7-E baseline** (V3 and V5, by flight):
* V3: failure to failure 12, **failure to goal success 6**, goal success to goal success 2;
* V5: goal success to goal success 20.

**R-P8-3, recovery activity:**

| | P8-P4R | P8-P5R |
|---|---:|---:|
| Flights with recovery | 17 of 30 | 14 of 30 |
| Episodes | 52 | 40 |
| Actions (R1 / R2 / R3) | 7 205 (971 / 2 206 / 4 028) | 7 442 (709 / 3 288 / 3 445) |
| FC-4 refusals | 2 | 1 |
| R4 guard starts | 0 | 0 |
| Time in recovery, median (max) | 17.0 s (61.6 s) | 17.9 s (68.9 s) |

**R-P8-4, failure modes:**
* P8-P4R: `hover_timeout` mid-course 10, `transit_deadline` 2;
* P8-P5R: `hover_timeout` mid-course 9, `transit_deadline` 2, and 1 other (slot 14, F-B2-1, with no supervisor abort
  reason);
* no near-goal `hover_timeout` occurred.

**R-P8-5:**
* recovery search wall time: median 4.4 ms, max 26.0 ms (92 searches);
* P2 timing passed on 59 flights. One `p2:timing` breach: block 2 slot 14 P8-P5R (F-B2-1). Its evaluation has no
  timing status.

**R-P8-6** (min, median, max):
* minimum truth clearance: P8-P4R 0.342, 0.488, 1.997 m; P8-P5R 0.362, 0.795, 5.566 m;
* time to goal: P8-P4R 41.0, 44.8, 144.8 s; P8-P5R 41.1, 43.5, 70.5 s (18 goal successes each).

**R-P8-7:**
* perception: 0 breaches of the accepted limits;
* inference over the 29 P8-P5R flights that invoked the policy: max 5.7 ms (limit 10 ms), p99 at most 0.94 ms (limit
  5 ms), participation 1.0;
* the breach items listed are evaluator items:
  * the 14 `policy:ledger:decision_mismatch`;
  * the 4 policy items of slot 14 P8-P5R, which made no policy decision.

**R-P8-8, host:** the host check passed in every block (CPU mean 1.0, 0.4 and 2.8 %); 0 breaches.

**R-P8-9, V5 regressions: none.**

**R-P8-10, integrity events** (all on P8-P5R; none on P8-P4R):
* `policy:ledger:decision_mismatch` 14. The D-M3-1 reconciliation flags 14, **explains 14**, and leaves 0 unexplained.
* `cleanup:gazebo:forced_kill_after_grace` 2.
* The 10 items of slot 14 (F-B2-1).

### Risk register (as the report carries it)

* Open: RR1, RR2, RR-P8-1 to RR-P8-5.
* The rule: P8 closes none by itself, and no threshold changes in response (contract §10).

### Statements carried by the report

* **Required statement (contract §12):** *"P8 is an engineering validation of a recovery layer. Its per-layout
  outcomes are descriptive; with 10 layouts per stage they do not establish a rate, and they do not show that any
  controller or variant is superior."*
* **Not claimed:**
  * superiority of any variant or stack;
  * statistical significance;
  * real-hardware behaviour;
  * results outside the declared layouts, mission, speed, static boxes or this host;
  * that recovery resolves RR1 or RR2;
  * any change to P2–P7-E acceptance.

## 5. Finding F-R1: H9's assessment comparison includes a label the P8 sequencer rewrites

### 5.1 What differs

**H9's assessment check** (`p8_report.py` `h9`):
```
again = p7e_assess.assess(run, f["record"], f["summary"])
mine  = {k: f["assessment"].get(k) for k in again if k not in ("stop_reasons",)}
reproduced = {k: again[k] for k in mine} == mine
```

**What the P8 sequencer writes** (`run_p8_block.py` `assess`, by design):
* the P7-E assessment, with `contract` set to `p8-flight-assessment-v1`;
* its own `stop_reasons` (with FC-5 and FC-6 added);
* an `h10` summary.

**The result:**
* The fresh P7-E assessment carries `contract: p7e-flight-assessment-v1`.
* The comparison excludes `stop_reasons` but not `contract`, so it can never be equal.

### 5.2 Read-only check (2026-11-28, after the run)

The same comparison was repeated read-only over the 60 flights, and the differing keys were listed:
* **For all 60 flights, the only differing key is `contract`** (`p7e-flight-assessment-v1` against
  `p8-flight-assessment-v1`).
* `validity`, `outcome`, `fc3`, `frozen`, `run`, `assessor_sha256`, `flight_record_sha256` and `summary_sha256`
  are equal on all 60.
* No P8 assessment carries a stop reason.
* Calling the assessor with the summary (as H9 does) or without it (as the sequencer did) gives the same result.

**Why the tests missed it:**
* No P8 test runs `h9` (or `build_report`) on a P8-format assessment.
* The 74 tests check the report plan's criteria list and the sequencer's `assess` output, but not this comparison.

**The defect is deterministic.** Any run of this bound report code on these 60 flights gives the same result. A rerun
would change nothing, and none was made.

### 5.3 For the operator (not decided here)

The report stays as written: "not passed". Three ways forward:

| Option | What it means | What it changes | Cost |
|---|---|---|---|
| **(A) Addendum** | A separately authorized, read-only verification outside the candidate (in `diagnostics/p8/`, as the P7-E block-recovery tool was). It repeats H9's assessment comparison, also excluding the sequencer's `contract` label, and records whether all 60 reproduce, with tests. You then decide whether P8's acceptance may rest on the report plus that addendum. | No bound file, run, threshold or report changes. The report still reads "not passed". | small; offline |
| **(B) Fix and re-fly** | Correct `p8_report.py` and add the missing test, then freeze a new candidate (Policy A, freeze) and fly all 60 flights again, each block separately authorized. Then run its report once. The code is bound and the block results bind `fe2a0c6e…`, so a corrected report cannot reuse these flights. | A new candidate. These 60 flights stay as evidence of `fe2a0c6e…`. | about 3.5 h of SITL, about 95 GB, and the authorizations |
| **(C) Close as not passed** | Record P8 as "engineering validation not passed", with F-R1 as the reason, and stop. | nothing | none |

**My recommendation is (A).**
* The failure is a label comparison: every substantive H9 check reproduces 60 of 60, and H1–H8, H10 and H11 are met.
* (A) records that openly, without changing the frozen code or any threshold, and keeps the "not passed" report as
  the first and only report.
* Whether that is enough for acceptance is a judgement on a frozen criterion, so it stays yours.

## 6. Addendum (A): RUN ONCE, "explained" (2026-11-28; operator "Prepare addendum (A)", then "Authorize the addendum")

§6.1–§6.4 are the preparation as it was approved; the result is in §6.5. The tool and its tests were committed
together with the record.

### 6.1 The tool

| File | Lines | SHA-256 |
|---|---:|---|
| [`diagnostics/p8/p8_h9_addendum.py`](../diagnostics/p8/p8_h9_addendum.py) | 234 | `5ef9d5e93fd2ea4daa8980837b780d477866b65e735dea9def9a5a94c29ca71b` |
| [`diagnostics/p8/test_p8_h9_addendum.py`](../diagnostics/p8/test_p8_h9_addendum.py) | 200 | `cfe71dd2cf59d7a27bcbaf52b50af663ff07ab6591a3865e034cb78cd2e42489` |

**Outside the candidate:**
* The P8 candidate binds `simulation/px4_gazebo_p8/{configs,scripts,validation,replay,analysis,ros2_ws}` and
  `tests/test_px4_gazebo_p8_*.py`. `diagnostics/p8/` is in neither, so the candidate does not drift.
* A test asserts that neither file is a bound P8 file, and the tool refuses to run if it is.
* It imports the bound report code (`p8_report.py`, with its P7-E functions and the sequencer's constants)
  unchanged, and changes none of it.

**What it does,** stopping at the first failure:
1. **Checks** (refused, writing nothing, otherwise):
   * the manifest `fe2a0c6e…` verifies with no drift;
   * the report has exactly the SHA-256 `f03253d0…` and binds this manifest;
   * its status is "not passed" for exactly "H9 not met";
   * H1–H8, H10 and H11 are met;
   * in its H9 detail, the manifest, the results binding, the worlds, the classifier re-run, H10 and the unchanged
     run files all passed on every flight, and the assessment check failed on every flight;
   * the output does not exist.
2. **For each of the 60 flights,** loaded through the report's own `load` (each file accepted only by its hash):
   * the same fresh P7-E assessment that H9 computed;
   * **rule O**, the report's comparison as it is. It must give the report's own result on every flight;
   * **rule C**, the corrected comparison. It is **stricter** than the report's, not looser:
     * every key is equal except `contract` and `stop_reasons`;
     * `contract` must be exactly the sequencer's relabel (`p7e-flight-assessment-v1` to `p8-flight-assessment-v1`),
       not merely ignored;
     * `stop_reasons` must equal the P7-E stop reasons plus the FC-5 or FC-6 reason the sequencer derives from the
       H10 record. The report did not check this key at all;
     * the assessment's `h10` summary must equal the H10 record and its hash, and no other key may be present.
   * No run file may change (size and mtime, before and after).
3. **Status "explained"** only if, on all 60 flights:
   * rule O reproduces the report;
   * the only key rule O finds different is `contract`;
   * rule C holds;
   * no run file changed.

   Anything else is "not explained".
4. **The record is written once** (exclusive). It carries the tool's hash, the report's hash and status, both
   rules, the 60 rows and this statement: the addendum does not change the report, whose status stays as written,
   and whether P8's acceptance may rest on the report and the addendum is the operator's decision.

### 6.2 Tests (offline): 28 passed (1.9 s)

* **Rule O and rule C:**
  * the relabel alone fails rule O and passes rule C;
  * a difference in `validity`, `outcome`, `fc3` or a record hash fails rule C;
  * a wrong label, an extra stop reason, an H10 summary that differs from its record, and an unexpected key each
    fail rule C;
  * a failed H10 must add exactly the sequencer's FC-5 or FC-6 reason.
* **The summary:** "explained" only when every flight is, with all 60 present.
* **The report checks:** the not-passed report is accepted. 10 variants are refused: another manifest, another
  status, another reason, another criterion failing, H9 met, a failing classifier or unchanged check, an assessment
  row that passed, an unverified manifest, and a flight count that differs.
* **Preconditions:** a wrong report hash, an existing output, drift and a bound tool are each refused.
* **One real flight** (block 1, flight 0), read-only: rule O fails as in the report, on `contract` alone; rule C
  holds; no run file changed.

### 6.3 Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| The tool's preconditions: manifest `fe2a0c6e…`, no drift, tool not bound, report hash `f03253d0…`, no output | passed |
| The tool's report checks on the real report | passed: 60 assessment rows, all failed; every other H9 check passed |
| Flights loaded through the report's `load` | 60, evidence complete 60 |
| Addendum output and evidence copy | neither exists |
| Live locks | none |
| The 74 P8 offline tests | 74 passed (34 s) |
| Working tree (Windows git) | HEAD `358c3bb`; only the two new `diagnostics/p8/` files and `.codegraph/` untracked |

No per-flight comparison was run in the pre-check. The §5.2 check, made right after the report, was an informal
version of rule O's key listing.

### 6.4 What the authorization does

**1. The run** (offline, minutes; it fails before writing if the tool's hash differs from §6.1):
```
cd /tmp && test "$(sha256sum <repo>/diagnostics/p8/p8_h9_addendum.py | cut -d' ' -f1)" = 5ef9d5e9…ca71b && \
GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  <repo>/diagnostics/p8/p8_h9_addendum.py \
  --manifest      /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json \
  --report        /home/joker0625/uav_autonomy/state/p8-engineering-report-fe2a0c6e9052.json \
  --report-sha256 f03253d064be761079a319b024a3540a0eb831a849fc2c76c476125f5ea477ec \
  --output        /home/joker0625/uav_autonomy/state/p8-h9-addendum-fe2a0c6e9052.json
```
* It runs detached, with log `~/uav_autonomy/logs/p8-h9-addendum-fe2a0c6e9052.log`.
* Exit codes: 0 explained, 1 not explained, 2 refused (nothing written).

**2. After the run** (part of the same authorization):
* copy the record byte for byte to `docs/evidence/v3_sim_p8_h9_addendum.json`;
* write the result here;
* commit the tool, its tests, this note and the copy together, locally. Nothing is pushed.

**3. Whatever the status,** the report is not changed and not rerun.
* If "explained": P8's acceptance is then your separate decision, on the report and the addendum together.
* If "not explained": F-R1 is not only the label, and the result is reported as it is.

### 6.5 Result (run once, 2026-11-28)

**Records:**
* the addendum: `state/p8-h9-addendum-fe2a0c6e9052.json`, 26 292 bytes, SHA-256
  `f2903a17211a7b39f8ba98ffb7c406c48847f8eefc8795904251c33d2b0a0783`;
* its byte-identical copy: [`evidence/v3_sim_p8_h9_addendum.json`](evidence/v3_sim_p8_h9_addendum.json) (same hash,
  compared with `cmp`).

**Run:**
* the tool's hash was checked first and equalled §6.1 (`5ef9d5e9…`);
* detached (PID 1018162), 24 s (13:09:47 to 13:10:11 UTC); log `~/uav_autonomy/logs/p8-h9-addendum-fe2a0c6e9052.log`;
* exit 0.

**In the record:**
* contract `p8-h9-assessment-addendum-v1`, finding F-R1;
* the tool `diagnostics/p8/p8_h9_addendum.py` (`5ef9d5e9…`), not bound by the candidate;
* the manifest `fe2a0c6e…`;
* the report `f03253d0…`: status "engineering validation not passed", reason "H9 not met", report code `815b0f7c…`.

**Status: "explained".**

| Over the 60 flights | Count |
|---|---:|
| Rule O reproduces the report's own result (fails) | 60 of 60 |
| Rule O differs on `contract` alone | 60 of 60 |
| **Rule C, the stricter corrected comparison, holds** | **60 of 60** |
| Run files unchanged | 60 of 60 |
| Rule C problems found | none |

**After the run:**
* the manifest still verifies, with no drift, and no lock is held;
* the report is unchanged (`f03253d0…`).

**What this means:**
* The single failing H9 check of the report is explained by the sequencer's `contract` relabel alone.
* On every flight, the P8 assessment equals a fresh P7-E assessment on every other key. Its stop reasons and its H10
  summary are exactly what the sequencer derives.
* With the other H9 checks (each 60 of 60 in the report itself), every substantive part of H9 reproduces.
* **What it does not mean:**
  * the report is not changed and still reads "engineering validation not passed";
  * the addendum is not a second report, and it sets or relaxes no threshold;
  * whether P8's acceptance may rest on the report and this addendum is the operator's decision, not made here.
