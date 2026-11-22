# v3-SIM P7-E E2 — the P7-E code, offline

**Status: WRITTEN AND TESTED, FOR OPERATOR ACCEPTANCE (2026-11-22).**
* **Authorized:** by the operator ("Authorize E2"), under the approved contract
  [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) E-C1, E-C2, §4–§8 and §13 item 4.
* **What it did:** the P7-E code in `simulation/px4_gazebo_p7e/` and three new test files. It covers the manifest,
  the runner and launcher, the fail-closed rules FC-1 to FC-3, and the engineering report code.
* **What it did not do:**
  * no frozen P7 file, candidate or evidence changed, and no P2–P6 bound file changed;
  * no schedule-selection record;
  * no candidate freeze, no Policy A approval, no SITL launch, no deletion, no push.
* **Tests:** the 76 P7-E tests pass, with no skips. Run together with the frozen P7 suites, 198 pass. The frozen P7
  candidate `0799c37e…` still verifies, P3–P6 inside it included.

## 1. Files (for acceptance)

| File | Role | Lines | SHA-256 |
|---|---|---|---|
| [`p7e_manifest.py`](../simulation/px4_gazebo_p7e/scripts/p7e_manifest.py) | candidate identity, P7 binding, authorizations, live locks | 312 | `c0512954…` |
| [`p7e_runner.py`](../simulation/px4_gazebo_p7e/scripts/p7e_runner.py) | one P7-E flight; imports the frozen P7 runner | 252 | `6370013a…` |
| [`run_p7e.sh`](../simulation/px4_gazebo_p7e/scripts/run_p7e.sh) | launcher; the frozen `run_p7.sh` renamed only | 37 | `d4ae1e57…` |
| [`p7e_namespace_owner.py`](../simulation/px4_gazebo_p7e/scripts/p7e_namespace_owner.py) | PID-1 owner; byte copy of the P3 owner the QGC build pins | — | `0d1c2d27…` |
| [`p3_runner.py`](../simulation/px4_gazebo_p7e/scripts/p3_runner.py) | owner entry shim → `p7e_runner.main` | 8 | `3764539c…` |
| [`p7e_host.py`](../simulation/px4_gazebo_p7e/scripts/p7e_host.py) | FC-2 around the frozen host check | 73 | `1cc0434f…` |
| [`run_p7e_block.py`](../simulation/px4_gazebo_p7e/scripts/run_p7e_block.py) | block sequencer with FC-1 | 346 | `b04297a7…` |
| [`p7e_assess.py`](../simulation/px4_gazebo_p7e/validation/p7e_assess.py) | FC-3 after the frozen classifier | 122 | `7fe6070d…` |
| [`p7e_report.py`](../simulation/px4_gazebo_p7e/analysis/p7e_report.py) | the engineering report (plan `15e7f7b4…`) | 621 | `8734bbe9…` |
| [`test_px4_gazebo_p7e_runner.py`](../tests/test_px4_gazebo_p7e_runner.py) | manifest, authorizations, runner, C6 | 382 | `729903d5…` |
| [`test_px4_gazebo_p7e_failclosed.py`](../tests/test_px4_gazebo_p7e_failclosed.py) | FC-1, FC-2, FC-3 | 370 | `a16c146f…` |
| [`test_px4_gazebo_p7e_report.py`](../tests/test_px4_gazebo_p7e_report.py) | the report, on a synthetic 60-flight batch | 364 | `70bfa9b2…` |

* The approved E1 files are unchanged: `p7e_schedule.json` (canonical `9b3deacb…`), `p7e_report_plan.yaml`
  (canonical `15e7f7b4…`), `p7e_specs.py` and `test_px4_gazebo_p7e_specs.py`.
* The P7-E candidate will bind **16 files**: the 12 in the P7-E tree and the 4 P7-E test files. It also binds the
  four spec hashes, the schedule, the report plan, and the reused P7 layouts `d742b7af…` and host check `9f78af75…`.
* **Not bound:** a read-only diagnostic,
  [`p7e_e2_pilot_check.py`](../diagnostics/p7e/p7e_e2_pilot_check.py). Its output is
  [`v3_sim_p7e_e2_pilot_check.json`](evidence/v3_sim_p7e_e2_pilot_check.json) (`661cddc7…`); see §6.

## 2. Reuse of the frozen P7 code (E-C1)

* **Imported unchanged, by path.** The frozen P7 runner, `p7_runner.py`, brings with it the accepted P5, P4 and P3
  runners. These P7 parts are also reused unchanged:
  * the world check C6 (`prepare_world`) and the D15 flight window (`run_controller`), both inherited;
  * `controller_summary`, `p7_init` and the scenario mapping;
  * from the P7 sequencer: `BlockRules`, `run_sequence` and `Ledger`;
  * the classifier `p7_classify.py` with its wrapper and environment;
  * the host check `p7_host.check`;
  * the interval functions in `p7_analysis.py`.
* **P7-E overrides only three runner methods:** `preconditions`, `prepare_run` and `execute`. A test checks each
  method's owner, and that the P7-E mixin defines nothing else.
* **The binding (E-C2).** The P7-E manifest runs the accepted `p7_manifest.verify_current`, which must return
  `0799c37e…`. That check recomputes P3–P6. `source_drift` also covers the P7 files P7-E imports.
* **Policy A.** The approval must name a P7-E candidate (`candidate_phase` starts with "P7-E"), so the P7 approval
  does not transfer.
* **Locks.** The live lock refuses to start while either a P7 or a P7-E sequence holds its lock.
* **Run evidence:**
  * Runs are named `…-p7e-<controller>-<layout>-…`, and dry runs `…-p7edry-…`. Only `-p7edry-` runs count toward a
    P7-E dry-run cap.
  * `p7/layout.json` stays where the frozen classifier hashes it.
  * The summary keeps the `p7` keys the classifier reads, and adds a `p7e` block. The gate is `P7-E`, and
    `acceptance_credit_possible` is false.
  * The accepted evaluators only record the gate, so they are unaffected.

## 3. The fail-closed rules (contract §8)

**FC-1, sequencer exceptions and accounting (`run_p7e_block.py`).**
* **The gap.** In the frozen sequencer, an exception left `stopped: false`. A test reproduces it: an unstopped
  result with no flights is refused as a previous block.
* **The rule.** Any exception during the sequence is a stop, interrupts included. The stop reason is
  `sequencer_exception:<type>`.
  * The flown flights are rebuilt from the fsynced ledger.
  * The flight in progress is named `interrupted`.
  * Every other planned flight, and any queued replacement, is listed in `not_flown`.
* **Accounting.** Every result carries a status for every planned flight:
  * `flown_valid`, `missing`, `interrupted`, `flown_stopped`, `not_flown`, `flown_before_stop`;
  * `flown_unresolved` or `unaccounted` makes the result incomplete.
* **The next block** needs the previous result to be:
  * for this manifest and the approved plan;
  * unstopped, with nothing unflown or interrupted;
  * complete on accounting, recomputed rather than trusted;
  * backed by an intact ledger.
* **Refusals before anything is consumed:** an existing ledger, or a held P7 or P7-E live lock.
* **Hardened `fly_one`.** Once the runtime has returned, `fly_one` never raises. A failure in evaluation or
  classification is recorded, and the block stops as `unclassified_flight`.

**FC-2, a failed `pgrep` (`p7e_host.py`).**
* **The gap.** The frozen check returns "no process" for a `pgrep` that exits 2; a test shows it.
* **The rule.** An exit status other than 0 or 1, or a missing `pgrep`, raises `ProcessCheckFailed`.
  * **Before a block:** the host check fails, nothing is flown, and the authorization is not consumed.
  * **After a flight:** the batch stops with `process_check_failed:…`, and the flight is already in the ledger.
* **Coverage.** The frozen process pattern matches the P7-E owner (`_namespace_owner\.py`). The P7-E runner runs
  inside that process, not as a separate one.

**FC-3, an unknown arm state (`p7e_assess.py`).**
* **Order.** The frozen classifier runs first, unchanged, and writes `p7/flight-record.json`. The assessment is then
  written once to `p7e/flight-assessment.json`, binding the record and the summary by hash.
* **The rule.** An invalid flight is re-flown only on positive evidence that it never armed:
  * `ulog_never_armed`: the ULog was read and never shows ARMED;
  * `controller_never_started`: the finalized summary lists no `ros-controller`;
  * `no_request_arm`: the controller's own result shows no REQUEST_ARM, and the finalized summary hashes that result.
* **Otherwise** the validity is `unknown`, and the stop is `safety_unverified:fc3_arm_state_unknown`. Two cases are
  unknown on purpose: a stale (unfinalized) summary, and a controller result the summary does not hash.
* **Test on the frozen record.** A test runs the frozen `build_record` on a flight with no readable ULog that reached
  REQUEST_ARM. The frozen rule says re-fly it; FC-3 stops instead.

## 4. The engineering report (`p7e_report.py`)

**What it reads.** It loads the 3 block results and accepts each flight's summary, evaluation, record and
assessment only if its hash matches the ledger. Limits come from the approved plan and are checked against their
source files. It refuses a changed plan. Its output is written once.

**The hard criteria:**
* **Safety (H1–H6):**
  * **H1** collisions = 0.
  * **H2** minimum clearance ≥ 0.20 m on every valid flight.
  * **H3** unsafe commands = 0.
  * **H4** Policy A breaches = 0.
  * **H5** every valid flight landed and disarmed with own commands 176/400/21, through the accepted P2 integrity
    check or, under D15, the ULog facts.
  * **H6** safety-unverified flights = 0, FC-3 included.
* **Evidence and execution:**
  * **H7** complete, hash-matched evidence for every flown flight, and every ledger intact, line for line.
  * **H8** each block's plan, recomputed accounting, replacement cap (2 of 20) and one replacement per flight. An
    unflown flight or block is explained only by a recorded stop.
* **Reproducibility (H9):**
  * the manifest re-verifies (P7 and P3–P6 inside it), the specs match, and every result binds the manifest;
  * every world equals its scheduled hash;
  * the frozen classifier, re-run through its own wrapper on a symlink view of every flight, reproduces the record
    exactly, and the FC-3 assessment reproduces too;
  * no file of the run changes: every file's size and mtime is checked.

**Reported criteria R1–R10** follow the plan:
* **R1:** Clopper–Pearson per controller.
* **R2:** paired counts, with the Tango interval for P5 − P4.
* **R3–R6:** the measured values per controller against the accepted limits.
* **R7:** Hodges–Lehmann for time to goal and path length.
* **R8:** host and cost, from each run's resource ledger and native clock.
* **R9:** abort types. **R10:** integrity events.
* **Also reported:** per-controller zero-event safety bounds, and the paired clearance (descriptive).
* **What the report contains:** no p-value (`p7_analysis.primary` is never called, because it computes one), the
  required statement verbatim, the not-claimed list, and the 30-pair limits.

**Interpretations for your acceptance.** Where the plan left a detail open, I chose the fail-closed reading. None of
these adds a threshold:
1. **Passed needs a completed batch.** "Engineering validation passed" also needs the declared 60 flights completed:
   3 blocks, none stopped, every flight accounted for. Contract §7.1 defines a pass as having "completed the declared
   60-flight engineering validation". H8 alone can be met by a batch that stopped with every gap explained.
2. **H2:** a valid flight with no measured clearance fails H2.
3. **H4:** a flight whose Policy A check did not run fails H4.
4. **H6:** a flight whose record or assessment is missing, or does not match its hash, counts as safety-unverified.
5. **H5:** a failsafe counts against H5 only on the D15 path, as the plan's measure states. On the accepted P2 path
   it stays the evaluator's integrity item.
6. **R breaches** are the accepted evaluator's own failure items for that criterion. The report does not compute
   new limits. Each breach becomes a proposed new risk-register entry, not an engineering failure.
7. **Which flights.** R1, R2 and R7 use each slot's final valid attempt. R3–R6 and R8–R10 use every flown flight.
8. **Safety events outside H1–H5.** A safety event the H1–H5 mapping does not cover makes the report "not passed".

## 5. Tests (offline; 76 P7-E tests, 0 skipped)

| File | Tests | Covers |
|---|---|---|
| `test_px4_gazebo_p7e_specs.py` (E1, unchanged) | 8 | schedule, report plan, non-contamination |
| `test_px4_gazebo_p7e_runner.py` | 20 | 16 bound files; spec hashes; Policy A non-transfer; block and dry-run authorizations; live locks vs P7; drift; **the P7 binding verifies `0799c37e…` and its tree is untouched**; owner, shim, launcher; overrides; slots and parse; gating; full preconditions (marker, slot, block, dev candidate, dry-run cap counting only `-p7edry-`); run-local inputs; summary keys; **C6 for all 30 worlds** |
| `test_px4_gazebo_p7e_failclosed.py` | 22 | FC-2 (frozen gap shown; pre-block and after-flight); FC-1 (exception, interrupt, failing drift check, queued replacement, accounting, previous-block rule, `fly_one`, `main` end to end with stub flights, refusals before consumption); FC-3 (7 evidence cases, pass-through, the frozen classifier's own record, write-once) |
| `test_px4_gazebo_p7e_report.py` | 26 | a clean 60-flight batch passes; frozen interval functions, P5 − P4; one negative case per H criterion (10 cases for H1–H6, plus H7, H8 and H9 cases); a stopped batch is never a pass; R breach → risk register; refusal of a changed plan; write-once; H9 view with the frozen wrapper; cost readers |

**How they were run:** in WSL, from `/tmp`, with `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1`:
`PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider …`.
* The P7-E files: 76 passed.
* With the four frozen P7 suites: 198 passed.

## 6. Read-only checks on real evidence

**The pilot check.** It runs the report code and FC-3 on the 40 retained pilot flights of `0799c37e…` (the plan's E2
test matrix). Output: `v3_sim_p7e_e2_pilot_check.json`.
* **H9 view.** All 40 records re-classify identically through the frozen wrapper. That took 169 s in total, about
  4 s per flight, and no run file changed.
* **FC-3** changed no validity. All 40 pilot flights are valid.
* **H1–H6** are met on the pilot:
  * collisions 0, minimum clearance 0.315 m, unsafe commands 0, Policy A breaches 0, safety-unverified 0;
  * H5 holds for all 40 flights. The pilot's one observation-gap flight passes through D15, with the ULog facts
    176/400/21, landed and disarmed.
* **The host check** (`p7e_host.py`, read-only) passes on this host today: overlay, AC, CPU, no simulator, lock free,
  disk floor.

## 7. What comes next (each needs its own explicit authorization, contract §13)

1. **E2 acceptance** (this report).
2. **The retention-policy approval** (D4-A/D4-B). It is separate from any deletion.
3. **The development dry run** of the new launch path: 2 flights on S2, one per controller. It needs three things:
   * a development-only Policy A approval that names P7-E;
   * a development freeze of the P7-E candidate;
   * a dry-run approval.

   The E2 files must be committed before any freeze, because the freeze records the git HEAD.
4. **Then, in order:** the production Policy A approval, the real freeze, blocks 1–3, and E5.
