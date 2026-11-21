# v3-SIM P7 M3 — flight classifier and frozen analysis code (offline)

**Status: ACCEPTED (2026-11-21); M3 COMPLETE.** This is M3 of the [P7 plan](v3_sim_p7_plan.md) (§7).
* **Authorized:** by the operator on 2026-11-21 ("Authorize M3").
* **Accepted:** by the operator on 2026-11-21 ("Accept M3"), with the classifier rules of §2 and the frozen
  analysis code `p7_analysis.py` `bd036a2b…` of §4.
* **Frozen:** from here, any change to the analysis code or the classifier is a new version and needs a new
  approval.

**What M3 did and did not do:**
* It ran offline only. Nothing flew, and no namespace or simulator started.
* No P2–P6 file changed, and the P3–P6 manifests verify.
* Retained evidence was only read. Classifier tests that write run on copies.

**Exit criterion (plan §7):**

| Criterion | Status |
|---|---|
| Tests on retained runs | **met:** R1–R4, and all 12 retained accepted P6 flights |
| Tests on synthetic data, including negative cases | **met:** every D14 class, the goal conditions, validity, both restated P6 rules, and the statistics against independent references |
| Analysis code frozen | **met:** `p7_analysis.py` `bd036a2b…`, §4 |

Altogether 113 P7 tests pass: 17 M1, 31 M2, 29 classifier and 36 analysis.

**One finding needs a decision: the M0 backup misses `venv-px4` (§5).**

## 1. Files

| File | Role | SHA-256 |
|---|---|---|
| `simulation/px4_gazebo_p7/validation/p7_classify.py` | per-flight classifier, writes `p7/flight-record.json` | `f093b4ec…` |
| `simulation/px4_gazebo_p7/scripts/verify_p7_evidence.sh` | runs it in the accepted evaluators' environment (`venv-px4`); the accepted `verify_p5_evidence.sh` with names changed | `99359ade…` |
| `simulation/px4_gazebo_p7/analysis/p7_analysis.py` | the analysis plan as code (**frozen**) | `bd036a2b3d483d78747b2e959be89ec850ddb3093fa5edb53a09bd03f5801edb` |
| `tests/test_px4_gazebo_p7_classify.py` | 29 tests | `1a0dd8a3…` |
| `tests/test_px4_gazebo_p7_analysis.py` | 36 tests | `f03012ba…` |

## 2. Classifier (`p7_classify.py`)

**How it runs.** It runs after the unchanged P4 or P5 evaluator, in that evaluator's own environment. The M2
sequencer's hook `classify(run)` calls `verify_p7_evidence.sh`. That runs the classifier under `venv-px4`, whose
`pyulog` reads the ULog through the accepted P2 reader. The record is written once (exclusive).

**Validity (analysis plan):**
* **Valid** = armed evidence: the ULog `vehicle_status` shows ARMED, or the P2 controller entered TAKEOFF.
* **Disagreement** between the two: the flight is valid, and the disagreement is an integrity event.
* **No armed evidence:** infrastructure-invalid. Only Policy A and a competing publisher can stop the batch on such
  a flight, because nothing moved.

**D14 classes, from the accepted evaluator's failure items:**

| Class | Items | Effect |
|---|---|---|
| Safety event | `safety:collision_contact`, `safety:truth_clearance`, `safety:setpoint_inside_inflated_obstacle`, `native_commands:*`, `p2:command_sequence`, `competing_flight_publisher`, `p2:final_landed_disarmed`, `unsafe:hold_tick_motion`, `unsafe:unrevalidated_resume` | failure; stop |
| Safety-unverified | `safety:truth_poses_missing`, `safety:truth_pose_interval`, `safety:truth_observer_error`, evaluator errors in the safety, P2 or Policy A parts, a missing contact ledger, no evaluation, no safety output | failure; stop |
| Mission failure | `safety:tracking`, `safety:response_latency` | failure; no stop |
| Integrity event | every other item | reported only |

* The P4 evaluator treats a missing contact ledger as "no contacts". The classifier checks for the file itself.
* **Goal success** needs all of:
  * `goal_reached_sim_s` recorded;
  * `p4_outcome` completed and final state COMPLETE;
  * landed and disarmed (the accepted evaluator's ULog check);
  * final goal distance ≤ `goal_tolerance_m` 0.30 m, measured as the accepted `outcome()` does;
  * no safety event, safety verified, and no mission failure.

**The two restated P6 §6.3 rules.** They use the same rule and threshold as the accepted P6 functions:
* **Hold-tick motion:** the next decision's target moves more than H35's 0.01 m (read from `p6_faults.yaml`) after a
  stale or P5 policy-failure tick.
* **Unrevalidated HOVER→AVOIDING return:** a return without a fresh (≤ 0.25 s), non-stale decision. A flight with no
  return is not a failure.

**Metrics.** Time to goal and path length come through `p5_comparison.flight_metrics()`. The record also carries
clearance, collisions, tracking, abort reason, hover time, RTF and P5 per-decision figures.

**Results:**
* **R1–R4:**
  * all valid, with the ULog and the transitions agreeing;
  * all goal successes, with no event of any class;
  * final goal distances (0.023, 0.067, 0.026, 0.029 m) and clearances equal to the accepted evaluations;
  * the P5 time to goal and path equal to the accepted `reported` block;
  * the P4 flights' time to goal (44.19, 47.58 s) is now computed the same way.
* **Full chain on a copy of R3:** the real ULog was read and the record written once. A second classification
  refuses.
* **Agreement with P6:** on all 12 retained accepted P6 flights, both restated rules return exactly what the
  accepted P6 functions return (`UNSAFE` moves, `RECOVERY` unrevalidated). At least one flight exercises a real
  HOVER→AVOIDING return.
* **Negative cases on copies:**
  * each of the 17 listed items lands in its class with the right stop and outcome;
  * removed safety evidence becomes safety-unverified;
  * an aborted flight, a final position 0.5 m off the goal, and not disarmed are each failures;
  * the validity cases behave as specified;
  * an injected hold-tick motion stops the batch.

**A consequence of D14 to note.** The accepted P2 controller, in RECOVERY, sends exactly one LAND when the vehicle is
still armed in Offboard, so the command sequence stays valid. If PX4 has already taken over (a failsafe), no LAND is
sent. `p2:command_sequence` then fails, and D14 stops the batch for the operator's review.

## 3. Analysis code (`p7_analysis.py`, frozen)

**What it implements.** `p7-analysis-plan-v1` exactly. It refuses to run if the plan's canonical hash differs from
the approved `c9c7a6f3…`.

**Methods:**
* **Primary:** exact two-sided McNemar.
* **Primary estimate:** the paired difference with a Tango score interval, using the closed-form restricted MLE
  derived in the code.
* **S1:** a fixed-seed paired bootstrap, the method of `p5_comparison.interval`, with the 2.5th percentile against
  −0.05 m.
* **S2/S3:** Wilcoxon signed-rank (exact or normal, with tie and continuity corrections), and Hodges–Lehmann with
  the Walsh-average interval.
* **Sequence:** S1, S2 and S3 are tested only in the fixed sequence.
* **Safety:** counts with exact bounds.
* **Pairs:** missing pairs are listed with the sensitivity analysis; unflown pairs are kept apart.
* **The pilot rule.**
* **Exploratory strata:** family, box count, first controller, block.

**Inputs.** It reads the block results. Every flight record must match the hash the block recorded, or the analysis
refuses.

**Verified against independent references (SciPy 1.11.4 in the tests only; the code needs only the standard
library and NumPy):**

| Method | Reference | Agreement |
|---|---|---|
| Exact McNemar | `scipy.stats.binomtest` and P5's `sign_test` | rel. 1e-12, 7 cases |
| Tango interval | brute-force score inversion, with the restricted MLE maximized numerically | 2e-4 (grid step), 6 cases incl. c = 0 and b = 0 |
| Wilcoxon, exact | `scipy.stats.wilcoxon(method="exact")` | rel. 1e-10, n = 6–25 |
| Wilcoxon, normal with ties and zeros | `scipy.stats.wilcoxon(method="approx", correction=True)` | rel. 1e-9 |
| Hodges–Lehmann interval | brute-force Walsh median; Hollander–Wolfe C = 9 at n = 10 | exact |
| Clopper–Pearson | `scipy.stats.beta.ppf` | 1e-9 |
| Bootstrap | `p5_comparison.interval` with the same seed | identical |

## 4. Freeze

**What is frozen.** The analysis code is `simulation/px4_gazebo_p7/analysis/p7_analysis.py`, SHA-256
`bd036a2b3d483d78747b2e959be89ec850ddb3093fa5edb53a09bd03f5801edb`.

**How it is bound.** The P7 manifest binds it with the classifier, the runner and the tests, because `analysis/` and
`validation/` are among its source folders. The real freeze (contract C7) must carry this hash.

**Changing it.** Any change after the operator accepts M3 is a new version and needs a new approval. That includes
the classifier, since it decides the outcome classes.

## 5. Finding: the M0 backup misses `venv-px4` (an error in my M0 inventory)

**What was missed.** The M0 inventory classified `~/uav_autonomy/venv-px4` (0.47 GB, 14,803 files) as Optional, "not
required by P7-A". That is wrong.
* The unchanged `verify_p4_evidence.sh` and `verify_p5_evidence.sh` run under `$SIM_ROOT/venv-px4/bin/python`.
  Every P7 flight is evaluated with them.
* The P7 classifier needs the venv's `pyulog`.

So it is an Essential runtime dependency. It is **not** in the C: backup.

**Why it matters.** The venv was built by `pip` from PX4's `requirements.txt` without pinned versions. It now holds
NumPy 2.5.3 and pyulog 1.2.4. A rebuild could therefore resolve different versions, so its reproducibility is not
verified ("do not assume a runtime build can be discarded").

**Done (2026-11-21, operator "Authorize venv backup"):** Part E was executed and verified (`E10-venv-px4.tar`
`bb247931…`, 16,387 entries, 0 problems, receipt `39bdd616…`). See inventory §9.

**The proposal as made:**
* A supplementary backup, Part E: `venv-px4` as one more uncompressed tar.
* It goes into the same destination, `C:\GWM-UAV-Backup\p7-selective-20261121\`, under a new sub-receipt.
* It uses the same procedure: drift check against the inventory's `O-venv-px4` tree digest `0c798cc9…`, read-back
  hash, and a restore check.
* About 0.47 GB; C: has 125.8 GB free.
* The inventory report gets an erratum that reclassifies `venv-px4` as Essential.

## 6. Next

* **Operator review** of M3: done; accepted on 2026-11-21 ("Accept M3").
* **Operator decision** on the supplementary `venv-px4` backup (§5).
* **M4** (the parity dry run: a development freeze and S2 flown by P4 and P5 through the P7 path) needs its own
  authorization. It is the first step that launches a simulator.
