# v3-SIM P7 M0 — read-only audit

**Status: complete (2026-11-21).** This is the M0 audit of the [P7 plan](v3_sim_p7_plan.md) (§7). The operator's
instruction "現在繼續p7" (continue P7) was taken as authorization for this read-only audit only.

**What it did and did not do:**
* Nothing was launched, frozen or authorized. No simulator ran.
* No P2–P6 file changed, and no evidence was written.
* The checks ran offline. They imported the bound P4 world generator and the bound P5 layout generator unchanged,
  and read retained evidence.
* The four P3–P6 manifests verify.

**Two new decisions came out of it, D13 and D14 (§8).** The operator approved both as recommended on 2026-11-21
("Approve as recommended"). They are encoded in contract §5 and §7.3. **M0 is complete.** M1 needs its own
authorization.

**Reproduction:** [`diagnostics/p7_m0/p7_m0_audit_checks.py`](../diagnostics/p7_m0/p7_m0_audit_checks.py), run
once under WSL `/usr/bin/python3` (Python 3.12.3, NumPy 1.26.4), wrote
[`v3_sim_p7_m0_audit_checks.json`](evidence/v3_sim_p7_m0_audit_checks.json) (SHA-256
`bee98e5d923cd245cb336b1fe666d4f00e87f6aeecc5fc7ad88d8f2a569e21da`). Every number below comes from that file or
from the code lines cited.

The dependency inventory and the Selective Dependency Backup (revised D5) are the other M0 items. They are
recorded in their own [report](v3_sim_p7_m0_dependency_inventory.md).

## 1. Findings at a glance

| # | Item | Result | Consequence |
|---|---|---|---|
| A1 | Runner override point | `self.scenarios` plus `prepare_world()` works as planned. The runners' `preconditions()` also require P4/P5 gate authorizations, markers and gate names. | P7 overrides `preconditions()`, `prepare_run()` and `execute()`, as P6 did (§2). |
| A2 | C6 world identity | S1–S6 regenerated offline equal the world SHA-256 of all 14 accepted P4 and P5 flights. | C6 can be tested offline in M2. |
| B1 | Accepted evaluators on a P7 layout | A declaration with `outcome: null` crashes `p4_evaluate.outcome()` with an uncaught `AttributeError`. The whole evaluation then becomes "evaluator_incomplete_evidence" and every safety result is lost. | **Decision D13.** |
| B2 | Evaluator `status` | It is not a mission outcome. A supervisor ABORT can pass, and a goal flight can fail on a performance item. | **Decision D14**: which evaluator items decide what. |
| C1 | Layout generator | The bound generator reproduces all three P5 splits exactly. The held-out layouts are byte-equal to the readiness report. | The P5 split-match exclusion is computable (1,650 layouts). |
| C2 | Generated boxes | They have no `name` field. The P3 runner and P4 world generator require one. | M1 adds a naming rule. |
| C3 | P5 split-match exclusion | It removes 15 % of admissible confirmatory layouts, mostly wall-with-gap (48 % of that family) and one-box layouts. It removes no multi-box layouts. | D8 stands. The population statement and the power figure are corrected (§4.3, §4.4). |
| C4 | Power | 0.83 at 200 pairs on the excluded population, 0.91 at 250. Planned: 0.87. | Within D3. The pilot rule may raise N to 250. |
| D1 | Endpoint records | `goal_reached_sim_s`, `final_state`, `p4_outcome` and the arming transitions are present. Final goal distance and clearance are computable. | Primary and secondaries are measurable from records. |
| D2 | Unsafe command (P6 §6.3) | Three of its five parts are already accepted-evaluator checks. The other two exist only as P6 functions over P6's run object. | P7 restates those two rules (M3). |
| E1 | Budget | 2.7–3.0 min of runner time and 1.20–1.27 GB per flight. WSL 840.6 GB free, D: 880.6 GB free. | 440 flights fit, leaving about 290 GB inside WSL. |
| E2 | Host check | Overlay `ded574b5` and AC power hold now. In P6 the check was manual. WSL can read both itself. | M2 adds a read-only host-check tool. |

## 2. Runner override point and world identity (A)

**What the plan assumed (plan §2.1).**
* `P4Runner.__init__` reads `self.scenarios` once.
* `prepare_world()` passes it to `p4_worlds.generate()`, which writes `p4/scenario.json` and the world.

Both hold ([`p4_runner.py:57, 248–268`](../simulation/px4_gazebo_p4/scripts/p4_runner.py)).

**What the plan did not list.** Overriding the scenarios alone is not enough.
* **`preconditions()`.** It requires several things a P7 flight would not have:
  * a P4 (or P5) gate authorization for that phase's manifest;
  * the consumption marker written by that phase's gate sequencer;
  * a gate in `FLIGHT_GATES` (lines 87–93 in P4, 103–109 in P5).
* **The marker file.** It is passed to the bound P4 perception and control nodes and the P5 control node. Those
  nodes check only that the file exists (`require_enabled(..., marker.is_file())`), so a P7-owned marker works
  without touching them.
* **`execute()`.** It writes the summary contract `p4-runtime-summary-v1` / `p5-runtime-summary-v1` and
  `acceptance_credit_possible: true`. It also builds the baselines from that phase's own `frozen` mapping.

**The precedent is P6** ([`p6_runner.py:161–261, 524`](../simulation/px4_gazebo_p6/scripts/p6_runner.py)). It
overrides `preconditions()`, `prepare_run()` and `execute()`, and supplies the accepted code's `frozen` mapping
through `p6_manifest.runner_view()`. Each phase also keeps two launch files:
* a byte-identical copy of the P3 namespace owner (`0d1c2d27…`, the hash the unattended QGC build pins);
* a `p3_runner.py` shim that dispatches to that phase's runner.

**For P7.**
* One owner copy and one shim, dispatching to a P7 runner that chooses the P4 or P5 subclass per flight.
* `runner_view()` must give each side everything the accepted runner and evaluators read from `frozen`, at least:
  * **P4 side:** `p3_baseline`, `runtime` (including the pinned world asset and the native command policy);
  * **P5 side:** `model`, `schema_sha256`, `controller_runtime`, `baselines`, `runtime`.
* P5 `p3_baseline` and `p4_baseline` must equal `p5_validation.yaml` (`p5_evaluate.py:266–267`), and P4
  `p3_baseline` must equal `p4_validation.yaml` (`p4_evaluate.py:436`).

**C6, world identity.** S1–S6 were regenerated from the pinned default world with the bound `p4_worlds.generate()`
and the frozen `p4_scenarios.json`. Each world's SHA-256 equals `generated_world_sha256` in the summary of every
accepted flight of that scenario:
* 14 flights in all, from the P4 and P5 gates;
* S2 and S6 share a world, because they differ only by the S6 fault.

So the C6 check needs no simulator and can be an offline M2 test.

## 3. The accepted evaluators on a generated layout (B)

**The crash (B1).**
* `p4_evaluate.outcome()` takes the expectation from the declaration and calls `expected.startswith(...)`
  ([`p4_evaluate.py:239–245`](../simulation/px4_gazebo_p4/validation/p4_evaluate.py)).
* A generated layout has no expected outcome. If its declaration carries `outcome: null`, that call raises
  `AttributeError`.
* The surrounding `try` catches only `ValueError`, `KeyError`, `FileNotFoundError` and `StopIteration`
  (line 497).
* So the error reaches `main()`, which records only `evaluator_incomplete_evidence`. The safety, P2 and Policy A
  results of that flight are lost.
* `p5_evaluate` calls the same function (line 331) and behaves the same.

**Any other non-null label is safe.** It runs no outcome check and adds no failure: `outcome()` checks only
`goal_reached*` and `safe_refusal`. Only that function reads the declaration's `outcome` field. This gives the two
options of D13.

**Evaluator status (B2).** The accepted evaluators' `status` is an acceptance verdict, not a mission outcome:
* A supervisor ABORT can pass every item when no expectation is set.
* A flight that reaches the goal fails if any perception-performance, P2-timing, depth-stream or policy-ledger
  limit is missed.
* The goal-success definition (contract §5) relies on "the accepted evaluator's safety items". But the evaluator
  has more `safety:` items than the three that definition lists:
  * tracking ≤ 0.30 m;
  * response latency ≤ 0.25 s;
  * the truth-pose interval;
  * the truth-observer error;
  * missing truth poses.

D14 classifies every item.

## 4. Layouts (C)

### 4.1 Reproducing the P5 splits

The bound `p5_layouts.split()` reproduces the dataset manifest's train (1,200 layouts, 1,609 seeds tried),
validation (150, 204) and held-out (300, 411) splits, including the rejection counts. The 300 held-out layouts are
byte-equal to the readiness report.
* This runs under WSL `/usr/bin/python3` with NumPy 1.26.4, the version the P5 training run recorded.
* The 1,650 layouts are the exclusion set for the P5 split-match rule. The train and validation layouts exist only
  through this regeneration, because the manifest records only their counts.

**For P7 (M1):**
* `split()` is fixed to the three P5 splits, so P7 needs its own seed loop over the unchanged `generate()` and
  `rejection()`.
* `generate()` returns boxes with only `center` and `size`. The P3 runner (`p3_runner.py:507`) and
  `p4_worlds._box()` require a `name`, so P7 needs a naming rule. Nothing in the P4/P5 stack treats names
  specially.
* The layout list must be produced and hash-bound under the same interpreter and NumPy version.

### 4.2 Fresh seed ranges

| Range | Seeds screened | Admitted | Start/goal disc | Unreachable | Acceptance match | P5-split match |
|---|---:|---:|---:|---:|---:|---:|
| pilot (5,000,000) | 100 | 55 | 29 | 5 | 0 | 11 |
| confirmatory (4,000,000) | 400 | 231 | 84 | 25 | 1 | 59 |

200 confirmatory pairs need about 350 seeds and 250 about 435; the pilot needs about 20. The admitted
confirmatory family mix is:

| Family | Admitted | Share | Generator weight |
|---|---:|---:|---:|
| pillars | 87 | 38 % | 0.40 |
| mixed | 58 | 25 % | 0.25 |
| partial wall | 49 | 21 % | 0.15 |
| wall with gap | 37 | 16 % | 0.20 |

### 4.3 What the P5 split-match exclusion removes

On the first 400 confirmatory seeds:

| Family / boxes | Admissible | Excluded |
|---|---:|---:|
| wall with gap / 2 | 71 | 34 (48 %) |
| partial wall / 1 | 69 | 20 (29 %) |
| pillars / 1 | 24 | 5 (21 %) |
| every multi-box pillar and mixed layout | 126 | 0 |

**Why:** the match rule (same box count, centres within 0.25 m, sizes within 0.10 m) is loose where a family has
few degrees of freedom, and wall-with-gap layouts are close to one another.

**Is the exclusion neutral for the comparison?** P5's own held-out set answers this, because it had no such
exclusion:
* 57 of its 300 layouts (19 %) match a train or validation layout.
* Those 57 held 4 discordant pairs (4 P5-only, 0 P4-only).
* The 243 unmatched layouts held 27 (22 P5-only, 5 P4-only).
* Most excluded layouts are ones where both rankers succeed: 38 of the 42 matched wall-with-gap layouts.

So the exclusion does not remove the effect, and it does what D8 intended: test on layouts P5 has not effectively
seen. **D8 stands as approved.**

**What must be stated in every P7 report:**
* **The confirmatory population:** generator layouts that are not near-duplicates of a P5 layout, with
  wall-with-gap under-represented (16 % admitted, against the generator's 20 % before admission).
* **The family mix**, as in §4.2.

### 4.4 Power on that population

Exact two-sided McNemar, α = 0.05, with the discordance and P5-only share measured on P5's held-out set:

| Population | Discordant | P5-only share | 150 pairs | 200 pairs | 250 pairs |
|---|---:|---:|---:|---:|---:|
| all held-out (the plan's planning figure) | 10.3 % | 0.84 | 0.73 | 0.87 | 0.94 |
| unmatched held-out (what P7 samples) | 11.1 % | 0.81 | 0.69 | **0.83** | 0.91 |

The plan's 0.87 is restated as **0.83 at 200 pairs**. N stays 200 (D3). The pilot rule (contract §4.3) may raise
it to the approved ceiling of 250 if the observed discordance is lower than planned.

## 5. Endpoints, validity and unsafe commands (D)

**Endpoint records.** The four required reference flights (R1–R4: P4 and P5, S2, Gates B and C) carry everything
the endpoints need:
* `controller_result.p4.goal_reached_sim_s` (117.3–122.2 s);
* `final_state` `COMPLETE`, `p4_outcome` `completed`;
* the full P2 transition list, including `REQUEST_ARM` and `VERIFY_ARMED`;
* in the evaluation, `final_goal_distance_m` (0.023–0.067 m) and `min_truth_clearance_m` (0.374–0.446 m).

`p5_evaluate.reported()` (time to goal, path length) runs in the P5 evaluation only. `p5_comparison.flight_metrics()`
calls it on P4 runs too, as P5's SITL comparison did. Time to goal is measured from the `TRANSIT` transition:
42.75 and 42.92 s on the two P5 flights.

**Validity boundary (contract §7.1).** "The vehicle armed" is decidable from records:
* the ULog `vehicle_status.arming_state`;
* the P2 transition into `VERIFY_ARMED` and beyond.

M3 fixes the rule. The recommendation is to use the ULog, with the transition list as a cross-check. A flight
with neither record is infrastructure-invalid.

**Unsafe command (P6 contract §6.3).** Its five parts map as follows:

| Part | Status for P7 |
|---|---|
| Setpoint inside an obstacle inflated by R_body + 0.10 m | accepted check `safety:setpoint_inside_inflated_obstacle` |
| `vehicle_command` outside Policy A, or more than one LAND | accepted `native_commands:*` and `p2:command_sequence` (exactly 176, 400, 21) |
| A setpoint not from the owner | accepted `competing_flight_publisher` |
| Target motion > 0.01 m or > 0.5° on a tick that must hold (stale, or P5 policy failure) | P6 `unsafe_setpoints()`, over P6's run object |
| Motion resumed without revalidation (§6.5) | P6 `recovery()`, which also fails a run with no HOVER return |

P7 restates the last two rules in its own classifier (M3), with "no HOVER return" not counted as a failure, and
tests them on retained P6 runs.

## 6. Time, disk and host (E)

**Time.** The four reference flights took 163–177 s of runner time each, including start-up and teardown.
* Between flights come the accepted evaluation, including the P5 policy replay, and the P7 classification. The
  plan's 3.1 min per flight came from consecutive gate runs.
* The pilot measures the full cycle.
* 440 flights (pilot and 200 pairs) are about 22–24 h in 11 authorizations.

**Disk:**
* 1.20–1.27 GB per flight; 440 flights is about 550 GB.
* **WSL:** 840.6 GB free of a 1,081 GB virtual disk, so about 290 GB would stay free, above the 150 GB floor
  (contract §7.4).
* **D: (holds `ext4.vhdx`, now 204 GB):** 880.6 GB free, so the image can grow by that much. The image does not
  shrink on its own.
* `runs/` now holds 329 runs (101.3 GB).

**Host.**
* **Read-only check now:** AC overlay `ded574b5…` (Best performance), on AC power, 24 vCPUs and 15 GB RAM in WSL.
* **In P6** this check was done by hand before each launch.
* **For P7:** WSL interop is enabled, and `reg.exe query …\PowerSchemes /v ActiveOverlayAcPowerScheme` works from
  inside WSL. So the block runner can run the D10 host check itself, read-only (M2).

## 7. Authorization and launch machinery (F)

What a P7 freeze and block reuse or add:
* **Manifest and approval.**
  * `p3_manifest.runtime_identity(..., policy_name="A", policy_approval=...)` binds a Policy A approval into each
    candidate. A P7 candidate needs its own approval, as D12 says; an earlier phase's approval never transfers
    (`p6_manifest.check_policy_approval`).
* **Freeze and verification.**
  * The P7 manifest verifies P3–P6 through `p6_manifest.verify_current()`, which includes P3, P4 and P5.
  * Every verify touches 39 `.git/modules` directory times in the MC-trace tree ([backup report
    §8](v3_sim_p7_m0_dependency_inventory.md)). That is harmless, and P7 verifies before every flight.
* **Lock and markers.**
  * The live-sequence lock and the consumption markers follow `p6_manifest` (`acquire_live_lock`, `consume`).
  * A block is one authorization with `max_attempts` 1, written to a permanent marker before the first flight.
* **Evaluation order.**
  * As in P6: the unchanged evaluator first (`verify_p4_evidence.sh` or `verify_p5_evidence.sh`), then the P7
    classifier.
  * P6's gate sequencer (`run_p6_gates.py`) is the template for running one item, evaluating it, and only then
    starting the next.

## 8. New decisions for the operator

**Both approved as recommended (operator, 2026-11-21, "Approve as recommended").**

### D13: how P7 runs the accepted evaluators on generated layouts

| Option | What it means |
|---|---|
| **A (recommended)** | The P7 layout declaration carries `outcome: "p7_not_predeclared"`, a non-null label outside the accepted set. The accepted P4/P5 evaluators then run end to end, unchanged, through their own `verify_p*_evidence.sh`, and record every safety, P2, Policy A and stream item. The P7 classifier decides the mission outcome from the raw records. |
| B | The declaration carries `outcome: null`, and P7 calls the accepted evaluator functions one by one, as P6 composed P5. This means more P7 code, which must mirror the accepted `evaluate()` order and its exception handling. |

**Why A:** the accepted evaluators stay the single implementation of every check, and option A needs no P7 copy
of their sequence. The label is P7's own field in P7's own declaration; no accepted file changes.

### D14: which evaluator results decide what

| Class | Items | Effect |
|---|---|---|
| **Safety event** | collision contact; truth clearance < 0.20 m; a setpoint inside an inflated obstacle; Policy A breach (`native_commands:*`); a command sequence other than 176, 400, 21, or more than one LAND; a competing publisher; not landed and disarmed at the end; the two P6 §6.3 rules restated by P7 (§5) | the flight is a failure, and **the batch stops** (contract §7.3) |
| **Safety-unverified** | truth poses missing, truth-pose interval > 0.05 s, truth-observer error, contact ledger missing | the flight counts as a failure and **the batch stops** for the operator. Safety cannot be shown, so it cannot be told apart from a safety event. |
| **Mission failure, no stop** (recommended) | tracking > 0.30 m; response latency > 0.25 s (the accepted P4 safety-envelope limits) | the flight is not a goal success; reported per flight |
| **Integrity event** | every other item: depth stream, perception performance, RGB absence, P2 timing, ULog dropout, MC causality, renderer, cleanup and shutdown, P5 policy-ledger limits | reported; the outcome does not change |

**Goal success** then means:
* the contract §5 record conditions;
* no safety event;
* safety verified;
* no tracking or response-latency failure.

**Alternative:** treat tracking and response latency as integrity events, so they do not affect success.

**Why the recommendation:** both are limits the accepted P4 contract set to keep the clearance argument valid.
Counting them against success is conservative and identical for both controllers.

## 9. What follows for M1–M3

* **M1 (specs), after D13:**
  * the P7 seed loop and the layout list format: seed, family, boxes with names, SHA-256;
  * the box naming rule;
  * the P5 split-match exclusion over the 1,650 regenerated layouts, under the recorded interpreter and NumPy;
  * the randomization schedule;
  * the analysis plan, with the corrected power figure.
* **M2 (runner, offline):**
  * the P4 and P5 subclasses with their own `preconditions()`, `prepare_run()` and `execute()`;
  * the two runner views, one owner copy and shim, the block sequencer and the host-check tool;
  * the C6 test offline, against the 14 accepted world hashes.
* **M3 (classifier, after D14):**
  * the validity rule, the outcome classes and the two restated P6 §6.3 rules;
  * the metrics, through `p5_comparison.flight_metrics()`;
  * tests on R1–R4 and on retained P6 runs, including negative cases.

**Plan and contract text this audit corrects (no decision changes):**
* plan §4.5: power 0.83 at 200 pairs on the sampled population;
* plan §4.1 and contract §4.1: the population statement of §4.3.

They are recorded in the plan and in contract §4.1. With D13 and D14 approved, contract §5 and §7.3 now encode
them.
