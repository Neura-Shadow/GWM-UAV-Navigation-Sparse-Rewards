# v3-SIM P7 M5 — pilot: stopped after 6 of 40 flights (2026-11-21)

**Status: STOPPED by a stop rule. Nothing more is flown until the operator decides (contract §7.3).**

**Approvals (operator, 2026-11-21):**

| Approval | Record |
|---|---|
| Production Policy A ("Approve Policy A for M5") | `state/p7-policy-A-approval-4e26f16.json`, binding 19 P7 files by canonical digest `727d9e98…` |
| Real freeze ("Freeze it (Recommended)") | candidate `6f60492fb1f9621810ee0663c714d7bd93b2fcfcfe91fb59c30acc1b0a7a7b9e` (`state/p7-candidate-4e26f16.json`), identical in content to the M4 development candidate |
| Pilot ("Authorize the pilot (Recommended)") | `state/p7-authorization-pilot-block-01-6f60492fb1f9.json`, consumed by its marker after the host check passed |

## 1. What happened

| Slot | Flight | Run | Outcome |
|---|---|---|---|
| 1 | P5, `p7p-5000010` | `…-p7-p5-p7p-5000010-…` | valid, goal success (41.18 s, clearance 0.825 m) |
| 1 | P4, `p7p-5000010` | `…-p7-p4-p7p-5000010-…` | valid, goal success (41.04 s, clearance 0.775 m) |
| 2 | P5, `p7p-5000013` | | valid, failure: supervisor `hover_timeout` abort, landed |
| 2 | P4, `p7p-5000013` | | valid, failure: `hover_timeout` abort, landed |
| 3 | P4, `p7p-5000016` | | valid, failure: `hover_timeout` abort, landed |
| 3 | P5, `p7p-5000016` | `20261121T134526Z-p7-p5-p7p-5000016-0706459a` | valid; **the runner exited 1; the evaluator crashed; the flight was classified safety-unverified, and the batch stopped** |

* **Result:** `state/p7-pilot-block-01-6f60492f….result.json`, with its ledger. 34 flights were not flown, there was no
  replacement, and the candidate was unchanged at the end.
* **Afterwards:** the live lock was released and no simulator process remains.

## 2. The stopped flight (read-only investigation)

**What happened.** In TRANSIT, while AVOIDING, the accepted **P2 controller aborted to RECOVERY** with
`observation_gap` at sim 79.0 s. That is its PX4-state freshness rule (`max_sample_gap_sim_s`). The run's minimum
interval real-time factor was 0.013, so the simulation briefly almost stalled; its mean RTF was 0.998. The P2
abort made `p5_control` exit 1. It was not a P5 policy failure.

**Was it safe? Yes, by every record**, checked with the accepted functions and their own environment:

| Check | Result |
|---|---|
| Truth safety (`p4_evaluate.safety`) | all items pass: clearance 0.918 m, 0 collisions, tracking 0.255 m, truth interval 0.036 s, no setpoint inside an obstacle |
| Commands (ULog) | own sequence exactly 176, 400, 21; the P2 RECOVERY sent its one LAND |
| Landing (ULog) | landed and disarmed; no PX4 failsafe |
| Policy A (`p4_evaluate.native_commands`) | clean |

**Why it still stopped the pilot. Two gaps:**
1. **A gap in my M2 runner.** The P7 runner reuses the accepted P4 `run_controller`. That function raises
   `controller_failed` before it writes the flight window. The unchanged evaluator then has no `sensor_window_sim_s`
   and crashes (`evaluator_incomplete_evidence`), which loses every result. The P6 runner had solved exactly this: it
   writes the accepted flight window (PRESTREAM to COMPLETE or RECOVERY) before it reports a fail-closed exit
   (`p6_runner.flight_window`). I missed it in M2.
2. **The accepted P2 integrity function cannot process this flight.** Its yaw-reference classification raises
   `stale_state:vehicle_local_position` because of the same gap. So even with gap 1 fixed, the accepted evaluator
   would record `p2_integrity_error`, and D14 maps that to safety-unverified, which stops the batch. Every P2
   observation-gap RECOVERY would stop the batch, however safely it lands.

**How often.** 1 of 6 flights so far had a P2 observation-gap abort. The pilot is meant to measure such
infrastructure rates. Under the contract a post-arming P2 abort is outcome data (a failure), so it affects both
controllers alike under the randomized order. But it adds noise to the comparison.

## 3. Decision needed: D15 (proposed)

| Option | What it means |
|---|---|
| **A (recommended)** | (1) **Runner fix:** write the accepted flight window before reporting a fail-closed exit, exactly as P6 does. (2) **Classifier refinement of D14:** if the accepted evaluator's P2 integrity part raises, the classifier checks the safety-relevant P2 facts itself, from the ULog with the accepted P2 reader: own command sequence 176, 400, 21, landed and disarmed, no PX4 failsafe, plus the accepted Policy A check. If they hold, the item is an integrity event; if not, a safety event; if the ULog is unreadable, safety-unverified. (3) **Tests:** offline, with this real flight as the fixture. (4) **New candidate:** a new real freeze, a new production Policy A approval (it binds the source digest) and a new pilot authorization. The stopped pilot stays on record as diagnostic data, and pilot data never enters the confirmatory analysis. |
| B | Apply the runner fix only, and keep D14 strict. Every P2 observation-gap RECOVERY then stops the batch for review, which on this evidence could be every few flights. |
| C | Halt P7 and investigate the simulator stall first (the minimum RTF of 0.013) before any change. |

**Why A:**
* The flight landed safely and every safety check passes from the raw records.
* The stop came from a runner gap that P6 had already solved, and from a P2 evaluator function that cannot handle the
  abort it reports.
* Option A keeps safety verified, now from the raw ULog, without lowering any threshold. A flight that fails those
  checks still stops the batch.
