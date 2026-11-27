# v3-SIM P7-E E5: the engineering report

**Status: ACCEPTED by the operator on 2026-11-28 ("Accept E5"), so E5 is COMPLETE.** The report says
**"engineering validation passed"** (§4), which completes the P7-E engineering validation under contract
`p7-engineering-validation-v1`.
* E5 was authorized on 2026-11-28 ("Authorize (Recommended)") and run once.
* The acceptance authorizes nothing else (§5): no push or merge, no cleanup or deletion, no change to P2–P6 or to
  the frozen P7 candidate, and no superiority claim.

**Candidate:** `state/p7e-candidate-2b6ee5d-r2.json`, manifest
`e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608`.
**Rules:** contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §5–§7 and §10; the approved
report plan `p7e-report-plan-v1` (canonical `15e7f7b4…`); E2's report tool, as accepted
([E2 §4](v3_sim_p7e_e2_code.md), interpretations 1–8).

## 1. What E5 does

E5 is offline only: no simulator, no flight, no change to any bound file. It runs the accepted
`simulation/px4_gazebo_p7e/analysis/p7e_report.py` (`8734bbe9…`) **once**:

```
cd /tmp && GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  …/simulation/px4_gazebo_p7e/analysis/p7e_report.py \
  --manifest ~/uav_autonomy/state/p7e-candidate-2b6ee5d-r2.json \
  --output   ~/uav_autonomy/state/p7e-engineering-report-e9561f05de28.json
```

* **Inputs:** the 3 block results and their ledgers. For each of the 60 flights, it reads the summary, the accepted
  P4 or P5 evaluation, the frozen P7 flight record and the P7-E assessment, each accepted only if its hash matches
  the ledger.
* **H1–H8:** computed from those records.
* **H9:**
  * the frozen P7 classifier is re-run on each of the 60 flights, through its own wrapper
    (`verify_p7_evidence.sh`, `99359ade…`, bound by P7 `0799c37e`). Each run gets a temporary symlink view without
    `p7/flight-record.json` and `p7e/`. The classifier writes its record only into that view.
  * The re-run must reproduce each record exactly, and FC-3 must reproduce each assessment. `assess` only reads.
  * Every run file's size and mtime must stay unchanged.
  * The manifest, the specs and the 60 world hashes must re-verify.
* **R1–R10:** descriptive only, with the frozen P7 interval functions and no p-values.
  * A breach of an accepted limit is the accepted evaluator's own failure item. It becomes a proposed new
    risk-register entry, not an engineering failure.
  * One such item is already known from E4: slot 27 P4, `performance:compute_max`.
* **Output:** written once (exclusive create). The tool refuses to run again if a report exists, and prints the
  status (exit 0 only for "engineering validation passed").

**After the run** (part of the same authorization):
* copy the report byte for byte to `docs/evidence/v3_sim_p7e_engineering_report.json`, with its hash recorded;
* write this document's results: the status, H1–H9, R1–R10, the risk register, the required statement, the
  not-claimed list and the 30-pair limits;
* commit locally; nothing is pushed.

**Launch:** detached, as the blocks were (`setsid nohup`, log `~/uav_autonomy/logs/p7e-e5-report-e9561f05de28.log`).
This way the end of a session cannot kill it. If it dies before writing, there is no report and nothing changes.
A second run then needs a new authorization.

**Duration:** minutes (plan §10). This was written before the run, when the classifier had not been timed on
these runs; the run took 3.4 min (§4).

**If the status is "not passed":** the report is kept as written, with no rerun and no change to any threshold.
The operator then decides (contract §7.3).

## 2. The stopped first attempt (`38e23bad…`)

The report tool reads only the results of the manifest it is given, so it covers the 60 flights of `e9561f05…`.
The stopped first attempt is described in this document, not in the report JSON:
* the first real candidate, `38e23bad…`;
* block 1 was terminated externally after 2 flown flights and 1 interrupted one;
* it was recovered under authorization into a recorded stop: `sequencer_terminated_externally`, all 3 flights safe.

That evidence stays protected. Its flights are not part of the 60.

**Re-flown layouts** (as the re-fly decision requires): three of the 60 flights had already been flown once on
`38e23bad…`, with the same code:
* slot 1, `p7c-4000351`, P4: then a valid goal success;
* slot 1, `p7c-4000351`, P5: then a valid failure. P2 `stale_action` ended in RECOVERY; the flight had integrity
  items only and no safety event;
* slot 2, `p7c-4000367`, P4: then the interrupted flight, evaluated afterwards as a valid goal success.

On `e9561f05…`, slots 1 and 2 were both-success pairs. This follows the approved re-fly plan: the first attempt was
not discarded, and its outcomes are not merged into R1 or R2.

## 3. Pre-check (read-only, 2026-11-28)

| Check | Result |
|---|---|
| manifest `e9561f05…` verifies (P7 `0799c37e` and P3–P6 inside), development false | yes |
| source drift | none |
| specs identity equals the manifest's | yes |
| report code `8734bbe9…`; report plan canonical `15e7f7b4…`; schedule canonical `9b3deacb…` | all match |
| block results 1, 2, 3 present, none stopped | yes |
| flights in the ledgers | 60 |
| evidence complete and hash-matched (summary, evaluation, record, assessment, ledger hash) | 60 of 60 |
| run directories and `p7/layout.json` present | 60 of 60 |
| classifier wrapper `99359ade…` present; bound by the P7 manifest | yes |
| classifier environment: `venv-px4`, ROS Jazzy, P2–P5 installs | present |
| report output and evidence copy | neither exists |
| live lock; simulator processes | none; none |
| free space (`/tmp` and the evidence root) | 687 GB |
| the 76 P7-E offline tests (`tests/test_px4_gazebo_p7e_*.py`) | 76 passed (32 s) |
| working tree (Windows git) | clean apart from the untracked `.codegraph/` |

**Note (RR5):** `git status` inside WSL lists many files as modified. It ignores the Windows `core.autocrlf`, so
the CRLF working copies look changed. Windows git shows none, and the manifest finds no drift. Nothing was changed.

No H or R value was computed in the pre-check.

## 4. Result (run once, 2026-11-28)

**Records:**
* the report: `state/p7e-engineering-report-e9561f05de28.json`, 80 574 bytes, SHA-256 `e2fb9607…c12d09c`;
* its byte-identical copy: [`evidence/v3_sim_p7e_engineering_report.json`](evidence/v3_sim_p7e_engineering_report.json)
  (same hash, compared with `cmp`).
* **Run:**
  * detached (PID 380014), 3.4 min (16:43:56 to 16:47:19 UTC); log `~/uav_autonomy/logs/p7e-e5-report-e9561f05de28.log`;
  * exit 0, printing `{"status": "engineering validation passed", "not_passed_reasons": []}`.
* **Bindings in the report:**
  * report code `8734bbe9…`, report plan canonical `15e7f7b4…`, schedule canonical `9b3deacb…`;
  * manifest `e9561f05…`, with P7 `0799c37e` and P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775`.
* **After the run:**
  * the manifest still verifies;
  * no H9 view directory is left in `/tmp`, and no lock is held;
  * H9 shows that no run file changed.

**Status: "engineering validation passed"**, with no not-passed reason. It carries the contract's statement of a
pass: *"the accepted P4 and P5 stacks completed the declared 60-flight engineering validation through the P7
infrastructure, with every hard criterion H1-H9 met"*.

### Hard criteria (all met)

| ID | Criterion | Limit | Value |
|---|---|---|---|
| H1 | collisions | 0 | 0 |
| H2 | minimum truth clearance, every valid flight (60, none unmeasured) | ≥ 0.20 m | 0.310 m |
| H3 | unsafe commands | 0 | 0 |
| H4 | Policy A breaches (60 flights checked, none unchecked) | 0 | 0 |
| H5 | landed and disarmed with own commands 176/400/21 | every valid flight | 60 of 60 |
| H6 | safety-unverified flights (FC-3 included) | 0 | 0 |
| H7 | complete, hash-matched evidence; ledgers intact line for line | 100 % | 60/60; 3 ledgers match |
| H8 | plan, accounting, replacement cap, stops explained | per block | 3 blocks unstopped, 20 `flown_valid` each, 0 replacements; batch completed |
| H9 | reproducibility | identical | manifest and specs verify; results bind the manifest; worlds 60/60; frozen classifier reproduces 60/60; FC-3 reproduces 60/60; run files unchanged 60/60 |

### Reported criteria (descriptive; every paired difference is P5 − P4; no p-value)

| ID | What | P4 | P5 |
|---|---|---|---|
| R1 | goal success (Clopper–Pearson 95 %) | 20/30 = 0.667 [0.472, 0.827] | 22/30 = 0.733 [0.541, 0.877] |
| R3 | P2 timing passed; observation-gap aborts | 30/30; 0 | 30/30; 0 |
| R4 | perception compute max (median / max); accepted limit 0.090 s | 0.025 / **0.640 s** (1 breach) | 0.024 / 0.043 s |
| R5 | P5 inference max (median / max); limit 0.010 s; failure ticks, fallbacks, outside-mask | — | 0.0017 / 0.0063 s; 0, 0, 0 |
| R6 | response latency max; tracking max (limits 0.25 s, 0.30 m) | 0.164 s; 0.263 m | 0.152 s; 0.269 m |
| R8 | wall time per flight (median / max); RTF mean (median) | 168 / 190 s; 0.998 | 168 / 233 s; 0.998 |
| R9 | abort types | `hover_timeout` 10 | `hover_timeout` 8 |
| R10 | integrity events | `performance:compute_max` 1 (slot 27) | `cleanup:gazebo:forced_kill_after_grace` 1 (slot 10) |

* **R2, paired (30 pairs, none missing):**
  * both reached the goal 20, neither 8, P5 only 2, P4 only 0;
  * difference +0.067, Tango 95 % [−0.054, 0.213].
* **R7, on the 20 both-success pairs (Hodges–Lehmann):**
  * time to goal −0.255 s [−1.010, 0.100] (medians 41.79 s and 41.85 s);
  * path length +0.027 m [−0.122, 0.141].
* **R8, host:**
  * every block's host check passed (CPU mean 0.0, 1.4, 2.0 %);
  * minimum available memory 6.6 GB;
  * every flight has at least one stall below RTF 0.5 (median 2 for both controllers), as in the pilot.
  * The 60 runs use 83.7 GB. The largest is slot 12 P5 at 2.81 GB, above the pilot's 1.82 GB maximum (RR4, no
    change).
* **Safety bounds:** 0 safety events per controller. The one-sided 95 % upper bound is 0.095 per flight for each
  controller.
* **Mission failures (corrected 2026-11-28, release closure).** 18 flights failed the mission: P4 10 of 30 and P5 8 of
  30 (R1, R9).
  * All 18 were safe `hover_timeout` aborts by the accepted P4 supervisor: hover, then land and disarm through
    P2's nominal sequence, with no safety event.
  * The report's field `safety_per_controller.mission_failure_flights` is 0 for both controllers, but it counts
    something narrower. It counts flights that carry a D14 mission-failure **class item**: `safety:tracking` or
    `safety:response_latency` (`p7_classify.MISSION_ITEMS`). There were none.
  * That field is not a count of failed missions. This document previously read "0 mission-failure flights",
    which repeated the field without that qualification.
  * The accepted report JSON is unchanged and correct under its own definitions; see the
    [release closure](v3_sim_p7e_release_closure.md) §1.2.
* **Paired clearance (descriptive, no non-inferiority test):** P5 − P4 median +0.002 m, range [−0.094, +0.593].

### Risk register

* **Open, and closed by neither result (contract §10):**
  * RR1, the P2 observation-gap RECOVERY: 0 aborts per controller here (the pilot had 2, both P5);
  * RR2, P5 repeat-behaviour inconsistency: P7-E cannot measure it.
* **Proposed new entry from an R breach:**
  * R4 `performance:compute_max`, slot 27 P4 (`20261127T155834Z-p7e-p4-p7c-4000352-8ae0dc7d`);
  * one perception frame took 0.640 s of compute (2 involuntary context switches), with that flight's p99 at
    0.045 s;
  * treatment proposed: record and monitor, with no threshold change.
* **R10 integrity events** are reported, not entered as risks:
  * slot 27 P4, the same frame;
  * slot 10 P5, a forced Gazebo kill at shutdown.

### Statements carried by the report

* **Required statement (verbatim):** "This engineering validation was not designed to compare P5 with P4. With 30
  pairs, a test of the planning effect would have about 3 % power. Observed differences between the controllers
  are descriptive and do not show that either is superior."
* **Not claimed:**
  * superiority or inferiority of either ranker;
  * statistical significance;
  * real-hardware behaviour;
  * results outside the declared layouts, mission, speed, static boxes or this host;
  * any change to P2–P6 acceptance.
* **30-pair limits:**
  * McNemar power 3 % at the planning effect, 8 % at the pilot's discordance;
  * Tango width about 0.29–0.32;
  * the 95 % interval for 15/30 is [0.313, 0.687];
  * the zero-event bound is 0.095 per flight over 30 flights, 0.049 over 60.

## 5. What the acceptance means (accepted 2026-11-28, "Accept E5")

Accepting this report closed E5 and the P7-E engineering validation under contract `p7-engineering-validation-v1`:
the accepted P4 and P5 stacks ran 60 flights safely and completely through the P7 infrastructure. It authorizes
nothing else.
* No push and no merge.
* No cleanup or deletion. Under D4-B, P7-E's raw depth becomes eligible only for a separately designed, tested and
  approved cleanup.
* No change to P2–P6 or to the frozen P7 candidate, and no use of the 200-pair schedule-selection record.
* No superiority claim.
