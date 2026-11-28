# v3-SIM P8: the validation blocks of `p8-validation-60` (V3, V4, V5)

**Candidate:** the real P8 candidate, manifest `fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`
(`state/p8-candidate-2a0ae57.json`; [M3 record](v3_sim_p8_m3_candidate.md) §8.3).

**Rules for every block:**
* Each block needs its own authorization phrase and a detached launch (contract §13 item 7).
* Nothing is retried automatically. A valid flight is never re-flown, and a result is written once.

## 1. Block 1 = V3: COMPLETED (2026-11-28; operator "Prepare V3 block 1", then "Authorize block 1")

**In short:** not stopped; 20 of 20 flights flown and valid; 0 safety events; H10 passed on every flight. The result
is in §1.5–§1.9. §1.1–§1.4 are the preparation as it was approved.

### 1.1 The plan (approved schedule `df90cf7b…`; 10 pairs, 20 flights, in this order)

The layouts are those of the P7-E hover-timeout failures. The schedule fixes the order within each pair.

| # | Slot | Variant | Layout | | # | Slot | Variant | Layout |
|---:|---:|---|---|---|---:|---:|---|---|
| 0 | 1 | P8-P5R | `p7c-4000355` | | 10 | 6 | P8-P5R | `p7c-4000363` |
| 1 | 1 | P8-P4R | `p7c-4000355` | | 11 | 6 | P8-P4R | `p7c-4000363` |
| 2 | 2 | P8-P5R | `p7c-4000381` | | 12 | 7 | P8-P4R | `p7c-4000366` |
| 3 | 2 | P8-P4R | `p7c-4000381` | | 13 | 7 | P8-P5R | `p7c-4000366` |
| 4 | 3 | P8-P5R | `p7c-4000396` | | 14 | 8 | P8-P4R | `p7c-4000360` |
| 5 | 3 | P8-P4R | `p7c-4000396` | | 15 | 8 | P8-P5R | `p7c-4000360` |
| 6 | 4 | P8-P4R | `p7c-4000375` | | 16 | 9 | P8-P5R | `p7c-4000379` |
| 7 | 4 | P8-P5R | `p7c-4000375` | | 17 | 9 | P8-P4R | `p7c-4000379` |
| 8 | 5 | P8-P4R | `p7c-4000377` | | 18 | 10 | P8-P4R | `p7c-4000385` |
| 9 | 5 | P8-P5R | `p7c-4000377` | | 19 | 10 | P8-P5R | `p7c-4000385` |

Two of these layouts were flown in the development dry run, on the retired development candidate and without credit:
`p7c-4000360` with P8-P4R and `p7c-4000375` with P8-P5R. Here they are flown again on the real candidate.

### 1.2 Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| Manifest | `verify_current` from `/tmp`: `fe2a0c6e…`; development `false`; source drift none (FC-7) |
| Inside it | P7-E `e9561f05…`, P7 `0799c37e…`, P3–P6, the build `1a78c6ba…`, the four specs, V1/V2 |
| MC trace build | `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1` selects the frozen build |
| Plan | 20 flights, stage V3 |
| FC-1 | block 1 needs no previous result |
| Records for block 1 | no result, ledger, consumption marker or authorization |
| Live locks | none (P8, P7-E, P7) |
| Host check (frozen, FC-2) | passed: AC power, overlay, host CPU, no simulator process, workspace lock free, disk floor |
| Disk | 744.4 GB free; the frozen floor needs 150 GB after the planned 26 GB |
| Git | HEAD `b9e0070`; clean apart from `.codegraph/` |

### 1.3 What the authorization does

**1. The operator gives the exact phrase:**
"authorize P8 p8-validation-60 block 1 fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a"

**2. The authorization file is written** (exclusive create):
```
p8_manifest.py authorize /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json --block 1
  --operator Neura-Shadow --confirm "<phrase>"
  --output /home/joker0625/uav_autonomy/state/p8-authorization-p8-validation-60-block-01-fe2a0c6e9052.json
```
It authorizes exactly one execution of block 1, `max_attempts: 1`, in simulated flight only.

**3. A detached launch:**
* the command, with the three explicit gates and the MC trace selector:
  ```
  run_p8_block.py --scope p8-validation-60 --block 1
    --manifest /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json --authorization <the file above>
  ```
* the log and PID go to `~/uav_autonomy/logs/p8-block-01-fe2a0c6e9052.{log,pid}`;
* a watcher, and keep-awake.

**4. The runner checks everything again before flying:**
* the manifest, the MC trace build and the authorization binding;
* FC-1, the locks and the host check.

If the host check fails, it writes a not-flown record, and the authorization is not consumed. Otherwise it consumes
the authorization (marker), takes the live lock and flies the 20 flights.

### 1.4 During the block

**Each flight:**
* runs through `run_p8.sh`;
* is evaluated by the unchanged P4 or P5 evidence script;
* is classified by the frozen P7 classifier, then assessed by FC-3;
* is checked by H10 (FC-5, FC-6);
* goes into the fsynced ledger before the next flight starts.

**Stops:** any stop rule ends the block:
* FC-1, an exception;
* FC-2, a failed process check;
* FC-3, an unknown arm state;
* FC-5 or FC-6, H10;
* FC-7, drift.

The result records every planned flight.

**Replacements:** only an infrastructure-invalid flight is re-flown.
* At most once per flight, with a cap of 2 for the block (floor of 10 % of 20).
* Hitting either limit stops the block.

**Expected duration:** the dry run took 7.8 min for 2 flights, so about 75–100 min here. The V3 layouts are failure
layouts, so hover timeouts and recovery may lengthen the flights.

**Disk:** about 28–54 GB (1.4–2.7 GB per flight), against 744 GB free.

**Evidence retention (D4-A):**
* Every flight records completely: every depth frame at zlib-1, as the frozen evaluators require.
* None of it is eligible for removal.
* The proposed P8 rule R-2 ([retention design](v3_sim_evidence_retention_design.md) §6) would make it eligible only
  after the P8 report is accepted, and only with a separate decision.

**Afterwards:**
* The result is recorded here, with no automatic re-run.
* Block 2 (V4) needs this block's result unstopped and complete (FC-1), and its own authorization.

### 1.5 The run (2026-11-28)

* **The authorization** (operator "Authorize block 1"):
  * the file `state/p8-authorization-p8-validation-60-block-01-fe2a0c6e9052.json` (`02d8dfd0…`);
  * block 1, stage V3, `max_attempts: 1`, authorized by Neura-Shadow, simulated flight only.
* **The launch:** detached, PID 618148, its own session with no terminal; log `~/uav_autonomy/logs/p8-block-01-fe2a0c6e9052.log`.
* **The runner's own checks passed:**
  * the host check: AC online, the overlay, CPU mean 1 %, no simulator process, the workspace lock free, 744.4 GB free;
  * then it consumed the authorization and took the live lock.
* **Duration:** 75.4 min for the 20 flights (07:23–08:38 UTC).

**The records:**

| Record | File (`state/`) | SHA-256 |
|---|---|---|
| Result | `p8-p8-validation-60-block-01-fe2a0c6e9052….result.json` | `8d768d01b78f909a21350b047b06ffa18d5a117b264a4fff41abd635ade1df95` |
| Ledger (20 lines, fsynced) | `p8-p8-validation-60-block-01-fe2a0c6e9052….ledger.jsonl` | `34c67ef8b235777d503c75296ec100be32ef8f35ff521cbf4f91daa62d45b2ab` |
| Consumption marker | `p8-p8-validation-60-block-01-fe2a0c6e9052….consumed.json` | `70dd58acb47199bd7dfb0074fedeb9ab17b5690be452293b673a687d463b5ca4` |

### 1.6 The result

**Not stopped:**
* stop reason none, no exception, not interrupted, not recovered from the ledger;
* 20 of 20 flown, all **valid**;
* no replacement (the cap was 2), no missing pair, nothing unflown;
* accounting complete (20 `flown_valid`).

**At the end:**
* the candidate unchanged and no drift (FC-7);
* the live lock released, and no simulator process left;
* `engineering_credit: true`, `dry_run: false`, `automatic_retry: false`.

**Safety (all 20 flights):**
* 0 safety events, 0 safety-unverified flights, 0 mission failures, FC-3 not applied, 0 collisions;
* every flight landed and disarmed, final state `COMPLETE`;
* the minimum truth clearance was 0.344 m (slot 10, P8-P4R). The limit is ≥ 0.20 m.
* **H10 passed on all 20 flights:** 0 violations, so no FC-5 and no FC-6. Every executed recovery action reproduces its
  frozen check, including all 142 recovery translations.

### 1.7 Outcomes (descriptive only)

**These counts are descriptive only.** The effectiveness target is none (P8-C1), and comparisons belong to the
report.

| Slot | Layout | P8-P4R | P8-P5R |
|---:|---|---|---|
| 1 | `p7c-4000355` | `hover_timeout` | `hover_timeout` |
| 2 | `p7c-4000381` | `hover_timeout` | goal (44.1 s) |
| 3 | `p7c-4000396` | `hover_timeout` | `hover_timeout` |
| 4 | `p7c-4000375` | goal (144.8 s) | `hover_timeout` |
| 5 | `p7c-4000377` | `hover_timeout` | `transit_deadline` |
| 6 | `p7c-4000363` | goal (69.4 s) | `hover_timeout` |
| 7 | `p7c-4000366` | `transit_deadline` | goal (70.5 s) |
| 8 | `p7c-4000360` | goal (64.2 s) | goal (51.9 s) |
| 9 | `p7c-4000379` | `hover_timeout` | `hover_timeout` |
| 10 | `p7c-4000385` | goal (65.5 s) | goal (44.8 s) |

* **Goal success:** P8-P4R 4 of 10, P8-P5R 4 of 10. The times are simulated time to the goal.
* **The 12 failures** are all the accepted supervisor's own abort (`supervisor_abort`), each followed by a landing:
  * 10 are `hover_timeout`;
  * 2 are `transit_deadline`.
* **By pair:**
  * both reached the goal: slots 8 and 10;
  * both failed: slots 1, 3, 5 and 9;
  * one of the two reached the goal: slots 2, 4, 6 and 7.

### 1.8 Recovery (descriptive)

**HOVER episodes:**
* 60 episodes in 18 flights. In 2 flights the vehicle never hovered: slot 2 P8-P5R and slot 10 P8-P5R.
* In slot 2 P8-P4R the one episode's search found its start unusable (`start_usable: false`). So recovery planned
  nothing and held, and the supervisor's `hover_timeout` followed.

**8,680 recovery actions were executed:**

| Phase | Actions | Flights |
|---|---:|---|
| R1 (yaw) | 1,216 | 5: slot 6 P4R, slot 7 P5R, slot 8 both, slot 10 P4R. All 5 reached the goal. |
| R2 (yaw) | 3,176 | 10 |
| R3 (yaw) | 4,146 | 6 |
| R3 (translation) | 142 | 6: slot 1 P5R (17), slot 5 P4R (12), slot 5 P5R (40), slot 7 P4R (4), slot 9 P5R (26), slot 9 P4R (43) |

**One FC-4 refusal** (slot 4 P8-P5R, `p7c-4000375`):
* an R3 translation was refused by the frozen `collision_check` (`occupied`);
* recovery held, which is not a stop;
* it is the same layout and variant as the dry run's one refusal.

### 1.9 Evaluations and integrity events (R10)

**Evaluations:**
* The accepted P4 evaluation passed on all 10 P8-P4R flights.
* The accepted P5 evaluation passed on 1 P8-P5R flight (slot 10). On the other 9 it reads "failed", each time on
  one of the two items below alone.

**`policy:ledger:decision_mismatch`, on 8 P8-P5R flights** (slots 1 and 3–9: every P8-P5R flight in which recovery
acted):
* This is the expected item of finding F-DR1.
* The D-M3-1 reconciliation (`p8_report.ledger_reconciliation`, run read-only) explains all 8 exactly:
  * the policy-ledger and decision counts are equal;
  * the 4,244 mismatches are the 4,244 recovery actions, as a set;
  * they differ in `action` alone, with 0 other-field mismatches.

**`cleanup:gazebo:forced_kill_after_grace`, on 1 flight** (slot 2 P8-P5R, a goal success):
* Gazebo needed a forced stop at shutdown.
* It is the same item as in P7-E block 1 (slot 10, P5).
* The flight is valid, with no safety item, and it is reported under R10.

### 1.10 Disk and retention

* **Disk:** 37.44 GB for the 20 flights (1.15–3.39 GB each). WSL free space went from 744.4 to 706.9 GB.
* **Retention:** every flight keeps its complete raw evidence (D4-A). None of it is eligible for removal (§1.4).

### 1.11 What comes next

* **FC-1 allows block 2:** block 1 is unstopped and complete.
* **Block 2 = V4 needs its own steps:**
  * its preparation (a read-only pre-check);
  * then its own phrase, "authorize P8 p8-validation-60 block 2 fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a".

## 2. Block 2 = V4: COMPLETED (2026-11-28; operator "Prepare V4 block 2", then "Authorize block 2")

**In short:** not stopped; 20 of 20 flights flown and valid; 0 safety events; H10 passed on every flight. One flight
(slot 14 P8-P5R) was a P2 fail-closed landing before navigation, after a 0.79 s stall in the accepted controller
(finding F-B2-1, §2.9). The result is in §2.5–§2.11. §2.1–§2.4 are the preparation as it was approved.

### 2.1 The plan (approved schedule; 10 pairs, 20 flights, in this order)

The layouts are the **10 held-out layouts** of [M1 specs](v3_sim_p8_m1_specs.md) §2:
* new seeds from 6,000,000, disjoint from every P5 and P7 set;
* 5 of F1 type and 5 of F2 type;
* none of them has been flown before, in any phase or dry run.

Outcomes here are reported absolutely, with no P7-E baseline (contract §5). The schedule fixes the order within
each pair.

| # | Slot | Variant | Layout | Type | | # | Slot | Variant | Layout | Type |
|---:|---:|---|---|---|---|---:|---:|---|---|---|
| 0 | 11 | P8-P5R | `p8h-6000027` | F2 | | 10 | 16 | P8-P5R | `p8h-6000050` | F2 |
| 1 | 11 | P8-P4R | `p8h-6000027` | F2 | | 11 | 16 | P8-P4R | `p8h-6000050` | F2 |
| 2 | 12 | P8-P4R | `p8h-6000023` | F1 | | 12 | 17 | P8-P4R | `p8h-6000039` | F1 |
| 3 | 12 | P8-P5R | `p8h-6000023` | F1 | | 13 | 17 | P8-P5R | `p8h-6000039` | F1 |
| 4 | 13 | P8-P5R | `p8h-6000038` | F1 | | 14 | 18 | P8-P4R | `p8h-6000057` | F2 |
| 5 | 13 | P8-P4R | `p8h-6000038` | F1 | | 15 | 18 | P8-P5R | `p8h-6000057` | F2 |
| 6 | 14 | P8-P4R | `p8h-6000033` | F1 | | 16 | 19 | P8-P5R | `p8h-6000014` | F1 |
| 7 | 14 | P8-P5R | `p8h-6000033` | F1 | | 17 | 19 | P8-P4R | `p8h-6000014` | F1 |
| 8 | 15 | P8-P5R | `p8h-6000019` | F2 | | 18 | 20 | P8-P4R | `p8h-6000065` | F2 |
| 9 | 15 | P8-P4R | `p8h-6000019` | F2 | | 19 | 20 | P8-P5R | `p8h-6000065` | F2 |

### 2.2 Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| Manifest | `verify_current` from `/tmp`: `fe2a0c6e…`; development `false`; source drift none (FC-7) |
| MC trace build | `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1` selects the frozen build |
| Plan | 20 flights, stage V4 |
| FC-1 | `previous_block_ok` passes. Block 1's result is for this manifest and plan, not stopped, with nothing unflown, accounting complete, and its ledger intact (20 records, `34c67ef8…`). |
| Records for block 2 | no result, ledger, consumption marker or authorization |
| Live locks | none (P8, P7-E, P7) |
| Host check (frozen, FC-2) | passed: AC power, overlay, host CPU (mean 1 %), no simulator process, workspace lock free, disk floor |
| Disk | 706.9 GB free; the frozen floor needs 150 GB after the planned 26 GB |
| Worlds (offline) | the accepted P4 generator, run in a temporary directory, regenerates all 10 V4 worlds to their approved hashes. It also regenerates the 10 V3 worlds, as a control. The pinned `default.sdf` matches. |
| Git | HEAD `b50161c`; clean apart from `.codegraph/` |

The world check matters here because these 10 worlds have never been flown. A differing world would abort a flight
(`p8_world_differs_from_the_approved_world`) after the authorization was consumed.

### 2.3 What the authorization does

**1. The operator gives the exact phrase:**
"authorize P8 p8-validation-60 block 2 fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a"

**2. The authorization file is written** (exclusive create):
```
p8_manifest.py authorize /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json --block 2
  --operator Neura-Shadow --confirm "<phrase>"
  --output /home/joker0625/uav_autonomy/state/p8-authorization-p8-validation-60-block-02-fe2a0c6e9052.json
```
It authorizes exactly one execution of block 2, `max_attempts: 1`, in simulated flight only.

**3. A detached launch:**
* as in block 1 (§1.3), with `--block 2` and the file above;
* the log and PID go to `~/uav_autonomy/logs/p8-block-02-fe2a0c6e9052.{log,pid}`;
* a watcher, and keep-awake.

**4. The runner checks everything again before flying,** FC-1 included. If the host check fails, it writes a
not-flown record, and the authorization is not consumed.

### 2.4 During and after the block

* **The rules are those of §1.4:**
  * stops FC-1, FC-2, FC-3, FC-5, FC-6 and FC-7;
  * replacements only for infrastructure-invalid flights, at most once each, with a cap of 2.
* **Expected duration:** block 1 took 75.4 min, so about 65–100 min.
* **Disk:** block 1 used 37.4 GB (1.15–3.39 GB per flight), against 706.9 GB free.
* **Retention:** every flight keeps its complete raw evidence (D4-A).
* **Afterwards:**
  * the result is recorded here, with no automatic re-run;
  * block 3 (V5) needs this block's result unstopped and complete (FC-1), and its own authorization.

### 2.5 The run (2026-11-28)

* **The authorization** (operator "Authorize block 2"):
  * the file `state/p8-authorization-p8-validation-60-block-02-fe2a0c6e9052.json` (`cc62b365…`);
  * block 2, stage V4, `max_attempts: 1`, authorized by Neura-Shadow, simulated flight only.
* **The launch:** detached, PID 748595, its own session with no terminal; log `~/uav_autonomy/logs/p8-block-02-fe2a0c6e9052.log`.
* **The runner's own checks passed:**
  * FC-1, and the host check (AC online, CPU mean 0.4 %, 706.9 GB free);
  * then it consumed the authorization and took the live lock.
* **Duration:** 66.8 min for the 20 flights.

**The records:**

| Record | File (`state/`) | SHA-256 |
|---|---|---|
| Result | `p8-p8-validation-60-block-02-fe2a0c6e9052….result.json` | `e1d48d76df25d29d094cbe94797dd9af96e24f155eed40305098c6da20d76155` |
| Ledger (20 lines, fsynced) | `p8-p8-validation-60-block-02-fe2a0c6e9052….ledger.jsonl` | `f6de21296e3891f839f5272b40ef47039a261d14f1d4dedc06502b9e22baca5f` |
| Consumption marker | `p8-p8-validation-60-block-02-fe2a0c6e9052….consumed.json` | `9415748cbba66dbea4020da1d859c0e88b6d58aff861b60cc9ebb0da99ff3f33` |

### 2.6 The result

**Not stopped:**
* stop reason none, no exception, not interrupted, not recovered from the ledger;
* 20 of 20 flown, all **valid**;
* no replacement (the cap was 2), no missing pair, nothing unflown;
* accounting complete (20 `flown_valid`).

**At the end:**
* the candidate unchanged and no drift (FC-7);
* the live lock released, and no simulator process left;
* `engineering_credit: true`, `dry_run: false`, `automatic_retry: false`.

**Safety (all 20 flights):**
* 0 safety events, 0 safety-unverified flights, FC-3 not applied, 0 collisions;
* every flight landed and disarmed. 19 ended in `COMPLETE`, and slot 14 P8-P5R ended in `RECOVERY` (§2.9);
* the minimum truth clearance was 0.342 m (slot 16, P8-P4R). The limit is ≥ 0.20 m.
* **H10 passed on all 20 flights:** 0 violations, so no FC-5 and no FC-6. Every executed recovery action reproduces its
  frozen check, including all 34 recovery translations.

### 2.7 Outcomes (descriptive only)

**These counts are descriptive only.** V4 has no baseline, and the effectiveness target is none (P8-C1).

| Slot | Layout | Type | P8-P4R | P8-P5R |
|---:|---|---|---|---|
| 11 | `p8h-6000027` | F2 | `hover_timeout` | `hover_timeout` |
| 12 | `p8h-6000023` | F1 | goal (62.6 s) | goal (43.6 s) |
| 13 | `p8h-6000038` | F1 | `transit_deadline` | `transit_deadline` |
| 14 | `p8h-6000033` | F1 | goal (41.4 s) | P2 `stale_action`, landed through RECOVERY (§2.9) |
| 15 | `p8h-6000019` | F2 | `hover_timeout` | `hover_timeout` |
| 16 | `p8h-6000050` | F2 | `hover_timeout` | `hover_timeout` |
| 17 | `p8h-6000039` | F1 | `hover_timeout` | `hover_timeout` |
| 18 | `p8h-6000057` | F2 | goal (45.1 s) | goal (42.5 s) |
| 19 | `p8h-6000014` | F1 | `hover_timeout` | goal (61.8 s) |
| 20 | `p8h-6000065` | F2 | goal (57.9 s) | goal (46.0 s) |

* **Goal success:** P8-P4R 4 of 10, P8-P5R 4 of 10. The times are simulated time to the goal.
* **The 12 failures:**
  * 11 are the accepted supervisor's own abort (`supervisor_abort`), each followed by a landing: 9 `hover_timeout`
    and 2 `transit_deadline`;
  * 1 is the P2 fail-closed landing of slot 14 P8-P5R.
* **By pair:**
  * both reached the goal: slots 12, 18 and 20;
  * both failed: slots 11, 13, 15, 16 and 17;
  * one of the two reached the goal: slots 14 and 19.

### 2.8 Recovery (descriptive)

**HOVER episodes:**
* 32 episodes in 13 flights. In 7 flights the vehicle never hovered: slot 12 P8-P5R, slot 14 (both), slot 18
  (both) and slot 20 (both).
* In slot 19 P8-P4R the one episode's search found its start unusable (`start_usable: false`). So recovery planned
  nothing and held, and the supervisor's `hover_timeout` followed.

**5,967 recovery actions were executed:**

| Phase | Actions | Flights |
|---|---:|---|
| R1 (yaw) | 464 | 2: slot 12 P4R, slot 19 P5R. Both reached the goal. |
| R2 (yaw) | 2,318 | 6 |
| R3 (yaw) | 3,151 | 4 |
| R3 (translation) | 34 | 3: slot 11 P5R (1), slot 13 P5R (21), slot 13 P4R (12) |

**Two FC-4 refusals** (slot 11 P8-P4R and slot 17 P8-P4R):
* each was an R3 translation refused by the frozen `collision_check` (`occupied`);
* recovery held, which is not a stop.

### 2.9 Finding F-B2-1: slot 14 P8-P5R, a P2 fail-closed landing before navigation

**What happened** (run `20261128T094355Z-p8-p8_p5r-p8h-6000033-44f39765`):
* The flight took off and passed the reference lock, the yaw handover and the 10 s initial hover.
* At 40.77 s sim, in `MAP_SCAN` (before the first policy decision), one control callback took **0.787 s** of wall
  time. The accepted P2 dispatch budget is 0.050 s.
  * Almost all of it was the accepted controller's `graph_check`, in `graph_json_persistence` (0.785 s).
  * In the other 19 flights of this block that step never took more than 8.2 ms, and no control callback took more
    than 20 ms.
* The accepted P2 controller failed closed (`stale_action`). It sent its one LAND (21) and landed through
  `RECOVERY`: own commands 176/400/21, landed and disarmed, no failsafe, Policy A checked.

**What it is not:**
* The P5 policy was never invoked (0 decisions).
* The recovery layer never acted: its ledger holds only the header, and H10 recorded 0 actions.
* There is no safety event and no safety-unverified class. The minimum clearance was 5.57 m.

**How the frozen rules filed it:**
* The frozen classifier recorded it as a **valid failure**:
  * the mission failure `safety:tracking` (its `tracking_max_m` is null: the flight never reached a tracking
    window);
  * 10 integrity events, including `runtime_status:Abort:controller_failed:stale_action`, `p2:timing`,
    `p2_integrity_error:ValueError:stale_state:vehicle_local_position` and `policy:no_policy_decisions`.
* So it is not infrastructure-invalid, and it was not replaced. It is counted as a V4 failure of P8-P5R.

**Context:**
* It is the same pattern as P7-E's stopped candidate `38e23bad`, block 1 flight 2: a P2 `stale_action`
  fail-closed RECOVERY, filed as a valid failure.
* It bears on **RR1** (a P2 fail-closed RECOVERY after a slowdown) and **RR-P8-5** (compute and I/O spikes).
  Contract §10 keeps both open and reported, and changes no threshold in response.
* In the report it falls under R-P8-5 (P2 timing) and R-P8-10 (integrity events).

### 2.10 Evaluations and integrity events (R10)

**Evaluations:**
* The accepted P4 evaluation passed on all 10 P8-P4R flights.
* The accepted P5 evaluation passed on 3 P8-P5R flights (slots 12, 18 and 20, where recovery never acted). It reads
  "failed" on the other 7:
  * 6 on `policy:ledger:decision_mismatch` (slot 13 also on `cleanup:gazebo:forced_kill_after_grace`);
  * slot 14 on the items of §2.9.

**17 integrity events:**
* **`policy:ledger:decision_mismatch`, on 6 P8-P5R flights** (slots 11, 13, 15, 16, 17 and 19: every P8-P5R flight
  in which recovery acted). The D-M3-1 reconciliation (run read-only) explains all 6 exactly:
  * the policy-ledger and decision counts are equal;
  * the 3,198 mismatches are the 3,198 recovery actions, as a set;
  * they differ in `action` alone, with 0 other-field mismatches.
* **`cleanup:gazebo:forced_kill_after_grace`, on 1 flight** (slot 13 P8-P5R): Gazebo needed a forced stop at
  shutdown. It is the same item as in block 1.
* **10 on slot 14 P8-P5R** (§2.9).

### 2.11 Disk, retention and what comes next

* **Disk:** 30.26 GB for the 20 flights (0.34–2.44 GB each). WSL free space went from 706.9 to 676.6 GB.
* **Retention:** every flight keeps its complete raw evidence (D4-A). Slot 14 P8-P5R is stale-action RECOVERY
  evidence, the kind that cleanup proposal v3 kept for the earlier phases.
* **FC-1 allows block 3:** block 2 is unstopped and complete.
* **Block 3 = V5 needs its own steps:**
  * its preparation (a read-only pre-check);
  * then its own phrase, "authorize P8 p8-validation-60 block 3 fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a".

## 3. Block 3 = V5: COMPLETED (2026-11-28; operator "Prepare V5 block 3", then "Authorize block 3")

**In short:** not stopped; 20 of 20 flights flown and valid; all 20 reached the goal; no regression against P7-E; 0
safety events; recovery never triggered. The result is in §3.5–§3.8. §3.1–§3.4 are the preparation as it was
approved.

### 3.1 The plan (approved schedule; 10 pairs, 20 flights, in this order)

V5 is the **regression** stage ([M1 specs](v3_sim_p8_m1_specs.md) §2):
* the 10 P7-E layouts on which both accepted stacks reached the goal;
* the first per family in slot order, with quotas pillars 3, wall_with_gap 3, mixed 3, partial_wall 1.

R-P8-9 will report each V5 flight that fails where its P7-E baseline succeeded, with its cause and whether recovery
acted. The schedule fixes the order within each pair.

| # | Slot | Variant | Layout | P7-E slot | | # | Slot | Variant | Layout | P7-E slot |
|---:|---:|---|---|---:|---|---:|---:|---|---|---:|
| 0 | 21 | P8-P4R | `p7c-4000351` | 1 | | 10 | 26 | P8-P5R | `p7c-4000394` | 5 |
| 1 | 21 | P8-P5R | `p7c-4000351` | 1 | | 11 | 26 | P8-P4R | `p7c-4000394` | 5 |
| 2 | 22 | P8-P4R | `p7c-4000357` | 11 | | 12 | 27 | P8-P5R | `p7c-4000373` | 8 |
| 3 | 22 | P8-P5R | `p7c-4000357` | 11 | | 13 | 27 | P8-P4R | `p7c-4000373` | 8 |
| 4 | 23 | P8-P5R | `p7c-4000374` | 13 | | 14 | 28 | P8-P4R | `p7c-4000398` | 4 |
| 5 | 23 | P8-P4R | `p7c-4000374` | 13 | | 15 | 28 | P8-P5R | `p7c-4000398` | 4 |
| 6 | 24 | P8-P5R | `p7c-4000354` | 20 | | 16 | 29 | P8-P4R | `p7c-4000349` | 10 |
| 7 | 24 | P8-P4R | `p7c-4000354` | 20 | | 17 | 29 | P8-P5R | `p7c-4000349` | 10 |
| 8 | 25 | P8-P5R | `p7c-4000359` | 17 | | 18 | 30 | P8-P4R | `p7c-4000367` | 2 |
| 9 | 25 | P8-P4R | `p7c-4000359` | 17 | | 19 | 30 | P8-P5R | `p7c-4000367` | 2 |

**The baseline:**
* In P7-E, every one of these 10 layouts was a goal success for both accepted stacks (`p7e_outcomes`).
* All 20 of those P7-E flights keep their flight records.
* Under cleanup C-7, 16 of them lost their raw depth. 4 still have it: slot 1 (P4 and P5), slot 2 P4 and slot 10 P5.
* P8 uses only their records, as the C-11 proof (`v3_sim_retention_record_only_p8.json`) showed: 0 `depth.bin` opens,
  and the outputs reproduce.

### 3.2 Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| Manifest | `verify_current` from `/tmp`: `fe2a0c6e…`; development `false`; source drift none (FC-7) |
| MC trace build | `GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1` selects the frozen build |
| Plan | 20 flights, stage V5 |
| FC-1 | `previous_block_ok` passes. Block 2's result is for this manifest and plan, not stopped, with nothing unflown, accounting complete, and its ledger intact (20 records, `f6de2129…`). |
| Records for block 3 | no result, ledger, consumption marker or authorization |
| Live locks | none (P8, P7-E, P7) |
| Host check (frozen, FC-2) | passed: AC power, overlay, host CPU (mean 1.8 %), no simulator process, workspace lock free, disk floor |
| Disk | 676.6 GB free; the frozen floor needs 150 GB after the planned 26 GB |
| Worlds (offline) | the accepted P4 generator, run in a temporary directory, regenerates all 10 V5 worlds to their approved hashes. The 20 V3 and V4 worlds also regenerate, as a control. The pinned `default.sdf` matches. |
| Git | HEAD `029737b`; clean apart from `.codegraph/` |

### 3.3 What the authorization does

**1. The operator gives the exact phrase:**
"authorize P8 p8-validation-60 block 3 fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a"

**2. The authorization file is written** (exclusive create):
```
p8_manifest.py authorize /home/joker0625/uav_autonomy/state/p8-candidate-2a0ae57.json --block 3
  --operator Neura-Shadow --confirm "<phrase>"
  --output /home/joker0625/uav_autonomy/state/p8-authorization-p8-validation-60-block-03-fe2a0c6e9052.json
```
It authorizes exactly one execution of block 3, `max_attempts: 1`, in simulated flight only.

**3. A detached launch:**
* as in blocks 1 and 2, with `--block 3` and the file above;
* the log and PID go to `~/uav_autonomy/logs/p8-block-03-fe2a0c6e9052.{log,pid}`;
* a watcher, and keep-awake.

**4. The runner checks everything again before flying,** FC-1 included. If the host check fails, it writes a
not-flown record, and the authorization is not consumed.

### 3.4 During and after the block

* **The rules are those of §1.4:**
  * stops FC-1, FC-2, FC-3, FC-5, FC-6 and FC-7;
  * replacements only for infrastructure-invalid flights, at most once each, with a cap of 2.
* **Expected duration:** blocks 1 and 2 took 75.4 and 66.8 min. V5 layouts were successes in P7-E, so the flights
  may be shorter: about 55–90 min.
* **Disk:** blocks 1 and 2 used 37.4 and 30.3 GB, against 676.6 GB free.
* **Retention:** every flight keeps its complete raw evidence (D4-A).
* **Afterwards:**
  * the result is recorded here, with no automatic re-run;
  * block 3 is the last block of `p8-validation-60`. The P8 report (`p8_report.py`, written once) and its acceptance
    need their own authorization (contract §13 item 8).

### 3.5 The run (2026-11-28)

* **The authorization** (operator "Authorize block 3"):
  * the file `state/p8-authorization-p8-validation-60-block-03-fe2a0c6e9052.json` (`199ec752…`);
  * block 3, stage V5, `max_attempts: 1`, authorized by Neura-Shadow, simulated flight only.
* **The launch:** detached, PID 879043, its own session with no terminal; log `~/uav_autonomy/logs/p8-block-03-fe2a0c6e9052.log`.
* **The runner's own checks passed:**
  * FC-1, and the host check (AC online, CPU mean 2.8 %, 676.6 GB free);
  * then it consumed the authorization and took the live lock.
* **Duration:** 63.2 min for the 20 flights.

**The records:**

| Record | File (`state/`) | SHA-256 |
|---|---|---|
| Result | `p8-p8-validation-60-block-03-fe2a0c6e9052….result.json` | `43bf1db8c7e61dbf9beaefccae715b7c29a173d763dbe34d775cad27d387cbb8` |
| Ledger (20 lines, fsynced) | `p8-p8-validation-60-block-03-fe2a0c6e9052….ledger.jsonl` | `f40d23afe93ac4dd953348535cc792e912749b234e01d137463ad6e6df187035` |
| Consumption marker | `p8-p8-validation-60-block-03-fe2a0c6e9052….consumed.json` | `48faa0fe5a3fe6c1da17b66e71862e481dc38d9a872e453a38f65d231a1706b7` |

### 3.6 The result

**Not stopped:**
* stop reason none, no exception, not interrupted, not recovered from the ledger;
* 20 of 20 flown, all **valid**;
* no replacement (the cap was 2), no missing pair, nothing unflown;
* accounting complete (20 `flown_valid`).

**At the end:**
* the candidate unchanged and no drift (FC-7);
* the live lock released, and no simulator process left;
* `engineering_credit: true`, `dry_run: false`, `automatic_retry: false`.

**Safety (all 20 flights):**
* 0 safety events, 0 safety-unverified flights, 0 mission failures, FC-3 not applied, 0 collisions;
* every flight landed and disarmed, final state `COMPLETE`;
* the minimum truth clearance was 0.362 m (slot 27, P8-P5R). The limit is ≥ 0.20 m.
* **H10 passed on all 20 flights,** with 0 actions to check (§3.7).

### 3.7 Outcomes and recovery (descriptive only)

| Slot | Layout | P7-E slot | P7-E (P4, P5) | P8-P4R | P8-P5R |
|---:|---|---:|---|---|---|
| 21 | `p7c-4000351` | 1 | goal, goal | goal (41.7 s) | goal (41.8 s) |
| 22 | `p7c-4000357` | 11 | goal, goal | goal (41.3 s) | goal (41.1 s) |
| 23 | `p7c-4000374` | 13 | goal, goal | goal (44.0 s) | goal (54.8 s) |
| 24 | `p7c-4000354` | 20 | goal, goal | goal (41.4 s) | goal (41.3 s) |
| 25 | `p7c-4000359` | 17 | goal, goal | goal (41.3 s) | goal (41.3 s) |
| 26 | `p7c-4000394` | 5 | goal, goal | goal (41.0 s) | goal (41.3 s) |
| 27 | `p7c-4000373` | 8 | goal, goal | goal (41.5 s) | goal (41.4 s) |
| 28 | `p7c-4000398` | 4 | goal, goal | goal (48.3 s) | goal (43.7 s) |
| 29 | `p7c-4000349` | 10 | goal, goal | goal (44.5 s) | goal (43.4 s) |
| 30 | `p7c-4000367` | 2 | goal, goal | goal (48.0 s) | goal (43.1 s) |

* **Goal success:** P8-P4R 10 of 10, P8-P5R 10 of 10. The times are simulated time to the goal.
* **R-P8-9 (regressions): none.** Every V5 flight reached the goal, as its P7-E baseline did.
* **Recovery never triggered:**
  * no flight entered HOVER, so there was no recovery episode and no recovery action;
  * every recovery ledger holds only its header.
  * So on V5 the recovery layer changed no decision of the accepted stacks.
* These counts are descriptive only (P8-C1).

### 3.8 Evaluations, disk and what comes next

* **Evaluations:** the accepted P4 and P5 evaluations passed on all 20 flights. There were 0 integrity events, and
  the D-M3-1 reconciliation finds 0 mismatches on the 10 P8-P5R flights.
* **Disk:** 26.59 GB for the 20 flights (1.18–1.52 GB each). WSL free space went from 676.6 to 649.9 GB.
* **Retention:** every flight keeps its complete raw evidence (D4-A).
* **Next:** the P8 report and its acceptance (contract §13 item 8). §4 summarizes the three blocks.

## 4. All three blocks of `p8-validation-60` (descriptive; the report computes the criteria)

| Block | Stage | Flown, valid | Stopped | Goal P8-P4R | Goal P8-P5R | Safety events | Min clearance (m) | Recovery actions | H10 |
|---:|---|---|---|---:|---:|---:|---:|---:|---|
| 1 | V3 | 20, 20 | no | 4 / 10 | 4 / 10 | 0 | 0.344 | 8,680 | passed |
| 2 | V4 | 20, 20 | no | 4 / 10 | 4 / 10 | 0 | 0.342 | 5,967 | passed |
| 3 | V5 | 20, 20 | no | 10 / 10 | 10 / 10 | 0 | 0.362 | 0 | passed |
| | **All** | **60, 60** | **none** | **18 / 30** | **18 / 30** | **0** | **0.342** | **14,647** | **passed** |

* **The declared 60 flights are complete:** 3 blocks, none stopped, every flight accounted for, no replacement.
* **Across the 60 flights:**
  * 0 collisions, 0 safety-unverified flights;
  * 176 recovery translations, each reproduced by H10;
  * 3 FC-4 refusals.
* **Findings to carry into the report:**
  * F-DR1 / D-M3-1: 14 `decision_mismatch` items, all reconciled exactly;
  * F-B2-1: slot 14 P8-P5R, a P2 fail-closed landing (RR1, RR-P8-5);
  * 2 `forced_kill_after_grace` items.
* **Not decided here:** whether engineering validation passed. That needs H1–H11 and the reported criteria R-P8-1 to
  R-P8-10, computed once by `p8_report.py`, and the report's own authorization and acceptance.
