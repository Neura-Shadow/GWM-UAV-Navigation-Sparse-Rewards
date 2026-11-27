# v3-SIM P7-E E4 — the engineering blocks (scope `engineering-30`)

**Status: COMPLETE (2026-11-27).** All three blocks were flown on the second real candidate; the report was accepted
in E5 (2026-11-28).

**Candidates:**
* **first:** `state/p7e-candidate-2b6ee5d.json`, manifest
  `38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be` (production Policy A `26595bba…`).
  * Its block 1 stopped after 3 flights and was recovered into a recorded stop.
  * The candidate is retired, and its records are protected.
* **second:** `state/p7e-candidate-2b6ee5d-r2.json`, manifest
  `e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608` (production Policy A r2 `56c264e0…`). It
  carries the same code, and it flew blocks 1–3 (60 of 60 valid).
**Rules:** contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §4, §8 and §13 item 8.
* **Authorization:** each block needs its own, written through `p7e_manifest.py authorize`, with `max_attempts` 1 and
  a permanent consumption marker.
* **Replacements:** an infrastructure-invalid flight is re-flown once at the end of the block, at most 2 per block.
* **Stops:** a safety event or a safety-unverified flight (including FC-3), manifest drift, a simulator outliving its
  flight, a failed process check (FC-2), missing evidence, or any exception (FC-1). Nothing is retried
  automatically. After a stop, the operator decides.
* **Order:** block 2 or 3 starts only if the previous block's result is unstopped and accounts for every planned
  flight (FC-1).
* **No credit and no claim of superiority (§7).** The blocks feed only the E5 engineering report.

## Block 1

**Status: INTERRUPTED (2026-11-22), with no result written. It is fail-closed: nothing can proceed on this candidate
until the operator decides.** See "Block 1: what happened" below.
* **Resolved (2026-11-27):**
  * the operator authorized the recovery, which wrote stop record `5f7c2326…`;
  * the operator then chose to re-fly with the same code on a second candidate (sections below);
  * this candidate flies nothing more.

**The plan:** 10 pairs, 20 flights, each pair back to back in the listed order, 5 of them P4 first. It takes about
65 min and about 26 GB.

| Slot | Layout | Pool index | Family | Boxes | Order |
|---:|---|---:|---|---:|---|
| 1 | `p7c-4000351` | 201 | pillars | 2 | P4 → P5 |
| 2 | `p7c-4000367` | 214 | wall with gap | 2 | P4 → P5 |
| 3 | `p7c-4000355` | 204 | wall with gap | 2 | P5 → P4 |
| 4 | `p7c-4000398` | 229 | partial wall | 1 | P5 → P4 |
| 5 | `p7c-4000394` | 227 | pillars | 1 | P5 → P4 |
| 6 | `p7c-4000396` | 228 | mixed | 3 | P4 → P5 |
| 7 | `p7c-4000381` | 222 | wall with gap | 2 | P5 → P4 |
| 8 | `p7c-4000373` | 216 | mixed | 2 | P4 → P5 |
| 9 | `p7c-4000377` | 219 | wall with gap | 2 | P5 → P4 |
| 10 | `p7c-4000349` | 200 | mixed | 3 | P4 → P5 |

**Read-only pre-check (2026-11-22):**
* the candidate verifies to `38e23bad…`, with no drift;
* the frozen MC-trace build is selected;
* no result, ledger or consumption marker exists for block 1, and no live lock is held;
* the host check passes: CPU 4, 1, 2, 1, 1 % (mean 1.8 %), 774 GB free.

**On authorization:**
1. **Write the authorization:**

   ```
   p7e_manifest.py authorize state/p7e-candidate-2b6ee5d.json --block 1 --operator Neura-Shadow \
     --confirm "authorize P7-E engineering-30 block 1 38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be" \
     --output state/p7e-authorization-engineering-30-block-01-38e23bad4bd4.json
   ```

2. **Run the block,** in the background:

   ```
   cd /tmp && export GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 GWM_ALLOW_PX4_LAUNCH=1 \
     GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1
   run_p7e_block.py --scope engineering-30 --block 1 --manifest state/p7e-candidate-2b6ee5d.json \
     --authorization <the authorization>
   ```

3. **What the sequencer does:**
   * It runs the host check again, and flies nothing if it fails. In that case the authorization is not consumed.
   * Otherwise it consumes the marker, takes the live lock, and flies the 20 flights in order.
   * It writes `state/p7e-engineering-30-block-01-38e23bad….result.json` and its ledger, exactly once.

**While the block runs:**
* No P7-E, P7 or test file may be edited: drift stops the block.
* Avoid heavy Windows workloads during the block.

## Block 1: what happened (read-only investigation, 2026-11-27)

**Authorized:** "Authorize (Recommended)" on 2026-11-22. The authorization is
`state/p7e-authorization-engineering-30-block-01-38e23bad4bd4.json` (`71e81930…`). It was consumed at 16:00:45 (all
times +08:00), and the live lock was taken.

**Flown:**

| # | Flight | Run | Outcome |
|---:|---|---|---|
| 1 | slot 1, P4 | `20261122T080108Z-p7e-p4-p7c-4000351-ce7909e5` | evaluator passed; valid **goal success**; FC-3 no change; in the ledger |
| 2 | slot 1, P5 | `20261122T080429Z-p7e-p5-p7c-4000351-884f1857` | valid **failure**, no stop reason; in the ledger (details below) |
| 3 | slot 2, P4 | `20261122T080735Z-p7e-p4-p7c-4000367-65260acf` | runtime finished cleanly at 16:10:22 (COMPLETE, finalized); **never evaluated, classified or put in the ledger** |

**Flight 2.** The P2 controller aborted on `stale_action` during transit and landed fail-closed (RECOVERY).
* Clearance 0.849 m, 0 collisions.
* The accepted P2 integrity check raised (`estimator_reference_reset:final_alignment_lost`). Under D15, the ULog facts
  held: own commands 176/400/21, landed and disarmed, no failsafe, Policy A checked.
* The frozen classifier filed everything as integrity events, with no stop:
  * `runtime_status`, `cleanup`, `performance:compute_max`, `rgb_absence`, `p2:timing`, `p2_integrity_error`,
    `mc_causal_yaw`.
* It belongs with the risk register's RR1 (a P2 fail-closed RECOVERY after a slowdown). Under the rules, continuing
  was correct.

**How it stopped:**
* **The sequencer died without an exception.** It was PID 457251, run as a background child of the Claude Code
  session. It wrote no result and released no lock.
* **The likely cause** is a hangup signal (SIGHUP) when that session ended. It arrived some time between 16:07:35
  and 16:10:22.
  * A signal is not a Python exception, so the FC-1 handler did not run.
  * Flight 3 ran as PID 1 of its own namespace. A namespace's PID 1 ignores that signal from outside, so the flight
    finished alone.
  * The log was in `/tmp`, so the exact moment is lost.
* **The WSL VM** stayed up until 16:21:28, then powered off. It was not the cause.

**State now:**
* marker consumed;
* ledger with 2 lines (`…block-01-38e23bad….ledger.jsonl`);
* **no result file**;
* a **stale live lock** `state/p7e-live-run.lock`, naming the dead PID 457251;
* no simulator process.

**Effect (fail-closed):**
* Block 2 is refused, because there is no unstopped, complete block 1 result.
* Block 1 cannot run again on this candidate: the ledger exists and the marker is consumed.
* The stale lock blocks every P7 and P7-E sequence and freeze.

**Gap found.** FC-1 turns every exception into a stop. Termination by a signal (SIGHUP, SIGTERM, SIGKILL) or a power
loss bypasses it. It still ends fail-closed, as above, but it leaves no record.

## Block 1: recovery (operator "Authorize recovery", 2026-11-27)

**Tool:** [`p7e_block_recovery.py`](../diagnostics/p7e/p7e_block_recovery.py). It lives in `diagnostics/`, outside the
P7-E candidate, so the candidate does not drift. Its 13 offline tests are in
[`test_p7e_block_recovery.py`](../diagnostics/p7e/test_p7e_block_recovery.py), also outside `tests/`, and all pass:
the stop record, a stop reason carried over from the orphan, and nine refusal cases that change nothing.

**What it did,** after a copy test. The full procedure first ran on a scratch root with copied state and a symlink
view of flight 3. The real run and the real state were unchanged by that test.
1. **Checks passed:**
   * no result existed; the marker matched the authorization;
   * the lock's PID 457251 is not a live sequencer;
   * no simulator process was running;
   * the ledger is the plan's prefix;
   * exactly one orphaned run, the next planned flight, was not yet evaluated.
2. **Flight 3** (slot 2, P4, `20261122T080735Z-p7e-p4-p7c-4000367-65260acf`) was verified as the sequencer would have
   done it. The unchanged `p4_evaluate` **passed** with no failure item. The frozen classifier gave a valid **goal
   success**, clearance 0.984 m, no event in any class. FC-3 changed nothing. These outputs are identical to the copy
   test's.
3. **The stop record** was written: `state/p7e-engineering-30-block-01-38e23bad….result.json` (`5f7c2326…`).
   * stopped, stop reason `sequencer_terminated_externally`;
   * flights: the 2 ledger flights; the ledger is unchanged and its hash matches;
   * interrupted: flight 3, with its record; not flown: 17;
   * accounting complete: 2 `flown_valid`, 1 `interrupted`, 17 `not_flown`;
   * marked `reconstructed`.
4. **The receipt:** `…block-01-38e23bad….recovery.json` (`0c7c3c77…`).
5. **The stale lock was removed.** Its content and hash are kept in the receipt.

**State after the recovery:**
* no live lock;
* the P7-E candidate `38e23bad…` and the P7 candidate `0799c37e…` both verify, with no drift;
* FC-1 refuses block 2 on this candidate, because block 1 is stopped.

**Safety of all three flown flights is verified:** 0 collisions, minimum clearance 0.849 m, no safety event, no
safety-unverified flight.

**Open: the operator's decision on how to continue.** Block 1 cannot resume on `38e23bad…`.

## Re-fly on a second real candidate (operator "Re-fly, same code", 2026-11-27)

**Decision.** The 60 flights are re-flown from block 1 on a new real candidate, with the same accepted E2 code
(`2b6ee5d`, 16 files, canonical `003b4d76…`). No code changes.
* **The stopped attempt on `38e23bad…`** stays: its 3 flights, stop record and receipt are protected evidence. E5
  reports it as a stopped attempt, and it does not count toward the 60.
* **Re-flown layouts.** Those 3 flights' layouts (slot 1, and slot 2 with P4) are flown again. E5 states that they
  were seen once before.

**PREPARED (2026-11-27): three items, each needing its own approval.**
1. **Production Policy A for the second real candidate:** `state/p7e-policy-A-approval-2b6ee5d-r2.json`.
   * Policy A `bf1ef9b1…`; `development_only: false`; it binds the same 16 files.
   * Scope: the 3 blocks of the second real P7-E candidate, each separately authorized, in simulated flight only.
     It transfers to no other candidate: not to `38e23bad…`, and not to any development candidate.
2. **The freeze:** `state/p7e-candidate-2b6ee5d-r2.json`, with the absolute approval path, then verified from `/tmp`.
3. **Block 1 on the new candidate:** the authorization through `p7e_manifest.py authorize` and its phrase, then the
   run, **launched detached**:

   ```
   mkdir -p ~/uav_autonomy/logs && cd /tmp && setsid nohup env GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 \
     GWM_ALLOW_PX4_LAUNCH=1 GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1 \
     /usr/bin/python3 …/run_p7e_block.py --scope engineering-30 --block 1 --manifest … --authorization … \
     > ~/uav_autonomy/logs/p7e-block-01-<sha12>.log 2>&1 < /dev/null &
   ```

   * **Why detached.** The run gets a new session with no controlling terminal, so the end of a Claude session cannot
     hang it up.
   * **Persistent log.** The log is kept in `~/uav_autonomy/logs`, not `/tmp`.
   * **Watching it.** A separate watcher only waits for the PID and reads the result. If the watcher dies, the block
     goes on.
   * **Keeping the host awake.** The app is asked to keep the computer awake during the block. AC sleep is already
     "never".
   * **Fallback.** Anything that still kills the sequencer (a hard kill or power loss) is fail-closed, and the
     recovery tool handles it.

**Read-only pre-check (2026-11-27):**
* the new candidate computes identically to `38e23bad…`: the same files, specs, P7 binding and baselines;
* development false;
* the MC-trace build is selected, and no live lock is held;
* the host check passes: CPU 0, 0, 1, 3, 3 %, 771 GB free;
* AC sleep is "never".

## Second real candidate `e9561f05…`: block 1 (COMPLETED 2026-11-27)

**Records:**
* production Policy A r2: `state/p7e-policy-A-approval-2b6ee5d-r2.json` (`56c264e0…`);
* the candidate: `state/p7e-candidate-2b6ee5d-r2.json`, manifest
  `e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608`;
* the block 1 authorization (`c85d471d…`);
* the result: `state/p7e-engineering-30-block-01-e9561f05….result.json` (`1c8672ab…`), and its ledger (`e7bf02ae…`).

**Run:**
* launched detached (PID 24681, its own session, no terminal); log `~/uav_autonomy/logs/p7e-block-01-e9561f05de28.log`;
* the host check passed (CPU 0 %);
* 61.6 min for the 20 flights.

**Result:**
* **not stopped**: 20 of 20 flown, all **valid**; no replacement, no missing pair, nothing unflown or interrupted;
* accounting complete (20 `flown_valid`);
* the candidate unchanged at the end; the live lock released.

**Safety (all 20 flights):**
* 0 safety events and 0 safety-unverified flights;
* minimum truth clearance 0.371 m (slot 7, P5). The limit is ≥ 0.20 m.

**Outcomes:**
* goal success: P4 6 of 10, P5 7 of 10;
* all 7 failures are the accepted P4 supervisor's `hover_timeout` abort, which hovers and then lands. Their accepted
  evaluations passed, and none has an event of any class.
* By pair:
  * both reached the goal: slots 1, 2, 4, 5, 8, 10;
  * both hovered out: slots 3, 6, 9;
  * slot 7: P5 reached the goal, P4 hovered out.
* **These counts are descriptive only.** This validation was not designed to compare P5 with P4 (contract §7).

**Integrity events:** one, in slot 10 with P5: `cleanup:gazebo:forced_kill_after_grace`. Gazebo needed a forced
stop at shutdown, so that flight's accepted P5 evaluation reports failed on this item. It is a valid goal success
with no safety item, and it is reported under R10.

**Block 2 pre-check (read-only):**
* the candidate verifies, with no drift;
* FC-1 allows block 2, because block 1 is unstopped and complete;
* no block 2 result, ledger or marker exists; no lock is held;
* the host check passes (743 GB free).

## Second real candidate `e9561f05…`: block 2 (COMPLETED 2026-11-27)

**Records:**
* the block 2 authorization: `state/p7e-authorization-engineering-30-block-02-e9561f05de28.json` (`33256d22…`);
* the result: `state/p7e-engineering-30-block-02-e9561f05….result.json` (`62c23933…`), and its ledger (`1723d124…`).

**Run:**
* launched detached (PID 140158, its own session, no terminal); log `~/uav_autonomy/logs/p7e-block-02-e9561f05de28.log`;
* the host check passed (743 GB free, AC online);
* 64.6 min for the 20 flights (slots 11–20).

**Result:**
* **not stopped**: 20 of 20 flown, all **valid**; no replacement, no missing pair, nothing unflown or interrupted; no
  exception, nothing recovered from the ledger;
* accounting complete (20 `flown_valid`);
* the candidate unchanged at the end, no drift; the live lock released; no simulator process left.

**Safety (all 20 flights):**
* 0 safety events, 0 safety-unverified flights, FC-3 not applied, 0 collisions;
* every flight's P7-E classification equals the frozen classifier's;
* minimum truth clearance 0.348 m (slot 15, P4). The limit is ≥ 0.20 m.

**Outcomes:**
* goal success: P4 5 of 10, P5 6 of 10;
* all 9 failures are the accepted P4 supervisor's `hover_timeout` abort (final state `COMPLETE`, outcome
  `supervisor_abort`). Their accepted evaluations passed, and none has an event of any class.
* By pair:
  * both reached the goal: slots 11, 13, 17, 19, 20;
  * both hovered out: slots 12, 14, 16, 18;
  * slot 15: P5 reached the goal, P4 hovered out.
* After two blocks: P4 11 of 20, P5 13 of 20. **These counts are descriptive only** (contract §7).

**Integrity events:** none in block 2.

**Block 3 pre-check (read-only):**
* the candidate verifies, with no drift; development false;
* FC-1 allows block 3, because block 2 is unstopped and complete and its ledger matches;
* no block 3 result, ledger or marker exists; no lock is held;
* the MC-trace build is selected;
* the host check passes: CPU mean 0.8 % (max 15 %), 714 GB free, AC online, no simulator process.

Block 3 plan (slots 21–30):

| Slot | Layout | Family | Order |
|---:|---|---|---|
| 21 | p7c-4000364 | pillars | p5→p4 |
| 22 | p7c-4000388 | partial_wall | p5→p4 |
| 23 | p7c-4000366 | mixed | p4→p5 |
| 24 | p7c-4000368 | pillars | p5→p4 |
| 25 | p7c-4000378 | mixed | p4→p5 |
| 26 | p7c-4000356 | wall_with_gap | p5→p4 |
| 27 | p7c-4000352 | pillars | p4→p5 |
| 28 | p7c-4000358 | wall_with_gap | p4→p5 |
| 29 | p7c-4000389 | pillars | p5→p4 |
| 30 | p7c-4000365 | pillars | p4→p5 |

Block 3 is the last block of `engineering-30`. It needs its own authorization:
"authorize P7-E engineering-30 block 3 e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608".

## Second real candidate `e9561f05…`: block 3 (COMPLETED 2026-11-27)

**Records:**
* the block 3 authorization: `state/p7e-authorization-engineering-30-block-03-e9561f05de28.json` (`87db8fe1…`);
* the result: `state/p7e-engineering-30-block-03-e9561f05….result.json` (`d4da3561…`), and its ledger (`7a6b3a79…`).

**Run:**
* launched detached (PID 262208, its own session, no terminal); log `~/uav_autonomy/logs/p7e-block-03-e9561f05de28.log`;
* the host check passed (714 GB free, CPU mean 2.0 %, AC online);
* 63.1 min for the 20 flights (slots 21–30).

**Result:**
* **not stopped**: 20 of 20 flown, all **valid**; no replacement, no missing pair, nothing unflown or interrupted; no
  exception, nothing recovered from the ledger;
* accounting complete (20 `flown_valid`);
* the candidate unchanged at the end, no drift; the live lock released; no simulator process left.

**Safety (all 20 flights):**
* 0 safety events, 0 safety-unverified flights, FC-3 not applied, 0 collisions;
* every flight's P7-E classification equals the frozen classifier's;
* minimum truth clearance 0.310 m (slot 29, P4). The limit is ≥ 0.20 m.

**Outcomes:**
* goal success: P4 9 of 10, P5 9 of 10;
* both failures are the accepted P4 supervisor's `hover_timeout` abort, in the same pair (slot 23). Their accepted
  evaluations passed, and neither has an event of any class.
* By pair:
  * both reached the goal: slots 21, 22, 24, 25, 26, 27, 28, 29, 30;
  * both hovered out: slot 23.
* **These counts are descriptive only** (contract §7).

**Integrity events:** one, in slot 27 with P4: `performance:compute_max`. One perception frame took 0.640 s of
compute (at sim time 76.6 s, 2 involuntary context switches); the accepted P4 limit is 0.090 s, and that flight's p99
was 0.045 s. So that flight's accepted P4 evaluation reports failed on this item alone (evaluator exit code 1). Its
safety section is clean: 0 collisions, clearance 0.469 m, no setpoint inside an inflated obstacle. It is a valid goal
success with no safety item, and it is reported under R10.

## `engineering-30` on `e9561f05…`: the batch (all three blocks COMPLETED)

| Block | Slots | Flown | Valid | Stopped | Replacements | Minutes | Min clearance (m) | Integrity events |
|---:|---|---:|---:|---|---:|---:|---:|---|
| 1 | 1–10 | 20 | 20 | no | 0 | 61.6 | 0.371 | s10 P5 `cleanup:gazebo:forced_kill_after_grace` |
| 2 | 11–20 | 20 | 20 | no | 0 | 64.6 | 0.348 | none |
| 3 | 21–30 | 20 | 20 | no | 0 | 63.1 | 0.310 | s27 P4 `performance:compute_max` |
| **all** | 1–30 | **60** | **60** | — | **0** | 189.3 | **0.310** | 2 |

* 0 safety events and 0 safety-unverified flights in all 60 flights; FC-1, FC-2 and FC-3 never stopped a block.
* Goal success: P4 20 of 30, P5 22 of 30. All 18 failures are `hover_timeout` aborts. **Descriptive only; no
  claim of superiority (contract §7).**
* Read-only check after block 3: the candidate verifies with no drift; FC-1 sees block 3 unstopped and complete; no
  lock is held; no simulator process remains.
* The stopped first attempt on `38e23bad…` (block 1, `sequencer_terminated_externally`) stays protected evidence,
  and the E5 report must describe it.

**E5 done (2026-11-28):** [E5 report](v3_sim_p7e_e5_report.md), "engineering validation passed", accepted by
the operator. **As planned:** E5, offline only. It re-classifies all 60 flights on read-only views (H9),
then writes the engineering report with H1–H9, R1–R10 and the risk register (`analysis/p7e_report.py`). After that
comes operator acceptance. Bound files stay unchanged until then.
