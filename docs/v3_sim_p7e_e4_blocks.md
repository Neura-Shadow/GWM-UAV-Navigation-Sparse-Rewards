# v3-SIM P7-E E4 — the engineering blocks (scope `engineering-30`)

**Candidate:** `state/p7e-candidate-2b6ee5d.json`, manifest
`38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be` (real; production Policy A `26595bba…`).
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
