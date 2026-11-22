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

**Status: PREPARED (2026-11-22), awaiting the operator's authorization. Nothing written or flown.**

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
