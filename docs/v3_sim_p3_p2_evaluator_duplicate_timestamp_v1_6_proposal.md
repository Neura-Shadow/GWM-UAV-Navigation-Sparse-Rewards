# P3 contract `p3-acceptance-v1.6` — proposal (P2 evaluator: duplicate ULog timestamps)

**Status: PROPOSAL ONLY — NOT APPROVED, NOT APPLIED.** Nothing was changed, built, frozen or
run for this proposal. P3 remains INCOMPLETE, qualification 0/3.

Basis: HEAD `d2285b4` and the retained v1.5 Gate C runs (acceptance report section AG).

v1.6 would change one comparison in the **accepted P2 evaluator**
(`validation/collect_p2_evidence.py`, function `verify`, lines 229–235). Everything else stays:
the 1e-6 m tolerance, the 100-pair minimum, every P2 and P3 limit, the P2 controller, mission
and configuration, sensors, renderer, shutdown profile, RGB scope, the v1.4 rules, the v1.5
recorder capacity, and gate sequencing.

## The defect (from Gate C qualification-2)

```python
# current, accepted P2 evaluator
u_by_t = {p["timestamp"]: p for p in ulog["vehicle_local_position"]}
matching = [p for p in streams["vehicle_local_position"] if p["timestamp"] in u_by_t]
require(len(matching) >= 100, "Insufficient overlapping ROS/ULog positions")
max_delta = max(abs(p[k]-u_by_t[p["timestamp"]][k]) for p in matching for k in ("x", "y", "z"))
require(max_delta < 1e-6, "ROS/ULog estimator mismatch")
```

The lookup assumes each ULog `vehicle_local_position` publication `timestamp` is unique. In
qualification-2, PX4 logged two different estimates with `timestamp` 15.680000 s (ULog rows
1356/1357, `timestamp_sample` 15.644000 and 15.680000 s). The dictionary keeps the **last** row.
ROS had received the **first**, which matches row 1356 bit-exactly (x, y, z and
`timestamp_sample`). Compared against row 1357, it differs by 3.95e-4 m, and the evaluator raised
`ROS/ULog estimator mismatch` although the estimator data agree.

Retained ULogs (read-only): a duplicate `timestamp` pair appears in 2 of the last 5 P3 flights
(Gate C qualification-1 at 71.516 s, where ROS did not receive that sample, and qualification-2)
and in none of the three Gate B flights. ROS carries about 39 % of ULog samples, so a false
failure needs both a duplicate and the earlier row to be the one bridged. Across three
consecutive qualification flights, this is a material risk to an otherwise passing Gate C.

## Proposed change

Key the ULog lookup by the full sample identity `(timestamp, timestamp_sample)`. Both fields are
present in the ROS message and the ULog topic, and `timestamp_sample` is what distinguishes the
two estimates. Fail closed if that key is itself ambiguous.

```python
# Exact timestamp/float comparison proves shared PX4 epoch independently of controller receipts.
# PX4 can log two estimates with one publication timestamp; timestamp_sample identifies the sample.
u_by_t, ambiguous = {}, set()
for p in ulog["vehicle_local_position"]:
    key = (p["timestamp"], p["timestamp_sample"])
    if key in u_by_t and any(u_by_t[key][k] != p[k] for k in ("x", "y", "z")):
        ambiguous.add(key)
    u_by_t[key] = p
key = lambda p: (p["timestamp"], p["timestamp_sample"])
matching = [p for p in streams["vehicle_local_position"] if key(p) in u_by_t]
require(not any(key(p) in ambiguous for p in matching), "Ambiguous ULog position sample")
require(len(matching) >= 100, "Insufficient overlapping ROS/ULog positions")
max_delta = max(abs(p[k]-u_by_t[key(p)][k]) for p in matching for k in ("x", "y", "z"))
require(max_delta < 1e-6, "ROS/ULog estimator mismatch")
report["estimator_correlation"] = {"exact_timestamp_pairs": len(matching), "max_position_difference_m": max_delta,
                                   "duplicate_ulog_timestamps": <count of timestamps logged more than once>}
```

* Tolerance `1e-6` and minimum `100` are unchanged. The check gets **stricter**: a ROS sample
  must now match the ULog row with the same `timestamp_sample`, not just any row with the same
  publication time.
* No other line of `collect_p2_evidence.py`, `yaw_evidence.py` or any evaluator changes. This is
  the only timestamp-keyed lookup of this kind in the P2/P3 evaluators (checked).
* `configs/p3_validation.yaml`: `contract: p3-acceptance-v1.6` and a `contract_revision`
  sentence; no other key changes.
* The alternative named in the request, matching each ROS sample against all ULog rows sharing
  its `timestamp` and taking the closest, also fixes the collision. It is not recommended: it
  accepts a sample that matches any estimate published in the same microsecond, while the keyed
  form requires the exact sample.

**Read-only check on retained runs (not credited, nothing written).** The current and proposed
comparisons were applied in memory to three retained v1.5 flights:

| Run | Current: pairs / max Δ | Proposed: pairs / max Δ |
|---|---|---|
| Gate C qualification-2 `…51e74376` | 5,011 / 3.95e-4 m **FAIL** | 5,011 / 0 PASS (0 ambiguous keys) |
| Gate C qualification-1 `…5f89e686` | 4,971 / 0 PASS | 4,971 / 0 PASS |
| Gate B `…6e322ae9` | 5,038 / 0 PASS | 5,038 / 0 PASS |

## Tests (offline, in `tests/test_px4_gazebo_reference.py` style with synthetic streams)

1. A duplicate ULog `timestamp` where ROS carries the first row: passes, delta 0,
   `duplicate_ulog_timestamps` 1 (the qualification-2 pattern).
2. The same, with ROS carrying the second row: passes.
3. ROS equal to neither row at that `(timestamp, timestamp_sample)`: `ROS/ULog estimator
   mismatch`.
4. A ROS sample whose `timestamp_sample` is absent from ULog is not paired; with fewer than 100
   pairs left: `Insufficient overlapping ROS/ULog positions`.
5. Two ULog rows with the same `(timestamp, timestamp_sample)` but different x/y/z, carried by
   ROS: `Ambiguous ULog position sample` (fail closed).
6. No duplicates (all accepted P2 and P3 data so far): identical `exact_timestamp_pairs` and
   `max_position_difference_m` to the current evaluator.
7. Tolerance unchanged: a 1.1e-6 m difference on a correctly keyed pair still fails.
8. Contract identity is v1.6; every numeric limit and the P2 configuration are identical.

## Impact on accepted P2 evidence

* **Pinned identity.** The accepted P2-R3 batch `20260912T035329Z-p2-repeat-84bb288d` lists
  `validation/collect_p2_evidence.py` at `a6acfd2d…` in `repeated_batch/frozen_inputs/files` of
  `docs/evidence/v3_sim_p2_summary.json`. That is the current file, and the change gives it a
  new hash. The P2 acceptance record itself is not changed: it stays a record of evaluation with
  `a6acfd2d…`. `run_p2_repeated.py` checks the evaluator digest against its frozen inputs, so a
  re-run of **that frozen P2 batch** would refuse the new file. No P2 re-run is proposed.
* **Verdicts cannot flip from pass to fail on accepted data.** Accepted P2 runs passed the current
  check, so every ROS sample there matched the single ULog row kept for its `timestamp`. The keyed
  lookup pairs it with the row having its own `timestamp_sample`, which is the same row unless a
  duplicate collided, and a collision would already have failed. The pair counts and maximum
  difference stay the same unless an accepted run contains a duplicate `(timestamp,
  timestamp_sample)` with different values, which test 5 would expose.
* **Proposed confirmation after approval (offline, read-only, writes nothing to the runs):**
  evaluate the accepted P2-R3 nominal smoke `20260912T034917Z-p2-flight-c74f35ba` and the 20
  batch trials with both evaluator versions in memory. Require every report field to be identical
  except the new `duplicate_ulog_timestamps` count, and record the per-run duplicate counts. The
  run directories are still on disk.
* **P2 scope.** This is a change to accepted P2 tooling, which the standing instructions reserve
  for the operator. It touches no P2 controller code, mission, configuration or threshold, and no
  P2 run is re-labelled.

## Candidate and gate impact

`validation/` is part of the candidate source hash, so the sequence after approval is: apply,
regress (Linux and Windows), run the offline confirmation above, commit, Policy A re-approval for
the new commit (policy content unchanged, `bf1ef9b1…`), freeze, then Gates A, B and C under
separate authorizations. The failed v1.5 Gate C result stays failed and is not re-evaluated.
Qualification-1's pass does not carry over.

## What v1.6 does not change or claim

* No threshold, freshness, sensor, renderer, shutdown, MC, cleanup, RGB, recorder or
  native-command rule changes.
* The secondary `mc_causal_yaw` failure in qualification-2 (`KeyError: 'handover_start_sim_s'`)
  follows from the missing P2 report and is not addressed separately. With a P2 report present,
  the MC evaluator runs unchanged.
* The host-slowdown `pending_newer_state` risk noted under v1.5 remains open.
