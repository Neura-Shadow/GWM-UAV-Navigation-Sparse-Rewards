# v3-SIM P7 M4 — parity dry run (S2 through the P7 path)

**Status: ACCEPTED (2026-11-21); M4 COMPLETE.** This is M4 of the [P7 plan](v3_sim_p7_plan.md) (§7). It was P7's
first simulator launch.
* **Authorized:** by the operator on 2026-11-21 ("Authorize M4").
* **Accepted:** by the operator on 2026-11-21 ("Accept M4").

**What it was:**
* a development candidate;
* simulated flight only (SITL, never real hardware);
* no credit, no confirmatory data, no retry.

**Exit criterion: met.** The P7 path reproduces the accepted S2 outcome with both controllers, within run-to-run
variation (§3).

## 1. Approvals and candidate

| Item | Value |
|---|---|
| Policy A (development only) | `state/p7-policy-A-approval-m4-development.json`. Approved by the operator in chat: "Approve for M4 (Recommended)". It covers the same native-command policy as P3–P6 (`bf1ef9b1…`), and only P7 development candidates of commit b977739 for this dry run. |
| Development candidate | `state/p7-candidate-dev-b977739.json`, SHA-256 `ac98356418e153c4bf66776bf9a9d91cb487e4c31ffb78b164be7efece440120` |
| What the candidate binds | P3–P6 (`22292775…` through the accepted P6 verification); the approved specs (`d742b7af…`, `ef74c71d…`, `c9c7a6f3…`) and host check (`9f78af75…`); 19 P7 source and test files; the owner copy |
| Dry-run approval | `state/p7-dry-run-approval-ac98356418e1-m4.json`, written through `p7_manifest.py approve-dry-run` and its confirmation phrase: items [S2, p4] and [S2, p5], at most 2 live runs, simulated flight only, no credit |

## 2. Execution (`run_p7_block.py --scope DRY`)

**Host check (read-only):** overlay `ded574b5`, AC Online, CPU 1, 1, 2, 0, 3 %, no simulator, lock free, disk floor
met. **Passed.**

**Flights.** In schedule order, each in a fresh private namespace through `run_p7.sh`, then the unchanged evaluator,
then the M3 classifier:

| Flight | Run | Runtime | Accepted evaluator | Classifier |
|---|---|---|---|---|
| S2, P4 | `20261121T130806Z-p7dry-p4-S2-d94cbdf7` | exit 0 | `p4_evaluate` **passed** | valid, goal success, no events |
| S2, P5 | `20261121T131125Z-p7dry-p5-S2-97a1db5f` | exit 0 | `p5_evaluate` **passed** (including the bit-exact policy replay) | valid, goal success, no events |

**Result:** `state/p7-dryrun-ac98356418e1-20261121T130734Z.result.json` (`f7f5db32…`) and its ledger (`78dba503…`).
* Not stopped, with no replacement, no missing pair and nothing unflown.
* The candidate was unchanged at the end.
* 392 s for both flights, including evaluation and classification.

**Afterwards:** the P3–P6 manifests and the P7 development manifest verify, and no simulator process remains.

**What this shows for the first time.** Offline tests could not show these two things:
* the P7 runner subclasses launch the accepted stacks inside a real namespace;
* the unchanged P4 and P5 evaluators accept a real P7 run. So the summary carries every field they read, and the QGC
  ownership receipt names the P7 owner copy correctly.

## 3. Parity with the accepted S2 flights

| Flight | World | Accepted label | Supervisor lineage | Final goal distance | Min. truth clearance | Time to goal | Path | Runner time | Disk |
|---|---|---|---|---:|---:|---:|---:|---:|---:|
| **P7 path, P4** | `65dc596c` | goal_reached_with_detour | OBSTACLE_DETECTED, AVOIDING, RESUME, NORMAL | 0.054 m | 0.375 m | 44.80 s | 9.856 m | 174 s | 1.23 GB |
| R1 P4 (Gate C) | `65dc596c` | goal_reached_with_detour | same | 0.023 m | 0.446 m | 44.19 s | 9.840 m | 163 s | 1.23 GB |
| R2 P4 (Gate B) | `65dc596c` | goal_reached_with_detour | same | 0.067 m | 0.409 m | 47.58 s | 10.116 m | 167 s | 1.27 GB |
| **P7 path, P5** | `65dc596c` | goal_reached_with_detour | same | 0.024 m | 0.417 m | 42.65 s | 9.764 m | 175 s | 1.19 GB |
| R3 P5 (Gate C) | `65dc596c` | goal_reached_with_detour | same | 0.026 m | 0.374 m | 42.75 s | 9.724 m | 171 s | 1.20 GB |
| R4 P5 (Gate B) | `65dc596c` | goal_reached_with_detour | same | 0.029 m | 0.420 m | 42.92 s | 9.574 m | 177 s | 1.20 GB |

**Reading.** S2 uses the accepted specification on the P7 path, so the accepted outcome check ran as in the gates.
* The world is byte-identical (C6), the accepted outcome label and supervisor lineage are identical, and both flights
  reach the goal well within 0.30 m.
* The metrics vary as much as accepted S2 flights vary among themselves. Plan §2.5 measured, over five P5-stack S2
  flights, time to goal 42.72–42.92 s, clearance 0.356–0.453 m and path 9.57–9.92 m.

**Two values sit at the edge, reported as found:**
* The P4 dry run's clearance (0.375 m) is below both P4 gate flights (0.409, 0.446 m). It is inside the S2 spread
  above and far above the 0.20 m limit.
* The P5 dry run's time to goal (42.65 s) is 0.07 s faster than the earlier range.

Neither is a change of behaviour. This is the run-to-run variation the pilot is designed to measure.

**Cost per flight.**
* Runner time is 174–175 s; the full cycle, including the evaluator and the classifier, is about 196 s.
* Disk is 1.19–1.23 GB, inside the 1.3 GB host-check plan.
* 440 flights therefore take about 24 h, in line with plan §4.5.

## 4. Next

**Operator review of M4:** done; accepted on 2026-11-21 ("Accept M4").

**M5, the pilot, needs its own authorization**, plus these steps from the accepted M2 design choices:
1. A **production Policy A approval** for the real P7 candidate (D12; development approvals never transfer).
2. The **real freeze** of the P7 candidate (`development: false`) before the pilot, so the pilot and the confirmatory
   blocks run on one manifest.
3. The **pilot authorization**: one block of 20 pairs (40 flights, about 2.2 h, about 48 GB) under the frozen pilot
   schedule.
4. Then the pilot rule, which selects the 200- or 250-pair schedule.
