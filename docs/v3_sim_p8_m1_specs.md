# v3-SIM P8-M1: the specifications (contract `p8-navigation-recovery-v1` §13 item 2)

**Status: APPROVED BY HASH (operator, 2026-11-28, "Approve specs"), so P8-M1 is COMPLETE.**
* They were generated and tested under "Authorize the specs" (commit `74be8a9`).
* The four specs of §1 are now frozen. Any change to a spec file or to its generator would be a new specification,
  needing a new approval.
* The P8 candidate (P8-M2 and later) must bind exactly these canonical hashes.
* Offline only: no simulator, no candidate, no flight, no change to any P2–P7-E file. All seven manifests still verify
  (P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775`, P7 `0799c37e`, P7-E `38e23bad` and `e9561f05`).
* Approving the specs authorizes nothing to run. Next, separately, comes P8-M2 (code and tests, with the offline
  stages V0–V2; contract §13 item 3).

## 1. What is approved by hash

| Spec | File | Canonical SHA-256 |
|---|---|---|
| Layouts (V3, V4, V5) | `simulation/px4_gazebo_p8/configs/p8_layouts.json` | `62ae950cbcce9981b013992ed7a8d22e1bcbc4436852745e249172de25b40e85` |
| Schedule | `simulation/px4_gazebo_p8/configs/p8_schedule.json` | `df90cf7ba1a172a100cad1d5f72cf864088190331ecf11f2af4263f5f8f3da93` |
| Recovery parameters | `simulation/px4_gazebo_p8/configs/p8_recovery.json` | `dee90d51098210daf714fe0b875b0e98040195362095800dc48836f78371333b` |
| Report plan | `simulation/px4_gazebo_p8/configs/p8_report_plan.yaml` | `a313a034b2c8e481c0a1de651dc9ebc530bdf86736c4577216a281db429c2b14` |

The hashes are canonical (sorted keys, compact JSON), so they do not depend on line endings.

**Code:**
* [`p8_specs.py`](../simulation/px4_gazebo_p8/scripts/p8_specs.py): `generate` and `verify`. The `verify` run
  regenerates all three generated files, V4's P5 exclusion and held-out screen included, and reproduced them exactly
  (22 s).
* [`p8_geometry.py`](../simulation/px4_gazebo_p8/scripts/p8_geometry.py): the clearance-capacity and goal-clearance
  measure.
* [`test_px4_gazebo_p8_specs.py`](../tests/test_px4_gazebo_p8_specs.py): 11 tests, all passed (29 s). No
  manifest's test glob matches the file (`p6_`, `p7_` and `p7e_` only).

**Reused unchanged, by import or as data:**
* the P5 generator and its filters;
* the P7 world path, box naming and random stream (`p7_specs`);
* the P4 avoidance and P2 control configurations.

**Inputs checked by hash:**
* the P7 layouts (`d742b7af…`) and the P7-E schedule (`9b3deacb…`);
* the three accepted P7-E block results (`1c8672ab…`, `62c23933…`, `d4da3561…`), and through them every flight
  record.

## 2. Layouts

**V3: the 10 P7-E failure layouts** (block 1). Slots 3, 6, 7, 9, 12, 14, 15, 16, 18 and 23. The generator checks
them against the accepted batch: this is the contract's list exactly.
* The frozen P7 records are used unchanged: boxes, seed, family and world hash.
* The worlds regenerate to their frozen hashes through the reused P7 path.

**V4: 10 held-out layouts** (block 2), selected by the rule of contract §6:
* **Seeds:** in seed order from 6,000,000, disjoint from every existing set:
  * P5 train, validation and held-out (1M, 2M, 3M);
  * P7 confirmatory (4M) and pilot (5M).
* **The screen:** 66 seeds tried.
  * 28 rejected by the P5 generator's filters, or as matches with P5 or P7 layouts:
    `goal_unreachable_inflated` 8, `start_or_goal_disc` 10, `p5_split_match` 13, `p7_layout_match` 7. The 1,650 P5
    split layouts were regenerated and checked against the P5 records, as P7 did.
  * 18 admitted but matching neither type.
  * The first 5 of each type were taken.

| Layout | Type | Family | Boxes | Clearance capacity (m) | Goal clearance (m) | World SHA-256 |
|---|---|---|---:|---:|---:|---|
| `p8h-6000014` | F1 | partial_wall | 1 | 1.799 | 2.18 | `4fd04924…` |
| `p8h-6000019` | F2 | mixed | 3 | 1.143 | 1.99 | `c0431d68…` |
| `p8h-6000023` | F1 | pillars | 3 | 1.584 | 1.96 | `1ea49050…` |
| `p8h-6000027` | F2 | wall_with_gap | 2 | 1.242 | 2.97 | `6e592478…` |
| `p8h-6000033` | F1 | pillars | 1 | 1.835 | 1.76 | `2c145626…` |
| `p8h-6000038` | F1 | mixed | 3 | 1.548 | 2.07 | `8e15cd9e…` |
| `p8h-6000039` | F1 | mixed | 3 | 1.386 | 2.23 | `2fa51718…` |
| `p8h-6000050` | F2 | mixed | 3 | 1.341 | 2.82 | `5e055c04…` |
| `p8h-6000057` | F2 | wall_with_gap | 2 | 1.296 | 4.59 | `b63e0f4d…` |
| `p8h-6000065` | F2 | partial_wall | 1 | 1.341 | 5.02 | `e0ee2e16…` |

* **The type rule.**
  * **F2-type:** clearance capacity in [1.10, 1.35] m.
  * **F1-type:** capacity above 1.35 m and the nearest box within 2.3 m of the goal.
  * Both bounds come from the P7-E evidence: the F2 failures had capacity 1.14–1.30 m, and the F1 failures had goal
    clearance 1.56–2.23 m. Neither bound is a safety limit.
* **How the rule classifies the 30 P7-E layouts** (in sample, so descriptive only; V4 is the out-of-sample check):

  | Type | Layouts | Failed flights |
  |---|---:|---:|
  | F2 | 9 | 11 of 18 |
  | F1 | 6 | 7 of 12 |
  | neither | 15 | 0 of 30 |

  All 10 V3 layouts fall into a type: 6 F2 and 4 F1.
* **Box names** follow the P7 rule (`p7_box_<k>`), so the reused P7 world path generates the worlds unchanged.

**V5: 10 P7-E both-success layouts** (block 3, regression). Slots 1, 2, 4, 5, 8, 10, 11, 13, 17 and 20.
* The rule: the first both-success layouts in slot order per family.
* The quotas are the batch's family counts by largest remainder: pillars 3, wall_with_gap 3, mixed 3, partial_wall 1.

## 3. Schedule

* **Blocks:** block 1 is V3, block 2 V4, block 3 V5; 10 pairs each, 60 flights in all.
* **Pairs:** each pair flies P8-P4R and P8-P5R back to back.
* **Order:** the order and the first variant come from the frozen P7 SHA-256 stream, seeded by the layouts' canonical
  hash, with labels `p8-block-<b>-order` and `p8-block-<b>-first`.
* **Balance:** 5 of the 10 pairs in every block fly P8-P4R first.

## 4. Recovery parameters (contract §4)

| Parameter | Value | Source |
|---|---|---|
| Trigger | HOVER with `no_progress_candidate`, no stale reason, budget left | contract §4 |
| Episode window | 20 s | `hover_timeout_sim_s` (unchanged) |
| Yaw rate | 10°/s, so at most 200° per episode | P2 `yaw_ramp_deg_s` (unchanged) |
| Translation | 0.2 m/s, so at most 4 m per episode | `transit_speed_m_s` (unchanged) |
| Attempts | one R1 → R2 → R3 pass per HOVER episode | contract S8 |
| R1 radius | 0.80 m from the goal | `goal_tolerance_m` 0.30 + 0.5 m (contract §4); F1 HOVERs were 0.15–0.48 m from the goal |
| Sector step (R1, R2) | 10° | `heading_step_deg` |
| R3 segment | at most 1.5 m, each re-checked by the frozen `collision_check` | `lengths_m` maximum |
| Dead-end memory | 0.39 m | `r_body_m` |
| **R4 (P8-P5R only)** | net progress < 0.05 m over 20 s **and** at least **225** `yaw_to_l4` interventions; then the frozen P4 selection for 20 s | `min_progress_m`; `hover_timeout_sim_s` |

**How the R4 threshold of 225 was derived.** It is one more than the largest count in any 20 s window of any P5 goal
success (224, slot 13, a window with progress).
* No P5 success had a single yaw intervention in a window without progress.
* On the 30 recorded P5 flights the guard would trigger only on slot 12, whose no-progress window held 398.
* The generator refuses to write the parameters if the guard would trigger on any success.

## 5. Report plan

* **H1–H9:** copied from the approved P7-E plan (`15e7f7b4…`) with unchanged limits and sources. A test checks this.
* **Added:**
  * H10 (recovery invariants, re-checked offline from the ledger);
  * H11 (V0–V2 passed and hash-bound before the first flight).
* **Reported:** R-P8-1 to R-P8-10.
* **No targets:** there is no effectiveness target (P8-C1), and no p-values.
* **The contract's §12 statement** is carried verbatim.

## 6. Decision

**APPROVED (operator, 2026-11-28, "Approve specs"):** the four specs, by the canonical hashes of §1.
* The approval fixes the layouts, schedule, recovery parameters and report plan before any P8 code exists.
* P8-M2 must bind exactly these hashes.
* It authorizes nothing to run. Next, separately authorized, comes P8-M2: the code and its tests, with the offline
  stages V0–V2 (contract §13 item 3).
