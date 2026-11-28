# v3-SIM P8 contract `p8-navigation-recovery-v1`: APPROVED

**Status: APPROVED by the operator (2026-11-28, "Approve contract"). In force for the P8 stage.**
* **This approval authorizes nothing to run, freeze, build or delete.** Each later step of §13 (items 2–9) needs its
  own explicit authorization. The next is §13 item 2, the P8-M1 specs.
* **Decided earlier:** P8 is adopted (P8-D1) with the recovery-wrapper architecture (P8-D2), both on 2026-11-28.
* **Decided by this approval:** the defaults of §14, namely P8-D3 to P8-D7 and P8-C1 to P8-C4.
  * P8-C1 is "descriptive only": no effectiveness target is set. Setting one later needs a separate decision before
    the specs are frozen.
* **History:** drafted under "起草 P8-M1 合約" (2026-11-28, commit `ef64ed6`), then approved unchanged. The file
  name keeps "draft", as the P7-E contract's does.

**The basis:**
* the [P8 proposal](v3_sim_p8_proposal.md);
* the read-only analysis [`v3_sim_p8_hover_timeout_analysis.json`](evidence/v3_sim_p8_hover_timeout_analysis.json)
  (`cde7fa52…`) of the accepted P7-E batch;
* the structure of the P7-E contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md).

## 1. Scope and claims

**Claimed on completion:** an engineering validation, in PX4 SITL, of two recovery variants above the accepted
stacks:
* **P8-P4R:** P4 `c110bd30` plus recovery;
* **P8-P5R:** P5 `2becfbbb`, model `ee006a80`, plus recovery.

On the declared layouts, it shows that:
1. every safety and integrity property of the accepted stacks still holds (§3, §7);
2. the recovery layer acts only in the two dead-end situations P7-E found:
   * F1, the terminal-approach deadlock;
   * F2, the narrow-passage local minimum;
3. each flight's outcome is reported per layout, against the accepted P7-E baseline for the same layout.

**Not claimed:**
* superiority of any variant over another, or over the accepted stacks, as a statistical finding;
* statistical significance;
* real-hardware behaviour;
* results outside the declared layouts, mission, speed, static boxes or this host;
* that recovery resolves RR1 (the P2 observation-gap RECOVERY) or RR2 (P5 repeat inconsistency);
* any change to P2–P7-E acceptance.

## 2. Boundaries (normative)

* **B1, preservation.** Nothing listed here changes:
  * the bound files, manifests, candidates and evidence of P2–P7-E, including P3 `1e205fec`, P4 `c110bd30`,
    P5 `2becfbbb`, P6 `22292775`, P7 `0799c37e` and P7-E `e9561f05`;
  * the stopped P7-E candidate `38e23bad` and its records;
  * the P7-E release tag.
  * P8 code reaches frozen code only by import, from a separate tree, `simulation/px4_gazebo_p8/` with a new ROS
    package. P7-E did the same with P7.
* **B2, simulation only.** PX4 SITL and Gazebo, on this host.
* **B3, no accepted limit changes.** The following stay unchanged:
  * **planner:** `r_infl_m` 1.0, `d_stop_m`, the heading set, lengths and `min_progress_m`;
  * **fence and clearance:** the geofence and its margin, and the `clearance_below_body_margin` abort
    (`r_body` + 0.10 m);
  * **perception timing:** `snapshot_max_age_sim_s` 0.25 and `unusable_frames_to_hover`;
  * **supervisor timers:** `stale_abort_sim_s` 10, `hover_timeout_sim_s` 20 and `transit_deadline_sim_s` 150;
  * **mission:** `goal_tolerance_m` 0.30, `settled_speed_m_s` 0.1, `transit_speed_m_s` 0.2, `goal_hover_sim_s` 5;
  * **P2:** the `p2-timing-v1` timing contract, `rate_hz` 20, `yaw_ramp_deg_s` 10, and the nominal landing;
  * **Policy A:** `bf1ef9b1…`.
* **B4, the goal rule stays (P8-D4 default).** A goal counts only in NORMAL, with the setpoint target on the goal,
  the vehicle within 0.30 m, and settled (`mission.py:284–290`).
* **B5, the model stays.** P5's model `ee006a80` is not retrained or replaced. Its ranking is unchanged, except for
  the R4 guard (§4).
* **B6, pre-registration.** Before the first P8 flight, these are frozen and bound by hash:
  * the layouts, schedule and report plan (§6);
  * every recovery parameter (§4).
* **B7, one run per authorization.** There are no automatic retries and no outcome-dependent exclusion. An
  infrastructure-invalid flight is replaced as in P7-E (once, at the block's end, at most 2 of 20).
* **B8, other phases.** P7-A stays suspended and its selection record unwritten, and no P7 pool layout is used. The
  P8 inputs stay protected (cleanup decisions C-2 and C-3).
* **B9, no push, merge or deletion** without explicit words.
* **Binding.** The P8 candidate binds:
  * P7-E `e9561f05`, through P7-E's verification, which binds P7 and P3–P6;
  * the P8 sources and tests;
  * the P8 specs;
  * the host check;
  * a P8 Policy A approval.

## 3. Safety invariants (normative)

Every P8 flight must satisfy all of them. H10 (§7) checks them offline from the recovery ledger.

| ID | Invariant |
|---|---|
| S1 | **Every translation** the vehicle executes satisfies the frozen `planner.collision_check` on the snapshot used for it, with `r_infl` 1.0 m and the fence margin. This covers the frozen planner's translations and the recovery's. It never enters a cell that is not observed FREE or traversed (L2). |
| S2 | **Every in-place yaw** (YAW_TO, SCAN, recovery scans) satisfies the frozen `planner.yaw_permitted` (L5). |
| S3 | **Staleness wins.** While the frozen supervisor reports any stale reason, recovery is inactive; the frozen stale-input HOVER (frozen translation and yaw) and the 10 s stale abort apply unchanged. |
| S4 | **The frozen supervisor stays authoritative.** Its state machine, timers and aborts (`clearance_below_body_margin`, `hover_timeout`, `transit_deadline`) run unchanged. Recovery acts only while the supervisor is in HOVER with reason `no_progress_candidate`, by replacing the HOLD action of that tick. It never changes the supervisor's state or timers. |
| S5 | **Commands.** The command set is unchanged: own commands 176, 400, 21 and offboard setpoints only, under Policy A. |
| S6 | **Landing.** An abort leads to the P2 nominal landing, exactly as accepted. |
| S7 | **Timing.** Recovery never delays the 20 Hz setpoint stream. Its search runs at most once per HOVER episode, and its wall time is recorded. The P2 timing contract and the evaluators' timing items stay the gate. |
| S8 | **Bounded.** Each HOVER episode has an attempt budget, and the frozen `hover_timeout` and `transit_deadline` bound everything. |
| S9 | **Deterministic.** Recovery functions are pure: identical inputs give identical outputs. |
| S10 | **Recorded.** Every recovery action goes to an fsynced ledger `p8/recovery-ledger.jsonl`, with the snapshot identity, the action, and the result of the frozen L2 or L5 check. |

**What S4 means for the design.** Recovery runs inside the frozen 20 s hover window.
* With the P2 yaw ramp of 10°/s, one episode allows at most about 200° of rotation. At the transit speed of 0.2 m/s,
  it allows at most about 4 m of translation.
* Recovery therefore scans only targeted sectors; a full 360° turn takes 36 s.
* Recovery succeeds when the frozen planner, from the new pose or with the new observations, finds a progress
  candidate. The frozen supervisor then leaves HOVER by its own rule.
* If V1 (§6) shows that the window is too short, the only remedy is a separate contract amendment. It would change
  an accepted limit, so it is not proposed here.

## 4. The recovery layer (normative definitions)

**Trigger (all):** the frozen supervisor is in HOVER with reason `no_progress_candidate`; there is no stale reason;
and the episode's budget is not exhausted.

| ID | Situation | Action | Ends when |
|---|---|---|---|
| R1 | **Terminal observation (F1).** The target is within `goal_tolerance_m` + 0.5 m of the goal, and the terminal segment's L2 blockers on the snapshot are UNKNOWN cells only (no OCCUPIED within `r_infl`). | In-place yaw (L5) toward the bearings of the blocking cells, nearest sector first, each sector once; any heading allowed, not only ±60°. | The frozen planner's terminal candidate becomes valid, or the sectors are exhausted |
| R2 | **Targeted out-of-cone observation (F2).** R1 does not apply, and the snapshot has reachable frontier cells (FREE C-space adjacent to UNKNOWN) outside the ±60° cone. | In-place yaw (L5) toward the frontier sectors, ordered by the frontier cell's goal distance, within the rotation available in the episode. | A progress candidate appears, or the sectors are exhausted |
| R3 | **Escape (F2).** After R2, still no progress candidate. | A search on the snapshot in the C-space of L2 (every cell within `r_infl` FREE or traversed, inside the fence margin). It targets the goal if reachable, else the reachable frontier cell with the least goal distance, excluding dead-end memory. Its segments are at most the frozen `lengths_m` maximum (1.5 m) and are executed at `transit_speed_m_s`. **Each segment is re-checked by the frozen `collision_check` at the tick it starts.** | A progress candidate appears, the segment budget is spent, or a check fails (HOLD) |
| R4 | **P5 yaw-oscillation guard (P8-P5R only).** Repeated YAW_TO interventions without net progress toward the goal. | For a bounded period, P8-P5R takes the frozen P4 selection instead of the ranker's, then R1–R3 apply as usual. | The period ends |

* **Dead-end memory.** Positions where a `no_progress_candidate` HOVER began are stored. Cells within `r_body` of
  them are excluded as escape targets for the rest of the flight.
* **Parameters to freeze in the specs (§6), each with its derivation:**
  * the R1 radius: `goal_tolerance_m` + 0.5 m, from the F1 evidence (0.15–0.48 m);
  * the rotation and segment budgets per episode: from the 20 s window, the yaw ramp and the transit speed (§3);
  * the R4 window and threshold: chosen so that the guard **never triggers on the 42 P7-E successes' recorded
    streams**. V1 checks this.
  * No parameter may relax an accepted limit (B3).

## 5. Variants and baselines

* **P8-P4R and P8-P5R,** as in §1. The accepted P4 and P5 packages are used unchanged.
* **Baselines:**
  * the accepted P7-E flights of the same layout (V3, V5);
  * none for the held-out set (V4), where outcomes are reported absolutely.
  * P8 re-flies no baseline, and no comparison is a statistical test.

## 6. Validation stages and specifications (normative)

| Stage | Content | Flights |
|---|---|---:|
| V0 | Offline unit and property tests of the recovery functions: S1, S2, S8 and S9 on synthetic grids; stale inputs mean no recovery | 0 |
| V1 | **Offline counterfactual replay** on the recorded maps, poses and decision streams of all 60 P7-E flights. Recovery triggers in the 18 failures and in no success. Its first action passes the frozen L2 or L5 checks. The budgets fit the 20 s window. | 0 |
| V2 | Offline perception replay through the P6 recorded suite, on the protected raw depth of the 18 failures and the 6 contrast flights (slots 7 P5, 15 P5, 19, 28): the maps that recovery relies on reproduce | 0 |
| V3 | SITL, block 1: the 10 P7-E failure layouts (slots 3, 6, 7, 9, 12, 14, 15, 16, 18, 23) × P8-P4R and P8-P5R | 20 |
| V4 | SITL, block 2: **10 held-out layouts** × 2 variants | 20 |
| V5 | SITL, block 3: **10 P7-E success layouts** × 2 variants (regression) | 20 |

* **V4 layouts (P8-D6 default):**
  * seed base **6,000,000**, disjoint from every existing set: P5 train, validation and held-out (1M, 2M, 3M), P7
    confirmatory (4M) and P7 pilot (5M);
  * the accepted P5 generator and its admissibility filters;
  * the same exclusion of matches with P5 split layouts as P7 used (D8).
  * **Selection, in seed order** until 10 are accepted, by a pre-registered rule: true clearance capacity in
    [1.10, 1.35] m (F2-type), or a goal with unobserved lateral space within `r_infl` at the approach (F1-type),
    5 of each.
  * The capacity is computed by the method of the P8 analysis, frozen in the specs. The bounds come from the P7-E
    evidence (failures 1.14–1.30 m), not from a safety limit.
* **V5 layouts:** the first 10 P7-E both-success layouts in slot order, stratified by family in the batch's
  proportions. The rule is frozen in the specs; the list is fixed before the first flight.
* **Order and blocks:** as in P7-E. Each pair is flown back to back, half of each block P8-P4R first, the order seeded
  by the specs' canonical hash. There are 3 blocks of 20 flights, each separately authorized.
* **The specs** (`p8_specs`, by canonical hash) contain:
  * the V3, V4 and V5 layout lists with their world hashes;
  * the schedule;
  * the recovery parameters;
  * the report plan;
  * the V1 and V2 acceptance records.

## 7. Engineering acceptance (normative)

**Hard criteria.** H1–H9 are inherited from P7-E, with the same limits and sources, applied to every P8 flight:
* collisions 0; truth clearance ≥ 0.20 m; unsafe commands 0; Policy A breaches 0;
* landing and disarm with 176/400/21; safety-unverified 0;
* evidence complete; batch execution accounted; reproducibility.

**Added:**

| ID | Criterion | Limit |
|---|---|---|
| H10 | Recovery invariants S1–S4 and S8–S10, re-checked offline from the ledger against the frozen L2, L5 and fence functions for every recovery action | 100 % reproduce |
| H11 | The offline stages V0–V2 passed and are hash-bound before the first flight | all passed |

**"Engineering validation passed"** requires all of:
* H1–H11 met;
* the declared 60 flights completed (3 blocks, none stopped, every flight accounted for);
* the reported criteria all computed.

**Effectiveness has no threshold here.** Goal completion on V3 and V4, and regressions on V5, are reported per
layout (R-P8-1, R-P8-2, R-P8-9). P8-C1 is adopted as "descriptive only", so no effectiveness target is set (§14).
Setting one would need a separate decision before the specs are frozen.

**Reported criteria** (descriptive):

| ID | What |
|---|---|
| R-P8-1 | Goal success per variant and stage (Clopper–Pearson) |
| R-P8-2 | The per-layout outcome table against the P7-E baseline (V3, V5) |
| R-P8-3 | Recovery activity: triggers, R1–R4 use, attempts, time in recovery, budget exhaustion |
| R-P8-4 | Failure modes after recovery (F1, F2, other) |
| R-P8-5 | Recovery wall time per search; P2 timing and freshness |
| R-P8-6 | Clearance, path length and time to goal |
| R-P8-7 | P3, P4 and P5 timing and inference, against their accepted limits |
| R-P8-8 | Host and cost |
| R-P8-9 | V5 regressions: each flight whose P7-E baseline succeeded but which failed, with its cause, and whether recovery acted |
| R-P8-10 | Integrity events |

## 8. Fail-closed rules (normative)

* **Inherited unchanged:** P7-E §8, including FC-1 to FC-3. P8's sequencer reuses the P7-E code by import.
* **Added:**

| ID | Condition | Behaviour |
|---|---|---|
| FC-4 | A recovery action fails its frozen L2 or L5 check at execution time | not executed; HOLD; recorded (not a stop) |
| FC-5 | The recovery ledger is missing, unparseable or not fsynced, or H10's offline check finds an executed action that violates S1–S4 | **safety-unverified: the batch stops** |
| FC-6 | Recovery acts while a stale reason is present, or after its budget | **safety event (S3, S8): the batch stops** |
| FC-7 | The P8 specs, V0–V2 records or recovery parameters drift from the frozen hashes | **the batch stops** (manifest drift) |

## 9. Evidence

* **As P7-E,** plus per flight:
  * the recovery ledger;
  * the snapshot identities each recovery action used;
  * the H10 check record.
* **Each P8 run directory** is named `-p8-` so that the cleanup manifests can recognize it.

## 10. Engineering risk register (normative)

* **RR1 and RR2 stay open.** P8 reports what bears on them and closes neither.
* **New:**
  * **RR-P8-1:** longer flights and more exposure per flight;
  * **RR-P8-2:** escape segments that move away from the goal. They are safe under S1, but they may lead to new
    dead ends (dead-end memory and budgets);
  * **RR-P8-3:** the R4 guard's interaction with P5's near-ties (median decision margin 0.12–0.15);
  * **RR-P8-4:** offline replay cannot show closed-loop effects, hence V3–V5;
  * **RR-P8-5:** compute spikes like slot 27 P4's 0.640 s frame (R-P8-5, R-P8-7).
* No threshold is changed in response to any risk.

## 11. Retention

* **D4-A and D4-B apply.** During P8, its runs keep complete evidence.
* **The P8 inputs stay protected** (decisions C-2 and C-3) until P8 is accepted:
  * the raw depth of the 18 P7-E failures;
  * the contrast flights (slots 7 P5, 15 P5, 19, 28).
* Retention approval is not deletion.

## 12. Statement carried by the report

> "P8 is an engineering validation of a recovery layer. Its per-layout outcomes are descriptive; with 10 layouts per
> stage they do not establish a rate, and they do not show that any controller or variant is superior."

## 13. Authorizations required, each separate and explicit

1. Approval of this contract. **Done on 2026-11-28 ("Approve contract").**
2. **P8-M1 specs.** The V3, V4 and V5 layout lists, the schedule, the recovery parameters and the report plan, with
   the generator and selection code. Approved by hash.
   * **Generated and tested on 2026-11-28** ("Authorize the specs"): [P8-M1 specs](v3_sim_p8_m1_specs.md).
   * Layouts `62ae950c…`, schedule `df90cf7b…`, recovery `dee90d51…`, report plan `a313a034…`.
   * **APPROVED by hash on 2026-11-28 ("Approve specs").** P8-M1 is complete.
3. **P8-M2 code and tests, offline.** The `px4_gazebo_p8` tree and the V0–V2 runs. Its acceptance is separate.
   * **Written and tested on 2026-11-28** ("Authorize P8-M2", then "Continue M2"): [M2 report](v3_sim_p8_m2_code.md).
   * 69 P8 tests pass. V1 (`1d6cc0e2…`) and V2 (`912297fb…`) passed.
   * **ACCEPTED on 2026-11-28 ("Accept M2")**, with interpretations I-1 to I-11. P8-M2 is complete.
4. A development dry run (2 flights) of the new launch path, with its own approval.
   * **Flown on 2026-11-28** on the development candidate `0a31ecd0…`; [M3 record](v3_sim_p8_m3_candidate.md).
   * **ACCEPTED on 2026-11-28 ("Accept, take (a)")**, with D-M3-1 (a), the P5 ledger reconciliation in R-P8-10.
5. The production Policy A approval for the P8 candidate.
   * **Done on 2026-11-28 ("Approve (Recommended)"):** `state/p8-policy-A-approval-2a0ae57.json` (`befd23a3…`),
     `development_only: false`, binding the 37 files with canonical `10eb4660…`.
6. The real freeze of the P8 candidate.
   * **Done on 2026-11-28 ("Freeze it (Recommended)"):** manifest
     **`fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`** (`state/p8-candidate-2a0ae57.json`),
     verified from `/tmp`. P8-M3 is complete.
7. **The V3, V4 and V5 blocks,** each with its own authorization.
8. The P8 report, and its acceptance.
9. Any push, tag, merge or cleanup.

## 14. Choices (decided by the contract's approval, 2026-11-28)

Each default below is adopted, as approved with the contract.

| ID | Choice | Adopted |
|---|---|---|
| P8-D3 | The recovery set R1–R4 and its budgets, within the 20 s window | adopt §4 |
| P8-D4 | The goal rule | keep (B4) |
| P8-D5 | Validation V0–V5 (60 SITL flights) | adopt §6 |
| P8-D6 | The held-out seed base and selection rule | 6,000,000; §6 rule |
| P8-D7 | The optional tracks: P8-S (offline P5 timing sensitivity), P8-T (scheduling data collected during V3–V5) | P8-S yes; P8-T within V3–V5 |
| P8-C1 | A pre-registered effectiveness target for V3 or V4 (a number of goal successes), or descriptive only | descriptive only; any number is the operator's |
| P8-C2 | The V5 selection rule (first 10 both-success layouts, stratified by family) | adopt |
| P8-C3 | The R4 derivation rule (never triggers on the 42 recorded successes) | adopt |
| P8-C4 | If V1 shows that the 20 s window is too short: stop at V1 and report, or prepare a separate amendment | stop and report |
