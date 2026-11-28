# v3-SIM P8: Navigation Robustness and Recovery (preliminary proposal)

**Status: PRELIMINARY PROPOSAL FOR THE OPERATOR'S REVIEW (2026-11-28).**
* **Decided (operator, 2026-11-28):** P8 is adopted (P8-D1), with the recovery-wrapper architecture (P8-D2).
* **Contract APPROVED (operator, 2026-11-28, "Approve contract"):**
  [`p8-navigation-recovery-v1`](v3_sim_p8_contract_draft.md), with its §14 defaults.
  * It authorizes nothing to run.
* **P8-M1 specs APPROVED by hash (operator, 2026-11-28, "Approve specs"):** [P8-M1 specs](v3_sim_p8_m1_specs.md).
  * Layouts `62ae950c…`, schedule `df90cf7b…`, recovery `dee90d51…`, report plan `a313a034…`.
* **P8-M2 ACCEPTED (operator, 2026-11-28, "Accept M2"):** [M2 report](v3_sim_p8_m2_code.md), code commit `c991387`.
* **P8-M3 COMPLETE (operator, 2026-11-28):** [M3 record](v3_sim_p8_m3_candidate.md).
  * It covered the build, the development dry run (accepted), the production Policy A and the real freeze.
  * **The real P8 candidate is manifest `fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`.**
* **The V3, V4 and V5 blocks were flown (2026-11-28):** [validation blocks](v3_sim_p8_validation_blocks.md). None
  stopped; 60 of 60 valid.
* **P8 ACCEPTED (operator, 2026-11-28, "Accept P8"):** [report](v3_sim_p8_report.md) §7.
  * The report reads "not passed" for "H9 not met" (finding F-R1, a label comparison in the bound report code).
  * The out-of-tree H9 addendum records "explained" on all 60 flights.
  * The acceptance rests on both together, and authorizes nothing else.
* It was prepared under the operator's request "P7-E Release Closure and Engineering Next-Stage Preparation", which
  authorizes read-only auditing and proposal preparation only.
* At that time nothing was implemented, no SITL had run and no candidate existed. No frozen P2–P7-E file has changed
  since then.

**Evidence:**
* [`v3_sim_p8_hover_timeout_analysis.json`](evidence/v3_sim_p8_hover_timeout_analysis.json) (`cde7fa52…`);
* the tool that wrote it, [`p8_hover_timeout_analysis.py`](../diagnostics/p8/p8_hover_timeout_analysis.py).

The tool only reads the accepted P7-E batch (candidate `e9561f05…`) and the runs' own records. It never reads raw
depth frames, and it never runs frozen code; it reads the accepted P4 configuration as data.

## 1. Motivation

In P7-E, 18 of 60 flights failed the mission (P4 10 of 30, P5 8 of 30).
* **All were safe.** Every one entered HOVER, waited the accepted 20 s, aborted with `hover_timeout`, then landed and
  disarmed through P2's nominal sequence, with 0 safety events.
* **The batch passed its engineering criteria.** The failures cost mission success only.
* **P8's purpose:** make the stack reach the goal in these situations, without weakening any safety property.

## 2. What the existing evidence shows

### 2.1 The failures are deterministic and shared by both controllers

* **The same failure on the same layouts:**
  * 16 of the 18 failures are in the 8 pairs where both controllers failed on the same layout (slots 3, 6, 9, 12, 14,
    16, 18 and 23);
  * in 6 of those 8 pairs, P4 and P5 entered their final HOVER within 0.31 m of each other (0.02–0.31 m);
  * the 2 discordant pairs are P4 failures (slots 7 and 15). There is no P5-only failure.
* **The non-learning P4 fails by the same mechanisms as P5.** AI inference is therefore not needed to explain the
  failures. §2.4 checks the P5-specific factors separately.
* **Persistent HOVER, not oscillation between states:**
  * every failure entered HOVER exactly once, with reason `no_progress_candidate`, and never left it;
  * it spent the 20 s holding: 7,151 `HOLD:no_collision_valid_progress_candidate` ticks and 68 `SCAN:unknown_blocked`
    ticks across all 18.
  * The successes spent at most 0.55 s in HOVER.
* **Abrupt, not gradual, loss of progress:**
  * in the 30 s before the final HOVER, the median failure still moved 3.9 m closer to the goal, with straightness
    0.89 (net over path length);
  * the best goal distance stopped improving a median of 18.9 s before the abort, so progress stopped at the HOVER
    entry;
  * the vehicles hit a dead end. They did not drift into one slowly.
  * The exception is slot 12 P5 (§2.4).

### 2.2 Two mechanisms

**F1: terminal-approach deadlock (6 failures: P4 4, P5 2; slots 15 P4, 16 P4, 18, 23).**
* **Where they stopped:** the final HOVER is 0.15–0.48 m from the goal, and 3 of the 6 are already inside the
  0.30 m goal tolerance.
* **What the planner saw:** 2–6 collision-valid candidates, none with progress above `min_progress_m` 0.05 m.
* **What blocks the direct terminal segment** (0.15–0.48 m long), on the last usable map: no OCCUPIED cell at all, but
  5–31 **never-observed** cells within `r_infl` (1.0 m). Most lie laterally, more than `r_body` from the goal line.
* **Why scanning did not help:** 10–12 `SCAN:unknown_blocked` actions did not clear them. Scan headings are limited
  to the planner's ±60° candidate cone, and the blocking cells lie to the side.
* **Why the goal was not declared:** the frozen goal rule needs the supervisor in NORMAL **and** the setpoint target
  exactly on the goal, plus the vehicle within 0.30 m and settled (`mission.py:284–290`).
* **Geometry is not the cause:** their layouts' true clearance capacity is 1.49–2.00 m, wide. The cause is the
  deterministic goal-approach logic of the frozen P4 planner and mission, which P5 shares.

**F2: mid-course narrow-passage local minimum (12 failures: P4 6, P5 6; slots 3, 6, 7 P4, 9, 12, 14, 16 P5).**
* **Where they stopped:** the final HOVER is 3.1–6.2 m from the goal (x 1.8–4.9 m), entered from AVOIDING (or
  RESUME), with **0 collision-valid candidates** in the ±60° cone.
* **Geometry:**
  * The true geometry has a path in every case, even at the accepted 1.0 m inflation (60 of 60 layouts are feasible
    at 1.00, 0.79 and 0.49 m).
  * But 11 of the 12 are in layouts whose **clearance capacity** is 1.14–1.30 m: the largest clearance radius with a
    true path is only 0.14–0.30 m above `r_infl`. The successes' median is 1.76 m.
  * Of the 16 flights in layouts with capacity ≤ 1.30 m, 11 failed; elsewhere 7 of 44 failed, 6 of them F1.
* **On the recorded map at the HOVER entry, under the accepted rule** (every cell within `r_infl` observed FREE or
  traversed):
  * no known-free path to the goal existed in any case;
  * safe in-place moves *outside* the ±60° cone existed in 11 of 12 cases (median about 27 of the 46 out-of-cone
    pairs of heading and length);
  * a reachable frontier of unobserved space existed (median 16 cells);
  * treating unknown space as free, a path existed in 4 of 12.
* **Mechanism:** a forward-cone local planner, with a 1.0 m inflation and an observed-FREE rule. It commits to one
  side, then reaches a narrow approach where every forward segment is blocked. It has no backtracking, no
  observation outside the cone, and no memory of dead ends.

### 2.3 Timing, perception stalls and scheduling are not the cause of these 18

* **No stall before any final HOVER:** no native-clock stall (RTF < 0.5) within 5 s before any of the 18 final HOVER
  entries. The nearest is 27 s away, with a median of 96 s.
* **No stale input:** no stale decision in any failure, and every snapshot age was ≤ 0.204 s against the 0.25 s
  limit. The 13 `snapshot_stale` ticks of the batch all occurred in successes.
* **Where timing does fail missions in P7 evidence,** it is a different, P2-level mechanism:
  * the 2 pilot observation-gap RECOVERY aborts (RR1);
  * the stopped attempt's slot 1 P5 `stale_action` RECOVERY (minimum interval RTF 0.005).
  * This stays a separate track: RR1 and §4 item P8-T.

### 2.4 P5-specific factors

* **Yaw oscillation (the accepted S5 limitation):**
  * slot 12 P5 spent 90 s in RESUME with 1,542 `yaw_to_l4` interventions and 0.03 m of net progress in its last 30 s;
  * it then entered HOVER 0.16 m from where P4 stopped on the same layout;
  * the oscillation cost time but did not cause a different dead end.
  * Across failures, YAW_TO decisions have a median of 14 against 0 in successes. Left/right switching is not higher
    in failures (median 0.0 per minute against 1.3).
* **Decision margins:**
  * the gap between P5's best and second-best allowed candidate is small in both groups: median 0.15 in failures,
    0.12 in successes;
  * 73–79 % of ticks have a margin below 0.25.
  * Near-ties are common, so timing sensitivity is plausible (RR2, from the pilot's repeats). The margins do not
    distinguish the failures.

### 2.5 Limits of this evidence

* **One flight per controller per layout.** Repeatability cannot be measured here (RR2).
* **The map reconstruction is approximate.** The traversed mask is rebuilt from the recorded positions. At 4 of the
  18 hover points, this rebuild makes the start cell unusable, so their move counts are 0 by construction.
* **The true geometry uses the layout boxes,** axis-aligned, in the world frame (offset about 3 cm from PX4 local).
* **No counterfactual flight was run.** What a recovery would have achieved needs closed-loop tests (§5).

## 3. Goals and non-negotiable constraints

**Goal:** on layouts where a true path exists at the accepted clearance, reach the goal instead of aborting, in both
F1 and F2 situations. Otherwise still abort safely as today.

**Unchanged (every P8 candidate must show each by test):**
* Frozen P2–P7-E implementations: no change to any bound file, manifest or accepted evidence. P8 is an isolated tree.
* **The safety shield and clearance:**
  * every translation passes the frozen L2 `collision_check` with `r_infl` 1.0 m, the fence margin, and the
    observed-FREE or traversed rule;
  * in-place yaw happens only under the frozen L5 `yaw_permitted`;
  * the `clearance_below_body_margin` abort stays;
  * Policy A stays.
* **Timing:** the P2 timing contract, the snapshot age limit of 0.25 s, the stale-input HOVER with frozen yaw, and
  the 10 s stale abort.
* **Landing:** the P2 nominal sequence and own commands 176/400/21 only. `hover_timeout` and the 150 s
  `transit_deadline` stay the terminal fail-safes.
* **Fail-closed runs:** FC-1 to FC-3 and the P7-E sequencing rules for any SITL.
* **No assumption that inference is the cause.** The first P8 work is deterministic, and P5's model is not retrained.

## 4. Preliminary architecture

**An isolated tree `simulation/px4_gazebo_p8/`,** binding P3–P7-E through the P7-E verification, as P7-E bound P7.

| Component | Role |
|---|---|
| `ros2_ws/src/gwm_navigation_recovery/` (new package) | A **recovery manager** between the frozen supervisor and the setpoint path. It imports the frozen P4 modules (`planner`, `supervisor`, `local_map`, `mission`) unchanged, as P5's package does, and is active only while the frozen supervisor is in HOVER with `no_progress_candidate`. Every action it emits is re-checked by the frozen L2, L5 and fence functions before it is used. |
| `recovery.py` (pure functions) | Detects the situation (F1 near the goal, F2 dead end, P5 yaw oscillation) and chooses one action: **R1 terminal observation**, then **R2 out-of-cone observation**, then **R3 escape**, as below. |
| `search.py` (pure functions) | A multi-step search on the recorded observed map in the same C-space as L2 (every cell within `r_infl` FREE or traversed, inside the fence margin). It targets the goal, else the reachable frontier nearest the goal. It returns waypoints whose every segment passes `collision_check`. |
| `memory.py` | Dead-end memory: cells where a `no_progress_candidate` HOVER occurred, so that the same pocket is not re-entered. |
| `replay/` | An **offline counterfactual harness**. It runs the recovery manager on the recorded maps, poses and decision streams of the P7-E flights, with no simulator. |
| `validation/` | The accepted P4 and P5 evaluators and the frozen P7 classifier, unchanged, plus recovery-invariant checks from the event log: every recovery translation passed L2, every yaw passed L5, and the budgets held. |
| `scripts/p8_manifest.py`, runner, launcher | As P7-E: Policy A per candidate, freeze, block authorizations, FC-1 to FC-3. |

**Recovery actions (deterministic; each bounded):**
* **R1, terminal observation (F1).** Applies within `goal_tolerance` + 0.5 m of the goal, when the terminal segment
  is blocked only by UNKNOWN cells. The vehicle yaws in place, under L5, toward the blocking sectors outside the
  cone so that they are observed. Then the frozen planner's terminal segment can become valid.
  * The goal rule and L2 are not relaxed.
  * A relaxed goal rule (accepting the goal from HOVER when the vehicle is within tolerance) changes the mission's
    success definition. It is offered only as decision P8-D4.
* **R2, out-of-cone observation (F2).** A full 360° scan under L5 at the dead end, to observe the unknown space
  outside the ±60° cone, then a re-plan.
* **R3, escape (F2).** If R2 finds no forward candidate, follow `search.py`'s waypoints toward the goal or the best
  frontier, including lateral or backward segments. Each segment is checked by the frozen `collision_check`. There
  is a bound on the steps and on the time.
* **R4, yaw-oscillation guard (P5).** Too many YAW_TO interventions without net progress hand control to R2 and R3.
  P5's model and ranking are unchanged outside that guard.
* **Budget:** each HOVER episode allows at most a fixed number of recovery attempts. When it is exhausted, the frozen
  path applies: `hover_timeout`, then landing. The 150 s transit deadline still bounds everything.

**The interface choice (decision P8-D2).** A wrapper above the frozen supervisor, which leaves P4 and P5 untouched
and creates the variants P8-P4R and P8-P5R, is recommended. A modified supervisor would be a new P4 candidate and
would reopen P4's acceptance.

## 5. Validation plan (offline first; SITL only where a requirement needs it)

| Stage | What | SITL |
|---|---|---|
| V0 | Unit and property tests of the pure functions. Every emitted segment passes the frozen L2; every yaw passes L5; the budget is bounded; staleness means HOLD. | none |
| V1 | **Counterfactual replay** on the 18 recorded HOVER situations and the 42 successes' decision streams: the manager triggers exactly in the 18, never changes a success's decisions, and its first action is valid. | none |
| V2 | Perception-level replay on the protected raw depth of the 18, plus contrast flights, through the P6 recorded suite: the maps the manager relies on reproduce. | none |
| V3 | **Targeted SITL** on the 10 P7-E failure layouts (slots 3, 6, 7, 9, 12, 14, 15, 16, 18, 23) × P8-P4R and P8-P5R: **20 flights** | 20 |
| V4 | A **held-out** set of 10 new layouts from a new seed base, selected for narrow passages (capacity 1.1–1.3 m) or a goal near unobserved space × 2 variants: **20 flights**. No P7 pool is used, so P7-A's selection stays untouched. | 20 |
| V5 | **Regression** on 10 P7-E success layouts (stratified by family) × 2: **20 flights** | 20 |

**Requirements tied to the flights:**
* **V3:** each failure layout reaches the goal, or still aborts safely with the recovery invariants met.
* **V4:** generalization, so that the 10 known layouts are not overfitted.
* **V5:** no success regresses.

**Size:** about 60 flights, 3.2 h, about 80 GB. There are no statistical superiority claims; each layout's outcome is
reported, with the hard criteria of P7-E (H1–H9) reused.

**P8-T, optional track (no extra flights).** It collects finer-grained scheduling data during V3–V5, for RR1 and the
P2 `stale_action` RECOVERY.

**P8-S, optional track (offline).** It measures P5's sensitivity to timing: it recomputes the ranker's inputs from
recorded maps with ±1-frame snapshot shifts and counts decision flips, using the frozen P5 feature code as data.

## 6. Milestones (each separately authorized)

| Milestone | Content |
|---|---|
| P8-M0 | This proposal: decisions P8-D1 to P8-D8 |
| P8-M1 | Contract `p8-navigation-recovery-v1`: requirements, invariants, success definition, specs of V3–V5 (layouts by hash) |
| P8-M2 | Code and tests offline (V0, V1, V2) |
| P8-M3 | Candidate: development dry run (2 flights), Policy A, freeze |
| P8-M4 | V3, V4, V5 blocks, one authorization each |
| P8-M5 | Report and acceptance |

## 7. Decisions for the operator

| ID | Decision | Recommendation |
|---|---|---|
| P8-D1 | Adopt P8 with the motivation and evidence of §1–§2 | **DECIDED (operator, 2026-11-28): adopted** |
| P8-D2 | Architecture: a recovery wrapper above the frozen supervisor (P8-P4R, P8-P5R), or a modified P4 supervisor (a new P4 candidate) | **DECIDED: the recovery wrapper** |
| P8-D3 | Recovery set R1–R4 with budgets; no change to `r_infl`, `hover_timeout` or the transit deadline | adopt |
| P8-D4 | The goal rule: keep the frozen rule (NORMAL and target on the goal), or allow goal capture from HOVER within tolerance | keep it; R1 first |
| P8-D5 | Validation: V0–V2 offline, then about 60 SITL flights (V3–V5) | adopt |
| P8-D6 | The held-out layouts: a new seed base, disjoint from every P7 pool | adopt |
| P8-D7 | Include the optional tracks P8-T and P8-S | P8-S yes (offline); P8-T with V3–V5 |
| P8-D8 | Retention: protect the P8 inputs (cleanup proposal §3 and C-3) until P8 is accepted | **covered:** C-2 (confirmed) and C-3 (slots 19 and 28 protected), 2026-11-28 |

**P8-D3 to P8-D7 are DECIDED** by the approval of the [contract](v3_sim_p8_contract_draft.md)
`p8-navigation-recovery-v1` (operator, 2026-11-28, "Approve contract"), as its §14 defaults. So are P8-C1 to P8-C4.
