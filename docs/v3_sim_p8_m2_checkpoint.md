# v3-SIM P8-M2 checkpoint: the recovery core and the offline stages V0–V2

**Status: CHECKPOINT PASSED.** The offline stages V0, V1 and V2 are done and pass (§2–§5).
* The operator chose "Continue M2" at the P8-C4 checkpoint (2026-11-28).
* The completed milestone is reported in the [M2 report](v3_sim_p8_m2_code.md).
* Offline only: no simulator, no ROS run, no build, no candidate, no deletion.
* No P2–P7-E file and no approved P8 spec changed. All seven manifests still verify.
* The four approved spec hashes are unchanged.

## 1. What is written

| Part | File | Content |
|---|---|---|
| Search | `ros2_ws/src/gwm_navigation_recovery/gwm_navigation_recovery/search.py` | the L2 C-space cell by cell (`r_infl` 1.0 m, fence margin), BFS, frontier, the escape target, and greedy segments; run once per HOVER episode (S7) |
| Manager | `…/recovery.py` | the trigger (HOVER entered with `no_progress_candidate`, HOLD, no stale reason, no frozen yaw, no P5 policy failure); R1, R2 and R3 in one pass; the frozen L5 check before every yaw; the frozen L4 cone and L2 `collision_check` before every translation; FC-4 refusals; the ledger records |
| R4 guard | `…/guard.py` | `yaw_to_l4` ≥ 225 within 20 s without 0.05 m of progress, then the frozen `P4Ranker` for 20 s |
| Wrapper | `…/wrapper.py` | wraps the frozen supervisor; every other attribute passes through to it, P5's `last_output` included |
| Missions | `…/mission.py` | `P8P4Mission` and `P8P5Mission`: the accepted missions, wrapped at MAP_SCAN → TRANSIT |
| Package | `package.xml`, `setup.py`, `setup.cfg` | the ament package `gwm_navigation_recovery` (entry points for the nodes still to write); not built |
| H10 | `validation/p8_h10_check.py` | the offline re-check of every ledger action against its snapshot with the frozen L2/L5 functions; the L4 cone, the window, the budgets, staleness, and the ledger-decision correspondence |
| V1 | `replay/p8_v1_replay.py` | the counterfactual replay on the 60 P7-E flights |
| V2 | `replay/p8_v2_perception.py` | the perception replay on the 24 protected inputs, through the frozen P6 `recorded.PerceptionReplay`; a frame missing from a recording is skipped and reported |
| Tests | `tests/test_px4_gazebo_p8_recovery.py` (32), `tests/test_px4_gazebo_p8_h10.py` (9) | V0 and H10; all 41 pass |

## 2. V0: unit and property tests (32 passed)

* **The trigger:** every non-trigger tick passes through unchanged. That covers NORMAL, AVOIDING, stale, a stale
  HOVER entry, frozen yaw, a P5 policy failure, a frozen SCAN, no HOVER, and ABORT.
* **R1:**
  * it yaws toward never-observed lateral cells near the goal, with L5 checked;
  * it does not apply when an occupied cell blocks the terminal segment.
* **S1 and S2, a property over 12 random scenes:** every emitted yaw passes the frozen `yaw_permitted`. Every
  emitted translation passes the frozen `collision_check` and lies in the L4 cone. The scenes produced 338 yaws and
  1,423 translations; a test keeps that coverage from becoming vacuous.
* **S8:** the rotation and translation budgets hold, and nothing is emitted after the window.
* **FC-4:** a refused check means HOLD and a ledger record.
* **S9:** the manager is deterministic.
* **The C-space:** it equals the frozen L2 rule at every sampled cell centre.
* **Encoding:** the traversed mask round-trips.
* **The guard:** it fires only on sustained interventions without progress.
* **The wrapper:** it passes every attribute through.
* **The mission:** the P4 mission installs the wrapper at TRANSIT.

## 3. H10 checker (9 passed)

* **A clean closed-loop episode** reproduces every check: more than 100 actions.
* **Each of 8 injected violations is found:**
  * a tampered segment end;
  * a translation outside L4;
  * an action on a stale tick;
  * a missing ledger (FC-5);
  * a recovery decision with no ledger record;
  * an action outside the window;
  * a budget overrun;
  * an unknown snapshot.

## 4. V1: counterfactual replay on the 60 P7-E flights

Evidence: [`v3_sim_p8_v1_replay.json`](evidence/v3_sim_p8_v1_replay.json) (`1d6cc0e2…`). **All five criteria pass.**

| Criterion | Result |
|---|---|
| triggers in exactly the 18 failures | yes (18 of 18; the only success HOVER was stale, slot 27 P4) |
| no trigger in a success | yes |
| every first action passes its frozen check | yes (17 first actions: R1 6, R2 8, R3 3; 0 refused) |
| every plan fits the 20 s window | yes (planned 7.6–19.6 s) |
| the R4 guard fires only on slot 12 | yes |

* **The search** takes at most 4 ms per episode.
* **Open-loop limit:** the recorded vehicle does not follow recovery. V1 judges triggers, plans, first actions and
  the guard; closed-loop effects are V3–V5.

**What the frozen 20 s window allows** (the substance of P8-C4):

| Situation | Episodes | What fits |
|---|---:|---|
| F1, near the goal (R1) | 6 | **every** blocking sector: 2–6 sectors, 76–115°, in 7.6–11.5 s, time to spare |
| F2, dead end, with an escape (R2 + R3) | 6 | one escape segment of 1.0–1.5 m, plus the scans that still fit |
| F2, dead end, scan only (R2) | 5 | scans only (125–184°). The escape found by the search (2–5 segments) needs a large turn first, which at P2's 10°/s does not fit in 20 s together with the move. |
| no plan | 1 | slot 7 P4: the hover point is not usable under L2, so no translation is possible and the approved R2 rule (reachable frontier) yields no sector |

**Implementation interpretation I-1, for M2's acceptance.** The approved time rule says that rotation time plus
translation time never exceeds the window left. It does not say which phase gets the time first.
* **R1 episodes** (an observation problem): the scans first, then R3 in what is left.
* **R2 episodes** (a motion problem): the escape is reserved first, then R2 sectors fill the rest.
* My first version budgeted the phases separately: 12 plans then exceeded 20 s, so V1 failed. I fixed it before
  recording the evidence.

## 5. V2: perception replay on the protected raw depth

Evidence: [`v3_sim_p8_v2_perception.json`](evidence/v3_sim_p8_v2_perception.json) (`912297fb…`). **Passed.**
* **Scope:** 24 flights, the 18 failures and 6 contrast flights (slots 7 P5, 15 P5, 19, 28). Every usable snapshot was
  replayed in order through the frozen P6 `recorded.PerceptionReplay`, each frame checked against its recorded
  SHA-256. That took 338 s, read-only.
* **Result:** of 32,316 snapshots, 32,200 reproduce the recorded grid bit for bit.
  * **23 of 24 flights:** every snapshot is identical.
  * **Slot 3 P4:** the recording lacks 2 frames that perception processed, at 7.43 s and 7.53 s, during take-off.
    The adapter recorder and perception are separate best-effort subscribers.
    * The replayed grid differs from snapshot 8 on, for 116 snapshots.
    * Later frames then re-cover those cells, and the grid matches again well before the HOVER.
    * Its final-HOVER snapshot (99.63 s) is identical.
* **Every failure's final-HOVER snapshot reproduces exactly,** the 18 of them. This is the map recovery would rely on.
* **Rule recorded with the result:** "a frame missing from a recording is reported, not hidden".

## 6. The P8-C4 decision (approved default: "stop and report")

* **Strictly, V1 does not show the window too short.** Every criterion passes, and every plan fits.
* **But the window limits F2.** About half of the dead-end episodes can only scan, because a backward escape needs a
  turn that 20 s at 10°/s cannot hold.
* **F1 fits comfortably.**

**The remaining M2 work, if it continues:**
* the P8 nodes: reviewed copies of the accepted P4 and P5 nodes, about 800 lines. The only additions are the P8
  mission and the persisted recovery ledger, and a test diffs each copy against its frozen original;
* the P8 runner, manifest, launcher and block sequencer, reusing the P7-E code;
* `analysis/p8_report.py` and its tests;
* the drift checks and the M2 report.

**Options for the operator:**
1. **Continue M2 as approved.** V3 will then show what the window allows in closed loop.
2. **Stop at V1** (P8-C4), and consider a separate contract amendment for the hover budget. That would change an
   accepted limit, which the approved contract does not propose.
3. **Continue, but ask first for a separate analysis** of an amendment, without changing anything now.
