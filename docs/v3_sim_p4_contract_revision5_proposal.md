# P4 contract `p4-acceptance-v1` — revision 5 proposal (how a supervisor `ABORT` lands)

**Status: PROPOSED — awaiting operator approval. Not applied.** It amends revisions 2, 3 and 4
(commits `d3c06b4`, `ea2c9e3`, `65b99e4`). A designed P4 supervisor `ABORT` lands through P2's
nominal landing sequence to `COMPLETE`, with the same single LAND (21), instead of through P2
`RECOVERY`. P2 safety errors still use `RECOVERY` and fail the run. No threshold, limit, speed,
scenario or gate changes, and no P2/P3 file changes. Nothing was launched to prepare this proposal.

## Problem

Gate S of candidate `1be1896a` failed at S5 ([plan §42](v3_sim_p4_plan.md)), although S5 behaved
exactly as the contract requires.

* **S5 behaviour:** HOVER at 90.07 s (`no_progress_candidate`), then ABORT at 110.12 s
  (`hover_timeout`). The target moved 0.0 m after HOVER, minimum truth clearance was 0.891 m, there
  were 0 collisions, and the single LAND was issued; the vehicle landed.
* **Why it failed:** contract §5 sends `ABORT` into P2 `RECOVERY` ("one LAND (21) through P2
  recovery rules"). The accepted P2/P3 evidence tools, which §8(a)/(b) and §9.1 F4 reuse unchanged,
  treat `RECOVERY` as a failed flight. That produced four of the five S5 failures.

| S5 failure | Cause in the unchanged P2/P3 code |
|---|---|
| `p2:timing` | `timing_evidence.trace_metrics` sets `status` to `failed` whenever the controller result has a `failure`, and the abort records `p4_supervisor_abort:hover_timeout`. It also **skips** the full flight timing requirements (prestream, publication gaps, late dispatch, stale source) in that case, so S5 was never actually timed. |
| `p2_integrity_error: estimator_reference_reset:final_alignment_lost` | The P2 reference monitor (`estimator_reference.py`, replayed by `reference_evidence.classify`) rejects any loss of in-air alignment outside `REQUEST_LAND`, `VERIFY_LANDING`, `VERIFY_LANDED_AND_DISARMED` and `COMPLETE`. A touchdown in `RECOVERY` always trips it. |
| `rgb_absence:snapshot_missing:window_end` | The P3 runner's RGB-absence snapshotter takes `window_end` only on the `COMPLETE` transition (`p3_runner.py`, `TRIGGERS`); a run ending in `RECOVERY` never reaches it. |
| `mc_causal_yaw:p2_integrity_missing` | This follows from the reference failure. |

S6 also ends with `ABORT` → `RECOVERY`. As written, **neither S5 nor S6 can pass**, whatever the
vehicle does. The contract is also internally inconsistent: §9.1 already bounds the S6 fault
window by `COMPLETE` (`W_fault` = (`t_inj`, `COMPLETE`]), and the MAVLink landing window runs "first
controller LAND through `COMPLETE`".

## Proposed changes (exact)

1. **§5 transition table.** The current row "any | P2 safety error, geofence breach, clearance to an
   occupied cell < `R_body` + 0.10 m, or `TRANSIT` deadline | `ABORT`" splits into two rows:
   * "any | clearance to an occupied cell < `R_body` + 0.10 m, or `TRANSIT` deadline | `ABORT`";
   * "any | P2 safety error or geofence breach | P2 `RECOVERY` (the run fails; unchanged P2 rules)".
2. **§5 `ABORT` row** becomes: "`ABORT` | hold, then exactly one LAND (21) through P2's nominal
   landing sequence `REQUEST_LAND` → `VERIFY_LANDING` → `VERIFY_LANDED_AND_DISARMED` → `COMPLETE`,
   under P2's unchanged landing checks and phase deadlines; landed and disarmed within 45 s wall of
   the LAND command | (terminal)".
3. **New rule after the table:** "A supervisor `ABORT` (`hover_timeout`, `stale_inputs:*`,
   `map_scan_stale_inputs:*`, `transit_deadline`, `clearance_below_body_margin`) is a P4 outcome,
   not a P2 failure. The controller records `p4_outcome` = `supervisor_abort` and the abort reason,
   and leaves the P2 failure field empty. Any other failure (P2 safety error, geofence breach,
   timing guard, ACK failure, observation gap, and so on) enters P2 `RECOVERY` exactly as before
   and fails the run. Routing never makes a run pass: every scenario's §7 outcome check still
   requires its own expected outcome and abort reason."
4. **§4 mission:** add after step 4: "On a supervisor `ABORT` in `MAP_SCAN` or `TRANSIT`, the
   mission goes from the hold directly to step 5 (`REQUEST_LAND`)."
5. **§9.1 F3, last item** becomes: "Exactly one LAND (21) is issued through `REQUEST_LAND` and
   accepted, and the vehicle reaches `COMPLETE` (landed and disarmed) within 45 s wall of the LAND
   command."
6. **Contract header and `p4_validation.yaml` `contract_revision`:** these will name revisions 2, 3,
   4 and 5.

## Unchanged

* All P2/P3 code, configs, evaluators and the P3 manifest `1e205fec…`.
* §8(a)/(b) and §9.1 F4 still reuse the P2/P3 checks unchanged. With this revision they apply **in
  full** to S5 and S6:
  * P2 timing runs with its full flight requirements, because no `failure` is recorded.
  * The reference monitor sees the landing phases it allows.
  * The RGB-absence `window_end` snapshot is taken at `COMPLETE`.
  * MC causality and yaw ownership get their P2 inputs.
  * The MAVLink landing window ends at `COMPLETE`.
* The 45 s wall bound from LAND to landed-and-disarmed (the former P2 recovery limit) stays, as an
  explicit P4 evaluator check, on top of P2's per-phase landing deadlines. Every §6 and §6.1
  threshold, the scenarios S1–S6, Gates A/B/S/C, revisions 3 and 4, and Policy A are unchanged.

## The compute-max spike (investigated offline; no contract change proposed)

S5's fifth failure was one perception frame of **0.0948 s** against the §6.1 maximum of 0.090 s.

* **Context:** it happened at takeoff (acquisition 19.31 s). The frame before took 0.0405 s, and only
  1 of 974 frames exceeded 0.05 s; the p99 was 0.0266 s.
* **The limit is wall time.** §6.1 measures per-frame thread wall time, so any delay of the thread
  counts.

Findings:

* **Not P4 compute.** Offline, the same production processor peaks at 0.019 s on S5 frames.
  Thread-CPU time equals wall time offline, so a 0.095 s frame means the thread waited about 75 ms.
* **Not Python garbage collection.** In a perception-like process (the node's rclpy/px4_msgs/
  sensor_msgs imports plus `P4Processor`, about 62,000 tracked objects), a full collection takes
  0.008 s. A simulated 120 s flight triggered 20 generation-0 and 1 generation-1 collections of about
  0.1 ms each, and no generation-2 collection. `gc.freeze()` made no difference (worst poll 0.019 s
  either way).
* **Not page faults.** There were 0–4 minor faults per poll after warm-up and no major faults.
* **Not GIL contention.** The node uses a single-threaded executor, so `poll` runs alone.
* **Local to the perception process.** At the same instant, `p4_control`'s slowest operation was
  5.4 ms. The VM was not CPU-saturated: WSL has 24 CPUs, and the stack used 4–5 cores. The P3 sensor
  adapter briefly doubled its CPU use (0.6 → 1.37 cores) in the same two seconds.
* **Assessment:** a scheduling or host-level preemption of the perception thread, outside the
  code. It cannot be removed by code changes. The limit stays as written; relaxing it is not
  proposed.
* **Optional diagnostic (implementation, not a contract change):** record per-frame thread-CPU
  time and involuntary context switches next to the wall time in each snapshot. A future spike can
  then be classified as preemption (wall time much larger than CPU time) or as compute. The
  evaluator keeps judging wall time.

## Implementation on approval (offline, then the usual gates)

* **`mission.py`:** a designed supervisor `ABORT` no longer raises out of the mission. It records the
  abort (event and reason) and enters `REQUEST_LAND` with the single LAND (21) request, the same code
  path as `GOAL_HOVER`. Geofence and P2 errors still raise and route to P2 `RECOVERY`.
* **`control_node.py`:**
  * `outcome()` returns `supervisor_abort` when the recorded P4 abort reached `COMPLETE`.
  * A `RECOVERY` remains `failed`.
  * The result keeps `abort_reason`.
* **`p4_evaluate.py`:**
  * S5 and S6 require `COMPLETE`, the expected abort reason, one accepted LAND, and landed and
    disarmed within 45 s wall of the LAND command.
  * F3 is updated.
  * Every unchanged P2/P3 check applies.
* **Tests:**
  * A supervisor abort routes to `REQUEST_LAND` → `COMPLETE` with exactly one LAND.
  * A P2 safety error or geofence breach still routes to `RECOVERY` and fails.
  * The outcome mapping, and the 45 s wall evaluator check.
  * The P2 timing status is `passed` with full requirements for an abort run.
* **Optional:** the perception diagnostic fields above.
* **Rebuild, then the gates:** the package is rebuilt (because `mission.py` and `control_node.py`
  change), then a new commit, Policy A approval, a freeze, and Gates A, B, S and C for the new
  manifest.
