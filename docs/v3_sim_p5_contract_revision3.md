# P5 contract `p5-acceptance-v1`, revision 3 — APPROVED

**Status.** Proposed in the [CPU recovery plan](v3_sim_p5_cpu_recovery_plan.md) §10, and approved by the
operator on 2026-11-06 ("Approve revision 3").

**What it changes.** Exactly one implementation detail of the observation adapter. It changes no number in
the contract, no model, weight, schema or normalization, no candidate geometry, no mask, selection,
shield, supervisor or timeout rule, no PX4 control ownership, no P2, P3 or P4 file, and no gate or
authorization rule.

## Why

The first Gate C qualification flight of manifest `db4c4779…` failed `policy:inference_max` on one tick:
10.84 ms against the 10 ms maximum (plan §13.13). That result is permanent and is preserved.

* **Root-cause evidence** (plan §13.14-13.15, [benchmark](v3_sim_p5_inference_benchmark.md), recovery
  plan §5-§9.1):
  * The policy's own computation is 0.2-0.4 ms per decision. The polar occupancy profile is about 60 % of
    it.
  * Slow calls fall into two classes, both outside the computation:
    1. guest-scheduler preemption, when the WSL guest is oversubscribed;
    2. CPU-charged slowness with no run-queue wait. Offline this reached 16.6 ms on the production path
       with no load.
  * In three instrumented S2 SITL flights the guest was never oversubscribed during decisions.
* **Remaining uncertainty.**
  * Gate C itself was not instrumented. Its breach is attributed to class 2 by elimination only.
  * Class 2 cannot be removed from inside the policy code; one offline call on the optimized path reached
    9.91 ms.
  * The change shortens the decision path, and with it the window in which a stall can land. It does not
    bound stalls.

## Changes

### Implementation (§3.2, polar profile)

`gwm_policy_control/observation_adapter.py`, function `polar_profile`:
* **Before:** a per-heading loop.
* **After:** one array expression over all headings.
  * `math.cos` and `math.sin` are evaluated per heading exactly as before.
  * Per element, the same IEEE-754 operations run in the same order: multiply, add, subtract, divide,
    floor, bounds test, cell and traversed lookup, first-hit search.
  * The output is therefore bit-identical.
* The signature, return value and docstring contract are unchanged. The function stays pure and stateless.

**Requirement.** The function must be bit-identical to the revision 1 loop on every input. This is
enforced by a regression test that keeps the revision 1 loop as an oracle.

### Unchanged

| Area | Unchanged |
|---|---|
| Model | checkpoint `p5-ranker-ee006a80…`; weights; architecture 46 → 64 → 64 → 1 float64 |
| Observation | schema `(26, 46)`; feature scaling; candidate order and slots; occupancy and unknown interpretation |
| Decision | allowed mask; masked argmax and P4 tie order; shield L4/L5; supervisor; policy-failure HOLD and ABORT |
| Limits | inference p99 ≤ 5 ms and max ≤ 10 ms (wall: feature build + forward + selection, as measured today); 20 ms runtime timeout; 0 failure ticks; participation 1.0; 0 outside mask; 0 fallbacks; all P2, P3 and P4 limits |
| Runtime | controller interpreter `/usr/bin/python3` with NumPy 1.26.4 and its reference BLAS; `OPENBLAS_NUM_THREADS=1`. No runtime or host prerequisite is added |

### Identity

The change produces a **new candidate**. The required sequence:
1. a new commit;
2. `build_p5.sh --build`, giving a new `state/p5-built.json` and P5 package hash;
3. new `p5_source_files`, and `contract_revision` in `p5_validation.yaml` naming revision 3;
4. **Policy A approval for the new commit**;
5. `p5_manifest.py freeze`, giving a new manifest SHA;
6. Gates **A → B → S → C**, each separately authorized and bound to the new SHA, one run each, no
   retries.

**No acceptance credit transfers from `db4c4779…`.** Its Gate A, B and S results and its failed Gate C
remain preserved as historical evidence.

### Required regression before the freeze

1. The revision 1 oracle test, on synthetic grids and fuzzed poses, headings, traversed masks and starts
   outside the grid.
2. The recovery-plan §4 differential against the revision 1 implementation:
   * the 4,267 exact flight tensors;
   * the 5,713 reproduced flight ticks (features, masks, scores, choices, decisions, sequential supervisor
     replay);
   * the 2,945 ground snapshots;
   * 200,000 polar fuzz cases.
3. The P5 offline suites, the ROS harness (P4 + P5) and the Windows full suite.
4. P3 and P4 manifest verification, and the drift guard proving no P2, P3 or P4 file changed.
5. A P5 package build receipt matching the new source.
