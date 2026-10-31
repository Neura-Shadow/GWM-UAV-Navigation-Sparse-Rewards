# P5 contract `p5-acceptance-v1`, revision 2 — APPROVED

Status: proposed and approved by the operator on 2026-10-31 ("Approve revision 2"). It replaces
revision 1's Gate A items 4 and 5 and clarifies the live checkpoint-mismatch start and Gate B. Nothing here
changes the P3/P4 manifests, P4 candidate geometry or safety rules, any numeric limit, the model,
weights or normalization, PX4 control ownership, or the gate sequence and authorization rules.

## Why

Revision 1 Gate A items 4 and 5 require the policy to rank, and to fail, on ground snapshots.
On the ground, the P4 planner never offers a progress candidate. Before the in-flight map scan, the
first 0.4 m ahead (the camera's near blind zone) and the sides of the 1.0 m inflated corridor are
unobserved, so every candidate is `unknown` or `occupied`. P4 is right to choose SCAN.

The evidence: all 600 in-window snapshots of the accepted P4 Gate A readiness, G2, G4 and G6 runs
have zero progress candidates, and P4's decision is SCAN on every one. With nothing allowed, the
policy is never invoked, so items 4 and 5 can never pass as written.

## Changes

### §9 Gate A, item (4) — replaced

> (4) **Ground policy evaluation, G2, G4 and G6** (inference correctness and safety masking, not
> AI-directed movement). For every in-window recorded snapshot, the evaluator:
>
> 1. builds the exact model input with the frozen production adapter;
> 2. runs the frozen model under the controller's pinned interpreter;
> 3. requires finite scores that are bit-identical on repeat, with adapter build plus inference
>    p99 ≤ 5 ms and max ≤ 10 ms;
> 4. requires the allowed mask to equal the P4 collision checker's progress set;
> 5. requires the P5 decision (planner replica with the policy ranker) to equal P4's. When no
>    progress candidate exists, that decision is SCAN or HOLD and the policy is not invoked;
> 6. fails on any policy-induced TRANSLATE or YAW_TO, or any executed segment through unknown or
>    occupied cells.

### §9 Gate A, item (5) — replaced

> (5) **Flight-snapshot policy-failure replay** (offline, no simulator, no PX4 command path). The
> evaluator uses the retained, accepted P4 S2 flight evidence as a read-only source.
>
> * **Primary source:** the Gate B smoke run `20261030T141506Z-p4-flight-S2-9b8ef2b0`.
> * **Second source:** the Gate C qualification-1 run `20261030T144136Z-p4-flight-S2-abeea2d5`.
> * **Integrity:** each source must be listed as passed in the accepted `c110bd30` gate-result
>   file, and its `p4-evaluation.json` must still match the recorded SHA-256. The digest of every
>   source file read is recorded.
>
> **Snapshot selection and validation:**
>
> * A TRANSIT tick qualifies only if all of these hold:
>   * it rebuilds from the run's own records: the local-map row with the decision's snapshot
>     time, the vehicle yaw from the sample of the same tick, the target and vehicle from the
>     decision, and the traversed cells from the recorded positions;
>   * the unchanged P4 candidate generator reproduces the recorded P4 plan field for field (valid
>     count, progress count, best candidate);
>   * it has at least one progress candidate.
> * The first qualifying tick in an avoidance state is replayed. At least one is required. If none
>   qualifies, the item fails and is reported as a limitation.
> * Recorded data is never modified. The candidate mask is never altered, and no admissible path
>   is made up.
>
> **Checks on that tick:**
>
> * The frozen adapter's allowed mask equals P4's progress set.
> * The true frozen model yields finite scores and a selection inside the mask.
> * Each of these faults is replayed through the production `PolicyRanker` and `PolicySupervisor`,
>   starting from the recorded supervisor state:
>
>   | Fault | Required classification |
>   |---|---|
>   | Missing checkpoint | `checkpoint_missing` |
>   | Checkpoint hash mismatch | refused at load (`checkpoint_sha256_mismatch`), then `checkpoint_missing` |
>   | Non-finite scores | `output_not_finite` |
>   | Inference timeout | `timeout` |
>   | Invalid output dimensions | `output_shape` |
>   | Invalid candidate selection | `selection_outside_mask` |
>
> * **Required response for every fault:** ticks 1 and 2 HOLD with the target and yaw frozen and
>   the state unchanged. Tick 3 is ABORT with reason `policy_unavailable`.
> * The replay module has no ROS or PX4 import and no publisher.

### §9 Gate A, item (6) — clarified

> The live start with a mismatched checkpoint (the bound file plus one byte) must refuse with
> `checkpoint_sha256_mismatch`. The refusal must happen before any node, run file or publisher
> exists.
>
> * The controller's nonzero exit (code 1) is accepted **only** in this declared `policy_fault`
>   scope, and only as the single `ros-controller` cleanup entry.
> * Every other process failure or cleanup violation still fails, in every scope.

### §9 Gate B — clarified, unchanged in structure

> One separately authorized S2 SITL smoke flight. It must show genuine frozen-policy participation
> in ranking, with the §6 participation, failure, mask and fallback limits met. It must also show
> shield-safe execution, the detour, goal completion and landing.
>
> Replay checks recorded after Gate B are additional evidence only. They never replace item (5).

### `p5_validation.yaml`

* **Gate A rows:**
  * `p3_readiness`;
  * `p5_readiness`;
  * `fixture` G2, G4 and G6, each with item (4);
  * **`flight_snapshot_replay` S2**, a new offline row with role `policy-failure-replay`. The
    sequencer runs it without a namespace, simulator or PX4, after the fixtures;
  * `policy_fault` `checkpoint_mismatch`.
* **`policy_replay.failure_cases`:** `missing_checkpoint`, `hash_mismatch`, `nan_output`,
  `timeout`, `output_shape` and `selection_outside_mask`.
* **New `policy_replay.sources`:** the two run names and the accepted P4 Gate B and C result
  files.
* All numeric limits are unchanged.
