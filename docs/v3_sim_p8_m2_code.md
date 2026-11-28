# v3-SIM P8-M2: code and tests, offline stages V0–V2 (contract `p8-navigation-recovery-v1` §13 item 3)

**Status: ACCEPTED by the operator on 2026-11-28 ("Accept M2"), together with interpretations I-1 to I-11 of §4.
P8-M2 is COMPLETE** (code commit `c991387`).
* It was authorized with "Authorize P8-M2", then "Continue M2" at the P8-C4 checkpoint.
* The acceptance authorizes nothing to build, run or freeze. Next, each separately authorized, comes P8-M3 (§5).
* No ROS run, no build, no simulator, no candidate, no deletion.
* No P2–P7-E file and no approved P8 spec changed.
* All seven manifests still verify: P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775`, P7 `0799c37e`, P7-E
  `38e23bad` and `e9561f05`.
* The runtime paths (the nodes, runner, launcher and sequencer) are exercised first in P8-M3's development dry run.

The offline stages and the P8-C4 checkpoint are reported in the [M2 checkpoint](v3_sim_p8_m2_checkpoint.md):
* V0: 32 tests;
* V1: `1d6cc0e2…`;
* V2: `912297fb…`.

## 1. Files (new, all in `simulation/px4_gazebo_p8/` or `tests/`)

| File | Lines | SHA-256 | Role |
|---|---:|---|---|
| `ros2_ws/src/gwm_navigation_recovery/…/search.py` | 147 | `61f3eafc…` | the L2 C-space, the BFS, the frontier, the escape target, the segments (once per episode) |
| `…/recovery.py` | 312 | `aaa22e61…` | the manager: the trigger, R1/R2/R3, the frozen L5 and L4/L2 checks, FC-4, the ledger records |
| `…/guard.py` | 47 | `8295efb4…` | R4 |
| `…/wrapper.py` | 57 | `bca8977b…` | `RecoverySupervisor` over the frozen supervisor |
| `…/mission.py` | 55 | `87d18b4c…` | `P8P4Mission`, `P8P5Mission` |
| `…/recovery_ledger.py` | 53 | `8efb115a…` | the parameter hash check and `p8/recovery-ledger.jsonl` |
| `…/p8_p4r_node.py`, `…/p8_p5r_node.py` | 406, 445 | `cdd0cc46…`, `7dd77be1…` | **generated** from the accepted P4 and P5 nodes by `p8_nodes.py` |
| `…/control_node.py`, `package.xml`, `setup.py`, `setup.cfg` | — | — | the entry points `p8_p4r_control` and `p8_p5r_control`; the ament package (not built) |
| `scripts/p8_nodes.py` | 136 | `631997fa…` | the node generator and its check |
| `scripts/p8_manifest.py` | 352 | `445292ca…` | the candidate identity, bindings, authorizations, dry runs and locks |
| `scripts/p8_runner.py` | 314 | `9c929c0d…` | the runtime supervisor over the accepted P7-E runner |
| `scripts/run_p8.sh`, `p8_namespace_owner.py`, `p3_runner.py` | — | owner `0d1c2d27…` | the launcher (the P7-E launcher plus the P8 receipt and install); the owner copy (byte-identical to P3's); the shim |
| `scripts/run_p8_block.py` | 252 | `379dbb8a…` | the block sequencer: FC-1..FC-3 reused, FC-5..FC-7 added |
| `scripts/p8_build.py`, `build_p8.sh` | 56, 18 | — | the build (an M3 step; **not run**) |
| `validation/p8_h10_check.py` | 135 | `937e1557…` | H10 |
| `replay/p8_v1_replay.py`, `replay/p8_v2_perception.py` | 219, 123 | — | V1, V2 |
| `analysis/p8_report.py` | 348 | `1bace23d…` | the report plan `a313a034…` as code |
| `tests/test_px4_gazebo_p8_recovery.py`, `…_h10.py`, `…_integration.py` | 287, 94, 254 | — | V0 (32), H10 (9), integration (17) |

**Tests:**
* the P8 tests: **69 pass** (with the 11 specs tests);
* the accepted P7-E tests: **76 still pass**;
* `p8_nodes.py check`: the generated nodes equal their declared edits.

## 2. The nodes: provable replicas

* **Why replicas.** The accepted P4 and P5 nodes define their node class inside `main()`, so they cannot be
  subclassed. P5's node is itself a reviewed replica of P4's (P5 decision D5).
* **How they are generated.** `p8_nodes.py` generates each P8 node from the frozen file by a declared list of edits.
  * Every anchor must occur exactly once.
  * The frozen sources must equal their accepted hashes: P4 `56af5cbd…` (bound by `c110bd30`) and P5 `a35bc385…`
    (bound by `2becfbbb`).
  * A test regenerates both nodes and requires byte equality. It also checks that the only removed lines are the
    relative imports and the mission constructor.
* **The edits:**
  * a generated-file header;
  * absolute imports;
  * `--recovery-params` and `--recovery-params-sha256`, checked before any ROS node exists;
  * `P8P4Mission` / `P8P5Mission`;
  * the recovery ledger, opened next to the accepted streams;
  * the pending records written after the tick's dispatch and events;
  * the ledger's fsynced summary in `controller-result.json` under `p8`.
* **Unchanged:** the node name, topics, QoS, guards, trace spans, evidence files and the controller identity
  (`p4_control`, `p5_control`). So the accepted evaluators and graph checks apply as they are.

## 3. The runner, launcher and sequencer

* **Runner.** `P8Flight` extends the accepted `P7EFlight`, and through it the frozen P7 runner and the accepted runners.
  It overrides five hooks:
  * `p7_init`: the V4 layouts are added through the frozen P7 mapping;
  * `preconditions`: the P8 manifest, authorization and schedule;
  * `prepare_run`: `-p8-`/`-p8dry-` naming and the run-local `p8_recovery.json`;
  * `prepare_world`: the accepted generation, then the P8 world hash (C6);
  * `p4_controller_command`: the accepted command with only its executable replaced, plus the recovery arguments.

  The P3 runner's own `args.variant` stays `None`; the P8 variant travels as `p8_variant`.
* **Launcher.** It is the P7-E launcher with the P8 build receipt required and the P8 install sourced.
* **Sequencer:**
  * reused by import from the accepted P7-E sequencer: FC-1 (`Tracker`, `sequence`, `accounting`), FC-2 (`p7e_host`),
    FC-3 (`p7e_assess.assess`), and the frozen `BlockRules`, `Ledger`, `run_sequence` and evidence scripts;
  * added per flight, as `assess`: the frozen classifier, FC-3, then H10. H10 failures map to FC-5
    (safety-unverified) or FC-6 (a safety event: a stale tick, the budget, the window), and both stop the block.
  * **FC-7:** drift of any P8, P7-E or P7 input.
* **Manifest:**
  * it binds P7-E `e9561f05` through `p7e_manifest.verify_current`, so P7 and P3–P6 too;
  * the four approved spec hashes, and the V1/V2 records, which must record a pass (H11);
  * the P8 tree and tests, the build receipt (package hash), the owner copy, and a Policy A approval naming P8.

## 4. Interpretations for the operator's acceptance

None of these adds or relaxes a limit.

| ID | Interpretation |
|---|---|
| I-1 | **The joint time budget:** rotation and translation together fit the window left. R1 episodes scan first, R3 after; R2 episodes reserve the escape first, and R2 sectors fill the rest. |
| I-2 | **L4 for recovery translations:** a translation is emitted only when its heading lies within the frozen ±25° of the vehicle yaw; otherwise the manager yaws first (under L5). This is stricter than S1's wording, and matches the frozen planner. |
| I-3 | Recovery replaces only a HOLD. A frozen SCAN of the tick goes first. It never acts on a frozen-yaw or a P5 policy-failure tick. |
| I-4 | **R2 needs reachable frontier cells** (contract §4). A hover point that is not L2-usable (slot 7 P4 in V1) therefore gets no plan. |
| I-5 | The generated nodes keep the accepted node name and controller identity. P8's identity is the ledger header, the `p8` block of the controller result, and the run summary. |
| I-6 | **The recovery ledger** is line-buffered and fsynced at close, which keeps fsync out of the 20 Hz loop (S7). A missing or unparseable ledger is FC-5. |
| I-7 | **R4** swaps the frozen `P4Ranker` into the P5 supervisor (`inner.ranker`) for 20 s, then restores the policy ranker. |
| I-8 | **H10 violations:** a stale tick, a budget overrun or an action outside the window is FC-6 (a safety event); any other violation is FC-5 (safety-unverified). |
| I-9 | **V2:** a frame missing from a recording is reported. The pass needs complete recordings identical and every final-HOVER snapshot identical. |
| I-10 | **P8's H9** adds H10 reproduction and the P8 world hashes. The assessment's reproduction compares the FC-3 fields. |
| I-11 | **The run summary** keeps the `p7` block the frozen classifier reads, plus a `p8` block. |

## 5. What M2 does not do (M3, each separately authorized)

* build the ament package (`build_p8.sh --build`, which writes `state/p8-built.json`);
* the P8 Policy A approval;
* the development candidate and its 2-flight dry run, the first runtime use of the nodes, runner, launcher and
  sequencer;
* the real freeze.
