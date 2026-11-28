# v3-SIM P8-M3: build, development Policy A, development candidate and dry run (contract §13 items 4–6)

**Status: P8-M3 COMPLETE (2026-11-28).** All items were done, each under its own authorization.
* Items 1–4 were done and the dry run was accepted (§5–§7).
* The production Policy A and the real freeze (§13 items 5–6) were done (§8.3).
* **The real P8 candidate is manifest `fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`.**
* The V3–V5 blocks come next, each under its own authorization. None is authorized yet.

*(Status before the authorizations: PREPARED, 2026-11-28, operator "Prepare M3".)*

## 1. The items

| # | Item | What happens | Writes | Runtime |
|---|---|---|---|---|
| 1 | **Build** the P8 package | `bash simulation/px4_gazebo_p8/scripts/build_p8.sh --build` (WSL). It mirrors `gwm_navigation_recovery` (package hash `1a78c6ba…`) into `~/uav_autonomy/p8_ws/1a78c6ba…/`, runs `colcon build` with the P2–P5 installs sourced, and writes `state/p8-built.json` | the mirror, the install, the receipt | none (no simulator, no node) |
| 2 | **Development Policy A** for P8 | writes `state/p8-policy-A-approval-dev.json` (§2): Policy A `bf1ef9b1…`, **`development_only: true`**, binding the 37 P8 source and test files (canonical `f6a04d32…`) | that approval | none |
| 3 | **Freeze the development candidate** | `p8_manifest.py freeze --development --policy-approval ~/uav_autonomy/state/p8-policy-A-approval-dev.json --output ~/uav_autonomy/state/p8-candidate-dev-<commit>.json`, with the absolute approval path, then `verify` from `/tmp` | the development manifest | none |
| 4 | **The development dry run** | the dry-run approval, then the run: `p8_manifest.py approve-dry-run` (§3), then `run_p8_block.py --scope DRY`, launched detached with a persistent log | the approval, 2 runs, the dry-run result | **2 SITL flights**, simulated flight only, no credit |

**What the dry run is for.** It is the first runtime use of the P8 nodes, runner, launcher, sequencer and H10. Both
flights are chosen to make recovery act:

| Flight | Layout | P7-E then | What it exercises |
|---|---|---|---|
| 1 | `p7c-4000360` (V3, slot 18, partial wall) | both stacks: an F1 near-goal HOVER | P8-P4R; R1 terminal-observation scans; the generated P4 node; the recovery ledger and H10 |
| 2 | `p7c-4000375` (V3, slot 12, mixed) | both stacks: an F2 dead end; P5 oscillated in yaw for 90 s | P8-P5R; R2/R3; the R4 guard; the generated P5 node |

Each flight takes about 3–4 min, so the run takes about 8 min and about 3 GB. A dry run earns no credit and never
counts for V3.

**Exit criterion:** the same shape as E8 for P7-E.
* both flights run to their end through the P8 path;
* the accepted P4 and P5 evaluators run;
* the frozen classifier records valid flights;
* FC-3 is valid;
* **H10 passes:** every recovery action reproduces its frozen check, with no FC-5 or FC-6;
* both worlds equal their approved hashes;
* the controller result carries the `p8` ledger summary (fsynced);
* nothing stops, and no simulator outlives a flight.

Goal success is not part of the criterion: the dry run tests the path, not the effectiveness. Any failure is reported
as found; nothing is re-run automatically.

## 2. The development Policy A approval to be written (item 2)

```json
{"contract": "p3-native-command-policy-approval-v1", "policy": "A",
 "policy_sha256": "bf1ef9b1080a52eab1673ac2a1ac6e20d30cfa7fd85b703cae8433dc55f7b3b0",
 "approved": true, "approved_by": "Neura-Shadow", "approved_utc": "<at writing>",
 "approval_channel": "operator chat answer in Claude Code session, <date>: \"<answer>\", answering P8-M3 item 2, the request to approve Policy A (the same native-command content as P3-P7-E) for P8 development candidates only: the P8-M3 dry run of 2 flights, simulated flight only, at most 2 live runs, no credit, no retry",
 "candidate_phase": "P8 p8-navigation-recovery-v1, M3 development candidates (2-flight dry run)",
 "candidate_p8_source_files": 37,
 "candidate_p8_source_files_sha256": "f6a04d329c8cf98956f71dfc7db555c0997f2e1f21c0eaff737310494b7a0be4",
 "candidate_repository_commit": "<HEAD at writing>", "development_only": true,
 "allowed": "<copied unchanged from the P7-E approvals>",
 "qgc_source_commit": "4568b585648e8c6065cca6c650b52731c1d3baa3",
 "qgc_unattended_elf_sha256": "829942834b71bbd7f30967cb32151e7a32bc1a7fbb9609fd15e2f4dde3e46d03",
 "scope": "P8 development candidates whose 37 P8 source and test files have canonical digest f6a04d32…: the P8-M3 dry run only (p7c-4000360 with P8-P4R, p7c-4000375 with P8-P5R), simulated flight only, at most 2 live runs, no credit, no retry; never a P8 block and no transfer to any other candidate"}
```

## 3. The dry-run approval to be written (item 4)

`p8_manifest.py approve-dry-run <development manifest>`, with:
* `--operator Neura-Shadow`;
* `--item p7c-4000360 p8_p4r` and `--item p7c-4000375 p8_p5r`;
* `--live-runs-max 2`;
* the confirmation phrase "dry run P8 on <sha> in simulated flight".

It writes `state/p8-dry-run-approval-<sha12>.json`. The run is launched detached, with its PID and log under
`~/uav_autonomy/logs`. A watcher waits for it, and the app keeps the computer awake.

## 4. Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| ROS Jazzy, `ros_ws`, colcon | present; colcon lists `gwm_navigation_recovery` (ros.ament_python) |
| Build receipts P2–P5, MC trace, QGC unattended, DDS agent | all present |
| `~/uav_autonomy/p8_ws`, `state/p8-built.json`, any `state/p8-*` | absent |
| P8 specs; V1/V2 records; owner copy | approved hashes; both passed; `0d1c2d27…` (= the P3 owner) |
| P7-E binding | `e9561f05…` verifies (P7 and P3–P6 inside) |
| P8 source and test files | 37, canonical `f6a04d32…`; package hash `1a78c6ba…` |
| The build identity | refused, as expected before item 1 ("not built") |
| Live locks; host check (2 flights); MC-trace build | none; passed (CPU 1.0 %, 687 GB free); selected |
| Generated nodes | `p8_nodes.py check`: ok |
| Git | HEAD `9b62523`; clean apart from the untracked `.codegraph/` |

## 5. Done (operator, 2026-11-28: all four items "Recommended")

| # | Item | Record |
|---|---|---|
| 1 | Build | `state/p8-built.json`: package `1a78c6ba…`, install `~/uav_autonomy/p8_ws/1a78c6ba…/install`, both executables installed. `--help` of both, under the launcher's sourced installs, imports and parses (no node started). |
| 2 | Development Policy A | `state/p8-policy-A-approval-dev.json` (`9bf4a9f6…`): `development_only: true`, 37 files, canonical `f6a04d32…`, commit `f76f4f5`. It passes the P8 phase check and the accepted `load_policy`. |
| 3 | Development candidate | `state/p8-candidate-dev-f76f4f5.json` (file `616dcdb6…`), **manifest `0a31ecd02f4441f056c169555cc9322d908065bb7063d02b158150de92a63810`**. Verified from `/tmp`: P7-E `e9561f05`, build `1a78c6ba`, the four specs, V1/V2, the absolute approval path. |
| 4 | Dry run | approval `state/p8-dry-run-approval-0a31ecd02f44.json` (`2fe51c17…`); launched detached (PID 496928, log `~/uav_autonomy/logs/p8-dryrun-0a31ecd02f44.log`); result `state/p8-dryrun-0a31ecd02f44-20261128T025230Z.result.json` (`79a4fd0b…`) |

## 6. The dry run (2 flights, 7.8 min; no credit)

**Exit criterion: MET.**
* **Path:** both flights ran to their end through the P8 path (runtime exit 0, summaries passed).
* **Evaluation and classification:** the accepted evaluators ran; the frozen classifier records both flights valid;
  FC-3 is valid.
* **H10 passed on both** (231 and 286 actions reproduce; no FC-5 or FC-6).
* **Worlds and ledgers:** both worlds equal their approved hashes. Both controller results carry the fsynced `p8`
  ledger summary (238 and 294 records, parameter hash `dee90d51…`), and the nodes reported their accepted identities
  (`p4_control`, `p5_control`).
* **Clean finish:** not stopped, accounting complete (2 `flown_valid`), no drift, the lock released, no simulator left.

| | Flight 1 | Flight 2 |
|---|---|---|
| Variant, layout | P8-P4R, `p7c-4000360` (V3 slot 18) | P8-P5R, `p7c-4000375` (V3 slot 12) |
| P7-E then | both stacks: an F1 near-goal `hover_timeout` | both stacks: an F2 `hover_timeout`; P5 oscillated in yaw for 90 s |
| **Outcome** | **goal success** (the evaluation passed, no event) | `hover_timeout`, a safe abort and landing |
| Recovery | 3 HOVER episodes, all **R1** (231 yaws). Each was followed by the frozen supervisor's own exit to AVOIDING; then RESUME → NORMAL → goal. | 3 episodes: R2 (240 yaws), R3 (46 alignment yaws). One **R3 translation refused** by the frozen `collision_check` (`occupied`, on the newer snapshot: FC-4). No recovery translation was executed. |
| Clearance; hover time | 0.387 m; 13.2 s | 0.395 m; 22.4 s |
| R4 guard | — | **not exercised**: 1,195 `yaw_to_l4` interventions, all while the vehicle progressed |
| Search wall time | — | 3.4–12.1 ms per episode |

**Finding F-DR1: the accepted P5 evaluator reports `policy:ledger:decision_mismatch` on flight 2** (an integrity item,
not a safety item; validity unaffected).
* **The cause:** the evaluator requires every policy-ledger record to equal the executed decision on `sim`, `action`
  and `state`. By design (S4), recovery replaces the policy supervisor's HOLD *after* the P5 ledger records it.
* **The reconciliation is exact:** all 286 mismatches are the 286 recovery actions, one to one. They differ only in
  `action` (HOLD in the ledger, YAW_TO executed), never in `sim` or `state`.
* **Consequence:** every P8-P5R flight in which recovery acts will carry this item. The accepted P5 evaluation status
  will read "failed" on it alone.
* **Decision D-M3-1, for the operator:**
  * (a) treat it as an expected integrity item, and add this exact reconciliation to `p8_report.py` (R-P8-10)
    **before the real freeze**: a small, separately authorized code change with a test;
  * (b) leave the code as it is, and explain it in the report text only.

## 7. Decisions (operator, 2026-11-28: "Accept, take (a)")

**The dry run is ACCEPTED, so items 1–4 of P8-M3 are complete.** The development candidate `0a31ecd0…` is retired.
* It flies nothing more.
* After the change below it drifts by design: `p8_report.py` and a test changed.

**D-M3-1 (a) is DONE: the reconciliation is in the report code.**
* **New:** `analysis/p8_report.py` `ledger_reconciliation` and `r10`. For every P8-P5R flight, R-P8-10 now carries
  `p5_ledger_reconciliation`.
* **The rule:** the item `policy:ledger:decision_mismatch` counts as explained only if all three hold:
  * the policy-ledger and decision counts are equal;
  * every mismatch differs in `action` alone;
  * the mismatching ticks equal the recovery `action` ticks, as a set.
* Anything else is reported as unexplained.
* **Tests:** 5 new tests in `tests/test_px4_gazebo_p8_integration.py`.
  * An exact match is explained.
  * An extra mismatch, a differing `state` and a count difference are each unexplained.
  * The real dry-run P5 flight reconciles 286 of 286.
  * **74 P8 tests pass.**
* **What changes for the candidate:** the P8 source set is still 37 files, with **canonical digest `10eb4660…`**,
  which the production Policy A must bind. The package hash is unchanged (`1a78c6ba…`), because the report code is
  not part of the ROS package, so the build stays valid.

**Next, each separately authorized:**
* the production Policy A (`development_only: false`, binding `10eb4660…`);
* the real freeze;
* then the V3, V4 and V5 blocks.

## 8. The production Policy A and the real freeze (§13 items 5–6): DONE (2026-11-28; prepared under "Prepare them")

Two items, each needing its own answer. Neither flies anything.

| # | Item | What happens | Writes |
|---|---|---|---|
| 5 | **Production Policy A** for the real P8 candidate | the approval of §8.1, **`development_only: false`**, binding the 37 P8 files with canonical `10eb4660…` | `state/p8-policy-A-approval-<commit>.json` |
| 6 | **The real freeze** | `p8_manifest.py freeze --policy-approval ~/uav_autonomy/state/p8-policy-A-approval-<commit>.json --output ~/uav_autonomy/state/p8-candidate-<commit>.json` (absolute approval path), then `verify` from `/tmp` | the real P8 candidate manifest |

**After the freeze,** only the real candidate can fly the 3 blocks of scope `p8-validation-60`.
* The blocks are V3, V4 and V5, 20 flights each, and **each needs its own authorization**: the phrase "authorize P8
  p8-validation-60 block <n> <sha>", then a detached launch.
* From the freeze on, no P8 bound file may change, because drift stops a block (FC-7).

### 8.1 The production approval to be written (item 5)

It has the same fields as the development approval (§2), except:
* `"development_only": false`;
* `"candidate_phase": "P8 p8-navigation-recovery-v1, the real (non-development) candidate"`;
* `"candidate_p8_source_files": 37`, with `"candidate_p8_source_files_sha256": "10eb46600d7fdd3aae7834fae7cf84f967c4bf00efd9faf3d68f7629f66c0bde"`;
* `"scope"`: the real P8 candidate whose 37 files have that digest. It covers the 3 blocks of `p8-validation-60`, each
  separately authorized, in simulated flight only. It never transfers to a development candidate or any other
  candidate.

### 8.2 Read-only pre-check (2026-11-28)

| Check | Result |
|---|---|
| P8 source and test files | 37, canonical `10eb4660…` |
| Specs; V1/V2 records; owner | approved hashes; both passed; `0d1c2d27…` |
| Build identity | valid: receipt `ba32f23f…`, package `1a78c6ba…` equal to the source |
| P7-E binding | `e9561f05…` verifies (P7 and P3–P6 inside) |
| Generated nodes | `p8_nodes.py check`: ok |
| Development candidate `0a31ecd0…` | drifts (`p8_source_files`), as expected after D-M3-1; retired |
| Live locks; existing production approval or candidate | none; none (only the development records exist) |
| Git | HEAD `148bdf3`; clean apart from `.codegraph/` |

### 8.3 Done (operator, 2026-11-28: item 5 "Approve (Recommended)", item 6 "Freeze it (Recommended)")

| # | Item | Record |
|---|---|---|
| 5 | Production Policy A | `state/p8-policy-A-approval-2a0ae57.json` (`befd23a3…`), written with exclusive create. It has `development_only: false`, the real-candidate phase and 37 files with canonical `10eb4660…` (asserted at write time), and binds commit `2a0ae57`. It passes the P8 phase check (`check_policy_approval(…, False)`) and the accepted `load_policy`. Policy `bf1ef9b1…`; `allowed` and the QGC fields are unchanged from P7-E. |
| 6 | Real freeze | `state/p8-candidate-2a0ae57.json` (file `6a8c4c17…`), **manifest `fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a`**. Frozen with the absolute approval path and without `--development`. `verify` from `/tmp`: ok. |

What the manifest holds:

| Field | Value |
|---|---|
| `development` | `false`; `engineering_credit_possible: true`; `execution_allowed_without_authorization: false` |
| `frozen_git_head` | `2a0ae57` |
| `scope`, `blocks` | `p8-validation-60`, 3 (1 = V3, 2 = V4, 3 = V5) |
| P7-E binding | `e9561f05…` (with the P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb`, P6 `22292775` and P7 `0799c37e` baselines) |
| Build | receipt `ba32f23f…`, package `1a78c6ba…` |
| Specs | layouts `62ae950c…`, schedule `df90cf7b…`, recovery `dee90d51…`, report plan `a313a034…` |
| Offline records | V1 `1d6cc0e2…`, V2 `912297fb…` |
| Namespace owner | `0d1c2d27…` |
| P8 source and test files | 37 |

After the freeze, **74 P8 tests pass**.

**From now on, no P8 bound file may change.** Drift stops a block (FC-7). The development candidate `0a31ecd0…` and
its approval stay retired.

**Next, each separately authorized:** the blocks of `p8-validation-60`, in order.
* Block 1 = V3 (10 layouts × P8-P4R and P8-P5R, 20 flights).
* Block 2 = V4, then block 3 = V5.
* Each needs its own phrase, "authorize P8 p8-validation-60 block <n>
  fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a", then a detached launch.
* A block starts only if the previous block's result exists and is unstopped, with every planned flight accounted for
  and its ledger intact (FC-1).
* Then comes the P8 report.
