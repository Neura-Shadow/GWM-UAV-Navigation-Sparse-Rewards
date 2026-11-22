# v3-SIM P7-E E8 — development dry run of the P7-E launch path (S2, P4 and P5)

**Status: PREPARED (2026-11-22), awaiting three separate operator approvals. Nothing has been written to `state/`,
frozen or launched.**
* **Authorized so far:** preparation only ("Prepare them").
* **Under:** contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §13 item 5, plan decision E8.
* **What it is:** the first SITL use of the P7-E code accepted in E2 (commit `2b6ee5d`). It is the M4 pattern:
  * S2 flown once with the accepted P4 stack and once with the accepted P5 stack;
  * a development candidate, simulated flight only (SITL, never real hardware);
  * no credit, no block result, no retry.
* **Cost:** about 7 min and about 2.5 GB.

## 1. The three approvals, each separate

**1. Policy A for P7-E development candidates only**
* **File:** `state/p7e-policy-A-approval-e8-development.json` (contract `p3-native-command-policy-approval-v1`).
* **Policy:** Policy A `bf1ef9b1…`. This is the same native-command content as P3–P7, with QGC `4568b585…` and ELF
  `82994283…`.
* **Approval fields:** `development_only: true`; `candidate_phase` "P7-E p7-engineering-validation-v1, E8
  development candidates (S2 launch-path dry run)".
* **What it binds:** the 16 accepted P7-E source and test files (canonical digest `003b4d76…`, commit `2b6ee5d`).
* **Scope:** this dry run only. At most 2 live runs, never a P7-E block, and no transfer to any other candidate.
* **Not a production approval.** The production approval for the real candidate is a later, separate step.

**2. The development freeze**
* **File:** `state/p7e-candidate-dev-2b6ee5d.json`, through `p7e_manifest.py freeze --development` with the absolute
  approval path.
* **What it binds:**
  * the frozen P7 candidate `0799c37e…`, re-verified with P3–P6 inside it;
  * the specs `9b3deacb…`, `15e7f7b4…`, `d742b7af…` and `9f78af75…`;
  * the 16 files and the owner copy `0d1c2d27…`;
  * the runtime identity, with the approval from item 1.
* **Development only.** A development candidate can never fly a block.

**3. The dry-run approval and the run**
* **File:** `state/p7e-dry-run-approval-<sha12>-e8.json`, through `p7e_manifest.py approve-dry-run`.
* **The confirmation phrase** is "dry run P7-E on <sha> in simulated flight".
* **What it allows:** items [S2, p4] and [S2, p5], at most 2 live runs.
* **The run:**

  ```
  cd /tmp && export GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 GWM_ALLOW_PX4_LAUNCH=1 \
    GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1
  /usr/bin/python3 …/px4_gazebo_p7e/scripts/run_p7e_block.py --scope DRY \
    --manifest ~/uav_autonomy/state/p7e-candidate-dev-2b6ee5d.json --authorization <dry-run approval>
  ```

* **What runs:**
  * the FC-2 host check first; nothing is flown if it fails;
  * each flight in a fresh private namespace through `run_p7e.sh`;
  * the unchanged P4 or P5 evaluator;
  * the frozen P7 classifier, then the FC-3 assessment.
* **Stop rules.** All stop rules and FC-1 to FC-3 apply. Nothing is retried. On a stop, the result is reported and
  you decide.

## 2. Read-only pre-check (2026-11-22)

* **The full development candidate computes:** in 20 s against a draft approval kept in `/tmp`, which was removed
  afterwards. It contains:
  * P7 `0799c37e…`;
  * the baselines P3 `1e205fec`, P4 `c110bd30`, P5 `2becfbbb` and P6 `22292775`;
  * the 16 files, the four specs, the owner copy and Policy A.
* **The MC-trace build:** `GWM_PX4_MC_TRACE` selects the frozen build.
* **No live lock** is held, by either P7 or P7-E.
* **The host check passes:** overlay, AC Online, CPU 1, 0, 1, 0, 2 %, no simulator, lock free, 776 GB free.
* **Nothing P7-E exists yet:** no P7-E state file and no `-p7edry-` run.

## 3. Exit criterion (as M4)

**Both flights must:**
* pass the unchanged P4 or P5 evaluator;
* be classified valid goal successes, with no safety event;
* show parity with the accepted S2: the same world, the same D13 label, and the same supervisor lineage.

**The new path must have produced:**
* `p7e/flight-assessment.json`, showing that FC-3 ran;
* the `p7e` summary block;
* a dry-run result whose accounting is complete;
* a released live lock, and the candidate unchanged at the end.

It carries no engineering credit. The result goes to you for acceptance before the production Policy A approval and
the real freeze.
