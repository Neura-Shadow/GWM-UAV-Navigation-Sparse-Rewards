# v3-SIM P7 M2 — runner, manifest, host check and block sequencer (offline)

**Status: built and tested offline (2026-11-21); awaiting operator review.** This is M2 of the
[P7 plan](v3_sim_p7_plan.md) (§7), authorized by the operator on 2026-11-21 ("Authorize M2").

**What M2 did and did not do:**
* Nothing flew, and no simulator, namespace or flight was started.
* No P2–P6 file changed. The P3–P6 manifests verify.
* Nothing is frozen or authorized. The code was committed locally as 413ceb0, with this report after it.

**Exit criterion (plan §7):**

| Criterion | Status |
|---|---|
| Byte-identical worlds for S1–S6 through the P7 path | **met** offline: S1–S6 through the P7 mapping and runner equal the hashes every accepted P4 and P5 flight recorded, and every one of the 260 P7 layouts gives its frozen world (C6) |
| Offline tests | **met**: 48 P7 tests pass (31 for M2, 17 for M1) |
| P3–P6 still verify | **met** |

**For the operator to review:** the host-check numbers (§4) and the design choices listed in §6.

## 1. Files

| File | Role | SHA-256 |
|---|---|---|
| `simulation/px4_gazebo_p7/scripts/p7_manifest.py` | candidate identity, P3–P6 binding, approved specs, authorizations, live lock | `fd6ae2cf…` |
| `simulation/px4_gazebo_p7/scripts/p7_runner.py` | `P7P4Runner` and `P7P5Runner` over the accepted runners | `d7a5fd0f…` |
| `simulation/px4_gazebo_p7/scripts/p7_host.py` | read-only pre-block host check | `82af524d…` |
| `simulation/px4_gazebo_p7/configs/p7_host_check.yaml` | its definition (canonical `9f78af75…`) | `70bb4456…` |
| `simulation/px4_gazebo_p7/scripts/run_p7_block.py` | block sequencer, ledger, replacement and stop rules | `bc9ad099…` |
| `simulation/px4_gazebo_p7/scripts/run_p7.sh` | one flight in a fresh private namespace (P6's launcher, renamed only) | `888cb052…` |
| `simulation/px4_gazebo_p7/scripts/p7_namespace_owner.py` | byte-identical P3 owner copy (the hash the QGC build pins) | `0d1c2d27…` |
| `simulation/px4_gazebo_p7/scripts/p3_runner.py` | owner shim, dispatches to `p7_runner.main` | `14876260…` |
| `tests/test_px4_gazebo_p7_runner.py` | 31 offline tests | `3f9c0a81…` |

All of these, and the M1 specifications, become hash-bound in the P7 manifest at freeze.

## 2. Manifest and authorizations (`p7_manifest.py`)

**What the P7 candidate binds:**
* **P3–P6:** through the accepted `p6_manifest.verify_current()`. That call recomputes P6, P5 inside it, and P3 and
  P4 inside P5. The P4 baseline file is also checked against `c110bd30` (C1).
* **What the accepted runners read:** the P4 side's P3 baseline, and the P5 model, schema, runtime, build and
  baselines, copied from the verified manifests (C2).
* **The three M1 specifications, by canonical hash.** `specs_identity()` refuses any hash other than the approved
  ones, so a changed specification can never be frozen (C7).
* **The host-check definition.**
* **The P7 sources and tests.**
* **The owner copy**, which must be byte-identical to the P3 owner.
* **The runtime identity with a P7 Policy A approval.** An approval for P3–P6 never transfers (D12).

**Freezes and authorizations:**
* **Development candidates** run dry runs only (M4).
* **Real candidates** run the pilot and the confirmatory blocks.
* **A block authorization** binds the scope, the block number, the manifest and the approved schedule hash, with
  `max_attempts` 1. It is consumed by a permanent marker.
* **A confirmatory block** also needs the pilot rule's schedule selection record (200 or 250 pairs) for that
  manifest.
* **A dry-run approval** names each flight as [item, controller], with a cap on live runs.
* **The live lock** refuses a second sequence or a freeze while a block runs.
* **The operator's CLI commands** check their confirmation phrases, as in P6: `freeze`, `verify`, `authorize`
  (`authorize P7 <scope> block <n> <sha>`) and `approve-dry-run`.

## 3. Runner (`p7_runner.py`)

**Reuse.** `P7P4Runner(P4Runner)` and `P7P5Runner(P5Runner)` share one mixin. A test asserts that everything else
is the accepted code, unchanged: start-up, isolation, PX4/Gazebo/QGC/DDS, sensors, perception, controller,
supervisor, S6 injection, shutdown and finalization.

**What the mixin overrides:**
* **`preconditions`:**
  * the accepted flight gating;
  * the gates, namespace, lock and disk reserve;
  * `p7_manifest.verify_current`;
  * then either the block authorization, its marker and the flight's slot in the frozen schedule (item and
    controller), or the dry-run approval and its cap;
  * the WSL host.
* **`prepare_run`:**
  * the accepted run-local configs, unchanged, with only the P4/P5 `enabled` flags set (tested byte for byte);
  * `p7/layout.json`, recording the item, slot, scenario, expected world and specification hashes;
  * a run-local marker for dry runs.
* **`prepare_world`:** the accepted generation, then the C6 check. If the world differs from the frozen layout's
  recorded hash, or from the accepted S1–S6 hash, the flight stops before PX4 starts.
* **`execute`:** the accepted sequence.
  * **Summary:** carries exactly the fields the unchanged P4/P5 evaluators check (tested against
    `p4_validation.yaml` and `p5_validation.yaml`), plus a `p7` block.
  * **Credit:** `acceptance_credit_possible: false`. A P7 flight never counts toward a P4 or P5 gate.

**The scenario mapping:**
* the 260 frozen layouts through `p7_specs.scenario_mapping` (D13 label `p7_not_predeclared`);
* the accepted S1–S6, unchanged, for the M4 parity dry run. Pilot and confirmatory flights may fly only P7 layouts.

**The QGC ownership receipt** names the P7 owner copy that runs as PID 1. It has the same bytes as the P3 owner.

## 4. Host check (`p7_host.py`, `p7_host_check.yaml`)

**What it checks.** Read-only, from WSL, before an authorization is consumed (contract §7.1, D10):

| Check | Rule | Number for review |
|---|---|---|
| AC power overlay | `ActiveOverlayAcPowerScheme` = Best performance | `ded574b5…` |
| Power line | `Online` | — |
| Host CPU | mean Windows `LoadPercentage` over 5 samples, 1 s apart | **≤ 15 %** |
| Simulator processes | none matching PX4, `gz sim`, QGC, the DDS agent, the parameter bridge, a namespace owner or a phase runner | pattern in the config |
| Workspace lock | `state/p1.lock` not held (probed read-only) | — |
| Disk floor | WSL free − planned flights × planned size ≥ 150 GB | **1.3 GB per flight** |

**On failure:** the block is "not flown", a not-flown record is written, and the authorization stays unconsumed.

**Live check today (read-only):**
* overlay `ded574b5`, AC Online;
* CPU samples 1, 0, 3, 1, 0 % (mean 1 %);
* no simulator, lock free;
* 840.6 GB free, 788.6 GB after a 40-flight block.

**Result: passed.** In P6 this check was manual.

**Why these numbers:**
* **15 %:** P6's pre-launch host load was 1–5 %, and P6 C-5 showed that load moves the accepted P2 timing.
* **1.3 GB:** just above the largest measured flight (1.27 GB). The pilot measures the real figure.

## 5. Block sequencer (`run_p7_block.py`)

**One invocation runs one block:** the pilot, a confirmatory block of the selected schedule, or a dry run.

**Before consuming anything:**
1. The gates are set, the manifest verifies, and the frozen MC-trace build is selected.
2. The authorization binds this block.
3. A confirmatory block needs the schedule selection and the previous block's unstopped result.
4. The M3 classifier is present.
5. The host check passes.

**Then:**
* **Start:** the marker is consumed and the live lock is taken.
* **Flights:** each pair's two flights run back to back, in the frozen order.
* **After each flight:**
  * the unchanged `verify_p4_evidence.sh` or `verify_p5_evidence.sh` runs;
  * then the M3 classifier (`classify(run) → {validity, stop_reasons}`, writing `p7/flight-record.json`);
  * then a fsynced ledger line;
  * then a residual-process check.
* **End:** the result is written once (exclusive), and the lock is released.

**Rules (contract §7, D6, D14):**

| Event | Effect |
|---|---|
| infrastructure-invalid flight | re-flown once at the end of the block; the pair's valid flight is kept |
| a second invalid attempt of the same flight | the pair is missing, and the batch **stops** (a second replacement would exceed the per-flight limit) |
| more replacements than 10 % of the block's flights (4 of 40, 2 of 20) | the pair is missing, and the batch **stops** |
| a classifier stop reason (safety event, safety-unverified) | **stop** |
| manifest drift before a flight | **stop**; the remaining flights are listed as not flown |
| a simulator process outliving its flight | **stop** |
| a flight that created no evidence, or one that cannot be classified | **stop** |

* A valid flight is never re-flown, and nothing is retried automatically.
* A dry run flies its approved items once each, under the same stop rules, with no replacement and no credit.

## 6. Design choices for the operator to review

1. **Host-check numbers:** CPU ≤ 15 % (mean of 5 samples) and 1.3 GB planned per flight (§4).
2. **The real candidate is frozen before the pilot.** The pilot is a block of the real candidate, so the pilot and
   the confirmatory blocks run on one manifest. The development candidate only runs the M4 dry run.
3. **Blocks run in order:** block k needs the unstopped result of block k − 1. After a stop, the operator decides.
   Resuming is not automated.
4. **A second invalid attempt stops the batch,** following contract §7.2 ("exceeding either limit stops the
   batch"). The pair is also listed as missing (§6 there).
5. **No-evidence and unclassifiable flights stop the block** rather than being counted, because neither can be
   classified without looking at outcomes.
6. **No block can start before M3.** The sequencer refuses to run until the classifier exists, including dry runs.

## 7. What offline tests cannot show

Two things need the M4 parity dry run (S2 with P4 and P5 through the P7 path, on a development candidate):
* the launch inside a real namespace;
* the unchanged P4/P5 evaluators passing on a real P7 run, which checks that the summary carries everything they
  read.

## 8. Next

* **Operator review** of §4 and §6.
* **M3 (classifier and analysis, offline)** needs its own authorization: the validity rule, the outcome classes,
  the two restated P6 §6.3 rules, metrics, the analysis code, and tests on R1–R4 and retained P6 runs.
