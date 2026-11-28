# v3-SIM P7-E release closure: selective evidence cleanup proposal (manifest v3)

**Status (2026-11-28): the operator decided C-1 to C-4 (§6.1), then C-5, C-6 and C-11, and executed both sets
(C-7, §6.2).**
* The P6 set (78 runs, 624 files, 26.68 GB) and the P7-E set (31 runs, 248 files, 34.94 GB) were removed, each
  under its own file-level authorization.
* Nothing else was deleted, and nothing was moved or compressed. C-8 to C-10 stay open.
* *(When this proposal was first written, no file had been deleted and no procedure was implemented.)*
* It is prepared under the operator's request "P7-E Release Closure and Engineering Next-Stage Preparation", which
  authorizes read-only auditing and proposal preparation only.
* The approved retention policy D4-A/D4-B and rule E10 ([retention proposal](v3_sim_p7e_retention_proposal.md))
  still govern. "Eligible" means only a candidate for a later, separately authorized decision.

**The records:**
* the manifest [`v3_sim_p7e_cleanup_manifest_v3.json`](evidence/v3_sim_p7e_cleanup_manifest_v3.json)
  (`ec5b22058502b3b1c5f4548cb47d1f0055e75696c2118e40fe27df0f8bdd4135`, 564,715 bytes);
* the tool that wrote it, [`p7e_cleanup_manifest_v3.py`](../diagnostics/p7e/p7e_cleanup_manifest_v3.py);
* the tool's offline tests, [`test_p7e_cleanup_manifest_v3.py`](../diagnostics/p7e/test_p7e_cleanup_manifest_v3.py)
  (11 passed).

Version 2 and its JSON (`8e04e6be…`) are unchanged.

## 1. What version 3 changes

| Change | Why |
|---|---|
| Includes the 65 runs added since v2 (63 P7-E flights of both candidates, 2 E8 dry runs) | the inventory must reflect the current filesystem |
| P7-E rule E10 applied after the acceptance (2026-11-28), **reading R-E10**: the batch's own records do not by themselves protect its raw depth. These are its block results, ledgers, the accepted report and this release's record-only analyses. | Each of those needs only records that D4-B always keeps (§5). The literal reading ("runs named by … evidence") would protect all 60, because the report names every flight. **Decision C-1.** |
| Named protections (operator request) | §3 |
| `diagnostics/` scanned as a dependency source (v2 skipped it). The M0 inventory files are excluded, because they list every file by design. | 3 P5 diagnostic flights named by `diagnostics/p5_inference_benchmark` became PROTECTED |
| Stale-action, observation-gap and RECOVERY evidence protected | 14 P6 dry runs that v2 listed as ELIGIBLE carry it, from injected faults |
| **Every ELIGIBLE file verified at manifest time.** Each `depth.bin` gets a whole-file SHA-256; every frame is checked against its `stored_sha256` in `depth-index.jsonl`; the index must cover the file exactly; P6 files are checked against the M0 inventory. Every `tmp/` file gets its size and SHA-256. Any failure means REVIEW. | 15 P6 depth-fault fixtures (F1.04, F1.06, F1.07) failed the coverage check by design. Their injected frames do not chain through the index, although their whole-file hashes match M0. They fell to REVIEW. |

**Reconciliation:**
* `runs/` holds 442 entries: 441 run directories and the protected P2 file. All 442 are accounted for.
* **Verification:** all 113 ELIGIBLE recordings verified, with 277,206 frames checked. All 78 P6 files match M0;
  the 35 P7-E files have no M0 entry, so their whole-file hash is recorded now.

| Class | Runs | Size | Raw depth |
|---|---:|---:|---:|
| PROTECTED | 288 | 174.3 GB | 146.2 GB |
| REVIEW | 40 | 0.8 GB | 0.13 GB |
| ELIGIBLE | 113 | 79.2 GB | 66.7 GB (eligible items) |
| **all** | 441 | 254.2 GB | 213.1 GB |

**Changes since v2 (32 runs):**
* 14 ELIGIBLE → PROTECTED (stale-action or RECOVERY evidence);
* 15 ELIGIBLE → REVIEW (the integrity check);
* 3 REVIEW → PROTECTED (named by diagnostics).

**New since v2:** 65 runs, of which 35 are ELIGIBLE and 30 PROTECTED.

## 2. Recoverable storage, by category

| Category | What | Items | Size | Class |
|---|---|---:|---:|---|
| **Eligible P6 development raw depth** | `sensors/depth.bin` of P6 development dry runs (Gate A content 53, C 15, S 6, B 4); each evaluated, with no safety item and no protecting dependency | 78 files | **26.68 GB** | ELIGIBLE |
| **Eligible P7-E raw depth** | `sensors/depth.bin` of 35 accepted-batch goal successes (slots 2 P5, 4, 5, 8, 10 P4, 11, 13, 17, 19–22, 24–26, 27 P5, 28–30) | 35 files | **40.03 GB** | ELIGIBLE (C-1) |
| Regenerable temporary | `tmp/` of the 113 eligible runs | 791 files | 0.010 GB | ELIGIBLE |
| Regenerable temporary | `/tmp/pytest-of-joker0625` | — | 0.006 GB | outside the evidence; housekeeping |
| Regenerable temporary | `/tmp/px4-sock-71`, `/tmp/px4_lock-71` (empty) | 2 | 0 | housekeeping, only while no simulator runs |
| Regenerable temporary | `~/uav_autonomy/recovery_build_checks` (P3-era environment-mismatch build checks) | — | 0.23 GB | **REVIEW**: kept unless a person confirms nothing cites it |
| Cache | `~/.cache/ccache` (compiler cache) | — | 0.36 GB | regenerable; slower rebuilds |
| Cache | `~/.cache/jedi`, Gazebo, mesa, Qt, matplotlib and PowerShell caches | — | 0.05 GB | regenerable |
| Cache | repository `__pycache__/` and `.pytest_cache/` (32 directories) | — | 0.002 GB | regenerable; no manifest binds them (verified) |
| Cache | `~/.cache/pip` | — | 4.73 GB | **REVIEW**: `venv-px4` was built from unpinned requirements (RR6), so this may be the only local copy of the exact wheels |
| Not proposed | `~/.cache/huggingface` 1.68 GB, `~/.cache/ms-playwright-go` 0.26 GB | — | 1.94 GB | outside this project; not ours to classify |

**Totals:**
* **Recommended set:** C-2 to C-4 plus the regenerable temporary files and caches, without pip or REVIEW. That is
  about **67.1 GB**, or 62.0 GB if decision C-3 keeps the 4 narrow-passage successes.
* **With pip:** about 71.9 GB.
* **Context:** `runs/` uses 254 GB, 213 GB of it raw depth.

**Where the space appears.** Removing files inside WSL frees space in its ext4 file system, but not on D:. The disk
image `D:\WSL\Ubuntu-24.04\ext4.vhdx` (322.4 GB) shrinks only after a separate compaction, which needs
`wsl --shutdown` (decision C-9).

**Not in scope (unchanged):**
* build trees, toolchains and runtime installs: `p2_ws`–`p5_ws`, `ros_ws`, `dds-install`, `upstream`, `qgc-*`,
  `mc_trace_builds` (bound by the manifests), `venv-px4`;
* `p5_training` (model provenance);
* `state/` and `logs/`;
* the C: backup.

## 3. What stays protected

| Protection | Runs | Rule |
|---|---:|---|
| Accepted baseline dependencies (P3–P6 gate evidence, candidates, approvals, results) | 173 named by state | v2 rule, unchanged |
| Named by evidence JSON, tests, code, configs or diagnostics | 137 / 5 / 3 / 14 (overlapping) | v2 rule plus diagnostics |
| Critical reference runs R1–R4 | 4 | v2 rule |
| All P7 runs (M4, both pilot attempts) | 48 | v2 rule |
| **Stopped candidate identity:** the 3 flights of `38e23bad…`, with its stop record and recovery receipt (`state/`) | 3 | E10 and the operator's request |
| **Accepted candidate identity:** manifest, approvals, authorizations, markers, results, ledgers and report (`state/`, never in scope) | — | D4-B |
| E8 development dry runs (launch-path parity) | 2 | new |
| Safety incidents and unresolved failures (missing, crashed or safety-relevant evaluation) | 25 REVIEW | v2 rule (kept complete) |
| **Stale-action, observation-gap and RECOVERY evidence:** the 2 pilot observation-gap RECOVERY flights (also P7), the `38e23bad…` slot 1 P5 `stale_action` flight, and 18 P6 stale-action or RECOVERY runs | 21 | new |
| **P5 repeat-inconsistency evidence:** the P7 pilot repeats (P7, protected) and the cross-candidate repeats (slot 1 P4/P5, slot 2 P4 on `e9561f05…`) | 3 (+ P7) | new |
| **Slot 27 P4** perception compute breach (R4) | 1 | new |
| **Slot 10 P5** Gazebo shutdown integrity event (R10) | 1 | new |
| **P8 inputs:** all 18 hover-timeout failures, and the discordant partners slot 7 P5 and slot 15 P5 | 20 | new (future investigations) |
| P6 depth-fault fixtures that fail the coverage check by design | 15 REVIEW | new (C-4: keep) |

Under v3, 28 of the 63 P7-E flights are PROTECTED (38.4 GB of raw depth) and 35 are ELIGIBLE.

## 4. What independent evaluation becomes impossible after removing a run's `depth.bin`

Verified from the code and by one test.
1. **Re-running the accepted P4 or P5 evaluator.** `p4_evaluate.py` and `p5_evaluate.py` call
   `p3_stream_eval.evaluate_stream`. It opens `sensors/depth.bin` (`p3_stream_eval.py:47`) and requires it among
   the recording files (`:202`). A re-run would fail the recording boundary. The recorded evaluation stays,
   hash-bound by the ledger (H7).
2. **The P3 independent stream accounting:** that every adapter receipt is in the hashed raw recording.
3. **The P6 offline suite on that run:** `p6/validation/recorded.py` `depth_frames` and `p6_passthrough_check.py`,
   that is, perception re-derived from raw frames.
4. **Regenerating the perception snapshots and local maps** (`p4/local_map.jsonl`) from raw frames, to check the
   recorded maps independently. This also covers testing any future perception change offline on that flight, which
   is why the P8 inputs are protected.
5. **Re-verifying each frame** against `depth-index.jsonl`. The index stays; the v3 manifest's check becomes the last
   one, so the receipt must carry it.

**What stays possible:**
* **H9-type re-classification by the frozen classifier.** It was tested on 2026-11-28 on a view of
  `20261127T130931Z-p7e-p5-p7c-4000367-7dc882ab` without `depth.bin`: the record was reproduced exactly, and the run
  was unchanged.
* H7 evidence hashes, FC-3, and R1–R10 from records;
* ULog checks (P2 integrity, D15);
* every decision, map and timing analysis (the P8 analysis uses none of the raw frames).

**For the P6 development runs**, removal also ends re-evaluating their Gate A–S content. They carry no gate credit.

## 5. Proposed cleanup procedure (design only; fail-closed)

**Tool (not written yet):** `diagnostics/p7e/p7e_cleanup_execute.py`, outside every manifest scope, with offline tests
in `diagnostics/p7e/`. Inputs:
* the v3 manifest path and its SHA-256;
* one **file-level authorization**;
* `--rehearse ROOT` for a copy root.

**File-level authorization** (`state/p7e-cleanup-authorization-<set>-<sha12>.json`, exclusive create):
* the manifest SHA-256 and the set name (`p6-dev-depth` or `p7e-depth`);
* every file as `{path, bytes, sha256, frames, frames_sha_verified}`, copied from the manifest;
* the totals;
* the operator, with the phrase "authorize cleanup <set> <authorization-sha> removing <n> files <bytes> bytes".

One authorization per set. Nothing that is not listed can be touched.

**Steps:**
1. **Preconditions (refuse on any failure):**
   * no P7 or P7-E live lock, and the FC-2 process check finds no simulator;
   * the P3–P6, P7 and both P7-E manifests verify;
   * the authorization's hash and phrase match;
   * the manifest hash matches;
   * the receipt path does not exist.
2. **Re-derive at execution time.** The v3 classification is re-run now. Refuse the whole set if any listed run
   changed class, reasons, dependencies, size or mtime, or if reconciliation fails.
3. **Per file, before removal:**
   * it is a regular file, not a symlink, inside `runs/<run>/`, and exactly `sensors/depth.bin` or under `tmp/`;
   * its size, whole-file SHA-256 and every frame's `stored_sha256` match the authorization.
   * **The first mismatch stops everything.** No later file is touched.
4. **Write-ahead ledger:** an fsynced line `{path, bytes, sha256, intent_unix_ns}` before each `unlink`, then a
   fsynced `done` line. Directories are never removed; `tmp/` and `sensors/` stay.
5. **After the set:**
   * the manifests verify again;
   * for each affected run, the hashes of `summary.json`, the evaluation, the flight record, the assessment and the
     ledgers are unchanged;
   * a sample of the affected P7-E runs is re-classified by the frozen classifier (it must reproduce, as in §4);
   * free space is measured.
6. **Permanent receipt**, `state/p7e-cleanup-receipt-<set>-<sha12>.json` (exclusive create, with a copy in
   `docs/evidence/`). It records:
   * the manifest and authorization hashes;
   * every removed file (`path`, `bytes`, `sha256`, `frames`, `frames_verified`, `removed_unix_ns`);
   * every refused or skipped file, with its reason;
   * the before and after checks, and the free space;
   * the tool's own hash.
   * A partial stop still writes the receipt, marked `stopped`, with the ledger as the truth.

**Never:**
* touch `state/`, `logs/`, the repository, build trees or anything outside the listed files;
* follow a symlink;
* compress or move anything;
* run while a block or report holds the lock;
* retry automatically.

**Tests before any use:**
* **offline, on synthetic roots:**
  * T1: the happy path;
  * T2: refusal with a live lock or a running simulator;
  * T3: a run whose class changed;
  * T4: a SHA or frame mismatch, which must stop before the unlink;
  * T5: a stop midway, which must leave a consistent receipt and ledger;
  * T6: an authorization listing a PROTECTED or unlisted file;
  * T7: a symlink;
  * T8: an existing receipt;
  * T9: a manifest that fails to verify;
  * T10: a second execution (an already removed file is refused, not passed);
* **then a copy rehearsal:** copy 2 ELIGIBLE runs (one P6, one P7-E) to a scratch root and execute there. Afterwards
  the frozen classifier must still reproduce, and every record hash must be unchanged.

**Optional archive first.** Copy each set to external media or the C: backup, with a verified restore, before
removal. It is a separate decision (C-8), because it costs space elsewhere.

## 6. Decisions for the operator

| ID | Decision | Recommendation |
|---|---|---|
| C-1 | Adopt manifest v3 for planning, with reading R-E10 (the batch's own records do not protect its raw depth), or the literal E10 (no P7-E flight eligible) | **DECIDED (operator, 2026-11-28): R-E10** |
| C-2 | Confirm the new protections of §3 | **DECIDED: confirmed** |
| C-3 | Also protect the narrow-passage successes, slots 19 and 28 (4 flights, 5.09 GB), as contrast for P8's narrow-passage replay | **DECIDED: protected** (§6.1) |
| C-4 | Keep the 15 P6 depth-fault fixtures (0.12 GB) permanently: they are the raw evidence of injected depth faults | **DECIDED: kept permanently** |
| C-5 | Authorize implementing and testing the cleanup procedure of §5 (no deletion) | **DONE (2026-11-28)**: `tools/evidence_retention/`, 29 offline tests (§6.2) |
| C-6 | After C-5, authorize the copy rehearsal | **DONE (2026-11-28)**: one P6 and one P7-E copy, all checks passed (§6.2) |
| C-7 | Each deletion set separately, with its file-level authorization: P6 first (26.68 GB), then P7-E (40.03 GB, or 34.94 GB with C-3) | **BOTH EXECUTED (2026-11-28)**: P6, then P7-E (§6.2) |
| C-8 | Archive before removal | optional |
| C-9 | Compact the WSL disk image afterwards (separate; needs `wsl --shutdown`) | only after C-7 |
| C-10 | Housekeeping of the caches (ccache, jedi, pytest, `__pycache__`); keep pip until the `venv-px4` requirements are pinned (RR6) | separate, low value (about 0.4 GB) |
| C-11 | The dependency rule after P8 named all 35 eligible P7-E flights: per artifact (a source protects raw depth unless a record-only proof covers its exact bytes) or per run | **DECIDED (2026-11-28): per artifact** (§6.2) |

### 6.1 Decisions recorded (operator, 2026-11-28)

The operator chose reading R-E10, confirmed C-2, protected slots 19 and 28 (C-3) and kept the 15 fixtures (C-4).
Nothing was deleted, and no cleanup is authorized.

**The planning set after these decisions.** The manifest v3 JSON (`ec5b2205…`) is evidence and stays unchanged. C-3
is applied on top of it, by name. Any later file-level authorization (§5) must exclude these four runs. When the
procedure of C-5 is built, its re-derived classification must also carry C-3 as a named protection.

| Slot | Controller | Run | `depth.bin` bytes | SHA-256 (from v3) |
|---:|---|---|---:|---|
| 19 | P5 | `20261127T150333Z-p7e-p5-p7c-4000382-998f6d1e` | 1,283,209,411 | `8a0c525c9343b65d…` |
| 19 | P4 | `20261127T150640Z-p7e-p4-p7c-4000382-ca6304df` | 1,258,003,789 | `32b4961a16728e81…` |
| 28 | P4 | `20261127T160514Z-p7e-p4-p7c-4000358-1a9e443f` | 1,294,159,109 | `7724a77c8495d67f…` |
| 28 | P5 | `20261127T160823Z-p7e-p5-p7c-4000358-3d8d3c23` | 1,254,360,274 | `de70aee4e7953970…` |

| Set (planning, not authorized) | Files | Size |
|---|---:|---:|
| Eligible P6 development raw depth | 78 | 26.68 GB |
| Eligible P7-E raw depth (35 minus the 4 above) | 31 | 34.94 GB |
| `tmp/` of the remaining eligible runs | — | about 0.01 GB |
| **Total** | 109 | **about 61.6 GB** |
| PROTECTED P7-E raw depth (28 + 4 flights) | 32 | 43.46 GB |

### 6.2 Progress recorded (operator, 2026-11-28: "C-5＋C-6＋備妥授權檔 (Recommended)" and C-11 "依檔案判定 (Recommended)")

* **F-CL1.** P8 evidence and a frozen P8 config now name all 35 eligible P7-E flights. These are the V1 replay, the
  hover-timeout analysis and `p8_recovery.json`.
* **C-11 proof.** Each of the three tools reproduced its evidence with every `depth.bin` open refused:
  `docs/evidence/v3_sim_retention_record_only_p8.json` (`6a8cc770…`).
* **Re-derived now (read-only):**
  * the P6 set is 78 of 78 ELIGIBLE, identical to v3;
  * the P7-E set is 31 of 31 ELIGIBLE under C-11;
  * the C-3 runs are PROTECTED.
* **C-5:** the executor and the manifest v3 lineage are in `tools/evidence_retention/`. They pass T1–T10 and 19
  further tests offline.
  * The cleanup's own records (authorizations, receipts, ledgers, proofs) are excluded as dependency sources.
    They list runs by design.
* **C-6:** two rehearsals on copies (`v3_sim_p7e_cleanup_rehearsal_p6.json` `6253751d…`, `…_p7e.json` `304d1964…`).
  Every check passed, including the frozen classifier reproducing the P7-E flight record without `depth.bin`.
* **Prepared:** `state/p7e-cleanup-authorization-p6-dev-depth-ec5b22058502.json` (`5b07faa8…`). It lists 78 runs,
  624 files and 26,682,094,504 bytes, each re-verified.
* **C-7 (P6) **EXECUTED (operator, 2026-11-28, "Execute P6 set")**: receipt `state/p7e-cleanup-receipt-p6-dev-depth-5b07faa8469f.json` (`ce326ddf…`, copy `docs/evidence/v3_sim_p7e_cleanup_receipt_p6-dev-depth.json`), status `completed`.**
  * The phrase used was "authorize cleanup p6-dev-depth
    5b07faa8469fb203d4e27374b2f31cfa9e71c1d05428c0dcec28e5e49ac5b7be removing 624 files 26682094504 bytes".
  * 624 of 624 files were removed (26,682,094,504 bytes). The removed set equals the authorized set (every SHA-256).
  * Ledger `p7e-cleanup-ledger-p6-dev-depth-5b07faa8469f.jsonl` (`490009dc…`): a header, 624 `intent` and 624
    `done` lines.
  * Before: no live lock, no simulator, re-derivation unchanged. Before and after: P8 `fe2a0c6e…` and P7-E
    `38e23bad…` verify. The retained records of all 78 runs are unchanged.
  * Every run keeps its depth index, evaluation (78 of 78 readable), ULog and empty `tmp/`. The other 233 recordings
    are untouched.
  * Free space in WSL: 682.83 → 709.52 GB. `runs/`: 258.5 → 231.8 GB. The disk image on D: stays 352.0 GB until
    C-9.
* The design for later phases is in [Evidence Retention Manager](v3_sim_evidence_retention_design.md).

* **P7-E set.** Operator "Prepare P7-E set", then "Execute P7-E set".
  * Authorization `state/p7e-cleanup-authorization-p7e-depth-ec5b22058502.json` (`b8c8b3ba…`): the 31 accepted-batch
    goal successes of `e9561f05…` that no protection covers (no C-3 run), with 248 files (31 `depth.bin` and 217
    `tmp/`) and 34,939,082,035 bytes.
  * Phrase: "authorize cleanup p7e-depth b8c8b3ba1bd1f5edb8eef190570231d046c9f90afb5c3c380afc180025b5d2ca removing
    248 files 34939082035 bytes".
  * **Receipt** `state/p7e-cleanup-receipt-p7e-depth-b8c8b3ba1bd1.json` (`68bb51b1…`, copy
    `docs/evidence/v3_sim_p7e_cleanup_receipt_p7e-depth.json`): `completed`, 248 of 248, equal to the authorized set.
  * Ledger `8f87528e…`: a header, 248 `intent` and 248 `done` lines.
  * The re-derivation was unchanged. The manifests verify before and after, and the retained records are unchanged.
  * **The frozen classifier reproduced the flight records** of the 3 sampled flights (first, middle and last of
    the 31) without `depth.bin`.
  * All 31 runs keep their index, flight record, ULog and evaluation.
  * The 32 other P7-E flights, including C-3, keep their complete raw depth.
  * 179 tests pass afterwards (P7-E 76, P8 74, retention 29).
  * Free space in WSL: 709.52 → 744.45 GB. `runs/`: 231.8 → 196.9 GB, with 202 recordings left. The disk image on D:
    stays 352.0 GB until C-9.

Still open: C-8 to C-10 (C-9 is what shrinks the disk image on D:).
