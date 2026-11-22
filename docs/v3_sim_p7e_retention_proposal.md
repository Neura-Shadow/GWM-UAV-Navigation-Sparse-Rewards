# v3-SIM P7-E — evidence retention revision (D4-A / D4-B) and read-only cleanup manifest

**Status: POLICY APPROVED by the operator (2026-11-22, "Approve retention"): D4-A and D4-B (decision E9) and the
P7-E rule E10 are in force, as written below.**
* **No deletion is authorized by this approval** (contract §11, clarification 5; §5 below). No file was deleted,
  moved or compressed, and the 107 ELIGIBLE, 28 REVIEW and 241 PROTECTED runs, and the protected P2 file, all stay.
* **Any cleanup still needs:** a separately designed, tested and approved procedure (§4), then a separate
  authorization of each execution, naming the manifest version and the exact file set.
* **Nothing was deleted, moved or changed.** No cleanup is implemented, and none can run without a separately
  tested and separately approved procedure (§4).
* **What exists:** the manifest tool [`diagnostics/p7e/p7e_cleanup_manifest.py`](../diagnostics/p7e/p7e_cleanup_manifest.py),
  which only reads (`stat` and existing records), and its output
  [`docs/evidence/v3_sim_p7e_cleanup_manifest_v2.json`](evidence/v3_sim_p7e_cleanup_manifest_v2.json) (SHA-256
  `8e04e6be414bd842415ce377807208db4d05707f485bd2d4265ba30c204464d4`).
* **Version 2 is reconciled:** 377 of 377 entries of `runs/` are accounted for. Version 1 (`e87ec4c5…`) read
  directories only and missed one file; it is superseded.

## 1. The current rule, and why revise it

**Current rule.** D4 of `p7-evaluation-v1` keeps the full evidence of every flight, with no deletion and verified
copies only.

**Why revise.** The raw depth recording (`sensors/depth.bin`) is about 85 % of a flight's 1.2–1.3 GB and does not
compress (1.04:1). `runs/` already holds 376 runs (377 entries) and 164.1 GB.

**What depth.bin is needed for.** The accepted evaluators use it to re-evaluate the depth stream. A flight's
evaluation, its classification and every safety result are separate files, and they stay.

## 2. Proposed policy

**D4-A, while flying and evaluating (unchanged from today):**
* every flight keeps its complete evidence, raw depth included, until its independent evaluation is done and its
  integrity verified;
* during that stage nothing is removed, compressed or moved.

**D4-B, long-term retention after evaluation and integrity verification. Always kept:**
* accepted baseline and candidate identities; manifests, approvals, authorizations, consumption markers, and gate,
  dry-run and block results with their ledgers;
* every flight's summary, controller result, events, accepted evaluation, P7 flight record, metrics, policy ledger
  and trace, truth poses, contacts, MAVLink record and **ULog**;
* the depth **index** with each frame's raw SHA-256, so that the recording's integrity stays provable after removal;
* the **complete** raw evidence of:
  * the reference runs used by tests (R1–R4, and runs named by tests);
  * every accepted P3–P6 gate run;
  * all P7 runs;
  * any run with a safety incident or an unresolved failure.

**Eligible, only after evaluation and verification, and only under a separately approved cleanup:**
* the raw depth recording of runs with no protecting dependency;
* regenerable temporary files (`tmp/`).

## 3. The read-only cleanup manifest (2026-11-22)

**Reconciliation (clarification 1).** `runs/` has 377 entries: 376 run directories and one file,
`20260912-p2-r3-historical-timing.json` (64 KB, a P2-era read-only timing reconstruction named by
`v3_sim_p2_timing_contract.md`). The file is **PROTECTED** and never eligible. The tool marks nothing eligible
unless every entry is accounted for; here it is (`reconciled: true`).

**What each run gets.** The tool walks all 376 run directories and records each one's:
* size, its `depth.bin` and `tmp/` sizes;
* every record that depends on it: gate, block and dry-run results, candidates and approvals in `state/`, tests,
  simulation code and configs, evidence JSON, documentation;
* the state of its evaluation;
* a classification.

Inventory-type files, meaning the M0 inventory and this proposal's own outputs, are not counted as dependencies.

| Class | Runs | Size | Raw depth | What it contains |
|---|---:|---:|---:|---|
| **PROTECTED** | 241 | 111.8 GB | 92.1 GB | 120 named by a gate or block result, candidate or approval; 69 named by an evidence JSON; 48 P7 runs (M4, both pilot attempts); 4 reference runs (R1–R4). Runs named by tests or code are inside these counts. |
| **REVIEW** | 28 | 3.8 GB | 3.1 GB | 25 without a readable evaluation (build and setup runs, P5 diagnostic flights, some early P6 dry runs); 3 early P6 ground dry runs whose evaluator crashed (`evaluator_incomplete_evidence`, the gap D15 fixed for P7). All are kept complete pending a person's review. |
| **ELIGIBLE** | 107 | 48.6 GB | 39.5 GB | **only P6 development dry runs** (Gate A content 68, C 25, S 10, B 4), each with a complete evaluation, no safety item, and no dependency other than its dry-run result record |

**Estimated savings if a cleanup were approved: 39.5 GB** of raw depth (107 files), plus 0.01 GB of `tmp/`.
* All 107 eligible `depth.bin` files already have a SHA-256 in the M0 inventory, so their integrity record exists.
* Their dry-run result records, evaluations, flight records and ULogs all stay.

**What removal would cost:**
* re-running the accepted depth-stream evaluation on those 107 development dry runs;
* any future investigation that needs their raw frames.

They carry no gate credit, and none is named by a test, code, config or evidence JSON.

**Not in scope:**
* build trees, toolchains and runtime installs (reproducibility not verified);
* `state/`;
* the C: backup;
* everything outside `runs/`.

## 4. Requirements for any future cleanup procedure

1. **Separate design, test and approval.** It is tested offline on copies, then approved explicitly before any use.
2. **It re-derives the manifest at execution time** and refuses any run whose classification or dependencies
   changed. It touches only ELIGIBLE items: `depth.bin` and `tmp/`.
3. **It never touches** accepted P3–P6 evidence, frozen P7 evidence (pilot, M4, P7-E), reference runs, runs named by
   tests, code, configs or evidence JSON, safety incidents or unresolved failures, unless a separate authorization
   names them.
4. **Before removing a file,** it verifies the file's SHA-256 against the M0 inventory or the per-frame index. It
   writes an exclusive-create ledger with path, size, SHA-256 and time, and keeps the index.
5. **Optionally archive first:** copy to the C: backup or external media with a verified restore. That is a separate
   decision, because it costs space elsewhere.
6. **It never runs while a block holds the live lock.** The P3–P6 manifests and the P7 candidate must verify before
   and after.

## 5. Policy approval is not a deletion authorization (clarification 5)

* **Policy only.** Approving D4-A/D4-B approves a retention **policy**. It authorizes no deletion, move or
  compression of any file.
* **Each deletion** needs its own explicit authorization, naming:
  * the procedure, already tested on copies (§4);
  * the reconciled manifest version;
  * the exact file set.
* **Protected evidence.** No existing P3–P7 evidence may be deleted under the policy approval. That includes the 107
  P6 development dry runs listed as ELIGIBLE, the 28 REVIEW runs, and every PROTECTED run and file.
* **Eligible is not approved.** "Eligible" only means a candidate for a future, separately authorized decision.

## 6. Decisions

* **E9:** adopt D4-A and D4-B as above.
* **E10:** P7-E runs stay complete until the P7-E report is accepted. Then their raw depth becomes eligible under D4-B,
  except the protected classes.
* **Later, separately:** design and test the cleanup procedure, and approve its execution, optionally archiving
  first.
