# v3-SIM P3–P5 raw-evidence inventory and backup plan — PROPOSED

**Status: planning only (2026-11-06).** The inventory below was taken read-only. Nothing was
deleted, moved, compressed, copied or uploaded, and no backup exists yet.

**Decided by the operator on 2026-11-06** ([P6 contract](v3_sim_p6_contract.md) §10):
* **D9:** two copies, on an external disk and in a versioned cloud object store.
* **D10:** keep all accepted, failed and cited diagnostic evidence indefinitely.

**Still needed before any copy or upload:** the external disk path, the cloud destination and
authorization of the backup step.

**Why a backup.** GitHub holds the sources, contracts, reports and small evidence summaries
(`docs/evidence/`). The raw acceptance evidence exists only on this machine, under
`~/uav_autonomy/` in WSL: the gate results, authorizations, consumption markers, flight runs,
ULogs, rosbags and evaluations. A single disk failure would leave the accepted gates unverifiable.

## 1. Inventory (read-only, 2026-11-06)

### 1.1 State: `~/uav_autonomy/state/` (73 files, about 0.6 MB)

**Anchors.** The first 16 hex digits of each file SHA-256 are shown. Full hashes go into the
backup manifest.

| Phase | Manifest (canonical SHA) | Manifest file SHA | Policy A approval | Build receipt |
|---|---|---|---|---|
| P3 | `p3-candidate-087722b.json` (`1e205fec…`) | `a38ce477b5143315` | `p3-policy-A-approval-087722b.json` `75aa0d63a53664aa` | `p3-built.json` `41bf4f875c147dc2` |
| P4 | `p4-candidate-50d5002.json` (`c110bd30…`) | `ab1d2af58a0fd400` | `p4-policy-A-approval-50d5002.json` `9b2d40c8d6f824f1` | `p4-built.json` `500258bdded4e040` |
| P5 | `p5-candidate-40a109b.json` (`2becfbbb…`) | `e50419c006ed31d7` | `p5-policy-A-approval-40a109b.json` `34783fb092c5309f` | `p5-built.json` `a94b7723ef0f0afd` |
| P5 history | `p5-candidate-ab7b0c6.json` (`db4c4779…`) `ac456eb992bb0bb5`; `p5-candidate-44ff340.json` (`baef37a0…`) `5e6b67cd068f1502` | | `…-ab7b0c6.json` `2f1da3cbfa4e27fb`; `…-44ff340.json` `e5803216129a0682` | `p5-built-35eaccd0.json` `626304223c00f8c1` |

| Gate result | File SHA (16) | Accepted |
|---|---|---|
| P3 A / B / C `1e205fec…` | `5d67e8f9d7a58227` / `52cbd0999c86a6f6` / `f4599afb7d1c6485` | yes / yes / yes |
| P4 A / B / S / C `c110bd30…` | `ffbd4ee9bf2db753` / `b2fe75c57035567d` / `8ef9cad4b2bab041` / `c7ce0752efc3cd52` | all yes |
| P5 A / B / S / C `2becfbbb…` | `3a30744c51f6a4e1` / `3c53a7e2f499f7a2` / `9df073fde9148f08` / `189710d1310de922` | all yes |
| P5 A / B / S / C `db4c4779…` | `9555a6854cbcc0b9` / `b0b543e77cac926d` / `97dbf4ce465cc349` / `018e03e5f04c1674` | yes / yes / yes / **no** (inference max) |
| P5 A `baef37a0…` | `a7375258c45a15c4` | **no** (evaluator gap) |

**Also in `state/`:**
* the gate authorizations (`p*-auth-*`), 16 files;
* the consumption markers (`*.consumed.json`), 16 files;
* the runtime receipts:
  * `p0-built.json` `9f0ec3e9a383ea0d`;
  * `p2-built.json` `66a67971c715739f`;
  * `px4-mc-trace-built.json` `1ef1efa69a5f257c`;
  * `qgc-unattended-built.json` `0c836b52211213a2`;
  * `p3-dds-agent-built.json`;
* `versions-final.json`;
* the `qgc-workload-authorizations/` directory.

### 1.2 Runs: `~/uav_autonomy/runs/` (185 directories, about 36.1 GB)

Each run is matched to the gates that reference it through the `runs` lists of the result files.

| Group | Runs | Files | Size | Notes |
|---|---|---|---|---|
| P3 `1e205fec` A/B/C | 12 | 1,524 | 3.70 GB | 4 flights with ULog and rosbag; 7 ground fixtures; 1 readiness |
| P4 `c110bd30` A/B/S/C | 14 | 3,879 | 8.58 GB | 7 flights, 7 ground runs; every one with ULog and rosbag |
| P5 `2becfbbb` A/B/S/C | 14 | 3,650 | 8.81 GB | 7 flights; 5 ground runs with a rosbag; replay (no ULog); policy fault (ULog, no bag) |
| P5 `db4c4779` (history, Gate C failed) | 12 | 3,084 | 7.75 GB | includes the failed Gate C S2 flight `20261031T121719Z-…-2e9825f4` |
| P5 `baef37a0` (history, Gate A failed) | 7 | 1,668 | 0.21 GB | |
| P5 scheduling diagnostic (no gate credit) | 3 | 855 | 3.40 GB | `20261101T07…-p5diag-flight-S2-*` |
| MC benchmark | 1 | 33 | 0.03 GB | `20261025T051926Z-mc-trace-benchmark-e6000074`, bound by every manifest |
| P2 (referenced by `docs/evidence/v3_sim_p2_summary.json`) | 77 | 15,049 | 3.08 GB | the summary JSON is in Git and in the P3 manifest |
| P1 | 34 | 7,841 | 0.55 GB | |
| setup / build / repeat / QGC setup | 11 | 58 | < 0.01 GB | |

**Run layout.**

* **Contents.** Each flight run holds:
  * the run-local configs;
  * `summary.json`, `controller-result.json`, `ros-events.jsonl`, `timing-trace.jsonl`;
  * the `p4/` and `p5/` ledgers;
  * `rosbag/`, `rootfs/log/<date>/*.ulg` and `mavlink/`;
  * `sensors/`, `source/` and `world/`;
  * `mc-trace.bin`;
  * `p*-evaluation.json`.
* **Symlinks.** `rootfs/etc` and `rootfs/test_data` are **symlinks** into
  `mc_trace_builds/ecdf4eaa…/PX4-Autopilot/build/…`. A backup must keep them as links and must also
  contain their targets.

### 1.3 Runtime trees needed to re-verify (not evidence, but not proven rebuildable)

The receipts record `clean_rebuild_proven: false` (`p0-built.json`) and `clean_rebuild_claim: false`
(`px4-mc-trace-built.json`). A rebuild might not reproduce the bound hashes, so these trees are
part of the backup:

| Tree | Size | Bound by |
|---|---|---|
| `upstream/` (PX4-Autopilot baseline build) | 2.9 GB | `p0-built.json`, pinned PX4 assets in every manifest |
| `mc_trace_builds/ecdf4eaa…` | 2.8 GB | `px4-mc-trace-built.json`; run `rootfs` symlinks |
| `qgc_closure_v1/bundles/82994283…` | 9.3 GB (whole `qgc_closure_v1`) | `qgc-unattended-built.json` ELF `82994283…` |
| `p3_ws`, `p4_ws`, `p5_ws` (installs `c5d37413…`, `c4bd5a55…`, `e7a59f35…` and `35eaccd0…`) | < 0.01 GB | `p3/p4/p5-built.json` |
| `p5_training/` | 1.2 GB | training provenance for model `ee006a80…` (the model files themselves are in Git) |
| `dds-install`, `p2_ws`, `ros_ws`, `venv-px4`, `qgc-v5.1.4` | about 1.4 GB | environment; rebuildable, so optional |

**Total for a complete backup:** about 36 GB of runs, plus about 16 GB of runtime trees, so about
**52 GB**, or about 54 GB with the optional trees. The WSL disk has 841 GB free; this machine's
free space is not a backup location.

## 2. Backup format (proposal)

* **One evidence pack per phase and candidate**, for example
  `gwm-evidence-P5-2becfbbb-v1/`, `gwm-evidence-P4-c110bd30-v1/`,
  `gwm-evidence-P5-history-db4c4779-baef37a0-v1/`, and `gwm-evidence-runtime-v1/` for §1.3.
* **Directory structure** mirrors `~/uav_autonomy`:
  `state/<files>`, `runs/<run-id>/…`, `runtime/<tree>/…`. Paths stay relative, so the bound
  absolute paths (`/home/joker0625/uav_autonomy/...`) can be re-rooted on restore.
* **`PACK-MANIFEST.json`** has one record per file:
  * relative path, size, mode, mtime and SHA-256;
  * for symlinks, `link_target` instead of a hash;
  * the pack identity: phase, canonical manifest SHA, gate result SHAs, Git commit of the sources
    (`1f06f39…` at planning time), creation time and tool version;
  * a `pack_sha256` = SHA-256 of the canonical JSON of all records.
* **Container (proposed for the copy only).** Uncompressed `tar` (POSIX, symlinks kept), one per
  pack, plus a detached `PACK-MANIFEST.json` and its SHA-256 written alongside.
  * Compression is optional on the copy, since rosbags and ULogs compress modestly.
  * The originals are never compressed or altered.
* **Not in Git.** About 52 GB is unsuitable for Git, and Git LFS is not recommended. Git keeps only
  the pack manifests' SHA-256s (a small `docs/evidence/` index), once approved.

## 3. Verification script (proposed, not written)

`simulation/px4_gazebo_p6/scripts/verify_evidence_pack.py` would live outside every manifest scope.
It runs read-only.

1. Recompute every file's SHA-256 and size, and check every symlink target, against
   `PACK-MANIFEST.json`. Recompute `pack_sha256`.
2. For each gate result in the pack:
   * check that its file SHA is the anchor of §1.1;
   * check that every referenced run is present;
   * check that `digest(run/p*-evaluation.json)` equals the result's `evaluation_sha256` (the same
     check `require_predecessor` uses).
3. Report missing, extra or changed files. The script exits non-zero on any difference.

## 4. Restore procedure (proposed)

1. Extract into a scratch root, for example `/mnt/backup-restore/uav_autonomy`, never over the
   originals.
2. Run the verification script (§3) on the restored tree.
3. **Re-verify the manifests against the restored root**, with `GWM_SIM_ROOT` set to it and the Git
   checkout at the recorded commit:
   * `p3_manifest.py verify`, `p4_manifest.py verify` and `p5_manifest.py verify` must return
     `1e205fec…`, `c110bd30…` and `2becfbbb…`;
   * this also re-checks the PX4, QGC and MC runtime receipts and the pinned assets, which is why
     §1.3 is included;
   * absolute paths inside receipts point to `/home/joker0625/uav_autonomy`, so a faithful restore
     either uses that path or a bind mount; M0 confirms which.
4. **Optional, strongest:** re-run the independent evaluators (`verify_p5_evidence.sh` and the
   others) on restored runs in a separate output directory. Compare the decisions with the stored
   evaluations. Re-running may need the ROS environment, and it never overwrites stored files.
5. Record the restore test (date, pack SHAs, results) in a restore log. Test a restore once after
   the first backup and then at each retention review.

## 5. Storage and retention (decisions D9, D10)

* **Copies:** at least two, on two devices, at least one off this machine. Examples: an external
  disk and a cloud object store with versioning. The operator chooses the targets and approves any
  upload. Nothing is uploaded during planning.
* **Integrity checks:** a quarterly `verify_evidence_pack.py` run on each copy.
* **Retention (recommended):**
  * keep every accepted gate's evidence (P3, P4, P5 `2becfbbb`) and every failed gate's evidence
    (P5 `db4c4779`, `baef37a0`) indefinitely;
  * keep the diagnostic runs as long as the documents that cite them;
  * no deletion is proposed.
  * *Note:* the P3 and P4 release cleanups already removed superseded runs. Those cleanups are
    recorded in `p4-release-cleanup-log.json` and in Git history.
* **P6 runs:** the same pack format, created at P6 acceptance.

## 6. Steps that need authorization

| Step | Needs |
|---|---|
| Write `verify_evidence_pack.py` and its tests | implementation approval (P6 M2 or separately) |
| Build the packs (read originals, write tar copies to the chosen target) | approval of D9 and the target path; disk space at the target |
| Any upload | explicit operator approval naming the destination |
| Any deletion, move or compression of originals | **not proposed** |
