# v3-SIM P7 M0 — dependency inventory and the Selective Dependency Backup (revised D5)

**Status: complete (2026-11-21).** This is the read-only dependency inventory the operator asked for, an M0 work
item for the revised D5. Nothing was copied, moved, deleted, overwritten, compressed or uploaded. No simulator
ran, and no P2–P6 file changed. The P3, P4, P5 and P6 manifests verify before and after.

**Approved (operator, 2026-11-21, "Approve set and C:"):**
* the minimal backup set of §5 (≈ 12.6 GB);
* the destination `C:\GWM-UAV-Backup\p7-selective-20261121\`.

**Executing the backup still needs its own authorization** (§7). Nothing has been copied.

## 1. Revised D5

**The operator's revision (2026-11-21):** D5 is now a **Selective Dependency Backup**, not a full historical backup.
This revises D5 only; D1–D4 and D6–D12 are unchanged.
* Only what P7-A needs for development, regression tests, runner parity, accepted-baseline verification and
  reproducibility is backed up.
* Optional historical evidence stays where it is, untouched, and is not backed up.
* Future-only P6 fault evidence is backed up only if P7-B is approved.
* Nothing existing is deleted, moved, overwritten, compressed or uploaded.
* The P7-A workflow must depend only on the Essential set and the Required reference runs.

## 2. Method

The inventory had four parts:
* **Manifest verification, traced:** each of the four `verify` commands ran under a Python audit hook that recorded
  every file opened or listed and every subprocess started. Their only write was the trace file itself, in the
  session scratchpad.
* **Runtime dependencies:** read from the build receipts and from the runners (`p3_runner.py`, `p4_runner.py`,
  `p5_runner.py`, `run_p3.sh`, `run_p6.sh`).
* **Retained-run dependencies:** every run ID referenced by configs, tools and tests (grep), and how each test
  behaves when its run is absent.
* **Exact inventory:** `diagnostics/p7_m0/p7_m0_inventory.py`, a read-only audit tool and not P7 code. It walked all
  of `~/uav_autonomy` without following symlinks and hashed every regular file with SHA-256 (opened with
  `O_NOATIME`). Its only outputs are new files, written with exclusive create:

  | Output | Location | Size | SHA-256 |
  |---|---|---|---|
  | summary, per group | `docs/evidence/v3_sim_p7_m0_dependency_inventory.json` | 559 KB | `f8d856ba…` |
  | system dependencies | `docs/evidence/v3_sim_p7_m0_system_dependencies.json` | 249 KB | `a2d1cd0e…` |
  | full per-file list, 318,015 entries | `diagnostics/p7_m0/results/p7_m0_inventory_files.jsonl` (git-ignored under `results/`) | 88 MB | `2303055a…` |

  The summary JSON carries a tree digest per group: the SHA-256 over the sorted (type, path, size, SHA-256 or link
  target) lines. It also lists every file, with its hash, for each non-optional group of up to 400 files. The full
  list is part of the proposed backup set.

## 3. Results

**Totals** (`~/uav_autonomy`, 120.16 GB across 267,115 files at 2026-11-21):

| Category | Groups | Files | Symlinks | Size |
|---|---|---|---|---|
| 1. Essential | 9 | 113,186 | 792 | **6.91 GB** |
| 2. Required reference runs | 4 | 1,124 | 8 | **4.91 GB** |
| 3. Optional historical | 331 | 142,777 | 1,214 | 85.72 GB |
| 4. Future-only (P7-B) | 3 | 10,028 | 70 | 22.62 GB |

**Integrity:**
* no unreadable file;
* no dangling symlink in categories 1, 2 or 4;
* the only symlinks that leave their group stay inside category 1:
  * the QGC bundle links into the stock QGC tree;
  * each retained run links into the MC-trace PX4 build.

  **Nothing in categories 1, 2 or 4 depends on category 3.**

### 3.1 Essential (category 1)

| Group | Path under `~/uav_autonomy` | Files | Size | Tree SHA-256 | Why it is needed |
|---|---|---|---|---|---|
| E1 | `state/`: 61 named files | 61 | 0.58 MB | `7b7eb87a…` | the 4 accepted manifests; 8 build receipts; 4 accepted Policy A approvals; every authorization, consumption marker and result of the accepted P3 (A/B/C), P4, P5 and P6 (A/B/S/C) gates |
| E2 | `runs/20261025T051926Z-mc-trace-benchmark-e6000074` | 33 | 31.8 MB | `d1f4dbb7…` | bound by every manifest's `runtime.mc_benchmark`; verify reads its `summary.json` |
| E3 | `mc_trace_builds/ecdf4eaa…/PX4-Autopilot` | 46,421 | 2.92 GB | `e13ac4eb…` | the flight PX4: binary, `rootfs/gz_env.sh`, overlay sources and generated headers verify hashes; verify runs `git rev-parse/diff/submodule status` here |
| E4 | `upstream/PX4-Autopilot` | 47,559 | 2.94 GB | `1800a233…` | baseline binary; 45 pinned Gazebo model/world files; gz plugins and `server.config` the runner loads; git state |
| E5 | `qgc_closure_v1/bundles/829942…`, `qgc_closure_v1/receipts`, `qgc_closure_v1/qt-tests/qt-tests-owner-0d1c2d27-20261025T050446Z` | 38 | 59.6 MB | `568d30e4…` | the accepted unattended QGC ELF and the receipts its build receipt references |
| E6 | `qgc-v5.1.4` | 599 | 0.72 GB | `0ef15251…` | **hidden dependency:** 383 symlinks of the accepted bundle point here (766 counting both bundles) |
| E7 | `p3_agent/d6bad178d69ddbe5`, `dds-install` | 258 | 7.6 MB | `dbaa8700…` | the accepted MicroXRCEAgent and its `library_dir` |
| E8 | `ros_ws` | 17,760 | 0.24 GB | `8361056e…` | the `px4_msgs` overlay that `run_p*.sh` sources |
| E9 | `p2_ws/0d994904…`, `p3_ws/c5d37413…`, `p4_ws/c4bd5a55…`, `p5_ws/e7a59f35…` | 457 | 2.2 MB | `b149db35…` | the installed P2–P5 packages named by the build receipts |

**Repository-side dependencies:** every manifest-bound source, config and test, and the P5 model checkpoint
`ee006a80…`, are tracked in git.
* They are on GitHub up to `a5a0df4`, the pushed P6 release.
* The P7 docs commits after it are local only. Pushing them needs its own authorization.

**System dependencies** (`docs/evidence/v3_sim_p7_m0_system_dependencies.json`): these are not in `~/uav_autonomy`
and can't be restored by copying files.

| Dependency | Identity | Restore path |
|---|---|---|
| `/opt/ros/jazzy` | 4 manifest-pinned files, all matching. Packages: ros-jazzy-fastrtps 2.14.6, rmw-fastrtps-shared-cpp 8.4.4, rclpy 7.1.12, ros-gz-bridge 1.0.24; 262 ros-jazzy packages installed | **all 262 have their exact `.deb` in `/var/cache/apt/archives`** (1,242 debs, 0.72 GB) |
| mesa `libgallium` | 25.2.8-0ubuntu0.24.04.2, pinned, matching | apt (no cached deb) |
| controller interpreter | `/usr/bin/python3.12` 3.12.3-1ubuntu0.17; numpy 1.26.4 (`.deb` cached) | apt |
| D3D12 and NVIDIA user-mode driver | 4 files under `/usr/lib/wsl`, pinned, matching | the Windows GPU driver; not backup-able |

No apt package is held. **A routine `apt upgrade` of these packages, or a Windows GPU-driver update, would break the
verification of all four manifests.** That is a host-configuration question for the operator. P7 changes nothing
here.

### 3.2 Required reference runs (category 2)

| Group | Run | Size | Tree SHA-256 | Used by |
|---|---|---|---|---|
| R1 | `20261030T144136Z-p4-flight-S2-abeea2d5` (P4 Gate C, S2) | 1.23 GB | `4035f3c3…` | P5 `policy_replay` source (`p5_validation.yaml`); P7 M4 parity reference for P4 |
| R2 | `20261030T141506Z-p4-flight-S2-9b8ef2b0` (P4 Gate B, S2) | 1.27 GB | `34695f7c…` | P5 `policy_replay` source; second P4 parity sample |
| R3 | `20261106T132442Z-p5-flight-S2-184a5a5e` (P5 Gate C, S2) | 1.20 GB | `06424e33…` | the P6 offline-suite recorded run (Gate A row 4; `test_px4_gazebo_p6_offline.py`, `_evaluate.py`); P7 M4 parity reference for P5 |
| R4 | `20261106T130044Z-p5-flight-S2-679dd050` (P5 Gate B, S2) | 1.20 GB | `4ccb1c1f…` | second P5 parity sample |

Each run is kept complete, including its raw depth recording (`sensors/depth.bin`, about 970 MB), because the
accepted evaluators need it to re-evaluate a run.

### 3.3 Future-only (category 4: only if P7-B is approved)

| Group | Content | Size | Tree SHA-256 |
|---|---|---|---|
| F1 | the 27 run directories of the accepted real P6 Gates A/B/S/C | 13.88 GB | `75d6191b…` |
| F2 | `20261106T131246Z-p5-flight-S6-d907451a` (accepted P5 S6), used only by P6 evaluator tests | 0.76 GB | `b3d70303…` |
| F3 | the 8 P6 development dry runs replayed by P6 evaluator tests | 7.97 GB | `48d7cca1…` |

The P6 tests that use F2 and F3 skip when the runs are absent, so P7-A does not need them.

### 3.4 Optional historical (category 3: kept in place, not backed up)

| Content | Size |
|---|---|
| 108 P6 development dry runs (not the 8 in F3) | 40.69 GB |
| 32 other P5 runs: the remaining accepted flights, the `db4c4779` failed-candidate runs and the snapshot replays | 14.85 GB |
| QGC build provenance: `qgc_closure_v1/{source,build,Qt,tools,development-*}` and the other bundle `34c39a80…` | 9.90 GB |
| 12 other P4 runs | 6.71 GB |
| P3 runs (3.97 GB), P5 scheduling diagnostics (3.65 GB), P2 runs (3.31 GB), P1 runs (0.59 GB) | 11.52 GB |
| P5 training artifacts (`p5_training`) | 1.24 GB |
| `venv-px4` (PX4 build toolchain) | 0.47 GB |
| `recovery_build_checks`, `upstream/Micro-XRCE-DDS-Agent` (agent source), older `p2_ws`/`p5_ws` hashes, 77 non-accepted state records, setup and diagnostic notes | about 0.35 GB |

**Rebuild provenance is not disposable.** The receipts say `clean_rebuild_proven: false` (P0),
`clean_rebuild_claim: false` (MC-trace PX4) and `built_offline_not_runtime_verified` (QGC). So the build trees in
this category must not be discarded on the assumption that they can be rebuilt. P7-A does not need them to run,
because the backed-up runtime artifacts (E3–E9) are the exact binaries the manifests pin.

## 4. Dependency relationships

```text
manifests (E1) ── verify ──> build receipts (E1) ──> E3 PX4 MC-trace build (binary, headers, overlay, git state)
     │                                           ├──> E4 PX4 upstream (baseline binary, 45 gz assets, git state)
     │                                           ├──> E5 QGC bundle ──symlinks──> E6 stock QGC tree
     │                                           ├──> E7 DDS agent + dds-install/lib
     │                                           └──> E9 P2–P5 installed workspaces
     ├── runtime.mc_benchmark ──> E2 benchmark run
     └── runtime pins ──> /opt/ros/jazzy (apt, cache), mesa, D3D12/NVIDIA (Windows driver)
runners (run_p*.sh) ──> /opt/ros/jazzy + E8 ros_ws + E9 + E3 rootfs/gz_env.sh + E4 gz models/worlds/plugins + E5 + E7
P5 policy_replay, P7 parity (P4) ──> R1, R2       P6 offline suite, P7 parity (P5) ──> R3, R4
retained runs (R, F) ──symlinks──> E3
P6 evaluator tests ──> F2, F3 (skip when absent)          P7-B (if approved) ──> F1
```

## 5. Proposed minimal backup set (for approval)

| Part | Content | Size |
|---|---|---|
| A | Essential E1–E9 | 6.91 GB |
| B | Required reference runs R1–R4 | 4.91 GB |
| C | the apt package cache (`/var/cache/apt/archives`, 1,242 debs, including every installed ros-jazzy package) | 0.72 GB |
| D | the inventory itself: the full per-file list, the summary and system JSONs, this report | 0.09 GB |
| | **Total** | **≈ 12.6 GB** |

Not included:
* the Optional historical evidence (85.7 GB, left untouched);
* the Future-only P6 evidence (22.6 GB, added only if P7-B is approved).

**Destination (recommended): `C:\GWM-UAV-Backup\p7-selective-20261121\`.**
* **Why C::** the WSL distribution in use (Ubuntu-24.04, `D:\WSL\Ubuntu-24.04`) and the repository are both on
  physical disk 1 (Predator GM7, 2 TB, D:, 892 GB free). C: is a separate physical disk (disk 0, 512 GB, 149 GB
  free), so a copy there survives a D: disk failure or a corrupted WSL image.
* **What it does not protect against:** the loss of the laptop.
* **Optional second copy:** an external drive the operator connects. None is attached now.
* **Not recommended:** D: as the only copy (the same disk as the data), or cloud upload (it would need its own
  decision, given the size and privacy).

**Format:**
* one **uncompressed** `tar` per group, which keeps Linux symlinks, permissions and timestamps that NTFS cannot hold
  natively;
* the SHA-256 of each tar;
* the per-file inventory.

**Verification after the copy:** extract into a scratch WSL directory, and compare every file's SHA-256 and every
link target with the inventory. Nothing is restored in place.

**Caveats:**
* The receipts and manifests carry absolute paths (`/home/joker0625/uav_autonomy/…`), so a restore that must verify
  needs the same paths.
* The Windows GPU driver and the apt pins are host state. A backup can't replace them; it only records them.

## 6. Keeping P7-A independent of historical evidence

These are constraints of the revised D5, checked in M2/M3:
* P7 code and P7 tests read only categories 1 and 2 (and P7's own new runs).
* No P7 tool globs `runs/` for history.
* Any existing test that needs other retained runs must skip when they are absent. The P6 tests that use F2 and F3
  already do.

**Approvals:**
1. this minimal set: **approved 2026-11-21**;
2. the destination `C:\GWM-UAV-Backup\p7-selective-20261121\`: **approved 2026-11-21**;
3. the authorization to execute the backup: **pending**.

## 7. Execution procedure (runs only under its own authorization)

At approval time: C: had 147.6 GB free; the destination did not exist; C: is physical disk 0 and D: is disk 1.

1. **Preconditions** (checked read-only; any failure stops the run before anything is written):
   * no simulator or P6/P7 runtime process;
   * the P3–P6 manifests verify;
   * C: has at least 12.6 GB plus 20 GB of margin free;
   * the destination directory does not exist.
2. **Drift check:** re-run the inventory tool to new outputs in the session scratchpad. Compare the tree digests of
   E1–E9 and R1–R4 and the apt-cache digest with this report. Any difference stops the run, and is reported rather
   than silently backed up.
3. **Copy:**
   * **Destination:** create the directory once, with exclusive create.
   * **Per group:** one uncompressed `tar` (pax format, numeric owners, symlinks stored as links, nothing
     followed), read from `~/uav_autonomy` with `-C`, so the sources are only read. The apt cache's `.deb` files go
     into their own tar.
   * **Inventory:** the two JSON files, this report, and the full per-file list are copied next to the tars.
4. **Read-back:** write `SHA256SUMS` for every tar, then read each tar back from C: and check it against
   `SHA256SUMS`.
5. **Restore verification:**
   * Extract every tar into a new scratch directory in WSL, never over `~/uav_autonomy`.
   * Compare every file's SHA-256 and every symlink target with the inventory's per-file list. All entries must
     match.
   * Then remove only that scratch extraction. It is this procedure's own temporary copy, not evidence.
6. **Report:** a `backup-receipt.json` in the destination, with tar names, sizes, SHA-256s, the verification result
   and timestamps, recorded in this report and the plan. `~/uav_autonomy` is left unchanged: no file is moved,
   deleted, overwritten or compressed.
