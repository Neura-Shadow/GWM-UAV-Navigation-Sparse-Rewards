# v3-SIM Evidence Retention Manager: design, storage study, and the first cleanup steps

**Status (2026-11-28):**
* **The approved cleanup workflow is complete for both planned sets**, under the operator's answers
  "C-5＋C-6＋備妥授權檔 (Recommended)", "Execute P6 set", "Prepare P7-E set" and "Execute P7-E set" (§1):
  * the procedure is implemented and tested (C-5);
  * it is rehearsed on copies (C-6);
  * the P6 set was executed: 624 files and 26.68 GB of raw P6 development depth and `tmp/`, receipt `ce326ddf…`,
    every check passed.
  * the P7-E set was executed: 248 files and 34.94 GB, receipt `68bb51b1…`, every check passed. The frozen
    classifier reproduced the sampled flight records.
  * **Nothing else was deleted, moved or compressed.**
* **New decision C-11** ("依檔案判定 (Recommended)"): a dependency protects only the files it reads. Proven here
  for P8 (§1.2).
* **The retention design (§3–§6) is a proposal.** It changes no frozen file, and no P8 recording behaviour.
  * Nothing here starts SITL or touches the P8 candidate `fe2a0c6e…`. That manifest verified before and after each
    rehearsal, and again at the end.
  * Nothing was pushed.

The project is an engineering platform, not a raw-data archive. A flight records about 1.4–2.7 GB, and about 87–90 %
of that is raw depth. The design keeps everything needed for safety verification, reproducibility, debugging and
regression. It stops raw recordings from accumulating after every phase.

## 1. What was done under this request

| Step | Result | Record |
|---|---|---|
| Review of the pending workflow | Decisions C-1 to C-4 were made. No deletion was ever authorized (C-5 to C-10 open). The operator chose how far to go. | this note |
| **Finding F-CL1** | Since manifest v3, P8 evidence and a frozen P8 config name **all 35 eligible P7-E flights**: the V1 replay, the hover-timeout analysis and `p8_recovery.json`. Under the v3 rule ("named → PROTECTED"), the P7-E set would no longer be eligible. | §1.2 |
| C-11 proof | The three P8 tools reproduce their evidence with every `depth.bin` open refused | `docs/evidence/v3_sim_retention_record_only_p8.json` (`6a8cc770…`) |
| **C-5:** procedure implemented and tested | `tools/evidence_retention/retention.py` (executor), `p7e_v3_lineage.py` (manifest v3, with C-1 to C-4 and C-11); **29 offline tests pass** (T1–T10 and more) | §3.6 |
| **C-6:** copy rehearsal | One P6 run and one P7-E run copied to `~/retention-rehearsal/`, then executed there. All checks passed, and the originals are untouched. | `v3_sim_p7e_cleanup_rehearsal_p6.json` (`6253751d…`), `…_p7e.json` (`304d1964…`) |
| P6 set authorization prepared | **78 runs, 624 files (78 `depth.bin` + 546 `tmp/`), 26,682,094,504 bytes.** Every file was re-verified (SHA-256, every frame). | `state/p7e-cleanup-authorization-p6-dev-depth-ec5b22058502.json` (`5b07faa8…`) |
| **P7-E set executed** ("Prepare P7-E set", "Execute P7-E set") | authorization `b8c8b3ba…` (31 goal successes, no C-3 run); `completed`: 248 of 248, 34,939,082,035 bytes. The frozen classifier reproduced 3 of 3 sampled flight records, and the retained records are unchanged. Free space 709.52 → 744.45 GB. 179 tests pass afterwards. | `state/p7e-cleanup-receipt-p7e-depth-b8c8b3ba1bd1.json` (`68bb51b1…`), copy `docs/evidence/v3_sim_p7e_cleanup_receipt_p7e-depth.json` |
| **P6 set executed** ("Execute P6 set") | `completed`: 624 of 624 removed, equal to the authorized set. Manifests verify before and after, and the retained records are unchanged. Free space 682.83 → 709.52 GB. | `state/p7e-cleanup-receipt-p6-dev-depth-5b07faa8469f.json` (`ce326ddf…`), copy `docs/evidence/v3_sim_p7e_cleanup_receipt_p6-dev-depth.json` |
| Investigation | camera, recorder, evaluator dependencies, per-flight composition, codecs, event windows, retained-record compressibility | §2, §4 |

### 1.1 The rehearsal (C-6)

| | P6 copy | P7-E copy (`…7dc882ab`) |
|---|---|---|
| Removed (from the copy) | 8 files, 3.9 MB | 8 files, 1.26 GB |
| Manifests before and after (P8 `fe2a0c6e…`, P7-E `38e23bad…`) | verified | verified |
| Re-derivation at execution | unchanged | unchanged |
| Retained records (size, mtime, key-record and ULog SHA-256) | unchanged | unchanged |
| Frozen classifier on the copy without `depth.bin` | not applicable | **reproduces the flight record** |
| Originals | `depth.bin` present, size and mtime unchanged | same |

The rehearsal copies (198 MB, without their raw depth) stay in `~/retention-rehearsal/` for review. They
are outside `~/uav_autonomy` and no record depends on them.

### 1.2 C-11: the artifact-level dependency rule

`record_only_check.py` runs a dependent tool under a PEP 578 audit hook. The hook refuses every open of
`sensors/depth.bin` (a positive control checks this first) and logs every file the tool opens under `runs/`.

| Tool (source that names the runs) | Runs read | `depth.bin` opens | Output | Files it reads |
|---|---:|---:|---|---|
| P8 V1 replay (`v3_sim_p8_v1_replay.json`) | 60 | 0 | equal (wall-clock keys excluded) | controller result, decisions, local map, policy ledger, flight record, events, `p4_avoidance.yaml` |
| Hover-timeout analysis (`v3_sim_p8_hover_timeout_analysis.json`) | 62 | 0 | byte-identical | the same, plus layout and clock |
| P8 specs `verify` (`p8_recovery.json` and the two other specs) | 60 | 0 | equal to the frozen configs | controller result, decisions, policy ledger, flight record |

Every file they read is one that D4-B keeps permanently. The only P8 tool that reads raw depth is V2 perception. Its
eligible runs are the four C-3 runs, which stay protected.

**Fail-closed.** A source counts as record-only only while its exact current bytes match the proof. If the source
changes or the proof is missing, the source protects again.

**After C-11, re-derived now (read-only):**
* the P6 set is 78 of 78 ELIGIBLE, identical to v3 (class, reasons, dependencies);
* the P7-E set is 31 of 31 ELIGIBLE (34.94 GB);
* the four C-3 runs are PROTECTED.

## 2. Measured facts

### 2.1 Camera and recorder

| Item | Value | Source |
|---|---|---|
| Depth sensor | OakD-Lite `StereoOV7251`, `depth_camera`, **640×480 `R_FLOAT32` (`32FC1`), 30 Hz**, clip 0.2–19.1 m, **no noise model** | PX4 `Tools/simulation/gz/models/OakD-Lite/model.sdf` |
| RGB sensor | `IMX214` 1920×1080, 30 Hz. Only its presence is evaluated; no frames are recorded. | same; `p3_sensors.yaml` |
| Raw frame | 1,228,800 bytes; 39–54 % of the pixels are `+inf` (beyond the far clip) | codec study |
| Recording rate | **every received frame, 30.3 Hz**, independent of the 10 Hz processing (`sensor_hz` 30, `processing_hz` 10) | `p3_sensors.yaml`, depth index |
| Recorder | `gwm_sensor_adapter/recorder.py` `RawRecorder`: one writer thread, a bounded queue of 32 (sticky overflow), **zlib level 1 per frame**, an index row per frame (`raw_sha256`, `stored_sha256`, offset, layout), fsync, and a close marker | code |
| Achieved compression | zlib-1 at 2.8–4.3× of raw per flight | depth indices |

### 2.2 One flight's evidence

| Flight (role) | Frames | Raw depth | `depth.bin` | Share | Everything else |
|---|---:|---:|---:|---:|---:|
| P7-E P5 goal success | 4,076 (134.5 s) | 5.01 GB | 1.26 GB | 87.1 % | 0.19 GB |
| P8-P4R dry run (goal via R1) | 4,642 | 5.70 GB | 1.32 GB | 86.9 % | 0.20 GB |
| P8-P5R dry run (hover timeout) | 5,641 | 6.93 GB | 2.46 GB | 90.1 % | 0.27 GB |

The rest of a flight, in decreasing order:
* `timing-trace.jsonl`, 41–60 MB;
* PX4 `rootfs/` with the ULog, 38–54 MB;
* `ros-events.jsonl`, 20–29 MB;
* `mavlink/`, `mc-trace.bin`, the sensor records (index, observations, health) and the P4 and P5 records.

Measured in `v3_sim_retention_storage_measurements.json` (`04788d50…`).

### 2.3 Who needs the raw depth

| Consumer | Needs `depth.bin`? | What it requires |
|---|---|---|
| P3 stream evaluation (`p3_stream_eval.read_frames`), used by the accepted P4/P5 evaluators and by P8 | **yes** | **every** receipt contiguous in the index; compression exactly `zlib-1` or `none`; stored and raw SHA-256; layout `32FC1`; no trailing data |
| P6 recorded replay, pass-through and safety checks | yes | frames by acquisition stamp |
| P8 V2 perception | yes | frames of the final-HOVER snapshots |
| Map regeneration from raw frames | yes | every processed frame |
| P7 classifier and P7-E H9 re-classification | **no** (tested 2026-11-28, and again in the C-6 rehearsal) | records |
| P7-E report H1–H10, P8 report H1–H11 and R-P8 | no | records and ledgers |
| P8 V1, specs, hover-timeout analysis | **no** (C-11 proof) | records |
| ULog and MAVLink checks | no | ULog, MAVLink record |

**Consequence.** Every phase that uses a frozen P3–P5 evaluator for credit must record exactly as today: every frame,
zlib-1. That covers P8. A different codec or rate is possible only in a new phase with a new evaluator version.

## 3. The design

### 3.1 Principles

1. **Generation is separate from retention.** A run always records completely while it is flown and evaluated (D4-A).
   What stays afterwards is decided later, by rule, with a receipt.
2. **Records are permanent; only raw payloads are removable.** Removable items are exactly `sensors/depth.bin` and
   files under `tmp/`.
3. **Dependencies are per artifact (C-11).**
   * A source protects the raw depth of a run if it names the run, unless a record-only proof covers its exact bytes.
   * A source that reads raw frames always protects them.
4. **Fail-closed everywhere.** If something is unknown, unreconciled, changed, or its proof is missing, it is kept.
5. **One authorization per set.**
   * It lists every file with its size, SHA-256, mtime and frame check.
   * The operator answers with a phrase that names the authorization's own SHA-256.
6. **Every removal leaves a permanent receipt.** The depth index, with every frame's raw SHA-256, stays, so the
   integrity of a removed recording stays provable.

### 3.2 The three profiles

A phase contract names one profile per flight class.

| | **FULL_VALIDATION** | **SELECTIVE_RETENTION** | **OPERATIONAL_MONITORING** |
|---|---|---|---|
| For | formal validation, new perception features, fault injection, critical reference runs | normal validation blocks, after their evaluation | ordinary development, operation, smoke tests |
| Recording | complete raw depth (today's recorder) | complete raw depth (today's recorder) | telemetry, decisions, metrics, ledgers, ULog; raw depth only in event windows plus 1 Hz key frames (§4.3) |
| During execution and evaluation | keep everything | keep everything (D4-A) | keep everything recorded |
| After evaluation and integrity verification | keep everything | remove the raw depth of flights with no protection trigger (§3.4), under §3.6 | keep the event clips and records; nothing more to remove |
| Frozen-evaluator compatible | yes | yes, until the removal (the evaluator re-run ends there) | **no**; no gate credit, no frozen-evaluator verdict |
| Long-term per flight (measured) | 1.4–2.7 GB | protected: as FULL; otherwise 0.19–0.27 GB | about 0.3 GB, or 0.13 GB with compressed records (§5.3) |

### 3.3 Artifact classes

| Class | Items | Rule |
|---|---|---|
| **ALWAYS KEPT** | summary; controller result; evaluations; P7 flight record and P7-E assessment; P4 decisions and local maps; P5 policy ledger and trace; P8 recovery ledger; ULog; MAVLink record; truth poses and contacts; events; timing trace; depth index, observations, health, adapter result and recorder close marker; every manifest, approval, authorization, marker, result, ledger, report and receipt in `state/` | never removable (the executor's path rule) |
| **RAW, removable after verification** | `sensors/depth.bin` | only when no protection trigger applies, under a file-level authorization |
| **REGENERABLE** | `tmp/` of a run; caches outside `runs/` | `tmp/` like raw; caches are a separate housekeeping decision (C-10) |

### 3.4 Protection triggers: the complete raw evidence is kept

Each trigger is evaluated per run at classification time and again at execution:
1. accepted baseline and candidate dependencies (gate, block and dry-run results, candidates, approvals) of P2–P8;
2. **safety incidents**: safety-relevant evaluation items, contacts, clearance, unsafe responses;
3. **unresolved failures**: no readable evaluation, a crashed evaluator, a mission failure under investigation;
4. **integrity events**: recorder overflow, frame-check failure, shutdown kill, compute breach, stale action,
   observation gap, RECOVERY;
5. **required reference runs** (R1–R4) and runs named by tests, code, configs, diagnostics or evidence, **unless a
   record-only proof covers that source** (C-11);
6. **regression dependencies** that read raw frames: replays, perception checks, map regeneration (always protecting);
7. named operator protections (for example C-3) and stopped-candidate identities;
8. **an inventory that does not reconcile**: nothing is eligible.

### 3.5 Lifecycle of a run

`RECORDING → EVALUATED → VERIFIED → CLASSIFIED → (PROTECTED | ELIGIBLE → AUTHORIZED → RAW REMOVED with a receipt)`

* **A phase's flights become CLASSIFIED only after its report is accepted.** This is E10's rule, generalized.
  Before that, nothing of the phase is eligible.
* A REVIEW class is for anything a person must look at. It is never removable.

### 3.6 Components (implemented: `tools/evidence_retention/`, outside every manifest scope)

| File | Role |
|---|---|
| `retention.py` | `prepare`, `execute`, `rehearse` (generic executor) |
| `p7e_v3_lineage.py` | one lineage: manifest v3, the frozen v3 classifier, C-1 to C-4, C-11, the manifests to verify, the after-check |
| `record_only_check.py` | the C-11 proof harness |
| `depth_codec_study.py`, `storage_measurements.py` | the measurements of §2 and §4 |
| `test_retention.py` | 29 offline tests |

**What `execute` does:**
1. **Refusable phase.** Nothing is written but the lock. The phase checks:
   * the phrase, the authorization and manifest hashes, and the root binding;
   * the receipt and ledger are absent, and no other cleanup holds the lock;
   * no `*live*.lock` exists, and FC-2 finds no simulator (the frozen `p7e_host` check);
   * P8 `fe2a0c6e…` and P7-E `38e23bad…` verify (P8 covers P7-E `e9561f05…`, P7 and P3–P6);
   * the classification re-derives unchanged;
   * every file passes a pre-check (regular file, no symlink, size and mtime).
   A refusal removes nothing and writes no receipt.
2. **Removal phase,** under an fsynced write-ahead ledger, per file:
   * the whole-file SHA-256, and for `depth.bin` every frame against the index;
   * an `intent` line;
   * the unlink, only if the same inode is still in place;
   * a directory fsync, then `done`.
   * The first mismatch or error stops everything. No later file is touched.
3. **After-checks:**
   * the manifests verify again;
   * every retained file of each run keeps its size and mtime, and the key records and ULogs their SHA-256;
   * the frozen classifier reproduces sampled P7-E flight records;
   * free space is measured.
4. **The receipt** is written once (exclusive create), `completed` or `stopped`. It carries the ledger hash, the
   tool's and classifier's hashes, and every removed and not-removed file.

**A new phase plugs in with its own lineage module:**
* its manifest of runs;
* its classifier, or the generic triggers of §3.4;
* its record-only proofs;
* the manifests to verify;
* its after-check.

The executor never changes.

## 4. Can less be stored? (measured; nothing applied)

### 4.1 Lossless codecs

150 frames per flight (10 blocks of 15 consecutive frames) from three flights were measured. Every result decoded
byte-identically. The table shows `v3_sim_retention_depth_codec_study.json` (`ffafdf42…`).

| Codec | Size vs today's `depth.bin` | Encode MB/s (one core) | Keeps 30 Hz on one core |
|---|---:|---:|---|
| zlib-1 (today) | 1.00 | 137–165 | yes |
| zlib-6 | 0.96–0.97 | 74–99 | yes |
| lz4 / brotli-1 | 1.18–1.70 | 720–4,270 | yes (larger) |
| shuffle + zlib-1 | 0.49–0.52 | 172–226 | yes |
| **hdelta + shuffle + zlib-1** (standard library and numpy only) | **0.24–0.29** | **197–245** | **yes (5× margin)** |
| hdelta + shuffle + brotli-5 | 0.16–0.21 | 116–161 | yes |
| hdelta + shuffle + lzma-1 | 0.15–0.19 | 73–107 | yes |
| temporal XOR + shuffle + zlib-1 | 0.58–0.59 | 147–200 | yes (worse than spatial) |

`hdelta` is the row-wise difference of the float32 bit patterns, modulo 2³². `shuffle` splits the four byte planes.
They work because the rendered depth has no noise and large constant regions (`+inf`).

**Limits:**
* A new codec needs a new recorder `compression` identifier and an evaluator version that decodes it. The frozen
  evaluator accepts only `zlib-1` or `none`.
* Each frame keeps its `raw_sha256`, so the decoded raw can be proven identical.
* Transcoding existing recordings in place would break the frozen evaluators and the recorded hashes. It is not done.
* **Lossy encoding** (for example millimetre `uint16`) would break `raw_sha256`. It is not acceptable for evidence.

### 4.2 Store the payload of processed frames only

Perception uses 10 of the 30 recorded frames per second.
* A future recorder can index **every** receipt with its `raw_sha256`, keeping the stream accounting complete, but
  store the payload only of the processed frames.
* That is a third of the depth.
* It needs a new stream-accounting contract, because the content of an unprocessed frame becomes provable by hash
  only.

### 4.3 Event-triggered recording

Measured over the 60 accepted P7-E flights, with windows of ±3 s around HOLD actions, supervisor transitions, stale
perception and P5 interventions, plus a 1 Hz key frame elsewhere:
* **21 % of frames are kept** (15 GB instead of 72.8 GB at today's codec);
* the median for successes is 18.5 %, and for failures 27.7 % (maximum 59 %).

It needs a ring buffer of at least 3 s in the recorder, and it is incompatible with the frozen stream accounting.
That is why OPERATIONAL_MONITORING only.

### 4.4 A compact, reversible archive of protected raw depth (option; would need its own procedure)

Recompressing each recorded frame with zlib 1.3 level 1 reproduces the stored bytes exactly:
* 4,076 of 4,076 frames of one full flight;
* 2,124 sampled frames of four more: P8 dry runs, a P4 reference run, a P6 run.

So a protected `depth.bin` could be stored as `hdelta + shuffle + lzma` (0.15–0.19) or `+ zlib` (0.24–0.29) and
**restored byte-identically on demand**, verified by every frame's `stored_sha256` and the whole-file SHA-256.

**Limits:**
* The restore depends on zlib's deterministic output. The archive must pin the library version, and a restore that
  does not verify fails closed.
* Converting protected evidence is a change of protected evidence. It needs a separately designed, tested and
  approved procedure, with a restore test before any original is removed.

### 4.5 Retained records

The 186.5 MB of records of one flight compress 4.8–5.6× as whole files (zlib-6: 38.9 MB; lzma-1: 33.4 MB).
* For frozen phases this is possible only inside a reversible archive (4.4), because the records are hash-bound.
* A future phase can write its JSONL compressed from the start.

## 5. Storage estimates: peak and long term (GB, decimal)

### 5.1 Today and after the planned sets

| | `runs/` | Note |
|---|---:|---|
| Now (445 entries) | 258.5 | WSL file system 343.3 GB used, 682.8 GB free; disk image 352.0 GB |
| After the P6 set (C-7, **done**; measured) | 231.8 | −26.68 |
| After the P7-E set (C-7, **done**; measured) | 196.9 | −34.94 |
| Of which protected raw depth | about 155 | P3–P6 gate evidence, P7, protected P7-E, C-3, P8 dry runs |
| With the archive option of 4.4 (×0.16–0.29) | about 67–87 | option, not proposed for now |

### 5.2 P8 V3–V5 (60 flights, frozen recorder)

| | Peak (all raw kept until the P8 report is accepted) | Long term |
|---|---:|---:|
| FULL for all | 85–165 (central 115) | same |
| SELECTIVE after acceptance, 40–60 % protected | 85–165 | **about 40–105 (central 65)** |

**Assumptions:**
* A flight is 1.4–2.7 GB: P7-E averaged 1.41 GB, and the P8 dry runs were 1.52 and 2.73 GB.
* Recovery adds hover and scan time, so P8 flights may be longer.

**The peak fits.** The WSL file system has 682.8 GB free, and D: 879 GB.
* If the P6 and P7-E sets are removed before the blocks, the blocks reuse those 61.6 GB of freed space.
* The disk image then grows by about 53 GB instead of about 115 GB (central case).

### 5.3 A future 60-flight phase (central case: 1.6 GB of raw depth and 0.2 GB of records per flight today)

| Recording and retention | Peak | Long term |
|---|---:|---:|
| Today's recorder, FULL | 108 | 108 |
| Today's recorder, SELECTIVE (50 % protected) | 108 | 60 |
| Recorder v2: hdelta + shuffle + zlib-1 (4.1), SELECTIVE | 38 | 25 |
| Recorder v2 + processed-frame payloads (4.2), SELECTIVE | 21 | 16 |
| OPERATIONAL_MONITORING: event windows + v2 codec (4.3) | 17 | 17 |
| OPERATIONAL_MONITORING with compressed records (4.5) | 8 | 8 |

**Without a policy:** at today's 1.3–2.7 GB per flight, ten 60-flight phases would accumulate 0.8–1.6 TB. With
SELECTIVE retention and recorder v2 they keep about 0.25 TB, and the protected raw evidence is still complete.

## 6. Decisions for the operator

| ID | Decision | Recommendation |
|---|---|---|
| **C-7 (P6)** | Execute the P6 set: 78 runs, 624 files, 26.68 GB of raw P6 development depth and `tmp/`. **It cannot be undone.** Records, indices, ULogs and evaluations stay. Removal ends the re-evaluation of these development dry runs, which carry no gate credit. | **EXECUTED (2026-11-28)**, receipt `ce326ddf…` |
| C-7 (P7-E) | The P7-E set (31 runs, 34.94 GB, under C-11) | **EXECUTED (2026-11-28)**, receipt `68bb51b1…` |
| **R-1** | Adopt this design (profiles, artifact classes, triggers, lifecycle, C-11) as the retention policy for phases after P8 | adopt |
| **R-2** | P8 retention rule: P8 V3–V5 raw depth becomes eligible only after the P8 report is accepted, except §3.4 triggers. Those include every failure, every FC stop, the guard firing (R4), every refused recovery action (FC-4), and V2-type replay inputs. Nothing changes in the frozen P8 candidate or its recording. | adopt (decide before the P8 report) |
| R-3 | Recorder v2 (4.1 codec, 4.2 processed-frame payloads, 4.3 event windows) and evaluator v2, for a future phase. Designed and tested separately; its first validation transcodes an accepted recording and must give identical evaluations. | later, separately |
| R-4 | The reversible compact archive of protected raw depth (4.4) | only if storage matters |
| C-8 / C-9 / C-10 | archive before removal; compact the WSL disk image (needs `wsl --shutdown`); cache housekeeping | unchanged; C-9 only after the removals |

**Not authorized by anything here:**
* any deletion, compression or movement of evidence beyond the two executed sets;
* any change of a frozen file or of P8 recording;
* SITL;
* a push.
