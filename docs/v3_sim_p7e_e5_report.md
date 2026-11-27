# v3-SIM P7-E E5: the engineering report

**Status: PREPARED, not run.** E5 needs the operator's explicit authorization (contract §13 item 9). Accepting the
report is a separate step after that.

**Candidate:** `state/p7e-candidate-2b6ee5d-r2.json`, manifest
`e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608`.
**Rules:** contract [`p7-engineering-validation-v1`](v3_sim_p7e_contract_draft.md) §5–§7 and §10; the approved
report plan `p7e-report-plan-v1` (canonical `15e7f7b4…`); E2's report tool, as accepted
([E2 §4](v3_sim_p7e_e2_code.md), interpretations 1–8).

## 1. What E5 does

E5 is offline only: no simulator, no flight, no change to any bound file. It runs the accepted
`simulation/px4_gazebo_p7e/analysis/p7e_report.py` (`8734bbe9…`) **once**:

```
cd /tmp && GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 \
  …/simulation/px4_gazebo_p7e/analysis/p7e_report.py \
  --manifest ~/uav_autonomy/state/p7e-candidate-2b6ee5d-r2.json \
  --output   ~/uav_autonomy/state/p7e-engineering-report-e9561f05de28.json
```

* **Inputs:** the 3 block results and their ledgers. For each of the 60 flights, it reads the summary, the accepted
  P4 or P5 evaluation, the frozen P7 flight record and the P7-E assessment, each accepted only if its hash matches
  the ledger.
* **H1–H8:** computed from those records.
* **H9:**
  * the frozen P7 classifier is re-run on each of the 60 flights, through its own wrapper
    (`verify_p7_evidence.sh`, `99359ade…`, bound by P7 `0799c37e`). Each run gets a temporary symlink view without
    `p7/flight-record.json` and `p7e/`. The classifier writes its record only into that view.
  * The re-run must reproduce each record exactly, and FC-3 must reproduce each assessment. `assess` only reads.
  * Every run file's size and mtime must stay unchanged.
  * The manifest, the specs and the 60 world hashes must re-verify.
* **R1–R10:** descriptive only, with the frozen P7 interval functions and no p-values.
  * A breach of an accepted limit is the accepted evaluator's own failure item. It becomes a proposed new
    risk-register entry, not an engineering failure.
  * One such item is already known from E4: slot 27 P4, `performance:compute_max`.
* **Output:** written once (exclusive create). The tool refuses to run again if a report exists, and prints the
  status (exit 0 only for "engineering validation passed").

**After the run** (part of the same authorization):
* copy the report byte for byte to `docs/evidence/v3_sim_p7e_engineering_report.json`, with its hash recorded;
* write this document's results: the status, H1–H9, R1–R10, the risk register, the required statement, the
  not-claimed list and the 30-pair limits;
* commit locally; nothing is pushed.

**Launch:** detached, as the blocks were (`setsid nohup`, log `~/uav_autonomy/logs/p7e-e5-report-e9561f05de28.log`).
This way the end of a session cannot kill it. If it dies before writing, there is no report and nothing changes.
A second run then needs a new authorization.

**Duration:** minutes (plan §10). I have not timed the classifier on these runs before authorization.

**If the status is "not passed":** the report is kept as written, with no rerun and no change to any threshold.
The operator then decides (contract §7.3).

## 2. The stopped first attempt (`38e23bad…`)

The report tool reads only the results of the manifest it is given, so it covers the 60 flights of `e9561f05…`.
The stopped first attempt is described in this document, not in the report JSON:
* the first real candidate, `38e23bad…`;
* block 1 was terminated externally after 2 flown flights and 1 interrupted one;
* it was recovered under authorization into a recorded stop: `sequencer_terminated_externally`, all 3 flights safe.

That evidence stays protected. Its flights are not part of the 60.

## 3. Pre-check (read-only, 2026-11-28)

| Check | Result |
|---|---|
| manifest `e9561f05…` verifies (P7 `0799c37e` and P3–P6 inside), development false | yes |
| source drift | none |
| specs identity equals the manifest's | yes |
| report code `8734bbe9…`; report plan canonical `15e7f7b4…`; schedule canonical `9b3deacb…` | all match |
| block results 1, 2, 3 present, none stopped | yes |
| flights in the ledgers | 60 |
| evidence complete and hash-matched (summary, evaluation, record, assessment, ledger hash) | 60 of 60 |
| run directories and `p7/layout.json` present | 60 of 60 |
| classifier wrapper `99359ade…` present; bound by the P7 manifest | yes |
| classifier environment: `venv-px4`, ROS Jazzy, P2–P5 installs | present |
| report output and evidence copy | neither exists |
| live lock; simulator processes | none; none |
| free space (`/tmp` and the evidence root) | 687 GB |
| the 76 P7-E offline tests (`tests/test_px4_gazebo_p7e_*.py`) | 76 passed (32 s) |
| working tree (Windows git) | clean apart from the untracked `.codegraph/` |

**Note (RR5):** `git status` inside WSL lists many files as modified. It ignores the Windows `core.autocrlf`, so
the CRLF working copies look changed. Windows git shows none, and the manifest finds no drift. Nothing was changed.

No H or R value was computed in the pre-check.
