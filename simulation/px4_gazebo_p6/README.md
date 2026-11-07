# P6 — fault injection and safety validation (PX4 SITL)

Contract: [`p6-acceptance-v1`](../../docs/v3_sim_p6_contract.md) (approved 2026-11-06).
Plan: [P6 plan](../../docs/v3_sim_p6_plan.md); fault list: [fault matrix](../../docs/v3_sim_p6_fault_matrix.md);
baseline audit: [M0](../../docs/v3_sim_p6_m0_audit.md).

P6 injects declared faults around the accepted P5 stack (manifest `2becfbbb…`) without changing it,
and checks detection and safe handling with an independent evaluator. This tree is outside every P3,
P4 and P5 manifest scope, so the accepted baselines keep verifying.

| Path | Purpose |
|---|---|
| `configs/p6_faults.yaml` | the 86 fault IDs: mechanism, detection, expected behaviour codes, tier and class (M1) |
| `configs/p6_scenarios.yaml` | the gate rows A/B/S/C, triggers, parameters and expected outcomes, and the characterization registry (M1) |
| `injection/` | the injectors (M2): declaration, fault ledger, triggers, depth/state/map/policy transforms, controller shim, depth and state relays, witness, competing publisher, process signals, load harness |
| `validation/p6_passthrough_check.py` | M2 exit check: pass-through bit identity on a recorded accepted flight |
| `validation/p6_offline_suite.py`, `validation/recorded.py` | M3: the hash-bound offline suite (Gate A row 4) and the read-only recorded-flight reader |
| `validation/p6_evaluate.py` and `p6_run`, `p6_fault_evaluate`, `p6_safety_evaluate`, `p6_timing_evaluate`, `p6_coverage_report` | M4: the independent evaluator (occurrence proof, response checks, composition with the unchanged P5 evaluator, timing, coverage) |
| `validation/p6_integration_evaluate.py` | M5: rule C11, the injector-process check (A-6), the A-3 relay figures and ground policy replay |
| `scripts/p6_runner.py`, `p6_manifest.py`, `run_p6_gates.py`, `run_p6.sh`, `verify_p6_evidence.sh`, `p6_node.py` | M5: the runner (built on the accepted P5 runner), the candidate manifest (development candidates, dry-run approvals, live-run lock, drift check), the Gate A sequencer with its dry-run mode, the namespace launch, the evaluation wrapper |

Status: M0, M1, M2 (2026-11-06), M3 and M4 (2026-11-07) complete. M5 in progress (plan §15): ground dry runs of
development candidates (no credit). The authorized dry run of `ce73e574` (commit `68ccf30`) passed 16 of 18 Gate A
rows. The two F4.07/F4.13 decisions are implemented (plan §15.12), and development candidate `44fa1111` (commit
`f74b71f`) awaits authorization for its dry run. No P6 gate has run.
Tests: `tests/test_px4_gazebo_p6_specs.py`, `_injection.py`, `_offline.py`, `_evaluate.py`, `_runner.py`.
