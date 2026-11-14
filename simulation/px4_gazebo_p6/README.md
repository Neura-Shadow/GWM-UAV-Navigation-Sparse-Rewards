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

Status: M0, M1, M2 (2026-11-06), M3 and M4 (2026-11-07) complete. M5 complete 2026-11-13 (plan §15): the authorized
dry run of development candidate `524f0714` (commit `e5d7fb5`) passed all 18 Gate A rows (plan §15.19), no credit. M6 in
progress (plan §16): the runner and sequencer carry the Gate B, S and C flight rows; the Gate B content dry run (B-0, B-1)
passed in simulated flight on development candidate `33f33c5a` (plan §16.5), and the Gate S content dry run passed 4 of 5
(S-5′ fails on four items the accepted evaluators raise for any P2 RECOVERY landing; decision open, plan §16.6), no credit. No P6 gate has run; the real P6
freeze and every gate need their own authorization.
Tests: `tests/test_px4_gazebo_p6_specs.py`, `_injection.py`, `_offline.py`, `_evaluate.py`, `_runner.py`.
