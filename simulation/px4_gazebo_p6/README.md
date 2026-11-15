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
(S-5′'s four P2-RECOVERY-landing items are now replaced by the RECOVERY_LANDING proof, plan §16.6–16.7), no credit.
The Gate C content dry run on development candidate `23d513ee` passed 1 of 5; its three P6 defects are fixed and the C-3
outcome rule adopted (plan §16.8–16.9). The rerun on development candidate `75289deb` (commit `e3c8b69`) passed 4 of 5;
C-5's 24-worker load made the P2 controller abort before the map loss (plan §16.10). The adopted C-5 load decision is
implemented (load at the map-loss trigger, the §5.3 LOAD check, the recorder proof, a clean harness stop), and
development candidate `0db5c60a` (code `4c76455`, frozen at `d25831d`) was rerun: C-1 to C-4 passed, and C-5 failed
on a gap in the recorder proof (the overflow fell inside the sensor window, adding two P3 delivery items; plan
§16.11–16.12). The adopted extension is implemented, and the rerun on development candidate `6b1d952d` (commit `32b5a6e`)
passed 3 of 5: C-2 hit a one-off shutdown crash of the accepted DDS agent, and C-5 passed every response check but
failed on a P6 map-drop proof boundary under load (plan §16.13–16.14). The adopted proof fix is implemented, and
the Gate C content dry run on development candidate `57b9af72` (commit `c08bfc8`) passed all 5 rows, no credit (plan
§16.15–16.16). Gate B (2 of 2) and Gate S (5 of 5) content then passed on that same candidate (plan §16.17): B, S and
C content have all passed on one unchanged development candidate, with no credit. Gate A content (18 of 18) then
passed on it too (plan §16.18): all four gates' content on one candidate. The production Policy A approval for the real
candidate is recorded (plan §16.19), and the real P6 candidate `22292775` is frozen (`state/p6-candidate-8e29f3f.json`,
the same sources as `57b9af72`; plan §16.20). **Real Gate A on `22292775` is ACCEPTED (18 of 18, credit; plan §16.21).** **Real Gate B is ACCEPTED (2 of 2,
credit, simulated flight; plan §16.22).** Gates S and C come next on the same manifest, each separately authorized;
every gate needs its own authorization.
Tests: `tests/test_px4_gazebo_p6_specs.py`, `_injection.py`, `_offline.py`, `_evaluate.py`, `_runner.py`.
