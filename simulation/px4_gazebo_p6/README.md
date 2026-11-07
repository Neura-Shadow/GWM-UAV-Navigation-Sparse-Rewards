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

Status: M0, M1 and M2 complete (2026-11-06). No simulator has been run for P6; the ROS node wrappers
are not yet exercised live (M5). Tests: `tests/test_px4_gazebo_p6_specs.py`, `tests/test_px4_gazebo_p6_injection.py`.
