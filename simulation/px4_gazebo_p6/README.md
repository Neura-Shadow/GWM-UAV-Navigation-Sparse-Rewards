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

Status: M0 and M1 complete; nothing is implemented beyond these specifications, and no simulator has
been run for P6. Consistency test: `tests/test_px4_gazebo_p6_specs.py`.
