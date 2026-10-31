# P5 — fixed-weight policy substitution (PX4 SITL)

Contract: [`p5-acceptance-v1`](../../docs/v3_sim_p5_contract.md) (revision 1, approved).
Design and integration facts: [plan](../../docs/v3_sim_p5_plan.md).

A frozen NumPy MLP ranks the 26 P4 motion candidates. Only P4's L3 selection is replaced: every
executed action passes the unchanged P4 safety rules, supervisor and P2 control checks. This tree is
outside every P3 and P4 manifest glob, so the accepted baselines keep verifying (`1e205fec…`,
`c110bd30…`). P5 is disabled by default and refuses untrained or unverifiable checkpoints.

| Path | Purpose |
|---|---|
| `configs/` | policy configuration (contract values pinned), acceptance limits and gates, frozen model schema |
| `ros2_ws/src/gwm_policy_control/` | policy runtime, observation adapter, ranker, planner and supervisor replicas with the selection hook, `P5Mission`, the policy ledger and the `p5_control` node |
| `scripts/` | source identity, baseline checks, runner, gate sequencer and wrappers (freeze and build follow in M7) |
| `validation/` | independent evaluator and the policy replays (run under the controller interpreter) |
| `models/`, `training/` | content-addressed model artifacts and offline training (plan M3, separately authorized) |

Status: offline milestones M0–M6 (contract revision 2). The candidate model `models/p5-ranker-ee006a80….npz` was trained
offline, meets the contract §4.2 readiness bar and was accepted by the operator. The `p5_control`
node, `P5Mission`, the policy ledger, the runner, the gate sequencer and the evaluator are implemented
(plan §13.3–13.4). Nothing is frozen, and no
simulator has been run for P5.
