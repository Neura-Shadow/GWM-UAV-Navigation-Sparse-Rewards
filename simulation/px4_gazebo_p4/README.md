# P4 — deterministic non-learning obstacle avoidance (PX4 SITL)

**P4 COMPLETE — Qualification 3/3 PASSED** (candidate `c110bd30`, Gates A, B, S and C; PX4 SITL only).
Final results: [final acceptance](../../docs/v3_sim_p4_final_acceptance.md).
Contract: [`p4-acceptance-v1`](../../docs/v3_sim_p4_contract.md) (revisions 2–5).
Design and integration facts: [design](../../docs/v3_sim_p4_plan.md).

This tree lies outside every P3 manifest glob, so the accepted P3 candidate keeps verifying. P4 is
disabled by default: its nodes refuse to start unless an authorized P4 run enables them in its
run-local configuration.

| Path | Purpose |
|---|---|
| `configs/` | avoidance parameters (contract values pinned), acceptance limits and gates, scenarios |
| `ros2_ws/src/gwm_obstacle_avoidance/` | perception (on the unchanged P3 `DepthProcessor`), local grid, planner, supervisor, P4 mission (on the unchanged P2 `Mission`), exclusive controller node, perception node |
| `scripts/` | build, manifest (freeze/verify/authorize), runner (subclass of the P3 runner), gate sequencer, scenario worlds, namespace owner copy and entry shim |
| `validation/` | independent evaluator and evaluator-only Gazebo truth observer |

Build, with no runtime (writes `state/p4-built.json`):

```bash
bash simulation/px4_gazebo_p4/scripts/build_p4.sh --build
```

Freeze after an operator Policy A approval for the P4 commit, then authorize and run one gate at a time:

```bash
python3 simulation/px4_gazebo_p4/scripts/p4_manifest.py freeze --output "$HOME/uav_autonomy/state/p4-candidate.json" \
  --policy-approval APPROVAL.json --mc-benchmark-run "$HOME/uav_autonomy/runs/20261025T051926Z-mc-trace-benchmark-e6000074"
```

```bash
python3 simulation/px4_gazebo_p4/scripts/p4_manifest.py authorize "$HOME/uav_autonomy/state/p4-candidate.json" \
  --gate A --operator NAME --confirm "authorize P4 Gate A <sha>" --output "$HOME/uav_autonomy/state/p4-auth-A.json"
```

```bash
GWM_ALLOW_OPTIONAL_RUNTIME=1 GWM_RUN_GAZEBO_PX4_TESTS=1 GWM_ALLOW_PX4_LAUNCH=1 GWM_PX4_MC_TRACE=px4-mc-causal-trace-v1 \
  /usr/bin/python3 simulation/px4_gazebo_p4/scripts/run_p4_gates.py --gate A \
  --manifest "$HOME/uav_autonomy/state/p4-candidate.json" --authorization "$HOME/uav_autonomy/state/p4-auth-A.json"
```

Gates B, S and C also need `GWM_ALLOW_SITL_COMMANDS=1` and `--allow-simulated-flight`, plus an
accepted predecessor gate for the same manifest. No P3 gate result counts toward P4.
