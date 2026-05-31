# Sparse Rewards in UAV Navigation

## Overview

Sparse rewards represent one of the most challenging aspects of reinforcement learning (RL) for autonomous navigation. Unlike dense rewards that provide continuous feedback at every timestep, sparse rewards only provide meaningful signal when specific conditions are met — such as reaching a goal or colliding with an obstacle.

This project uses sparse reward bonuses to encourage goal-reaching behavior in a world-model-guided navigation framework.

## Reward Structure

### Dense Components (per-step costs)

These costs provide continuous gradient signal for the MPC planner:

| Component | Formula | Weight | Purpose |
|-----------|---------|--------|---------|
| Goal cost | `‖pos - goal‖` | 1.0 | Attract toward goal |
| Obstacle cost | `max(0, threshold - obs_dist) × w` | 15.0 | Repel from obstacles |
| Smoothness cost | `‖action - last_action‖ × w` | 0.15 | Smooth trajectories |
| Energy cost | `‖action‖² × w` | 0.02 | Minimize energy usage |

### Sparse Component (event-triggered)

| Component | Formula | Weight | Trigger |
|-----------|---------|--------|---------|
| Sparse bonus | `-120.0` (negative cost = reward) | 1.0 | `goal_dist < goal_reach_dist` |

The sparse bonus of **-120.0** is applied when the UAV comes within `goal_reach_dist` (default: 3.0m) of the goal. This large negative cost (equivalent to a positive reward) provides a strong signal that overwhelms the accumulated dense costs.

## Why Sparse Rewards Are Hard

1. **Credit Assignment**: The agent must execute hundreds of correct actions before receiving any sparse signal. Attributing the final reward to earlier decisions is difficult.

2. **Exploration**: Without the sparse signal, the agent has no incentive to reach the goal beyond the dense goal-distance cost. If the goal-distance landscape has local minima (e.g., around obstacles), the agent may never discover the sparse reward.

3. **Sample Efficiency**: Learning from sparse signals requires many more episodes compared to dense rewards, as most episodes produce zero sparse reward.

## Our Approach: World Model + MPC

This framework addresses the sparse reward challenge through:

### 1. World Model Imagination
The learned dynamics model allows the planner to **imagine** future trajectories without executing them. This enables:
- Evaluating thousands of candidate action sequences in simulation
- Discovering paths that reach the sparse reward zone
- Planning over long horizons (default: 12 steps)

### 2. Random Shooting MPC
Instead of gradient-based policy optimization, we use random shooting:
- Sample `num_samples` (default: 120) random action sequences
- Roll out each through the world model
- Evaluate total cost including potential sparse bonus
- Execute the first action of the best sequence

### 3. Cost Function Design
The dense costs create a smooth landscape that guides exploration:
- Goal cost provides a global gradient toward the target
- Obstacle cost creates repulsive barriers
- The sparse bonus provides a "cliff" at the goal that the planner can discover through rollouts

## Configuration

```yaml
# configs/default.yaml
cost_weights:
  goal: 1.0
  obstacle: 15.0
  smooth: 0.15
  energy: 0.02
  sparse_bonus: -120.0

safety:
  goal_reach_dist: 3.0  # Sparse reward trigger distance

planner:
  horizon: 12      # Planning horizon (steps)
  num_samples: 120 # Random shooting samples
```

## Implemented Mitigation Strategies

1. **Curriculum Learning**: Implemented in `src/rl/curriculum.py` and configurable via `configs/curriculum.yaml`. It dynamically scales navigation task difficulty (goal distance, target size, step limit, and obstacle density) as training progresses to ensure stable convergence under sparse reward environments.

## Future Directions

1. **Hindsight Experience Replay (HER)**: Relabel failed episodes with achieved goals to increase learning signal
2. **Curiosity-Driven Exploration**: Intrinsic rewards based on world model prediction error
3. **Multi-Agent Sparse Rewards**: Cooperative tasks where the sparse signal requires coordination

## References

- Andrychowicz et al., "Hindsight Experience Replay" (NeurIPS 2017)
- Pathak et al., "Curiosity-driven Exploration by Self-Predictive Next" (ICML 2017)
- Ha & Schmidhuber, "World Models" (NeurIPS 2018)
