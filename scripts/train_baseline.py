#!/usr/bin/env python3
"""Train baseline world-model-guided UAV navigation.

Reproduces the original ``main.py`` loop using the refactored modules:
plan → safe_action → step → remember → train.

Usage
-----
    # Mock environment (no AirSim needed):
    python scripts/train_baseline.py --env mock --max-steps 200

    # AirSim environment:
    python scripts/train_baseline.py --env airsim

    # Custom config:
    python scripts/train_baseline.py --config configs/default.yaml --env mock
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
import torch.optim as optim
import yaml

# Ensure the project root is importable when running the script directly.
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.env.base_env import BaseNavigationEnv
from src.env.mock_env import MockNavigationEnv
from src.rl.mpc_planner import CostWeights, MPCPlanner
from src.rl.replay_buffer import ReplayBuffer
from src.rl.trainer import TrainerConfig, WorldModelTrainer
from src.rl.world_model_baseline import BaselineWorldModel

logger = logging.getLogger("train_baseline")


# ------------------------------------------------------------------
# Config loading
# ------------------------------------------------------------------

def load_config(path: str) -> Dict[str, Any]:
    """Load a YAML config file and return a flat dict of parameters."""
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    # Flatten the nested structure into a single-level dict
    flat: Dict[str, Any] = {}
    if "goal" in raw:
        flat["goal_x"] = raw["goal"].get("x", 60.0)
        flat["goal_y"] = raw["goal"].get("y", 20.0)
        flat["goal_z"] = raw["goal"].get("z", -8.0)
    if "planner" in raw:
        flat["target_speed"] = raw["planner"].get("target_speed", 4.0)
        flat["control_dt"] = raw["planner"].get("control_dt", 0.4)
        flat["horizon"] = raw["planner"].get("horizon", 12)
        flat["num_samples"] = raw["planner"].get("num_samples", 120)
        flat["max_steps"] = raw["planner"].get("max_steps", 600)
    if "world_model" in raw:
        flat["state_dim"] = raw["world_model"].get("state_dim", 8)
        flat["action_dim"] = raw["world_model"].get("action_dim", 3)
        flat["learning_rate"] = raw["world_model"].get("learning_rate", 1e-3)
    if "training" in raw:
        flat["train_after_steps"] = raw["training"].get("train_after_steps", 40)
        flat["train_every"] = raw["training"].get("train_every", 4)
        flat["train_epochs"] = raw["training"].get("train_epochs", 2)
        flat["batch_size"] = raw["training"].get("batch_size", 128)
        flat["replay_buffer_size"] = raw["training"].get("replay_buffer_size", 50_000)
        flat["grad_clip"] = raw["training"].get("grad_clip_norm", 5.0)
    if "safety" in raw:
        flat["min_obstacle_dist"] = raw["safety"].get("min_obstacle_dist", 4.0)
        flat["goal_reach_dist"] = raw["safety"].get("goal_reach_dist", 3.0)
    if "cost_weights" in raw:
        flat["obstacle_cost_weight"] = raw["cost_weights"].get("obstacle", 15.0)
        flat["smooth_cost_weight"] = raw["cost_weights"].get("smooth", 0.15)
        flat["energy_cost_weight"] = raw["cost_weights"].get("energy", 0.02)
        flat["sparse_bonus"] = raw["cost_weights"].get("sparse_bonus", -120.0)
    if "env" in raw:
        flat["airsim_host"] = raw["env"].get("airsim_host", "127.0.0.1")
        flat["airsim_port"] = raw["env"].get("airsim_port", 41451)

    return flat


# ------------------------------------------------------------------
# Environment factory
# ------------------------------------------------------------------

def make_env(env_name: str, cfg: Dict[str, Any]) -> BaseNavigationEnv:
    """Instantiate a navigation environment by name."""
    goal = (cfg.get("goal_x", 60.0), cfg.get("goal_y", 20.0), cfg.get("goal_z", -8.0))

    if env_name == "mock":
        return MockNavigationEnv(
            goal=goal,
            dt=cfg.get("control_dt", 0.4),
            min_obstacle_dist=cfg.get("min_obstacle_dist", 4.0),
            goal_reach_dist=cfg.get("goal_reach_dist", 3.0),
            max_steps=cfg.get("max_steps", 600),
        )
    elif env_name == "airsim":
        # Lazy import so --help works without AirSim
        from src.env.airsim_adapter import AirSimNavigationEnv

        return AirSimNavigationEnv(
            goal=goal,
            target_altitude=cfg.get("goal_z", -8.0),
            control_dt=cfg.get("control_dt", 0.4),
            min_obstacle_dist=cfg.get("min_obstacle_dist", 4.0),
            goal_reach_dist=cfg.get("goal_reach_dist", 3.0),
            host=cfg.get("airsim_host"),
            port=cfg.get("airsim_port", 41451),
        )
    else:
        raise ValueError(f"Unknown environment: {env_name!r}")


# ------------------------------------------------------------------
# Safe action (mock version – no yaw available outside AirSim)
# ------------------------------------------------------------------

def safe_action_mock(
    action: np.ndarray, state: np.ndarray, min_obstacle_dist: float
) -> np.ndarray:
    """Simple obstacle-avoidance override for the mock environment.

    Without yaw information (no AirSim), we retreat in the opposite
    direction of the current velocity.
    """
    action = action.copy()
    obstacle_dist = state[7]
    if obstacle_dist < min_obstacle_dist:
        # Retreat opposite to current velocity
        vel = state[3:6]
        speed = float(np.linalg.norm(vel))
        if speed > 0.01:
            retreat_dir = -vel / speed
        else:
            retreat_dir = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        action[0] = 2.0 * retreat_dir[0]
        action[1] = 2.0 * retreat_dir[1]
        action[2] = -0.2
        logger.warning(
            "Obstacle avoidance override (dist=%.2f m)", obstacle_dist
        )
    return action


# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------

def run(args: argparse.Namespace) -> None:
    """Execute the baseline training loop."""
    # Load config
    cfg = load_config(args.config) if Path(args.config).exists() else {}

    # CLI overrides
    if args.max_steps is not None:
        cfg["max_steps"] = args.max_steps

    max_steps = cfg.get("max_steps", 600)
    state_dim = cfg.get("state_dim", 8)
    action_dim = cfg.get("action_dim", 3)

    # Device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info("Using device: %s", device)

    # Environment
    env = make_env(args.env, cfg)

    # World model
    model = BaselineWorldModel(state_dim, action_dim).to(device)
    optimizer = optim.Adam(model.parameters(), lr=cfg.get("learning_rate", 1e-3))

    # Replay buffer
    replay_buffer = ReplayBuffer(maxlen=cfg.get("replay_buffer_size", 50_000))

    # MPC planner
    goal = np.array(
        [cfg.get("goal_x", 60.0), cfg.get("goal_y", 20.0), cfg.get("goal_z", -8.0)],
        dtype=np.float32,
    )
    cost_weights = CostWeights(
        obstacle=cfg.get("obstacle_cost_weight", 15.0),
        smooth=cfg.get("smooth_cost_weight", 0.15),
        energy=cfg.get("energy_cost_weight", 0.02),
        sparse_bonus=cfg.get("sparse_bonus", -120.0),
    )
    planner = MPCPlanner(
        state_dim=state_dim,
        action_dim=action_dim,
        horizon=cfg.get("horizon", 12),
        num_samples=cfg.get("num_samples", 120),
        target_speed=cfg.get("target_speed", 4.0),
        cost_weights=cost_weights,
        goal=goal,
        min_obstacle_dist=cfg.get("min_obstacle_dist", 4.0),
        goal_reach_dist=cfg.get("goal_reach_dist", 3.0),
    )

    # Trainer
    trainer_config = TrainerConfig(
        batch_size=cfg.get("batch_size", 128),
        train_epochs=cfg.get("train_epochs", 2),
        grad_clip=cfg.get("grad_clip", 5.0),
        min_buffer_size=cfg.get("train_after_steps", 40),
    )
    trainer = WorldModelTrainer(model, optimizer, replay_buffer, device, trainer_config)

    # Reset environment
    obs = env.reset()
    state = obs.to_state_vector()
    last_action = np.zeros(action_dim, dtype=np.float32)
    train_every = cfg.get("train_every", 4)
    min_obstacle_dist = cfg.get("min_obstacle_dist", 4.0)

    logger.info("Starting baseline loop: env=%s  max_steps=%d", args.env, max_steps)
    start_time = time.time()

    try:
        for step in range(max_steps):
            # Plan
            action = planner.plan_action(state, model, device, last_action)

            # Safety override
            if args.env == "airsim":
                # AirSim env has its own safe_action with yaw
                action = env.safe_action(action, state)  # type: ignore[attr-defined]
            else:
                action = safe_action_mock(action, state, min_obstacle_dist)

            # Step
            obs, reward, done, info = env.step(action)
            next_state = obs.to_state_vector()

            # Remember
            replay_buffer.add(state, action, next_state)
            last_action = action.copy()

            # Train
            if not args.no_train and step % train_every == 0:
                loss = trainer.train_step()
                if loss is not None:
                    logger.debug("Step %04d  train_loss=%.6f", step, loss)

            state = next_state
            dist_goal = state[6]
            obs_dist = state[7]

            logger.info(
                "[STEP %04d] dist_goal=%6.2f m | obs=%5.2f m | action=%s",
                step, dist_goal, obs_dist, np.round(action, 2),
            )

            if done:
                if info.get("goal_reached", dist_goal < cfg.get("goal_reach_dist", 3.0)):
                    logger.info("SUCCESS – reached goal area.")
                elif info.get("collision", False):
                    logger.warning("COLLISION – episode terminated.")
                else:
                    logger.info("Episode ended (timeout or other).")
                break

        elapsed = time.time() - start_time
        logger.info(
            "Mission finished in %.1f s  |  %d steps  |  buffer=%d  |  updates=%d",
            elapsed, step + 1, len(replay_buffer), trainer.total_updates,
        )
    finally:
        env.close()


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train the baseline world-model-guided UAV navigation agent.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--config",
        default="configs/default.yaml",
        help="Path to YAML configuration file (default: configs/default.yaml).",
    )
    parser.add_argument(
        "--env",
        choices=["airsim", "mock"],
        default="mock",
        help="Environment backend (default: mock).",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="Override max episode steps from config.",
    )
    parser.add_argument(
        "--no-train",
        action="store_true",
        help="Disable world-model training (planning only).",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable DEBUG-level logging.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    run(args)


if __name__ == "__main__":
    main()
