#!/usr/bin/env python3
"""Train the latent world model on environment transitions.

Collects transition data from a mock environment using random actions,
then trains either the baseline MLP or the latent encoder-dynamics-decoder
world model.

Usage
-----
    # Quick training run (mock env, 100 training steps):
    python scripts/train_world_model.py --env mock --steps 100

    # Train the latent world model:
    python scripts/train_world_model.py --env mock --steps 200 --model latent

    # Custom data collection:
    python scripts/train_world_model.py --env mock --collect-steps 500 --steps 300
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.optim as optim

# Ensure the project root is importable when running the script directly.
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.env.mock_env import MockNavigationEnv
from src.rl.replay_buffer import ReplayBuffer
from src.rl.trainer import TrainerConfig, WorldModelTrainer
from src.rl.world_model_baseline import BaselineWorldModel
from src.world_model.latent_world_model import LatentWorldModel

logger = logging.getLogger("train_world_model")


# ------------------------------------------------------------------
# Data collection
# ------------------------------------------------------------------

def collect_transitions(
    env: MockNavigationEnv,
    replay_buffer: ReplayBuffer,
    num_steps: int,
) -> int:
    """Run random actions in the environment and fill the replay buffer.

    Returns the number of transitions collected.
    """
    obs = env.reset()
    state = obs.to_state_vector()
    collected = 0

    for _ in range(num_steps):
        action = np.array([
            np.random.uniform(-4.0, 4.0),
            np.random.uniform(-4.0, 4.0),
            np.random.uniform(-1.0, 1.0),
        ], dtype=np.float32)

        obs, _reward, done, _info = env.step(action)
        next_state = obs.to_state_vector()

        replay_buffer.add(state, action, next_state)
        collected += 1

        if done:
            obs = env.reset()
            next_state = obs.to_state_vector()

        state = next_state

    return collected


# ------------------------------------------------------------------
# Model factory
# ------------------------------------------------------------------

def make_model(
    model_type: str,
    state_dim: int = 8,
    action_dim: int = 3,
    latent_dim: int = 32,
    hidden_dim: int = 128,
) -> torch.nn.Module:
    """Instantiate a world model by type name."""
    if model_type == "baseline":
        return BaselineWorldModel(state_dim=state_dim, action_dim=action_dim)
    elif model_type == "latent":
        return LatentWorldModel(
            state_dim=state_dim,
            action_dim=action_dim,
            latent_dim=latent_dim,
            hidden_dim=hidden_dim,
        )
    else:
        raise ValueError(f"Unknown model type: {model_type!r}")


# ------------------------------------------------------------------
# Main training loop
# ------------------------------------------------------------------

def run(args: argparse.Namespace) -> None:
    """Execute the world model training pipeline."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info("Using device: %s", device)

    # Create environment
    env = MockNavigationEnv(
        goal=(60.0, 20.0, -8.0),
        dt=0.4,
        max_steps=600,
    )

    # Replay buffer
    replay_buffer = ReplayBuffer(maxlen=50_000)

    # Collect transition data
    logger.info("Collecting %d transitions from mock environment...", args.collect_steps)
    t0 = time.time()
    n_collected = collect_transitions(env, replay_buffer, args.collect_steps)
    t_collect = time.time() - t0
    logger.info("Collected %d transitions in %.1f s", n_collected, t_collect)

    # Create model
    model = make_model(
        model_type=args.model,
        state_dim=env.state_dim,
        action_dim=env.action_dim,
        latent_dim=args.latent_dim,
        hidden_dim=args.hidden_dim,
    ).to(device)
    logger.info("Model: %s (%d parameters)", args.model, sum(p.numel() for p in model.parameters()))

    # Optimizer and trainer
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    trainer_config = TrainerConfig(
        batch_size=args.batch_size,
        train_epochs=1,  # one gradient step per train_step call
        grad_clip=5.0,
        min_buffer_size=args.batch_size,
    )
    trainer = WorldModelTrainer(model, optimizer, replay_buffer, device, trainer_config)

    # Training loop
    logger.info("Training for %d steps (batch_size=%d, lr=%.1e)...", args.steps, args.batch_size, args.lr)
    t0 = time.time()
    losses = []

    for step in range(1, args.steps + 1):
        loss = trainer.train_step()
        if loss is not None:
            losses.append(loss)

        if step % max(1, args.steps // 10) == 0 or step == 1:
            recent_loss = np.mean(losses[-10:]) if losses else float("nan")
            logger.info(
                "[STEP %04d/%04d] loss=%.6f  updates=%d",
                step, args.steps, recent_loss, trainer.total_updates,
            )

    elapsed = time.time() - t0

    # Save checkpoint
    ckpt_dir = Path(args.checkpoint_dir)
    ckpt_path = str(ckpt_dir / f"world_model_{args.model}.pt")
    final_loss = np.mean(losses[-10:]) if losses else float("nan")
    trainer.save_checkpoint(
        ckpt_path,
        model_type=args.model,
        config=vars(args),
        final_loss=None if np.isnan(final_loss) else float(final_loss),
    )

    # Summary

    logger.info("=" * 60)
    logger.info("Training Summary")
    logger.info("=" * 60)
    logger.info("Model type       : %s", args.model)
    logger.info("Training steps   : %d", args.steps)
    logger.info("Total updates    : %d", trainer.total_updates)
    logger.info("Final loss (avg) : %.6f", final_loss)
    logger.info("Time             : %.1f s", elapsed)
    logger.info("Checkpoint       : %s", ckpt_path)
    logger.info("=" * 60)

    env.close()


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train the world model on collected environment transitions.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--env", choices=["mock"], default="mock",
        help="Environment backend (default: mock).",
    )
    parser.add_argument(
        "--model", choices=["baseline", "latent"], default="baseline",
        help="World model type (default: baseline).",
    )
    parser.add_argument(
        "--steps", type=int, default=100,
        help="Number of training steps (default: 100).",
    )
    parser.add_argument(
        "--collect-steps", type=int, default=200,
        help="Number of env steps for data collection (default: 200).",
    )
    parser.add_argument(
        "--batch-size", type=int, default=64,
        help="Mini-batch size (default: 64).",
    )
    parser.add_argument(
        "--lr", type=float, default=1e-3,
        help="Learning rate (default: 1e-3).",
    )
    parser.add_argument(
        "--latent-dim", type=int, default=32,
        help="Latent dimension for latent model (default: 32).",
    )
    parser.add_argument(
        "--hidden-dim", type=int, default=128,
        help="Hidden layer width for dynamics MLP (default: 128).",
    )
    parser.add_argument(
        "--checkpoint-dir", type=str, default="checkpoints",
        help="Directory for saving checkpoints (default: checkpoints).",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
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
