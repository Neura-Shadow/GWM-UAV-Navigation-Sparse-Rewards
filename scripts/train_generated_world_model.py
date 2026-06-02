#!/usr/bin/env python3
"""Train the Phase 4-A Generated World Model baseline on synthetic data."""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any, Dict

import torch
import torch.optim as optim

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is a repository dependency
    yaml = None  # type: ignore[assignment]

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.generated_world_model import (  # noqa: E402
    GWMConfig,
    build_baseline_components,
    make_synthetic_training_batch,
    train_synthetic_step,
)

logger = logging.getLogger("train_generated_world_model")


def load_config(path: str | None) -> Dict[str, Any]:
    """Load a YAML config, returning an empty dict when omitted."""
    if path is None:
        return {}
    if yaml is None:
        raise RuntimeError("PyYAML is required to load generated world model config files.")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("Config root must be a mapping.")
    return data


def run(args: argparse.Namespace) -> None:
    """Run synthetic CPU training."""
    if not args.synthetic:
        raise RuntimeError("Phase 4-A only supports --synthetic training data.")

    raw_config = load_config(args.config)
    model_config = dict(raw_config.get("model", {}))
    for key, value in {
        "image_height": args.image_height,
        "image_width": args.image_width,
        "context_length": args.context_length,
        "horizon": args.horizon,
        "latent_dim": args.latent_dim,
        "hidden_dim": args.hidden_dim,
    }.items():
        if value is not None:
            model_config[key] = value
    config = GWMConfig.from_any(model_config)

    device = torch.device("cpu")
    torch.manual_seed(args.seed)
    encoder, conditioner, model = build_baseline_components(config)
    encoder.to(device)
    conditioner.to(device)
    model.to(device)

    optimizer = optim.Adam(
        list(encoder.parameters()) + list(conditioner.parameters()) + list(model.parameters()),
        lr=args.lr,
    )

    last_metrics: Dict[str, float] = {}
    logger.info(
        "Training Phase 4-A GWM baseline: steps=%d batch=%d horizon=%d image=%dx%d",
        args.steps,
        args.batch_size,
        config.horizon,
        config.image_height,
        config.image_width,
    )
    for step in range(1, args.steps + 1):
        batch = make_synthetic_training_batch(
            config=config,
            batch_size=args.batch_size,
            seed=args.seed + step,
        )
        last_metrics = train_synthetic_step(
            encoder=encoder,
            conditioner=conditioner,
            model=model,
            optimizer=optimizer,
            batch=batch,
            device=device,
        )
        if step == 1 or step == args.steps or step % max(1, args.steps // 5) == 0:
            logger.info("step=%d/%d loss=%.6f", step, args.steps, last_metrics["loss"])

    if args.checkpoint_dir:
        checkpoint_dir = Path(args.checkpoint_dir)
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        checkpoint_path = checkpoint_dir / "generated_world_model_phase4a.pt"
        torch.save(
            {
                "config": config.__dict__,
                "encoder": encoder.state_dict(),
                "conditioner": conditioner.state_dict(),
                "model": model.state_dict(),
                "metrics": last_metrics,
            },
            checkpoint_path,
        )
        logger.info("checkpoint=%s", checkpoint_path)

    logger.info("final_loss=%.6f", last_metrics.get("loss", float("nan")))
    print(f"final_loss={last_metrics.get('loss', float('nan')):.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train the lightweight Phase 4-A Generated World Model baseline.",
    )
    parser.add_argument("--synthetic", action="store_true", help="Use synthetic tensor batches.")
    parser.add_argument("--steps", type=int, default=20, help="Training steps.")
    parser.add_argument("--batch-size", type=int, default=4, help="Synthetic batch size.")
    parser.add_argument("--context-length", type=int, default=None, help="Past context frames.")
    parser.add_argument("--horizon", type=int, default=None, help="Future prediction horizon.")
    parser.add_argument("--image-height", type=int, default=None, help="Synthetic image height.")
    parser.add_argument("--image-width", type=int, default=None, help="Synthetic image width.")
    parser.add_argument("--latent-dim", type=int, default=None, help="Latent size.")
    parser.add_argument("--hidden-dim", type=int, default=None, help="Dynamics hidden size.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed.")
    parser.add_argument("--config", type=str, default=None, help="Optional YAML config.")
    parser.add_argument("--checkpoint-dir", type=str, default=None, help="Optional checkpoint dir.")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable debug logging.")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    run(args)


if __name__ == "__main__":
    main()
