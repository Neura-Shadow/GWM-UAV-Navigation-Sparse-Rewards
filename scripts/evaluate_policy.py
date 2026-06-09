#!/usr/bin/env python3
"""Evaluate a navigation policy and report aggregate metrics.

This script runs a policy inside an environment for ``--num-episodes``
episodes and prints summary statistics (success rate, collision rate, path
efficiency, etc.) using ``src.evaluation.metrics.MetricsTracker``.

When ``--env mock`` is specified (default), the script synthesises random
mock episodes so it can run anywhere without a GPU, AirSim, or any
simulator installed. ``--env airsim`` is intentionally not routed through this
legacy evaluator; use ``scripts/run_multisim_gwm_demo.py`` or the guarded
AirSim-family validation scripts for optional simulator-backed checks.

Example::

    python scripts/evaluate_policy.py --num-episodes 50 --env mock
    python scripts/evaluate_policy.py --config configs/eval.yaml \\
           --policy-path outputs/world_model/best.pt --env mock
"""

from __future__ import annotations

import argparse
import json
import logging
import random
import sys
from pathlib import Path
from typing import Dict

logger = logging.getLogger(__name__)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments for policy evaluation."""
    parser = argparse.ArgumentParser(
        description="Evaluate a navigation policy and report metrics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Optional path to YAML evaluation configuration file.",
    )
    parser.add_argument(
        "--policy-path",
        type=str,
        default=None,
        help="Path to a saved policy checkpoint (unused in mock mode).",
    )
    parser.add_argument(
        "--env",
        type=str,
        choices=["airsim", "mock"],
        default="mock",
        help="Environment backend to evaluate in.",
    )
    parser.add_argument(
        "--num-episodes",
        type=int,
        default=20,
        help="Number of evaluation episodes to run.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Optional path to save JSON metrics summary.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility.",
    )
    return parser.parse_args(argv)


def _load_config(config_path: str | None) -> dict:
    """Load a YAML config file if provided and exists."""
    if config_path is None:
        return {}
    path = Path(config_path)
    if path.exists():
        try:
            import yaml

            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except ImportError:
            logger.warning("pyyaml not installed; skipping config load.")
    else:
        logger.warning("Config file %s not found; using defaults.", config_path)
    return {}


def _run_mock_episode(rng: random.Random) -> Dict[str, object]:
    """Simulate a single mock episode with random outcomes.

    Returns a dict matching the ``EpisodeMetrics`` fields.
    """
    steps = rng.randint(50, 300)
    success = rng.random() < 0.6
    collision = (not success) and (rng.random() < 0.4)
    path_length = rng.uniform(30.0, 120.0)
    optimal = path_length * rng.uniform(0.5, 0.95)

    return {
        "success": success,
        "collision": collision,
        "total_return": rng.uniform(-5.0, 10.0) if success else rng.uniform(-10.0, 0.0),
        "path_length": path_length,
        "optimal_path_length": optimal,
        "steps": steps,
        "sparse_reward_achieved": success and rng.random() < 0.8,
        "takeover_count": rng.randint(0, 3),
        "uncertainty_fallback_count": rng.randint(0, 5),
        "multi_agent_conflicts": rng.randint(0, 2),
    }


def main(argv: list[str] | None = None) -> None:
    """Entry point for policy evaluation."""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = parse_args(argv)

    config = _load_config(args.config)
    rng = random.Random(args.seed)

    # Import metrics from the project
    try:
        from src.evaluation.metrics import EpisodeMetrics, MetricsTracker
    except ImportError:
        # Fallback: add project root to path
        project_root = str(Path(__file__).resolve().parent.parent)
        sys.path.insert(0, project_root)
        from src.evaluation.metrics import EpisodeMetrics, MetricsTracker

    logger.info("=" * 60)
    logger.info("Policy Evaluation")
    logger.info("=" * 60)
    logger.info("Environment  : %s", args.env)
    logger.info("Policy path  : %s", args.policy_path or "(random / mock)")
    logger.info("Num episodes : %d", args.num_episodes)
    logger.info("Seed         : %d", args.seed)

    if args.env == "airsim":
        logger.error("AirSim-family evaluation is intentionally not handled by this legacy evaluator.")
        logger.error("Use --env mock here, or use run_multisim_gwm_demo.py with explicit runtime gates.")
        sys.exit(1)

    # Run mock episodes
    tracker = MetricsTracker()
    for ep in range(1, args.num_episodes + 1):
        result = _run_mock_episode(rng)
        metrics = EpisodeMetrics(**result)
        tracker.record_episode(metrics)
        if ep % max(1, args.num_episodes // 5) == 0:
            logger.info(
                "  Episode %d/%d — success=%s, return=%.2f",
                ep,
                args.num_episodes,
                metrics.success,
                metrics.total_return,
            )

    # Print summary
    summary = tracker.summary()
    logger.info("-" * 60)
    logger.info("Aggregate Metrics:")
    for key, value in summary.items():
        logger.info("  %-35s %.4f", key, value)
    logger.info("=" * 60)

    # Optionally save to file
    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        logger.info("Metrics saved to %s", args.output)


if __name__ == "__main__":
    main()
