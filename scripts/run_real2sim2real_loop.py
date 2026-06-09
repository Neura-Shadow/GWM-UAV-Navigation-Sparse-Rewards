#!/usr/bin/env python3
"""Real2Sim2Real pipeline loop (mock mode).

Demonstrates the full pipeline:
1. Run a mock episode to generate a real-world-like trajectory
2. Extract corner-case scenarios from the trajectory
3. Generate domain-randomised variants of each scenario
4. Summarise mock training/evaluation coverage for each variant
5. Output a JSON summary report

Usage
-----
    # Basic mock pipeline:
    python scripts/run_real2sim2real_loop.py --mock

    # Custom settings:
    python scripts/run_real2sim2real_loop.py --mock --episode-steps 300 --variants 5 --output-dir results/r2s2r
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

# Ensure the project root is importable when running the script directly.
_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.digital_twin.domain_randomization import DomainRandomizer
from src.digital_twin.scenario_extractor import ScenarioExtractor
from src.env.mock_env import MockNavigationEnv
from src.utils.data_types import ScenarioSpec

logger = logging.getLogger("real2sim2real")


# ------------------------------------------------------------------
# Pipeline steps
# ------------------------------------------------------------------

def step1_collect_trajectory(
    env: MockNavigationEnv,
    extractor: ScenarioExtractor,
    max_steps: int,
) -> tuple[List[Dict[str, Any]], List[ScenarioSpec]]:
    """Run a mock episode and extract scenarios."""
    logger.info("Step 1: Collecting trajectory (%d max steps)...", max_steps)
    trajectory, scenarios = extractor.extract_from_mock_episode(
        env=env, policy_fn=None, max_steps=max_steps,
    )
    logger.info(
        "  -> Trajectory length: %d steps, %d scenarios extracted.",
        len(trajectory), len(scenarios),
    )
    return trajectory, scenarios


def step2_generate_variants(
    scenarios: List[ScenarioSpec],
    randomizer: DomainRandomizer,
    num_variants: int,
) -> Dict[str, List[ScenarioSpec]]:
    """Generate domain-randomised variants of each extracted scenario."""
    logger.info("Step 2: Generating %d variants per scenario...", num_variants)
    all_variants: Dict[str, List[ScenarioSpec]] = {}
    for scenario in scenarios:
        variants = randomizer.randomize(scenario, num_variations=num_variants)
        all_variants[scenario.scenario_id] = variants
        logger.info(
            "  -> Scenario '%s' (%s): %d variants.",
            scenario.scenario_id,
            scenario.metadata.get("tag", "unknown"),
            len(variants),
        )
    return all_variants


def step3_simulate_training(
    all_variants: Dict[str, List[ScenarioSpec]],
) -> List[Dict[str, Any]]:
    """Summarise scoped mock training/evaluation coverage for each variant.

    Loading variants into a live simulator and training policies against them
    is a planned research extension. The current mock-first pipeline records
    deterministic per-variant coverage metadata for reports and tests.
    """
    logger.info("Step 3: Summarising mock training/evaluation coverage for variants...")
    results: List[Dict[str, Any]] = []
    for scenario_id, variants in all_variants.items():
        for variant in variants:
            results.append({
                "base_scenario": scenario_id,
                "variant_id": variant.scenario_id,
                "tag": variant.metadata.get("tag", "unknown"),
                "weather": variant.weather,
                "physics": variant.physics,
                "lighting": variant.metadata.get("lighting"),
                "status": "simulated",
            })
    logger.info("  -> Processed %d total variants.", len(results))
    return results


def step4_write_report(
    trajectory_length: int,
    scenarios: List[ScenarioSpec],
    training_results: List[Dict[str, Any]],
    output_dir: str,
    elapsed: float,
) -> str:
    """Write a JSON summary report."""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    report_path = str(out / "r2s2r_report.json")

    report = {
        "pipeline": "Real2Sim2Real (mock mode)",
        "trajectory_length": trajectory_length,
        "scenarios_extracted": len(scenarios),
        "scenario_details": [
            {
                "scenario_id": s.scenario_id,
                "tag": s.metadata.get("tag", "unknown"),
                "duration_steps": s.metadata.get("duration_steps", 0),
            }
            for s in scenarios
        ],
        "total_variants_processed": len(training_results),
        "training_results": training_results,
        "elapsed_seconds": round(elapsed, 2),
    }

    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    logger.info("Step 4: Report written to %s", report_path)
    return report_path


# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------

def run(args: argparse.Namespace) -> None:
    """Execute the Real2Sim2Real pipeline."""
    t0 = time.time()

    # Environment
    env = MockNavigationEnv(
        goal=(60.0, 20.0, -8.0),
        dt=0.4,
        max_steps=args.episode_steps,
    )

    # Extractor
    extractor = ScenarioExtractor(
        near_collision_threshold=3.0,
        uncertainty_threshold=0.7,
        velocity_change_threshold=3.0,
        control_correction_threshold=5.0,
        min_scenario_duration=3,
    )

    # Domain randomizer
    randomizer = DomainRandomizer(seed=args.seed)

    # Pipeline
    trajectory, scenarios = step1_collect_trajectory(env, extractor, args.episode_steps)
    all_variants = step2_generate_variants(scenarios, randomizer, args.variants)
    training_results = step3_simulate_training(all_variants)

    elapsed = time.time() - t0
    report_path = step4_write_report(
        len(trajectory), scenarios, training_results, args.output_dir, elapsed,
    )

    # Summary
    logger.info("=" * 60)
    logger.info("Real2Sim2Real Pipeline Summary")
    logger.info("=" * 60)
    logger.info("Mode             : mock")
    logger.info("Trajectory steps : %d", len(trajectory))
    logger.info("Scenarios found  : %d", len(scenarios))
    logger.info("Variants created : %d", sum(len(v) for v in all_variants.values()))
    logger.info("Time             : %.1f s", elapsed)
    logger.info("Report           : %s", report_path)
    logger.info("=" * 60)

    env.close()


# ------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the Real2Sim2Real pipeline loop.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mock", action="store_true", default=True,
        help="Use mock environment (default: True).",
    )
    parser.add_argument(
        "--episode-steps", type=int, default=200,
        help="Maximum steps per mock episode (default: 200).",
    )
    parser.add_argument(
        "--variants", type=int, default=3,
        help="Domain-randomised variants per scenario (default: 3).",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="RNG seed for domain randomization (default: 42).",
    )
    parser.add_argument(
        "--output-dir", type=str, default="results/r2s2r",
        help="Output directory for the report (default: results/r2s2r).",
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
