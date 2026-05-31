"""Simulation-to-real transfer manager.

Tracks the lifecycle of policy versions trained in simulation and their
deployment to real-world vehicles.  Provides a simple ledger-style API
for registering training runs, querying the latest policy, logging
deployment outcomes, and estimating the sim-to-real performance gap.
"""

from __future__ import annotations

import logging
import time
import uuid
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class Sim2RealManager:
    """Manages the simulation-to-real-world policy transfer pipeline.

    Tracks which simulation scenarios have been used for training,
    policy versions, and deployment history.
    """

    def __init__(self) -> None:
        self.training_history: List[Dict[str, Any]] = []
        self.policy_versions: List[Dict[str, Any]] = []
        self.deployment_log: List[Dict[str, Any]] = []
        logger.info("Sim2RealManager initialised.")

    # ------------------------------------------------------------------
    # Training
    # ------------------------------------------------------------------

    def register_training_run(
        self,
        scenarios: List[str],
        policy_path: str,
        metrics: Dict[str, float],
    ) -> str:
        """Register a completed training run.

        Args:
            scenarios: List of scenario IDs used during training.
            policy_path: Filesystem path to the saved policy checkpoint.
            metrics: Evaluation metrics from simulation (e.g. success_rate).

        Returns:
            A unique version string for the newly registered policy.
        """
        version_id = f"v{len(self.policy_versions) + 1}_{uuid.uuid4().hex[:6]}"
        record: Dict[str, Any] = {
            "version": version_id,
            "scenarios": list(scenarios),
            "policy_path": policy_path,
            "metrics": dict(metrics),
            "timestamp": time.time(),
        }
        self.training_history.append(record)
        self.policy_versions.append(record)
        logger.info(
            "Registered training run '%s' — %d scenario(s), metrics=%s.",
            version_id,
            len(scenarios),
            metrics,
        )
        return version_id

    # ------------------------------------------------------------------
    # Policy retrieval
    # ------------------------------------------------------------------

    def get_latest_policy(self) -> Optional[Dict[str, Any]]:
        """Return the latest registered policy record, or *None*."""
        if not self.policy_versions:
            logger.warning("No policy versions registered yet.")
            return None
        return self.policy_versions[-1]

    def get_policy_by_version(self, version: str) -> Optional[Dict[str, Any]]:
        """Look up a policy record by its version string."""
        for record in self.policy_versions:
            if record["version"] == version:
                return record
        logger.warning("Policy version '%s' not found.", version)
        return None

    # ------------------------------------------------------------------
    # Deployment tracking
    # ------------------------------------------------------------------

    def log_deployment(
        self,
        policy_version: str,
        environment: str,
        outcome: Dict[str, Any],
    ) -> None:
        """Log a real-world deployment result.

        Args:
            policy_version: Version string of the deployed policy.
            environment: Identifier for the deployment environment.
            outcome: Dict with at least ``success_rate`` and any other
                metrics observed in the real world.
        """
        entry: Dict[str, Any] = {
            "policy_version": policy_version,
            "environment": environment,
            "outcome": dict(outcome),
            "timestamp": time.time(),
        }
        self.deployment_log.append(entry)
        logger.info(
            "Deployment logged: policy=%s, env=%s, outcome=%s.",
            policy_version,
            environment,
            outcome,
        )

    # ------------------------------------------------------------------
    # Analytics
    # ------------------------------------------------------------------

    def get_sim2real_gap(self) -> Optional[float]:
        """Estimate the sim-to-real performance gap.

        Computes the difference between the average simulation
        ``success_rate`` and the average real-world ``success_rate``
        across all deployment records.

        Returns:
            Gap value (sim - real), or *None* if data is insufficient.
        """
        if not self.deployment_log or not self.policy_versions:
            logger.warning("Not enough data to compute sim2real gap.")
            return None

        sim_rates = [
            r["metrics"]["success_rate"]
            for r in self.policy_versions
            if "success_rate" in r.get("metrics", {})
        ]
        real_rates = [
            d["outcome"]["success_rate"]
            for d in self.deployment_log
            if "success_rate" in d.get("outcome", {})
        ]

        if not sim_rates or not real_rates:
            logger.warning("Missing success_rate in sim or real records.")
            return None

        gap = float(sum(sim_rates) / len(sim_rates) - sum(real_rates) / len(real_rates))
        logger.info("Sim2Real gap: %.4f (sim avg=%.4f, real avg=%.4f).",
                     gap,
                     sum(sim_rates) / len(sim_rates),
                     sum(real_rates) / len(real_rates))
        return gap
