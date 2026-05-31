"""
Evaluation metrics for World-Model-Guided UAV Navigation.

Provides:
- EpisodeMetrics: per-episode data collected during rollout.
- MetricsTracker: accumulates episodes and computes aggregate statistics
  including success rate, collision rate, path efficiency, sparse reward
  success ratio, takeover frequency, uncertainty fallback counts,
  multi-agent conflict counts, and sim-to-real performance gap.
"""

import logging
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)


@dataclass
class EpisodeMetrics:
    """Metrics collected during a single episode.

    Attributes:
        success: Whether the agent reached the goal within tolerance.
        collision: Whether a collision occurred during the episode.
        total_return: Cumulative reward obtained over the episode.
        path_length: Actual path length traversed by the agent.
        optimal_path_length: Shortest feasible path length to the goal.
        steps: Number of environment steps taken.
        sparse_reward_achieved: Whether the sparse goal reward was obtained.
        takeover_count: Number of times the safety controller took over.
        uncertainty_fallback_count: Number of uncertainty-triggered fallbacks.
        multi_agent_conflicts: Number of inter-agent spatial conflicts detected.
    """

    success: bool = False
    collision: bool = False
    total_return: float = 0.0
    path_length: float = 0.0
    optimal_path_length: float = 0.0
    steps: int = 0
    sparse_reward_achieved: bool = False
    takeover_count: int = 0
    uncertainty_fallback_count: int = 0
    multi_agent_conflicts: int = 0


class MetricsTracker:
    """Accumulates metrics across episodes and computes aggregate statistics.

    Usage::

        tracker = MetricsTracker()
        for episode in range(num_episodes):
            metrics = run_episode(env, policy)
            tracker.record_episode(metrics)
        print(tracker.summary())
    """

    def __init__(self) -> None:
        self.episodes: List[EpisodeMetrics] = []

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def record_episode(self, metrics: EpisodeMetrics) -> None:
        """Append a completed episode's metrics to the tracker."""
        self.episodes.append(metrics)
        logger.debug(
            "Recorded episode %d: success=%s, return=%.3f, steps=%d",
            len(self.episodes),
            metrics.success,
            metrics.total_return,
            metrics.steps,
        )

    # ------------------------------------------------------------------
    # Aggregate properties
    # ------------------------------------------------------------------

    @property
    def success_rate(self) -> float:
        """Fraction of episodes that reached the goal."""
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.success) / len(self.episodes)

    @property
    def collision_rate(self) -> float:
        """Fraction of episodes where a collision occurred."""
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.collision) / len(self.episodes)

    @property
    def path_efficiency(self) -> float:
        """Average ratio of optimal_path_length / actual_path_length.

        Episodes with zero actual path length are skipped.  Returns 0.0
        when no valid episodes exist.
        """
        ratios: List[float] = []
        for e in self.episodes:
            if e.path_length > 0.0:
                ratios.append(e.optimal_path_length / e.path_length)
        if not ratios:
            return 0.0
        return sum(ratios) / len(ratios)

    @property
    def average_return(self) -> float:
        """Mean cumulative return across recorded episodes."""
        if not self.episodes:
            return 0.0
        return sum(e.total_return for e in self.episodes) / len(self.episodes)

    @property
    def sparse_reward_success_ratio(self) -> float:
        """Fraction of episodes where the sparse goal reward was obtained."""
        if not self.episodes:
            return 0.0
        return sum(1 for e in self.episodes if e.sparse_reward_achieved) / len(
            self.episodes
        )

    @property
    def takeover_frequency(self) -> float:
        """Average takeover count per episode."""
        if not self.episodes:
            return 0.0
        return sum(e.takeover_count for e in self.episodes) / len(self.episodes)

    @property
    def uncertainty_fallback_count(self) -> float:
        """Average uncertainty-triggered fallback count per episode."""
        if not self.episodes:
            return 0.0
        return sum(e.uncertainty_fallback_count for e in self.episodes) / len(
            self.episodes
        )

    @property
    def multi_agent_conflict_count(self) -> float:
        """Average multi-agent conflict count per episode."""
        if not self.episodes:
            return 0.0
        return sum(e.multi_agent_conflicts for e in self.episodes) / len(
            self.episodes
        )

    # ------------------------------------------------------------------
    # Sim-to-real gap
    # ------------------------------------------------------------------

    def sim2real_performance_gap(self, sim_tracker: "MetricsTracker") -> float:
        """Compute performance gap between this tracker (real) and *sim_tracker*.

        The gap is defined as ``sim_success_rate - real_success_rate``.  A
        positive value means the policy degrades when transferred to the real
        domain; a value near zero indicates successful sim-to-real transfer.
        """
        return sim_tracker.success_rate - self.success_rate

    # ------------------------------------------------------------------
    # Summary / reset
    # ------------------------------------------------------------------

    def summary(self) -> Dict[str, float]:
        """Return a dictionary of all aggregate metrics."""
        return {
            "num_episodes": float(len(self.episodes)),
            "success_rate": self.success_rate,
            "collision_rate": self.collision_rate,
            "path_efficiency": self.path_efficiency,
            "average_return": self.average_return,
            "sparse_reward_success_ratio": self.sparse_reward_success_ratio,
            "takeover_frequency": self.takeover_frequency,
            "uncertainty_fallback_count": self.uncertainty_fallback_count,
            "multi_agent_conflict_count": self.multi_agent_conflict_count,
        }

    def reset(self) -> None:
        """Clear all recorded episodes."""
        self.episodes.clear()
        logger.info("MetricsTracker reset.")
