"""Generated-world-model candidate planner skeleton."""

from __future__ import annotations

from typing import Any, Dict, Sequence

import torch

from src.generated_world_model.rollout import AutoregressiveRollout
from src.generated_world_model.trajectory_sampler import CandidateTrajectorySampler
from src.generated_world_model.trajectory_scorer import TrajectoryScorer
from src.generated_world_model.types import ObservationBatch, TrajectoryCandidate


class GeneratedWorldModelPlanner:
    """Rank sampled trajectories using generated future observations."""

    def __init__(
        self,
        rollout: AutoregressiveRollout,
        scorer: TrajectoryScorer | None = None,
        sampler: CandidateTrajectorySampler | None = None,
    ) -> None:
        self.rollout = rollout
        self.scorer = scorer or TrajectoryScorer()
        self.sampler = sampler or CandidateTrajectorySampler()

    def plan(
        self,
        context: ObservationBatch,
        start: Sequence[float],
        goal: Sequence[float],
        safety_context: Dict[str, Any] | None = None,
        num_candidates: int = 8,
    ) -> Dict[str, Any]:
        """Return the best candidate and score from a small candidate set."""
        candidates = self.sampler.sample(start=start, goal=goal, num_candidates=num_candidates)
        scored = []
        for candidate in candidates:
            actions = torch.as_tensor(candidate.actions, dtype=torch.float32).unsqueeze(0)
            actions = actions.expand(context.batch_size, -1, -1).contiguous()
            generated = self.rollout.rollout(context, actions, horizon=actions.shape[1])
            score = self.scorer.score(generated, candidate, goal, safety_context or {})
            scored.append((candidate, score))

        best_candidate, best_score = max(scored, key=lambda item: item[1]["total_score"])
        return {
            "candidate": best_candidate,
            "score": best_score,
            "all_scores": [score for _, score in scored],
        }
