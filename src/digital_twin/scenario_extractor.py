"""Scenario extraction from flight logs for corner-case replay.

Scans recorded trajectories for interesting events — near collisions,
high world-model uncertainty, goal failures, and sharp manoeuvres — and
packages each into a ``ScenarioSpec`` that the digital-twin pipeline can
replay or randomise.
"""

from __future__ import annotations

import json
import logging
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

try:
    import yaml  # optional — only needed for YAML log files
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]

from src.utils.data_types import ScenarioSpec

logger = logging.getLogger(__name__)


class ScenarioExtractor:
    """Extracts interesting scenarios (corner cases) from flight logs.

    Identifies moments where the UAV encountered:
    - Near-collisions (obstacle distance below threshold)
    - High uncertainty (world model was uncertain)
    - Goal failures (episode ended without reaching goal)
    - Sharp manoeuvres (sudden velocity changes)
    """

    def __init__(
        self,
        near_collision_threshold: float = 3.0,
        uncertainty_threshold: float = 0.7,
        velocity_change_threshold: float = 3.0,
        control_correction_threshold: float = 5.0,
        min_scenario_duration: int = 5,
    ) -> None:
        self.near_collision_threshold = near_collision_threshold
        self.uncertainty_threshold = uncertainty_threshold
        self.velocity_change_threshold = velocity_change_threshold
        self.control_correction_threshold = control_correction_threshold
        self.min_scenario_duration = min_scenario_duration
        logger.info(
            "ScenarioExtractor initialised — thresholds: collision=%.1f, "
            "uncertainty=%.2f, vel_change=%.1f, ctrl_correction=%.1f, min_dur=%d",
            near_collision_threshold,
            uncertainty_threshold,
            velocity_change_threshold,
            control_correction_threshold,
            min_scenario_duration,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def extract_from_trajectory(
        self, trajectory: List[Dict[str, Any]]
    ) -> List[ScenarioSpec]:
        """Extract corner-case scenarios from a recorded trajectory.

        Args:
            trajectory: List of dicts with keys:
                ``timestamp``, ``pose`` (3-list), ``velocity`` (3-list),
                ``obstacle_dist`` (float), ``uncertainty`` (float, optional).

        Returns:
            List of ``ScenarioSpec`` describing each detected corner case.
        """
        if not trajectory:
            logger.warning("Empty trajectory — nothing to extract.")
            return []

        scenarios: List[ScenarioSpec] = []

        # Detect near-collision windows
        scenarios.extend(self._detect_near_collisions(trajectory))

        # Detect high-uncertainty windows
        scenarios.extend(self._detect_high_uncertainty(trajectory))

        # Detect sharp manoeuvres
        scenarios.extend(self._detect_sharp_manoeuvres(trajectory))

        # Detect large control corrections
        scenarios.extend(self._detect_large_control_corrections(trajectory))

        # Detect goal failure (trajectory ended far from goal)
        goal_failure = self._detect_goal_failure(trajectory)
        if goal_failure is not None:
            scenarios.append(goal_failure)

        logger.info(
            "Extracted %d scenario(s) from trajectory of length %d.",
            len(scenarios),
            len(trajectory),
        )
        return scenarios

    def extract_from_log_file(self, log_path: str) -> List[ScenarioSpec]:
        """Load trajectory from a JSON or YAML log file and extract scenarios.

        The file must contain a top-level key ``"trajectory"`` whose value is
        a list of timestep dicts.
        """
        path = Path(log_path)
        if not path.exists():
            raise FileNotFoundError(f"Log file not found: {log_path}")

        text = path.read_text(encoding="utf-8")

        if path.suffix in (".yaml", ".yml"):
            if yaml is None:
                raise ImportError("PyYAML is required to read YAML log files.")
            data = yaml.safe_load(text)
        else:
            data = json.loads(text)

        trajectory = data.get("trajectory", data)
        if not isinstance(trajectory, list):
            raise ValueError("Expected a list under 'trajectory' key.")

        return self.extract_from_trajectory(trajectory)

    # ------------------------------------------------------------------
    # Private detection helpers
    # ------------------------------------------------------------------

    def _detect_near_collisions(
        self, trajectory: List[Dict[str, Any]]
    ) -> List[ScenarioSpec]:
        """Find windows where obstacle_dist < threshold."""
        return self._detect_threshold_violations(
            trajectory,
            key="obstacle_dist",
            threshold=self.near_collision_threshold,
            below=True,
            tag="near_collision",
        )

    def _detect_high_uncertainty(
        self, trajectory: List[Dict[str, Any]]
    ) -> List[ScenarioSpec]:
        """Find windows where uncertainty > threshold."""
        return self._detect_threshold_violations(
            trajectory,
            key="uncertainty",
            threshold=self.uncertainty_threshold,
            below=False,
            tag="high_uncertainty",
        )

    def _detect_sharp_manoeuvres(
        self, trajectory: List[Dict[str, Any]]
    ) -> List[ScenarioSpec]:
        """Find timesteps with large velocity change."""
        scenarios: List[ScenarioSpec] = []
        violation_start: Optional[int] = None

        for i in range(1, len(trajectory)):
            v_prev = np.asarray(trajectory[i - 1].get("velocity", [0, 0, 0]), dtype=np.float32)
            v_curr = np.asarray(trajectory[i].get("velocity", [0, 0, 0]), dtype=np.float32)
            delta = float(np.linalg.norm(v_curr - v_prev))

            if delta >= self.velocity_change_threshold:
                if violation_start is None:
                    violation_start = i - 1
            else:
                if violation_start is not None:
                    scenarios.extend(
                        self._maybe_create_scenario(trajectory, violation_start, i - 1, "sharp_manoeuvre")
                    )
                    violation_start = None

        # Handle window reaching the end
        if violation_start is not None:
            scenarios.extend(
                self._maybe_create_scenario(trajectory, violation_start, len(trajectory) - 1, "sharp_manoeuvre")
            )

        return scenarios

    def _detect_goal_failure(
        self, trajectory: List[Dict[str, Any]]
    ) -> Optional[ScenarioSpec]:
        """Check if the final position is far from any plausible goal.

        A simple heuristic: if the last step has ``obstacle_dist`` data and
        the trajectory has a ``goal`` key, use it.  Otherwise we look at
        whether the last step's metadata contains ``reached_goal=False``.
        """
        if not trajectory:
            return None
        last = trajectory[-1]
        reached = last.get("reached_goal", last.get("metadata", {}).get("reached_goal"))
        if reached is False:
            return self._build_scenario(
                trajectory,
                start_idx=max(0, len(trajectory) - self.min_scenario_duration),
                end_idx=len(trajectory) - 1,
                tag="goal_failure",
            )
        return None

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _detect_threshold_violations(
        self,
        trajectory: List[Dict[str, Any]],
        key: str,
        threshold: float,
        below: bool,
        tag: str,
    ) -> List[ScenarioSpec]:
        """Generic scanner for consecutive threshold violations."""
        scenarios: List[ScenarioSpec] = []
        violation_start: Optional[int] = None

        for i, step in enumerate(trajectory):
            value = step.get(key)
            if value is None:
                continue
            violated = (value < threshold) if below else (value > threshold)

            if violated:
                if violation_start is None:
                    violation_start = i
            else:
                if violation_start is not None:
                    scenarios.extend(
                        self._maybe_create_scenario(trajectory, violation_start, i - 1, tag)
                    )
                    violation_start = None

        if violation_start is not None:
            scenarios.extend(
                self._maybe_create_scenario(trajectory, violation_start, len(trajectory) - 1, tag)
            )

        return scenarios

    def _maybe_create_scenario(
        self,
        trajectory: List[Dict[str, Any]],
        start_idx: int,
        end_idx: int,
        tag: str,
    ) -> List[ScenarioSpec]:
        """Create a scenario only if the window is long enough."""
        duration = end_idx - start_idx + 1
        if duration < self.min_scenario_duration:
            return []
        return [self._build_scenario(trajectory, start_idx, end_idx, tag)]

    def _build_scenario(
        self,
        trajectory: List[Dict[str, Any]],
        start_idx: int,
        end_idx: int,
        tag: str,
    ) -> ScenarioSpec:
        """Build a ScenarioSpec from a trajectory window."""
        start_step = trajectory[start_idx]
        end_step = trajectory[end_idx]

        start_pos = tuple(start_step.get("pose", (0.0, 0.0, 0.0)))
        goal_pos = tuple(end_step.get("pose", (0.0, 0.0, 0.0)))

        # Collect any obstacles mentioned in the window
        obstacles: List[Dict[str, Any]] = []
        for step in trajectory[start_idx : end_idx + 1]:
            obs = step.get("obstacles")
            if obs:
                obstacles.extend(obs)

        scenario_id = f"{tag}_{uuid.uuid4().hex[:8]}"
        logger.debug(
            "Created scenario %s from indices [%d, %d].", scenario_id, start_idx, end_idx,
        )

        return ScenarioSpec(
            scenario_id=scenario_id,
            description=f"Auto-extracted {tag} scenario (steps {start_idx}-{end_idx})",
            start_position=(float(start_pos[0]), float(start_pos[1]), float(start_pos[2])),
            goal_position=(float(goal_pos[0]), float(goal_pos[1]), float(goal_pos[2])),
            obstacles=obstacles,
            metadata={
                "tag": tag,
                "start_idx": start_idx,
                "end_idx": end_idx,
                "duration_steps": end_idx - start_idx + 1,
            },
        )

    def _detect_large_control_corrections(
        self, trajectory: List[Dict[str, Any]]
    ) -> List[ScenarioSpec]:
        """Find windows where consecutive actions differ significantly.

        Uses the ``action`` key in each trajectory step (if available).
        """
        scenarios: List[ScenarioSpec] = []
        violation_start: Optional[int] = None

        for i in range(1, len(trajectory)):
            a_prev = np.asarray(
                trajectory[i - 1].get("action", [0, 0, 0]), dtype=np.float32
            )
            a_curr = np.asarray(
                trajectory[i].get("action", [0, 0, 0]), dtype=np.float32
            )
            delta = float(np.linalg.norm(a_curr - a_prev))

            if delta >= self.control_correction_threshold:
                if violation_start is None:
                    violation_start = i - 1
            else:
                if violation_start is not None:
                    scenarios.extend(
                        self._maybe_create_scenario(
                            trajectory, violation_start, i - 1,
                            "large_control_correction",
                        )
                    )
                    violation_start = None

        if violation_start is not None:
            scenarios.extend(
                self._maybe_create_scenario(
                    trajectory, violation_start, len(trajectory) - 1,
                    "large_control_correction",
                )
            )
        return scenarios

    # ------------------------------------------------------------------
    # Mock-episode convenience
    # ------------------------------------------------------------------

    def extract_from_mock_episode(
        self,
        env: Any,
        policy_fn: Any = None,
        max_steps: int = 200,
    ) -> tuple[List[Dict[str, Any]], List[ScenarioSpec]]:
        """Run one episode in *env*, record trajectory, and extract scenarios.

        Parameters
        ----------
        env:
            A ``BaseNavigationEnv`` instance (typically ``MockNavigationEnv``).
        policy_fn:
            Callable ``(state: np.ndarray) -> np.ndarray`` returning an action.
            If ``None``, uses a random policy.
        max_steps:
            Maximum episode steps.

        Returns
        -------
        trajectory:
            List of per-step dicts suitable for ``extract_from_trajectory``.
        scenarios:
            Extracted corner-case scenarios.
        """
        trajectory: List[Dict[str, Any]] = []
        obs = env.reset()
        state = obs.to_state_vector()
        last_action = np.zeros(3, dtype=np.float32)

        for step_idx in range(max_steps):
            if policy_fn is not None:
                action = policy_fn(state)
            else:
                action = np.array([
                    np.random.uniform(-4.0, 4.0),
                    np.random.uniform(-4.0, 4.0),
                    np.random.uniform(-1.0, 1.0),
                ], dtype=np.float32)

            obs, reward, done, info = env.step(action)
            next_state = obs.to_state_vector()

            trajectory.append({
                "timestamp": float(step_idx),
                "pose": list(obs.pose),
                "velocity": list(obs.velocity),
                "obstacle_dist": obs.obstacle_distance,
                "uncertainty": 0.0,
                "action": action.tolist(),
                "reward": float(reward),
                "reached_goal": info.get("goal_reached", False),
            })

            state = next_state
            last_action = action.copy()

            if done:
                break

        scenarios = self.extract_from_trajectory(trajectory)
        return trajectory, scenarios

