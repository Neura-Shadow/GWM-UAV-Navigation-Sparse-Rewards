"""Tests for Phase 2-B: Sparse Reward Curriculum.

Covers CurriculumScheduler level management, env_overrides,
serialisation, and integration with MockNavigationEnv.
"""

from __future__ import annotations

import sys
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

import numpy as np
import pytest

from src.env.mock_env import MockNavigationEnv
from src.rl.curriculum import CurriculumConfig, CurriculumScheduler


# ======================================================================
# TestCurriculumScheduler
# ======================================================================

class TestCurriculumScheduler:
    """Unit tests for the CurriculumScheduler."""

    def test_initial_level_is_zero(self):
        sched = CurriculumScheduler()
        assert sched.level == 0

    def test_level_advances_on_high_success_rate(self):
        sched = CurriculumScheduler(CurriculumConfig(window_size=10))
        for _ in range(10):
            sched.report_episode(True)
        assert sched.level == 1

    def test_level_does_not_advance_below_threshold(self):
        sched = CurriculumScheduler(CurriculumConfig(
            window_size=10, success_rate_threshold=0.7
        ))
        # 6 successes + 4 failures = 60% < 70%
        for _ in range(6):
            sched.report_episode(True)
        for _ in range(4):
            sched.report_episode(False)
        assert sched.level == 0

    def test_level_regresses_on_low_success_rate(self):
        cfg = CurriculumConfig(window_size=5, success_rate_threshold=0.7, failure_rate_threshold=0.3)
        sched = CurriculumScheduler(cfg)
        # Advance to level 1
        for _ in range(5):
            sched.report_episode(True)
        assert sched.level == 1
        # Now fail a lot
        for _ in range(5):
            sched.report_episode(False)
        assert sched.level == 0

    def test_level_does_not_go_below_zero(self):
        cfg = CurriculumConfig(window_size=5, failure_rate_threshold=0.3)
        sched = CurriculumScheduler(cfg)
        assert sched.level == 0
        for _ in range(5):
            sched.report_episode(False)
        assert sched.level == 0

    def test_level_does_not_exceed_max(self):
        cfg = CurriculumConfig(
            window_size=3,
            success_rate_threshold=0.7,
            initial_goal_distance=10.0,
            max_goal_distance=20.0,
            goal_distance_step=5.0,
            initial_obstacle_count=0,
            max_obstacle_count=10,
            obstacle_count_step=1,
            initial_max_steps=100,
            max_max_steps=600,
            steps_increment=50,
        )
        sched = CurriculumScheduler(cfg)
        # Try to advance many times
        for _ in range(100):
            for _ in range(3):
                sched.report_episode(True)
        assert sched.level <= sched.max_level

    def test_get_env_overrides_returns_valid_dict(self):
        sched = CurriculumScheduler()
        overrides = sched.get_env_overrides()
        assert "goal_distance" in overrides
        assert "num_obstacles" in overrides
        assert "max_steps" in overrides
        assert isinstance(overrides["goal_distance"], float)
        assert isinstance(overrides["num_obstacles"], int)
        assert isinstance(overrides["max_steps"], int)

    def test_goal_distance_increases_with_level(self):
        cfg = CurriculumConfig(window_size=3, success_rate_threshold=0.7)
        sched = CurriculumScheduler(cfg)
        dist_0 = sched.get_goal_distance()
        for _ in range(3):
            sched.report_episode(True)
        dist_1 = sched.get_goal_distance()
        assert dist_1 > dist_0

    def test_obstacle_count_increases_with_level(self):
        cfg = CurriculumConfig(window_size=3, success_rate_threshold=0.7)
        sched = CurriculumScheduler(cfg)
        obs_0 = sched.get_obstacle_count()
        for _ in range(3):
            sched.report_episode(True)
        obs_1 = sched.get_obstacle_count()
        assert obs_1 > obs_0

    def test_state_dict_round_trip(self):
        cfg = CurriculumConfig(window_size=5, success_rate_threshold=0.7)
        sched = CurriculumScheduler(cfg)
        for _ in range(5):
            sched.report_episode(True)
        original_level = sched.level

        state = sched.state_dict()
        sched2 = CurriculumScheduler(cfg)
        sched2.load_state_dict(state)
        assert sched2.level == original_level


# ======================================================================
# TestCurriculumWithMockEnv
# ======================================================================

class TestCurriculumWithMockEnv:
    """Integration tests: curriculum + MockNavigationEnv."""

    def test_update_difficulty_changes_goal(self):
        env = MockNavigationEnv()
        original_goal = env._goal.copy()
        env.update_difficulty(goal_distance=20.0, num_obstacles=1, max_steps=200, seed=42)
        new_goal = env._goal
        assert not np.allclose(original_goal, new_goal)
        actual_dist = float(np.linalg.norm(new_goal - env._start_position))
        assert abs(actual_dist - 20.0) < 0.1

    def test_update_difficulty_changes_obstacles(self):
        env = MockNavigationEnv()
        env.update_difficulty(goal_distance=30.0, num_obstacles=4, max_steps=300, seed=42)
        assert len(env._obstacles) == 4

    def test_update_difficulty_zero_obstacles(self):
        env = MockNavigationEnv()
        env.update_difficulty(goal_distance=15.0, num_obstacles=0, max_steps=100, seed=42)
        assert len(env._obstacles) == 0

    def test_update_difficulty_updates_max_steps(self):
        env = MockNavigationEnv()
        env.update_difficulty(goal_distance=15.0, num_obstacles=0, max_steps=200, seed=42)
        assert env._max_steps == 200

    def test_baseline_training_not_broken(self):
        """Verify that importing train_baseline still works."""
        import importlib
        mod = importlib.import_module("scripts.train_baseline")
        assert hasattr(mod, "main")
