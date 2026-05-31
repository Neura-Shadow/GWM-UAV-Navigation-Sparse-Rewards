"""Tests for Phase 2-D: Real2Sim2Real Pipeline Skeleton.

Covers mock episode extraction, large control correction detection,
pipeline end-to-end execution, and JSON report output.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

import numpy as np
import pytest

from src.digital_twin.scenario_extractor import ScenarioExtractor
from src.env.mock_env import MockNavigationEnv


# ======================================================================
# TestExtractFromMockEpisode
# ======================================================================

class TestExtractFromMockEpisode:
    """Tests for ScenarioExtractor.extract_from_mock_episode."""

    def test_produces_trajectory(self):
        env = MockNavigationEnv(max_steps=50)
        extractor = ScenarioExtractor(min_scenario_duration=1)
        trajectory, scenarios = extractor.extract_from_mock_episode(
            env=env, policy_fn=None, max_steps=50,
        )
        assert len(trajectory) > 0
        assert isinstance(trajectory, list)

    def test_trajectory_has_expected_keys(self):
        env = MockNavigationEnv(max_steps=20)
        extractor = ScenarioExtractor(min_scenario_duration=1)
        trajectory, _ = extractor.extract_from_mock_episode(
            env=env, policy_fn=None, max_steps=20,
        )
        step = trajectory[0]
        expected_keys = {"timestamp", "pose", "velocity", "obstacle_dist", "action", "reward"}
        assert expected_keys.issubset(step.keys())

    def test_with_custom_policy(self):
        env = MockNavigationEnv(max_steps=20)
        extractor = ScenarioExtractor(min_scenario_duration=1)

        def constant_policy(state):
            return np.array([1.0, 0.5, 0.0], dtype=np.float32)

        trajectory, _ = extractor.extract_from_mock_episode(
            env=env, policy_fn=constant_policy, max_steps=20,
        )
        assert len(trajectory) > 0
        # All actions should be the constant policy
        for step in trajectory:
            np.testing.assert_allclose(step["action"], [1.0, 0.5, 0.0], atol=1e-5)

    def test_respects_done_flag(self):
        """Episode should stop when env returns done=True."""
        env = MockNavigationEnv(max_steps=10)
        extractor = ScenarioExtractor(min_scenario_duration=1)
        trajectory, _ = extractor.extract_from_mock_episode(
            env=env, policy_fn=None, max_steps=1000,
        )
        # MockNavigationEnv has max_steps=10, so trajectory should be <= 10
        assert len(trajectory) <= 10


# ======================================================================
# TestLargeControlCorrection
# ======================================================================

class TestLargeControlCorrection:
    """Tests for large_control_correction detection."""

    def test_detects_large_correction(self):
        """Trajectory with a big action jump → scenario detected."""
        trajectory = []
        for i in range(20):
            action = [1.0, 0.0, 0.0]
            if i == 10:
                action = [10.0, 10.0, 5.0]  # sudden large change
            trajectory.append({
                "timestamp": float(i),
                "pose": [float(i), 0.0, -5.0],
                "velocity": [1.0, 0.0, 0.0],
                "obstacle_dist": 10.0,
                "uncertainty": 0.1,
                "action": action,
            })

        extractor = ScenarioExtractor(
            control_correction_threshold=5.0,
            min_scenario_duration=1,
        )
        scenarios = extractor.extract_from_trajectory(trajectory)
        ctrl_scenarios = [s for s in scenarios if "large_control_correction" in s.scenario_id]
        assert len(ctrl_scenarios) >= 1

    def test_no_detection_for_smooth_actions(self):
        """Smooth trajectory → no control correction scenarios."""
        trajectory = []
        for i in range(20):
            trajectory.append({
                "timestamp": float(i),
                "pose": [float(i), 0.0, -5.0],
                "velocity": [1.0, 0.0, 0.0],
                "obstacle_dist": 10.0,
                "uncertainty": 0.1,
                "action": [1.0, 0.0, 0.0],
            })

        extractor = ScenarioExtractor(
            control_correction_threshold=5.0,
            min_scenario_duration=1,
        )
        scenarios = extractor.extract_from_trajectory(trajectory)
        ctrl_scenarios = [s for s in scenarios if "large_control_correction" in s.scenario_id]
        assert len(ctrl_scenarios) == 0


# ======================================================================
# TestR2S2RPipeline
# ======================================================================

class TestR2S2RPipeline:
    """Integration tests for the Real2Sim2Real pipeline script."""

    def test_pipeline_runs_successfully(self, tmp_path):
        result = subprocess.run(
            [
                sys.executable,
                str(_project_root / "scripts" / "run_real2sim2real_loop.py"),
                "--mock",
                "--episode-steps", "30",
                "--variants", "2",
                "--output-dir", str(tmp_path / "r2s2r"),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert result.returncode == 0, f"Pipeline failed:\n{result.stderr}"

    def test_pipeline_produces_json_report(self, tmp_path):
        subprocess.run(
            [
                sys.executable,
                str(_project_root / "scripts" / "run_real2sim2real_loop.py"),
                "--mock",
                "--episode-steps", "30",
                "--variants", "2",
                "--output-dir", str(tmp_path / "r2s2r"),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        report_path = tmp_path / "r2s2r" / "r2s2r_report.json"
        assert report_path.exists(), "Report file not created"

        with open(report_path, "r", encoding="utf-8") as f:
            report = json.load(f)

        assert "trajectory_length" in report
        assert "scenarios_extracted" in report
        assert "training_results" in report
        assert report["trajectory_length"] > 0
