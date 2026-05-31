"""Tests for the Digital Twin (Axis 3) subsystem — WP6.

Covers:
- ScenarioExtractor: near-collision detection, goal failure, empty trajectory
- SimSceneBuilder: valid scene output, YAML serialisation
- DomainRandomizer: variation count, obstacle perturbation, seed determinism
- Sim2RealManager: register / retrieve policy versions
"""

from __future__ import annotations

import json
import os
import tempfile
from typing import Any, Dict, List

import numpy as np
import pytest

from src.digital_twin.domain_randomization import DomainRandomizer
from src.digital_twin.scenario_extractor import ScenarioExtractor
from src.digital_twin.sim2real import Sim2RealManager
from src.digital_twin.sim_scene_builder import SimSceneBuilder
from src.utils.data_types import ScenarioSpec


# =========================================================================
# Fixtures / helpers
# =========================================================================

def _make_trajectory(
    n: int = 20,
    obstacle_dist: float = 10.0,
    uncertainty: float = 0.3,
    reached_goal: bool = True,
) -> List[Dict[str, Any]]:
    """Create a simple straight-line trajectory for testing."""
    traj: List[Dict[str, Any]] = []
    for i in range(n):
        traj.append({
            "timestamp": float(i),
            "pose": [float(i), 0.0, -5.0],
            "velocity": [1.0, 0.0, 0.0],
            "obstacle_dist": obstacle_dist,
            "uncertainty": uncertainty,
        })
    # Mark the last step with reached_goal
    traj[-1]["reached_goal"] = reached_goal
    return traj


def _make_base_scenario() -> ScenarioSpec:
    return ScenarioSpec(
        scenario_id="base_001",
        description="Unit test base scenario",
        start_position=(0.0, 0.0, -5.0),
        goal_position=(50.0, 0.0, -5.0),
        obstacles=[
            {"position": [10.0, 2.0, -5.0], "size": 1.0, "type": "sphere"},
            {"position": [25.0, -1.0, -5.0], "size": 2.0, "type": "cube"},
        ],
        weather={"wind_speed": 1.0},
        physics={"friction": 1.0},
        sensor_noise={"lidar": 0.01},
    )


# =========================================================================
# ScenarioExtractor
# =========================================================================

class TestScenarioExtractor:
    """Tests for ScenarioExtractor."""

    def test_scenario_extractor_finds_near_collision(self) -> None:
        """Near-collision window (obstacle_dist < threshold) → scenario."""
        traj = _make_trajectory(n=20, obstacle_dist=10.0)
        # Inject a near-collision window at steps 5-12
        for i in range(5, 13):
            traj[i]["obstacle_dist"] = 1.5

        extractor = ScenarioExtractor(
            near_collision_threshold=3.0,
            min_scenario_duration=5,
        )
        scenarios = extractor.extract_from_trajectory(traj)

        near_collision = [s for s in scenarios if "near_collision" in s.scenario_id]
        assert len(near_collision) >= 1, "Should detect at least one near-collision scenario"
        assert near_collision[0].metadata["tag"] == "near_collision"

    def test_scenario_extractor_finds_goal_failure(self) -> None:
        """Trajectory ending with reached_goal=False → goal_failure scenario."""
        traj = _make_trajectory(n=15, reached_goal=False)
        extractor = ScenarioExtractor(min_scenario_duration=1)
        scenarios = extractor.extract_from_trajectory(traj)

        goal_failures = [s for s in scenarios if "goal_failure" in s.scenario_id]
        assert len(goal_failures) == 1, "Should detect exactly one goal-failure scenario"

    def test_scenario_extractor_empty_trajectory(self) -> None:
        """Empty trajectory → no scenarios, no crash."""
        extractor = ScenarioExtractor()
        scenarios = extractor.extract_from_trajectory([])
        assert scenarios == []

    def test_scenario_extractor_high_uncertainty(self) -> None:
        """High-uncertainty window → scenario."""
        traj = _make_trajectory(n=20, uncertainty=0.3)
        for i in range(3, 10):
            traj[i]["uncertainty"] = 0.9

        extractor = ScenarioExtractor(
            uncertainty_threshold=0.7,
            min_scenario_duration=5,
        )
        scenarios = extractor.extract_from_trajectory(traj)
        uncertain = [s for s in scenarios if "high_uncertainty" in s.scenario_id]
        assert len(uncertain) >= 1

    def test_scenario_extractor_sharp_manoeuvre(self) -> None:
        """Large velocity change → sharp_manoeuvre scenario."""
        traj = _make_trajectory(n=20)
        # Inject velocity jumps at steps 8-13
        for i in range(8, 14):
            traj[i]["velocity"] = [10.0, 5.0, 0.0]

        extractor = ScenarioExtractor(
            velocity_change_threshold=3.0,
            min_scenario_duration=1,
        )
        scenarios = extractor.extract_from_trajectory(traj)
        sharp = [s for s in scenarios if "sharp_manoeuvre" in s.scenario_id]
        assert len(sharp) >= 1

    def test_scenario_extractor_from_log_file(self) -> None:
        """Load trajectory from a JSON file."""
        traj = _make_trajectory(n=10, obstacle_dist=1.0)
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False, encoding="utf-8"
        ) as fh:
            json.dump({"trajectory": traj}, fh)
            tmp_path = fh.name

        try:
            extractor = ScenarioExtractor(
                near_collision_threshold=3.0, min_scenario_duration=5,
            )
            scenarios = extractor.extract_from_log_file(tmp_path)
            assert len(scenarios) >= 1
        finally:
            os.unlink(tmp_path)


# =========================================================================
# SimSceneBuilder
# =========================================================================

class TestSimSceneBuilder:
    """Tests for SimSceneBuilder."""

    def test_sim_scene_builder_produces_valid_scene(self) -> None:
        """build() returns a dict with all expected top-level keys."""
        builder = SimSceneBuilder()
        spec = _make_base_scenario()
        scene = builder.build(spec)

        required_keys = {
            "scenario_id", "description", "world_bounds",
            "spawn_point", "goal", "obstacles", "environment", "sensors",
        }
        assert required_keys.issubset(scene.keys())
        assert scene["spawn_point"]["x"] == spec.start_position[0]
        assert scene["goal"]["z"] == spec.goal_position[2]
        assert len(scene["obstacles"]) == len(spec.obstacles)

    def test_sim_scene_builder_yaml_output(self) -> None:
        """to_yaml() writes a valid YAML file that can be parsed back."""
        pytest.importorskip("yaml")
        import yaml

        builder = SimSceneBuilder()
        scene = builder.build(_make_base_scenario())

        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as fh:
            tmp_path = fh.name

        try:
            builder.to_yaml(scene, tmp_path)
            with open(tmp_path, "r", encoding="utf-8") as fh:
                loaded = yaml.safe_load(fh)
            assert loaded["scenario_id"] == scene["scenario_id"]
        finally:
            os.unlink(tmp_path)

    def test_sim_scene_builder_json_output(self) -> None:
        """to_json() writes valid JSON."""
        builder = SimSceneBuilder()
        scene = builder.build(_make_base_scenario())

        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", delete=False, encoding="utf-8"
        ) as fh:
            tmp_path = fh.name

        try:
            builder.to_json(scene, tmp_path)
            with open(tmp_path, "r", encoding="utf-8") as fh:
                loaded = json.load(fh)
            assert loaded["scenario_id"] == scene["scenario_id"]
        finally:
            os.unlink(tmp_path)


# =========================================================================
# DomainRandomizer
# =========================================================================

class TestDomainRandomizer:
    """Tests for DomainRandomizer."""

    def test_domain_randomizer_produces_correct_count(self) -> None:
        """randomize() produces exactly the requested number of variations."""
        randomizer = DomainRandomizer(seed=42)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=7)
        assert len(variations) == 7

    def test_domain_randomizer_varies_obstacles(self) -> None:
        """Each variation should have different obstacle positions."""
        randomizer = DomainRandomizer(seed=42)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=3)

        # Obstacle lists should differ between variations
        obs_lists = [str(v.obstacles) for v in variations]
        assert len(set(obs_lists)) > 1, "Variations should differ"

    def test_domain_randomizer_deterministic_with_seed(self) -> None:
        """Same seed → identical variations."""
        base = _make_base_scenario()
        r1 = DomainRandomizer(seed=123)
        r2 = DomainRandomizer(seed=123)
        v1 = r1.randomize(base, num_variations=3)
        v2 = r2.randomize(base, num_variations=3)

        for a, b in zip(v1, v2):
            assert a.weather == b.weather
            assert a.physics == b.physics
            assert a.sensor_noise == b.sensor_noise
            # Obstacle positions should match (same seed → same RNG output)
            for oa, ob in zip(a.obstacles, b.obstacles):
                if "position" in oa and "position" in ob:
                    np.testing.assert_allclose(oa["position"], ob["position"])

    def test_domain_randomizer_unique_ids(self) -> None:
        """Each variation should have a unique scenario_id."""
        randomizer = DomainRandomizer(seed=99)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=5)
        ids = [v.scenario_id for v in variations]
        assert len(set(ids)) == 5


# =========================================================================
# Sim2RealManager
# =========================================================================

class TestSim2RealManager:
    """Tests for Sim2RealManager."""

    def test_sim2real_manager_register_and_retrieve(self) -> None:
        """Register a training run, then retrieve it as the latest policy."""
        mgr = Sim2RealManager()
        version = mgr.register_training_run(
            scenarios=["sc_001", "sc_002"],
            policy_path="/tmp/policy_v1.pt",
            metrics={"success_rate": 0.85, "avg_reward": 42.0},
        )
        assert version is not None

        latest = mgr.get_latest_policy()
        assert latest is not None
        assert latest["version"] == version
        assert latest["metrics"]["success_rate"] == 0.85

    def test_sim2real_gap_calculation(self) -> None:
        """Sim2Real gap = sim success_rate - real success_rate."""
        mgr = Sim2RealManager()
        v = mgr.register_training_run(
            scenarios=["sc_001"],
            policy_path="/tmp/policy.pt",
            metrics={"success_rate": 0.90},
        )
        mgr.log_deployment(v, "lab", {"success_rate": 0.70})

        gap = mgr.get_sim2real_gap()
        assert gap is not None
        assert abs(gap - 0.20) < 1e-6

    def test_sim2real_no_data(self) -> None:
        """Gap should be None when no data is available."""
        mgr = Sim2RealManager()
        assert mgr.get_sim2real_gap() is None

    def test_sim2real_multiple_versions(self) -> None:
        """Latest policy should be the most recently registered."""
        mgr = Sim2RealManager()
        mgr.register_training_run(["s1"], "/tmp/v1.pt", {"success_rate": 0.5})
        v2 = mgr.register_training_run(["s2"], "/tmp/v2.pt", {"success_rate": 0.9})

        latest = mgr.get_latest_policy()
        assert latest is not None
        assert latest["version"] == v2


# =========================================================================
# DomainRandomizer — Phase 2-C extensions
# =========================================================================

class TestDomainRandomizerPhase2C:
    """Tests for Phase 2-C extensions: lighting, from_config, serialisation."""

    def test_lighting_in_variations(self) -> None:
        """Each variation should have lighting metadata."""
        randomizer = DomainRandomizer(seed=42)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=3)
        for v in variations:
            assert "lighting" in v.metadata
            light = v.metadata["lighting"]
            assert "time_of_day" in light
            assert "intensity" in light
            assert "color_temperature" in light

    def test_lighting_within_ranges(self) -> None:
        """Lighting values should be within configured ranges."""
        randomizer = DomainRandomizer(
            seed=42,
            lighting_time_range=(8.0, 16.0),
            lighting_intensity_range=(0.8, 1.2),
            lighting_color_temp_range=(4000, 6000),
        )
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=20)
        for v in variations:
            light = v.metadata["lighting"]
            assert 8.0 <= light["time_of_day"] <= 16.0
            assert 0.8 <= light["intensity"] <= 1.2
            assert 4000 <= light["color_temperature"] <= 6000

    def test_from_config(self) -> None:
        """from_config() should create a DomainRandomizer with correct params."""
        config = {
            "domain_randomization": {
                "seed": 42,
                "obstacle_position_noise": 3.0,
                "wind_speed_range": [1.0, 10.0],
                "lighting": {
                    "time_of_day_range": [7.0, 17.0],
                    "intensity_range": [0.6, 1.4],
                    "color_temp_range": [3500, 6500],
                },
            }
        }
        dr = DomainRandomizer.from_config(config)
        assert dr.obstacle_position_noise == 3.0
        assert dr.wind_speed_range == (1.0, 10.0)
        assert dr.lighting_time_range == (7.0, 17.0)

    def test_to_dict(self) -> None:
        """to_dict produces a JSON-serialisable dict."""
        spec = _make_base_scenario()
        d = DomainRandomizer.to_dict(spec)
        assert d["scenario_id"] == spec.scenario_id
        assert isinstance(d["obstacles"], list)
        # Verify it's JSON-serialisable
        json.dumps(d)

    def test_to_yaml_string(self) -> None:
        """to_yaml_string produces valid YAML."""
        pytest.importorskip("yaml")
        import yaml

        randomizer = DomainRandomizer(seed=42)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=2)
        yaml_str = DomainRandomizer.to_yaml_string(variations)
        loaded = yaml.safe_load(yaml_str)
        assert "scenarios" in loaded
        assert len(loaded["scenarios"]) == 2

    def test_to_yaml_file(self) -> None:
        """to_yaml writes to file and can be read back."""
        pytest.importorskip("yaml")
        import yaml

        randomizer = DomainRandomizer(seed=42)
        base = _make_base_scenario()
        variations = randomizer.randomize(base, num_variations=2)

        with tempfile.NamedTemporaryFile(
            suffix=".yaml", delete=False, mode="w"
        ) as fh:
            tmp_path = fh.name

        try:
            DomainRandomizer.to_yaml(variations, tmp_path)
            with open(tmp_path, "r", encoding="utf-8") as fh:
                loaded = yaml.safe_load(fh)
            assert len(loaded["scenarios"]) == 2
        finally:
            os.unlink(tmp_path)

