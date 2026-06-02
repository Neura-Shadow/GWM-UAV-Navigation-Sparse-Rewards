"""Tests for Phase 3-B mock-first Isaac Sim / OpenUSD descriptors."""

from __future__ import annotations

import json

import pytest

import src.digital_twin.isaac_sim_builder as isaac_builder_module
from src.digital_twin import IsaacSimSceneBuilder
from src.digital_twin.mock_isaac_sim import MockUSDStage
from src.digital_twin.sim_scene_builder import SimSceneBuilder
from src.utils.data_types import ScenarioSpec


def _make_scenario() -> ScenarioSpec:
    return ScenarioSpec(
        scenario_id="isaac_scene_001",
        description="Phase 3-B descriptor test",
        start_position=(0.0, 0.0, -5.0),
        goal_position=(50.0, 5.0, -8.0),
        obstacles=[
            {"type": "sphere", "position": [10.0, 1.0, -5.0], "radius": 2.0},
            {"type": "cube", "position": [25.0, -2.0, -6.0], "size": [2.0, 3.0, 4.0]},
        ],
        weather={"wind_speed": 1.5},
        physics={"friction": 0.9},
        sensor_noise={"lidar": 0.02},
    )


def test_build_returns_openusd_descriptor_with_coordinate_metadata() -> None:
    builder = IsaacSimSceneBuilder()
    descriptor = builder.build(_make_scenario())

    assert descriptor["schema_version"] == "gwm_openusd_descriptor_v1"
    assert descriptor["stage"]["root_path"] == "/World"
    assert descriptor["metadata"]["source_coordinate_frame"] == "project_default"
    assert descriptor["metadata"]["target_coordinate_frame"] == "isaac_z_up_pending"
    assert descriptor["metadata"]["coordinate_conversion_applied"] is False


def test_descriptor_contains_expected_prim_paths() -> None:
    descriptor = IsaacSimSceneBuilder().build(_make_scenario())
    paths = {prim["path"] for prim in descriptor["prims"]}

    assert "/World" in paths
    assert "/World/GroundPlane" in paths
    assert "/World/Obstacles/Obstacle_0" in paths
    assert "/World/Obstacles/Obstacle_1" in paths
    assert "/World/Vehicle/UAV" in paths
    assert "/World/Vehicle/UAV/Sensors/Lidar" in paths
    assert "/World/Vehicle/UAV/Sensors/DepthCamera" in paths
    assert "/World/Goal" in paths


def test_descriptor_preserves_project_coordinates() -> None:
    spec = _make_scenario()
    descriptor = IsaacSimSceneBuilder().build(spec)

    assert descriptor["vehicle"]["position"] == list(spec.start_position)
    assert descriptor["goal"]["position"] == list(spec.goal_position)
    assert descriptor["obstacles"][0]["attributes"]["position"] == spec.obstacles[0]["position"]


def test_mock_usd_stage_records_prims_and_exports_json(tmp_path) -> None:
    stage = MockUSDStage()
    stage.DefinePrim("/World", "Xform")
    obstacle = stage.DefinePrim("/World/Obstacles/Obstacle_0", "Sphere")
    obstacle.set_attribute("position", [1.0, 2.0, 3.0])

    assert stage.GetPrimAtPath("/World") is not None
    assert stage.GetPrimAtPath("/Missing") is None

    output_path = tmp_path / "mock_stage.json"
    stage.Export(str(output_path))
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["prims"][1]["path"] == "/World/Obstacles/Obstacle_0"
    assert payload["prims"][1]["attributes"]["position"] == [1.0, 2.0, 3.0]


def test_builder_populates_mock_stage_from_descriptor() -> None:
    builder = IsaacSimSceneBuilder()
    descriptor = builder.build(_make_scenario())
    stage = builder.populate_mock_stage(MockUSDStage(), descriptor)

    assert stage.GetPrimAtPath("/World/Vehicle/UAV") is not None
    assert stage.GetPrimAtPath("/World/Goal") is not None
    assert len(stage.define_log) == len(descriptor["prims"])


def test_export_json_writes_descriptor(tmp_path) -> None:
    builder = IsaacSimSceneBuilder()
    descriptor = builder.build(_make_scenario())
    output_path = tmp_path / "scene_descriptor.json"

    builder.export_json(descriptor, str(output_path))
    loaded = json.loads(output_path.read_text(encoding="utf-8"))

    assert loaded["scenario_id"] == "isaac_scene_001"
    assert loaded["metadata"]["coordinate_conversion_applied"] is False


def test_build_usd_stage_raises_without_isaac_or_openusd(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(isaac_builder_module, "_HAS_ISAAC_SIM", False)
    monkeypatch.setattr(isaac_builder_module, "_HAS_OPENUSD", False)

    with pytest.raises(RuntimeError, match="Isaac Sim / OpenUSD"):
        IsaacSimSceneBuilder().build_usd_stage(_make_scenario(), str(tmp_path / "scene.usd"))


def test_sim_scene_builder_factory_keeps_mock_default() -> None:
    builder = SimSceneBuilder.create()

    assert isinstance(builder, SimSceneBuilder)
    assert builder.backend == "mock"


def test_sim_scene_builder_factory_returns_isaac_builder() -> None:
    builder = SimSceneBuilder.create(backend="isaac_sim")

    assert isinstance(builder, IsaacSimSceneBuilder)


def test_isaac_builder_normalizes_existing_flat_sensor_config() -> None:
    builder = SimSceneBuilder.create(
        backend="isaac_sim",
        sensor_config={
            "lidar_channels": 32,
            "lidar_range": 75.0,
            "camera_fov": 100,
            "depth_enabled": True,
        },
    )
    descriptor = builder.build(_make_scenario())
    sensors = {sensor["path"]: sensor for sensor in descriptor["vehicle"]["sensors"]}

    assert "/World/Vehicle/UAV/Sensors/Lidar" in sensors
    assert "/World/Vehicle/UAV/Sensors/DepthCamera" in sensors
    assert "/World/Vehicle/UAV/Sensors/LidarChannels" not in sensors
    assert sensors["/World/Vehicle/UAV/Sensors/Lidar"]["attributes"]["config"]["channels"] == 32
    assert sensors["/World/Vehicle/UAV/Sensors/Lidar"]["attributes"]["config"]["range"] == 75.0


def test_sim_scene_builder_factory_rejects_unknown_backend() -> None:
    with pytest.raises(ValueError, match="Unsupported scene builder backend"):
        SimSceneBuilder.create(backend="unknown")
