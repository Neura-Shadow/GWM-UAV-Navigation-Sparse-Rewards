"""Tests for Phase 4-A Generated World Model core."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

from src.generated_world_model import (  # noqa: E402
    ActionConditioner,
    AutoregressiveRollout,
    CandidateTrajectorySampler,
    GeneratedWorldModelDataset,
    GeneratedWorldModelPlanner,
    GWMConfig,
    ObservationBuffer,
    ObservationEncoder,
    TrajectoryCandidate,
    TrajectoryScorer,
    VideoDynamicsModel,
    build_baseline_components,
    create_synthetic_batch,
    generated_world_model_loss,
    load_npz_sequence,
    save_npz_sequence,
    train_synthetic_step,
)
from src.utils.data_types import SensorObservation  # noqa: E402


def _small_config() -> GWMConfig:
    return GWMConfig(
        image_height=16,
        image_width=16,
        context_length=3,
        horizon=2,
        latent_dim=12,
        visual_feature_dim=12,
        state_feature_dim=8,
        conditioning_dim=10,
        hidden_dim=16,
    )


def _sample() -> dict:
    config = _small_config()
    return create_synthetic_batch(
        batch_size=2,
        context_length=config.context_length,
        horizon=config.horizon,
        image_height=config.image_height,
        image_width=config.image_width,
        pose_dim=config.pose_dim,
        action_dim=config.action_dim,
        intention_dim=config.intention_dim,
        seed=11,
    )


def test_generated_world_model_package_imports() -> None:
    """The package exports the Phase 4-A public interfaces."""
    assert ObservationEncoder is not None
    assert ActionConditioner is not None
    assert VideoDynamicsModel is not None
    assert AutoregressiveRollout is not None
    assert TrajectoryScorer is not None


def test_observation_encoder_output_shape() -> None:
    config = _small_config()
    sample = _sample()
    encoder = ObservationEncoder(config)

    latent = encoder.encode(sample["context"])

    assert latent.shape == (2, config.context_length, config.latent_dim)


def test_action_conditioner_output_shape() -> None:
    config = _small_config()
    sample = _sample()
    conditioner = ActionConditioner(config)

    conditioning = conditioner.encode(sample["action_sequence"])

    assert conditioning.shape == (2, config.horizon, config.conditioning_dim)


def test_video_dynamics_forward_output_shape() -> None:
    config = _small_config()
    sample = _sample()
    encoder = ObservationEncoder(config)
    conditioner = ActionConditioner(config)
    model = VideoDynamicsModel(config)

    context = encoder.encode(sample["context"])
    conditioning = conditioner.encode(sample["action_sequence"])
    prediction = model(context, conditioning)

    assert prediction.predicted_rgb.shape == (2, config.horizon, 3, 16, 16)
    assert prediction.predicted_depth.shape == (2, config.horizon, 1, 16, 16)
    assert prediction.predicted_latent.shape == (2, config.horizon, config.latent_dim)
    assert prediction.uncertainty.shape == (2, config.horizon, 1)


def test_autoregressive_rollout_length() -> None:
    config = _small_config()
    sample = _sample()
    encoder, conditioner, model = build_baseline_components(config)
    rollout = AutoregressiveRollout(model, encoder, conditioner, mode="autoregressive")

    generated = rollout.rollout(
        sample["context"],
        sample["action_sequence"],
        horizon=config.horizon,
    )

    assert generated.horizon == config.horizon
    assert generated.predicted_rgb.shape[1] == config.horizon


def test_trajectory_scorer_returns_deterministic_score_dict() -> None:
    config = _small_config()
    sample = _sample()
    encoder, conditioner, model = build_baseline_components(config)
    generated = AutoregressiveRollout(model, encoder, conditioner).rollout(
        sample["context"],
        sample["action_sequence"],
        horizon=config.horizon,
    )
    candidate = TrajectoryCandidate(
        positions=np.array([[0.0, 0.0, -5.0], [1.0, 0.0, -5.0]], dtype=np.float32),
        actions=np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32),
    )
    scorer = TrajectoryScorer(weights={"goal_progress": 1.0})

    first = scorer.score(
        generated,
        candidate,
        goal=(4.0, 0.0, -5.0),
        safety_context={"min_safe_depth": 0.5, "altitude_bounds": (1.0, 20.0)},
    )
    second = scorer.score(
        generated,
        candidate,
        goal=(4.0, 0.0, -5.0),
        safety_context={"min_safe_depth": 0.5, "altitude_bounds": (1.0, 20.0)},
    )

    assert first == second
    assert "total_score" in first
    assert "goal_progress" in first["components"]


def test_synthetic_dataset_sample_format(tmp_path: Path) -> None:
    sample = _sample()
    dataset_root = tmp_path / "generated_world_model"
    sequence_dir = dataset_root / "sequences"
    sequence_dir.mkdir(parents=True)
    with open(dataset_root / "metadata.json", "w", encoding="utf-8") as handle:
        json.dump({"name": "synthetic_test"}, handle)

    shard_path = sequence_dir / "seq_000001.npz"
    save_npz_sequence(shard_path, sample)
    loaded = load_npz_sequence(shard_path)
    dataset = GeneratedWorldModelDataset(dataset_root)

    assert len(dataset) == 1
    assert loaded["context_rgb"].shape == sample["context"].rgb.shape
    assert dataset[0]["actions"].shape == sample["actions"].shape
    assert dataset.metadata["name"] == "synthetic_test"


def test_observation_buffer_converts_sensor_observations_to_context() -> None:
    buffer = ObservationBuffer(context_length=2, image_size=(16, 16))
    image = np.zeros((16, 16, 3), dtype=np.uint8)
    depth = np.ones((16, 16), dtype=np.float32)

    buffer.append(
        SensorObservation(
            pose=(0.0, 0.0, -5.0),
            velocity=(0.0, 0.0, 0.0),
            image=image,
            depth=depth,
        )
    )
    assert buffer.is_ready is False
    buffer.append(
        SensorObservation(
            pose=(1.0, 0.0, -5.0),
            velocity=(1.0, 0.0, 0.0),
            image=image,
            depth=depth,
        )
    )

    batch = buffer.as_observation_batch()

    assert buffer.is_ready is True
    assert batch.rgb.shape == (1, 2, 3, 16, 16)
    assert batch.depth.shape == (1, 2, 1, 16, 16)
    assert batch.pose.shape == (1, 2, 6)
    assert batch.velocity.shape == (1, 2, 3)


def test_generated_world_model_planner_scores_candidates_with_batched_context() -> None:
    config = _small_config()
    sample = _sample()
    encoder, conditioner, model = build_baseline_components(config)
    rollout = AutoregressiveRollout(model, encoder, conditioner)
    planner = GeneratedWorldModelPlanner(
        rollout=rollout,
        scorer=TrajectoryScorer(),
        sampler=CandidateTrajectorySampler(horizon=config.horizon, seed=5),
    )

    result = planner.plan(
        context=sample["context"],
        start=(0.0, 0.0, -5.0),
        goal=(5.0, 0.0, -5.0),
        safety_context={"min_safe_depth": 0.5, "altitude_bounds": (1.0, 20.0)},
        num_candidates=3,
    )

    assert isinstance(result["candidate"], TrajectoryCandidate)
    assert "total_score" in result["score"]
    assert len(result["all_scores"]) == 3


def test_training_step_backward_pass_works() -> None:
    config = _small_config()
    encoder, conditioner, model = build_baseline_components(config)
    optimizer = torch.optim.Adam(
        list(encoder.parameters()) + list(conditioner.parameters()) + list(model.parameters()),
        lr=1e-3,
    )
    sample = _sample()

    metrics = train_synthetic_step(encoder, conditioner, model, optimizer, sample)

    assert metrics["loss"] > 0.0
    assert any(param.grad is not None for param in model.parameters())


def test_generated_world_model_loss_backward() -> None:
    config = _small_config()
    sample = _sample()
    encoder, conditioner, model = build_baseline_components(config)
    prediction = model(
        encoder.encode(sample["context"]),
        conditioner.encode(sample["action_sequence"]),
    )

    loss, metrics = generated_world_model_loss(prediction, sample["future"])
    loss.backward()

    assert metrics["rgb_loss"] >= 0.0
    assert loss.item() > 0.0


def test_synthetic_training_script_runs_with_no_robotics_dependencies() -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "train_generated_world_model.py"),
            "--synthetic",
            "--steps",
            "5",
            "--batch-size",
            "2",
            "--image-height",
            "16",
            "--image-width",
            "16",
            "--latent-dim",
            "12",
            "--hidden-dim",
            "16",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "final_loss=" in result.stdout
    assert "rclpy" not in sys.modules
    assert "mavsdk" not in sys.modules
    assert "omni.isaac.core" not in sys.modules


def test_training_script_respects_config_dimensions(tmp_path: Path) -> None:
    config_path = tmp_path / "gwm.yaml"
    config_path.write_text(
        """
model:
  image_height: 16
  image_width: 16
  context_length: 2
  horizon: 1
  latent_dim: 8
  visual_feature_dim: 8
  state_feature_dim: 4
  conditioning_dim: 8
  hidden_dim: 12
""".strip(),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(_project_root / "scripts" / "train_generated_world_model.py"),
            "--synthetic",
            "--steps",
            "1",
            "--batch-size",
            "1",
            "--config",
            str(config_path),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )

    assert result.returncode == 0, result.stderr
    assert "image=16x16" in result.stderr
