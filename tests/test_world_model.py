"""Tests for the Axis 1 — Latent World Model modules.

Covers encoder, latent dynamics, future predictor, uncertainty estimation,
and policy intent mapping.  All tests run without GPU, AirSim, or ROS2.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

from src.utils.data_types import LatentState, SensorObservation
from src.world_model.encoder import IdentityEncoder, MLPEncoder
from src.world_model.latent_dynamics import LinearDynamics, MLPDynamics
from src.world_model.policy_intent import PolicyIntentMapper
from src.world_model.predictor import FuturePredictor
from src.world_model.uncertainty import EnsembleUncertainty, ThresholdUncertainty


# ── Helpers ────────────────────────────────────────────────────────────────

def _make_obs(
    pose: tuple = (1.0, 2.0, -3.0),
    velocity: tuple = (0.5, 0.1, 0.0),
    timestamp: float = 0.0,
) -> SensorObservation:
    """Create a minimal SensorObservation for testing."""
    return SensorObservation(timestamp=timestamp, pose=pose, velocity=velocity)


def _make_latent(dim: int = 8, uncertainty: float = 0.0) -> LatentState:
    """Create a LatentState with a random vector of given dimension."""
    return LatentState(
        vector=np.random.randn(dim).astype(np.float32),
        uncertainty=uncertainty,
    )


# ── Encoder tests ─────────────────────────────────────────────────────────

class TestIdentityEncoder:
    def test_output_shape(self) -> None:
        enc = IdentityEncoder()
        obs = _make_obs()
        latent = enc.encode(obs)

        assert isinstance(latent, LatentState)
        vec = np.asarray(latent.vector)
        # pose(3) + velocity(3) = 6
        assert vec.shape == (6,)

    def test_preserves_values(self) -> None:
        obs = _make_obs(pose=(10.0, 20.0, 30.0), velocity=(1.0, 2.0, 3.0))
        latent = IdentityEncoder().encode(obs)
        np.testing.assert_allclose(
            latent.vector, [10.0, 20.0, 30.0, 1.0, 2.0, 3.0]
        )

    def test_timestamp_propagated(self) -> None:
        obs = _make_obs(timestamp=42.0)
        latent = IdentityEncoder().encode(obs)
        assert latent.timestamp == 42.0


class TestMLPEncoder:
    def test_output_shape(self) -> None:
        enc = MLPEncoder(input_dim=8, latent_dim=16)
        obs = _make_obs()
        latent = enc.encode(obs)

        assert isinstance(latent, LatentState)
        vec = np.asarray(latent.vector)
        assert vec.shape == (16,)

    def test_different_latent_dims(self) -> None:
        for dim in [4, 32, 64]:
            enc = MLPEncoder(input_dim=6, latent_dim=dim)
            latent = enc.encode(_make_obs())
            assert np.asarray(latent.vector).shape == (dim,)

    def test_forward_batch(self) -> None:
        enc = MLPEncoder(input_dim=8, latent_dim=32)
        x = torch.randn(5, 8)
        out = enc.forward(x)
        assert out.shape == (5, 32)


# ── Latent dynamics tests ─────────────────────────────────────────────────

class TestMLPDynamics:
    def test_preserves_dim(self) -> None:
        dyn = MLPDynamics(latent_dim=8, action_dim=3)
        latent = _make_latent(dim=8)
        action = np.array([1.0, 0.0, -0.5], dtype=np.float32)

        next_latent = dyn.predict(latent, action)
        assert np.asarray(next_latent.vector).shape == (8,)

    def test_forward_batch(self) -> None:
        dyn = MLPDynamics(latent_dim=8, action_dim=3)
        s = torch.randn(4, 8)
        a = torch.randn(4, 3)
        delta = dyn.forward(s, a)
        assert delta.shape == (4, 8)


class TestLinearDynamics:
    def test_simple_step(self) -> None:
        dyn = LinearDynamics(dt=1.0)
        latent = LatentState(
            vector=np.array([0.0, 0.0, 0.0], dtype=np.float32)
        )
        action = np.array([1.0, 2.0, 3.0], dtype=np.float32)

        result = dyn.predict(latent, action)
        np.testing.assert_allclose(result.vector, [1.0, 2.0, 3.0])

    def test_dt_scaling(self) -> None:
        dyn = LinearDynamics(dt=0.5)
        latent = LatentState(
            vector=np.array([10.0, 0.0], dtype=np.float32)
        )
        action = np.array([2.0, 4.0], dtype=np.float32)

        result = dyn.predict(latent, action)
        np.testing.assert_allclose(result.vector, [11.0, 2.0])

    def test_action_padding(self) -> None:
        """Action shorter than state vector should be zero-padded."""
        dyn = LinearDynamics(dt=1.0)
        latent = LatentState(
            vector=np.array([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
        )
        action = np.array([1.0, 1.0], dtype=np.float32)

        result = dyn.predict(latent, action)
        np.testing.assert_allclose(result.vector, [2.0, 3.0, 3.0, 4.0])


# ── Predictor tests ───────────────────────────────────────────────────────

class TestFuturePredictor:
    def test_trajectory_length(self) -> None:
        dyn = LinearDynamics(dt=0.4)
        pred = FuturePredictor(dyn, horizon=5)
        initial = _make_latent(dim=3)
        actions = np.random.randn(5, 3).astype(np.float32)

        traj = pred.predict_trajectory(initial, actions)
        assert len(traj) == 5

    def test_trajectory_respects_horizon(self) -> None:
        dyn = LinearDynamics(dt=0.1)
        pred = FuturePredictor(dyn, horizon=3)
        actions = np.random.randn(10, 3).astype(np.float32)

        traj = pred.predict_trajectory(_make_latent(dim=3), actions)
        assert len(traj) == 3

    def test_collision_probability_range(self) -> None:
        dyn = LinearDynamics(dt=0.4)
        pred = FuturePredictor(dyn, horizon=5)

        # Build trajectory with 8-dim latent (index 7 = obstacle dist)
        trajectory = []
        for dist in [10.0, 3.0, 1.0, 0.0, 5.0]:
            vec = np.zeros(8, dtype=np.float32)
            vec[7] = dist
            trajectory.append(LatentState(vector=vec))

        probs = pred.predict_collision_probability(trajectory, obstacle_threshold=4.0)

        assert len(probs) == 5
        for p in probs:
            assert 0.0 <= p <= 1.0

        # dist=10 -> prob=0, dist=0 -> prob=1
        assert probs[0] == 0.0
        assert probs[3] == 1.0

    def test_collision_probability_short_latent(self) -> None:
        """Latent vectors shorter than 8 should produce 0.0 probabilities."""
        traj = [LatentState(vector=np.zeros(3)) for _ in range(4)]
        pred = FuturePredictor(LinearDynamics(), horizon=10)
        probs = pred.predict_collision_probability(traj)
        assert all(p == 0.0 for p in probs)


# ── Uncertainty tests ─────────────────────────────────────────────────────

class TestThresholdUncertainty:
    def test_clamps_to_01(self) -> None:
        est = ThresholdUncertainty()

        assert est.estimate(LatentState(vector=np.zeros(1), uncertainty=-0.5)) == 0.0
        assert est.estimate(LatentState(vector=np.zeros(1), uncertainty=0.3)) == pytest.approx(0.3)
        assert est.estimate(LatentState(vector=np.zeros(1), uncertainty=1.5)) == 1.0

    def test_boundary_values(self) -> None:
        est = ThresholdUncertainty()
        assert est.estimate(LatentState(vector=np.zeros(1), uncertainty=0.0)) == 0.0
        assert est.estimate(LatentState(vector=np.zeros(1), uncertainty=1.0)) == 1.0


class TestEnsembleUncertainty:
    def test_default_no_models(self) -> None:
        est = EnsembleUncertainty()
        result = est.estimate(LatentState(vector=np.zeros(1)))
        assert result == 0.5

    def test_with_models_deferred_extension_sentinel(self) -> None:
        est = EnsembleUncertainty(models=["model_a", "model_b"])
        result = est.estimate(LatentState(vector=np.zeros(1)))
        assert result == 0.0  # deterministic sentinel until variance logic is added


# ── PolicyIntentMapper tests ──────────────────────────────────────────────

class TestPolicyIntentMapper:
    def test_produces_valid_intent(self) -> None:
        mapper = PolicyIntentMapper(goal=(60.0, 20.0, -8.0), max_velocity=4.0)
        latent = LatentState(vector=np.zeros(8), uncertainty=0.1)
        traj = [LatentState(vector=np.zeros(8)) for _ in range(5)]
        c_probs = [0.0, 0.1, 0.2, 0.05, 0.0]

        intent = mapper.map_to_intent(latent, traj, c_probs)

        assert intent.target_position == (60.0, 20.0, -8.0)
        assert intent.risk_score == pytest.approx(0.2)
        assert intent.desired_velocity == pytest.approx(4.0 * 0.8)
        assert intent.confidence == pytest.approx(0.9)
        assert intent.horizon == 5

    def test_zero_risk(self) -> None:
        mapper = PolicyIntentMapper(goal=(0.0, 0.0, 0.0), max_velocity=3.0)
        latent = LatentState(vector=np.zeros(4), uncertainty=0.0)
        intent = mapper.map_to_intent(latent, [], [])

        assert intent.risk_score == 0.0
        assert intent.desired_velocity == pytest.approx(3.0)
        assert intent.confidence == pytest.approx(1.0)

    def test_max_risk(self) -> None:
        mapper = PolicyIntentMapper(goal=(1.0, 1.0, 1.0), max_velocity=5.0)
        latent = LatentState(vector=np.zeros(4), uncertainty=0.0)
        traj = [LatentState(vector=np.zeros(4))]
        intent = mapper.map_to_intent(latent, traj, [1.0])

        assert intent.risk_score == pytest.approx(1.0)
        assert intent.desired_velocity == pytest.approx(0.0)
