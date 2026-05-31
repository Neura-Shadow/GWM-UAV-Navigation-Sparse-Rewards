"""Tests for Phase 2-A: World Model Training & Latent Dynamics.

Covers LatentWorldModel forward/backward, trainer checkpoint save/load,
training loss decrease, and the train_world_model.py script.
"""

from __future__ import annotations

import sys
import subprocess
import tempfile
from pathlib import Path

_project_root = Path(__file__).resolve().parents[1]
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

import numpy as np
import pytest
import torch
import torch.optim as optim

from src.rl.replay_buffer import ReplayBuffer
from src.rl.trainer import TrainerConfig, WorldModelTrainer
from src.rl.world_model_baseline import BaselineWorldModel
from src.world_model.latent_world_model import LatentWorldModel


# ======================================================================
# TestLatentWorldModel
# ======================================================================

class TestLatentWorldModel:
    """Unit tests for the LatentWorldModel nn.Module."""

    def test_forward_output_shape(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        state = torch.randn(4, 8)
        action = torch.randn(4, 3)
        delta = model(state, action)
        assert delta.shape == (4, 8)

    def test_forward_single_sample(self):
        model = LatentWorldModel()
        state = torch.randn(1, 8)
        action = torch.randn(1, 3)
        delta = model(state, action)
        assert delta.shape == (1, 8)

    def test_backward_computes_gradients(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        state = torch.randn(4, 8)
        action = torch.randn(4, 3)
        delta = model(state, action)
        loss = delta.pow(2).mean()
        loss.backward()

        # Encoder, dynamics, and decoder should all have gradients
        for name, param in model.named_parameters():
            assert param.grad is not None, f"No gradient for {name}"

    def test_encode_output_shape(self):
        model = LatentWorldModel(state_dim=8, latent_dim=16)
        state = torch.randn(4, 8)
        z = model.encode(state)
        assert z.shape == (4, 16)

    def test_forward_with_reconstruction(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        state = torch.randn(4, 8)
        action = torch.randn(4, 3)
        delta, recon = model.forward_with_reconstruction(state, action)
        assert delta.shape == (4, 8)
        assert recon.shape == (4, 8)

    def test_reconstruction_gradients(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        state = torch.randn(4, 8)
        action = torch.randn(4, 3)
        delta, recon = model.forward_with_reconstruction(state, action)
        loss = delta.pow(2).mean() + 0.1 * (recon - state).pow(2).mean()
        loss.backward()
        for name, param in model.named_parameters():
            assert param.grad is not None, f"No gradient for {name}"

    def test_default_dimensions(self):
        model = LatentWorldModel()
        assert model.state_dim == 8
        assert model.action_dim == 3
        assert model.latent_dim == 32


# ======================================================================
# TestTrainerCheckpoint
# ======================================================================

class TestTrainerCheckpoint:
    """Tests for checkpoint save/load functionality."""

    def _make_trainer(self):
        model = BaselineWorldModel(state_dim=8, action_dim=3)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=1000)
        device = torch.device("cpu")
        return WorldModelTrainer(model, opt, buf, device)

    def test_save_creates_file(self, tmp_path):
        trainer = self._make_trainer()
        path = str(tmp_path / "ckpt.pt")
        trainer.save_checkpoint(path)
        assert Path(path).exists()

    def test_load_restores_weights(self, tmp_path):
        # Save
        trainer1 = self._make_trainer()
        path = str(tmp_path / "ckpt.pt")
        trainer1.save_checkpoint(path)
        original_weights = {k: v.clone() for k, v in trainer1.model.state_dict().items()}

        # Load into a fresh trainer
        trainer2 = self._make_trainer()
        trainer2.load_checkpoint(path)
        for k, v in trainer2.model.state_dict().items():
            assert torch.equal(v, original_weights[k]), f"Weight mismatch: {k}"

    def test_load_restores_update_count(self, tmp_path):
        trainer = self._make_trainer()
        trainer._total_updates = 42
        path = str(tmp_path / "ckpt.pt")
        trainer.save_checkpoint(path)

        trainer2 = self._make_trainer()
        trainer2.load_checkpoint(path)
        assert trainer2.total_updates == 42


# ======================================================================
# TestTrainingLossDecreases
# ======================================================================

class TestTrainingLossDecreases:
    """Verify that training loss decreases on synthetic data."""

    def _fill_buffer(self, buf: ReplayBuffer, n: int = 200):
        """Create synthetic transitions with a learnable pattern."""
        for _ in range(n):
            state = np.random.randn(8).astype(np.float32)
            action = np.random.randn(3).astype(np.float32)
            # next_state has a simple linear relationship
            next_state = state + 0.1 * np.concatenate([action, np.zeros(5)])
            buf.add(state, action, next_state.astype(np.float32))

    def test_baseline_loss_decreases(self):
        model = BaselineWorldModel(state_dim=8, action_dim=3)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=5000)
        self._fill_buffer(buf, 300)

        device = torch.device("cpu")
        config = TrainerConfig(batch_size=64, train_epochs=1, min_buffer_size=64)
        trainer = WorldModelTrainer(model, opt, buf, device, config)

        losses = []
        for _ in range(50):
            loss = trainer.train_step()
            if loss is not None:
                losses.append(loss)

        assert len(losses) >= 10
        first_avg = np.mean(losses[:5])
        last_avg = np.mean(losses[-5:])
        assert last_avg < first_avg, f"Loss did not decrease: {first_avg:.6f} -> {last_avg:.6f}"

    def test_latent_loss_decreases(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=5000)
        self._fill_buffer(buf, 300)

        device = torch.device("cpu")
        config = TrainerConfig(batch_size=64, train_epochs=1, min_buffer_size=64)
        trainer = WorldModelTrainer(model, opt, buf, device, config)

        losses = []
        for _ in range(50):
            loss = trainer.train_step()
            if loss is not None:
                losses.append(loss)

        assert len(losses) >= 10
        first_avg = np.mean(losses[:5])
        last_avg = np.mean(losses[-5:])
        assert last_avg < first_avg, f"Loss did not decrease: {first_avg:.6f} -> {last_avg:.6f}"


# ======================================================================
# TestTrainerWithLatentModel
# ======================================================================

class TestTrainerWithLatentModel:
    """Verify WorldModelTrainer works correctly with LatentWorldModel."""

    def test_reconstruction_branch_active(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=1000)
        device = torch.device("cpu")
        trainer = WorldModelTrainer(model, opt, buf, device)
        assert trainer._has_reconstruction is True

    def test_baseline_no_reconstruction(self):
        model = BaselineWorldModel(state_dim=8, action_dim=3)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=1000)
        device = torch.device("cpu")
        trainer = WorldModelTrainer(model, opt, buf, device)
        assert trainer._has_reconstruction is False

    def test_latent_train_step_returns_loss(self):
        model = LatentWorldModel(state_dim=8, action_dim=3, latent_dim=16)
        opt = optim.Adam(model.parameters(), lr=1e-3)
        buf = ReplayBuffer(maxlen=1000)
        for _ in range(100):
            buf.add(
                np.random.randn(8).astype(np.float32),
                np.random.randn(3).astype(np.float32),
                np.random.randn(8).astype(np.float32),
            )
        device = torch.device("cpu")
        config = TrainerConfig(batch_size=32, train_epochs=1, min_buffer_size=32)
        trainer = WorldModelTrainer(model, opt, buf, device, config)
        loss = trainer.train_step()
        assert loss is not None
        assert loss > 0.0


# ======================================================================
# TestTrainWorldModelScript
# ======================================================================

class TestTrainWorldModelScript:
    """Integration test: run the training script as a subprocess."""

    def test_script_runs_successfully(self):
        result = subprocess.run(
            [
                sys.executable,
                str(_project_root / "scripts" / "train_world_model.py"),
                "--env", "mock",
                "--steps", "5",
                "--collect-steps", "50",
                "--batch-size", "32",
                "--checkpoint-dir", str(_project_root / "checkpoints" / "_test"),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert result.returncode == 0, f"Script failed:\n{result.stderr}"

    def test_script_latent_model(self):
        result = subprocess.run(
            [
                sys.executable,
                str(_project_root / "scripts" / "train_world_model.py"),
                "--env", "mock",
                "--model", "latent",
                "--steps", "5",
                "--collect-steps", "50",
                "--batch-size", "32",
                "--checkpoint-dir", str(_project_root / "checkpoints" / "_test_latent"),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert result.returncode == 0, f"Script failed:\n{result.stderr}"
