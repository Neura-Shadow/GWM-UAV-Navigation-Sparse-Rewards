"""Tests for Phase 4-B Future Frame Projection geometry prior."""

from __future__ import annotations

import sys

import numpy as np
import pytest
import torch

from src.generated_world_model import (
    CameraIntrinsics,
    FutureFrameProjection,
    ProjectionConfig,
    ProjectionResult,
    future_frame_projection_loss,
)


def _frame(width: int = 8, height: int = 6) -> torch.Tensor:
    values = torch.linspace(0.0, 1.0, steps=width * height).reshape(1, 1, height, width)
    return values.repeat(1, 3, 1, 1)


def _depth(width: int = 8, height: int = 6, value: float = 2.0) -> torch.Tensor:
    return torch.full((1, 1, height, width), value, dtype=torch.float32)


def _intrinsics(width: int = 8, height: int = 6) -> CameraIntrinsics:
    return CameraIntrinsics.from_image_size(width=width, height=height, fov_degrees=90.0)


def test_package_import_exports() -> None:
    """Phase 4-B projection symbols are exported at package level."""
    assert CameraIntrinsics is not None
    assert FutureFrameProjection is not None
    assert ProjectionConfig is not None
    assert ProjectionResult is not None


def test_camera_intrinsics_from_image_size() -> None:
    intrinsics = CameraIntrinsics.from_image_size(width=32, height=16, fov_degrees=90.0)

    assert intrinsics.width == 32
    assert intrinsics.height == 16
    assert intrinsics.fx == pytest.approx(16.0)
    assert intrinsics.fy == pytest.approx(16.0)
    assert intrinsics.cx == pytest.approx(15.5)
    assert intrinsics.cy == pytest.approx(7.5)


def test_identity_pose_projection_preserves_shape() -> None:
    projector = FutureFrameProjection()
    frame = _frame()
    depth = _depth()
    pose = torch.zeros(1, 6)

    result = projector.project(frame, depth, pose, pose, _intrinsics())

    assert result.projected_rgb.shape == frame.shape
    assert result.projected_depth.shape == depth.shape
    assert result.valid_mask.shape == depth.shape
    assert result.metadata["valid_pixel_count"] == depth.numel()


def test_identity_pose_projection_mostly_preserves_pixel_values() -> None:
    projector = FutureFrameProjection()
    frame = _frame()
    depth = _depth()
    pose = torch.zeros(1, 6)

    result = projector.project(frame, depth, pose, pose, _intrinsics())

    assert torch.allclose(result.projected_rgb, frame, atol=1e-5)
    assert torch.allclose(result.projected_depth, depth, atol=1e-5)


def test_small_translation_produces_nonempty_valid_mask() -> None:
    projector = FutureFrameProjection()
    frame = _frame()
    depth = _depth()
    past_pose = torch.zeros(1, 6)
    future_pose = torch.tensor([[0.05, 0.0, 0.0, 0.0, 0.0, 0.0]])

    result = projector.project(frame, depth, past_pose, future_pose, _intrinsics())

    assert int(result.valid_mask.sum().item()) > 0
    assert 0.0 < result.metadata["coverage_ratio"] <= 1.0


def test_numpy_inputs_are_accepted_and_return_torch_tensors() -> None:
    projector = FutureFrameProjection()
    frame = _frame().numpy()
    depth = _depth().numpy()
    pose = np.zeros((1, 6), dtype=np.float32)

    result = projector.project(frame, depth, pose, pose, _intrinsics())

    assert isinstance(result.projected_rgb, torch.Tensor)
    assert isinstance(result.projected_depth, torch.Tensor)
    assert isinstance(result.valid_mask, torch.Tensor)


def test_invalid_shapes_raise_clear_value_error() -> None:
    projector = FutureFrameProjection()
    frame = torch.zeros(1, 3, 8, 8)
    depth = torch.zeros(1, 8, 8)
    pose = torch.zeros(1, 6)

    with pytest.raises(ValueError, match="past_depth"):
        projector.project(frame, depth, pose, pose, _intrinsics(width=8, height=8))


def test_projection_loss_masks_invalid_pixels() -> None:
    predicted = torch.ones(1, 3, 2, 2)
    projected = torch.zeros(1, 3, 2, 2)
    valid_mask = torch.tensor([[[[1.0, 0.0], [0.0, 0.0]]]])

    loss = future_frame_projection_loss(predicted, projected, valid_mask)

    assert loss.item() == pytest.approx(1.0)


def test_projection_loss_handles_empty_valid_mask_safely() -> None:
    predicted = torch.ones(1, 3, 2, 2, requires_grad=True)
    projected = torch.zeros(1, 3, 2, 2)
    valid_mask = torch.zeros(1, 1, 2, 2)

    loss = future_frame_projection_loss(predicted, projected, valid_mask)
    loss.backward()

    assert loss.item() == 0.0
    assert predicted.grad is not None


def test_no_optional_robotics_dependency_is_imported() -> None:
    """Projection core must stay independent of real robotics runtimes."""
    assert "rclpy" not in sys.modules
    assert "mavsdk" not in sys.modules
    assert "omni.isaac.core" not in sys.modules
    assert "pxr" not in sys.modules
