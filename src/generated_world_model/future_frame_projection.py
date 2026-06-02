"""Future Frame Projection geometry prior for Generated World Models.

This module implements a small ANWM-inspired projection prototype. It preserves
the project's current coordinate convention and does not apply Isaac Z-up or
other simulator-specific frame conversions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict

import numpy as np
import torch
import torch.nn.functional as F


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics for projection."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_image_size(
        cls,
        width: int,
        height: int,
        fov_degrees: float = 90.0,
    ) -> "CameraIntrinsics":
        """Create square-pixel intrinsics from image size and horizontal FOV."""
        if width <= 1 or height <= 1:
            raise ValueError("width and height must be greater than 1.")
        if fov_degrees <= 0.0 or fov_degrees >= 180.0:
            raise ValueError("fov_degrees must be in (0, 180).")
        focal = (float(width) * 0.5) / math.tan(math.radians(fov_degrees) * 0.5)
        return cls(
            fx=float(focal),
            fy=float(focal),
            cx=(float(width) - 1.0) * 0.5,
            cy=(float(height) - 1.0) * 0.5,
            width=int(width),
            height=int(height),
        )


@dataclass(frozen=True)
class ProjectionConfig:
    """Projection behavior and depth validity bounds."""

    min_depth: float = 0.1
    max_depth: float = 100.0
    align_corners: bool = True


@dataclass
class ProjectionResult:
    """Projected future-view tensors."""

    projected_rgb: torch.Tensor
    projected_depth: torch.Tensor
    valid_mask: torch.Tensor
    metadata: Dict[str, Any] = field(default_factory=dict)


class FutureFrameProjection:
    """Warp a past RGB/depth observation into a future camera viewpoint."""

    def __init__(self, config: ProjectionConfig | Dict[str, Any] | None = None) -> None:
        if config is None:
            self.config = ProjectionConfig()
        elif isinstance(config, ProjectionConfig):
            self.config = config
        else:
            self.config = ProjectionConfig(**config)
        if self.config.min_depth <= 0.0:
            raise ValueError("min_depth must be positive.")
        if self.config.max_depth <= self.config.min_depth:
            raise ValueError("max_depth must be greater than min_depth.")

    def project(
        self,
        past_frame: Any,
        past_depth: Any,
        past_pose: Any,
        future_pose: Any,
        camera_intrinsics: CameraIntrinsics | Dict[str, Any],
    ) -> ProjectionResult:
        """Project a past RGB/depth frame into the future camera pose."""
        frame = _as_tensor(past_frame).float()
        if frame.ndim != 4:
            raise ValueError("past_frame must have shape [B, C, H, W].")
        depth = _as_tensor(past_depth, device=frame.device).float()
        if depth.ndim != 4 or depth.shape[1] != 1:
            raise ValueError("past_depth must have shape [B, 1, H, W].")
        if depth.shape[0] != frame.shape[0] or depth.shape[-2:] != frame.shape[-2:]:
            raise ValueError("past_frame and past_depth must share batch and image size.")

        intrinsics = _coerce_intrinsics(camera_intrinsics)
        batch_size, _channels, height, width = frame.shape
        if intrinsics.width != width or intrinsics.height != height:
            raise ValueError("camera intrinsics width/height must match past_frame size.")

        past_transform = _pose_to_transform(past_pose, batch_size, frame.device, frame.dtype)
        future_transform = _pose_to_transform(future_pose, batch_size, frame.device, frame.dtype)

        pixel_grid = _pixel_grid(
            batch_size=batch_size,
            height=height,
            width=width,
            device=frame.device,
            dtype=frame.dtype,
        )
        depth_values = depth[:, 0]
        valid_depth = (
            (depth_values >= self.config.min_depth)
            & (depth_values <= self.config.max_depth)
            & torch.isfinite(depth_values)
        )
        safe_depth = torch.clamp(depth_values, min=self.config.min_depth)

        points_past = _back_project(pixel_grid, safe_depth, intrinsics)
        ones = torch.ones(
            batch_size,
            1,
            height * width,
            device=frame.device,
            dtype=frame.dtype,
        )
        points_past_h = torch.cat([points_past, ones], dim=1)
        points_world = torch.bmm(past_transform, points_past_h)
        points_future = torch.bmm(torch.linalg.inv(future_transform), points_world)[:, :3]

        z = points_future[:, 2].reshape(batch_size, height, width)
        projected_u = intrinsics.fx * (points_future[:, 0] / points_future[:, 2].clamp_min(1e-6))
        projected_v = intrinsics.fy * (points_future[:, 1] / points_future[:, 2].clamp_min(1e-6))
        projected_u = projected_u.reshape(batch_size, height, width) + intrinsics.cx
        projected_v = projected_v.reshape(batch_size, height, width) + intrinsics.cy

        valid_projection = (
            valid_depth
            & torch.isfinite(projected_u)
            & torch.isfinite(projected_v)
            & (z >= self.config.min_depth)
            & (z <= self.config.max_depth)
            & (projected_u >= 0.0)
            & (projected_u <= width - 1)
            & (projected_v >= 0.0)
            & (projected_v <= height - 1)
        )
        sample_grid = torch.stack(
            [
                _normalize_grid(projected_u, size=width, align_corners=self.config.align_corners),
                _normalize_grid(projected_v, size=height, align_corners=self.config.align_corners),
            ],
            dim=-1,
        )

        projected_rgb = F.grid_sample(
            frame,
            sample_grid,
            mode="bilinear",
            padding_mode="zeros",
            align_corners=self.config.align_corners,
        )
        projected_depth = F.grid_sample(
            depth,
            sample_grid,
            mode="bilinear",
            padding_mode="zeros",
            align_corners=self.config.align_corners,
        )
        valid_mask = valid_projection.unsqueeze(1).to(dtype=frame.dtype)
        valid_count = int(valid_projection.sum().item())
        total_count = int(valid_projection.numel())
        return ProjectionResult(
            projected_rgb=projected_rgb,
            projected_depth=projected_depth,
            valid_mask=valid_mask,
            metadata={
                "coverage_ratio": float(valid_count / total_count) if total_count else 0.0,
                "valid_pixel_count": valid_count,
                "total_pixel_count": total_count,
                "coordinate_conversion_applied": False,
                "coordinate_frame_note": "project_default; Isaac Z-up conversion remains future work",
            },
        )


def _coerce_intrinsics(value: CameraIntrinsics | Dict[str, Any]) -> CameraIntrinsics:
    if isinstance(value, CameraIntrinsics):
        return value
    return CameraIntrinsics(
        fx=float(value["fx"]),
        fy=float(value["fy"]),
        cx=float(value["cx"]),
        cy=float(value["cy"]),
        width=int(value["width"]),
        height=int(value["height"]),
    )


def _as_tensor(value: Any, device: torch.device | None = None) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        return value.to(device=device) if device is not None else value
    tensor = torch.from_numpy(np.asarray(value))
    return tensor.to(device=device) if device is not None else tensor


def _pixel_grid(
    *,
    batch_size: int,
    height: int,
    width: int,
    device: torch.device,
    dtype: torch.dtype,
) -> torch.Tensor:
    ys, xs = torch.meshgrid(
        torch.arange(height, device=device, dtype=dtype),
        torch.arange(width, device=device, dtype=dtype),
        indexing="ij",
    )
    grid = torch.stack([xs.reshape(-1), ys.reshape(-1)], dim=0)
    return grid.unsqueeze(0).expand(batch_size, -1, -1)


def _back_project(
    pixel_grid: torch.Tensor,
    depth: torch.Tensor,
    intrinsics: CameraIntrinsics,
) -> torch.Tensor:
    batch_size, height, width = depth.shape
    z = depth.reshape(batch_size, 1, height * width)
    u = pixel_grid[:, 0:1]
    v = pixel_grid[:, 1:2]
    x = (u - intrinsics.cx) / intrinsics.fx * z
    y = (v - intrinsics.cy) / intrinsics.fy * z
    return torch.cat([x, y, z], dim=1)


def _normalize_grid(value: torch.Tensor, *, size: int, align_corners: bool) -> torch.Tensor:
    if align_corners:
        return 2.0 * value / max(size - 1, 1) - 1.0
    return 2.0 * (value + 0.5) / size - 1.0


def _pose_to_transform(
    pose: Any,
    batch_size: int,
    device: torch.device,
    dtype: torch.dtype,
) -> torch.Tensor:
    tensor = _as_tensor(pose, device=device).to(dtype=dtype)
    if tensor.ndim == 2 and tensor.shape == (4, 4):
        tensor = tensor.unsqueeze(0)
    if tensor.ndim == 1 and tensor.shape[0] == 6:
        tensor = tensor.unsqueeze(0)
    if tensor.ndim == 3 and tensor.shape[1:] == (4, 4):
        if tensor.shape[0] == 1 and batch_size > 1:
            tensor = tensor.expand(batch_size, -1, -1)
        if tensor.shape[0] != batch_size:
            raise ValueError("pose batch size must match past_frame batch size.")
        return tensor
    if tensor.ndim == 2 and tensor.shape[1] == 6:
        if tensor.shape[0] == 1 and batch_size > 1:
            tensor = tensor.expand(batch_size, -1)
        if tensor.shape[0] != batch_size:
            raise ValueError("pose batch size must match past_frame batch size.")
        return _euler_pose_to_transform(tensor)
    raise ValueError("pose must have shape [B, 6] or [B, 4, 4].")


def _euler_pose_to_transform(pose: torch.Tensor) -> torch.Tensor:
    x, y, z, roll, pitch, yaw = [pose[:, index] for index in range(6)]
    cr, sr = torch.cos(roll), torch.sin(roll)
    cp, sp = torch.cos(pitch), torch.sin(pitch)
    cy, sy = torch.cos(yaw), torch.sin(yaw)

    row0 = torch.stack([cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], dim=-1)
    row1 = torch.stack([sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], dim=-1)
    row2 = torch.stack([-sp, cp * sr, cp * cr], dim=-1)
    rotation = torch.stack([row0, row1, row2], dim=1)

    transform = torch.eye(4, device=pose.device, dtype=pose.dtype).unsqueeze(0).repeat(
        pose.shape[0], 1, 1
    )
    transform[:, :3, :3] = rotation
    transform[:, :3, 3] = torch.stack([x, y, z], dim=-1)
    return transform
