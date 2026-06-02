"""Observation history buffer for generated-world-model context windows."""

from __future__ import annotations

from collections import deque
from typing import Deque, Iterable, Optional

import numpy as np
import torch

from src.generated_world_model.types import ObservationBatch
from src.utils.data_types import SensorObservation


class ObservationBuffer:
    """Fixed-length buffer that converts recent observations into model context."""

    def __init__(self, context_length: int = 4, image_size: tuple[int, int] = (32, 32)) -> None:
        if context_length <= 0:
            raise ValueError("context_length must be positive.")
        self.context_length = int(context_length)
        self.image_size = image_size
        self._items: Deque[SensorObservation] = deque(maxlen=context_length)

    def append(self, observation: SensorObservation) -> None:
        """Append one sensor observation."""
        self._items.append(observation)

    def extend(self, observations: Iterable[SensorObservation]) -> None:
        """Append multiple observations."""
        for observation in observations:
            self.append(observation)

    @property
    def is_ready(self) -> bool:
        """Return True when the buffer has enough context."""
        return len(self._items) == self.context_length

    def clear(self) -> None:
        """Clear buffered observations."""
        self._items.clear()

    def as_observation_batch(self) -> ObservationBatch:
        """Return a single-batch context window shaped for the encoder."""
        if not self.is_ready:
            raise RuntimeError("ObservationBuffer is not ready.")
        height, width = self.image_size
        rgb_frames = []
        depth_frames = []
        poses = []
        velocities = []

        for obs in self._items:
            rgb_frames.append(_image_tensor(obs.image, channels=3, height=height, width=width))
            depth_frames.append(_image_tensor(obs.depth, channels=1, height=height, width=width))
            pose = list(obs.pose) + [0.0, 0.0, 0.0]
            poses.append(pose[:6])
            velocities.append(list(obs.velocity))

        return ObservationBatch(
            rgb=torch.stack(rgb_frames, dim=0).unsqueeze(0),
            depth=torch.stack(depth_frames, dim=0).unsqueeze(0),
            pose=torch.tensor(poses, dtype=torch.float32).unsqueeze(0),
            velocity=torch.tensor(velocities, dtype=torch.float32).unsqueeze(0),
            metadata={"source": "observation_buffer"},
        )


def _image_tensor(
    value: Optional[np.ndarray],
    *,
    channels: int,
    height: int,
    width: int,
) -> torch.Tensor:
    if value is None:
        return torch.zeros(channels, height, width, dtype=torch.float32)
    array = np.asarray(value, dtype=np.float32)
    if channels == 3:
        if array.ndim == 3 and array.shape[-1] == 3:
            array = np.transpose(array, (2, 0, 1))
        elif array.ndim == 3 and array.shape[0] == 3:
            pass
        else:
            raise ValueError("RGB image must have shape [H, W, 3] or [3, H, W].")
    else:
        if array.ndim == 2:
            array = array[None, :, :]
        elif array.ndim == 3 and array.shape[0] == 1:
            pass
        else:
            raise ValueError("Depth image must have shape [H, W] or [1, H, W].")
    tensor = torch.from_numpy(array).float()
    if tensor.max() > 1.0 and channels == 3:
        tensor = tensor / 255.0
    if tensor.shape[-2:] != (height, width):
        tensor = torch.nn.functional.interpolate(
            tensor.unsqueeze(0),
            size=(height, width),
            mode="bilinear",
            align_corners=False,
        ).squeeze(0)
    return tensor
