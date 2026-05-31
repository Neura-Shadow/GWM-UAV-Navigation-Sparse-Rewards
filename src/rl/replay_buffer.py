"""Experience replay buffer for world-model training.

Direct port of the original ``deque(maxlen=50000)``-based memory, extracted
into a standalone, testable class.
"""

from __future__ import annotations

import random
from collections import deque
from typing import Tuple

import numpy as np


class ReplayBuffer:
    """Fixed-capacity ring buffer storing ``(state, action, next_state)`` transitions.

    Parameters
    ----------
    maxlen:
        Maximum number of transitions to store.  When the buffer is full the
        oldest transition is dropped automatically.
    """

    def __init__(self, maxlen: int = 50_000) -> None:
        self._buffer: deque[Tuple[np.ndarray, np.ndarray, np.ndarray]] = deque(
            maxlen=maxlen
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add(
        self,
        state: np.ndarray,
        action: np.ndarray,
        next_state: np.ndarray,
    ) -> None:
        """Append a transition, copying the arrays to avoid aliasing."""
        self._buffer.append(
            (state.copy(), action.copy(), next_state.copy())
        )

    def sample(
        self, batch_size: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Uniformly sample a mini-batch of transitions.

        Returns
        -------
        states:
            ``(batch_size, state_dim)`` array.
        actions:
            ``(batch_size, action_dim)`` array.
        next_states:
            ``(batch_size, state_dim)`` array.

        Raises
        ------
        ValueError
            If the buffer contains fewer than *batch_size* transitions.
        """
        if len(self._buffer) < batch_size:
            raise ValueError(
                f"Not enough transitions in buffer ({len(self._buffer)}) "
                f"to sample a batch of size {batch_size}."
            )

        batch = random.sample(self._buffer, batch_size)
        states = np.stack([t[0] for t in batch])
        actions = np.stack([t[1] for t in batch])
        next_states = np.stack([t[2] for t in batch])
        return states, actions, next_states

    def __len__(self) -> int:
        return len(self._buffer)

    def __repr__(self) -> str:
        cap = self._buffer.maxlen
        return f"ReplayBuffer(len={len(self)}, maxlen={cap})"
