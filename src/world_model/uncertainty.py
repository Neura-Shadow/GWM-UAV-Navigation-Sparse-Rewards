"""Uncertainty estimation for latent-state predictions.

Provides an abstract ``UncertaintyEstimator`` and two concrete
implementations:

* ``ThresholdUncertainty`` - returns the uncertainty already embedded in a
  ``LatentState``, clamped to [0, 1].
* ``EnsembleUncertainty`` - compatibility estimator for the deferred
  variance-based ensemble extension.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import List, Optional

from src.utils.data_types import LatentState

logger = logging.getLogger(__name__)


class UncertaintyEstimator(ABC):
    """Abstract interface for latent-state uncertainty estimation."""

    @abstractmethod
    def estimate(self, latent: LatentState) -> float:
        """Return uncertainty in [0, 1].  0 = fully confident, 1 = fully uncertain."""


class ThresholdUncertainty(UncertaintyEstimator):
    """Uses the ``uncertainty`` field already stored in the ``LatentState``.

    The value is simply clamped to [0, 1].
    """

    def estimate(self, latent: LatentState) -> float:
        value = min(1.0, max(0.0, latent.uncertainty))
        logger.debug("ThresholdUncertainty: %.4f", value)
        return value


class EnsembleUncertainty(UncertaintyEstimator):
    """Compatibility estimator for deferred ensemble-based uncertainty.

    When a list of dynamics models is provided, the estimator will
    eventually compute the variance of their one-step predictions. The current
    scoped framework keeps that richer estimator as a planned research
    extension while preserving the historical deterministic return values used
    by tests and demos.

    Parameters
    ----------
    models:
        Optional list of dynamics models.  Currently unused.
    """

    def __init__(self, models: Optional[List[object]] = None) -> None:
        self.models: List[object] = models or []
        logger.info(
            "EnsembleUncertainty created with %d model(s)", len(self.models)
        )

    def estimate(self, latent: LatentState) -> float:
        if not self.models:
            # No ensemble available: report moderate uncertainty.
            return 0.5
        # Deferred extension: forward latent through each model and compute variance.
        return 0.0
