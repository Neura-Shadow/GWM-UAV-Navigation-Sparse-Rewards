"""Frozen, inference-only policy runtime (contract §4).

The checkpoint is verified (file SHA-256, architecture, shapes, schema hash, `trained` flag, finite
weights) before use. Weights are copied into read-only float64 arrays; there is no gradient,
optimizer or parameter write path and no state carried between calls, so identical inputs give
bit-identical scores. The canonical forward pass is NumPy float64 on CPU.
"""
import hashlib
import io
import json
import time
import zipfile

import numpy as np

from .contracts import ARCHITECTURE, FEATURES, LAYER_SHAPES, SLOTS

METADATA_ENTRY = "metadata.json"


class PolicyError(Exception):
    """A checkpoint or inference failure; `reason` is the logged classification."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def save_model(path, arrays, metadata):
    """Deterministic artifact: a zip of .npy entries with fixed timestamps plus a metadata entry.
    Returns the file SHA-256 (the content address)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_STORED) as archive:
        for name in sorted(LAYER_SHAPES):
            entry = io.BytesIO()
            np.lib.format.write_array(entry, np.ascontiguousarray(arrays[name], dtype=np.float64), allow_pickle=False)
            archive.writestr(zipfile.ZipInfo(name + ".npy", date_time=(1980, 1, 1, 0, 0, 0)), entry.getvalue())
        archive.writestr(zipfile.ZipInfo(METADATA_ENTRY, date_time=(1980, 1, 1, 0, 0, 0)),
                         json.dumps(metadata, sort_keys=True, separators=(",", ":"), allow_nan=False))
    data = buffer.getvalue()
    with open(path, "wb") as stream:
        stream.write(data)
    return sha256_bytes(data)


class FrozenPolicy:
    def __init__(self, arrays, metadata, sha256):
        self.metadata = dict(metadata)
        self.sha256 = sha256
        self._weights = {}
        for name, shape in LAYER_SHAPES.items():
            value = np.array(arrays[name], dtype=np.float64, copy=True)
            if value.shape != shape:
                raise PolicyError("weight_shape:" + name)
            if not np.isfinite(value).all():
                raise PolicyError("weight_not_finite:" + name)
            value.setflags(write=False)
            self._weights[name] = value

    @classmethod
    def load(cls, path, expected_sha256, schema_sha256, require_trained=True):
        try:
            with open(path, "rb") as stream:
                data = stream.read()
        except OSError as exc:
            raise PolicyError("checkpoint_missing") from exc
        sha = sha256_bytes(data)
        if expected_sha256 is None or sha != expected_sha256:
            raise PolicyError("checkpoint_sha256_mismatch")
        try:
            with zipfile.ZipFile(io.BytesIO(data)) as archive:
                names = set(archive.namelist())
                if names != {n + ".npy" for n in LAYER_SHAPES} | {METADATA_ENTRY}:
                    raise PolicyError("checkpoint_entries")
                metadata = json.loads(archive.read(METADATA_ENTRY))
                arrays = {n: np.lib.format.read_array(io.BytesIO(archive.read(n + ".npy")), allow_pickle=False)
                          for n in LAYER_SHAPES}
        except (zipfile.BadZipFile, ValueError, KeyError) as exc:
            raise PolicyError("checkpoint_unreadable") from exc
        if metadata.get("architecture") != ARCHITECTURE:
            raise PolicyError("architecture_mismatch")
        if metadata.get("schema_sha256") != schema_sha256:
            raise PolicyError("schema_mismatch")
        if require_trained and metadata.get("trained") is not True:
            raise PolicyError("untrained_model_refused")
        return cls(arrays, metadata, sha)

    def scores(self, features):
        """(26, 46) float64 features -> (26,) float64 scores; no state, no side effects."""
        x = np.asarray(features)
        if x.shape != (SLOTS, FEATURES) or x.dtype != np.float64:
            raise PolicyError("input_schema")
        if not np.isfinite(x).all():
            raise PolicyError("input_not_finite")
        w = self._weights
        h = np.maximum(x @ w["W1"] + w["b1"], 0.0)
        h = np.maximum(h @ w["W2"] + w["b2"], 0.0)
        return (h @ w["W3"] + w["b3"])[:, 0]

    def timed_scores(self, features):
        start = time.perf_counter()
        scores = self.scores(features)
        return scores, time.perf_counter() - start
