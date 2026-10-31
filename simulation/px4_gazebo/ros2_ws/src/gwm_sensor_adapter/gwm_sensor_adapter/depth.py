"""Bounded native depth decoding, per-pixel classification and statistics.

No value is filled, interpolated, clamped or replaced. Unavailable depth stays
unavailable; it never becomes a zero-distance or far-range measurement.
"""
import math

import numpy as np

VALID, INVALID_NAN, TOO_CLOSE, NO_RETURN = 0, 1, 2, 3
REASONS = {VALID: "valid", INVALID_NAN: "invalid_nan", TOO_CLOSE: "too_close", NO_RETURN: "no_return"}
MAX_PAYLOAD = 64 * 1024 * 1024


class FrameRejected(ValueError):
    """A received message is incompatible; the reason is a stable identifier."""


def _int(value, name, low=0, high=2**32 - 1):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise FrameRejected("invalid_" + name)
    return value


def decode(fields, config, strict_layout=True):
    """Return a contiguous little-endian float32 array of shape (height, width).

    `fields` holds width/height/encoding/is_bigendian/step/data. The pure decoder
    supports explicit endianness and row padding; the registered native runtime
    (`strict_layout`) additionally requires the exact observed unpadded layout.
    """
    width = _int(fields.get("width"), "width", 1, 16384)
    height = _int(fields.get("height"), "height", 1, 16384)
    if (width, height) != (config["width"], config["height"]):
        raise FrameRejected("dimension_mismatch")
    if fields.get("encoding") != config["encoding"]:
        raise FrameRejected("unsupported_encoding")
    bigendian = fields.get("is_bigendian")
    if bigendian not in (0, 1) or isinstance(bigendian, bool):
        raise FrameRejected("invalid_endianness")
    step = _int(fields.get("step"), "step", 1)
    if step < width * 4 or step % 4:
        raise FrameRejected("invalid_step")
    data = fields.get("data")
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise FrameRejected("invalid_payload_type")
    size = len(data)
    if size > MAX_PAYLOAD or size != step * height:
        raise FrameRejected("payload_length_mismatch")
    if strict_layout and (step != width * 4 or bigendian != 0):
        raise FrameRejected("unregistered_native_layout")
    rows = np.frombuffer(data, dtype=">f4" if bigendian else "<f4").reshape(height, step // 4)
    return np.ascontiguousarray(rows[:, :width], dtype="<f4")


def classify(depth, near, far):
    """NaN invalid; -Inf and finite < near too close; +Inf and finite >= far no return."""
    codes = np.full(depth.shape, VALID, dtype=np.uint8)
    codes[np.isnan(depth)] = INVALID_NAN
    codes[np.isneginf(depth) | (np.isfinite(depth) & (depth < near))] = TOO_CLOSE
    codes[np.isposinf(depth) | (np.isfinite(depth) & (depth >= far))] = NO_RETURN
    return codes


def _summary(values):
    if values.size == 0:
        return {"valid_pixels": 0, "median_m": None, "p05_m": None, "p95_m": None, "min_m": None, "max_m": None}
    p05, p50, p95 = np.percentile(values, [5, 50, 95])
    return {"valid_pixels": int(values.size), "median_m": float(p50), "p05_m": float(p05), "p95_m": float(p95),
            "min_m": float(values.min()), "max_m": float(values.max())}


def statistics(depth, codes, regions):
    total = int(codes.size)
    counts = {name: int(np.count_nonzero(codes == code)) for code, name in REASONS.items()}
    valid = depth[codes == VALID]
    result = {"pixels": total, "reason_counts": counts,
              "valid_fraction": counts["valid"] / total if total else 0.0, **_summary(valid), "regions": {}}
    for name, (u0, v0, u1, v1) in regions.items():
        region_depth, region_codes = depth[v0:v1, u0:u1], codes[v0:v1, u0:u1]
        part = region_depth[region_codes == VALID]
        result["regions"][name] = {"roi_uv": [u0, v0, u1, v1], "pixels": int(region_codes.size),
                                   "valid_fraction": float(part.size / region_codes.size), **_summary(part)}
    return result


def sample_pixels(width, height, columns, rows):
    """Fixed, declared interior grid; identical for every frame."""
    us = [int(round((i + 0.5) * width / columns)) for i in range(columns)]
    vs = [int(round((j + 0.5) * height / rows)) for j in range(rows)]
    return [(u, v) for v in vs for u in us]


def finite_or_none(value):
    value = float(value)
    return value if math.isfinite(value) else None
