"""Strict image layout and lossless reason classification (no hole filling)."""
from enum import IntEnum
import numpy as np


class Reason(IntEnum):
    VALID = 0
    INVALID = 1
    TOO_CLOSE = 2
    NO_RETURN = 3
    UNOBSERVED = 4
    STALE = 5
    CALIBRATION_UNAVAILABLE = 6
    TRANSFORM_UNAVAILABLE = 7


def decode(data, width, height, encoding, step, is_bigendian):
    if encoding != '32FC1':
        raise ValueError('unsupported_encoding')
    if any(type(v) is not int for v in (width, height, step, is_bigendian)):
        raise ValueError('malformed_layout')
    if not (0 < width <= 640 and 0 < height <= 480 and is_bigendian in (0, 1)):
        raise ValueError('malformed_layout')
    if step < width * 4 or step > width * 4 + 4096 or len(data) != height * step:
        raise ValueError('malformed_payload')
    return np.ndarray((height, width), dtype='>f4' if is_bigendian else '<f4',
                      buffer=data, strides=(step, 4)).copy()


def classify(depth, near, far):
    if not (np.isfinite(near) and np.isfinite(far) and 0 < near < far):
        raise ValueError('invalid_sensor_range')
    mask = np.full(depth.shape, Reason.INVALID, dtype=np.uint8)
    finite = np.isfinite(depth)
    mask[finite & (depth >= near) & (depth < far)] = Reason.VALID
    mask[(finite & (depth < near)) | np.isneginf(depth)] = Reason.TOO_CLOSE
    mask[(finite & (depth >= far)) | np.isposinf(depth)] = Reason.NO_RETURN
    return mask
