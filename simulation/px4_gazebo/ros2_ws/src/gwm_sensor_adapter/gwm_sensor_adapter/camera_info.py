"""CameraInfo validation, static calibration identity and cache invalidation."""
import hashlib
import json
import math


class CalibrationRejected(ValueError):
    pass


def expected_focal_px(config):
    """fx from the pinned SDF horizontal FOV, with its declared-precision bound."""
    half = config["horizontal_fov_rad"] / 2
    resolution = config["horizontal_fov_declared_resolution_rad"] / 2
    focal = (config["width"] / 2) / math.tan(half)
    bound = max(abs((config["width"] / 2) / math.tan(half - resolution) - focal),
                abs((config["width"] / 2) / math.tan(half + resolution) - focal))
    return focal, bound


def _numbers(values, count, name):
    values = list(values)
    if len(values) != count or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                   or not math.isfinite(v) for v in values):
        raise CalibrationRejected("invalid_" + name)
    return [float(v) for v in values]


def validate(fields, config):
    if (fields.get("width"), fields.get("height")) != (config["width"], config["height"]):
        raise CalibrationRejected("dimension_mismatch")
    k = _numbers(fields.get("k", ()), 9, "k")
    p = _numbers(fields.get("p", ()), 12, "p")
    r = _numbers(fields.get("r", ()), 9, "r")
    d = _numbers(fields.get("d", ()), len(list(fields.get("d", ()))), "d")
    model = fields.get("distortion_model")
    if model not in ("plumb_bob", "") or any(v != 0 for v in d):
        raise CalibrationRejected("distortion_unsupported")
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if not (fx > 0 and fy > 0) or [k[1], k[3], k[6], k[7], k[8]] != [0, 0, 0, 0, 1]:
        raise CalibrationRejected("invalid_intrinsic_matrix")
    if p[:3] != k[:3] or p[4:7] != k[3:6] or p[8:11] != k[6:9] or p[3] != 0 or p[7] != 0 or p[11] != 0:
        raise CalibrationRejected("projection_not_rectified_monocular")
    if r != [1, 0, 0, 0, 1, 0, 0, 0, 1]:
        raise CalibrationRejected("rectification_not_identity")
    if not (0 <= cx <= config["width"] and 0 <= cy <= config["height"]):
        raise CalibrationRejected("principal_point_outside_image")
    focal, bound = expected_focal_px(config)
    if abs(fx - focal) > bound or abs(fy - fx) > bound:
        raise CalibrationRejected("focal_length_inconsistent_with_pinned_fov")
    if fields.get("binning_x", 0) not in (0, 1) or fields.get("binning_y", 0) not in (0, 1):
        raise CalibrationRejected("binning_unsupported")
    roi = fields.get("roi", {})
    if any(roi.get(key, 0) not in (0, False) for key in ("x_offset", "y_offset", "height", "width", "do_rectify")):
        raise CalibrationRejected("roi_unsupported")
    static = {"width": config["width"], "height": config["height"], "distortion_model": model,
              "d": d, "k": k, "p": p, "r": r}
    identity = hashlib.sha256(json.dumps(static, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return {"identity": identity, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "static": static,
            "sdf_focal_px": focal, "sdf_focal_bound_px": bound}


class CalibrationCache:
    """The first valid calibration locks the run; any later change invalidates it."""

    def __init__(self, config):
        self.config = config
        self.calibration = None
        self.invalid_reason = None
        self.received = 0
        self.rejected = {}
        self.first_header = None

    def update(self, fields, header=None):
        self.received += 1
        try:
            calibration = validate(fields, self.config)
        except CalibrationRejected as exc:
            self.rejected[str(exc)] = self.rejected.get(str(exc), 0) + 1
            if self.calibration is not None:
                self.invalid_reason = "calibration_rejected_after_lock:" + str(exc)
            return False
        if self.calibration is None:
            self.calibration = calibration
            self.first_header = header
        elif calibration["identity"] != self.calibration["identity"]:
            self.invalid_reason = "calibration_identity_changed"
            return False
        return True

    @property
    def usable(self):
        return self.calibration is not None and self.invalid_reason is None
