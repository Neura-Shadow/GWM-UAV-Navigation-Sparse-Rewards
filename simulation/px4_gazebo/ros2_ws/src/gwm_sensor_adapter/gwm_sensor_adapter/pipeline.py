"""Deterministic newest-frame depth observation pipeline and health state.

Acquisition (native 30 Hz), processing (simulation-time phase accumulator at
the configured rate) and raw recording are separate accounting boundaries.
Processing may skip older frames on purpose; that never hides recorder loss,
which is accounted for independently by the recorder and the evaluator.
"""
import math

import numpy as np

from . import depth as depth_module
from .association import Associator
from .camera_info import CalibrationCache
from .contracts import CONTRACT, OBSERVATION_SCHEMA
from .geometry import Extrinsic, backproject, frd_to_local_ned

FRESH, STALE, UNAVAILABLE, CALIBRATION_UNAVAILABLE = "fresh", "stale", "unavailable", "calibration_unavailable"


class Accounting:
    def __init__(self):
        self.received = self.decoded = self.processed = self.overwritten = 0
        self.rejected = {}
        self.missed_periods = self.due_without_new_frame = 0
        self.max_receipt_gap_wall_s = self.max_source_gap_sim_s = 0.0
        self.max_source_to_receipt_sim_s = self.max_observation_age_sim_s = 0.0
        self.min_source_to_receipt_sim_s = None

    def reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0) + 1

    def as_dict(self):
        result = dict(vars(self))
        result["skipped_by_processing"] = self.decoded - self.processed
        return result


class DepthProcessor:
    def __init__(self, config, run_id, require_state, extrinsic=None):
        self.config = config
        self.run_id = run_id
        self.require_state = require_state
        self.calibration = CalibrationCache(config)
        self.associator = Associator(config["state_capacity"], config["state_mismatch_s"])
        self.extrinsic = extrinsic or Extrinsic(config["optical_origin_base_flu_m"], identity=config["extrinsic_id"])
        self.accounting = Accounting()
        self.period = 1.0 / config["processing_hz"]
        self.samples = depth_module.sample_pixels(config["width"], config["height"],
                                                  config["sample_grid"]["columns"], config["sample_grid"]["rows"])
        self.regions = {k: tuple(v) for k, v in config["report_regions_uv"].items()}
        self.slot = None
        self.last_stamp_ns = None
        self.last_receipt_wall = None
        self.last_receipt_id = 0
        self.last_processed_id = 0
        self.next_due_sim = None
        self.observation_id = 0
        self.status = UNAVAILABLE
        self.status_reason = "no_frame_received"
        self.transitions = []

    # Inputs -----------------------------------------------------------------
    def on_camera_info(self, fields, header=None):
        return self.calibration.update(fields, header)

    def on_image(self, fields, stamp_ns, frame_id, receipt_wall, receipt_sim):
        """Validate one native frame. Returns (accepted, receipt_record)."""
        a = self.accounting
        a.received += 1
        receipt_id = a.received
        record = {"receipt_id": receipt_id, "acquisition_stamp_ns": stamp_ns, "frame_id": frame_id,
                  "receipt_wall_s": receipt_wall, "receipt_sim_s": receipt_sim,
                  "width": fields.get("width"), "height": fields.get("height"), "encoding": fields.get("encoding"),
                  "step": fields.get("step"), "is_bigendian": fields.get("is_bigendian"), "accepted": False}
        try:
            if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
                raise depth_module.FrameRejected("missing_acquisition_stamp")
            if frame_id != self.config["raw_frame"]:
                raise depth_module.FrameRejected("unexpected_raw_frame_id")
            if self.last_stamp_ns is not None and stamp_ns == self.last_stamp_ns:
                raise depth_module.FrameRejected("duplicate_acquisition_stamp")
            if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
                raise depth_module.FrameRejected("acquisition_stamp_backwards")
            image = depth_module.decode(fields, self.config)
        except depth_module.FrameRejected as exc:
            a.reject(str(exc))
            record["rejection"] = str(exc)
            return False, record
        if self.last_stamp_ns is not None:
            a.max_source_gap_sim_s = max(a.max_source_gap_sim_s, (stamp_ns - self.last_stamp_ns) / 1e9)
        if self.last_receipt_wall is not None:
            a.max_receipt_gap_wall_s = max(a.max_receipt_gap_wall_s, receipt_wall - self.last_receipt_wall)
        if receipt_sim is not None:
            latency = receipt_sim - stamp_ns / 1e9
            a.max_source_to_receipt_sim_s = max(a.max_source_to_receipt_sim_s, latency)
            a.min_source_to_receipt_sim_s = latency if a.min_source_to_receipt_sim_s is None else min(
                a.min_source_to_receipt_sim_s, latency)
        self.last_stamp_ns, self.last_receipt_wall, self.last_receipt_id = stamp_ns, receipt_wall, receipt_id
        a.decoded += 1
        if self.slot is not None:
            a.overwritten += 1
        self.slot = {"receipt_id": receipt_id, "stamp_ns": stamp_ns, "image": image,
                     "receipt_wall": receipt_wall, "receipt_sim": receipt_sim}
        record["accepted"] = True
        return True, record

    def on_position(self, message):
        return self.associator.add_position(message)

    def on_attitude(self, message):
        return self.associator.add_attitude(message)

    # Health -----------------------------------------------------------------
    def evaluate_health(self, sim_now, wall_now):
        if self.last_receipt_wall is None:
            status, reason = UNAVAILABLE, "no_frame_received"
        elif not self.calibration.usable:
            status, reason = CALIBRATION_UNAVAILABLE, self.calibration.invalid_reason or "no_valid_camera_info"
        elif wall_now - self.last_receipt_wall > self.config["stall_wall_s"]:
            status, reason = STALE, "receipt_stall_wall"
        elif sim_now is not None and sim_now - self.last_stamp_ns / 1e9 > self.config["source_age_sim_s"]:
            status, reason = STALE, "source_age_sim"
        else:
            status, reason = FRESH, None
        changed = (status, reason) != (self.status, self.status_reason)
        if changed:
            self.transitions.append({"wall_s": wall_now, "sim_s": sim_now, "from": self.status,
                                     "to": status, "reason": reason, "last_receipt_id": self.last_receipt_id})
        self.status, self.status_reason = status, reason
        return changed

    def health(self, sim_now, wall_now):
        return {"schema": OBSERVATION_SCHEMA, "contract": CONTRACT, "kind": "health", "run_id": self.run_id,
                "status": self.status, "reason": self.status_reason, "sim_s": sim_now, "wall_s": wall_now,
                "last_receipt_id": self.last_receipt_id, "last_processed_receipt_id": self.last_processed_id,
                "last_acquisition_stamp_ns": self.last_stamp_ns,
                "receipt_age_wall_s": None if self.last_receipt_wall is None else wall_now - self.last_receipt_wall,
                "source_age_sim_s": None if self.last_stamp_ns is None or sim_now is None
                else sim_now - self.last_stamp_ns / 1e9,
                "calibration_identity": self.calibration.calibration["identity"] if self.calibration.calibration else None}

    # Processing -------------------------------------------------------------
    def poll(self, sim_now, wall_now):
        """Return (health_changed, observation_or_None). Never re-publishes a frame."""
        changed = self.evaluate_health(sim_now, wall_now)
        if sim_now is None:
            return changed, None
        if self.next_due_sim is None:
            self.next_due_sim = sim_now
        if sim_now < self.next_due_sim:
            return changed, None
        missed = int(math.floor((sim_now - self.next_due_sim) / self.period))
        self.accounting.missed_periods += missed
        self.next_due_sim += (missed + 1) * self.period
        frame, self.slot = self.slot, None
        if frame is None or self.status != FRESH:
            self.accounting.due_without_new_frame += 1
            return changed, None
        return changed, self.observe(frame, sim_now, wall_now)

    def observe(self, frame, sim_now, wall_now):
        config, calibration = self.config, self.calibration.calibration
        image = frame["image"]
        codes = depth_module.classify(image, config["near_m"], config["far_m"])
        stats = depth_module.statistics(image, codes, self.regions)
        age = sim_now - frame["stamp_ns"] / 1e9
        a = self.accounting
        a.processed += 1
        a.max_observation_age_sim_s = max(a.max_observation_age_sim_s, age)
        self.last_processed_id = frame["receipt_id"]
        self.observation_id += 1
        if self.require_state:
            association = self.associator.associate(int(frame["stamp_ns"] // 1000))
        else:
            association = {"status": "not_required_ground_fixture"}
        samples = []
        for u, v in self.samples:
            code = int(codes[v, u])
            row = {"u": u, "v": v, "reason": depth_module.REASONS[code], "depth_m": None, "optical_m": None,
                   "body_flu_m": None, "body_frd_m": None, "local_ned_m": None}
            if code == depth_module.VALID:
                z = float(image[v, u])
                optical = backproject(u, v, z, calibration)
                frd = self.extrinsic.optical_to_frd(optical)
                row.update(depth_m=z, optical_m=optical.tolist(),
                           body_flu_m=self.extrinsic.optical_to_flu(optical).tolist(), body_frd_m=frd.tolist())
                if association.get("status") == "matched":
                    row["local_ned_m"] = frd_to_local_ned(frd, association["position_ned"],
                                                          association["q_wxyz"]).tolist()
            samples.append(row)
        return {"schema": OBSERVATION_SCHEMA, "contract": CONTRACT, "kind": "observation", "run_id": self.run_id,
                "observation_id": self.observation_id, "receipt_id": frame["receipt_id"],
                "acquisition_stamp_ns": frame["stamp_ns"], "acquisition_sim_s": frame["stamp_ns"] / 1e9,
                "receipt_wall_s": frame["receipt_wall"], "receipt_sim_s": frame["receipt_sim"],
                "processing_wall_s": wall_now, "processing_sim_s": sim_now, "observation_age_sim_s": age,
                "receipt_to_processing_wall_s": wall_now - frame["receipt_wall"],
                "raw_frame": config["raw_frame"], "optical_frame": config["optical_frame"],
                "calibration_identity": calibration["identity"], "extrinsic_identity": self.extrinsic.identity,
                "extrinsic_translation_base_flu_m": self.extrinsic.t.tolist(),
                "depth_semantics": "optical_axis_Z_m", "statistics": stats, "samples": samples,
                "association": association, "health": self.status}


def summarize_costs(values):
    if not values:
        return {"count": 0, "max_s": None, "p50_s": None, "p99_s": None}
    array = np.asarray(values)
    return {"count": int(array.size), "max_s": float(array.max()), "p50_s": float(np.percentile(array, 50)),
            "p99_s": float(np.percentile(array, 99))}
