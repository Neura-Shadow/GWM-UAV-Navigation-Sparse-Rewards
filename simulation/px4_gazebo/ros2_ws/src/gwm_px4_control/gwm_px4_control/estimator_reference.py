"""Versioned, bounded yaw-initialization reference reconciliation; no ROS."""
import copy
import math

from .frames import finite, normalized_quaternion, wrap, yaw_from_quaternion

V1 = "p2-estimator-reference-v1"
V2 = "p2-estimator-reference-v2"
INITIALIZING = ("TAKEOFF", "STABILIZE_REFERENCE")
FIXED = ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter", "vz_reset_counter",
         "dist_bottom_reset_counter", "ref_timestamp", "ref_lat", "ref_lon", "ref_alt", "xy_global", "z_global")
SOURCE_FALSE = ("cs_ev_yaw", "cs_gnss_yaw", "cs_yaw_manual", "cs_mag_fault", "cs_mag_field_disturbed",
                "cs_gnss_fault", "cs_inertial_dead_reckoning")
FAULTS = ("fs_bad_mag_x", "fs_bad_mag_y", "fs_bad_mag_z", "fs_bad_hdg", "fs_bad_mag_decl",
          "fs_bad_airspeed", "fs_bad_sideslip", "fs_bad_optflow_x", "fs_bad_optflow_y",
          "fs_bad_acc_vertical", "fs_bad_acc_clipping")
FIXED_SOURCES = ("cs_gnss_pos", "cs_gnss_vel", "cs_baro_hgt", "cs_gps_hgt", "cs_ev_pos",
                 "cs_ev_vel", "cs_ev_hgt", "cs_rng_hgt", "cs_opt_flow")


def counter(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 255:
        raise ValueError("invalid_uint8_reset_counter")
    return value


class ReferenceManager:
    def __init__(self, config, data, origin, anchor):
        self.policy = config.get("reference_policy", V1)
        self.settings = config.get("reference_settings", {})
        self.origin = finite(origin, 3)
        self.initial_anchor = self.anchor = wrap(anchor)
        self.fixed = {k: data["vehicle_local_position"][k] for k in FIXED}
        self.baseline = {"heading": counter(data["vehicle_local_position"]["heading_reset_counter"]),
                         "quaternion": counter(data["vehicle_attitude"]["quat_reset_counter"])}
        self.initial_counters = dict(self.baseline)
        self.fixed_sources = {k: data["estimator_status_flags"][k] for k in FIXED_SOURCES}
        self.last_delta = {"heading": data["vehicle_local_position"]["delta_heading"],
                           "quaternion": list(data["vehicle_attitude"]["delta_q_reset"])}
        self.last_stamps = {}
        self.phase = "GROUND"
        self.state = "prepared"
        self.pending = None
        self.accepted = []
        self.events = []
        self.rejected = None
        self.stable_since = None
        self.lock_sim_s = None
        self.unaligned_seen = data["estimator_status_flags"].get("cs_mag_aligned_in_flight") is False
        self.alignment_rise = None
        self.previous_alignment = data["estimator_status_flags"].get("cs_mag_aligned_in_flight")
        self.aligned_at_preparation = (self.previous_alignment is True
                                       and data["vehicle_local_position"].get("heading_good_for_control") is True
                                       and data["estimator_status_flags"].get("cs_mag_3d") is True)

    def reject(self, reason, sim):
        if self.rejected is None:
            self.rejected = reason
            self.state = "rejected"
            self.events.append({"event": "reference_rejected", "reason": reason, "sim_s": sim,
                                "phase": self.phase, "pending": copy.deepcopy(self.pending)})
        raise ValueError("estimator_reference_reset:"+self.rejected)

    def inspect(self, data, sample, sim, wall, phase):
        try:
            return self._inspect(data, sample, sim, wall, phase)
        except (ValueError, KeyError, TypeError) as exc:
            if self.rejected is None:
                self.reject("invalid_reference_evidence:"+str(exc), sim)
            raise

    def _inspect(self, data, sample, sim, wall, phase):
        self.phase = phase
        if self.rejected:
            self.reject(self.rejected, sim)
        p, a, e = (data[k] for k in ("vehicle_local_position", "vehicle_attitude", "estimator_status_flags"))
        for key, value in (("position", p), ("attitude", a), ("flags", e)):
            stamp = value["timestamp"]
            if stamp < self.last_stamps.get(key, stamp):
                self.reject("reference_timestamp_backwards", sim)
            self.last_stamps[key] = stamp
        if {k: p.get(k) for k in FIXED} != self.fixed:
            self.reject("position_velocity_origin_or_terrain_change", sim)
        if {k: e.get(k) for k in FIXED_SOURCES} != self.fixed_sources:
            self.reject("estimator_aiding_source_change", sim)
        if (any(e.get(k) is not False for k in SOURCE_FALSE+FAULTS)
                or e.get("cs_tilt_align") is not True or e.get("cs_yaw_align") is not True
                or e.get("cs_gnss_pos") is not True or e.get("cs_mag") is not True):
            self.reject("unsupported_estimator_source_or_fault", sim)
        aligned = e.get("cs_mag_aligned_in_flight")
        if aligned is False:
            self.unaligned_seen = True
        if aligned is True and self.previous_alignment is False:
            self.alignment_rise = e["timestamp"]/1e6
        self.previous_alignment = aligned
        current = {"heading": counter(p["heading_reset_counter"]), "quaternion": counter(a["quat_reset_counter"])}
        changed = {k: (current[k]-self.baseline[k]) % 256 for k in current}
        deltas = {"heading": p["delta_heading"], "quaternion": list(a["delta_q_reset"])}
        if any(changed[k] == 0 and deltas[k] != self.last_delta[k] for k in changed):
            self.reject("delta_changed_without_counter", sim)
        if any(changed.values()):
            if self.policy != V2:
                self.reject("strict_v1", sim)
            if phase not in INITIALIZING or self.lock_sim_s is not None:
                self.reject("reset_outside_initialization", sim)
            if len(self.accepted) >= self.settings["max_events"]:
                self.reject("additional_initialization_reset", sim)
            if any(n not in (0, 1) for n in changed.values()):
                self.reject("skipped_reset_counter", sim)
            if sample["arming_state"] != 2 or sample["nav_state"] != 14 or sample["landed"] is not False:
                self.reject("reset_without_owned_airborne_offboard", sim)
            if self.pending is None:
                self.pending = {"start_sim_s": sim, "start_wall_s": wall, "old_counters": dict(self.baseline),
                                "heading": None, "quaternion": None, "anchor_before": self.anchor}
                self.state = "pending"
                self.stable_since = None
                self.events.append({"event": "reference_pending", "sim_s": sim, "phase": phase})
            for key, message, fields in (("heading", p, ("heading", "delta_heading")),
                                         ("quaternion", a, ("q", "delta_q_reset"))):
                if changed[key] == 1:
                    if self.pending[key] is None:
                        self.pending[key] = {"counter": current[key], "timestamp": message["timestamp"],
                                             **{k: copy.deepcopy(message[k]) for k in fields}}
                    elif self.pending[key]["counter"] != current[key]:
                        self.reject("multiple_pending_events", sim)
                    else:
                        field = "delta_heading" if key == "heading" else "delta_q_reset"
                        previous = self.pending[key][field]
                        latest = message[field]
                        delta_changed = list(previous) != list(latest) if key == "quaternion" else previous != latest
                        if delta_changed:
                            self.reject("inconsistent_duplicate_delta", sim)
        if self.pending:
            pending = self.pending
            if sim-pending["start_sim_s"] > self.settings["pair_sim_s"] or wall-pending["start_wall_s"] > self.settings["pair_wall_s"]:
                self.reject("reset_pair_or_alignment_timeout", sim)
            h, q = pending["heading"], pending["quaternion"]
            if any(pending[k] is not None and current[k] != pending[k]["counter"] for k in current):
                self.reject("pending_counter_changed", sim)
            if h is not None and q is not None:
                delta = finite([h["delta_heading"]], 1)[0]
                dq = normalized_quaternion(q["delta_q_reset"])
                finite([h["heading"]], 1)
                normalized_quaternion(q["q"])
                if abs(h["timestamp"]-q["timestamp"])/1e6 > self.settings["pair_skew_s"]:
                    self.reject("reset_timestamp_pair_skew", sim)
                if max(abs(dq[1]), abs(dq[2])) > self.settings["tilt_component_tolerance"]:
                    self.reject("non_yaw_quaternion_reset", sim)
                if abs(wrap(yaw_from_quaternion(dq)-delta)) > self.settings["yaw_consistency_rad"]:
                    self.reject("inconsistent_reset_deltas", sim)
                if abs(delta) > math.radians(self.settings["max_yaw_deg"]):
                    self.reject("excessive_yaw_reset", sim)
                last_event_s = max(h["timestamp"], q["timestamp"])/1e6
                context = (self.unaligned_seen and self.alignment_rise is not None
                           and abs(self.alignment_rise-last_event_s) <= self.settings["pair_sim_s"]
                           and e.get("cs_in_air") is True and e.get("cs_vehicle_at_rest") is False
                           and aligned is True and (e.get("cs_mag_hdg") is True or e.get("cs_mag_3d") is True))
                if context and p["timestamp"]/1e6 >= last_event_s+self.settings["post_event_sim_s"]:
                    self.anchor = wrap(self.anchor+delta)
                    record = {**copy.deepcopy(pending), "new_counters": dict(current), "delta_heading": delta,
                              "anchor_after": self.anchor, "accepted_sim_s": sim, "phase": phase,
                              "alignment_rise_sim_s": self.alignment_rise,
                              "classification": "allowlisted_yaw_initialization_pattern"}
                    self.accepted.append(record)
                    self.baseline = dict(current)  # only after the complete allowlist succeeds
                    self.last_delta = copy.deepcopy(deltas)
                    self.pending = None
                    self.state = "accepted"
                    self.events.append({"event": "reference_accepted", **record})
                    return {"pending": False, "correction": delta, "accepted": True}
            return {"pending": True, "correction": 0.0}
        final = (p.get("heading_good_for_control") is True and aligned is True
                 and e.get("cs_mag_3d") is True and e.get("cs_in_air") is True)
        if phase in INITIALIZING + ("LOCK_REFERENCE",) and final:
            if self.stable_since is None:
                self.stable_since = sim
        elif self.lock_sim_s is None:
            self.stable_since = None
        elif phase not in ("REQUEST_LAND", "VERIFY_LANDING", "VERIFY_LANDED_AND_DISARMED", "COMPLETE") and not final:
            self.reject("final_alignment_lost", sim)
        return {"pending": False, "correction": 0.0}

    def ready(self, sim):
        return (self.pending is None and self.rejected is None and self.stable_since is not None
                and (self.aligned_at_preparation or bool(self.accepted))
                and sim-self.stable_since >= self.settings["stable_sim_s"])

    def lock(self, sim):
        if not self.ready(sim):
            self.reject("missing_stable_final_alignment", sim)
        self.lock_sim_s = sim
        self.state = "locked"
        self.events.append({"event": "reference_locked", "sim_s": sim, "anchor": self.anchor,
                            "ground_origin": self.origin, "stable_since_sim_s": self.stable_since})

    def summary(self):
        return {"policy": self.policy, "state": self.state, "phase": self.phase, "ground_origin": self.origin,
                "initial_anchor": self.initial_anchor, "anchor": self.anchor, "fixed_reference": self.fixed,
                "fixed_aiding_sources": self.fixed_sources,
                "aligned_at_preparation": self.aligned_at_preparation,
                "initial_counters": self.initial_counters, "accepted": self.accepted,
                "accepted_count": len(self.accepted), "rejected_count": int(self.rejected is not None),
                "rejection": self.rejected, "lock_sim_s": self.lock_sim_s, "stable_since_sim_s": self.stable_since}
