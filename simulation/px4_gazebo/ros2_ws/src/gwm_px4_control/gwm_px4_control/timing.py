"""Advancing simulation stamps and independent monotonic receipt watchdogs."""
import math

from .frames import finite, yaw_from_quaternion

STATE_TOPICS = ("vehicle_local_position", "vehicle_status", "vehicle_attitude",
                "vehicle_land_detected", "estimator_status_flags", "failsafe_flags")


class StateCache:
    def __init__(self, config):
        self.config = config
        self.data = {}
        self.receipts = {}
        self.stats = {}
        self.sim = None
        self.clock_wall = None
        self.frozen_reference = None
        self.reference_manager = None

    def clock(self, sim, wall):
        finite([sim, wall], 2)
        if self.sim is not None and sim < self.sim:
            raise ValueError("clock_backwards")
        fresh = self.sim is None or sim > self.sim
        if fresh:
            self.clock_wall = wall
            self.sim = sim
        return fresh

    def update(self, name, data, wall):
        timestamp = data.get("timestamp")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("invalid_timestamp:" + name)
        old = self.data.get(name)
        if old and timestamp < old["timestamp"]:
            raise ValueError("state_time_backwards:" + name)
        stats = self.stats.setdefault(name, {"received": 0, "unique": 0, "first_us": timestamp,
                                             "last_us": timestamp, "max_gap_s": 0.0})
        stats["received"] += 1
        fresh = old is None or timestamp > old["timestamp"]
        if fresh:
            if old:
                stats["max_gap_s"] = max(stats["max_gap_s"], (timestamp-old["timestamp"])/1e6)
            stats["unique"] += 1
            stats["last_us"] = timestamp
            self.data[name] = data
            self.receipts[name] = wall
        return fresh

    def reference(self):
        p = self.data["vehicle_local_position"]
        a = self.data["vehicle_attitude"]
        return tuple(p[k] for k in ("xy_reset_counter", "z_reset_counter", "heading_reset_counter",
                                    "vxy_reset_counter", "vz_reset_counter", "ref_timestamp")) + (a["quat_reset_counter"],)

    def validate(self, sim, wall, terminal_landed=False):
        c = self.config
        if self.clock_wall is None or wall-self.clock_wall > c["clock_stall_wall_s"]:
            raise ValueError("clock_stalled")
        if any(name not in self.data for name in STATE_TOPICS):
            raise ValueError("missing_state")
        offsets = {}
        for name in STATE_TOPICS:
            data = self.data[name]
            age = sim-data["timestamp"]/1e6
            limit = c["position_fresh_sim_s"] if name == "vehicle_local_position" else (
                c["attitude_fresh_sim_s"] if name == "vehicle_attitude" else c["flags_fresh_sim_s"])
            if age < -c["clock_offset_tolerance_s"]:
                raise ValueError("clock_epoch_mismatch:" + name)
            if age > limit or wall-self.receipts[name] > c["state_fresh_wall_s"]:
                raise ValueError("stale_state:" + name)
            offsets[name] = age
        p, s = self.data["vehicle_local_position"], self.data["vehicle_status"]
        a, f = self.data["vehicle_attitude"], self.data["failsafe_flags"]
        e = self.data["estimator_status_flags"]
        # PX4 v1.17 isYawFinalAlignComplete() includes in-flight magnetic
        # alignment. Ground readiness instead uses tilt/yaw alignment below,
        # valid position/velocity and PX4's own pre-flight health decision.
        if any(p.get(k) is not True for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")):
            raise ValueError("invalid_estimate")
        if e.get("cs_tilt_align") is not True or e.get("cs_yaw_align") is not True:
            raise ValueError("estimator_alignment_invalid")
        if any(f.get(k) is not False for k in ("local_position_invalid", "local_altitude_invalid",
                                               "attitude_invalid", "angular_velocity_invalid")):
            raise ValueError("estimator_failsafe_flags")
        # Commander publishes canArm(current nav_state), not general health.
        # After confirmed LAND/disarm it restores the previous mode intention;
        # Offboard can no longer arm after the intentional stream handover.
        terminal = (terminal_landed and s.get("arming_state") == 1
                    and self.data["vehicle_land_detected"].get("landed") is True)
        if s.get("failsafe") is not False or (not terminal and s.get("pre_flight_checks_pass") is not True):
            raise ValueError("vehicle_health_invalid")
        if s.get("system_id") != c["vehicle_system"] or s.get("component_id") != c["vehicle_component"]:
            raise ValueError("vehicle_identity_mismatch")
        if self.frozen_reference is not None and self.reference() != self.frozen_reference:
            raise ValueError("estimator_reference_reset")
        if self.reference_manager is not None and self.reference_manager.rejected:
            raise ValueError("estimator_reference_reset:"+self.reference_manager.rejected)
        position = finite([p[k] for k in ("x", "y", "z")], 3)
        velocity = finite([p[k] for k in ("vx", "vy", "vz")], 3)
        yaw = yaw_from_quaternion(a["q"])
        return {"t": p["timestamp"]/1e6, "ros_sim_s": sim, "receipt_monotonic_s": wall,
                "position": position, "velocity": velocity, "yaw": yaw,
                "arming_state": s["arming_state"], "nav_state": s["nav_state"],
                "landed": self.data["vehicle_land_detected"]["landed"],
                "clock_ages_s": offsets}
