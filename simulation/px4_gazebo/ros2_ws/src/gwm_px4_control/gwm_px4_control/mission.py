"""Bounded position Offboard state machine. No ROS imports or process actions."""
import math

from .frames import offset_target, wrap
from .protocol import AckTracker
from .estimator_reference import ReferenceManager, V2

ORDER = ("WAIT_FOR_CLOCK_AND_CONNECTION", "WAIT_FOR_VALID_ESTIMATION", "PRESTREAM_SAFE_SETPOINTS",
         "REQUEST_OFFBOARD", "VERIFY_OFFBOARD", "REQUEST_ARM", "VERIFY_ARMED", "TAKEOFF",
         "INITIAL_HOVER", "EAST_TEST", "RETURN_AFTER_EAST", "NORTH_TEST", "RETURN_AFTER_NORTH",
         "YAW_TEST", "RESTORE_INITIAL_YAW", "FINAL_HOVER", "REQUEST_LAND", "VERIFY_LANDING",
         "VERIFY_LANDED_AND_DISARMED", "COMPLETE")
MOTION = ORDER[7:16]
ORDER_V2 = ORDER[:8] + ("STABILIZE_REFERENCE", "LOCK_REFERENCE") + ORDER[8:]
STREAMING = ORDER[2:16] + ("STABILIZE_REFERENCE", "LOCK_REFERENCE")


def ramp(current, goal, speed, dt):
    delta = [b-a for a, b in zip(current, goal)]
    distance = math.sqrt(sum(v*v for v in delta))
    scale = min(1.0, speed*dt/distance) if distance else 0.0
    return tuple(a+d*scale for a, d in zip(current, delta))


class Mission:
    def __init__(self, config, cache, start_wall, flight):
        self.config, self.cache, self.start_wall, self.flight = config, cache, start_wall, flight
        self.order = ORDER_V2 if config.get("reference_policy") == V2 else ORDER
        self.reference = None
        self.state = ORDER[0]
        self.enter_sim = None
        self.enter_wall = start_wall
        self.start_sim = None
        self.last_sim = None
        self.last_sample = None
        self.stable_since = None
        self.stable_reference = None
        self.origin = None
        self.initial_yaw = None
        self.target = None
        self.yaw_target = None
        self.window = []
        self.windows = {}
        self.transitions = []
        self.transactions = AckTracker(config)
        self.ack_status = None
        self.failure = None
        self.recovery_wall = None
        self.done = False
        self.connectivity_passed = False
        self.prestream_first_sim = None
        self.events = []

    def transition(self, next_state, sim, wall, **evidence):
        if next_state not in ("ABORTED", "RECOVERY"):
            if self.state not in self.order or self.order.index(next_state) != self.order.index(self.state)+1:
                raise ValueError("illegal_transition:" + self.state + "->" + next_state)
        record = {"from": self.state, "to": next_state, "sim_s": sim, "monotonic_s": wall,
                  "evidence": evidence}
        self.transitions.append(record)
        self.events.append({"event": "transition", **record})
        self.state, self.enter_sim, self.enter_wall = next_state, sim, wall
        self.window = []
        self.ack_status = None
        if next_state == "COMPLETE":
            self.done = True

    def request(self, command, sim, wall):
        self.transactions.issue(command, self.state, sim, wall)
        params = [0.0]*7
        if command == 176:
            params[0:2] = [1.0, 6.0]
        elif command == 400:
            params[0] = 1.0  # param2 stays zero: normal arm only.
        elif command != 21:
            raise ValueError("unsupported_command")
        return {"command": command, "params": params}

    def ack(self, ack, sim, wall):
        outcome = self.transactions.receive(ack, self.state, sim, wall)
        self.events.append({"event": "ack_match", "outcome": outcome, "ack": ack, "sim_s": sim})
        if outcome != "unmatched":
            self.ack_status = outcome

    def abort(self, reason, sim, wall):
        if self.failure:
            return None
        self.failure, self.recovery_wall = reason, wall
        if self.reference is not None:
            self.events.extend(self.reference.events)
            self.reference.events.clear()
        self.events.append({"event": "abort", "reason": reason, "sim_s": sim, "monotonic_s": wall})
        try:
            sample = self.cache.validate(sim, wall)
        except (ValueError, KeyError):
            sample = None
        previous = self.state
        self.transition("RECOVERY", sim, wall, reason=reason)
        # Stop old setpoints. A healthy owned Offboard vehicle may receive one
        # normal LAND command; otherwise observe the existing PX4 fallback.
        if sample and sample["arming_state"] == 1 and sample["landed"] is True:
            self.done = True
        elif (sample and sample["arming_state"] == 2 and sample["nav_state"] == 14
              and self.transactions.pending is None
              and not any(r["command"] == 21 for r in self.transactions.records)
              and previous not in ("REQUEST_LAND", "VERIFY_LANDING", "VERIFY_LANDED_AND_DISARMED")):
            return self.request(21, sim, wall)
        return None

    def goal(self):
        c = self.config
        offset = (c["east_m"] if self.state == "EAST_TEST" else 0.0,
                  c["north_m"] if self.state == "NORTH_TEST" else 0.0, c["height_m"])
        yaw = wrap(self.initial_yaw - math.radians(c["yaw_enu_deg"])) if self.state == "YAW_TEST" else self.initial_yaw
        return offset_target(self.origin, offset), yaw, offset

    def tick(self, sim, wall, graph_ok):
        c = self.config
        actions = {"setpoint": None, "command": None, "sample": None, "heartbeat_only": False}
        if self.done:
            return actions
        if wall-self.start_wall > c["wall_deadline_s"]:
            self.failure = self.failure or "wall_deadline"
            self.done = True
            return actions
        if self.failure:
            status, land = self.cache.data.get("vehicle_status", {}), self.cache.data.get("vehicle_land_detected", {})
            if (status.get("arming_state") == 1 and land.get("landed") is True
                    and wall-self.cache.receipts.get("vehicle_status", 0) < c["state_fresh_wall_s"]
                    and wall-self.cache.receipts.get("vehicle_land_detected", 0) < c["state_fresh_wall_s"]):
                self.done = True
            if wall-self.recovery_wall > c["recovery_wall_s"]:
                self.done = True
            return actions
        if self.cache.clock_wall is not None and wall-self.cache.clock_wall > c["clock_stall_wall_s"]:
            raise ValueError("clock_stalled")
        if sim <= 0:
            if wall-self.start_wall > c["startup_wall_s"]:
                raise TimeoutError("startup_clock_timeout")
            return actions
        if self.start_sim is None:
            self.start_sim = sim
        if sim-self.start_sim > c["simulation_deadline_s"]:
            raise TimeoutError("simulation_deadline")
        if self.state in ORDER[:2] and wall-self.start_wall > c["startup_wall_s"]:
            raise TimeoutError("startup_connection_estimation_timeout")
        if self.state not in ORDER[:2] and (sim-self.enter_sim > c["phase_deadline_sim_s"]
                                            or wall-self.enter_wall > c["phase_deadline_wall_s"]):
            raise TimeoutError("state_transition_timeout:" + self.state)
        self.transactions.check_deadline(sim, wall)
        if self.ack_status in ("temporarily_rejected", "denied", "unsupported", "failed", "cancelled", "unknown_result"):
            raise ValueError("command_ack_" + self.ack_status)
        if self.state in ORDER[:2]:
            if self.cache.data.get("vehicle_status", {}).get("arming_state") == 2:
                raise ValueError("restart_reject_armed")
            if self.state == ORDER[0]:
                if graph_ok:
                    self.transition(ORDER[1], sim, wall)
                return actions
            try:
                sample = self.cache.validate(sim, wall)
            except ValueError:
                self.stable_since = None
                return actions
            if sample["landed"] is not True:
                raise ValueError("restart_reject_airborne")
            reference = self.cache.reference()
            if self.stable_since is None or reference != self.stable_reference:
                self.stable_since, self.stable_reference = sim, reference
            required = c["estimator_stable_sim_s"] if self.flight else c["observation_sim_s"]
            if graph_ok and sim-self.stable_since >= required:
                self.connectivity_passed = True
                if not self.flight:
                    self.done = True
                    return actions
                self.origin, self.initial_yaw = sample["position"], sample["yaw"]
                if self.config.get("reference_policy") == V2:
                    self.reference = ReferenceManager(c, self.cache.data, self.origin, self.initial_yaw)
                    self.cache.reference_manager = self.reference
                else:
                    self.cache.frozen_reference = reference
                self.target, self.yaw_target = self.origin, self.initial_yaw
                self.transition(ORDER[2], sim, wall, origin_ned=self.origin, yaw_ned=self.initial_yaw)
            return actions
        if not graph_ok:
            raise ValueError("control_graph_invalid_or_competing_owner")
        sample = self.cache.validate(sim, wall, terminal_landed=(
            self.config.get("reference_policy") == V2 and self.state == "VERIFY_LANDED_AND_DISARMED"))
        actions["sample"] = sample
        reference_pending = False
        if self.reference is not None:
            reconciled = self.reference.inspect(self.cache.data, sample, sim, wall, self.state)
            reference_pending = reconciled["pending"]
            if reconciled.get("accepted"):
                old_yaw = self.yaw_target
                self.initial_yaw = self.reference.anchor
                self.yaw_target = wrap(self.yaw_target+reconciled["correction"])
                self.events.append({"event": "reference_target_correction", "sim_s": sim,
                                    "yaw_before": old_yaw, "yaw_after": self.yaw_target,
                                    "position_before": self.target, "position_after": self.target,
                                    "delta_heading": reconciled["correction"]})
            self.events.extend(self.reference.events)
            self.reference.events.clear()
        if self.last_sample is not None and sample["t"]-self.last_sample > c["max_sample_gap_sim_s"]:
            raise ValueError("observation_gap")
        fresh = self.last_sample is None or sample["t"] > self.last_sample
        self.last_sample = sample["t"]
        if self.last_sim is not None and sim < self.last_sim:
            raise ValueError("clock_backwards")
        dt = max(0.0, sim-(self.last_sim if self.last_sim is not None else sim))
        self.last_sim = sim
        if math.hypot(sample["position"][0]-self.origin[0], sample["position"][1]-self.origin[1]) > c["max_horizontal_radius_m"]:
            raise ValueError("flight_envelope_horizontal")
        height = self.origin[2]-sample["position"][2]
        if not c["min_height_m"] <= height <= c["max_height_m"]:
            raise ValueError("flight_envelope_height")
        if self.state in MOTION + ("STABILIZE_REFERENCE", "LOCK_REFERENCE") and (sample["arming_state"] != 2 or sample["nav_state"] != 14):
            raise ValueError("offboard_armed_state_lost")
        if reference_pending:
            actions["heartbeat_only"] = True
            return actions
        if (self.state == "PRESTREAM_SAFE_SETPOINTS" and self.prestream_first_sim is not None
                and sim-self.prestream_first_sim >= c["prestream_sim_s"]):
            self.transition("REQUEST_OFFBOARD", sim, wall)
            actions["command"] = self.request(176, sim, wall)
        elif self.state == "REQUEST_OFFBOARD" and self.ack_status == "accepted":
            self.transition("VERIFY_OFFBOARD", sim, wall, ack_accepted=True)
        elif self.state == "VERIFY_OFFBOARD" and sample["nav_state"] == 14:
            self.transition("REQUEST_ARM", sim, wall, observed_nav_state=14)
            actions["command"] = self.request(400, sim, wall)
        elif self.state == "REQUEST_ARM" and self.ack_status == "accepted":
            self.transition("VERIFY_ARMED", sim, wall, ack_accepted=True)
        elif self.state == "VERIFY_ARMED" and sample["arming_state"] == 2 and sample["nav_state"] == 14:
            self.transition("TAKEOFF", sim, wall, observed_arming_state=2)
        elif self.state == "STABILIZE_REFERENCE":
            if self.reference.ready(sim):
                self.transition("LOCK_REFERENCE", sim, wall, final_alignment_stable=True)
        elif self.state == "LOCK_REFERENCE":
            self.reference.lock(sim)
            self.events.extend(self.reference.events)
            self.reference.events.clear()
            self.transition("INITIAL_HOVER", sim, wall, reference_lock_sim_s=sim)
        elif self.state in MOTION:
            goal, yaw, offset = self.goal()
            self.target = ramp(self.target, goal, c["ramp_m_s"], min(dt, c["max_sample_gap_sim_s"]))
            yaw_step = max(-math.radians(c["yaw_ramp_deg_s"])*dt,
                           min(math.radians(c["yaw_ramp_deg_s"])*dt, wrap(yaw-self.yaw_target)))
            self.yaw_target = wrap(self.yaw_target+yaw_step)
            arrived = math.dist(self.target, goal) < 1e-6 and abs(wrap(yaw-self.yaw_target)) < 1e-6
            horizontal = math.hypot(sample["position"][0]-goal[0], sample["position"][1]-goal[1])
            vertical = abs(sample["position"][2]-goal[2])
            yaw_error = abs(wrap(sample["yaw"]-yaw))
            settled = (arrived and horizontal <= c["horizontal_tolerance_m"]
                       and vertical <= c["height_tolerance_m"] and yaw_error <= math.radians(c["yaw_tolerance_deg"])
                       and math.sqrt(sum(v*v for v in sample["velocity"])) <= c["settled_speed_m_s"]
                       and sample["landed"] is False)
            if self.window and (horizontal > c["horizontal_tolerance_m"] or vertical > c["height_tolerance_m"]
                                or yaw_error > math.radians(c["yaw_tolerance_deg"]) or sample["landed"] is not False):
                raise ValueError("fixed_dwell_window_failed:" + self.state)
            if fresh and (self.window or settled):
                self.window.append({**sample, "goal_ned": goal, "goal_yaw_ned": yaw, "offset_enu": offset})
                duration = (c["initial_hover_sim_s"] if self.state == "INITIAL_HOVER" else
                            c["final_hover_sim_s"] if self.state == "FINAL_HOVER" else c["dwell_sim_s"])
                if self.state == "TAKEOFF" or self.window[-1]["t"]-self.window[0]["t"] >= duration:
                    if self.state != "TAKEOFF":
                        self.windows[self.state] = {"start_sim_s": self.window[0]["t"], "end_sim_s": self.window[-1]["t"],
                            "count": len(self.window), "goal_ned": goal, "goal_yaw_ned": yaw, "offset_enu": offset}
                    next_state = self.order[self.order.index(self.state)+1]
                    self.transition(next_state, sim, wall, settled_observed=True)
                    if next_state == "REQUEST_LAND":
                        actions["command"] = self.request(21, sim, wall)
        elif self.state == "REQUEST_LAND" and self.ack_status == "accepted":
            self.transition("VERIFY_LANDING", sim, wall, ack_accepted=True)
        elif self.state == "VERIFY_LANDING" and sample["nav_state"] == 18:
            self.transition("VERIFY_LANDED_AND_DISARMED", sim, wall, observed_nav_state=18)
        elif self.state == "VERIFY_LANDED_AND_DISARMED" and sample["landed"] is True and sample["arming_state"] == 1:
            self.transition("COMPLETE", sim, wall, observed_landed=True, observed_disarmed=True)
        if self.state in STREAMING and dt > 0:
            actions["setpoint"] = {"position": self.target, "yaw": self.yaw_target}
            if self.prestream_first_sim is None:
                self.prestream_first_sim = sim
        return actions
