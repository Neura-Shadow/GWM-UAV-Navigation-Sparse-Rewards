"""P4 mission: the accepted P2 state machine plus MAP_SCAN, TRANSIT and GOAL_HOVER (contract §4).

Initialization, estimator reference, yaw handover, ACK tracking, freshness, the
landing sequence and recovery are the unchanged P2 `Mission` (imported, never
edited). Only the three P4 phases are new; their per-tick safety checks repeat
the P2 checks with the P4 geofence in place of the P2 mission radius.
Frames: P2 keeps NED targets; the planner works in PX4 local ENU.
"""
import math

from gwm_px4_control.estimator_reference import V3
from gwm_px4_control.frames import wrap
from gwm_px4_control.mission import ORDER_V3, Mission

from . import planner, supervisor
from .local_map import Grid

P4_STATES = ("MAP_SCAN", "TRANSIT", "GOAL_HOVER")
P4_ORDER = ORDER_V3[:ORDER_V3.index("INITIAL_HOVER") + 1] + P4_STATES + (
    "REQUEST_LAND", "VERIFY_LANDING", "VERIFY_LANDED_AND_DISARMED", "COMPLETE")
ABORT_PREFIX = "p4_supervisor_abort:"


class P4Abort(ValueError):
    """A designed supervisor ABORT; the node routes it through P2 recovery (one LAND)."""


def enu_from_ned(position):
    return float(position[1]), float(position[0])


def yaw_enu_from_ned(yaw):
    return wrap(math.pi / 2 - yaw)


def p4_p2_config(p2_config, p4_config):
    """P2 limits unchanged, except the P2 mission radius, replaced by the P4 geofence (contract §3/§4)."""
    fence = p4_config["geofence"]
    merged = dict(p2_config)
    merged["max_horizontal_radius_m"] = math.hypot(max(abs(fence["x_min_m"]), fence["x_max_m"]), fence["y_abs_max_m"])
    if fence["height_max_m"] > p2_config["max_height_m"]:
        raise ValueError("P4 ceiling exceeds the P2 ceiling")
    return merged


class P4Mission(Mission):
    def __init__(self, p2_config, p4_config, cache, start_wall, flight, ground_diagnostic=False, ground_duration=3.0):
        if p2_config.get("reference_policy") != V3:
            raise ValueError("P4 requires the accepted P2 v3 reference policy")
        super().__init__(p4_p2_config(p2_config, p4_config), cache, start_wall, flight, ground_diagnostic, ground_duration)
        self.order = P4_ORDER
        self.p4 = p4_config
        self.max_age = p4_config["perception"]["snapshot_max_age_sim_s"]
        self.snapshot = None
        self.grid = None
        self.traversed = None
        self.consecutive_unusable = 0
        self.adapter_health = None
        self.positions = []
        self.last_reference = None
        self.reset_sim = None
        self.supervisor = None
        self.scan = None
        self.goal_window = []
        self.goal_reached_sim = None
        self.p4_failure = None

    # Inputs from the node ------------------------------------------------------------------------
    def on_snapshot(self, snapshot):
        if snapshot.get("usable") and snapshot.get("grid"):
            grid = Grid.decode(snapshot["grid"])
            if self.grid is None or grid.cells.shape != self.grid.cells.shape:
                self.traversed = None
            self.grid, self.snapshot = grid, snapshot
            self.consecutive_unusable = 0
            self._update_traversed()
        else:
            self.consecutive_unusable = int(snapshot.get("consecutive_unusable") or self.consecutive_unusable + 1)

    def on_adapter_health(self, record):
        self.adapter_health = record

    # Helpers ------------------------------------------------------------------------------------
    @property
    def origin_enu(self):
        return enu_from_ned(self.origin)

    @property
    def goal_enu(self):
        goal = self.p4["mission"]["goal_enu_m"]
        return self.origin_enu[0] + goal[0], self.origin_enu[1] + goal[1]

    def _update_traversed(self):
        if self.grid is None:
            return
        if self.traversed is None:
            self.traversed = planner.traversed_mask(self.grid, self.positions, self.p4["envelope"]["r_body_m"])

    def _record_position(self, sample, sim):
        xy = enu_from_ned(sample["position"])
        if not self.positions or math.dist(self.positions[-1], xy) >= 0.02:
            self.positions.append(xy)
            if self.grid is not None and self.traversed is not None:
                self.traversed |= planner.traversed_mask(self.grid, [xy], self.p4["envelope"]["r_body_m"])
        reference = self.cache.reference()
        if self.last_reference is not None and reference != self.last_reference:
            self.reset_sim = sim
            self.events.append({"event": "p4_estimator_reset_observed", "sim_s": sim, "before": self.last_reference,
                                "after": reference})
        self.last_reference = reference

    def _inputs(self, sim, sample):
        return {"sim": sim, "snapshot": self.snapshot, "grid": self.grid,
                "traversed": self.traversed, "adapter_health": self.adapter_health,
                "consecutive_unusable": self.consecutive_unusable, "reset_sim": self.reset_sim,
                "vehicle": enu_from_ned(sample["position"]), "vehicle_yaw": yaw_enu_from_ned(sample["yaw"]),
                "target": enu_from_ned(self.target), "goal": self.goal_enu, "origin": self.origin_enu,
                "geofence": self.p4["geofence"]}

    def _set_target_enu(self, xy):
        self.target = (xy[1], xy[0], self.origin[2] - self.p4["mission"]["height_m"])

    def _check_geofence(self, sample):
        fence = self.p4["geofence"]
        x, y = enu_from_ned(sample["position"])
        x, y = x - self.origin_enu[0], y - self.origin_enu[1]
        height = self.origin[2] - sample["position"][2]
        if not (fence["x_min_m"] <= x <= fence["x_max_m"] and abs(y) <= fence["y_abs_max_m"]):
            raise ValueError("p4_geofence_horizontal")
        if not fence["height_min_m"] <= height <= fence["height_max_m"]:
            raise ValueError("p4_geofence_height")

    # Tick ---------------------------------------------------------------------------------------
    def tick(self, sim, wall, graph_ok):
        if self.state not in P4_STATES:
            actions = super().tick(sim, wall, graph_ok)
            if actions.get("sample") is not None and self.origin is not None:
                self._record_position(actions["sample"], sim)
            return actions
        return self._p4_tick(sim, wall, graph_ok)

    def _phase_deadline(self, sim, wall):
        c = self.config
        if self.state == "MAP_SCAN":
            limit = self.p4["mission"]["map_scan_deg"] / c["yaw_ramp_deg_s"] + self.p4["supervisor"]["stale_abort_sim_s"] + 5.0
        elif self.state == "TRANSIT":
            limit = self.p4["supervisor"]["transit_deadline_sim_s"] + 5.0  # the supervisor aborts first at 150 s
        else:
            limit = c["phase_deadline_sim_s"]
        if sim - self.enter_sim > limit or (self.state == "GOAL_HOVER" and wall - self.enter_wall > c["phase_deadline_wall_s"]):
            raise TimeoutError("state_transition_timeout:" + self.state)

    def _p4_tick(self, sim, wall, graph_ok):
        """The P2 per-tick safety checks, then the P4 phase logic, then one setpoint."""
        self.control_selection = None
        self._wall = wall
        c = self.config
        actions = {"setpoint": None, "command": None, "sample": None, "heartbeat_only": False}
        if self.done:
            return actions
        if wall - self.start_wall > c["wall_deadline_s"]:
            self.failure = self.failure or "wall_deadline"
            self.done = True
            return actions
        if self.cache.clock_wall is not None and wall - self.cache.clock_wall > c["clock_stall_wall_s"]:
            raise ValueError("clock_stalled")
        if sim - self.start_sim > c["simulation_deadline_s"]:
            raise TimeoutError("simulation_deadline")
        self._phase_deadline(sim, wall)
        self.transactions.check_deadline(sim, wall)
        if self.ack_status in ("temporarily_rejected", "denied", "unsupported", "failed", "cancelled", "unknown_result"):
            raise ValueError("command_ack_" + self.ack_status)
        if not graph_ok:
            raise ValueError("control_graph_invalid_or_competing_owner")
        sample = self.cache.validate(sim, wall)
        actions["sample"] = sample
        self.control_selection = sample
        reference_pending = False
        if self.reference is not None:
            reconciled = self.reference.inspect(self.cache.data, sample, sim, wall, self.state)
            reference_pending = reconciled["pending"]
            if reconciled.get("accepted"):
                old_yaw = self.yaw_target
                self.initial_yaw = self.reference.anchor
                self.yaw_target = wrap(self.yaw_target + reconciled["correction"])
                self.events.append({"event": "reference_target_correction", "sim_s": sim, "yaw_before": old_yaw,
                                    "yaw_after": self.yaw_target, "position_before": self.target,
                                    "position_after": self.target, "delta_heading": reconciled["correction"]})
            self.events.extend(self.reference.events)
            self.reference.events.clear()
        if self.last_sample is not None and sample["t"] - self.last_sample > c["max_sample_gap_sim_s"]:
            raise ValueError("observation_gap")
        fresh = self.last_sample is None or sample["t"] > self.last_sample
        self.last_sample = sample["t"]
        if self.last_sim is not None and sim < self.last_sim:
            raise ValueError("clock_backwards")
        dt = max(0.0, sim - (self.last_sim if self.last_sim is not None else sim))
        self.last_sim = sim
        self._check_geofence(sample)
        if sample["arming_state"] != 2 or sample["nav_state"] != 14 or sample["landed"] is not False:
            raise ValueError("offboard_armed_state_lost")
        self._record_position(sample, sim)
        if not reference_pending:
            step = min(dt, c["max_sample_gap_sim_s"])
            if self.state == "MAP_SCAN":
                self._map_scan(sim, sample, step)
            elif self.state == "TRANSIT":
                self._transit(sim, sample, step)
            elif self.state == "GOAL_HOVER":
                actions["command"] = self._goal_hover(sim, wall, sample, fresh)
        if self.state in P4_STATES and dt > 0:
            actions["setpoint"] = self.position_action()
        return actions

    # MAP_SCAN: one full in-place revolution as an L5 yaw (revision 3) ----------------------------
    def _map_scan(self, sim, sample, dt):
        if self.scan is None:
            self.scan = {"rotated_rad": 0.0, "completed_sim_s": None, "start_sim_s": sim, "held_ticks": 0,
                         "stale_since": None, "hold_reasons": {}}
            self.events.append({"event": "p4_map_scan_started", "sim_s": sim, "yaw_ned": self.yaw_target})
        scan = self.scan
        total = math.radians(self.p4["mission"]["map_scan_deg"])
        gate = supervisor.Supervisor(self.p4, sim)
        stale = gate.stale_reason(self._inputs(sim, sample)) if self.grid is not None else "no_usable_snapshot"
        if stale:
            scan["stale_since"] = scan["stale_since"] if scan["stale_since"] is not None else sim
            if sim - scan["stale_since"] > self.p4["supervisor"]["stale_abort_sim_s"]:
                raise P4Abort(ABORT_PREFIX + "map_scan_stale_inputs:" + stale)
            scan["hold_reasons"][stale] = scan["hold_reasons"].get(stale, 0) + 1
            return
        scan["stale_since"] = None
        if scan["rotated_rad"] < total:
            ctx = planner.Context(enu_from_ned(self.target), enu_from_ned(sample["position"]),
                                  yaw_enu_from_ned(sample["yaw"]), self.goal_enu, self.origin_enu,
                                  sim - self.snapshot["acquisition_sim_s"], self.p4["geofence"])
            permitted, reason = planner.yaw_permitted(self.grid, self.traversed, ctx, self.p4["planner"],
                                                      self.p4["envelope"], self.max_age)
            if not permitted:
                scan["held_ticks"] += 1
                scan["hold_reasons"][reason] = scan["hold_reasons"].get(reason, 0) + 1
                return
            step = min(math.radians(self.config["yaw_ramp_deg_s"]) * dt, total - scan["rotated_rad"])
            scan["rotated_rad"] += step
            self.yaw_target = wrap(self.yaw_target - step)  # counter-clockwise in ENU = decreasing NED yaw
            if scan["rotated_rad"] >= total - 1e-12:
                self.yaw_target = self.initial_yaw
                scan["completed_sim_s"] = sim
            return
        if self.snapshot["acquisition_sim_s"] > scan["completed_sim_s"]:
            self.supervisor = supervisor.Supervisor(self.p4, sim)
            self.events.append({"event": "p4_map_scan_completed", "sim_s": sim, **{k: v for k, v in scan.items()}})
            self.transition("TRANSIT", sim, self._wall, map_scan_rotated_deg=math.degrees(scan["rotated_rad"]))

    # TRANSIT: supervisor + planner ------------------------------------------------------------------
    def _transit(self, sim, sample, dt):
        before = len(self.supervisor.transitions)
        decision = self.supervisor.step(self._inputs(sim, sample))
        for record in self.supervisor.transitions[before:]:
            self.events.append({"event": "p4_supervisor_transition", **record})
        self.events.append({"event": "p4_decision", "sim_s": sim, "state": decision["state"],
                            "action": decision["action"], "heading": decision["heading"], "end": decision["end"],
                            "frozen_yaw": decision["frozen_yaw"], "reason": decision["reason"],
                            "stale": decision["stale"], "target_enu": enu_from_ned(self.target),
                            "vehicle_enu": enu_from_ned(sample["position"]),
                            "snapshot_acquisition_sim_s": (self.snapshot or {}).get("acquisition_sim_s"),
                            "path_valid": (decision["path"] or {}).get("valid"),
                            "occupied_in_corridor": (decision["path"] or {}).get("occupied_in_corridor"),
                            "plan": decision["plan"]})
        if decision["state"] == supervisor.ABORT:
            raise P4Abort(ABORT_PREFIX + str(decision["reason"]))
        # Revision 4: TRANSIT translates at the pinned P4 speed, below P2's ramp (kept for every P2 phase).
        target, yaw = supervisor.apply_action(enu_from_ned(self.target), yaw_enu_from_ned(self.yaw_target), decision,
                                              self.p4["mission"]["transit_speed_m_s"],
                                              math.radians(self.config["yaw_ramp_deg_s"]), dt, self.goal_enu)
        self._set_target_enu(target)
        self.yaw_target = wrap(math.pi / 2 - yaw)
        m = self.p4["mission"]
        at_goal = math.dist(enu_from_ned(self.target), self.goal_enu) < 1e-6
        horizontal = math.dist(enu_from_ned(sample["position"]), self.goal_enu)
        speed = math.sqrt(sum(v * v for v in sample["velocity"]))
        if decision["state"] == supervisor.NORMAL and at_goal and horizontal <= m["goal_tolerance_m"] \
                and speed <= m["settled_speed_m_s"]:
            self.goal_reached_sim = sim
            self.transition("GOAL_HOVER", sim, self._wall, goal_enu=self.goal_enu)

    # GOAL_HOVER --------------------------------------------------------------------------------------
    def _goal_hover(self, sim, wall, sample, fresh):
        m = self.p4["mission"]
        horizontal = math.dist(enu_from_ned(sample["position"]), self.goal_enu)
        speed = math.sqrt(sum(v * v for v in sample["velocity"]))
        if horizontal > m["goal_tolerance_m"] or speed > m["settled_speed_m_s"]:
            self.goal_window = []
            return None
        if fresh:
            self.goal_window.append(sample["t"])
        if self.goal_window and self.goal_window[-1] - self.goal_window[0] >= m["goal_hover_sim_s"]:
            self.windows["GOAL_HOVER"] = {"start_sim_s": self.goal_window[0], "end_sim_s": self.goal_window[-1],
                                          "count": len(self.goal_window), "goal_enu": self.goal_enu}
            self.transition("REQUEST_LAND", sim, wall, settled_observed=True)
            return self.request(21, sim, wall)
        return None

    def summary(self):
        return {"supervisor_transitions": self.supervisor.transitions if self.supervisor else [],
                "supervisor_state": self.supervisor.state if self.supervisor else None,
                "abort_reason": self.supervisor.abort_reason if self.supervisor else None,
                "map_scan": self.scan, "goal_reached_sim_s": self.goal_reached_sim,
                "goal_enu": self.goal_enu if self.origin is not None else None,
                "origin_enu": self.origin_enu if self.origin is not None else None,
                "positions_recorded": len(self.positions)}
