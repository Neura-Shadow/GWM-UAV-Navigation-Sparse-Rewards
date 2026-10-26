"""Deterministic safety supervisor (contract §5 transition table, revisions 2 and 3).

States: NORMAL, OBSTACLE_DETECTED, AVOIDING, HOVER, RESUME, ABORT. Pure: the
caller supplies time, the newest usable map, health and pose; the supervisor
returns one action. A stale-input HOVER freezes translation and yaw.
"""
import math

from . import planner

NORMAL, OBSTACLE_DETECTED, AVOIDING, HOVER, RESUME, ABORT = (
    "NORMAL", "OBSTACLE_DETECTED", "AVOIDING", "HOVER", "RESUME", "ABORT")


class Supervisor:
    def __init__(self, config, start_sim):
        self.config = config
        self.state = NORMAL
        self.transit_start = start_sim
        self.hover_since = None
        self.stale_since = None
        self.scanned = set()
        self.transitions = []
        self.abort_reason = None
        self.max_age = config["perception"]["snapshot_max_age_sim_s"]

    def _go(self, state, sim, reason, **evidence):
        if state == self.state:
            return
        self.transitions.append({"from": self.state, "to": state, "sim_s": sim, "reason": reason, **evidence})
        if state == HOVER:
            self.hover_since = sim
        elif self.state == HOVER:
            self.hover_since = None
            self.scanned = set()
        self.state = state
        if state == ABORT:
            self.abort_reason = reason

    def stale_reason(self, inputs):
        snapshot = inputs.get("snapshot")
        if snapshot is None:
            return "no_usable_snapshot"
        if inputs["sim"] - snapshot["acquisition_sim_s"] > self.max_age:
            return "snapshot_stale"
        health = inputs.get("adapter_health") or {}
        if health.get("status") != "fresh":
            return "adapter_health_not_fresh"
        # A stopped adapter stops publishing health: an old 'fresh' record is not fresh.
        if health.get("sim_s") is None or inputs["sim"] - health["sim_s"] > self.config["supervisor"]["adapter_health_max_age_sim_s"]:
            return "adapter_health_missing"
        if health.get("calibration_identity") != snapshot.get("calibration_identity"):
            return "calibration_mismatch"
        if inputs.get("consecutive_unusable", 0) >= self.config["perception"]["unusable_frames_to_hover"]:
            return "unusable_frames"
        reset = inputs.get("reset_sim")
        if reset is not None and reset >= snapshot["acquisition_sim_s"]:
            return "estimator_reset_since_last_usable_frame"
        return None

    def step(self, inputs):
        """One tick. inputs: sim, snapshot (newest usable: acquisition_sim_s, calibration_identity),
        grid, traversed, adapter_health, consecutive_unusable, reset_sim, vehicle, vehicle_yaw,
        target, goal, origin."""
        sim, c = inputs["sim"], self.config
        out = {"state": None, "action": planner.HOLD, "heading": None, "end": None, "length": None,
               "frozen_yaw": False, "reason": None, "path": None, "plan": None, "stale": None}
        if self.state == ABORT:
            out.update(state=ABORT, reason=self.abort_reason, frozen_yaw=True)
            return out
        if sim - self.transit_start > c["supervisor"]["transit_deadline_sim_s"]:
            self._go(ABORT, sim, "transit_deadline")
            out.update(state=ABORT, reason=self.abort_reason, frozen_yaw=True)
            return out
        stale = self.stale_reason(inputs)
        out["stale"] = stale
        if stale:
            if self.stale_since is None:
                self.stale_since = sim
            if sim - self.stale_since > c["supervisor"]["stale_abort_sim_s"]:
                self._go(ABORT, sim, "stale_inputs:" + stale)
            else:
                self._go(HOVER, sim, "stale_inputs:" + stale)
            out.update(state=self.state, reason=self.abort_reason if self.state == ABORT else stale, frozen_yaw=True)
            return out
        self.stale_since = None
        grid, traversed = inputs["grid"], inputs["traversed"]
        clearance_limit = c["envelope"]["r_body_m"] + c["supervisor"]["min_clearance_margin_m"]
        if grid.occupied_distance(inputs["vehicle"], inputs["vehicle"], clearance_limit) is not None:
            self._go(ABORT, sim, "clearance_below_body_margin")
            out.update(state=ABORT, reason=self.abort_reason, frozen_yaw=True)
            return out
        age = sim - inputs["snapshot"]["acquisition_sim_s"]
        ctx = planner.Context(inputs["target"], inputs["vehicle"], inputs["vehicle_yaw"], inputs["goal"],
                              inputs["origin"], age, inputs["geofence"],
                              scanned={round(h, 6) for h in self.scanned})
        path = planner.path_status(grid, traversed, ctx, c["planner"], self.max_age)
        decision = planner.plan(grid, traversed, ctx, c["planner"], c["envelope"], self.max_age)
        progress = decision["action"] in (planner.TRANSLATE, planner.YAW_TO)
        evidence = {"snapshot_acquisition_sim_s": inputs["snapshot"]["acquisition_sim_s"]}
        if self.state == NORMAL:
            if not path["valid"]:
                if path["occupied_in_corridor"]:
                    self._go(OBSTACLE_DETECTED, sim, "occupied_in_corridor", **evidence)
                else:
                    self._go(HOVER, sim, "path_" + str(path["reason"]), **evidence)
        if self.state == OBSTACLE_DETECTED:
            self._go(AVOIDING if progress else HOVER, sim, "progress_candidate" if progress else "no_progress_candidate",
                     **evidence)
        elif self.state == AVOIDING:
            if path["valid"]:
                self._go(RESUME, sim, "path_valid", **evidence)
            elif not progress:
                self._go(HOVER, sim, "no_progress_candidate", **evidence)
        elif self.state == RESUME:
            if path["valid"] and planner.deviation(ctx.target, ctx) <= c["supervisor"]["resume_deviation_m"]:
                self._go(NORMAL, sim, "rejoined_path", **evidence)
            elif not progress:
                self._go(HOVER, sim, "no_progress_candidate", **evidence)
        elif self.state == HOVER:
            if progress:
                self._go(AVOIDING, sim, "progress_candidate", **evidence)
            elif sim - self.hover_since > c["supervisor"]["hover_timeout_sim_s"]:
                self._go(ABORT, sim, "hover_timeout", **evidence)
                out.update(state=ABORT, reason=self.abort_reason, frozen_yaw=True, path=path)
                return out
        if self.state == HOVER and decision["action"] == planner.SCAN:
            self.scanned.add(decision["heading"])
        if self.state == HOVER and decision["action"] in (planner.TRANSLATE, planner.YAW_TO):
            decision = dict(decision, action=planner.HOLD, reason="hover")  # unreachable: progress leaves HOVER
        out.update(state=self.state, action=decision["action"], heading=decision["heading"], end=decision["end"],
                   length=decision["length"], reason=decision["reason"], path=path,
                   plan={k: decision[k] for k in ("candidates_valid", "candidates_progress", "best", "path_heading")})
        return out


def apply_action(target, yaw_target, decision, speed, yaw_rate, dt, goal):
    """Advance the position target along a TRANSLATE segment (never past its end or the goal) and
    the yaw target toward the decision heading; HOLD and frozen states keep both."""
    if decision["action"] == planner.HOLD or decision["frozen_yaw"] or decision["state"] == ABORT:
        return target, yaw_target
    heading = decision["heading"]
    step = max(-yaw_rate * dt, min(yaw_rate * dt, planner.wrap(heading - yaw_target)))
    new_yaw = planner.wrap(yaw_target + step)
    if decision["action"] != planner.TRANSLATE:
        return target, new_yaw
    end = decision["end"]
    remaining = math.dist(target, end)
    if math.dist(target, goal) <= min(remaining, speed * dt) + 1e-9:
        return tuple(goal), new_yaw  # the goal lies on the remaining step: stop exactly there
    move = min(speed * dt, remaining)
    if remaining <= 0:
        return target, new_yaw
    return (target[0] + (end[0] - target[0]) * move / remaining, target[1] + (end[1] - target[1]) * move / remaining), new_yaw
