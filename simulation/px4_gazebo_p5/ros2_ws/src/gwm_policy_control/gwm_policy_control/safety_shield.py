"""P5 supervisor: P4 `Supervisor.step` replica that plans through `decision.plan_with_ranker` (contract §5).

Every P4 rule is unchanged and runs in P4's order: transit deadline, staleness HOVER/ABORT,
clearance ABORT, the transition table, hover timeout, SCAN bookkeeping. The single addition is the
policy-failure rule: a failing tick holds (target and yaw frozen, state unchanged) and the third
consecutive failure ABORTs with `policy_unavailable`. With `P4Ranker` the outputs equal P4's.
"""
from gwm_obstacle_avoidance import planner
from gwm_obstacle_avoidance.supervisor import (ABORT, AVOIDING, HOVER, NORMAL, OBSTACLE_DETECTED, RESUME,
                                               Supervisor)

from . import decision as decision_module


class PolicySupervisor(Supervisor):
    def __init__(self, config, start_sim, ranker, failures_to_abort=3):
        super().__init__(config, start_sim)
        self.ranker = ranker
        self.failures_to_abort = failures_to_abort
        self.consecutive_policy_failures = 0

    def step(self, inputs):
        sim, c = inputs["sim"], self.config
        out = {"state": None, "action": planner.HOLD, "heading": None, "end": None, "length": None,
               "frozen_yaw": False, "reason": None, "path": None, "plan": None, "stale": None, "policy": None}
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
        decision = decision_module.plan_with_ranker(grid, traversed, ctx, c["planner"], c["envelope"], self.max_age,
                                                    self.ranker, self.state)
        progress = decision["action"] in (planner.TRANSLATE, planner.YAW_TO)
        evidence = {"snapshot_acquisition_sim_s": inputs["snapshot"]["acquisition_sim_s"]}
        policy = decision["policy"]
        out["policy"] = policy
        if policy is not None and policy["failure"] is not None:
            self.consecutive_policy_failures += 1
            if self.consecutive_policy_failures >= self.failures_to_abort:
                self._go(ABORT, sim, "policy_unavailable", **evidence)
                out.update(state=ABORT, reason=self.abort_reason, frozen_yaw=True, path=path)
                return out
            out.update(state=self.state, action=planner.HOLD, reason=decision["reason"], frozen_yaw=True, path=path,
                       plan={k: decision[k] for k in ("candidates_valid", "candidates_progress", "best", "path_heading")})
            return out
        if policy is not None:
            self.consecutive_policy_failures = 0
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

