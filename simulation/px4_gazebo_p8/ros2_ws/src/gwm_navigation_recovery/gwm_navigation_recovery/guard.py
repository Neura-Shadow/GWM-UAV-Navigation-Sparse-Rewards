"""P8 R4: the P5 yaw-oscillation guard (contract `p8-navigation-recovery-v1` §4; `p8_recovery.json` `r4`).

Pure. It watches the P5 supervisor's outputs, and it fires when both hold:
* within the last `window_s` of TRANSIT there were at least `yaw_to_l4_at_least` `yaw_to_l4` interventions;
* the vehicle's net progress toward the goal over that window is below `net_progress_below_m`.

While it is active (`guard_period_s`), P8-P5R uses the frozen `P4Ranker` instead of the policy ranker. P5's model
and ranking are otherwise unchanged (contract B5). The threshold was derived so that it never fires on a recorded
P5 success.
"""
from collections import deque
import math

INTERVENTION = "yaw_to_l4"


class YawOscillationGuard:
    def __init__(self, params):
        r4 = params["r4"]
        self.window, self.k = r4["window_s"], r4["yaw_to_l4_at_least"]
        self.progress, self.period = r4["net_progress_below_m"], r4["guard_period_s"]
        self.yaws = deque()
        self.distances = deque()
        self.active_until = None

    def observe(self, sim, out, vehicle, goal):
        """Returns "start", "active", "end" or None for this tick."""
        policy = out.get("policy") or {}
        if str(policy.get("intervention") or "").split(":")[0] == INTERVENTION:
            self.yaws.append(sim)
        self.distances.append((sim, math.dist(vehicle, goal)))
        while self.yaws and self.yaws[0] < sim - self.window:
            self.yaws.popleft()
        while len(self.distances) > 1 and self.distances[1][0] <= sim - self.window:
            self.distances.popleft()
        if self.active_until is not None:
            if sim >= self.active_until:
                self.active_until = None
                self.yaws.clear()
                return "end"
            return "active"
        covered = self.distances and self.distances[0][0] <= sim - self.window + 1e-9
        net = self.distances[0][1] - self.distances[-1][1] if self.distances else 0.0
        if covered and len(self.yaws) >= self.k and net < self.progress:
            self.active_until = sim + self.period
            return "start"
        return None
