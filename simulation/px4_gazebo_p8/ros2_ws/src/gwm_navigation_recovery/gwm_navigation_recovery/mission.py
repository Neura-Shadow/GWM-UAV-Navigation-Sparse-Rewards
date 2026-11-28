"""P8 missions: the accepted P4 and P5 missions with the recovery wrapper installed at TRANSIT.

Everything is the unchanged accepted mission, imported and never edited: the P2 state machine and checks,
MAP_SCAN, TRANSIT, the `p4_decision` events, the abort landing and GOAL_HOVER.
* At MAP_SCAN -> TRANSIT the accepted mission creates its supervisor, as before. P8 then wraps it in
  `RecoverySupervisor` at the same simulation time, before its first step.
* The recovery records wait in `recovery_ledger` (the manager's `records`) until the node persists them.
"""
from gwm_obstacle_avoidance.mission import P4Mission
from gwm_policy_control.candidate_ranker import P4Ranker
from gwm_policy_control.mission import P5Mission

from .guard import YawOscillationGuard
from .recovery import RecoveryManager
from .wrapper import RecoverySupervisor


class _Recovery:
    def _install(self, guard):
        if not isinstance(self.supervisor, RecoverySupervisor):
            manager = RecoveryManager(self.p4, self.recovery_params)
            self.supervisor = RecoverySupervisor(self.supervisor, manager,
                                                 YawOscillationGuard(self.recovery_params) if guard else None,
                                                 P4Ranker if guard else None)

    @property
    def recovery_ledger(self):
        return self.supervisor.manager.records if isinstance(self.supervisor, RecoverySupervisor) else []


class P8P4Mission(_Recovery, P4Mission):
    def __init__(self, p2_config, p4_config, recovery_params, cache, start_wall, flight, ground_diagnostic=False,
                 ground_duration=3.0):
        super().__init__(p2_config, p4_config, cache, start_wall, flight, ground_diagnostic, ground_duration)
        self.recovery_params = recovery_params

    def transition(self, next_state, sim, wall, **evidence):
        result = super().transition(next_state, sim, wall, **evidence)
        if next_state == "TRANSIT":
            self._install(guard=False)
        return result


class P8P5Mission(_Recovery, P5Mission):
    def __init__(self, p2_config, p4_config, p5_config, policy, schema, recovery_params, cache, start_wall, flight,
                 ground_diagnostic=False, ground_duration=3.0, **kwargs):
        super().__init__(p2_config, p4_config, p5_config, policy, schema, cache, start_wall, flight, ground_diagnostic,
                         ground_duration, **kwargs)
        self.recovery_params = recovery_params

    def transition(self, next_state, sim, wall, **evidence):
        result = super().transition(next_state, sim, wall, **evidence)
        if next_state == "TRANSIT":
            self._install(guard=True)
        return result
