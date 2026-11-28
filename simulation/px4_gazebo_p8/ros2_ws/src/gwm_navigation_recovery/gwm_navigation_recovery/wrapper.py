"""P8 recovery wrapper above the frozen supervisor (contract `p8-navigation-recovery-v1` §4; decision P8-D2).

`RecoverySupervisor` wraps the frozen P4 `Supervisor`, or the P5 `LedgerSupervisor`, unchanged:
* each tick it calls the frozen `step`, then lets the `RecoveryManager` replace a HOLD (S4);
* for P8-P5R it also runs the R4 guard, which swaps in the frozen `P4Ranker` for the guard period;
* every other attribute (`state`, `transitions`, `hover_since`, `scanned`, `last_output`, …) is read and written
  on the frozen supervisor itself, so the frozen P4 and P5 missions see it exactly as before.
"""
from gwm_obstacle_avoidance.supervisor import HOVER

from .recovery import LEDGER

_OWN = ("inner", "manager", "guard", "saved_ranker", "p4_ranker_factory")


class RecoverySupervisor:
    def __init__(self, inner, manager, guard=None, p4_ranker_factory=None):
        object.__setattr__(self, "inner", inner)
        object.__setattr__(self, "manager", manager)
        object.__setattr__(self, "guard", guard)
        object.__setattr__(self, "saved_ranker", None)
        object.__setattr__(self, "p4_ranker_factory", p4_ranker_factory)

    def __getattr__(self, name):
        return getattr(object.__getattribute__(self, "inner"), name)

    def __setattr__(self, name, value):
        if name in _OWN:
            object.__setattr__(self, name, value)
        else:
            setattr(self.inner, name, value)

    def hover(self):
        """(hover_since, entry reason) of the frozen supervisor's current HOVER, or None."""
        inner = self.inner
        if inner.state != HOVER or inner.hover_since is None:
            return None
        entry = next((t for t in reversed(inner.transitions) if t["to"] == HOVER), None)
        return (inner.hover_since, entry["reason"] if entry else None)

    def step(self, inputs):
        out = self.inner.step(inputs)
        if self.guard is not None:
            self._guard(inputs, out)
        return self.manager.step(inputs, out, self.hover())

    def _guard(self, inputs, out):
        event = self.guard.observe(inputs["sim"], out, inputs["vehicle"], inputs["goal"])
        if event == "start":
            self.saved_ranker = self.inner.ranker
            self.inner.ranker = self.p4_ranker_factory()
        elif event == "end" and self.saved_ranker is not None:
            self.inner.ranker, self.saved_ranker = self.saved_ranker, None
        if event in ("start", "end"):
            self.manager.records.append({"ledger": LEDGER, "kind": "r4_guard_" + event, "sim_s": inputs["sim"],
                                         "vehicle": list(inputs["vehicle"]), "goal": list(inputs["goal"]),
                                         "window_s": self.guard.window, "threshold": self.guard.k})
