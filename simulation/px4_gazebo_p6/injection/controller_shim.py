"""P6 controller shim (CS, decision D2): the accepted `p5_control` with declared wrappers installed.

The entry point imports the accepted modules, installs wrappers on their classes before `main`, then
hands the remaining arguments to the unchanged `gwm_policy_control.control_node.main`. No accepted
file is edited and no publisher is added.

* `pass-through`: every wrapper calls the original with the same arguments and returns its result
  unchanged (proven bit for bit on recorded inputs, plan §12).
* `active`: exactly the declared fault, from the declared trigger. The trigger is evaluated on the
  mission's own decision events before the node persists them. Every faulted item goes to the shim's
  fault ledger.

Wrapped points (only those the declared fault needs, plus `tick` for the trigger):

* `P5Mission.tick` — trigger watch, delayed-snapshot release, the F6.03 tick delay;
* `P5Mission.on_snapshot` / `on_adapter_health` — map and health input faults (F4, F8.03, F8.06);
* `FrozenPolicy.scores` and `candidate_ranker.policy_choice` — policy faults (F3, F7.02, F8.01);
* `StateCache.update` and `P5Mission.ack` — app-side state and ACK faults (F2, F5).
"""
import argparse
import sys
import time

from . import declaration as declaration_module
from .fault_ledger import FaultLedger
from .policy_faults import corrupt_features, corrupt_scores, outside_mask, plan_for, worst_allowed
from .schedule import Trigger
from .state_faults import AckInjector, MapInjector, StateInjector

POLICY = {"F3.04", "F3.05", "F3.06", "F3.07", "F3.08", "F3.09", "F3.10", "F3.11", "F3.12", "F3.13", "F7.02", "F8.01"}
CHOICE = {"F3.06", "F3.13"}
MAP = {"F4.01", "F4.02", "F4.03", "F4.04", "F4.06", "F8.03", "F8.06"}
STATE = {"F2.01", "F2.02", "F2.03", "F2.04", "F2.05", "F2.06", "F2.07", "F2.08", "F5.05", "F5.07", "F5.10"}
ACK = {"F5.03", "F5.04", "F5.06"}
TICK_DELAY = {"F6.03"}
SHIM_MECHANISMS = ("CS", "MR")  # map/health input faults live in the shim too (D1a)
_MISSING = object()


def shim_mode(declaration):
    modes = {declaration_module.uses(declaration, mechanism) for mechanism in SHIM_MECHANISMS} - {"off"}
    return modes.pop() if modes else "off"


class Shim:
    def __init__(self, declaration, ledger=None, sleep=time.sleep, boxes=()):
        self.d = declaration
        self.mode = shim_mode(declaration)
        self.fault = declaration["fault"]
        self.p = declaration["parameters"]
        self.ledger, self.sleep = ledger, sleep
        self.trigger = Trigger(declaration["trigger"], boxes) if declaration.get("trigger") else None
        self.now = None
        self.originals = []
        self.counts = {"ticks": 0, "invocations": 0, "faulted_invocations": 0}
        self.plan = plan_for(self.fault, self.p) if self.fault in POLICY else None
        self.tick_delays = 1 if self.fault in TICK_DELAY else 0
        f, m, p = self.fault, self.mode, self.p
        self.map = MapInjector(f, p, m) if f in MAP else MapInjector(f, {}, "pass-through")
        self.state = StateInjector(f, p, m) if f in STATE else None
        self.acks = AckInjector(f, p, m) if f in ACK else None
        self.injectors = [i for i in (self.map, self.state, self.acks) if i is not None]

    # ---- trigger ------------------------------------------------------------------------------
    def fired(self):
        return self.trigger is not None and self.trigger.fired_sim is not None

    def active(self):
        return self.mode == "active" and self.fired()

    def _watch(self, mission, events):
        if self.trigger is None or self.fired():
            return
        if self.trigger.origin_enu is None and getattr(mission, "origin", None) is not None:
            self.trigger.origin_enu = mission.origin_enu
        for event in events:
            fired = self.trigger.feed(event)
            if fired is not None:
                for injector in self.injectors:
                    injector.activate(fired)
                self._log("activated", fired, trigger=self.trigger.name)
                return

    # ---- ledger -------------------------------------------------------------------------------
    def _log(self, kind, sim, **fields):
        if self.ledger is not None and self.mode == "active":
            self.ledger.record(kind, sim, **fields)

    def _drain(self):
        for injector in self.injectors:
            while injector.affected:
                key, sim, what = injector.affected.pop(0)
                self._log("affected", sim, item=key, what=what)

    # ---- install ------------------------------------------------------------------------------
    def _patch(self, owner, name, wrapper):
        self.originals.append((owner, name, owner.__dict__.get(name, _MISSING)))
        setattr(owner, name, wrapper)

    def install(self):
        from gwm_policy_control import candidate_ranker
        from gwm_policy_control.mission import P5Mission
        from gwm_policy_control.policy_runtime import FrozenPolicy
        from gwm_px4_control.timing import StateCache
        shim = self
        tick, on_snapshot, on_health = P5Mission.tick, P5Mission.on_snapshot, P5Mission.on_adapter_health
        scores, choice, update, ack = FrozenPolicy.scores, candidate_ranker.policy_choice, StateCache.update, P5Mission.ack

        def tick_wrapper(mission, sim, wall, graph_ok):
            shim.now = sim
            shim.counts["ticks"] += 1
            for snapshot in shim.map.due(sim):
                on_snapshot(mission, snapshot)
            if shim.tick_delays and shim.active():
                shim.tick_delays -= 1
                shim._log("affected", sim, item="mission_tick", what="delay", seconds=shim.p["mission_tick_delay_s"])
                shim.sleep(shim.p["mission_tick_delay_s"])
            before = len(mission.events)
            try:
                return tick(mission, sim, wall, graph_ok)
            finally:
                shim._watch(mission, mission.events[before:])
                shim._drain()

        def snapshot_wrapper(mission, snapshot):
            now = shim.now if shim.now is not None else 0.0  # before the first tick nothing is active
            for item in shim.map.snapshot(snapshot, now):
                on_snapshot(mission, item)

        def health_wrapper(mission, record):
            for item in shim.map.health(record, shim.now if shim.now is not None else 0.0):
                on_health(mission, item)

        def scores_wrapper(policy, features):
            if not shim.active():
                return scores(policy, features)
            shim.counts["invocations"] += 1
            if not shim.plan.faulted(shim.now - shim.trigger.fired_sim):
                return scores(policy, features)
            shim.counts["faulted_invocations"] += 1
            f = shim.fault
            shim._log("affected", shim.now, item="policy_invocation", what=f, index=shim.counts["invocations"])
            if f in ("F3.07", "F3.08", "F8.01", "F3.09"):
                shim.sleep(shim.p["sleep_s"])  # no default: every delay comes from the frozen declaration
            elif f == "F3.10":
                shim.sleep(shim.p["block_s"])
            elif f in ("F3.11", "F7.02"):
                raise RuntimeError("p6_injected_exception")
            elif f == "F3.12":
                return scores(policy, corrupt_features(features))
            return corrupt_scores(f, scores(policy, features))

        def choice_wrapper(values, allowed, rows_by_slot):
            if not shim.active() or shim.fault not in CHOICE:
                return choice(values, allowed, rows_by_slot)
            if shim.fault == "F3.13":
                shim._log("affected", shim.now, item="policy_choice", what=shim.fault)
                return worst_allowed(values, allowed, rows_by_slot)
            forbidden = outside_mask(allowed)
            if forbidden is None:  # every slot allowed on this tick: nothing to inject, the accepted choice stands
                shim._log("affected", shim.now, item="policy_choice", what="F3.06_not_injectable")
                return choice(values, allowed, rows_by_slot)
            shim._log("affected", shim.now, item="policy_choice", what=shim.fault)
            return forbidden

        def update_wrapper(cache, name, data, wall):
            if shim.state is None:
                return update(cache, name, data, wall)
            fresh = False
            for item in shim.state.state(name, data, cache.sim if cache.sim is not None else shim.now):
                fresh = update(cache, name, item, wall) or fresh
            return fresh

        def ack_wrapper(mission, data, sim, wall):
            if shim.acks is None:
                return ack(mission, data, sim, wall)
            for item in shim.acks.ack(data, sim):
                ack(mission, item, sim, wall)

        self._patch(P5Mission, "tick", tick_wrapper)
        if self.fault in MAP or self.mode == "pass-through":
            self._patch(P5Mission, "on_snapshot", snapshot_wrapper)
            self._patch(P5Mission, "on_adapter_health", health_wrapper)
        if self.fault in POLICY or self.mode == "pass-through":
            self._patch(FrozenPolicy, "scores", scores_wrapper)
            self._patch(candidate_ranker, "policy_choice", choice_wrapper)
        if self.fault in STATE or self.mode == "pass-through":
            self._patch(StateCache, "update", update_wrapper)
        if self.fault in ACK or self.mode == "pass-through":
            self._patch(P5Mission, "ack", ack_wrapper)
        if self.ledger is not None:
            self.ledger.record("declared", None, declaration=self.d)
            self.ledger.record("armed", None, mode=self.mode, wrapped=[o.__name__ + "." + n for o, n, _ in self.originals])
        return self

    def uninstall(self):
        while self.originals:
            owner, name, original = self.originals.pop()
            if original is _MISSING:
                delattr(owner, name)
            else:
                setattr(owner, name, original)
        if self.ledger is not None:
            self.ledger.record("finished", self.now, mode=self.mode, counts=self.counts)


def main(argv=None):
    """`controller_shim --p6-declaration D --p6-ledger L -- <unchanged p5_control arguments>`."""
    argv = list(sys.argv[1:] if argv is None else argv)
    split = argv.index("--") if "--" in argv else len(argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--p6-declaration", required=True)
    parser.add_argument("--p6-ledger", required=True)
    args = parser.parse_args(argv[:split])
    declared = declaration_module.load(args.p6_declaration)
    if shim_mode(declared) == "off":
        raise SystemExit("the declaration does not use the controller shim; run the accepted p5_control instead")
    ledger = FaultLedger(args.p6_ledger, declared["row"], declared["fault"], "controller_shim")
    shim = Shim(declared, ledger).install()
    from gwm_policy_control import control_node
    try:
        return control_node.main(argv[split + 1:])
    finally:
        shim.uninstall()
        ledger.close()


if __name__ == "__main__":
    raise SystemExit(main())
