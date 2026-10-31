"""P5 mission: the accepted `P4Mission` with the P5 supervisor installed at TRANSIT (contract §2, §5).

Everything else is the unchanged P4 mission, imported and never edited: the P2 state machine and
checks, MAP_SCAN, the TRANSIT phase logic, the `p4_decision` events, the revision 5 abort landing and
GOAL_HOVER. At MAP_SCAN -> TRANSIT, P4 creates its `Supervisor`; P5 replaces it, at the same
simulation time and before its first step, with the `PolicySupervisor` over the frozen ranker.
Each TRANSIT supervisor step appends one policy-ledger record for the node to persist.
"""
import time

from gwm_obstacle_avoidance.mission import P4Mission

from . import candidate_ranker, policy_ledger, safety_shield


class LedgerSupervisor(safety_shield.PolicySupervisor):
    """The P5 supervisor, unchanged, keeping its last output for the ledger."""

    def step(self, inputs):
        self.last_output = None
        self.last_output = super().step(inputs)
        return self.last_output


class P5Mission(P4Mission):
    def __init__(self, p2_config, p4_config, p5_config, policy, schema, cache, start_wall, flight,
                 ground_diagnostic=False, ground_duration=3.0, clock=time.perf_counter):
        super().__init__(p2_config, p4_config, cache, start_wall, flight, ground_diagnostic, ground_duration)
        inference = p5_config["inference"]
        self.policy = policy
        self.ranker = candidate_ranker.PolicyRanker(policy, schema, inference["failure_wall_s"], clock)
        self.failures_to_abort = inference["consecutive_failures_to_abort"]
        self.ledger = []  # records awaiting persistence by the node
        self.ledger_ticks = 0
        self.policy_stats = {"invocations": 0, "failures": {}, "interventions": {}, "disagreements": 0,
                             "inference_s": []}

    def transition(self, next_state, sim, wall, **evidence):
        if next_state == "TRANSIT":
            self.supervisor = LedgerSupervisor(self.p4, sim, self.ranker, self.failures_to_abort)
        return super().transition(next_state, sim, wall, **evidence)

    def _transit(self, sim, sample, dt):
        try:
            super()._transit(sim, sample, dt)
        finally:
            out = getattr(self.supervisor, "last_output", None)
            if out is not None:
                self.supervisor.last_output = None
                self._record(sim, out)

    def _record(self, sim, out):
        self.ledger_ticks += 1
        record = policy_ledger.entry(self.ledger_ticks, sim, out)
        self.ledger.append(record)
        if record["invoked"]:
            s = self.policy_stats
            s["invocations"] += 1
            s["inference_s"].append(record["inference_s"])
            if record["failure"] is not None:
                s["failures"][record["failure"]] = s["failures"].get(record["failure"], 0) + 1
            elif record["choice_slot"] != record["p4_slot"]:
                s["disagreements"] += 1
            if record["intervention"] is not None:
                kind = record["intervention"].split(":")[0]
                s["interventions"][kind] = s["interventions"].get(kind, 0) + 1

    def policy_summary(self):
        """Self-reported policy counters for the controller result; the evaluator recomputes them from the ledger."""
        s = self.policy_stats
        times = sorted(s["inference_s"])

        def nearest_rank(q):
            return times[min(len(times) - 1, int(q * len(times)))] if times else None
        return {"policy_sha256": self.policy.sha256 if self.policy else None, "ledger_records": self.ledger_ticks,
                "invocations": s["invocations"], "failures": dict(s["failures"]), "disagreements": s["disagreements"],
                "interventions": dict(s["interventions"]),
                "inference_s": {"p50": nearest_rank(0.50), "p95": nearest_rank(0.95), "p99": nearest_rank(0.99),
                                "max": times[-1] if times else None}}
