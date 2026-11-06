"""Controller-interpreter score equivalence on every exact flight tensor (diagnostics only).

For each invoked record of the Gate C S2 policy ledger (stored feature tensor, mask, recorded scores and
choice): the frozen `FrozenPolicy.scores` and the CPU-D `optimized.Forward` must reproduce the recorded scores
bit for bit, and the masked selection must equal the recorded choice. Also times the forward pass alone on the
two spike ticks. Usage (controller interpreter): equivalence_cpu.py
"""
import json
import time

import numpy as np

import common
import optimized


def main():
    policy, _ = common.load_policy()
    fast = optimized.Forward(policy)
    flight = common.ledger_ticks(None)
    result = {"records": len(flight), "frozen_bit_exact": 0, "optimized_bit_exact": 0, "choice_equal": 0}
    for tick, (F, allowed, recorded, choice) in flight.items():
        s = policy.scores(F)
        result["frozen_bit_exact"] += bool(np.array_equal(s, recorded))
        result["optimized_bit_exact"] += bool(np.array_equal(fast(policy, F), recorded))
        masked = np.where(allowed, s, -np.inf)
        result["choice_equal"] += int(np.argmax(masked)) == choice
    spikes = {}
    for tick in common.SPIKE_TICKS:
        F = flight[tick][0]
        times = []
        for _ in range(2000):
            t = time.perf_counter_ns()
            policy.scores(F)
            times.append((time.perf_counter_ns() - t) / 1e6)
        spikes[tick] = common.quantiles(times)
    result["spike_tick_forward_ms_2000_reps"] = spikes
    common.RESULTS.mkdir(exist_ok=True)
    (common.RESULTS / "equivalence-cpu.json").write_text(json.dumps(result, indent=1))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
