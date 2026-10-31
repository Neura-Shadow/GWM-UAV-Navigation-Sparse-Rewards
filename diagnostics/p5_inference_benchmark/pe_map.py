"""Does a Linux CPU number inside WSL2 identify a host performance or efficiency core? (diagnostics only)

WSL2 exposes 24 vCPUs as 12 cores x 2 threads, which is not the host's 8 P-cores (16 threads) + 8 E-cores
topology. If a vCPU were bound to one host logical processor, a fixed single-thread workload pinned to it
would run at a stable, clustered speed (P-cores ~1.3-1.5x E-cores at this CPU's turbo limits). If vCPUs float
across host processors, the speed of a given vCPU changes from round to round. Pins only this process.
Usage: pe_map.py [rounds] [seconds_per_sample]
"""
import json
import os
import sys
import time

import common

rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
window = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
cpus = sorted(os.sched_getaffinity(0))


def work(seconds):
    """Fixed integer workload; returns iterations per second."""
    n, end = 0, time.perf_counter() + seconds
    while time.perf_counter() < end:
        x = 0
        for k in range(2000):
            x += k * k
        n += 1
    return n / seconds


speeds = {c: [] for c in cpus}
for r in range(rounds):
    for c in cpus:
        os.sched_setaffinity(0, {c})
        work(0.03)  # settle on the CPU
        speeds[c].append(work(window))
    time.sleep(1.0)
os.sched_setaffinity(0, set(cpus))
fastest = max(max(v) for v in speeds.values())
table = {c: {"relative_speed_per_round": [round(s / fastest, 3) for s in v],
             "spread": round((max(v) - min(v)) / max(v), 3)} for c, v in speeds.items()}
spreads = sorted(t["spread"] for t in table.values())
means = sorted(round(sum(v) / len(v) / fastest, 3) for v in speeds.values())
out = {"rounds": rounds, "seconds_per_sample": window, "per_cpu": table,
       "median_round_to_round_spread": spreads[len(spreads) // 2], "max_spread": spreads[-1],
       "mean_relative_speeds_sorted": means}
common.RESULTS.mkdir(exist_ok=True)
(common.RESULTS / "pe-map.json").write_text(json.dumps(out, indent=1))
print(json.dumps({k: out[k] for k in ("median_round_to_round_spread", "max_spread", "mean_relative_speeds_sorted")}))
for c in cpus:
    print(c, table[c]["relative_speed_per_round"])
