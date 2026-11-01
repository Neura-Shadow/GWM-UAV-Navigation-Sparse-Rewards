"""Matched timing of the gated quantity for CPU-A (production) vs CPU-D (vectorized polar profile), with
non-intrusive scheduler evidence. Diagnostics only; the production module file is never modified.

Both variants run through the unchanged production `PolicyRanker.choose` (its own `inference_s` clock); only
the module attribute `observation_adapter.polar_profile` is swapped in this process. Calls come in pairs on
the same tick, in alternating order (A,D then D,A), each in its own timer slot, so tick content, order and
time-of-run are matched.

Per call: `inference_s`; thread CPU time; run-queue wait and timeslices from /proc/thread-self/schedstat
(time the thread was runnable but not running: the guest-scheduler preemption signature); voluntary and
involuntary context switches; Linux CPU before/after; guest runnable tasks (/proc/stat procs_running); the
polar-profile and `FrozenPolicy.scores` (validation + forward) durations; and, when paced, the wake-up lateness
against an absolute 20 Hz schedule (a missed slot is lateness >= one period).
Usage: recovery_bench.py --label NAME [--pause 0.05] [--duration 120] [--trials 5] [--affinity 4]
"""
import argparse
import ctypes
import json
import os
import resource
import time

import numpy as np

import common
import optimized

libc = ctypes.CDLL("libc.so.6", use_errno=True)


def schedstat():
    with open("/proc/thread-self/schedstat") as f:
        run, wait, slices = f.read().split()
    return int(run), int(wait), int(slices)


def procs_running():
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("procs_running"):
                return int(line.split()[1])
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True)
    parser.add_argument("--pause", type=float, default=0.05, help="timer period, s (0 = back to back)")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--affinity", default=None)
    args = parser.parse_args()

    from gwm_policy_control import observation_adapter
    from gwm_policy_control.candidate_ranker import PolicyRanker

    if args.affinity:
        os.sched_setaffinity(0, {int(c) for c in args.affinity.split(",")})
    policy, schema = common.load_policy()
    ticks = common.reconstruct_ticks(120)
    production_polar = observation_adapter.polar_profile
    timing = {"polar": 0, "scores": 0}
    impl = {"A": production_polar, "D": optimized.polar_profile}
    current = ["A"]

    def polar(*a, **k):
        t = time.perf_counter_ns()
        out = impl[current[0]](*a, **k)
        timing["polar"] = time.perf_counter_ns() - t
        return out
    observation_adapter.polar_profile = polar
    original_scores = policy.scores

    def scores(features):
        t = time.perf_counter_ns()
        out = original_scores(features)
        timing["scores"] = time.perf_counter_ns() - t
        return out
    policy.scores = scores  # instance attribute in this process only
    ranker = PolicyRanker(policy, schema, 0.020)
    reference = PolicyRanker(policy, schema, float("inf"))
    expected = {}
    for t in ticks:
        tick, _, state, rows, progress, ctx, grid, traversed, settings, _ = t
        per = {}
        for v in ("A", "D"):
            current[0] = v
            _, r = reference.choose(rows, progress, ctx, grid, traversed, state, settings)
            per[v] = (r["scores"].copy(), r["choice_slot"], r["features"].copy())
        assert np.array_equal(per["A"][0], per["D"][0]) and per["A"][1] == per["D"][1] and np.array_equal(per["A"][2], per["D"][2]), tick
        expected[tick] = per["A"]

    out = {"label": args.label, "pause_s": args.pause, "duration_s": args.duration, "affinity": sorted(os.sched_getaffinity(0)),
           "ticks": len(ticks), "trials": []}
    for trial in range(args.trials):
        calls = {"A": [], "D": []}
        lateness = []
        t0 = time.monotonic()
        end, k = t0 + args.duration, 0
        while time.monotonic() < end:
            pair = k // 2
            variant = ("A", "D")[(k + pair) % 2]  # A,D then D,A
            tick, sim, state, rows, progress, ctx, grid, traversed, settings, _ = ticks[pair % len(ticks)]
            if args.pause > 0:
                target = t0 + k * args.pause
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
                lateness.append((time.monotonic() - target) * 1e3)
            current[0] = variant
            run0, wait0, sl0 = schedstat()
            r0 = resource.getrusage(resource.RUSAGE_THREAD)
            running = procs_running()
            c0, cpu_a = time.thread_time_ns(), libc.sched_getcpu()
            _, record = ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
            cpu_ns, cpu_b = time.thread_time_ns() - c0, libc.sched_getcpu()
            r1 = resource.getrusage(resource.RUSAGE_THREAD)
            run1, wait1, sl1 = schedstat()
            failure = record["failure"]
            if failure not in (None, "timeout") or (failure is None and record["choice_slot"] != expected[tick][1]):
                raise SystemExit(f"decision changed at tick {tick} ({variant}): {failure}")
            calls[variant].append({"tick": tick, "ms": record["inference_s"] * 1e3, "cpu_ms": cpu_ns / 1e6,
                                   "runqueue_wait_ms": (wait1 - wait0) / 1e6, "timeslices": sl1 - sl0,
                                   "vcsw": r1.ru_nvcsw - r0.ru_nvcsw, "ivcsw": r1.ru_nivcsw - r0.ru_nivcsw,
                                   "cpu_before": cpu_a, "cpu_after": cpu_b, "procs_running": running,
                                   "polar_ms": timing["polar"] / 1e6, "scores_ms": timing["scores"] / 1e6,
                                   "timeout": failure == "timeout"})
            k += 1
        summary = {"trial": trial, "calls": k}
        for v, rows_ in calls.items():
            slow = [r for r in rows_ if r["ms"] > 3.0]
            summary[v] = {
                "inference_ms": common.quantiles([r["ms"] for r in rows_]),
                "cpu_ms": common.quantiles([r["cpu_ms"] for r in rows_]),
                "polar_ms": common.quantiles([r["polar_ms"] for r in rows_]),
                "scores_ms": common.quantiles([r["scores_ms"] for r in rows_]),
                "rest_ms": common.quantiles([r["ms"] - r["polar_ms"] - r["scores_ms"] for r in rows_]),
                "over_5ms": sum(r["ms"] > 5 for r in rows_), "over_10ms": sum(r["ms"] > 10 for r in rows_),
                "timeouts_20ms": sum(r["timeout"] for r in rows_),
                "calls_with_runqueue_wait": sum(r["runqueue_wait_ms"] > 0 for r in rows_),
                "calls_with_ivcsw": sum(r["ivcsw"] > 0 for r in rows_),
                "migrated": sum(r["cpu_before"] != r["cpu_after"] for r in rows_),
                "slow_calls_over_3ms": len(slow),
                "slow_with_runqueue_wait": sum(r["runqueue_wait_ms"] > 0.1 for r in slow),
                "slow_charged_as_cpu_no_wait": sum(r["runqueue_wait_ms"] <= 0.1 and r["cpu_ms"] >= 0.9 * r["ms"] for r in slow),
                "slowest": sorted(rows_, key=lambda r: -r["ms"])[:6],
                "procs_running": common.quantiles([r["procs_running"] for r in rows_ if r["procs_running"] is not None])}
        if lateness:
            summary["wakeup_lateness_ms"] = common.quantiles(lateness)
            summary["missed_slots"] = sum(x >= args.pause * 1e3 for x in lateness)
            summary["late_over_5ms"] = sum(x > 5 for x in lateness)
        summary["max_rss_kb"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        out["trials"].append(summary)
        brief = {v: {q: round(summary[v]["inference_ms"][q], 3) for q in ("p50", "p95", "p99", "max")} for v in ("A", "D")}
        print(json.dumps({"label": args.label, "trial": trial, "calls": k, **brief,
                          "over10": {v: summary[v]["over_10ms"] for v in ("A", "D")},
                          "late_max": round(summary.get("wakeup_lateness_ms", {}).get("max", 0), 2)}), flush=True)
    common.RESULTS.mkdir(exist_ok=True)
    path = common.RESULTS / f"recovery-{args.label}.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)


if __name__ == "__main__":
    main()
