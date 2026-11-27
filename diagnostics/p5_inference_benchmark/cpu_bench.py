"""CPU benchmark of the P5 policy decision path (diagnostics only; no ROS graph, no simulator, nothing published).

Each trial cycles the reconstructed ticks (common.py) and alternates two kinds of call, with the chosen pacing
between every call:

* production: the unchanged `PolicyRanker.choose`, timed by its own clock (`inference_s`, the gated quantity);
* phased: the same steps in the same order, timed per phase:
    build_candidates  `observation_adapter.build` minus its polar profile (candidate features, scaling, mask)
    polar_profile     the context polar profile inside `build` (timed through a module-level wrapper)
    validate          mask == progress set, then the input shape/dtype/finite checks of `FrozenPolicy.scores`
    forward           the three dense layers and ReLUs (expression of `FrozenPolicy.scores`, or `--variant`)
    rank              `policy_choice` (masked argmax with the P4 tie order)
    select            the allowed-mask re-check and the slot lookup
  Every phased call's scores must equal the production scores bit for bit, and its choice must equal the
  production choice, or the trial aborts.

Per call it also records thread CPU time, the Linux CPU before and after, and the thread's voluntary and
involuntary context switches. Usage (controller interpreter):
  cpu_bench.py --label NAME --pause 0.05 --duration 60 [--trials 3] [--affinity 0,2] [--variant baseline|optimized]
"""
import argparse
import ctypes
import json
import os
import platform
import resource
import time

import numpy as np

import common

libc = ctypes.CDLL("libc.so.6", use_errno=True)


def cpu():
    return libc.sched_getcpu()


def switches():
    r = resource.getrusage(resource.RUSAGE_THREAD)
    return r.ru_nvcsw, r.ru_nivcsw


def forward_baseline(policy, x):
    w = policy._weights  # the frozen read-only weights; same expression as FrozenPolicy.scores
    h = np.maximum(x @ w["W1"] + w["b1"], 0.0)
    h = np.maximum(h @ w["W2"] + w["b2"], 0.0)
    return (h @ w["W3"] + w["b3"])[:, 0]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True)
    parser.add_argument("--pause", type=float, default=0.05, help="sleep between calls, s (0 = back to back)")
    parser.add_argument("--duration", type=float, default=60.0, help="seconds per trial")
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--affinity", default=None, help="comma-separated Linux CPUs (process-scoped, this run only)")
    parser.add_argument("--variant", choices=("baseline", "optimized"), default="baseline")
    parser.add_argument("--limit", type=int, default=120)
    args = parser.parse_args()

    from gwm_policy_control import observation_adapter
    from gwm_policy_control.candidate_ranker import PolicyRanker, policy_choice
    from gwm_policy_control.contracts import FEATURES, SLOTS
    from gwm_obstacle_avoidance import planner

    if args.affinity:
        os.sched_setaffinity(0, {int(c) for c in args.affinity.split(",")})
    policy, schema = common.load_policy()
    ticks = common.reconstruct_ticks(args.limit)
    ranker = PolicyRanker(policy, schema, 0.020)
    forward = forward_baseline
    build = observation_adapter.build
    original_polar = observation_adapter.polar_profile
    polar = original_polar
    if args.variant == "optimized":
        import optimized
        forward, polar = optimized.Forward(policy), optimized.polar_profile

    polar_time = [0.0]
    production_call = [False]

    def timed_polar(*a, **k):  # benchmark-process wrapper; the source module file is not modified
        t = time.perf_counter_ns()
        out = (original_polar if production_call[0] else polar)(*a, **k)
        polar_time[0] = time.perf_counter_ns() - t
        return out
    observation_adapter.polar_profile = timed_polar
    production_call[0] = True  # the reference pass and production calls always use the production profile

    # reference per tick: production scores and choice; the ledger's recorded choice must match too
    from gwm_policy_control.policy_ledger import decode_array
    reference, fidelity = {}, {"features_bit_exact": 0, "scores_bit_exact": 0, "max_feature_abs_diff": 0.0,
                               "max_score_abs_diff": 0.0, "choice_equal": 0}
    unbounded = PolicyRanker(policy, schema, float("inf"))  # reference decisions only; never times out
    for t in ticks:
        tick, _, state, rows, progress, ctx, grid, traversed, settings, recorded = t
        _, record = unbounded.choose(rows, progress, ctx, grid, traversed, state, settings)
        assert record["failure"] is None, tick
        flight_features, flight_scores = decode_array(recorded["features"]), np.array(recorded["scores"])
        fidelity["features_bit_exact"] += bool(np.array_equal(record["features"], flight_features))
        fidelity["scores_bit_exact"] += bool(np.array_equal(record["scores"], flight_scores))
        fidelity["choice_equal"] += record["choice_slot"] == recorded["choice_slot"]
        fidelity["max_feature_abs_diff"] = max(fidelity["max_feature_abs_diff"], float(np.abs(record["features"] - flight_features).max()))
        fidelity["max_score_abs_diff"] = max(fidelity["max_score_abs_diff"], float(np.abs(record["scores"] - flight_scores).max()))
        reference[tick] = (record["scores"].copy(), record["choice_slot"])
    print("reconstruction fidelity vs the flight ledger:", json.dumps(fidelity), flush=True)

    out = {"label": args.label, "variant": args.variant, "pause_s": args.pause, "duration_s": args.duration,
           "affinity": sorted(os.sched_getaffinity(0)), "python": platform.python_version(), "numpy": np.__version__,
           "ticks": len(ticks), "tick_ids": [t[0] for t in ticks], "reconstruction_fidelity": fidelity, "trials": []}
    for trial in range(args.trials):
        prod, phased = [], []
        cpu0, wall0 = time.process_time(), time.monotonic()
        end, i = time.monotonic() + args.duration, 0
        while time.monotonic() < end:
            tick, sim, state, rows, progress, ctx, grid, traversed, settings, _ = ticks[(i // 2) % len(ticks)]
            ref_scores, ref_choice = reference[tick]
            production_call[0] = i % 2 == 0
            sw0, c0, cpu_a = switches(), time.thread_time_ns(), cpu()
            if i % 2 == 0:
                w0 = time.perf_counter_ns()
                _, record = ranker.choose(rows, progress, ctx, grid, traversed, state, settings)
                wall = time.perf_counter_ns() - w0
                cpu_ns, cpu_b, sw1 = time.thread_time_ns() - c0, cpu(), switches()
                # a 20 ms runtime timeout is data (the production HOLD path); any other change aborts
                if record["failure"] not in (None, "timeout") or (record["failure"] is None and record["choice_slot"] != ref_choice):
                    raise SystemExit(f"production decision changed at tick {tick}: {record['failure']}")
                prod.append({"tick": tick, "inference_ms": record["inference_s"] * 1e3, "wall_ms": wall / 1e6,
                             "timeout": record["failure"] == "timeout",
                             "cpu_ms": cpu_ns / 1e6, "cpu_before": cpu_a, "cpu_after": cpu_b,
                             "vcsw": sw1[0] - sw0[0], "ivcsw": sw1[1] - sw0[1]})
            else:
                t0 = time.perf_counter_ns()
                features, allowed = build(rows, ctx, settings, grid, traversed, state, schema)
                t1 = time.perf_counter_ns()
                by_slot = {observation_adapter.slot_of(r, settings): r for r in rows}
                if {observation_adapter.slot_of(r, settings) for r in progress} != set(np.flatnonzero(allowed).tolist()):
                    raise SystemExit("mask_mismatch")
                x = np.asarray(features)
                if x.shape != (SLOTS, FEATURES) or x.dtype != np.float64 or not np.isfinite(x).all():
                    raise SystemExit("input_schema")
                t2 = time.perf_counter_ns()
                scores = forward(policy, x)
                t3 = time.perf_counter_ns()
                choice = policy_choice(scores, allowed, by_slot)
                t4 = time.perf_counter_ns()
                if not allowed[choice]:
                    raise SystemExit("selection_outside_mask")
                best = by_slot[choice]
                t5 = time.perf_counter_ns()
                cpu_ns, cpu_b, sw1 = time.thread_time_ns() - c0, cpu(), switches()
                if not np.array_equal(scores, ref_scores) or choice != ref_choice or best is None:
                    raise SystemExit(f"phased decision differs from production at tick {tick}")
                p = polar_time[0]
                phased.append({"tick": tick, "build_candidates_ms": (t1 - t0 - p) / 1e6, "polar_profile_ms": p / 1e6,
                               "validate_ms": (t2 - t1) / 1e6, "forward_ms": (t3 - t2) / 1e6, "rank_ms": (t4 - t3) / 1e6,
                               "select_ms": (t5 - t4) / 1e6, "total_ms": (t5 - t0) / 1e6, "cpu_ms": cpu_ns / 1e6,
                               "cpu_before": cpu_a, "cpu_after": cpu_b, "vcsw": sw1[0] - sw0[0], "ivcsw": sw1[1] - sw0[1]})
            i += 1
            if args.pause > 0:
                time.sleep(args.pause)
        elapsed = time.monotonic() - wall0
        usage = resource.getrusage(resource.RUSAGE_SELF)
        summary = {"trial": trial, "calls": i, "elapsed_s": elapsed,
                   "process_cpu_utilization": (time.process_time() - cpu0) / elapsed, "max_rss_kb": usage.ru_maxrss,
                   "production_inference_ms": common.quantiles([r["inference_ms"] for r in prod]),
                   "production_cpu_ms": common.quantiles([r["cpu_ms"] for r in prod]),
                   "phased": {k: common.quantiles([r[k] for r in phased]) for k in
                              ("build_candidates_ms", "polar_profile_ms", "validate_ms", "forward_ms", "rank_ms",
                               "select_ms", "total_ms", "cpu_ms")},
                   "production_over_5ms": sum(r["inference_ms"] > 5 for r in prod),
                   "production_over_10ms": sum(r["inference_ms"] > 10 for r in prod),
                   "production_timeouts_20ms": sum(r["timeout"] for r in prod),
                   "slowest_production": sorted(prod, key=lambda r: -r["inference_ms"])[:5],
                   "slowest_phased": sorted(phased, key=lambda r: -r["total_ms"])[:5],
                   "migrated_fraction": sum(r["cpu_before"] != r["cpu_after"] for r in prod + phased) / max(1, len(prod + phased)),
                   "ivcsw_calls": sum(r["ivcsw"] > 0 for r in prod + phased), "vcsw_calls": sum(r["vcsw"] > 0 for r in prod + phased),
                   "cpus_seen": sorted({r["cpu_before"] for r in prod + phased})}
        per_tick = {}
        for r in prod:
            per_tick.setdefault(r["tick"], []).append(r["inference_ms"])
        summary["per_tick_median_ms"] = common.quantiles([sorted(v)[len(v) // 2] for v in per_tick.values()])
        out["trials"].append(summary)
        print(json.dumps({"label": args.label, "trial": trial, "calls": i,
                          "production_ms": {k: round(v, 3) for k, v in summary["production_inference_ms"].items() if k != "n"},
                          "phased_total_ms": {k: round(v, 3) for k, v in summary["phased"]["total_ms"].items() if k != "n"},
                          "over_5ms": summary["production_over_5ms"], "over_10ms": summary["production_over_10ms"], "timeouts_20ms": summary["production_timeouts_20ms"],
                          "cpu_util": round(summary["process_cpu_utilization"], 3)}), flush=True)
    common.RESULTS.mkdir(exist_ok=True)
    path = common.RESULTS / f"cpu-{args.label}.json"
    path.write_text(json.dumps(out, indent=1))
    print(path)


if __name__ == "__main__":
    main()
