"""Read-only analysis of the scheduling-diagnostic flights. Usage: analyze.py [run_dir ...] (default: all p5diag runs)

Per run: policy-call latency quantiles and exceedances; every slow call (> 3 ms) attributed by its run-queue
wait (guest preemption) or as CPU-charged slowness; the guest's runnable tasks at each call and in the nearest
10 Hz host sample, with the processes that had runnable threads then; per-process CPU over the flight; steal
time; the flight outcome; and, where it runs, the standard P5 evaluator's safety and policy result.
"""
import bisect
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "p5_inference_benchmark"))
import common  # noqa: E402


def load(path):
    return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]


def analyze(run):
    out = {"run": run.name}
    lines = load(run / "p5diag/sched.jsonl")
    head, calls = lines[0], [c for c in lines[1:] if c.get("inference_ms") is not None]
    out["variant"], out["polar_profile"] = head["variant"], head["polar_profile"]
    ms = [c["inference_ms"] for c in calls]
    out["calls"] = len(calls)
    out["inference_ms"] = common.quantiles(ms)
    out["over_5ms"], out["over_10ms"] = sum(x > 5 for x in ms), sum(x > 10 for x in ms)
    out["failures"] = sum(c["failure"] is not None for c in calls)
    out["calls_with_runqueue_wait"] = sum(c["runqueue_wait_ms"] > 0 for c in calls)
    out["procs_running_at_call"] = common.quantiles([c["procs_running"] for c in calls])
    host = [h for h in load(run / "p5diag/host.jsonl") if "monotonic_ns" in h and "procs" in h]
    stamps = [h["monotonic_ns"] for h in host]
    slow = []
    for c in sorted(calls, key=lambda c: -c["inference_ms"]):
        if c["inference_ms"] <= 3.0:
            break
        i = min(max(bisect.bisect_left(stamps, c["monotonic_ns"]), 0), len(host) - 1) if host else None
        h = host[i] if host else {}
        runnable = sorted(((p[3], p[0]) for p in h.get("procs", {}).values() if p[3] > 0), reverse=True)[:6]
        slow.append({"inference_ms": round(c["inference_ms"], 3), "cpu_ms": round(c["cpu_ms"], 3),
                     "runqueue_wait_ms": round(c["runqueue_wait_ms"], 3), "ivcsw": c["ivcsw"], "state": c["state"],
                     "procs_running_at_call": c["procs_running"], "host_procs_running": h.get("procs_running"),
                     "host_sample_offset_ms": None if not host else round((h["monotonic_ns"] - c["monotonic_ns"]) / 1e6, 1),
                     "runnable_processes": runnable,
                     "class": "guest_preemption" if c["runqueue_wait_ms"] > 0.1 else
                              ("cpu_charged" if c["cpu_ms"] >= 0.9 * c["inference_ms"] else "unattributed")})
    out["slow_calls"] = slow
    out["slow_by_class"] = {k: sum(s["class"] == k for s in slow) for k in ("guest_preemption", "cpu_charged", "unattributed")}
    if host:
        pr = [h["procs_running"] for h in host]
        out["host_procs_running"] = common.quantiles(pr)
        out["host_samples_over_24_runnable"] = sum(x > 24 for x in pr)
        out["host_samples"] = len(host)
        j0, j1 = host[0]["cpu_jiffies"], host[-1]["cpu_jiffies"]
        out["steal_jiffies"] = j1[7] - j0[7] if len(j0) > 7 else None
        span = (host[-1]["monotonic_ns"] - host[0]["monotonic_ns"]) / 1e9
        seen = {}  # pid -> [name, first ticks, last ticks, first ns, last ns, max runnable]
        for h in host:
            for pid, (name, ticks, _threads, runnable) in h["procs"].items():
                s = seen.setdefault(pid, [name, ticks, ticks, h["monotonic_ns"], h["monotonic_ns"], 0])
                s[2], s[4], s[5] = ticks, h["monotonic_ns"], max(s[5], runnable)
        cpu, runnable = {}, {}
        for name, t0, t1, n0, n1, rmax in seen.values():
            if n1 > n0:
                cpu[name] = cpu.get(name, 0) + (t1 - t0) / 100 / ((n1 - n0) / 1e9)
            runnable[name] = max(runnable.get(name, 0), rmax)
        out["process_cpu_cores_while_alive"] = dict(sorted(((k, round(v, 2)) for k, v in cpu.items()), key=lambda kv: -kv[1])[:10])
        out["max_runnable_threads_per_process"] = dict(sorted(runnable.items(), key=lambda kv: -kv[1])[:8])
        out["total_namespace_cpu_cores_mean"] = round(sum(cpu.values()), 2)
    summary = json.loads((run / "summary.json").read_text())
    result = summary.get("controller_result") or {}
    p4 = result.get("p4") or {}
    out["status"], out["failure"] = summary.get("status"), summary.get("failure")
    out["outcome"] = result.get("p4_outcome")
    out["transitions"] = [t["to"] for t in p4.get("supervisor_transitions", [])]
    out["abort_reason"] = p4.get("abort_reason")
    try:
        sys.path.insert(0, str(common.P5 / "validation"))
        import p5_evaluate
        e = p5_evaluate.evaluate(run)
        out["evaluator"] = {"status": e.get("status"), "failures": e.get("failures"),
                            "outcome": (e.get("outcome") or {}).get("label"),
                            "safety": {k: (e.get("safety") or {}).get(k) for k in ("min_truth_clearance_m", "collisions", "tracking_max_m")},
                            "policy_inference_s": (e.get("policy") or {}).get("inference_s")}
    except Exception as exc:  # the evaluator is built for gate runs; report, do not fail the analysis
        out["evaluator"] = {"error": type(exc).__name__ + ": " + str(exc)[:200]}
    return out


if __name__ == "__main__":
    runs = [Path(a) for a in sys.argv[1:]] or sorted(common.RUNS.glob("*-p5diag-flight-*"))
    results = [analyze(r) for r in runs if (r / "p5diag/sched.jsonl").exists()]
    common.RESULTS.mkdir(exist_ok=True)
    (common.RESULTS / "sched-diag.json").write_text(json.dumps(results, indent=1))
    for r in results:
        brief = {k: r.get(k) for k in ("run", "variant", "status", "outcome", "transitions", "calls", "over_5ms", "over_10ms",
                                       "failures", "slow_by_class", "steal_jiffies", "host_samples_over_24_runnable")}
        print(json.dumps(brief))
        print("   inference_ms", {k: round(v, 3) for k, v in r["inference_ms"].items() if k != "n"},
              "| procs_running at call", r["procs_running_at_call"], "| host", r.get("host_procs_running"))
        print("   process CPU cores (while alive)", r.get("process_cpu_cores_while_alive"), "total", r.get("total_namespace_cpu_cores_mean"))
        print("   max runnable threads", r.get("max_runnable_threads_per_process"))
        for s in r["slow_calls"][:6]:
            print("   slow", json.dumps(s))
        print("   evaluator", json.dumps(r["evaluator"])[:600])
