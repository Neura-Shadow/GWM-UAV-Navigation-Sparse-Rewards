"""Read-only: decision-window guest load and the standard P5 evaluator result for one diagnostic run (in memory;
nothing is written to the run or to gate state). Usage: evaluate_one.py RUN_DIR"""
import json
from pathlib import Path
import sys

run = Path(sys.argv[1])
calls = [json.loads(x) for x in (run / "p5diag/sched.jsonl").read_text().splitlines()[1:]]
host = [h for h in (json.loads(x) for x in (run / "p5diag/host.jsonl").read_text().splitlines()) if "procs" in h]
c0, c1 = calls[0]["monotonic_ns"], calls[-1]["monotonic_ns"]
window = sorted(h["procs_running"] for h in host if c0 <= h["monotonic_ns"] <= c1)
print(json.dumps({"window_host_samples": len(window), "window_procs_running_max": window[-1],
                  "window_procs_running_p99": window[int(0.99 * len(window))], "window_samples_over_24": sum(x > 24 for x in window),
                  "calls_with_runqueue_wait": sum(c["runqueue_wait_ms"] > 0 for c in calls),
                  "max_runqueue_wait_ms": round(max(c["runqueue_wait_ms"] for c in calls), 3),
                  "calls_with_ivcsw": sum(c["ivcsw"] > 0 for c in calls)}))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation/px4_gazebo_p5/validation"))
import p5_evaluate  # noqa: E402
try:
    e = p5_evaluate.evaluate(run)
    policy = e.get("policy") or {}
    print("evaluator", json.dumps({
        "status": e.get("status"), "failures": e.get("failures"), "outcome": (e.get("outcome") or {}).get("label"),
        "safety": {k: (e.get("safety") or {}).get(k) for k in ("min_truth_clearance_m", "collisions", "tracking_max_m")},
        "policy": {k: policy.get(k) for k in ("invoked", "failure_ticks", "outside_mask", "fallback_decisions",
                                              "participation_fraction", "score_mismatches", "argmax_mismatches",
                                              "executed_mismatches", "inference_s")}}))
except Exception as exc:
    print("evaluator error", type(exc).__name__, str(exc)[:400])
