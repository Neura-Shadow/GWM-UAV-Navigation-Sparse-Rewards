"""Read-only: when did processes with many runnable threads run, relative to the policy-call window?

For each diagnostic run: the alive window of every process that ever had >= 4 runnable threads, and whether it
overlapped the window of policy calls (first to last call). Also the host samples with > 24 runnable tasks.
Usage: burst_windows.py [run_dir ...] (default: all p5diag runs)
"""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "p5_inference_benchmark"))
import common  # noqa: E402

for run in [Path(a) for a in sys.argv[1:]] or sorted(common.RUNS.glob("*-p5diag-flight-*")):
    calls = [json.loads(x) for x in (run / "p5diag/sched.jsonl").read_text().splitlines()[1:]]
    host = [json.loads(x) for x in (run / "p5diag/host.jsonl").read_text().splitlines()]
    host = [h for h in host if "procs" in h]
    t0 = host[0]["monotonic_ns"]
    c0, c1 = calls[0]["monotonic_ns"], calls[-1]["monotonic_ns"]
    windows = {}
    for h in host:
        for pid, (name, ticks, threads, runnable) in h["procs"].items():
            w = windows.setdefault(pid, {"name": name, "first_s": h["monotonic_ns"], "last_s": h["monotonic_ns"], "max_runnable": 0})
            w["last_s"], w["max_runnable"] = h["monotonic_ns"], max(w["max_runnable"], runnable)
    print(run.name, f"| policy calls {(c0 - t0) / 1e9:.1f}-{(c1 - t0) / 1e9:.1f} s after sampler start")
    for pid, w in sorted(windows.items(), key=lambda kv: kv[1]["first_s"]):
        if w["max_runnable"] >= 4:
            overlap = w["first_s"] <= c1 and w["last_s"] >= c0
            print(f"   {w['name']:<18} pid {pid:>6} alive {(w['first_s'] - t0) / 1e9:6.1f}-{(w['last_s'] - t0) / 1e9:6.1f} s"
                  f"  max runnable {w['max_runnable']:>2}  overlaps policy window: {overlap}")
    over = [((h["monotonic_ns"] - t0) / 1e9, h["procs_running"]) for h in host if h.get("procs_running", 0) > 24]
    print("   host samples > 24 runnable (s, n):", [(round(s, 1), n) for s, n in over])
