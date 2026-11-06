"""Print a compact table from the result files (diagnostics only). Usage: summarize.py [glob]"""
import json
import sys
from pathlib import Path

import common


def pooled(trials, key):
    """Pool per-trial quantiles conservatively: median of p50, max of p95/p99/max across trials."""
    rows = [t[key] if not isinstance(key, tuple) else t[key[0]][key[1]] for t in trials]
    rows = [r for r in rows if r]
    return {"p50": sorted(r["p50"] for r in rows)[len(rows) // 2], "p95": max(r["p95"] for r in rows),
            "p99": max(r["p99"] for r in rows), "max": max(r["max"] for r in rows), "n": sum(r["n"] for r in rows)}


pattern = sys.argv[1] if len(sys.argv) > 1 else "*.json"
for path in sorted(common.RESULTS.glob(pattern)):
    d = json.loads(path.read_text())
    if "trials" not in d:
        continue
    if path.name.startswith("cpu-"):
        prod = pooled(d["trials"], "production_inference_ms")
        phases = {k: pooled(d["trials"], ("phased", k)) for k in d["trials"][0]["phased"]}
        over5 = sum(t["production_over_5ms"] for t in d["trials"])
        over10 = sum(t["production_over_10ms"] for t in d["trials"])
        print(f"{path.stem:40s} prod n={prod['n']:6d} p50 {prod['p50']:.3f} p95 {prod['p95']:.3f} p99 {prod['p99']:.3f} "
              f"max {prod['max']:.3f}  >5 {over5} >10 {over10} timeout20 {sum(t.get('production_timeouts_20ms', 0) for t in d['trials'])}  cpu_util {[round(t['process_cpu_utilization'], 2) for t in d['trials']]}")
        print("    phases p50/p99/max: " + "  ".join(f"{k.replace('_ms', '')} {v['p50']:.3f}/{v['p99']:.3f}/{v['max']:.3f}"
                                                   for k, v in phases.items()))
        print(f"    per-trial max: {[round(t['production_inference_ms']['max'], 2) for t in d['trials']]}  "
              f"ivcsw calls {[t['ivcsw_calls'] for t in d['trials']]}  migrated {[round(t['migrated_fraction'], 4) for t in d['trials']]}  "
              f"cpus {d['trials'][0]['cpus_seen'][:8]}  rss_kb {d['trials'][-1]['max_rss_kb']}")
        worst = max((r for t in d["trials"] for r in t["slowest_production"]), key=lambda r: r["inference_ms"])
        print(f"    worst production call: {json.dumps(worst)}")
    else:
        fwd = pooled(d["trials"], "forward_only_ms")
        e2e = pooled(d["trials"], ("end_to_end", "total_ms"))
        parts = {k: pooled(d["trials"], ("end_to_end", k)) for k in d["trials"][0]["end_to_end"]}
        print(f"{path.stem:40s} forward n={fwd['n']} p50 {fwd['p50']:.3f} p95 {fwd['p95']:.3f} p99 {fwd['p99']:.3f} max {fwd['max']:.3f} | "
              f"e2e p50 {e2e['p50']:.3f} p95 {e2e['p95']:.3f} p99 {e2e['p99']:.3f} max {e2e['max']:.3f} "
              f">10 {sum(t['end_to_end_over_10ms'] for t in d['trials'])} mismatches {sum(t['choice_mismatches_vs_cpu'] for t in d['trials'])}")
        print("    e2e parts p50/p99/max: " + "  ".join(f"{k.replace('_ms', '')} {v['p50']:.3f}/{v['p99']:.3f}/{v['max']:.3f}"
                                                      for k, v in parts.items()))
        print(f"    cold forward ms {[round(x, 2) for x in d['cold_forward_ms']]}  context+upload {d['context_and_upload_s']:.2f} s")
