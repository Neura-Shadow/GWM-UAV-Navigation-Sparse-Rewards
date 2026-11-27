"""P6 coverage report (M4, contract §6.7): aggregate `p6/p6-evaluation.json` files into coverage and
research metrics. Descriptive only (plan §8): one run per row supports existence claims, not rates."""
import argparse
import json
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def load(paths):
    out = []
    for path in paths:
        path = Path(path)
        target = path / "p6" / "p6-evaluation.json" if path.is_dir() else path
        out.append(json.loads(target.read_text(encoding="utf-8")))
    return out


def summary(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return {"n": len(values), "min": min(values), "median": statistics.median(values), "max": max(values)}


def report(evaluations):
    from injection import specs
    faults = specs.faults()
    rows = {}
    for e in evaluations:
        rows.setdefault(e.get("row"), []).append(e.get("status"))
    required = {f: v["classes"] for f, v in faults.items() if set(v["classes"]) & {"R-A", "R-B", "R-S", "R-C"}}
    evaluated_faults = {e.get("fault") for e in evaluations if e.get("status") in ("passed", "characterized")}
    timing = [e.get("timing") or {} for e in evaluations]
    gated = [e for e in evaluations if not str(e.get("row", "")).startswith("X-")]
    return {
        "contract": "p6-coverage-v1",
        "runs": len(evaluations),
        "rows": rows,
        "passed_rows": sorted(r for r, s in rows.items() if s and all(x == "passed" for x in s)),
        "required_faults_without_a_passing_run": sorted(f for f in required if f not in evaluated_faults),
        "detection_latency_sim_s": summary([t.get("detection_latency_sim_s") for t in timing]),
        "fault_to_safe_action_sim_s": summary([t.get("fault_to_safe_action_sim_s") for t in timing]),
        "degraded_sim_s": summary([t.get("degraded_sim_s") for t in timing]),
        "recoveries": sum(1 for t in timing if t.get("recovered")),
        "unsafe_setpoint_runs": sum(1 for e in evaluations if not (e.get("response", {}).get("UNSAFE") or {}).get("passed", True)),
        "gated_runs_failed": sorted(e.get("row") for e in gated if e.get("status") != "passed"),
        "note": "existence demonstrations under declared conditions; no detection, false-alarm or reliability rate is supported",
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("evaluations", nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    out = report(load(args.evaluations))
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(out, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({k: out[k] for k in ("runs", "passed_rows", "gated_runs_failed")}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
