"""Read-only measurement of gate-D stall diagnostics (docs/v3_sim_p3_stall_attribution.md).

Per run, inside its fixed wall window: 1 s local RTF from the Gazebo-native /clock, the wall
duration of every 1.000 s sim interval (the land_detected 1 Hz family), controller
land_detected gaps, per-process CPU, and Gazebo thread states split by local RTF.
Never acceptance evidence.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import statistics


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def jsonl(path, inputs):
    inputs[str(path)] = sha(path)
    return [json.loads(line) for line in open(path) if line.strip()]


def quantile(values, p):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(p * (len(ordered) - 1))))] if ordered else None


def crossings(points):
    """Wall time at which sim time crosses each integer second (linear between samples)."""
    out = []
    for (w0, s0), (w1, s1) in zip(points, points[1:]):
        k = int(s0) + 1
        while k <= s1 and s1 > s0:
            out.append(w0 + (w1 - w0) * (k - s0) / (s1 - s0))
            k += 1
    return out


def measure(run, inputs):
    summary = json.load(open(run / "summary.json"))
    inputs[str(run / "summary.json")] = sha(run / "summary.json")
    begin, end = summary["diagnostic_window_monotonic_s"]
    clock = [(c["receipt_monotonic_ns"] / 1e9, c["sim_s"]) for c in jsonl(run / "source/clock.jsonl", inputs)]
    clock = [p for p in clock if begin <= p[0] <= end]

    def rtf_between(a, b):
        seg = [p for p in clock if a <= p[0] <= b]
        return (seg[-1][1] - seg[0][1]) / (seg[-1][0] - seg[0][0]) if len(seg) >= 2 and seg[-1][0] > seg[0][0] else None

    bins = [r for r in (rtf_between(t, t + 1.0) for t in [begin + i for i in range(int(end - begin))]) if r is not None]
    family = [b - a for a, b in zip(crossings(clock), crossings(clock)[1:])]
    out = {"run": run.name, "variant": summary.get("diagnostic_variant"), "status": summary["status"],
           "failure": summary.get("failure"), "window_wall_s": round(end - begin, 2),
           "renderer": (summary.get("renderer") or {}).get("strings"),
           "rtf_1s": {"n": len(bins), "mean": round(statistics.fmean(bins), 3), "p05": round(quantile(bins, .05), 3),
                      "min": round(min(bins), 3), "below_0.55": sum(r < 0.55 for r in bins)},
           "sim_second_wall_s": {"n": len(family), "p50": round(quantile(family, .5), 3),
                                 "p95": round(quantile(family, .95), 3), "max": round(max(family), 3),
                                 "over_1.8": sum(x > 1.8 for x in family), "over_2.0": sum(x > 2.0 for x in family)}}
    events = run / "ros-events.jsonl"
    if events.exists():
        land = [e for e in jsonl(events, inputs)
                if e.get("event") == "received" and e.get("topic_key") == "vehicle_land_detected"
                and begin <= e["monotonic_s"] <= end]
        gaps = [b["monotonic_s"] - a["monotonic_s"] for a, b in zip(land, land[1:])]
        out["controller_land_gap_wall_s"] = {"n": len(gaps), "max": round(max(gaps), 3) if gaps else None,
                                             "over_1.8": sum(g > 1.8 for g in gaps)}
    resources = [r for r in jsonl(run / "resources.jsonl", inputs) if begin - 1 <= r["monotonic_s"] <= end + 1]
    cores = collections.defaultdict(list)
    for a, b in zip(resources, resources[1:]):
        dt = b["monotonic_s"] - a["monotonic_s"]
        for name in b["processes"]:
            if name in a["processes"]:
                cores[name].append((b["processes"][name]["cpu_s"] - a["processes"][name]["cpu_s"]) / dt)
    out["cpu_cores_median"] = {n: round(statistics.median(v), 3) for n, v in sorted(cores.items())}
    out["cpu_cores_total_median"] = round(sum(out["cpu_cores_median"].values()), 3)
    samples = [s for s in jsonl(run / "diagnostic-gazebo-threads.jsonl", inputs) if begin <= s["monotonic_s"] <= end]
    split = {"low_rtf": collections.Counter(), "normal_rtf": collections.Counter()}
    dwchan = {"low_rtf": collections.Counter(), "normal_rtf": collections.Counter()}
    n_samples = {"low_rtf": 0, "normal_rtf": 0}
    busy = collections.Counter()
    previous = None
    for s in samples:
        rtf = rtf_between(s["monotonic_s"] - 1.0, s["monotonic_s"])
        if rtf is None:
            continue
        key = "low_rtf" if rtf < 0.55 else "normal_rtf"
        n_samples[key] += 1
        for tid, comm, state, wchan, ticks in s["threads"]:
            split[key][state] += 1
            if state == "D":
                dwchan[key][wchan] += 1
        if previous:
            before = {t[0]: (t[1], t[4]) for t in previous["threads"]}
            for tid, comm, state, wchan, ticks in s["threads"]:
                if tid in before:
                    busy[comm] += ticks - before[tid][1]
        previous = s
    out["gazebo_threads"] = {
        "samples": n_samples,
        "state_share": {k: {st: round(c / max(1, sum(v.values())), 4) for st, c in v.most_common()} for k, v in split.items()},
        "d_state_wchan_top": {k: v.most_common(5) for k, v in dwchan.items()},
        "busiest_threads_ticks": busy.most_common(8)}
    mhz = [s["cpu_mhz_mean"] for s in samples if s.get("cpu_mhz_mean")]
    out["cpu_mhz"] = {"mean": round(statistics.fmean(mhz), 1), "min": round(min(mhz), 1)} if mhz else None
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path, help="state/p3-gate-D-<sha>.result.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    inputs = {str(args.result): sha(args.result)}
    result = json.load(open(args.result))
    runs = [measure(Path(row["run"]), inputs) for row in result["runs"] if row.get("run")
            and (Path(row["run"]) / "summary.json").exists()
            and json.load(open(Path(row["run"]) / "summary.json")).get("diagnostic_window_monotonic_s")]
    report = {"contract": "p3-stall-diagnostic-measurement-v1", "acceptance_credit": False,
              "gate_result": {k: result.get(k) for k in ("manifest_sha256", "accepted", "first_failure", "planned")},
              "starts": [{"role": r["role"], "run": r["run"], "exit": r["runtime_exit_code"]} for r in result["runs"]],
              "runs": runs, "inputs_sha256": inputs}
    args.output.write_text(json.dumps(report, indent=1) + "\n")
    print(json.dumps([{k: r[k] for k in ("variant", "status", "rtf_1s", "sim_second_wall_s", "cpu_cores_total_median")}
                      for r in runs], indent=1))


if __name__ == "__main__":
    main()
