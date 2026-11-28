"""Evidence Retention Manager: the storage measurements behind the retention design. It reads only.

1. **Composition** of sample flights: bytes by artifact group, depth frames, raw and stored bytes, frame rate.
2. **zlib-1 recompression identity:** whether `zlib.compress(zlib.decompress(stored), 1)` reproduces each recorded
   stored frame byte for byte. If it does, a recording can be archived in another lossless codec and restored to
   the identical `depth.bin`, verified by every frame's `stored_sha256`.
3. **Event-window estimate** over the accepted P7-E batch (60 flights): the fraction of 30 Hz frames inside ±W s of
   an event (HOLD action, supervisor transition, stale perception, P5 intervention), plus one key frame per second
   elsewhere.
4. **Retained-record compressibility** of one flight (everything except `sensors/depth.bin` and `tmp/`), in memory.
5. **Disk state:** the WSL file system, `runs/`, and the size of the WSL disk image.

Runs are identified by role and by the SHA-256 of their depth index, never by name.
Usage: cd /tmp && python3 storage_measurements.py OUTPUT.json ROLE=RUN_DIR [ROLE=RUN_DIR ...]
"""
import bisect
import hashlib
import json
import lzma
import os
from pathlib import Path
import shutil
import statistics
import sys
import zlib

ROOT = Path.home() / "uav_autonomy"
P7E_SHA = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
VHDX = Path("/mnt/d/WSL/Ubuntu-24.04/ext4.vhdx")
WINDOW_S = 3.0


def jsonl(path):
    return [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]


def group(rel):
    if rel.startswith("sensors/depth.bin"):
        return "sensors/depth.bin"
    if rel.startswith("tmp/"):
        return "tmp/"
    return rel.split("/")[0] + ("/" if "/" in rel else "")


def files(run):
    for dirpath, _dirs, names in os.walk(run, followlinks=False):
        for name in names:
            p = Path(dirpath) / name
            if not p.is_symlink():
                yield p, p.relative_to(run).as_posix()


def composition(run):
    sizes = {}
    for p, rel in files(run):
        sizes[group(rel)] = sizes.get(group(rel), 0) + p.stat().st_size
    index = [r for r in jsonl(run / "sensors/depth-index.jsonl") if r.get("accepted")]
    t = [r["receipt_sim_s"] for r in index]
    total = sum(sizes.values())
    return {"depth_index_sha256": hashlib.sha256((run / "sensors/depth-index.jsonl").read_bytes()).hexdigest(),
            "total_bytes": total, "frames": len(index), "sim_span_s": round(t[-1] - t[0], 1),
            "frame_rate_hz": round((len(t) - 1) / (t[-1] - t[0]), 2), "raw_bytes": sum(r["raw_bytes"] for r in index),
            "stored_bytes": sum(r["stored_bytes"] for r in index),
            "zlib1_ratio": round(sum(r["raw_bytes"] for r in index) / sum(r["stored_bytes"] for r in index), 3),
            "retained_bytes": total - sizes.get("sensors/depth.bin", 0) - sizes.get("tmp/", 0),
            "groups_top": dict(sorted(sizes.items(), key=lambda kv: -kv[1])[:10])}


def recompression(run, step=1):
    rows = [r for r in jsonl(run / "sensors/depth-index.jsonl") if r.get("offset") is not None][::step]
    same = n = 0
    with open(run / "sensors/depth.bin", "rb") as stream:
        for r in rows:
            if r["compression"] != "zlib-1":
                continue
            stream.seek(r["offset"])
            stored = stream.read(r["stored_bytes"])
            n += 1
            same += zlib.compress(zlib.decompress(stored), 1) == stored
    return {"frames_checked": n, "step": step, "byte_identical": same, "all_identical": same == n > 0}


def event_windows():
    flights = []
    for block in (1, 2, 3):
        flights += json.loads((ROOT / "state" / ("p7e-engineering-30-block-%02d-%s.result.json" % (block, P7E_SHA))).read_text())["flights"]
    rows = []
    for f in flights:
        run = Path(f["run"])
        t = [r["receipt_sim_s"] for r in jsonl(run / "sensors/depth-index.jsonl") if r.get("accepted")]
        events = [d.get("sim_s") for d in jsonl(run / "p4/decisions.jsonl")
                  if d.get("event") == "p4_supervisor_transition" or d.get("action") == "HOLD" or d.get("stale")]
        if (run / "p5/policy-ledger.jsonl").exists():
            events += [d.get("sim_s") for d in jsonl(run / "p5/policy-ledger.jsonl") if d.get("intervention")]
        events = sorted(x for x in events if x is not None)
        kept = key = 0
        last = -1e9
        for x in t:
            j = bisect.bisect_left(events, x - WINDOW_S)
            if j < len(events) and events[j] <= x + WINDOW_S:
                kept += 1
            elif x - last >= 1.0:
                key += 1
                last = x
        outcome = json.loads((run / "p7/flight-record.json").read_text())["outcome"]
        rows.append({"success": outcome == "goal_success", "frames": len(t), "event_frames": kept, "key_frames": key})

    def summary(sel):
        fr = [(r["event_frames"] + r["key_frames"]) / r["frames"] for r in sel]
        return {"flights": len(sel), "kept_fraction_median": round(statistics.median(fr), 3),
                "kept_fraction_min": round(min(fr), 3), "kept_fraction_max": round(max(fr), 3),
                "event_only_median": round(statistics.median([r["event_frames"] / r["frames"] for r in sel]), 3)}
    return {"window_s": WINDOW_S, "key_frame_hz": 1, "events": "HOLD action, supervisor transition, stale perception, P5 intervention",
            "successes": summary([r for r in rows if r["success"]]), "failures": summary([r for r in rows if not r["success"]]),
            "all_kept_fraction": round(sum(r["event_frames"] + r["key_frames"] for r in rows) / sum(r["frames"] for r in rows), 3)}


def retained_compressibility(run):
    groups = {}
    for p, rel in files(run):
        if rel == "sensors/depth.bin" or rel.startswith("tmp/"):
            continue
        kind = "ulog" if rel.endswith(".ulg") else ("json/jsonl" if rel.endswith((".json", ".jsonl")) else
                                                    ("mc-trace.bin" if rel.endswith("mc-trace.bin") else "other"))
        b = p.read_bytes()
        g = groups.setdefault(kind, [0, 0, 0])
        g[0] += len(b)
        g[1] += len(zlib.compress(b, 6))
        g[2] += len(lzma.compress(b, preset=1))
    total = [sum(v[i] for v in groups.values()) for i in range(3)]
    return {"groups": {k: {"bytes": v[0], "zlib6_ratio": round(v[0] / v[1], 2), "lzma1_ratio": round(v[0] / v[2], 2)}
                       for k, v in groups.items()},
            "total_bytes": total[0], "zlib6_bytes": total[1], "lzma1_bytes": total[2]}


def disk():
    usage = shutil.disk_usage(ROOT)
    runs = sum(p.stat().st_size for d in (ROOT / "runs").iterdir() if d.is_dir() and not d.is_symlink()
               for p, _ in files(d))
    return {"wsl_fs_total_bytes": usage.total, "wsl_fs_used_bytes": usage.used, "wsl_fs_free_bytes": usage.free,
            "runs_entries": len(list((ROOT / "runs").iterdir())), "runs_bytes": runs,
            "vhdx_bytes": VHDX.stat().st_size if VHDX.exists() else None}


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    samples = dict(a.split("=", 1) for a in argv[1:])
    value = {"contract": "evidence-storage-measurements-v1", "read_only": True, "zlib_version": zlib.ZLIB_RUNTIME_VERSION,
             "tool_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             "composition": {role: composition(Path(run)) for role, run in samples.items()},
             "recompression": {role: recompression(Path(run), 1 if i == 0 else 7) for i, (role, run) in enumerate(samples.items())},
             "event_windows_p7e_batch": event_windows(),
             "retained_compressibility": {role: retained_compressibility(Path(run)) for role, run in list(samples.items())[:1]},
             "disk": disk()}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: value[k] for k in ("recompression", "event_windows_p7e_batch", "disk")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
