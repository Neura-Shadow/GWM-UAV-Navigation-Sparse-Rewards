"""P7 M0 read-only audit checks (plan §7 M0): the numbers behind docs/v3_sim_p7_m0_audit.md.

Offline only: no simulator, no runtime and no change to any P2-P6 file. It imports the bound P4 world generator and
the bound P5 layout generator unchanged, reads retained evidence, and writes one JSON file (exclusive create).

1. The bound P5 generator reproduces the P5 train, validation and held-out splits (counts, seeds tried, rejections,
   and the held-out layouts byte-equal to the readiness report).
2. The fresh P7 seed ranges (pilot 5,000,000, confirmatory 4,000,000): admission outcomes with the P5 split-match
   exclusion (contract §4.1), by family and box count.
3. The P5 held-out comparison split by near-duplicate status (matched to a train or validation layout).
4. Exact McNemar power for the planned population and for the excluded population.
5. C6: S1-S6 regenerated from the pinned default world against every accepted P4 and P5 flight's world SHA-256.
6. The endpoint records of the four required reference flights (R1-R4).

Usage (WSL, from /tmp): PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B p7_m0_audit_checks.py <output.json>
"""
import glob
import hashlib
import json
import math
import multiprocessing
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROOT = Path.home() / "uav_autonomy"
P4 = REPO / "simulation/px4_gazebo_p4"
P5 = REPO / "simulation/px4_gazebo_p5"
MODEL = P5 / "models/p5-ranker-ee006a805ba9c327a29f8f97ef6a13d0608b8459b37f0ca873a5155c59eaf73c"
MANIFESTS = {"p4": "c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b",
             "p5": "2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632"}
REFERENCE = ("20261030T144136Z-p4-flight-S2-abeea2d5", "20261030T141506Z-p4-flight-S2-9b8ef2b0",
             "20261106T132442Z-p5-flight-S2-184a5a5e", "20261106T130044Z-p5-flight-S2-679dd050")
PILOT, CONFIRMATORY = (5_000_000, 100), (4_000_000, 400)
sys.path.insert(0, str(P5 / "training"))
sys.path.insert(0, str(P4 / "scripts"))

import numpy  # noqa: E402
import p5_layouts  # noqa: E402
from p4_worlds import generate as world  # noqa: E402


def pmap(pool):
    return lambda fn, seeds: pool.imap(fn, seeds, chunksize=8)


def splits(pool):
    manifest = json.loads(Path(str(MODEL) + ".dataset_manifest.json").read_text())
    readiness = json.loads(Path(str(MODEL) + ".readiness_report.json").read_text())
    out, layouts = {}, {}
    for name in ("train", "validation", "held_out"):
        got = p5_layouts.split(name, mapper=pmap(pool))
        record = manifest["splits"][name]
        same = {k: got[k] == record[k] for k in ("seed_base", "seeds_tried", "rejected")}
        same["count"] = len(got["layouts"]) == record["count"]
        if name == "held_out":
            same["layouts_equal_readiness_report"] = got["layouts"] == readiness["layouts"]
        out[name] = same
        layouts[name] = got["layouts"]
    return out, layouts, readiness


def screen(pool, base, seeds, exclusion):
    outcomes, strata = {}, {}
    for layout, reason in pool.imap(p5_layouts._screen, range(base, base + seeds), chunksize=8):
        if reason is None:
            key = "%s/%d-box" % (layout["family"], len(layout["boxes"]))
            row = strata.setdefault(key, {"admissible": 0, "p5_match": 0})
            row["admissible"] += 1
            if any(p5_layouts.matches(layout["boxes"], other) for other in exclusion):
                reason = "p5_split_match"
                row["p5_match"] += 1
        outcomes[reason or "admitted"] = outcomes.get(reason or "admitted", 0) + 1
    return {"seed_base": base, "seeds_screened": seeds, "outcomes": outcomes, "by_family_and_box_count": dict(sorted(strata.items()))}


def held_out_by_match(layouts, readiness):
    seen = [layout["boxes"] for name in ("train", "validation") for layout in layouts[name]]
    boxes = {layout["id"]: layout["boxes"] for layout in readiness["layouts"]}
    rows = {}
    for e in readiness["episodes"]:
        matched = any(p5_layouts.matches(boxes[e["id"]], other) for other in seen)
        g4, g5 = e["p4"]["outcome"] == "goal_reached", e["policy"]["outcome"] == "goal_reached"
        cell = "both" if g4 and g5 else "p5_only" if g5 else "p4_only" if g4 else "neither"
        for key in ("all", "matched" if matched else "unmatched", e["family"] + ("/matched" if matched else "/unmatched")):
            row = rows.setdefault(key, {"layouts": 0, "p5_only": 0, "p4_only": 0, "both": 0, "neither": 0})
            row["layouts"] += 1
            row[cell] += 1
    return dict(sorted(rows.items()))


def sign_p(k, n):
    if n == 0:
        return 1.0
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(0, min(k, n - k) + 1)) / 2 ** n)


def mcnemar_power(n, discordant, share, alpha=0.05):
    total = 0.0
    for d in range(n + 1):
        weight = math.comb(n, d) * discordant ** d * (1 - discordant) ** (n - d)
        if weight < 1e-14:
            continue
        total += weight * sum(math.comb(d, k) * share ** k * (1 - share) ** (d - k) for k in range(d + 1) if sign_p(k, d) < alpha)
    return round(total, 3)


def power(rows):
    out = {}
    for label, row in (("planned_all_held_out", rows["all"]), ("excluded_population_unmatched", rows["unmatched"])):
        disc = row["p5_only"] + row["p4_only"]
        out[label] = {"discordant_proportion": disc / row["layouts"], "p5_only_share": row["p5_only"] / disc,
                      "power": {str(n): mcnemar_power(n, disc / row["layouts"], row["p5_only"] / disc) for n in (150, 200, 250)}}
    return out


def world_parity():
    scenarios = json.loads((P4 / "configs/p4_scenarios.json").read_text())
    receipt = json.loads((ROOT / "state/px4-mc-trace-built.json").read_text())
    env = (Path(receipt["build"]) / "rootfs/gz_env.sh").read_text()
    worlds = next(line.split("=", 1)[1] for line in env.splitlines() if line.startswith("export PX4_GZ_WORLDS="))
    base = (Path(worlds) / "default.sdf").read_text()
    regenerated = {n: hashlib.sha256(world(base, n, scenarios)[0].encode()).hexdigest() for n in sorted(scenarios["scenarios"])}
    recorded = {}
    for phase, sha in MANIFESTS.items():
        for path in sorted(glob.glob(str(ROOT / "state" / f"{phase}-gate-*-{sha}.result.json"))):
            for row in json.loads(Path(path).read_text())["runs"]:
                if row.get("scope") == "flight":
                    run = Path(row["run"]) if row["run"].startswith("/") else ROOT / "runs" / row["run"]
                    s = json.loads((run / "summary.json").read_text())
                    recorded.setdefault(s["scenario"]["scenario"], []).append({"phase": phase, "run": run.name,
                                                                              "sha256": s["generated_world_sha256"]})
    return {n: {"regenerated_sha256": regenerated[n], "accepted_flights": len(recorded.get(n, [])),
                "phases": sorted({r["phase"] for r in recorded.get(n, [])}),
                "all_equal": bool(recorded.get(n)) and all(r["sha256"] == regenerated[n] for r in recorded[n])}
            for n in regenerated}


def reference_runs():
    out = {}
    for name in REFERENCE:
        run = ROOT / "runs" / name
        s = json.loads((run / "summary.json").read_text())
        r = s.get("controller_result") or {}
        e = json.loads((run / ("p5-evaluation.json" if "-p5-" in name else "p4-evaluation.json")).read_text())
        transitions = [t["to"] for t in r.get("transitions", [])]
        out[name] = {"controller": r.get("controller"), "final_state": r.get("final_state"), "p4_outcome": r.get("p4_outcome"),
                     "goal_reached_sim_s": (r.get("p4") or {}).get("goal_reached_sim_s"),
                     "armed_transition_recorded": "VERIFY_ARMED" in transitions,
                     "final_goal_distance_m": (e.get("outcome") or {}).get("final_goal_distance_m"),
                     "min_truth_clearance_m": (e.get("safety") or {}).get("min_truth_clearance_m"),
                     "evaluation_status": e.get("status"), "wall_duration_s": s.get("wall_duration_s"),
                     "bytes": sum(f.stat().st_size for f in run.rglob("*") if f.is_file() and not f.is_symlink())}
    return out


def main(argv):
    if len(argv) != 1:
        raise SystemExit(__doc__)
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists; never overwritten")
    with multiprocessing.Pool(min(16, os.cpu_count() or 1)) as pool:
        reproduced, layouts, readiness = splits(pool)
        exclusion = [layout["boxes"] for name in ("train", "validation", "held_out") for layout in layouts[name]]
        result = {"contract": "p7-m0-audit-checks-v1", "python": sys.version.split()[0], "numpy": numpy.__version__,
                  "p5_split_reproduction": reproduced, "exclusion_layouts": len(exclusion),
                  "p7_seed_screen": {"pilot": screen(pool, *PILOT, exclusion), "confirmatory": screen(pool, *CONFIRMATORY, exclusion)}}
    rows = held_out_by_match(layouts, readiness)
    result.update(held_out_by_near_duplicate=rows, mcnemar_power=power(rows), c6_world_parity=world_parity(),
                  reference_flights=reference_runs())
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, indent=1, sort_keys=True, allow_nan=False) + "\n")
    print(hashlib.sha256(output.read_bytes()).hexdigest())


if __name__ == "__main__":
    main(sys.argv[1:])
