"""P7-E release closure: read-only cleanup manifest v3. It deletes, moves and compresses nothing.

It refreshes version 2 (`p7e_cleanup_manifest.py`, output `v3_sim_p7e_cleanup_manifest_v2.json`; both stay
unchanged) against the current filesystem. It reuses v2's measurement and reference scan. Changes:

1. **The completed P7-E runs.**
   * Decision E10: the raw depth of the accepted batch (candidate e9561f05) becomes a cleanup candidate only after
     the report's acceptance (2026-11-28), and only under a separately approved cleanup.
   * The batch's own records do not by themselves protect its raw depth: its block results, ledgers, the accepted
     engineering report, and this release's record-only analyses. Each needs only records that D4-B always keeps,
     never raw frames. This is reading R-E10, for the operator's decision; the literal reading protects all 60
     flights.
   * The stopped candidate's flights (38e23bad) and the E8 development dry runs stay PROTECTED.
2. **Named protections** (operator request, 2026-11-28), PROTECTED with their reason:
   * slot 27 P4 (the R4 compute breach) and slot 10 P5 (the R10 Gazebo shutdown);
   * every hover-timeout failure, plus the partners of the discordant pairs (P8 offline replay inputs);
   * the layouts flown on both P7-E candidates (cross-candidate repeat evidence);
   * every run whose evaluation or flight record carries stale-action, observation-gap or RECOVERY evidence.
3. **`diagnostics/` is scanned** as a dependency source (v2 skipped it). The M0 inventory files list every file by
   design, so they are inventory-like and do not count.
4. **Integrity at manifest time.** Every ELIGIBLE `depth.bin` is read once:
   * its whole-file SHA-256 is computed;
   * every frame is checked against the stored SHA-256 recorded in `depth-index.jsonl`;
   * the index must cover the file exactly;
   * P6 files are also compared with the M0 inventory.
   Every ELIGIBLE `tmp/` file is listed with its size and SHA-256. Any mismatch makes the run REVIEW, and nothing
   of it is eligible.
5. **Outside `runs/`:** the sizes of caches and temporary directories, each with a classification, for information.
   Nothing outside `runs/` is ever marked eligible here.

Reconciliation is as in v2: if any entry of `runs/` is unaccounted for, nothing is ELIGIBLE.
Usage: cd /tmp && python3 -I p7e_cleanup_manifest_v3.py <output.json>   (exclusive create)
"""
import hashlib
import json
import os
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p7e_cleanup_manifest as v2  # noqa: E402  (unchanged; reused for size, references, evaluation_state)

CONTRACT = "p7e-cleanup-manifest-v3"
ACCEPTED = "e9561f05de28860d2826ed8c821bde132a8f817db01f7264c262c0ab895c5608"
STOPPED = "38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be"
NAMED = {  # (slot, controller) -> reason, for the accepted batch
    (27, "p4"): "R4 breach: slot 27 P4 perception compute time (risk-register proposal)",
    (10, "p5"): "R10 integrity event: slot 10 P5 Gazebo forced kill at shutdown",
    (7, "p5"): "P8 contrast: partner of a discordant pair (slot 7)",
    (15, "p5"): "P8 contrast: partner of a discordant pair (slot 15)",
    (1, "p4"): "cross-candidate repeat evidence: layout first flown on 38e23bad",
    (1, "p5"): "cross-candidate repeat evidence: layout first flown on 38e23bad (then a stale_action RECOVERY)",
    (2, "p4"): "cross-candidate repeat evidence: layout first flown on 38e23bad (the interrupted flight)",
}
RECORD_ONLY = {"v3_sim_p7e_engineering_report.json", "v3_sim_p8_hover_timeout_analysis.json"}  # need no raw frames
TIMING = re.compile(r"stale_action|observation_gap|RECOVERY")
EVIDENCE_FILES = ("p3-evaluation.json", "p4-evaluation.json", "p5-evaluation.json", "p6/p6-evaluation.json",
                  "evaluation.json", "p7/flight-record.json")
CHUNK = 1 << 22


def sources(root, repo):
    out, seen = [], set()
    for kind, path in v2.sources():
        name = Path(path).name
        if kind == "state" and ACCEPTED[:12] in name and ("engineering-30-block" in name or "engineering-report" in name):
            kind = "p7e_batch_record"
        if kind == "evidence_json" and name in RECORD_ONLY:
            kind = "p7e_batch_record"
        out.append((kind, path))
        seen.add(str(path))
    out += [("p7e_batch_record", p) for p in sorted((repo / "docs/evidence").glob("*.json"))
            if p.name in RECORD_ONLY and str(p) not in seen]
    for p in sorted((repo / "diagnostics").rglob("*")):
        if not p.is_file() or "__pycache__" in p.parts or p.suffix not in (".py", ".sh", ".json", ".jsonl", ".yaml", ".txt"):
            continue
        if "p7_m0" in p.parts and "results" in p.parts:
            continue  # the M0 inventory lists every file by design
        if p.name.startswith("p7e_cleanup_manifest"):
            continue  # this tool and v2 name only the reference runs, which are protected anyway
        out.append(("diagnostics", p))
    return out


def summary_p7e(run):
    try:
        return (json.loads((run / "summary.json").read_text()).get("p7e") or {}), \
               (json.loads((run / "summary.json").read_text()).get("p7") or {})
    except (OSError, ValueError):
        return {}, {}


def timing_evidence(run):
    found = set()
    for name in EVIDENCE_FILES:
        path = run / name
        if path.exists():
            found |= set(TIMING.findall(path.read_text(errors="ignore")))
    return sorted(found)


def outcome(run):
    try:
        return json.loads((run / "p7/flight-record.json").read_text()).get("outcome")
    except (OSError, ValueError):
        return None


def verify_depth(run, m0_sha=None):
    """Whole-file SHA-256 of depth.bin and every frame against depth-index.jsonl; never writes."""
    depth, index = run / "sensors/depth.bin", run / "sensors/depth-index.jsonl"
    out = {"path": str(depth), "bytes": depth.stat().st_size, "sha256": None, "frames": 0, "frames_ok": 0,
           "index_covers_file": False, "m0_inventory_match": None, "ok": False, "error": None}
    try:
        frames = sorted((json.loads(line) for line in index.read_text().splitlines() if line.strip()),
                        key=lambda r: r.get("offset", -1))
        frames = [r for r in frames if r.get("accepted") and r.get("offset") is not None]
        cursor, contiguous = 0, True
        for r in frames:
            contiguous &= r["offset"] == cursor
            cursor = r["offset"] + r["stored_bytes"]
        out["index_covers_file"] = contiguous and cursor == out["bytes"]
        whole = hashlib.sha256()
        with depth.open("rb") as stream:
            for r in frames:
                data = stream.read(r["stored_bytes"])
                whole.update(data)
                out["frames"] += 1
                out["frames_ok"] += hashlib.sha256(data).hexdigest() == r["stored_sha256"]
            while True:  # anything after the last frame still counts in the whole-file hash
                data = stream.read(CHUNK)
                if not data:
                    break
                whole.update(data)
        out["sha256"] = whole.hexdigest()
        if m0_sha is not None:
            out["m0_inventory_match"] = m0_sha == out["sha256"]
        out["ok"] = out["index_covers_file"] and out["frames"] == out["frames_ok"] > 0 and out["m0_inventory_match"] is not False
    except (OSError, ValueError, KeyError) as exc:
        out["error"] = type(exc).__name__ + ":" + str(exc)[:200]
    return out


def tmp_files(run):
    out = []
    for dirpath, _dirs, files in os.walk(run / "tmp", followlinks=False):
        for name in sorted(files):
            p = Path(dirpath) / name
            if p.is_symlink():
                out.append({"path": str(p), "symlink": True})
                continue
            h = hashlib.sha256(p.read_bytes()).hexdigest()
            out.append({"path": str(p), "bytes": p.stat().st_size, "sha256": h})
    return out


def classify(run, deps, evaluation):
    """(class, reasons, category) for one run directory; fail-closed."""
    kinds = {d.split(":", 1)[0] for d in deps}
    p7e, p7 = summary_p7e(run) if "-p7e" in run.name else ({}, {})
    timing = timing_evidence(run)
    reasons = []
    if "-p7-" in run.name or "-p7dry-" in run.name:
        reasons.append("P7 evidence (M4 or a pilot attempt): frozen, protected")
    if "-p7edry-" in run.name:
        reasons.append("P7-E E8 development dry run: accepted launch-path parity evidence")
    if "-p7e-" in run.name and p7e.get("p7e_manifest_sha256") == STOPPED:
        reasons.append("flight of the stopped first P7-E candidate 38e23bad (protected evidence)")
    if "-p7e-" in run.name and p7e.get("p7e_manifest_sha256") not in (ACCEPTED, STOPPED):
        reasons.append("P7-E run of an unknown candidate")
    if run.name in v2.REFERENCE:
        reasons.append("required reference run (R1-R4; regression tests)")
    if timing:
        reasons.append("stale-action / observation-gap / RECOVERY evidence: " + ", ".join(timing))
    batch = "-p7e-" in run.name and p7e.get("p7e_manifest_sha256") == ACCEPTED and p7e.get("dry_run") is False
    if batch:
        key = (p7.get("slot"), p7.get("controller"))
        if key in NAMED:
            reasons.append(NAMED[key])
        if outcome(run) not in (None, "goal_success"):
            reasons.append("P8 investigation input: hover-timeout mission failure (raw frames for offline replay)")
    for kind, why in (("state", "named by a gate result, block result, candidate or approval"), ("test", "named by a test"),
                      ("code_or_config", "named by simulation code or config"), ("evidence_json", "named by an evidence JSON"),
                      ("diagnostics", "named by a diagnostics script or result")):
        if kind in kinds:
            reasons.append(why)
    if reasons:
        return "PROTECTED", reasons, None
    if not evaluation.get("present") or evaluation.get("unreadable"):
        return "REVIEW", ["no readable evaluation (possible unresolved failure)"], None
    if evaluation.get("safety_relevant_items"):
        return "REVIEW", ["safety-relevant evaluation items (complete raw evidence kept)"], None
    if batch:
        return "ELIGIBLE", ["accepted P7-E batch flight, report accepted 2026-11-28 (D4-B, E10, reading R-E10); "
                            "only its own batch records name it"], "p7e_raw_depth"
    if "documentation" in kinds:
        return "REVIEW", ["cited in documentation"], None
    category = "p6_development_raw_depth" if ("-p6dry-" in run.name or "state_dry_run" in kinds) else "other_raw_depth"
    return "ELIGIBLE", ["evaluated, no safety item, no protecting dependency"], category


def du(path):
    path = Path(path)
    if not path.exists():
        return None
    return v2.size(path) if path.is_dir() else path.stat().st_size


def outside(root, repo):
    home = Path.home()
    caches = {p.name: du(p) for p in sorted((home / ".cache").iterdir())} if (home / ".cache").exists() else {}
    pyc = [p for p in repo.rglob("*") if p.is_dir() and p.name in ("__pycache__", ".pytest_cache") and ".git" not in p.parts]
    return {
        "regenerable_temporary": {
            "/tmp/pytest-of-%s" % home.name: du(Path("/tmp/pytest-of-%s" % home.name)),
            "/tmp/px4-sock-* and /tmp/px4_lock-* (empty; only while no simulator runs)": sum(du(p) or 0 for p in Path("/tmp").glob("px4*")),
            str(root / "recovery_build_checks") + " (P3-era build checks; REVIEW)": du(root / "recovery_build_checks")},
        "caches": {"~/.cache/" + k: v for k, v in caches.items()},
        "repo_python_caches": {"directories": len(pyc), "bytes": sum(du(p) or 0 for p in pyc),
                               "note": "no manifest binds __pycache__ or .pyc files (verified 2026-11-28)"},
        "not_in_scope": ["build trees, toolchains and runtime installs (p2_ws-p5_ws, ros_ws, dds-install, upstream, "
                         "qgc-v5.1.4, qgc_closure_v1, mc_trace_builds, venv-px4)", "p5_training (model provenance)",
                         "state/ and logs/", "the C: backup", "the WSL disk image itself"],
    }


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    root, repo = v2.ROOT, v2.REPO
    entries = sorted((root / "runs").iterdir())
    runs = [p for p in entries if p.is_dir() and not p.is_symlink()]
    others = [p for p in entries if p not in runs]
    refs = v2.references({p.name for p in entries}, sources(root, repo))
    m0 = {}
    if v2.INVENTORY.exists():
        with v2.INVENTORY.open(encoding="utf-8") as stream:
            for line in stream:
                e = json.loads(line)
                if e["path"].endswith("/sensors/depth.bin"):
                    m0[e["path"]] = e.get("sha256")
    rows = []
    for run in runs:
        depth, tmp = run / "sensors/depth.bin", run / "tmp"
        evaluation = v2.evaluation_state(run)
        cls, reasons, category = classify(run, refs[run.name], evaluation)
        row = {"run": run.name, "bytes": v2.size(run), "depth_bin_bytes": depth.stat().st_size if depth.exists() else 0,
               "tmp_bytes": v2.size(tmp) if tmp.exists() else 0, "dependencies": refs[run.name], "evaluation": evaluation,
               "classification": cls, "reasons": reasons, "category": category, "eligible_items": []}
        if cls == "ELIGIBLE":
            if row["depth_bin_bytes"]:
                check = verify_depth(run, m0.get("runs/%s/sensors/depth.bin" % run.name))
                row["depth_bin"] = check
                if not check["ok"]:
                    row.update(classification="REVIEW", category=None,
                               reasons=reasons + ["integrity check failed at manifest time: nothing eligible"])
            if row["classification"] == "ELIGIBLE":
                row["tmp_files"] = tmp_files(run) if tmp.exists() else []
                row["eligible_items"] = (["sensors/depth.bin"] if row["depth_bin_bytes"] else []) + (["tmp/"] if row["tmp_bytes"] else [])
        rows.append(row)
    non_runs = v2.other_entries(others, refs)
    accounted = len(rows) + len(non_runs)
    reconciled = accounted == len(entries) == len(list((root / "runs").iterdir()))
    if not reconciled:
        for row in rows:
            if row["classification"] == "ELIGIBLE":
                row.update(classification="REVIEW", eligible_items=[], category=None, reasons=row["reasons"] + ["inventory not reconciled"])
    totals = {c: {"runs": sum(1 for r in rows if r["classification"] == c), "bytes": sum(r["bytes"] for r in rows if r["classification"] == c)}
              for c in ("PROTECTED", "REVIEW", "ELIGIBLE")}
    eligible = [r for r in rows if r["classification"] == "ELIGIBLE"]
    categories = {}
    for r in eligible:
        c = categories.setdefault(r["category"], {"runs": 0, "depth_bin_bytes": 0, "tmp_bytes": 0})
        c["runs"] += 1
        c["depth_bin_bytes"] += r["depth_bin_bytes"]
        c["tmp_bytes"] += r["tmp_bytes"]
    v2_rows = {}
    v2_path = repo / "docs/evidence/v3_sim_p7e_cleanup_manifest_v2.json"
    if v2_path.exists():
        v2_rows = {r["run"]: r["classification"] for r in json.loads(v2_path.read_text())["rows"]}
    changes = [{"run": r["run"], "v2": v2_rows.get(r["run"]), "v3": r["classification"], "reasons": r["reasons"]}
               for r in rows if r["run"] in v2_rows and v2_rows[r["run"]] != r["classification"]]
    value = {"contract": CONTRACT, "read_only": True, "deletes_nothing": True, "supersedes_for_planning": "p7e-cleanup-manifest-v2",
             "root": str(root / "runs"), "accepted_p7e_candidate": ACCEPTED, "stopped_p7e_candidate": STOPPED,
             "reading": "R-E10 (the batch's own records do not protect its raw depth); the literal E10 reading protects all 60",
             "runs": len(rows),
             "reconciliation": {"entries_in_runs": len(entries), "run_directories": len(rows), "other_entries": len(non_runs),
                                "accounted": accounted, "reconciled": reconciled},
             "other_entries": non_runs, "totals": totals, "eligible_by_category": categories,
             "eligible_total_bytes": sum(r["depth_bin_bytes"] + r["tmp_bytes"] for r in eligible),
             "review_depth_bin_bytes": sum(r["depth_bin_bytes"] for r in rows if r["classification"] == "REVIEW"),
             "changes_since_v2": changes, "new_since_v2": [r["run"] for r in rows if r["run"] not in v2_rows],
             "outside_runs": outside(root, repo), "rows": rows}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: value[k] for k in ("runs", "reconciliation", "totals", "eligible_by_category", "eligible_total_bytes",
                                            "review_depth_bin_bytes")}, indent=1))
    print("changes since v2:", len(changes), "new since v2:", len(value["new_since_v2"]))


if __name__ == "__main__":
    main(sys.argv[1:])
