"""P7-E proposal: read-only cleanup manifest for the revised retention policy (proposed D4-B). It deletes nothing.

For every run under ~/uav_autonomy/runs it records:
* the size, and the size of its raw depth recording (`sensors/depth.bin`) and of `tmp/`;
* every record that depends on the run: state records (gate, dry-run and block results, candidates, approvals),
  tests, simulation code and configs, evidence JSON, and documentation;
* a safety classification.

Classes:
* **PROTECTED** — never eligible without separate authorization:
  * accepted or attempted gate evidence of P3-P6, and runs named by any state record;
  * all P7 runs (M4, both pilot attempts);
  * runs named by tests, code, configs or evidence JSON;
  * the required reference runs.
* **REVIEW** — a person must look first: cited only in documentation; or a missing, crashed or safety-relevant
  evaluation (a possible unresolved failure or safety incident, whose complete raw evidence is kept).
* **ELIGIBLE** — a candidate for a future, separately approved cleanup. It has a complete evaluation without safety
  items and no dependency. Only its raw depth recording and its `tmp/` would be eligible. Removing `depth.bin`
  ends re-evaluation of the depth stream, while the per-frame index with its SHA-256s stays.

Sizes come from `stat`. SHA-256s are the M0 inventory's where it has the file. The output is one JSON file, written
with exclusive create.
Usage: python3 p7e_cleanup_manifest.py <output.json>
"""
import json
import os
from pathlib import Path
import re
import sys

ROOT = Path.home() / "uav_autonomy"
REPO = Path(__file__).resolve().parents[2]
INVENTORY = REPO / "diagnostics/p7_m0/results/p7_m0_inventory_files.jsonl"
REFERENCE = {"20261030T144136Z-p4-flight-S2-abeea2d5", "20261030T141506Z-p4-flight-S2-9b8ef2b0",
             "20261106T132442Z-p5-flight-S2-184a5a5e", "20261106T130044Z-p5-flight-S2-679dd050"}
EVALUATIONS = ("p3-evaluation.json", "p4-evaluation.json", "p5-evaluation.json", "p6/p6-evaluation.json", "evaluation.json")
SAFETY_ITEM = re.compile(r"^(safety:|native_commands:|p2:final_landed_disarmed|p2:command_sequence|competing_flight_publisher|"
                         r"evaluator_incomplete_evidence|response:UNSAFE|unsafe)")
TOKEN = re.compile(r"\d{8}T\d{6}Z-[A-Za-z0-9'_.-]+")


def size(path):
    total = 0
    for dirpath, _dirs, files in os.walk(path, followlinks=False):
        for name in files:
            p = os.path.join(dirpath, name)
            if not os.path.islink(p):
                total += os.stat(p).st_size
    return total


def references(names, sources):
    """name -> sorted list of 'kind:path' that mention it (whole-name matches only)."""
    found = {n: set() for n in names}
    for kind, path in sources:
        try:
            text = Path(path).read_text(errors="ignore")
        except OSError:
            continue
        for token in TOKEN.findall(text):
            for n in (token, token.rstrip(".")):
                if n in found:
                    found[n].add("%s:%s" % (kind, path))
                    break
            else:
                for n in found:  # a token followed by a path or punctuation
                    if token.startswith(n):
                        found[n].add("%s:%s" % (kind, path))
    return {n: sorted(v) for n, v in found.items()}


INVENTORY_LIKE = ("v3_sim_p7_m0_", "v3_sim_p7e_")  # inventories and this proposal's own outputs list runs; not dependencies


def sources():
    out = []
    for p in sorted((ROOT / "state").glob("*.json")):
        out.append(("state_dry_run" if "dryrun" in p.name or "dry-run" in p.name else "state", p))
    out += [("test", p) for p in sorted((REPO / "tests").rglob("*.py"))]
    out += [("code_or_config", p) for p in sorted((REPO / "simulation").rglob("*"))
            if p.is_file() and p.suffix in (".py", ".json", ".yaml", ".sh", ".txt") and "__pycache__" not in p.parts]
    out += [("evidence_json", p) for p in sorted((REPO / "docs/evidence").glob("*.json")) if not p.name.startswith(INVENTORY_LIKE)]
    out += [("documentation", p) for p in sorted((REPO / "docs").glob("*.md"))]
    return out


def evaluation_state(run):
    found = [(n, run / n) for n in EVALUATIONS if (run / n).exists()]
    if not found:
        return {"present": False}
    items, statuses = [], []
    for name, path in found:
        try:
            value = json.loads(path.read_text())
        except ValueError:
            return {"present": True, "unreadable": name}
        statuses.append((name, value.get("status")))
        items += [i for i in value.get("failures", []) if SAFETY_ITEM.match(str(i))]
    return {"present": True, "statuses": statuses, "safety_relevant_items": sorted(set(items))[:20]}


def main(argv):
    output = Path(argv[0])
    if output.exists():
        raise SystemExit("output exists")
    runs = sorted(p for p in (ROOT / "runs").iterdir() if p.is_dir())
    refs = references({r.name for r in runs}, sources())
    known = {}
    if INVENTORY.exists():
        with INVENTORY.open(encoding="utf-8") as stream:
            for line in stream:
                e = json.loads(line)
                if e["path"].endswith("/sensors/depth.bin"):
                    known[e["path"]] = e.get("sha256")
    rows, totals = [], {"PROTECTED": [0, 0], "REVIEW": [0, 0], "ELIGIBLE": [0, 0]}
    eligible_depth = eligible_tmp = 0
    for run in runs:
        depth, tmp = run / "sensors/depth.bin", run / "tmp"
        row = {"run": run.name, "bytes": size(run), "depth_bin_bytes": depth.stat().st_size if depth.exists() else 0,
               "tmp_bytes": size(tmp) if tmp.exists() else 0, "dependencies": refs[run.name],
               "evaluation": evaluation_state(run),
               "depth_bin_sha256_m0_inventory": known.get("runs/%s/sensors/depth.bin" % run.name)}
        kinds = {d.split(":", 1)[0] for d in row["dependencies"]}
        reasons = []
        if "-p7-" in run.name or "-p7dry-" in run.name:
            reasons.append("P7 evidence (M4 or a pilot attempt): frozen, protected")
        if run.name in REFERENCE:
            reasons.append("required reference run (R1-R4; regression tests)")
        for kind, why in (("state", "named by a gate result, block result, candidate or approval"),
                          ("test", "named by a test"), ("code_or_config", "named by simulation code or config"),
                          ("evidence_json", "named by an evidence JSON")):
            if kind in kinds:
                reasons.append(why)
        if reasons:
            cls = "PROTECTED"
        else:
            ev = row["evaluation"]
            if "documentation" in kinds:
                reasons.append("cited in documentation")
            if "state_dry_run" in kinds:
                row["dry_run_record"] = True  # a development dry run (no gate credit): a dependency, not a protection
            if not ev.get("present") or ev.get("unreadable"):
                reasons.append("no readable evaluation (possible unresolved failure)")
            elif ev.get("safety_relevant_items"):
                reasons.append("safety-relevant evaluation items (complete raw evidence kept)")
            cls = "REVIEW" if reasons else "ELIGIBLE"
            if cls == "ELIGIBLE":
                reasons.append("evaluated, no safety item, no dependency")
                eligible_depth += row["depth_bin_bytes"]
                eligible_tmp += row["tmp_bytes"]
        row.update(classification=cls, reasons=reasons,
                   eligible_items=(["sensors/depth.bin"] if row["depth_bin_bytes"] else []) + (["tmp/"] if row["tmp_bytes"] else [])
                   if cls == "ELIGIBLE" else [])
        totals[cls][0] += 1
        totals[cls][1] += row["bytes"]
        rows.append(row)
    value = {"contract": "p7e-cleanup-manifest-v1", "read_only": True, "deletes_nothing": True,
             "root": str(ROOT / "runs"), "runs": len(rows),
             "totals": {k: {"runs": v[0], "bytes": v[1]} for k, v in totals.items()},
             "estimated_savings_if_approved": {"depth_bin_bytes": eligible_depth, "tmp_bytes": eligible_tmp,
                                               "total_bytes": eligible_depth + eligible_tmp},
             "review_depth_bin_bytes": sum(r["depth_bin_bytes"] for r in rows if r["classification"] == "REVIEW"),
             "not_in_scope": "build trees, toolchains and runtime installs (reproducibility not verified); state; the C: backup",
             "rows": rows}
    with open(output, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: value[k] for k in ("runs", "totals", "estimated_savings_if_approved", "review_depth_bin_bytes")}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
