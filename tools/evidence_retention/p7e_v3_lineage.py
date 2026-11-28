"""Evidence Retention Manager lineage: the P7-E cleanup manifest v3 (`ec5b2205…`) and the operator's decisions.

* **C-1:** reading R-E10. The accepted batch's own records do not protect its raw depth.
* **C-2:** the v3 protections, confirmed.
* **C-3:** slots 19 and 28 (4 runs) are protected by name.
* **C-4:** the 15 P6 depth-fault fixtures are kept (they are REVIEW in v3 and never in a set).
* **C-11 (2026-11-28, "依檔案判定"):** a dependency protects only the files it reads. A source that names a run
  protects its raw depth unless a passed record-only proof (`record_only_check.py`) covers that source's exact
  current bytes. Without the proof, or if its hash differs, the source protects, as in v3 (fail-closed).

The classification is re-derived with the frozen v3 classifier (`diagnostics/p7e/p7e_cleanup_manifest_v3.py`,
unchanged) on the current filesystem. Two filters apply:
* the cleanup's own records (authorizations, receipts, ledgers, proofs) are not dependencies, because they list runs
  by design, like the inventories that v2 already excludes;
* the proven record-only sources become the non-protecting kind `record_only_proven`.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

NAME = "p7e-cleanup-manifest-v3"
ROOT = Path(os.environ.get("GWM_SIM_ROOT", str(Path.home() / "uav_autonomy")))
REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "docs/evidence/v3_sim_p7e_cleanup_manifest_v3.json"
MANIFEST_SHA = "ec5b22058502b3b1c5f4548cb47d1f0055e75696c2118e40fe27df0f8bdd4135"
PROOF = REPO / "docs/evidence/v3_sim_retention_record_only_p8.json"
SETS = {"p6-dev-depth": {"category": "p6_development_raw_depth"}, "p7e-depth": {"category": "p7e_raw_depth"}}
C3 = {"20261127T150333Z-p7e-p5-p7c-4000382-998f6d1e", "20261127T150640Z-p7e-p4-p7c-4000382-ca6304df",
      "20261127T160514Z-p7e-p4-p7c-4000358-1a9e443f", "20261127T160823Z-p7e-p5-p7c-4000358-3d8d3c23"}
DECISIONS = ["C-1 reading R-E10", "C-2 v3 protections confirmed", "C-3 slots 19 and 28 protected by name",
             "C-4 the 15 P6 depth-fault fixtures kept", "C-11 artifact-level dependency rule (record-only proofs)",
             "C-5 procedure implemented and tested", "C-6 copy rehearsal"]
OWN_STATE = ("p7e-cleanup-",)
OWN_EVIDENCE = ("v3_sim_p7e_cleanup_", "v3_sim_retention_")
MANIFESTS = (  # P8 verifies P7-E e9561f05 inside it, which verifies P7 0799c37e and P3-P6
    ("simulation/px4_gazebo_p8/scripts/p8_manifest.py", "state/p8-candidate-2a0ae57.json",
     "fe2a0c6e9052e69a8eb5be6fa2a15b55c13d034f9a9c9d213a3dd3bc73dfdc6a"),
    ("simulation/px4_gazebo_p7e/scripts/p7e_manifest.py", "state/p7e-candidate-2b6ee5d.json",
     "38e23bad4bd4f425c1fc47f7d6d50af42ea81a6f5812b56a8c2f709ec7e844be"),
)
_cache = {}


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def modules():
    if "v3" not in _cache:
        path = REPO / "diagnostics/p7e"
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
        import p7e_cleanup_manifest as v2  # noqa: E402  (frozen; read-only)
        import p7e_cleanup_manifest_v3 as v3  # noqa: E402
        _cache.update(v2=v2, v3=v3)
    return _cache["v2"], _cache["v3"]


def manifest():
    if "manifest" not in _cache:
        data = MANIFEST.read_bytes()
        if hashlib.sha256(data).hexdigest() != MANIFEST_SHA:
            raise RuntimeError("the cleanup manifest v3 differs from ec5b2205")
        _cache["manifest"] = json.loads(data)
    return _cache["manifest"]


def manifest_identity():
    return {"path": str(MANIFEST.relative_to(REPO)), "sha256": sha(MANIFEST), "contract": "p7e-cleanup-manifest-v3"}


def classifier_identity():
    return {name: sha(REPO / name) for name in ("diagnostics/p7e/p7e_cleanup_manifest.py",
                                                "diagnostics/p7e/p7e_cleanup_manifest_v3.py")}


# ---- the plan -----------------------------------------------------------------------------------------------------
def plan(set_name, only=None):
    """Every removable item of one set, as verified in the manifest. C-3 runs are never in a set."""
    category = SETS[set_name]["category"]
    rows = [r for r in manifest()["rows"] if r["classification"] == "ELIGIBLE" and r["category"] == category
            and r["run"] not in C3]
    if only is not None:
        missing = set(only) - {r["run"] for r in rows}
        if missing:
            raise RuntimeError("not in the set %s: %s" % (set_name, sorted(missing)))
        rows = [r for r in rows if r["run"] in set(only)]
    items = []
    for r in rows:
        base = "%s/runs/%s/" % (manifest()["root"].rsplit("/runs", 1)[0], r["run"])
        if r.get("depth_bin_bytes"):
            d = r["depth_bin"]
            if not d.get("ok"):
                raise RuntimeError("manifest depth check not ok: %s" % r["run"])
            items.append({"run": r["run"], "path": "sensors/depth.bin", "bytes": d["bytes"], "sha256": d["sha256"],
                          "frames": d["frames"]})
        for t in r.get("tmp_files") or []:
            if t.get("symlink") or not t["path"].startswith(base + "tmp/"):
                raise RuntimeError("manifest tmp item not removable: %s" % t["path"])
            items.append({"run": r["run"], "path": t["path"][len(base):], "bytes": t["bytes"], "sha256": t["sha256"]})
    return items


# ---- the re-derivation --------------------------------------------------------------------------------------------
def record_only_sources():
    """{absolute source path: tool} for sources whose exact current bytes a passed record-only proof covers."""
    try:
        proof = json.loads(PROOF.read_text())
    except (OSError, ValueError):
        return {}
    out = {}
    for rel, entry in (proof.get("sources") or {}).items():
        p = proof.get("proofs", {}).get(entry.get("proof")) or {}
        path = REPO / rel
        if p.get("passed") is True and path.is_file() and sha(path) == entry.get("sha256"):
            out[str(path)] = entry["proof"]
    return out


def sources():
    v2, v3 = modules()
    proven = record_only_sources()
    out = []
    for kind, path in v3.sources(ROOT, REPO):
        name = Path(path).name
        if kind in ("state", "state_dry_run") and name.startswith(OWN_STATE):
            continue
        if kind == "evidence_json" and name.startswith(OWN_EVIDENCE):
            continue
        out.append(("record_only_proven" if str(path) in proven else kind, path))
    return out


def derive(runs_root, names):
    """{run: {classification, category, reasons, dependencies}} now, with C-3 and C-11."""
    v2, v3 = modules()
    runs_root = Path(runs_root)
    others = {e["entry"] for e in manifest()["other_entries"]}
    entries = list(runs_root.iterdir())
    reconciled = all((p.is_dir() and not p.is_symlink()) or p.name in others for p in entries)
    refs = v2.references(set(names), sources())
    out = {}
    for name in names:
        run = runs_root / name
        if run.is_symlink() or not run.is_dir():
            out[name] = {"classification": "MISSING", "category": None, "reasons": ["no run directory"], "dependencies": []}
            continue
        cls, reasons, category = v3.classify(run, refs[name], v2.evaluation_state(run))
        if name in C3:
            cls, category, reasons = "PROTECTED", None, reasons + ["C-3: narrow-passage contrast for P8 (slots 19, 28)"]
        if cls == "ELIGIBLE" and not reconciled:
            cls, category, reasons = "REVIEW", None, reasons + ["inventory not reconciled"]
        out[name] = {"classification": cls, "category": category, "reasons": reasons, "dependencies": refs[name]}
    return out


# ---- preconditions and after-checks -------------------------------------------------------------------------------
def verify_manifests():
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", GWM_PX4_MC_TRACE="px4-mc-causal-trace-v1")
    items = []
    for script, path, want in MANIFESTS:
        done = subprocess.run(["/usr/bin/python3", str(REPO / script), "verify", str(ROOT / path)], cwd="/tmp", env=env,
                              capture_output=True, text=True)
        got = (done.stdout.strip().splitlines() or [""])[-1]
        items.append({"manifest": path, "expected": want, "printed": got, "exit_code": done.returncode,
                      "verified": done.returncode == 0 and got == want})
    return {"verified": all(i["verified"] for i in items), "items": items}


class PreconditionFailed(RuntimeError):
    """The executor treats any exception before the first removal as a refusal."""


def preconditions(checks, after=False):
    if not after:
        locks = sorted(p.name for p in (ROOT / "state").glob("*live*.lock"))
        checks["live_locks"] = locks
        if locks:
            raise PreconditionFailed("a live lock exists: %s" % locks)
        p7e_scripts = REPO / "simulation/px4_gazebo_p7e/scripts"
        if str(p7e_scripts) not in sys.path:
            sys.path.insert(0, str(p7e_scripts))
        import p7e_host  # noqa: E402  (frozen FC-2 check, used unchanged)
        found = p7e_host.simulator_processes(p7e_host.pattern())  # raises ProcessCheckFailed if pgrep fails
        checks["simulator_processes"] = found
        if found:
            raise PreconditionFailed("simulator processes are running (FC-2)")
    checks["manifests"] = verify_manifests()
    if not checks["manifests"]["verified"]:
        raise PreconditionFailed("a manifest does not verify")


def after_check(runs_root, removed_runs):
    """For removed P7-E flights: the frozen classifier, on a view of the run, must reproduce its flight record."""
    flights = [r for r in removed_runs if "-p7e-" in r]
    if not flights:
        return {"passed": True, "not_applicable": "no P7-E flight among the removed runs; the retained records are "
                                                  "checked unchanged instead"}
    sample = flights if len(flights) <= 3 else [flights[0], flights[len(flights) // 2], flights[-1]]
    spec = importlib.util.spec_from_file_location("p7e_report_accepted", REPO / "simulation/px4_gazebo_p7e/analysis/p7e_report.py")
    rep7 = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rep7)  # the accepted report code, unchanged
    rows = []
    with tempfile.TemporaryDirectory(prefix="retention-h9-") as work:
        for n, name in enumerate(sample):
            run = Path(runs_root) / name
            record, error = rep7.reclassify(run, Path(work) / str(n))
            want = json.loads((run / "p7/flight-record.json").read_text())
            rows.append({"run": name, "reproduced": record is not None and record == want, "error": error})
    return {"passed": all(r["reproduced"] for r in rows), "sample": rows, "of": len(flights)}
