"""P7 M0 (read-only audit tool, not P7 code): exact dependency inventory of ~/uav_autonomy for the revised D5
(Selective Dependency Backup). It classifies every entry into the four D5 categories and hashes every regular
file with SHA-256.

It never writes, moves, deletes or compresses anything it inventories:
* files are opened read-only with O_NOATIME;
* symlinks are recorded as links and never followed;
* its only outputs are two new files, opened with exclusive create.

Usage: p7_m0_inventory.py <summary.json> <files.jsonl>
"""
import hashlib
import json
import os
import stat
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HOME = Path.home() / "uav_autonomy"
P3, P4, P5, P6 = ("1e205feca482e11f10d5742fdac0474af3cd5689d4ec9c74d7f38a81eb39510c",
                  "c110bd30be22fb43e596cf1f4b01397cb97145138f8912d6535a7dc4c15da42b",
                  "2becfbbb794006833d0a224b819db702135b0a0c9ff25da6577a7c1dd5238632",
                  "222927752a507fdb2fb0ef13f44d6ce8163d50b7d22f405ebbf27df6c21bfa41")


def gate_files(phase, sha, gates, auth):
    out = []
    for g in gates:
        out += [auth.format(g=g), f"{phase}-gate-{g}-{sha}.consumed.json", f"{phase}-gate-{g}-{sha}.result.json"]
    return out


STATE_ESSENTIAL = (
    ["p3-candidate-087722b.json", "p4-candidate-50d5002.json", "p5-candidate-40a109b.json", "p6-candidate-8e29f3f.json",
     "p0-built.json", "p2-built.json", "p3-built.json", "p3-dds-agent-built.json", "p4-built.json", "p5-built.json",
     "px4-mc-trace-built.json", "qgc-unattended-built.json",
     "p3-policy-A-approval-087722b.json", "p4-policy-A-approval-50d5002.json", "p5-policy-A-approval-40a109b.json",
     "p6-policy-A-approval-c08bfc8.json"]
    + gate_files("p3", P3, "ABC", "p3-auth-{g}-" + P3[:12] + ".json")
    + gate_files("p4", P4, "ABSC", "p4-auth-{g}-" + P4[:12] + ".json")
    + gate_files("p5", P5, "ABSC", "p5-auth-{g}-" + P5[:12] + ".json")
    + gate_files("p6", P6, "ABSC", "p6-gate-{g}-authorization-" + P6[:12] + ".json"))

MC = "mc_trace_builds/ecdf4eaadce421a2e940bf8ed43878c9ac41cd0db07ab38ae01c530e645c0bf9"
QGC_BUNDLE = "qgc_closure_v1/bundles/829942834b71bbd7f30967cb32151e7a32bc1a7fbb9609fd15e2f4dde3e46d03"
WS = {"p2_ws": "0d994904f2e43310219e4505c246cc8ecd2f95f2467ed3463a5308c124840456",
      "p3_ws": "c5d374137f6556df710c310f8eec994c778c297174b5407866c0d582f820c9b6",
      "p4_ws": "c4bd5a55abd997207f74fc019759f820ef97b8d07dd2ab103c1e54b44faf2486",
      "p5_ws": "e7a59f3506ea655dfc558be9119aba6185d3b13fed8147497d1849317f4c99e2"}

# (id, category, paths relative to HOME, why it is needed)
GROUPS = [
    ("E1-state", "essential", ["state/" + f for f in STATE_ESSENTIAL],
     "accepted P3-P6 manifests; the build receipts every verify and runner reads; accepted Policy A approvals; "
     "authorization, consumption marker and result of every accepted gate"),
    ("E2-mc-benchmark-run", "essential", ["runs/20261025T051926Z-mc-trace-benchmark-e6000074"],
     "bound by every manifest's runtime.mc_benchmark; verify reads its summary.json"),
    ("E3-px4-mc-trace-build", "essential", [MC],
     "the flight PX4 (receipt px4-mc-trace-built.json): binary, rootfs/gz_env.sh, overlay sources and generated "
     "headers that verify hashes; verify runs git rev-parse/diff/submodule status in this tree"),
    ("E4-px4-upstream", "essential", ["upstream/PX4-Autopilot"],
     "baseline PX4 binary, pinned gz models/worlds (45 files verify hashes), gz plugins and server.config that the "
     "runner loads; verify runs git commands in this tree"),
    ("E5-qgc-bundle", "essential", [QGC_BUNDLE, "qgc_closure_v1/receipts",
                                    "qgc_closure_v1/qt-tests/qt-tests-owner-0d1c2d27-20261025T050446Z"],
     "the accepted unattended QGC ELF (receipt qgc-unattended-built.json) and the receipts it references"),
    ("E6-qgc-stock-tree", "essential", ["qgc-v5.1.4"],
     "symlink target: 766 links in the accepted QGC bundle point into this tree"),
    ("E7-dds-agent", "essential", ["p3_agent/d6bad178d69ddbe5", "dds-install"],
     "the accepted MicroXRCEAgent binary (receipt p3-dds-agent-built.json) and its library_dir"),
    ("E8-ros-overlay", "essential", ["ros_ws"],
     "px4_msgs overlay sourced by run_p3/run_p4/run_p5/run_p6.sh ($ROS_WS/install/setup.bash)"),
    ("E9-workspaces", "essential", [f"{k}/{v}" for k, v in WS.items()],
     "the installed P2/P3/P4/P5 packages named by p2/p3/p4/p5-built.json"),
    ("R1-p4-S2-gateC", "reference", ["runs/20261030T144136Z-p4-flight-S2-abeea2d5"],
     "accepted P4 Gate C S2; P5 policy_replay source (p5_validation.yaml); P7 M4 parity reference for P4"),
    ("R2-p4-S2-gateB", "reference", ["runs/20261030T141506Z-p4-flight-S2-9b8ef2b0"],
     "accepted P4 Gate B S2; P5 policy_replay source; second P4 parity sample"),
    ("R3-p5-S2-gateC", "reference", ["runs/20261106T132442Z-p5-flight-S2-184a5a5e"],
     "accepted P5 Gate C S2; the P6 offline-suite recorded run (Gate A row 4, tests); P7 M4 parity reference for P5"),
    ("R4-p5-S2-gateB", "reference", ["runs/20261106T130044Z-p5-flight-S2-679dd050"],
     "accepted P5 Gate B S2; second P5 parity sample"),
    ("F2-p5-S6-run", "future", ["runs/20261106T131246Z-p5-flight-S6-d907451a"],
     "accepted P5 S6; used only by P6 evaluator regression tests (skip when absent)"),
]


def p6_real_runs():
    out = []
    for g in "ABSC":
        result = json.loads((HOME / "state" / f"p6-gate-{g}-{P6}.result.json").read_text())
        out += [Path(r["run"]).relative_to(HOME).as_posix() for r in result["runs"] if r.get("run")]
    return sorted(set(out))


def p6_test_dry_runs():
    prefixes = ["20261114T034747Z-p6dry-S-5p-", "20261114T083341Z-p6dry-C-1-", "20261114T084020Z-p6dry-C-3-",
                "20261114T084352Z-p6dry-C-4-", "20261114T084636Z-p6dry-C-5-", "20261114T110631Z-p6dry-C-5-",
                "20261114T123723Z-p6dry-C-5-", "20261115T032134Z-p6dry-C-5-"]
    names = sorted(os.listdir(HOME / "runs"))
    return ["runs/" + n for p in prefixes for n in names if n.startswith(p)]


GROUPS += [("F1-p6-real-gate-runs", "future", p6_real_runs(),
            "the 27 run directories of the accepted real P6 Gates A/B/S/C; needed only if P7-B is approved"),
           ("F3-p6-test-dry-runs", "future", p6_test_dry_runs(),
            "P6 development dry runs replayed by P6 evaluator regression tests (skip when absent)")]


def walk(rel):
    """Yield (relpath, lstat) for rel and everything under it, never following symlinks."""
    root = HOME / rel
    st = os.lstat(root)
    if not stat.S_ISDIR(st.st_mode):
        yield rel, st
        return
    yield rel, st
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        dirnames.sort()
        for name in sorted(dirnames + filenames):
            p = Path(dirpath) / name
            yield p.relative_to(HOME).as_posix(), os.lstat(p)


def sha256(rel):
    flags = os.O_RDONLY | getattr(os, "O_NOATIME", 0)
    try:
        fd = os.open(HOME / rel, flags)
    except PermissionError:
        fd = os.open(HOME / rel, os.O_RDONLY)
    h = hashlib.sha256()
    try:
        with os.fdopen(fd, "rb", buffering=0) as stream:
            while chunk := stream.read(8 << 20):
                h.update(chunk)
    except OSError as error:
        return rel, "unreadable: " + type(error).__name__
    return rel, h.hexdigest()


def entry(rel, st):
    if stat.S_ISLNK(st.st_mode):
        target = os.readlink(HOME / rel)
        resolved = os.path.realpath(HOME / rel)
        return {"path": rel, "type": "symlink", "target": target, "resolved": resolved,
                "resolved_exists": os.path.exists(resolved)}
    if stat.S_ISDIR(st.st_mode):
        return {"path": rel, "type": "dir"}
    if stat.S_ISREG(st.st_mode):
        return {"path": rel, "type": "file", "size": st.st_size}
    return {"path": rel, "type": "other", "mode": oct(st.st_mode)}


def main():
    summary_path, files_path = Path(sys.argv[1]), Path(sys.argv[2])
    started = time.time()
    claimed, records = {}, []
    for gid, category, rels, why in GROUPS:
        for rel in rels:
            for path, st in walk(rel):
                if path in claimed:
                    raise SystemExit(f"{path} claimed by {claimed[path]} and {gid}")
                claimed[path] = gid
                records.append((gid, category, entry(path, st)))
    def owner(rel):
        """The claiming group of rel or of its nearest claimed ancestor, else None."""
        parts = rel.split("/")
        for i in range(len(parts), 0, -1):
            gid = claimed.get("/".join(parts[:i]))
            if gid:
                return gid
        return None

    # Everything not claimed is optional historical evidence, grouped by top-level entry, with one group per run
    # directory and per second-level directory of the larger trees.
    split = ("runs", "p2_ws", "p5_ws", "upstream", "qgc_closure_v1", "p5_training", "mc_trace_builds", "qgc-v5.1.4")
    for top in sorted(os.listdir(HOME)):
        for path, st in walk(top):
            if owner(path):
                continue
            parts = path.split("/")
            gid = "O-" + ("/".join(parts[:2]) if parts[0] in split and len(parts) > 1 else parts[0])
            records.append((gid, "optional", entry(path, st)))
    files = [e["path"] for _, _, e in records if e["type"] == "file"]
    hashes = {}
    with ProcessPoolExecutor(max_workers=8) as pool:
        for rel, digest in pool.map(sha256, files, chunksize=64):
            hashes[rel] = digest
    groups = {}
    for gid, category, e in records:
        if e["type"] == "file":
            e["sha256"] = hashes[e["path"]]
        g = groups.setdefault(gid, {"category": category, "files": 0, "symlinks": 0, "dirs": 0, "other": 0, "bytes": 0,
                                    "external_symlinks": {}, "dangling_symlinks": 0, "_lines": []})
        g[{"file": "files", "symlink": "symlinks", "dir": "dirs"}.get(e["type"], "other")] += 1
        g["bytes"] += e.get("size", 0)
        if e["type"] == "symlink":
            if not e["resolved_exists"]:
                g["dangling_symlinks"] += 1
            res = e["resolved"]
            if res.startswith(str(HOME) + "/"):
                target_owner = owner(Path(res).relative_to(HOME).as_posix())
                key = target_owner or "optional (unclaimed ~/uav_autonomy)"
            else:
                target_owner, key = None, "outside ~/uav_autonomy: " + "/".join(res.split("/")[:4])
            if category == "optional" and target_owner is None and res.startswith(str(HOME) + "/"):
                same = res.startswith(str(HOME / gid[2:]) + "/") or res == str(HOME / gid[2:])
                if not same:
                    g["external_symlinks"][key] = g["external_symlinks"].get(key, 0) + 1
            elif target_owner != gid:
                g["external_symlinks"][key] = g["external_symlinks"].get(key, 0) + 1
        g["_lines"].append("\t".join([e["type"], e["path"], str(e.get("size", "")), e.get("sha256") or e.get("target", "")]))
    with files_path.open("x", encoding="utf-8") as stream:
        for gid, category, e in records:
            stream.write(json.dumps({"group": gid, "category": category, **e}, sort_keys=True) + "\n")
    for gid, g in groups.items():
        g["tree_sha256"] = hashlib.sha256("\n".join(sorted(g.pop("_lines"))).encode()).hexdigest()
    totals = {}
    for g in groups.values():
        t = totals.setdefault(g["category"], {"groups": 0, "files": 0, "symlinks": 0, "bytes": 0})
        t["groups"] += 1
        t["files"] += g["files"]
        t["symlinks"] += g["symlinks"]
        t["bytes"] += g["bytes"]
    small = {gid: [{k: e[k] for k in ("path", "size", "sha256") if k in e} for g2, _, e in records
                   if g2 == gid and e["type"] == "file"]
             for gid, g in groups.items() if g["category"] != "optional" and g["files"] <= 400}
    out = {"contract": "p7-m0-dependency-inventory-v1", "root": str(HOME), "generated_unix_s": int(started),
           "duration_s": round(time.time() - started, 1), "read_only": True,
           "files_inventory": {"path": str(files_path), "sha256": None, "lines": len(records)},
           "groups": {gid: {**g, "paths": next((r for i, c, r, w in GROUPS if i == gid), None),
                            "why": next((w for i, c, r, w in GROUPS if i == gid), "not required by P7-A")}
                      for gid, g in sorted(groups.items())},
           "totals": totals, "per_file": small}
    out["files_inventory"]["sha256"] = hashlib.sha256(files_path.read_bytes()).hexdigest()
    with summary_path.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(out, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"duration_s": out["duration_s"], "totals": totals}))


if __name__ == "__main__":
    main()
