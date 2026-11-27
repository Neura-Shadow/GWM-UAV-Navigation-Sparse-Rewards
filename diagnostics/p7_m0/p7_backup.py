"""P7 Selective Dependency Backup (revised D5): one execution of the approved procedure, from inventory report §7.

Operator: set and destination approved 2026-11-21 ("Approve set and C:"); execution authorized 2026-11-21
("Authorize the backup"). Attempt 1 stopped on the apt-cache tar (see write_tar); its folder was renamed to
p7-selective-20261121.attempt1-stopped and this fixed version re-run from the start, authorized 2026-11-21
("Authorize re-run").

The sources under ~/uav_autonomy and /var/cache/apt/archives are only read:
* tar runs with -C (and --atime-preserve=system for ~/uav_autonomy), stores symlinks as links, and does not
  compress;
* nothing is moved, deleted, overwritten or uploaded.

It writes only:
* the new destination directory (exclusive create);
* its own scratch drift-check outputs;
* a temporary restore-verification directory in WSL. That directory is removed after the check; it is this
  tool's own copy, not evidence.

Any failed check stops the run.
Usage: p7_backup.py <scratch-dir>
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path("/mnt/d/GWM-UAV-Navigation-Sparse-Rewards")
HOME = Path.home() / "uav_autonomy"
APT = Path("/var/cache/apt/archives")
DEST = Path("/mnt/c/GWM-UAV-Backup/p7-selective-20261121")
VERIFY = Path.home() / "p7-backup-verify-20261121"
SUMMARY = REPO / "docs/evidence/v3_sim_p7_m0_dependency_inventory.json"
SYSTEM = REPO / "docs/evidence/v3_sim_p7_m0_system_dependencies.json"
FULL = REPO / "diagnostics/p7_m0/results/p7_m0_inventory_files.jsonl"
FULL_SHA256 = "2303055a49cdd54ff722ff1b5875c149e931d629a87893b151e81e9e4f54762c"
GROUPS = ["E1-state", "E2-mc-benchmark-run", "E3-px4-mc-trace-build", "E4-px4-upstream", "E5-qgc-bundle",
          "E6-qgc-stock-tree", "E7-dds-agent", "E8-ros-overlay", "E9-workspaces",
          "R1-p4-S2-gateC", "R2-p4-S2-gateB", "R3-p5-S2-gateC", "R4-p5-S2-gateB"]
MANIFESTS = [("simulation/px4_gazebo/scripts/p3_manifest.py", "p3-candidate-087722b.json", "1e205fec"),
             ("simulation/px4_gazebo_p4/scripts/p4_manifest.py", "p4-candidate-50d5002.json", "c110bd30"),
             ("simulation/px4_gazebo_p5/scripts/p5_manifest.py", "p5-candidate-40a109b.json", "2becfbbb"),
             ("simulation/px4_gazebo_p6/scripts/p6_manifest.py", "p6-candidate-8e29f3f.json", "22292775")]
SIM = r"px4|gz sim|QGroundControl|MicroXRCE|parameter_bridge|p6_runner|run_p6_gates|p5_runner|p4_runner"


def log(*parts):
    print(time.strftime("%H:%M:%S"), *parts, flush=True)


def stop(reason):
    log("STOP:", reason)
    raise SystemExit(1)


def sha_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        while chunk := stream.read(8 << 20):
            h.update(chunk)
    return h.hexdigest()


def preconditions():
    out = subprocess.run(["pgrep", "-af", SIM], capture_output=True, text=True).stdout.splitlines()
    procs = [line for line in out if "pgrep" not in line and "p7_backup" not in line]
    if procs:
        stop("simulator or runtime processes present: " + "; ".join(procs))
    env = dict(os.environ, GWM_PX4_MC_TRACE="px4-mc-causal-trace-v1", PYTHONDONTWRITEBYTECODE="1")
    shas = {}
    for script, manifest, prefix in MANIFESTS:
        r = subprocess.run(["/usr/bin/python3", str(REPO / script), "verify", str(HOME / "state" / manifest)],
                           capture_output=True, text=True, cwd="/tmp", env=env)
        sha = r.stdout.strip()
        if r.returncode != 0 or not sha.startswith(prefix):
            stop(f"{manifest} does not verify: {r.returncode} {sha} {r.stderr[-300:]}")
        shas[manifest] = sha
    free = shutil.disk_usage("/mnt/c").free
    if free < 32.6e9:
        stop(f"C: free {free / 1e9:.1f} GB < 32.6 GB")
    if DEST.exists() or VERIFY.exists():
        stop("destination or verification directory already exists")
    if sha_file(FULL) != FULL_SHA256:
        stop("approved per-file inventory changed")
    log("preconditions passed; C: free %.1f GB" % (free / 1e9))
    return shas, free


def drift_check(scratch, approved):
    summary, files = scratch / "drift-summary.json", scratch / "drift-files.jsonl"
    r = subprocess.run(["/usr/bin/python3", str(REPO / "diagnostics/p7_m0/p7_m0_inventory.py"), str(summary), str(files)],
                       capture_output=True, text=True, cwd="/tmp", env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    if r.returncode != 0:
        stop("drift inventory failed: " + r.stderr[-500:])
    now = json.loads(summary.read_text())
    drift = {g: (approved["groups"][g]["tree_sha256"], now["groups"][g]["tree_sha256"]) for g in GROUPS
             if approved["groups"][g]["tree_sha256"] != now["groups"][g]["tree_sha256"]}
    system = json.loads(SYSTEM.read_text())
    debs = sorted(APT.glob("*.deb"))
    current = {d.name: sha_file(d) for d in debs}
    expected = {Path(f["path"]).name: f["sha256"] for f in system["apt_cache"]["files"]}
    if current != expected:
        drift["S1-apt-cache"] = (len(expected), len(current))
    if drift:
        stop("drift against the approved inventory: " + json.dumps(drift))
    log("drift check passed: %d groups and %d debs unchanged" % (len(GROUPS), len(expected)))
    return {"groups": {g: now["groups"][g]["tree_sha256"] for g in GROUPS}, "debs": len(expected)}


def write_tar(name, cwd, members, scratch, preserve_atime=True):
    """tar stream -> new file; write-time SHA-256 of the stream; then an independent read-back hash.

    tar's stderr goes to a scratch file, not a pipe: a pipe that is read only after stdout reaches EOF deadlocks
    once tar writes more than 64 KB of diagnostics (attempt 1, 2026-11-21). --atime-preserve=system opens with
    O_NOATIME, which the kernel refuses (EPERM) for files the caller does not own, so it is used only for the
    operator-owned ~/uav_autonomy groups, not for the root-owned apt cache, whose access times are not evidence.
    """
    target = DEST / name
    listing = scratch / (name + ".members")
    listing.write_text("\n".join(members) + "\n")
    cmd = ["tar", "--create", "--format=pax", "--numeric-owner"] + (["--atime-preserve=system"] if preserve_atime
                                                                    else []) + [
           "--file=-", "-C", str(cwd), "--files-from", str(listing)]
    h, size = hashlib.sha256(), 0
    with open(target, "xb") as out, open(scratch / (name + ".stderr"), "xb") as errfile:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=errfile)
        while chunk := proc.stdout.read(8 << 20):
            h.update(chunk)
            out.write(chunk)
            size += len(chunk)
        proc.wait()
        err = (scratch / (name + ".stderr")).read_bytes().decode(errors="replace")
        if proc.returncode != 0 or err.strip():
            stop(f"tar {name} failed: rc={proc.returncode} {err[-500:]}")
        out.flush()
        os.fsync(out.fileno())
    readback = sha_file(target)
    if readback != h.hexdigest():
        stop(f"read-back mismatch for {name}")
    log(f"{name}: {size / 1e9:.3f} GB, sha256 {readback[:16]} (read-back matches)")
    return {"tar": name, "bytes": size, "sha256": readback, "write_time_sha256": h.hexdigest(), "members": len(members),
            "atime_preserved": preserve_atime}


def expected_entries():
    by_group = {g: {} for g in GROUPS}
    with open(FULL, encoding="utf-8") as stream:
        for line in stream:
            e = json.loads(line)
            if e["group"] in by_group:
                by_group[e["group"]][e["path"]] = e
    return by_group


def verify_tree(root, expected):
    """Every expected file and symlink present with equal hash, size or target; no extra file or link; extra
    directories only as ancestors created by extraction."""
    problems, seen = [], set()
    for path, e in expected.items():
        p = root / path
        if e["type"] == "file":
            if not p.is_file() or p.is_symlink():
                problems.append(("missing_file", path))
            elif p.stat().st_size != e["size"] or sha_file(p) != e["sha256"]:
                problems.append(("file_differs", path))
        elif e["type"] == "symlink":
            if not p.is_symlink() or os.readlink(p) != e["target"]:
                problems.append(("symlink_differs", path))
        elif e["type"] == "dir":
            if not p.is_dir() or p.is_symlink():
                problems.append(("missing_dir", path))
        seen.add(path)
    ancestors = {"/".join(k.split("/")[:i]) for k in expected for i in range(1, len(k.split("/")))}
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in dirnames + filenames:
            rel = (Path(dirpath) / name).relative_to(root).as_posix()
            if rel not in seen and rel not in ancestors:
                problems.append(("extra", rel))
    return problems


def main():
    scratch = Path(sys.argv[1])
    started = time.time()
    approved = json.loads(SUMMARY.read_text())
    shas, free = preconditions()
    drift = drift_check(scratch, approved)
    DEST.parent.mkdir(exist_ok=True)
    DEST.mkdir()  # exclusive: fails if it exists
    expected = expected_entries()
    tars = []
    for g in GROUPS:
        roots = approved["groups"][g]["paths"]
        tars.append({"group": g, **write_tar(g + ".tar", HOME, roots, scratch)})
    system = json.loads(SYSTEM.read_text())
    debs = sorted(Path(f["path"]).name for f in system["apt_cache"]["files"])
    tars.append({"group": "S1-apt-cache", **write_tar("S1-apt-cache.tar", APT, debs, scratch, preserve_atime=False)})
    inventory = DEST / "inventory"
    inventory.mkdir()
    for src in (SUMMARY, SYSTEM, FULL, REPO / "docs/v3_sim_p7_m0_dependency_inventory.md",
                REPO / "diagnostics/p7_m0/p7_m0_inventory.py", REPO / "diagnostics/p7_m0/p7_backup.py"):
        shutil.copyfile(src, inventory / src.name)
        if sha_file(inventory / src.name) != sha_file(src):
            stop("inventory copy differs: " + src.name)
    log("inventory files copied")
    # Restore verification in a new WSL scratch directory, never over ~/uav_autonomy.
    VERIFY.mkdir()
    results = {}
    try:
        for t in tars:
            target = VERIFY / ("apt-cache" if t["group"] == "S1-apt-cache" else "uav_autonomy")
            target.mkdir(exist_ok=True)
            r = subprocess.run(["tar", "--extract", "--numeric-owner", "--file", str(DEST / t["tar"]), "-C", str(target)],
                               capture_output=True, text=True)
            if r.returncode != 0:
                stop(f"extract {t['tar']} failed: {r.stderr[-500:]}")
        root = VERIFY / "uav_autonomy"
        allexp = {}
        for g in GROUPS:
            allexp.update(expected[g])
            results[g] = {"entries": len(expected[g])}
        problems = verify_tree(root, allexp)
        apt_expected = {Path(f["path"]).name: f for f in system["apt_cache"]["files"]}
        apt_root = VERIFY / "apt-cache"
        for name, f in apt_expected.items():
            p = apt_root / name
            if not p.is_file() or p.stat().st_size != f["size"] or sha_file(p) != f["sha256"]:
                problems.append(("deb_differs", name))
        extra_debs = sorted(set(os.listdir(apt_root)) - set(apt_expected))
        problems += [("extra", "apt-cache/" + n) for n in extra_debs]
        verified = {"entries_checked": len(allexp) + len(apt_expected), "problems": problems[:50],
                    "problem_count": len(problems), "passed": not problems}
        log("restore verification: %d entries, %d problems" % (verified["entries_checked"], len(problems)))
    finally:
        if str(VERIFY).startswith(str(Path.home() / "p7-backup-verify-")) and "uav_autonomy" not in VERIFY.name:
            def writable(func, path, _info):
                os.chmod(path, 0o700)
                func(path)
            shutil.rmtree(VERIFY, onerror=writable)
            log("removed the temporary verification directory " + str(VERIFY))
    if not verified["passed"]:
        stop("restore verification failed: " + json.dumps(problems[:10]))
    sums = []
    for p in sorted(DEST.rglob("*")):
        if p.is_file():
            sums.append(f"{sha_file(p)}  {p.relative_to(DEST).as_posix()}")
    (DEST / "SHA256SUMS").write_text("\n".join(sums) + "\n")
    receipt = {
        "contract": "p7-selective-backup-receipt-v1",
        "decision": "D5 revised to a Selective Dependency Backup; set and destination approved 2026-11-21 "
                    "(\"Approve set and C:\"); execution authorized 2026-11-21 (\"Authorize the backup\")",
        "source_root": str(HOME), "apt_cache": str(APT), "destination": "C:\\GWM-UAV-Backup\\p7-selective-20261121",
        "started_unix_s": int(started), "completed_unix_s": int(time.time()), "duration_s": round(time.time() - started, 1),
        "preconditions": {"manifests_verified": shas, "c_free_bytes_before": free, "no_runtime_processes": True},
        "drift_check": {"passed": True, **drift},
        "approved_inventory": {"summary_sha256": sha_file(SUMMARY), "system_sha256": sha_file(SYSTEM),
                               "files_jsonl_sha256": FULL_SHA256},
        "tars": tars, "total_bytes": sum(t["bytes"] for t in tars),
        "restore_verification": verified, "compression": "none", "sources_modified": False,
        "tool_sha256": sha_file(REPO / "diagnostics/p7_m0/p7_backup.py"),
    }
    with open(DEST / "backup-receipt.json", "x", encoding="utf-8") as stream:
        stream.write(json.dumps(receipt, indent=1, sort_keys=True) + "\n")
    log("receipt written; total %.2f GB in %.0f s" % (receipt["total_bytes"] / 1e9, receipt["duration_s"]))
    print(json.dumps({"receipt_sha256": sha_file(DEST / "backup-receipt.json"),
                      "sha256sums_sha256": sha_file(DEST / "SHA256SUMS"), "total_bytes": receipt["total_bytes"]}))


if __name__ == "__main__":
    main()
