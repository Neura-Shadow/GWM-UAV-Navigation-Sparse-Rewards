"""P7 Selective Dependency Backup, supplementary Part E: `venv-px4` (inventory erratum §9; M3 report §5).

Operator: "Authorize venv backup" (2026-11-21).

It reuses the executed procedure of `p7_backup.py` unchanged: its `write_tar`, `verify_tree`, drift check, logging
and stop path.
* **Preconditions:**
  * no simulator or runtime process;
  * the four manifests verify;
  * C: has enough free space;
  * the main backup's receipt and SHA256SUMS are unchanged;
  * no Part E output exists yet;
  * the approved per-file inventory is unchanged.
* **Drift check:** the full inventory is re-run into the scratch directory. The main set must still be unchanged,
  and `venv-px4` must still have its inventoried tree digest.
* **Copy:** one uncompressed pax tar, `E10-venv-px4.tar`, with its read-back hash checked.
* **Restore check:** extraction into a temporary WSL directory. Every inventoried file, link and directory must match,
  with nothing extra. The directory is then removed.
* **Records:** `SHA256SUMS-part-e` and `backup-receipt-part-e.json`, both with exclusive create.

The main backup's files are never modified, and `~/uav_autonomy` is only read.
Usage: p7_backup_part_e.py <scratch-dir>
"""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

_spec = importlib.util.spec_from_file_location("p7_backup", Path(__file__).resolve().parent / "p7_backup.py")
base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(base)

GROUP = "O-venv-px4"
TAR = "E10-venv-px4.tar"
MAIN_RECEIPT_SHA256 = "dce926264a0320cf060eb0af6c0d5b81beacbaf5f434d3978cb21e952b2e8184"
MAIN_SUMS_SHA256 = "2820239062bed1d2192b7a1c62cc0218ebc0ec2eba28dcdee6b51e7a804925fa"
VERIFY = Path.home() / "p7-backup-verify-20261121-part-e"
OUTPUTS = (TAR, "SHA256SUMS-part-e", "backup-receipt-part-e.json", "inventory/part-e")


def preconditions(approved):
    out = subprocess.run(["pgrep", "-af", base.SIM], capture_output=True, text=True).stdout.splitlines()
    procs = [line for line in out if "pgrep" not in line and "p7_backup" not in line]
    if procs:
        base.stop("simulator or runtime processes present: " + "; ".join(procs))
    env = dict(os.environ, GWM_PX4_MC_TRACE="px4-mc-causal-trace-v1", PYTHONDONTWRITEBYTECODE="1")
    shas = {}
    for script, manifest, prefix in base.MANIFESTS:
        r = subprocess.run(["/usr/bin/python3", str(base.REPO / script), "verify", str(base.HOME / "state" / manifest)],
                           capture_output=True, text=True, cwd="/tmp", env=env)
        if r.returncode != 0 or not r.stdout.strip().startswith(prefix):
            base.stop(f"{manifest} does not verify: {r.returncode} {r.stderr[-300:]}")
        shas[manifest] = r.stdout.strip()
    need = approved["groups"][GROUP]["bytes"] + 20e9
    free = shutil.disk_usage("/mnt/c").free
    if free < need:
        base.stop("C: free %.1f GB < %.1f GB" % (free / 1e9, need / 1e9))
    if base.sha_file(base.DEST / "backup-receipt.json") != MAIN_RECEIPT_SHA256 \
            or base.sha_file(base.DEST / "SHA256SUMS") != MAIN_SUMS_SHA256:
        base.stop("the main backup's receipt or SHA256SUMS changed")
    if any((base.DEST / o).exists() for o in OUTPUTS) or VERIFY.exists():
        base.stop("a Part E output or the verification directory already exists")
    if base.sha_file(base.FULL) != base.FULL_SHA256:
        base.stop("approved per-file inventory changed")
    base.log("preconditions passed; C: free %.1f GB" % (free / 1e9))
    return shas, free


def main():
    scratch = Path(sys.argv[1])
    started = time.time()
    approved = json.loads(base.SUMMARY.read_text())
    shas, free = preconditions(approved)
    drift = base.drift_check(scratch, approved)  # stops on any drift of the main set
    now = json.loads((scratch / "drift-summary.json").read_text())["groups"][GROUP]["tree_sha256"]
    if now != approved["groups"][GROUP]["tree_sha256"]:
        base.stop("venv-px4 drifted from the inventory: %s != %s" % (now, approved["groups"][GROUP]["tree_sha256"]))
    base.log("venv-px4 unchanged since the inventory: %s" % now[:16])
    expected = {}
    with open(base.FULL, encoding="utf-8") as stream:
        for line in stream:
            e = json.loads(line)
            if e["group"] == GROUP:
                expected[e["path"]] = e
    members = sorted({path.split("/")[0] for path in expected})
    if members != ["venv-px4"]:
        base.stop("unexpected roots for %s: %s" % (GROUP, members))
    tar = {"group": GROUP, **base.write_tar(TAR, base.HOME, members, scratch)}
    copies = base.DEST / "inventory/part-e"
    copies.mkdir(parents=True)
    for src in (Path(__file__).resolve(), base.REPO / "docs/v3_sim_p7_m0_dependency_inventory.md"):
        shutil.copyfile(src, copies / src.name)
        if base.sha_file(copies / src.name) != base.sha_file(src):
            base.stop("copy differs: " + src.name)
    VERIFY.mkdir()
    try:
        target = VERIFY / "uav_autonomy"
        target.mkdir()
        r = subprocess.run(["tar", "--extract", "--numeric-owner", "--file", str(base.DEST / TAR), "-C", str(target)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            base.stop("extract failed: " + r.stderr[-500:])
        problems = base.verify_tree(target, expected)
        verified = {"entries_checked": len(expected), "problems": problems[:50], "problem_count": len(problems),
                    "passed": not problems}
        base.log("restore verification: %d entries, %d problems" % (len(expected), len(problems)))
    finally:
        if str(VERIFY).startswith(str(Path.home() / "p7-backup-verify-")) and "uav_autonomy" not in VERIFY.name:
            def writable(func, path, _info):
                os.chmod(path, 0o700)
                func(path)
            shutil.rmtree(VERIFY, onerror=writable)
            base.log("removed the temporary verification directory " + str(VERIFY))
    if not verified["passed"]:
        base.stop("restore verification failed: " + json.dumps(problems[:10]))
    sums = [f"{base.sha_file(base.DEST / TAR)}  {TAR}"]
    sums += [f"{base.sha_file(p)}  {p.relative_to(base.DEST).as_posix()}" for p in sorted(copies.iterdir())]
    with open(base.DEST / "SHA256SUMS-part-e", "x", encoding="utf-8") as stream:
        stream.write("\n".join(sums) + "\n")
    receipt = {
        "contract": "p7-selective-backup-receipt-part-e-v1",
        "decision": "inventory erratum §9: venv-px4 is Essential (the accepted evaluators run under it); supplementary "
                    "Part E authorized 2026-11-21 (\"Authorize venv backup\")",
        "supplements": {"receipt": "backup-receipt.json", "receipt_sha256": MAIN_RECEIPT_SHA256,
                        "sha256sums_sha256": MAIN_SUMS_SHA256},
        "source_root": str(base.HOME), "destination": "C:\\GWM-UAV-Backup\\p7-selective-20261121",
        "started_unix_s": int(started), "completed_unix_s": int(time.time()), "duration_s": round(time.time() - started, 1),
        "preconditions": {"manifests_verified": shas, "c_free_bytes_before": free, "no_runtime_processes": True},
        "drift_check": {"passed": True, "venv_px4_tree_sha256": now, **drift},
        "tar": tar, "restore_verification": verified, "compression": "none", "sources_modified": False,
        "external_symlinks": "venv-px4/bin/python* point to /usr/bin/python3.12 (system Python; its packages are in the apt cache)",
        "tool_sha256": base.sha_file(Path(__file__).resolve()),
    }
    with open(base.DEST / "backup-receipt-part-e.json", "x", encoding="utf-8") as stream:
        stream.write(json.dumps(receipt, indent=1, sort_keys=True) + "\n")
    base.log("part E receipt written; %.3f GB in %.0f s" % (tar["bytes"] / 1e9, receipt["duration_s"]))
    print(json.dumps({"receipt_sha256": base.sha_file(base.DEST / "backup-receipt-part-e.json"),
                      "sha256sums_sha256": base.sha_file(base.DEST / "SHA256SUMS-part-e"), "tar_sha256": tar["sha256"],
                      "tar_bytes": tar["bytes"]}))


if __name__ == "__main__":
    main()
