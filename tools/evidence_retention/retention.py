"""Evidence Retention Manager: the fail-closed removal executor (P7-E cleanup proposal v3 §5; decisions C-5, C-6).

It separates evidence generation from permanent retention. A run is always recorded completely (D4-A). After its
evaluation and integrity verification, only regenerable or re-derivable raw items may be removed. Each removal needs
a file-level authorization and the operator's phrase. Everything else is kept.

**Removable items, relative to a run directory:** exactly `sensors/depth.bin`, and regular files under `tmp/`.
Nothing else can be listed. This keeps the ULog, local maps, decisions, the policy and recovery ledgers, the
evaluations, the flight records, the depth index and every other record. Directories are never removed.

**Flow:**
1. `prepare` writes a file-level authorization (exclusive create). It lists every file with its bytes, SHA-256,
   mtime and frame check, the classification re-derived now, and the totals. It removes nothing.
2. The operator answers with the exact phrase
   "authorize cleanup <set> <authorization-sha256> removing <n> files <bytes> bytes".
3. `execute` then:
   * checks the preconditions: no receipt or ledger, the cleanup lock, no live lock, no simulator process (FC-2),
     the manifests verify, and the phrase, authorization and manifest hashes match;
   * re-derives the classification and refuses on any change;
   * pre-checks every file (regular, no symlink, size and mtime as authorized);
   * per file: verifies the whole-file SHA-256 (and, for depth.bin, every frame against `depth-index.jsonl`), writes
     a write-ahead ledger line, unlinks it only if the same inode is still in place, then fsyncs and writes `done`;
   * stops at the first mismatch or error, touching no later file;
   * runs the after-checks: manifests, retained records unchanged, the lineage's re-classification and free space;
   * writes a permanent receipt (exclusive create), even after a stop.
4. `rehearse` copies named runs to a scratch root and does all of the above there (decision C-6).

It never follows a symlink, never compresses or moves anything, never retries, and never touches `state/`, `logs/`,
the repository or build trees. A lineage module supplies the classification, the manifest checks and the after-check
(`p7e_v3_lineage.py` for the P7-E cleanup manifest v3).
"""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import time

AUTH_CONTRACT = "evidence-removal-authorization-v1"
RECEIPT_CONTRACT = "evidence-removal-receipt-v1"
LEDGER_CONTRACT = "evidence-removal-ledger-v1"
RUN_NAME = re.compile(r"^\d{8}T\d{6}Z-[A-Za-z0-9_.-]+$")
REMOVABLE = re.compile(r"^(sensors/depth\.bin|tmp/[^\x00]+)$")
KEY_RECORDS = ("summary.json", "controller-result.json", "p3-evaluation.json", "p4-evaluation.json",
               "p5-evaluation.json", "p6/p6-evaluation.json", "evaluation.json", "p7/flight-record.json",
               "p7e/flight-assessment.json", "p4/decisions.jsonl", "p4/local_map.jsonl", "p5/policy-ledger.jsonl",
               "p8/recovery-ledger.jsonl", "sensors/depth-index.jsonl")
CHUNK = 1 << 22
HERE = Path(__file__).resolve().parent


class Refused(RuntimeError):
    """A precondition, a re-derivation or a file check failed: nothing more is touched."""


# ---- hashing and verification ---------------------------------------------------------------------------------------
def file_sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(CHUNK), b""):
            h.update(block)
    return h.hexdigest()


def tool_identity():
    return {p.name: file_sha(p) for p in sorted(HERE.glob("*.py")) if not p.name.startswith("test_")}


def read_index(index_path):
    fd = os.open(index_path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, "r", encoding="utf-8") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise Refused("depth index is not a regular file: %s" % index_path)
        rows = [json.loads(line) for line in stream if line.strip()]
    frames = sorted((r for r in rows if r.get("accepted") and r.get("offset") is not None), key=lambda r: r["offset"])
    return frames


def verify_open(path, index_path=None):
    """Open without following links; return (stat, check). check: bytes, sha256, frames, frames_ok, covers."""
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, "rb") as stream:
        st = os.fstat(stream.fileno())
        if not stat.S_ISREG(st.st_mode):
            raise Refused("not a regular file: %s" % path)
        whole, frames, frames_ok, covers = hashlib.sha256(), 0, 0, None
        if index_path is not None:
            rows = read_index(index_path)
            cursor, contiguous = 0, True
            for r in rows:
                contiguous &= r["offset"] == cursor
                cursor = r["offset"] + r["stored_bytes"]
            covers = contiguous and cursor == st.st_size
            for r in rows:
                data = stream.read(r["stored_bytes"])
                whole.update(data)
                frames += 1
                frames_ok += hashlib.sha256(data).hexdigest() == r["stored_sha256"]
        for block in iter(lambda: stream.read(CHUNK), b""):
            whole.update(block)
    check = {"bytes": st.st_size, "sha256": whole.hexdigest(), "mtime_ns": st.st_mtime_ns}
    if index_path is not None:
        check.update(frames=frames, frames_ok=frames_ok, index_covers_file=covers,
                     frames_sha_verified=bool(covers and frames == frames_ok > 0))
    return st, check


def removable(run, rel):
    parts = str(rel).split("/")
    return bool(RUN_NAME.match(str(run)) and REMOVABLE.match(str(rel)) and "" not in parts and "." not in parts
                and ".." not in parts)


def no_symlink_path(runs_root, run, rel):
    """The file path, if every component below runs_root is a real directory and the file a regular file."""
    if not removable(run, rel):
        raise Refused("not a removable item: %s/%s" % (run, rel))
    current = Path(runs_root)
    if Path(runs_root).is_symlink() or not Path(runs_root).is_dir():
        raise Refused("runs root is not a real directory: %s" % runs_root)
    parts = [run] + rel.split("/")
    for i, part in enumerate(parts):
        current = current / part
        st = os.lstat(current)  # raises FileNotFoundError
        if stat.S_ISLNK(st.st_mode):
            raise Refused("symlink in path: %s" % current)
        last = i == len(parts) - 1
        if (last and not stat.S_ISREG(st.st_mode)) or (not last and not stat.S_ISDIR(st.st_mode)):
            raise Refused("unexpected file type: %s" % current)
    return current


def check_file(runs_root, item):
    path = no_symlink_path(runs_root, item["run"], item["path"])
    index = path.parent / "depth-index.jsonl" if item["path"] == "sensors/depth.bin" else None
    return path, verify_open(path, index)


def retained_state(run_dir):
    """Every retained file's (size, mtime_ns), and the SHA-256 of the key records."""
    run_dir = Path(run_dir)
    files = {}
    for dirpath, dirs, names in os.walk(run_dir, followlinks=False):
        for name in names:
            p = Path(dirpath) / name
            rel = p.relative_to(run_dir).as_posix()
            if REMOVABLE.match(rel) or p.is_symlink() or not p.is_file():
                continue
            st = p.stat()
            files[rel] = [st.st_size, st.st_mtime_ns]
    keys = {rel: file_sha(run_dir / rel) for rel in KEY_RECORDS if rel in files}
    ulogs = {rel: file_sha(run_dir / rel) for rel in files if rel.endswith(".ulg")}
    return {"files": files, "key_sha256": keys, "ulog_sha256": ulogs}


# ---- authorization ------------------------------------------------------------------------------------------------
def phrase(set_name, auth_sha, n, nbytes):
    return "authorize cleanup %s %s removing %d files %d bytes" % (set_name, auth_sha, n, nbytes)


def write_exclusive(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "x", encoding="utf-8") as stream:
        stream.write(json.dumps(value, indent=1, sort_keys=True) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    return file_sha(path)


def prepare(lineage, set_name, runs_root, output, operator, only=None, rehearsal=False, now=time.time_ns):
    """Build and write the file-level authorization for one set. Removes nothing."""
    if Path(output).exists():
        raise Refused("authorization exists: %s" % output)
    plan = lineage.plan(set_name, only)  # [{run, path, bytes, sha256, frames?}] from the reconciled manifest
    runs = sorted({p["run"] for p in plan})
    derived = lineage.derive(runs_root, runs)
    category = lineage.SETS[set_name]["category"]
    bad = {r: d for r, d in derived.items() if d["classification"] != "ELIGIBLE" or d["category"] != category}
    if bad:
        raise Refused("not ELIGIBLE now in %s: %s" % (category, sorted(bad)[:5]))
    files = []
    for item in plan:
        _path, (_st, check) = check_file(runs_root, item)
        for key in ("bytes", "sha256"):
            if check[key] != item[key]:
                raise Refused("%s differs from the manifest: %s/%s" % (key, item["run"], item["path"]))
        if item["path"] == "sensors/depth.bin":
            if not check["frames_sha_verified"] or check["frames"] != item["frames"]:
                raise Refused("frame check failed: %s" % item["run"])
        files.append(dict(item, mtime_ns=check["mtime_ns"], **({k: check[k] for k in ("frames", "frames_sha_verified")}
                                                               if "frames" in check else {})))
    value = {"contract": AUTH_CONTRACT, "lineage": lineage.NAME, "set": set_name, "category": category,
             "runs_root": str(runs_root), "rehearsal": bool(rehearsal), "subset": only is not None,
             "manifest": lineage.manifest_identity(), "decisions": lineage.DECISIONS,
             "classifier": lineage.classifier_identity(), "tool": tool_identity(),
             "prepared_by": operator, "prepared_unix_ns": now(), "runs": derived, "files": files,
             "totals": {"runs": len(runs), "files": len(files), "bytes": sum(f["bytes"] for f in files)},
             "removes_nothing_by_itself": True,
             "phrase_template": phrase(set_name, "<sha256 of this file>", len(files), sum(f["bytes"] for f in files)),
             "statement": "This file lists the only files that the set may remove. It authorizes nothing until the "
                          "operator answers with the exact phrase, which names this file's own SHA-256."}
    sha = write_exclusive(output, value)
    return sha, phrase(set_name, sha, len(files), value["totals"]["bytes"])


def load_authorization(path, lineage):
    data = Path(path).read_bytes()
    value = json.loads(data)
    if value.get("contract") != AUTH_CONTRACT or value.get("lineage") != lineage.NAME:
        raise Refused("not an authorization of lineage %s" % lineage.NAME)
    set_name = value.get("set")
    if set_name not in lineage.SETS or value.get("category") != lineage.SETS[set_name]["category"]:
        raise Refused("unknown set or category")
    files, runs = value.get("files") or [], value.get("runs") or {}
    seen = set()
    for f in files:
        key = (f.get("run"), f.get("path"))
        if key in seen:
            raise Refused("duplicate item %s/%s" % key)
        seen.add(key)
        if f.get("run") not in runs or not removable(f.get("run"), f.get("path")):
            raise Refused("item not removable or not listed: %s/%s" % key)
        if runs[f["run"]].get("classification") != "ELIGIBLE":
            raise Refused("run not ELIGIBLE in the authorization: %s" % f["run"])
    totals = value.get("totals") or {}
    if not files or totals.get("files") != len(files) or totals.get("bytes") != sum(f["bytes"] for f in files):
        raise Refused("totals do not match the listed files")
    return value, hashlib.sha256(data).hexdigest()


# ---- the ledger ---------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self, path):
        self.stream = open(path, "x", encoding="utf-8")

    def write(self, **record):
        self.stream.write(json.dumps(record, sort_keys=True) + "\n")
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def close(self):
        self.stream.close()


def fsync_dir(path):
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


# ---- execution ----------------------------------------------------------------------------------------------------
PREFIX = "p7e-cleanup"


def record_paths(records_dir, set_name, auth_sha):
    base = Path(records_dir)
    tag = "%s-%s" % (set_name, auth_sha[:12])
    return (base / ("%s-receipt-%s.json" % (PREFIX, tag)), base / ("%s-ledger-%s.jsonl" % (PREFIX, tag)),
            base / ("%s.lock" % PREFIX))


def execute(lineage, authorization, confirm, operator, runs_root, records_dir, unlink=os.unlink, now=time.time_ns,
            free=shutil.disk_usage):
    """Remove exactly the authorized files, fail-closed.

    Phase 1 (refusable): every check before the first removal. A refusal raises Refused, removes nothing and writes no
    receipt, so the same authorization can be executed once the cause is gone.
    Phase 2 (under the write-ahead ledger): the removals. From here on a receipt is always written, `completed` or
    `stopped`, and the authorization can never be executed again."""
    value, auth_sha = load_authorization(authorization, lineage)
    set_name, files = value["set"], value["files"]
    receipt_path, ledger_path, lock_path = record_paths(records_dir, set_name, auth_sha)
    if receipt_path.exists() or ledger_path.exists():
        raise Refused("this authorization was already executed (receipt or ledger exists)")
    expected = phrase(set_name, auth_sha, len(files), value["totals"]["bytes"])
    if confirm != expected:
        raise Refused("confirmation phrase must be exactly: " + expected)
    if str(Path(runs_root)) != value["runs_root"]:
        raise Refused("the authorization binds runs root %s" % value["runs_root"])
    receipt = {"contract": RECEIPT_CONTRACT, "lineage": lineage.NAME, "set": set_name, "authorization": str(authorization),
               "authorization_sha256": auth_sha, "confirmation_phrase": confirm, "operator": operator,
               "rehearsal": value["rehearsal"], "runs_root": value["runs_root"], "manifest": value["manifest"],
               "tool": tool_identity(), "classifier": lineage.classifier_identity(), "started_unix_ns": now(),
               "status": "running", "stop_reason": None, "removed": [], "not_removed": [], "checks": {}}
    Path(records_dir).mkdir(parents=True, exist_ok=True)
    try:
        lock = os.open(lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        raise Refused("another cleanup holds %s" % lock_path)
    os.write(lock, json.dumps({"pid": os.getpid(), "authorization": auth_sha}).encode())
    os.close(lock)
    try:
        lineage.preconditions(receipt["checks"])  # live locks, FC-2, manifests verify
        if lineage.manifest_identity() != value["manifest"]:
            raise Refused("the cleanup manifest differs from the authorization")
        derived = lineage.derive(runs_root, sorted(value["runs"]))
        changed = sorted(r for r in value["runs"] if derived.get(r) != value["runs"][r])
        receipt["checks"]["rederived"] = {"runs": len(derived), "changed": changed}
        if changed:
            raise Refused("classification changed for %d runs, e.g. %s" % (len(changed), changed[0]))
        for f in files:  # a cheap pre-check of the whole set before anything is removed
            path = no_symlink_path(runs_root, f["run"], f["path"])
            st = os.lstat(path)
            if st.st_size != f["bytes"] or st.st_mtime_ns != f["mtime_ns"]:
                raise Refused("size or mtime changed: %s/%s" % (f["run"], f["path"]))
        before = {r: retained_state(Path(runs_root) / r) for r in sorted(value["runs"])}
        receipt["checks"]["free_bytes_before"] = free(runs_root).free
        ledger = Ledger(ledger_path)
    except Refused:
        os.unlink(lock_path)
        raise
    except Exception as exc:
        os.unlink(lock_path)
        raise Refused("precondition failed: %s:%s" % (type(exc).__name__, str(exc)[:300]))
    try:
        ledger.write(event="header", contract=LEDGER_CONTRACT, authorization_sha256=auth_sha, files=len(files),
                     unix_ns=now())
        for f in files:
            try:
                path, (st, check) = check_file(runs_root, f)
            except (Refused, OSError, ValueError, KeyError) as exc:
                raise Refused("check failed before removal: %s/%s: %s" % (f["run"], f["path"], exc))
            for key in ("bytes", "sha256", "mtime_ns"):
                if check[key] != f[key]:
                    raise Refused("%s mismatch before removal: %s/%s" % (key, f["run"], f["path"]))
            if f["path"] == "sensors/depth.bin" and not (check["frames_sha_verified"] and check["frames"] == f["frames"]):
                raise Refused("frame mismatch before removal: %s" % f["run"])
            ledger.write(event="intent", run=f["run"], path=f["path"], bytes=f["bytes"], sha256=f["sha256"], unix_ns=now())
            again = os.lstat(path)
            if (again.st_dev, again.st_ino, again.st_size, again.st_mtime_ns) != (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns):
                raise Refused("file replaced after verification: %s/%s" % (f["run"], f["path"]))
            unlink(path)
            fsync_dir(path.parent)
            removed_ns = now()
            ledger.write(event="done", run=f["run"], path=f["path"], unix_ns=removed_ns)
            receipt["removed"].append(dict({k: f[k] for k in ("run", "path", "bytes", "sha256")},
                                           frames=check.get("frames"), frames_verified=check.get("frames_sha_verified"),
                                           removed_unix_ns=removed_ns))
        receipt["status"] = "completed"
    except BaseException as exc:  # any failure is a stop; the ledger is the truth
        receipt["status"] = "stopped"
        receipt["stop_reason"] = type(exc).__name__ + ":" + str(exc)[:400]
    finally:
        ledger.close()
        done = {(r["run"], r["path"]) for r in receipt["removed"]}
        receipt["not_removed"] = [{"run": f["run"], "path": f["path"], "bytes": f["bytes"]} for f in files
                                  if (f["run"], f["path"]) not in done]
        receipt["removed_bytes"] = sum(r["bytes"] for r in receipt["removed"])
        receipt["ledger"] = str(ledger_path)
        receipt["ledger_sha256"] = file_sha(ledger_path)
        try:
            after_checks(lineage, receipt, runs_root, value, before, free)
        except Exception as exc:
            receipt["checks"]["passed"] = False
            receipt["checks"]["after_check_error"] = type(exc).__name__ + ":" + str(exc)[:300]
        receipt["finished_unix_ns"] = now()
        try:
            receipt_sha = write_exclusive(receipt_path, receipt)
        finally:
            os.unlink(lock_path)
    return receipt_path, receipt_sha, receipt


def after_checks(lineage, receipt, runs_root, value, before, free):
    checks = receipt["checks"]
    try:
        post = {}
        lineage.preconditions(post, after=True)
        checks["manifests_after"] = post.get("manifests")
    except Exception as exc:
        checks["manifests_after"] = {"verified": False, "error": type(exc).__name__ + ":" + str(exc)[:300]}
    unchanged = {r: retained_state(Path(runs_root) / r) == before.get(r) for r in sorted(value["runs"])}
    checks["retained_records_unchanged"] = {"runs": len(unchanged), "changed": sorted(r for r, ok in unchanged.items() if not ok)}
    try:
        checks["reclassification"] = lineage.after_check(runs_root, sorted({r["run"] for r in receipt["removed"]}))
    except Exception as exc:
        checks["reclassification"] = {"passed": False, "error": type(exc).__name__ + ":" + str(exc)[:300]}
    checks["free_bytes_after"] = free(runs_root).free
    checks["passed"] = bool((checks.get("manifests_after") or {}).get("verified")) and not checks["retained_records_unchanged"]["changed"] \
        and (checks["reclassification"] or {}).get("passed", False)


# ---- rehearsal ----------------------------------------------------------------------------------------------------
def rehearse(lineage, set_name, runs, copy_root, operator, source_runs_root):
    """C-6: copy the named runs to a scratch root, then prepare and execute there. The originals are never touched."""
    copy_root = Path(copy_root)
    if copy_root.exists():
        raise Refused("copy root exists: %s" % copy_root)
    (copy_root / "runs").mkdir(parents=True)
    for run in runs:
        source = Path(source_runs_root) / run
        if source.is_symlink() or not source.is_dir():
            raise Refused("not a run directory: %s" % source)
        shutil.copytree(source, copy_root / "runs" / run, symlinks=True, copy_function=shutil.copy2)
    records = copy_root / "state"
    auth = records / ("%s-authorization-%s-rehearsal.json" % (PREFIX, set_name))
    sha, words = prepare(lineage, set_name, copy_root / "runs", auth, operator, only=runs, rehearsal=True)
    return execute(lineage, auth, words, operator, copy_root / "runs", records)


# ---- entry --------------------------------------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(description="Evidence Retention Manager: prepare, execute or rehearse a removal set.")
    parser.add_argument("--lineage", default="p7e_v3_lineage")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("prepare")
    p.add_argument("--set", required=True)
    p.add_argument("--operator", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--runs", nargs="*")
    e = sub.add_parser("execute")
    e.add_argument("--authorization", type=Path, required=True)
    e.add_argument("--confirm", required=True)
    e.add_argument("--operator", required=True)
    r = sub.add_parser("rehearse")
    r.add_argument("--set", required=True)
    r.add_argument("--runs", nargs="+", required=True)
    r.add_argument("--copy-root", type=Path, required=True)
    r.add_argument("--operator", required=True)
    args = parser.parse_args(argv)
    sys.path.insert(0, str(HERE))
    lineage = importlib.import_module(args.lineage)
    root = lineage.ROOT
    if args.command == "prepare":
        sha, words = prepare(lineage, args.set, root / "runs", args.output, args.operator, only=args.runs)
        print(json.dumps({"authorization": str(args.output), "sha256": sha, "phrase": words}, indent=1))
    elif args.command == "execute":
        path, sha, receipt = execute(lineage, args.authorization, args.confirm, args.operator, root / "runs", root / "state")
        print(json.dumps({"receipt": str(path), "sha256": sha, "status": receipt["status"], "stop_reason": receipt["stop_reason"],
                          "removed": len(receipt["removed"]), "removed_bytes": receipt["removed_bytes"],
                          "checks_passed": receipt["checks"].get("passed")}, indent=1))
        return 0 if receipt["status"] == "completed" and receipt["checks"].get("passed") else 1
    else:
        path, sha, receipt = rehearse(lineage, args.set, args.runs, args.copy_root, args.operator, root / "runs")
        print(json.dumps({"receipt": str(path), "sha256": sha, "status": receipt["status"], "stop_reason": receipt["stop_reason"],
                          "removed": len(receipt["removed"]), "removed_bytes": receipt["removed_bytes"],
                          "checks": receipt["checks"]}, indent=1, default=str))
        return 0 if receipt["status"] == "completed" and receipt["checks"].get("passed") else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
