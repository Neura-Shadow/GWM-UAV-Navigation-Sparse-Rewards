"""Offline tests of the Evidence Retention Manager executor (C-5) on synthetic roots. Nothing real is touched.

Run: cd /tmp && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider THIS_FILE
T1 happy path; T2 live lock or simulator; T3 class changed; T4 SHA or frame mismatch stops before the unlink;
T5 stop midway; T6 protected or unlisted item; T7 symlink; T8 existing receipt; T9 manifest fails; T10 second run.
"""
import hashlib
import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import retention as R  # noqa: E402
import p7e_v3_lineage as L  # noqa: E402

RUNS = ["20260101T000001Z-p6dry-a-0001", "20260101T000002Z-p6dry-b-0002", "20260101T000003Z-p6dry-c-0003"]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def make_run(runs_root, name, frames=(b"frame-one", b"frame-two-longer")):
    run = runs_root / name
    for d in ("sensors", "tmp/sub", "p4"):
        (run / d).mkdir(parents=True)
    offset, rows = 0, []
    for i, f in enumerate(frames):
        rows.append({"accepted": True, "offset": offset, "stored_bytes": len(f), "stored_sha256": sha(f), "seq": i})
        offset += len(f)
    (run / "sensors/depth.bin").write_bytes(b"".join(frames))
    (run / "sensors/depth-index.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (run / "summary.json").write_text('{"finalized": true}\n')
    (run / "p4/decisions.jsonl").write_text('{"event": "p4_decision"}\n')
    (run / "flight.ulg").write_bytes(b"ULG" * 10)
    (run / "tmp/a.txt").write_text("temporary")
    (run / "tmp/sub/b.bin").write_bytes(b"\x00\x01")
    return run


class Fake:
    """A lineage with controllable classification, preconditions and manifest identity."""
    NAME = "test-lineage"
    SETS = {"dev": {"category": "dev_raw_depth"}}
    DECISIONS = ["test"]

    def __init__(self, runs_root, names):
        self.runs_root, self.names = runs_root, names
        self.classes = {n: "ELIGIBLE" for n in names}
        self.fail = None
        self.identity = {"path": "manifest.json", "sha256": "0" * 64}
        self.after_called = []

    def plan(self, set_name, only=None):
        items = []
        for n in (only or self.names):
            run = self.runs_root / n
            data = (run / "sensors/depth.bin").read_bytes()
            items.append({"run": n, "path": "sensors/depth.bin", "bytes": len(data), "sha256": sha(data), "frames": 2})
            for rel in ("tmp/a.txt", "tmp/sub/b.bin"):
                b = (run / rel).read_bytes()
                items.append({"run": n, "path": rel, "bytes": len(b), "sha256": sha(b)})
        return items

    def derive(self, runs_root, names):
        return {n: {"classification": self.classes[n], "category": "dev_raw_depth" if self.classes[n] == "ELIGIBLE" else None,
                    "reasons": [], "dependencies": []} for n in names}

    def manifest_identity(self):
        return self.identity

    def classifier_identity(self):
        return {"fake": "1"}

    def preconditions(self, checks, after=False):
        if self.fail and not after:
            raise RuntimeError(self.fail)
        checks["manifests"] = {"verified": True}

    def after_check(self, runs_root, removed):
        self.after_called.append(list(removed))
        return {"passed": True}


@pytest.fixture
def env(tmp_path):
    runs_root = tmp_path / "runs"
    runs_root.mkdir()
    for n in RUNS:
        make_run(runs_root, n)
    lineage = Fake(runs_root, RUNS)
    records = tmp_path / "state"
    auth = records / "p7e-cleanup-authorization-dev-test.json"
    auth_sha, words = R.prepare(lineage, "dev", runs_root, auth, "tester")
    return dict(root=runs_root, lineage=lineage, records=records, auth=auth, sha=auth_sha, words=words, tmp=tmp_path)


def run_exec(env, **kw):
    return R.execute(env["lineage"], env["auth"], env.get("confirm", env["words"]), "tester", env["root"], env["records"], **kw)


def depth_present(env):
    return [(env["root"] / n / "sensors/depth.bin").exists() for n in RUNS]


# ---- T1 -------------------------------------------------------------------------------------------------------------
def test_t1_happy_path(env):
    retained_before = {n: R.retained_state(env["root"] / n) for n in RUNS}
    path, receipt_sha, receipt = run_exec(env)
    assert receipt["status"] == "completed" and receipt["checks"]["passed"] is True
    assert depth_present(env) == [False, False, False]
    for n in RUNS:
        run = env["root"] / n
        assert not (run / "tmp/a.txt").exists() and not (run / "tmp/sub/b.bin").exists()
        assert (run / "tmp").is_dir() and (run / "tmp/sub").is_dir() and (run / "sensors").is_dir()
        assert (run / "sensors/depth-index.jsonl").exists() and (run / "flight.ulg").exists()
        assert R.retained_state(run) == retained_before[n]
    assert len(receipt["removed"]) == 9 and receipt["not_removed"] == []
    assert all(r["frames_verified"] for r in receipt["removed"] if r["path"] == "sensors/depth.bin")
    lines = [json.loads(x) for x in Path(receipt["ledger"]).read_text().splitlines()]
    assert lines[0]["event"] == "header" and [x["event"] for x in lines[1:]] == ["intent", "done"] * 9
    assert R.file_sha(path) == receipt_sha and not (env["records"] / "p7e-cleanup.lock").exists()
    assert env["lineage"].after_called == [sorted(RUNS)]


def test_prepare_records_every_file_and_the_phrase(env):
    value = json.loads(env["auth"].read_text())
    assert value["totals"]["files"] == 9 and value["removes_nothing_by_itself"] is True
    assert env["words"] == R.phrase("dev", env["sha"], 9, value["totals"]["bytes"])
    assert all("mtime_ns" in f for f in value["files"])
    assert depth_present(env) == [True, True, True]


# ---- T2 -------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("why", ["a live lock exists: p8-live-run.lock", "simulator processes are running (FC-2)",
                                 "pgrep_failed:2"])
def test_t2_refused_with_live_lock_or_simulator(env, why):
    env["lineage"].fail = why
    with pytest.raises(R.Refused, match="precondition failed"):
        run_exec(env)
    assert depth_present(env) == [True, True, True]
    assert not list(env["records"].glob("p7e-cleanup-receipt-*")) and not list(env["records"].glob("p7e-cleanup-ledger-*"))
    assert not (env["records"] / "p7e-cleanup.lock").exists()
    env["lineage"].fail = None  # the cause is gone: the same authorization can run
    assert run_exec(env)[2]["status"] == "completed"


def test_refused_while_another_cleanup_holds_the_lock(env):
    env["records"].mkdir(exist_ok=True)
    (env["records"] / "p7e-cleanup.lock").write_text("{}")
    with pytest.raises(R.Refused, match="another cleanup"):
        run_exec(env)
    assert depth_present(env) == [True, True, True]


# ---- T3 -------------------------------------------------------------------------------------------------------------
def test_t3_class_changed(env):
    env["lineage"].classes[RUNS[2]] = "PROTECTED"
    with pytest.raises(R.Refused, match="classification changed"):
        run_exec(env)
    assert depth_present(env) == [True, True, True]


# ---- T4 -------------------------------------------------------------------------------------------------------------
def test_t4_sha_mismatch_stops_before_the_unlink(env):
    target = env["root"] / RUNS[1] / "sensors/depth.bin"
    st = target.stat()
    data = bytearray(target.read_bytes())
    data[0] ^= 0xFF
    target.write_bytes(bytes(data))
    os.utime(target, ns=(st.st_atime_ns, st.st_mtime_ns))  # same size and mtime: only the hash shows it
    _path, _sha, receipt = run_exec(env)
    assert receipt["status"] == "stopped" and "mismatch before removal" in receipt["stop_reason"]
    assert depth_present(env) == [False, True, True]
    assert (env["root"] / RUNS[2] / "tmp/a.txt").exists()  # no later file touched
    assert {(r["run"], r["path"]) for r in receipt["not_removed"]} >= {(RUNS[1], "sensors/depth.bin"), (RUNS[2], "sensors/depth.bin")}
    events = [json.loads(x)["event"] for x in Path(receipt["ledger"]).read_text().splitlines()]
    assert events.count("intent") == events.count("done") == 3  # no intent for the mismatching file


def test_t4_frame_mismatch_stops_before_the_unlink(env):
    index = env["root"] / RUNS[0] / "sensors/depth-index.jsonl"
    rows = [json.loads(x) for x in index.read_text().splitlines()]
    rows[1]["stored_sha256"] = "f" * 64
    index.write_text("".join(json.dumps(r) + "\n" for r in rows))
    _path, _sha, receipt = run_exec(env)
    assert receipt["status"] == "stopped" and "frame mismatch" in receipt["stop_reason"]
    assert depth_present(env) == [True, True, True] and receipt["removed"] == []


# ---- T5 -------------------------------------------------------------------------------------------------------------
def test_t5_stop_midway_leaves_a_consistent_receipt_and_ledger(env):
    calls = []

    def flaky(path):
        calls.append(path)
        if len(calls) == 4:
            raise OSError("disk went away")
        os.unlink(path)
    _path, _sha, receipt = run_exec(env, unlink=flaky)
    assert receipt["status"] == "stopped" and "disk went away" in receipt["stop_reason"]
    lines = [json.loads(x) for x in Path(receipt["ledger"]).read_text().splitlines()]
    intents = [(x["run"], x["path"]) for x in lines if x["event"] == "intent"]
    dones = [(x["run"], x["path"]) for x in lines if x["event"] == "done"]
    assert len(intents) == 4 and len(dones) == 3 and intents[:3] == dones
    assert [(r["run"], r["path"]) for r in receipt["removed"]] == dones
    assert intents[3] in {(r["run"], r["path"]) for r in receipt["not_removed"]}
    assert (env["root"] / intents[3][0] / intents[3][1]).exists()
    assert receipt["ledger_sha256"] == R.file_sha(receipt["ledger"])
    assert not (env["records"] / "p7e-cleanup.lock").exists()


# ---- T6 -------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("item", [{"path": "p4/decisions.jsonl"}, {"path": "flight.ulg"}, {"path": "sensors/depth-index.jsonl"},
                                  {"path": "tmp/../summary.json"}, {"path": "../state/x.json"},
                                  {"run": "20260101T000009Z-unlisted-0009", "path": "sensors/depth.bin"}])
def test_t6_protected_or_unlisted_item_is_refused(env, item):
    value = json.loads(env["auth"].read_text())
    value["files"].append(dict({"run": RUNS[0], "bytes": 1, "sha256": "0" * 64, "mtime_ns": 0}, **item))
    value["totals"] = {"runs": 3, "files": len(value["files"]), "bytes": sum(f["bytes"] for f in value["files"])}
    forged = env["tmp"] / "forged.json"
    forged.write_text(json.dumps(value))
    with pytest.raises(R.Refused):
        R.load_authorization(forged, env["lineage"])


def test_t6_prepare_refuses_a_protected_run(env):
    env["lineage"].classes[RUNS[0]] = "PROTECTED"
    with pytest.raises(R.Refused, match="not ELIGIBLE"):
        R.prepare(env["lineage"], "dev", env["root"], env["tmp"] / "other.json", "tester")


def test_removable_rule():
    ok = R.removable
    assert ok(RUNS[0], "sensors/depth.bin") and ok(RUNS[0], "tmp/a/b.txt")
    for rel in ("sensors/depth-index.jsonl", "p4/local_map.jsonl", "p5/policy-ledger.jsonl", "p8/recovery-ledger.jsonl",
                "x.ulg", "tmp", "tmp/", "tmp//a", "tmp/./a", "tmp/../a", "summary.json", "sensors/depth.bin.bak"):
        assert not ok(RUNS[0], rel), rel
    assert not ok("../state", "sensors/depth.bin")


# ---- T7 -------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("where", ["file", "sensors", "run"])
def test_t7_symlink_is_refused(env, where):
    run = env["root"] / RUNS[1]
    if where == "file":
        real = env["tmp"] / "elsewhere.bin"
        target = run / "sensors/depth.bin"
        real.write_bytes(target.read_bytes())
        target.unlink()
        target.symlink_to(real)
    elif where == "sensors":
        moved = env["tmp"] / "sensors-moved"
        (run / "sensors").rename(moved)
        (run / "sensors").symlink_to(moved)
    else:
        moved = env["tmp"] / "run-moved"
        run.rename(moved)
        run.symlink_to(moved)
    with pytest.raises(R.Refused):
        run_exec(env)
    assert (env["root"] / RUNS[0] / "sensors/depth.bin").exists()


# ---- T8 -------------------------------------------------------------------------------------------------------------
def test_t8_existing_receipt_is_refused(env):
    receipt, _ledger, _lock = R.record_paths(env["records"], "dev", env["sha"])
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text("{}")
    with pytest.raises(R.Refused, match="already executed"):
        run_exec(env)
    assert depth_present(env) == [True, True, True]


# ---- T9 -------------------------------------------------------------------------------------------------------------
def test_t9_manifest_fails_to_verify(env):
    env["lineage"].fail = "a manifest does not verify"
    with pytest.raises(R.Refused, match="a manifest does not verify"):
        run_exec(env)
    env["lineage"].fail = None
    env["lineage"].identity = {"path": "manifest.json", "sha256": "1" * 64}
    with pytest.raises(R.Refused, match="cleanup manifest differs"):
        run_exec(env)
    assert depth_present(env) == [True, True, True]


def test_wrong_phrase_or_root_is_refused(env):
    env["confirm"] = env["words"].replace("removing 9", "removing 8")
    with pytest.raises(R.Refused, match="phrase"):
        run_exec(env)
    env["confirm"] = env["words"]
    with pytest.raises(R.Refused, match="runs root"):
        R.execute(env["lineage"], env["auth"], env["words"], "tester", env["tmp"] / "other", env["records"])
    assert depth_present(env) == [True, True, True]


def test_tampered_authorization_changes_its_hash(env):
    value = json.loads(env["auth"].read_text())
    value["prepared_by"] = "someone else"
    env["auth"].write_text(json.dumps(value))
    with pytest.raises(R.Refused, match="phrase"):
        run_exec(env)


# ---- T10 ------------------------------------------------------------------------------------------------------------
def test_t10_second_execution_is_refused(env):
    path, _sha, receipt = run_exec(env)
    assert receipt["status"] == "completed"
    with pytest.raises(R.Refused, match="already executed"):
        run_exec(env)
    path.unlink()
    Path(receipt["ledger"]).unlink()
    with pytest.raises(R.Refused):  # an already removed file is refused, never passed
        run_exec(env)


# ---- rehearsal ------------------------------------------------------------------------------------------------------
def test_rehearsal_touches_only_the_copy(env, monkeypatch):
    lineage = env["lineage"]
    copy_root = env["tmp"] / "rehearsal"
    original = {n: R.retained_state(env["root"] / n) for n in RUNS[:2]}
    monkeypatch.setattr(lineage, "runs_root", copy_root / "runs")  # the fake plans from the copy
    _path, _sha, receipt = R.rehearse(lineage, "dev", RUNS[:2], copy_root, "tester", env["root"])
    assert receipt["status"] == "completed" and receipt["rehearsal"] is True
    assert depth_present(env) == [True, True, True]
    assert {n: R.retained_state(env["root"] / n) for n in RUNS[:2]} == original
    assert not (copy_root / "runs" / RUNS[0] / "sensors/depth.bin").exists()
    with pytest.raises(R.Refused):
        R.rehearse(lineage, "dev", RUNS[:2], copy_root, "tester", env["root"])  # copy root exists


# ---- the P7-E lineage's C-11 rule (offline parts) -------------------------------------------------------------------
def test_c11_record_only_sources_are_fail_closed(tmp_path, monkeypatch):
    source = tmp_path / "docs/evidence/dep.json"
    source.parent.mkdir(parents=True)
    source.write_text('{"runs": ["x"]}')
    proof = tmp_path / "proof.json"
    monkeypatch.setattr(L, "REPO", tmp_path)
    monkeypatch.setattr(L, "PROOF", proof)
    good = {"proofs": {"t": {"passed": True}}, "sources": {"docs/evidence/dep.json": {"proof": "t", "sha256": L.sha(source)}}}
    proof.write_text(json.dumps(good))
    assert L.record_only_sources() == {str(source): "t"}
    proof.write_text(json.dumps(dict(good, proofs={"t": {"passed": False}})))
    assert L.record_only_sources() == {}
    proof.write_text(json.dumps(good))
    source.write_text('{"runs": ["x", "y"]}')  # the source changed after its proof
    assert L.record_only_sources() == {}
    proof.unlink()
    assert L.record_only_sources() == {}


def test_c3_runs_are_never_planned(monkeypatch):
    rows = [{"run": r, "classification": "ELIGIBLE", "category": "p7e_raw_depth", "depth_bin_bytes": 3,
             "depth_bin": {"ok": True, "bytes": 3, "sha256": "0" * 64, "frames": 1}} for r in sorted(L.C3)]
    rows.append(dict(rows[0], run="20261127T999999Z-p7e-p4-p7c-4000999-00000000"))
    monkeypatch.setitem(L._cache, "manifest", {"root": "/x/runs", "rows": rows, "other_entries": []})
    assert [i["run"] for i in L.plan("p7e-depth")] == ["20261127T999999Z-p7e-p4-p7c-4000999-00000000"]
    with pytest.raises(RuntimeError):
        L.plan("p7e-depth", only=[sorted(L.C3)[0]])
