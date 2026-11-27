"""Offline tests of p7e_cleanup_manifest_v3.py's fail-closed rules on synthetic runs (outside tests/, so no
candidate's test binding changes). Run: cd /tmp && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider THIS_FILE
"""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p7e_cleanup_manifest_v3 as v3  # noqa: E402

EVALUATED = {"present": True, "statuses": [("p4-evaluation.json", "passed")], "safety_relevant_items": []}


def make_run(tmp_path, name, p7e=None, p7=None, outcome="goal_success", evaluation=None, frames=(b"abc", b"defgh")):
    run = tmp_path / name
    (run / "sensors").mkdir(parents=True)
    (run / "p7").mkdir()
    (run / "summary.json").write_text(json.dumps({"p7e": p7e or {}, "p7": p7 or {}}))
    (run / "p7/flight-record.json").write_text(json.dumps({"outcome": outcome}))
    (run / "p4-evaluation.json").write_text(json.dumps(evaluation or {"status": "passed", "failures": []}))
    offset, index = 0, []
    with (run / "sensors/depth.bin").open("wb") as stream:
        for data in frames:
            stream.write(data)
            index.append({"accepted": True, "offset": offset, "stored_bytes": len(data),
                          "stored_sha256": hashlib.sha256(data).hexdigest()})
            offset += len(data)
    (run / "sensors/depth-index.jsonl").write_text("".join(json.dumps(r) + "\n" for r in index))
    return run


def batch(tmp_path, slot, controller, outcome="goal_success", sha=v3.ACCEPTED, **kw):
    return make_run(tmp_path, "20261127T000000Z-p7e-%s-x-%d" % (controller, slot), {"p7e_manifest_sha256": sha, "dry_run": False},
                    {"slot": slot, "controller": controller}, outcome, **kw)


def test_a_clean_file_verifies_and_every_corruption_fails(tmp_path):
    run = make_run(tmp_path, "r-good")
    good = v3.verify_depth(run)
    assert good["ok"] and good["frames"] == good["frames_ok"] == 2 and good["index_covers_file"]
    assert good["sha256"] == hashlib.sha256(b"abcdefgh").hexdigest()
    assert v3.verify_depth(run, m0_sha=good["sha256"])["m0_inventory_match"] is True
    assert v3.verify_depth(run, m0_sha="00" * 32)["ok"] is False  # an M0 mismatch fails
    data = bytearray((run / "sensors/depth.bin").read_bytes())
    data[4] ^= 1
    (run / "sensors/depth.bin").write_bytes(bytes(data))
    bad = v3.verify_depth(run)
    assert not bad["ok"] and bad["frames_ok"] == 1
    trailing = make_run(tmp_path, "r-trailing")
    with (trailing / "sensors/depth.bin").open("ab") as stream:
        stream.write(b"x")  # bytes the index does not cover
    assert v3.verify_depth(trailing)["ok"] is False
    missing = make_run(tmp_path, "r-noindex")
    (missing / "sensors/depth-index.jsonl").unlink()
    out = v3.verify_depth(missing)
    assert out["ok"] is False and out["error"]


def test_an_accepted_batch_success_named_only_by_its_own_records_is_eligible(tmp_path):
    run = batch(tmp_path, 4, "p4")
    cls, reasons, category = v3.classify(run, ["p7e_batch_record:/x", "documentation:/d.md"], EVALUATED)
    assert (cls, category) == ("ELIGIBLE", "p7e_raw_depth")


@pytest.mark.parametrize("case", ["named by a test", "hover-timeout failure", "slot 27 P4", "slot 10 P5",
                                  "stopped candidate", "unknown candidate", "stale action", "diagnostics"])
def test_protections_win(tmp_path, case):
    deps, kw, slot, controller, outcome, sha = ["p7e_batch_record:/x"], {}, 4, "p4", "goal_success", v3.ACCEPTED
    if case == "named by a test":
        deps.append("test:/t.py")
    elif case == "hover-timeout failure":
        outcome = "failure"
    elif case == "slot 27 P4":
        slot = 27
    elif case == "slot 10 P5":
        slot, controller = 10, "p5"
    elif case == "stopped candidate":
        sha = v3.STOPPED
    elif case == "unknown candidate":
        sha = "ab" * 32
    elif case == "stale action":
        kw["evaluation"] = {"status": "failed", "failures": ["runtime_status:Abort:controller_failed:stale_action"]}
    elif case == "diagnostics":
        deps.append("diagnostics:/d.py")
    run = batch(tmp_path, slot, controller, outcome, sha, **kw)
    cls, reasons, _ = v3.classify(run, deps, EVALUATED)
    assert cls == "PROTECTED" and reasons


def test_p6_development_runs_and_the_review_cases(tmp_path):
    p6 = make_run(tmp_path, "20261114T000000Z-p6dry-A-1-x")
    assert v3.classify(p6, ["state_dry_run:/s.json"], EVALUATED)[::2] == ("ELIGIBLE", "p6_development_raw_depth")
    assert v3.classify(p6, ["state_dry_run:/s.json"], {"present": False})[0] == "REVIEW"
    assert v3.classify(p6, [], dict(EVALUATED, safety_relevant_items=["safety:collision_contact"]))[0] == "REVIEW"
    assert v3.classify(p6, ["documentation:/d.md"], EVALUATED)[0] == "REVIEW"
    p7 = make_run(tmp_path, "20261121T000000Z-p7-p5-p7p-1")
    assert v3.classify(p7, [], EVALUATED)[0] == "PROTECTED"
