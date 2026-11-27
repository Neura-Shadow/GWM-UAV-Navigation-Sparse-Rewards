"""Offline tests of p7e_block_recovery.py on a synthetic block (kept outside tests/, so the P7-E candidate's test
binding is unchanged). Run: cd /tmp && PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m pytest -q -p no:cacheprovider THIS_FILE
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p7e_block_recovery as rec  # noqa: E402
from p3_provenance import digest, strict_json  # noqa: E402

SHA = "cd" * 32
FROZEN = {"p7": {"sha256": rec.m.P7_SHA256}, "development": False, "baselines": {}}


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True))


def make_run(root, name, flight):
    run = Path(root) / "runs" / name
    write(run / "summary.json", {"status": "passed", "failure": None, "finalized": True,
                                 "p7": {"item": flight["layout"], "controller": flight["controller"], "slot": flight["slot"]},
                                 "p7e": {"p7e_manifest_sha256": SHA, "block": flight["block"], "dry_run": False}})
    (run / "p7").mkdir(exist_ok=True)
    return run


def staged(tmp_path, flown=2, orphan=True):
    planned = rec.seq.plan_flights(rec.seq.schedule(), 1)
    records = []
    for f in planned[:flown]:
        run = make_run(tmp_path, "r%02d-p7e-%s" % (f["plan_index"], f["controller"]), f)
        records.append(dict(f, run=str(run), classification={"validity": "valid", "stop_reasons": []}))
    result = rec.m.result_path(tmp_path, 1, SHA)
    write(result.with_name(result.name.replace(".result.json", ".ledger.jsonl")), {})
    result.with_name(result.name.replace(".result.json", ".ledger.jsonl")).write_text(
        "".join(json.dumps(r, sort_keys=True) + "\n" for r in records))
    auth = tmp_path / "state/p7e-authorization-engineering-30-block-01-x.json"
    write(auth, rec.m.authorization_template(1, SHA, "operator"))
    write(rec.m.marker_path(tmp_path, 1, SHA), {"authorization_sha256": digest(auth)})
    write(rec.m.live_lock(tmp_path), {"manifest_sha256": SHA, "pid": 999999, "started_unix_ns": 1})
    if orphan:
        make_run(tmp_path, "r%02d-p7e-orphan" % flown, planned[flown])
    return planned


def evaluate(run):
    write(Path(run) / "p4-evaluation.json", {"status": "passed"})
    return SimpleNamespace(returncode=0, stderr="")


def classify_with(value):
    def classify(run):
        write(Path(run) / "p7/flight-record.json", {"validity": value["validity"]})
        write(Path(run) / "p7e/flight-assessment.json", value)
        return value
    return classify


def go(tmp_path, **kwargs):
    kw = dict(residual=lambda: [], live=lambda pid: False, evaluate=evaluate,
              classify=classify_with({"validity": "valid", "outcome": "goal_success", "stop_reasons": []}))
    kw.update(kwargs)
    return rec.recover(tmp_path, FROZEN, SHA, 1, "test statement", **kw)


def test_the_recovery_writes_the_stop_fc1_would_have_and_clears_the_lock(tmp_path):
    staged(tmp_path)
    result, receipt = go(tmp_path)
    written = strict_json(rec.m.result_path(tmp_path, 1, SHA))
    assert written == json.loads(json.dumps(result)) and written["stopped"] is True
    assert written["stop_reason"] == "sequencer_terminated_externally"
    assert [f["plan_index"] for f in written["flights"]] == [0, 1] and written["interrupted"]["plan_index"] == 2
    assert len(written["not_flown"]) == 17 and written["accounting"]["complete"]
    assert written["accounting"]["status_counts"] == {"flown_valid": 2, "interrupted": 1, "not_flown": 17}
    assert written["interrupted_flight_record"]["classification"]["outcome"] == "goal_success"
    assert written["interrupted_flight_record"]["evaluation_status"] == "passed" and written["reconstructed"]["lock"]["pid"] == 999999
    assert written["ledger"]["sha256"] == digest(Path(written["ledger"]["path"]))
    assert not rec.m.live_lock(tmp_path).exists() and json.loads(strict_json(receipt)["lock_content"])["pid"] == 999999
    assert rec.seq.previous_block_ok(tmp_path, 2, SHA)[0] is False  # FC-1: block 2 stays refused
    with pytest.raises(rec.Refused, match="already has a result"):
        go(tmp_path)


def test_a_stop_reason_of_the_orphan_is_kept_and_an_unclassified_orphan_stops_too(tmp_path):
    staged(tmp_path)
    result, _ = go(tmp_path, classify=classify_with({"validity": "valid", "stop_reasons": ["safety_event:safety:collision_contact"]}))
    assert result["stop_reason"] == "sequencer_terminated_externally;stop_rule:safety_event:safety:collision_contact"
    assert result["accounting"]["complete"]


def test_without_an_orphan_the_rest_is_not_flown(tmp_path):
    staged(tmp_path, orphan=False)
    result, _ = go(tmp_path)
    assert result["interrupted"] is None and len(result["not_flown"]) == 18 and result["accounting"]["complete"]


@pytest.mark.parametrize("case", ["live pid", "simulator running", "pgrep fails", "lock other manifest", "no lock",
                                  "two orphans", "orphan not next", "orphan evaluated", "no marker"])
def test_the_recovery_refuses_and_changes_nothing(tmp_path, case):
    planned = staged(tmp_path)
    kwargs = {}
    if case == "live pid":
        kwargs["live"] = lambda pid: True
    elif case == "simulator running":
        kwargs["residual"] = lambda: ["12 /bin/px4"]
    elif case == "pgrep fails":
        def failing():
            raise rec.p7e_host.ProcessCheckFailed("pgrep_failed:2")
        kwargs["residual"] = failing
    elif case == "lock other manifest":
        write(rec.m.live_lock(tmp_path), {"manifest_sha256": "ef" * 32, "pid": 1})
    elif case == "no lock":
        rec.m.live_lock(tmp_path).unlink()
    elif case == "two orphans":
        make_run(tmp_path, "r99-p7e-second", planned[3])
    elif case == "orphan not next":
        run = next((tmp_path / "runs").glob("*-orphan"))
        write(run / "summary.json", dict(strict_json(run / "summary.json"), p7=dict(item=planned[5]["layout"], controller="p5", slot=planned[5]["slot"])))
    elif case == "orphan evaluated":
        write(next((tmp_path / "runs").glob("*-orphan")) / "p4-evaluation.json", {})
    elif case == "no marker":
        rec.m.marker_path(tmp_path, 1, SHA).unlink()
    lock_before = rec.m.live_lock(tmp_path).read_text() if rec.m.live_lock(tmp_path).exists() else None
    with pytest.raises((rec.Refused, rec.p7e_host.ProcessCheckFailed)):
        go(tmp_path, **kwargs)
    assert not rec.m.result_path(tmp_path, 1, SHA).exists()
    assert (rec.m.live_lock(tmp_path).read_text() if rec.m.live_lock(tmp_path).exists() else None) == lock_before


def test_a_live_sequencer_pid_is_recognised_by_its_command_line(tmp_path):
    (tmp_path / "7").mkdir()
    (tmp_path / "7/cmdline").write_bytes(b"/usr/bin/python3\0/x/run_p7e_block.py\0--block\0001")
    (tmp_path / "8").mkdir()
    (tmp_path / "8/cmdline").write_bytes(b"/usr/bin/sleep\0100")
    assert rec.pid_is_live_sequencer(7, tmp_path) is True
    assert rec.pid_is_live_sequencer(8, tmp_path) is False and rec.pid_is_live_sequencer(9, tmp_path) is False
