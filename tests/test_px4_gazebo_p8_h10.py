"""P8 H10 checker tests (contract `p8-navigation-recovery-v1` §7 H10; FC-5, FC-6). Offline, on synthetic runs built
from a closed-loop recovery episode of the V0 rig."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_px4_gazebo_p8_recovery as v0  # noqa: E402  (the V0 rig: paths, config, scenes)

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "simulation/px4_gazebo_p8/validation"))
import p8_h10_check as h10  # noqa: E402
from gwm_obstacle_avoidance import local_map, planner  # noqa: E402


def write_run(tmp_path, seed=3, mutate=None):
    grid, start, yaw = v0.random_scene(seed)
    m, emitted = v0.closed_loop(grid, start, yaw)
    ticks = {}
    for i, out in emitted:
        ticks[round(i["sim"], 6)] = out
    sims = sorted({round(r["sim_s"], 6) for r in m.records})
    run = tmp_path / "run"
    (run / "p4").mkdir(parents=True)
    (run / "p8").mkdir()
    encoded = grid.encode()
    with (run / "p4/local_map.jsonl").open("w") as f:
        for r in m.records:
            s = r["snapshot"]
            f.write(json.dumps({"observation_id": s["observation_id"], "acquisition_sim_s": s["acquisition_sim_s"],
                                "usable": True, "grid": encoded}) + "\n")
    with (run / "p4/decisions.jsonl").open("w") as f:
        for sim in sims:
            out = ticks.get(sim)
            f.write(json.dumps({"event": "p4_decision", "sim_s": sim, "state": "HOVER", "stale": None,
                                "action": out["action"] if out else "HOLD",
                                "reason": out["reason"] if out else "no_collision_valid_progress_candidate"}) + "\n")
    records = copy.deepcopy(m.records)
    if mutate:
        mutate(records, run)
    with (run / "p8/recovery-ledger.jsonl").open("w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")
    return run, m


def check(run):
    return h10.check_run(run, v0.CONFIG, v0.PARAMS)


def test_a_clean_episode_reproduces_every_check(tmp_path):
    run, m = write_run(tmp_path)
    got = check(run)
    assert got["ok"], got["violations"][:5]
    assert got["actions"] > 100 and got["reproduced"] == got["actions"] + got["refusals"]


def first(records, kind, action=None):
    return next(r for r in records if r["kind"] == kind and (action is None or r["action"] == action))


@pytest.mark.parametrize("case", ["tampered end", "outside l4", "stale tick", "missing ledger", "unlisted decision",
                                  "outside window", "budget", "unknown snapshot"])
def test_every_violation_is_found(tmp_path, case):
    def mutate(records, run):
        if case == "tampered end":
            r = first(records, "action", planner.TRANSLATE)
            r["end"] = [r["end"][0] + 3.0, r["end"][1] + 3.0]
        elif case == "outside l4":
            r = first(records, "action", planner.TRANSLATE)
            r["vehicle_yaw"] = r["heading"] + 1.0
        elif case == "outside window":
            first(records, "action")["episode_hover_since_sim_s"] = -100.0
        elif case == "budget":
            first(records, "action")["rotation_used_rad"] = 10.0
        elif case == "unknown snapshot":
            first(records, "action")["snapshot"]["observation_id"] = 999
    run, _ = write_run(tmp_path, mutate=mutate)
    if case == "stale tick":
        lines = [json.loads(x) for x in (run / "p4/decisions.jsonl").read_text().splitlines()]
        next(d for d in lines if str(d["reason"]).startswith("p8_recovery:"))["stale"] = "snapshot_stale"
        (run / "p4/decisions.jsonl").write_text("".join(json.dumps(d) + "\n" for d in lines))
    elif case == "missing ledger":
        (run / "p8/recovery-ledger.jsonl").unlink()
    elif case == "unlisted decision":
        lines = [json.loads(x) for x in (run / "p4/decisions.jsonl").read_text().splitlines()]
        lines.append({"event": "p4_decision", "sim_s": 9999.0, "state": "HOVER", "stale": None, "action": "YAW_TO",
                      "reason": "p8_recovery:R2"})
        (run / "p4/decisions.jsonl").write_text("".join(json.dumps(d) + "\n" for d in lines))
    got = check(run)
    assert not got["ok"] and got["violations"]
