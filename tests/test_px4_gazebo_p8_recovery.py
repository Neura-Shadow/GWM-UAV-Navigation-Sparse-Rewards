"""P8 V0: unit and property tests of the recovery layer (contract `p8-navigation-recovery-v1` §3, §4, §6 V0).

Offline; never starts ROS or a simulator. The frozen P4 planner and supervisor are used unchanged. These invariants
are checked:
* S1: every translation passes the frozen `collision_check` and the frozen L4 cone;
* S2: every yaw passes the frozen `yaw_permitted`;
* S3: staleness wins;
* S4: only a HOLD of a `no_progress_candidate` HOVER is replaced;
* S8: the budgets;
* S9: determinism;
* S10: every action is recorded;
* plus the R4 guard and the wrapper's pass-through.
"""
import copy
import json
import math
from pathlib import Path
import random
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402  (puts the accepted P2, P3 and P4 packages on the path)

REPO = Path(__file__).resolve().parents[1]
for _path in (REPO / "simulation/px4_gazebo_p5/ros2_ws/src/gwm_policy_control",
              REPO / "simulation/px4_gazebo_p8/ros2_ws/src/gwm_navigation_recovery"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from gwm_obstacle_avoidance import local_map, planner, supervisor  # noqa: E402
from gwm_navigation_recovery import guard as guard_module, recovery, search, wrapper  # noqa: E402

CONFIG = syn.p4_config()
PARAMS = json.loads((REPO / "simulation/px4_gazebo_p8/configs/p8_recovery.json").read_text(encoding="utf-8"))
GOAL = tuple(CONFIG["mission"]["goal_enu_m"])
PL, ENV = CONFIG["planner"], CONFIG["envelope"]
HALF = math.radians(PL["observable_half_angle_deg"])


def grid_with(free=True):
    p = CONFIG["perception"]
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], p["grid_margin_m"], p["grid_resolution_m"])
    if free:
        grid.cells[:] = local_map.FREE
    return grid


def put(grid, x0, x1, y0, y1, value):
    for i in range(grid.nx):
        for j in range(grid.ny):
            x, y = grid.center(i, j)
            if x0 <= x <= x1 and y0 <= y <= y1:
                grid.cells[i, j] = value


def inputs(grid, target, yaw=0.0, sim=100.0, age=0.05, traversed=None):
    return {"sim": sim, "snapshot": {"acquisition_sim_s": sim - age, "observation_id": 7, "calibration_identity": "c"},
            "grid": grid, "traversed": traversed if traversed is not None else np.zeros(grid.cells.shape, dtype=bool),
            "vehicle": tuple(target), "vehicle_yaw": yaw, "target": tuple(target), "goal": GOAL, "origin": (0.0, 0.0),
            "geofence": CONFIG["geofence"]}


def hover_out(action=planner.HOLD, stale=None, frozen_yaw=False, state=supervisor.HOVER, **extra):
    out = {"state": state, "action": action, "heading": None, "end": None, "length": None, "frozen_yaw": frozen_yaw,
           "reason": "no_collision_valid_progress_candidate", "path": None, "plan": None, "stale": stale}
    out.update(extra)
    return out


def manager():
    return recovery.RecoveryManager(CONFIG, PARAMS, clock=lambda: 0.0)


def ctx_of(i):
    return planner.Context(i["target"], i["vehicle"], i["vehicle_yaw"], i["goal"], i["origin"],
                           i["sim"] - i["snapshot"]["acquisition_sim_s"], i["geofence"])


# ---- the trigger (S3, S4) ------------------------------------------------------------------------------------------
@pytest.mark.parametrize("case", ["normal", "avoiding", "stale", "stale entry", "frozen yaw", "policy failure", "scan",
                                  "no hover", "abort"])
def test_recovery_leaves_every_non_trigger_tick_unchanged(case):
    m, grid = manager(), grid_with()
    i, hover = inputs(grid, (7.8, 0.0)), (90.0, "no_progress_candidate")
    out = {"normal": hover_out(state=supervisor.NORMAL), "avoiding": hover_out(state=supervisor.AVOIDING),
           "stale": hover_out(stale="snapshot_stale", frozen_yaw=True), "stale entry": hover_out(),
           "frozen yaw": hover_out(frozen_yaw=True), "policy failure": hover_out(policy={"failure": "timeout"}),
           "scan": hover_out(action=planner.SCAN, heading=0.5), "no hover": hover_out(),
           "abort": hover_out(state=supervisor.ABORT, frozen_yaw=True)}[case]
    if case == "stale entry":
        hover = (90.0, "stale_inputs:snapshot_stale")
    if case == "no hover":
        hover = None
    before = copy.deepcopy(out)
    assert m.step(i, out, hover) == before
    assert not [r for r in m.records if r["kind"] in ("action", "refused")]


# ---- R1 ---------------------------------------------------------------------------------------------------------------
def test_r1_yaws_toward_never_observed_lateral_cells_near_the_goal():
    m, grid = manager(), grid_with()
    put(grid, 7.5, 8.3, 0.55, 0.95, local_map.UNKNOWN)  # lateral, within r_infl of the terminal segment
    i = inputs(grid, (7.8, 0.0), yaw=0.0)
    out = m.step(i, hover_out(), (95.0, "no_progress_candidate"))
    start = next(r for r in m.records if r["kind"] == "episode_start")
    assert start["r1_applies"] and start["terminal_occupied"] == 0 and start["terminal_unknown"] > 0
    assert out["action"] == planner.YAW_TO and out["reason"] == "p8_recovery:R1"
    assert math.pi / 4 <= out["heading"] <= 3 * math.pi / 4  # toward the +y side, outside the +/-25 degree view
    action = next(r for r in m.records if r["kind"] == "action")
    assert action["check"] == {"function": "planner.yaw_permitted", "result": [True, None]}
    assert planner.yaw_permitted(grid, recovery.decode_mask(action["traversed"]), ctx_of(i), PL, ENV, 0.25)[0]


def test_r1_does_not_apply_when_an_occupied_cell_blocks_the_terminal_segment():
    m, grid = manager(), grid_with()
    put(grid, 8.4, 8.6, 0.2, 0.4, local_map.OCCUPIED)
    m.step(inputs(grid, (7.8, 0.0)), hover_out(), (95.0, "no_progress_candidate"))
    assert next(r for r in m.records if r["kind"] == "episode_start")["r1_applies"] is False


# ---- S1, S2, S8 over randomized scenes (property) ------------------------------------------------------------------
def closed_loop(grid, target, yaw, ticks=500, dt=0.05, seed_hover=50.0, traversed=None):
    """Feed a static map, a HOVER/no_progress HOLD every tick, and the frozen apply_action; return the emitted actions."""
    m, emitted = manager(), []
    yaw_target = yaw
    for k in range(ticks):
        sim = seed_hover + k * dt
        i = inputs(grid, target, yaw=yaw_target, sim=sim, traversed=traversed)
        out = m.step(i, hover_out(), (seed_hover, "no_progress_candidate"))
        if out["action"] in (planner.TRANSLATE, planner.YAW_TO):
            emitted.append((i, out))
        target, yaw_target = supervisor.apply_action(target, yaw_target, out, CONFIG["mission"]["transit_speed_m_s"],
                                                     math.radians(10.0), dt, GOAL)
    return m, emitted


def random_scene(seed):
    rnd = random.Random(seed)
    grid = grid_with()
    for _ in range(rnd.randint(1, 3)):
        x, y = rnd.uniform(2.0, 6.5), rnd.uniform(-1.8, 1.8)
        put(grid, x, x + rnd.uniform(0.2, 0.6), y, y + rnd.uniform(0.2, 2.0), local_map.OCCUPIED)
    for _ in range(rnd.randint(0, 3)):
        x, y = rnd.uniform(0.5, 8.0), rnd.uniform(-2.5, 2.5)
        put(grid, x, x + rnd.uniform(0.3, 1.5), y, y + rnd.uniform(0.3, 1.5), local_map.UNKNOWN)
    return grid, (rnd.uniform(1.0, 2.0), rnd.uniform(-0.5, 0.5)), rnd.uniform(-0.5, 0.5)


def test_the_property_scenes_exercise_both_action_kinds():
    kinds = {planner.YAW_TO: 0, planner.TRANSLATE: 0}
    for seed in range(12):
        for _, out in closed_loop(*random_scene(seed))[1]:
            kinds[out["action"]] += 1
    assert kinds[planner.YAW_TO] > 100 and kinds[planner.TRANSLATE] > 1000  # the property test is not vacuous


@pytest.mark.parametrize("seed", range(12))
def test_every_emitted_action_passes_the_frozen_checks(seed):
    rnd = random.Random(seed)
    grid = grid_with()
    for _ in range(rnd.randint(1, 3)):
        x, y = rnd.uniform(2.0, 6.5), rnd.uniform(-1.8, 1.8)
        put(grid, x, x + rnd.uniform(0.2, 0.6), y, y + rnd.uniform(0.2, 2.0), local_map.OCCUPIED)
    for _ in range(rnd.randint(0, 3)):
        x, y = rnd.uniform(0.5, 8.0), rnd.uniform(-2.5, 2.5)
        put(grid, x, x + rnd.uniform(0.3, 1.5), y, y + rnd.uniform(0.3, 1.5), local_map.UNKNOWN)
    start = (rnd.uniform(1.0, 2.0), rnd.uniform(-0.5, 0.5))
    m, emitted = closed_loop(grid, start, rnd.uniform(-0.5, 0.5))
    for i, out in emitted:
        ctx = ctx_of(i)
        if out["action"] == planner.YAW_TO:
            assert planner.yaw_permitted(grid, i["traversed"], ctx, PL, ENV, 0.25)[0]  # S2
        else:
            assert planner.collision_check(grid, i["traversed"], i["target"], out["end"], ctx, PL, 0.25)[0]  # S1
            assert abs(planner.wrap(out["heading"] - i["vehicle_yaw"])) <= HALF + 1e-9  # L4
    rot = sum(abs(planner.wrap(a[1]["heading"] - a[0]["vehicle_yaw"])) for a in emitted if a[1]["action"] == planner.YAW_TO)
    assert all(a[0]["sim"] - 50.0 < PARAMS["episode"]["window_s"] for a in emitted)  # S8: inside the window
    ends = [r for r in m.records if r["kind"] == "action"]
    assert len(ends) == len(emitted)  # S10: every action recorded
    assert rot >= 0.0


def test_budgets_bound_rotation_and_translation():
    grid = grid_with()
    put(grid, 3.0, 3.4, -3.5, 3.5, local_map.OCCUPIED)  # a full wall: no forward progress
    put(grid, -0.5, 2.0, 1.0, 3.0, local_map.UNKNOWN)
    m, emitted = closed_loop(grid, (1.5, 0.0), 0.0, ticks=600)
    e = m.episode or {}
    rot = (e.get("rot") if e else None)
    trans = (e.get("trans") if e else None)
    ends = [r for r in m.records if r["kind"] == "episode_end"]
    rot = rot if rot is not None else ends[-1]["rotation_used_rad"]
    trans = trans if trans is not None else ends[-1]["translation_used_m"]
    assert rot <= math.radians(PARAMS["episode"]["max_rotation_deg"]) + math.radians(10.0) * 0.05 + 1e-9
    assert trans <= PARAMS["episode"]["max_translation_m"] + CONFIG["mission"]["transit_speed_m_s"] * 0.05 + 1e-9
    assert all(i["sim"] - 50.0 < PARAMS["episode"]["window_s"] for i, _ in emitted)


def test_a_refused_check_holds_and_is_recorded():
    m, grid = manager(), grid_with()
    put(grid, 7.65, 7.95, -0.15, 0.15, local_map.UNKNOWN)  # the envelope disc is not observed: L5 refuses, R1 applies
    i = inputs(grid, (7.8, 0.0))
    assert planner.yaw_permitted(grid, i["traversed"], ctx_of(i), PL, ENV, 0.25) == (False, "envelope_disc_not_observed")
    out = m.step(i, hover_out(), (95.0, "no_progress_candidate"))
    assert out["action"] == planner.HOLD
    assert next(r for r in m.records if r["kind"] == "episode_start")["r1_applies"]
    refused = [r for r in m.records if r["kind"] == "refused"]
    assert refused and refused[0]["check"] == {"function": "planner.yaw_permitted", "result": [False, "envelope_disc_not_observed"]}


def test_the_manager_is_deterministic():
    grid = grid_with()
    put(grid, 3.0, 3.4, -1.0, 3.5, local_map.OCCUPIED)
    put(grid, 1.0, 2.5, -2.5, -1.0, local_map.UNKNOWN)
    a, ea = closed_loop(grid, (1.5, 0.5), 0.2)
    b, eb = closed_loop(grid, (1.5, 0.5), 0.2)
    assert [o for _, o in ea] == [o for _, o in eb] and a.records == b.records


def test_the_c_space_equals_the_frozen_l2_rule_at_cell_centres():
    grid = grid_with()
    put(grid, 3.0, 3.4, -1.0, 1.0, local_map.OCCUPIED)
    put(grid, 5.0, 6.0, 1.0, 2.0, local_map.UNKNOWN)
    traversed = np.zeros(grid.cells.shape, dtype=bool)
    ok = search.c_space(grid, traversed, PL["r_infl_m"], (0.0, 0.0), CONFIG["geofence"])
    rnd = random.Random(3)
    for _ in range(400):
        i, j = rnd.randrange(grid.nx), rnd.randrange(grid.ny)
        p = grid.center(i, j)
        ctx = planner.Context(p, p, 0.0, GOAL, (0.0, 0.0), 0.05, CONFIG["geofence"])
        assert bool(ok[i, j]) == planner.collision_check(grid, traversed, p, p, ctx, PL, 0.25)[0]


def test_the_mask_encoding_round_trips():
    mask = np.zeros((145, 100), dtype=bool)
    mask[3:50, 7:9] = True
    assert np.array_equal(recovery.decode_mask(recovery.encode_mask(mask)), mask)


# ---- R4 and the wrapper ---------------------------------------------------------------------------------------------
def test_the_guard_fires_only_on_sustained_yaw_interventions_without_progress():
    params = copy.deepcopy(PARAMS)
    params["r4"]["yaw_to_l4_at_least"] = 30
    g = guard_module.YawOscillationGuard(params)
    events = [g.observe(t * 0.05, {"policy": {"intervention": "yaw_to_l4:3"}}, (3.0, 0.0), GOAL) for t in range(500)]
    assert "start" in events and events.index("start") * 0.05 >= params["r4"]["window_s"] - 1e-9
    g2 = guard_module.YawOscillationGuard(params)
    moving = [g2.observe(t * 0.05, {"policy": {"intervention": "yaw_to_l4:3"}}, (t * 0.01, 0.0), GOAL) for t in range(500)]
    assert "start" not in moving  # progress: never fires
    assert PARAMS["r4"]["yaw_to_l4_at_least"] == 225


class Inner:
    def __init__(self):
        self.state, self.hover_since, self.transitions = supervisor.HOVER, 90.0, [{"to": "HOVER", "reason": "no_progress_candidate"}]
        self.last_output, self.ranker, self.scanned = "x", "policy", set()

    def step(self, inputs):
        return hover_out()


def test_the_wrapper_passes_every_attribute_through_to_the_frozen_supervisor():
    inner = Inner()
    w = wrapper.RecoverySupervisor(inner, manager())
    assert w.state == supervisor.HOVER and w.transitions is inner.transitions and w.last_output == "x"
    w.last_output = None
    assert inner.last_output is None and "last_output" not in vars(w)
    assert w.hover() == (90.0, "no_progress_candidate")
    grid = grid_with()
    put(grid, 7.5, 8.3, 0.55, 0.95, local_map.UNKNOWN)
    assert w.step(inputs(grid, (7.8, 0.0)))["action"] == planner.YAW_TO


def test_the_missions_install_the_wrapper_at_transit():
    from gwm_navigation_recovery import mission as p8_mission
    from gwm_px4_control import contracts, timing
    p2 = contracts.load_config(REPO / "simulation/px4_gazebo/configs/p2_control.yaml")
    m = p8_mission.P8P4Mission(p2, CONFIG, PARAMS, timing.StateCache(p2), 0.0, True)
    m.state = "MAP_SCAN"
    m.supervisor = supervisor.Supervisor(CONFIG, 0.0)
    frozen = m.supervisor
    m.transition("TRANSIT", 1.0, 1.0)
    assert isinstance(m.supervisor, wrapper.RecoverySupervisor) and m.supervisor.inner is frozen
    assert m.supervisor.guard is None and m.recovery_ledger == []
