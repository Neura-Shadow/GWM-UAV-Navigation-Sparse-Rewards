"""P4 M3: deterministic planner (L1-L7) and safety supervisor (contract §5), offline."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
from gwm_obstacle_avoidance import local_map, planner, supervisor  # noqa: E402

CONFIG = syn.p4_config()
PL, ENV, P = CONFIG["planner"], CONFIG["envelope"], CONFIG["perception"]
GOAL = tuple(CONFIG["mission"]["goal_enu_m"])
SCENARIOS = syn.scenarios()["scenarios"]


def free_grid(radius=6.0):
    """Every cell observed FREE within `radius` of the origin (an idealized fully observed map)."""
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    ii, jj = np.meshgrid(np.arange(grid.nx), np.arange(grid.ny), indexing="ij")
    cx, cy = grid.x_min + (ii + 0.5) * grid.resolution, grid.y_min + (jj + 0.5) * grid.resolution
    grid.cells[(np.abs(cx) < 20) & (np.hypot(cx - 4, cy) < radius + 4)] = local_map.FREE
    return grid


def occupy(grid, x0, x1, y0, y1):
    for i in range(grid.nx):
        for j in range(grid.ny):
            x, y = grid.center(i, j)
            if x0 <= x <= x1 and y0 <= y <= y1:
                grid.cells[i, j] = local_map.OCCUPIED


def ctx(target=(0.0, 0.0), yaw=0.0, age=0.05, scanned=()):
    return planner.Context(target, target, yaw, GOAL, (0.0, 0.0), age, CONFIG["geofence"], scanned)


def none(grid):
    return np.zeros(grid.cells.shape, dtype=bool)


# Planner ------------------------------------------------------------------------------------------
def test_clear_map_translates_straight_along_the_path_with_the_long_segment():
    grid = free_grid()
    decision = planner.plan(grid, none(grid), ctx(), PL, ENV, 0.25)
    assert decision["action"] == planner.TRANSLATE and decision["best"]["k"] == 0 and decision["length"] == 1.5


def test_symmetric_obstacle_resolves_to_plus_y_and_decisions_are_deterministic():
    grid = free_grid()
    occupy(grid, 1.4, 1.8, -0.2, 0.2)  # blocks both straight candidates (inflation 1.0 m)
    first = planner.plan(grid, none(grid), ctx(), PL, ENV, 0.25)
    second = planner.plan(grid, none(grid), ctx(), PL, ENV, 0.25)
    assert first == second
    assert first["best"]["k"] > 0 and first["action"] in (planner.TRANSLATE, planner.YAW_TO)


def test_translation_authority_is_separate_from_collision_validity():
    grid = free_grid()
    decision = planner.plan(grid, none(grid), ctx(yaw=math.radians(40)), PL, ENV, 0.25)
    assert decision["action"] == planner.YAW_TO and abs(decision["heading"]) < 1e-9


def test_in_place_yaw_is_refused_next_to_an_obstacle():
    grid = free_grid()
    occupy(grid, 0.5, 0.7, -0.1, 0.1)  # within R_body + 0.10 + tracking (0.79 m)
    permitted, reason = planner.yaw_permitted(grid, none(grid), ctx(), PL, ENV, 0.25)
    assert not permitted and reason == "occupied_within_yaw_disc"


def test_in_place_yaw_needs_the_body_disc_free_or_traversed_and_a_fresh_snapshot():
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    assert planner.yaw_permitted(grid, none(grid), ctx(), PL, ENV, 0.25) == (False, "envelope_disc_not_observed")
    traversed = planner.traversed_mask(grid, [(0.0, 0.0)], ENV["r_body_m"])
    assert planner.yaw_permitted(grid, traversed, ctx(), PL, ENV, 0.25) == (True, None)
    assert planner.yaw_permitted(grid, traversed, ctx(age=0.3), PL, ENV, 0.25) == (False, "stale_snapshot")


def test_unknown_only_blockage_scans_once_per_heading_and_never_translates():
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    traversed = planner.traversed_mask(grid, [(0.0, 0.0)], ENV["r_body_m"])
    decision = planner.plan(grid, traversed, ctx(), PL, ENV, 0.25)
    assert decision["action"] == planner.SCAN and decision["candidates_valid"] == 0
    again = planner.plan(grid, traversed, ctx(scanned={round(decision["heading"], 6)}), PL, ENV, 0.25)
    assert again["action"] == planner.SCAN and again["heading"] != decision["heading"]


def test_occupied_and_geofence_invalidate_and_stale_snapshot_holds():
    grid = free_grid()
    occupy(grid, 1.0, 1.2, -0.1, 0.1)
    valid, reason, _, _ = planner.collision_check(grid, none(grid), (0, 0), (1.5, 0), ctx(), PL, 0.25)
    assert not valid and reason == "occupied"
    valid, reason, _, _ = planner.collision_check(grid, none(grid), (0, 1.5), (0, 2.3), ctx(), PL, 0.25)
    assert not valid and reason == "geofence"  # |y| must stay <= 3.0 - r_infl
    decision = planner.plan(grid, none(grid), ctx(age=0.26), PL, ENV, 0.25)
    assert decision["action"] == planner.HOLD and decision["candidates_valid"] == 0


def test_candidates_are_clipped_at_the_goal_and_the_last_step_reaches_it():
    grid = free_grid()
    decision = planner.plan(grid, none(grid), ctx(target=(7.96, 0.0)), PL, ENV, 0.25)
    assert decision["action"] == planner.TRANSLATE and math.dist(decision["end"], GOAL) < 1e-9


# Supervisor ---------------------------------------------------------------------------------------
def inputs(sim, grid, **override):
    value = {"sim": sim, "snapshot": {"acquisition_sim_s": sim - 0.05, "calibration_identity": "cal"}, "grid": grid,
             "traversed": none(grid), "adapter_health": {"status": "fresh", "calibration_identity": "cal", "sim_s": 1e9},
             "consecutive_unusable": 0, "reset_sim": None, "vehicle": (0.0, 0.0), "vehicle_yaw": 0.0,
             "target": (0.0, 0.0), "goal": GOAL, "origin": (0.0, 0.0), "geofence": CONFIG["geofence"]}
    value.update(override)
    return value


STALE = {
    "no_usable_snapshot": lambda sim: {"snapshot": None},
    "snapshot_stale": lambda sim: {"snapshot": {"acquisition_sim_s": 9.70, "calibration_identity": "cal"}},
    "adapter_health_not_fresh": lambda sim: {"adapter_health": {"status": "stale", "calibration_identity": "cal", "sim_s": 1e9}},
    "calibration_mismatch": lambda sim: {"adapter_health": {"status": "fresh", "calibration_identity": "other", "sim_s": 1e9}},
    "unusable_frames": lambda sim: {"consecutive_unusable": 3},
    "estimator_reset_since_last_usable_frame": lambda sim: {"reset_sim": sim - 0.01},
    "adapter_health_missing": lambda sim: {"adapter_health": {"status": "fresh", "calibration_identity": "cal",
                                                               "sim_s": sim - 2.01}},
}


@pytest.mark.parametrize("reason", sorted(STALE))
def test_stale_inputs_hover_with_yaw_frozen_then_abort_after_ten_seconds(reason):
    grid = free_grid()
    sup = supervisor.Supervisor(CONFIG, 0.0)
    out = sup.step(inputs(10.0, grid, **STALE[reason](10.0)))
    assert out["state"] == supervisor.HOVER and out["frozen_yaw"] and out["action"] == planner.HOLD
    assert out["stale"] == reason
    assert sup.step(inputs(19.9, grid, **STALE[reason](19.9)))["state"] == supervisor.HOVER
    final = sup.step(inputs(20.11, grid, **STALE[reason](20.11)))
    assert final["state"] == supervisor.ABORT and sup.abort_reason == "stale_inputs:" + reason


def test_fresh_inputs_after_a_stale_hover_resume_progress():
    grid = free_grid()
    sup = supervisor.Supervisor(CONFIG, 0.0)
    sup.step(inputs(10.0, grid, snapshot=None))
    out = sup.step(inputs(10.5, grid))
    assert out["state"] == supervisor.AVOIDING and out["action"] == planner.TRANSLATE


def test_obstacle_detection_avoidance_and_hover_timeout():
    grid = free_grid()
    occupy(grid, 1.2, 1.6, -3.5, 3.5)  # wall across the geofence: no progress candidate
    sup = supervisor.Supervisor(CONFIG, 0.0)
    out = sup.step(inputs(1.0, grid))
    assert [t["to"] for t in sup.transitions] == [supervisor.OBSTACLE_DETECTED, supervisor.HOVER]
    assert out["action"] == planner.HOLD
    assert sup.step(inputs(20.9, grid))["state"] == supervisor.HOVER
    out = sup.step(inputs(21.1, grid))
    assert out["state"] == supervisor.ABORT and sup.abort_reason == "hover_timeout"


def test_clearance_below_body_margin_and_transit_deadline_abort():
    grid = free_grid()
    occupy(grid, 0.3, 0.45, -0.05, 0.05)
    sup = supervisor.Supervisor(CONFIG, 0.0)
    assert sup.step(inputs(1.0, grid))["state"] == supervisor.ABORT and sup.abort_reason == "clearance_below_body_margin"
    sup = supervisor.Supervisor(CONFIG, 0.0)
    assert sup.step(inputs(150.1, free_grid()))["state"] == supervisor.ABORT and sup.abort_reason == "transit_deadline"


def test_apply_action_respects_ramps_and_holds():
    decision = {"state": supervisor.NORMAL, "action": planner.TRANSLATE, "heading": 0.0, "end": (1.5, 0.0),
                "frozen_yaw": False}
    target, yaw = supervisor.apply_action((0.0, 0.0), math.radians(30), decision, 0.3, math.radians(10), 0.05, GOAL)
    assert target == pytest.approx((0.015, 0.0)) and yaw == pytest.approx(math.radians(29.5))
    held = supervisor.apply_action((0.0, 0.0), 0.3, dict(decision, action=planner.HOLD), 0.3, 0.17, 0.05, GOAL)
    assert held == ((0.0, 0.0), 0.3)
    frozen = supervisor.apply_action((0.0, 0.0), 0.3, dict(decision, frozen_yaw=True), 0.3, 0.17, 0.05, GOAL)
    assert frozen == ((0.0, 0.0), 0.3)
    yawing = supervisor.apply_action((0.0, 0.0), 0.0, dict(decision, action=planner.YAW_TO, heading=1.0), 0.3, 0.17, 0.05, GOAL)
    assert yawing[0] == (0.0, 0.0) and yawing[1] == pytest.approx(0.17 * 0.05)


# Closed loop: independent renderer -> production perception -> supervisor (ideal tracking) -------
@pytest.fixture(scope="module")
def runs():
    return {name: syn.closed_loop(SCENARIOS[name]["boxes"]) for name in ("S1", "S2", "S3", "S4", "S5")}


def states(run):
    return [t["to"] for t in run["supervisor"].transitions]


def test_s1_clear_path_reaches_the_goal_without_avoiding_or_hovering(runs):
    run = runs["S1"]
    assert math.dist(run["final"], GOAL) < 1e-6 and states(run) == []
    assert max(abs(p[1][1]) for p in run["trajectory"]) <= 0.30


@pytest.mark.parametrize("name,side", [("S2", 1.2), ("S3", -0.7), ("S4", None)])
def test_obstacle_scenarios_detour_resume_and_reach_the_goal(runs, name, side):
    run = runs[name]
    assert math.dist(run["final"], GOAL) < 1e-6
    assert states(run)[:2] == [supervisor.OBSTACLE_DETECTED, supervisor.AVOIDING]
    assert supervisor.RESUME in states(run) and states(run)[-1] == supervisor.NORMAL
    assert supervisor.HOVER not in states(run) and supervisor.ABORT not in states(run)
    assert run["min_clearance"] >= 0.20
    if side:
        lateral = [p[1][1] for p in run["trajectory"]]
        # The pass needs r_infl from the pillar surface: S2 |y| >= 1.2 on +y, S3 y <= 0.3 - 1.0 = -0.7.
        assert (max(lateral) if side > 0 else -min(lateral)) >= abs(side) - 0.01


def test_s5_blocked_path_refuses_safely(runs):
    run = runs["S5"]
    assert run["supervisor"].state == supervisor.ABORT and run["supervisor"].abort_reason == "hover_timeout"
    assert supervisor.HOVER in states(run) and run["min_clearance"] >= 0.20
    hover_at = next(t["sim_s"] for t in run["supervisor"].transitions if t["to"] == supervisor.HOVER)
    after = [p[1] for p in run["trajectory"] if p[0] >= hover_at]
    assert max(math.dist(after[0], p) for p in after) <= 0.01  # no progress after HOVER


def test_s6_sensor_loss_hovers_frozen_then_aborts():
    stop = 36.1 + 2.0 / 0.3  # perception stops once the vehicle is about 2 m from origin
    run = syn.closed_loop(SCENARIOS["S6"]["boxes"], stop_perception_at=stop)
    sup = run["supervisor"]
    hover = next(t for t in sup.transitions if t["to"] == supervisor.HOVER)
    last_usable = max(d for d in [stop - 0.1 * k for k in range(3)] if d < stop)
    assert hover["sim_s"] <= last_usable + 0.35 and hover["reason"].startswith("stale_inputs:")
    frozen = [p for p in run["trajectory"] if p[0] >= hover["sim_s"]]
    assert max(math.dist(frozen[0][1], p[1]) for p in frozen) <= 0.01
    assert max(abs(planner.wrap(p[2] - frozen[0][2])) for p in frozen) <= math.radians(0.5)
    assert sup.state == supervisor.ABORT and sup.abort_reason.startswith("stale_inputs:")
    abort = next(t for t in sup.transitions if t["to"] == supervisor.ABORT)
    assert abort["sim_s"] - hover["sim_s"] == pytest.approx(10.0, abs=0.1)
