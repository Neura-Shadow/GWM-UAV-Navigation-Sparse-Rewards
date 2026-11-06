"""P5 M1: frozen policy runtime and observation adapter (offline; interface-only models)."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
from gwm_obstacle_avoidance import local_map, planner  # noqa: E402
from gwm_policy_control import observation_adapter, policy_runtime  # noqa: E402
from gwm_policy_control.policy_runtime import FrozenPolicy, PolicyError  # noqa: E402

C = p5.syn.p4_config()
PL, ENV = C["planner"], C["envelope"]


def saved(tmp_path, arrays=None, **meta):
    path = tmp_path / "model.npz"
    sha = policy_runtime.save_model(path, arrays or p5.interface_arrays(), p5.interface_metadata(**meta))
    return path, sha


# ---- checkpoint loading and integrity ----------------------------------------------------------------
def test_artifact_is_deterministic_and_round_trips(tmp_path):
    path, sha = saved(tmp_path, trained=True)
    again = tmp_path / "again.npz"
    assert policy_runtime.save_model(again, p5.interface_arrays(), p5.interface_metadata(trained=True)) == sha
    policy = FrozenPolicy.load(path, sha, p5.SCHEMA_SHA)
    assert policy.sha256 == sha and policy.metadata["trained"] is True


@pytest.mark.parametrize("case,reason", [("sha", "checkpoint_sha256_mismatch"), ("missing", "checkpoint_missing"),
                                         ("architecture", "architecture_mismatch"), ("schema", "schema_mismatch"),
                                         ("untrained", "untrained_model_refused"), ("unbound", "checkpoint_sha256_mismatch")])
def test_checkpoint_rejections(tmp_path, case, reason):
    meta = {"trained": case != "untrained"}
    if case == "architecture":
        meta["architecture"] = "other"
    if case == "schema":
        meta["schema_sha256"] = "0" * 64
    path, sha = saved(tmp_path, **meta)
    expected = {"sha": "f" * 64, "unbound": None}.get(case, sha)
    with pytest.raises(PolicyError) as raised:
        FrozenPolicy.load(tmp_path / "absent.npz" if case == "missing" else path, expected, p5.SCHEMA_SHA)
    assert raised.value.reason == reason


def test_interface_models_load_only_when_a_trained_model_is_not_required(tmp_path):
    path, sha = saved(tmp_path, trained=False)
    assert FrozenPolicy.load(path, sha, p5.SCHEMA_SHA, require_trained=False).metadata["trained"] is False


@pytest.mark.parametrize("name,value,reason", [("W2", np.zeros((64, 63)), "weight_shape:W2"),
                                               ("b1", np.full(64, np.nan), "weight_not_finite:b1")])
def test_bad_weights_are_rejected(name, value, reason):
    arrays = p5.interface_arrays()
    arrays[name] = value
    with pytest.raises(PolicyError, match=reason):
        FrozenPolicy(arrays, p5.interface_metadata(), "x")


def test_weights_are_read_only_and_inference_is_stateless_and_bit_reproducible():
    policy = p5.interface_policy()
    with pytest.raises(ValueError):
        policy._weights["W1"][0, 0] = 1.0
    rng = np.random.default_rng(1)
    x, y = rng.normal(size=(26, 46)), rng.normal(size=(26, 46))
    first = policy.scores(x)
    policy.scores(y)  # another episode in between must not change anything
    assert np.array_equal(first, policy.scores(x)) and first.dtype == np.float64 and first.shape == (26,)
    w = p5.interface_arrays()  # independent reference forward
    h = np.maximum(x @ w["W1"] + w["b1"], 0)
    h = np.maximum(h @ w["W2"] + w["b2"], 0)
    assert np.allclose(first, (h @ w["W3"] + w["b3"])[:, 0], rtol=0, atol=1e-12)
    scores, seconds = policy.timed_scores(x)
    assert np.array_equal(scores, first) and seconds >= 0


@pytest.mark.parametrize("x,reason", [(np.zeros((25, 46)), "input_schema"), (np.zeros((26, 46), dtype=np.float32), "input_schema"),
                                      (np.full((26, 46), np.nan), "input_not_finite")])
def test_input_schema_is_enforced(x, reason):
    with pytest.raises(PolicyError, match=reason):
        p5.interface_policy().scores(x)


# ---- observation adapter -----------------------------------------------------------------------------
def free_grid():
    grid = local_map.Grid.around((0.0, 0.0), C["geofence"], C["perception"]["grid_margin_m"], C["perception"]["grid_resolution_m"])
    grid.cells[:] = local_map.FREE
    return grid


def ctx(target=(0.0, 0.0), yaw=0.0):
    return planner.Context(target, target, yaw, tuple(C["mission"]["goal_enu_m"]), (0.0, 0.0), 0.05, C["geofence"])


def test_adapter_slots_follow_p4_candidate_order_and_mask_equals_p4_progress_set():
    grid, c = free_grid(), ctx()
    grid.cells[grid.index(2.0, 0.0)] = local_map.OCCUPIED
    traversed = np.zeros(grid.cells.shape, dtype=bool)
    rows = planner.evaluate_candidates(grid, traversed, c, PL, 0.25)
    features, allowed = observation_adapter.build(rows, c, PL, grid, traversed, "OBSTACLE_DETECTED", p5.SCHEMA)
    assert features.shape == (26, 46) and features.dtype == np.float64
    assert [observation_adapter.slot_of(r, PL) for r in rows] == list(range(26))  # P4 order is slot order
    reaches = lambda r: math.dist(r["end"], c.goal) < 1e-9 and r["progress"] > 0
    progress = {i for i, r in enumerate(rows) if r["valid"] and (r["progress"] > PL["min_progress_m"] or reaches(r))}
    assert set(np.flatnonzero(allowed).tolist()) == progress and 0 < len(progress) < 26
    assert features[:, 0].min() == -1.0 and features[:, 0].max() == 1.0  # k / 6
    assert set(np.unique(features[:, 7:11])) <= {0.0, 1.0}
    assert (features[:, 11:] == features[0, 11:]).all()  # shared context broadcast
    assert features[0, 15 + 1] == 1.0 and features[0, 15:20].sum() == 1.0  # OBSTACLE_DETECTED one-hot


def test_polar_profile_reports_first_occupied_and_first_non_free_distances():
    grid, c = free_grid(), ctx()
    grid.cells[grid.index(2.05, 0.0)] = local_map.OCCUPIED  # straight ahead (k = 0)
    grid.cells[grid.index(1.0 * math.cos(math.radians(10)), 1.0 * math.sin(math.radians(10)))] = local_map.UNKNOWN
    traversed = np.zeros(grid.cells.shape, dtype=bool)
    profile = observation_adapter.polar_profile(grid, traversed, c.target, c.path_heading, PL, p5.SCHEMA["polar_profile"]) * 4.0
    occupied, nonfree = profile[:13], profile[13:]
    assert occupied[6] == pytest.approx(2.0, abs=0.06) and nonfree[6] == occupied[6]
    assert occupied[7] == 4.0 and nonfree[7] == pytest.approx(1.0, abs=0.06)  # k = +1: unknown, not occupied
    assert occupied[0] == 4.0 and nonfree[0] == 4.0  # k = -6: clear to the cap


def revision1_polar_profile(grid, traversed, start, path_heading, settings, profile):
    """Oracle: the revision 1 per-heading loop, verbatim (contract revision 3 requires bit identity with it)."""
    cap, ds = profile["cap_m"], profile["step_m"]
    s = np.arange(1, int(round(cap / ds)) + 1) * ds
    n = settings["heading_steps"]
    step = math.radians(settings["heading_step_deg"])
    occupied, nonfree = np.full(2 * n + 1, cap), np.full(2 * n + 1, cap)
    for idx, k in enumerate(range(-n, n + 1)):
        heading = planner.wrap(path_heading + k * step)
        i = np.floor((start[0] + s * math.cos(heading) - grid.x_min) / grid.resolution).astype(np.int64)
        j = np.floor((start[1] + s * math.sin(heading) - grid.y_min) / grid.resolution).astype(np.int64)
        inside = (i >= 0) & (i < grid.nx) & (j >= 0) & (j < grid.ny)
        states = np.full(s.size, -1, dtype=np.int64)
        states[inside] = grid.cells[i[inside], j[inside]]
        walked = np.zeros(s.size, dtype=bool)
        walked[inside] = traversed[i[inside], j[inside]]
        hit = states == local_map.OCCUPIED
        free = (states == local_map.FREE) | (walked & inside & ~hit)
        if hit.any():
            occupied[idx] = s[int(np.argmax(hit))]
        if (~free).any():
            nonfree[idx] = s[int(np.argmax(~free))]
    return np.concatenate([occupied, nonfree]) / cap


def test_polar_profile_is_bit_identical_to_the_revision1_loop():
    rng = np.random.default_rng(20261106)
    profile, step = p5.SCHEMA["polar_profile"], math.radians(PL["heading_step_deg"])
    base = free_grid()
    for case in range(3000):
        grid = local_map.Grid(base.x_min, base.y_min, base.nx, base.ny, base.resolution,
                              rng.choice(np.array([local_map.FREE, local_map.OCCUPIED, local_map.UNKNOWN], dtype=np.uint8),
                                         size=base.cells.shape, p=(0.80, 0.08, 0.12)))
        traversed = rng.random(grid.cells.shape) < (0.0, 0.05, 0.3, 0.9)[case % 4]
        width, height = grid.nx * grid.resolution, grid.ny * grid.resolution
        if case % 3 == 0:  # anywhere around the grid, including outside it
            start = (rng.uniform(grid.x_min - 2.0, grid.x_min + width + 2.0), rng.uniform(grid.y_min - 2.0, grid.y_min + height + 2.0))
        else:              # exactly on cell boundaries
            start = (grid.x_min + rng.integers(-3, grid.nx + 3) * grid.resolution, grid.y_min + rng.integers(-3, grid.ny + 3) * grid.resolution)
        heading = (rng.integers(-24, 25) * step, rng.uniform(-math.pi, math.pi), (0.0, math.pi / 2, -math.pi, math.pi)[case % 4])[case % 3]
        expected = revision1_polar_profile(grid, traversed, start, heading, PL, profile)
        actual = observation_adapter.polar_profile(grid, traversed, start, heading, PL, profile)
        assert actual.dtype == expected.dtype and actual.shape == expected.shape and np.array_equal(actual, expected), case


def test_adapter_rejects_non_planning_states_and_incomplete_candidate_sets():
    grid, c = free_grid(), ctx()
    traversed = np.zeros(grid.cells.shape, dtype=bool)
    rows = planner.evaluate_candidates(grid, traversed, c, PL, 0.25)
    with pytest.raises(ValueError, match="state_not_a_planning_state"):
        observation_adapter.build(rows, c, PL, grid, traversed, "ABORT", p5.SCHEMA)
    with pytest.raises(ValueError, match="candidate_count"):
        observation_adapter.build(rows[:-1], c, PL, grid, traversed, "NORMAL", p5.SCHEMA)
