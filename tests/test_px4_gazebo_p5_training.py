"""P5 M3: training environment, layouts and acceptance exclusion, oracle, trainer, and the exported
model artifact with its readiness, training and dataset reports."""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402

sys.path.insert(0, str(p5.P5 / "training"))
import p5_dataset  # noqa: E402
import p5_episode  # noqa: E402
import p5_layouts  # noqa: E402
import p5_oracle  # noqa: E402
import p5_render  # noqa: E402
import p5_train  # noqa: E402
from gwm_obstacle_avoidance import planner, supervisor  # noqa: E402
from gwm_policy_control import contracts, observation_adapter, policy_runtime  # noqa: E402

CONFIG = p5_episode.configs()[1]
SCENARIOS = p5.syn.scenarios()
ARTIFACTS = sorted((p5.P5 / "models").glob("p5-ranker-*.npz"))


# Environment ------------------------------------------------------------------------------------
def test_training_renderer_equals_the_independent_p4_test_renderer():
    rng = np.random.default_rng(3)
    for _ in range(25):
        boxes = [{"center": [float(rng.uniform(0.5, 8)), float(rng.uniform(-3, 3))],
                  "size": [float(rng.uniform(0.2, 1.0)), float(rng.uniform(0.2, 4))]} for _ in range(rng.integers(0, 4))]
        vehicle, yaw = (float(rng.uniform(-0.5, 8)), float(rng.uniform(-2, 2)), 2.0), float(rng.uniform(-math.pi, math.pi))
        ours = p5_render.render(vehicle, yaw, p5_render.boxes_3d(boxes), columns=p5_episode.COLUMNS)
        reference = p5.syn.render(vehicle, yaw, p5.syn.boxes_3d(boxes), columns=p5_episode.COLUMNS)
        assert np.array_equal(ours, reference, equal_nan=True)


def test_episode_reaches_the_goal_and_a_cached_scan_changes_nothing():
    boxes = SCENARIOS["scenarios"]["S2"]["boxes"]
    fresh = p5_episode.run_episode(boxes, lambda c, s: supervisor.Supervisor(c, s))
    cached = p5_episode.run_episode(boxes, lambda c, s: supervisor.Supervisor(c, s), scan_cells=p5_episode.map_scan(boxes))
    assert fresh == cached
    assert fresh["outcome"] == "goal_reached" and fresh["min_clearance_m"] > 0.5 and fresh["track_max_m"] < 0.25


# Layouts ----------------------------------------------------------------------------------------
def test_split_seed_ranges_are_disjoint_and_generation_is_deterministic():
    ranges = sorted((base, base + p5_layouts.SEED_RANGE) for base, _ in p5_layouts.SPLITS.values())
    assert all(a[1] <= b[0] for a, b in zip(ranges, ranges[1:]))
    assert p5_layouts.generate(1_000_123) == p5_layouts.generate(1_000_123)
    assert p5_layouts.SPLITS["train"][1] >= 1000 and p5_layouts.SPLITS["held_out"][1] >= 200


def test_acceptance_match_rule_follows_contract_4_1():
    s2 = SCENARIOS["scenarios"]["S2"]["boxes"]
    near = [{"center": [4.2, -0.1], "size": [0.48, 0.33]}]
    assert p5_layouts.matches(near, s2)
    assert not p5_layouts.matches([{"center": [4.3, 0.0], "size": [0.4, 0.4]}], s2)  # centre 0.30 m away
    assert not p5_layouts.matches([{"center": [4.0, 0.0], "size": [0.55, 0.4]}], s2)  # size 0.15 m off
    assert not p5_layouts.matches(near + [{"center": [6.0, 2.0], "size": [0.3, 0.3]}], s2)  # a different set
    s4 = SCENARIOS["scenarios"]["S4"]["boxes"]
    assert p5_layouts.matches([s4[1], s4[0]], s4)  # any pairing
    for name, boxes in p5_layouts.acceptance_sets().items():
        assert name in p5_layouts.matching_acceptance(boxes)
    assert sorted(p5_layouts.acceptance_sets()) == ["G2", "G4", "G6", "S1", "S2", "S3", "S4", "S5", "S6"]


def test_acceptance_layouts_and_blocked_layouts_are_rejected():
    for name in ("S2", "S3", "S4"):
        assert p5_layouts.rejection({"boxes": SCENARIOS["scenarios"][name]["boxes"]}, CONFIG) == "acceptance_layout"
    assert p5_layouts.rejection({"boxes": [{"center": [3.0, 0.0], "size": [0.3, 8.0]}]}, CONFIG) == "goal_unreachable_inflated"
    assert p5_layouts.rejection({"boxes": [{"center": [1.0, 0.0], "size": [0.4, 0.4]}]}, CONFIG) == "start_or_goal_disc"


def test_screened_layouts_never_match_an_acceptance_set(monkeypatch):
    monkeypatch.setattr(p5_layouts, "SPLITS", {name: (base, 30) for name, (base, _) in p5_layouts.SPLITS.items()})
    seen = set()
    for name in p5_layouts.SPLITS:
        result = p5_layouts.split(name)
        assert len(result["layouts"]) == 30 and result["seeds_tried"] >= 30
        for layout in result["layouts"]:
            assert not p5_layouts.matching_acceptance(layout["boxes"]) and 1 <= len(layout["boxes"]) <= 4
            assert layout["seed"] not in seen
            seen.add(layout["seed"])
    assert p5_layouts.split("train") == p5_layouts.split("train")


# Oracle -----------------------------------------------------------------------------------------
def test_cost_to_go_is_the_inflated_shortest_path():
    empty = p5_oracle.CostToGo([], CONFIG)
    assert empty.at((0.0, 0.0)) == pytest.approx(8.0) and empty.at((8.0, 0.0)) == 0.0
    assert empty.at((4.0, 1.0)) == pytest.approx(math.hypot(4.0, 1.0), rel=0.035)
    pillar = SCENARIOS["scenarios"]["S2"]["boxes"]
    detour = p5_oracle.CostToGo(pillar, CONFIG)
    # Around a disc of radius 1.2 m (pillar half-width plus r_infl): longer than 8 m, shorter than the box hull.
    assert 8.2 < detour.at((0.0, 0.0)) < 8.0 + 2 * 1.2
    wall = [{"center": [4.0, 0.0], "size": [0.3, 8.0]}]
    assert not p5_oracle.CostToGo(wall, CONFIG, strict=True).feasible_from((0.0, 0.0))
    inside_band = (3.0, 0.0)  # 0.8 m from the pillar: inside r_infl, still finite and penalized
    assert math.isfinite(detour.at(inside_band)) and detour.at(inside_band) > empty.at(inside_band) + 1.0


def test_segment_clearance_and_targets():
    pillar = [{"center": [4.0, 0.0], "size": [0.4, 0.4]}]
    assert p5_oracle.segment_clearance((0.0, 1.0), (8.0, 1.0), pillar) == pytest.approx(0.8 - 0.39, abs=1e-9)
    assert p5_oracle.segment_clearance((0.0, 0.0), (1.0, 0.0), pillar) == pytest.approx(2.8 - 0.39, abs=1e-9)
    assert p5_oracle.segment_clearance((0.0, 0.0), (1.0, 0.0), []) == math.inf
    costs = np.array([np.nan, 1.0, 1.0, 2.0])
    allowed = np.array([False, True, True, True])
    target = p5_oracle.target_distribution(costs, allowed)
    assert target[0] == 0.0 and target.sum() == pytest.approx(1.0) and target[1] == target[2] > target[3]


def _planning_state(boxes):
    S, C, _ = p5_episode.configs()
    world = p5_episode.World(boxes, S, C)
    world.grid.cells[...] = p5_episode.map_scan(boxes)
    traversed = planner.traversed_mask(world.grid, [(0.0, 0.0)], C["envelope"]["r_body_m"])
    ctx = planner.Context((0.0, 0.0), (0.0, 0.0), 0.0, C["mission"]["goal_enu_m"], (0.0, 0.0), 0.0, C["geofence"])
    rows = planner.evaluate_candidates(world.grid, traversed, ctx, C["planner"], C["perception"]["snapshot_max_age_sim_s"])
    progress = [r for r in rows if observation_adapter.progress_qualified(r, ctx, C["planner"])[0]]
    return C, world, traversed, ctx, rows, progress


def test_recording_ranker_drives_like_p4_and_labels_only_allowed_slots():
    boxes = [{"center": [3.0, 0.4], "size": [0.4, 0.4]}]
    C, world, traversed, ctx, rows, progress = _planning_state(boxes)
    field = p5_oracle.CostToGo(boxes, C)
    ranker = p5_dataset.RecordingRanker(None, p5.SCHEMA, field, boxes, 0.2, math.radians(10), stride=1, phase=0)
    best, record = ranker.choose(rows, progress, ctx, world.grid, traversed, supervisor.NORMAL, C["planner"])
    assert record is None and best is planner.select(progress, C["planner"], world.grid.resolution)
    candidate, context, allowed, costs, driver_slot, state = ranker.samples[0]
    features, mask = observation_adapter.build(rows, ctx, C["planner"], world.grid, traversed, supervisor.NORMAL, p5.SCHEMA)
    assert np.array_equal(p5_train.assemble(candidate[None], context[None])[0], features) and np.array_equal(allowed, mask)
    assert np.isnan(costs[~allowed]).all() and np.isfinite(costs[allowed]).all() and allowed.sum() >= 2
    assert driver_slot == observation_adapter.slot_of(best, C["planner"]) and state == 0


# Trainer ----------------------------------------------------------------------------------------
def _toy_batch(seed=0, n=6):
    rng = np.random.default_rng(seed)
    x = rng.normal(size=(n, 26, 46))
    allowed = rng.random((n, 26)) < 0.5
    allowed[:, :2] = True
    costs = np.where(allowed, rng.uniform(0, 3, (n, 26)), np.nan)
    return x, allowed, p5_train.targets(costs, allowed, 0.1), costs


def test_listwise_gradients_match_finite_differences():
    params = p5_train.init_params(1)
    x, allowed, target, _ = _toy_batch()
    _, grads = p5_train.loss_and_grads(params, x, allowed, target)
    rng = np.random.default_rng(2)
    for name in contracts.LAYER_SHAPES:
        for _ in range(3):
            index = tuple(int(rng.integers(s)) for s in params[name].shape)
            plus = {k: v.copy() for k, v in params.items()}
            minus = {k: v.copy() for k, v in params.items()}
            plus[name][index] += 1e-6
            minus[name][index] -= 1e-6
            numeric = (p5_train.loss_and_grads(plus, x, allowed, target)[0]
                       - p5_train.loss_and_grads(minus, x, allowed, target)[0]) / 2e-6
            assert grads[name][index] == pytest.approx(numeric, rel=1e-4, abs=1e-8)


def test_training_forward_equals_the_runtime_forward():
    params = p5_train.init_params(4)
    x, _, _, _ = _toy_batch(5)
    policy = policy_runtime.FrozenPolicy(params, {}, sha256="test")
    scores, _ = p5_train.forward(params, x)
    for i in range(x.shape[0]):
        assert np.abs(policy.scores(x[i]) - scores[i]).max() <= 1e-12


def test_training_is_deterministic_and_reduces_validation_regret():
    x, allowed, _, costs = _toy_batch(7, n=300)
    data = {"candidate": x[..., :11], "context": x[:, 0, 11:], "allowed": allowed, "costs": costs,
            "driver_slot": np.zeros(300, dtype=np.int64), "state": np.zeros(300, dtype=np.int64),
            "layout_index": np.zeros(300, dtype=np.int64)}
    hyper = dict(p5_train.HYPER, epochs=6, batch=64)
    a, history = p5_train.train(data, data, hyper=hyper, log=lambda m: None)
    b, _ = p5_train.train(data, data, hyper=hyper, log=lambda m: None)
    assert all(np.array_equal(a[k], b[k]) for k in a)
    assert p5_train.weights_sha256(a) == p5_train.weights_sha256(b)
    assert history[-1]["val_loss"] < history[0]["val_loss"]
    assert sum(1 for h in history if h.get("selected")) == 1


# Exported model artifact (present once M3 has produced a model that passed readiness) -----------
needs_artifact = pytest.mark.skipif(not ARTIFACTS, reason="no trained P5 model has been exported")


def _artifact():
    assert len(ARTIFACTS) == 1, "exactly one P5 model artifact is expected"
    path = ARTIFACTS[0]
    sha = path.name[len("p5-ranker-"):-len(".npz")]
    return path, sha


@needs_artifact
def test_artifact_is_content_addressed_trained_and_accepted_by_the_runtime():
    path, sha = _artifact()
    assert path.stat().st_size <= 1_000_000  # operator decision D4
    policy = policy_runtime.FrozenPolicy.load(path, sha, p5.SCHEMA_SHA, require_trained=True)
    meta = policy.metadata
    assert meta["trained"] is True and meta["architecture"] == contracts.ARCHITECTURE
    assert meta["weights_sha256"] == p5_train.weights_sha256(policy._weights)
    assert meta["training"]["method"].startswith("D1a") and meta["training"]["dagger_rounds"] == 3
    assert len(meta["training"]["code_commit"]) == 40
    with pytest.raises(policy_runtime.PolicyError, match="checkpoint_sha256_mismatch"):
        policy_runtime.FrozenPolicy.load(path, "0" * 64, p5.SCHEMA_SHA)


@needs_artifact
def test_artifact_reports_are_bound_by_hash_and_readiness_passes_contract_4_2():
    path, sha = _artifact()
    meta = policy_runtime.FrozenPolicy.load(path, sha, p5.SCHEMA_SHA).metadata
    reports = {}
    for kind in ("dataset_manifest", "training_report", "readiness_report"):
        report = path.with_name("p5-ranker-%s.%s.json" % (sha, kind))
        assert policy_runtime.sha256_bytes(report.read_bytes()) == meta[kind + "_sha256"]
        reports[kind] = json.loads(report.read_text(encoding="utf-8"))
    readiness = reports["readiness_report"]
    assert readiness["weights_sha256"] == meta["weights_sha256"] and readiness["schema_sha256"] == p5.SCHEMA_SHA
    summary, bar = readiness["summary"], json.loads((p5.P5 / "configs/p5_validation.yaml").read_text(encoding="utf-8"))["readiness"]
    assert summary["ready"] is True and all(check["pass"] for check in summary["checks"].values())
    assert summary["p4"]["episodes"] == summary["policy"]["episodes"] == len(readiness["layouts"]) >= bar["held_out_layouts_min"]
    decisions = summary["policy_decisions"]
    assert decisions["outside_mask"] == decisions["executed_mismatch"] == 0 == bar["shield_violations_allowed"]
    assert summary["p4"]["goal_success_pct"] - summary["policy"]["goal_success_pct"] <= bar["goal_success_max_drop_points"]
    assert summary["policy"]["collisions"] == 0 and decisions["failures"] == 0
    for kind in ("training_report", "dataset_manifest"):
        assert reports[kind]["contract"].startswith("p5-acceptance-v1")
    assert all(r["forward_parity"]["max_abs_difference"] <= 1e-9 for r in reports["training_report"]["rounds"])


@needs_artifact
def test_held_out_layouts_are_disjoint_from_training_and_from_every_acceptance_layout():
    path, sha = _artifact()
    readiness = json.loads(path.with_name("p5-ranker-%s.readiness_report.json" % sha).read_text(encoding="utf-8"))
    base = p5_layouts.SPLITS["held_out"][0]
    for layout in readiness["layouts"]:
        assert base <= layout["seed"] < base + p5_layouts.SEED_RANGE and layout["id"].startswith("held_out-")
        assert p5_layouts.generate(layout["seed"])["boxes"] == layout["boxes"]
        assert not p5_layouts.matching_acceptance(layout["boxes"])
    splits = json.loads(path.with_name("p5-ranker-%s.dataset_manifest.json" % sha).read_text(encoding="utf-8"))["splits"]
    assert splits["train"]["count"] >= 1000 and splits["held_out"]["count"] == len(readiness["layouts"])
