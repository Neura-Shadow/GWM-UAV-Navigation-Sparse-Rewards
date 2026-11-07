"""P6 M3: the offline fault-validation suite (contract §8.1 row 4). No ROS, no PX4, no simulator.

* `test_R_<fault>_*` — the 31 required offline (R-O) items of `p6_faults.yaml`, each against the
  accepted P2–P5 code it targets. Recorded-input checks use one retained accepted flight
  (`GWM_P6_RECORDED_RUN`, default the P5 `2becfbbb` Gate C S2 flight) and skip where it is absent.
* `test_C_<fault>_*` — characterization (C) measurements on recorded or synthetic inputs. They assert
  only what the accepted code guarantees, and write the measured risk to
  `GWM_P6_CHARACTERIZATION_OUT` (JSON) for the M3 report.
* `test_rehearsal_*` — the flight rows not already rehearsed in M2, in the accepted P5 mission rig.
  Depth, pose and process faults are emulated at the mission's inputs, so a rehearsal checks the
  decision logic, not the transport.
"""
import functools
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
import test_px4_gazebo_p4_planner as p4t  # noqa: E402  (accepted P4 supervisor test inputs)
import test_px4_gazebo_p5_mission as m5  # noqa: E402  (accepted P5 mission rig)
import test_px4_gazebo_p6_injection as m2  # noqa: E402  (M2 rig flights and helpers)
from gwm_obstacle_avoidance import local_map, perception, planner, supervisor  # noqa: E402
from gwm_obstacle_avoidance.mission import P4Mission  # noqa: E402
from gwm_policy_control import candidate_ranker, control_node, safety_shield  # noqa: E402
from gwm_policy_control.candidate_ranker import PolicyRanker  # noqa: E402
from gwm_px4_control import execution_timing, timing  # noqa: E402
from gwm_sensor_adapter import association, depth, pipeline  # noqa: E402

REPO = p5.syn.REPO
sys.path.insert(0, str(REPO / "simulation/px4_gazebo_p6"))
sys.path.insert(0, str(REPO / "simulation/px4_gazebo_p6/validation"))
sys.path.insert(0, str(REPO / "simulation/px4_gazebo_p5/validation"))
from injection import controller_shim, declaration, depth_faults, policy_faults, schedule, state_faults  # noqa: E402

RUN = Path(os.environ.get("GWM_P6_RECORDED_RUN", Path.home() / "uav_autonomy/runs/20261106T132442Z-p5-flight-S2-184a5a5e"))
needs_run = pytest.mark.skipif(not (RUN / "sensors/depth.bin").exists(), reason="retained accepted flight not available")
CONFIG = p4t.CONFIG
MEASURED = {}
R_BODY = CONFIG["envelope"]["r_body_m"]


def measure(fault, **values):
    MEASURED.setdefault(fault, {}).update(values)


@pytest.fixture(scope="module", autouse=True)
def _write_measurements():
    yield
    out = os.environ.get("GWM_P6_CHARACTERIZATION_OUT")
    if out:
        Path(out).write_text(json.dumps(MEASURED, indent=2, sort_keys=True, allow_nan=False) + "\n")


# ---- shared supervisor helpers ------------------------------------------------------------------
class FailingRanker:
    def __init__(self, reason="timeout", pattern=None):
        self.reason, self.pattern, self.calls = reason, pattern, 0

    def choose(self, rows, progress, ctx, grid, traversed, state, settings):
        self.calls += 1
        failing = self.pattern is None or self.pattern(self.calls)
        if failing:
            return None, {"invoked": True, "failure": self.reason}
        return planner.select(progress, settings, grid.resolution), {"invoked": True, "failure": None}


def blocked_grid():
    """Free map with an occupied pillar in the straight corridor 2 m ahead (detours available)."""
    grid = p4t.free_grid()
    p4t.occupy(grid, 1.8, 2.2, -0.2, 0.2)
    return grid


def run_supervisor(sup, steps, start=10.0, make_inputs=None, dt=0.05):
    outs = []
    for k in range(steps):
        sim = start + k * dt
        outs.append((sim, sup.step(make_inputs(sim))))
    return outs


# ---- recorded-input helpers -----------------------------------------------------------------------
@functools.lru_cache(maxsize=None)
def recorded():
    import recorded as rec
    return rec


@functools.lru_cache(maxsize=None)
def snapshots():
    return recorded().usable_snapshots(RUN)




@functools.lru_cache(maxsize=None)
def p3_config():
    return recorded().configs(RUN)["p3"]


def recorded_frames(n=120):
    return list(recorded().iter_depth_frames(RUN, [s["acquisition_stamp_ns"] for s in snapshots()[:n]]))


# ===================================================================================================
# Required offline items (R-O)
# ===================================================================================================
@pytest.mark.parametrize("reason,override", [
    ("snapshot_stale", None), ("adapter_health_not_fresh", "health_stale"), ("adapter_health_missing", "health_old"),
    ("calibration_mismatch", "calibration"), ("unusable_frames", "unusable"), ("estimator_reset_since_last_usable_frame", "reset")])
def test_R_stale_rules_hover_by_t_last_plus_0_35_with_frozen_target_then_abort_at_10_s(reason, override):
    """H35 and A10 for every stale reason a fault row produces (F1.01/04/06/07/11, F2.04/09, F4.02/06)."""
    grid, t_last = p4t.free_grid(), 10.0

    def inputs(sim):
        snap_time = min(sim, t_last) if reason == "snapshot_stale" else sim - 0.05
        value = p4t.inputs(sim, grid, snapshot={"acquisition_sim_s": snap_time, "calibration_identity": "cal"})
        fault_on = sim > t_last
        if fault_on and override == "health_stale":
            value["adapter_health"] = {"status": "stale", "calibration_identity": "cal", "sim_s": sim}
        if override == "health_old":
            value["adapter_health"] = {"status": "fresh", "calibration_identity": "cal", "sim_s": min(sim, t_last)}
        if fault_on and override == "calibration":
            value["adapter_health"] = {"status": "fresh", "calibration_identity": "other", "sim_s": sim}
        if fault_on and override == "unusable":
            value["consecutive_unusable"] = 3
        if fault_on and override == "reset":
            value["reset_sim"] = sim - 0.01
        return value
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, candidate_ranker.P4Ranker())
    outs = run_supervisor(sup, 300, start=9.0, make_inputs=inputs)
    first = next((sim, o) for sim, o in outs if o["stale"] == reason)
    detect_limit = {"adapter_health_missing": t_last + 2.0 + 0.10}.get(reason, t_last + 0.35)
    assert first[1]["state"] == supervisor.HOVER and first[1]["frozen_yaw"] and first[0] <= detect_limit + 1e-9
    held = [o for sim, o in outs if sim >= first[0] and o["state"] == supervisor.HOVER]
    assert all(o["action"] == planner.HOLD for o in held)
    abort = next(sim for sim, o in outs if o["state"] == supervisor.ABORT)
    assert sup.abort_reason == "stale_inputs:" + reason and abs(abort - (first[0] + 10.0)) <= 0.10 + 1e-9


@needs_run
def test_R_F1_06_stamp_faults_on_recorded_frames_give_exactly_the_injected_rejections():
    d = declaration.build("A-5.F1.06")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    processor = pipeline.DepthProcessor(p3_config(), "m3", require_state=False)
    accepted_stamps = []
    for i, f in enumerate(recorded_frames(120)):
        if i == 5:
            injector.activate(f["sim"])
        for out in injector.image(f, f["sim"]):
            ok, _ = processor.on_image(out, out["stamp_ns"], out["frame_id"], 1000.0 + i * 0.1, f["sim"])
            accepted_stamps.append(out["stamp_ns"]) if ok else None
    assert processor.accounting.rejected == {"duplicate_acquisition_stamp": 5, "acquisition_stamp_backwards": 5,
                                             "missing_acquisition_stamp": 5}
    assert accepted_stamps == sorted(accepted_stamps)  # the pipeline never accepted an out-of-order frame


@needs_run
def test_R_F1_07_and_F1_12_layout_and_frame_id_faults_are_rejected_on_recorded_frames():
    d = declaration.build("A-5.F1.07")
    dim = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    dim.activate(0.0)
    fid = depth_faults.DepthInjector("F1.12", {"frame_id": "camera_optical"}, "active")
    fid.activate(0.0)
    processor = pipeline.DepthProcessor(p3_config(), "m3", require_state=False)
    for i, f in enumerate(recorded_frames(12)):
        for out in dim.image(f, f["sim"]) + fid.image(dict(f, stamp_ns=f["stamp_ns"] + 1), f["sim"]):
            processor.on_image(out, out["stamp_ns"], out["frame_id"], 1000.0 + i, f["sim"])
    assert processor.accounting.rejected == {"dimension_mismatch": 10, "unexpected_raw_frame_id": 12}


def corrupted_replay(transform, n, window=None):
    """Replay the first n recorded snapshots twice (clean and with `transform(image, k)` on frames inside
    `window` = (k0, k1)); returns the two grid sequences' final cells and the per-step cells."""
    rec = recorded()
    snaps = snapshots()[:n]
    clean, bad = rec.PerceptionReplay(RUN), rec.PerceptionReplay(RUN)
    history = []
    stream = rec.iter_depth_frames(RUN, [s["acquisition_stamp_ns"] for s in snaps])
    for k, (s, frame) in enumerate(zip(snaps, stream)):
        image = depth.decode(frame, p3_config())
        clean.step(s, image)
        assert np.array_equal(clean.grid.cells, local_map.Grid.decode(s["grid"]).cells)  # exact replay first
        faulty = transform(image.copy(), k) if window is None or window[0] <= k < window[1] else image
        bad.step(s, faulty)
        history.append((clean.grid.cells.copy(), bad.grid.cells.copy()))
    return clean.grid, bad.grid, history


@needs_run
def test_R_F1_08_nan_region_never_creates_free_or_occupied_space_on_recorded_frames():
    def nan_band(image, k):
        image[:, 200:440] = np.nan
        return image
    clean, bad, history = corrupted_replay(nan_band, 300)
    for c, b in history:
        assert not ((b == local_map.FREE) & (c != local_map.FREE)).any()
        assert not ((b == local_map.OCCUPIED) & (c != local_map.OCCUPIED)).any()
    lost = int(((clean.cells == local_map.FREE) & (bad.cells != local_map.FREE)).sum())
    measure("F1.08", recorded_snapshots=300, free_cells_lost_to_unknown=lost, free_or_occupied_created=0)


@needs_run
def test_R_F1_10_coverage_loss_only_removes_information_on_recorded_frames():
    injector = depth_faults.DepthInjector("F1.10", {"keep_every": 10}, "active")
    injector.activate(0.0)

    def coverage(image, k):
        frame = {"width": image.shape[1], "height": image.shape[0], "data": image.astype("<f4").tobytes(),
                 "stamp_ns": k + 1, "frame_id": "x", "encoding": "32FC1", "is_bigendian": 0, "step": image.shape[1] * 4}
        out = injector.image(frame, 1.0)[0]
        return np.frombuffer(out["data"], dtype="<f4").reshape(image.shape)
    clean, bad, history = corrupted_replay(coverage, 300, window=(150, 300))
    for c, b in history:
        assert not ((b == local_map.FREE) & (c != local_map.FREE)).any()
    measure("F1.10", unknown_ray_fraction_note="90 % NaN pixels make nearly every band column UNKNOWN",
            free_cells_lost=int(((clean.cells == local_map.FREE) & (bad.cells != local_map.FREE)).sum()))


def test_R_F1_11_calibration_change_makes_health_not_fresh_and_the_supervisor_hover():
    import test_px4_gazebo_p3_sensor as p3t
    from gwm_sensor_adapter import camera_info
    d = declaration.build("A-5.F1.11")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    injector.activate(0.0)
    cache = camera_info.CalibrationCache(p5.syn.sensors())
    cache.update(p3t.info_fields())
    cache.update(injector.camera_info(p3t.info_fields(), 1.0))
    assert not cache.usable  # health becomes calibration_unavailable: not fresh -> stale rule tested above


@needs_run
def test_R_F2_04_reset_epoch_bracket_makes_frames_unusable_and_the_supervisor_hover():
    history = association.Associator(300, 0.05)
    base = {"x": 0.0, "y": 0.0, "z": -2.0, "heading": 0.0, "xy_valid": True, "z_valid": True, "xy_reset_counter": 0,
            "z_reset_counter": 0, "heading_reset_counter": 0, "ref_timestamp": 1, "timestamp_sample": 0}
    for k in range(20):
        t = 10_000_000 + k * 10_000
        counter = 0 if k < 10 else 1  # reset between samples 9 and 10
        history.add_position(dict(base, timestamp=t, timestamp_sample=t, xy_reset_counter=counter))
        history.add_attitude({"timestamp": t, "timestamp_sample": t, "q": [1, 0, 0, 0], "quat_reset_counter": 0})
    assert history.associate(10_095_000)["status"] == "position:reset_epoch_bracket"
    grid = p4t.free_grid()
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, candidate_ranker.P4Ranker())
    out = sup.step(p4t.inputs(10.0, grid, reset_sim=9.99))
    assert out["state"] == supervisor.HOVER and out["stale"] == "estimator_reset_since_last_usable_frame"


@pytest.mark.parametrize("fault,params,reason", [
    ("F2.05", {"backwards_us": 100000}, "state_time_backwards:vehicle_local_position"),
    ("F2.06", {"future_us": 200000}, "clock_epoch_mismatch:vehicle_local_position"),
    ("F2.07", {}, "invalid_estimate"),
    ("F5.07", {"window_sim_s": 5.0}, "stale_state:vehicle_status"),
    ("F5.10", {}, "vehicle_health_invalid")])
def test_R_app_side_state_faults_hit_the_intended_p2_check(fault, params, reason):
    cache = timing.StateCache(m5.P2)
    injector = state_faults.StateInjector(fault, params, "active")
    raised = None
    for k in range(50):
        sim = 10.0 + k * 0.05
        cache.clock(sim, sim)
        if k == 1:
            injector.activate(sim)
        try:
            for name, data in m2.states(sim).items():
                for item in injector.state(name, data, sim):
                    cache.update(name, item, sim)
            cache.validate(sim, sim)
        except ValueError as exc:
            raised = (sim, str(exc))
            break
    assert raised is not None and raised[1].startswith(reason)
    if fault == "F5.07":  # stale only after the 1.5 s sim flags limit
        assert raised[0] - 10.05 > m5.P2["flags_fresh_sim_s"] - 0.05


@needs_run
def test_R_F2_09_withheld_pose_makes_recorded_frames_unassociable():
    rec = recorded()
    receipts = [r for r in rec.jsonl(RUN / "ros-events.jsonl") if r.get("event") == "received"
                and r.get("topic_key") in ("vehicle_local_position", "vehicle_attitude")]
    snaps = snapshots()
    cut = snaps[600]["acquisition_stamp_ns"] // 1000  # pose withheld from this acquisition time on
    history = association.Associator(300, 0.05)
    for r in receipts:
        f = r["fields"]
        if int(f.get("timestamp_sample") or f["timestamp"]) >= cut:
            continue
        (history.add_position if r["topic_key"] == "vehicle_local_position" else history.add_attitude)(f)
    before = [history.associate(s["acquisition_stamp_ns"] // 1000)["status"] for s in snaps[590:600]]  # inside the history span
    after = [history.associate(s["acquisition_stamp_ns"] // 1000)["status"] for s in snaps[606:700]]
    assert before.count("matched") == len(before)  # the recorded association is reproduced before the cut
    assert "matched" not in after and all(s.endswith("pending_newer_state") for s in after)


def test_R_F3_03_schema_change_refuses_start_before_any_node(tmp_path):
    from types import SimpleNamespace
    schema = json.loads((p5.P5 / "configs/p5_model_schema.json").read_text(encoding="utf-8"))
    schema["candidate_features"][0]["description"] = schema["candidate_features"][0].get("description", "") + " "
    path = tmp_path / "schema.json"
    path.write_text(json.dumps(schema), encoding="utf-8")
    args = SimpleNamespace(model_sha256="x", schema=path, schema_sha256=p5.SCHEMA_SHA, model=tmp_path / "absent.npz")
    with pytest.raises(ValueError, match="p5_model_schema_differs_from_frozen_hash"):
        control_node.verified_policy({"model": {"sha256": "x"}}, args)


def ranker_case(blocked=False):
    """One decision's inputs; `blocked` puts a pillar 2 m ahead: the long straight slots become invalid."""
    from gwm_policy_control.observation_adapter import progress_qualified
    grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], CONFIG["perception"]["grid_margin_m"],
                                 CONFIG["perception"]["grid_resolution_m"])
    grid.cells[:] = local_map.FREE
    if blocked:
        p4t.occupy(grid, 2.0, 2.4, -0.2, 0.2)
    ctx = planner.Context((0.0, 0.0), (0.0, 0.0), 0.0, tuple(CONFIG["mission"]["goal_enu_m"]), (0.0, 0.0), 0.05, CONFIG["geofence"])
    traversed = np.zeros(grid.cells.shape, dtype=bool)
    rows = planner.evaluate_candidates(grid, traversed, ctx, CONFIG["planner"], 0.25)
    progress = [r for r in rows if progress_qualified(r, ctx, CONFIG["planner"])[0]]
    return rows, progress, ctx, grid, traversed


class Wrapped:
    def __init__(self, inner, transform):
        self.inner, self.transform, self.sha256 = inner, transform, "wrapped"

    def scores(self, features):
        return self.transform(self.inner, features)


@pytest.mark.parametrize("fault,transform,expected", [
    ("F3.05", lambda p, x: policy_faults.corrupt_scores("F3.05", p.scores(x)), "output_shape"),
    ("F3.11", lambda p, x: (_ for _ in ()).throw(RuntimeError("p6")), "exception:RuntimeError"),
    ("F3.12", lambda p, x: p.scores(policy_faults.corrupt_features(x)), "input_not_finite"),
    ("F3.09", lambda p, x: (__import__("time").sleep(0.060), p.scores(x))[1], "timeout")])
def test_R_policy_faults_are_classified_by_the_accepted_ranker(fault, transform, expected):
    rows, progress, ctx, grid, traversed = ranker_case()
    ranker = PolicyRanker(Wrapped(p5.interface_policy(), transform), p5.SCHEMA)
    best, record = ranker.choose(rows, progress, ctx, grid, traversed, "NORMAL", CONFIG["planner"])
    assert best is None and record["failure"] == expected
    if fault == "F3.09":
        assert record["inference_s"] >= 0.060


def test_R_F3_06_selection_outside_the_mask_is_classified(monkeypatch):
    rows, progress, ctx, grid, traversed = ranker_case(blocked=True)
    assert 0 < len(progress) < 26  # some slots forbidden
    monkeypatch.setattr(candidate_ranker, "policy_choice", lambda scores, allowed, by_slot: policy_faults.outside_mask(allowed))
    best, record = PolicyRanker(p5.interface_policy(), p5.SCHEMA).choose(rows, progress, ctx, grid, traversed, "NORMAL",
                                                                         CONFIG["planner"])
    assert best is None and record["failure"] == "selection_outside_mask"
    assert policy_faults.outside_mask(np.ones(26, dtype=bool)) is None  # never an invented index
    measure("F3.06", note="the accepted re-check `allowed[choice]` would accept a negative index by Python wrap-around; "
                          "unreachable from `policy_choice` (it returns allowed indices only), and any such value would "
                          "raise KeyError outside the ranker's try, failing closed through P2 RECOVERY")


def test_R_F3_09_and_F6_03_overruns_trip_the_dispatch_guard_and_land_once():
    guard = execution_timing.DispatchGuard(0.050, 1.5)
    guard.begin(100.0)
    with pytest.raises(ValueError, match="stale_action"):
        guard.check(100.061, graph_checked=100.0, graph_valid=True)
    guard.end()
    rig = m5.Rig(m5.p5_mission(p5.interface_policy()))
    request = rig.m.abort("stale_action", rig.sim, rig.wall)
    assert request == {"command": 21, "params": [0.0] * 7} and rig.m.state == "RECOVERY"
    assert rig.m.abort("again", rig.sim, rig.wall) is None  # F7.05: never a second LAND


def test_R_F4_11_health_and_map_freshness_are_checked_independently():
    grid = p4t.free_grid()
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, candidate_ranker.P4Ranker())
    out = sup.step(p4t.inputs(10.0, grid, adapter_health={"status": "stale", "calibration_identity": "cal", "sim_s": 10.0}))
    assert out["stale"] == "adapter_health_not_fresh"
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, candidate_ranker.P4Ranker())
    out = sup.step(p4t.inputs(10.0, grid, snapshot={"acquisition_sim_s": 9.70, "calibration_identity": "cal"}))
    assert out["stale"] == "snapshot_stale"


def ack(command, result, sim):
    return {"command": command, "result": result, "target_system": m5.P2["source_system"],
            "target_component": m5.P2["source_component"], "from_external": False, "timestamp": int(sim * 1e6)}


@pytest.mark.parametrize("fault,state,command", [("F5.03", "REQUEST_OFFBOARD", 176), ("F5.06", "REQUEST_LAND", 21)])
def test_R_ack_rewrites_raise_command_ack_denied_in_the_accepted_mission(fault, state, command):
    rig = m5.Rig(m5.p5_mission(p5.interface_policy()))
    m = rig.m
    m.state = state
    m.request(command, rig.sim, rig.wall)
    injector = state_faults.AckInjector(fault, {}, "active")
    for item in injector.ack(ack(command, 0, rig.sim), rig.sim):
        m.ack(item, rig.sim, rig.wall)
    assert m.ack_status == "denied"
    rig.sim += 0.05
    rig.wall += 0.05
    rig.publish_state()
    with pytest.raises(ValueError, match="command_ack_denied"):
        m.tick(rig.sim, rig.wall, True)


def test_R_F5_05_wrong_nav_state_is_offboard_armed_state_lost():
    rig = m5.Rig(m5.p5_mission(p5.interface_policy()))
    rig.sim += 0.05
    rig.wall += 0.05
    rig.publish_state(nav_state=4)
    with pytest.raises(ValueError, match="offboard_armed_state_lost"):
        rig.m.tick(rig.sim, rig.wall, True)
    assert rig.m.abort("offboard_armed_state_lost", rig.sim, rig.wall) is None  # not nav 14: no LAND, PX4 fallback


def test_R_F7_02_alternating_failures_never_reach_policy_unavailable():
    grid = blocked_grid()
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, FailingRanker(pattern=lambda n: n % 2 == 1))
    outs = run_supervisor(sup, 400, make_inputs=lambda sim: p4t.inputs(sim, grid))
    assert all(o["state"] != supervisor.ABORT for _, o in outs)
    assert sup.consecutive_policy_failures <= 1


LEGAL = {("NORMAL", "OBSTACLE_DETECTED"), ("NORMAL", "HOVER"), ("OBSTACLE_DETECTED", "AVOIDING"),
         ("OBSTACLE_DETECTED", "HOVER"), ("AVOIDING", "RESUME"), ("AVOIDING", "HOVER"), ("RESUME", "NORMAL"),
         ("RESUME", "HOVER"), ("HOVER", "AVOIDING")}


def legal(transition):
    a, b = transition["from"], transition["to"]
    return (a, b) in LEGAL or b == supervisor.ABORT or (b == supervisor.HOVER and transition["reason"].startswith("stale_inputs"))


def test_R_F7_04_and_F7_06_fuzzed_supervisor_transitions_are_legal_and_abort_is_absorbing():
    rng = np.random.default_rng(20261107)
    count = 0
    for episode in range(60):
        grid = p4t.free_grid()
        for _ in range(rng.integers(0, 4)):
            x, y = rng.uniform(1.0, 7.0), rng.uniform(-2.0, 2.0)
            p4t.occupy(grid, x - 0.2, x + 0.2, y - 0.2, y + 0.2)
        ranker = FailingRanker(pattern=lambda n, p=rng.uniform(0, 0.4), r=rng: r.random() < p)
        sup = safety_shield.PolicySupervisor(CONFIG, 0.0, ranker)
        target = [0.0, 0.0]
        for k in range(80):
            sim = 10.0 + 0.05 * k
            stale = rng.random() < 0.05
            value = p4t.inputs(sim, grid, target=tuple(target), vehicle=tuple(target),
                               snapshot=None if stale else {"acquisition_sim_s": sim - 0.05, "calibration_identity": "cal"})
            out = sup.step(value)
            if out["action"] == planner.TRANSLATE and out["end"] is not None:
                target = list(out["end"])
            if sup.state == supervisor.ABORT:
                reason = sup.abort_reason
                for j in range(5):
                    again = sup.step(p4t.inputs(sim + 0.05 * (j + 1), grid))
                    assert again["state"] == supervisor.ABORT and sup.abort_reason == reason
                break
        assert all(legal(t) for t in sup.transitions), sup.transitions
        count += len(sup.transitions)
    assert count > 60


@needs_run
def test_R_F7_04_every_recorded_accepted_flight_transition_is_legal():
    runs = sorted(RUN.parent.glob("20261106T13*-p5-flight-*"))
    seen = 0
    for run in runs:
        result = json.loads((run / "controller-result.json").read_text(encoding="utf-8"))
        for t in result["p4"]["supervisor_transitions"]:
            assert legal(t), (run.name, t)
            seen += 1
    assert len(runs) == 7 and seen > 0


def test_R_F7_07_target_moves_only_by_apply_action_and_never_while_held():
    rng = np.random.default_rng(7)
    for _ in range(2000):
        target, yaw = (rng.uniform(-1, 9), rng.uniform(-3, 3)), rng.uniform(-math.pi, math.pi)
        action = rng.choice([planner.HOLD, planner.TRANSLATE, planner.YAW_TO, planner.SCAN])
        frozen = bool(rng.random() < 0.3)
        end = (target[0] + rng.uniform(-1.5, 1.5), target[1] + rng.uniform(-1.5, 1.5))
        decision = {"action": action, "frozen_yaw": frozen, "state": "AVOIDING", "heading": rng.uniform(-math.pi, math.pi),
                    "end": end}
        new_target, new_yaw = supervisor.apply_action(target, yaw, decision, 0.2, math.radians(10), 0.05, (8.0, 0.0))
        if action == planner.HOLD or frozen:
            assert (new_target, new_yaw) == (target, yaw)
        if action != planner.TRANSLATE:
            assert new_target == target
        assert math.dist(new_target, target) <= 0.2 * 0.05 + 1e-9
        assert abs(planner.wrap(new_yaw - yaw)) <= math.radians(10) * 0.05 + 1e-9


def test_R_F8_04_policy_failure_rule_precedes_the_transition_table():
    grid = blocked_grid()
    sup = safety_shield.PolicySupervisor(CONFIG, 0.0, FailingRanker())
    outs = run_supervisor(sup, 3, make_inputs=lambda sim: p4t.inputs(sim, grid))
    assert [o["state"] for _, o in outs] == [supervisor.NORMAL, supervisor.NORMAL, supervisor.ABORT]
    assert sup.abort_reason == "policy_unavailable" and not any(t["to"] == "OBSTACLE_DETECTED" for t in sup.transitions)


def test_R_F8_05_a_controller_started_against_an_armed_vehicle_refuses():
    cache = timing.StateCache(m5.P2)
    m = P4Mission(m5.P2, CONFIG, cache, 0.0, True)
    cache.clock(5.0, 5.0)
    for name, data in m2.states(5.0).items():
        cache.update(name, data, 5.0)
    with pytest.raises(ValueError, match="restart_reject_armed"):
        m.tick(5.0, 5.0, True)


# ===================================================================================================
# Characterization (C): measured, reported, never gated
# ===================================================================================================
def footprint_mask(grid, box, margin=0.0):
    (cx, cy), (sx, sy) = box["center"], box["size"]
    ii, jj = np.meshgrid(np.arange(grid.nx), np.arange(grid.ny), indexing="ij")
    x, y = grid.x_min + (ii + 0.5) * grid.resolution, grid.y_min + (jj + 0.5) * grid.resolution
    return (np.abs(x - cx) <= sx / 2 + margin) & (np.abs(y - cy) <= sy / 2 + margin)


PILLAR = {"center": [4.0, 0.0], "size": [0.4, 0.4]}
GOAL = tuple(CONFIG["mission"]["goal_enu_m"])


def segment_hits(start, end, box, margin):
    """The straight segment passes within `margin` of the box (sampled every 0.02 m)."""
    (cx, cy), (sx, sy) = box["center"], box["size"]
    n = max(1, int(math.ceil(math.dist(start, end) / 0.02)))
    for k in range(n + 1):
        x = start[0] + (end[0] - start[0]) * k / n
        y = start[1] + (end[1] - start[1]) * k / n
        if abs(x - cx) <= sx / 2 + margin and abs(y - cy) <= sy / 2 + margin:
            return True
    return False


def unsafe_candidates(grid, traversed, vehicle_xy, box):
    """Collision-valid P4 candidates (the accepted L2 test) whose segment passes within R_body of the true
    obstacle: each one is a move the shield would allow into the real obstacle."""
    ctx = planner.Context(vehicle_xy, vehicle_xy, 0.0, GOAL, (0.0, 0.0), 0.05, CONFIG["geofence"])
    rows = planner.evaluate_candidates(grid, traversed, ctx, CONFIG["planner"], CONFIG["perception"]["snapshot_max_age_sim_s"])
    return sum(1 for r in rows if r["valid"] and segment_hits(ctx.target, r["end"], box, R_BODY))


@functools.lru_cache(maxsize=None)
def pillar_first_observed():
    snaps = snapshots()
    mask = footprint_mask(local_map.Grid.decode(snaps[0]["grid"]), PILLAR)
    return next(k for k, s in enumerate(snaps) if (local_map.Grid.decode(s["grid"]).cells[mask] == local_map.OCCUPIED).any())


PROBES = ((1.5, 0.0), (2.0, 0.0), (2.3, 0.0), (2.3, 0.4), (2.3, -0.4))  # approach points in front of the pillar


def hazard(history, window, every=3):
    """Over sampled snapshots from the window on: FREE cells inside the true footprint, and unsafe candidates from
    the recorded vehicle position and from fixed approach points (would the shield allow a move into the real pillar
    with this map?), corrupted versus clean (maxima)."""
    snaps = snapshots()
    out = {"footprint_free_bad": 0, "footprint_free_clean": 0, "unsafe_candidates_bad": 0, "unsafe_candidates_clean": 0}
    for k in range(window[0], len(history), every):
        n, e, _ = snaps[k]["position_ned"]
        for tag, cells in (("clean", history[k][0]), ("bad", history[k][1])):
            grid = local_map.Grid.decode(snaps[k]["grid"])
            grid.cells[:] = cells
            path = [(s["position_ned"][1], s["position_ned"][0]) for s in snaps[:k + 1:5]]
            traversed = planner.traversed_mask(grid, path, R_BODY)  # as P4Mission keeps it
            free = int(((cells == local_map.FREE) & footprint_mask(grid, PILLAR)).sum())
            unsafe = max(unsafe_candidates(grid, traversed, xy, PILLAR) for xy in ((e, n),) + PROBES)
            out["footprint_free_" + tag] = max(out["footprint_free_" + tag], free)
            out["unsafe_candidates_" + tag] = max(out["unsafe_candidates_" + tag], unsafe)
    return out


@needs_run
def test_C_F1_09_inf_corruption_marks_unobserved_obstacle_cells_free_on_recorded_frames():
    first = pillar_first_observed()
    window = (max(0, first - 30), first + 30)

    def infinity(image, k):
        image[:, 240:400] = np.inf
        return image
    _, _, history = corrupted_replay(infinity, window[1] + 60, window=window)
    created = [int(((b == local_map.FREE) & (c != local_map.FREE)).sum()) for c, b in history]
    found = hazard(history, window)
    measure("F1.09", corrupted_columns=[240, 400], window_snapshots=list(window), pillar_first_occupied_snapshot=first,
            max_free_cells_created_vs_clean=max(created), **found)
    assert max(created) > 0  # the gap is real: corrupted no-return creates FREE space the clean map never had


@needs_run
def test_C_F1_05_frozen_frames_project_old_content_at_new_poses_on_recorded_frames():
    first = pillar_first_observed()
    window = (max(0, first - 30), first + 30)
    held = {}

    def frozen(image, k):
        held.setdefault("image", image.copy())
        return held["image"].copy()
    _, _, history = corrupted_replay(frozen, window[1] + 60, window=window)
    created = [int(((b == local_map.FREE) & (c != local_map.FREE)).sum()) for c, b in history]
    misplaced = [int(((b == local_map.OCCUPIED) & (c != local_map.OCCUPIED)).sum()) for c, b in history]
    measure("F1.05", window_snapshots=list(window), max_free_cells_created_vs_clean=max(created),
            max_occupied_cells_misplaced=max(misplaced), **hazard(history, window))


@needs_run
def test_C_F4_03_reordered_snapshots_can_regress_the_controllers_occupancy():
    from gwm_policy_control.mission import P5Mission
    snaps = recorded().jsonl(RUN / "p4/local_map.jsonl")
    order = []
    for k in range(0, len(snaps) - 1, 2):
        order += [snaps[k + 1], snaps[k]]
    m = P5Mission(m5.P2, CONFIG, m5.P5C, p5.interface_policy(), p5.SCHEMA, timing.StateCache(m5.P2), 0.0, True)
    regressions, worst, newest = 0, 0, None
    for s in order:
        before = None if m.grid is None else int((m.grid.cells == local_map.OCCUPIED).sum())
        m.on_snapshot(s)
        newest = s if newest is None or s["acquisition_sim_s"] > newest["acquisition_sim_s"] else newest
        if before is not None and m.grid is not None:
            lost = before - int((m.grid.cells == local_map.OCCUPIED).sum())
            if lost > 0:
                regressions += 1
                worst = max(worst, lost)
    measure("F4.03", snapshots=len(order), regressions=regressions, max_occupied_cells_lost=worst,
            note="each regression lasts until the next snapshot (about 0.1 s) and stays inside the 0.25 s age limit")
    assert regressions > 0  # on_snapshot keeps the newest received, not the newest acquired


def synthetic_grid_through(path_xy, boxes, offset=(0.0, 0.0), grid=None, scan=False):
    """Accepted P4 perception on synthetic depth along a path; poses reported with `offset` subtracted
    (an estimator reset shifts the reported frame), the world stays fixed. `scan` first performs the
    MAP_SCAN revolution at the first path point, as every real flight does."""
    from gwm_sensor_adapter.geometry import Extrinsic
    S, P = p5.syn.sensors(), CONFIG["perception"]
    if grid is None:
        grid = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    scene = p5.syn.boxes_3d(boxes)
    poses = [(path_xy[0], math.radians(a)) for a in range(0, 360, 10)] if scan else []
    for (x, y), yaw in poses + [(xy, 0.0) for xy in path_xy]:
        physical = (x, y, 2.0)
        image = p5.syn.render(physical, yaw, scene, columns=m5.COLUMNS)
        reported = (x - offset[0], y - offset[1], 2.0)
        position, q = p5.syn.px4_pose(reported, yaw)
        cam, rays = perception.extract_scan(image, p5.syn.CAL, Extrinsic(S["optical_origin_base_flu_m"]), position, q, P,
                                            S["near_m"], S["far_m"], 0.0)
        perception.integrate(grid, cam, rays, P["max_hit_range_m"], S["near_m"])
    return grid


@pytest.mark.parametrize("shift", [0.5, 1.0])
def test_C_F2_04_map_reused_after_an_estimator_reset_misplaces_the_obstacle(shift):
    """Physically the vehicle hovers at 2 m; after an XY reset of `shift` it is reported at 2 - shift, so in the new
    frame the pillar is at 4 - shift while the reused grid still has it at 4 m."""
    boxes = [PILLAR]
    path = [(0.1 * k, 0.0) for k in range(21)]
    grid = synthetic_grid_through(path, boxes, scan=True)  # MAP_SCAN, then 0 to 2 m
    true_box = {"center": [4.0 - shift, 0.0], "size": PILLAR["size"]}
    reported = (2.0 - shift, 0.0)
    traversed = planner.traversed_mask(grid, path, R_BODY)  # recorded before the reset, in the old frame
    before = {"footprint_free": int(((grid.cells == local_map.FREE) & footprint_mask(grid, true_box)).sum()),
              "unsafe_candidates": unsafe_candidates(grid, traversed, reported, true_box)}
    synthetic_grid_through([(2.0, 0.0)], boxes, offset=(shift, 0.0), grid=grid)  # first fresh frame, reported shifted
    after = {"footprint_free": int(((grid.cells == local_map.FREE) & footprint_mask(grid, true_box)).sum()),
             "unsafe_candidates": unsafe_candidates(grid, traversed, reported, true_box)}
    measure("F2.04", **{"shift_%.1f_m" % shift: {"before_reobservation": before, "after_first_fresh_frame": after}})
    if shift >= 1.0:
        assert before["unsafe_candidates"] > 0  # the reused map allows a move into the real obstacle
    assert after["unsafe_candidates"] == 0  # one fresh matched frame re-marks it (states only rise)


def test_C_F4_10_restarted_perception_misaligns_the_kept_traversed_mask():
    P = CONFIG["perception"]
    old = local_map.Grid.around((0.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    path = [(0.1 * k, 0.0) for k in range(31)]
    traversed = planner.traversed_mask(old, path, R_BODY)
    new = local_map.Grid.around((3.0, 0.0), CONFIG["geofence"], P["grid_margin_m"], P["grid_resolution_m"])
    assert new.cells.shape == old.cells.shape  # so P4Mission.on_snapshot keeps the old mask
    over_pillar = int((traversed & footprint_mask(new, PILLAR)).sum())
    kept = unsafe_candidates(new, traversed, (3.0, 0.0), PILLAR)  # new grid all UNKNOWN; old mask kept, misaligned
    reset = unsafe_candidates(new, np.zeros(new.cells.shape, dtype=bool), (3.0, 0.0), PILLAR)
    measure("F4.10", restart_at_m=3.0, traversed_cells_inside_pillar_footprint=over_pillar,
            unsafe_candidates_with_kept_mask=kept, unsafe_candidates_if_mask_were_reset=reset)
    assert over_pillar > 0 and reset == 0


@needs_run
@pytest.mark.parametrize("fault", ["F2.02", "F2.03", "F2.08"])
def test_C_app_side_pose_faults_change_the_accepted_decisions(fault):
    import p5_policy_eval
    policy, schema = m5.trained_policy(), p5.SCHEMA
    changed = total = 0
    for _, event, _row, grid, traversed, ctx, config in list(p5_policy_eval.reproduced_ticks(RUN))[:120]:
        settings = config["planner"]
        target, vehicle, yaw = ctx.target, ctx.vehicle, ctx.vehicle_yaw
        if fault == "F2.02":
            vehicle = (vehicle[0] + 0.5, vehicle[1] + 0.5)
        elif fault == "F2.03":
            yaw = planner.wrap(yaw + math.radians(20))
        else:
            vehicle, target = (vehicle[1], vehicle[0]), (target[1], target[0])
        faulty = planner.Context(target, vehicle, yaw, ctx.goal, ctx.origin, ctx.snapshot_age, ctx.geofence, ctx.scanned)
        args = (grid, traversed, settings, config["envelope"], config["perception"]["snapshot_max_age_sim_s"])
        clean = safety_shield.decision_module.plan_with_ranker(grid, traversed, ctx, *args[2:], PolicyRanker(policy, schema), event["state"])
        bad = safety_shield.decision_module.plan_with_ranker(grid, traversed, faulty, *args[2:], PolicyRanker(policy, schema), event["state"])
        total += 1
        changed += int((clean["action"], clean["heading"], clean["end"]) != (bad["action"], bad["heading"], bad["end"]))
    measure(fault, recorded_ticks=total, decisions_changed=changed)
    assert total > 50


@needs_run
def test_C_F4_08_a_fabricated_usable_snapshot_is_accepted_without_any_check():
    from gwm_policy_control.mission import P5Mission
    s = snapshots()[700]
    grid = local_map.Grid.decode(s["grid"])
    grid.cells[:] = local_map.FREE
    forged = dict(s, grid=grid.encode(), observation_id=10**9)
    m = P5Mission(m5.P2, CONFIG, m5.P5C, p5.interface_policy(), p5.SCHEMA, timing.StateCache(m5.P2), 0.0, True)
    m.on_snapshot(forged)
    accepted = bool((m.grid.cells == local_map.FREE).all())
    measure("F4.08", forged_all_free_snapshot_accepted=accepted, check_present=False)
    assert accepted


def test_C_F5_02_first_tick_after_a_stall_sees_old_state_and_old_sim_time():
    """Settles the M0 open item from the rclpy (Jazzy) executor order: within one wait set ready timers run
    before ready subscriptions, one callback per spin_once, so the tick after SIGCONT runs with the pre-stop
    cache *and* the pre-stop /clock. Its ages look fresh: neither stale_state nor observation_gap can fire on it."""
    cache = timing.StateCache(m5.P2)
    for k in range(10):
        sim = 10.0 + 0.05 * k
        cache.clock(sim, sim)
        for name, data in m2.states(sim).items():
            cache.update(name, data, sim)
    stall = 0.3
    sample = cache.validate(10.45, 10.45 + stall)  # sim unchanged (queued /clock), wall advanced by the stall
    assert sample["t"] == pytest.approx(10.45)
    measure("F5.02", first_tick_after_stall="passes every P2 check on pre-stop state (sim time also pre-stop)",
            later_ticks="queues drain one message per subscription per wait-set pass; observation_gap fires only "
                        "if consecutive ticks' samples differ by > 0.2 s sim; most likely the stall goes undetected "
                        "in flight and only the post-run P2 timing evaluation sees the publication gap",
            px4="no failsafe: 0.3 s < COM_OF_LOSS_T 1.0 s", confirm="characterization flight X-F5.02")


# ===================================================================================================
# Rehearsals of the flight rows not rehearsed in M2 (decision logic at the mission inputs)
# ===================================================================================================
class FaultRig(m5.Rig):
    """The accepted rig; after the trigger the perception input follows `mode`."""

    def __init__(self, make_mission, boxes, trigger, mode, windows=None):
        self.trigger, self.mode, self.windows, self.fed = trigger, mode, windows, 0
        super().__init__(make_mission, boxes)

    def perceive(self):
        t = self.trigger.fired_sim if self.trigger is not None else None
        if t is not None:
            if self.mode == "stop":
                return
            if self.mode == "unusable":
                self.m.on_snapshot({"usable": False, "consecutive_unusable": self.m.consecutive_unusable + 1})
                return
            if self.mode == "bursts" and self.windows.active(self.sim - t):
                return
        super().perceive()

    def step(self):
        actions = super().step()
        if self.trigger is not None:
            if self.trigger.origin_enu is None:
                self.trigger.origin_enu = self.m.origin_enu
            for event in self.m.events[self.fed:]:
                self.trigger.feed(event)
            self.fed = len(self.m.events)
        return actions


def rehearse(boxes, trigger_name, mode, windows=None, row=None, limit=200.0):
    clock = m2.Clock()
    policy = m5.trained_policy()
    from gwm_policy_control.mission import P5Mission
    factory = lambda cache: P5Mission(m5.P2, m5.P4C, m5.P5C, policy, p5.SCHEMA, cache, 0.0, True, clock=clock)
    shim = controller_shim.Shim(declaration.build(row), sleep=clock.sleep, boxes=boxes).install() if row else None
    try:
        rig = FaultRig(factory, boxes, schedule.Trigger(trigger_name, boxes=boxes), mode, windows)
        end = rig.sim + limit
        while rig.sim < end and rig.m.state in ("MAP_SCAN", "TRANSIT"):
            rig.step()
        return rig.m, rig.trigger, clock
    finally:
        if shim is not None:
            shim.uninstall()


def decisions(m, after=None):
    return [e for e in m.events if e["event"] == "p4_decision" and (after is None or e["sim_s"] > after)]


@m5.needs_artifact
@pytest.mark.parametrize("row,trigger,mode", [("S-1", "avoiding_near", "stop"), ("S-2'", "avoiding_first", "unusable")])
def test_rehearsal_sensor_loss_rows_hover_then_abort_snapshot_stale_and_land(row, trigger, mode):
    m, t, _ = rehearse(m2.S2_BOXES, trigger, mode)
    assert t.fired_sim is not None and m.p4_abort == "p4_supervisor_abort:stale_inputs:snapshot_stale"
    after = decisions(m, t.fired_sim)
    hover = next(e for e in after if e["state"] == "HOVER")
    last_snapshot = max(e["snapshot_acquisition_sim_s"] for e in after)
    assert hover["sim_s"] <= last_snapshot + 0.35 + 1e-9 and hover["frozen_yaw"]
    held = [e for e in after if e["state"] == "HOVER"]
    assert max(math.dist(e["target_enu"], held[0]["target_enu"]) for e in held) <= 0.01
    assert m.state == "REQUEST_LAND"


@m5.needs_artifact
def test_rehearsal_C1_depth_bursts_stale_exactly_when_old_and_recover_to_the_goal():
    windows = schedule.Windows([0.0, 4.0, 8.0], [0.1, 0.5, 1.5])
    m, t, _ = rehearse([{"center": [4.0, 0.5], "size": [0.4, 0.4]}], "avoiding_first", "bursts", windows)
    for e in decisions(m, t.fired_sim):
        age = e["sim_s"] - e["snapshot_acquisition_sim_s"]
        assert (e["stale"] == "snapshot_stale") == (age > CONFIG["perception"]["snapshot_max_age_sim_s"])  # the supervisor's own test
    assert m.state == "GOAL_HOVER"


@m5.needs_artifact
def test_rehearsal_C4_staleness_precedes_the_policy_and_the_run_aborts_and_lands():
    m, t, clock = rehearse(m2.S2_BOXES, "avoiding_first", "stop", row="C-4")
    stale = [e["sim_s"] for e in decisions(m, t.fired_sim) if e["stale"] is not None]
    if stale:  # once stale, the policy is never invoked (staleness precedes planning)
        assert not [r for r in m.ledger if r["invoked"] and r["sim_s"] >= stale[0]]
    else:
        assert m.p4_abort == "p4_supervisor_abort:policy_unavailable"  # three timeouts came before staleness
    assert m.p4_abort in ("p4_supervisor_abort:policy_unavailable", "p4_supervisor_abort:stale_inputs:snapshot_stale")
    measure("C-4 rehearsal", abort=m.p4_abort, policy_failures_before_stale=sum(
        1 for r in m.ledger if r["invoked"] and r["failure"] is not None))


@m5.needs_artifact
def test_rehearsal_C5_map_loss_part_hovers_and_completes():
    m, _, _, shim = m2.fly([{"center": [3.5, 0.0], "size": [0.4, 0.4]}, {"center": [6.0, -1.4], "size": [0.4, 0.4]}],
                           "C-5", limit=220.0)
    t0 = shim.trigger.fired_sim
    assert any(e["stale"] == "snapshot_stale" and e["state"] == "HOVER" for e in decisions(m, t0) if e["sim_s"] <= t0 + 1.4)
    assert m.state in ("GOAL_HOVER", "REQUEST_LAND")  # recovery or A10; the load part needs SITL
