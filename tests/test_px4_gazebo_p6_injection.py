"""P6 M2: fault injectors, offline (no ROS, no PX4). Every injected fault is checked against the
accepted P3/P4/P5 code it targets, and every pass-through against the unwrapped accepted code.

The mission flights use the accepted P5 mission rig with a deterministic policy clock, so inference
times (and therefore timeouts) are exact and the comparisons are bit for bit.
"""
import ast
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p5_synthetic as p5  # noqa: E402
import test_px4_gazebo_p5_mission as m5  # noqa: E402  (the accepted P5 mission rig)
from gwm_obstacle_avoidance import local_map, planner  # noqa: E402
from gwm_policy_control import candidate_ranker, mission as p5_mission_module, observation_adapter  # noqa: E402
from gwm_policy_control.policy_runtime import FrozenPolicy, PolicyError  # noqa: E402
from gwm_px4_control import timing  # noqa: E402
from gwm_sensor_adapter import camera_info, contracts as sensor_contracts, depth, pipeline  # noqa: E402

REPO = p5.syn.REPO
P6 = REPO / "simulation/px4_gazebo_p6"
sys.path.insert(0, str(P6))
from injection import (controller_shim, declaration, depth_faults, fault_ledger, load_harness,  # noqa: E402
                       policy_faults, process_faults, schedule, specs, state_faults)

SENSORS = sensor_contracts.load_config(p5.syn.SIM / "configs/p3_sensors.yaml")
S2_BOXES = [{"center": [4.0, 0.0], "size": [0.4, 0.4]}]
S4_BOXES = [{"center": [3.5, 0.0], "size": [0.4, 0.4]}, {"center": [6.0, -1.4], "size": [0.4, 0.4]}]
POSIX = sys.platform.startswith("linux")


# ---- ledger and declaration ---------------------------------------------------------------------
def test_ledger_is_exclusive_sequenced_strict_and_validated(tmp_path):
    path = tmp_path / "ledger.jsonl"
    ledger = fault_ledger.FaultLedger(path, "B-1", "F3.08", "controller_shim", clock=lambda: 1.0)
    ledger.record("declared", None, declaration={"row": "B-1"})
    ledger.record("affected", 12.5, item="policy_invocation")
    with pytest.raises(ValueError):
        ledger.record("affected", float("nan"))
    with pytest.raises(ValueError):
        ledger.record("guessed", 1.0)
    ledger.close()
    records = fault_ledger.read(path)
    assert [r["seq"] for r in records] == [1, 2] and records[1]["sim_s"] == 12.5
    with pytest.raises(FileExistsError):
        fault_ledger.FaultLedger(path, "B-1", "F3.08", "controller_shim")


def test_declarations_are_the_frozen_rows_and_nothing_else(tmp_path):
    d = declaration.build("B-1")
    assert d["fault"] == "F3.08" and d["mechanisms"] == ["CS"] and d["mode"] == "active"
    assert d["parameters"] == {"sleep_s": 0.025, "consecutive_invocations": 3} and d["trigger"] == "avoiding_first"
    assert declaration.uses(d, "CS") == "active" and declaration.uses(d, "DR") == "off"
    passthrough = declaration.build("B-0")
    assert passthrough["mode"] == "pass-through" and passthrough["mechanisms"] == ["CS", "DR", "SR"]
    with pytest.raises(ValueError):
        declaration.build("B-0", "active")
    assert declaration.build("C-4")["mechanisms"] == ["PS", "CS"]
    path = tmp_path / "decl.json"
    declaration.write(path, d)
    assert declaration.load(path) == d
    tampered = dict(d, parameters={"sleep_s": 0.010, "consecutive_invocations": 3})
    with pytest.raises(ValueError, match="differs from the frozen scenario row"):
        declaration.validate(tampered)
    with pytest.raises(ValueError, match="approved bound"):
        declaration._check_bounds({"load_workers": 25})
    with pytest.raises(ValueError, match="approved bound"):
        declaration._check_bounds({"load_window_s": 61.0})


# ---- schedule -----------------------------------------------------------------------------------
def decision(sim, state, xy):
    return {"event": "p4_decision", "sim_s": sim, "state": state, "vehicle_enu": list(xy)}


def test_triggers_fire_once_on_the_controllers_own_decision_events():
    origin = {"event": "transition", "to": "PRESTREAM_SAFE_SETPOINTS", "evidence": {"origin_ned": [0.2, 0.1, 0.0]}}
    first = schedule.Trigger("transit_start")
    assert first.feed(decision(50.0, "NORMAL", (0.1, 0.2))) == 50.0 == first.feed(decision(51.0, "NORMAL", (0, 0)))
    avoiding = schedule.Trigger("avoiding_first")
    assert avoiding.feed(decision(50.0, "OBSTACLE_DETECTED", (1, 0))) is None
    assert avoiding.feed(decision(50.05, "AVOIDING", (1, 0))) == 50.05
    near = schedule.Trigger("avoiding_near", boxes=S2_BOXES)
    near.feed(origin)
    assert near.origin_enu == (0.1, 0.2)
    assert near.feed(decision(60.0, "AVOIDING", (2.1, 0.2))) is None  # 2.0 m from the pillar
    assert near.feed(decision(61.0, "AVOIDING", (2.7, 0.2))) == 61.0  # 1.4 m
    normal = schedule.Trigger("normal_2m")
    normal.feed(origin)
    assert normal.feed(decision(70.0, "NORMAL", (2.0, 0.2))) is None
    assert normal.feed(decision(71.0, "NORMAL", (2.11, 0.2))) == 71.0
    with pytest.raises(ValueError):
        schedule.Trigger("whenever")


def test_windows_and_invocation_plans():
    w = schedule.Windows([0.0, 4.0, 8.0], [0.1, 0.5, 1.5])
    assert [w.active(t) for t in (0.0, 0.09, 0.1, 4.2, 4.6, 9.4, 9.6)] == [True, True, False, True, False, True, False]
    plan = schedule.InvocationPlan("consecutive", count=3)
    assert [plan.faulted(t) for t in range(5)] == [True, True, True, False, False]
    plan = schedule.InvocationPlan("offsets", offsets=[5.0, 10.0])
    assert [plan.faulted(t) for t in (0.0, 4.95, 5.0, 5.05, 9.0, 10.2, 10.25)] == [False, False, True, False, False, True, False]
    plan = schedule.InvocationPlan("alternating")
    assert [plan.faulted(0) for _ in range(4)] == [True, False, True, False]


# ---- depth relay transforms against the accepted P3 decoder and pipeline ------------------------
def frame(i, value=5.0, start_ns=4_392_000_000):
    image = np.full((480, 640), value, dtype="<f4")
    return {"width": 640, "height": 480, "encoding": "32FC1", "is_bigendian": 0, "step": 2560, "data": image.tobytes(),
            "stamp_ns": start_ns + i * 32_000_000, "frame_id": SENSORS["raw_frame"]}


def sim_of(f):
    return f["stamp_ns"] / 1e9 + 0.016


def test_depth_pass_through_returns_the_very_same_frame():
    for row in ("C-1", "A-5.F1.06", "A-5.F1.07"):
        d = declaration.build(row)
        injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "pass-through")
        injector.activate(0.0)
        f = frame(0)
        assert injector.image(f, 10.0)[0] is f and not injector.affected


def test_depth_bursts_drop_exactly_inside_the_declared_windows():
    d = declaration.build("C-1")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    frames = [frame(i) for i in range(400)]
    injector.activate(sim_of(frames[10]))
    delivered = [f for f in frames for f in injector.image(f, sim_of(f))]
    dropped = {i for i, f in enumerate(frames) if f not in delivered}
    rel = [sim_of(f) - injector.trigger_sim for f in frames]
    expected = {i for i, t in enumerate(rel) if schedule.Windows([0, 4, 8], [0.1, 0.5, 1.5]).active(t)}
    assert dropped == expected and len(dropped) > 0


def test_depth_dimension_fault_is_rejected_by_the_accepted_decoder_as_declared():
    d = declaration.build("A-5.F1.07")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    injector.activate(0.0)
    reasons = []
    for i in range(15):
        outputs = injector.image(frame(i), 1.0)
        injector.affected.clear()  # the relay drains this list into its ledger after every frame (M5 live finding)
        for out in outputs:
            try:
                depth.decode(out, SENSORS)
                reasons.append("decoded")
            except depth.FrameRejected as exc:
                reasons.append(str(exc))
    assert reasons == ["dimension_mismatch"] * 10 + ["decoded"] * 5


def test_depth_stamp_faults_give_exactly_the_declared_rejection_counts_in_the_accepted_pipeline():
    d = declaration.build("A-5.F1.06")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    processor = pipeline.DepthProcessor(SENSORS, "p6-test", require_state=False)
    for i in range(400):  # 12.8 s of frames: 15 faulted stamps, a 5 s future window, then order restored
        f = frame(i)
        if i == 20:
            injector.activate(sim_of(f))
        for out in injector.image(f, sim_of(f)):
            processor.on_image(out, out["stamp_ns"], out["frame_id"], 1000.0 + i * 0.032, sim_of(f))
    assert processor.accounting.rejected == {"duplicate_acquisition_stamp": 5, "acquisition_stamp_backwards": 5,
                                             "missing_acquisition_stamp": 5}
    kinds = [what for _, what in injector.affected]
    assert kinds.count("future") > 100 and kinds.count("dropped_to_restore_order") > 10


def test_depth_value_faults_are_classified_by_the_accepted_code_as_intended():
    for fault, code in (("F1.08", depth.INVALID_NAN), ("F1.09", depth.NO_RETURN)):
        injector = depth_faults.DepthInjector(fault, {"rows": [100, 300], "cols": [200, 400]}, "active")
        injector.activate(0.0)
        out = injector.image(frame(0), 1.0)[0]
        codes = depth.classify(depth.decode(out, SENSORS), SENSORS["near_m"], SENSORS["far_m"])
        assert (codes[100:300, 200:400] == code).all() and (codes[:100] == depth.VALID).all()


def test_calibration_fault_changes_or_invalidates_the_accepted_calibration():
    import test_px4_gazebo_p3_sensor as p3t
    d = declaration.build("A-5.F1.11")
    injector = depth_faults.DepthInjector(d["fault"], d["parameters"], "active")
    injector.activate(0.0)
    original = p3t.info_fields()
    changed = injector.camera_info(original, 1.0)
    image = frame(0)
    assert injector.image(image, 1.0) == [image]  # images pass untouched (the relay crashed here in M5)
    cache = camera_info.CalibrationCache(SENSORS)
    cache.update(original)
    cache.update(changed)
    assert changed["k"][0] == pytest.approx(original["k"][0] * 1.01) and not cache.usable
    assert cache.invalid_reason in ("calibration_identity_changed",) or cache.invalid_reason.startswith(
        "calibration_rejected_after_lock")


# ---- state, ACK and map transforms against the accepted P2 StateCache ---------------------------
def states(sim, **override):
    stamp = int(sim * 1e6)
    out = {"vehicle_local_position": {"timestamp": stamp, "x": 0.0, "y": 0.0, "z": -2.0, "vx": 0.0, "vy": 0.0, "vz": 0.0,
                                      "xy_valid": True, "z_valid": True, "v_xy_valid": True, "v_z_valid": True,
                                      "heading_good_for_control": True, "xy_reset_counter": 0, "z_reset_counter": 0,
                                      "heading_reset_counter": 0, "vxy_reset_counter": 0, "vz_reset_counter": 0,
                                      "ref_timestamp": 1000000},
           "vehicle_attitude": {"timestamp": stamp, "q": [1.0, 0.0, 0.0, 0.0], "quat_reset_counter": 0},
           "vehicle_status": {"timestamp": stamp, "arming_state": 2, "nav_state": 14, "system_id": 72, "component_id": 1,
                              "failsafe": False, "pre_flight_checks_pass": True},
           "vehicle_land_detected": {"timestamp": stamp, "landed": False},
           "estimator_status_flags": {"timestamp": stamp, "cs_tilt_align": True, "cs_yaw_align": True},
           "failsafe_flags": {"timestamp": stamp, "local_position_invalid": False, "local_altitude_invalid": False,
                              "attitude_invalid": False, "angular_velocity_invalid": False}}
    out.update(override)
    return out


@pytest.mark.parametrize("fault,params,reason", [
    ("F2.05", {"backwards_us": 100000}, "state_time_backwards:vehicle_local_position"),  # 0.1 s back: before the last stamp
    ("F2.06", {"future_us": 200000}, "clock_epoch_mismatch:vehicle_local_position"),
    ("F2.07", {}, "invalid_estimate"),
    ("F5.10", {}, "vehicle_health_invalid")])
def test_app_side_state_faults_raise_the_intended_accepted_p2_check(fault, params, reason):
    cache = timing.StateCache(m5.P2)
    injector = state_faults.StateInjector(fault, params, "active")
    for sim in (10.0, 10.05):
        cache.clock(sim, sim)
        if sim == 10.05:
            injector.activate(sim)
        with pytest.raises(ValueError, match=reason) if fault == "F2.05" and sim == 10.05 else _null():
            for name, data in states(sim).items():
                for item in injector.state(name, data, sim):
                    cache.update(name, item, sim)
    if fault != "F2.05":
        with pytest.raises(ValueError, match=reason):
            cache.validate(10.05, 10.05)


class _null:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def test_withholding_ack_and_map_faults_and_their_pass_through():
    injector = state_faults.StateInjector("F2.09", {"withheld_topics": ["vehicle_local_position", "vehicle_attitude"]}, "active")
    message = object()
    assert injector.state("vehicle_local_position", message, 1.0) == [message]  # not yet triggered
    injector.activate(1.0)
    assert injector.state("vehicle_local_position", message, 1.1) == [] and injector.state("vehicle_status", message, 1.1) == [message]
    acks = state_faults.AckInjector("F5.03", {}, "active")
    assert acks.ack({"command": 176, "result": 0}, 1.0) == [{"command": 176, "result": 2}]
    assert state_faults.AckInjector("F5.04", {}, "active").ack({"command": 21, "result": 0}, 1.0) == []
    m = state_faults.MapInjector("F4.03", {}, "active")
    m.activate(0.0)
    assert m.snapshot("a", 1.0) == [] and m.snapshot("b", 1.1) == ["b", "a"]
    delay = state_faults.MapInjector("F4.02", {"delay_s": 0.30, "window_sim_s": 5.0}, "active")
    delay.activate(0.0)
    assert delay.snapshot("a", 1.0) == [] and delay.due(1.29) == [] and delay.due(1.30) == ["a"]
    passing = state_faults.MapInjector("F4.01", {"drop_window_sim_s": 1.0}, "pass-through")
    passing.activate(0.0)
    item = object()
    assert passing.snapshot(item, 0.5)[0] is item and passing.health(item, 0.5)[0] is item


# ---- policy transforms against the accepted selection --------------------------------------------
def candidate_rows():
    grid = local_map.Grid.around((0.0, 0.0), m5.P4C["geofence"], m5.P4C["perception"]["grid_margin_m"],
                                 m5.P4C["perception"]["grid_resolution_m"])
    grid.cells[:] = local_map.FREE
    ctx = planner.Context((0.0, 0.0), (0.0, 0.0), 0.0, tuple(m5.P4C["mission"]["goal_enu_m"]), (0.0, 0.0), 0.05,
                          m5.P4C["geofence"])
    rows = planner.evaluate_candidates(grid, np.zeros(grid.cells.shape, dtype=bool), ctx, m5.P4C["planner"], 0.25)
    return {observation_adapter.slot_of(r, m5.P4C["planner"]): r for r in rows}


def test_policy_corruptions_are_refused_by_the_accepted_selection_and_the_adversary_stays_in_the_mask():
    by_slot = candidate_rows()
    scores = np.linspace(-1.0, 1.0, 26)
    allowed = np.zeros(26, dtype=bool)
    allowed[[3, 7, 12, 20]] = True
    with pytest.raises(PolicyError, match="output_not_finite"):
        candidate_ranker.policy_choice(policy_faults.corrupt_scores("F3.04", scores), allowed, by_slot)
    with pytest.raises(PolicyError, match="output_shape"):
        candidate_ranker.policy_choice(policy_faults.corrupt_scores("F3.05", scores), allowed, by_slot)
    assert candidate_ranker.policy_choice(scores, allowed, by_slot) == 20
    assert policy_faults.worst_allowed(scores, allowed, by_slot) == 3
    assert not allowed[policy_faults.outside_mask(allowed)]
    with pytest.raises(PolicyError, match="input_not_finite"):
        p5.interface_policy().scores(policy_faults.corrupt_features(np.zeros((26, 46))))


# ---- the controller shim in the accepted P5 mission rig ------------------------------------------
class Clock:
    """Deterministic policy clock: each reading advances 10 us; `sleep` advances it exactly."""

    def __init__(self):
        self.t = 0.0
        self.sleeps = []

    def __call__(self):
        self.t += 1e-5
        return self.t

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.t += seconds


def fly(boxes, row=None, mode="active", limit=160.0, ledger=None):
    """One rig flight from MAP_SCAN until the mission leaves TRANSIT; returns (mission, actions, clock, shim)."""
    clock = Clock()
    policy = m5.trained_policy()
    factory = lambda cache: p5_mission_module.P5Mission(m5.P2, m5.P4C, m5.P5C, policy, p5.SCHEMA, cache, 0.0, True,
                                                        clock=clock)
    shim = None
    if row is not None:
        shim = controller_shim.Shim(declaration.build(row, mode), ledger, sleep=clock.sleep, boxes=boxes).install()
    try:
        rig = m5.Rig(factory, boxes)
        actions = []
        end = rig.sim + limit
        while rig.sim < end:
            actions.append(rig.step())
            if rig.m.state not in ("MAP_SCAN", "TRANSIT"):
                break
        return rig.m, actions, clock, shim
    finally:
        if shim is not None:
            shim.uninstall()


def comparable(m):
    return {"events": m.events, "ledger": m.ledger, "state": m.state, "transitions": m.transitions,
            "target": m.target, "policy_stats": m.policy_stats, "p4_abort": m.p4_abort}


@m5.needs_artifact
def test_pass_through_shim_is_bit_identical_to_the_accepted_mission(tmp_path):
    plain, plain_actions, _, _ = fly(S2_BOXES)
    ledger = fault_ledger.FaultLedger(tmp_path / "shim.jsonl", "B-0", None, "controller_shim")
    shimmed, shimmed_actions, _, shim = fly(S2_BOXES, "B-0", "pass-through", ledger=ledger)
    ledger.close()
    assert plain.state == "GOAL_HOVER" and shimmed_actions == plain_actions
    assert json.dumps(comparable(shimmed), sort_keys=True, default=str) == json.dumps(comparable(plain), sort_keys=True, default=str)
    assert vars(p5_mission_module.P5Mission).get("tick") is None  # uninstalled: the accepted class again
    assert FrozenPolicy.scores.__name__ == "scores" and candidate_ranker.policy_choice.__name__ == "policy_choice"
    kinds = [r["kind"] for r in fault_ledger.read(tmp_path / "shim.jsonl")]
    assert kinds == ["declared", "armed", "finished"]  # pass-through writes nothing per tick


def faulted(m):
    return [r for r in m.ledger if r["invoked"] and r["failure"] is not None]


@m5.needs_artifact
@pytest.mark.parametrize("row,failure", [("B-1", "timeout"), ("S-3", "output_not_finite")])
def test_policy_fault_rows_hold_three_ticks_then_abort_policy_unavailable(row, failure, tmp_path):
    ledger = fault_ledger.FaultLedger(tmp_path / "shim.jsonl", row, None, "controller_shim")
    m, actions, clock, shim = fly(S2_BOXES, row, ledger=ledger)
    ledger.close()
    bad = faulted(m)
    assert [r["failure"] for r in bad] == [failure] * 3
    assert [r["tick"] for r in bad] == list(range(bad[0]["tick"], bad[0]["tick"] + 3))  # consecutive ticks
    assert m.p4_abort == "p4_supervisor_abort:policy_unavailable" and m.state == "REQUEST_LAND"
    assert actions[-1]["command"] == {"command": 21, "params": [0.0] * 7}
    assert shim.trigger.fired_sim is not None and bad[0]["sim_s"] > shim.trigger.fired_sim
    if failure == "timeout":
        assert clock.sleeps == [0.025] * 3 and all(0.025 <= r["inference_s"] < 0.026 for r in bad)
    records = fault_ledger.read(tmp_path / "shim.jsonl")
    assert [r["kind"] for r in records].count("affected") == 3 and records[2]["kind"] == "activated"


@m5.needs_artifact
def test_isolated_timeouts_hold_only_their_ticks_and_the_mission_completes():
    m, _, clock, _ = fly(S4_BOXES, "C-2", limit=200.0)
    bad = faulted(m)
    assert [r["failure"] for r in bad] == ["timeout"] * 5 and clock.sleeps == [0.025] * 5
    assert all(b["tick"] - a["tick"] > 1 for a, b in zip(bad, bad[1:]))  # never consecutive
    assert m.state == "GOAL_HOVER" and m.p4_abort is None
    others = [r["inference_s"] for r in m.ledger if r["invoked"] and r["failure"] is None]
    assert max(others) < 0.005  # the P5 limit holds on every other tick


@m5.needs_artifact
def test_adversarial_choice_stays_inside_the_mask_on_every_tick():
    m, _, _, shim = fly(S2_BOXES, "C-3", limit=200.0)
    invoked = [r for r in m.ledger if r["invoked"] and r["failure"] is None and r["sim_s"] > shim.trigger.fired_sim]
    rows_ok = 0
    for record in invoked:
        allowed, scores = np.array(record["allowed"]), np.array(record["scores"])
        assert allowed[record["choice_slot"]]
        assert scores[record["choice_slot"]] <= scores[allowed].min() + 1e-9  # the worst allowed, as declared
        rows_ok += 1
    assert rows_ok > 50 and m.state in ("GOAL_HOVER", "REQUEST_LAND")


@m5.needs_artifact
def test_map_loss_hovers_while_stale_then_recovers_and_completes():
    m, _, _, shim = fly([{"center": [4.0, 0.5], "size": [0.4, 0.4]}], "S-4", limit=200.0)
    t0 = shim.trigger.fired_sim
    window = [e for e in m.events if e["event"] == "p4_decision" and t0 < e["sim_s"] <= t0 + 1.4]
    assert any(e["stale"] == "snapshot_stale" and e["state"] == "HOVER" and e["frozen_yaw"] for e in window)
    after = [e for e in m.events if e["event"] == "p4_decision" and e["sim_s"] > t0 + 1.4]
    assert any(e["state"] == "AVOIDING" for e in after) and m.state == "GOAL_HOVER"


@m5.needs_artifact
def test_shim_serves_both_of_its_mechanisms_and_nothing_else():
    assert controller_shim.shim_mode(declaration.build("S-4")) == "active"  # MR: map input faults
    assert controller_shim.shim_mode(declaration.build("B-1")) == "active"  # CS: policy faults
    assert controller_shim.shim_mode(declaration.build("S-1")) == "off"  # PS: the runner signals the bridge
    assert controller_shim.shim_mode(declaration.build("B-0")) == "pass-through"


@m5.needs_artifact
def test_tick_delay_sleeps_once_after_normal_at_two_metres():
    m, _, clock, shim = fly([], "S-5'", limit=200.0)
    assert clock.sleeps == [0.060] and shim.trigger.fired_sim is not None


# ---- isolation, ownership and the static guarantees ---------------------------------------------
def calls(path, attribute):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return [n for n in ast.walk(tree) if isinstance(n, ast.Attribute) and n.attr == attribute]


def test_no_injector_publishes_to_px4_and_the_witness_and_competitor_never_publish():
    injection = P6 / "injection"
    assert not calls(injection / "competing_publisher.py", "publish")
    assert not calls(injection / "witness.py", "create_publisher") and not calls(injection / "witness.py", "publish")
    for path in injection.glob("*.py"):
        text = path.read_text(encoding="utf-8")
        if path.name != "competing_publisher.py":
            assert "/fmu/in" not in text, path.name


def test_accepted_code_never_refers_to_p6():
    import p5_manifest
    for base in ("simulation/px4_gazebo", "simulation/px4_gazebo_p4", "simulation/px4_gazebo_p5"):
        for path in (REPO / base).rglob("*.py"):
            assert "px4_gazebo_p6" not in path.read_text(encoding="utf-8", errors="ignore"), path
    assert all("px4_gazebo_p6" not in name for name in p5_manifest.p5_source_files())


@pytest.mark.skipif(not POSIX, reason="needs /proc and POSIX signals")
def test_process_stop_is_proven_from_proc_and_bounded():
    child = subprocess.Popen(["sleep", "30"])
    try:
        record = process_faults.inject(child.pid, "SIGSTOP", resume_after_wall_s=0.2)
        assert record["proved"] and record["stopped_state"] == "T" and record["resumed_state"] != "T"
        with pytest.raises(ValueError):
            process_faults.inject(child.pid, "SIGSTOP")
        process_faults.inject(child.pid, "SIGINT")
        assert child.wait(timeout=5) != 0
    finally:
        if child.poll() is None:
            child.kill()


def test_load_bounds_are_the_approved_ones():
    load_harness.check(24, 60.0, 0.25)
    for bad in ((25, 1.0, 0.0), (1, 60.5, 0.0), (1, 1.0, 0.3), (1, 0.0, 0.0)):
        with pytest.raises(ValueError):
            load_harness.check(*bad)


@pytest.mark.skipif(not POSIX, reason="fork start method and /proc/meminfo")
def test_load_workers_stop_themselves_at_their_deadline():
    record = load_harness.run(1, 0.3)
    assert record["exit_codes"] == [0] and 0.25 <= record["end_wall_s"] - record["start_wall_s"] < 3.0
