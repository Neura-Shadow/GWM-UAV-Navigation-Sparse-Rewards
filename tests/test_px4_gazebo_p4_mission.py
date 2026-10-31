"""P4 M4: P4 mission on the unchanged P2 state machine and the exclusive controller, offline.

PX4 state is synthetic (P2 StateCache message shapes); the vehicle tracks the commanded
target ideally. Maps come from production perception on independently rendered depth.
"""
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import p4_synthetic as syn  # noqa: E402
from gwm_obstacle_avoidance import control_node, local_map, mission, perception, planner  # noqa: E402
from gwm_px4_control import contracts as p2_contracts, timing  # noqa: E402
from gwm_px4_control.mission import ORDER_V3  # noqa: E402
from gwm_sensor_adapter.geometry import Extrinsic  # noqa: E402

P2 = p2_contracts.load_config(syn.SIM / "configs/p2_control.yaml")
P4C = syn.p4_config()
SENSORS = syn.sensors()
EXTRINSIC = Extrinsic(SENSORS["optical_origin_base_flu_m"])
COLUMNS = np.arange(2, 640, 4)
YAW0_NED = math.pi / 2  # the vehicle spawns facing world +x (east)


class Rig:
    """Synthetic PX4 state, ideal tracking, perception at 10 Hz, mission ticks at 20 Hz."""

    def __init__(self, boxes=(), state="MAP_SCAN", stop_perception_at=None):
        self.cache = timing.StateCache(P2)
        self.m = mission.P4Mission(P2, P4C, self.cache, 0.0, True)
        self.boxes, self.scene = list(boxes), syn.boxes_3d(list(boxes))
        self.grid = local_map.Grid.around((0.0, 0.0), P4C["geofence"], P4C["perception"]["grid_margin_m"],
                                          P4C["perception"]["grid_resolution_m"])
        self.sim, self.wall, self.tick_count = 40.0, 40.0, 0
        self.stop_perception_at = stop_perception_at
        m = self.m
        m.origin, m.initial_yaw, m.yaw_target = (0.0, 0.0, 0.0), YAW0_NED, YAW0_NED
        m.target = (0.0, 0.0, -2.0)
        m.handover = {"completed_sim_s": 30.0}
        m.start_sim, m.state, m.enter_sim, m.enter_wall = 1.0, state, self.sim, self.wall
        m.positions = [(0.0, 0.0)]
        m.on_adapter_health({"status": "fresh", "calibration_identity": "cal", "sim_s": 1e9})
        self.publish_state()
        self.perceive()

    def publish_state(self, landed=False, **status):
        n, e, d = self.m.target
        yaw = self.m.yaw_target
        stamp = int(self.sim * 1e6)
        self.cache.clock(self.sim, self.wall)
        messages = {
            "vehicle_local_position": {"timestamp": stamp, "x": n, "y": e, "z": d, "vx": 0.0, "vy": 0.0, "vz": 0.0,
                                       "xy_valid": True, "z_valid": True, "v_xy_valid": True, "v_z_valid": True,
                                       "heading_good_for_control": True, "xy_reset_counter": 0, "z_reset_counter": 0,
                                       "heading_reset_counter": 0, "vxy_reset_counter": 0, "vz_reset_counter": 0,
                                       "ref_timestamp": 1000000},
            "vehicle_attitude": {"timestamp": stamp, "q": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
                                 "quat_reset_counter": 0},
            "vehicle_status": {"timestamp": stamp, "arming_state": 2, "nav_state": 14, "system_id": 72,
                               "component_id": 1, "failsafe": False, "pre_flight_checks_pass": True, **status},
            "vehicle_land_detected": {"timestamp": stamp, "landed": landed},
            "estimator_status_flags": {"timestamp": stamp, "cs_tilt_align": True, "cs_yaw_align": True},
            "failsafe_flags": {"timestamp": stamp, "local_position_invalid": False, "local_altitude_invalid": False,
                               "attitude_invalid": False, "angular_velocity_invalid": False}}
        for name, data in messages.items():
            self.cache.update(name, data, self.wall)

    def perceive(self):
        if self.stop_perception_at is not None and self.sim >= self.stop_perception_at:
            return
        n, e, d = self.m.target
        vehicle = (e, n, -d)
        yaw_enu = mission.yaw_enu_from_ned(self.m.yaw_target)
        position, q = syn.px4_pose(vehicle, yaw_enu)
        image = syn.render(vehicle, yaw_enu, self.scene, columns=COLUMNS)
        camera_xy, rays = perception.extract_scan(image, syn.CAL, EXTRINSIC, position, q, P4C["perception"],
                                                  SENSORS["near_m"], SENSORS["far_m"], 0.0)
        perception.integrate(self.grid, camera_xy, rays, P4C["perception"]["max_hit_range_m"], SENSORS["near_m"])
        self.m.on_snapshot({"usable": True, "grid": self.grid.encode(), "acquisition_sim_s": self.sim,
                            "calibration_identity": "cal"})

    def step(self):
        self.sim += 0.05
        self.wall += 0.05
        self.tick_count += 1
        self.publish_state()
        if self.tick_count % 2 == 0:
            self.perceive()
        return self.m.tick(self.sim, self.wall, True)

    def run_until(self, predicate, limit_s):
        end = self.sim + limit_s
        while self.sim < end:
            actions = self.step()
            if predicate(actions):
                return actions
        raise AssertionError("condition not reached in %.1f s (state %s)" % (limit_s, self.m.state))


def test_p4_order_is_the_p2_order_with_p4_phases_after_initial_hover():
    prefix = ORDER_V3[:ORDER_V3.index("INITIAL_HOVER") + 1]
    assert mission.P4_ORDER[:len(prefix)] == prefix
    assert mission.P4_ORDER[len(prefix):] == ("MAP_SCAN", "TRANSIT", "GOAL_HOVER", "REQUEST_LAND", "VERIFY_LANDING",
                                              "VERIFY_LANDED_AND_DISARMED", "COMPLETE")


def test_only_the_p2_mission_radius_is_replaced_by_the_p4_geofence():
    merged = mission.p4_p2_config(P2, P4C)
    changed = {k for k in merged if merged[k] != P2[k]}
    assert changed == {"max_horizontal_radius_m"}
    assert merged["max_horizontal_radius_m"] == pytest.approx(math.hypot(9.5, 3.0))
    for key in ("rate_hz", "ramp_m_s", "yaw_ramp_deg_s", "state_fresh_wall_s", "max_sample_gap_sim_s",
                "simulation_deadline_s", "ack_deadline_sim_s", "phase_deadline_sim_s"):
        assert merged[key] == P2[key]
    with pytest.raises(ValueError, match="v3 reference"):
        mission.P4Mission(dict(P2, reference_policy="p2-estimator-reference-v2"), P4C, timing.StateCache(P2), 0, True)


def test_map_scan_is_one_counter_clockwise_revolution_then_transit():
    rig = Rig()
    yaws = []
    rig.run_until(lambda a: (yaws.append(rig.m.yaw_target) or rig.m.state == "TRANSIT"), 45.0)
    assert math.degrees(rig.m.scan["rotated_rad"]) == pytest.approx(360.0)
    steps = [mission.wrap(b - a) for a, b in zip(yaws, yaws[1:])]
    assert max(abs(s) for s in steps) <= math.radians(10) * 0.05 + 1e-9  # P2 yaw ramp
    assert all(s <= 1e-12 for s in steps)  # NED yaw decreases: counter-clockwise in ENU
    assert rig.m.yaw_target == pytest.approx(YAW0_NED)
    assert rig.m.target == (0.0, 0.0, -2.0)  # in place
    assert 35.5 <= rig.m.scan["completed_sim_s"] - rig.m.scan["start_sim_s"] <= 36.6


def test_map_scan_holds_on_stale_depth_and_aborts_after_ten_seconds():
    rig = Rig(stop_perception_at=40.5)
    held = rig.m.yaw_target
    actions = rig.run_until(lambda a: a["command"] is not None, 12.0)
    # Revision 5: the stale MAP_SCAN abort lands through P2's nominal sequence, not RECOVERY.
    assert actions["command"]["command"] == 21 and rig.m.state == "REQUEST_LAND" and rig.m.failure is None
    assert rig.m.p4_abort.startswith("p4_supervisor_abort:map_scan_stale_inputs")
    # Rotation continues only until the last snapshot ages past 0.25 s (about 0.7 s at 10 deg/s).
    assert abs(mission.wrap(rig.m.yaw_target - held)) < math.radians(8)


def test_s1_transit_goal_hover_and_single_land_request():
    rig = Rig()
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    rig.run_until(lambda a: rig.m.state == "GOAL_HOVER", 50.0)  # 8 m at the pinned 0.20 m/s
    assert mission.enu_from_ned(rig.m.target) == pytest.approx((8.0, 0.0))
    assert rig.m.supervisor.transitions == []  # S1: no avoidance, no hover
    actions = rig.run_until(lambda a: a["command"] is not None, 7.0)
    assert actions["command"] == {"command": 21, "params": [0.0] * 7}
    assert rig.m.state == "REQUEST_LAND" and rig.m.windows["GOAL_HOVER"]["end_sim_s"] - rig.m.windows["GOAL_HOVER"]["start_sim_s"] >= 5.0
    decisions = [e for e in rig.m.events if e["event"] == "p4_decision"]
    assert decisions and all(d["state"] == "NORMAL" for d in decisions)


def test_transit_translates_at_the_pinned_p4_speed_and_p2_keeps_its_ramp():
    """Revision 4: TRANSIT advances the target at transit_speed_m_s (0.20 m/s), not P2's ramp_m_s."""
    assert P4C["mission"]["transit_speed_m_s"] == 0.2 and P2["ramp_m_s"] == 0.3
    rig = Rig()
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    steps, previous = [], mission.enu_from_ned(rig.m.target)
    for _ in range(100):
        rig.step()
        current = mission.enu_from_ned(rig.m.target)
        steps.append(math.dist(previous, current))
        previous = current
    assert max(steps) == pytest.approx(0.2 * 0.05, abs=1e-9)  # 20 Hz ticks at 0.20 m/s
    assert sum(steps) / len(steps) == pytest.approx(0.2 * 0.05, abs=1e-6)


def land_step(rig, landed=False, **status):
    rig.sim += 0.05
    rig.wall += 0.05
    rig.publish_state(landed=landed, **status)
    return rig.m.tick(rig.sim, rig.wall, True)


def test_supervisor_abort_lands_through_p2_nominal_sequence_with_one_land():
    """Revision 5: hold, one LAND, REQUEST_LAND -> VERIFY_LANDING -> VERIFY_LANDED_AND_DISARMED -> COMPLETE."""
    rig = Rig(boxes=[{"name": "wall", "center": [4.0, 0.0], "size": [0.3, 8.0]}])
    rig.run_until(lambda a: rig.m.state == "TRANSIT", 45.0)
    actions = rig.run_until(lambda a: a["command"] is not None, 60.0)
    assert actions["command"] == {"command": 21, "params": [0.0] * 7}
    assert rig.m.state == "REQUEST_LAND" and rig.m.failure is None
    assert rig.m.p4_abort == "p4_supervisor_abort:hover_timeout" and rig.m.supervisor.state == "ABORT"
    assert rig.m.transitions[-1]["from"] == "TRANSIT" and rig.m.transitions[-1]["evidence"]["p4_abort"] == rig.m.p4_abort
    assert [e for e in rig.m.events if e["event"] == "transition"][-1]["from"] == "TRANSIT"
    assert any(e["event"] == "p4_supervisor_abort" for e in rig.m.events)
    hold = rig.m.target
    rig.m.ack({"command": 21, "timestamp": math.ceil(rig.sim * 1e6), "result": 0, "target_system": P2["source_system"],
               "target_component": P2["source_component"], "from_external": False}, rig.sim, rig.wall)
    assert land_step(rig)["command"] is None and rig.m.state == "VERIFY_LANDING" and rig.m.target == hold
    land_step(rig, nav_state=18)
    assert rig.m.state == "VERIFY_LANDED_AND_DISARMED"
    land_step(rig, landed=True, nav_state=18, arming_state=1)
    assert rig.m.state == "COMPLETE" and rig.m.done and rig.m.failure is None
    assert [r["command"] for r in rig.m.transactions.records] == [21]  # exactly one LAND in this rig
    assert control_node.outcome(rig.m, True) == "supervisor_abort"


def test_p2_safety_errors_still_route_to_recovery_and_fail():
    from gwm_obstacle_avoidance.supervisor import Supervisor
    rig = Rig(state="TRANSIT")
    rig.m.supervisor = Supervisor(P4C, rig.sim)
    rig.m.target = (3.5, 0.0, -2.0)  # outside the geofence: a P2-class safety error, not a supervisor abort
    with pytest.raises(ValueError, match="p4_geofence_horizontal") as raised:
        land_step(rig)
    assert not isinstance(raised.value, mission.P4Abort)
    rig.m.abort(str(raised.value), rig.sim, rig.wall)  # what the node's fail() does
    assert rig.m.state == "RECOVERY" and rig.m.p4_abort is None
    assert control_node.outcome(rig.m, True) == "failed"


def test_p4_phase_safety_checks_raise():
    from gwm_obstacle_avoidance.supervisor import Supervisor

    def advance(rig, **status):
        rig.sim += 0.05
        rig.wall += 0.05
        rig.publish_state(**status)
        return rig.sim, rig.wall

    rig = Rig(state="TRANSIT")
    rig.m.supervisor = Supervisor(P4C, rig.sim)
    with pytest.raises(ValueError, match="control_graph_invalid"):
        rig.m.tick(*advance(rig), False)
    rig.m.target = (3.5, 0.0, -2.0)  # NED north 3.5 m = ENU y 3.5 m, outside |y| <= 3.0
    with pytest.raises(ValueError, match="p4_geofence_horizontal"):
        rig.m.tick(*advance(rig), True)
    rig.m.target = (0.0, 0.0, -3.2)  # 3.2 m above the origin, above the 3.0 m ceiling
    with pytest.raises(ValueError, match="p4_geofence_height"):
        rig.m.tick(*advance(rig), True)
    rig2 = Rig(state="TRANSIT")
    rig2.m.supervisor = Supervisor(P4C, rig2.sim)
    with pytest.raises(ValueError, match="offboard_armed_state_lost"):
        rig2.m.tick(*advance(rig2, nav_state=4), True)


def test_only_p2_commands_are_possible():
    rig = Rig()
    for command in (176, 400, 21):
        assert rig.m.request(command, rig.sim, rig.wall)["command"] == command
        rig.m.transactions.pending = None
    with pytest.raises(ValueError, match="unsupported_command"):
        rig.m.request(16, rig.sim, rig.wall)


def test_control_node_is_importable_without_ros_and_refuses_without_gates(tmp_path, monkeypatch):
    for key in ("GWM_ALLOW_OPTIONAL_RUNTIME", "GWM_RUN_GAZEBO_PX4_TESTS", "GWM_ALLOW_PX4_LAUNCH"):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(ValueError, match="explicit gates"):
        control_node.main(["--config", str(syn.SIM / "configs/p2_control.yaml"), "--timing-config",
                           str(syn.SIM / "configs/p2_timing.yaml"), "--p4-config",
                           str(syn.P4 / "configs/p4_avoidance.yaml"), "--run-dir", str(tmp_path),
                           "--authorization-marker", str(tmp_path / "none")])
    assert control_node.EVENT_TOPIC == "/gwm/p2/events"
