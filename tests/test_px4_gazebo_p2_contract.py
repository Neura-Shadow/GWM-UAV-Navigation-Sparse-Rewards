"""P2 protocol, frame, clock, state and negative gates; no ROS/runtime imports."""
import copy
import importlib
import json
import math
from pathlib import Path
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]/"simulation/px4_gazebo/ros2_ws/src/gwm_px4_control"
sys.path.insert(0, str(PACKAGE))
from gwm_px4_control import acceptance, contracts, frames, mission, protocol, timing


@pytest.fixture
def config():
    return contracts.load_config(PACKAGE.parents[2]/"configs/p2_control.yaml")


@pytest.mark.parametrize("enu,ned", [((1,0,0),(0,1,0)), ((0,1,0),(1,0,0)), ((0,0,1),(0,0,-1)),
                                    ((-2,3,-4),(3,-2,4))])
def test_independent_axis_fixtures(enu, ned):
    assert frames.enu_to_ned(enu) == ned
    assert frames.ned_to_enu(ned) == enu


def test_nonzero_origin_and_body_signs():
    assert frames.offset_target((8,-3,6), (1,2,3)) == (10,-2,3)
    assert frames.flu_to_frd((1,2,3)) == (1,-2,-3)
    assert frames.frd_to_flu((1,-2,-3)) == (1,2,3)


@pytest.mark.parametrize("heading,expected", [(0,(1,0,0)), (math.pi/2,(0,1,0)), (-math.pi/2,(0,-1,0))])
def test_body_to_world_multiple_headings(heading, expected):
    q = (math.cos(heading/2),0,0,math.sin(heading/2))
    assert frames.body_frd_to_ned((1,0,0), q) == pytest.approx(expected)
    assert frames.ned_to_body_frd(expected, q) == pytest.approx((1,0,0))
    assert frames.yaw_from_quaternion(q) == pytest.approx(heading)


@pytest.mark.parametrize("enu_deg,ned_deg", [(0,90),(90,0),(180,-90),(-90,-180),(270,-180)])
def test_absolute_yaw_and_wrap(enu_deg, ned_deg):
    assert math.degrees(frames.yaw_enu_to_ned(math.radians(enu_deg))) == pytest.approx(ned_deg)
    assert frames.wrap(frames.yaw_ned_to_enu(math.radians(ned_deg))-math.radians(enu_deg)) == pytest.approx(0)


def test_signed_yaw_increment_is_not_absolute_conversion():
    initial = math.radians(-170)
    assert math.degrees(frames.wrap(initial-math.radians(30))) == pytest.approx(160)
    assert math.degrees(frames.wrap(initial+math.radians(30))) == pytest.approx(-140)


@pytest.mark.parametrize("value", [(1,2,float("nan")), (True,2,3), (1,2,float("inf")), (1,2)])
def test_invalid_coordinate_rejected(value):
    with pytest.raises(ValueError):
        frames.enu_to_ned(value)


def test_quaternion_is_not_blind_reordering():
    with pytest.raises(ValueError):
        frames.yaw_from_quaternion((0,0,0,0))
    assert frames.body_frd_to_ned((0,1,0),(0,1,0,0)) == (0,-1,0)


def test_position_activation_nan_and_strict_json():
    wire = contracts.position_setpoint((1,2,-2), 0, 1000000)
    assert wire["position"] == [1,2,-2]
    assert all(math.isnan(v) for k in ("velocity","acceleration","jerk") for v in wire[k])
    assert math.isnan(wire["yawspeed"])
    encoded = json.loads(contracts.strict_json(contracts.encode_wire(wire)))
    assert encoded["fields"]["velocity"] == [None]*3 and encoded["active"]["velocity"] is False
    assert encoded["active"]["position"] is True
    with pytest.raises(ValueError):
        contracts.strict_json({"measurement": float("nan")})
    assert contracts.json_message({"x":float("nan")}) == {"fields":{"x":None},"nonfinite_fields":["$.x"]}
    mode = contracts.offboard_mode(1000000)
    assert mode["position"] is True and not any(v for k,v in mode.items() if k not in ("timestamp","position"))


@pytest.mark.parametrize("gate", contracts.GATES+("GWM_ALLOW_SITL_COMMANDS",))
def test_each_missing_gate_blocks(gate):
    env = {k:"1" for k in contracts.GATES+("GWM_ALLOW_SITL_COMMANDS",)}
    del env[gate]
    with pytest.raises(ValueError):
        contracts.gates(env, True)


def test_observation_needs_no_flight_gate():
    contracts.gates({k:"1" for k in contracts.GATES}, False)


def ack(config, **changes):
    value = {"command":176, "timestamp":10100000, "result":0,
             "target_system":config["source_system"], "target_component":config["source_component"], "from_external":False}
    return {**value, **changes}


@pytest.mark.parametrize("change", [{"command":400},{"target_system":72},{"target_component":1},
                                   {"timestamp":9999999},{"timestamp":float("nan")},{"from_external":True}])
def test_unmatched_and_stale_ack(config, change):
    tracker = protocol.AckTracker(config)
    tracker.issue(176, "REQUEST_OFFBOARD", 10, 50)
    assert tracker.receive(ack(config, **change), "REQUEST_OFFBOARD", 10.2, 50.2) == "unmatched"


@pytest.mark.parametrize("result,label", protocol.RESULTS.items())
def test_all_ack_outcomes(config, result, label):
    tracker = protocol.AckTracker(config)
    tracker.issue(176, "REQUEST_OFFBOARD", 10, 50)
    assert tracker.receive(ack(config,result=result), "REQUEST_OFFBOARD",10.2,50.2) == label
    assert (tracker.pending is not None) == (result == 5)


def test_ack_deadline_and_no_blind_retry(config):
    tracker = protocol.AckTracker(config)
    tracker.issue(176,"REQUEST_OFFBOARD",10,50)
    with pytest.raises(ValueError, match="Outstanding"):
        tracker.issue(176,"REQUEST_OFFBOARD",10.1,50.1)
    with pytest.raises(TimeoutError):
        tracker.check_deadline(14,51)
    with pytest.raises(ValueError,match="blind"):
        tracker.issue(176,"REQUEST_OFFBOARD",14,51)


def state_cache(config, sim=10, wall=50):
    cache = timing.StateCache(config)
    cache.clock(sim,wall)
    messages = {
      "vehicle_local_position":{"timestamp":int(sim*1e6),"x":8.,"y":-3.,"z":6.,"vx":0.,"vy":0.,"vz":0.,
          "xy_valid":True,"z_valid":True,"v_xy_valid":True,"v_z_valid":True,"heading_good_for_control":True,
          "xy_reset_counter":0,"z_reset_counter":0,"heading_reset_counter":0,"vxy_reset_counter":0,"vz_reset_counter":0,"ref_timestamp":1000000},
      "vehicle_attitude":{"timestamp":int(sim*1e6),"q":[1.,0.,0.,0.],"quat_reset_counter":0},
      "vehicle_status":{"timestamp":int(sim*1e6),"arming_state":1,"nav_state":4,"system_id":72,"component_id":1,"failsafe":False,"pre_flight_checks_pass":True},
      "vehicle_land_detected":{"timestamp":int(sim*1e6),"landed":True},
      "estimator_status_flags":{"timestamp":int(sim*1e6),"cs_tilt_align":True,"cs_yaw_align":True},
      "failsafe_flags":{"timestamp":int(sim*1e6),"local_position_invalid":False,"local_altitude_invalid":False,"attitude_invalid":False,"angular_velocity_invalid":False}}
    for name,data in messages.items():
        cache.update(name,data,wall)
    return cache


def test_clock_pause_and_backwards_time(config):
    cache = state_cache(config)
    assert cache.clock(10,50.5) is False
    with pytest.raises(ValueError,match="clock_stalled"):
        cache.validate(10,52)
    with pytest.raises(ValueError,match="clock_backwards"):
        cache.clock(9,52)


def test_ground_alignment_uses_preflight_yaw_not_inflight_mag_completion(config):
    cache = state_cache(config)
    cache.data["vehicle_local_position"]["heading_good_for_control"] = False
    assert cache.validate(10,50)["landed"] is True
    cache.data["estimator_status_flags"]["cs_yaw_align"] = False
    with pytest.raises(ValueError, match="alignment_invalid"):
        cache.validate(10,50)


def test_healthy_abort_requests_one_normal_land_and_stops_targets(config):
    cache = state_cache(config)
    cache.data["vehicle_status"].update(arming_state=2, nav_state=14)
    cache.data["vehicle_land_detected"]["landed"] = False
    control = mission.Mission(config, cache, 50, True)
    control.state = "EAST_TEST"
    command = control.abort("flight_envelope_horizontal", 10, 50)
    assert command == {"command": 21, "params": [0.0]*7}
    assert control.abort("another_fault", 10, 50) is None
    assert control.tick(10, 50.1, True)["setpoint"] is None
    assert len(control.transactions.records) == 1


def test_reference_reset_aborts_without_assuming_land_control(config):
    cache = state_cache(config)
    cache.data["vehicle_status"].update(arming_state=2, nav_state=14)
    cache.data["vehicle_land_detected"]["landed"] = False
    cache.frozen_reference = cache.reference()
    cache.data["vehicle_local_position"]["heading_reset_counter"] += 1
    control = mission.Mission(config, cache, 50, True)
    control.state = "TAKEOFF"
    assert control.abort("estimator_reference_reset", 10, 50) is None
    assert control.tick(11, 51, True)["setpoint"] is None
    assert not control.transactions.records
    control.tick(60, 50+config["recovery_wall_s"]+1, True)
    assert control.done and control.failure == "estimator_reference_reset"


@pytest.mark.parametrize("failure", ["smoke", "offline", "hash", "run_id"])
def test_repeated_acceptance_requires_independently_passed_smoke(failure):
    sys.path.insert(0, str(PACKAGE.parents[2]/"scripts"))
    repeated = importlib.import_module("run_p2_repeated")
    smoke = {"kind": "flight", "status": "passed", "run_id": "p2-smoke", "controller_result": {"flight": "passed"}}
    offline = {"run_id": "p2-smoke", "recording_integrity": "passed", "flight_acceptance": "passed", "evaluator_sha256": "fixed"}
    repeated.check_smoke(smoke, offline, "fixed")
    if failure == "smoke":
        smoke["controller_result"]["flight"] = "failed"
    elif failure == "offline":
        offline["flight_acceptance"] = "unknown"
    elif failure == "hash":
        offline["evaluator_sha256"] = "changed"
    else:
        offline["run_id"] = "p1-or-another-run"
    with pytest.raises(ValueError, match="must both pass"):
        repeated.check_smoke(smoke, offline, "fixed")


@pytest.mark.parametrize("failure", ["stale","nan","flags","reference","missing","epoch"])
def test_state_failures(config, failure):
    cache = state_cache(config)
    if failure == "stale":
        cache.clock(11,50.5)
    elif failure == "nan":
        cache.data["vehicle_local_position"]["x"] = float("nan")
    elif failure == "flags":
        cache.data["vehicle_local_position"]["xy_valid"] = False
    elif failure == "reference":
        cache.frozen_reference = cache.reference()
        cache.data["vehicle_local_position"]["z_reset_counter"] += 1
    elif failure == "missing":
        del cache.data["vehicle_attitude"]
    else:
        cache.data["vehicle_attitude"]["timestamp"] += 1000000
    with pytest.raises(ValueError):
        cache.validate(11 if failure=="stale" else 10,50.5)


def test_duplicates_not_new_observations(config):
    cache = state_cache(config)
    assert cache.update("vehicle_status",cache.data["vehicle_status"],51) is False
    assert cache.stats["vehicle_status"]["unique"] == 1
    assert cache.receipts["vehicle_status"] == 50


@pytest.mark.parametrize("armed,landed", [(2,True),(1,False)])
def test_restarted_controller_rejects_flight_state(config,armed,landed):
    cache = state_cache(config)
    cache.data["vehicle_status"]["arming_state"] = armed
    cache.data["vehicle_land_detected"]["landed"] = landed
    control = mission.Mission(config,cache,50,True)
    if armed == 1:
        control.tick(10,50,True)
    with pytest.raises(ValueError,match="restart_reject"):
        control.tick(10,50,True)


def test_accepted_ack_does_not_complete_mode_or_arm(config):
    cache = state_cache(config)
    control = mission.Mission(config,cache,50,True)
    control.state, control.enter_sim, control.enter_wall = "REQUEST_OFFBOARD",10,50
    control.origin, control.target, control.initial_yaw, control.yaw_target = (8,-3,6),(8,-3,6),0,0
    control.ack_status = "accepted"
    control.tick(10,50,True)
    assert control.state == "VERIFY_OFFBOARD"
    assert control.tick(10,50,True)["command"] is None
    assert control.state == "VERIFY_OFFBOARD"
    with pytest.raises(ValueError,match="illegal_transition"):
        control.transition("TAKEOFF",10,50)


def test_competing_control_and_state_deadline(config):
    cache = state_cache(config)
    control = mission.Mission(config,cache,50,True)
    control.state, control.enter_sim, control.enter_wall = "PRESTREAM_SAFE_SETPOINTS",10,50
    control.origin,control.target,control.initial_yaw,control.yaw_target = (8,-3,6),(8,-3,6),0,0
    with pytest.raises(ValueError,match="competing_owner"):
        control.tick(10,50,False)
    cache.clock(60,100)
    with pytest.raises(TimeoutError,match="state_transition_timeout"):
        control.tick(60,100,True)


def test_ramp_is_rate_limited():
    assert mission.ramp((0,0,0),(10,0,0),.3,.05) == pytest.approx((.015,0,0))
    assert math.dist((0,0,0),mission.ramp((0,0,0),(10,10,-10),.3,.05)) == pytest.approx(.015)


@pytest.mark.parametrize("problem", ["missing","wrong_east_sign","wrong_yaw_sign","short","gap","mode"])
def test_independent_evaluator_rejects_false_success(config, problem):
    fixture = acceptance.expected_fixture((8,-3,6),0,config)
    target = fixture["EAST_TEST" if problem != "wrong_yaw_sign" else "YAW_TEST"]
    samples = [{"t":i*.05,"position":target["position"],"yaw":target["yaw"],"arming_state":2,"nav_state":14,"landed":False} for i in range(101)]
    if problem == "missing":
        samples=[]
    elif problem == "wrong_east_sign":
        for s in samples: s["position"]=(8,-4,4)
    elif problem == "wrong_yaw_sign":
        for s in samples: s["yaw"]=math.pi/6
    elif problem == "short": samples.pop()
    elif problem == "gap": del samples[30:45]
    else: samples[30]["nav_state"]=18
    assert acceptance.evaluate_window(samples,target,5,config)["status"] != "passed"


def test_evaluator_missing_windows_and_accepted_only(config):
    assert acceptance.evaluate_flight({"final_state":"VERIFY_OFFBOARD"},[],config)["status"] == "failed"
    assert acceptance.evaluate_flight({"final_state":"COMPLETE"},[],config)["status"] == "unknown"


def test_runtime_module_import_is_inert(monkeypatch):
    import subprocess
    def forbidden(*args, **kwargs):
        raise AssertionError("Process launch during import")
    monkeypatch.setattr(subprocess,"Popen",forbidden)
    importlib.import_module("gwm_px4_control.node")
    assert "rclpy" not in sys.modules


def test_nominal_state_machine_requires_observed_states_and_full_windows(config):
    cache = state_cache(config)
    control = mission.Mission(config, cache, 50, True)
    position, velocity, yaw = (8.,-3.,6.), (0.,0.,0.), 0.
    nav, armed, landed = 4, 1, True
    pending_state = None
    commands, setpoints = [], []
    land_start = None
    for tick in range(4000):
        sim, wall = 10+tick*.05, 50+tick*.05
        if pending_state and tick >= pending_state[0]:
            if pending_state[1] == 176: nav = 14
            elif pending_state[1] == 400: armed = 2
            else: nav,land_start = 18,sim
            pending_state = None
        if land_start is not None:
            old = position
            position = (position[0],position[1],min(6.,position[2]+.3*.05))
            velocity = tuple((b-a)/.05 for a,b in zip(old,position))
            if position[2] == 6.: landed,armed = True,1
        elif position[2] < 5.98:
            landed = False
        cache.clock(sim,wall)
        for name, data in list(cache.data.items()):
            data = copy.deepcopy(data)
            data["timestamp"] = int(sim*1e6)
            if name == "vehicle_local_position":
                data.update(dict(zip(("x","y","z"),position)))
                data.update(dict(zip(("vx","vy","vz"),velocity)))
            elif name == "vehicle_attitude": data["q"]=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]
            elif name == "vehicle_status": data.update(arming_state=armed,nav_state=nav)
            elif name == "vehicle_land_detected": data["landed"]=landed
            cache.update(name,data,wall)
        action = control.tick(sim,wall,True)
        if action["command"]:
            command = action["command"]["command"]
            commands.append((command,sim))
            pending_state = (tick+4,command)
            control.ack(ack(config,command=command,timestamp=int(sim*1e6)),sim,wall)
        if action["setpoint"]:
            assert land_start is None and control.state not in mission.ORDER[16:]
            setpoints.append((sim,action["setpoint"]))
            old = position
            position,yaw = action["setpoint"]["position"],action["setpoint"]["yaw"]
            velocity = tuple((b-a)/.05 for a,b in zip(old,position))
        if control.done:
            break
    assert control.state == "COMPLETE"
    assert [c[0] for c in commands] == [176,400,21]
    assert commands[0][1]-setpoints[0][0] >= 2
    assert tuple(control.windows) == acceptance.PHASES
    assert [t["to"] for t in control.transitions] == list(mission.ORDER[1:])
    assert control.tick(sim+.05,wall+.05,True)["command"] is None
