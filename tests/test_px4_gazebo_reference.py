"""Independent reset fixtures and v1/v2 policy regressions; no ROS runtime."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

SIM = Path(__file__).resolve().parents[1]/"simulation/px4_gazebo"
sys.path.insert(0, str(SIM/"ros2_ws/src/gwm_px4_control"))
from gwm_px4_control import acceptance, contracts, mission, timing
from gwm_px4_control.estimator_reference import ReferenceManager, V1, V2


@pytest.fixture
def config():
    return contracts.load_config(SIM/"configs/p2_control.yaml")


@pytest.fixture
def data():
    return json.loads((Path(__file__).parent/"fixtures/p2_strict_v1_reset.json").read_text())["snapshots"][0]["data"]


def sample():
    return {"arming_state": 2, "nav_state": 14, "landed": False}


def advance(data, sim):
    for value in data.values():
        value["timestamp"] = int(sim*1e6)


def reset(data, angle=.02, heading=True, quaternion=True, aligned=True):
    p, a, e = (data[k] for k in ("vehicle_local_position", "vehicle_attitude", "estimator_status_flags"))
    if heading:
        p["heading_reset_counter"] = (p["heading_reset_counter"]+1)%256
        p["delta_heading"] = angle
    if quaternion:
        a["quat_reset_counter"] = (a["quat_reset_counter"]+1)%256
        a["delta_q_reset"] = [math.cos(angle/2), 0., 0., math.sin(angle/2)]
    e.update(cs_in_air=True, cs_vehicle_at_rest=False, cs_mag_aligned_in_flight=aligned)


def manager(config, data, anchor=3.13):
    return ReferenceManager(config, data, (8., -3., 6.), anchor)


def accept(ref, data, angle=.02):
    advance(data,23)
    reset(data,angle)
    assert ref.inspect(data,sample(),23,50,"TAKEOFF")["pending"]
    advance(data,23.12)
    return ref.inspect(data,sample(),23.12,50.12,"TAKEOFF")


def test_historical_strict_v1_rejection_and_v2_classification(config):
    fixture=json.loads((Path(__file__).parent/"fixtures/p2_strict_v1_reset.json").read_text())
    before,changed,later=fixture["snapshots"][:3]
    strict=manager({**config,"reference_policy":V1},before["data"])
    with pytest.raises(ValueError,match="strict_v1"):
        strict.inspect(changed["data"],sample(),changed["sim_s"],50,"TAKEOFF")
    revised=manager(config,before["data"])
    revised.inspect(changed["data"],sample(),changed["sim_s"],50,"TAKEOFF")
    result=revised.inspect(later["data"],sample(),later["sim_s"],50.2,"TAKEOFF")
    assert result["accepted"] and result["correction"] == pytest.approx(-.0007634164649061859)
    assert len(revised.accepted)==1
    assert acceptance.evaluate_flight({"final_state":"RECOVERY","failure":fixture["original_failure"]},[],config)["status"]=="failed"


@pytest.mark.parametrize("angle", [.02,-.02,.08,-.08])
def test_anchor_corrected_once_with_wrap_and_fixed_geometry(config,data,angle):
    ref=manager(config,data)
    original=ref.fixed.copy()
    assert accept(ref,data,angle)["accepted"]
    expected=math.atan2(math.sin(3.13+angle),math.cos(3.13+angle))
    for i in range(10):
        advance(data,23.2+i*.01)
        result=ref.inspect(data,sample(),23.2+i*.01,50.2+i*.01,"TAKEOFF")
        assert result["correction"]==0
    assert ref.anchor==pytest.approx(expected) and ref.origin==(8,-3,6) and ref.fixed==original
    assert len(ref.accepted)==1
    fixture=acceptance.expected_fixture(ref.origin,ref.anchor,config)
    assert fixture["EAST_TEST"]["position"]==(8,-2,4)
    assert fixture["NORTH_TEST"]["position"]==(9,-3,4)
    json.dumps(ref.summary(),allow_nan=False)


@pytest.mark.parametrize("first", ["heading","quaternion"])
def test_asynchronous_orders_and_independent_counters(config,data,first):
    data["vehicle_attitude"]["quat_reset_counter"]=17
    ref=manager(config,data)
    advance(data,23)
    reset(data,heading=first=="heading",quaternion=first=="quaternion")
    assert ref.inspect(data,sample(),23,50,"TAKEOFF")["pending"]
    advance(data,23.04)
    reset(data,heading=first!="heading",quaternion=first!="quaternion")
    assert ref.inspect(data,sample(),23.04,50.04,"TAKEOFF")["pending"]
    advance(data,23.16)
    assert ref.inspect(data,sample(),23.16,50.16,"TAKEOFF")["accepted"]
    assert ref.baseline=={"heading":2,"quaternion":18}


@pytest.mark.parametrize("phase", ["PRESTREAM_SAFE_SETPOINTS","VERIFY_ARMED","INITIAL_HOVER","YAW_TEST","VERIFY_LANDING"])
def test_reset_outside_initialization_rejected(config,data,phase):
    ref=manager(config,data);advance(data,23);reset(data)
    with pytest.raises(ValueError,match="outside_initialization"):
        ref.inspect(data,sample(),23,50,phase)


@pytest.mark.parametrize("fault", ["angle","tilt","inconsistent","nan","position","velocity","origin","terrain","source","skipped","missing"])
def test_fail_closed_candidates(config,data,fault):
    ref=manager(config,data);advance(data,23);reset(data)
    p,a,e=(data[k] for k in ("vehicle_local_position","vehicle_attitude","estimator_status_flags"))
    if fault=="angle":p["delta_heading"]=.2;a["delta_q_reset"]=[math.cos(.1),0,0,math.sin(.1)]
    elif fault=="tilt":a["delta_q_reset"]=[math.cos(.01),math.sin(.01),0,0]
    elif fault=="inconsistent":p["delta_heading"]=-.02
    elif fault=="nan":p["delta_heading"]=float("nan")
    elif fault=="position":p["xy_reset_counter"]+=1
    elif fault=="velocity":p["vz_reset_counter"]+=1
    elif fault=="origin":p["ref_lat"]+=.001
    elif fault=="terrain":p["dist_bottom_reset_counter"]+=1
    elif fault=="source":e["cs_ev_yaw"]=True
    elif fault=="skipped":a["quat_reset_counter"]+=1
    else:del a["delta_q_reset"]
    with pytest.raises(ValueError):ref.inspect(data,sample(),23,50,"TAKEOFF")
    assert ref.rejected and len(ref.accepted)==0


def test_missing_pair_times_out_and_second_event_rejected(config,data):
    ref=manager(config,data);advance(data,23);reset(data,quaternion=False)
    ref.inspect(data,sample(),23,50,"TAKEOFF")
    advance(data,24.6)
    with pytest.raises(ValueError,match="timeout"):ref.inspect(data,sample(),24.6,51.6,"TAKEOFF")
    data["vehicle_local_position"]["heading_reset_counter"]-=1
    data["estimator_status_flags"]["cs_mag_aligned_in_flight"]=False
    ref=manager(config,data)
    advance(data,25);reset(data);ref.inspect(data,sample(),25,52,"TAKEOFF")
    advance(data,25.12);ref.inspect(data,sample(),25.12,52.12,"TAKEOFF")
    advance(data,25.2);reset(data)
    with pytest.raises(ValueError,match="additional"):ref.inspect(data,sample(),25.2,52.2,"TAKEOFF")


def test_uint8_wrap_independently(config,data):
    data["vehicle_local_position"]["heading_reset_counter"]=255
    data["vehicle_attitude"]["quat_reset_counter"]=91
    ref=manager(config,data)
    assert accept(ref,data)["accepted"]
    assert ref.baseline=={"heading":0,"quaternion":92}


def test_final_alignment_requires_flags_and_stability_not_timer(config,data):
    ref=manager(config,data)
    advance(data,23);ref.inspect(data,sample(),23,50,"TAKEOFF")
    advance(data,35);ref.inspect(data,sample(),35,62,"STABILIZE_REFERENCE")
    assert not ref.ready(35)
    data["vehicle_local_position"]["heading_good_for_control"]=True
    data["estimator_status_flags"].update(cs_mag_aligned_in_flight=True,cs_mag_3d=True,cs_in_air=True)
    ref=manager(config,data)  # zero-reset path: complete alignment already observed at preparation
    advance(data,36);ref.inspect(data,sample(),36,63,"STABILIZE_REFERENCE")
    assert not ref.ready(40.99)
    advance(data,41);ref.inspect(data,sample(),41,68,"STABILIZE_REFERENCE")
    assert ref.ready(41)
    ref.lock(41)
    assert ref.lock_sim_s==41 and ref.accepted==[]
    advance(data,41.1);reset(data)
    with pytest.raises(ValueError,match="outside_initialization"):ref.inspect(data,sample(),41.1,68.1,"INITIAL_HOVER")


def test_source_faithful_cached_px4_correction_and_future_ros_publish(config,data):
    ref=manager(config,data,anchor=.4)
    cached_yaw=.4;cached_stamp=22.95;px4_counter=1
    advance(data,23);reset(data,angle=.03)
    ref.inspect(data,sample(),23,50,"TAKEOFF")
    # Literal pinned MC position-control condition, independent expected value.
    p=data["vehicle_local_position"]
    if cached_stamp*1e6<p["timestamp"] and px4_counter!=p["heading_reset_counter"]:
        cached_yaw+=p["delta_heading"]
    px4_counter=p["heading_reset_counter"]
    assert cached_yaw==pytest.approx(.43)
    advance(data,23.12);ref.inspect(data,sample(),23.12,50.12,"TAKEOFF")
    for sim in (23.12,23.17,23.22):
        cached_stamp=sim;cached_yaw=ref.anchor
        advance(data,sim+.004)
        if cached_stamp*1e6<data["vehicle_local_position"]["timestamp"] and px4_counter!=data["vehicle_local_position"]["heading_reset_counter"]:
            cached_yaw+=.03
        assert cached_yaw==pytest.approx(.43)


def test_retained_late_notification_reproduces_cached_compensation_loss():
    f=json.loads((Path(__file__).parent/'fixtures/p2_r1_late_notification.json').read_text())
    saved_counter=1;cached=f['anchor_before']
    # PX4 corrects the old cached target and advances its counter.
    if f['last_pre_reset_target_us']<f['reset_us'] and saved_counter!=2:
        cached+=f['delta_heading']
    saved_counter=2
    assert cached==pytest.approx(f['anchor_after'])
    assert f['late_old_target_receipt_wall_s']<f['first_heading_notification_receipt_wall_s']
    cached=f['anchor_before']  # ROS publishes before its reset notification arrives.
    if f['late_old_target_us']<23496000 and saved_counter!=2:
        cached+=f['delta_heading']
    assert cached==pytest.approx(f['internal_setpoints'][0]['yaw'],abs=1e-7)
    assert abs(cached-f['anchor_after'])>1e-5


def test_independent_ulog_evaluator_rejects_retained_compensation_race(data):
    import numpy as np
    from types import SimpleNamespace
    sys.path.insert(0,str(SIM/'validation'))
    from reference_evidence import ulog_reference
    f=json.loads((Path(__file__).parent/'fixtures/p2_r1_late_notification.json').read_text())
    stamps=np.array([23360000,23408000,23496000,23600000,23704000])
    p={k:np.array([v]*5) for k,v in data['vehicle_local_position'].items() if isinstance(v,(int,float,bool))}
    p.update(timestamp=stamps,heading_reset_counter=np.array([1,2,2,2,2]),
             delta_heading=np.array([0]+[f['delta_heading']]*4))
    dq=[math.cos(f['delta_heading']/2),0,0,math.sin(f['delta_heading']/2)]
    a={'timestamp':stamps,'quat_reset_counter':np.array([1,2,2,2,2]),
       **{f'delta_q_reset[{i}]':np.array([0]+[v]*4) for i,v in enumerate(dq)}}
    e={'timestamp':stamps,**{k:np.array([True]*5) for k in (
       'cs_mag_aligned_in_flight','cs_mag_3d','cs_in_air','cs_gnss_pos')},
       'cs_vehicle_at_rest':np.array([False]*5)}
    status={'timestamp':stamps,'filter_fault_flags':np.zeros(5),
       **{k:np.ones(5) for k in ('accel_device_id','gyro_device_id','mag_device_id','baro_device_id')}}
    control={k:np.array([row[k] for row in f['internal_setpoints']]) for k in ('timestamp','yaw')}
    sets=dict(vehicle_local_position=p,vehicle_attitude=a,estimator_status_flags=e,
              estimator_status=status,vehicle_local_position_setpoint=control)
    log=SimpleNamespace(data_list=[SimpleNamespace(name='estimator_status',multi_id=0)],
                        get_dataset=lambda name:SimpleNamespace(data=sets[name]))
    reference={'accepted_count':1,'lock_sim_s':None,'accepted':[{
        'delta_heading':f['delta_heading'],'quaternion':{'delta_q_reset':dq},
        'anchor_after':f['anchor_after'],'accepted_sim_s':f['controller_pair_accepted_s']}]}
    result={'transitions':[{'to':'PRESTREAM_SAFE_SETPOINTS','sim_s':13.5}]}
    with pytest.raises(ValueError,match='cached setpoint correction missing/doubled'):
        ulog_reference(log,result,reference)


def test_reference_backwards_timestamp(config,data):
    ref=manager(config,data);advance(data,23);ref.inspect(data,sample(),23,50,"TAKEOFF")
    advance(data,22.9)
    with pytest.raises(ValueError,match="backwards"):ref.inspect(data,sample(),23.1,50.1,"TAKEOFF")


@pytest.mark.parametrize("fault", ["duplicate_delta","untracked_delta","aiding_source"])
def test_ambiguous_metadata_and_source_change_rejected(config,data,fault):
    ref=manager(config,data);advance(data,23)
    if fault=='duplicate_delta':
        reset(data,quaternion=False);ref.inspect(data,sample(),23,50,'TAKEOFF')
        data['vehicle_local_position']['delta_heading']=-.02
    elif fault=='untracked_delta':data['vehicle_local_position']['delta_heading']=.03
    else:data['estimator_status_flags']['cs_ev_vel']=True
    advance(data,23.04)
    with pytest.raises(ValueError):ref.inspect(data,sample(),23.04,50.04,'TAKEOFF')


@pytest.mark.parametrize("reference", [None,{}, {'state':'locked','policy':V2,'lock_sim_s':40,'stable_since_sim_s':39}])
def test_acceptance_rejects_missing_fabricated_alignment(config,reference):
    result={'final_state':'COMPLETE','origin_ned':[8,-3,6],'initial_yaw_ned':0,'reference':reference}
    assert acceptance.evaluate_flight(result,[],config)['status']!='passed'


def test_revised_mission_preserves_full_geometry_windows_and_periodic_yaw(config,data):
    cache=timing.StateCache(config)
    control=mission.Mission(config,cache,50,True)
    position=(8.,-3.,6.);yaw=0.;nav=4;armed=1;landed=True
    reset_done=False;reset_time=None;land_time=None;commands=[];samples=[]
    for tick in range(4000):
        sim=10+tick*.05;wall=50+tick*.05
        advance(data,sim)
        if armed==2 and position[2]<4.4 and not reset_done:
            reset(data,.02);yaw+=.02;reset_done=True;reset_time=sim
        e=data['estimator_status_flags']
        e.update(cs_in_air=not landed,cs_vehicle_at_rest=landed,
                 cs_mag_aligned_in_flight=reset_done,cs_mag_3d=reset_done)
        p=data['vehicle_local_position']
        p.update(x=position[0],y=position[1],z=position[2],vx=0.,vy=0.,vz=0.,
                 heading_good_for_control=reset_done and sim-reset_time>1)
        data['vehicle_attitude']['q']=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]
        data['vehicle_status'].update(arming_state=armed,nav_state=nav,
                                     pre_flight_checks_pass=land_time is None or armed == 2)
        data['vehicle_land_detected']['landed']=landed
        cache.clock(sim,wall)
        for key,value in data.items():cache.update(key,copy.deepcopy(value),wall)
        action=control.tick(sim,wall,True)
        if action['sample']:samples.append(action['sample'])
        if action['command']:
            cmd=action['command']['command'];commands.append(cmd)
            control.ack({'timestamp':int(sim*1e6),'command':cmd,'result':0,'target_system':201,'target_component':191,'from_external':False},sim,wall)
            if cmd==176:nav=14
            elif cmd==400:armed=2
            else:nav=18;land_time=sim
        if action['setpoint']:
            position=action['setpoint']['position'];yaw=action['setpoint']['yaw']
            if control.reference and control.reference.accepted:
                assert control.initial_yaw==pytest.approx(.02)
            if position[2]<5.98:landed=False
        if land_time is not None and sim-land_time>1:landed=True;armed=1;nav=14
        if control.done:break
    assert control.state=='COMPLETE' and commands==[176,400,21]
    assert control.origin==(8.,-3.,6.)
    assert tuple(control.windows)==acceptance.PHASES
    assert len(control.reference.accepted)==1
    assert control.reference.lock_sim_s>reset_time+6
    for phase,window in control.windows.items():
        expected=10 if phase=='INITIAL_HOVER' else 5
        assert window['end_sim_s']-window['start_sim_s']>=expected
    assert control.windows['EAST_TEST']['goal_ned']==(8,-2,4)
    assert control.windows['NORTH_TEST']['goal_ned']==(9,-3,4)


@pytest.mark.parametrize("fault", [None,"in_flight","not_landed","failsafe","stale"])
def test_terminal_arming_eligibility_does_not_bypass_health(config,data,fault):
    advance(data,30)
    data['vehicle_status'].update(arming_state=1,nav_state=14,pre_flight_checks_pass=False)
    data['vehicle_land_detected']['landed']=True
    if fault=='in_flight':data['vehicle_status']['arming_state']=2
    if fault=='not_landed':data['vehicle_land_detected']['landed']=False
    if fault=='failsafe':data['vehicle_status']['failsafe']=True
    if fault=='stale':data['vehicle_attitude']['timestamp']=29000000
    cache=timing.StateCache(config);cache.clock(30,50)
    for key,value in data.items():cache.update(key,value,50)
    with pytest.raises(ValueError):cache.validate(30,50)
    if fault:
        with pytest.raises(ValueError):cache.validate(30,50,terminal_landed=True)
    else:
        assert cache.validate(30,50,terminal_landed=True)['arming_state']==1


@pytest.mark.parametrize("fault", [None,"complete","armed","continued_target"])
def test_offline_ground_abort_is_failed_without_invented_flight(config,tmp_path,monkeypatch,fault):
    sys.path.insert(0,str(SIM/'validation'))
    import collect_p2_evidence as offline
    keys=timing.STATE_TOPICS
    events=[{'event':'received','topic_key':key} for key in keys]
    abort={'event':'abort','reason':'observation_gap','sim_s':14.184}
    events.append(abort)
    (tmp_path/'ros-events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in events))
    (tmp_path/'summary.json').write_text(json.dumps({
        'run_id':'ground-abort','kind':'flight','controller_result':{
            'final_state':'COMPLETE' if fault=='complete' else 'RECOVERY'},
        'config':config,'ulog_files':[{'path':'fixture','sha256':'fixture'}],'rosbag_files':[]}))
    streams={key:[{}] for key in keys}
    streams.update(vehicle_command=[],trajectory_setpoint=[{'timestamp':15000000 if fault=='continued_target' else 14000000}])
    ulog={'vehicle_command':[],'vehicle_status':[{'arming_state':2 if fault=='armed' else 1}],
          'vehicle_land_detected':[{'landed':True}]}
    from types import SimpleNamespace
    monkeypatch.setattr(offline,'digest',lambda path:'fixture')
    monkeypatch.setattr(offline,'read_bag',lambda run:(streams,{},[abort]))
    monkeypatch.setattr(offline,'read_ulog',lambda *args:(SimpleNamespace(dropouts=[]),ulog))
    monkeypatch.setattr(offline,'wire_checks',lambda *args:{})
    if fault:
        with pytest.raises(ValueError):offline.verify(tmp_path)
    else:
        report=offline.verify(tmp_path)
        assert report['recording_integrity']=='passed' and report['flight_acceptance']=='failed'
        assert report['command_ids']==[] and report['observed_recovery']=='remained_grounded'
