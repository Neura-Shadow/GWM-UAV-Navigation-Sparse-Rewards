"""R2 wire, bounded handover and declared PX4 scheduling-model invariants."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

SIM=Path(__file__).resolve().parents[1]/'simulation/px4_gazebo'
sys.path.insert(0,str(SIM/'ros2_ws/src/gwm_px4_control'))
from gwm_px4_control import contracts,mission,timing
from gwm_px4_control.estimator_reference import ReferenceManager,V3


@pytest.fixture
def config():return contracts.load_config(SIM/'configs/p2_control.yaml')


@pytest.fixture
def data():return json.loads((Path(__file__).parent/'fixtures/p2_strict_v1_reset.json').read_text())['snapshots'][0]['data']


def advance(data,sim):
    for value in data.values():value['timestamp']=int(sim*1e6)


class PinnedPositionYawModel:
    """MC update reads cache, applies counter once, then loads current heading.

    PositionControl's resolved output never mutates the trajectory cache.
    This model does not represent attitude dynamics or prove runtime timing.
    """
    def __init__(self):self.counter=1;self.cached_yaw=math.nan;self.stamp=1
    def publish(self,yaw,stamp):self.cached_yaw=yaw;self.stamp=stamp
    def update(self,heading,stamp,counter,delta):
        if self.stamp and self.stamp<stamp and self.counter!=counter:
            self.cached_yaw=math.atan2(math.sin(self.cached_yaw+delta),math.cos(self.cached_yaw+delta))
        self.counter=counter
        return self.cached_yaw if math.isfinite(self.cached_yaw) else heading


@pytest.mark.parametrize('delta',[.03,-.03,.08,-.08])
@pytest.mark.parametrize('offset',[-1,0,1])
@pytest.mark.parametrize('delay',[1,4])
def test_nan_publication_invariant_across_reset_scheduling(delta,offset,delay):
    model=PinnedPositionYawModel();anchor=3.13
    corrected=math.atan2(math.sin(anchor+delta),math.cos(anchor+delta))
    model.publish(math.nan,100+offset)
    assert model.update(corrected,100,2,delta)==corrected
    for tick in range(delay+3):
        # Publication can precede notification by multiple complete control ticks.
        fields=contracts.position_setpoint((8.,-3.,4.),None,101+tick,contracts.INITIALIZATION)
        model.publish(fields['yaw'],fields['timestamp'])
        assert model.update(corrected,101+tick,2,delta)==corrected
        assert math.isnan(model.cached_yaw) and fields['yawspeed']==0.
    model.publish(corrected,120)
    assert model.update(corrected,121,2,delta)==corrected


def test_publication_before_reset_and_r1_overwrite_still_reproduced():
    model=PinnedPositionYawModel();model.publish(.4,10)
    assert model.update(.43,11,2,.03)==pytest.approx(.43)
    model.publish(.4,12)
    assert model.update(.43,13,2,.03)==.4
    model.publish(math.nan,14)
    assert model.update(.43,15,2,.03)==.43


@pytest.mark.parametrize('bad',[0.,math.nan,math.inf])
def test_initialization_requires_intent_not_invalid_number(bad):
    with pytest.raises(ValueError):contracts.position_setpoint((0,0,0),bad,1,contracts.INITIALIZATION)


def test_mode_specific_wire_masks_and_strict_json():
    wire=contracts.position_setpoint((0,0,-2),None,1,contracts.INITIALIZATION)
    evidence=contracts.encode_wire(wire,contracts.INITIALIZATION)
    assert evidence['fields']['yaw'] is None and evidence['active']['yaw'] is False
    assert evidence['fields']['yawspeed']==0. and evidence['active']['yawspeed'] is True
    assert 'NaN' not in contracts.strict_json(evidence)
    with pytest.raises(ValueError):contracts.encode_wire(wire)
    wire['yaw']=math.inf
    with pytest.raises(ValueError):contracts.encode_wire(wire,contracts.INITIALIZATION)


@pytest.mark.parametrize('first',['heading','quaternion'])
def test_v3_classifier_keeps_independent_pairing_and_idempotence(config,data,first):
    ref=ReferenceManager(config,data,(8,-3,6),.4)
    data['vehicle_local_position']['heading']=.4
    data['vehicle_attitude']['q']=[math.cos(.2),0,0,math.sin(.2)]
    advance(data,23)
    def change(key):
        if key=='heading':data['vehicle_local_position'].update(heading_reset_counter=2,delta_heading=.03,heading=.43)
        else:data['vehicle_attitude'].update(quat_reset_counter=2,delta_q_reset=[math.cos(.015),0,0,math.sin(.015)],q=[math.cos(.215),0,0,math.sin(.215)])
    change(first)
    sample={'arming_state':2,'nav_state':14,'landed':False}
    assert ref.inspect(data,sample,23,50,'TAKEOFF')['pending']
    assert max(abs(v) for v in ref.heading_drift(data).values())<1e-8
    change('quaternion' if first=='heading' else 'heading')
    data['estimator_status_flags'].update(cs_mag_aligned_in_flight=True,cs_in_air=True,cs_vehicle_at_rest=False)
    advance(data,23.04);ref.inspect(data,sample,23.04,50.04,'TAKEOFF')
    advance(data,23.2);assert ref.inspect(data,sample,23.2,50.2,'TAKEOFF')['accepted']
    for tick in (23.25,23.3):
        advance(data,tick);assert not ref.inspect(data,sample,tick,50+tick-23,'TAKEOFF').get('accepted')
    assert ref.anchor==pytest.approx(.43) and len(ref.accepted)==1


@pytest.mark.parametrize('fault',[None,'drift','post_lock_reset','health','stale'])
def test_full_v3_handover_and_fail_closed_states(config,data,fault):
    cache=timing.StateCache(config);control=mission.Mission(config,cache,50,True)
    position=(8.,-3.,6.);yaw=0.;nav=4;armed=1;landed=True;land_time=None;reset_time=None
    sent=[];commands=[];fault_applied=False;rejection=None
    for tick in range(4000):
        sim=round(10+tick*.05,6);wall=50+tick*.05;advance(data,sim)
        if armed==2 and position[2]<4.4 and reset_time is None:
            reset_time=sim;yaw+=.045  # .02 reference correction plus .025 real drift
            data['vehicle_local_position'].update(heading_reset_counter=2,delta_heading=.02)
            data['vehicle_attitude'].update(quat_reset_counter=2,delta_q_reset=[math.cos(.01),0,0,math.sin(.01)])
        if fault and control.state=='YAW_HANDOVER' and not fault_applied:
            fault_applied=True
            if fault=='drift':yaw+=.2
            elif fault=='post_lock_reset':data['vehicle_local_position']['heading_reset_counter']=3
            elif fault=='health':data['vehicle_status']['failsafe']=True
        data['estimator_status_flags'].update(cs_in_air=not landed,cs_vehicle_at_rest=landed,
            cs_mag_aligned_in_flight=reset_time is not None,cs_mag_3d=reset_time is not None)
        data['vehicle_local_position'].update(x=position[0],y=position[1],z=position[2],vx=0.,vy=0.,vz=0.,heading=yaw,
            heading_good_for_control=reset_time is not None and sim-reset_time>1)
        data['vehicle_attitude']['q']=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]
        data['vehicle_status'].update(arming_state=armed,nav_state=nav,pre_flight_checks_pass=land_time is None or armed==2)
        data['vehicle_land_detected']['landed']=landed
        cache.clock(sim,wall)
        for key,value in data.items():
            if fault=='stale' and fault_applied and key=='vehicle_attitude':continue
            cache.update(key,copy.deepcopy(value),wall)
        try:action=control.tick(sim,wall,True)
        except ValueError as exc:rejection=str(exc);break
        if action['command']:
            cmd=action['command']['command'];commands.append(cmd)
            control.ack({'timestamp':int(sim*1e6),'command':cmd,'result':0,'target_system':201,'target_component':191,'from_external':False},sim,wall)
            if cmd==176:nav=14
            elif cmd==400:armed=2
            else:nav=18;land_time=sim
        if action['setpoint']:
            sp=action['setpoint'];sent.append((sim,sp,control.reference.lock_sim_s if control.reference else None,yaw))
            position=sp['position']
            if sp['mode']==contracts.INITIALIZATION:assert sp['yaw'] is None
            else:
                assert control.reference.lock_sim_s is not None
                yaw=sp['yaw']
            if position[2]<5.98:landed=False
        if land_time is not None and sim-land_time>1:landed=True;armed=1;nav=14
        if control.done:break
    if fault:
        assert rejection is not None and control.state!='COMPLETE'
        return
    assert rejection is None, rejection
    assert commands==[176,400,21] and control.state=='COMPLETE'
    finite=[x for x in sent if x[1]['mode']==contracts.NOMINAL]
    assert finite[0][1]['yaw']==pytest.approx(finite[0][3])
    assert finite[0][1]['yaw']==pytest.approx(.045)
    for a,b in zip(finite,finite[1:]):
        assert abs(mission.wrap(b[1]['yaw']-a[1]['yaw']))<=math.radians(10)*(b[0]-a[0])+1e-8
    assert control.initial_yaw==pytest.approx(.02) and control.origin==(8,-3,6)
    assert control.windows['EAST_TEST']['goal_ned']==(8,-2,4)
    assert control.windows['NORTH_TEST']['goal_ned']==(9,-3,4)
    assert len(control.windows)==8 and control.handover['completed_sim_s']>control.handover['start_sim_s']


@pytest.mark.parametrize('fault',[None,'mask','early_finite','rate','drift','resolved','seed','reanchor','handover'])
def test_independent_mode_evaluator_rejects_fabricated_evidence(config,fault):
    import numpy as np
    from types import SimpleNamespace
    sys.path.insert(0,str(SIM/'validation'))
    from yaw_evidence import verify_yaw_ownership
    targets=[];events=[]
    for stamp in range(500000,6150000,50000):
        mode=contracts.INITIALIZATION if stamp<6000000 else contracts.NOMINAL
        fields=contracts.position_setpoint((0,0,-2),None if mode==contracts.INITIALIZATION else .4,stamp,mode)
        targets.append(fields)
        events.append({'event':'setpoint','mode':mode,'yaw_phase':'initialization' if stamp<6000000 else 'handover' if stamp<6100000 else 'nominal',**contracts.encode_wire(fields,mode)})
    stamps=np.arange(0,6300000,8000)
    p={'timestamp':stamps,'heading':np.full(len(stamps),.4),'heading_reset_counter':np.ones(len(stamps)),'delta_heading':np.zeros(len(stamps))}
    a={'timestamp':stamps,'quat_reset_counter':np.ones(len(stamps)),
       **{f'q[{i}]':np.full(len(stamps),v) for i,v in enumerate([math.cos(.2),0,0,math.sin(.2)])},
       **{f'delta_q_reset[{i}]':np.full(len(stamps),v) for i,v in enumerate([1,0,0,0])}}
    internal={'timestamp':np.arange(500000,6000000,100000),'yaw':np.full(55,.4),'yawspeed':np.zeros(55)}
    sets=dict(vehicle_local_position=p,vehicle_attitude=a,vehicle_local_position_setpoint=internal)
    log=SimpleNamespace(get_dataset=lambda name:SimpleNamespace(data=sets[name]))
    ref={'lock_sim_s':6.,'initial_anchor':.4,'accepted':[]}
    result={'initial_yaw_ned':.4,'origin_ned':[0,0,0],
            'handover':{'start_sim_s':6.,'completed_sim_s':6.1,'position_timestamp':6000000},
            'transitions':[{'to':'VERIFY_OFFBOARD','sim_s':.5},{'to':'INITIAL_HOVER','sim_s':6.1}]}
    streams={'trajectory_setpoint':targets,'vehicle_local_position':[{'timestamp':6000000,'heading':.4}]}
    if fault=='mask':events[0]['active']['yaw']=True
    elif fault=='early_finite':targets[0]['yaw']=.4
    elif fault=='rate':targets[0]['yawspeed']=.01
    elif fault=='drift':p['heading'][:]=.5
    elif fault=='resolved':internal['yaw'][2]=.3
    elif fault=='seed':streams['vehicle_local_position'][0]['heading']=.3
    elif fault=='reanchor':result['initial_yaw_ned']=.3
    elif fault=='handover':result['handover']['start_sim_s']=5.
    if fault:
        with pytest.raises(ValueError):verify_yaw_ownership(log,streams,events,result,config,ref)
    else:
        report=verify_yaw_ownership(log,streams,events,result,config,ref)
        assert report['status']=='passed' and report['matched_internal_samples']==55
