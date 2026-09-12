"""Deterministic control-consumption and bounded tracing regressions."""
import copy
import json
from pathlib import Path
import sys

import pytest

SIM=Path(__file__).resolve().parents[1]/'simulation/px4_gazebo'
sys.path.insert(0,str(SIM/'ros2_ws/src/gwm_px4_control'))
sys.path.insert(0,str(SIM/'validation'))
from gwm_px4_control import contracts, mission, timing
from gwm_px4_control.execution_timing import (TraceBuffer, transport_allowed,
    DispatchGuard, PublicationCoverage, controller_transport_env)
from timing_evidence import reconstruct, trace_metrics


def ground_fixture():
    config=contracts.load_config(SIM/'configs/p2_control.yaml')
    data=json.loads((Path(__file__).parent/'fixtures/p2_strict_v1_reset.json').read_text())['snapshots'][0]['data']
    data['vehicle_status'].update(arming_state=1,nav_state=4,pre_flight_checks_pass=True)
    data['vehicle_land_detected']['landed']=True
    cache=timing.StateCache(config)
    control=mission.Mission(config,cache,50,True,ground_diagnostic=True)
    return config,data,cache,control


def update(data,cache,sim):
    wall=50+sim-10
    cache.clock(sim,wall)
    for key,value in data.items():
        value['timestamp']=round(sim*1e6)
        cache.update(key,copy.deepcopy(value),wall)
    return wall


def test_ground_diagnostic_full_prestream_never_requests_command():
    config,data,cache,control=ground_fixture()
    targets=[]
    for tick in range(220):
        sim=round(10+tick*.05,6)
        wall=update(data,cache,sim)
        action=control.tick(sim,wall,True)
        assert action['command'] is None
        if action['setpoint']:
            assert action['setpoint']['mode']==contracts.INITIALIZATION
            targets.append(sim)
        if control.done:break
    assert control.done and not control.failure and control.state=='PRESTREAM_SAFE_SETPOINTS'
    assert targets[-1]-targets[0]>=2.9 and not control.transactions.records
    with pytest.raises(ValueError,match='ground_diagnostic'):
        control.request(176,sim,wall)
    with pytest.raises(ValueError,match='ground_diagnostic'):
        control.transition('REQUEST_OFFBOARD',sim,wall)


def test_continuous_source_does_not_hide_0344_consumption_gap():
    config,data,cache,control=ground_fixture()
    for tick in range(130):
        sim=round(10+tick*.05,6)
        wall=update(data,cache,sim)
        control.tick(sim,wall,True)
        if control.last_sample is not None:break
    # Deliver fresh source updates continuously without consuming them.
    cache.stats['vehicle_local_position']['max_gap_s']=0.
    for offset in [i*.02 for i in range(1,18)]+[.344]:
        wall=update(data,cache,round(sim+offset,6))
    assert cache.stats['vehicle_local_position']['max_gap_s']<=.024
    with pytest.raises(ValueError,match='observation_gap'):
        control.tick(sim+.344,wall,True)
    assert not control.transactions.records


@pytest.mark.parametrize('flight,ground',[(False,True),(True,True),(False,False)])
def test_transport_rejects_commands_independently_of_mission(flight,ground):
    with pytest.raises(ValueError,match='transport boundary'):
        transport_allowed('vehicle_command',flight,ground)
    transport_allowed('trajectory_setpoint',flight,ground)


def test_trace_is_bounded_and_records_cold_calls_without_disk(tmp_path):
    now=iter([1.,1.3,2.,2.001,3.,3.002])
    trace=TraceBuffer(2,lambda:next(now))
    for _ in range(3):
        with trace.span('publish',10.,source_timestamp=10000000):pass
    assert trace.overflow==1 and len(trace.records)==2
    assert trace.records[0]['duration_wall_s']==pytest.approx(.3)
    assert [r['call_index'] for r in trace.records]==[1,2]
    assert not list(tmp_path.iterdir())
    receipt=trace.persist(tmp_path/'trace.jsonl')
    assert not receipt['complete'] and receipt['overflow']==1


def test_old_post_publish_json_is_not_publication_or_receipt_time():
    received={'event':'received','topic_key':'vehicle_local_position','fields':{'timestamp':13820000},
              'monotonic_s':100.,'ros_sim_s':13.82}
    transition={'event':'transition','to':'PRESTREAM_SAFE_SETPOINTS','monotonic_s':99.9,'ros_sim_s':13.7}
    sample={'event':'sample','ros_sim_s':13.832,'sample':{'t':13.82,'receipt_monotonic_s':100.01}}
    target={'event':'setpoint','ros_sim_s':13.832,'monotonic_s':100.344,'fields':{'timestamp':13832000}}
    result={'reference_policy':'p2-estimator-reference-v3','failure':None,'transactions':[]}
    report=reconstruct([transition,received,target,sample],result)
    assert report['selection_to_post_publish_event_wall_s']==pytest.approx(.334)
    assert report['callback_entry_to_post_publish_event_wall_s']==pytest.approx(.344)
    assert report['publish_call_duration_wall_s'] is None
    assert report['source_age_at_dispatch_sim_s'] is None


def test_dispatch_fails_closed_after_slow_sink_without_next_command():
    now=[10.]
    guard=DispatchGuard(.05,1.5)
    guard.begin(now[0])
    commands=[]
    # Synchronous evidence is retained because native SHM publish was measured
    # as the cause. A slow sink is not preempted; no following action may pass.
    def fake_sink():now[0]+=.333
    fake_sink()
    with pytest.raises(ValueError,match='stale_action'):
        guard.check(now[0],10.,True)
        commands.append(176)
    assert not commands
    guard.end()


@pytest.mark.parametrize('checked,valid',[(None,True),(8.,True),(10.,False)])
def test_dispatch_requires_current_graph_ownership(checked,valid):
    guard=DispatchGuard(.05,1.5)
    guard.begin(10.)
    with pytest.raises(ValueError,match='stale_or_invalid_graph'):
        guard.check(10.01,checked,valid)


def test_single_owner_and_no_reentrant_control():
    owner=[1]
    guard=DispatchGuard(.05,1.5,lambda:owner[0])
    guard.begin(10.)
    with pytest.raises(ValueError,match='concurrent_or_reentrant'):
        guard.begin(10.)
    owner[0]=2
    with pytest.raises(ValueError,match='control_owner_missing'):
        guard.check(10.01,10.,True)
    guard.end()
    with pytest.raises(ValueError,match='concurrent_or_reentrant'):
        guard.begin(11.)


def test_prestream_uses_successful_publications_not_elapsed_mission_time():
    coverage=PublicationCoverage(.2)
    with pytest.raises(ValueError,match='incomplete'):coverage.require(2.)
    for n in range(40):coverage.record(20.+n*.05)
    with pytest.raises(ValueError,match='incomplete'):coverage.require(2.)
    coverage.record(22.)
    coverage.require(2.)
    with pytest.raises(ValueError,match='publication_gap'):coverage.record(22.344)
    with pytest.raises(ValueError,match='publication_gap'):coverage.record(22.)


def test_trace_snapshot_order_and_failed_persistence(tmp_path):
    trace=TraceBuffer(3)
    value={'source':[1,2]}
    trace.append('received',10.,10.01,2.,acquisition=value)
    value['source'][0]=99
    trace.append('received',11.,11.01,3.,acquisition=value)
    assert trace.records[0]['acquisition']['source']==[1,2]
    class FailedSink:
        def open(self,*args):raise OSError('full disk')
    with pytest.raises(OSError,match='full disk'):trace.persist(FailedSink())
    receipt=trace.persist(tmp_path/'trace.jsonl')
    rows=[json.loads(s) for s in (tmp_path/'trace.jsonl').read_text().splitlines()]
    assert receipt['complete'] and rows==trace.records
    assert [r['seq'] for r in rows]==[1,2]
    assert rows[0]['start_wall_s']==10.


def test_callback_receipt_stays_original_during_later_selection():
    config,data,cache,_=ground_fixture()
    wall=update(data,cache,10.)
    sample=cache.validate(10.01,wall+.01)
    assert sample['source_callback_entry_monotonic_s']==wall
    assert sample['selection_monotonic_s']==wall+.01
    # Duplicate source data does not acquire a newer callback freshness stamp.
    cache.update('vehicle_local_position',copy.deepcopy(data['vehicle_local_position']),wall+.02)
    assert cache.receipts['vehicle_local_position']==wall


def test_profile_is_controller_local_and_rejects_unregistered_relaxation():
    import os
    config=json.loads((SIM/'configs/p2_timing.yaml').read_text())
    before=dict(os.environ)
    env=controller_transport_env(config)
    assert env['FASTDDS_BUILTIN_TRANSPORTS']=='UDPv4'
    assert env['RMW_FASTRTPS_PUBLICATION_MODE']=='SYNCHRONOUS'
    assert dict(os.environ)==before
    config['dispatch_budget_wall_s']=.4
    with pytest.raises(ValueError,match='Unregistered'):controller_transport_env(config)


def timing_fixture():
    trace=TraceBuffer(1000)
    streams={k:[] for k in ('offboard_control_mode','trajectory_setpoint','vehicle_command')}
    config=contracts.load_config(SIM/'configs/p2_control.yaml')
    tc=json.loads((SIM/'configs/p2_timing.yaml').read_text())
    for n in range(41):
        sim,wall=10+n*.05,20+n*.05
        trace.append('mission_tick',wall,wall+.001,sim,selected_source_timestamp=round(sim*1e6),
                     source_age_at_control_sim_s=0.)
        for key in ('offboard_control_mode','trajectory_setpoint'):
            trace.append('publish:'+key,wall+.002,wall+.003,sim,
                source_timestamp=round(sim*1e6),selected_source_timestamp=round(sim*1e6),source_age_at_dispatch_sim_s=0.)
            streams[key].append({'timestamp':round(sim*1e6)})
        trace.append('control_callback',wall,wall+.004,sim)
    result={'trace':{'complete':True,'overflow':0,'records':len(trace.records),'calls':dict(trace.calls)},
            'failure':None,'flight':'not_run','timing_configuration':tc}
    return trace.records,result,streams,config


def test_independent_timing_reconciles_and_rejects_missing_trace_or_output():
    rows,result,streams,config=timing_fixture()
    report=trace_metrics(rows,result,streams,[],config)
    assert report['status']=='passed' and report['actual_prestream_publication_sim_s']==2.
    assert report['publish_call_duration_wall_s']==pytest.approx(.001)
    streams['trajectory_setpoint'].pop()
    with pytest.raises(ValueError,match='Publish/bag'):trace_metrics(rows,result,streams,[],config)
    result['trace']['overflow']=1
    with pytest.raises(ValueError,match='Incomplete'):trace_metrics(rows,result,streams,[],config)


def test_independent_timing_rejects_late_native_call_even_with_complete_bag():
    rows,result,streams,config=timing_fixture()
    for row in rows[:4]:
        if row['operation'].startswith('publish:'):
            row['end_wall_s']+=.333
            row['duration_wall_s']+=.333
        if row['operation']=='control_callback':row['end_wall_s']+=.333
    with pytest.raises(ValueError,match='Late publication'):trace_metrics(rows,result,streams,[],config)
