"""Exercise production offline sequencing with fake children, no runtime."""
import json
from pathlib import Path
import signal
import subprocess
import sys

import pytest

SIM=Path(__file__).resolve().parents[1]/'simulation/px4_gazebo'
sys.path.insert(0,str(SIM/'scripts')); sys.path.insert(0,str(SIM/'validation'))
import offline_jobs as jobs
from p3_provenance import atomic_json


@pytest.fixture
def case(tmp_path):
    run=tmp_path/'run'; run.mkdir()
    frozen={'files':{f'validation/{name}.py':name for name in
        ('collect_p2_evidence','collect_p3_coexistence','reference_evidence','yaw_evidence','timing_evidence')}}
    frozen['sample_evidence_contract']='p3-sample-evidence-v2'
    shared=dict(run_id='run',sample_evidence_contract='p3-sample-evidence-v2',frozen_inputs=frozen,
                analysis_inputs=frozen,evaluation_finalized=True,historical_reanalysis=False)
    control=dict(shared,recording_integrity='passed',flight_acceptance='passed',
        evaluator_sha256='collect_p2_evidence',reference_evaluator_sha256='reference_evidence',
        yaw_evaluator_sha256='yaw_evidence',timing_evaluator_sha256='timing_evidence',
        yaw_ownership={'evidence':{'contract':'p3-yaw-causal-evidence-v1',
            'A_external_initialization':{'status':'verified'},'B_handover':{'status':'verified'},
            'C_physical_behavior':{'status':'verified'},
            'D_exact_internal_relation':{'status':'verified'}}})
    sensor=dict(shared,status='passed',evaluator_sha256='collect_p3_coexistence')
    trial=dict(run_id='run',kind='flight',status='passed',controller_result={'flight':'passed'})
    return run,frozen,control,sensor,trial


@pytest.mark.parametrize('failure', ['timeout','nonzero','partial','stale','other_run','hash','missing_hash','unknown','unobservable','contradicted'])
def test_control_failure_stops_before_sensor(case,tmp_path,failure):
    run,frozen,control,sensor,trial=case; calls=[]
    def execute(command,output,deadline):
        calls.append(command)
        if failure=='timeout': raise subprocess.TimeoutExpired(command,deadline)
        if failure=='nonzero': return 2
        if failure=='partial': (run/'p2-offline-evaluation.json').write_text('{'); return 0
        if failure=='stale': control['analysis_inputs']={}
        if failure=='other_run': control['run_id']='other'
        if failure=='hash': control['evaluator_sha256']='old'
        if failure=='missing_hash': control.pop('timing_evaluator_sha256')
        if failure=='unknown': control['flight_acceptance']='unknown'
        if failure=='unobservable': control['yaw_ownership']['evidence']['D_exact_internal_relation']['status']='unobservable_from_recording'
        if failure=='contradicted': control['yaw_ownership']['evidence']['D_exact_internal_relation']['status']='contradicted'
        atomic_json(run/'p2-offline-evaluation.json',control,exclusive=True); return 0
    with pytest.raises(jobs.OfflineJobError) as error:
        jobs.evaluate_trial(SIM,run,tmp_path,1,frozen,trial,{},execute=execute)
    assert len(calls)==1
    assert error.value.stage=='control'
    assert error.value.kind==('infrastructure_timeout' if failure=='timeout' else 'offline_evaluation_failure')
    assert trial['status']=='passed'


@pytest.mark.parametrize('timeout', [False,True])
def test_sensor_waits_for_valid_control_and_has_own_finite_budget(case,tmp_path,timeout):
    run,frozen,control,sensor,trial=case; calls=[]; entry={}
    def execute(command,output,deadline):
        calls.append(deadline)
        if len(calls)==1: atomic_json(run/'p2-offline-evaluation.json',control,exclusive=True)
        else:
            assert (run/'p2-offline-evaluation.json').is_file()
            if timeout: raise subprocess.TimeoutExpired(command,deadline)
            sensor['control_evaluation_sha256']=jobs.digest(run/'p2-offline-evaluation.json')
            atomic_json(run/'p3-coexistence-evaluation.json',sensor,exclusive=True)
        return 0
    if timeout:
        with pytest.raises(jobs.OfflineJobError) as error:
            jobs.evaluate_trial(SIM,run,tmp_path,1,frozen,trial,entry,execute=execute)
        assert error.value.stage=='sensor' and error.value.kind=='infrastructure_timeout'
    else: jobs.evaluate_trial(SIM,run,tmp_path,1,frozen,trial,entry,execute=execute)
    assert calls==[90,120]


def test_stale_preexisting_result_prevents_even_control_launch(case,tmp_path):
    run,frozen,control,sensor,trial=case
    atomic_json(run/'p2-offline-evaluation.json',control)
    def unexpected(*a): pytest.fail('stale result must stop before launch')
    with pytest.raises(jobs.OfflineJobError):
        jobs.evaluate_trial(SIM,run,tmp_path,1,frozen,trial,{},execute=unexpected)


def test_owned_group_cleanup_includes_children_after_leader_exits(tmp_path,monkeypatch):
    signals=[]
    class Child:
        pid=1234
        def wait(self,timeout): raise subprocess.TimeoutExpired('fake',timeout)
        def poll(self): return 0  # parent exit does not mean descendants exited
    monkeypatch.setattr(jobs.subprocess,'Popen',lambda *a,**k:Child())
    monkeypatch.setattr(jobs.os,'killpg',lambda pid,sig:signals.append((pid,sig)),raising=False)
    with pytest.raises(subprocess.TimeoutExpired): jobs.execute_offline(['fake'],tmp_path/'job.log',.1)
    assert (1234,signal.SIGTERM) in signals and (1234,getattr(signal,'SIGKILL',9)) in signals


@pytest.mark.parametrize('failure', ['timeout','nonzero','unknown'])
def test_actual_qualification_main_never_starts_next_flight_after_offline_failure(case,tmp_path,monkeypatch,failure):
    import run_p3_qualification as runner
    smoke,frozen,control,sensor,trial=case
    matrix=tmp_path/'matrix'; matrix.mkdir(); atomic_json(matrix/'summary.json',{})
    atomic_json(smoke/'p2-offline-evaluation.json',control)
    sensor['control_evaluation_sha256']=jobs.digest(smoke/'p2-offline-evaluation.json')
    atomic_json(smoke/'p3-coexistence-evaluation.json',sensor)
    trial.update(identity={'p3':'fixture'},started_unix_ns=2,runtime_identity={})
    atomic_json(smoke/'summary.json',trial)
    atomic_json(smoke/'p3-prerequisites.json',dict(ground_matrix=str(matrix),
        ground_matrix_sha256=jobs.digest(matrix/'summary.json'),readiness={'run':str(smoke)}))
    root=tmp_path/'workspace'; (root/'runs').mkdir(parents=True)
    monkeypatch.setenv('GWM_SIM_ROOT',str(root))
    monkeypatch.setattr(sys,'argv',['qualification','--run-qualification','--allow-simulated-flight','--smoke-run',str(smoke)])
    monkeypatch.setattr(runner,'require_gates',lambda *a:None)
    monkeypatch.setattr(runner,'inputs',lambda *a:frozen)
    monkeypatch.setattr(runner,'require_finalized',lambda *a:None)
    monkeypatch.setattr(runner,'require_ground_matrix',lambda *a:dict(finalized_unix_ns=1,calibration_id='c'))
    monkeypatch.setattr(runner,'predecessor',lambda *a:dict(finalized_unix_ns=1))
    real_digest=runner.digest
    monkeypatch.setattr(runner,'digest',lambda p: frozen['files']['validation/'+p.name] if p.suffix=='.py' else real_digest(p))
    flights=[]; offline=[]
    def launch(command,output,deadline=600):
        flights.append(command); run=root/'runs'/'fresh-trial'; run.mkdir()
        atomic_json(run/'summary.json',{**trial,'run_id':run.name,'started_unix_ns':3})
        output.write_text('P2 evidence: '+str(run)+'\n'); return 0
    monkeypatch.setattr(runner,'execute',launch)
    def evaluate(command,output,deadline):
        offline.append(command)
        if failure=='timeout': raise subprocess.TimeoutExpired(command,deadline)
        if failure=='nonzero': return 2
        atomic_json(Path(command[-1])/'p2-offline-evaluation.json',
                    {**control,'run_id':'fresh-trial','flight_acceptance':'unknown'}); return 0
    monkeypatch.setattr(jobs,'execute_offline',evaluate)
    assert runner.main()==1
    batch=next(p for p in (root/'runs').iterdir() if 'qualification' in p.name)
    result=json.loads((batch/'summary.json').read_text())
    assert len(flights)==len(offline)==len(result['trials'])==1
    assert result['passed']==0 and result['status']=='incomplete'
    assert result['failure_kind']==('infrastructure_timeout' if failure=='timeout' else 'offline_evaluation_failure')
    assert result['trials'][0]['launcher_exit']==0
