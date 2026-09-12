"""Runtime-free finalization, identity and lossless sensor-state contracts."""
import json
from pathlib import Path
import sys
import pytest

SIM = Path(__file__).resolve().parents[1] / 'simulation/px4_gazebo'
sys.path.insert(0, str(SIM / 'validation'))
sys.path.insert(0, str(SIM / 'ros2_ws/src/gwm_sensor_adapter'))


def test_explicit_enclosing_identity_rejects_cross_run(tmp_path):
    from gwm_sensor_adapter.contracts import run_identity
    assert run_identity(tmp_path / 'flight-123' / 'sensors', 'flight-123') == 'flight-123'
    with pytest.raises(ValueError, match='run_identity'):
        run_identity(tmp_path / 'flight-123' / 'sensors', 'flight-456')
    with pytest.raises(ValueError, match='run_identity'):
        run_identity(tmp_path / 'sensors', 'sensors')


@pytest.mark.parametrize('text', ['{"a":NaN}', '{"a":1,"a":2}', '{"a":Infinity}'])
def test_strict_json_rejects_non_json_and_duplicate_keys(tmp_path, text):
    from p3_provenance import strict_json
    path=tmp_path/'data.json'; path.write_text(text)
    with pytest.raises(ValueError): strict_json(path)


def test_partial_jsonl_and_unfinalized_runtime_rejected(tmp_path):
    from p3_provenance import strict_jsonl, require_finalized
    path=tmp_path/'data.jsonl'; path.write_text('{"a":1}')
    with pytest.raises(ValueError, match='partial'): strict_jsonl(path)
    with pytest.raises(ValueError, match='finalization'): require_finalized(tmp_path)


def test_atomic_report_never_overwrites_history(tmp_path):
    from p3_provenance import atomic_json
    path=tmp_path/'original.json'
    atomic_json(path, {'status':'failed'}, exclusive=True)
    original=path.read_bytes()
    with pytest.raises(FileExistsError): atomic_json(path, {'status':'passed'}, exclusive=True)
    assert path.read_bytes() == original


def test_source_tree_extra_file_changes_frozen_identity(tmp_path):
    from p3_provenance import tree_manifest
    (tmp_path/'one.py').write_text('a=1')
    first=tree_manifest(tmp_path)
    (tmp_path/'extra.py').write_text('b=2')
    assert tree_manifest(tmp_path) != first


def state(pub, sample, x=1., callback=1):
    return dict(timestamp_us=pub,timestamp_sample_us=sample,x=x,y=0.,z=-1.,heading=0.,
                xy_valid=True,z_valid=True,callback_id=f'run:position:{callback}',
                receipt_monotonic_ns=callback*1000000,run_id='run')


def test_sensor_history_retains_distinct_equal_publication_outputs():
    from gwm_sensor_adapter.state import StateHistory
    h=StateHistory(300,.05)
    h.add(97568000, (0,0), state(97568000,97552000,callback=1))
    h.add(97568000, (0,0), state(97568000,97560000,x=2.,callback=2))
    assert len(h.items)==2
    assert h.match(97.568) is None  # no unsupported choice inside conflicting time group


def test_sensor_duplicate_does_not_refresh_original_receipt():
    from gwm_sensor_adapter.state import StateHistory
    h=StateHistory()
    h.add(1000000,(0,0),state(1000000,990000,callback=1))
    h.add(1000000,(0,0),state(1000000,990000,callback=2))
    assert len(h.items)==1 and h.reused==1
    assert h.match(1.)['state']['receipt_monotonic_ns']==1000000


def test_sensor_regression_and_reset_collision_remain_rejected():
    from gwm_sensor_adapter.state import StateHistory
    h=StateHistory()
    h.add(1000000,(0,0),state(1000000,990000))
    with pytest.raises(ValueError,match='backwards'):
        h.add(900000,(0,0),state(900000,890000,callback=2))
    assert h.match(1.) is None
    h=StateHistory(); h.add(1000000,(0,0),state(1000000,990000))
    h.add(1000000,(1,0),state(1000000,995000,callback=2))
    assert len(h.items)==2 and h.match(1.) is None


def finalized_ground(tmp_path,monkeypatch,name='ground-123',summary_fields=None,calibration_id=None):
    import p3_provenance as p
    run=tmp_path/name; run.mkdir()
    shared=dict(run_id=run.name,sample_evidence_contract=p.CONTRACT)
    config=run/'p3_sensors.yaml'; config.write_text('{}\n')
    frozen=dict(sample_evidence_contract=p.CONTRACT,files={
        'ros2_ws/src/gwm_sensor_adapter/gwm_sensor_adapter/node.py':'a'*64,
        'validation/collect_p3_evidence.py':'b'*64})
    p.atomic_json(run/'sensor-startup.json',dict(shared,config_sha256=p.digest(config),adapter_source_sha256='a'*64))
    p.atomic_json(run/'sensor-ready.json',dict(shared,calibration_id=calibration_id))
    p.atomic_json(run/'sensor-result.json',dict(shared,status='complete',received=0,
        recorder=dict(accepted=0,written=0,overflow=0,fsync_completed=True,writer_closed=True,drain_completed=True)))
    (run/'sensor-events.jsonl').write_text(json.dumps(dict(shared,kind='health',status='fresh'))+'\n')
    (run/'depth-index.jsonl').write_text(''); (run/'depth.bin').write_bytes(b'')
    (run/'gazebo-source-headers.jsonl').write_text('')
    summary=dict(shared,frozen_inputs=frozen,processes=[dict(name='sensor',exit_code=0)],status='collected')
    summary.update(summary_fields or {})
    p.atomic_json(run/'summary.json',summary)
    monkeypatch.setattr(p,'owned_process_snapshot',lambda:dict(owned_pid_namespace=True,remaining_live_processes=[]))
    p.seal_run(run,summary,run)
    return run,frozen


def test_finalized_receipt_hashes_exact_closed_records(tmp_path,monkeypatch):
    from p3_provenance import require_finalized
    run,frozen=finalized_ground(tmp_path,monkeypatch)
    assert require_finalized(run,frozen)['status']=='finalized'
    (run/'depth.bin').write_bytes(b'changed')
    with pytest.raises(ValueError,match='artifact_hash'): require_finalized(run,frozen)


@pytest.mark.parametrize('changed_case',range(7))
def test_ground_matrix_rechecks_each_raw_artifact_after_evaluation(tmp_path,monkeypatch,changed_case):
    import p3_provenance as p
    readiness=tmp_path/'readiness'; (readiness/'sensors').mkdir(parents=True)
    runtime_identity={'fixture':'same measured runtime'}
    p.atomic_json(readiness/'runtime-finalized.json',{'finalized_unix_ns':1})
    p.atomic_json(readiness/'summary.json',{'runtime_identity':runtime_identity})
    p.atomic_json(readiness/'sensors/sensor-ready.json',{'calibration_id':'calibration-1'})
    predecessor=p.predecessor(readiness)
    cases=['plane2','plane4','plane6','oblique','asymmetric','out_of_range','interruption']
    records=[]
    for index,case in enumerate(cases):
        run,frozen=finalized_ground(tmp_path,monkeypatch,name='ground-'+case,
            summary_fields=dict(readiness=predecessor,started_unix_ns=2+index,
                                runtime_identity=runtime_identity),calibration_id='calibration-1')
        p.atomic_json(run/'p3-evaluation.json',dict(run_id=run.name,
            sample_evidence_contract=p.CONTRACT,frozen_inputs=frozen,evaluator_sha256='b'*64,
            status='passed_expected_failure_diagnostic' if case=='interruption' else 'passed'))
        records.append(dict(case=case,run_id=run.name,launcher_exit=0,evaluator_exit=0,
            finalization_sha256=p.digest(run/'runtime-finalized.json'),
            evaluation_sha256=p.digest(run/'p3-evaluation.json')))
    matrix=tmp_path/'matrix'; matrix.mkdir()
    p.atomic_json(matrix/'summary.json',dict(status='passed',frozen_inputs=frozen,
        readiness=predecessor,runtime_identity=runtime_identity,calibration_id='calibration-1',
        plane2=records[0],trials=records[1:]))
    assert p.require_ground_matrix(matrix,frozen,readiness)['status']=='passed'
    changed=tmp_path/records[changed_case]['run_id']
    (changed/'depth.bin').write_bytes(b'mutated after the passing evaluation')
    # Both prerequisite digests still match; consuming the matrix must rehash raw data.
    assert p.digest(changed/'runtime-finalized.json')==records[changed_case]['finalization_sha256']
    assert p.digest(changed/'p3-evaluation.json')==records[changed_case]['evaluation_sha256']
    with pytest.raises(ValueError,match='artifact_hash_mismatch:depth.bin'):
        p.require_ground_matrix(matrix,frozen,readiness)


def test_finalization_rejects_cross_run_events_and_incomplete_drain(tmp_path,monkeypatch):
    import p3_provenance as p
    run,frozen=finalized_ground(tmp_path,monkeypatch)
    seal=p.strict_json(run/'runtime-finalized.json')
    result=p.strict_json(run/'sensor-result.json'); result['recorder']['drain_completed']=False
    p.atomic_json(run/'sensor-result.json',result)
    # Even an updated artifact digest cannot turn a non-drained writer into completion.
    seal['artifacts']['sensor-result.json']=dict(bytes=(run/'sensor-result.json').stat().st_size,sha256=p.digest(run/'sensor-result.json'))
    p.atomic_json(run/'runtime-finalized.json',seal)
    with pytest.raises(ValueError,match='writer_not_finalized'): p.require_finalized(run,frozen)


def test_cross_run_or_changed_arithmetic_evaluator_cannot_qualify(tmp_path):
    from p3_provenance import CONTRACT, atomic_json,require_evaluation
    run=tmp_path/'flight-123'; run.mkdir()
    frozen=dict(sample_evidence_contract=CONTRACT,files={'validation/collect_p3_coexistence.py':'a'*64})
    path=run/'p3-coexistence-evaluation.json'
    result=dict(run_id='flight-other',sample_evidence_contract=CONTRACT,frozen_inputs=frozen,evaluator_sha256='a'*64,status='passed')
    atomic_json(path,result)
    with pytest.raises(ValueError,match='run_contract'): require_evaluation(run,path.name,frozen)
    result.update(run_id=run.name,evaluator_sha256='b'*64); atomic_json(path,result)
    with pytest.raises(ValueError,match='arithmetic_changed'): require_evaluation(run,path.name,frozen)


def test_sensor_source_integer_precision_and_invalid_sample_rejected():
    from gwm_sensor_adapter.state import StateHistory
    stamp=2**53+17
    h=StateHistory(); h.add(stamp,(0,0),state(stamp,stamp-8))
    assert h.items[0][0]==stamp and type(h.items[0][0]) is int
    assert h.match(stamp/1000000,native_ns=stamp*1000)['timestamp_us']==stamp
    h=StateHistory()
    with pytest.raises(ValueError,match='invalid_state_sample'):
        h.add(1000000,(0,0),state(1000000,None))


def test_sensor_nearest_time_tie_prefers_earlier_group_without_value_search():
    from gwm_sensor_adapter.state import StateHistory
    h=StateHistory()
    h.add(1000000,(0,0),state(1000000,990000,x=100.,callback=1))
    h.add(1020000,(0,0),state(1020000,1010000,x=0.,callback=2))
    assert h.match(1.01)['state']['x']==100.
