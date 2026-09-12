"""Pure P3-R1 identity, immutable JSON and closed-runtime prerequisites."""
import hashlib
import json
import os
from pathlib import Path
import time
import uuid
import subprocess

CONTRACT = 'p3-sample-evidence-v2'


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def _pairs(pairs):
    result={}
    for key,value in pairs:
        if key in result: raise ValueError('duplicate_json_key:'+key)
        result[key]=value
    return result


def _constant(value): raise ValueError('non_json_constant:'+value)


def strict_loads(text):
    return json.loads(text,object_pairs_hook=_pairs,parse_constant=_constant)


def strict_json(path): return strict_loads(Path(path).read_text())


def strict_jsonl(path):
    text=Path(path).read_text()
    if text and not text.endswith('\n'): raise ValueError('partial_jsonl:'+str(path))
    return [strict_loads(line) for line in text.splitlines()]


def atomic_json(path, value, exclusive=False):
    path=Path(path)
    raw=json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n'
    temporary=path.with_name(path.name+'.tmp-'+uuid.uuid4().hex)
    with temporary.open('x') as stream:
        stream.write(raw); stream.flush(); os.fsync(stream.fileno())
    try:
        if exclusive:
            # Hard-link publication is atomic and cannot replace historical data.
            os.link(temporary,path)
        else: os.replace(temporary,path)
    finally:
        if temporary.exists(): temporary.unlink()


def tree_manifest(directory):
    directory=Path(directory)
    return {p.relative_to(directory).as_posix():digest(p) for p in sorted(directory.rglob('*'))
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}


def frozen_inputs(sim):
    """All arithmetic and launch inputs; no mutable run/build receipt paths."""
    sim=Path(sim)
    files={}
    for folder in ('scripts','configs','validation','ros2_ws/src/gwm_px4_control','ros2_ws/src/gwm_sensor_adapter'):
        files.update({folder+'/'+name:value for name,value in tree_manifest(sim/folder).items()})
    return dict(sample_evidence_contract=CONTRACT,files=files)


def runtime_inputs(root,sim):
    """Measured dependency/build identity shared by readiness, ground and flight."""
    root=Path(root); sim=Path(sim)
    packages={}
    for lane,package in (('p2','gwm_px4_control'),('p3','gwm_sensor_adapter')):
        receipt=strict_json(root/'state'/(lane+'-built.json'))
        source=tree_manifest(sim/'ros2_ws/src'/package)
        identity=hashlib.sha256(json.dumps(source,sort_keys=True).encode()).hexdigest()
        mirror=Path(receipt['workspace'])/'src'/package
        installed=next(Path(receipt['install']).rglob('site-packages/'+package+'/__init__.py')).parent
        if (receipt['package_hash']!=identity or tree_manifest(mirror)!=source
                or tree_manifest(installed)!=receipt['installed_source_files']
                or receipt['installed_source_files']!=tree_manifest(mirror/package)):
            raise ValueError('runtime_build_identity:'+package)
        packages[package]=dict(package_hash=identity,installed_source_files=receipt['installed_source_files'])
        if lane=='p3' and receipt.get('dependency_controller_package_hash')!=packages['gwm_px4_control']['package_hash']:
            raise ValueError('Sensor build dependency controller changed')
    px4=root/'upstream/PX4-Autopilot'
    assets={}
    for name in ('x500_depth','OakD-Lite','x500','x500_base'):
        for path in (px4/'Tools/simulation/gz/models'/name).rglob('*'):
            if path.is_file(): assets[path.relative_to(px4).as_posix()]=digest(path)
    for name in ('Tools/simulation/gz/worlds/default.sdf','src/modules/simulation/gz_bridge/server.config',
                 'ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500',
                 'ROMFS/px4fmu_common/init.d-posix/airframes/4002_gz_x500_depth'):
        assets[name]=digest(px4/name)
    binary=digest(px4/'build/px4_sitl_default_linux/bin/px4')
    if binary!=strict_json(root/'state/p0-built.json')['measured']['px4_binary_sha256']:
        raise ValueError('runtime_px4_binary_changed')
    revisions={name:subprocess.check_output(['git','rev-parse','HEAD'],cwd=path,text=True).strip()
        for name,path in (('px4',px4),('models',px4/'Tools/simulation/gz'),
            ('px4_msgs',root/'ros_ws/src/px4_msgs'),('dds_agent',root/'upstream/Micro-XRCE-DDS-Agent'))}
    lock=strict_json(sim/'configs/versions.lock.yaml')
    if any(revisions[name]!=pin['commit'] for name,pin in lock['sources'].items()):
        raise ValueError('runtime_dependency_revision_changed')
    return dict(packages=packages,asset_hashes=assets,px4_binary_sha256=binary,dependency_revisions=revisions)


def require_same_inputs(actual, expected):
    if actual != expected: raise ValueError('frozen_inputs_mismatch')


def artifact_manifest(directory):
    return {p.name:dict(bytes=p.stat().st_size,sha256=digest(p))
            for p in sorted(Path(directory).iterdir()) if p.is_file()}


def verify_artifacts(directory, artifacts):
    directory=Path(directory).resolve()
    for name,item in artifacts.items():
        path=(directory/name).resolve()
        if directory not in path.parents: raise ValueError('artifact_outside_run')
        if path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise ValueError('artifact_hash_mismatch:'+name)


def _sensor_completion(run, sensor):
    result=strict_json(sensor/'sensor-result.json')
    recorder=result.get('recorder',{})
    if (result.get('run_id')!=run.name or result.get('sample_evidence_contract')!=CONTRACT
            or result.get('status')!='complete' or recorder.get('overflow')!=0
            or not all(recorder.get(k) is True for k in ('fsync_completed','writer_closed','drain_completed'))
            or recorder.get('accepted')!=recorder.get('written') or recorder.get('written')!=result.get('received')):
        raise ValueError('sensor_writer_not_finalized')
    for name in ('sensor-startup.json','sensor-ready.json'):
        metadata=strict_json(sensor/name)
        if metadata.get('run_id')!=run.name or metadata.get('sample_evidence_contract')!=CONTRACT:
            raise ValueError('sensor_metadata_run_identity:'+name)
    for name in ('sensor-events.jsonl','depth-index.jsonl','gazebo-source-headers.jsonl'):
        for row in strict_jsonl(sensor/name):
            if row.get('run_id')!=run.name or row.get('sample_evidence_contract')!=CONTRACT:
                raise ValueError('sensor_record_run_identity:'+name)
    return result


def owned_process_snapshot():
    """PID-1 launchers own the complete private namespace, including orphans."""
    if os.getpid()!=1: raise ValueError('finalization_requires_owned_pid_namespace')
    while True:
        try:
            child,_=os.waitpid(-1,os.WNOHANG)
            if not child: break
        except ChildProcessError: break
    remaining=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid(): continue
        try:
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z': remaining.append(dict(pid=int(entry.name),state=fields[0]))
        except FileNotFoundError: pass
    if remaining: raise ValueError('owned_runtime_descendants_remain:'+repr(remaining))
    return dict(owned_pid_namespace=True,remaining_live_processes=[])


def seal_run(run, summary, sensor_dir):
    """Called only after owned runtime exits and the final summary is persisted."""
    run=Path(run); sensor_dir=Path(sensor_dir)
    if summary.get('run_id')!=run.name or any(p.get('exit_code') is None for p in summary['processes']):
        raise ValueError('owned_process_finalization_incomplete')
    process_snapshot=owned_process_snapshot()
    _sensor_completion(run,sensor_dir)
    if sensor_dir!=run:
        controller=strict_json(run/'controller-result.json')
        if controller.get('run_id')!=run.name: raise ValueError('controller_result_run_identity')
        if not summary.get('ulog_files') or not summary.get('rosbag_files'):
            raise ValueError('control_recording_finalization_incomplete')
    # Runtime files only; later immutable evaluator reports are separate dependents.
    artifacts={p.relative_to(run).as_posix():dict(bytes=p.stat().st_size,sha256=digest(p))
               for p in sorted(run.rglob('*')) if p.is_file() and p.name!='runtime-finalized.json'}
    value=dict(schema_version=2,run_id=run.name,sample_evidence_contract=CONTRACT,status='finalized',
        finalized_unix_ns=time.time_ns(),owned_processes_stopped=True,sensor_directory=sensor_dir.relative_to(run).as_posix(),
        process_snapshot=process_snapshot,frozen_inputs=summary['frozen_inputs'],
        runtime_identity=summary.get('runtime_identity'),artifacts=artifacts)
    atomic_json(run/'runtime-finalized.json',value,exclusive=True)
    return value


def require_finalized(run, expected_frozen=None, dependent=None):
    run=Path(run)
    if not (run/'runtime-finalized.json').is_file(): raise ValueError('runtime_finalization_missing')
    seal=strict_json(run/'runtime-finalized.json')
    if (seal.get('run_id')!=run.name or seal.get('sample_evidence_contract')!=CONTRACT
            or seal.get('status')!='finalized' or seal.get('owned_processes_stopped') is not True):
        raise ValueError('runtime_finalization_invalid')
    if expected_frozen is not None: require_same_inputs(seal['frozen_inputs'],expected_frozen)
    required={'summary.json','sensor-result.json','sensor-startup.json','sensor-ready.json','sensor-events.jsonl','depth-index.jsonl','depth.bin','gazebo-source-headers.jsonl','p3_sensors.yaml'}
    sensor=run/seal['sensor_directory']
    prefix='' if sensor==run else seal['sensor_directory']+'/'
    required={'summary.json'}|{prefix+n for n in required-{'summary.json'}}
    if not required.issubset(seal['artifacts']): raise ValueError('finalization_artifact_list_incomplete')
    verify_artifacts(run,seal['artifacts'])
    summary=strict_json(run/'summary.json')
    if summary.get('run_id')!=run.name or summary.get('frozen_inputs')!=seal['frozen_inputs']:
        raise ValueError('finalization_summary_identity')
    if summary.get('runtime_identity')!=seal.get('runtime_identity'): raise ValueError('finalization_runtime_identity')
    _sensor_completion(run,sensor)
    startup=strict_json(sensor/'sensor-startup.json')
    if startup.get('config_sha256')!=digest(sensor/'p3_sensors.yaml'):
        raise ValueError('startup_config_hash_mismatch')
    if startup.get('adapter_source_sha256')!=seal['frozen_inputs']['files']['ros2_ws/src/gwm_sensor_adapter/gwm_sensor_adapter/node.py']:
        raise ValueError('startup_adapter_hash_mismatch')
    if sensor!=run:
        control_required={'controller-result.json','sensor-artifacts.json','p3-prerequisites.json',
                          'p2-build-receipt.json','p3-build-receipt.json'}
        if not control_required.issubset(seal['artifacts']): raise ValueError('control_finalization_artifact_list_incomplete')
        if strict_json(run/'controller-result.json').get('run_id')!=run.name: raise ValueError('controller_result_run_identity')
        manifest=strict_json(run/'sensor-artifacts.json')
        if manifest.get('run_id')!=run.name or manifest.get('sample_evidence_contract')!=CONTRACT:
            raise ValueError('sensor_manifest_run_identity')
        sealed_sensor={n[len(prefix):]:v for n,v in seal['artifacts'].items() if n.startswith(prefix)}
        if manifest['artifacts']!=sealed_sensor: raise ValueError('sensor_manifest_incomplete')
    if dependent:
        evaluation=strict_json(run/dependent)
        if evaluation.get('run_id')!=run.name: raise ValueError('dependent_evaluator_run_identity')
        if evaluation.get('sample_evidence_contract')!=CONTRACT: raise ValueError('dependent_sample_contract')
    return seal


def predecessor(run):
    run=Path(run)
    seal=strict_json(run/'runtime-finalized.json')
    return dict(run_id=run.name,run=str(run),finalization_sha256=digest(run/'runtime-finalized.json'),
                finalized_unix_ns=seal['finalized_unix_ns'])


def require_evaluation(run, name, frozen):
    result=strict_json(Path(run)/name)
    if result.get('run_id')!=Path(run).name or result.get('sample_evidence_contract')!=CONTRACT:
        raise ValueError('evaluator_run_contract:'+name)
    require_same_inputs(result.get('frozen_inputs'),frozen)
    source={'p2-offline-evaluation.json':'collect_p2_evidence.py',
            'p3-coexistence-evaluation.json':'collect_p3_coexistence.py','p3-evaluation.json':'collect_p3_evidence.py'}[name]
    if result.get('evaluator_sha256')!=frozen['files']['validation/'+source]:
        raise ValueError('evaluator_arithmetic_changed:'+name)
    # Imported arithmetic hashes are named explicitly by the control evaluator;
    # the complete frozen map also covers future arithmetic modules.
    if name=='p2-offline-evaluation.json':
        for field,file in (('reference_evaluator_sha256','reference_evidence.py'),
                           ('yaw_evaluator_sha256','yaw_evidence.py'),('timing_evaluator_sha256','timing_evidence.py')):
            if (field in result or result.get('flight_acceptance')!='not_run') and result.get(field)!=frozen['files']['validation/'+file]:
                raise ValueError('dependent_evaluator_arithmetic:'+field)
    return result


def require_historical_analysis(run, name, analysis_inputs, recorded_inputs):
    """A current analysis of old bytes is separate from the recorded runtime."""
    result=strict_json(Path(run)/name)
    if (result.get('run_id')!=Path(run).name or result.get('evaluation_finalized') is not True
            or result.get('historical_reanalysis') is not True or result.get('qualification_credit') is not False
            or result.get('recording_integrity')!='passed'
            or result.get('sample_evidence_contract')!=CONTRACT
            or result.get('analysis_inputs')!=analysis_inputs or result.get('frozen_inputs')!=recorded_inputs):
        raise ValueError('historical_analysis_identity_or_integrity')
    for field,source in (('evaluator_sha256','collect_p2_evidence.py'),
        ('reference_evaluator_sha256','reference_evidence.py'),('yaw_evaluator_sha256','yaw_evidence.py'),
        ('timing_evaluator_sha256','timing_evidence.py'),('sample_evaluator_sha256','sample_evidence.py')):
        if result.get(field)!=analysis_inputs['files']['validation/'+source]:
            raise ValueError('historical_analysis_arithmetic:'+field)
    return result


def require_readiness(run, frozen):
    seal=require_finalized(run,frozen)
    summary=strict_json(Path(run)/'summary.json')
    control=require_evaluation(run,'p2-offline-evaluation.json',frozen)
    sensor=require_evaluation(run,'p3-coexistence-evaluation.json',frozen)
    if (summary.get('kind')!='observe' or summary.get('status')!='passed'
            or control.get('recording_integrity')!='passed' or control.get('flight_acceptance')!='not_run'
            or sensor.get('status')!='passed'):
        raise ValueError('final_readiness_not_passed')
    return seal


def require_ground_matrix(matrix, frozen, readiness):
    matrix=Path(matrix)
    result=strict_json(matrix/'summary.json')
    require_same_inputs(result.get('frozen_inputs'),frozen)
    if result.get('status')!='passed' or result.get('readiness')!=predecessor(readiness):
        raise ValueError('final_ground_matrix_prerequisite')
    ready_summary=strict_json(Path(readiness)/'summary.json')
    if result['runtime_identity']!=ready_summary['runtime_identity']:
        raise ValueError('matrix_readiness_runtime_identity')
    if result['calibration_id']!=strict_json(Path(readiness)/'sensors/sensor-ready.json')['calibration_id']:
        raise ValueError('matrix_readiness_calibration_identity')
    records=[result['plane2'],*result['trials']]
    if [r['case'] for r in records]!=['plane2','plane4','plane6','oblique','asymmetric','out_of_range','interruption']:
        raise ValueError('final_ground_matrix_cases')
    ids=set()
    for record in records:
        run=matrix.parent/record['run_id']
        if run.name in ids: raise ValueError('reused_ground_run')
        ids.add(run.name)
        for filename,key in (('runtime-finalized.json','finalization_sha256'),('p3-evaluation.json','evaluation_sha256')):
            if digest(run/filename)!=record[key]: raise ValueError('ground_prerequisite_hash_changed')
        require_finalized(run,frozen)
        summary=strict_json(run/'summary.json')
        if summary['readiness']!=result['readiness'] or summary['started_unix_ns']<=result['readiness']['finalized_unix_ns']:
            raise ValueError('ground_readiness_chronology')
        if summary['runtime_identity']!=result['runtime_identity']:
            raise ValueError('ground_runtime_identity_changed')
        if strict_json(run/'sensor-ready.json')['calibration_id']!=result['calibration_id']:
            raise ValueError('ground_calibration_identity_changed')
        evaluation=require_evaluation(run,'p3-evaluation.json',frozen)
        expected='passed_expected_failure_diagnostic' if record['case']=='interruption' else 'passed'
        if evaluation['status']!=expected or record['launcher_exit']!=0 or record['evaluator_exit']!=0:
            raise ValueError('ground_case_not_passed')
    return result
