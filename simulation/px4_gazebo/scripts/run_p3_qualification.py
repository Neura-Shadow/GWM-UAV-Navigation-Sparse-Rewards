"""Exactly three fresh coexistence flights after one independently passed smoke."""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time
import uuid
from p1_contract import require_gates
from run_p2_repeated import execute, check_smoke, digest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation'))
from p3_provenance import (CONTRACT, strict_json, frozen_inputs, require_finalized,
    require_evaluation, require_ground_matrix, predecessor, atomic_json)


def inputs(sim):
    return frozen_inputs(sim)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run-qualification',action='store_true')
    p.add_argument('--allow-simulated-flight',action='store_true'); p.add_argument('--smoke-run',type=Path)
    args=p.parse_args()
    if not args.run_qualification: print('No runtime launched. Explicit qualification, flight flag and smoke required.'); return 0
    require_gates(os.environ,True)
    if not args.allow_simulated_flight or not args.smoke_run: raise ValueError('Explicit flight and smoke required')
    sim=Path(__file__).resolve().parents[1]; root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    frozen=inputs(sim)
    require_finalized(args.smoke_run,frozen)
    smoke=strict_json(args.smoke_run/'summary.json')
    offline=require_evaluation(args.smoke_run,'p2-offline-evaluation.json',frozen)
    sensor=require_evaluation(args.smoke_run,'p3-coexistence-evaluation.json',frozen)
    prerequisites=strict_json(args.smoke_run/'p3-prerequisites.json')
    matrix=Path(prerequisites['ground_matrix'])
    ground=require_ground_matrix(matrix,frozen,Path(prerequisites['readiness']['run']))
    if (digest(matrix/'summary.json')!=prerequisites['ground_matrix_sha256']
            or smoke['started_unix_ns']<=ground['finalized_unix_ns']):
        raise ValueError('Smoke ground chronology mismatch')
    check_smoke(smoke,offline,digest(sim/'validation/collect_p2_evidence.py'))
    if 'p3' not in smoke['identity'] or sensor['status']!='passed': raise ValueError('Depth smoke required')
    if sensor['evaluator_sha256']!=digest(sim/'validation/collect_p3_coexistence.py'): raise ValueError('Sensor evaluator changed')
    batch=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-p3-qualification-'+uuid.uuid4().hex[:8])
    batch.mkdir()
    report=dict(schema_version=2,run_id=batch.name,sample_evidence_contract=CONTRACT,status='incomplete',required_consecutive=3,passed=0,trials=[],
        smoke_run=str(args.smoke_run),smoke=predecessor(args.smoke_run),ground_matrix=str(matrix),
        frozen_inputs=frozen,failure=None,started_unix_ns=time.time_ns())
    report['runtime_identity']=smoke['runtime_identity']
    report['calibration_id']=ground['calibration_id']
    def save(): atomic_json(batch/'summary.json',report)
    print('P3 qualification: '+str(batch),flush=True); save()
    try:
        used={args.smoke_run.name}
        previous=predecessor(args.smoke_run)
        for index in range(1,4):
            if inputs(sim)!=report['frozen_inputs']: raise ValueError('Qualification input changed')
            log=batch/f'trial-{index}.log'
            code=execute(['bash',str(sim/'scripts/run_p3_coexistence.sh'),'--run','--allow-simulated-flight',
                '--ground-matrix',str(matrix)],log)
            entry=dict(index=index,launcher_exit=code); report['trials'].append(entry)
            match=re.search(r'^P2 evidence: (.+)$',log.read_text(),re.MULTILINE)
            if not match: raise ValueError('No run identity')
            run=Path(match.group(1)); entry['run_id']=run.name; save()
            if run.name in used: raise ValueError('Cross-run qualification reuse')
            used.add(run.name)
            if code: raise ValueError('Coexistence runtime failed:'+run.name)
            require_finalized(run,frozen)
            trial=strict_json(run/'summary.json')
            if trial['started_unix_ns']<=previous['finalized_unix_ns']: raise ValueError('Qualification chronology')
            entry['predecessor']=previous
            if trial['identity']!=smoke['identity']: raise ValueError('Runtime differs from smoke')
            control_code=execute(['bash',str(sim/'scripts/verify_p2_evidence.sh'),str(run)],batch/f'control-{index}.log',90)
            sensor_code=execute(['bash',str(sim/'scripts/verify_p3_coexistence.sh'),str(run)],batch/f'sensor-{index}.log',120)
            entry.update(control_evaluator_exit=control_code,sensor_evaluator_exit=sensor_code)
            if code or control_code or sensor_code: raise ValueError('Coexistence failed: '+run.name)
            control=require_evaluation(run,'p2-offline-evaluation.json',frozen)
            sensor=require_evaluation(run,'p3-coexistence-evaluation.json',frozen)
            check_smoke(trial,control,digest(sim/'validation/collect_p2_evidence.py'))
            if sensor['status']!='passed': raise ValueError('Sensor acceptance failed')
            entry['finalization_sha256']=digest(run/'runtime-finalized.json')
            entry['control_evaluation_sha256']=digest(run/'p2-offline-evaluation.json')
            entry['sensor_evaluation_sha256']=digest(run/'p3-coexistence-evaluation.json')
            previous=predecessor(run)
            if inputs(sim)!=report['frozen_inputs']: raise ValueError('Input changed during qualification')
            report['passed']+=1; save(); print(f"P3 consecutive: {report['passed']}/3 ({run.name})",flush=True)
        report['status']='passed'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='incomplete',failure=str(exc) or 'interrupted')
    finally: save()
    print(json.dumps({k:v for k,v in report.items() if k!='frozen_inputs'},indent=2),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__': raise SystemExit(main())
