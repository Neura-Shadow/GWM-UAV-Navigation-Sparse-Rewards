"""Exactly three fresh coexistence flights after one independently passed smoke."""
import argparse
import json
import os
from pathlib import Path
import re
import time
import uuid
from p1_contract import require_gates
from run_p2_repeated import execute, current_inputs, check_smoke, digest
from p3_build import manifest
from p2_build import package_hash


def inputs(sim):
    files=[*sorted((sim/'configs').glob('p3*')),
           *(sim/'scripts'/n for n in ('p3_coexistence.py','run_p3_coexistence.sh','run_p3_qualification.py')),
           *(sim/'validation'/n for n in ('collect_p3_coexistence.py','p3_source_probe.py'))]
    return dict(p2=current_inputs(sim),sensor_package=package_hash(manifest(sim)),
                files={p.relative_to(sim).as_posix():digest(p) for p in files})


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run-qualification',action='store_true')
    p.add_argument('--allow-simulated-flight',action='store_true'); p.add_argument('--smoke-run',type=Path)
    args=p.parse_args()
    if not args.run_qualification: print('No runtime launched. Explicit qualification, flight flag and smoke required.'); return 0
    require_gates(os.environ,True)
    if not args.allow_simulated_flight or not args.smoke_run: raise ValueError('Explicit flight and smoke required')
    sim=Path(__file__).resolve().parents[1]; root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    smoke=json.loads((args.smoke_run/'summary.json').read_text())
    offline=json.loads((args.smoke_run/'p2-offline-evaluation.json').read_text())
    sensor=json.loads((args.smoke_run/'p3-coexistence-evaluation.json').read_text())
    check_smoke(smoke,offline,digest(sim/'validation/collect_p2_evidence.py'))
    if 'p3' not in smoke['identity'] or sensor['status']!='passed': raise ValueError('Depth smoke required')
    if sensor['evaluator_sha256']!=digest(sim/'validation/collect_p3_coexistence.py'): raise ValueError('Sensor evaluator changed')
    batch=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-p3-qualification-'+uuid.uuid4().hex[:8])
    batch.mkdir()
    report=dict(schema_version=1,status='incomplete',required_consecutive=3,passed=0,trials=[],
        smoke_run=str(args.smoke_run),frozen_inputs=inputs(sim),failure=None)
    def save(): (batch/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('P3 qualification: '+str(batch),flush=True); save()
    try:
        for index in range(1,4):
            if inputs(sim)!=report['frozen_inputs']: raise ValueError('Qualification input changed')
            log=batch/f'trial-{index}.log'
            code=execute(['bash',str(sim/'scripts/run_p3_coexistence.sh'),'--run','--allow-simulated-flight',
                '--ground-matrix',smoke['identity']['p3']['ground_matrix']],log)
            entry=dict(index=index,launcher_exit=code); report['trials'].append(entry)
            match=re.search(r'^P2 evidence: (.+)$',log.read_text(),re.MULTILINE)
            if not match: raise ValueError('No run identity')
            run=Path(match.group(1)); entry['run_id']=run.name; save()
            trial=json.loads((run/'summary.json').read_text())
            if trial['identity']!=smoke['identity']: raise ValueError('Runtime differs from smoke')
            control_code=execute(['bash',str(sim/'scripts/verify_p2_evidence.sh'),str(run)],batch/f'control-{index}.log',90)
            sensor_code=execute(['bash',str(sim/'scripts/verify_p3_coexistence.sh'),str(run)],batch/f'sensor-{index}.log',120)
            entry.update(control_evaluator_exit=control_code,sensor_evaluator_exit=sensor_code)
            if code or control_code or sensor_code: raise ValueError('Coexistence failed: '+run.name)
            if inputs(sim)!=report['frozen_inputs']: raise ValueError('Input changed during qualification')
            report['passed']+=1; save(); print(f"P3 consecutive: {report['passed']}/3 ({run.name})",flush=True)
        report['status']='passed'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='incomplete',failure=str(exc) or 'interrupted')
    finally: save()
    print(json.dumps({k:v for k,v in report.items() if k!='frozen_inputs'},indent=2),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__': raise SystemExit(main())
