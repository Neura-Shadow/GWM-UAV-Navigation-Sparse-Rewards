"""Fixed remaining ground cases, each once; no retries or flight authority."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import uuid
from p1_contract import require_gates
from p2_build import sha


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run-matrix',action='store_true'); p.add_argument('--plane2-run',type=Path)
    args=p.parse_args()
    if not args.run_matrix: print('No runtime launched. Explicit --run-matrix and --plane2-run required.'); return 0
    require_gates(os.environ)
    initial=json.loads((args.plane2_run/'p3-evaluation.json').read_text())
    original=json.loads((args.plane2_run/'summary.json').read_text())
    if initial['status']!='passed' or original['profile']!='p3-x500-depth-native-shm64-v1': raise ValueError('Passed current plane2 required')
    sim=Path(__file__).resolve().parents[1]; root=args.plane2_run.parent.parent
    run=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-p3-ground-matrix-'+uuid.uuid4().hex[:8])
    run.mkdir()
    cases=['plane4','plane6','oblique','asymmetric','out_of_range','interruption']
    files=[p for folder in ('configs','ros2_ws/src/gwm_sensor_adapter') for p in (sim/folder).rglob('*')
           if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc']
    files += [sim/'scripts'/n for n in ('p3_runner.py','run_p3_sensing.sh','common.sh','p3_build.py')]
    files += [sim/'validation'/n for n in ('collect_p3_evidence.py','p3_source_probe.py')]
    identity={p.relative_to(sim).as_posix():sha(p) for p in files}
    result=dict(schema_version=1,status='incomplete',cases=cases,plane2_run=str(args.plane2_run),trials=[],identity=identity)
    def save(): (run/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    save(); print('P3 ground matrix: '+str(run),flush=True)
    for case in cases:
        if any(sha(sim/n)!=h for n,h in identity.items()): raise ValueError('Frozen ground source changed')
        command=['bash',str(sim/'scripts/run_p3_sensing.sh'),'--run','--case',case]
        attempt=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (run/(case+'.log')).write_text(attempt.stdout)
        print(attempt.stdout,flush=True)
        lines=[line for line in attempt.stdout.splitlines() if line.startswith('{"run_id"')]
        if not lines: raise RuntimeError('Missing ground attempt identity')
        outcome=json.loads(lines[-1]); trial=root/'runs'/outcome['run_id']
        evaluation=subprocess.run(['/usr/bin/python3',str(sim/'validation/collect_p3_evidence.py'),str(trial)],
                                  text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (run/(case+'-evaluation.log')).write_text(evaluation.stdout)
        record=dict(case=case,run_id=trial.name,launcher_exit=attempt.returncode,evaluator_exit=evaluation.returncode)
        if (trial/'p3-evaluation.json').exists(): record['evaluation']=json.loads((trial/'p3-evaluation.json').read_text())
        result['trials'].append(record); save()
        print(json.dumps({k:record[k] for k in ('case','run_id','launcher_exit','evaluator_exit')}),flush=True)
    result['status']='passed' if all(t['evaluator_exit']==0 for t in result['trials']) else 'failed'
    save(); print('Ground matrix '+result['status'],flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__': raise SystemExit(main())
