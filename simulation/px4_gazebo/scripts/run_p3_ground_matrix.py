"""One fixed final-input ground sequence after current sealed readiness."""
import argparse
import os
from pathlib import Path
import re
import sys
import time
import uuid
from p1_contract import require_gates
from run_p2_repeated import execute
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation'))
from p3_provenance import (CONTRACT, strict_json, frozen_inputs, require_finalized,
    require_readiness, require_evaluation, require_same_inputs, predecessor, digest, atomic_json)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run-matrix',action='store_true'); p.add_argument('--plane2-run',type=Path)
    args=p.parse_args()
    if not args.run_matrix: print('No runtime launched. Explicit --run-matrix --plane2-run required.'); return 0
    require_gates(os.environ)
    if not args.plane2_run: raise ValueError('Final plane2 run required')
    sim=Path(__file__).resolve().parents[1]; root=args.plane2_run.parent.parent
    frozen=frozen_inputs(sim)
    require_finalized(args.plane2_run,frozen)
    initial=require_evaluation(args.plane2_run,'p3-evaluation.json',frozen)
    original=strict_json(args.plane2_run/'summary.json')
    readiness=Path(original['readiness']['run'])
    require_readiness(readiness,frozen)
    if initial['status']!='passed' or original['case']!='plane2' or original['readiness']!=predecessor(readiness):
        raise ValueError('Passed final-input plane2 required')
    run=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-p3-ground-matrix-'+uuid.uuid4().hex[:8])
    run.mkdir()
    cases=['plane4','plane6','oblique','asymmetric','out_of_range','interruption']
    def entry(case,trial,launcher,evaluator):
        return dict(case=case,run_id=trial.name,launcher_exit=launcher,evaluator_exit=evaluator,
            finalization_sha256=digest(trial/'runtime-finalized.json'),evaluation_sha256=digest(trial/'p3-evaluation.json'))
    result=dict(schema_version=2,run_id=run.name,sample_evidence_contract=CONTRACT,status='incomplete',cases=cases,
        plane2_run=str(args.plane2_run),plane2=entry('plane2',args.plane2_run,0,0),trials=[],frozen_inputs=frozen,
        readiness=predecessor(readiness),started_unix_ns=time.time_ns(),failure=None)
    result['runtime_identity']=original['runtime_identity']
    result['calibration_id']=strict_json(readiness/'sensors/sensor-ready.json')['calibration_id']
    def save(): atomic_json(run/'summary.json',result)
    save(); print('P3 ground matrix: '+str(run),flush=True)
    try:
        for case in cases:
            require_same_inputs(frozen_inputs(sim),frozen)
            log=run/(case+'.log')
            code=execute(['bash',str(sim/'scripts/run_p3_sensing.sh'),'--run','--case',case,
                '--readiness-run',str(readiness)],log)
            match=re.search(r'^P3 evidence: (.+)$',log.read_text(),re.MULTILINE)
            if not match: raise ValueError('Missing ground attempt identity')
            trial=Path(match.group(1))
            record=dict(case=case,run_id=trial.name,launcher_exit=code,evaluator_exit=None)
            result['trials'].append(record); save()
            if code: raise ValueError('Ground attempt failed:'+trial.name)
            require_finalized(trial,frozen)
            evaluator=execute(['/usr/bin/python3',str(sim/'validation/collect_p3_evidence.py'),str(trial)],run/(case+'-evaluation.log'),120)
            record.update(entry(case,trial,code,evaluator)); save()
            evaluation=require_evaluation(trial,'p3-evaluation.json',frozen)
            expected='passed_expected_failure_diagnostic' if case=='interruption' else 'passed'
            if evaluator or evaluation['status']!=expected: raise ValueError('Ground evaluation failed:'+trial.name)
            require_same_inputs(frozen_inputs(sim),frozen)
            print(case+' passed: '+trial.name,flush=True)
        result['status']='passed'
        result['finalized_unix_ns']=time.time_ns()
    except (Exception,KeyboardInterrupt) as exc:
        result.update(status='incomplete',failure=str(exc) or 'interrupted')
    finally: save()
    print('Ground matrix '+result['status'],flush=True)
    return 0 if result['status']=='passed' else 1


if __name__=='__main__': raise SystemExit(main())
