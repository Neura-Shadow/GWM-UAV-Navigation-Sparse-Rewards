"""Fixed historical workflow measurements through production wrappers, no runtime.

Each repetition starts new processes. OS caches are neither dropped nor claimed
cold. Sensor invocation after an unknown control result is diagnostic-only here;
the production qualification runner forbids it. All outputs are exclusive.
"""
import argparse
from pathlib import Path
import sys
import time

from offline_jobs import budget, execute_offline
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation'))
from p3_provenance import atomic_json, strict_json, frozen_inputs, digest

RUNS=('20260912T113430Z-p3-flight-0dc41a18','20260912T141235Z-p3-flight-778e3f60')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs-root',type=Path,required=True)
    p.add_argument('--output-directory',type=Path,required=True)
    p.add_argument('--label',required=True)
    args=p.parse_args()
    if not args.label.replace('-','').isalnum(): raise ValueError('Alphanumeric measurement label required')
    args.output_directory.mkdir(exist_ok=False)
    sim=Path(__file__).resolve().parents[1]; frozen=frozen_inputs(sim); limits=budget(sim)
    results=[]
    for repetition in (1,2):
        for run_id in RUNS:
            run=args.runs_root/run_id
            summary=strict_json(run/'summary.json')
            control=f'{args.label}-r{repetition}-control.json'
            sensor=f'{args.label}-r{repetition}-sensor.json'
            for stage,name in (('control',control),('sensor',sensor)):
                if frozen_inputs(sim)!=frozen: raise ValueError('Measurement inputs changed')
                key=f'{run_id}-r{repetition}-{stage}'
                timing=args.output_directory/(key+'.time')
                command=['bash',str(sim/'scripts'/('verify_p2_evidence.sh' if stage=='control' else 'verify_p3_coexistence.sh')),
                         str(run),'--output-name',name,'--historical-analysis']
                if stage=='control' and run_id==RUNS[0]: command+=['--sample-contract','p3-sample-evidence-v2']
                if stage=='sensor': command+=['--control-name',control]
                start=time.perf_counter()
                code=execute_offline(['/usr/bin/time','-o',str(timing),'-f','%e %U %S %M',*command],
                    args.output_directory/(key+'.log'),limits[stage+'_seconds'])
                elapsed=time.perf_counter()-start
                wall,user,system,rss=map(float,timing.read_text().splitlines()[-1].split())
                result=strict_json(run/name)
                if (result.get('evaluation_finalized') is not True or result.get('run_id')!=run_id
                        or result.get('analysis_inputs')!=frozen): raise ValueError('Invalid measurement output')
                if stage=='control' and result.get('recording_integrity')!='passed': raise ValueError('Control analysis incomplete')
                if stage=='sensor' and result.get('failures')!=['control_acceptance']:
                    raise ValueError('Unexpected sensor diagnostic result')
                item=dict(run_id=run_id,stage=stage,repetition=repetition,command=command,
                    wrapper_wall_s=wall,supervisor_wall_s=elapsed,cpu_s=user+system,peak_rss_kib=int(rss),
                    exit_code=code,result=str(run/name),result_sha256=digest(run/name),
                    cache='uncontrolled existing cache' if repetition==1 else 'warm filesystem after first repetition; fresh process',
                    budget_seconds=limits[stage+'_seconds'],runtime_credit=False,
                    recording_seal='runtime-finalized.json' if (run/'runtime-finalized.json').exists() else 'legacy artifact manifests only')
                results.append(item)
                atomic_json(args.output_directory/'measurements.json',dict(contract='p3-offline-measurement-v1',
                    analysis_inputs=frozen,budget=limits,measurements=results,complete=len(results)==8))
                print(item,flush=True)
    return 0


if __name__=='__main__': raise SystemExit(main())
