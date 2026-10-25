"""Finite validation campaign: paired ground reads, readiness, matrix, smoke, 3 flights.

All original launcher, evaluator, sensor and flight deadlines apply. Every
attempt is retained. A failed prerequisite stops the campaign before its next step.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

SIM=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM/'validation'))
from p1_contract import require_gates
from p3_provenance import atomic_json,digest,frozen_inputs,strict_json,require_finalized,require_readiness
from mc_trace_runtime import CONTRACT,selected_build
from offline_jobs import execute_offline,budget,require_current_result,require_causal_control


def command(command,log,deadline,env):
    started=time.monotonic_ns()
    with log.open('x') as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        try: code=process.wait(timeout=deadline)
        finally:
            if process.poll() is None:
                import signal
                os.killpg(process.pid,signal.SIGTERM)
                try: process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=10)
    elapsed=(time.monotonic_ns()-started)/1e9
    if code: raise ValueError(f'Command exited {code}: {log}')
    return elapsed


def extract(log,prefix):
    match=re.search('^'+re.escape(prefix)+r'(.+)$',log.read_text(),re.M)
    if not match: raise ValueError('Run identity absent: '+str(log))
    return Path(match.group(1))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run',action='store_true')
    p.add_argument('--benchmark-run',type=Path); p.add_argument('--verification',type=Path)
    args=p.parse_args()
    if not args.run: print('No runtime launched. Explicit --run --benchmark-run --verification required.'); return 0
    require_gates(os.environ,True)
    if not args.benchmark_run or not args.verification: raise ValueError('Native benchmark and ordinary verification required')
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    receipt=selected_build(root,SIM)
    if not receipt: raise ValueError('Explicit instrumented build selection required')
    benchmark=strict_json(args.benchmark_run/'summary.json')
    frozen=frozen_inputs(SIM); verification=strict_json(args.verification)
    if (benchmark.get('status')!='passed' or benchmark.get('declared_pairs')!=3 or len(benchmark.get('pairs',[]))!=3
            or benchmark.get('build')!=receipt or benchmark.get('benchmark_source_sha256')!=digest(SIM/'validation/mc_trace_benchmark.cpp')):
        raise ValueError('Matching complete native benchmark required')
    if verification.get('status')!='passed' or verification.get('frozen_inputs')!=frozen:
        raise ValueError('Final ordinary verification required')
    run=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-mc-trace-campaign-'+uuid.uuid4().hex[:8]); run.mkdir()
    result=dict(contract='px4-mc-campaign-v1',run_id=run.name,status='incomplete',stage='ground_overhead',
        frozen_inputs=frozen,build=receipt,benchmark_run=str(args.benchmark_run),benchmark_sha256=digest(args.benchmark_run/'summary.json'),
        verification=str(args.verification),verification_sha256=digest(args.verification),attempts=[],
        smoke_attempts_allowed=1,qualification_flights=3,qualification_passed=0,stop_first_failure=True,
        declared_ground_pairs=1,ground_control_commands=0)
    env=dict(os.environ); limits=budget(SIM)
    def save(): atomic_json(run/'summary.json',result)
    def invoke(label,args,deadline,prefix=None,environment=None):
        if frozen_inputs(SIM)!=frozen: raise ValueError('Campaign inputs changed')
        log=run/(label+'.log'); entry=dict(label=label,command=args,log=str(log),started_unix_ns=time.time_ns())
        result['attempts'].append(entry); save()
        try: entry['wall_s']=command(args,log,deadline,environment or env); entry['status']='passed'
        except Exception as exc: entry.update(status='failed',error=str(exc)); save(); raise
        if prefix: entry['run']=str(extract(log,prefix))
        save(); print(label+' passed'+(': '+entry['run'] if prefix else ''),flush=True)
        return Path(entry['run']) if prefix else entry
    def evaluate(label,trial):
        require_finalized(trial,frozen)
        invoke(label+'-control',['bash',str(SIM/'scripts/verify_p2_evidence.sh'),str(trial)],limits['control_seconds'])
        control=require_current_result(trial,'p2-offline-evaluation.json',frozen)
        invoke(label+'-sensor',['bash',str(SIM/'scripts/verify_p3_coexistence.sh'),str(trial)],limits['sensor_seconds'])
        sensor=require_current_result(trial,'p3-coexistence-evaluation.json',frozen)
        if control['recording_integrity']!='passed' or sensor['status']!='passed': raise ValueError(label+' independent gate failed')
        return control,sensor
    save(); print('MC trace campaign: '+str(run),flush=True)
    try:
        baseline_env=dict(env); baseline_env.pop('GWM_PX4_MC_TRACE',None)
        paired=[]
        for label,e in (('baseline',baseline_env),('instrumented',env)):
            trial=invoke('overhead-'+label,['bash',str(SIM/'scripts/run_p3_coexistence.sh'),'--run','--observe'],450,
                         'P2 evidence: ',e)
            control,sensor=evaluate('overhead-'+label,trial)
            summary=strict_json(trial/'summary.json')
            before=summary['px4_proc_stat_start'].rsplit(')',1)[1].split()
            after=summary['px4_proc_stat_end'].rsplit(')',1)[1].split()
            process_cpu_s=sum(int(after[i])-int(before[i]) for i in (11,12))/os.sysconf('SC_CLK_TCK')
            peak=re.search(r'^VmHWM:\s+(\d+) kB$',summary['px4_memory_before_stop'],re.M)
            if not peak: raise ValueError('Missing actual PX4 peak resident memory')
            expected_binary=receipt['baseline_binary_sha256'] if label=='baseline' else receipt['binary_sha256']
            if summary['px4_executable_sha256']!=expected_binary: raise ValueError('Measured binary identity mismatch')
            paired.append(dict(label=label,run=str(trial),summary_sha256=digest(trial/'summary.json'),
                finalization_sha256=digest(trial/'runtime-finalized.json'),
                control_evaluation_sha256=digest(trial/'p2-offline-evaluation.json'),
                sensor_evaluation_sha256=digest(trial/'p3-coexistence-evaluation.json'),
                control_timing=control.get('timing'),trace=control.get('mc_trace_acquisition'),
                px4_process_cpu_s=process_cpu_s,px4_peak_resident_bytes=int(peak.group(1))*1024,
                config_sha256=summary['identity']['config_sha256'],
                px4_proc_stat_start=summary.get('px4_proc_stat_start'),px4_proc_stat_end=summary.get('px4_proc_stat_end'),
                px4_memory_before_stop=summary.get('px4_memory_before_stop'),wall_duration_s=summary['wall_duration_s']))
        if paired[0]['config_sha256']!=paired[1]['config_sha256']:
            raise ValueError('Ground comparison changed controller configuration')
        overhead=dict(contract='px4-mc-overhead-v1',status='passed',frozen_inputs=frozen,build=receipt,
            benchmark_run=str(args.benchmark_run),benchmark_sha256=digest(args.benchmark_run/'summary.json'),
            ground_pair=paired,completed_unix_ns=time.time_ns(),qualification_credit=False,
            process_cpu_difference_s=paired[1]['px4_process_cpu_s']-paired[0]['px4_process_cpu_s'],
            peak_resident_difference_bytes=paired[1]['px4_peak_resident_bytes']-paired[0]['px4_peak_resident_bytes'],
            measurement_limit='Native benchmark isolates identical-input added cost; ground process CPU also includes startup and scheduling variation.')
        atomic_json(run/'overhead.json',overhead,exclusive=True)
        atomic_json(root/'state/mc-trace-overhead.json',dict(path=str(run/'overhead.json'),sha256=digest(run/'overhead.json')))
        result['stage']='final_readiness'; save()
        readiness=invoke('final-readiness',['bash',str(SIM/'scripts/run_p3_coexistence.sh'),'--run','--observe'],450,'P2 evidence: ')
        evaluate('final-readiness',readiness); require_readiness(readiness,frozen)
        result['readiness']=str(readiness); result['stage']='ground_matrix'; save()
        plane=invoke('plane2',['bash',str(SIM/'scripts/run_p3_sensing.sh'),'--run','--case','plane2','--readiness-run',str(readiness)],300,'P3 evidence: ')
        invoke('plane2-evaluation',['/usr/bin/python3',str(SIM/'validation/collect_p3_evidence.py'),str(plane)],120)
        matrix=invoke('ground-matrix',['/usr/bin/python3',str(SIM/'scripts/run_p3_ground_matrix.py'),'--run-matrix','--plane2-run',str(plane)],2520,'P3 ground matrix: ')
        result['ground_matrix']=str(matrix); result['stage']='smoke'; save()
        smoke=invoke('smoke',['bash',str(SIM/'scripts/run_p3_coexistence.sh'),'--run','--allow-simulated-flight','--ground-matrix',str(matrix)],600,'P2 evidence: ')
        result['smoke']=str(smoke); save()
        control,_=evaluate('smoke',smoke)
        require_causal_control(control)
        if control['flight_acceptance']!='passed': raise ValueError('Smoke acceptance did not pass')
        result['stage']='qualification'; save()
        qualification=invoke('qualification',['/usr/bin/python3',str(SIM/'scripts/run_p3_qualification.py'),
            '--run-qualification','--allow-simulated-flight','--smoke-run',str(smoke)],2400,'P3 qualification: ')
        completed=strict_json(qualification/'summary.json')
        result.update(qualification=str(qualification),qualification_passed=completed['passed'])
        if completed['status']!='passed' or completed['passed']!=3: raise ValueError('Qualification not complete')
        result.update(status='passed',stage='complete',completed_unix_ns=time.time_ns())
    except Exception as exc:
        result.update(status='incomplete',error=str(exc))
    finally: save()
    return 0 if result['status']=='passed' else 1


if __name__=='__main__': raise SystemExit(main())
