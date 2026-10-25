"""Three declared paired native benchmarks; no retries and no runtime launches."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid

import numpy as np

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(SIM/'validation'))
from p3_provenance import atomic_json, digest, strict_json
from mc_trace_runtime import selected_build, prepare
from mc_trace_evidence import acquisition
from mc_trace_schema import schema


def compile_benchmark(source, build, output, instrumented, trace_source):
    commands = strict_json(build/'compile_commands.json')
    entry = next(c for c in commands if c['file'].endswith('/PositionControl/PositionControl.cpp'))
    words = shlex.split(entry['command']); flags = []; skip = False
    for word in words[1:]:
        if skip: skip=False; continue
        if word == '-o': skip=True; continue
        if word == '-c' or word == entry['file']: continue
        flags.append(word)
    archive = build/'src/modules/mc_pos_control/PositionControl/libPositionControl.a'
    objects = [str(archive)]
    if instrumented:
        objects += [str(build/'src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/GwmMcTrace.cpp.o')]
    command = [words[0],*flags,'-I'+str(trace_source/'src/modules/mc_pos_control'),
        '-DGWM_BENCH_TRACE='+str(int(instrumented)),str(SIM/'validation/mc_trace_benchmark.cpp'),*objects,'-o',str(output)]
    subprocess.run(command,cwd=build,check=True,capture_output=True,text=True)
    return dict(command=command,binary_sha256=digest(output),library_sha256=digest(archive),
        controller_source_sha256=digest(source/'src/modules/mc_pos_control/PositionControl/PositionControl.cpp'))


def main():
    p=argparse.ArgumentParser(); p.add_argument('--measure',action='store_true'); args=p.parse_args()
    if not args.measure: print('No benchmark. Explicit --measure required.'); return
    root=Path(os.environ.get('GWM_SIM_ROOT',str(Path.home()/'uav_autonomy')))
    receipt=selected_build(root,SIM)
    if not receipt: raise ValueError('Explicit instrumented binary selection required')
    run=root/'runs'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'-mc-trace-benchmark-'+uuid.uuid4().hex[:8])
    run.mkdir()
    result=dict(contract='px4-mc-overhead-v1',run_id=run.name,status='incomplete',pairs=[],
        declared_pairs=3,iterations_per_binary=10000,build=receipt,benchmark_source_sha256=digest(SIM/'validation/mc_trace_benchmark.cpp'),
        qualification_credit=False,simulation_launches=0,flight_commands=0)
    atomic_json(run/'summary.json',result)
    baseline=root/'upstream/PX4-Autopilot'; source=Path(receipt['source'])
    try:
        result['executables']={label:compile_benchmark(s,b,run/label,instrumented,source)
            for label,s,b,instrumented in [('baseline',baseline,baseline/'build/px4_sitl_default_linux',False),
                ('instrumented',source,Path(receipt['build']),True)]}
        for pair in range(1,4):
            entry=dict(index=pair); result['pairs'].append(entry)
            for label in ('baseline','instrumented'):
                folder=run/f'{pair}-{label}'; folder.mkdir()
                env=dict(os.environ)
                if label=='instrumented': env.update(prepare(folder,receipt))
                completed=subprocess.run([str(run/label),str(folder/'outputs.bin'),str(folder/'timings.bin')],
                    env=env,capture_output=True,text=True,check=True,timeout=30)
                native=json.loads(completed.stdout)
                headers={name:(Path(receipt['build'])/'uORB/topics'/name).read_text() for name in receipt['generated_headers']}
                expected=schema(headers)
                if any(native[k]!=expected[k] for k in ('header_size','record_size','offsets')):
                    raise ValueError('Python/native ABI mismatch')
                timings=np.fromfile(folder/'timings.bin',dtype='<u8').reshape((-1,2))/1e9
                if len(timings)!=10000: raise ValueError('Incomplete benchmark timings')
                metrics={clock:dict(max=float(timings[:,i].max()),p99=float(np.percentile(timings[:,i],99)),
                    mean=float(timings[:,i].mean()),total=float(timings[:,i].sum())) for i,clock in enumerate(('wall_s','thread_cpu_s'))}
                entry[label]=dict(metrics=metrics,output_sha256=digest(folder/'outputs.bin'),schema=native)
                if label=='instrumented':
                    summary=dict(run_id=folder.name,identity=dict(mc_trace=receipt,binary_sha256=receipt['binary_sha256']),
                        config=strict_json(SIM/'configs/p3_control.yaml'))
                    entry[label]['acquisition']=acquisition(folder,summary)
                    if entry[label]['acquisition']['status']!='verified': raise ValueError('Benchmark trace contradicted')
            if entry['baseline']['output_sha256']!=entry['instrumented']['output_sha256']:
                raise ValueError('Baseline/instrumented control output bits differ')
            entry['all_output_bits_equal']=True
            entry['mean_added_wall_s']=entry['instrumented']['metrics']['wall_s']['mean']-entry['baseline']['metrics']['wall_s']['mean']
            entry['mean_added_thread_cpu_s']=entry['instrumented']['metrics']['thread_cpu_s']['mean']-entry['baseline']['metrics']['thread_cpu_s']['mean']
            if entry['instrumented']['metrics']['wall_s']['max']>.05: raise ValueError('Existing 50 ms wall limit exceeded')
            atomic_json(run/'summary.json',result)
        result['status']='passed'
    except Exception as exc:
        result.update(status='failed',error=str(exc))
        if isinstance(exc,subprocess.CalledProcessError): result['compiler_output']=exc.stderr
        raise
    finally:
        atomic_json(run/'summary.json',result)
        print('MC benchmark: '+str(run),flush=True)


if __name__=='__main__': main()
