"""Explicit instrumented binary selection and immutable run provenance."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from p3_provenance import atomic_json, digest, strict_json, tree_manifest

CONTRACT = 'px4-mc-causal-trace-v1'


def selected_build(root, sim):
    """No environment selection means the unchanged baseline; never auto-select."""
    choice = os.environ.get('GWM_PX4_MC_TRACE')
    if choice is None: return None
    if choice != CONTRACT: raise ValueError('Unknown PX4 trace selection')
    root, sim = Path(root), Path(sim)
    receipt = strict_json(root/'state/px4-mc-trace-built.json')
    if receipt.get('contract') != CONTRACT or receipt.get('status') != 'built':
        raise ValueError('Instrumented build not complete')
    files = tree_manifest(sim/'validation/mc_trace')
    files['build_mc_trace.py'] = digest(sim/'scripts/build_mc_trace.py')
    identity = hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
    if receipt['overlay_files'] != files or receipt['overlay_id'] != identity:
        raise ValueError('Trace source differs from built overlay')
    source, binary = Path(receipt['source']), Path(receipt['binary'])
    if source.parent != root/'mc_trace_builds'/identity or binary != Path(receipt['build'])/'bin/px4':
        raise ValueError('Trace build path mismatch')
    if digest(binary)!=receipt['binary_sha256']: raise ValueError('Trace binary changed')
    for name,change in receipt['source_changes'].items():
        if digest(source/name)!=change['after']: raise ValueError('Instrumented source changed: '+name)
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    changed=set(subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=source,text=True).splitlines())
    expected={name for name,change in receipt['source_changes'].items() if change['before'] is not None}
    if head!=receipt['source_commit'] or changed!=expected:
        raise ValueError('Unreviewed instrumented source changes')
    for path in (source,root/'upstream/PX4-Autopilot'):
        modules=subprocess.check_output(['git','submodule','status','--recursive'],cwd=path,text=True)
        if any(line.startswith(('+','-','U')) for line in modules.splitlines()):
            raise ValueError('Instrumented dependency checkout mismatch')
    baseline = root/'upstream/PX4-Autopilot/build/px4_sitl_default_linux/bin/px4'
    if digest(baseline)!=receipt['baseline_binary_sha256']: raise ValueError('Preserved baseline changed')
    for name,value in receipt['generated_headers'].items():
        if digest(Path(receipt['build'])/'uORB/topics'/name)!=value: raise ValueError('Native ABI changed')
    return receipt


def prepare(run, receipt):
    run = Path(run)
    atomic_json(run/'mc-trace-build.json', receipt, exclusive=True)
    folder = run/'mc-trace-abi'; folder.mkdir()
    for name in receipt['generated_headers']:
        shutil.copyfile(Path(receipt['build'])/'uORB/topics'/name, folder/name)
    return {'GWM_MC_TRACE_FILE':str(run/'mc-trace.bin'), 'GWM_MC_TRACE_RUN_ID':run.name}


def finalize(run, summary):
    if 'mc_trace' not in summary['identity']: return
    from mc_trace_evidence import acquisition
    report = acquisition(Path(run), summary)
    atomic_json(Path(run)/'mc-trace-acquisition.json', report, exclusive=True)
    if report['status'] != 'verified': raise ValueError('Trace acquisition incomplete')


def require_overhead(root, frozen, receipt, readiness=None):
    pointer=strict_json(Path(root)/'state/mc-trace-overhead.json')
    path=Path(pointer['path'])
    if digest(path)!=pointer['sha256']: raise ValueError('Overhead receipt changed')
    report=strict_json(path)
    if (report.get('contract')!='px4-mc-overhead-v1' or report.get('status')!='passed'
            or report.get('frozen_inputs')!=frozen or report.get('build')!=receipt):
        raise ValueError('Current measured overhead evidence required')
    benchmark=Path(report['benchmark_run'])/'summary.json'
    if digest(benchmark)!=report['benchmark_sha256'] or strict_json(benchmark).get('status')!='passed':
        raise ValueError('Native overhead benchmark mismatch')
    from p3_provenance import require_finalized,require_evaluation
    if [row['label'] for row in report.get('ground_pair',[])]!=['baseline','instrumented']:
        raise ValueError('Complete baseline/instrumented ground pair required')
    for row in report['ground_pair']:
        run=Path(row['run']); require_finalized(run,frozen)
        for file,key in (('summary.json','summary_sha256'),('runtime-finalized.json','finalization_sha256'),
            ('p2-offline-evaluation.json','control_evaluation_sha256'),('p3-coexistence-evaluation.json','sensor_evaluation_sha256')):
            if digest(run/file)!=row[key]: raise ValueError('Overhead ground evidence changed')
        control=require_evaluation(run,'p2-offline-evaluation.json',frozen)
        sensor=require_evaluation(run,'p3-coexistence-evaluation.json',frozen)
        if control.get('recording_integrity')!='passed' or sensor.get('status')!='passed':
            raise ValueError('Overhead ground gate failed')
        if row['label']=='instrumented' and control.get('mc_trace_acquisition',{}).get('status')!='verified':
            raise ValueError('Overhead causal acquisition not verified')
    if readiness and strict_json(Path(readiness)/'summary.json')['started_unix_ns'] <= report['completed_unix_ns']:
        raise ValueError('Final readiness must follow measured overhead')
    return dict(path=str(path),sha256=digest(path))
