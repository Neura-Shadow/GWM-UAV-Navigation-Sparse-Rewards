"""Immutable historical P3 adjudication; decoding only, no nodes or replay."""
import argparse
import json
from pathlib import Path
import time

from collect_p2_evidence import digest, verify
from p3_provenance import CONTRACT, atomic_json, frozen_inputs, strict_json


def adjudicate(run, output_name):
    if Path(output_name).name != output_name or not output_name.startswith('p3-r1-') or not output_name.endswith('.json'):
        raise ValueError('Distinct P3-R1 JSON basename required')
    output=run/output_name
    if output.exists(): raise ValueError('Historical analysis already exists')
    preserved=['p2-offline-evaluation.json','p3-coexistence-evaluation.json','summary.json',
               'controller-result.json','p3-smoke-window-diagnostic.json',
               'p3-r1-original-evaluator-reproduction.json','p3-r1-native-record-reconstruction-v1.json']
    originals={name:dict(sha256=digest(run/name),bytes=(run/name).stat().st_size) for name in preserved}
    earlier={path.name:dict(sha256=digest(path),bytes=path.stat().st_size)
             for pattern in ('p3-r1-historical-adjudication-*.json','p3-r1-quaternion-reconstruction-diagnostic-*.json')
             for path in sorted(run.glob(pattern)) if path != output}
    original=strict_json(run/'p2-offline-evaluation.json')
    frozen=frozen_inputs(Path(__file__).resolve().parents[1])
    started=time.time_ns()
    try:
        revised=verify(run,CONTRACT)
    except Exception as exc:
        revised=dict(run_id=run.name,recording_integrity='unproven',flight_acceptance='failed',
                     error=str(exc),exception_type=type(exc).__name__)
    if any(digest(run/name)!=entry['sha256'] for name,entry in originals.items()):
        raise ValueError('Original historical artifact changed')
    if any(digest(run/name)!=entry['sha256'] for name,entry in earlier.items()):
        raise ValueError('Earlier adjudication or diagnostic changed')
    if frozen!=frozen_inputs(Path(__file__).resolve().parents[1]):
        raise ValueError('Evaluator inputs changed during historical adjudication')
    report=dict(schema_version=2,kind='read_only_historical_adjudication',run_id=run.name,
        sample_evidence_contract=CONTRACT,evaluator_sha256=digest(Path(__file__)),frozen_inputs=frozen,
        started_unix_ns=started,completed_unix_ns=time.time_ns(),qualification_credit=False,new_runtime=False,
        original_outcome=original,original_immutable_artifacts=originals,revised_analysis=revised,
        earlier_immutable_analyses=earlier,
        original_evaluator_sha256=original['evaluator_sha256'],
        revised_control_evaluator_sha256=digest(Path(__file__).with_name('collect_p2_evidence.py')),
        provenance_limitations=['Original sensor observations retain historical run_id=sensors; enclosing manifest attribution only.',
            'Original run predates explicit recording-finalization seals and final run-ID build.',
            'Original controller cache hid one equal-publication distinct attitude output; reanalysis reconstructs the actual original selections.',
            'Historical reanalysis cannot qualify the final controller, sensor build or three-flight campaign.'])
    atomic_json(output,report,exclusive=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path)
    p.add_argument('--output-name',default='p3-r1-historical-adjudication-v2.json');args=p.parse_args()
    report=adjudicate(args.run.resolve(),args.output_name)
    revised=report['revised_analysis']
    print(json.dumps(dict(run_id=args.run.name,output=args.output_name,sha256=digest(args.run/args.output_name),
        original=report['original_outcome']['flight_acceptance'],revised=revised.get('flight_acceptance'),
        error=revised.get('error'),yaw=revised.get('yaw_ownership'),qualification_credit=False),indent=2))
    raise SystemExit(0 if revised.get('flight_acceptance')=='passed' else 1)
