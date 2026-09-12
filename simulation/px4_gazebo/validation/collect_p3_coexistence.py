"""Independent sensor recording and PX4 state association during P2 activity."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/gwm_px4_control'))
from collect_p3_evidence import digest, acquisition_ns
from collect_p2_evidence import read_bag


def evaluate(run, control_name='p2-offline-evaluation.json', historical_analysis=False):
    out=dict(schema_version=1,status='failed',run_id=run.name,failures=[],evaluator_sha256=digest(Path(__file__)))
    def check(value,reason):
        if not value: out['failures'].append(reason)
    summary=json.loads((run/'summary.json').read_text())
    revised=summary.get('config',{}).get('sample_evidence_contract')=='p3-sample-evidence-v2'
    from p3_provenance import strict_json, frozen_inputs
    frozen=frozen_inputs(Path(__file__).resolve().parents[1])
    if revised:
        from p3_provenance import require_finalized, frozen_inputs, require_evaluation
        require_finalized(run,summary['frozen_inputs'] if historical_analysis else frozen,dependent=control_name)
        if not historical_analysis: require_evaluation(run,control_name,frozen)
        out.update(schema_version=2,sample_evidence_contract='p3-sample-evidence-v2',frozen_inputs=summary['frozen_inputs'])
    control=strict_json(run/control_name)
    if historical_analysis:
        from p3_provenance import require_historical_analysis
        require_historical_analysis(run,control_name,frozen,summary.get('frozen_inputs'))
    elif revised:
        if (control.get('evaluation_finalized') is not True or control.get('analysis_inputs') != frozen
                or control.get('historical_reanalysis') is not False
                or control.get('recording_integrity') != 'passed'
                or control.get('flight_acceptance') != ('passed' if summary['kind']=='flight' else 'not_run')):
            raise ValueError('Control evaluation must finalize successfully before sensor composite')
    out['control_evaluation_sha256']=digest(run/control_name)
    check(summary['status']=='passed' and control['recording_integrity']=='passed','control_recording')
    check(control['flight_acceptance']==('passed' if summary['kind']=='flight' else 'not_run'),'control_acceptance')
    sensor=run/'sensors'
    artifacts=json.loads((run/'sensor-artifacts.json').read_text())
    if revised:
        check(artifacts['run_id']==run.name and artifacts['sample_evidence_contract']=='p3-sample-evidence-v2','sensor_manifest_run_identity')
        artifacts=artifacts['artifacts']
    for name,item in artifacts.items(): check(digest(sensor/name)==item['sha256'],'sensor_artifact:'+name)
    result=json.loads((sensor/'sensor-result.json').read_text())
    rows=[json.loads(l) for l in (sensor/'depth-index.jsonl').read_text().splitlines()]
    events=[json.loads(l) for l in (sensor/'sensor-events.jsonl').read_text().splitlines()]
    source=[json.loads(l) for l in (sensor/'gazebo-source-headers.jsonl').read_text().splitlines()]
    streams,_,control_events=read_bag(run)
    positions=streams['vehicle_local_position']
    begin,end=positions[0]['timestamp']/1e6,positions[-1]['timestamp']/1e6
    begin_ns,end_ns=positions[0]['timestamp']*1000,positions[-1]['timestamp']*1000
    check(end>begin,'empty_control_window')
    selected=[r for r in rows if begin_ns<=acquisition_ns(r,revised)<=end_ns]
    observations=[e for e in events if e['kind']=='observation' and begin_ns<=acquisition_ns(e['image'],revised)<=end_ns]
    source=[s for s in source if begin_ns<=acquisition_ns(s,revised)<=end_ns]
    health=[h for h in events if h['kind']=='health' and begin<=h['sim_s']<=end]
    check(result['status']=='complete' and result['recorder']['overflow']==0,'recorder_failed')
    check(result['received']==result['recorder']['written']==len(rows),'raw_coverage')
    offset=0
    with (sensor/'depth.bin').open('rb') as raw:
        import hashlib
        for index,row in enumerate(rows):
            check(row['id']==index+1 and row['offset']==offset,'index_continuity')
            data=raw.read(row['bytes']); offset+=len(data)
            check(hashlib.sha256(data).hexdigest()==row['sha256'],'frame_integrity')
        check(raw.read(1)==b'','trailing_raw_data')
    check(len(selected)>1 and len(observations)>1 and len(source)>1,'missing_sensor_window')
    if len(selected)<2 or len(observations)<2 or len(source)<2: return out
    stamps_ns=np.array([acquisition_ns(r,revised) for r in selected],dtype=np.int64)
    source_stamps={acquisition_ns(s,revised) for s in source}
    missing=len(source_stamps-set(stamps_ns))
    check(missing==0,'source_to_recording_loss')
    sequence=[int(s['header_data']['seq'][0]) for s in source]
    check(all(b-a==1 for a,b in zip(sequence,sequence[1:])),'source_probe_incomplete')
    stamps=stamps_ns/1e9; gaps=np.diff(stamps_ns)/1e9
    rate=(len(stamps)-1)*1e9/int(stamps_ns[-1]-stamps_ns[0])
    check(np.all(gaps>0) and gaps.max()<=.12 and rate>=25,'source_timing')
    check(stamps[0]-begin<=.12 and end-stamps[-1]<=.12,'sensor_window_coverage')
    age=max(o['processing_return_sim_s']-o['image']['acquisition_sim_s'] for o in observations)
    check(age<=.25,'processed_source_age')
    obs_times=[acquisition_ns(o['image'],revised) for o in observations]
    obs_rate=(len(obs_times)-1)*1e9/(obs_times[-1]-obs_times[0])
    check(obs_rate>=8,'processed_rate')
    check(health and all(h['status']=='fresh' for h in health),'sensor_health')
    errors=[e for e in events if e['kind']=='error' and begin<=e['sim_s']<=end]
    check(not errors,'sensor_errors_during_control')
    # Match against independently recorded PX4 messages, not latest-at-processing.
    from collections import defaultdict
    by_stamp=defaultdict(list)
    for position in positions: by_stamp[position['timestamp']].append(position)
    if revised:
        from sample_evidence import validate_stream, payload
        from gwm_px4_control.sample_identity import source_record
        classified=validate_stream(positions,'vehicle_local_position',run.name)
        out['position_record_classification']=classified['statistics']
    used=set(); reused=0; unmatched=0; ambiguous=0
    matched=0; mismatch=0.
    for observation in observations:
        associated=observation['state_association']
        check(associated is not None,'missing_state_association')
        if associated is None: continue
        state=associated['state']; stamp=state['timestamp_us']
        mismatch=max(mismatch,abs(stamp*1000-acquisition_ns(observation['image'],revised))/1e9)
        check(mismatch<=.05,'state_time_mismatch')
        if stamp in by_stamp:
            candidates=by_stamp[stamp]
            if revised:
                ref=state['source_reference']
                check(state['run_id']==run.name and ref['run_id']==run.name,'state_run_identity')
                candidates=[p for p in candidates if p['timestamp_sample']==state['timestamp_sample_us']]
                valid=[p for p in candidates if source_record('vehicle_local_position',payload(p),run.name)['source_id']==ref['source_id']
                       and source_record('vehicle_local_position',payload(p),run.name)['payload_sha256']==ref['payload_sha256']]
                check(bool(valid) and len(valid)==len(candidates),'state_exact_payload_identity')
                if not valid or len(valid)!=len(candidates):
                    ambiguous+=bool(candidates); unmatched+=not bool(candidates); continue
                identity=ref['source_id']
                reused+=identity in used; used.add(identity)
                # Every candidate is now an exact equivalent payload; record all.
                p=valid[0]
            else:
                check(len(candidates)==1,'ambiguous_legacy_state_timestamp')
                if len(candidates)!=1: ambiguous+=1; continue
                p=candidates[0]
            check(all(abs(p[k]-state[k])<1e-9 for k in ('x','y','z','heading')),'state_record_mismatch')
            check(associated['epoch']==[p[k] for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter','vxy_reset_counter','vz_reset_counter')],'state_epoch_mismatch')
            matched+=1
        else:
            unmatched+=1
            # One selected state may bracket the controller recorder's endpoints.
            check(stamp<positions[0]['timestamp'] or stamp>positions[-1]['timestamp'],'state_not_in_independent_recording')
    check(matched>=len(observations)-2,'state_recording_coverage')
    out.update(window_sim_s=[begin,end],raw_frames=len(rows),window_frames=len(selected),raw_bytes=offset,
        source_missing=missing,delivered_hz=rate,source_gap_max_sim_s=float(gaps.max()),processed_hz=obs_rate,
        observations=len(observations),observation_age_max_sim_s=age,state_mismatch_max_s=mismatch,
        matched_px4_observations=matched,recorder=result,control_acceptance=control['flight_acceptance'],
        state_correlation=dict(matched=matched,unmatched=unmatched,ambiguous=ambiguous,reused=reused,
            distinct_matched_sources=len(used),loss_or_full_parity_claim=False),
        control_timing=control.get('timing'),sensor_errors=errors,
        status='passed' if not out['failures'] else 'failed')
    return out


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('run',type=Path)
    p.add_argument('--output-name',default='p3-coexistence-evaluation.json')
    p.add_argument('--control-name',default='p2-offline-evaluation.json')
    p.add_argument('--historical-analysis',action='store_true'); args=p.parse_args()
    for name in (args.output_name,args.control_name):
        if Path(name).name!=name or not name.endswith('.json'): raise ValueError('JSON basename required')
    if args.historical_analysis and (args.output_name=='p3-coexistence-evaluation.json' or args.control_name=='p2-offline-evaluation.json'):
        raise ValueError('Historical analysis requires distinct report names')
    if not args.historical_analysis and args.control_name!='p2-offline-evaluation.json':
        raise ValueError('Runtime composite requires canonical control evaluation')
    if (args.run/args.output_name).exists(): raise ValueError('Retain the existing evaluation')
    from p3_provenance import atomic_json,frozen_inputs
    analysis_inputs=frozen_inputs(Path(__file__).resolve().parents[1])
    try: result=evaluate(args.run,args.control_name,args.historical_analysis)
    except Exception as exc: result=dict(schema_version=2,status='failed',run_id=args.run.name,failure=str(exc),
        sample_evidence_contract='p3-sample-evidence-v2',evaluator_sha256=digest(Path(__file__)))
    if frozen_inputs(Path(__file__).resolve().parents[1])!=analysis_inputs:
        result.update(status='failed',failure='analysis_inputs_changed_during_evaluation')
    result.update(analysis_inputs=analysis_inputs,
        historical_reanalysis=args.historical_analysis,evaluation_finalized=True,
        qualification_credit=False if args.historical_analysis else None)
    atomic_json(args.run/args.output_name,result,exclusive=True)
    print(json.dumps({k:v for k,v in result.items() if k not in ('control_timing','recorder')},indent=2,allow_nan=False))
    raise SystemExit(0 if result['status']=='passed' else 1)
