"""Read-only timing reconstruction. Application callbacks are not DDS arrival."""
import argparse
import json
from pathlib import Path


def maximum_gap(values):
    return max((b-a for a,b in zip(values, values[1:])), default=None)


def trace_metrics(traces, result, streams, events, config):
    """Pure reconciliation shared by ground and full-flight offline evaluation."""
    from collections import defaultdict
    from collect_p2_evidence import require
    receipt = result['trace']
    require(receipt['complete'] and receipt['overflow'] == 0, 'Incomplete timing trace')
    require(len(traces) == receipt['records'] and [r['seq'] for r in traces] == list(range(1,len(traces)+1)), 'Trace count/sequence mismatch')
    groups = defaultdict(list)
    for record in traces:
        require(record['end_wall_s'] >= record['start_wall_s'], 'Negative trace interval')
        groups[record['operation']].append(record)
    require({k:len(v) for k,v in groups.items()} == receipt['calls'], 'Trace call count mismatch')
    for rows in groups.values():
        require([r['call_index'] for r in rows] == list(range(1,len(rows)+1)), 'Trace call order mismatch')
    spans = {k:{'count':len(v),'cold_wall_s':v[0]['duration_wall_s'],
        'max_wall_s':max(r['duration_wall_s'] for r in v),
        'later_max_wall_s':max((r['duration_wall_s'] for r in v[1:]),default=None)} for k,v in groups.items()}
    publications = {}
    for key in ('offboard_control_mode','trajectory_setpoint','vehicle_command'):
        rows = groups['publish:'+key]
        require([r['source_timestamp'] for r in rows] == [r['timestamp'] for r in streams[key]], 'Publish/bag count or identity mismatch: '+key)
        publications[key] = rows
    for event in (e for e in events if e['event'] in ('setpoint','command_sent')):
        key = 'trajectory_setpoint' if event['event']=='setpoint' else 'vehicle_command'
        rows = [r for r in publications[key] if r['source_timestamp']==event['fields']['timestamp']]
        require(len(rows)==1 and event['evidence_schema']==2, 'Publication event identity/schema mismatch')
        row, pub = rows[0], event['publication']
        require(pub['entry_wall_s'] <= row['start_wall_s'] <= row['end_wall_s'] <= pub['return_wall_s'] <= event['monotonic_s'], 'Publication/event ordering mismatch')
        require(pub['selected_source_timestamp']==row['selected_source_timestamp'], 'Dispatch selection mismatch')
    selected = [r for r in groups['mission_tick'] if 'selected_source_timestamp' in r]
    consumed = [r['selected_source_timestamp']/1e6 for r in selected]
    if config.get('sample_evidence_contract') == 'p3-sample-evidence-v2':
        native = [r['selected_source_timestamp'] for r in selected]
        require(all(type(t) is int and t > 0 for t in native), 'Invalid native consumed timestamp')
        require(all(b >= a for a,b in zip(native,native[1:])), 'Consumed source time regression')
        # Repeats add no time, while a delayed controller still leaves its full
        # source-consumption jump. Continuous incoming data cannot bridge it.
        require(not native or max((b-a for a,b in zip(native,native[1:])),default=0)
                <= round(config['max_sample_gap_sim_s']*1e6), 'Control consumption gap')
    # A failing attempted selection stays in the trace. It cannot be replaced
    # with intervening continuously received source samples.
    commands = publications['vehicle_command']
    cutoff = commands[0]['start_wall_s'] if commands else float('inf')
    prestream = [r for r in publications['trajectory_setpoint'] if r['start_wall_s'] < cutoff]
    coverage = prestream[-1]['sim_s']-prestream[0]['sim_s'] if prestream else 0.
    pre_gaps = {k:maximum_gap([r['sim_s'] for r in publications[k] if r['start_wall_s'] < cutoff])
                for k in ('offboard_control_mode','trajectory_setpoint')}
    metrics = {'contract':'p2-timing-v1','status':'failed' if result['failure'] else 'passed',
        'trace':receipt,'spans':spans,'control_consumed_sample_gap_sim_s':maximum_gap(consumed),
        'source_sample_gap_sim_s':maximum_gap([r['timestamp']/1e6 for r in streams.get('vehicle_local_position',[])]),
        'source_age_at_control_sim_s':max((r['source_age_at_control_sim_s'] for r in selected),default=None),
        'source_age_at_dispatch_sim_s':max((r['source_age_at_dispatch_sim_s'] for rows in publications.values() for r in rows),default=None),
        'callback_entry_gap_wall_s':maximum_gap([r['start_wall_s'] for r in groups['subscription_callback:vehicle_local_position']]),
        'actual_prestream_publication_sim_s':coverage,'prestream_publish_gaps_sim_s':pre_gaps,
        'heartbeat_publish_gap_sim_s':maximum_gap([r['sim_s'] for r in publications['offboard_control_mode']]),
        'trajectory_publish_gap_sim_s':maximum_gap([r['sim_s'] for r in publications['trajectory_setpoint']]),
        'control_callback_duration_wall_s':spans.get('control_callback',{}).get('max_wall_s'),
        'publish_call_duration_wall_s':max((r['duration_wall_s'] for rows in publications.values() for r in rows),default=None),
        'evidence_enqueue_duration_wall_s':None,
        'evidence_persist_delay_wall_s':max((r['duration_wall_s'] for key,rows in groups.items() if key.startswith('subscription_callback:') for r in rows),default=None),
        'graph_check_duration_wall_s':spans.get('graph_check',{}).get('max_wall_s'),
        'publication_bag_event_identity_verified':True,
        'limitations':['Application callback entry is not DDS arrival; the same-process bag is not independent middleware arrival.',
            'Persistence delay brackets callback acquisition through synchronous serialization, write return and text flush; no fsync durability or physical-disk latency claim.',
            'No evidence queue is used; enqueue latency is unavailable. A guard detects but cannot preempt a blocking middleware or disk call.']}
    callbacks=groups['control_callback']
    if len(callbacks)>1:
        elapsed=callbacks[-1]['start_wall_s']-callbacks[0]['start_wall_s']
        metrics.update(control_callback_rate_wall_hz=(len(callbacks)-1)/elapsed,
            control_callback_entry_gap_wall_s=maximum_gap([r['start_wall_s'] for r in callbacks]),
            observed_sim_seconds_per_wall_second=(callbacks[-1]['sim_s']-callbacks[0]['sim_s'])/elapsed)
    if not result['failure'] and (publications['trajectory_setpoint'] or result['flight']=='passed'):
        require(coverage >= config['prestream_sim_s']-1.1e-6, 'Actual publication prestream incomplete')
        require(all(v is not None and v<=config['max_sample_gap_sim_s']+1e-9 for v in pre_gaps.values()), 'Actual prestream publication gap')
        require(metrics['control_consumed_sample_gap_sim_s'] <= config['max_sample_gap_sim_s']+1e-9, 'Control consumption gap')
        require(not result.get('evidence_error'), 'Recorder error')
        budget=result['timing_configuration']['dispatch_budget_wall_s']
        for rows in publications.values():
            for pub in rows:
                owner = [r for r in groups['control_callback'] if r['start_wall_s'] <= pub['start_wall_s'] <= r['end_wall_s']]
                require(len(owner)==1 and pub['end_wall_s']-owner[0]['start_wall_s']<=budget, 'Late publication/action')
                require(pub['source_age_at_dispatch_sim_s']<=config['position_fresh_sim_s'], 'Stale dispatch source')
    return metrics


def verify_timing(run, result, streams, events, config):
    from collect_p2_evidence import digest, require
    from collections import Counter
    traces=[json.loads(line) for line in (run/'timing-trace.jsonl').read_text().splitlines()]
    report=trace_metrics(traces,result,streams,events,config)
    operations=Counter(r['operation'] for r in traces)
    topic_map=json.loads((run/'topic-contract.json').read_text())
    for key,meta in topic_map.items():
        require(operations['bag_write:'+meta['topic']]==len(streams[key]), 'Trace/persisted bag count mismatch: '+key)
    require(operations['bag_write:/gwm/p2/events']==len(events), 'Trace/event persistence count mismatch')
    require(sum(e['event']=='setpoint' for e in events)==len(streams['trajectory_setpoint']), 'Missing setpoint event')
    require(sum(e['event']=='command_sent' for e in events)==len(streams['vehicle_command']), 'Missing command event')
    report.update(trace_sha256=digest(run/'timing-trace.jsonl'),evaluator_sha256=digest(__file__))
    return report


def reconstruct(events, result):
    received = [e for e in events if e['event']=='received' and e['topic_key']=='vehicle_local_position']
    samples = [e for e in events if e['event']=='sample']
    targets = [e for e in events if e['event']=='setpoint']
    prep = next(e for e in events if e['event']=='transition' and e['to']=='PRESTREAM_SAFE_SETPOINTS')
    first = targets[0]
    selection = next(e for e in samples if e['ros_sim_s']==first['ros_sim_s'])
    # Historical StateCache.validate() assigned its wall argument to this
    # unfortunately named field. It is selection time, not callback receipt.
    selected_wall = selection['sample']['receipt_monotonic_s']
    prior = [e for e in received if e['monotonic_s']<=selected_wall]
    callback = next(e for e in reversed(prior) if e['fields']['timestamp']/1e6==selection['sample']['t'])
    abort = next((e for e in events if e['event']=='abort'), None)
    end_wall = abort['monotonic_s'] if abort else first['monotonic_s']+0.5
    focused = [e for e in received if prep['monotonic_s']-.1<=e['monotonic_s']<=end_wall]
    consumed = [e['sample']['t'] for e in samples]
    # Mission raises before emitting the rejected action/sample. The received
    # cache update preceding abort supplies that attempted sample identity.
    rejected = None
    if abort and abort['reason']=='observation_gap':
        rejected = next(e for e in reversed(received) if e['monotonic_s']<=abort['monotonic_s'])['fields']['timestamp']/1e6
        consumed.append(rejected)
    return {'reference_policy':result['reference_policy'],'controller_failure':result['failure'],
        'source_sample_gap_sim_s':maximum_gap([e['fields']['timestamp']/1e6 for e in received]),
        'callback_entry_gap_wall_s':maximum_gap([e['monotonic_s'] for e in focused]),
        'control_consumed_sample_gap_sim_s':maximum_gap(consumed),
        'prestream_transition':prep,'first_selected_source_sim_s':selection['sample']['t'],
        'first_selection_wall_s':selected_wall,'first_callback_entry_wall_s':callback['monotonic_s'],
        'first_source_age_at_control_sim_s':selection['ros_sim_s']-selection['sample']['t'],
        'first_target_event':first,
        'selection_to_post_publish_event_wall_s':first['monotonic_s']-selected_wall,
        'callback_entry_to_post_publish_event_wall_s':first['monotonic_s']-callback['monotonic_s'],
        'source_age_at_dispatch_sim_s':None,'publish_call_duration_wall_s':None,
        'evidence_persist_delay_wall_s':None,'control_callback_duration_wall_s':None,
        'rejected_source_sim_s':rejected,'abort':abort,'commands':result['transactions'],
        'callback_timeline':[{'source_sim_s':e['fields']['timestamp']/1e6,'callback_entry_wall_s':e['monotonic_s'],
                              'callback_ros_sim_s':e['ros_sim_s']} for e in focused],
        'limitations':['Historical sample.receipt_monotonic_s is Mission selection time, not callback receipt.',
            'received.monotonic_s brackets application callback entry; not independent DDS arrival.',
            'Post-publish event time includes uninstrumented construction/publish/recording work.',
            'Same-process bag timestamps do not independently measure middleware arrival.']}


def historical(run):
    from collect_p2_evidence import digest, read_bag, read_ulog
    manifest=json.loads((run/'summary.json').read_text())
    result=json.loads((run/'controller-result.json').read_text())
    events=[json.loads(line) for line in (run/'ros-events.jsonl').read_text().splitlines()]
    report=reconstruct(events,result)
    bag,counts,_=read_bag(run)
    log,ulog=read_ulog(manifest['ulog_files'][0]['path'])
    first=report['first_target_event']['fields']['timestamp']/1e6
    report.update(run_id=run.name,summary_sha256=digest(run/'summary.json'),
        events_sha256=digest(run/'ros-events.jsonl'),recorded_status=manifest['status'],
        cleanup=manifest['cleanup'],final_console_state=manifest['final_console_state'],
        ulog_dropouts=len(log.dropouts),bag_counts=dict(counts))
    report['ulog_source_gap_near_first_sim_s']=maximum_gap([r['timestamp']/1e6 for r in ulog['vehicle_local_position'] if first-.5<=r['timestamp']/1e6<=first+.5])
    report['recorded_input_header_gaps_sim_s']={key:maximum_gap([r['timestamp']/1e6 for r in bag[key]]) for key in ('offboard_control_mode','trajectory_setpoint')}
    report['ulog_trajectory_near_first']=[r for r in ulog['trajectory_setpoint'] if first-.1<=r['timestamp']/1e6<=first+.5]
    # Preserve intentional NaNs without emitting invalid JSON.
    from math import isfinite
    def clean(v):
        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
        if isinstance(v,list):return [clean(x) for x in v]
        return None if isinstance(v,float) and not isfinite(v) else v
    return clean(report)


def traced_ground(run):
    from collections import defaultdict
    import math
    from collect_p2_evidence import digest, read_bag, read_ulog, require
    manifest=json.loads((run/'summary.json').read_text())
    result=json.loads((run/'controller-result.json').read_text())
    config=manifest['config']
    for artifact in manifest['ulog_files']+manifest['rosbag_files']:
        require(digest(artifact['path'])==artifact['sha256'],'Recording hash changed')
    streams,counts,events=read_bag(run)
    raw=[json.loads(line) for line in (run/'ros-events.jsonl').read_text().splitlines()]
    require(events==[e for e in raw if e['event']!='received'],'Event ledger mismatch')
    log,ulog=read_ulog(manifest['ulog_files'][0]['path'])
    traces=[json.loads(line) for line in (run/'timing-trace.jsonl').read_text().splitlines()]
    require(result['trace']['complete'] and result['trace']['overflow']==0,'Incomplete trace')
    require(len(traces)==result['trace']['records'] and [r['seq'] for r in traces]==list(range(1,len(traces)+1)),'Trace count/sequence mismatch')
    groups=defaultdict(list)
    for r in traces:groups[r['operation']].append(r)
    spans={k:{'count':len(v),'cold_wall_s':v[0]['duration_wall_s'],
        'max_wall_s':max(r['duration_wall_s'] for r in v),
        'later_max_wall_s':max((r['duration_wall_s'] for r in v[1:]),default=None)} for k,v in groups.items()}
    require(not streams['vehicle_command'] and not result['transactions'],'Ground command published')
    require(streams['vehicle_status'] and streams['vehicle_land_detected'], 'Ground state evidence missing')
    require(all(s['arming_state']==1 for s in streams['vehicle_status']),'Ground run armed')
    require(all(s['landed'] for s in streams['vehicle_land_detected']),'Ground run left landed state')
    require(all(s['arming_state']==1 for s in ulog['vehicle_status']) and all(s['landed'] for s in ulog['vehicle_land_detected']), 'ULog ground state violated')
    require(not any(t['to'] in ('REQUEST_OFFBOARD','REQUEST_ARM') for t in result['transitions']),'Ground command transition')
    targets=streams['trajectory_setpoint']
    for t in targets:
        require(all(math.isfinite(v) for v in t['position']) and math.isnan(t['yaw']) and t['yawspeed']==0,'Ground wire semantics')
        require(all(math.isnan(v) for k in ('velocity','acceleration','jerk') for v in t[k]),'Inactive translation')
        require(math.dist(t['position'],result['origin_ned'])<1e-5,'Ground target moved')
    report={'run_id':run.name,'contract':'p2-timing-v1','kind':'ground_timing_diagnostic',
        'recording_integrity':'passed','ground_diagnostic':'passed' if not result['failure'] and manifest['status']=='passed' else 'failed',
        'controller_failure':result['failure'],'no_vehicle_commands':True,'landed_disarmed_throughout':True,
        'trace':result['trace'],'trace_sha256':digest(run/'timing-trace.jsonl'),'spans':spans,
        'ulog_dropouts':len(log.dropouts),'bag_counts':dict(counts),'cleanup':manifest['cleanup'],
        'source_sample_gap_sim_s':maximum_gap([p['timestamp']/1e6 for p in streams['vehicle_local_position']]),
        'ulog_source_gap_sim_s':maximum_gap([p['timestamp']/1e6 for p in ulog['vehicle_local_position']]),
        'callback_entry_gap_wall_s':maximum_gap([r['start_wall_s'] for r in groups['subscription_callback:vehicle_local_position']]),
        'control_consumed_sample_gap_sim_s':maximum_gap([r['source_timestamp']/1e6 for r in groups['mission_tick'] if r.get('source_timestamp') is not None]),
        'publish_header_gaps_sim_s':{k:maximum_gap([r['timestamp']/1e6 for r in streams[k]]) for k in ('offboard_control_mode','trajectory_setpoint')},
        'actual_recorded_prestream_sim_s':(targets[-1]['timestamp']-targets[0]['timestamp'])/1e6 if targets else 0,
        'evaluator_sha256':digest(__file__),
        'limitations':['Callback entry is not DDS arrival. Same-process bag is not independent middleware arrival.',
            'Old first-publication spans cannot be retroactively separated by this new trace.']}
    if report['ground_diagnostic']=='passed':
        require(report['actual_recorded_prestream_sim_s']>=config['prestream_sim_s'],'Insufficient actual ground prestream')
        require(all(v is not None and v<=config['max_sample_gap_sim_s'] for v in report['publish_header_gaps_sim_s'].values()),'Ground publication gap')
    if result.get('evidence_schema')==2:
        report['timing']=verify_timing(run,result,streams,events,config)
        report['control_consumed_sample_gap_sim_s']=report['timing']['control_consumed_sample_gap_sim_s']
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--historical',nargs='+',type=Path)
    mode.add_argument('--ground',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=traced_ground(args.ground) if args.ground else {
        'timing_contract':'p2-timing-v1','kind':'read_only_historical_reconstruction',
        'runs':[historical(p) for p in args.historical]}
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    if args.ground:
        print(json.dumps({k:v for k,v in report.items() if k not in ('trace','bag_counts','spans')},indent=2))
        print(json.dumps(sorted(report['spans'].items(),key=lambda v:v[1]['max_wall_s'],reverse=True)[:12],indent=2))
    else:
        print(json.dumps([{k:r[k] for k in ('run_id','controller_failure','source_sample_gap_sim_s',
        'callback_entry_gap_wall_s','control_consumed_sample_gap_sim_s','selection_to_post_publish_event_wall_s',
        'callback_entry_to_post_publish_event_wall_s','ulog_source_gap_near_first_sim_s')} for r in report['runs']],indent=2))
