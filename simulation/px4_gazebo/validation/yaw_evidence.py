"""Independent v3 initialization/handover wire and sampled PX4-output checks."""
import math
from bisect import bisect_left
import numpy as np

from collect_p2_evidence import require, yaw


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class YawEvidenceError(ValueError):
    """A failed frozen yaw check with retained diagnostic measurements."""
    def __init__(self, report):
        self.report = report
        super().__init__(
            'Prior-state yaw consistency failed: '
            f'{report["failed_samples"]}/{report["internal_samples"]} samples; '
            f'max_error_rad={report["max_resolved_yaw_error_rad"]}; '
            f'max_age_s={report["max_match_age_s"]}; '
            'exact MC consumed-source identity is not recorded'
        )


def handover_source(rows, handover, topic, run_id=None, require_reference=False):
    """Resolve a full source reference, or require one historical stamp candidate.

    Equal publication timestamps are retained in a candidate list. A historical
    timestamp alone never chooses the first or last candidate. For new evidence,
    several exact copies of one verified source are reported as delivery copies.
    """
    from sample_evidence import payload
    from gwm_px4_control.sample_identity import source_record
    prefix = 'position' if topic == 'vehicle_local_position' else 'attitude'
    publication = handover.get(prefix + '_timestamp')
    reference = handover.get(prefix + '_source')
    candidates = [row for row in rows if row['timestamp'] == publication]
    require(candidates, 'Missing handover source: ' + topic)
    if reference is None:
        require(not require_reference, 'Missing full handover source reference: ' + topic)
        require(len(candidates) == 1, 'Ambiguous historical handover publication: ' + topic)
        return candidates[0], dict(policy='historical_unique_publication_candidate',
            publication_us=publication, raw_candidates=1, matched_delivery_copies=1,
            actual_callback_source_identity='not_recorded')
    require(isinstance(reference, dict) and reference.get('run_id') == run_id,
            'Cross-run or invalid handover source reference: ' + topic)
    required = ('contract', 'run_id', 'topic', 'message_version', 'instance',
                'publication_us', 'sample_us', 'reference_generation', 'payload_sha256', 'source_id')
    require(all(key in reference for key in required), 'Incomplete handover source reference: ' + topic)
    matched = []
    for row in candidates:
        calculated = source_record(topic, payload(row), run_id, instance=row.get('_instance', 0))
        if all(reference[key] == calculated[key] for key in required):
            matched.append(row)
    require(matched, 'Unmatched handover source identity or payload: ' + topic)
    # A full identity includes payload hash: all matches are equivalent source
    # copies, so selecting one cannot choose a more favorable measured value.
    return matched[0], dict(policy='exact_recorded_source_identity_and_payload',
        publication_us=publication, sample_us=reference['sample_us'],
        source_id=reference['source_id'], raw_candidates=len(candidates),
        matched_delivery_copies=len(matched), actual_callback_source_identity='verified')


def prior_state_yaw_consistency(positions, control, start_us, end_us, run_id):
    """Frozen prior-state correlation; never claims the actual MC input source.

    Pinned MC copies local position on nav_and_controllers and later stamps its
    output with HRT. Direct EKF2 runs on INS0. The output lacks a consumed-source
    timestamp. Consequently same-time cross-topic publication order is unknown.
    Use the latest strictly earlier group and its final validated native output,
    preserving the original 8.001 ms age and 1e-5 rad error bounds.
    """
    from sample_evidence import validate_stream
    p = validate_stream(positions, 'vehicle_local_position', run_id)['records']
    c = validate_stream(control, 'vehicle_local_position_setpoint', run_id)['records']
    selected = [row for row in c if start_us <= row['timestamp'] < end_us]
    require(selected, 'Missing sampled internal yaw outputs')
    times = [row['timestamp'] for row in p]
    matches, violations = [], []
    for row in selected:
        stamp = row['timestamp']
        index = bisect_left(times, stamp)-1
        if index < 0:
            violations.append(dict(control_publication_us=stamp, reason='missing_strictly_earlier_heading'))
            continue
        source = p[index]
        age_us = stamp-source['timestamp']
        delta = float(row['yaw'])-float(source['heading'])
        error = abs(angle(delta)) if math.isfinite(delta) else math.nan
        finite_or_none = lambda value: value if math.isfinite(value) else None
        evidence = dict(control_publication_us=stamp, control_record_id=row['_evidence']['record_id'],
            position_publication_us=source['timestamp'], position_sample_us=source['timestamp_sample'],
            position_record_id=source['_evidence']['record_id'], match_age_us=age_us,
            resolved_yaw_error_rad=finite_or_none(error), internal_yaw=finite_or_none(float(row['yaw'])),
            prior_heading=finite_or_none(float(source['heading'])))
        matches.append(evidence)
        reasons = []
        if not 0 < age_us <= 8001:
            reasons.append('prior_heading_age_exceeded')
        if not math.isfinite(error) or error > 1e-5:
            reasons.append('prior_heading_yaw_mismatch')
        if float(row['yawspeed']) != 0:
            reasons.append('internal_yaw_feedforward_nonzero')
        if reasons:
            violations.append({**evidence, 'reasons': reasons})
    return dict(status='failed' if violations else 'passed',
        policy='latest_strictly_earlier_publication_group_final_validated_output',
        meaning='prior-state feedback consistency, not exact MC consumed-source identity',
        exact_mc_consumed_source_identity='unproven', internal_samples=len(selected),
        distinct_internal_observations=len({row['_evidence']['source_id'] for row in selected}),
        duplicate_internal_deliveries=sum(row['_evidence']['classification']=='duplicate_reuse' for row in selected),
        matched_internal_samples=len(matches), failed_samples=len(violations),
        max_resolved_yaw_error_rad=max((r['resolved_yaw_error_rad'] for r in matches
                                       if r['resolved_yaw_error_rad'] is not None), default=None),
        max_match_age_s=max((r['match_age_us']/1e6 for r in matches), default=None),
        match_age_limit_s=.008001, yaw_error_limit_rad=1e-5,
        matches=matches, violations=violations,
        internal_max_sample_gap_s=max((b['timestamp']-a['timestamp'] for a,b in zip(selected,selected[1:])), default=0)/1e6)


def native_dataset(log, topic, run_id=None):
    """Keep ULog topic/instance/native ordinals; reconstruct array fields once."""
    dataset = log.get_dataset(topic)
    require({entry.multi_id for entry in log.data_list if entry.name == topic} == {0},
            'Unsupported internal yaw ULog instance: ' + topic)
    data = dataset.data
    rows = []
    for ordinal in range(len(data['timestamp'])):
        row = {key: value[ordinal].item() for key,value in data.items()}
        for field, size in (('q',4), ('delta_q_reset',4), ('delta_xy',2), ('delta_vxy',2)):
            if field+'[0]' in row:
                row[field] = [row.pop(f'{field}[{index}]') for index in range(size)]
        row.update(_instance=dataset.multi_id, _record_id=f'{run_id or "legacy"}:ulog:{topic}:{dataset.multi_id}:{ordinal+1}',
                   _native_dataset_ordinal=ordinal+1)
        if run_id is not None: row['_run_id']=run_id
        rows.append(row)
    return rows


def verify_yaw_ownership(log, streams, events, result, config, reference):
    revised = config.get('sample_evidence_contract') == 'p3-sample-evidence-v2'
    targets = streams['trajectory_setpoint']
    records = [e for e in events if e['event'] == 'setpoint']
    require(len(records) == len(targets), 'Missing wire-mode ledger')
    first = next((i for i,t in enumerate(targets) if math.isfinite(t['yaw'])), None)
    require(first is not None and first > 0, 'Missing initialization or finite handover')
    start = targets[first]['timestamp']/1e6
    handover = result.get('handover') or {}
    # Normal float-seconds -> integer-microseconds serialization can truncate
    # by one microsecond. This does not permit a control-cycle timing shift.
    epsilon=1.1e-6
    require(handover.get('start_sim_s') is not None and abs(start-handover['start_sim_s'])<=epsilon
            and start >= reference['lock_sim_s']-epsilon, 'Handover before reference lock')
    complete = handover.get('completed_sim_s')
    require(complete is not None and complete > start, 'Missing bounded handover completion')
    hover = next(t['sim_s'] for t in result['transitions'] if t['to']=='INITIAL_HOVER')
    require(hover >= complete, 'Nominal window before handover')
    for i,(t,r) in enumerate(zip(targets,records)):
        init = i < first
        mode = 'POSITION_INITIALIZATION_YAW_UNSPECIFIED' if init else 'POSITION_NOMINAL_YAW_TARGET'
        require(r.get('mode') == mode, 'Fabricated yaw mode')
        require(r.get('yaw_phase') == ('initialization' if init else 'handover' if t['timestamp']/1e6 < complete-epsilon else 'nominal'), 'Wrong yaw phase')
        inactive = {'velocity','acceleration','jerk','yaw' if init else 'yawspeed'}
        require(r['fields']['timestamp']==t['timestamp'], 'Wire/event timestamp mismatch')
        for key in ('position','velocity','acceleration','jerk','yaw','yawspeed'):
            require(r['active'][key] == (key not in inactive), 'Wrong activation mask: '+key)
            raw = t[key] if isinstance(t[key], (list,tuple)) else [t[key]]
            recorded = r['fields'][key] if isinstance(r['fields'][key],list) else [r['fields'][key]]
            require(len(raw)==len(recorded), 'Wire field shape mismatch')
            for v,expected in zip(raw,recorded):
                require(math.isnan(v) and expected is None if key in inactive else
                        expected is not None and math.isfinite(v) and abs(v-expected)<1e-6,
                        'CDR/JSON field mismatch: '+key)
        require(t['yawspeed']==0 if init else math.isnan(t['yawspeed']), 'Wrong yaw rate semantics')
        if init:
            require(math.isnan(t['yaw']), 'Finite external initialization yaw')
            require(math.dist(t['position'][:2],result['origin_ned'][:2])<1e-5, 'Initialization horizontal target moved')
    run_ids = {row['_run_id'] for row in streams['vehicle_local_position'] if '_run_id' in row}
    run_id = result.get('run_id') or (next(iter(run_ids)) if len(run_ids) == 1 else None)
    new_metadata = result.get('sample_evidence_contract') == 'p3-sample-evidence-v2'
    if revised:
        from sample_evidence import validate_stream
        require(run_id is not None, 'Missing run identity for yaw source validation')
        for topic in ('vehicle_local_position', 'vehicle_attitude'):
            validate_stream(streams[topic], topic, run_id)
            validate_stream(native_dataset(log, topic, run_id), topic, run_id)
    seed, seed_evidence = handover_source(streams['vehicle_local_position'], handover,
        'vehicle_local_position', run_id, require_reference=new_metadata)
    require(-epsilon<=start-seed['timestamp']/1e6<=config['position_fresh_sim_s'], 'Missing fresh handover heading')
    attitude_evidence = None
    if revised:
        attitude_seed, attitude_evidence = handover_source(streams['vehicle_attitude'], handover,
            'vehicle_attitude', run_id, require_reference=new_metadata)
        require(-epsilon<=start-attitude_seed['timestamp']/1e6<=config['attitude_fresh_sim_s'], 'Missing fresh handover attitude')
        from sample_evidence import controller_yaw_reconstruction
        require(abs(angle(controller_yaw_reconstruction(attitude_seed['q'])-handover['attitude_yaw']))<1e-8,
                'Handover attitude source differs from recorded selection')
    require(abs(angle(targets[first]['yaw']-seed['heading']))<1e-5, 'First finite yaw not seeded from fresh aligned heading')
    anchor = angle(reference['initial_anchor']+sum(e['delta_heading'] for e in reference['accepted']))
    require(abs(angle(anchor-result['initial_yaw_ned']))<1e-8, 'Mission reanchored to hide drift')
    end_target = next(t for t in targets if t['timestamp']/1e6>=complete)
    require(abs(angle(end_target['yaw']-anchor))<1e-5, 'Handover did not reach original corrected anchor')
    for e in reference['accepted']:
        pending = [t['position'] for t in targets if e['start_sim_s']<=t['timestamp']/1e6<e['accepted_sim_s']]
        require(pending and all(math.dist(p,pending[0])<1e-6 for p in pending), 'Pending reset did not hold position')
    # Use each estimator topic's own reset generation for drift, not controller acceptance time.
    p=log.get_dataset('vehicle_local_position').data
    a=log.get_dataset('vehicle_attitude').data
    begin = targets[0]['timestamp']
    max_drift={}
    for label,d,counter,measurement,delta in (
            ('heading',p,'heading_reset_counter',lambda i:float(p['heading'][i]),lambda i:float(p['delta_heading'][i])),
            ('attitude',a,'quat_reset_counter',lambda i:yaw([float(a[f'q[{n}]'][i]) for n in range(4)]),
             lambda i:yaw([float(a[f'delta_q_reset[{n}]'][i]) for n in range(4)]))):
        indexes=np.flatnonzero((d['timestamp']>=begin)&(d['timestamp']<=complete*1e6))
        require(len(indexes)>0,'Missing initialization drift coverage')
        current_anchor=reference['initial_anchor'];previous=int(d[counter][indexes[0]])
        errors=[]
        for i in indexes:
            count=int(d[counter][i])
            if count!=previous:
                require((count-previous)%256==1,'Ambiguous drift counter')
                current_anchor=angle(current_anchor+delta(i));previous=count
            errors.append(abs(angle(measurement(i)-current_anchor)))
        max_drift[label]=math.degrees(max(errors))
        require(max_drift[label]<=config['yaw_tolerance_deg'],'Initialization drift exceeded original bound')
    # Internal setpoints are sampled at 10 Hz. Match each sample to the most
    # recent native estimator publication; never search a window for a nicer yaw.
    control=log.get_dataset('vehicle_local_position_setpoint').data
    offboard=next(t['sim_s'] for t in result['transitions'] if t['to']=='VERIFY_OFFBOARD')
    prior_state = None
    if revised:
        prior_state = prior_state_yaw_consistency(native_dataset(log, 'vehicle_local_position',run_id),
            native_dataset(log, 'vehicle_local_position_setpoint',run_id), round(offboard*1e6),
            targets[first]['timestamp'], run_id)
        require(prior_state['distinct_internal_observations'] >= 50, 'Insufficient distinct resolved-yaw samples')
        if prior_state['status'] != 'passed':
            raise YawEvidenceError(prior_state)
    indexes=np.flatnonzero((control['timestamp']>=offboard*1e6)&(control['timestamp']<start*1e6))
    require(len(indexes)>=50,'Insufficient resolved-yaw samples')
    errors=[];ages=[]
    for i in ([] if revised else indexes):
        j=int(np.searchsorted(p['timestamp'],control['timestamp'][i],side='right'))-1
        require(j>=0,'No matched heading')
        age=(int(control['timestamp'][i])-int(p['timestamp'][j]))/1e6
        require(0<=age<=.008001,'Unmatched native estimator/control sample')
        error=abs(angle(float(control['yaw'][i])-float(p['heading'][j])))
        require(error<=1e-5,'PX4 unspecified yaw did not resolve to matched current heading')
        require(float(control['yawspeed'][i])==0,'PX4 yaw feedforward not zero')
        errors.append(error);ages.append(age)
    if revised:
        errors = [row['resolved_yaw_error_rad'] for row in prior_state['matches']]
        ages = [row['match_age_us']/1e6 for row in prior_state['matches']]
    stamps=control['timestamp'][indexes].astype(float)/1e6
    return {'status':'passed','initialization_wire_samples':first,'handover_start_sim_s':start,
            'handover_complete_sim_s':complete,'first_finite_yaw':targets[first]['yaw'],
            'fresh_aligned_heading':seed['heading'],'corrected_mission_anchor':anchor,
            'handover_position_source':seed_evidence,'handover_attitude_source':attitude_evidence,
            'prior_state_consistency':prior_state,
            'max_initialization_drift_deg':max_drift,'matched_internal_samples':len(indexes),
            'max_resolved_yaw_error_rad':max(errors),'max_match_age_s':max(ages),
            'internal_max_sample_gap_s':prior_state['internal_max_sample_gap_s'] if revised else float(np.max(np.diff(stamps))),
            'coverage_limit':'Internal output sampled about 10 Hz; intervals and downstream scheduling are not proven at every update.'}
