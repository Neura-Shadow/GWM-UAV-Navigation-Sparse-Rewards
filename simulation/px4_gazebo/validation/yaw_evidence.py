"""Independent v3 initialization/handover wire and sampled PX4-output checks."""
import math
import numpy as np

from collect_p2_evidence import require, yaw


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def verify_yaw_ownership(log, streams, events, result, config, reference):
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
    positions = {p['timestamp']:p for p in streams['vehicle_local_position']}
    seed = positions.get(handover.get('position_timestamp'))
    require(seed is not None and -epsilon<=start-seed['timestamp']/1e6<=config['position_fresh_sim_s'], 'Missing fresh handover heading')
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
    indexes=np.flatnonzero((control['timestamp']>=offboard*1e6)&(control['timestamp']<start*1e6))
    require(len(indexes)>=50,'Insufficient resolved-yaw samples')
    errors=[];ages=[]
    for i in indexes:
        j=int(np.searchsorted(p['timestamp'],control['timestamp'][i],side='right'))-1
        require(j>=0,'No matched heading')
        age=(int(control['timestamp'][i])-int(p['timestamp'][j]))/1e6
        require(0<=age<=.008001,'Unmatched native estimator/control sample')
        error=abs(angle(float(control['yaw'][i])-float(p['heading'][j])))
        require(error<=1e-5,'PX4 unspecified yaw did not resolve to matched current heading')
        require(float(control['yawspeed'][i])==0,'PX4 yaw feedforward not zero')
        errors.append(error);ages.append(age)
    stamps=control['timestamp'][indexes].astype(float)/1e6
    return {'status':'passed','initialization_wire_samples':first,'handover_start_sim_s':start,
            'handover_complete_sim_s':complete,'first_finite_yaw':targets[first]['yaw'],
            'fresh_aligned_heading':seed['heading'],'corrected_mission_anchor':anchor,
            'max_initialization_drift_deg':max_drift,'matched_internal_samples':len(indexes),
            'max_resolved_yaw_error_rad':max(errors),'max_match_age_s':max(ages),
            'internal_max_sample_gap_s':float(np.max(np.diff(stamps))),
            'coverage_limit':'Internal output sampled about 10 Hz; intervals and downstream scheduling are not proven at every update.'}
