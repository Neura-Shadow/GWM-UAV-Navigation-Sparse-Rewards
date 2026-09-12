"""P3-R2 evidence roles and a pure model for independently paired trace data.

No current ULog adapter produces this trace schema. Synthetic verification tests
the proposed causal model, not observability or approval of an instrumented build.
"""
import math

CONTRACT = 'p3-yaw-causal-evidence-v1'
LIMIT_RAD = 1e-5


def unobservable(reason='The pinned output serializes no consumed input or update identity'):
    return dict(status='unobservable_from_recording', required=True, reason=reason,
                contract=CONTRACT, pairs=[], yaw_error_limit_rad=LIMIT_RAD)


def exact_relation(sources, updates, run_id, binary_sha256):
    """Use only independently captured IDs/order, never timestamps or yaw to pair.

    None encodes an explicitly recorded NaN setpoint. Missing keys mean missing
    evidence. Order is an acquisition-proven execution ordinal, never ULog order.
    The acquisition's completeness and authenticity must be separately qualified.
    """
    if not updates:
        return unobservable()
    index = {}
    for source in sources:
        identity = source.get('source_id')
        if not identity or identity in index:
            return dict(status='missing_required_evidence', contract=CONTRACT,
                        reason='Missing or ambiguous independently captured source ID', pairs=[])
        index[identity] = source
    pairs, statuses, used_updates = [], [], set()
    for update in updates:
        identity = update.get('consumed_source_id')
        if not identity or any(update.get(k) is None for k in
            ('update_id','copy_order','output_order','consumed_generation')):
            statuses.append('unobservable_from_recording'); continue
        source = index.get(identity)
        needed_input = ('run_id','binary_sha256','topic','instance','timestamp','timestamp_sample',
                        'generation','reference_generation','heading','publication_order')
        needed_output = ('run_id','binary_sha256','reference_generation','active_yaw','active_yawspeed',
                         'path','update_valid','output_yaw','output_yawspeed','output_timestamp')
        if source is None or any(k not in source for k in needed_input) or any(k not in update for k in needed_output):
            statuses.append('missing_required_evidence'); continue
        violations = []
        if any(row['run_id'] != run_id or row['binary_sha256'] != binary_sha256 for row in (source, update)):
            violations.append('run_or_binary_mismatch')
        if source['topic'] != 'vehicle_local_position' or source['instance'] != 0:
            violations.append('unsupported_consumed_topic_instance')
        if update['update_id'] in used_updates:
            violations.append('duplicate_update_identity')
        used_updates.add(update['update_id'])
        if source['generation'] != update['consumed_generation'] or source['reference_generation'] != update['reference_generation']:
            violations.append('consumed_generation_mismatch')
        times = [source['timestamp_sample'], source['timestamp'], update['output_timestamp']]
        orders = [source['publication_order'], update['copy_order'], update['output_order']]
        if (not all(type(v) is int and v >= 0 for v in times+orders)
                or not times[0] <= times[1] <= times[2] or not orders[0] < orders[1] < orders[2]):
            violations.append('invalid_causal_order')
        if update['path'] not in ('normal','last_valid_fallback','generated_failsafe') or update['update_valid'] is not True:
            statuses.append('missing_required_evidence'); continue
        expected = source['heading'] if update['active_yaw'] is None else update['active_yaw']
        feedforward = 0. if update['active_yawspeed'] is None else update['active_yawspeed']
        numbers = [expected, feedforward, update['output_yaw'], update['output_yawspeed']]
        error = None
        if not all(type(v) in (int,float) and math.isfinite(v) for v in numbers):
            violations.append('nonfinite_paired_value')
        else:
            delta = update['output_yaw']-expected
            error = abs(math.atan2(math.sin(delta),math.cos(delta)))
            if error > LIMIT_RAD: violations.append('paired_yaw_mismatch')
            if update['output_yawspeed'] != feedforward: violations.append('paired_feedforward_mismatch')
        status = 'contradicted' if violations else 'verified'
        statuses.append(status)
        pairs.append(dict(update_id=update['update_id'],source_id=identity,path=update['path'],
                          status=status,yaw_error_rad=error,violations=violations))
    status = next((s for s in ('contradicted','missing_required_evidence','unobservable_from_recording') if s in statuses), 'verified')
    return dict(contract=CONTRACT,status=status,required=True,pairs=pairs,
                observed_updates=len(updates),paired_updates=len(pairs),yaw_error_limit_rad=LIMIT_RAD,
                acquisition_completeness='requires_separate_qualified_acquisition')


def evidence_roles(report):
    """Called only after all existing wire, handover and initialization checks."""
    prior = report['prior_state_consistency']
    return dict(contract=CONTRACT,
        A_external_initialization=dict(status='verified',samples=report['initialization_wire_samples']),
        B_handover=dict(status='verified' if all(source and source.get('actual_callback_source_identity')=='verified'
            for source in (report['handover_position_source'],report['handover_attitude_source'])) else 'unobservable_from_recording',
                        position_source=report['handover_position_source'],
                        attitude_source=report['handover_attitude_source']),
        C_physical_behavior=dict(status='missing_required_evidence',
            initialization_drift_status='verified',max_initialization_drift_deg=report['max_initialization_drift_deg'],
            reason='Full mission windows and terminal checks are evaluated by the enclosing evaluator'),
        D_exact_internal_relation=unobservable(),
        E_prior_state_diagnostic=dict(status='contradicted' if prior['status']=='failed' else 'verified',
            required_for_exact_causality=False,original_policy=prior['policy'],
            prior_record_age_is_control_latency=False,measurement=prior))


def incomplete_roles(progress, reason):
    roles={key:progress.get(key,dict(status='missing_required_evidence',reason='Check not completed'))
        for key in ('A_external_initialization','B_handover','C_physical_behavior','E_prior_state_diagnostic')}
    active=progress.get('active_role')
    if active in roles:
        roles[active]={**roles[active], 'status':'missing_required_evidence' if 'missing' in reason.lower() or 'insufficient' in reason.lower()
            else 'contradicted', 'reason':reason}
    return dict(contract=CONTRACT,D_exact_internal_relation=unobservable(),**roles)
