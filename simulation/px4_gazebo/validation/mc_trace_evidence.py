"""Source-bound, all-update validation of the MC's actual copied input and output.

The relationship is recorded within one invocation. No timestamp-nearest input
selection, producer order inferred from HRT, or favorable yaw matching is used.
"""
import ctypes as C
import math
from pathlib import Path
import struct
from collections import Counter
from mc_trace_schema import layouts, schema, WORDS
from p3_provenance import digest, strict_json

CONTRACT = 'px4-mc-causal-trace-v1'
LIMIT = 1e-5


def require(condition, reason):
    if not condition: raise ValueError('mc_trace:'+reason)


def bits(value): return struct.pack('<f', value)


def read_trace(run, summary):
    run = Path(run)
    receipt = strict_json(run/'mc-trace-build.json')
    require(receipt == summary['identity'].get('mc_trace'), 'build_identity')
    require(receipt.get('contract') == CONTRACT and receipt.get('status') == 'built', 'build_contract')
    require(receipt['binary_sha256'] == summary['identity']['binary_sha256'], 'binary_identity')
    headers = {}
    for name,value in receipt['generated_headers'].items():
        path = run/'mc-trace-abi'/name
        require(digest(path) == value, 'native_abi_hash')
        headers[name] = path.read_text()
    Header, Record = layouts(headers)
    raw = (run/'mc-trace.bin').read_bytes()
    require((run/'mc-trace.bin.closed').read_bytes() == b'closed-drained-fsynced\n', 'drain_marker')
    require(len(raw) >= C.sizeof(Header), 'short_header')
    header = Header.from_buffer_copy(raw)
    w = header.word
    require(w[0] == 0x314543415254434d and w[1] == 1, 'magic_version')
    require(w[2] == C.sizeof(Record) and w[3] == 200000, 'record_layout_capacity')
    require(w[10] == C.sizeof(Record._fields_[1][1]) and
            w[11] == C.sizeof(Record._fields_[2][1]._type_) and
            w[12] == C.sizeof(Record._fields_[7][1]), 'native_sizes')
    require(header.run_id.decode() == run.name == summary['run_id'], 'run_identity')
    require(header.overlay_id.decode() == receipt['overlay_id'], 'overlay_identity')
    require(w[8] == 0 and w[9] == 1 and w[13] == 0, 'overflow_unclosed_or_clock_error')
    require(0 < w[4] == w[5] <= w[3], 'copy_count')
    require(len(raw) == C.sizeof(Header)+w[5]*C.sizeof(Record), 'partial_or_extra_records')
    records = (Record*w[5]).from_buffer_copy(raw, C.sizeof(Header))
    return header, records, receipt, schema(headers)


def validate_records(header, records, config, flight=False, window=None):
    outputs = attempts = 0
    previous = None
    paths = Counter(); errors = []; violations = []
    durations = []; cpu = []; overhead = []; overhead_cpu = []; ages = []; gaps = []
    output_stamps = []; initialization = []
    for number, row in enumerate(records, 1):
        w = dict(zip(WORDS, row.word))
        require(w['sequence'] == number, 'missing_duplicate_or_reordered_sequence')
        require(w['begin_wall'] <= w['copy_wall'] <= w['end_wall'] and
                w['begin_cpu'] <= w['copy_cpu'] <= w['end_cpu'], 'monotonic_execution_order')
        require(0 < row.input.timestamp_sample <= row.input.timestamp <= w['begin_hrt'] <= w['end_hrt'], 'native_input_time')
        require(w['attempts'] <= 3 and w['published'] <= 1, 'bounded_attempts_outputs')
        if previous:
            require(0 < (w['generation']-previous['generation']) % 2**32 < 2**31, 'input_generation_order')
            require(w['begin_wall'] >= previous['end_wall'], 'overlapping_invocations')
            gaps.append((row.input.timestamp-previous['timestamp'])/1e6)
            require(gaps[-1] >= 0, 'input_publication_regression')
        previous = dict(w, timestamp=row.input.timestamp)
        durations.append((w['end_wall']-w['begin_wall'])/1e9)
        cpu.append((w['end_cpu']-w['begin_cpu'])/1e9)
        overhead.append(w['overhead_wall']/1e9); overhead_cpu.append(w['overhead_cpu']/1e9)
        attempts += w['attempts']; outputs += w['published']
        if not w['published']:
            require(w['attempts'] == 0, 'unpaired_attempt')
            continue
        require(w['flags'] & 1 and 1 <= w['attempts'] <= 3, 'inactive_output')
        require(bits(row.state_yaw) == bits(row.input.heading), 'state_yaw_differs_from_copied_heading')
        last_order = w['state_order']
        for index in range(w['attempts']):
            assignment, begin, end = row.attempt_order[index]
            path, valid, result = row.attempt_info[index]
            yaw, incoming, speed, dt = row.attempt_yaw[index]
            require(path in (1,2,3) and valid in (0,1) and result in (0,1), 'attempt_metadata')
            require(0 < assignment < begin < end and last_order < begin, 'attempt_causal_order')
            require(bits(yaw) == bits(row.input.heading), 'attempt_uses_different_state')
            require(.002-1e-8 <= dt <= .04+1e-8, 'dt_outside_pinned_controller_clamp')
            clock = row.attempt_clock[index]
            require(w['copy_wall'] <= clock[0] <= clock[1] <= w['end_wall'] and
                    w['copy_cpu'] <= clock[2] <= clock[3] <= w['end_cpu'], 'attempt_execution_clock')
            if index == 0:
                require(path == 1 and assignment < w['state_order'], 'initial_input_path')
            elif row.attempt_info[index-1][2]:
                require(False, 'attempt_after_success')
            if index == 2: require(path == 3, 'third_attempt_not_generated_failsafe')
            # A repeated update without a new setInputSetpoint can consume a yaw
            # already resolved by the failed prior update. The snapshot records it.
            if index == 0 or assignment != row.attempt_order[index-1][0]:
                require(bits(incoming) == bits(row.supplied[index].yaw) and
                        bits(speed) == bits(row.supplied[index].yawspeed), 'effective_input_assignment')
            paths[str(path)] += 1; last_order = end
        require(last_order < w['output_order'] <= w['events'], 'output_causal_order')
        require(row.input.timestamp <= row.output.timestamp <= w['end_hrt'], 'output_time')
        output_stamps.append(row.output.timestamp)
        ages.append((row.output.timestamp-row.input.timestamp)/1e6)
        n = w['attempts']-1
        incoming, speed = row.attempt_yaw[n][1:3]
        expected = incoming if math.isfinite(incoming) else row.input.heading
        expected_speed = speed if math.isfinite(speed) else 0.0
        valid = row.attempt_info[n][1] == 1 and row.attempt_info[n][2] == 1
        finite = all(math.isfinite(v) for v in (expected,expected_speed,row.output.yaw,row.output.yawspeed))
        error = abs(math.atan2(math.sin(row.output.yaw-expected),math.cos(row.output.yaw-expected))) if finite else None
        if error is not None: errors.append(error)
        if not valid or not finite or error > LIMIT or bits(row.output.yawspeed) != bits(expected_speed):
            violations.append(dict(update_id=number,reason='invalid_update_or_exact_yaw_relation',yaw_error_rad=error))
        if window and window[0] <= row.output.timestamp < window[1]:
            initialization.append(number)
            if math.isfinite(incoming) or expected_speed != 0:
                violations.append(dict(update_id=number,reason='initialization_effective_yaw_or_feedforward'))
    require(outputs == header.word[6] and attempts == header.word[7], 'final_count_reconciliation')
    if flight:
        require(outputs > 0 and initialization, 'flight_or_initialization_coverage_empty')
        require(output_stamps[0] <= window[0] and output_stamps[-1] >= window[1], 'initialization_window_not_bracketed')
    # Existing 50 ms wall dispatch limit and 0.2 s consumption limit remain
    # conservative additional bounds on the instrumented PX4 path.
    wall_limit = .05; gap_limit = config['max_sample_gap_sim_s']
    require(max(durations) <= wall_limit and max(overhead) <= wall_limit, 'existing_wall_budget_exceeded')
    require(max(gaps,default=0) <= gap_limit, 'existing_consumption_gap_exceeded')
    require(max(ages,default=0) <= config['position_fresh_sim_s'], 'existing_position_freshness_exceeded')
    return dict(contract=CONTRACT,status='contradicted' if violations else 'verified',required=True,
        acquisition_completeness='all_successful_MC_copies_and_all_attempts_until_owned_stop',
        consumed_topic='vehicle_local_position',consumed_instance=0,
        copied_updates=len(records),paired_updates=outputs,control_attempts=attempts,
        initialization_updates=len(initialization),first_update_id=1,last_update_id=len(records),
        paths=dict(paths),violations=violations,yaw_error_limit_rad=LIMIT,max_yaw_error_rad=max(errors,default=None),
        timing=dict(copy_through_output_max_wall_s=max(durations),copy_through_output_max_thread_cpu_s=max(cpu),
            instrumentation_measured_max_wall_s=max(overhead),instrumentation_measured_max_thread_cpu_s=max(overhead_cpu),
            instrumentation_measured_total_wall_s=sum(overhead),instrumentation_measured_total_thread_cpu_s=sum(overhead_cpu),
            instrumentation_span_limit='Clock-boundary and path-tag costs require the paired benchmark; not a full upper bound by themselves.',
            actual_input_to_output_age_max_sim_s=max(ages,default=None),consumed_input_gap_max_sim_s=max(gaps,default=None),
            wall_limit_s=wall_limit,consumed_gap_limit_sim_s=gap_limit))


def acquisition(run, summary):
    header, records, receipt, layout = read_trace(run,summary)
    report = validate_records(header,records,summary['config'])
    report.update(run_id=run.name,binary_sha256=receipt['binary_sha256'],overlay_id=receipt['overlay_id'],
        trace_sha256=digest(run/'mc-trace.bin'),trace_bytes=(run/'mc-trace.bin').stat().st_size,
        schema=layout,allocated_bytes=header.word[3]*header.word[2],initialization_wall_ns=header.word[14],
        initialization_cpu_ns=header.word[15],overflow=header.word[8],drained=True,fsynced=True)
    return report


def exact_flight(run, summary, log, result, ownership):
    header, records, receipt, layout = read_trace(run,summary)
    offboard = next(t['sim_s'] for t in result['transitions'] if t['to']=='VERIFY_OFFBOARD')
    window = (round(offboard*1e6),round(ownership['handover_start_sim_s']*1e6))
    report = validate_records(header,records,summary['config'],True,window)
    # ULog samples are corroboration of outputs, not the input pairing authority.
    # Group by exact output timestamp then require an exact raw payload match.
    observed = log.get_dataset('vehicle_local_position_setpoint').data
    grouped = {}
    for row in records:
        if row.word[WORDS.index('published')]:
            grouped.setdefault(row.output.timestamp,[]).append(row.output)
    matched = 0
    fields = [(key,kind) for key,kind in type(records[0].output)._fields_ if not key.startswith('_padding')]
    for index,timestamp in enumerate(observed['timestamp']):
        candidates = grouped.get(int(timestamp),[])
        def equal(candidate):
            for key,kind in fields:
                value = getattr(candidate,key)
                if isinstance(value,C.Array):
                    if any(bits(v)!=bits(float(observed[f'{key}[{j}]'][index])) for j,v in enumerate(value)): return False
                elif key == 'timestamp':
                    if value != int(timestamp): return False
                elif bits(value)!=bits(float(observed[key][index])): return False
            return True
        require(any(equal(c) for c in candidates), 'ulog_output_without_exact_trace_payload')
        matched += 1
    require(matched > 0,'no_ulog_output_corroboration')
    report.update(run_id=run.name,binary_sha256=receipt['binary_sha256'],overlay_id=receipt['overlay_id'],
        trace_sha256=digest(run/'mc-trace.bin'),ulog_outputs_corroborated=matched,
        selection='same_Run_copy_and_actual_update_entry_snapshot_before_inspecting_output',
        producer_order_claim=False,initialization_window_us=list(window))
    return report
