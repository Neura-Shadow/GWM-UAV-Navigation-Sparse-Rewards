"""Read-only, lossless P3 sample classification and causal evidence projections.

No sorting, interpolation, timestamp substitution, or favorable-value matching.
The fixed window clock is publication microseconds. See the versioned contract.
"""
from bisect import bisect_left
from collections import defaultdict
import hashlib
import json
import math

CONTRACT = "p3-sample-evidence-v2"
ESTIMATOR_TOPICS = {"vehicle_local_position": 1, "vehicle_attitude": 0}
TOPIC_VERSIONS = {**ESTIMATOR_TOPICS, 'vehicle_status': 1}
RESET_FIELDS = ("ref_timestamp", "xy_reset_counter", "z_reset_counter", "heading_reset_counter",
                "vxy_reset_counter", "vz_reset_counter", "quat_reset_counter")
META = {"bag_sim_ns", "timestamp_us", "timestamp_sample_us", "source_id", "record_id"}


def require(value, reason):
    if not value:
        raise ValueError(reason)


def integer_us(value, field="timestamp"):
    require(type(value) is int and value > 0, "invalid_integer_"+field)
    return value


def reference_generation(row):
    return {key: row[key] for key in RESET_FIELDS if key in row}


def controller_yaw_reconstruction(q):
    """Reproduce the frozen controller's norm-first quaternion decoding.

    This binds recorded metadata to its actual producer. It does not change the
    independent physical-window or sampled MC consistency calculations.
    """
    require(len(q) == 4 and all(type(v) in (int, float) and math.isfinite(v) for v in q), 'invalid_controller_quaternion')
    norm=math.sqrt(sum(float(value)*float(value) for value in q))
    require(.95 <= norm <= 1.05, 'invalid_controller_quaternion_norm')
    w,x,y,z=(float(value)/norm for value in q)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def _value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": "nan" if math.isnan(value) else "+inf" if value > 0 else "-inf"}
    if isinstance(value, dict):
        return {key: _value(item) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [_value(item) for item in value]
    return value


def payload(row):
    # ULog exposes arrays both flattened and reconstructed; retain one exact view.
    return {key: value for key, value in row.items() if not key.startswith("_") and key not in META
            and not ("[" in key and key.split("[")[0] in row)}


def fingerprint(value):
    return hashlib.sha256(json.dumps(_value(value), sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def identity_key(row, topic):
    from gwm_px4_control.sample_identity import stamp_us
    pub = stamp_us(row["timestamp"])
    sample = integer_us(row.get("timestamp_sample"), "timestamp_sample") if topic in ESTIMATOR_TOPICS else None
    if sample is not None:
        require(sample <= pub, "sample_after_publication:"+topic)
    return (topic, row.get('_message_version', TOPIC_VERSIONS.get(topic, 0)), row.get("_instance", 0), pub, sample,
            tuple(reference_generation(row).items()))


def validate_stream(rows, topic, run_id):
    """Classify every native record; never erase equal-time records or regressions."""
    from gwm_px4_control.sample_identity import source_record, classify
    require(isinstance(run_id, str) and run_id and run_id != "sensors", "invalid_run_id")
    output, seen, raw_ids = [], {}, set()
    last_pub = last_sample = previous_source = None
    counts = dict(raw_records=len(rows), distinct_observations=0, duplicate_reuses=0,
                  equal_publication_distinct_outputs=0, covered_publication_times=0)
    for ordinal, row in enumerate(rows, 1):
        require(row.get("_run_id", run_id) == run_id, "cross_run_record:"+topic)
        if "_record_id" in row:
            require(row["_record_id"] not in raw_ids, "duplicate_native_record_id:"+topic)
            raw_ids.add(row["_record_id"])
        key = identity_key(row, topic)
        source = source_record(topic, payload(row), run_id, version=key[1], instance=key[2])
        # This is the same identity/order contract used by the online cache.
        # The additional full-history checks below detect nonadjacent reuse.
        pub, sample = key[3:5]
        require(last_pub is None or pub >= last_pub, "publication_time_regression:"+topic)
        body = fingerprint(payload(row))
        duplicate = key in seen
        if duplicate:
            require(seen[key] == body, "conflicting_source_identity:"+topic)
            require(sample is None or last_sample is None or sample >= last_sample,
                    "source_selection_sample_regression:"+topic)
            counts["duplicate_reuses"] += 1
            classification = "duplicate_reuse"
        else:
            if sample is not None:
                require(last_sample is None or sample > last_sample, "sample_time_nonprogress:"+topic)
            elif last_pub == pub:
                raise ValueError("ambiguous_same_publication_state:"+topic)
            seen[key] = body
            counts["distinct_observations"] += 1
            classification = "distinct_same_publication" if pub == last_pub else "distinct_publication"
            counts["equal_publication_distinct_outputs"] += pub == last_pub
            last_sample = sample
        if last_pub != pub:
            counts["covered_publication_times"] += 1
        classify(previous_source, source)
        record = dict(row)
        record["_evidence"] = dict(run_id=run_id, topic=topic, message_version=key[1], instance=key[2],
            native_ordinal=ordinal, record_id=row.get("_record_id", f"{run_id}:{topic}:record:{ordinal}"),
            source_id=source["source_id"], timestamp_us=pub, timestamp_sample_us=sample,
            reference_generation=reference_generation(row), payload_sha256=source['payload_sha256'], classification=classification)
        output.append(record)
        last_pub = pub
        previous_source = source
    return {"records": output, "statistics": counts}


def _equal(left, right):
    if isinstance(left, (float, int, bool)) and isinstance(right, (float, int, bool)):
        return left == right or (isinstance(left, float) and isinstance(right, float)
                                 and math.isnan(left) and math.isnan(right))
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(_equal(a, b) for a, b in zip(left, right))
    return left == right


def exact_overlap(left, right, topic, run_id):
    """Match exact common payloads by composite identity, with explicit rate limits.

    Several identical right records are equivalent delivery copies; retain and
    report them. Distinct conflicting candidates are rejected, never selected.
    """
    a = validate_stream(left, topic, run_id)
    b = validate_stream(right, topic, run_id)
    index = defaultdict(list)
    for ordinal, row in enumerate(b["records"]):
        index[identity_key(row, topic)].append((ordinal, row))
    used, pairs, unmatched, reused, equivalent = set(), 0, 0, 0, 0
    for row in a["records"]:
        candidates = index.get(identity_key(row, topic), [])
        if not candidates:
            unmatched += 1
            continue
        for _, candidate in candidates:
            common = payload(row).keys() & payload(candidate).keys()
            required = {"timestamp", "timestamp_sample", "x", "y", "z"} if topic == "vehicle_local_position" else {"timestamp", "timestamp_sample", "q"}
            require(required <= common, "missing_exact_overlap_payload:"+topic)
            require(all(_equal(row[key], candidate[key]) for key in common),
                    "ambiguous_or_conflicting_exact_overlap:"+topic)
        source = identity_key(row, topic)
        reused += source in used
        used.add(source)
        equivalent += max(0, len(candidates)-1)
        pairs += 1
    matched_right = sum(identity_key(row, topic) in used for row in b["records"])
    return dict(status="passed", left=a["statistics"], right=b["statistics"], exact_payload_matches=pairs,
                unmatched_left=unmatched, unmatched_right=len(right)-matched_right, ambiguous_matches=0,
                reused_matches=reused, equivalent_candidate_deliveries=equivalent,
                coverage_limit="ROS DDS is rate limited; unmatched ULog outputs do not establish ROS packet loss or full parity.")


def causal_asof(rows, timestamp_us, freshness_s, topic):
    """Latest strictly earlier group; validated estimator order resolves its tie."""
    require(rows and all("_evidence" in row for row in rows), "causal_join_requires_validated_native_records")
    require(len({row["_evidence"]["run_id"] for row in rows}) == 1, "cross_run_causal_join")
    require(all(row["_evidence"]["topic"] == topic for row in rows), "cross_topic_causal_join")
    times = [row["timestamp"] for row in rows]
    require(all(b >= a for a, b in zip(times, times[1:])), "causal_join_input_regression")
    index = bisect_left(times, timestamp_us)-1
    require(index >= 0, "missing_strictly_earlier_state:"+topic)
    selected = rows[index]
    require(0 < timestamp_us-selected["timestamp"] <= round(freshness_s*1e6), "causal_state_gap:"+topic)
    return selected


def joined_samples_v2(streams, config, start_us, end_us, run_id):
    from collect_p2_evidence import yaw
    validated = {topic: validate_stream(rows, topic, run_id)["records"] for topic, rows in streams.items()
                 if topic in ("vehicle_local_position", "vehicle_attitude", "vehicle_status", "vehicle_land_detected",
                              "estimator_status_flags", "failsafe_flags")}
    output = []
    for p in validated["vehicle_local_position"]:
        stamp = p["timestamp"]
        if not start_us <= stamp <= end_us:
            continue
        a = causal_asof(validated["vehicle_attitude"], stamp, config["attitude_fresh_sim_s"], "vehicle_attitude")
        s = causal_asof(validated["vehicle_status"], stamp, config["flags_fresh_sim_s"], "vehicle_status")
        land = causal_asof(validated["vehicle_land_detected"], stamp, config["flags_fresh_sim_s"], "vehicle_land_detected")
        estimator = causal_asof(validated['estimator_status_flags'], stamp, config['flags_fresh_sim_s'], 'estimator_status_flags')
        flags = causal_asof(validated['failsafe_flags'], stamp, config['flags_fresh_sim_s'], 'failsafe_flags')
        require(all(p[key] for key in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")), "invalid_recorded_estimate")
        e = p["_evidence"]
        output.append(dict(t=stamp/1e6, timestamp_us=stamp, timestamp_sample_us=p["timestamp_sample"],
            source_id=e["source_id"], record_id=e["record_id"], reference_generation=e["reference_generation"],
            position=[p[key] for key in ("x", "y", "z")], yaw=yaw(a["q"]),
            nav_state=int(s["nav_state"]), arming_state=int(s["arming_state"]), landed=bool(land["landed"]),
            health_valid=not bool(s.get("failsafe", False)) and bool(s['pre_flight_checks_pass'])
                and bool(estimator['cs_tilt_align']) and bool(estimator['cs_yaw_align'])
                and not any(flags[k] for k in ('local_position_invalid','local_altitude_invalid','attitude_invalid','angular_velocity_invalid')),
            component_sources={name: row["_evidence"] for name, row in (("position", p), ("attitude", a), ("status", s), ("land", land),
                                                                       ('estimator_status_flags',estimator),('failsafe_flags',flags))}))
    return output


def all_record_window_checks(streams, start_us, end_us, target, config):
    """Inspect all native component records, including those between projections."""
    from collect_p2_evidence import yaw
    counts, maximum_yaw = {}, 0.0
    for topic in ("vehicle_local_position", "vehicle_attitude", "vehicle_status", "vehicle_land_detected",
                  "estimator_status_flags", "failsafe_flags"):
        rows = [r for r in streams.get(topic, []) if start_us <= r["timestamp"] <= end_us]
        require(rows, "missing_native_window_component:"+topic)
        counts[topic] = len(rows)
        generations = [reference_generation(row) for row in rows]
        require(all(g == generations[0] for g in generations), "native_window_reference_reset:"+topic)
        for row in rows:
            if topic == "vehicle_local_position":
                require(all(row[k] for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")), "native_invalid_estimate")
                require(all(math.isfinite(row[k]) for k in ("x", "y", "z", "vx", "vy", "vz", "heading")), "native_nonfinite_estimate")
                require(math.hypot(row["x"]-target["position"][0], row["y"]-target["position"][1]) <= config["horizontal_tolerance_m"]
                        and abs(row["z"]-target["position"][2]) <= config["height_tolerance_m"], "native_position_tracking")
                error = abs(math.atan2(math.sin(row["heading"]-target["yaw"]), math.cos(row["heading"]-target["yaw"])))
                require(error <= math.radians(config["yaw_tolerance_deg"]), "native_heading_tracking")
            elif topic == "vehicle_attitude":
                error = abs(math.atan2(math.sin(yaw(row["q"])-target["yaw"]), math.cos(yaw(row["q"])-target["yaw"])))
                maximum_yaw = max(maximum_yaw, math.degrees(error))
                require(error <= math.radians(config["yaw_tolerance_deg"]), "native_attitude_tracking")
            elif topic == "vehicle_status":
                require(row["nav_state"] == 14 and row["arming_state"] == 2 and not row["failsafe"]
                        and row["pre_flight_checks_pass"], "native_mode_arm_health")
            elif topic == "vehicle_land_detected":
                require(not row["landed"], "native_landed_in_flight_window")
            elif topic == "estimator_status_flags":
                require(row["cs_tilt_align"] and row["cs_yaw_align"], "native_alignment_invalid")
            elif topic == "failsafe_flags":
                require(not any(row[k] for k in ("local_position_invalid", "local_altitude_invalid", "attitude_invalid",
                                                 "angular_velocity_invalid")), "native_failsafe_flags")
    return dict(status="passed", all_native_record_counts=counts, max_native_attitude_error_deg=maximum_yaw)


def reconstruct_controller_samples(events, config, run_id, historical=False):
    """Reconstruct available per-component state from callback order, not as-of.

    Historical mode models the recorded original publication-only cache. New
    mode models the frozen identity classifier. Neither trusts a fresh flag.
    """
    from collect_p2_evidence import yaw
    from gwm_px4_control.sample_identity import stamp_us
    topics = ("vehicle_local_position", "vehicle_attitude", "vehicle_status", "vehicle_land_detected",
              "estimator_status_flags", "failsafe_flags")
    raw, delivery_rows = defaultdict(list), {}
    action_samples, canonical_samples, source_ledger = {}, {}, {}
    if not historical:
        for event in events:
            if event.get('event') == 'source_delivery':
                reference = event['source']
                require(reference['delivery_id'] not in source_ledger, 'duplicate_delivery_reference')
                source_ledger[reference['delivery_id']] = reference
            if event.get('event') == 'sample':
                identity = event['sample']['selection_id']
                require(identity not in action_samples, 'duplicate_action_sample_selection')
                action_samples[identity] = event['sample']
    for ordinal, event in enumerate(events, 1):
        if event.get("event") == "received" and event.get("topic_key") in topics:
            require(event.get("run_id", run_id) == run_id, "cross_run_callback")
            row = dict(event["fields"], _record_id=f"{run_id}:callback:{ordinal}", _run_id=run_id)
            row["_event_ordinal"] = ordinal
            row["_callback_monotonic_s"] = event["monotonic_s"]
            row['_message_version'] = event.get('message_version', TOPIC_VERSIONS.get(event['topic_key'], 0))
            row["_delivery_id"] = event.get("delivery_id", row["_record_id"])
            row["_callback_monotonic_ns"] = event.get("callback_entry_monotonic_ns")
            raw[event["topic_key"]].append(row)
    for topic in topics:
        classified = validate_stream(raw[topic], topic, run_id)
        for row in classified["records"]:
            delivery_rows[row["_event_ordinal"]] = row
    selected, output, hidden = {}, [], 0
    evaluations, nonselections, errors, evaluation_ids = 0, 0, [], set()
    last_evaluation = None
    last_evaluation_entry_ns = None
    for ordinal, event in enumerate(events, 1):
        if ordinal in delivery_rows:
            row = delivery_rows[ordinal]
            topic = event["topic_key"]
            previous = selected.get(topic)
            if historical:
                fresh = previous is None or row["timestamp"] > previous["timestamp"]
                hidden += not fresh and row["_evidence"]["classification"] == "distinct_same_publication"
            else:
                fresh = row["_evidence"]["classification"] != "duplicate_reuse"
            if fresh:
                selected[topic] = row
        event_type = "sample" if historical else "control_evaluation"
        if event.get("event") != event_type:
            continue
        evaluations += 1
        if not historical:
            identity = event['evaluation_id']
            require(identity not in evaluation_ids and identity == f'{run_id}:evaluation:{evaluations}', 'duplicate_or_cross_run_evaluation')
            evaluation_ids.add(identity)
            require(event['run_id'] == run_id and event['selection_id'] == identity, 'evaluation_identity_mismatch')
            stamp_us(event['evaluation_entry_monotonic_ns'],'evaluation_entry_monotonic_ns')
            stamp_us(event['evaluation_return_monotonic_ns'],'evaluation_return_monotonic_ns')
            require(event['evaluation_return_monotonic_ns'] >= event['evaluation_entry_monotonic_ns'], 'evaluation_clock_regression')
            require(last_evaluation_entry_ns is None or event['evaluation_entry_monotonic_ns'] > last_evaluation_entry_ns,
                    'evaluation_entry_clock_regression')
            last_evaluation_entry_ns = event['evaluation_entry_monotonic_ns']
            require(event.get('sample_evidence_contract') == CONTRACT, 'evaluation_sample_contract')
            for topic, data in selected.items():
                require(data['_callback_monotonic_ns'] is not None
                        and data['_callback_monotonic_ns'] <= event['evaluation_entry_monotonic_ns'], 'future_component_callback:'+topic)
                reference = event['component_sources'][topic]
                require(reference['run_id'] == run_id and reference['topic'] == topic
                        and reference['source_id'] == data['_evidence']['source_id']
                        and reference['accepted_source_delivery_id'] == data['_delivery_id'], 'evaluation_component_mismatch:'+topic)
                require(reference == source_ledger.get(data['_delivery_id']), 'evaluation_full_component_reference:'+topic)
            if event.get('error'):
                errors.append(dict(evaluation_id=identity, error=event['error'], phase=event.get('evaluated_phase')))
            if event.get('sample') is None:
                nonselections += 1
                continue
        require(all(topic in selected for topic in topics), "selection_without_all_components")
        require(event.get("run_id", run_id) == run_id, "cross_run_selection")
        original = event["sample"]
        if not historical:
            require(original['selection_id'] == event['selection_id'] and original['evaluation_id'] == event['evaluation_id'],
                    'canonical_sample_evaluation_identity')
            canonical_samples[original['selection_id']] = original
        p, a, status, land = (selected[name] for name in topics[:4])
        stamp = p["timestamp"]
        require(round(original["t"]*1e6) == stamp, "recorded_selection_source_timestamp_mismatch")
        require(all(abs(original["position"][i]-p[key]) < 1e-12 for i, key in enumerate(("x", "y", "z"))),
                "recorded_selection_position_mismatch")
        reconstructed_yaw=controller_yaw_reconstruction(a['q'])
        require(abs(math.atan2(math.sin(original["yaw"]-reconstructed_yaw), math.cos(original["yaw"]-reconstructed_yaw))) < 1e-12,
                "recorded_selection_attitude_mismatch")
        require(original["nav_state"] == status["nav_state"] and original["arming_state"] == status["arming_state"]
                and original["landed"] == bool(land["landed"]), "recorded_selection_mode_mismatch")
        evaluation = original.get("selection_monotonic_s", original["receipt_monotonic_s"])
        require(last_evaluation is None or evaluation > last_evaluation, "selection_monotonic_time_nonprogress")
        require(all(row["_callback_monotonic_s"] <= evaluation for row in selected.values()), "future_callback_in_selection")
        require(abs(original["source_callback_entry_monotonic_s"]-p["_callback_monotonic_s"]) < 1e-9,
                "recorded_selection_callback_mismatch")
        row = dict(original)
        metadata = p["_evidence"]
        if not historical:
            require(original.get("timestamp_us") == stamp and original.get("timestamp_sample_us") == p["timestamp_sample"]
                    and original.get("source_id") == metadata["source_id"], "recorded_position_source_identity_mismatch")
            for topic, data in selected.items():
                reference = original["component_sources"][topic]
                require(reference["run_id"] == run_id and reference["source_id"] == data["_evidence"]["source_id"]
                        and reference["accepted_source_delivery_id"] == data["_delivery_id"]
                        and reference["accepted_source_receipt_monotonic_ns"] == data["_callback_monotonic_ns"],
                        "recorded_component_source_reference_mismatch:"+topic)
        row.update(timestamp_us=stamp, timestamp_sample_us=p["timestamp_sample"], source_id=metadata["source_id"],
                   reference_generation=metadata["reference_generation"],
                   selection_id=f"{run_id}:historical-reconstructed-selection:{ordinal}" if historical else original['selection_id'],
                   analysis_record_id=f'{run_id}:analysis:{ordinal}',
                   component_sources={topic: data["_evidence"] for topic, data in selected.items()})
        flags, estimator = selected["failsafe_flags"], selected["estimator_status_flags"]
        row["health_valid"] = (not status["failsafe"] and bool(status["pre_flight_checks_pass"])
            and all(p[k] for k in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"))
            and bool(estimator["cs_tilt_align"]) and bool(estimator["cs_yaw_align"])
            and not any(flags[k] for k in ("local_position_invalid", "local_altitude_invalid", "attitude_invalid", "angular_velocity_invalid")))
        if not historical:
            # Every selected evaluation is checked, including transit and reuse
            # outside the fixed tracking windows. Match the existing terminal
            # exception using the phase captured before Mission.tick.
            terminal = (config.get('reference_policy') in ('p2-estimator-reference-v2', 'p2-estimator-reference-v3')
                        and event.get('evaluated_phase') == 'VERIFY_LANDED_AND_DISARMED'
                        and status['arming_state'] == 1 and land['landed'] is True)
            row['health_valid'] = (status['failsafe'] is False
                and (terminal or status['pre_flight_checks_pass'] is True)
                and all(p[k] is True for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid'))
                and estimator['cs_tilt_align'] is True and estimator['cs_yaw_align'] is True
                and all(flags[k] is False for k in ('local_position_invalid', 'local_altitude_invalid',
                                                   'attitude_invalid', 'angular_velocity_invalid')))
            require(row['health_valid'], 'selected_evaluation_health_invalid')
            require(status.get('system_id') == config['vehicle_system']
                    and status.get('component_id') == config['vehicle_component'], 'selected_evaluation_vehicle_identity')
            require(all(type(p[k]) in (int, float) and math.isfinite(p[k])
                        for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')), 'selected_evaluation_nonfinite_estimate')
            require(all(type(value) in (int, float) and math.isfinite(value)
                        for value in (*original['position'], *original['velocity'], original['yaw'])),
                    'selected_evaluation_nonfinite_sample')
            require(len(original['velocity']) == 3
                    and all(abs(original['velocity'][i]-p[key]) < 1e-12 for i, key in enumerate(('vx', 'vy', 'vz'))),
                    'recorded_selection_velocity_mismatch')
        for topic, data in selected.items():
            limit = config["position_fresh_sim_s"] if topic == "vehicle_local_position" else config["attitude_fresh_sim_s"] if topic == "vehicle_attitude" else config["flags_fresh_sim_s"]
            age = original["ros_sim_s"]-data["timestamp"]/1e6
            require(-config["clock_offset_tolerance_s"] <= age <= limit, "selection_source_stale:"+topic)
            require(evaluation-data["_callback_monotonic_s"] <= config["state_fresh_wall_s"], "selection_receipt_stale:"+topic)
        output.append(row)
        last_evaluation = evaluation
    if not historical:
        require(all(identity in canonical_samples and sample == canonical_samples[identity]
                    for identity, sample in action_samples.items()), 'action_sample_canonical_evaluation_mismatch')
    return dict(samples=output, statistics=dict(raw_callbacks=sum(map(len, raw.values())), controller_evaluations=evaluations,
        selected_evaluations=len(output),nonselected_evaluations=nonselections,evaluation_errors=errors,
        action_sample_events=len(action_samples),canonical_sample_bindings_verified=not historical,
        distinct_position_observations=len({row["source_id"] for row in output}),
        reused_position_evaluations=len(output)-len({row["source_id"] for row in output}),
        original_cache_hidden_same_publication_outputs=hidden, reconstruction="original_cache" if historical else CONTRACT))


def verify_source_deliveries(streams, events, run_id):
    """Recompute runtime metadata from bag payloads, independently of JSON claims."""
    from gwm_px4_control.sample_identity import IdentityTracker
    tracker = IdentityTracker(run_id)
    deliveries, positions, ledger = [], defaultdict(int), {}
    for event in events:
        if event.get('event') in ('source_delivery', 'source_delivery_rejected'):
            require(event['event'] != 'source_delivery_rejected', 'rejected_source_delivery')
            source = event['source']
            require(event.get('sample_evidence_contract') == CONTRACT and event['topic_key'] == source['topic'],
                    'source_delivery_envelope_contract_or_topic')
            identity = source['delivery_id']
            require(identity not in ledger and event.get('run_id') == run_id, 'duplicate_or_cross_run_delivery_ledger')
            ledger[identity] = source
    global_ordinal = 0
    topics = ('vehicle_local_position','vehicle_attitude','vehicle_status','vehicle_land_detected','estimator_status_flags','failsafe_flags')
    for event in events:
        if event.get('event') != 'received':
            continue
        global_ordinal += 1
        topic = event['topic_key']
        positions[topic] += 1
        require(event.get('run_id') == run_id and event.get('sample_evidence_contract') == CONTRACT,
                'callback_run_contract')
        require(event['delivery_id'] == f'{run_id}:delivery:{global_ordinal}'
                and event['delivery_ordinal'] == global_ordinal and event['topic_delivery_ordinal'] == positions[topic],
                'callback_delivery_ordinal')
        if topic not in topics:
            continue
        require(positions[topic] <= len(streams[topic]), 'missing_raw_bag_delivery')
        raw = streams[topic][positions[topic]-1]
        source = tracker.observe(topic,payload(raw),event['delivery_id'],event['callback_entry_monotonic_ns'],
                                 event['message_version'],event['uorb_instance'])
        require(ledger.get(event['delivery_id']) == source, 'source_delivery_metadata_or_payload_mismatch:'+topic)
        deliveries.append(event['delivery_id'])
    require(set(deliveries) == set(ledger), 'missing_or_extra_source_delivery_ledger')
    return dict(status='passed',raw_callback_deliveries=global_ordinal,verified_source_deliveries=len(deliveries),
                unique_delivery_ids=len(set(deliveries)),per_topic_callbacks=dict(positions))
