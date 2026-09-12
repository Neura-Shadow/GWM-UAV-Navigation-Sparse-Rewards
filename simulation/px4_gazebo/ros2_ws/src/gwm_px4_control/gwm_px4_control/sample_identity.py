"""Pure, versioned source identity; no clocks, ROS imports or flight limits.

Publication timestamps group coverage. Native sample timestamps validate the
order of PX4 predictor outputs; neither timestamp alone identifies a delivery.
"""
from copy import deepcopy
import hashlib
import json
import math

CONTRACT = 'p3-sample-evidence-v2'
ESTIMATOR_TOPICS = ('vehicle_local_position', 'vehicle_attitude')
VERSIONS = {'vehicle_local_position': 1, 'vehicle_attitude': 0,
            'vehicle_status': 1, 'vehicle_land_detected': 0,
            'estimator_status_flags': 0, 'failsafe_flags': 0}
REFERENCE_FIELDS = {
    'vehicle_local_position': ('xy_reset_counter', 'z_reset_counter',
        'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter',
        'ref_timestamp'),
    'vehicle_attitude': ('quat_reset_counter',),
}


def stamp_us(value, field='timestamp'):
    """Accept an unchanged native uint64; floats cannot stand in for integers."""
    if type(value) is not int or not 0 <= value < 2**64:
        raise ValueError('invalid_integer_timestamp:' + field)
    return value


def _typed(value):
    if value is None:
        return ['null']
    if isinstance(value, bool):
        return ['bool', value]
    if isinstance(value, int):
        return ['int', str(value)]
    if isinstance(value, float):
        # Nonfinite source fields are data, never non-standard JSON tokens.
        # Canonical NaN is adequate for decoded payload equivalence; raw byte
        # artifacts retain any distinction between NaN payload bit patterns.
        return ['float', 'nan' if math.isnan(value) else value.hex()]
    if isinstance(value, str):
        return ['str', value]
    if isinstance(value, dict):
        if any(not isinstance(k, str) for k in value):
            raise ValueError('non_string_payload_key')
        return ['map', [[k, _typed(value[k])] for k in sorted(value)]]
    if hasattr(value, 'tolist'):
        return _typed(value.tolist())
    if isinstance(value, (list, tuple)) or hasattr(value, 'typecode'):
        return ['list', [_typed(v) for v in value]]
    raise ValueError('unsupported_identity_payload:' + type(value).__name__)


def payload_fingerprint(data):
    encoded = json.dumps(_typed(data), separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def source_record(topic, data, run_id, version=None, instance=0):
    if not isinstance(run_id, str) or not run_id or '/' in run_id or '\\' in run_id:
        raise ValueError('invalid_run_identity')
    if not isinstance(topic, str) or not topic or not isinstance(data, dict):
        raise ValueError('invalid_source_topic_or_payload')
    version = VERSIONS.get(topic, 0) if version is None else version
    if type(version) is not int or version < 0 or type(instance) is not int or instance < 0:
        raise ValueError('invalid_source_version_or_instance')
    if topic in VERSIONS and version != VERSIONS[topic]:
        raise ValueError('unsupported_estimator_message_version:' + topic)
    publication = stamp_us(data.get('timestamp'))
    sample = None
    if topic in ESTIMATOR_TOPICS:
        sample = stamp_us(data.get('timestamp_sample'), 'timestamp_sample')
        if sample == 0 or sample > publication:
            raise ValueError('invalid_sample_publication_order:' + topic)
    elif 'timestamp_sample' in data:
        sample = stamp_us(data['timestamp_sample'], 'timestamp_sample')
    reference = {k: stamp_us(data[k], k) for k in REFERENCE_FIELDS.get(topic, ()) if k in data}
    key = dict(contract=CONTRACT, run_id=run_id, topic=topic,
               message_version=version, instance=instance, publication_us=publication,
               sample_us=sample, reference_generation=reference)
    return {**key, 'payload_sha256': payload_fingerprint(data),
            'source_id': run_id + ':source:' + payload_fingerprint(key)}


def classify(previous, current):
    """Classify already constructed source metadata without reordering rows."""
    if previous is None:
        return 'distinct_publication'
    scope = ('contract', 'run_id', 'topic', 'message_version', 'instance')
    if any(previous[k] != current[k] for k in scope):
        raise ValueError('source_scope_mismatch')
    if current['publication_us'] < previous['publication_us']:
        raise ValueError('state_time_backwards:' + current['topic'])
    same_time = current['publication_us'] == previous['publication_us']
    if current['source_id'] == previous['source_id']:
        if current['payload_sha256'] != previous['payload_sha256']:
            raise ValueError('source_identity_payload_conflict:' + current['topic'])
        return 'duplicate_reuse'
    if current['topic'] in ESTIMATOR_TOPICS:
        if current['sample_us'] <= previous['sample_us']:
            raise ValueError('sample_time_nonadvancing_distinct:' + current['topic'])
    elif same_time:
        raise ValueError('ambiguous_equal_publication:' + current['topic'])
    return 'distinct_same_publication' if same_time else 'distinct_publication'


class IdentityTracker:
    """One accepted source per topic/instance, separate callback deliveries.

    The caller owns raw recording. This object retains only current source and
    last delivery metadata per topic/instance. Delivery IDs must be allocated
    monotonically by the owning caller; source ordering is validated here.
    """
    def __init__(self, run_id):
        if not isinstance(run_id, str) or not run_id or '/' in run_id or '\\' in run_id:
            raise ValueError('invalid_run_identity')
        self.run_id = run_id
        self.accepted = {}
        self.deliveries = {}

    def observe(self, topic, data, delivery_id, receipt_monotonic_ns=None,
                version=None, instance=0):
        candidate = source_record(topic, data, self.run_id, version, instance)
        scope = (topic, candidate['message_version'], instance)
        if not isinstance(delivery_id, str) or not delivery_id:
            raise ValueError('invalid_delivery_identity')
        if receipt_monotonic_ns is not None:
            stamp_us(receipt_monotonic_ns, 'receipt_monotonic_ns')
        prior_delivery = self.deliveries.get(scope)
        if prior_delivery is not None:
            if delivery_id == prior_delivery['delivery_id']:
                raise ValueError('repeated_delivery_identity')
            before = prior_delivery['receipt_monotonic_ns']
            if before is not None and receipt_monotonic_ns is not None and receipt_monotonic_ns < before:
                raise ValueError('receipt_time_backwards')
        previous = self.accepted.get(scope)
        kind = classify(previous, candidate)
        result = {**candidate, 'classification': kind, 'delivery_id': delivery_id,
                  'receipt_monotonic_ns': receipt_monotonic_ns,
                  'topic_delivery_ordinal': 1 if prior_delivery is None else prior_delivery['topic_delivery_ordinal']+1}
        if kind == 'duplicate_reuse':
            result['accepted_source_delivery_id'] = previous['accepted_source_delivery_id']
            result['accepted_source_receipt_monotonic_ns'] = previous['accepted_source_receipt_monotonic_ns']
            result['source_observation_ordinal'] = previous['source_observation_ordinal']
        else:
            result['accepted_source_delivery_id'] = delivery_id
            result['accepted_source_receipt_monotonic_ns'] = receipt_monotonic_ns
            result['source_observation_ordinal'] = 1 if previous is None else previous['source_observation_ordinal']+1
        # Publish accepted metadata only after every identity/order check passes.
        self.deliveries[scope] = deepcopy(result)
        if kind != 'duplicate_reuse':
            self.accepted[scope] = deepcopy(result)
        return result
