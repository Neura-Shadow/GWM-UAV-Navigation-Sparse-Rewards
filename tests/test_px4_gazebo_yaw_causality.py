"""Synthetic execution traces; these are not observations from the pinned binary."""
import hashlib
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'simulation/px4_gazebo/validation'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'simulation/px4_gazebo/ros2_ws/src/gwm_px4_control'))
from yaw_provenance import exact_relation, CONTRACT


def trace():
    source = dict(source_id='a', run_id='run', binary_sha256='a'*64,
        topic='vehicle_local_position', instance=0, timestamp=100, timestamp_sample=96,
        generation=7, reference_generation={'heading_reset_counter': 2}, heading=.2,
        publication_order=1)
    update = dict(run_id='run', binary_sha256='a'*64, update_id=1,
        consumed_source_id='a', consumed_generation=7,
        reference_generation={'heading_reset_counter': 2}, copy_order=2, output_order=4,
        output_timestamp=108, active_yaw=None, active_yawspeed=0.,
        path='normal', update_valid=True, output_yaw=.2, output_yawspeed=0.)
    return [source], [update]


def evaluate(sources, updates):
    return exact_relation(sources, updates, 'run', 'a'*64)


def test_proven_input_selected_independently_of_output_and_intervening_publication():
    sources, updates = trace()
    sources.append({**sources[0], 'source_id':'b', 'timestamp':104,
                    'timestamp_sample':100, 'generation':8, 'heading':.8, 'publication_order':3})
    assert evaluate(sources, updates)['status'] == 'verified'
    updates[0]['output_yaw'] = .8
    result = evaluate(sources, updates)
    assert result['status'] == 'contradicted'
    assert result['pairs'][0]['source_id'] == 'a'


def test_same_hrt_input_and_output_does_not_erase_causal_pair():
    sources, updates = trace(); updates[0]['output_timestamp'] = 100
    assert evaluate(sources, updates)['status'] == 'verified'


@pytest.mark.parametrize('change', [{'consumed_source_id':None}, {'copy_order':None}])
def test_missing_identity_or_execution_order_is_unobservable_even_with_matching_candidate(change):
    sources, updates = trace(); updates[0].update(change)
    assert evaluate(sources, updates)['status'] == 'unobservable_from_recording'


def test_rate_limited_logger_cannot_supply_missing_consumed_input():
    sources, updates = trace(); updates[0]['consumed_source_id'] = 'not_logged'
    assert evaluate(sources, updates)['status'] == 'missing_required_evidence'


@pytest.mark.parametrize('field,value', [('output_yaw',.20002), ('output_yawspeed',.01),
    ('reference_generation',{'heading_reset_counter':3}), ('consumed_generation',8),
    ('output_order',1), ('binary_sha256','b'*64)])
def test_correct_pair_contradictions_are_never_hidden(field, value):
    sources, updates = trace(); updates[0][field] = value
    assert evaluate(sources, updates)['status'] == 'contradicted'


@pytest.mark.parametrize('path', ['normal','last_valid_fallback','generated_failsafe'])
def test_actual_active_input_and_path_must_be_recorded(path):
    sources, updates = trace(); updates[0].update(path=path,active_yaw=.6,output_yaw=.6)
    assert evaluate(sources, updates)['status'] == 'verified'
    updates[0]['output_yaw'] = .2
    assert evaluate(sources, updates)['status'] == 'contradicted'


def test_equal_time_distinct_updates_remain_separate_and_all_are_checked():
    sources, updates = trace(); updates.append({**updates[0], 'update_id':2,
        'copy_order':5,'output_order':6,'output_yaw':.3})
    result = evaluate(sources, updates)
    assert len(result['pairs']) == 2 and result['status'] == 'contradicted'
    assert result['contract'] == CONTRACT


def test_no_trace_is_not_an_empty_pass():
    assert evaluate([], [])['status'] == 'unobservable_from_recording'


def test_both_historical_proxy_failures_are_reproduced_without_new_causal_credit():
    from yaw_evidence import prior_state_yaw_consistency
    path=Path(__file__).with_name('fixtures')/'p3_r2_prior_yaw.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest()=='07cb7374706bbb042236081320b5c0245f9dfd221fef9e84bc82cfc04ddf38b6'
    data=json.loads(path.read_text())
    results=[]
    for case in data['cases']:
        result=prior_state_yaw_consistency(case['positions'],case['controls'],*case['window_us'],case['run_id'])
        assert result==case['expected']
        assert result['exact_mc_consumed_source_identity']=='unproven'
        assert result['match_age_limit_s']==.008001 and result['yaw_error_limit_rad']==1e-5
        results.append(result['failed_samples'])
    assert results==[79,68]
