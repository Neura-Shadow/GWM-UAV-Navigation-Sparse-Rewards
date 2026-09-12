"""Pure source binding and frozen prior-state yaw correlation; no runtime."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

SIM = Path(__file__).resolve().parents[1] / 'simulation/px4_gazebo'
sys.path.insert(0, str(SIM / 'ros2_ws/src/gwm_px4_control'))
sys.path.insert(0, str(SIM / 'validation'))
from gwm_px4_control.sample_identity import source_record
from yaw_evidence import handover_source, prior_state_yaw_consistency


def position(pub, sample, heading=0., **extra):
    return dict(timestamp=pub, timestamp_sample=sample, x=0., y=0., z=-2., heading=heading,
                heading_reset_counter=0, xy_reset_counter=0, z_reset_counter=0,
                vxy_reset_counter=0, vz_reset_counter=0, ref_timestamp=100000, **extra)


def control(pub, yaw=0., rate=0.):
    return dict(timestamp=pub, yaw=yaw, yawspeed=rate)


def test_historical_handover_requires_one_publication_candidate_without_overwrite():
    rows = [position(1000000, 980000, .1), position(1000000, 990000, .2)]
    with pytest.raises(ValueError, match='Ambiguous historical'):
        handover_source(rows, {'position_timestamp': 1000000}, 'vehicle_local_position')
    # Identical historical delivery copies are also unresolved by timestamp alone.
    with pytest.raises(ValueError, match='Ambiguous historical'):
        handover_source([rows[0], copy.deepcopy(rows[0])], {'position_timestamp': 1000000}, 'vehicle_local_position')


def test_new_handover_binds_composite_source_and_full_payload_instead_of_heading_closeness():
    rows = [position(1000000, 980000, .1), position(1000000, 990000, .2)]
    reference = source_record('vehicle_local_position', rows[0], 'run-a')
    handover = dict(position_timestamp=1000000, position_source=reference, aligned_heading=.2)
    selected, evidence = handover_source(rows, handover, 'vehicle_local_position', 'run-a', True)
    assert selected is rows[0]
    assert selected['heading'] == .1
    assert evidence['raw_candidates'] == 2
    assert evidence['matched_delivery_copies'] == 1


def test_new_handover_reports_equivalent_deliveries_without_dropping_raw_candidates():
    row = position(1000000, 990000, .1)
    handover = dict(position_timestamp=1000000, position_source=source_record('vehicle_local_position', row, 'run-a'))
    _, evidence = handover_source([row, copy.deepcopy(row)], handover, 'vehicle_local_position', 'run-a', True)
    assert evidence['raw_candidates'] == evidence['matched_delivery_copies'] == 2


@pytest.mark.parametrize('change', [dict(run_id='run-b'), dict(payload_sha256='0'*64),
                                    dict(sample_us=980000), dict(message_version=0)])
def test_changed_handover_identity_or_payload_is_rejected(change):
    row = position(1000000, 990000, .1)
    reference = source_record('vehicle_local_position', row, 'run-a')
    reference.update(change)
    with pytest.raises(ValueError):
        handover_source([row], dict(position_timestamp=1000000, position_source=reference),
                        'vehicle_local_position', 'run-a', True)


def test_new_handover_cannot_fall_back_to_timestamp_only_metadata():
    with pytest.raises(ValueError, match='Missing full handover'):
        handover_source([position(1000000, 990000)], {'position_timestamp': 1000000},
                        'vehicle_local_position', 'run-a', True)


def test_unique_historical_handover_remains_usable_with_explicit_identity_limit():
    row = position(1000000, 990000, .1)
    selected, evidence = handover_source([row], {'position_timestamp': 1000000}, 'vehicle_local_position')
    assert selected is row
    assert evidence['actual_callback_source_identity'] == 'not_recorded'


def test_attitude_handover_uses_its_own_topic_version_and_sample_reference():
    row = dict(timestamp=1000000, timestamp_sample=996000, q=[1.,0.,0.,0.], quat_reset_counter=2)
    reference = source_record('vehicle_attitude', row, 'run-a')
    selected, evidence = handover_source([row], dict(attitude_timestamp=1000000, attitude_source=reference),
                                        'vehicle_attitude', 'run-a', True)
    assert selected['quat_reset_counter'] == 2
    assert evidence['sample_us'] == 996000


def test_same_time_publication_is_not_assumed_to_precede_internal_control():
    rows = [position(992000, 988000, 0.), position(1000000, 996000, .02)]
    actual = prior_state_yaw_consistency(rows, [control(1000000)], 1000000, 1000001, 'run-a')
    assert actual['status'] == 'passed'
    assert actual['matches'][0]['position_publication_us'] == 992000
    assert actual['max_match_age_s'] == .008
    assert actual['exact_mc_consumed_source_identity'] == 'unproven'


def test_no_favorable_fallback_to_matching_same_time_heading():
    rows = [position(992000, 988000, 0.), position(1000000, 996000, .02)]
    actual = prior_state_yaw_consistency(rows, [control(1000000, .02)], 1000000, 1000001, 'run-a')
    assert actual['status'] == 'failed'
    assert actual['matches'][0]['position_publication_us'] == 992000
    assert actual['max_resolved_yaw_error_rad'] == pytest.approx(.02)


def test_prior_group_last_native_output_is_selected_without_matching_by_yaw():
    rows = [position(992000, 984000, .02), position(992000, 988000, 0.)]
    actual = prior_state_yaw_consistency(rows, [control(1000000, .02)], 1000000, 1000001, 'run-a')
    assert actual['status'] == 'failed'
    assert actual['matches'][0]['position_sample_us'] == 988000
    assert actual['matches'][0]['prior_heading'] == 0.


@pytest.mark.parametrize('age_us,expected', [(8001,'passed'), (8002,'failed')])
def test_original_internal_match_age_bound_is_preserved(age_us, expected):
    rows = [position(1000000-age_us, 980000)]
    actual = prior_state_yaw_consistency(rows, [control(1000000)], 1000000, 1000001, 'run-a')
    assert actual['status'] == expected
    assert actual['match_age_limit_s'] == .008001


@pytest.mark.parametrize('error,expected', [(9.9e-6,'passed'), (1.01e-5,'failed')])
def test_original_internal_yaw_error_bound_is_preserved(error, expected):
    actual = prior_state_yaw_consistency([position(992000, 988000)], [control(1000000,error)],
                                        1000000, 1000001, 'run-a')
    assert actual['status'] == expected
    assert actual['yaw_error_limit_rad'] == 1e-5


def test_failed_consistency_retains_every_internal_record_and_maximum():
    rows = [position(992000, 988000), position(1092000, 1088000), position(1192000, 1188000)]
    controls = [control(1000000,.01), control(1100000,.03), control(1200000,.02)]
    actual = prior_state_yaw_consistency(rows, controls, 1000000, 1200001, 'run-a')
    assert actual['internal_samples'] == actual['failed_samples'] == 3
    assert len(actual['matches']) == len(actual['violations']) == 3
    assert actual['max_resolved_yaw_error_rad'] == pytest.approx(.03)


def test_duplicate_internal_records_do_not_inflate_distinct_sample_coverage():
    row = control(1000000)
    actual = prior_state_yaw_consistency([position(992000,988000)], [row,copy.deepcopy(row)],
                                        1000000, 1000001, 'run-a')
    assert actual['internal_samples'] == 2
    assert actual['distinct_internal_observations'] == 1
    assert actual['duplicate_internal_deliveries'] == 1


def test_unknown_same_time_only_heading_is_explicitly_failed():
    actual = prior_state_yaw_consistency([position(1000000,996000)], [control(1000000)],
                                        1000000, 1000001, 'run-a')
    assert actual['status'] == 'failed'
    assert actual['violations'][0]['reason'] == 'missing_strictly_earlier_heading'


def test_nonfinite_internal_measurement_yields_a_strict_json_failure_report():
    actual = prior_state_yaw_consistency([position(992000,988000)], [control(1000000,math.nan)],
                                        1000000, 1000001, 'run-a')
    assert actual['status'] == 'failed'
    assert actual['matches'][0]['resolved_yaw_error_rad'] is None
    json.dumps(actual, allow_nan=False)
