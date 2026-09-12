"""Source identity and cache regressions with no ROS/runtime imports."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

PKG = Path(__file__).resolve().parents[1] / 'simulation/px4_gazebo/ros2_ws/src/gwm_px4_control'
sys.path.insert(0, str(PKG))
from gwm_px4_control import sample_identity as identity, timing, mission


def position(pub=97_568_000, sample=97_552_000, **changes):
    return dict(timestamp=pub, timestamp_sample=sample, x=1., y=2., z=-2.,
        vx=0., vy=0., vz=0., heading=.01, xy_valid=True, z_valid=True,
        v_xy_valid=True, v_z_valid=True, xy_reset_counter=0, z_reset_counter=0,
        vxy_reset_counter=0, vz_reset_counter=0, heading_reset_counter=1,
        ref_timestamp=1_000_000, **changes)


def attitude(pub=111_008_000, sample=110_988_000, q=None):
    return dict(timestamp=pub, timestamp_sample=sample,
                q=q or [1., 0., 0., 0.], quat_reset_counter=0)


def tracker():
    return identity.IdentityTracker('run-a')


def test_equal_publication_distinct_sample_preserves_both_source_ids():
    t = tracker()
    a = t.observe('vehicle_local_position', position(), 'd1', 100)
    b = t.observe('vehicle_local_position', {**position(sample=97_560_000), 'x': 1.01}, 'd2', 101)
    assert a['classification'] == 'distinct_publication'
    assert b['classification'] == 'distinct_same_publication'
    assert a['publication_us'] == b['publication_us'] == 97_568_000
    assert a['sample_us'] == 97_552_000 and b['sample_us'] == 97_560_000
    assert a['source_id'] != b['source_id']


def test_exact_repeat_retains_new_delivery_and_original_source_receipt():
    t = tracker()
    a = t.observe('vehicle_local_position', position(), 'd1', 100)
    b = t.observe('vehicle_local_position', position(), 'd2', 900)
    assert b['classification'] == 'duplicate_reuse'
    assert b['delivery_id'] == 'd2' and b['source_id'] == a['source_id']
    assert b['accepted_source_delivery_id'] == 'd1'
    assert b['accepted_source_receipt_monotonic_ns'] == 100
    assert b['receipt_monotonic_ns'] == 900


@pytest.mark.parametrize('bad', [None, True, 1.0, -1, 2**64, float('nan')])
def test_native_integer_timestamp_validation(bad):
    with pytest.raises(ValueError):
        identity.stamp_us(bad)


def test_uint64_precision_and_payload_types_are_not_float_coerced():
    high = 2**63+11
    a = identity.source_record('vehicle_local_position', position(high, high-1), 'run-a')
    assert a['publication_us'] == high and a['sample_us'] == high-1
    assert identity.stamp_us(2**64-1) == 2**64-1
    assert identity.payload_fingerprint({'value': high}) != identity.payload_fingerprint({'value': high+1})
    assert identity.payload_fingerprint({'value': 1}) != identity.payload_fingerprint({'value': 1.})


def test_nonfinite_identity_encoding_is_stable_strict_json():
    a = identity.source_record('vehicle_local_position', {**position(), 'limit': math.inf}, 'run-a')
    b = identity.source_record('vehicle_local_position', {**position(), 'limit': math.inf}, 'run-a')
    assert a == b
    json.dumps(a, allow_nan=False)
    assert identity.payload_fingerprint({'x': math.nan}) == identity.payload_fingerprint({'x': math.nan})
    assert identity.payload_fingerprint({'x': math.nan}) != identity.payload_fingerprint({'x': None})
    assert identity.payload_fingerprint({'x': -0.}) != identity.payload_fingerprint({'x': 0.})


@pytest.mark.parametrize('delta', [dict(x=9.), dict(heading_reset_counter=2)])
def test_same_time_and_sample_conflict_cannot_hide_value_or_reset(delta):
    t = tracker()
    t.observe('vehicle_local_position', position(), 'd1', 100)
    with pytest.raises(ValueError):
        t.observe('vehicle_local_position', {**position(), **delta}, 'd2', 101)
    # Rejection did not mutate accepted source.
    assert t.observe('vehicle_local_position', position(), 'd3', 102)['accepted_source_delivery_id'] == 'd1'


@pytest.mark.parametrize('pub,sample', [(97_567_000, 97_560_000),
    (97_569_000, 97_551_000), (97_569_000, 97_552_000),
    (97_568_000, 0), (97_568_000, 97_568_001)])
def test_regression_nonadvancing_or_invalid_sample_fails(pub, sample):
    t = tracker()
    t.observe('vehicle_local_position', position(), 'd1', 100)
    with pytest.raises(ValueError):
        t.observe('vehicle_local_position', position(pub, sample), 'd2', 101)


def test_missing_sample_cannot_fall_back_to_publication_clock():
    data = position()
    del data['timestamp_sample']
    with pytest.raises(ValueError):
        identity.source_record('vehicle_local_position', data, 'run-a')


def test_status_same_time_changed_health_is_ambiguous():
    t = tracker()
    t.observe('vehicle_status', dict(timestamp=100, failsafe=False), 'd1')
    with pytest.raises(ValueError, match='conflict|ambiguous'):
        t.observe('vehicle_status', dict(timestamp=100, failsafe=True), 'd2')


def test_all_pinned_state_message_versions_match_explicit_ros_metadata():
    expected = {'vehicle_local_position': 1, 'vehicle_attitude': 0,
        'vehicle_status': 1, 'vehicle_land_detected': 0,
        'estimator_status_flags': 0, 'failsafe_flags': 0}
    for topic, version in expected.items():
        data = position() if topic == 'vehicle_local_position' else attitude() if topic == 'vehicle_attitude' else dict(timestamp=100)
        implicit = identity.source_record(topic, data, 'run-a')
        explicit = identity.source_record(topic, data, 'run-a', version=version)
        assert implicit == explicit
        with pytest.raises(ValueError, match='version'):
            identity.source_record(topic, data, 'run-a', version=version+1)


def test_cross_run_or_topic_records_cannot_share_identity():
    a = identity.source_record('vehicle_local_position', position(), 'run-a')
    b = identity.source_record('vehicle_local_position', position(), 'run-b')
    assert a['source_id'] != b['source_id']
    with pytest.raises(ValueError):
        identity.classify(a, b)
    with pytest.raises(ValueError):
        identity.classify(a, identity.source_record('vehicle_attitude', attitude(), 'run-a'))


def test_duplicate_delivery_id_and_receipt_regression_are_rejected():
    t = tracker()
    t.observe('vehicle_local_position', position(), 'd1', 100)
    with pytest.raises(ValueError):
        t.observe('vehicle_local_position', position(), 'd1', 101)
    with pytest.raises(ValueError):
        t.observe('vehicle_local_position', position(), 'd2', 99)


def config():
    c = json.loads((PKG.parents[2]/'configs/p3_control.yaml').read_text())
    c['sample_evidence_contract'] = identity.CONTRACT
    return c


def ready_cache():
    c = timing.StateCache(config(), run_id='run-a')
    c.clock(10., 50.)
    rows = {
        'vehicle_local_position': position(10_000_000, 9_992_000),
        'vehicle_attitude': attitude(10_000_000, 9_996_000),
        'vehicle_status': dict(timestamp=10_000_000, arming_state=2, nav_state=14,
            system_id=72, component_id=1, failsafe=False, pre_flight_checks_pass=True),
        'vehicle_land_detected': dict(timestamp=10_000_000, landed=False),
        'estimator_status_flags': dict(timestamp=10_000_000, cs_tilt_align=True, cs_yaw_align=True),
        'failsafe_flags': dict(timestamp=10_000_000, local_position_invalid=False,
            local_altitude_invalid=False, attitude_invalid=False, angular_velocity_invalid=False)}
    for topic, data in rows.items():
        c.update(topic, data, 50.)
    return c


def test_cache_duplicate_does_not_refresh_freshness():
    c = ready_cache()
    data = copy.deepcopy(c.data['vehicle_local_position'])
    assert c.update('vehicle_local_position', data, 50.9) is False
    assert c.receipts['vehicle_local_position'] == 50.
    assert c.delivery_receipts['vehicle_local_position'] == 50.9
    assert c.stats['vehicle_local_position']['reuse'] == 1
    c.clock(10.01, 53.)
    with pytest.raises(ValueError, match='stale_state'):
        c.validate(10.01, 53.)


def test_cache_accepts_distinct_equal_pub_attitude_and_selection_refs_every_component():
    c = ready_cache()
    old = c.validate(10., 50.)
    q = [math.cos(.2), 0., 0., math.sin(.2)]
    assert c.update('vehicle_attitude', attitude(10_000_000, 9_998_000, q), 50.01)
    new = c.validate(10.01, 50.02)
    assert new['yaw'] == pytest.approx(.4)
    assert new['component_sources']['vehicle_local_position']['source_id'] == old['component_sources']['vehicle_local_position']['source_id']
    assert new['component_sources']['vehicle_attitude']['source_id'] != old['component_sources']['vehicle_attitude']['source_id']
    assert set(new['component_sources']) == set(timing.STATE_TOPICS)
    assert new['position_publication_us'] == 10_000_000
    assert new['position_sample_us'] == 9_992_000
    assert c.stats['vehicle_attitude']['same_publication_distinct'] == 1


def test_cache_equal_pub_reference_change_remains_visible_without_inspect_side_effect():
    c = ready_cache()
    class Owner:
        rejected = None
        def inspect(self, *args):
            raise AssertionError('Cache must not own reference reconciliation')
    c.reference_manager = Owner()
    c.frozen_reference = c.reference()
    p = {**c.data['vehicle_local_position'], 'timestamp_sample': 9_996_000,
         'heading_reset_counter': 2}
    c.update('vehicle_local_position', p, 50.01)
    with pytest.raises(ValueError, match='reference_reset'):
        c.validate(10.01, 50.02)


def test_cache_invalid_new_output_is_not_installed():
    c = ready_cache()
    previous = copy.deepcopy(c.data['vehicle_local_position'])
    with pytest.raises(ValueError):
        c.update('vehicle_local_position', {**previous, 'timestamp_sample': 10_000_001}, 50.01)
    assert c.data['vehicle_local_position'] == previous
    assert c.receipts['vehicle_local_position'] == 50.


def test_legacy_cache_behavior_remains_timestamp_only():
    c = timing.StateCache({})
    assert c.update('vehicle_attitude', dict(timestamp=100, q=[1.,0.,0.,0.]), 1.)
    assert c.update('vehicle_attitude', dict(timestamp=100, q=[0.,0.,0.,1.]), 2.) is False
    assert c.receipts['vehicle_attitude'] == 1.


def fixed_hover():
    c = ready_cache()
    control = mission.Mission(config(), c, 49., True)
    control.state = 'FINAL_HOVER'
    control.enter_sim, control.enter_wall = 9., 49.
    control.origin, control.initial_yaw = (1.,2.,0.), 0.
    control.target, control.yaw_target = (1.,2.,-2.), 0.
    control.handover = {'completed_sim_s': 9.}
    return c, control


def test_repeated_position_evaluations_add_no_window_coverage_or_count():
    c, control = fixed_hover()
    first = control.tick(10., 50., True)['sample']
    assert len(control.window) == 1
    for index in range(1, 4):
        selected = control.tick(10.+index*.02, 50.+index*.02, True)['sample']
        assert selected['source_id'] == first['source_id']
        assert selected['position_observation_reused'] is True
        assert selected['position_coverage_progress'] is False
    assert len(control.window) == 1 and not control.windows
    assert control.state == 'FINAL_HOVER'


def test_repeated_position_changed_unsafe_attitude_fails_live_fixed_window():
    c, control = fixed_hover()
    control.tick(10., 50., True)
    c.update('vehicle_attitude', attitude(10_000_000, 9_998_000,
        [math.cos(.2),0.,0.,math.sin(.2)]), 50.01)
    with pytest.raises(ValueError, match='fixed_dwell_window_failed:FINAL_HOVER'):
        control.tick(10.02, 50.02, True)
    assert control.control_selection['position_observation_reused'] is True
    assert control.control_selection['yaw'] == pytest.approx(.4)


def test_real_consumption_gap_remains_failure_after_identity_repair():
    c, control = fixed_hover()
    control.tick(10., 50., True)
    c.clock(10.201, 50.201)
    p = {**c.data['vehicle_local_position'], 'timestamp': 10_201_000,
         'timestamp_sample': 10_193_000}
    c.update('vehicle_local_position', p, 50.201)
    c.update('vehicle_attitude', attitude(10_201_000, 10_197_000), 50.201)
    with pytest.raises(ValueError, match='observation_gap'):
        control.tick(10.201, 50.201, True)


def test_equal_pub_distinct_bad_position_is_checked_without_coverage_progress():
    c, control = fixed_hover()
    control.tick(10., 50., True)
    p = {**c.data['vehicle_local_position'], 'timestamp_sample': 9_996_000, 'x': 1.3}
    c.update('vehicle_local_position', p, 50.01)
    with pytest.raises(ValueError, match='fixed_dwell_window_failed'):
        control.tick(10.01, 50.01, True)
    assert control.control_selection['position_observation_reused'] is False
    assert control.control_selection['position_coverage_progress'] is False
