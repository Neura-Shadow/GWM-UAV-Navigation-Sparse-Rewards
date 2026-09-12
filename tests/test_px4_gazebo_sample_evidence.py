"""Recorded P3-R1 numeric regressions and pure sample-evidence contracts.

No ROS node, simulator, playback, raw bag, or ULog is opened by this suite.
The frozen fixture is a complete numeric excerpt of the two historical windows;
reanalysis never gives that historical flight new qualification credit.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "simulation/px4_gazebo/ros2_ws/src/gwm_px4_control"
VALIDATION = ROOT / "simulation/px4_gazebo/validation"
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(VALIDATION))
from gwm_px4_control import acceptance

FIXTURE = Path(__file__).with_name("fixtures") / "p3_r1_historical_sample_windows.json"
FIXTURE_SHA256 = "05ce269f55eeb435bb1a48a8b73aa1ef9f21e1741428f6f0465c3b42476f4016"


def recorded():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def window_v2(name):
    """Attach identity from recorded integer source references, not float time."""
    fixture = recorded()
    window = fixture[name]
    rows = []
    for source in window["samples"]:
        row = copy.deepcopy(source)
        row["timestamp_us"] = source["source_timestamp_us"]
        row["source_id"] = source.get("source_callback_id", source.get("record_id"))
        row["reference_generation"] = source["source_reset_generation"]
        rows.append(row)
    return rows, window["target"], window["duration_s"], fixture["config"]


def test_historical_numeric_excerpt_bytes_and_attribution_are_immutable():
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == FIXTURE_SHA256
    fixture = recorded()
    assert fixture["historical_run_id"] == "20260912T113430Z-p3-flight-0dc41a18"
    assert fixture["source_evaluator_commit"] == "76f014d"
    assert fixture["source_evaluator_sha256"] == (
        "a6acfd2da9e7a07a8f75d1b294f3382b3abd669842fe77d655d8a63b33e18cbf"
    )
    assert "not fresh runtime acceptance" in fixture["purpose"]


@pytest.mark.parametrize("name,count", [("FINAL_HOVER", 165), ("RESTORE_INITIAL_YAW", 625)])
def test_original_strict_evaluator_retains_both_historical_failures(name, count):
    fixture = recorded()
    window = fixture[name]
    assert len(window["samples"]) == count
    actual = acceptance.evaluate_window(
        window["samples"], window["target"], window["duration_s"], fixture["config"]
    )
    assert actual == window["original_strict_result"]
    assert actual["status"] == "failed"


def test_recorded_final_hover_reuses_exact_source_callbacks_and_position():
    rows = recorded()["FINAL_HOVER"]["samples"]
    pairs = [(a, b) for a, b in zip(rows, rows[1:])
             if a["source_timestamp_us"] == b["source_timestamp_us"]]
    assert [a["source_timestamp_us"] for a, _ in pairs] == [102516000, 104464000, 104936000]
    assert len({row["source_callback_id"] for row in rows}) == 162
    for first, second in pairs:
        assert first["source_callback_id"] == second["source_callback_id"]
        assert first["source_callback_entry_monotonic_s"] == second["source_callback_entry_monotonic_s"]
        assert first["position"] == second["position"]
        assert first["timestamp_sample_us"] == second["timestamp_sample_us"]
        assert first["selection_monotonic_s"] < second["selection_monotonic_s"]
        assert first["yaw"] != second["yaw"]  # Other components really changed.


def test_recorded_ulog_collision_preserves_distinct_outputs_in_native_order():
    rows = recorded()["RESTORE_INITIAL_YAW"]["samples"]
    pair = [r for r in rows if r["source_timestamp_us"] == 97568000]
    assert [r["timestamp_sample_us"] for r in pair] == [97552000, 97560000]
    assert [r["native_dataset_ordinal"] for r in pair] == [11687, 11688]
    assert pair[0]["native_file_ordinal"] < pair[1]["native_file_ordinal"]
    assert pair[0]["position"] != pair[1]["position"]
    assert pair[0]["source_reset_generation"] == pair[1]["source_reset_generation"]
    assert len({r["record_id"] for r in rows}) == 625
    assert len({r["source_timestamp_us"] for r in rows}) == 624


@pytest.mark.parametrize("name,raw,distinct,reused,covered", [
    ("FINAL_HOVER", 165, 162, 3, 162),
    ("RESTORE_INITIAL_YAW", 625, 625, 0, 624),
])
def test_revised_historical_views_classify_all_records_without_changing_windows(
        name, raw, distinct, reused, covered):
    rows, target, duration, config = window_v2(name)
    before = copy.deepcopy(rows)
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "passed"
    assert actual["raw_evaluations"] == raw
    assert actual["distinct_observations"] == distinct
    assert actual["reused_observations"] == reused
    assert actual["covered_publication_times"] == covered
    assert actual["window_publication_us"] == [
        100740000 if name == "FINAL_HOVER" else 95700000,
        105760000 if name == "FINAL_HOVER" else 100700000,
    ]
    assert actual["duration_sim_s"] == pytest.approx(5.02 if name == "FINAL_HOVER" else 5.0)
    original = recorded()[name]["original_strict_result"]
    for metric in ("max_horizontal_error_m", "max_height_error_m", "max_yaw_error_deg"):
        assert actual[metric] == original[metric]
    assert rows == before


def test_repeated_position_with_changed_unsafe_attitude_still_fails():
    rows, target, duration, config = window_v2("FINAL_HOVER")
    repeated = next(b for a, b in zip(rows, rows[1:]) if a["source_id"] == b["source_id"])
    repeated["yaw"] = target["yaw"] + math.radians(5.01)
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["checks"]["all_record_yaw"] is False
    assert actual["reused_observations"] == 3


@pytest.mark.parametrize("change", [
    {"nav_state": 18}, {"arming_state": 1}, {"landed": True}, {"health_valid": False},
])
def test_repeated_position_does_not_discard_changed_mode_or_health(change):
    rows, target, duration, config = window_v2("FINAL_HOVER")
    repeated = next(b for a, b in zip(rows, rows[1:]) if a["source_id"] == b["source_id"])
    repeated.update(change)
    assert acceptance.evaluate_window_v2(rows, target, duration, config)["status"] == "failed"


@pytest.mark.parametrize("collision_member", [0, 1])
def test_all_record_extrema_include_each_distinct_equal_time_output(collision_member):
    rows, target, duration, config = window_v2("RESTORE_INITIAL_YAW")
    collision = [r for r in rows if r["timestamp_us"] == 97568000]
    collision[collision_member]["position"][0] = target["position"][0] + .201
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["checks"]["all_record_horizontal"] is False
    assert actual["distinct_observations"] == 625


def test_same_source_reference_with_conflicting_position_is_rejected():
    rows, target, duration, config = window_v2("FINAL_HOVER")
    repeated = next(b for a, b in zip(rows, rows[1:]) if a["source_id"] == b["source_id"])
    repeated["position"][0] += .001
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["reason"] == "conflicting_source_identity"


def test_repeated_record_cannot_make_a_short_fixed_window_long_enough():
    rows, target, duration, config = window_v2("FINAL_HOVER")
    short = rows[:-1]
    inflated = [copy.deepcopy(row) for row in short for _ in range(4)]
    actual = acceptance.evaluate_window_v2(inflated, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["checks"]["required_duration"] is False
    assert actual["window_publication_us"][-1] < 105740000


def test_reuses_do_not_bridge_a_real_uncovered_interval():
    rows, target, duration, config = window_v2("FINAL_HOVER")
    cut = [row for row in rows if not 102500000 < row["timestamp_us"] < 102800000]
    inflated = [copy.deepcopy(row) for row in cut for _ in range(3)]
    actual = acceptance.evaluate_window_v2(inflated, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["checks"]["maximum_uncovered_interval"] is False
    assert actual["max_gap_sim_s"] > .2


def test_true_publication_time_regression_remains_rejected():
    rows, target, duration, config = window_v2("RESTORE_INITIAL_YAW")
    rows[100], rows[101] = rows[101], rows[100]
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["reason"] == "publication_time_regression"


def test_reference_change_is_visible_even_in_a_reused_position_evaluation():
    rows, target, duration, config = window_v2("FINAL_HOVER")
    repeated = next(b for a, b in zip(rows, rows[1:]) if a["source_id"] == b["source_id"])
    repeated["reference_generation"]["xy_reset_counter"] += 1
    actual = acceptance.evaluate_window_v2(rows, target, duration, config)
    assert actual["status"] == "failed"
    assert actual["reason"] == "reference_generation_changed_in_window"


@pytest.mark.parametrize("bad_sample", [None, 0, -1, True, 97560000.0, 100700001])
def test_invalid_sample_timestamp_does_not_switch_to_publication_clock(bad_sample):
    rows, target, duration, config = window_v2("RESTORE_INITIAL_YAW")
    rows[200]["timestamp_sample_us"] = bad_sample
    assert acceptance.evaluate_window_v2(rows, target, duration, config)["status"] != "passed"


def test_missing_sample_timestamp_does_not_switch_to_legacy_seconds():
    rows, target, duration, config = window_v2("RESTORE_INITIAL_YAW")
    del rows[200]["timestamp_sample_us"]
    assert acceptance.evaluate_window_v2(rows, target, duration, config)["status"] != "passed"


def test_native_integer_microseconds_preserve_one_microsecond_progress_above_float_precision():
    _, _, _, config = window_v2("FINAL_HOVER")
    base = 2**53 + 123456
    rows = [dict(timestamp_us=base+i, timestamp_sample_us=base+i-1, source_id=f"run:p:{i}",
                 position=[0., 0., -2.], yaw=0., nav_state=14, arming_state=2, landed=False,
                 reference_generation={"xy_reset_counter": 0}) for i in range(3)]
    actual = acceptance.evaluate_window_v2(rows, {"position": [0., 0., -2.], "yaw": 0.}, .000002, config)
    assert actual["status"] == "passed"
    assert actual["window_publication_us"] == [base, base+2]
    assert actual["duration_sim_s"] == .000002
    assert actual["max_gap_sim_s"] == .000001


def test_source_sample_regression_cannot_hide_behind_a_verified_earlier_identity():
    _, _, _, config = window_v2("FINAL_HOVER")
    def row(pub, sample, identity):
        return dict(timestamp_us=pub, timestamp_sample_us=sample, source_id=identity,
                    position=[0., 0., -2.], yaw=0., nav_state=14, arming_state=2, landed=False,
                    reference_generation={"xy_reset_counter": 0})
    first = row(1000000, 980000, "r:p:0")
    rows = [first, row(1000000, 990000, "r:p:1"), copy.deepcopy(first), row(1100000, 1090000, "r:p:2")]
    assert acceptance.evaluate_window_v2(rows, {"position": [0., 0., -2.], "yaw": 0.}, .1, config)["status"] == "failed"


def position_record(pub, sample, x=0., **changes):
    """Independent small raw-message fixture with explicit estimator generation."""
    row = dict(timestamp=pub, timestamp_sample=sample, x=x, y=0., z=-2.,
               vx=0., vy=0., vz=0., heading=0.,
               xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
               ref_timestamp=100000, xy_reset_counter=0, z_reset_counter=0,
               heading_reset_counter=0, vxy_reset_counter=0, vz_reset_counter=0)
    row.update(changes)
    return row


def test_native_stream_classification_retains_every_record_and_its_order():
    from sample_evidence import validate_stream
    first = position_record(1000000, 980000, x=.01)
    second = position_record(1000000, 990000, x=.02)
    rows = [first, copy.deepcopy(first), second, position_record(1100000, 1090000)]
    before = copy.deepcopy(rows)
    result = validate_stream(rows, "vehicle_local_position", "run-a")
    assert result["statistics"] == dict(raw_records=4, distinct_observations=3, duplicate_reuses=1,
                                        equal_publication_distinct_outputs=1, covered_publication_times=2)
    assert [r["x"] for r in result["records"]] == [.01, .01, .02, 0.]
    evidence = [r["_evidence"] for r in result["records"]]
    assert len({r["record_id"] for r in evidence}) == 4
    assert evidence[0]["source_id"] == evidence[1]["source_id"]
    assert evidence[0]["source_id"] != evidence[2]["source_id"]
    assert [r["native_ordinal"] for r in evidence] == [1, 2, 3, 4]
    assert rows == before


def test_raw_duplicate_after_distinct_same_time_output_remains_a_sample_regression():
    from sample_evidence import validate_stream
    first = position_record(1000000, 980000)
    rows = [first, position_record(1000000, 990000), copy.deepcopy(first)]
    with pytest.raises(ValueError):
        validate_stream(rows, "vehicle_local_position", "run-a")


@pytest.mark.parametrize("bad_sample", [None, 0, -1, True, 990000.0, 1000001])
def test_raw_stream_cannot_silently_select_another_clock(bad_sample):
    from sample_evidence import validate_stream
    with pytest.raises(ValueError):
        validate_stream([position_record(1000000, bad_sample)], "vehicle_local_position", "run-a")


def test_raw_stream_sample_nonprogress_with_conflicting_identity_fails():
    from sample_evidence import validate_stream
    with pytest.raises(ValueError):
        validate_stream([position_record(1000000, 990000), position_record(1010000, 990000)],
                        "vehicle_local_position", "run-a")


def test_reset_counters_are_retained_in_distinct_equal_time_source_identity():
    from sample_evidence import validate_stream
    rows = [position_record(1000000, 980000), position_record(1000000, 990000, xy_reset_counter=1)]
    result = validate_stream(rows, "vehicle_local_position", "run-a")
    assert [r["_evidence"]["reference_generation"]["xy_reset_counter"] for r in result["records"]] == [0, 1]
    assert result["statistics"]["distinct_observations"] == 2


def test_lossless_exact_overlap_keeps_both_timestamp_collision_candidates():
    from sample_evidence import exact_overlap
    rows = [position_record(1000000, 980000, .01), position_record(1000000, 990000, .02)]
    actual = exact_overlap(rows, copy.deepcopy(rows), "vehicle_local_position", "run-a")
    assert actual["exact_payload_matches"] == 2
    assert actual["unmatched_left"] == actual["unmatched_right"] == 0
    assert actual["left"]["distinct_observations"] == actual["right"]["distinct_observations"] == 2
    assert actual["ambiguous_matches"] == actual["reused_matches"] == 0


def test_exact_overlap_conflicting_same_identity_never_chooses_a_favorable_value():
    from sample_evidence import exact_overlap
    left = [position_record(1000000, 990000, .01)]
    right = [position_record(1000000, 990000, .02)]
    with pytest.raises(ValueError, match="ambiguous_or_conflicting"):
        exact_overlap(left, right, "vehicle_local_position", "run-a")


def test_ambiguous_right_candidates_are_rejected_before_any_first_or_last_match():
    from sample_evidence import exact_overlap
    left = [position_record(1000000, 990000, .01)]
    right = [position_record(1000000, 990000, .01), position_record(1000000, 990000, .02)]
    with pytest.raises(ValueError, match="conflicting_source_identity"):
        exact_overlap(left, right, "vehicle_local_position", "run-a")


def test_rate_limited_stream_reports_unmatched_outputs_without_false_loss_or_parity():
    from sample_evidence import exact_overlap
    ulog = [position_record(1000000 + i*10000, 990000 + i*10000, i*.001) for i in range(5)]
    ros = [copy.deepcopy(ulog[0]), copy.deepcopy(ulog[3])]
    actual = exact_overlap(ros, ulog, "vehicle_local_position", "run-a")
    assert actual["exact_payload_matches"] == 2
    assert actual["unmatched_left"] == 0
    assert actual["unmatched_right"] == 3
    assert actual["left"]["raw_records"] == 2
    assert actual["right"]["raw_records"] == 5
    assert "do not establish ROS packet loss or full parity" in actual["coverage_limit"]


def test_duplicate_deliveries_are_reported_as_reused_correlations():
    from sample_evidence import exact_overlap
    source = position_record(1000000, 990000, .01)
    actual = exact_overlap([source, copy.deepcopy(source)], [copy.deepcopy(source)],
                           "vehicle_local_position", "run-a")
    assert actual["exact_payload_matches"] == 2
    assert actual["reused_matches"] == 1
    assert actual["left"]["duplicate_reuses"] == 1
    assert actual["right"]["raw_records"] == 1


def test_cross_run_raw_records_cannot_join():
    from sample_evidence import exact_overlap
    source = position_record(1000000, 990000)
    other = position_record(1000000, 990000, _run_id="run-b")
    with pytest.raises(ValueError, match="cross_run_record"):
        exact_overlap([source], [other], "vehicle_local_position", "run-a")


def test_causal_join_never_borrows_equal_time_or_future_state():
    from sample_evidence import causal_asof, validate_stream
    rows = validate_stream([dict(timestamp=900000, nav_state=14),
                            dict(timestamp=1000000, nav_state=18),
                            dict(timestamp=1100000, nav_state=18)], "vehicle_status", "run-a")["records"]
    assert causal_asof(rows, 1000000, .2, "vehicle_status")["nav_state"] == 14
    with pytest.raises(ValueError, match="missing_strictly_earlier"):
        causal_asof(rows, 900000, .2, "vehicle_status")
    with pytest.raises(ValueError, match="causal_state_gap"):
        causal_asof(rows, 1400000, .2, "vehicle_status")


def test_causal_join_retains_validated_estimator_tie_order_in_prior_group():
    from sample_evidence import causal_asof, validate_stream
    rows = validate_stream([position_record(1000000, 980000, .01),
                            position_record(1000000, 990000, .02)],
                           "vehicle_local_position", "run-a")["records"]
    selected = causal_asof(rows, 1010000, .2, "vehicle_local_position")
    assert selected["timestamp_sample"] == 990000
    assert selected["x"] == .02


def test_causal_join_rejects_unvalidated_or_reordered_inputs():
    from sample_evidence import causal_asof, validate_stream
    native = [dict(timestamp=900000, nav_state=14), dict(timestamp=1000000, nav_state=14)]
    with pytest.raises(ValueError, match="validated_native"):
        causal_asof(native, 1100000, .2, "vehicle_status")
    validated = validate_stream(native, "vehicle_status", "run-a")["records"]
    with pytest.raises(ValueError, match="input_regression"):
        causal_asof(list(reversed(validated)), 1100000, .2, "vehicle_status")


def test_non_estimator_same_time_conflict_is_not_given_an_invented_order():
    from sample_evidence import validate_stream
    with pytest.raises(ValueError):
        validate_stream([dict(timestamp=900000, nav_state=14), dict(timestamp=900000, nav_state=18)],
                        "vehicle_status", "run-a")


def component_streams():
    """Known safe snapshots surrounding a fixed 1.000--1.100 second window."""
    return {
        "vehicle_local_position": [position_record(1000000, 990000), position_record(1100000, 1090000)],
        "vehicle_attitude": [dict(timestamp=t, timestamp_sample=t-1000, q=[1., 0., 0., 0.],
                                  quat_reset_counter=0) for t in (950000, 1025000, 1075000)],
        "vehicle_status": [dict(timestamp=t, nav_state=14, arming_state=2, failsafe=False,
                                pre_flight_checks_pass=True) for t in (950000, 1050000)],
        "vehicle_land_detected": [dict(timestamp=t, landed=False) for t in (950000, 1050000)],
        "estimator_status_flags": [dict(timestamp=t, cs_tilt_align=True, cs_yaw_align=True)
                                   for t in (950000, 1050000)],
        "failsafe_flags": [dict(timestamp=t, local_position_invalid=False, local_altitude_invalid=False,
                                attitude_invalid=False, angular_velocity_invalid=False) for t in (950000, 1050000)],
    }


def test_native_attitude_between_position_projections_cannot_hide_a_tracking_violation():
    from sample_evidence import all_record_window_checks, joined_samples_v2
    _, _, _, config = window_v2("FINAL_HOVER")
    streams = component_streams()
    streams["vehicle_attitude"][1]["q"] = [math.cos(math.radians(3)), 0., 0., math.sin(math.radians(3))]
    projected = joined_samples_v2(streams, config, 1000000, 1100000, "run-a")
    assert [row["yaw"] for row in projected] == [0., 0.]
    with pytest.raises(ValueError, match="native_attitude_tracking"):
        all_record_window_checks(streams, 1000000, 1100000, {"position": [0., 0., -2.], "yaw": 0.}, config)


@pytest.mark.parametrize("topic,change", [
    ("vehicle_status", {"failsafe": True}),
    ("vehicle_land_detected", {"landed": True}),
    ("estimator_status_flags", {"cs_yaw_align": False}),
    ("failsafe_flags", {"attitude_invalid": True}),
])
def test_every_native_component_health_change_is_checked(topic, change):
    from sample_evidence import all_record_window_checks
    _, _, _, config = window_v2("FINAL_HOVER")
    streams = component_streams()
    streams[topic][-1].update(change)
    with pytest.raises(ValueError):
        all_record_window_checks(streams, 1000000, 1100000, {"position": [0., 0., -2.], "yaw": 0.}, config)


def test_fixed_window_projection_retains_both_equal_time_position_outputs():
    from sample_evidence import joined_samples_v2
    _, _, _, config = window_v2("FINAL_HOVER")
    streams = component_streams()
    streams["vehicle_local_position"].insert(1, position_record(1000000, 995000, .001))
    projected = joined_samples_v2(streams, config, 1000000, 1100000, "run-a")
    assert [row["timestamp_us"] for row in projected] == [1000000, 1000000, 1100000]
    assert [row["timestamp_sample_us"] for row in projected] == [990000, 995000, 1090000]
    assert [row["position"][0] for row in projected] == [0., .001, 0.]
    assert len({row["source_id"] for row in projected}) == 3


def test_causal_join_rejects_mixed_validated_runs():
    from sample_evidence import causal_asof, validate_stream
    earlier = validate_stream([dict(timestamp=900000, nav_state=14)], "vehicle_status", "run-a")["records"]
    later = validate_stream([dict(timestamp=1000000, nav_state=14)], "vehicle_status", "run-b")["records"]
    with pytest.raises(ValueError):
        causal_asof(earlier + later, 1100000, .2, "vehicle_status")


def test_causal_join_rejects_a_validated_but_wrong_topic():
    from sample_evidence import causal_asof, validate_stream
    rows = validate_stream([dict(timestamp=900000, landed=False)], "vehicle_land_detected", "run-a")["records"]
    with pytest.raises(ValueError):
        causal_asof(rows, 1000000, .2, "vehicle_status")


def test_explicit_raw_record_ids_must_be_unique_even_for_distinct_publications():
    from sample_evidence import validate_stream
    rows = [position_record(1000000, 990000, _record_id="run-a:position:1"),
            position_record(1100000, 1090000, _record_id="run-a:position:1")]
    with pytest.raises(ValueError):
        validate_stream(rows, "vehicle_local_position", "run-a")


def runtime_callback_fixture(raw_changes=None):
    """Use the upstream callback identity producer, with independent counts/data.

    Identity hashing itself has separate tests. Here actual producer metadata is
    fed into the independent reader, and expected classifications are constants.
    """
    from gwm_px4_control.sample_identity import IdentityTracker
    raw = {
        'vehicle_local_position': position_record(1000000, 990000),
        'vehicle_attitude': dict(timestamp=1000000, timestamp_sample=996000, q=[1.,0.,0.,0.], quat_reset_counter=0),
        'vehicle_status': dict(timestamp=1000000, nav_state=14, arming_state=2, failsafe=False, pre_flight_checks_pass=True,
                               system_id=72, component_id=1),
        'vehicle_land_detected': dict(timestamp=1000000, landed=False),
        'estimator_status_flags': dict(timestamp=1000000, cs_tilt_align=True, cs_yaw_align=True),
        'failsafe_flags': dict(timestamp=1000000, local_position_invalid=False, local_altitude_invalid=False,
                              attitude_invalid=False, angular_velocity_invalid=False),
    }
    for topic, changes in (raw_changes or {}).items():
        raw[topic].update(changes)
    versions = {'vehicle_local_position': 1, 'vehicle_attitude': 0, 'vehicle_status': 1,
                'vehicle_land_detected': 0, 'estimator_status_flags': 0, 'failsafe_flags': 0}
    tracker = IdentityTracker('run-a')
    events, sources, streams = [], {}, {}
    for ordinal, (topic, fields) in enumerate(raw.items(), 1):
        stamp_ns = 10000000000 + ordinal*1000000
        delivery_id = f'run-a:delivery:{ordinal}'
        source = tracker.observe(topic, copy.deepcopy(fields), delivery_id, stamp_ns, version=versions[topic])
        sources[topic] = copy.deepcopy(source)
        streams[topic] = [copy.deepcopy(fields)]
        events.append(dict(event='received', topic_key=topic, fields=copy.deepcopy(fields), nonfinite_fields=[],
                           monotonic_s=stamp_ns/1e9, ros_sim_s=1.01, run_id='run-a',
                           sample_evidence_contract='p3-sample-evidence-v2', delivery_id=delivery_id,
                           delivery_ordinal=ordinal, topic_delivery_ordinal=1, callback_entry_monotonic_ns=stamp_ns,
                           message_version=versions[topic], uorb_instance=0))
        events.append(dict(event='source_delivery', run_id='run-a', topic_key=topic,
                           sample_evidence_contract='p3-sample-evidence-v2', source=source))
    p = sources['vehicle_local_position']
    sample = dict(t=1., timestamp_us=1000000, timestamp_sample_us=990000, source_id=p['source_id'],
                  position=[raw['vehicle_local_position'][key] for key in ('x', 'y', 'z')],
                  velocity=[raw['vehicle_local_position'][key] for key in ('vx', 'vy', 'vz')], yaw=0.,
                  nav_state=raw['vehicle_status']['nav_state'], arming_state=raw['vehicle_status']['arming_state'],
                  landed=raw['vehicle_land_detected']['landed'],
                  ros_sim_s=1.02, receipt_monotonic_s=10.1, selection_monotonic_s=10.1,
                  source_callback_entry_monotonic_s=10.001, component_sources=copy.deepcopy(sources))
    return streams, events, sources, sample, tracker


def control_evaluation(ordinal, sources, sample=None, error=None, entry_ns=10090000000):
    sample = copy.deepcopy(sample)
    if sample is not None:
        sample.update(selection_id=f'run-a:evaluation:{ordinal}', evaluation_id=f'run-a:evaluation:{ordinal}')
    return dict(event='control_evaluation', run_id='run-a', sample_evidence_contract='p3-sample-evidence-v2',
                evaluation_id=f'run-a:evaluation:{ordinal}', selection_id=f'run-a:evaluation:{ordinal}',
                evaluated_phase='FINAL_HOVER', evaluation_entry_monotonic_ns=entry_ns,
                evaluation_return_monotonic_ns=entry_ns+20000000, component_sources=copy.deepcopy(sources),
                sample=sample, error=error)


def test_all_control_evaluations_retain_errors_and_nonselections_with_real_counts():
    from sample_evidence import reconstruct_controller_samples
    _, callbacks, sources, sample, _ = runtime_callback_fixture()
    first = control_evaluation(1, {}, error='missing_state', entry_ns=9000000000)
    selected = control_evaluation(2, sources, sample)
    last = control_evaluation(3, sources, error='graph_unavailable', entry_ns=10200000000)
    events = [first, *callbacks, selected, last]
    before = copy.deepcopy(events)
    actual = reconstruct_controller_samples(events, recorded()['config'], 'run-a')
    stats = actual['statistics']
    assert stats['controller_evaluations'] == 3
    assert stats['selected_evaluations'] == 1
    assert stats['nonselected_evaluations'] == 2
    assert stats['evaluation_errors'] == [
        dict(evaluation_id='run-a:evaluation:1', error='missing_state', phase='FINAL_HOVER'),
        dict(evaluation_id='run-a:evaluation:3', error='graph_unavailable', phase='FINAL_HOVER'),
    ]
    assert stats['distinct_position_observations'] == 1
    assert len(actual['samples']) == 1
    assert events == before


def test_controller_reconstruction_keeps_repeated_position_and_changed_attitude():
    from sample_evidence import reconstruct_controller_samples
    streams, events, sources, sample, tracker = runtime_callback_fixture()
    events.append(control_evaluation(1, sources, sample))
    attitude = dict(timestamp=1010000, timestamp_sample=1006000,
                    q=[math.cos(.001),0.,0.,math.sin(.001)], quat_reset_counter=0)
    source = tracker.observe('vehicle_attitude', attitude, 'run-a:delivery:7', 10150000000, version=0)
    events.append(dict(event='received', topic_key='vehicle_attitude', fields=attitude,
                       monotonic_s=10.15, ros_sim_s=1.02, run_id='run-a', delivery_id='run-a:delivery:7',
                       callback_entry_monotonic_ns=10150000000, message_version=0))
    events.append(dict(event='source_delivery', run_id='run-a', topic_key='vehicle_attitude',
                       sample_evidence_contract='p3-sample-evidence-v2', source=copy.deepcopy(source)))
    sources['vehicle_attitude'] = source
    repeated = copy.deepcopy(sample)
    repeated.update(yaw=.002, selection_monotonic_s=10.2, receipt_monotonic_s=10.2,
                    ros_sim_s=1.03, component_sources=copy.deepcopy(sources))
    events.append(control_evaluation(2, sources, repeated, entry_ns=10190000000))
    actual = reconstruct_controller_samples(events, recorded()['config'], 'run-a')
    assert actual['statistics']['selected_evaluations'] == 2
    assert actual['statistics']['distinct_position_observations'] == 1
    assert actual['statistics']['reused_position_evaluations'] == 1
    assert [row['yaw'] for row in actual['samples']] == [0., .002]


@pytest.mark.parametrize('topic,change,reason', [
    ('vehicle_status', {'pre_flight_checks_pass': False}, 'selected_evaluation_health_invalid'),
    ('vehicle_status', {'failsafe': True}, 'selected_evaluation_health_invalid'),
    ('vehicle_status', {'system_id': 73}, 'selected_evaluation_vehicle_identity'),
    ('vehicle_status', {'component_id': 2}, 'selected_evaluation_vehicle_identity'),
    ('vehicle_local_position', {'v_xy_valid': False}, 'selected_evaluation_health_invalid'),
    ('estimator_status_flags', {'cs_yaw_align': False}, 'selected_evaluation_health_invalid'),
    ('failsafe_flags', {'attitude_invalid': True}, 'selected_evaluation_health_invalid'),
    ('vehicle_local_position', {'vx': float('nan')}, 'selected_evaluation_nonfinite_estimate'),
    ('vehicle_local_position', {'vz': float('inf')}, 'selected_evaluation_nonfinite_estimate'),
])
def test_selected_evaluation_health_and_finite_state_are_checked_outside_tracking_windows(topic, change, reason):
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture({topic: change})
    evaluation = control_evaluation(1, sources, sample)
    evaluation['evaluated_phase'] = 'TAKEOFF'
    events.append(evaluation)
    with pytest.raises(ValueError, match=reason):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


@pytest.mark.parametrize('phase,arming,landed,allowed', [
    ('VERIFY_LANDED_AND_DISARMED', 1, True, True),
    ('PRESTREAM_SAFE_SETPOINTS', 1, True, False),
    ('FINAL_HOVER', 1, True, False),
    ('VERIFY_LANDED_AND_DISARMED', 2, True, False),
    ('VERIFY_LANDED_AND_DISARMED', 1, False, False),
])
def test_selected_evaluation_terminal_preflight_exception_matches_live_phase(phase, arming, landed, allowed):
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture({
        'vehicle_status': {'pre_flight_checks_pass': False, 'arming_state': arming},
        'vehicle_land_detected': {'landed': landed},
    })
    evaluation = control_evaluation(1, sources, sample)
    evaluation['evaluated_phase'] = phase
    events.append(evaluation)
    if allowed:
        actual = reconstruct_controller_samples(events, recorded()['config'], 'run-a')
        assert actual['samples'][0]['health_valid'] is True
    else:
        with pytest.raises(ValueError, match='selected_evaluation_health_invalid'):
            reconstruct_controller_samples(events, recorded()['config'], 'run-a')


def test_reused_position_with_new_unhealthy_status_is_rejected_outside_tracking_windows():
    from sample_evidence import reconstruct_controller_samples
    streams, events, sources, sample, tracker = runtime_callback_fixture()
    first = control_evaluation(1, sources, sample)
    first['evaluated_phase'] = 'TAKEOFF'
    events.append(first)
    status = dict(streams['vehicle_status'][0], timestamp=1010000, pre_flight_checks_pass=False)
    source = tracker.observe('vehicle_status', status, 'run-a:delivery:7', 10150000000, version=1)
    events.extend([
        dict(event='received', topic_key='vehicle_status', fields=status, monotonic_s=10.15,
             ros_sim_s=1.02, run_id='run-a', delivery_id='run-a:delivery:7',
             callback_entry_monotonic_ns=10150000000, message_version=1),
        dict(event='source_delivery', source=copy.deepcopy(source)),
    ])
    sources['vehicle_status'] = source
    repeated = dict(sample, selection_monotonic_s=10.2, receipt_monotonic_s=10.2,
                    ros_sim_s=1.03, component_sources=copy.deepcopy(sources))
    second = control_evaluation(2, sources, repeated, entry_ns=10190000000)
    second['evaluated_phase'] = 'TAKEOFF'
    events.append(second)
    with pytest.raises(ValueError, match='selected_evaluation_health_invalid'):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


def test_selected_velocity_metadata_must_match_its_raw_source():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture()
    sample['velocity'][0] = 1.
    events.append(control_evaluation(1, sources, sample))
    with pytest.raises(ValueError, match='recorded_selection_velocity_mismatch'):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


@pytest.mark.parametrize('topic,initial,next_state', [
    ('vehicle_status', {'arming_state': 1, 'nav_state': 4, 'pre_flight_checks_pass': False},
     {'arming_state': 1, 'nav_state': 4, 'pre_flight_checks_pass': True}),
    ('failsafe_flags', {'local_position_invalid': True}, {'local_position_invalid': False}),
    ('estimator_status_flags', {'cs_tilt_align': False}, {'cs_tilt_align': True}),
    ('vehicle_land_detected', {'landed': True}, {'landed': True}),
])
def test_boot_origin_nonestimator_state_is_retained_as_raw_uint64_publication(topic, initial, next_state):
    from sample_evidence import validate_stream
    rows = [dict(timestamp=0, **initial), dict(timestamp=500000, **next_state)]
    before = copy.deepcopy(rows)
    actual = validate_stream(rows, topic, 'run-a')
    assert actual['statistics']['raw_records'] == 2
    assert actual['statistics']['distinct_observations'] == 2
    assert [row['timestamp'] for row in actual['records']] == [0, 500000]
    assert all(actual['records'][0][key] == value for key, value in initial.items())
    assert rows == before


def test_boot_origin_unhealthy_status_cannot_receive_selected_evaluation_credit():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture({
        'vehicle_status': {'timestamp': 0, 'pre_flight_checks_pass': False, 'arming_state': 1},
        'vehicle_land_detected': {'landed': True},
    })
    evaluation = control_evaluation(1, sources, sample)
    evaluation['evaluated_phase'] = 'PRESTREAM_SAFE_SETPOINTS'
    events.append(evaluation)
    with pytest.raises(ValueError, match='selected_evaluation_health_invalid'):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


@pytest.mark.parametrize('topic', ['vehicle_local_position', 'vehicle_attitude'])
@pytest.mark.parametrize('sample_us', [0, 1])
def test_boot_origin_publication_does_not_relax_estimator_sample_invariants(topic, sample_us):
    from sample_evidence import validate_stream
    row = (position_record(0, sample_us) if topic == 'vehicle_local_position' else
           dict(timestamp=0, timestamp_sample=sample_us, q=[1., 0., 0., 0.], quat_reset_counter=0))
    with pytest.raises(ValueError):
        validate_stream([row], topic, 'run-a')


def test_nonselection_evaluation_clock_regression_is_not_discarded():
    from sample_evidence import reconstruct_controller_samples
    events = [control_evaluation(1, {}, entry_ns=10000000000),
              control_evaluation(2, {}, entry_ns=9000000000)]
    with pytest.raises(ValueError):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


def test_nonselection_component_metadata_cannot_claim_a_different_run():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, _, _ = runtime_callback_fixture()
    sources['vehicle_attitude']['run_id'] = 'run-b'
    events.append(control_evaluation(1, sources))
    with pytest.raises(ValueError):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


@pytest.mark.parametrize('change', [dict(payload_sha256='0'*64),
                                    dict(accepted_source_receipt_monotonic_ns=1)])
def test_nonselection_full_component_reference_cannot_change_payload_or_receipt(change):
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, _, _ = runtime_callback_fixture()
    sources['vehicle_attitude'].update(change)
    events.append(control_evaluation(1, sources))
    with pytest.raises(ValueError, match='evaluation_full_component_reference'):
        reconstruct_controller_samples(events, recorded()['config'], 'run-a')


@pytest.mark.parametrize('change', [dict(evaluation_id='run-b:evaluation:1'),
                                    dict(selection_id='run-b:evaluation:1'),
                                    dict(evaluation_return_monotonic_ns=1)])
def test_malformed_nonselection_identity_or_execution_interval_is_rejected(change):
    from sample_evidence import reconstruct_controller_samples
    event = control_evaluation(1, {})
    event.update(change)
    with pytest.raises(ValueError):
        reconstruct_controller_samples([event], recorded()['config'], 'run-a')


def test_delivery_reader_verifies_producer_metadata_from_raw_payloads():
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    actual = verify_source_deliveries(streams, events, 'run-a')
    assert actual['status'] == 'passed'
    assert actual['raw_callback_deliveries'] == actual['verified_source_deliveries'] == actual['unique_delivery_ids'] == 6
    assert actual['per_topic_callbacks'] == {topic: 1 for topic in streams}


def test_exact_duplicate_delivery_retains_old_source_freshness_metadata():
    from sample_evidence import verify_source_deliveries
    streams, events, sources, _, tracker = runtime_callback_fixture()
    duplicate = copy.deepcopy(streams['vehicle_local_position'][0])
    source = tracker.observe('vehicle_local_position', duplicate, 'run-a:delivery:7', 10500000000, version=1)
    streams['vehicle_local_position'].append(duplicate)
    events.append(dict(event='received', topic_key='vehicle_local_position', fields=duplicate,
                       run_id='run-a', sample_evidence_contract='p3-sample-evidence-v2',
                       delivery_id='run-a:delivery:7', delivery_ordinal=7, topic_delivery_ordinal=2,
                       callback_entry_monotonic_ns=10500000000, message_version=1, uorb_instance=0))
    events.append(dict(event='source_delivery', topic_key='vehicle_local_position', run_id='run-a',
                       sample_evidence_contract='p3-sample-evidence-v2', source=source))
    assert source['classification'] == 'duplicate_reuse'
    assert source['accepted_source_receipt_monotonic_ns'] == 10001000000
    assert source['accepted_source_delivery_id'] == sources['vehicle_local_position']['delivery_id']
    assert verify_source_deliveries(streams, events, 'run-a')['verified_source_deliveries'] == 7


def test_duplicate_source_delivery_ledger_identity_is_rejected():
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    events.append(copy.deepcopy(events[1]))
    with pytest.raises(ValueError, match='duplicate_or_cross_run'):
        verify_source_deliveries(streams, events, 'run-a')


@pytest.mark.parametrize('change', [dict(run_id='run-b'), dict(payload_sha256='0'*64),
                                    dict(classification='duplicate_reuse'), dict(accepted_source_receipt_monotonic_ns=1)])
def test_source_delivery_metadata_cannot_override_raw_evidence(change):
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    events[1]['source'].update(change)
    with pytest.raises(ValueError, match='metadata_or_payload_mismatch'):
        verify_source_deliveries(streams, events, 'run-a')


def test_received_delivery_cross_run_and_global_ordinal_are_checked():
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    events[0]['run_id'] = 'run-b'
    with pytest.raises(ValueError, match='callback_run_contract'):
        verify_source_deliveries(streams, events, 'run-a')
    events[0]['run_id'] = 'run-a'
    events[0]['delivery_ordinal'] = 2
    with pytest.raises(ValueError, match='callback_delivery_ordinal'):
        verify_source_deliveries(streams, events, 'run-a')


def test_rejected_delivery_is_retained_as_failure_even_without_selection():
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    events.append(dict(event='source_delivery_rejected', run_id='run-a', reason='sample_time_backwards'))
    with pytest.raises(ValueError, match='rejected_source_delivery'):
        verify_source_deliveries(streams, events, 'run-a')


@pytest.mark.parametrize('topic', ['estimator_status_flags', 'failsafe_flags'])
def test_causal_flag_join_requires_a_real_fresh_earlier_record(topic):
    from sample_evidence import joined_samples_v2
    streams = component_streams()
    for values in streams.values():
        for row in values:
            row['timestamp'] += 2000000
            if 'timestamp_sample' in row:
                row['timestamp_sample'] += 2000000
    streams[topic][0]['timestamp'] = 1000000  # 2 s stale; later 3.050 s cannot repair it.
    with pytest.raises(ValueError, match='causal_state_gap:' + topic):
        joined_samples_v2(streams, recorded()['config'], 3000000, 3100000, 'run-a')


def test_causal_flag_join_cannot_use_only_equal_time_or_future_health():
    from sample_evidence import joined_samples_v2
    streams = component_streams()
    streams['failsafe_flags'][0]['timestamp'] = 1000000
    with pytest.raises(ValueError, match='missing_strictly_earlier_state:failsafe_flags'):
        joined_samples_v2(streams, recorded()['config'], 1000000, 1100000, 'run-a')


def test_native_fixed_boundaries_are_exact_and_never_remapped_from_display_time():
    base = 2**53 + 123456
    window = dict(start_us=base, end_us=base+5000000, start_sim_s=base/1e6, end_sim_s=(base+5000000)/1e6)
    assert acceptance.fixed_window_us(window) == (base, base+5000000)
    with pytest.raises(ValueError, match='invalid_native_window'):
        acceptance.fixed_window_us({**window, 'start_sim_s': window['start_sim_s']+.01})


def test_historical_boundary_mapping_requires_explicit_historical_mode():
    window = dict(start_sim_s=100.740, end_sim_s=105.760)
    assert acceptance.fixed_window_us(window, historical=True) == (100740000, 105760000)
    with pytest.raises(KeyError):
        acceptance.fixed_window_us(window)


@pytest.mark.parametrize('start,end', [(-1, 1000000), (1000000.0, 2000000), (True, 2000000), (2000000, 1000000)])
def test_invalid_native_window_integer_or_order_is_rejected(start, end):
    with pytest.raises(ValueError):
        acceptance.fixed_window_us(dict(start_us=start, end_us=end, start_sim_s=start/1e6, end_sim_s=end/1e6))


def test_action_sample_must_equal_its_canonical_control_evaluation():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture()
    canonical = control_evaluation(1, sources, sample)
    action = dict(event='sample', run_id='run-a', sample=copy.deepcopy(canonical['sample']))
    actual = reconstruct_controller_samples([*events, action, canonical], recorded()['config'], 'run-a')
    assert actual['statistics']['action_sample_events'] == 1
    assert actual['statistics']['selected_evaluations'] == 1
    action['sample']['yaw'] = .01
    with pytest.raises(ValueError, match='action_sample_canonical_evaluation_mismatch'):
        reconstruct_controller_samples([*events, action, canonical], recorded()['config'], 'run-a')


def test_orphan_action_sample_cannot_replace_a_missing_control_evaluation():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, sample, _ = runtime_callback_fixture()
    canonical = control_evaluation(1, sources, sample)
    action = dict(event='sample', run_id='run-a', sample=canonical['sample'])
    with pytest.raises(ValueError, match='action_sample_canonical_evaluation_mismatch'):
        reconstruct_controller_samples([*events, action], recorded()['config'], 'run-a')


@pytest.mark.parametrize('change', [dict(evaluation_entry_monotonic_ns=1.0),
                                    dict(evaluation_entry_monotonic_ns=False)])
def test_nonselection_execution_clock_metadata_preserves_native_integer_type(change):
    from sample_evidence import reconstruct_controller_samples
    event = control_evaluation(1, {})
    event.update(change)
    with pytest.raises(ValueError):
        reconstruct_controller_samples([event], recorded()['config'], 'run-a')


def test_nonselection_cannot_observe_callbacks_from_its_recorded_future():
    from sample_evidence import reconstruct_controller_samples
    _, events, sources, _, _ = runtime_callback_fixture()
    event = control_evaluation(1, sources, entry_ns=9000000000)
    with pytest.raises(ValueError):
        reconstruct_controller_samples([*events, event], recorded()['config'], 'run-a')


@pytest.mark.parametrize('change', [dict(sample_evidence_contract='different-contract'),
                                    dict(topic_key='vehicle_attitude')])
def test_source_delivery_envelope_must_agree_with_its_embedded_source(change):
    from sample_evidence import verify_source_deliveries
    streams, events, _, _, _ = runtime_callback_fixture()
    events[1].update(change)
    with pytest.raises(ValueError):
        verify_source_deliveries(streams, events, 'run-a')
