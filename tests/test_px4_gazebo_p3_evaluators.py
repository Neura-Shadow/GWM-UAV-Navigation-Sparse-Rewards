"""Independent P3 evaluator regression on synthetic run directories.

The synthetic frames come from closed-form plane geometry written here (not the
evaluator's ray/box code), flow through the production pipeline and recorder,
and are then judged by the independent evaluator. This proves format and
decision logic offline only; it is not native sensor or runtime evidence.
"""
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "simulation/px4_gazebo"
sys.path[:0] = [str(SIM / "validation"), str(SIM / "ros2_ws/src/gwm_sensor_adapter")]
import p3_fixtures  # noqa: E402
import p3_geometry_eval  # noqa: E402
import p3_stream_eval  # noqa: E402
from gwm_sensor_adapter import pipeline, recorder  # noqa: E402

FX, CX, CY = 432.496042035043, 320.0, 240.0
BASE_WORLD = '<sdf version="1.9"><world name="default"><model name="ground_plane"><static>true</static>' \
             '<link name="link"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry>' \
             '</collision></link></model></world></sdf>'
VALIDATION = json.loads((SIM / "configs/p3_validation.yaml").read_text())
SENSORS = json.loads((SIM / "configs/p3_sensors.yaml").read_text())
OPTICAL = SENSORS["optical_origin_model_m"]


def closed_form_depth(distance, yaw):
    """Vehicle at the declared pose, yaw 0; ray (1, -(u-cx)/fx, .) in FLU against normal (cos a, sin a, 0):
    Z = d cos(a) / (cos(a) - sin(a)(u-cx)/fx)."""
    v, u = np.mgrid[0:480, 0:640].astype(float)
    if math.isinf(distance):
        return np.full((480, 640), np.inf, dtype="<f4")
    denominator = math.cos(yaw) - math.sin(yaw) * (u - CX) / FX
    return (distance * math.cos(yaw) / denominator).astype("<f4")


def info():
    return {"width": 640, "height": 480, "distortion_model": "plumb_bob", "d": [0.0] * 5,
            "k": [FX, 0, CX, 0, FX, CY, 0, 0, 1], "p": [FX, 0, CX, 0, 0, FX, CY, 0, 0, 0, 1, 0],
            "r": [1, 0, 0, 0, 1, 0, 0, 0, 1], "binning_x": 0, "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}


def write_jsonl(path, rows):
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))


def synthetic_run(tmp_path, fixture="plane4", bias=0.0, drop=None, move_vehicle=0.0, depth_override=None,
                  seconds=6.0, stale_after=None):
    run = tmp_path / ("20260101T000000Z-p3-" + fixture)
    (run / "world").mkdir(parents=True)
    (run / "source").mkdir()
    (run / "sensors").mkdir()
    text, declaration = p3_fixtures.generate(BASE_WORLD, fixture, VALIDATION, OPTICAL[0])
    (run / "world/gwm_p3_ground.sdf").write_text(text)
    case = p3_fixtures.geometry_case(fixture)
    spec = VALIDATION["fixtures"][case]
    image = depth_override if depth_override is not None else closed_form_depth(
        math.inf if case == "out_of_range" else spec["distance_m"], spec["yaw_rad"])
    image = (image + bias).astype("<f4")
    proc = pipeline.DepthProcessor(SENSORS, run.name, require_state=False)
    proc.on_camera_info(info())
    rec = recorder.RawRecorder(run / "sensors", 1000)  # synthetic producer is faster than real time
    observations, health, headers, clock, poses = [], [], [], [], []
    begin, end = 1.0, 1.0 + seconds
    vehicle = {"position": [0.0, 0.0, 0.1], "q_wxyz": [1.0, 0.0, 0.0, 0.0]}
    plane = next(b for b in declaration["boxes"] if b["name"] == "front_plane")
    for n in range(int(seconds * 30) + 1):
        t = round(begin + n / 30, 9)
        stamp = int(round(t * 1e9))
        headers.append({"id": n + 1, "stamp_ns": stamp, "seq": 1000 + n, "frame_id": "camera_link", "width": 640,
                        "height": 480, "step": 2560, "pixel_format": 13, "payload_bytes": 1228800,
                        "receipt_monotonic_ns": int((100 + t) * 1e9)})
        if drop is None or n != drop:
            fields = {"width": 640, "height": 480, "encoding": "32FC1", "is_bigendian": 0, "step": 2560,
                      "data": image.tobytes()}
            _, record = proc.on_image(fields, stamp, "camera_link", 100 + t + 0.002, t + 0.004)
            rec.submit(record, fields["data"])
        wall = 100 + t + 0.004
        if stale_after is not None and t > stale_after:
            wall += 2.0
        changed, observation = proc.poll(t + 0.004, wall)
        if changed or n % 15 == 0:
            health.append(proc.health(t + 0.004, wall))
        if observation:
            observations.append(observation)
        clock.append({"id": n + 1, "sim_s": t, "receipt_monotonic_ns": int((100 + t) * 1e9)})
        if n % 6 == 0:
            moved = dict(vehicle, position=[move_vehicle * n / (seconds * 30), 0.0, 0.1])
            poses.append({"id": len(poses) + 1, "stamp_ns": stamp, "receipt_monotonic_ns": int((100 + t) * 1e9),
                          "poses": {"x500_depth_71": moved,
                                    "front_plane": {"position": plane["center_m"], "q_wxyz": [1, 0, 0, 0]}}})
    # The synthetic producer outruns real time, so the production close deadline would
    # measure host load, not the recorder. Let the writer drain first; close then has no
    # backlog and the recording must be complete, or the harness itself fails.
    while rec.written < rec.accepted and rec.error is None and rec.thread.is_alive():
        time.sleep(0.01)
    closed = rec.close(10)
    assert closed["complete"], closed
    write_jsonl(run / "sensors/observations.jsonl", observations)
    write_jsonl(run / "sensors/health.jsonl", health)
    write_jsonl(run / "source/depth-headers.jsonl", headers)
    write_jsonl(run / "source/rgb-headers.jsonl", [])
    write_jsonl(run / "source/clock.jsonl", clock)
    write_jsonl(run / "source/poses.jsonl", poses)
    (run / "source/camera-info.json").write_text(json.dumps({"width": 640, "height": 480, "k": info()["k"],
                                                             "p": info()["p"]}))
    adapter = {"status": "passed", "recorder": closed, "accounting": proc.accounting.as_dict(),
               "calibration": proc.calibration.calibration, "calibration_rejected": {},
               "calibration_invalid_reason": None, "costs": {},
               "observed_own_publications": {SENSORS["observation_topic"]: ["std_msgs/msg/String"],
                                             SENSORS["health_topic"]: ["std_msgs/msg/String"]}}
    (run / "sensors/adapter-result.json").write_text(json.dumps(adapter))
    window = [begin + 0.5, end - 0.2]
    summary = {"fixture": fixture, "world": "gwm_p3_ground", "model": "x500_depth_71",
               "measurement_window_sim_s": window}
    (run / "summary.json").write_text(json.dumps(summary))
    return run


@pytest.fixture
def limits():
    return dict(VALIDATION, delivered_hz_min=25)


@pytest.mark.parametrize("fixture", ["plane2", "plane4", "plane6", "oblique", "out_of_range"])
def test_synthetic_closed_form_fixtures_pass_independent_evaluation(tmp_path, fixture):
    run = synthetic_run(tmp_path, fixture)
    result = p3_geometry_eval.evaluate_fixture(run, VALIDATION, SENSORS)
    assert result["status"] == "passed", result["failures"]
    if fixture == "oblique":
        assert result["errors"]["euclidean_range_hypothesis_median_error_m"] > result["errors"]["median_limit_m"]
    if fixture == "out_of_range":
        assert result["pixel_counts"]["positive_infinity"] > 0 and result["pixel_counts"]["zero"] == 0


def test_depth_bias_beyond_frozen_threshold_fails(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", bias=0.2), VALIDATION, SENSORS)
    assert "median_depth_error" in result["failures"]


def test_bias_within_threshold_still_passes(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", bias=0.03), VALIDATION, SENSORS)
    assert result["status"] == "passed", result["failures"]


def test_euclidean_range_rendering_is_rejected_for_oblique(tmp_path):
    z = closed_form_depth(4, 0.35)
    v, u = np.mgrid[0:480, 0:640].astype(float)
    ranged = (z * np.sqrt(1 + ((u - CX) / FX) ** 2 + ((v - CY) / FX) ** 2)).astype("<f4")
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "oblique", depth_override=ranged),
                                               VALIDATION, SENSORS)
    assert result["status"] == "failed" and "p95_depth_error" in result["failures"]


def test_mirrored_oblique_image_fails_left_right_orientation(tmp_path):
    flipped = closed_form_depth(4, 0.35)[:, ::-1].copy()
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "oblique", depth_override=flipped),
                                               VALIDATION, SENSORS)
    assert "median_depth_error" in result["failures"] or "p95_depth_error" in result["failures"]


def test_out_of_range_fixture_with_fabricated_values_fails(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(
        synthetic_run(tmp_path, "out_of_range", depth_override=np.full((480, 640), 19.0, dtype="<f4")),
        VALIDATION, SENSORS)
    assert "out_of_range_reported_valid" in result["failures"]


def test_zero_filled_unknowns_are_detected(tmp_path):
    image = closed_form_depth(4, 0)
    image[400:, :] = 0.0
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", depth_override=image),
                                               VALIDATION, SENSORS)
    assert "zero_depth_fabricated" in result["failures"]


def test_delivery_loss_is_never_hidden_by_processing(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", drop=60), VALIDATION, SENSORS)
    assert "stream:source_to_ros_delivery_loss" in result["failures"]


def test_unsettled_vehicle_fails(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", move_vehicle=0.05),
                                               VALIDATION, SENSORS)
    assert "unsettled_vehicle_pose" in result["failures"]


def test_stale_health_inside_window_fails(tmp_path):
    result = p3_geometry_eval.evaluate_fixture(synthetic_run(tmp_path, "plane4", stale_after=3.0), VALIDATION, SENSORS)
    assert any("health_not_fresh_in_window" in f or "processed_rate" in f for f in result["failures"])


def test_tampered_raw_frame_is_rejected(tmp_path):
    run = synthetic_run(tmp_path, "plane4")
    data = bytearray((run / "sensors/depth.bin").read_bytes())
    data[100] ^= 0xFF
    (run / "sensors/depth.bin").write_bytes(bytes(data))
    with pytest.raises(ValueError, match="raw_stored_hash"):
        p3_geometry_eval.evaluate_fixture(run, VALIDATION, SENSORS)


def test_missing_source_ledger_cannot_pass(tmp_path):
    run = synthetic_run(tmp_path, "plane4")
    (run / "source/depth-headers.jsonl").unlink()
    with pytest.raises(FileNotFoundError):
        p3_geometry_eval.evaluate_fixture(run, VALIDATION, SENSORS)


def test_partial_jsonl_is_rejected(tmp_path):
    run = synthetic_run(tmp_path, "plane4")
    path = run / "sensors/observations.jsonl"
    path.write_text(path.read_text()[:-5])
    with pytest.raises(ValueError):
        p3_stream_eval.load_sensor_artifacts(run)


def test_source_probe_sequence_gap_is_reported(tmp_path, limits):
    rows = [{"id": i + 1, "stamp_ns": int((1 + i / 30) * 1e9), "seq": i + (5 if i > 40 else 0)} for i in range(90)]
    result, _ = p3_stream_eval.evaluate_source(rows, [1.2, 3.8], limits, "depth")
    assert "depth:probe_sequence_gap_or_duplicate" in result["failures"]


def test_ray_box_matches_closed_form_for_rotated_plane():
    _, boxes = p3_geometry_eval.world_boxes(p3_fixtures.generate(BASE_WORLD, "oblique", VALIDATION, OPTICAL[0])[0])
    vs, us = np.mgrid[80:200, 160:480].astype(float)
    expected, labels, _, _ = p3_geometry_eval.expected_depth(us, vs, (FX, FX, CX, CY), np.array([0, 0, 0.1]),
                                                             np.eye(3), OPTICAL, boxes)
    assert np.allclose(expected, closed_form_depth(4, 0.35)[80:200, 160:480], atol=1e-4)
    assert set(labels.ravel()) == {"front_plane"}


def test_asymmetric_regions_hit_the_declared_boxes():
    _, boxes = p3_geometry_eval.world_boxes(p3_fixtures.generate(BASE_WORLD, "asymmetric", VALIDATION, OPTICAL[0])[0])
    for name, (u0, v0, u1, v1) in VALIDATION["fixtures"]["asymmetric"]["region_uv"].items():
        vs, us = np.mgrid[v0:v1, u0:u1].astype(float)
        expected, labels, _, _ = p3_geometry_eval.expected_depth(us, vs, (FX, FX, CX, CY), np.array([0, 0, 0.1]),
                                                                 np.eye(3), OPTICAL, boxes)
        assert set(labels.ravel()) == {name}
        assert np.allclose(expected, {"left_high": 3.0, "right_low": 2.0}[name], atol=1e-9)


def test_generated_world_only_adds_static_fixtures_and_vehicle():
    text, declaration = p3_fixtures.generate(BASE_WORLD, "asymmetric", VALIDATION, OPTICAL[0])
    assert 'name="gwm_p3_ground"' in text and "model://x500_depth" in text
    assert [b["name"] for b in declaration["boxes"]] == ["front_plane", "right_low", "left_high"]
    assert declaration["boxes"][0]["front_m"] == [4 + OPTICAL[0], 0.0, 4]
    with pytest.raises(ValueError):
        p3_fixtures.generate(BASE_WORLD, "unknown", VALIDATION, OPTICAL[0])


def test_scene_change_target_is_one_metre_further():
    pose = p3_fixtures.scene_change_pose(VALIDATION, OPTICAL[0])
    assert math.isclose(pose["position_m"][0], 5 + OPTICAL[0] + 0.05)


def finalize_synthetic(run, gazebo_exit=0, residual=None):
    import p3_provenance
    processes = [{"name": name, "stop_method": "SIGINT", "grace_s": 15, "graceful_codes": [0],
                  "exit_code": gazebo_exit if name == "gazebo" else 0, "reaped": True, "forced": False,
                  "remaining_group_members": [], "stop_requested_monotonic_s": 1.0, "exit_observed_monotonic_s": 1.5}
                 for name in ("gazebo", "source-probe", "clock-bridge", "sensor-bridge", "sensor-adapter")]
    summary = json.loads((run / "summary.json").read_text())
    summary.update(p3_kind="fixture", status="passed", finalized=True, frozen_manifest_sha256="f" * 64,
                   processes=processes, residual_namespace_processes=residual or [])
    summary["artifacts"] = [{"path": p.relative_to(run).as_posix(), "sha256": p3_provenance.digest(p)}
                            for p in sorted(run.rglob("*")) if p.is_file() and p.name != "summary.json"]
    (run / "summary.json").write_text(json.dumps(summary))


@pytest.mark.parametrize("gate,strings,software,failure", [
    ("V", ["Mesa", "llvmpipe (LLVM 20.1.2, 256 bits)"], True, None),
    ("V", ["D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)"], False, "validation_renderer_not_llvmpipe"),
    ("A", ["Mesa", "llvmpipe (LLVM 20.1.2, 256 bits)"], True, "renderer_unidentified")])
def test_renderer_requirement_by_gate_and_validation_never_credits(monkeypatch, gate, strings, software, failure):
    import p3_evaluate
    summary = {"p3_kind": "readiness", "gate": gate, "status": "passed",
               "renderer": {"identified": not software, "software": software, "strings": strings}}
    real = p3_evaluate.strict_json
    monkeypatch.setattr(p3_evaluate, "strict_json",
                        lambda path: summary if Path(path).name == "summary.json" else real(path))
    for name in ("common", "mandatory_artifacts", "p2_component", "native_commands", "mc_component"):
        monkeypatch.setattr(p3_evaluate, name, lambda *a, **k: None)
    monkeypatch.setattr(p3_evaluate, "evaluate_stream", lambda *a, **k: {"failures": []})
    monkeypatch.setattr(p3_evaluate, "evaluate_rgb", lambda *a, **k: {"failures": []})
    monkeypatch.setattr(p3_evaluate, "native_rtf", lambda run: {"rtf": 0.5})
    summary.update(sensor_window_sim_s=[1, 2], controller_publishers_only_from_controller=True)
    result = p3_evaluate.evaluate("/tmp/x")
    assert (failure in result["failures"]) if failure else (result["status"] == "passed")
    assert result["acceptance_credit"] is (gate == "A" and result["status"] == "passed")


def test_full_offline_evaluation_passes_only_with_graceful_cleanup(tmp_path):
    import p3_evaluate
    run = synthetic_run(tmp_path, "plane2")
    finalize_synthetic(run)
    result = p3_evaluate.evaluate(run)
    assert result["status"] == "passed", result["failures"]
    assert result["acceptance_credit"] is True and len(result["evaluator_sha256"]) >= 8


@pytest.mark.parametrize("extra,ok", [
    ({"/rosout": ["rcl_interfaces/msg/Log"], "/parameter_events": ["rcl_interfaces/msg/ParameterEvent"]}, True),
    ({"/rosout": ["std_msgs/msg/String"]}, False),
    ({"/fmu/in/trajectory_setpoint": ["px4_msgs/msg/TrajectorySetpoint"]}, False)])
def test_adapter_publication_boundary_allows_only_exact_ros_infrastructure(tmp_path, extra, ok):
    run = synthetic_run(tmp_path, "plane4")
    path = run / "sensors/adapter-result.json"
    adapter = json.loads(path.read_text())
    adapter["observed_own_publications"].update(extra)
    path.write_text(json.dumps(adapter))
    summary = json.loads((run / "summary.json").read_text())
    result = p3_stream_eval.evaluate_stream(run, summary["measurement_window_sim_s"], VALIDATION, SENSORS)
    assert ("adapter_publication_boundary" not in result["failures"]) == ok


def test_early_runtime_abort_records_a_failed_evaluation(tmp_path):
    """Gate A c4b00bed readiness aborted before any stack; the evaluator crashed and wrote nothing."""
    import p3_evaluate
    run = tmp_path / "20261025T053026Z-p3-readiness-test"
    run.mkdir()
    (run / "summary.json").write_text(json.dumps({"p3_kind": "readiness", "gate": "A", "status": "failed",
                                                  "failure": "Abort:mc_trace_build_differs_from_manifest"}))
    assert p3_evaluate.main([str(run)]) == 1
    result = json.loads((run / "p3-evaluation.json").read_text())
    assert result["status"] == "failed" and result["acceptance_credit"] is False
    assert result["failures"][0] == "runtime_status:Abort:mc_trace_build_differs_from_manifest"


def test_historical_gazebo_sigsegv_cannot_be_graceful(tmp_path):
    """Archived Gate A: every fixture's Gazebo exited -11 but generic cleanup said 'passed'."""
    import p3_evaluate
    run = synthetic_run(tmp_path, "plane2")
    finalize_synthetic(run, gazebo_exit=-11)
    result = p3_evaluate.evaluate(run)
    assert result["status"] == "failed" and "cleanup:gazebo:terminated_by_SIGSEGV" in result["failures"]
    assert result["acceptance_credit"] is False


def test_residual_process_or_changed_artifact_fails(tmp_path):
    import p3_evaluate
    run = synthetic_run(tmp_path, "plane2")
    finalize_synthetic(run, residual=[{"pid": 9, "cmdline": "gz sim", "state": "S"}])
    (run / "sensors/health.jsonl").write_text((run / "sensors/health.jsonl").read_text() + "\n")
    result = p3_evaluate.evaluate(run)
    assert "residual_namespace_processes" in result["failures"]
    assert any(f.startswith("artifact_hash_changed:sensors/health.jsonl") for f in result["failures"])


def test_missing_mc_trace_selection_cannot_pass():
    import p3_evaluate
    failures = p3_stream_eval.Failures()
    out = {}
    p3_evaluate.mc_component(None, {"identity": {}}, failures, out, True, None)
    assert failures.items == ["mc_causal_trace_not_selected"]
