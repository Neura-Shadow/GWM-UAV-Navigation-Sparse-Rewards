"""Pure P3 behavior; never starts a ROS or simulator runtime."""
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1] / 'simulation/px4_gazebo/ros2_ws/src/gwm_sensor_adapter'
sys.path.insert(0, str(PKG))


def test_depth_layout_honors_padding_endian_and_preserves_unknowns():
    from gwm_sensor_adapter.depth import decode, classify, Reason
    data = np.array([1, np.nan, np.inf, -np.inf], dtype='>f4').tobytes()
    raw = data[:8] + b'PAD!' + data[8:] + b'PAD!'
    depth = decode(raw, 2, 2, '32FC1', 12, 1)
    mask = classify(depth, .2, 19.1)
    assert mask.tolist() == [[Reason.VALID, Reason.INVALID], [Reason.NO_RETURN, Reason.TOO_CLOSE]]
    assert np.isposinf(depth[1, 0])


def calibration():
    return dict(width=640, height=480, frame_id='camera_link', distortion_model='plumb_bob',
                d=[0.] * 5, k=[430.,0.,320.,0.,430.,240.,0.,0.,1.],
                p=[430.,0.,320.,0.,0.,430.,240.,0.,0.,0.,1.,0.],
                r=[1.,0.,0.,0.,1.,0.,0.,0.,1.], binning_x=0, binning_y=0,
                roi=dict(x_offset=0,y_offset=0,width=0,height=0,do_rectify=False))


def test_optical_to_body_uses_full_origin_and_signed_axes():
    from gwm_sensor_adapter.geometry import Calibration, optical_to_body
    c = Calibration(calibration())
    xyz = c.point(420, 140, 4.)
    flu, frd = optical_to_body(xyz, 'camera_link')
    assert flu == pytest.approx([4.13233, -400/430, 400/430+.02078])
    assert frd == pytest.approx([flu[0], -flu[1], -flu[2]])


@pytest.mark.parametrize('change', [dict(width=1920), dict(frame_id='rgb'), dict(d=[.1]*5),
    dict(k=[0.]*9), dict(p=[float('nan')]*12), dict(binning_x=2), dict(r=[0.]*9)])
def test_incompatible_calibration_rejected(change):
    from gwm_sensor_adapter.geometry import Calibration
    c = calibration(); c.update(change)
    with pytest.raises(ValueError): Calibration(c)


@pytest.mark.parametrize('change', [dict(encoding='16UC1'), dict(step=3), dict(is_bigendian=2),
    dict(width=0), dict(data=b'123'), dict(width=True)])
def test_bad_image_rejected(change):
    from gwm_sensor_adapter.depth import decode
    args = dict(data=b'\0'*16,width=2,height=2,encoding='32FC1',step=8,is_bigendian=0)
    args.update(change)
    with pytest.raises(ValueError): decode(**args)


def test_health_does_not_refresh_duplicate_and_latches_backwards_time():
    from gwm_sensor_adapter.state import Health
    h = Health(.25, 1.)
    h.receive(1., 10.)
    with pytest.raises(ValueError, match='repeated'): h.receive(1., 10.9)
    assert h.status(1.3, 10.95) == 'stale'
    assert h.status(1., 11.1) == 'unavailable'
    with pytest.raises(ValueError, match='backwards'): h.receive(.9, 11.2)
    assert h.status(1., 11.3) == 'backwards_time'


def test_bounded_acquisition_state_association_rejects_reset_boundary():
    from gwm_sensor_adapter.state import StateHistory, NewestSlot
    h = StateHistory(3, .05)
    h.add(1., (0,0), {'id':1}); h.add(1.02, (0,0), {'id':2})
    assert h.match(1.001)['state']['id'] == 1
    h.add(1.04, (1,0), {'id':3})
    assert h.match(1.03) is None
    assert h.match(2.) is None
    h.add(1.06, (1,0), {'id':4})
    assert len(h.items) == 3
    slot=NewestSlot(); slot.put(1); slot.put(2)
    assert slot.take() == 2 and slot.overwritten == 1 and slot.take() is None


def test_unknown_transform_and_missing_calibration_are_explicit():
    from gwm_sensor_adapter.geometry import Calibration, optical_to_body
    with pytest.raises(ValueError): Calibration(None)
    with pytest.raises(ValueError, match='transform'): optical_to_body([0,0,1], 'unknown')


def test_recorder_preserves_every_raw_byte_and_completes_index(tmp_path):
    import json
    from gwm_sensor_adapter.recording import RawRecorder
    recorder=RawRecorder(tmp_path,8)
    recorder.submit({'id':1},b'one'); recorder.submit({'id':2},b'two')
    result=recorder.close()
    assert result['written']==result['accepted']==2 and result['overflow']==0
    assert (tmp_path/'depth.bin').read_bytes()==b'onetwo'
    rows=[json.loads(line) for line in (tmp_path/'depth-index.jsonl').read_text().splitlines()]
    assert [r['offset'] for r in rows]==[0,3]


def test_recorder_failure_cannot_create_success_receipt(tmp_path):
    from gwm_sensor_adapter.recording import RawRecorder
    recorder=RawRecorder(tmp_path/'absent',1)
    with pytest.raises(RuntimeError,match='recorder_failure'): recorder.close()
    with pytest.raises(RuntimeError,match='recorder_failure'): recorder.submit({'id':1},b'x')


def test_full_recorder_buffer_latches_failure(tmp_path,monkeypatch):
    import threading
    import gwm_sensor_adapter.recording as recording
    entered,release=threading.Event(),threading.Event()
    original=recording.hashlib.sha256
    def blocked_hash(raw):
        entered.set(); assert release.wait(3)
        return original(raw)
    monkeypatch.setattr(recording.hashlib,'sha256',blocked_hash)
    recorder=recording.RawRecorder(tmp_path,1)
    try:
        recorder.submit({'id':1},b'x'); assert entered.wait(3)
        recorder.submit({'id':2},b'y')
        with pytest.raises(RuntimeError,match='overflow'): recorder.submit({'id':3},b'z')
    finally:
        release.set()
        with pytest.raises(RuntimeError,match='overflow'): recorder.close()


def test_all_invalid_and_mixed_pixels_never_become_free_space():
    from gwm_sensor_adapter.depth import classify
    d=np.array([[np.nan,np.inf,-np.inf,0,19.1,20,.1,2]],dtype=float)
    assert classify(d,.2,19.1).tolist()==[[1,3,2,2,3,3,2,0]]


def test_independent_evaluator_does_not_import_production_geometry():
    import importlib.util
    path=PKG.parents[2]/'validation/collect_p3_evidence.py'
    spec=importlib.util.spec_from_file_location('p3_evaluator_test',path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    fixture=dict(optical_origin_model_m=[.13233,0,.26078],
        boxes=[dict(center=[4.18233,0,4],size=[.1,20,20],yaw_rad=0)])
    depth,labels,ray=module.expected_depth(np.array([320.,420.]),np.array([140.,140.]),
        calibration(),np.array([0,0,-.013]),np.eye(3),fixture)
    assert depth==pytest.approx([4,4]) and (labels==0).all() and (ray>1).all()
    assert 'gwm_sensor_adapter' not in path.read_text()


@pytest.mark.parametrize('change',[dict(world='default'),dict(model='x500_71'),dict(sensor_hz=15),
    dict(image_topic='/arbitrary'),dict(require_px4_state=True),dict(processing_hz=True)])
def test_registered_ground_profile_rejects_drift(tmp_path,change):
    import json
    from gwm_sensor_adapter.contracts import load_config
    source=PKG.parents[2]/'configs/p3_sensors.yaml'
    c=load_config(source); c.update(change)
    p=tmp_path/'bad.json'; p.write_text(json.dumps(c))
    with pytest.raises(ValueError,match='unsupported'): load_config(p)


def test_adapter_has_no_flight_or_truth_transport():
    import ast
    code=(PKG/'gwm_sensor_adapter/node.py').read_text()
    for forbidden in ('VehicleCommand','TrajectorySetpoint','OffboardControlMode','ObstacleDistance','gz.transport','pose/info'):
        assert forbidden not in code
    tree=ast.parse(code)
    imports=[n for n in tree.body if isinstance(n,(ast.Import,ast.ImportFrom))]
    assert all(not getattr(n,'module','').startswith('rclpy') for n in imports if getattr(n,'module',None))


def test_depth_flight_profile_is_explicit_and_keeps_p2_limits(tmp_path):
    import json
    sys.path.insert(0,str(PKG.parent/'gwm_px4_control'))
    from gwm_px4_control.contracts import load_config
    baseline=json.loads((PKG.parents[2]/'configs/p2_control.yaml').read_text())
    depth=dict(baseline,world='gwm_p3_flight',model='x500_depth_71',simulation_profile='p3-depth-coexistence-v1')
    p=tmp_path/'config.json'; p.write_text(json.dumps(depth))
    assert load_config(p)['max_sample_gap_sim_s']==.2
    depth['world']='arbitrary'; p.write_text(json.dumps(depth))
    with pytest.raises(ValueError): load_config(p)


def test_processing_schedule_preserves_sim_rate_when_physics_is_slower():
    from gwm_sensor_adapter.state import ProcessingSchedule
    schedule=ProcessingSchedule(10)
    times=[.79*wall for wall in np.arange(0,10,.02)]
    processed=[t for t in times if schedule.due(t)]
    assert len(processed)==79
    assert max(np.diff(processed))<.12
    assert schedule.due(100.)
    assert not schedule.due(100.)  # bounded newest only; never replay backlog
    with pytest.raises(ValueError): schedule.due(99.)


def test_observation_identity_uses_enclosing_flight_run_not_sensor_folder():
    from gwm_sensor_adapter.contracts import run_identity
    assert run_identity(Path('/runs/ground-123')) == 'ground-123'
    assert run_identity(Path('/runs/flight-456/sensors')) == 'flight-456'
