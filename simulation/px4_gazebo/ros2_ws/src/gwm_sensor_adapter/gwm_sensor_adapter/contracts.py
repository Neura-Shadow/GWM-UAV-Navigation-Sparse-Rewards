"""Explicit P3 ground profile allowlist. No arbitrary endpoints or flight mode."""
import json
from pathlib import Path


def run_identity(directory, explicit=None):
    """Flight recording lives in a sensors child of the unique run directory."""
    directory = Path(directory)
    actual=directory.parent.name if directory.name == 'sensors' else directory.name
    if not actual or actual in ('.','..','sensors') or explicit is not None and explicit!=actual:
        raise ValueError('run_identity_mismatch')
    return actual


def load_config(path):
    c=json.loads(Path(path).read_text())
    expected=dict(schema_version=1,sample_evidence_contract='p3-sample-evidence-v2',profile='p3-x500-depth-native-shm64-v1',model='x500_depth_71',world='gwm_p3_ground',
        width=640,height=480,sensor_hz=30,processing_hz=10,near_m=.2,far_m=19.1,horizontal_fov_rad=1.274,
        encoding='32FC1',raw_frame='camera_link',optical_frame='gwm_front_depth_optical',
        origin_model_m=[.13233,0,.26078],origin_base_flu_m=[.13233,0,.02078],
        native_rgb='retained_1920x1080_30Hz_no_RGB_bridge',
        image_topic='/gwm/sensors/front_depth/image_raw',info_topic='/gwm/sensors/front_depth/camera_info',
        observation_topic='/gwm/sensors/front_depth/observation',health_topic='/gwm/sensors/front_depth/health',
        source_age_sim_s=.25,stall_wall_s=1.,recorder_capacity=8,state_capacity=300,state_mismatch_s=.05,
        require_px4_state=False)
    if c.get('world')=='gwm_p3_flight':
        expected.update(world='gwm_p3_flight',require_px4_state=True)
    if c!=expected or any(type(c[k]) is not type(v) for k,v in expected.items()):
        raise ValueError('unsupported_p3_profile')
    return c
