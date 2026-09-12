"""The pinned, undistorted OakD depth calibration and resolved SDF extrinsics."""
import hashlib
import json
import numpy as np

EXTRINSIC_ID = 'b6127f4-x500-depth-optical-to-base-v1'
OPTICAL_FRAME = 'gwm_front_depth_optical'
ORIGIN_FLU = np.array([.13233, 0., .02078])


class Calibration:
    def __init__(self, info):
        try:
            self.info = json.loads(json.dumps(info, allow_nan=False))
            c = self.info
            if c['width'] != 640 or c['height'] != 480 or c['frame_id'] != 'camera_link':
                raise ValueError('calibration_identity')
            k, p, r = (np.asarray(c[key], dtype=float) for key in ('k','p','r'))
            if k.shape != (9,) or p.shape != (12,) or r.shape != (9,):
                raise ValueError('calibration_matrix')
            if not all(np.isfinite(a).all() for a in (k,p,r)):
                raise ValueError('calibration_nonfinite')
            if c['distortion_model'] not in ('', 'plumb_bob') or any(v != 0 for v in c['d']):
                raise ValueError('unsupported_distortion')
            if c['binning_x'] not in (0,1) or c['binning_y'] not in (0,1) or any(c['roi'].values()):
                raise ValueError('unsupported_roi_binning')
            fx, fy, cx, cy = p[0], p[5], p[2], p[6]
            expected = [fx,0,cx,0,fy,cy,0,0,1]
            if not (fx > 0 and fy > 0 and 0 <= cx < 640 and 0 <= cy < 480):
                raise ValueError('calibration_intrinsics')
            if not (np.allclose(k, expected, atol=1e-9) and np.allclose(r, np.eye(3).ravel(), atol=1e-9)
                    and np.allclose(p, [fx,0,cx,0,0,fy,cy,0,0,0,1,0], atol=1e-9)):
                raise ValueError('unsupported_rectification')
            self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
            self.identity = hashlib.sha256(json.dumps(c, sort_keys=True).encode()).hexdigest()
        except (TypeError, KeyError, OverflowError) as exc:
            raise ValueError('calibration_unavailable') from exc

    def point(self, u, v, depth):
        if not np.isfinite([u,v,depth]).all() or depth <= 0:
            raise ValueError('invalid_point')
        return [(u-self.cx)*depth/self.fx, (v-self.cy)*depth/self.fy, depth]


def optical_to_body(point, raw_frame):
    if raw_frame != 'camera_link':
        raise ValueError('transform_unavailable')
    x,y,z = point
    flu = np.array([z,-x,-y]) + ORIGIN_FLU
    return flu.tolist(), (flu*np.array([1,-1,-1])).tolist()
