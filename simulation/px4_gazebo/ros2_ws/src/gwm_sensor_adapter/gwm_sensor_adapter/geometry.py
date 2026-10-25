"""Explicit camera-optical, vehicle-body and PX4 local-frame geometry.

Conventions (all explicit, none inferred from header frame IDs):

* Optical: X image right, Y image down, Z optical forward. Depth is optical Z,
  not Euclidean range along the pixel ray.
* Body FLU: X forward, Y left, Z up (Gazebo link convention, REP 103).
* Body FRD: X forward, Y right, Z down (PX4).
* Local NED: PX4 VehicleLocalPosition frame; ENU is [east, north, up].
* Quaternions are Hamilton [w, x, y, z]; PX4 VehicleAttitude.q rotates body FRD
  vectors into NED.

The full rigid transform is p_body = R_body_optical * p_optical + t_body_optical.
For the pinned x500_depth composition every SDF rotation is zero, so the only
rotation is the fixed optical-axis convention change below; the translation is
re-derived from the SDF chain at build time (see `sdf_chain`).
"""
import math

import numpy as np

# Columns are the optical X/Y/Z axes expressed in body FLU.
R_FLU_FROM_OPTICAL = np.array([[0.0, 0.0, 1.0],
                               [-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0]])
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def intrinsics(calibration):
    return calibration["fx"], calibration["fy"], calibration["cx"], calibration["cy"]


def backproject(u, v, depth_z, calibration):
    """Pixel (u, v) and optical-axis depth -> optical point; X=(u-cx)Z/fx, Y=(v-cy)Z/fy."""
    fx, fy, cx, cy = intrinsics(calibration)
    return np.array([(u - cx) * depth_z / fx, (v - cy) * depth_z / fy, depth_z], dtype=float)


def ray_length(u, v, depth_z, calibration):
    """Euclidean camera-centre range for an optical-axis depth (always >= depth)."""
    return float(np.linalg.norm(backproject(u, v, depth_z, calibration)))


class Extrinsic:
    def __init__(self, translation_base_flu, rotation_flu_from_optical=R_FLU_FROM_OPTICAL, identity=None):
        self.t = np.asarray(translation_base_flu, dtype=float).reshape(3)
        self.r = np.asarray(rotation_flu_from_optical, dtype=float).reshape(3, 3)
        if not np.allclose(self.r @ self.r.T, np.eye(3), atol=1e-12) or not math.isclose(np.linalg.det(self.r), 1.0):
            raise ValueError("Extrinsic rotation must be proper orthonormal")
        self.identity = identity

    def optical_to_flu(self, point):
        return self.r @ np.asarray(point, dtype=float) + self.t

    def optical_to_frd(self, point):
        return FLU_TO_FRD @ self.optical_to_flu(point)


def quaternion_matrix(q_wxyz):
    q = np.asarray(q_wxyz, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Invalid quaternion")
    norm = float(np.linalg.norm(q))
    if not 0.95 <= norm <= 1.05:
        raise ValueError("Invalid quaternion norm")
    w, x, y, z = q / norm
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def frd_to_local_ned(point_frd, position_ned, q_wxyz):
    return np.asarray(position_ned, dtype=float) + quaternion_matrix(q_wxyz) @ np.asarray(point_frd, dtype=float)


def ned_to_enu(vector):
    n, e, d = vector
    return np.array([e, n, -d], dtype=float)


def rpy_matrix(roll, pitch, yaw):
    """SDF pose rotation: extrinsic X-Y-Z (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def compose(parent, child):
    """Compose (R, t) poses: child expressed in parent -> child expressed in parent's frame."""
    return parent[0] @ child[0], parent[0] @ child[1] + parent[1]


def pose_from_text(text):
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Unsupported SDF pose")
    return rpy_matrix(*values[3:]), np.array(values[:3])


def sdf_chain(load_model, root_model, base_link, sensor_name):
    """Resolve link/sensor poses through merged includes of the pinned SDF tree.

    `load_model(uri)` returns an ElementTree <model> element. Only the default
    pose frame (parent model / link) is supported; explicit relative_to other
    than the enclosing frame is rejected rather than guessed.
    """
    links, sensors = {}, {}

    def walk(model, model_pose):
        for element in model:
            if element.tag == "include":
                pose = element.find("pose")
                if pose is not None and pose.get("relative_to") not in (None, "__model__"):
                    raise ValueError("Unsupported include pose frame")
                uri = element.findtext("uri").strip()
                walk(load_model(uri), compose(model_pose, pose_from_text(pose.text if pose is not None else None)))
            elif element.tag == "link":
                pose = element.find("pose")
                if pose is not None and pose.get("relative_to") not in (None, "__model__"):
                    raise ValueError("Unsupported link pose frame")
                link_pose = compose(model_pose, pose_from_text(pose.text if pose is not None else None))
                links[element.get("name")] = link_pose
                for sensor in element.findall("sensor"):
                    spose = sensor.find("pose")
                    if spose is not None and spose.get("relative_to") not in (None, element.get("name")):
                        raise ValueError("Unsupported sensor pose frame")
                    sensors[sensor.get("name")] = (element.get("name"), compose(
                        link_pose, pose_from_text(spose.text if spose is not None else None)))

    identity = (np.eye(3), np.zeros(3))
    walk(load_model(root_model), identity)
    if base_link not in links or sensor_name not in sensors:
        raise ValueError("Required link or sensor absent from pinned SDF chain")
    base_r, base_t = links[base_link]
    link_name, (sensor_r, sensor_t) = sensors[sensor_name]
    relative_r = base_r.T @ sensor_r
    relative_t = base_r.T @ (sensor_t - base_t)
    return {"base_origin_model_m": base_t.tolist(), "sensor_origin_model_m": sensor_t.tolist(),
            "sensor_link": link_name, "sensor_rotation_base": relative_r.tolist(),
            "optical_origin_base_flu_m": relative_t.tolist(),
            "all_rotations_identity": bool(np.allclose(relative_r, np.eye(3), atol=1e-12)
                                           and np.allclose(base_r, np.eye(3), atol=1e-12))}
