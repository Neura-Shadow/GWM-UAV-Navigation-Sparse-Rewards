"""Aligned ENU/NED, FLU/FRD and Hamilton body-to-world rotations."""
import math


def finite(values, count):
    if len(values) != count or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                    or not math.isfinite(v) for v in values):
        raise ValueError("Expected finite numeric coordinates")
    return tuple(float(v) for v in values)


def wrap(angle):
    finite([angle], 1)
    return (angle + math.pi) % (2 * math.pi) - math.pi


def enu_to_ned(vector):
    east, north, up = finite(vector, 3)
    return (north, east, -up)


def ned_to_enu(vector):
    north, east, down = finite(vector, 3)
    return (east, north, -down)


def flu_to_frd(vector):
    front, left, up = finite(vector, 3)
    return (front, -left, -up)


def frd_to_flu(vector):
    return flu_to_frd(vector)


def yaw_enu_to_ned(angle):
    return wrap(math.pi / 2 - angle)


def yaw_ned_to_enu(angle):
    return wrap(math.pi / 2 - angle)


def offset_target(origin_ned, offset_enu):
    return tuple(a + b for a, b in zip(finite(origin_ned, 3), enu_to_ned(offset_enu)))


def normalized_quaternion(q):
    q = finite(q, 4)
    norm = math.sqrt(sum(v * v for v in q))
    if not 0.95 <= norm <= 1.05:
        raise ValueError("Invalid attitude quaternion norm")
    return tuple(v / norm for v in q)


def body_frd_to_ned(vector, q_wxyz):
    x, y, z = finite(vector, 3)
    w, a, b, c = normalized_quaternion(q_wxyz)
    return ((1 - 2 * (b*b + c*c))*x + 2*(a*b - w*c)*y + 2*(a*c + w*b)*z,
            2*(a*b + w*c)*x + (1 - 2*(a*a + c*c))*y + 2*(b*c - w*a)*z,
            2*(a*c - w*b)*x + 2*(b*c + w*a)*y + (1 - 2*(a*a + b*b))*z)


def ned_to_body_frd(vector, q_wxyz):
    w, x, y, z = normalized_quaternion(q_wxyz)
    return body_frd_to_ned(vector, (w, -x, -y, -z))


def yaw_from_quaternion(q_wxyz):
    w, x, y, z = normalized_quaternion(q_wxyz)
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
