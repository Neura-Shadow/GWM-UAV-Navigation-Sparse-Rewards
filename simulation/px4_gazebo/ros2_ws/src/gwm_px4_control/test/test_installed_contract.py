import math

from gwm_px4_control.frames import enu_to_ned, offset_target, yaw_enu_to_ned
from gwm_px4_control.contracts import position_setpoint, encode_wire, strict_json


def test_package_coordinate_and_wire_contract():
    assert enu_to_ned((1, 0, 0)) == (0, 1, 0)
    assert enu_to_ned((0, 1, 0)) == (1, 0, 0)
    assert offset_target((5, -4, 7), (1, 0, 2)) == (5, -3, 5)
    assert math.isclose(yaw_enu_to_ned(0), math.pi/2)
    wire = position_setpoint((5, -3, 5), 0, 1000000)
    assert all(math.isnan(v) for v in wire["velocity"])
    assert 'NaN' not in strict_json(encode_wire(wire))
