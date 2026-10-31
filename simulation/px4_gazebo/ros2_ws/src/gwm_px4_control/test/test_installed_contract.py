import math

from gwm_px4_control.frames import enu_to_ned, offset_target, yaw_enu_to_ned
from gwm_px4_control.contracts import position_setpoint, encode_wire, strict_json, INITIALIZATION, NOMINAL


def test_package_coordinate_and_wire_contract():
    assert enu_to_ned((1, 0, 0)) == (0, 1, 0)
    assert enu_to_ned((0, 1, 0)) == (1, 0, 0)
    assert offset_target((5, -4, 7), (1, 0, 2)) == (5, -3, 5)
    assert math.isclose(yaw_enu_to_ned(0), math.pi/2)
    wire = position_setpoint((5, -3, 5), 0, 1000000)
    assert all(math.isnan(v) for v in wire["velocity"])
    assert 'NaN' not in strict_json(encode_wire(wire))


def test_installed_cdr_yaw_modes():
    from px4_msgs.msg import TrajectorySetpoint
    from rclpy.serialization import serialize_message, deserialize_message
    for mode,yaw_value in ((INITIALIZATION,None),(NOMINAL,.4)):
        fields=position_setpoint((5,-3,5),yaw_value,1000000,mode)
        msg=TrajectorySetpoint()
        for key,value in fields.items():setattr(msg,key,value)
        decoded=deserialize_message(serialize_message(msg),TrajectorySetpoint)
        evidence=encode_wire(fields,mode)
        assert all(math.isnan(v) for v in decoded.velocity)
        if mode==INITIALIZATION:
            assert math.isnan(decoded.yaw) and decoded.yawspeed==0.
            assert evidence['fields']['yaw'] is None and evidence['active']['yawspeed']
        else:
            assert math.isclose(decoded.yaw,.4,abs_tol=1e-7) and math.isnan(decoded.yawspeed)
