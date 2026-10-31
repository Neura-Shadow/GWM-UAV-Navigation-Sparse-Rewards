"""Installed-package checks: real ROS message classes, still no ROS node created."""
import numpy as np

from gwm_sensor_adapter.contracts import FORBIDDEN_TYPES
from gwm_sensor_adapter.depth import NO_RETURN, VALID, classify


def test_installed_classification():
    codes = classify(np.array([[1.0, np.inf]], dtype="<f4"), 0.2, 19.1)
    assert codes.tolist() == [[VALID, NO_RETURN]]
    assert "px4_msgs/msg/VehicleCommand" in FORBIDDEN_TYPES


def test_installed_image_layout_round_trip():
    from sensor_msgs.msg import Image
    from rclpy.serialization import deserialize_message, serialize_message
    from gwm_sensor_adapter.depth import decode
    values = np.arange(640 * 480, dtype="<f4").reshape(480, 640) / 1000 + 1
    msg = Image(width=640, height=480, encoding="32FC1", is_bigendian=0, step=2560, data=values.tobytes())
    decoded = deserialize_message(serialize_message(msg), Image)
    fields = {"width": decoded.width, "height": decoded.height, "encoding": decoded.encoding,
              "is_bigendian": decoded.is_bigendian, "step": decoded.step, "data": memoryview(decoded.data).cast("B")}
    image = decode(fields, {"width": 640, "height": 480, "encoding": "32FC1"})
    assert np.array_equal(image, values)
