def test_installed_depth_transport_cdr_layout():
    import numpy as np
    from sensor_msgs.msg import Image
    from rclpy.serialization import serialize_message, deserialize_message
    from gwm_sensor_adapter.depth import decode, classify
    msg = Image(width=2,height=1,encoding='32FC1',step=8,is_bigendian=0,
                data=np.array([4.,float('inf')],dtype='<f4').tobytes())
    actual=deserialize_message(serialize_message(msg),Image)
    image=decode(bytes(actual.data),actual.width,actual.height,actual.encoding,actual.step,actual.is_bigendian)
    assert classify(image,.2,19.1).tolist()==[[0,3]]
