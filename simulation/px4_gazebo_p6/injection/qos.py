"""QoS profiles of the P6 ROS nodes (safe to import without ROS; rclpy is imported inside the functions).

* A relay copies the source publisher's reliability and durability, which its unchanged consumers already
  match. Graph endpoint information reports no history or depth (history "unknown", depth 0, M5 finding),
  so those are set explicitly: keep-last with the declared depth (the accepted bridge's `publisher_queue`).
* `/clock` is published best-effort by the clock bridge; a reliable subscription never matches it (M5
  finding), so every P6 node subscribes to it best-effort.
"""
DEPTH_QUEUE = 8  # p3_sensor_bridge.yaml publisher_queue / subscriber_queue
STATE_QUEUE = 10


def relay_profile(endpoint_qos, depth):
    from rclpy.qos import HistoryPolicy, QoSProfile
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth, reliability=endpoint_qos.reliability,
                      durability=endpoint_qos.durability)


def clock_profile():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)
