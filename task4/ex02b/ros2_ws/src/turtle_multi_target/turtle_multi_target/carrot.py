import math

# messages
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
# rclpy
import rclpy
from rclpy.node import Node
# tf2
from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):
    def __init__(self):
        super().__init__("dynamic_frame_tf2_broadcaster")
        self.radius = self.declare_parameter(
            "radius", 2.
        ).get_parameter_value().double_value
        self.direction = self.declare_parameter(
            'direction_of_rotation', 1
        ).get_parameter_value().integer_value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        seconds, nanosecs = self.get_clock().now().seconds_nanoseconds()
        time = (seconds + nanosecs * 0.000000001)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = self.radius * math.cos((-1) * time*self.direction)
        t.transform.translation.y = self.radius * math.sin((-1) * time*self.direction)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.get_logger().info(f"Publishing transform from {t.header.frame_id} to {t.child_frame_id}")

        self.tf_broadcaster.sendTransform(t)
        

def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    