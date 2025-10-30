import math
import threading
import select
import sys

# messages
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
# rclpy
import rclpy
from rclpy.node import Node
# tf2
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import TransformException


class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        
        self.switch_threshold = self.declare_parameter('switch_threshold', 1.0).value
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        self.current_target = self.targets[self.current_target_index]
        
        self.angle_offset = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.switch_timer = self.create_timer(0.5, self.check_auto_switch)
        
        self.target_pub = self.create_publisher(String, '/current_target_name', 10)
        
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        self.get_logger().info(f'Target switcher started. Initial target: {self.current_target}')
        self.get_logger().info('Press "n" to manually switch targets')

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        self.angle_offset += 0.1
        
        carrot1_tf = TransformStamped()
        carrot1_tf.header.stamp = current_time
        carrot1_tf.header.frame_id = 'turtle1'
        carrot1_tf.child_frame_id = 'carrot1'
        radius1 = 2.0
        carrot1_tf.transform.translation.x = radius1 * math.cos(self.angle_offset)
        carrot1_tf.transform.translation.y = radius1 * math.sin(self.angle_offset)
        carrot1_tf.transform.translation.z = 0.0
        q1 = self.quaternion_from_euler(0, 0, self.angle_offset + math.pi)
        carrot1_tf.transform.rotation.x = q1[0]
        carrot1_tf.transform.rotation.y = q1[1]
        carrot1_tf.transform.rotation.z = q1[2]
        carrot1_tf.transform.rotation.w = q1[3]
        self.tf_broadcaster.sendTransform(carrot1_tf)
        
        carrot2_tf = TransformStamped()
        carrot2_tf.header.stamp = current_time
        carrot2_tf.header.frame_id = 'turtle3'
        carrot2_tf.child_frame_id = 'carrot2'
        radius2 = 3.0
        carrot2_tf.transform.translation.x = radius2 * math.cos(self.angle_offset + math.pi)
        carrot2_tf.transform.translation.y = radius2 * math.sin(self.angle_offset + math.pi)
        carrot2_tf.transform.translation.z = 0.0
        q2 = self.quaternion_from_euler(0, 0, self.angle_offset)
        carrot2_tf.transform.rotation.x = q2[0]
        carrot2_tf.transform.rotation.y = q2[1]
        carrot2_tf.transform.rotation.z = q2[2]
        carrot2_tf.transform.rotation.w = q2[3]
        self.tf_broadcaster.sendTransform(carrot2_tf)
        
        static_tf = TransformStamped()
        static_tf.header.stamp = current_time
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = 'static_target'
        static_tf.transform.translation.x = 8.0
        static_tf.transform.translation.y = 2.0
        static_tf.transform.translation.z = 0.0
        qs = self.quaternion_from_euler(0, 0, 0)
        static_tf.transform.rotation.x = qs[0]
        static_tf.transform.rotation.y = qs[1]
        static_tf.transform.rotation.z = qs[2]
        static_tf.transform.rotation.w = qs[3]
        self.tf_broadcaster.sendTransform(static_tf)
        
        target_msg = String()
        target_msg.data = self.current_target
        self.target_pub.publish(target_msg)

    def check_auto_switch(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'turtle2',
                self.current_target,
                rclpy.time.Time()
            )
            distance = math.sqrt(
                transform.transform.translation.x ** 2 + 
                transform.transform.translation.y ** 2
            )
            if distance < self.switch_threshold:
                self.switch_to_next_target()
        except TransformException:
            pass

    def switch_to_next_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f'Auto-switched to: {self.current_target}')

    def switch_target_manual(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f'Manual switch to: {self.current_target}')

    def keyboard_listener(self):
        while rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.readline().strip()
                    if key == 'n':
                        self.switch_target_manual()
            except Exception:
                pass

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q


def main(args=None):
    rclpy.init(args=args)
    node = TargetSwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
