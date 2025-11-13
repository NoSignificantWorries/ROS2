import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


def normalize_angle_rad(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def shortest_angle_diff_rad(current, target):
    return normalize_angle_rad(target - current)


class LisajousPublisher(Node):
    def __init__(self):
        super().__init__('my_movement')
        
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.last_xp = 0.0
        self.last_yp = 0.0
        self.last_ang = 0.0

        self.x_amplitude = 2.0
        self.y_amplitude = 1.5
        self.omega_x = 1.0
        self.omega_y = 2.0
        self.phi_x = 0.0
        self.phi_y = math.pi / 2
        
        self.start_time = self.get_clock().now()
        self.get_logger().info("Lisajous movement node started")

    def timer_callback(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        xp = self.x_amplitude * math.cos(self.omega_x * t + self.phi_x)
        yp = self.y_amplitude * math.cos(self.omega_y * t + self.phi_y)

        dx = xp - self.last_xp
        dy = yp - self.last_yp
        dist = math.sqrt(dx**2 + dy**2)
        linear_vel = dist / 0.02

        max_linear_vel = 0.3
        if abs(linear_vel) > max_linear_vel:
            linear_vel = math.copysign(max_linear_vel, linear_vel)

        if abs(dx) > 1e-6 or abs(dy) > 1e-6:
            target_ang = math.atan2(dy, dx)
        else:
            target_ang = self.last_ang

        angle_diff = shortest_angle_diff_rad(self.last_ang, target_ang)
        angular_vel = angle_diff / 0.02

        max_angular_vel = 1.0
        if abs(angular_vel) > max_angular_vel:
            angular_vel = math.copysign(max_angular_vel, angular_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        
        self.publisher_.publish(twist_msg)
        
        if int(t * 10) % 50 == 0:
            self.get_logger().info(f'Linear: {linear_vel:.3f} m/s, Angular: {angular_vel:.3f} rad/s')
            self.get_logger().info(f'Position: x={xp:.2f}, y={yp:.2f}, target_angle={target_ang:.2f}')

        self.last_xp = xp
        self.last_yp = yp
        self.last_ang = target_ang

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LisajousPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {str(e)}")
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
