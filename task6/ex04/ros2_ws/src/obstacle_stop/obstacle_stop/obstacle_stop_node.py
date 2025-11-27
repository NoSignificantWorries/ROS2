import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopNode(Node):
    def __init__(self):
        super().__init__('obstacle_stop_node')

        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/robot/scan', self.scan_callback, 10)

        self.safe_distance = 0.8
        self.max_speed = 0.2

        self.get_logger().info('Obstacle stop node started')

    def scan_callback(self, scan: LaserScan):
        front_ranges = scan.ranges[len(scan.ranges)//3: 2*len(scan.ranges)//3]  
        front_dist = min(front_ranges) if front_ranges else float('inf')

        tolerance = 0.05

        twist = Twist()
        if front_dist > (self.safe_distance + tolerance):
            # Двигаемся вперед
            twist.linear.x = self.max_speed
            twist.angular.z = 0.0
            self.get_logger().info(f'Path clear: ({front_dist:.2f} m). Moving forward.')
        elif front_dist < (self.safe_distance - tolerance):
            # Припятствие близко - отъехать
            twist.linear.x = -0.05
            twist.angular.z = 0.0
            self.get_logger().info(f'Path not clear: ({front_dist:.2f} m). Moving back.')
        else:
            # Остановка
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f'At safe distance ({front_dist:.2f} m). Stopping.')

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
