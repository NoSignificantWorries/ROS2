import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.move)
        self.get_logger().info('Circle movement node started!')

    def move(self):
        msg = Twist()
        msg.linear.x = 0.6
        msg.angular.z = 1.2
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
