#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.speed_list = []
        self.index = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.index < len(self.speed_list):
            msg = Twist()
            speed_params = self.speed_list[self.index]
            msg.linear.x = speed_params[0]
            msg.angular.z = speed_params[1]
            self.publisher_.publish(msg)
            self.get_logger().info(f'Sended: linear={speed_params[0]}, angular={speed_params[1]}')
            self.index += 1
        else:
            self.get_logger().info('Done')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    p = 6.28318530718
    node.speed_list = [
        [p, 6.28318530718],
        [p * 1.7, -6.28318530718],
    ]
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

