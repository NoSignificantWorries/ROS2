#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Messages
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# My actioin format
from action_cleaning_robot.action import CleaningTask


class CleaningActionServer(Node):
    def __init__(self) -> None:
        super().__init__("cleaning_action_server")
        self._action_server = ActionServer(
            self,
            CleaningTask,
            "CleaningTask",
            self.execute_callback
        )
    
    def execute_callback(self, goal_handle):
        ...


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Working interrupted by keyboard")
        node.destroy_node()


if __name__ == "__main__":
    main()
