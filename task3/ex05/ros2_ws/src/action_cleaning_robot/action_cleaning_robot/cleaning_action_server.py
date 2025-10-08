#!/usr/bin/env python3
import time
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

        self.get_logger().info("Server ready to work")
    
    def execute_callback(self, goal_handle):
        task_type = goal_handle.request.task_type
        area_size = goal_handle.request.area_size
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        self.get_logger().info(f"Received new goal: type=\"{task_type}\" | \
                                 area={area_size} | \
                                 target position: x={target_x:.6f} y={target_y:.6f}")
        
        cleaning_size = 100
        
        current_cleaned_points = 0
        current_distance = 0.0
        current_x = 0.0
        current_y = 0.0

        n = 10
        
        result = CleaningTask.Result()
        for i in range(1, n + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result.success = False
                result.cleaned_points = current_cleaned_points
                result.total_distance = current_distance
                    
                return result
            
            time.sleep(1.0)
            
            current_cleaned_points += cleaning_size
            current_distance += math.sqrt((i * 0.1) ** 2 + (i * 0.3) ** 2)
            current_x += i * 0.1
            current_y += i * 0.3

            feedback_msg = CleaningTask.Feedback()
            feedback_msg.progress_percent = int((i / n) * 100)
            feedback_msg.current_cleaned_points = current_cleaned_points
            feedback_msg.current_x = current_x
            feedback_msg.current_y = current_y
            
            self.get_logger().info(f"Sending progress: progress={int((i / n) * 100)}% | \
                                     cleaned={current_cleaned_points} | \
                                     position: x={current_x:.6f} y={current_y:.6f}")

            goal_handle.publish_feedback(feedback_msg)
        
        goal_handle.succeed()

        result.success = True
        result.cleaned_points = current_cleaned_points
        result.total_distance = current_distance

        self.get_logger().info("Goal completed successfully!")
        
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
