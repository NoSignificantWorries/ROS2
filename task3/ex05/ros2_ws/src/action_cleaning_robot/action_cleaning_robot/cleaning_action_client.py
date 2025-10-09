#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# My action format
from action_cleaning_robot.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__("cleaning_action_client")
        
        self._action_client = ActionClient(
            self,
            CleaningTask,
            "CleaningTask"
        )
        
        self.declare_parameter("x", 7.5)
        self.declare_parameter("y", 7.5)
        self.declare_parameter("type", "square")
        self.declare_parameter("area", 16.0)
        
        target_x = self.get_parameter("x").get_parameter_value().double_value
        target_y = self.get_parameter("y").get_parameter_value().double_value
        task_type = self.get_parameter("type").get_parameter_value().string_value
        area = self.get_parameter("area").get_parameter_value().double_value

        self.send_goal(task_type, area, target_x, target_y)
    
    def send_goal(self, task_type, area_size, target_x, target_y):
        self.get_logger().info("Waiting for server...")
        self._action_client.wait_for_server()

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        
        self.get_logger().info("Sending goal.")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("ERROR: Goal rejected by server.")
            return
        
        self.get_logger().info("Goal accepted by server, executing...")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"progress={feedback.progress_percent}% | \
                                 current cleaned points={feedback.current_cleaned_points} | \
                                 current position: x={feedback.current_x:.6f} y={feedback.current_y:.6f}")
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal results: succes={result.success} | \
                                 total cleaned={result.cleaned_points} | \
                                 total distance={result.total_distance:.6f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CleaningActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
