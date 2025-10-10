#!/usr/bin/env python3
import time
import math

import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Messages
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# My actioin format
from action_cleaning_robot.action import CleaningTask


class Geometry:
    def normalize_angle(angle) -> float:
        return math.remainder(angle, math.pi * 2)
    
    def distance(p1, p2) -> float:
        return math.sqrt(
            (p1["x"] - p2["x"]) ** 2 +
            (p1["y"] - p2["y"]) ** 2
        )

    def angle(p1, p2) -> float:
        return math.atan2(
            p2["y"] - p1["y"],
            p2["x"] - p1["x"]
        )


class CleaningActionServer(Node):
    def __init__(self) -> None:
        super().__init__("cleaning_action_server")

        self.declare_parameter("cleaning_size", 1.0)
        self.declare_parameter("name", "turtle1")
        
        self.cleaning_size = self.get_parameter("cleaning_size").get_parameter_value().double_value
        self.r = math.sqrt(self.cleaning_size / math.pi)
        self.turtle_name = self.get_parameter("name").get_parameter_value().string_value
        
        self.velocity_publisher = self.create_publisher(
            Twist,
            f"/{self.turtle_name}/cmd_vel",
            10
        )
        self.pose_subscriber = self.create_subscription(
            Pose,
            f"/{self.turtle_name}/pose",
            self.update_pose,
            10
        )
        
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.05
        self.linear_k = 2.0
        self.angular_k = 2.0
        self.linear_lim = (-2.0, 2.0)
        self.angular_lim = (-2.0, 2.0)

        self.current_position = None
        self.goal_position = None

        self.timer = None

        self.get_logger().info("Waiting for a turtlesim...")
        while self.current_position is None and rclpy.ok():
            rclpy.spin_once(self)

        self._action_server = ActionServer(
            self,
            CleaningTask,
            "CleaningTask",
            self.execute_callback
        )

        self.result = CleaningTask.Result()
        self.goal_handle = None
        self.get_logger().info("Server ready to work.")

    def update_pose(self, data):
        self.current_position = {
            "x": data.x,
            "y": data.y,
            "theta": data.theta
        }

    def angular_error(self, p1=None, p2=None) -> float:
        if p1 is None:
            p1 = self.current_position
        if p2 is None:
            p2 = self.goal_position
        if p1 is None or p2 is None:
            return 0.0

        angle_to_goal = Geometry.angle(p1, p2)
        return Geometry.normalize_angle(angle_to_goal - p1["theta"])
    
    def calc_distance(self, p1=None, p2=None) -> float:
        if p1 is None:
            p1 = self.current_position
        if p2 is None:
            p2 = self.goal_position
        if p1 is None or p2 is None:
            return float("inf")

        return Geometry.distance(p1, p2)
    
    def linear(self, goal):
        def linear_with_goal(self):
            linear = 0.0
            angular = 0.0
            
            ang_err = self.angular_error(p2=goal)
            dist = self.calc_distance(p2=goal)

            if abs(dist) >= self.distance_tolerance:
                linear = dist * self.linear_k
            else:
                return None, None
                
            angular = ang_err * self.angular_k

            linear = max(min(linear, self.linear_lim[1]), self.linear_lim[0])
            angular = max(min(angular, self.angular_lim[1]), self.angular_lim[0])
            
            return linear, angular
        
        return linear_with_goal
    
    def rotation(self, theta):
        def rotation_with_theta(self):
            angular = 0.0
            
            ang_err = Geometry.normalize_angle(theta - self.current_position["theta"])
            
            if abs(ang_err) >= self.angle_tolerance:
                angular = ang_err * self.angular_k
            else:
                return None, None

            angular = max(min(angular, self.angular_lim[1]), self.angular_lim[0])
            
            return 0.0, angular
        
        return rotation_with_theta
    
    def p2p(self, p1, p2):
        return [
            self.rotation(Geometry.angle(p1, p2)),
            self.linear(p2)
        ]
    
    def generate_points(self):
        L = math.sqrt(self.area_size)
        D = 2 * math.sqrt(self.cleaning_size / math.pi)
        
        overlap = 0.15
        step = D * (1 - overlap)
        
        n_points = int(L / step) + 2

        x_points = np.linspace(-L / 2, L / 2, n_points)
        y_points = np.linspace(-L / 2, L / 2, n_points)
        
        points = np.array([[x, y] for x in x_points for y in y_points])
        points += np.array([self.goal_position["x"], self.goal_position["y"]])
        
        route = []
        for i in range(n_points):
            if i % 2 == 0:
                route.extend(points[i * n_points:(i + 1) * n_points])
            else:
                route.extend(points[(i + 1) * n_points - 1:i * n_points - 1:-1])
        
        compact_route = [route[0]]
        for i in range(1, len(route) - 1):
            if ((route[i][0] == route[i + 1][0]) and (route[i][0] == route[i - 1][0])) or \
               ((route[i][1] == route[i + 1][1]) and (route[i][1] == route[i - 1][1])):
               continue
            else:
                compact_route.append(route[i])
        compact_route.append(route[-1])

        dict_route = []
        for point in compact_route:
            dict_route.append({"x": point[0], "y": point[1]})

        return dict_route
    
    def execute_callback(self, goal_handle):
        self.goal_handle = goal_handle
        self.result.success = False
        self.result.cleaned_points = 0
        self.result.total_distance = 0.0
        
        if self.current_position is None:
            self.get_logger().error("ERROR: No current position")
            return self.result

        task_type = self.goal_handle.request.task_type
        self.area_size = self.goal_handle.request.area_size
        target_x = self.goal_handle.request.target_x
        target_y = self.goal_handle.request.target_y
        
        self.goal_position = {
            "x": target_x,
            "y": target_y,
            "theta": self.current_position["theta"]
        }

        self.R = math.sqrt(self.area_size / math.pi)

        self.get_logger().info(f"Received new goal: type=\"{task_type}\" | \
                                 area={self.area_size} | \
                                 target position: x={target_x:.6f} y={target_y:.6f}")

        self.current_cleaned_points = 0
        self.current_distance = 0.0
        
        if task_type == "circle":
            self.movement_step = 0
            self.home = self.current_position
            self.movement_pipeline = [self.rotation(Geometry.angle(self.goal_position, self.current_position)),
                                      self.linear(self.goal_position),
                                      self.rotation(self.goal_position["theta"]),
                                      # self.spiral(),
                                      self.rotation(self.home["theta"]),
                                      self.linear(self.home),
                                      self.rotation(self.home["theta"])]
        elif task_type == "square":
            self.movement_step = 0
            self.home = self.current_position
            route = self.generate_points()
            route = [self.current_position] + route + [self.home]
            self.movement_pipeline = []
            for i in range(len(route) - 1):
                self.movement_pipeline += self.p2p(route[i], route[i + 1])
            self.movement_pipeline.append(self.rotation(self.home["theta"]))
        else:
            self.get_logger().error("ERROR: Unknown command")
            return self.result
        
        # self.movement_timer()
        
        # ================================================================
        self.ticks = 0
        self.delay = 0.1
        while rclpy.ok() and (self.movement_step < len(self.movement_pipeline)):
            rclpy.spin_once(self, timeout_sec=0.1)
            self.ticks += 1
            if self.current_position is None:
                continue

            vel_msg = Twist()
            
            linear, angular = self.movement_pipeline[self.movement_step](self)
            
            if linear is None or angular is None:
                self.movement_step += 1
                if self.movement_step >= len(self.movement_pipeline):
                    self.current_cleaned_points += self.area_size
                continue
            
            d = linear * self.delay
            deltaS = math.pi * self.r ** 2 - 2 * self.r ** 2 * math.acos(d / (2 * self.r)) + d / (2 * math.sqrt(4 * self.r ** 2 - d ** 2))
            self.current_cleaned_points += deltaS
            self.current_distance += d

            if self.ticks % 20 == 0:
                progress = int((self.movement_step + 1) / len(self.movement_pipeline) * 100)
                feedback_msg = CleaningTask.Feedback()
                feedback_msg.progress_percent = progress
                feedback_msg.current_cleaned_points = int(self.current_cleaned_points)
                feedback_msg.current_x = self.current_position["x"]
                feedback_msg.current_y = self.current_position["y"]
                
                self.get_logger().info(f"Sending progress: progress={progress}% | \
                                         cleaned={self.current_cleaned_points} | \
                                         position: x={self.current_position["x"]:.6f} y={self.current_position["y"]:.6f}")

                self.goal_handle.publish_feedback(feedback_msg)

            vel_msg.linear.x = linear
            vel_msg.angular.z = angular
            self.velocity_publisher.publish(vel_msg)
            
            time.sleep(self.delay)

            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(vel_msg)
        # ================================================================


        self.result.success = True
        self.result.cleaned_points = int(self.current_cleaned_points)
        self.result.total_distance = self.current_distance

        self.get_logger().info(f"Goal completed successfully! Results: total_points={int(self.current_cleaned_points)} distance={self.current_distance}")
        
        return self.result
    
    def movement_timer(self):
        self.ticks = 0
        self.delay = 0.1
        self.timer = self.create_timer(self.delay, self.control_loop)
        
    def control_loop(self):
        self.ticks += 1
        if self.current_position is None:
            return

        vel_msg = Twist()
        
        linear, angular = self.movement_pipeline[self.movement_step](self)
        
        if linear is None or angular is None:
            self.movement_step += 1
            if self.movement_step >= len(self.movement_pipeline):
                self.current_cleaned_points += self.area_size
                self.destroy_timer(self.timer)
                self.timer = None
            return
        
        d = linear * self.delay
        deltaS = math.pi * self.r ** 2 - 2 * self.r ** 2 * math.acos(d / (2 * self.r)) + d / (2 * math.sqrt(4 * self.r ** 2 - d ** 2))
        self.current_cleaned_points += deltaS
        self.current_distance += d

        if self.ticks % 20 == 0:
            progress = int((self.movement_step + 1) / len(self.movement_pipeline) * 100)
            feedback_msg = CleaningTask.Feedback()
            feedback_msg.progress_percent = progress
            feedback_msg.current_cleaned_points = int(self.current_cleaned_points)
            feedback_msg.current_x = self.current_position["x"]
            feedback_msg.current_y = self.current_position["y"]
            
            self.get_logger().info(f"Sending progress: progress={progress}% | \
                                     cleaned={self.current_cleaned_points} | \
                                     position: x={self.current_position["x"]:.6f} y={self.current_position["y"]:.6f}")

            self.goal_handle.publish_feedback(feedback_msg)

        vel_msg.linear.x = linear
        vel_msg.angular.z = angular

        self.velocity_publisher.publish(vel_msg)


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
