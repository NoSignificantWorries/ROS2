#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys


class Geometry:
    def normalize_angle(angle) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
    
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


class TurtleController:
    def __init__(self,
                 linear_k=2.0,
                 angle_k = 1.5,
                 linear_vel_limits=(-2.0, 2.0),
                 angle_vel_limits=(-2.0, 2.0),
                 distance_tolerance=0.1,
                 angle_tolerance=0.05,
                 width_limits=(0.0, 11.0),
                 height_limits=(0.0, 11.0),
                 current_position=None,
                 goal_position=None) -> None:
        
        self.linear_k = linear_k
        self.angle_k = angle_k
        self.linear_lim = linear_vel_limits
        self.angle_lim = angle_vel_limits
        self.dist_tolerance = distance_tolerance
        self.ang_tolerance = angle_tolerance
        
        self.wlim = width_limits
        self.hlim = height_limits

        self.current_position = current_position

        self.goal_position = goal_position
        if not self._test_position(self.goal_position):
            raise ValueError("ERROR: Wrong goal position")
        
        self.start_position = None
        self.transition_teta = None

    def _test_position(self, position) -> bool:
        return (self.wlim[0] <= position["x"] <= self.wlim[1]) and \
               (self.hlim[0] <= position["y"] <= self.hlim[1]) or position is None
    
    def set_current_position(self, position) -> None:
        self.current_position = position
    
    def set_goal_position(self, position) -> bool:
        if not self._test_position(position):
            return False
        self.goal_position = position
        return True
    
    def angular_error(self) -> float:
        if self.current_position is None:
            return 0.0
        angle_to_goal = Geometry.angle(self.current_position, self.goal_position)
        return Geometry.normalize_angle(angle_to_goal - self.current_position["theta"])
    
    def calc_distance(self) -> float:
        if self.current_position is None:
            return float("inf")
        return Geometry.distance(self.current_position, self.goal_position)
    
    def start(self) -> None:
        self.start_position = self.current_position
    
    def stop(self) -> None:
        self.start_position = None
    
    def movement_step(self, delay=0.1):
        if self.start_position is None:
            self.start()

        linear = 0.0
        angular = 0.0

        dist_err = self.calc_distance()
        ang_err = self.angular_error()
        
        if dist_err >= self.dist_tolerance:
            linear = dist_err * self.linear_k
        
        if ang_err >= self.ang_tolerance:
            angular = ang_err * self.angle_k

        linear = max(min(linear, self.linear_lim[1]), self.linear_lim[0])
        angular = max(min(angular, self.angle_lim[1]), self.angle_lim[0])
        
        return linear, angular


class MoveToGoal(Node):
    def __init__(self) -> None:
        super().__init__("move_to_goal")
        
        self.declare_parameter("x", 5.5)
        self.declare_parameter("y", 5.5)
        self.declare_parameter("theta", 0.0)
        self.declare_parameter("name", "turtle1")
        
        goal_x = self.get_parameter("x").get_parameter_value().double_value
        goal_y = self.get_parameter("y").get_parameter_value().double_value
        goal_theta = self.get_parameter("theta").get_parameter_value().double_value
        self.turtle_name = self.get_parameter("name").get_parameter_value().string_value

        goal_position = {
            "x": goal_x,
            "y": goal_y,
            "theta": goal_theta
        }
        
        try:
            self.controller = TurtleController(goal_position=goal_position)
        except ValueError:
            self.get_logger().error("ERROR: Goal out of bounds")
            sys.exit(1)
        
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
        
        while self.controller.current_position is None and rclpy.ok():
            rclpy.spin_once(self)
        
        self.get_logger().info(f"name: {self.turtle_name} | goal: x={goal_x:.4f} y={goal_y:.4f} theta={goal_theta:.4f}")
        self.position_reached = False
        self.move_to_goal()
    
    def update_pose(self, data) -> None:
        self.controller.set_current_position({
            "x": data.x,
            "y": data.y,
            "theta": data.theta
        })
    
    def move_to_goal(self) -> None:
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self) -> None:
        if self.controller.current_position is None:
            return
        
        vel_msg = Twist()
        
        linear, angular = self.controller.movement_step()
        
        if linear == 0 and angular == 0:
            self.get_logger().info(f"Finished")
            self.destroy_timer(self.timer)
            return
        
        vel_msg.linear.x = linear
        vel_msg.angular.z = angular
        
        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
