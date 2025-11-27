from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_stop_depth',
            executable='obstacle_stop_depth_node',
            output='screen'
        ),
    ])
