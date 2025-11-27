from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_stop',
            executable='obstacle_stop_node',
            output='screen'
        ),
    ])
