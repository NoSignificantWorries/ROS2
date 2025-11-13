from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_movement',
            executable='my_movement',
            name='my_movement',
            output='screen'
        ),
    ])
