from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='two_turtles_one_carrot',
            executable='broadcaster',
            name='broadcaster_1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='carrot1',
            description='Target frame name'
        ),
        Node(
            package='two_turtles_one_carrot',
            executable='broadcaster',
            name='broadcaster_2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='two_turtles_one_carrot',
            executable='listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])
