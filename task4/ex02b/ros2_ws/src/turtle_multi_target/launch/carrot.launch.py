import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
    	DeclareLaunchArgument(
            'target_frame', default_value='carrot1',
            description='Target frame name'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            					get_package_share_directory('two_turtles_one_carrot'), 
            					'launch'),
                                       		'/turtles.launch.py']),
        	launch_arguments={'target_frame': LaunchConfiguration('target_frame')}.items(),
    	),
        
        DeclareLaunchArgument(
            'radius', default_value="5.0",
            description='Radius of carrot.'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation', default_value='-1',
            description='Direction of rotation of the carrot.'
        ),
        DeclareLaunchArgument(
            'switch_threshold', default_value='1.0',
            description='Distance threshold for switching targets.'
        ),
        Node(
            package='two_turtles_one_carrot',
            executable='carrot',
            name='dynamic_broadcaster',
            parameters=[
                {
                    'radius': LaunchConfiguration('radius'),
                },
                {
                    "direction_of_rotation": LaunchConfiguration('direction_of_rotation')
                }
            ]
        ),
    ])
