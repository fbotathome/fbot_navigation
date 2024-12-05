import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    urg_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensors_description"), 'launch', 'urg_node.launch.py')
        
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    # controllers_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'start_controllers.launch.py')
        
    #     ),
    #     launch_arguments={
    #         'use_rviz': 'false',
    #     }.items()
    # )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("boris_description"), 'launch', 'boris_description.launch.py')
        
        ),
        launch_arguments={
            'gui': 'false',
            # 'arm_z_position':'0.23'
        }.items()
    )

    return LaunchDescription([

        robot_launch,
        urg_node_launch,
        # controllers_launch,
            
    ])