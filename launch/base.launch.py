import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("boris_description"), 'launch', 'boris_description.launch.py')
        
        ),
        launch_arguments={
            'use_rviz': 'false',
            # 'arm_z_position':'0.23'
        }.items()
    )
    
    urg_node_ground = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensors_description"), 'launch', 'urg_node_ground.launch.py')
        
        ),
        launch_arguments={
            'sensor_interface': 'ground',
            'use_rviz': 'false',
        }.items()
    )

    urg_node_back = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensors_description"), 'launch', 'urg_node_back.launch.py')
        
        ),
        launch_arguments={
            'sensor_interface': 'back',
            'use_rviz': 'false',
        }.items()
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensors_description"), 'launch', 'bno055.launch.py')
        
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'robot_localization.launch.py')
        )
    )

    # controllers_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'start_controllers.launch.py')
        
    #     ),
    #     launch_arguments={
    #         'use_rviz': 'false',
    #     }.items()
    # )


    return LaunchDescription([

        robot_launch,
        urg_node_ground,
        urg_node_back,
        imu,
        robot_localization,
        # controllers_launch,
            
    ])