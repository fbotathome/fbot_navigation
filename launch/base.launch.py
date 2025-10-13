import os

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_description',
            default_value='true',
            description='Parameter to use the robot description, if you will use the fbot_navigation standalone' \
            'launch file set it to true. Otherwise, set it to false to use fbot_bringup' \
        )
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("boris_description"), 'launch', 'boris_description.launch.py')
        
        ),
        launch_arguments={
            'arm_z_position':'0.23',
            'use_rviz': 'false'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_description')
        )
    )

    sick_lms_1xx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sensors_description"), 'launch', 'sick_lms_1xx.launch.py')    
        ),
        launch_arguments={
            'use_rviz': 'false',
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

    return LaunchDescription([
        *declared_arguments,
        #sick_lms_1xx,
        description,
        urg_node_ground,
        urg_node_back,
        imu,
        robot_localization,
            
    ])
