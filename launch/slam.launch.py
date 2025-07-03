import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'rviz/mapping.rviz')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'base.launch.py')
        ),
        launch_arguments={
            'arm_z_position': '0.23'
        }.items()
    )
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'))
    )
    
    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                os.path.join(config_dir, 'slam_toolbox.yaml')
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        nav_launch,

        robot_launch,
            
    ])