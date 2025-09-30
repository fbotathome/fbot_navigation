import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'rviz/navigation.rviz')
    param_file = os.path.join(config_dir, 'nav2_params.yaml')

    use_description_arg = DeclareLaunchArgument(
        'use_description_slam',
        default_value='true',
        description='Parameter to use the robot description or not'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz_slam',
        default_value='true',
        description='Run rviz2'
    )

    use_description = LaunchConfiguration("use_description_slam")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'base.launch.py')
        ),
        launch_arguments={
            'use_description': use_description
        }.items()
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': param_file
        }.items()
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            condition=IfCondition(LaunchConfiguration('use_rviz_slam')),
            arguments=['-d', rviz_config_dir],
            output='screen')

    slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                os.path.join(config_dir, 'slam_toolbox.yaml')
            }],
        )
    
    return LaunchDescription([
        use_description_arg,
        use_rviz_arg,
        rviz_node,
        slam_node,
        robot_launch,
        nav_launch,    
    ])
