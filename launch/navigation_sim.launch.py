import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('fbot_navigation')

    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='arena_gazebo.yaml',
        description='Map file inside fbot_navigation/maps')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value='nav2_params_sim.yaml',
        description='Nav2 params file inside fbot_navigation/param')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Run rviz2')

    map_file = PathJoinSubstitution(
        [FindPackageShare('fbot_navigation'), 'maps', LaunchConfiguration('map_file')])
    params_file = PathJoinSubstitution(
        [FindPackageShare('fbot_navigation'), 'param', LaunchConfiguration('params_file')])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'param', 'ekf_sim.yaml'), {'use_sim_time': True}],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'map': map_file,
            'params_file': params_file,
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        use_rviz_arg,
        ekf_node,
        nav2_bringup,
        rviz_node,
    ])
