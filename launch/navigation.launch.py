"""Navigation only: Nav2 (AMCL + map) or SLAM, optionally with keepout zones.

It NEVER starts the robot description, ros2_control, sensors or the EKF. Start
those first (fbot_bringup/launch/boris.launch.py), or use
`ros2 launch fbot_bringup boris.launch.py use_navigation:=true`.

  ros2 launch fbot_navigation navigation.launch.py map_file:=lab_2026_2.yaml
  ros2 launch fbot_navigation navigation.launch.py use_slam:=true
  ros2 launch fbot_navigation navigation.launch.py use_keepout:=true params_file:=<...>/nav2_params_keepout.yaml

Required topics/TF: /scan* (lasers), /odom + odom->base_footprint (EKF), robot_description.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('fbot_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_slam = LaunchConfiguration('use_slam').perform(context).lower() == 'true'
    use_keepout = LaunchConfiguration('use_keepout').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    map_file = LaunchConfiguration('map_file').perform(context)
    if not os.path.isabs(map_file):
        map_file = os.path.join(share, 'maps', map_file)

    slam_params = LaunchConfiguration('slam_params_file').perform(context)
    if not os.path.isabs(slam_params):
        slam_params = os.path.join(share, 'param', slam_params)

    actions = []

    if use_slam:
        # navigate while mapping: no map server / AMCL
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'params_file': params_file,
            }.items(),
        ))
        actions.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ))
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'map': map_file,
                'params_file': params_file,
            }.items(),
        ))

    if use_keepout:
        # keepout zones: mask map server + filter info server + their lifecycle manager
        actions.append(Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters', output='screen', emulate_tty=True,
            parameters=[{'autostart': True,
                         'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
        ))
        actions.append(Node(
            package='nav2_map_server', executable='map_server', name='filter_mask_server',
            output='screen', emulate_tty=True, parameters=[params_file],
        ))
        actions.append(Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server', output='screen', emulate_tty=True,
            parameters=[params_file],
        ))

    return actions


def generate_launch_description():
    share = get_package_share_directory('fbot_navigation')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d', os.path.join(share, 'rviz', 'navigation.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_slam', default_value='false',
                              description='true: slam_toolbox + Nav2 without map; false: Nav2 with map + AMCL'),
        DeclareLaunchArgument('use_keepout', default_value='false',
                              description='Start the keepout-zone filter nodes (use with nav2_params_keepout.yaml)'),
        DeclareLaunchArgument('map_file', default_value='lab_2026_2.yaml',
                              description='Map yaml: file name inside maps/ or an absolute path (ignored with use_slam)'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'param', 'hockuyos_params.yaml'),
                              description='Nav2 parameters (absolute path)'),
        DeclareLaunchArgument('slam_params_file', default_value='improve_slam_toolbox.yaml',
                              description='slam_toolbox params: file name inside param/ or absolute path'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Start RViz2 with navigation.rviz'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        rviz,
        OpaqueFunction(function=_launch_setup),
    ])
