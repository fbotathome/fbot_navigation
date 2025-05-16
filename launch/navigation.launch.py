import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'rviz/navigation.rviz')
    param_file = os.path.join(config_dir, 'nav2_params.yaml')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'base.launch.py')
        ),
    )

    map_file = PathJoinSubstitution(
        [FindPackageShare("fbot_navigation"), "maps", "my_map.yaml"]
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': map_file,
            'params_file': param_file
        }.items()
    )

    keepout_zones_arg = DeclareLaunchArgument(
        'use_keepout_zones',
        default_value='true',
        description='Use keepout zones on the costmaps'
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen')

    #KEEPOUT ZONES 3 necessary nodes
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{'autostart': True},
                    {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
        condition=IfCondition(LaunchConfiguration('use_keepout_zones'))
    )

    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        condition=IfCondition(LaunchConfiguration('use_keepout_zones'))
    )

    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        condition=IfCondition(LaunchConfiguration('use_keepout_zones'))
    )

    return LaunchDescription([
        keepout_zones_arg,
        robot_launch,
        nav2_bringup_launch,
        lifecycle_node,
        filter_mask_server_node,
        costmap_filter_info_server_node,
        rviz_node
        
    ])