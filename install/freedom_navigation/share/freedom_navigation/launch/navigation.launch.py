import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('freedom_navigation'), 'rviz/mapping.rviz')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('freedom_navigation'), 'launch', 'base.launch.py')
        ),
    )

    map_file = PathJoinSubstitution(
        [FindPackageShare("freedom_navigation"), "maps", "my_map.yaml"]
    )

    return LaunchDescription([
        robot_launch,

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{'yaml_filename': map_file}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])