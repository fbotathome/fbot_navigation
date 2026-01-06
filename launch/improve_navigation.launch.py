import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'rviz/navigation.rviz')

    # Declare launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether to run SLAM (true) or normal navigation (false)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='CBR_2025_HOME.yaml',
        description='Map file name (only used when use_slam is false)'
    )

    nav_params_file_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value='nav2_params.yaml',
        description='Nav2 parameters file name'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value='slam_toolbox.yaml',
        description='SLAM Toolbox parameters file name'
    )

    use_description_arg = DeclareLaunchArgument(
        'use_description',
        default_value='true',
        description='Parameter to use the robot description or not'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Run rviz2'
    )

    # Get launch configurations
    use_slam = LaunchConfiguration('use_slam')
    map_file_name = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_description = LaunchConfiguration('use_description')
    use_rviz = LaunchConfiguration('use_rviz')

    # Construct full paths
    map_file = PathJoinSubstitution(
        [FindPackageShare("fbot_navigation"), "maps", map_file_name]
    )

    nav_param_file_path = PathJoinSubstitution(
        [FindPackageShare("fbot_navigation"), "param", nav_params_file]
    )

    slam_param_file_path = PathJoinSubstitution(
        [FindPackageShare("fbot_navigation"), "param", slam_params_file]
    )

    # Robot base launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'base.launch.py')
        ),
        launch_arguments={
            'use_description': use_description
        }.items()
    )

    # Normal navigation launch (with map)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'map': map_file,
            'params_file': nav_param_file_path
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    # SLAM navigation launch (without map)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav_param_file_path
        }.items(),
        condition=IfCondition(use_slam)
    )

    # SLAM Toolbox node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_param_file_path],
        condition=IfCondition(use_slam)
    )

    # Rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        use_slam_arg,
        map_file_arg,
        nav_params_file_arg,
        slam_params_file_arg,
        use_description_arg,
        use_rviz_arg,
        robot_launch,
        nav2_bringup_launch,       # Runs when use_slam=false
        nav2_navigation_launch,    # Runs when use_slam=true
        slam_node,                 # Runs when use_slam=true
        rviz_node,
    ])
