import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'param')
    rviz_config_dir = os.path.join(get_package_share_directory('fbot_navigation'), 'rviz/navigation.rviz')
    param_file = os.path.join(config_dir, 'nav2_params.yaml')

    #declared_arguments = []
    
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
    
    use_description = LaunchConfiguration("use_description")

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fbot_navigation'), 'launch', 'base.launch.py')
        ),
        launch_arguments={
            'use_description': use_description
        }.items()
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

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', rviz_config_dir],
            output='screen')


    return LaunchDescription([
        use_description_arg,
        use_rviz_arg,
        rviz_node,
        robot_launch,
        nav2_bringup_launch,
        
    ])