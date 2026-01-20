import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

# # Padrão
#     navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
#         ),
#         launch_arguments={
#             'use_description':'true'
#         }.items()
#     )

# Padrão com mapa diferente
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
        ),
        launch_arguments={
            'map_file': 'lab_map_v2.yaml'
        }.items()
    )

# # Padrão com yaml diferente
#     navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
#         ),
#         launch_arguments={
#             'map_file_arg': 'lab_map_v2.yaml',
#             'nav_params_file_arg': 'nav2_params_keepout.yaml'
#         }.items()
#     )

# # Slam
#     navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
#         ),
#         launch_arguments={
#             'use_slam':'true'
#         }.items()
#     )

# # Slam com improve_slam_toolbox
#     navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
#         ),
#         launch_arguments={
#             'use_slam':'true',
#             'slam_params_file':'improve_slam_toolbox.yaml'
#         }.items()
#     )

# # Slam com câmera
#     navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory("fbot_navigation"), 'launch', 'improve_navigation.launch.py')
        
#         ),
#         launch_arguments={
#             'use_slam':'true',
#             'nav_params_file_arg': 'slam_camera_params.yaml',
#             'slam_params_file':'improve_slam_toolbox.yaml'
#         }.items()
#     )

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_bringup"), 'launch', 'world.launch.py')
        
        ),
        launch_arguments={
            'config_file_name':'pose_inspection'
        }.items()
    )

    neck = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fbot_bringup"), 'launch', 'neck.launch.py')
        ),
    )

    return LaunchDescription([
        neck,
        navigation,
        world,
    ])