import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('localization')
    maps = get_package_share_directory('mapping')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(maps, 'maps', 'office.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'localization_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # map_server + AMCL + lifecycle manager (Nav2's own localization launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'use_composition': 'False',
                              'autostart': 'true'}.items()),
    ])
