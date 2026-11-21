"""
Starts: map_server + AMCL, Nav2 (planner/controller/costmaps), safety zones, human perception and intent prediction.

  ros2 launch navigation navigation.launch.py 
"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, name, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(pkg), 'launch', name)),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    pkg = share('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2 = IfCondition(LaunchConfiguration('nav2'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'navigation_params.yaml')),
        DeclareLaunchArgument('map', default_value=os.path.join(share('mapping'), 'maps', 'office.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true', description='start localization + Nav2'),
        DeclareLaunchArgument('perception', default_value='true', description='start human_tracker (YOLO)'),

        # planner, controller, smoothers, BT navigator, behaviours, costmaps (Nav2's own navigation launch).
        # The local costmap's `human_layer` reads /human_predicted_grid from prediction.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': LaunchConfiguration('params_file'),
                              'use_sim_time': use_sim_time,
                              'use_composition': 'False',
                              'autostart': 'true'}.items(),
            condition=nav2),

        include('localization', 'localization.launch.py',
                {'map': LaunchConfiguration('map'), 'use_sim_time': use_sim_time,
                 'params_file': os.path.join(share('localization'), 'config', 'localization_params.yaml')}, nav2),

        # /cmd_vel -> /cmd_vel_safe (the Gazebo bridge listens on /cmd_vel_safe)
        include('safety', 'safety.launch.py', {'use_sim_time': use_sim_time}),
        # camera + lidar -> /human_tracks, then intent -> /human_predicted_grid + /speed_limit
        include('perception', 'perception.launch.py', {'use_sim_time': use_sim_time},
                IfCondition(LaunchConfiguration('perception'))),
        include('prediction', 'prediction.launch.py', {'use_sim_time': use_sim_time}),
    ])