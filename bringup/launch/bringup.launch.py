"""
One-command version of the 3 manual steps.
  ros2 launch bringup bringup.launch.py                                   # everything
  ros2 launch bringup bringup.launch.py rviz:=false                       # sim + perception/intent/safety

Manual equivalent (3 terminals):
  ros2 launch sim sim.launch.py
  ros2 launch navigation navigation.launch.py
  rviz2 --ros-args -p use_sim_time:=true                                  # then File > Open Config > nav.rviz
"""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(pkg, name, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share(pkg), 'launch', name)),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gz_extra', default_value='',
                              description='extra args for ign gazebo, e.g. "--render-engine ogre"'),
        DeclareLaunchArgument('nav', default_value='true', description='start localization + Nav2'),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),
        DeclareLaunchArgument('nav_delay', default_value='5.0', description='seconds to wait for Gazebo before navigation'),
        DeclareLaunchArgument('map', default_value=os.path.join(share('mapping'), 'maps', 'office.yaml')),

        include('sim', 'sim.launch.py', {'gz_extra': LaunchConfiguration('gz_extra')}),
        TimerAction(period=LaunchConfiguration('nav_delay'), actions=[
            include('navigation', 'navigation.launch.py',
                    {'map': LaunchConfiguration('map'), 'nav2': LaunchConfiguration('nav')}),
        ]),
        include('bringup', 'rviz.launch.py', condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
