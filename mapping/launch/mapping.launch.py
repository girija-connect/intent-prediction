"""
SLAM mapping (slam_toolbox, async mode).

  ros2 launch sim sim.launch.py          # terminal 1
  ros2 launch mapping mapping.launch.py  # terminal 2   (use sim_mode:=false on the real robot)

  Drive the robot, then save:
  ros2 run nav2_map_server map_saver_cli -f src/mapping/maps/office
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')
    use_sim_time = PythonExpression(["'", sim_mode, "' == 'true'"])

    config_file = os.path.join(get_package_share_directory('mapping'), 'config', 'slam_toolbox_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('sim_mode', default_value='true',
                              description='Use simulation time if true'),
        Node(package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             parameters=[config_file, {'use_sim_time': use_sim_time}],
             output='screen'),
    ])