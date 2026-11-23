from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Predicts human intent and feeds Nav2 (/human_predicted_grid + /speed_limit).
        # It does NOT publish /cmd_vel: Nav2 drives to the goal.
        Node(package='prediction', executable='intent_nav_bridge', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
