import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sim')
    models = os.path.join(get_package_share_directory('description'), 'models')
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')

    def with_models(var):
        return os.pathsep.join([p for p in (models, tb3_models, os.environ.get(var, '')) if p])

    def static_tf(x, y, z, parent, child):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                               '--frame-id', parent, '--child-frame-id', child])

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/cmd_vel_safe@geometry_msgs/msg/Twist]ignition.msgs.Twist',   # from safety_zones
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('gz_extra', default_value='',
                              description='extra args for ign gazebo, e.g. "--render-engine ogre"'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', with_models('IGN_GAZEBO_RESOURCE_PATH')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', with_models('GZ_SIM_RESOURCE_PATH')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -v 3 ', LaunchConfiguration('gz_extra'), ' ',
                                          os.path.join(pkg, 'worlds', 'office.sdf')]}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=bridge_args, output='screen'),
        # robot TF tree: odom -> base_footprint comes from Gazebo's diff-drive plugin
        static_tf(0.0, 0.0, 0.010, 'base_footprint', 'base_link'),
        static_tf(-0.064, 0.0, 0.111, 'base_link', 'base_scan'),
        static_tf(0.064, -0.065, 0.094, 'base_link', 'camera_link'),
    ])
