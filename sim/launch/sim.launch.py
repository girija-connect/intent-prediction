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

    # URDF for RViz's RobotModel display + the robot's TF tree (base_footprint -> base_link -> sensors/wheels)
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf) as f:
        robot_desc = f.read().replace('${namespace}', '')

    def with_models(var):
        return os.pathsep.join([p for p in (models, tb3_models, os.environ.get(var, '')) if p])

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
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'use_sim_time': True}]),
    ])