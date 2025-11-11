from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
import os

def generate_launch_description():
    pkg = get_package_share_directory('hardware_bringup')
    livox_share = get_package_share_directory('livox_ros_driver2')

    urdf_path   = os.path.join(pkg, 'urdf', 'g1_23dof.urdf')

    laser_topic     = LaunchConfiguration('laser_scan_topic')
    cmd_vel_topic   = LaunchConfiguration('cmd_vel_topic')
    base_link       = LaunchConfiguration('base_link_name')
    odom_frame      = LaunchConfiguration('odom_frame')

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, 'launch_ROS2', 'rviz_MID360_launch.py')
        )
    )

    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_to_scan',
        remappings=[('cloud_in', '/livox/lidar'), ('scan', laser_topic)],
        parameters=[{
            'target_frame': 'mid360_link',
            'min_height':  0.05,
            'max_height':  0.50,
            'angle_min':  -3.14159,
            'angle_max':   3.14159,
            'angle_increment': 0.0043633,
            'range_min': 0.3,
            'range_max': 20.0,
            'scan_time': 0.05,
            'use_inf': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('laser_scan_topic',  default_value='/scan_mid360'),
        DeclareLaunchArgument('cmd_vel_topic',     default_value='/cmd_vel'),
        DeclareLaunchArgument('base_link_name',    default_value='base_link'),
        DeclareLaunchArgument('odom_frame',        default_value='odom'),

        # Odom to base_link
        Node(
            package='twist_to_g1',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
        ),

        livox_launch,

        # Robot State
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        TimerAction(period=2.0, actions=[pcl_to_scan]),

    ])
