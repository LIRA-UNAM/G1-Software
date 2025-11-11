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
    urdf_path = os.path.join(pkg,'urdf','g1_23dof.urdf')
    slam_params_path = os.path.join(pkg,'config','slam_toolbox_g1.yaml')
    teleop_yaml = os.path.join(pkg, 'config', 'teleop_joy_g1.yaml')
    laser_topic = LaunchConfiguration('laser_scan_topic')

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
        # Odom to base_link
        Node(
            package='twist_to_g1',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
            #parameters=[{'odom_frame':'odom','base_frame':'base_link'}]
        ),

        # URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev':'/dev/input/js0',
                'deadzone':0.05,
            }]
        ),

        #Joystick teleop

        # Node(
        #     package='teleop_twist_joy',
        #     executable='teleop_node',
        #     name='teleop_twist_joy',
        #     output='screen',
        #     parameters=[os.path.join(pkg, 'config', 'teleop_joy_g1.yaml')],
        # ),

        #SLAM toolbox

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_path]
        ),

        livox_launch,

        TimerAction(period=2.0, actions=[pcl_to_scan]),
  
    ])
