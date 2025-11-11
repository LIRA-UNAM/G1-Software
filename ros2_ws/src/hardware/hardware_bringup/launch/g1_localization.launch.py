from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('hardware_bringup')
    urdf_path = os.path.join(pkg,'urdf','g1_23dof.urdf')
    map_path = os.path.join(pkg, 'maps', 'map_1761851145.yaml')
    amcl_path    = os.path.join(pkg, 'config','amcl_params.yaml')

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

        # PointCloud2 to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='mid360_to_scan',
            remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan_mid360')],
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
        ),

        # Map server
        
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path}]
        ),

        #AMCL

        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_path,
                        {'scan_topic': '/scan_mid360',
                         'base_frame_id': 'base_link',
                         'odom_frame_id': 'odom',
                         'global_frame_id': 'map'}]
        ),

        #Lifecycle Manager

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{
                 'use_sim_time': False,
                 'autostart': True,
                 'bond_timeout': 10.0,
                 'node_names': ['map_server', 'amcl']
             }]),
  
    ])
