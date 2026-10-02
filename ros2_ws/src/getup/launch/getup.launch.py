"""Run the G1 low-level bridge and the getup policy node.

Example:
  ros2 launch getup getup.launch.py policy_path:=/abs/path/g1_getup.onnx
  ros2 service call /getup_policy_node/start std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('getup')

    policy_path = LaunchConfiguration('policy_path')
    policy_params = LaunchConfiguration('policy_params')
    bridge_params = LaunchConfiguration('bridge_params')
    enable_lowcmd = LaunchConfiguration('enable_lowcmd')
    use_bridge = LaunchConfiguration('use_bridge')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_path',
            default_value=os.path.join(share, 'models', 'g1_getup.onnx'),
            description='Exported ONNX policy'),
        DeclareLaunchArgument(
            'policy_params',
            default_value=os.path.join(share, 'config', 'g1_getup_policy.yaml')),
        DeclareLaunchArgument(
            'bridge_params',
            default_value=os.path.join(share, 'config', 'g1_lowlevel_bridge.yaml')),
        DeclareLaunchArgument(
            'enable_lowcmd', default_value='false',
            description='Send LowCmd to the robot (robot must be in debug mode)'),
        DeclareLaunchArgument(
            'use_bridge', default_value='true',
            description='Start the unitree_hg bridge (false when /imu and /joint_states '
                        'come from elsewhere, e.g. a simulator)'),
        DeclareLaunchArgument('auto_start', default_value='false'),

        Node(
            package='getup',
            executable='g1_lowlevel_bridge',
            name='g1_lowlevel_bridge',
            output='screen',
            condition=IfCondition(use_bridge),
            parameters=[
                bridge_params,
                {'enable_lowcmd': ParameterValue(enable_lowcmd, value_type=bool)},
            ],
        ),
        Node(
            package='getup',
            executable='getup_policy_node',
            name='getup_policy_node',
            output='screen',
            parameters=[
                policy_params,
                {
                    'policy_path': policy_path,
                    'auto_start': ParameterValue(auto_start, value_type=bool),
                },
            ],
        ),
    ])
