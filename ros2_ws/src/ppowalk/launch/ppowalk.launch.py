"""Run the G1 low-level bridge and the ppowalk (velocity) policy node.

Example:
  ros2 launch ppowalk ppowalk.launch.py policy_path:=/abs/path/g1_ppowalk.onnx
  ros2 launch surge_et_ambula ppowalk.launch.py ...   # + web GUI on :8084
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
    share = get_package_share_directory('ppowalk')
    getup_share = get_package_share_directory('getup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_path', default_value='',
            description='Exported ONNX velocity policy (empty: Home works, Start is refused)'),
        DeclareLaunchArgument(
            'policy_params',
            default_value=os.path.join(share, 'config', 'g1_ppowalk_policy.yaml')),
        DeclareLaunchArgument(
            'enable_lowcmd', default_value='false',
            description='Send LowCmd to the robot (robot must be in debug mode)'),
        DeclareLaunchArgument('use_bridge', default_value='true'),

        Node(
            package='getup',
            executable='g1_lowlevel_bridge',
            name='g1_lowlevel_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_bridge')),
            parameters=[
                os.path.join(getup_share, 'config', 'g1_lowlevel_bridge.yaml'),
                os.path.join(share, 'config', 'g1_bridge_overrides.yaml'),
                {'enable_lowcmd': ParameterValue(
                    LaunchConfiguration('enable_lowcmd'), value_type=bool)},
            ],
        ),
        Node(
            package='ppowalk',
            executable='ppowalk_policy_node',
            name='ppowalk_policy_node',
            output='screen',
            parameters=[
                LaunchConfiguration('policy_params'),
                {'policy_path': LaunchConfiguration('policy_path')},
            ],
        ),
    ])
