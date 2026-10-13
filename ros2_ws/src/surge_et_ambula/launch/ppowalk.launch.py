"""Launch the ppowalk velocity policy (with the low-level bridge) and its web GUI.

Example:
  ros2 launch surge_et_ambula ppowalk.launch.py policy_path:=/abs/path/g1_ppowalk.onnx
  # then open http://<robot-ip>:8084
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('policy_path', default_value='',
                              description='Exported ONNX velocity policy'),
        DeclareLaunchArgument('enable_lowcmd', default_value='false',
                              description='Send LowCmd (debug mode); can be enabled from the GUI'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ppowalk'), 'launch', 'ppowalk.launch.py'])),
            launch_arguments={
                'policy_path': LaunchConfiguration('policy_path'),
                'enable_lowcmd': LaunchConfiguration('enable_lowcmd'),
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ppowalk_gui'), 'launch', 'ppowalk_gui.launch.py']))),
    ])
