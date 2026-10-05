"""Launch the getup policy (with the low-level bridge) and the web GUI together.

Example:
  ros2 launch surge_et_ambula getup.launch.py policy_path:=/abs/path/g1_getup.onnx
  # then open http://<robot-ip>:8082 for the e-stop and live parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    getup_launch = PathJoinSubstitution(
        [FindPackageShare('getup'), 'launch', 'getup.launch.py'])
    gui_launch = PathJoinSubstitution(
        [FindPackageShare('getup_gui'), 'launch', 'getup_gui.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_path', default_value=PathJoinSubstitution(
                [FindPackageShare('getup'), 'models', 'g1_getup.onnx']),
            description='Exported ONNX getup policy'),
        DeclareLaunchArgument(
            'enable_lowcmd', default_value='false',
            description='Send LowCmd to the robot (robot must be in debug mode); '
                        'can also be enabled later from the GUI'),
        DeclareLaunchArgument('auto_start', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(getup_launch),
            launch_arguments={
                'policy_path': LaunchConfiguration('policy_path'),
                'enable_lowcmd': LaunchConfiguration('enable_lowcmd'),
                'auto_start': LaunchConfiguration('auto_start'),
                # The GUI is included below; do not start a second one.
                'use_gui': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gui_launch)),
    ])
