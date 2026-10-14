"""Launch the low-level bridge, the motion recorder and its web GUI.

Example:
  ros2 launch surge_et_ambula motion.launch.py
  # then open http://<robot-ip>:8083 to record / replay joint motions

The getup policy is NOT started: it publishes on the same command topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lowcmd', default_value='false',
            description='Send LowCmd to the robot (robot must be in debug mode); '
                        'can also be enabled later from the GUI'),

        Node(
            package='getup',
            executable='g1_lowlevel_bridge',
            name='g1_lowlevel_bridge',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('getup'), 'config', 'g1_lowlevel_bridge.yaml']),
                {'enable_lowcmd': ParameterValue(
                    LaunchConfiguration('enable_lowcmd'), value_type=bool)},
            ],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('motion_recorder'), 'launch', 'motion_recorder.launch.py']))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('motion_recorder_gui'), 'launch',
             'motion_recorder_gui.launch.py']))),
    ])
