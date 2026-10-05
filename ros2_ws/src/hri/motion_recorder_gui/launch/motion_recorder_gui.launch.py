from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("motion_recorder_gui")
    return LaunchDescription([
        Node(
            package="motion_recorder_gui",
            executable="motion_recorder_gui",
            name="motion_recorder_gui_bridge",
            output="screen",
            parameters=[share + "/config/gui.yaml"],
        ),
    ])
