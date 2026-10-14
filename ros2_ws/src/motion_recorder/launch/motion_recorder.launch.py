from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("motion_recorder")
    return LaunchDescription([
        Node(
            package="motion_recorder",
            executable="motion_recorder_node",
            name="motion_recorder",
            output="screen",
            parameters=[share + "/config/motion_recorder.yaml"],
        ),
    ])
