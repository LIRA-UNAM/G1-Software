from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gui_share = get_package_share_directory("getup_gui")

    getup_gui_node = Node(
        package="getup_gui",
        executable="getup_gui",
        name="getup_gui_bridge",
        output="screen",
        parameters=[gui_share + "/config/gui.yaml"],
    )

    return LaunchDescription([
        getup_gui_node,
    ])
