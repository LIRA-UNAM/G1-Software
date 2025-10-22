from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_bringup',
            executable='g1_initializer',
            name='g1_initializer',
            output='screen',
            parameters=[{'priority': 5}],
        )
    ])