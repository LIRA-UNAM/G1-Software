from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('twist_to_g1')
    params = os.path.join(pkg, 'config', 'bridge_params.yaml')
    return LaunchDescription([
        Node(
            package='twist_to_g1',
            executable='twist_to_g1',
            name='twist_to_g1_bridge',
            output='screen',
            parameters=[params],
        )
    ])
