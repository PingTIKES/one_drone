"""Read-only MicoVIO/PX4 telemetry check; no flight-control nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('microvio')) / 'config/params.yaml'
    return LaunchDescription([
        Node(package='microvio', executable='microvio', name='microvio',
             output='screen', parameters=[str(config)]),
    ])
