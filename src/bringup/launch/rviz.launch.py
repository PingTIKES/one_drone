"""Laptop-only RViz viewer for the robot ROS graph; no algorithm/control nodes."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup = Path(get_package_share_directory('bringup'))
    values = yaml.safe_load((bringup / 'params/rviz_remote.yaml').read_text())['remote_rviz']
    config = Path(values['config']).expanduser()
    if not config.is_absolute():
        config = bringup / 'rviz' / config
    if not config.is_file():
        raise FileNotFoundError(f'RViz configuration does not exist: {config}')
    common = {'use_sim_time': bool(values['use_sim_time'])}
    # Local filtered TF uses separate outputs, even if the robot runs its own viewer.
    rviz_tf_filter = Node(
        package='rviz_tf_shift', executable='tf_display_filter.py',
        name='remote_rviz_tf_filter', output='screen',
        parameters=[common, {'frames': ['map', 'odom', 'base_link']}],
        remappings=[('/rviz/tf', '/remote_rviz/tf'),
                    ('/rviz/tf_static', '/remote_rviz/tf_static')])
    remote_rviz = Node(
        package='rviz2', executable='rviz2', name='remote_navigation_rviz',
        output='screen', arguments=['-d', str(config)],
        parameters=[common],
        remappings=[('/tf', '/remote_rviz/tf'),
                    ('/tf_static', '/remote_rviz/tf_static')])
    return LaunchDescription([
        rviz_tf_filter,
        remote_rviz,
    ])
