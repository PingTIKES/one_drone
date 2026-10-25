"""Check that the laptop entry point stays separate from the robot runtime."""
import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def test_remote_viewer_only_starts_rviz_and_local_tf_filter():
    bringup = Path(get_package_share_directory('bringup'))
    spec = importlib.util.spec_from_file_location('remote_viewer', bringup / 'launch/rviz.launch.py')
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    nodes = m.generate_launch_description().entities
    assert len(nodes) == 2 and all(isinstance(n, Node) for n in nodes)
    assert {n.node_executable for n in nodes} == {'rviz2', 'tf_display_filter.py'}
    values = yaml.safe_load((bringup / 'params/rviz_remote.yaml').read_text())['remote_rviz']
    config = yaml.safe_load((bringup / 'rviz' / values['config']).read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert manager['Global Options']['Background Color'] == '255; 255; 255'
    displays = {d['Name']: d for d in manager['Displays']}
    for name in ('PriorMap', 'LocalizationOdom', 'TF', 'EGOOptimalPath', 'EGOGlobalPath', 'EGOGoal'):
        assert displays[name]['Enabled'] is True
    for name in ('EGOInflatedOccupancy', 'DepthObstacles', 'Infrared'):
        assert displays[name]['Enabled'] is False
    assert manager['Tools'] == [{'Class': 'rviz_default_plugins/MoveCamera'}]
    assert all(panel['Class'] != 'tf_shift_panel/Rviz2Panel' for panel in config['Panels'])
