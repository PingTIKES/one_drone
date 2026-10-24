"""Verify estimator exclusivity and camera-free startup without touching hardware."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


@pytest.fixture
def startup():
    path = Path(get_package_share_directory('bringup')) / 'launch/startup.launch.py'
    spec = importlib.util.spec_from_file_location('startup_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_nodes(startup, sim, source, perception):
    values = yaml.safe_load((Path(get_package_share_directory('bringup')) /
                            'params/launch.yaml').read_text())
    params = {**values['startup'], **values['simulation' if sim else 'hardware']}
    params.update(sim=sim, rviz=False, flight_control=False,
                  localization_source=source, perception_enabled=perception)
    if not perception and source == 'microvio':
        # A camera-free check must not read or require any camera calibration.
        params.update(calibration_dir='/does/not/exist', cam0_topic='', cam1_topic='',
                      imu_topic='', depth_topic='', depth_info_topic='')
    context = LaunchContext()
    context.launch_configurations.update({k: startup.launch_value(v) for k, v in params.items()})
    nodes = {}
    for action in startup.setup(context):
        if isinstance(action, Node) and (action.condition is None or action.condition.evaluate(context)):
            executable = perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable))
            nodes[executable] = action
    return context, nodes


def test_microvio_without_camera_has_no_estimator_feedback_or_planner(startup):
    context, nodes = launch_nodes(startup, False, 'microvio', False)
    assert 'microvio' in nodes
    assert 'ego_odom_adapter' in nodes
    assert 'modify_map_to_odom_node' in nodes
    for executable in ('run_subscribe_msckf', 'vio_to_px4.py', 'sensor_relay',
                       'software_stereo', 'depth_filter', 'stereo_depth_node',
                       'ego_planner_node', 'traj_server', 'goal_manager',
                       'decision_manager', 'flight_bridge'):
        assert executable not in nodes
    # Validate the actual ROS interface overrides passed to the selected node.
    params = nodes['microvio']._Node__parameters
    merged = {}
    for group in evaluate_parameters(context, params):
        merged.update(group)
    assert merged['odom_topic'] == '/odom'
    assert merged['health_topic'] == '/vio_health'
    assert merged['diagnostics_topic'] == '/vio_diagnostics'
    assert merged['reset_topic'] == '/vio_reset_event'
    assert merged['odom_frame'] == 'odom'
    assert merged['body_frame'] == 'base_link'


@pytest.mark.parametrize('sim,source', [(True, 'openvins'), (False, 'openvins'),
                                      (False, 'microvio')])
def test_camera_navigation_keeps_planning_and_selects_one_estimator(startup, sim, source):
    _, nodes = launch_nodes(startup, sim, source, True)
    assert 'ego_planner_node' in nodes
    assert 'depth_filter' in nodes
    assert ('run_subscribe_msckf' in nodes) == (source == 'openvins')
    assert ('vio_to_px4.py' in nodes) == (source == 'openvins')
    assert ('microvio' in nodes) == (source == 'microvio')
