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


def launch_nodes(startup, sim, source, perception, flight_control=False, yaw_control=True,
                 mavros_ns='/mavros'):
    values = yaml.safe_load((Path(get_package_share_directory('bringup')) /
                            'params/launch.yaml').read_text())
    params = {**values['startup'], **values['simulation' if sim else 'hardware']}
    params.update(sim=sim, rviz=False, flight_control=flight_control, yaw_control=yaw_control,
                  localization_source=source, perception_enabled=perception, mavros_ns=mavros_ns)
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


def test_microvio_without_camera_keeps_planner_without_estimator_feedback(startup):
    context, nodes = launch_nodes(startup, False, 'microvio', False)
    assert 'microvio' in nodes
    assert 'ego_odom_adapter' in nodes
    assert 'modify_map_to_odom_node' in nodes
    for executable in ('run_subscribe_msckf', 'vio_to_px4.py', 'sensor_relay',
                       'software_stereo', 'depth_filter', 'stereo_depth_node',
                       'flight_bridge'):
        assert executable not in nodes
    for executable in ('ego_planner_node', 'traj_server', 'goal_manager', 'decision_manager'):
        assert executable in nodes
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


@pytest.mark.parametrize('enabled', [True, False])
def test_launch_passes_yaw_control_to_flight_bridge(startup, enabled):
    context, nodes = launch_nodes(startup, False, 'microvio', False,
                                  flight_control=True, yaw_control=enabled)
    merged = {}
    for group in evaluate_parameters(context, nodes['flight_bridge']._Node__parameters):
        merged.update(group)
    assert merged['yaw_control'] is enabled


def test_mavros_keeps_internal_plugin_names_and_is_single_fcu_owner(startup):
    context, nodes = launch_nodes(startup, False, 'microvio', False, flight_control=True)
    mavros = nodes['mavros_node']
    # A process-wide __node remap would rename every MAVROS plugin node,
    # breaking per-plugin YAML matching and private service resolution.
    assert mavros._Node__node_name is None
    groups = evaluate_parameters(context, mavros._Node__parameters)
    merged = {}
    for group in groups:
        if isinstance(group, dict):
            merged.update(group)
    assert merged['tgt_system'] == 1
    assert merged['fcu_url'].endswith(':921600')
    assert not any('Agent' in executable for executable in nodes)


@pytest.mark.parametrize('namespace,name,parent', [('/mavros', 'mavros', '/'),
                                                 ('/uav/control', 'control', '/uav')])
def test_mavros_scoped_uas_name_matches_configured_interface_root(startup, namespace, name, parent):
    _, nodes = launch_nodes(startup, False, 'microvio', False, mavros_ns=namespace)
    mavros = nodes['mavros_node']
    assert mavros._Node__node_name is None
    assert mavros._Node__ros_arguments == ['-r', f'mavros:__node:={name}',
                                          '-r', f'{name}:__ns:={parent}']


def test_yaml_alone_selects_simulation_or_hardware(startup, tmp_path, monkeypatch):
    """Resolve the real LaunchDescription without command-line launch arguments."""
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    original_share = startup.get_package_share_directory
    bringup = Path(original_share('bringup'))
    for name in ('map', 'rviz'):
        (tmp_path / name).symlink_to(bringup / name, target_is_directory=True)
    (tmp_path / 'params').mkdir()
    for config in (bringup / 'params').iterdir():
        if config.name != 'launch.yaml':
            (tmp_path / 'params' / config.name).symlink_to(config)
    monkeypatch.setattr(startup, 'get_package_share_directory',
                        lambda name: str(tmp_path) if name == 'bringup' else original_share(name))
    values = yaml.safe_load((bringup / 'params/launch.yaml').read_text())
    for sim in (True, False):
        values['startup'].update(sim=sim, rviz=False, flight_control=True)
        (tmp_path / 'params/launch.yaml').write_text(yaml.safe_dump(values))
        context = LaunchContext()
        nodes = {}

        def resolve(action):
            if isinstance(action, Node):
                if action.condition is None or action.condition.evaluate(context):
                    executable = perform_substitutions(
                        context, normalize_to_list_of_substitutions(action.node_executable))
                    nodes[executable] = action
            elif isinstance(action, (DeclareLaunchArgument, OpaqueFunction)):
                for child in action.execute(context) or []:
                    resolve(child)

        for action in startup.generate_launch_description().entities:
            resolve(action)
        assert ('run_subscribe_msckf' in nodes) is sim
        assert ('microvio' in nodes) is not sim
        for name in ('mavros_node', 'ego_planner_node', 'traj_server', 'goal_manager',
                     'decision_manager', 'flight_bridge', 'modify_map_to_odom_node'):
            assert name in nodes
        params = {}
        for group in evaluate_parameters(context, nodes['flight_bridge']._Node__parameters):
            params.update(group)
        assert params['use_sim_time'] is sim
        assert params['bootstrap_takeoff'] is values['simulation' if sim else 'hardware']['bootstrap_takeoff']
        assert ('depth_filter' in nodes) is sim


def test_camera_free_microvio_does_not_import_camera_code(startup, monkeypatch):
    import builtins
    original = builtins.__import__

    def without_camera(name, *args, **kwargs):
        if name in ('numpy', 'vio_bridge.calibration', 'vio_bridge.vio_geometry',
                    'stereo_depth.stereo_matcher'):
            raise ImportError('Camera/VIO implementation unavailable in this check')
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', without_camera)
    spec = importlib.util.spec_from_file_location('camera_free_startup', startup.__file__)
    isolated_startup = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(isolated_startup)
    _, nodes = launch_nodes(isolated_startup, False, 'microvio', False)
    assert 'ego_planner_node' in nodes and 'microvio' in nodes


def test_microvio_stereo_depth_needs_images_but_not_camera_imu(startup):
    values = yaml.safe_load((Path(get_package_share_directory('bringup')) /
                            'params/launch.yaml').read_text())
    params = {**values['startup'], **values['hardware'],
              'sim': False, 'rviz': False, 'flight_control': False,
              'perception_enabled': True, 'depth_source': 'software', 'imu_topic': ''}
    context = LaunchContext()
    context.launch_configurations.update({key: startup.launch_value(value)
                                          for key, value in params.items()})
    nodes = {perform_substitutions(context, normalize_to_list_of_substitutions(action.node_executable)): action
             for action in startup.setup(context)
             if isinstance(action, Node) and (action.condition is None or action.condition.evaluate(context))}
    assert 'microvio' in nodes and 'software_stereo' in nodes
    assert 'run_subscribe_msckf' not in nodes
    relay = {}
    for group in evaluate_parameters(context, nodes['sensor_relay']._Node__parameters):
        relay.update(group)
    assert relay['imu'] == ''
