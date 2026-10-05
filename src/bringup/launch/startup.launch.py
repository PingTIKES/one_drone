"""Start sensing, OpenVINS, EGO-Planner, PX4 control and RViz."""
from pathlib import Path
import tempfile
import numpy as np
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from vio_bridge.calibration import (hardware_depth_transform, read_yaml,
                                    transform, validate_config, write_opencv_yaml)
from vio_bridge.vio_geometry import quaternion
from stereo_depth.stereo_matcher import StereoMatcher


def node_config(package_name, node_name):
    path = Path(get_package_share_directory(package_name)) / 'config/params.yaml'
    values = yaml.safe_load(path.read_text())[node_name]['ros__parameters']
    return {key: value for key, value in values.items() if value is not None}


def selected_map_file(bringup, override):
    # The YAML is selected here; its image: field selects the matching PGM.
    choice = override or (yaml.safe_load(
        (bringup / 'params/global_config.yaml').read_text()) or {}).get('map')
    if not isinstance(choice, str) or not choice.strip():
        raise ValueError('set map in bringup/params/global_config.yaml or pass map_file:=...')
    path = Path(choice).expanduser()
    if not path.is_absolute():
        path = bringup / 'map' / path
    path = path.resolve()
    if not path.is_file() or path.suffix.lower() not in ('.yaml', '.yml'):
        raise FileNotFoundError(f'prior map YAML does not exist: {path}')
    return str(path)


def generate_launch_description():
    # Runtime calibration depends on launch arguments. setup() builds the named
    # node list below after those arguments have been resolved.
    arguments = [('sim', 'true'), ('rviz', 'true'), ('depth_source', 'software'),
                 ('calibration_dir', ''), ('target_system', ''), ('px4_ns', ''),
                 ('map_file', ''),
                 ('cam0_topic', ''), ('cam1_topic', ''),
                 ('imu_topic', ''), ('depth_topic', ''),
                 ('depth_info_topic', ''), ('depth_scale', '0.001'),
                 ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in arguments],
        OpaqueFunction(function=setup),
    ])


def setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    sim = arg('sim').lower() == 'true'
    mode = arg('depth_source')
    if mode not in ('software', 'hardware'):
        raise ValueError('depth_source must be software or hardware')
    if sim and mode != 'software':
        raise ValueError('simulation uses raw stereo, not Gazebo ideal depth')
    if not sim and not arg('calibration_dir'):
        raise ValueError('hardware needs a measured calibration_dir')
    if not sim and (not arg('target_system') or int(arg('target_system')) <= 0):
        raise ValueError('hardware needs explicit PX4 target_system')
    if not sim and any(not arg(key) for key in
                       ('cam0_topic', 'cam1_topic', 'imu_topic', 'depth_topic', 'depth_info_topic')):
        raise ValueError('hardware needs explicit measured camera, IMU and depth topics')
    target_system = int(arg('target_system') or '2')
    px4_ns = arg('px4_ns') or ('px4_1' if sim else '/')
    bringup_prefix = Path(get_package_prefix('bringup')).resolve()
    ov_prefix = Path(get_package_prefix('ov_msckf')).resolve()
    if ov_prefix.parent != bringup_prefix.parent:
        raise RuntimeError(
            f'OpenVINS resolves outside this workspace: {ov_prefix}; '
            'source one_drone/install/setup.bash after any old ROS overlays')
    bringup = Path(get_package_share_directory('bringup'))
    map_file = selected_map_file(bringup, arg('map_file'))
    process_config = yaml.safe_load((bringup / 'params/launch.yaml').read_text())['ego_planner']
    if float(process_config['respawn_delay']) < 0:
        raise ValueError('respawn_delay must be nonnegative')
    if sim:
        cfg_dir = Path(get_package_share_directory('vio_bridge')) / 'config/openvins_sim'
        config, cameras, imu = validate_config(cfg_dir / 'estimator_config.yaml')
        t_body_imu = np.eye(4)
        t_body_imu[:3, 3] = [.17, 0., -.06]
        for cam in cameras.values():
            cam['T_imu_cam'] = (np.linalg.inv(t_body_imu) @ np.array(cam['T_imu_cam'])).tolist()
        imu.update(update_rate=200.,
                   gyroscope_noise_density=float(.0017 / np.sqrt(200)),
                   accelerometer_noise_density=float(.02 / np.sqrt(200)))
    else:
        cfg_dir = Path(arg('calibration_dir'))
        config, cameras, imu = validate_config(cfg_dir / 'estimator_config.yaml')
        body = read_yaml(cfg_dir / 'body.yaml')
        t_body_imu = transform(body['T_body_imu'])
    # Estimator tuning is read from calibration_dir/estimator_config.yaml.
    for i in range(2):
        cameras[f'cam{i}']['rostopic'] = f'/uav1/cam{i}/image_raw'
    imu['rostopic'] = '/uav1/imu0'
    work = Path(tempfile.mkdtemp(prefix='one_drone_'))
    for name, values in [('estimator_config.yaml', config),
                         ('kalibr_imucam_chain.yaml', cameras),
                         ('kalibr_imu_chain.yaml', {'imu0': imu})]:
        write_opencv_yaml(work / name, values)
    ov_config = str(work / 'estimator_config.yaml')
    common = {'use_sim_time': sim}
    bridge_topics = [f'/uav1/vio_cam{i}/image@sensor_msgs/msg/Image[gz.msgs.Image'
                     for i in range(2)]
    bridge_topics += ['/uav1/d435i/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                      '/uav1/d435i/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    gazebo_sensors = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='gazebo_sensors', arguments=bridge_topics,
        condition=IfCondition('true' if sim else 'false'),
        remappings=[('/uav1/vio_cam0/image', '/uav1/cam0/image_raw'),
                    ('/uav1/vio_cam1/image', '/uav1/cam1/image_raw'),
                    ('/uav1/d435i/imu', '/uav1/imu0')])
    sensor_relay = Node(
        package='camera_stream', executable='sensor_relay',
        namespace='uav1', name='sensor_relay', output='screen',
        condition=IfCondition('false' if sim else 'true'),
        parameters=[common, node_config('camera_stream', 'sensor_relay'),
                    {'cam0': arg('cam0_topic'), 'cam1': arg('cam1_topic'),
                     'imu': arg('imu_topic')}])
    openvins = Node(
        package='ov_msckf', executable='run_subscribe_msckf',
        namespace='uav1', name='openvins', output='screen',
        parameters=[common, node_config('vio_bridge', 'openvins'),
                    {'config_path': ov_config,
                     'publish_global_to_imu_tf': False,
                     'publish_calibration_tf': False}])
    vio_bridge = Node(
        package='vio_bridge', executable='vio_to_px4.py',
        name='vio_bridge', output='screen',
        parameters=[common, node_config('vio_bridge', 'vio_bridge'),
                    {'px4_ns': px4_ns,
                     't_body_imu': t_body_imu.ravel().tolist()}],
        remappings=[('odomimu', '/uav1/odomimu'),
                    ('cam0/image_raw', '/uav1/cam0/image_raw'),
                    ('cam1/image_raw', '/uav1/cam1/image_raw'),
                    ('imu0', '/uav1/imu0')])
    expected_depth_frame = ''
    if mode == 'software':
        matcher = StereoMatcher(ov_config, t_body_imu)
        t_body_camera = matcher.body_optical
        source_depth_topic = '/uav1/d435i/depth/image_raw'
        depth_info_topic = '/uav1/d435i/depth/camera_info'
    else:
        matcher = None
        t_body_camera, expected_depth_frame = hardware_depth_transform(
            body, cameras, arg('cam0_topic'), arg('depth_topic'),
            arg('depth_info_topic'))
        source_depth_topic = arg('depth_topic')
        depth_info_topic = arg('depth_info_topic')
    software_stereo = Node(
        package='stereo_depth', executable='software_stereo',
        namespace='uav1', name='software_stereo', output='screen',
        condition=IfCondition('true' if mode == 'software' else 'false'),
        parameters=[common, node_config('stereo_depth', 'software_stereo'),
                    {'config': ov_config,
                     't_body_imu': t_body_imu.ravel().tolist(),
                     'sync_slop': node_config('stereo_depth', 'software_stereo').get('sync_slop', .015 if sim else .003)}])
    depth_filter = Node(
        package='depth_filter', executable='depth_filter',
        namespace='uav1', name='depth_filter', output='screen',
        parameters=[common, node_config('depth_filter', 'depth_filter'),
                    {'depth_scale': float(arg('depth_scale')),
                     'expected_frame_id': expected_depth_frame,
                     'camera_info_topic': depth_info_topic}],
        remappings=[('depth/image_raw', source_depth_topic),
                    ('depth/image_filtered', '/uav1/d435i/depth/image_filtered')])
    depth_remaps = [('d435i/depth/image_raw', '/uav1/d435i/depth/image_filtered'),
                    ('d435i/depth/camera_info', depth_info_topic)]
    stereo_depth_node = Node(
        package='obstacle_cloud', executable='stereo_depth_node',
        namespace='uav1', name='stereo_depth_node', output='screen',
        parameters=[common, node_config('obstacle_cloud', 'stereo_depth_node'),
                    {'cam_xyz': t_body_camera[:3, 3].tolist(),
                     'cam_rotation': t_body_camera[:3, :3].ravel().tolist(),
                     'fx': float(matcher.p[0, 0]) if matcher else 337.2,
                     'fy': float(matcher.p[1, 1]) if matcher else 337.2,
                     'cx': float(matcher.p[0, 2]) if matcher else 319.5,
                     'cy': float(matcher.p[1, 2]) if matcher else 239.5,
                     'require_camera_info': mode == 'hardware',
                     'preserve_stamp': True,
                     'depth_scale': float(arg('depth_scale')),
                     'self_mask_model': 'x500' if sim else 'none',
                     'frame_decimation': node_config('obstacle_cloud', 'stereo_depth_node').get('frame_decimation', 1 if mode == 'software' else 3)}],
        remappings=depth_remaps)
    ego_depth_topic = '/uav1/d435i/depth/image_filtered'
    ego_info_topic = depth_info_topic
    ego_odom_adapter = Node(
        package='ego_bridge', executable='ego_odom_adapter',
        name='ego_odom_adapter', output='screen',
        parameters=[common, node_config('ego_bridge', 'ego_odom_adapter'),
                    {'depth_topic': ego_depth_topic}])
    ego_planner = Node(
        package='ego_planner', executable='ego_planner_node',
        name='ego_planner_node', output='screen',
        # Keep the control bridge alive and restart only the planner process.
        # A small non-zero delay prevents a tight fork/crash loop.
        respawn=bool(process_config['respawn']),
        respawn_delay=float(process_config['respawn_delay']),
        parameters=[str(bringup / 'params/ego_params.yaml'), common,
                    {'grid_map/cam2body': t_body_camera.ravel().tolist()}],
        remappings=[
            ('odom_world', '/ego/odom'),
            ('grid_map/odom', '/ego/odom'),
            ('grid_map/depth', ego_depth_topic),
            ('grid_map/camera_info', ego_info_topic),
            ('grid_map/cloud', '/ego/unused_cloud'),
            ('planning/bspline', '/ego/planning/bspline'),
            ('planning/data_display', '/ego/planning/data_display'),
            ('planning/broadcast_bspline_from_planner', '/ego/broadcast_bspline'),
            ('planning/broadcast_bspline_to_planner', '/ego/broadcast_bspline'),
            ('grid_map/occupancy', '/ego/occupancy'),
            ('grid_map/occupancy_inflate', '/ego/occupancy_inflate'),
            ('goal_point', '/ego/visualization/goal'),
            ('global_list', '/ego/visualization/global_path'),
            ('init_list', '/ego/visualization/initial_path'),
            ('optimal_list', '/ego/visualization/optimal_path'),
            ('a_star_list', '/ego/visualization/a_star')])
    ego_traj_server = Node(
        package='ego_planner', executable='traj_server',
        name='traj_server', output='screen',
        parameters=[str(bringup / 'params/ego_params.yaml'), common],
        remappings=[('planning/bspline', '/ego/planning/bspline'),
                    ('position_cmd', '/ego/position_cmd')])
    camera_q = quaternion(t_body_camera[:3, :3])
    camera_optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_optical_tf', arguments=[
            '--x', str(float(t_body_camera[0, 3])),
            '--y', str(float(t_body_camera[1, 3])),
            '--z', str(float(t_body_camera[2, 3])),
            '--qx', str(float(camera_q[1])), '--qy', str(float(camera_q[2])),
            '--qz', str(float(camera_q[3])), '--qw', str(float(camera_q[0])),
            '--frame-id', 'base_link', '--child-frame-id', 'camera_optical'])
    goal_manager = Node(
        package='goal_manager', executable='goal_manager',
        name='goal_manager', output='screen',
        parameters=[common, node_config('goal_manager', 'goal_manager'),
                    {'goal_frame': 'odom'}])
    modify_map_to_odom = Node(
        package='modify_map_to_odom', executable='modify_map_to_odom_node',
        name='modify_map_to_odom', output='screen',
        parameters=[str(Path(get_package_share_directory('modify_map_to_odom')) /
                        'config/config.yaml'), common])
    map_container = ComposableNodeContainer(
        name='map_container', namespace='', package='rclcpp_components',
        executable='component_container', output='screen', parameters=[common])
    load_map_server = LoadComposableNodes(
        target_container='map_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server', plugin='nav2_map_server::MapServer',
                name='map_server', parameters=[common, {'yaml_filename': map_file}]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[common, {'autostart': True,
                                     'node_names': ['map_server']}]),
        ])
    flight_bridge = Node(
        package='flight_bridge', executable='flight_bridge',
        name='flight_bridge', output='screen',
        parameters=[common, node_config('flight_bridge', 'flight_bridge'),
                    {'target_system': target_system, 'px4_ns': px4_ns}])
    one_drone_rviz = Node(
        package='rviz2', executable='rviz2', name='one_drone_rviz',
        output='screen', condition=IfCondition(arg('rviz').lower()),
        parameters=[common],
        arguments=['-d', str(bringup / 'rviz/ego_navigation.rviz')])

    # Startup inventory. EGO-Planner is part of this single launch chain.
    return LaunchDescription([
        gazebo_sensors,              # sim: Gazebo camera, IMU, clock bridge
        sensor_relay,                # hardware: measured camera/IMU topics
        openvins,                    # stereo VIO
        vio_bridge,                  # VIO -> PX4 and odom TF
        software_stereo,             # sim/software depth
        depth_filter,                # temporal/spatial depth cleanup
        stereo_depth_node,           # filtered depth -> obstacle cloud
        camera_optical_tf,           # measured base_link -> camera_optical
        modify_map_to_odom,          # adjustable map -> odom TF
        map_container,               # composable prior PGM map server
        load_map_server,             # map server + lifecycle activation
        ego_odom_adapter,            # body twist -> world twist + depth health
        ego_planner,                 # depth GridMap + local B-spline planning
        ego_traj_server,             # B-spline -> PositionCommand
        goal_manager,                # RViz goal -> EGO target
        flight_bridge,               # EGO PositionCommand -> PX4 Offboard
        one_drone_rviz,
    ]).entities
