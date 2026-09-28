"""Start sensing, localization, map, flight control and RViz; launch Nav2 separately."""
from pathlib import Path
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vio_bridge.calibration import read_yaml, transform, validate_config, write_opencv_yaml
from vio_bridge.vio_geometry import quaternion
from stereo_depth.stereo_matcher import StereoMatcher


def node_config(package_name, node_name):
    path = Path(get_package_share_directory(package_name)) / 'config/params.yaml'
    values = yaml.safe_load(path.read_text())[node_name]['ros__parameters']
    return {key: value for key, value in values.items() if value is not None}


def selected_map_file(bringup, override):
    """Resolve the one prior map selected in bringup/params/global_config.yaml."""
    if override:
        choice = override
    else:
        config = yaml.safe_load((bringup / 'params/global_config.yaml').read_text()) or {}
        choice = config.get('map')
    if not isinstance(choice, str) or not choice.strip():
        raise ValueError('set map in bringup/params/global_config.yaml or pass map_file:=...')
    path = Path(choice).expanduser()
    if not path.is_absolute():
        path = bringup / 'map' / path
    path = path.resolve()
    if not path.is_file() or path.suffix.lower() not in ('.yaml', '.yml'):
        raise ValueError(f'prior map YAML does not exist: {path}')
    image = (yaml.safe_load(path.read_text()) or {}).get('image')
    if not isinstance(image, str) or not image.strip():
        raise ValueError(f'prior map YAML needs an image field: {path}')
    image_path = Path(image)
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    if not image_path.is_file():
        raise ValueError(f'prior map image does not exist: {image_path}')
    return str(path)


def generate_launch_description():
    # Runtime calibration depends on launch arguments. setup() builds the named
    # node list below after those arguments have been resolved.
    arguments = [('sim', 'true'), ('rviz', 'true'), ('depth_source', 'software'),
                 ('calibration_dir', ''), ('target_system', ''), ('px4_ns', ''),
                 ('map_file', ''), ('cam0_topic', ''), ('cam1_topic', ''),
                 ('imu_topic', ''), ('depth_topic', ''),
                 ('depth_info_topic', ''), ('depth_scale', '0.001'),
                 ('navigation_mode', 'odom')]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in arguments],
        OpaqueFunction(function=setup),
    ])


def setup(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    sim = arg('sim').lower() == 'true'
    navigation_mode = arg('navigation_mode')
    if navigation_mode not in ('map', 'odom'):
        raise ValueError('navigation_mode must be map or odom')
    use_prior_map = navigation_mode == 'map'
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
    bringup = Path(get_package_share_directory('bringup'))
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
        t_body_imu = transform(read_yaml(cfg_dir / 'body.yaml')['T_body_imu'])
    config.update(verbosity='WARNING', record_timing_information=True,
                  record_timing_filepath='/tmp/one_drone_openvins_timing.txt',
                  num_opencv_threads=2, use_multi_threading_subs=False)
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
                    ('cam1/image_raw', '/uav1/cam1/image_raw')])
    if mode == 'software':
        matcher = StereoMatcher(ov_config, t_body_imu)
        t_body_camera = matcher.body_optical
        depth_remaps = []
    else:
        matcher = None
        t_body_camera = transform(read_yaml(cfg_dir / 'body.yaml')['T_body_depth'])
        depth_remaps = [('d435i/depth/image_raw', arg('depth_topic')),
                        ('d435i/depth/camera_info', arg('depth_info_topic'))]
    software_stereo = Node(
        package='stereo_depth', executable='software_stereo',
        namespace='uav1', name='software_stereo', output='screen',
        condition=IfCondition('true' if mode == 'software' else 'false'),
        parameters=[common, node_config('stereo_depth', 'software_stereo'),
                    {'config': ov_config,
                     't_body_imu': t_body_imu.ravel().tolist(),
                     'sync_slop': .015 if sim else .003}])
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
                     'frame_decimation': 1 if mode == 'software' else 3}],
        remappings=depth_remaps)
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
    modify_map_to_odom = Node(
        package='modify_map_to_odom', executable='modify_map_to_odom_node',
        name='modify_map_to_odom', output='screen',
        condition=IfCondition('true' if use_prior_map else 'false'),
        parameters=[str(Path(get_package_share_directory('modify_map_to_odom')) /
                        'config/config.yaml'), common])
    map_odom = Node(
        package='map_alignment', executable='map_odom',
        name='map_odom', output='screen',
        condition=IfCondition('true' if use_prior_map else 'false'),
        parameters=[common])
    height_slice = Node(
        package='obstacle_filter', executable='height_slice',
        name='height_slice', output='screen',
        parameters=[common, node_config('obstacle_filter', 'height_slice')])
    goal_manager = Node(
        package='goal_manager', executable='goal_manager',
        name='goal_manager', output='screen',
        parameters=[common, node_config('goal_manager', 'goal_manager'),
                    {'goal_frame': navigation_mode,
                     'require_map_alignment': use_prior_map}])
    flight_bridge = Node(
        package='flight_bridge', executable='flight_bridge',
        name='flight_bridge', output='screen',
        parameters=[common, node_config('flight_bridge', 'flight_bridge'),
                    {'target_system': target_system, 'px4_ns': px4_ns,
                     'require_map_alignment': use_prior_map}])
    map_file = selected_map_file(bringup, arg('map_file')) if use_prior_map else ''
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        condition=IfCondition('true' if use_prior_map else 'false'),
        parameters=[common, {'yaml_filename': map_file}])
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        condition=IfCondition('true' if use_prior_map else 'false'),
        parameters=[common, {'autostart': True, 'node_names': ['map_server']}])
    one_drone_rviz = Node(
        package='rviz2', executable='rviz2', name='one_drone_rviz',
        output='screen', condition=IfCondition(arg('rviz').lower()),
        parameters=[common],
        arguments=['-d', str(bringup / ('rviz/navigation.rviz' if use_prior_map
                                         else 'rviz/navigation_odom.rviz'))])

    # Startup inventory. Nav2 planner/controller/smoother start in nav/bringup_launch.py.
    return LaunchDescription([
        gazebo_sensors,              # sim: Gazebo camera, IMU, clock bridge
        sensor_relay,                # hardware: measured camera/IMU topics
        openvins,                    # stereo VIO
        vio_bridge,                  # VIO -> PX4 and odom TF
        software_stereo,             # sim/software depth
        stereo_depth_node,           # depth -> obstacle cloud
        camera_optical_tf,           # base_link -> camera_optical
        modify_map_to_odom,          # manual relocalization interface
        map_odom,                    # map -> odom TF
        height_slice,                # depth cloud at flight height
        goal_manager,                # RViz goal -> Nav2 action
        flight_bridge,               # cmd_vel_smoothed -> PX4 Offboard
        map_server,                  # prior PGM map
        lifecycle_manager_map,       # activate prior map independently
        one_drone_rviz,
    ]).entities
