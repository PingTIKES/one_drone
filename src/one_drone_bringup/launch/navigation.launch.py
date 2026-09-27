"""Single-drone algorithm: Gazebo supplies sensors, OpenVINS/Nav2/PX4 fly."""
from pathlib import Path
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_localization.calibration import read_yaml, transform, validate_config, write_opencv_yaml
from uav_localization.vio_geometry import quaternion
from uav_perception.stereo_matcher import StereoMatcher


def node_config(package_name, node_name):
    path = Path(get_package_share_directory(package_name)) / 'config/params.yaml'
    values = yaml.safe_load(path.read_text())[node_name]['ros__parameters']
    return {key: value for key, value in values.items() if value is not None}


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
    package = Path(get_package_share_directory('one_drone_bringup'))
    if sim:
        cfg_dir = Path(get_package_share_directory('uav_localization')) / 'config/openvins_sim'
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
    nodes = []
    if sim:
        bridge = [f'/uav1/vio_cam{i}/image@sensor_msgs/msg/Image[gz.msgs.Image' for i in range(2)]
        bridge += ['/uav1/d435i/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                   '/uav1/d435i/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                          name='gazebo_sensors', arguments=bridge,
                          remappings=[('/uav1/vio_cam0/image', '/uav1/cam0/image_raw'),
                                      ('/uav1/vio_cam1/image', '/uav1/cam1/image_raw'),
                                      ('/uav1/d435i/imu', '/uav1/imu0')]))
    else:
        nodes.append(Node(package='uav_perception', executable='sensor_relay',
                          namespace='uav1', name='sensor_relay', output='screen',
                          parameters=[common, node_config('uav_perception', 'sensor_relay'),
                                      {'cam0': arg('cam0_topic'),
                                               'cam1': arg('cam1_topic'),
                                               'imu': arg('imu_topic')}]))
    nodes.append(Node(package='ov_msckf', executable='run_subscribe_msckf',
                      namespace='uav1', name='openvins', output='screen',
                      parameters=[common, node_config('uav_localization', 'openvins'),
                                  {'config_path': ov_config,
                                           'publish_global_to_imu_tf': False,
                                           'publish_calibration_tf': False}]))
    nodes.append(Node(package='uav_localization', executable='vio_to_px4.py',
                      name='vio_bridge', output='screen',
                      parameters=[common, node_config('uav_localization', 'vio_bridge'),
                                  {'px4_ns': px4_ns,
                                           't_body_imu': t_body_imu.ravel().tolist()}],
                      remappings=[('odomimu', '/uav1/odomimu'),
                                  ('cam0/image_raw', '/uav1/cam0/image_raw'),
                                  ('cam1/image_raw', '/uav1/cam1/image_raw')]))
    if mode == 'software':
        matcher = StereoMatcher(ov_config, t_body_imu)
        t_body_camera = matcher.body_optical
        nodes.append(Node(package='uav_perception', executable='software_stereo',
                          namespace='uav1', name='software_stereo', output='screen',
                          parameters=[common, node_config('uav_perception', 'software_stereo'),
                                      {'config': ov_config,
                                               't_body_imu': t_body_imu.ravel().tolist(),
                                               'sync_slop': .015 if sim else .003}]))
        depth_remaps = []
    else:
        t_body_camera = transform(read_yaml(cfg_dir / 'body.yaml')['T_body_depth'])
        depth_remaps = [('d435i/depth/image_raw', arg('depth_topic')),
                        ('d435i/depth/camera_info', arg('depth_info_topic'))]
    nodes.append(Node(package='uav_perception', executable='stereo_depth_node',
                      namespace='uav1', name='stereo_depth_node', output='screen',
                      parameters=[common, node_config('uav_perception', 'stereo_depth_node'),
                                  {'cam_xyz': t_body_camera[:3, 3].tolist(),
                                           'cam_rotation': t_body_camera[:3, :3].ravel().tolist(),
                                           'fx': float(matcher.p[0, 0]) if mode == 'software' else 337.2,
                                           'fy': float(matcher.p[1, 1]) if mode == 'software' else 337.2,
                                           'cx': float(matcher.p[0, 2]) if mode == 'software' else 319.5,
                                           'cy': float(matcher.p[1, 2]) if mode == 'software' else 239.5,
                                           'require_camera_info': mode == 'hardware',
                                           'preserve_stamp': True,
                                           'depth_scale': float(arg('depth_scale')),
                                           'self_mask_model': 'x500' if sim else 'none',
                                           'frame_decimation': 1 if mode == 'software' else 3}],
                      remappings=depth_remaps))
    camera_q = quaternion(t_body_camera[:3, :3])
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                      name='camera_optical_tf', arguments=[
                          '--x', str(float(t_body_camera[0, 3])),
                          '--y', str(float(t_body_camera[1, 3])),
                          '--z', str(float(t_body_camera[2, 3])),
                          '--qx', str(float(camera_q[1])), '--qy', str(float(camera_q[2])),
                          '--qz', str(float(camera_q[3])), '--qw', str(float(camera_q[0])),
                          '--frame-id', 'base_link', '--child-frame-id', 'camera_optical']))
    nodes += [Node(package='one_drone_navigation', executable='map_odom',
                   name='map_odom', output='screen',
                   parameters=[common, node_config('one_drone_navigation', 'map_odom')]),
              Node(package='one_drone_navigation', executable='height_slice',
                   name='height_slice', output='screen',
                   parameters=[common, node_config('one_drone_navigation', 'height_slice')]),
              Node(package='one_drone_navigation', executable='goal_manager',
                   name='goal_manager', output='screen',
                   parameters=[common, node_config('one_drone_navigation', 'goal_manager')]),
              Node(package='one_drone_control', executable='flight_bridge',
                   name='flight_bridge', output='screen',
                   parameters=[common, node_config('one_drone_control', 'flight_bridge'),
                               {'target_system': target_system,
                                'px4_ns': px4_ns}])]
    nav = yaml.safe_load((package / 'config/nav2.yaml').read_text())
    for name, section in nav.items():
        if name in ('local_costmap', 'global_costmap'):
            section[name]['ros__parameters']['use_sim_time'] = sim
        else:
            section['ros__parameters']['use_sim_time'] = sim
    nav_file = work / 'nav2.yaml'
    nav_file.write_text(yaml.safe_dump(nav, sort_keys=False))
    map_file = arg('map_file') or str(package / 'maps/rmuc_2025_prior.yaml')
    bt_file = str(package / 'config/navigate.xml')
    nav_nodes = [('nav2_map_server', 'map_server', 'map_server', {'yaml_filename': map_file}),
                 ('nav2_planner', 'planner_server', 'planner_server', {}),
                 ('nav2_controller', 'controller_server', 'controller_server', {}),
                 ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
                  {'default_nav_to_pose_bt_xml': bt_file,
                   'default_nav_through_poses_bt_xml': str(package / 'config/unused_through_poses.xml')}),
                 ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', {})]
    for package_name, executable, name, override in nav_nodes:
        nodes.append(Node(package=package_name, executable=executable, name=name,
                          output='screen', parameters=[str(nav_file), override]))
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[common, {'autostart': True,
                                           'node_names': [item[2] for item in nav_nodes]}]))
    if arg('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', name='one_drone_rviz',
                          parameters=[common], arguments=['-d', str(package / 'config/navigation.rviz')]))
    return nodes


def generate_launch_description():
    args = [('sim', 'true'), ('rviz', 'true'), ('depth_source', 'software'),
            ('calibration_dir', ''), ('target_system', ''), ('px4_ns', ''),
            ('map_file', ''),
            ('cam0_topic', ''), ('cam1_topic', ''),
            ('imu_topic', ''),
            ('depth_topic', ''),
            ('depth_info_topic', ''),
            ('depth_scale', '0.001')]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in args] + [OpaqueFunction(function=setup)])
