"""Run the unchanged ros2 launch command using only an isolated launch.yaml.

Hardware profiles use a loopback MAVLink FCU; no serial device is ever opened.
For --sim --plan-only, start the project Gazebo/PX4 simulation first.
Use an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

import integration_mavros as mavros_test

ROOT = Path(__file__).resolve().parents[1]
args_parser = argparse.ArgumentParser()
args_parser.add_argument('--perception', action='store_true')
args_parser.add_argument('--depth-source', choices=('hardware', 'software'), default='hardware')
args_parser.add_argument('--sim', action='store_true')
args_parser.add_argument('--plan-only', action='store_true')


def main():
    args = args_parser.parse_args()
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 180, 'Use an isolated ROS domain >=180'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert not args.sim or args.plan_only, 'This simulation check does not arm PX4'
    if args.sim and os.environ.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp':
        # Loopback-only tests disable multicast. Cyclone's default ten auto
        # participant slots are fewer than this launch's process count.
        os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                              '<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
                              '</Discovery></Domain></CycloneDDS>')
    mavros_test.BOOTSTRAP = True
    fcu = None if args.sim else mavros_test.FakeFCU()
    rclpy.init(args=[])
    probe = rclpy.create_node('startup_profile_probe', parameter_overrides=[
        Parameter('use_sim_time', value=args.sim)])
    state = {'flight': [], 'decisions': [], 'goals': [], 'paths': [], 'depth': 0, 'maps': [], 'odom': None}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        probe.create_subscription(String, '/flight_state', lambda m: state['flight'].append(m.data), 10),
        probe.create_subscription(String, '/decision/state', lambda m: state['decisions'].append(json.loads(m.data)), latched),
        probe.create_subscription(Odometry, '/ego/odom', lambda m: state.update(odom=m), 10),
        probe.create_subscription(PoseStamped, '/ego/goal', state['goals'].append, 10),
        probe.create_subscription(Marker, '/ego/visualization/optimal_path',
                                  lambda m: state['paths'].append(m) if m.points else None, 10),
        probe.create_subscription(OccupancyGrid, '/map', state['maps'].append, latched),
        probe.create_subscription(Image, '/uav1/d435i/depth/image_filtered',
                                  lambda m: state.update(depth=state['depth']+1), qos_profile_sensor_data),
        probe.create_subscription(State, '/mavros/state', lambda m: state.update(fcu=m), qos_profile_sensor_data),
        probe.create_subscription(Bool, '/flight_navigation_ready', lambda m: state.update(ready=m.data), latched)]
    goal_pub = probe.create_publisher(PoseStamped, '/navigation_goal', 10)
    client = probe.create_client(Trigger, '/takeoff')
    child = None
    label = 'simulation' if args.sim else ('microvio_'+args.depth_source if args.perception else 'microvio_no_camera')
    log_path = Path('/tmp/one_drone_profile_'+label+'.log')
    log = log_path.open('w+')
    try:
        # This prefix has the same parent as the workspace packages. All assets
        # are shared; only launch.yaml is copied and changed for the test.
        with tempfile.TemporaryDirectory(dir=ROOT/'install', prefix='profile_test_') as folder:
            prefix = Path(folder)
            index = prefix/'share/ament_index/resource_index/packages'
            index.mkdir(parents=True)
            (index/'bringup').touch()
            share = prefix/'share/bringup'
            share.mkdir()
            original = ROOT/'install/bringup/share/bringup'
            for asset in ('map', 'rviz'):
                (share/asset).symlink_to(original/asset, target_is_directory=True)
            (share/'launch').mkdir()
            for source in (original/'launch').iterdir():
                if source.is_file():
                    (share/'launch'/source.name).symlink_to(source)
            (share/'params').mkdir()
            for source in (original/'params').iterdir():
                if source.name != 'launch.yaml':
                    (share/'params'/source.name).symlink_to(source)
            config = yaml.safe_load((original/'params/launch.yaml').read_text())
            config['startup'].update(sim=args.sim, rviz=False, flight_control=True)
            if fcu is not None:
                config['hardware'].update(
                    localization_source='microvio', perception_enabled=args.perception,
                    depth_source=args.depth_source, imu_topic='',
                    calibration_dir=str(ROOT/'deploy/calibration/uav1') if args.perception else '/no-camera-calibration',
                    fcu_url=f'udp://127.0.0.1:{fcu.mavros_port}@127.0.0.1:{fcu.port}')
            (share/'params/launch.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
            environment = dict(os.environ)
            environment['AMENT_PREFIX_PATH'] = str(prefix)+':'+environment.get('AMENT_PREFIX_PATH', '')
            child = subprocess.Popen(['ros2', 'launch', 'bringup', 'startup.launch.py'],
                                     env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

            def spin(seconds):
                until = time.monotonic()+seconds
                while time.monotonic() < until:
                    assert child.poll() is None, 'startup launch exited'
                    if fcu is not None:
                        fcu.tick()
                    rclpy.spin_once(probe, timeout_sec=.005)

            def wait(predicate, seconds, message):
                deadline = time.monotonic()+seconds
                while not predicate() and time.monotonic() < deadline:
                    spin(.05)
                assert predicate(), message+'; flights='+repr(state['flight'][-5:])+'; decisions='+repr(state['decisions'][-2:])

            wait(lambda: state['maps'] and client.service_is_ready() and
                 state.get('fcu') is not None and state['fcu'].connected, 30., 'map and control startup')
            if args.plan_only:
                wait(lambda: state['odom'] is not None, 45., 'OpenVINS initialization')
            else:
                # Only attitude, vertical and range feedback exists on the ground.
                assert state['odom'] is None and not fcu.armed
                accepted = False
                deadline = time.monotonic()+10.
                while not accepted and time.monotonic() < deadline:
                    request = client.call_async(Trigger.Request())
                    wait(request.done, 3., 'takeoff response')
                    accepted = request.result().success
                    if not accepted:
                        spin(.2)
                assert accepted, 'vertical feedback did not become usable'
                wait(lambda: state.get('ready', False), 25., 'airborne localization and navigation takeover')
                assert all(stage in state['flight'] for stage in ('BOOTSTRAP_PRESTREAM', 'BOOTSTRAP_TAKEOFF', 'TAKEOFF'))
                wait(lambda: any(d.get('completed_laps', 0) >= 1 for d in state['decisions']),
                     60., 'decision_1 completes a patrol lap')
                assert state['paths'] and state['goals'] and 'CRUISE' in state['flight']
                assert state['depth'] == 0, 'Hardware check unexpectedly received depth'
                print('PASS: YAML-only '+label+' -> no-odom liftoff -> navigation -> decision_1 patrol without depth', flush=True)

            before = len(state['paths'])
            manual = PoseStamped()
            manual.header.frame_id = 'map'
            manual.header.stamp = probe.get_clock().now().to_msg()
            manual.pose.orientation.w = 1.
            manual.pose.position.x, manual.pose.position.y = -5.0, 5.0
            goal_pub.publish(manual)
            wait(lambda: len(state['paths']) > before, 10., 'RViz map goal produces a path')
            if args.plan_only:
                assert not state['fcu'].armed, 'Planning must not arm the simulator'
            else:
                wait(lambda: any(d['state'] == 'CANCELED_MANUAL_GOAL' for d in state['decisions']),
                     3., 'manual RViz goal takes over from decision')
            print('PASS: unchanged startup command, prior map and map-frame manual goal; '+label, flush=True)
    except BaseException:
        log.flush();log.seek(0)
        print(log.read()[-7000:])
        raise
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        log.close()
        probe.destroy_node()
        rclpy.shutdown()
        if fcu is not None:
            fcu.sock.close()


if __name__ == '__main__':
    main()
