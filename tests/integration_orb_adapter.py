"""Offline ROS integration smoke test; synthetic static input, isolated domain, no PX4."""
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path
import importlib.util
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from ament_index_python.packages import get_package_prefix, get_package_share_directory

ROOT = Path(__file__).resolve().parents[1]

def main():
    if os.environ.get('ROS_DOMAIN_ID') != '82':
        raise RuntimeError('Run with ROS_DOMAIN_ID=82 to isolate synthetic inputs')
    spec = importlib.util.spec_from_file_location('orb_calibration',ROOT/'src/localization/orb_slam3/scripts/prepare_calibration.py')
    mod = importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    rclpy.init();node=rclpy.create_node('orb_adapter_test')
    status=[];epochs=[];odom=[]
    node.create_subscription(String,'/orbslam/tracking_state',lambda m:status.append(m.data),10)
    node.create_subscription(Odometry,'/odomimu',lambda m:odom.append(m),10)
    node.create_subscription(Header,'/estimator_reset',lambda m:epochs.append(m.frame_id),QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    left=node.create_publisher(Image,'/cam0/image_raw',qos_profile_sensor_data)
    right=node.create_publisher(Image,'/cam1/image_raw',qos_profile_sensor_data)
    imu=node.create_publisher(Imu,'/imu0',qos_profile_sensor_data)
    prefix=Path(get_package_prefix('orb_slam3'));share=Path(get_package_share_directory('orb_slam3'))
    texture=np.random.default_rng(41).integers(0,256,(480,640),dtype=np.uint8)
    def image(data):
        m=Image(height=480,width=640,encoding='mono8',step=640,data=data.tobytes());m.header.stamp=node.get_clock().now().to_msg();return m
    with tempfile.TemporaryDirectory() as d:
        settings,maps,offset=mod.prepare(ROOT/'deploy/calibration/uav1/estimator_config.yaml',share/'config/params.yaml',d)
        with open('/tmp/one_drone_orb_adapter.log','w') as log:
            proc=subprocess.Popen([str(prefix/'lib/orb_slam3/stereo_inertial'),'--ros-args','--params-file',str(share/'config/params.yaml'),'-p','settings_file:='+settings,'-p','rectification_file:='+maps,'-p','vocabulary_file:='+str(share/'vocabulary/ORBvoc.txt'),'-p','camera_imu_timeshift:='+str(offset)],stdout=log,stderr=subprocess.STDOUT)
            try:
                deadline=time.monotonic()+20
                while not epochs and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.1)
                    assert proc.poll() is None,'ORB startup exited'
                assert epochs,'ORB did not finish startup'
                deadline=time.monotonic()+3;next_image=0.
                while time.monotonic()<deadline:
                    m=Imu();m.header.stamp=node.get_clock().now().to_msg();m.linear_acceleration.y=-9.81;imu.publish(m)
                    if time.monotonic()>=next_image:
                        a=image(texture);b=image(np.roll(texture,-4,axis=1));b.header.stamp=a.header.stamp;left.publish(a);right.publish(b);next_image=time.monotonic()+1/30
                    rclpy.spin_once(node,timeout_sec=.002)
                    time.sleep(.003)
                    assert proc.poll() is None,'ORB exited while tracking'
                assert status,'No tracking status received'
                assert not odom,'Static input must not publish uninitialized inertial odometry'
                bad=image(texture);bad.header.stamp.sec-=1;left.publish(bad)
                deadline=time.monotonic()+3
                while not any('STAMP_REGRESSION' in x for x in epochs) and time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                assert any('STAMP_REGRESSION' in x for x in epochs),'Source time regression must emit reset'
                print('PASS: startup, real image/IMU processing, no uninitialized odometry, time regression reset')
            finally:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
