#!/usr/bin/env python3
"""Guarded OpenVINS odomimu -> PX4 external vision. No ground-truth input."""
import math
import json
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
from vio_bridge.vio_geometry import convert
from vio_bridge.vio_tf_geometry import FLIP, vio_body_pose
from vio_bridge.vio_geometry import quaternion
from vio_bridge.calibration import transform
from vio_bridge.vio_recovery import VioRecovery, unexplained_rotation
from vio_bridge.vio_quality import normalized_covariance, quality_state


class VioBridge(Node):
    def __init__(self):
        super().__init__('vio_to_px4')
        for k,v in dict(uav_id=1,px4_ns='px4_1',max_age=.5,invalid_age=2.,max_position_variance=1.,
                        max_orientation_variance=.25,max_velocity_variance=1.,
                        max_speed=4.,max_jump=.4,max_continuous_gap=2.,
                        expected_world='global',expected_imu='imu',
                        recovery_stable_time=.5,recovery_timeout=2.,recovery_max_correction=.75,
                        recovery_max_angle_deg=35.,recovery_sample_gap=.1,recovery_residual=.15,
                        recovery_max_source_gap=3.,recovery_gap_max_correction=2.5,
                        recovery_gap_max_angle_deg=90.,
                        feature_check_period=.5,feature_bad_count=15,feature_good_count=40,
                        feature_downsample=2,feature_max_corners=200,
                        feature_quality_level=.01,feature_min_distance=4.,feature_block_size=3,
                        quality_velocity_window=.1,quality_acceleration_warn=6.,
                        quality_angular_rate_warn=.6,
                        t_body_imu=np.eye(4).ravel().tolist()).items(): self.declare_parameter(k,v)
        self.p = lambda k: self.get_parameter(k).value
        if (int(self.p('feature_downsample')) < 1 or int(self.p('feature_max_corners')) < 1
                or not 0 < self.p('feature_quality_level') <= 1
                or self.p('feature_min_distance') < 0
                or int(self.p('feature_block_size')) < 1
                or int(self.p('feature_block_size')) % 2 == 0
                or not 0 <= self.p('feature_bad_count') < self.p('feature_good_count')):
            raise ValueError('invalid feature quality parameters')
        if not 0 < self.p('max_age') < self.p('invalid_age'):
            raise ValueError('VIO freshness requires 0 < max_age < invalid_age')
        if self.p('max_continuous_gap') <= self.p('max_age'):
            raise ValueError('max_continuous_gap must be greater than max_age')
        uid = int(self.p('uav_id'))
        self.odom_frame, self.body_frame = 'odom', 'base_link'
        self.extrinsic = transform(np.array(self.p('t_body_imu')).reshape(4,4))
        self.last_stamp = self.last_position = None
        self.last_quat = self.last_velocity = self.last_omega = None
        self.recovery = self.new_recovery()
        self.reason = 'WAITING_FOR_DATA'
        self.reset_count, self.latched = 0, False
        self.last_good = -math.inf
        self.image_at = [-math.inf,-math.inf]
        self.feature_counts = [None, None]
        self.feature_checked_at = [-math.inf, -math.inf]
        self.imu_at = -math.inf
        self.imu_angular_rate = 0.0
        self.imu_acceleration = 0.0
        self.quality_confidence = 0.0
        self.quality_reasons = ['WAITING_FOR_DATA']
        self.quality_velocity = None
        self.quality_velocity_stamp = None
        self.quality_acceleration = 0.0
        self.covariance_ratio = math.inf
        self.position_covariance = self.orientation_covariance = self.velocity_covariance = math.inf
        px4_ns = str(self.p('px4_ns')).strip('/')
        px4_root = '/' + (px4_ns + '/' if px4_ns else '') + 'fmu/'
        self.pub = self.create_publisher(VehicleOdometry,px4_root + 'in/vehicle_visual_odometry',qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self.reset_pub = self.create_publisher(Header, '/vio_reset_event',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_pub = TransformBroadcaster(self)
        self.health = self.create_publisher(String,'vio_health',1)
        self.diagnostics = self.create_publisher(String,'vio_diagnostics',1)
        self.create_subscription(Odometry,'odomimu',self.callback,qos_profile_sensor_data)
        for i in range(2):
            self.create_subscription(Image,f'cam{i}/image_raw',lambda msg,index=i:self.image(msg,index),qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu0', self.imu, qos_profile_sensor_data)
        self.create_service(Trigger,'reset_vio_bridge',self.reset)
        self.create_timer(.1,self.watchdog)
        self.notify_reset('BRIDGE_START')

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def notify_reset(self, reason):
        # The unique epoch is independent of the 8-bit PX4 reset counter.
        self.reset_pub.publish(Header(stamp=self.get_clock().now().to_msg(),
                                      frame_id=f'{time.time_ns()}:{reason}'))

    def new_recovery(self):
        return VioRecovery(*(float(self.p(k)) for k in (
            'recovery_stable_time','recovery_timeout','recovery_max_correction',
            'recovery_max_angle_deg','recovery_sample_gap','recovery_residual',
            'recovery_max_source_gap','recovery_gap_max_correction',
            'recovery_gap_max_angle_deg')))

    def image(self,msg,index):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.image_at[index] = stamp
        if stamp - self.feature_checked_at[index] < float(self.p('feature_check_period')):
            return
        self.feature_checked_at[index] = stamp
        try:
            image = self.gray_image(msg)
            step = int(self.p("feature_downsample"))
            image = image[::step, ::step]
            points = cv2.goodFeaturesToTrack(
                image, maxCorners=int(self.p("feature_max_corners")),
                qualityLevel=float(self.p("feature_quality_level")),
                minDistance=float(self.p("feature_min_distance")),
                blockSize=int(self.p("feature_block_size")), useHarrisDetector=False)
            self.feature_counts[index] = 0 if points is None else len(points)
        except ValueError:
            self.feature_counts[index] = None

    @staticmethod
    def gray_image(msg):
        channels = {'mono8': 1, '8UC1': 1, 'rgb8': 3, 'bgr8': 3,
                    'rgba8': 4, 'bgra8': 4}
        if msg.encoding not in channels:
            raise ValueError('unsupported image encoding')
        channel_count = channels[msg.encoding]
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        packed = rows[:, :msg.width * channel_count]
        image = packed.reshape(msg.height, msg.width, channel_count)
        if channel_count == 1:
            return image[:, :, 0]
        conversion = (cv2.COLOR_RGBA2GRAY if msg.encoding == 'rgba8' else
                      cv2.COLOR_BGRA2GRAY if msg.encoding == 'bgra8' else
                      cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else
                      cv2.COLOR_BGR2GRAY)
        return cv2.cvtColor(image, conversion)

    def imu(self, msg):
        self.imu_at = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_angular_rate = math.sqrt(
            msg.angular_velocity.x ** 2 + msg.angular_velocity.y ** 2 +
            msg.angular_velocity.z ** 2)
        self.imu_acceleration = math.sqrt(
            msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 +
            msg.linear_acceleration.z ** 2)

    def publish_vio_odom(self, msg, stamp, position, orientation, velocity, omega, pv, ov, vv):
        # OpenVINS is the sole odom -> base_link authority. Its first valid
        # world pose defines odom; PX4's local estimate never shifts this TF.
        point, body_rotation = vio_body_pose(position, orientation)
        quat = quaternion(body_rotation)
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.body_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, point)
        tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z = map(float, quat)
        self.tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = tf.transform.translation.x
        odom.pose.pose.position.y = tf.transform.translation.y
        odom.pose.pose.position.z = tf.transform.translation.z
        odom.pose.pose.orientation = tf.transform.rotation
        for i, variance in enumerate((*pv, *ov)):
            odom.pose.covariance[i * 6 + i] = float(variance)
        body_velocity, body_omega = FLIP @ velocity, FLIP @ omega
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, body_omega)
        for i, variance in enumerate(vv):
            odom.twist.covariance[i * 6 + i] = float(variance)
        self.odom_pub.publish(odom)

    def reset(self,request,response):
        self.last_stamp = self.last_position = None
        self.last_quat = self.last_velocity = self.last_omega = None
        self.recovery = self.new_recovery()
        self.reset_count = (self.reset_count+1)%256
        self.notify_reset('MANUAL_RESET')
        self.latched = False
        self.reason = 'MANUAL_RESET'
        self.last_good = -math.inf
        self.quality_velocity = self.quality_velocity_stamp = None
        self.quality_acceleration = 0.0
        self.quality_confidence = 0.0
        self.quality_reasons = ['WAITING_FOR_DATA']
        response.success, response.message = True,'Reset acknowledged; awaiting fresh VIO. Re-arm separately.'
        return response

    def watchdog(self):
        now = self.now()
        if self.recovery.expired(now):
            self.latched, self.reason = True, 'RECOVERY_TIMEOUT'
        if self.recovery.active and not all(0 <= now-t <= self.p('max_age') for t in self.image_at):
            self.recovery.previous = self.recovery.stable_since = None
        ages = [now-t for t in [self.last_good]+self.image_at+[self.imu_at]]
        fresh = all(0 <= age <= self.p('max_age') for age in ages)
        within_grace = all(0 <= age <= self.p('invalid_age') for age in ages)
        healthy = not self.latched and not self.recovery.active and fresh
        if healthy:
            health = 'DEGRADED' if self.quality_reasons else 'VALID'
        elif not self.latched and not self.recovery.active and within_grace:
            # A bounded sensor/output pause is recoverable. Consumers pause
            # translation while retaining their mission and resume on VALID.
            health = 'DEGRADED'
        else:
            health = 'INVALID'
        self.health.publish(String(data=health))
        self.diagnostics.publish(String(data=json.dumps(dict(
            state='LATCHED' if self.latched else ('RECOVERING' if self.recovery.active else health),
            reason=self.reason, reset_counter=self.reset_count,
            odom_age=None if not math.isfinite(self.last_good) else round(ages[0],4),
            cam0_age=None if not math.isfinite(self.image_at[0]) else round(ages[1],4),
            cam1_age=None if not math.isfinite(self.image_at[1]) else round(ages[2],4),
            imu_age=None if not math.isfinite(self.imu_at) else round(ages[3],4),
            quality='GOOD' if health == 'VALID' else ('DEGRADED' if health == 'DEGRADED' else 'BAD'),
            confidence=round(self.quality_confidence,3),
            feature_count=None if any(v is None for v in self.feature_counts) else min(self.feature_counts),
            position_covariance=None if not math.isfinite(self.position_covariance) else round(self.position_covariance,6),
            orientation_covariance=None if not math.isfinite(self.orientation_covariance) else round(self.orientation_covariance,6),
            velocity_covariance=None if not math.isfinite(self.velocity_covariance) else round(self.velocity_covariance,6),
            covariance_ratio=None if not math.isfinite(self.covariance_ratio) else round(self.covariance_ratio,3),
            angular_rate=round(self.imu_angular_rate,3),
            acceleration=round(self.imu_acceleration,3),
            velocity_change_rate=round(self.quality_acceleration,3),
            quality_reasons=self.quality_reasons))))

    def reject(self, reason):
        self.reason = reason
        if self.recovery.active:
            self.recovery.previous = self.recovery.stable_since = None

    def callback(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        now = self.now()
        if self.latched: return
        if not 0 <= now-stamp <= self.p('max_age'):
            self.reject('ODOMETRY_STALE'); return
        if not all(0<=now-t<=self.p('max_age') for t in self.image_at):
            self.reject('IMAGES_STALE'); return
        if msg.header.frame_id != self.p('expected_world') or msg.child_frame_id != self.p('expected_imu'):
            self.reject('FRAME_MISMATCH'); return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            if stamp < self.last_stamp: self.latched, self.reason = True, 'STAMP_REGRESSION'
            return
        p,q,v,w = msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear,msg.twist.twist.angular
        try:
            result = convert([p.x,p.y,p.z],[q.w,q.x,q.y,q.z],np.array([v.x,v.y,v.z]),np.array([w.x,w.y,w.z]),
                             msg.pose.covariance,msg.twist.covariance,self.extrinsic)
        except ValueError:
            self.reject('INVALID_COVARIANCE_OR_POSE'); return
        if not all(np.isfinite(x).all() for x in result):
            self.reject('NONFINITE_DATA'); return
        pos,quat,vel,omega,pv,ov,vv = result
        if np.linalg.norm(vel)>self.p('max_speed'):
            self.reject('EXCESSIVE_SPEED'); return
        if max(pv)>self.p('max_position_variance') or max(ov)>self.p('max_orientation_variance') or max(vv)>self.p('max_velocity_variance'):
            self.reject('EXCESSIVE_VARIANCE'); return
        if (self.quality_velocity_stamp is None or
                stamp-self.quality_velocity_stamp >= float(self.p('quality_velocity_window'))):
            dt_quality = (None if self.quality_velocity_stamp is None else
                          stamp-self.quality_velocity_stamp)
            self.quality_acceleration = (
                0.0 if dt_quality is None or dt_quality <= 0 else
                float(np.linalg.norm(vel-self.quality_velocity))/dt_quality)
            self.quality_velocity = vel.copy()
            self.quality_velocity_stamp = stamp
        self.position_covariance = float(max(pv))
        self.orientation_covariance = float(max(ov))
        self.velocity_covariance = float(max(vv))
        self.covariance_ratio = normalized_covariance(
            pv, ov, vv, (self.p('max_position_variance'),
                        self.p('max_orientation_variance'),
                        self.p('max_velocity_variance')))
        feature_count = (None if any(v is None for v in self.feature_counts)
                         else min(self.feature_counts))
        self.quality_confidence, self.quality_reasons = quality_state(
            self.covariance_ratio, feature_count,
            int(self.p('feature_bad_count')), int(self.p('feature_good_count')),
            self.quality_acceleration, float(self.p('quality_acceleration_warn')),
            float(np.linalg.norm(omega)), float(self.p('quality_angular_rate_warn')))
        if self.last_stamp is not None and not self.recovery.active:
            dt = stamp-self.last_stamp
            angle_residual = unexplained_rotation(
                self.last_quat, quat, dt, self.last_omega, omega)
            position_jump = np.linalg.norm(pos-self.last_position)
            source_gap = dt > self.p('max_continuous_gap')
            if source_gap or position_jump>self.p('max_jump')+self.p('max_speed')*dt or angle_residual>math.radians(self.p('recovery_max_angle_deg')):
                self.recovery.begin(now,self.last_stamp,self.last_position,self.last_quat,
                                    self.last_velocity,self.last_omega,source_gap=source_gap)
                self.reason = 'DATA_GAP' if source_gap else ('ORIENTATION_DISCONTINUITY' if angle_residual>math.radians(self.p('recovery_max_angle_deg')) else 'POSITION_DISCONTINUITY')
                self.notify_reset(self.reason)
                self.health.publish(String(data='INVALID'))
                self.get_logger().warn(f'VIO quarantine: {self.reason}; dt={dt:.4f}s jump={position_jump:.3f}m')
        if self.recovery.active:
            if not self.recovery.accept(now,stamp,pos,quat,vel,omega): return
            # Explicitly inform EKF2 of the accepted discontinuity. Its reset
            # deltas must be handled by the controller; never conceal a shift.
            self.reset_count = (self.reset_count+1)%256
            self.recovery = self.new_recovery()
            self.reason = 'RECOVERED_WITH_RESET'
            self.notify_reset(self.reason)
            self.get_logger().info('VIO stable again; publishing EV reset_counter')
        out = VehicleOdometry()
        # Hardware uses XRCE clock conversion. Algorithm SITL uses /clock on
        # both sides via the PX4-side RM27_SIM_CLOCK patch; do not offset twice.
        out.timestamp, out.timestamp_sample = int(now*1e6),int(stamp*1e6)
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out.position,out.q,out.velocity,out.angular_velocity = pos.tolist(),quat.tolist(),vel.tolist(),omega.tolist()
        out.position_variance,out.orientation_variance,out.velocity_variance = pv.tolist(),ov.tolist(),vv.tolist()
        out.reset_counter,out.quality = self.reset_count,100
        self.pub.publish(out)
        self.publish_vio_odom(msg,stamp,pos,quat,vel,omega,pv,ov,vv)
        self.last_stamp,self.last_position,self.last_good = stamp,pos,stamp
        self.last_quat,self.last_velocity,self.last_omega = quat,vel,omega
        self.reason = 'OK'


def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
