"""Software depth from synchronized raw IR images; shared sim/real path."""
import cv2
import copy
import numpy as np
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from uav_perception.stereo_matcher import StereoMatcher


class SoftwareStereo(Node):
    def __init__(self):
        super().__init__('software_stereo')
        for k,v in dict(config='',t_body_imu=np.eye(4).ravel().tolist(),rate=5.,sync_slop=.003,
                        texture_std_min=1.0).items(): self.declare_parameter(k,v)
        p = lambda k:self.get_parameter(k).value
        self.matcher = StereoMatcher(p('config'),np.array(p('t_body_imu')).reshape(4,4),
                                     texture_std_min=p('texture_std_min'))
        cv2.setNumThreads(2)
        self.period,self.last = 1/p('rate'),-float('inf')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image,'d435i/depth/image_raw',qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo,'d435i/depth/camera_info',qos_profile_sensor_data)
        self.subs = [message_filters.Subscriber(self,Image,f'cam{i}/image_raw',qos_profile=qos_profile_sensor_data) for i in range(2)]
        self.image_counts = [0, 0]
        self.pair_count = 0
        self.depth_count = 0
        for i, sub in enumerate(self.subs):
            sub.registerCallback(lambda _msg, index=i: self._count_image(index))
        self.sync = message_filters.ApproximateTimeSynchronizer(self.subs,5,p('sync_slop'))
        self.sync.registerCallback(self.callback)
        self.create_timer(5., self.report_status)

    def _count_image(self, index):
        self.image_counts[index] += 1

    def report_status(self):
        if not self.image_counts[0] or not self.image_counts[1]:
            self.get_logger().warn(f'等待双目图像：cam0={self.image_counts[0]}, cam1={self.image_counts[1]}')
        elif not self.pair_count:
            self.get_logger().warn('收到双目图像，但采集时间戳未落在同步窗口内；检查双目时钟/帧率')
        elif not self.depth_count:
            self.get_logger().warn('双目已同步，但软件深度尚未产出；检查图像编码、分辨率和节点日志')
        self.image_counts = [0, 0]
        self.pair_count = 0
        self.depth_count = 0

    def callback(self,left,right):
        self.pair_count += 1
        stamp = left.header.stamp.sec+left.header.stamp.nanosec*1e-9
        if stamp<self.last: self.last=-float('inf')
        if stamp-self.last < self.period: return
        self.last=stamp
        a,b = [self.bridge.imgmsg_to_cv2(m,'mono8') for m in (left,right)]
        depth = self.matcher.depth(a,b)
        self.depth_count += 1
        header = copy.deepcopy(left.header if self.matcher.order[0]=='cam0' else right.header)
        header.frame_id=self.get_namespace().strip('/')+'_stereo_depth_optical'
        out = self.bridge.cv2_to_imgmsg(depth,'32FC1')
        out.header=header
        self.pub.publish(out)
        info = CameraInfo()
        info.header=header
        info.width,info.height = self.matcher.size
        info.p=self.matcher.p.ravel().tolist()
        info.k=self.matcher.p[:3,:3].ravel().tolist()
        info.r=np.eye(3).ravel().tolist()
        info.distortion_model='plumb_bob'
        info.d=[0.]*5
        self.info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node=SoftwareStereo()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
