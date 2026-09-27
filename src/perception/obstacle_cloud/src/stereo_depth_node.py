"""Convert front depth images to body-FLU obstacle points for one drone.

The same node handles synthetic stereo depth and calibrated hardware depth.
It preserves image timestamps and filters range, subsampling, and the optional
simulated x500 self mask before publishing /uav1/obstacles in base_link.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from obstacle_cloud.depth_geometry import decode_depth
from obstacle_cloud.self_mask import x500_external_mask

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        # ---- 参数 ----
        self.declare_parameter('uav_id', 1)        # frame_id = base_link
        # 深度相机内参（默认 = worlds/models/d435i/model.sdf 的 depth 传感器：
        # 640x480, hfov 1.5184 rad → fx=fy≈337.2, 主点取中心）
        self.declare_parameter('fx', 337.2)
        self.declare_parameter('fy', 337.2)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)
        # Gazebo base_link 已经是 FLU，不是 PX4 的 NED/FRD。
        # 挂点 (0.17,0,-0.06) + 左红外/深度镜头 (0,0.025,0)。
        self.declare_parameter('cam_xyz', [0.17, 0.025, -0.06])
        self.declare_parameter('cam_rotation', [0., 0., 1., -1., 0., 0., 0., -1., 0.])
        self.declare_parameter('depth_scale', .001)
        self.declare_parameter('require_camera_info', False)
        self.declare_parameter('preserve_stamp', True)
        self.declare_parameter('step', 2)              # 像素抽稀步长（2→320x240）
        self.declare_parameter('frame_decimation', 3)  # 每 k 帧处理 1 帧（15→5Hz）
        self.declare_parameter('min_range', 0.3)       # 盲区/噪声截断 m
        self.declare_parameter('max_range', 8.0)       # 远界 m
        self.declare_parameter('self_mask_model', 'none')  # 仅仿真 x500 可选；真机须实测机架

        uid = int(self.get_parameter('uav_id').value)
        self.frame_id = 'base_link'
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.cam_xyz = [float(v) for v in self.get_parameter('cam_xyz').value]
        self.rotation = np.array(self.get_parameter('cam_rotation').value).reshape(3,3)
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(self.rotation),1):
            raise ValueError('cam_rotation must be a proper optical-to-body rotation')
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.require_info = bool(self.get_parameter('require_camera_info').value)
        self.preserve_stamp = bool(self.get_parameter('preserve_stamp').value)
        self.info_shape = None
        self.last_info_warning = -float('inf')
        self.step = int(self.get_parameter('step').value)
        self.decim = int(self.get_parameter('frame_decimation').value)
        self.min_r = float(self.get_parameter('min_range').value)
        self.max_r = float(self.get_parameter('max_range').value)
        self.self_mask_model = str(self.get_parameter('self_mask_model').value)
        if self.self_mask_model not in ('none', 'x500'):
            raise ValueError('self_mask_model must be none or x500')

        self.create_subscription(Image, 'd435i/depth/image_raw', self._cb_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, 'd435i/depth/camera_info', self._cb_info, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'obstacles', 5)

        self._count = 0
        self._depth_count_window = 0
        self._cloud_count_window = 0
        self._self_points_window = 0
        self._candidate_points_window = 0
        self._last_empty_warning = -float('inf')
        self._grid_cache = {}  # (h, w) -> (v, u) 抽稀后的像素网格
        self.create_timer(5., self._report_status)
        self.get_logger().info(
            f'深度转点云就绪：订阅 ~d435i/depth/image_raw，发布 ~obstacles '
            f'(frame={self.frame_id}，step={self.step}，'
            f'每 {self.decim} 帧处理 1 帧)')

    def _report_status(self):
        if self.self_mask_model != 'none' and self._candidate_points_window:
            self.get_logger().info(
                f'5 秒内自体掩膜过滤 {self._self_points_window}/{self._candidate_points_window} 个深度点'
            )
        if not self._depth_count_window:
            self.get_logger().warn('5 秒内未收到深度图；检查深度话题及发布/订阅 QoS')
        elif not self._cloud_count_window:
            self.get_logger().warn(f'5 秒内收到 {self._depth_count_window} 帧深度图，但没有发布点云；检查深度解码、有效量程和节点日志')
        self._depth_count_window = 0
        self._cloud_count_window = 0
        self._self_points_window = 0
        self._candidate_points_window = 0

    def _cb_info(self, msg):
        # This node consumes rectified depth, hence P, not distorted-image K.
        fx, fy, cx, cy = msg.p[0], msg.p[5], msg.p[2], msg.p[6]
        if not np.all(np.isfinite([fx,fy,cx,cy])) or fx <= 0 or fy <= 0: return
        self.fx, self.fy, self.cx, self.cy = fx,fy,cx,cy
        self.info_shape = (msg.height,msg.width)

    def _cb_depth(self, msg: Image):
        self._count += 1
        self._depth_count_window += 1
        if self._count % self.decim != 0:
            return
        h, w, s = msg.height, msg.width, self.step
        if self.require_info and self.info_shape != (h,w):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_info_warning >= 5:
                self.get_logger().warn(f'等待匹配深度图 {w}x{h} 的 CameraInfo；当前 {self.info_shape}')
                self.last_info_warning = now
            return
        try:
            depth = decode_depth(msg.data,h,w,msg.step,msg.encoding,msg.is_bigendian,self.depth_scale)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return
        if (h, w) not in self._grid_cache:
            v, u = np.mgrid[0:h:s, 0:w:s]
            self._grid_cache[(h, w)] = (v.astype(np.float32),
                                        u.astype(np.float32))
        v, u = self._grid_cache[(h, w)]

        z = depth[::s, ::s]
        valid = np.isfinite(z) & (z > self.min_r) & (z < self.max_r)
        if not np.any(valid):
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_empty_warning >= 5:
                self.get_logger().warn('深度图已到达，但 0.3–8 m 内没有有效像素；检查双目纹理、视差和深度单位')
                self._last_empty_warning = now
            return
        z = z[valid]
        u = u[valid]
        v = v[valid]

        # 光学系反投影（z 前、x 右、y 下）
        x_o = (u - self.cx) * z / self.fx
        y_o = (v - self.cy) * z / self.fy
        # 光学系 → 机体 FLU（x 前、y 左、z 上）+ 安装平移
        cloud = np.stack([x_o, y_o, z], axis=1) @ self.rotation.T + self.cam_xyz
        if self.self_mask_model == 'x500':
            keep = x500_external_mask(cloud)
            self._candidate_points_window += len(cloud)
            self._self_points_window += int(len(cloud) - np.count_nonzero(keep))
            cloud = cloud[keep]
        if len(cloud) == 0:
            return
        # 深度图、地图与 TF 使用同一 ROS 时钟。
        hdr = Header()
        hdr.stamp = msg.header.stamp if self.preserve_stamp else self.get_clock().now().to_msg()
        hdr.frame_id = self.frame_id
        out = point_cloud2.create_cloud_xyz32(hdr, cloud)
        self.pub.publish(out)
        self._cloud_count_window += 1


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
