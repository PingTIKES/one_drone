"""Explicit driver-to-algorithm topic adapters preserving messages and stamps."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


class SensorRelay(Node):
    def __init__(self):
        super().__init__('sensor_relay')
        self.pubs=[]
        for key,typ,dest in [('cam0',Image,'cam0/image_raw'),('cam1',Image,'cam1/image_raw'),('imu',Imu,'imu0')]:
            self.declare_parameter(key,'')
            source=self.get_parameter(key).value
            if not source:raise ValueError('missing driver topic '+key)
            # OpenVINS stereo subscribers request Reliable QoS; the driver may use BestEffort.
            pub=self.create_publisher(typ,dest,5)
            self.pubs.append(pub)
            self.create_subscription(typ,source,pub.publish,qos_profile_sensor_data)


def main(args=None):
    rclpy.init(args=args)
    node=SensorRelay()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
