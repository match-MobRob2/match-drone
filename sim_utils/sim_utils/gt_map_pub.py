#!/usr/bin/env python3
import rclpy, open3d as o3d, numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class GTMapPublisher(Node):
    def __init__(self):
        super().__init__('gt_map_pub')
        self.declare_parameter('map_file', '')
        self.declare_parameter('frame_id', 'map')
        map_file = self.get_parameter('map_file').value
        frame_id = self.get_parameter('frame_id').value

        # Load once
        cloud = o3d.io.read_point_cloud(map_file)
        pts = np.asarray(cloud.points, dtype=np.float32)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PointCloud2, '/gt_cloud', qos)

        # Build latched message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        self.msg = create_cloud(header, fields, pts)

        # Publish periodically
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = GTMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
