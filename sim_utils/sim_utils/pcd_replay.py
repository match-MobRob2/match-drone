#!/usr/bin/env python3
import rclpy, time, glob, os
import numpy as np
import open3d as o3d
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs_py.point_cloud2 import create_cloud
from geometry_msgs.msg import Vector3, Quaternion

class PCDReplay(Node):
    def __init__(self):
        super().__init__('pcd_replay')
        self.declare_parameter('pcd_dir', '')
        self.declare_parameter('imu_csv', '')
        self.declare_parameter('frame_id', 'os_lidar')
        self.declare_parameter('rate_hz', 10.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.rate = float(self.get_parameter('rate_hz').value)
        pcd_dir = self.get_parameter('pcd_dir').value
        imu_csv = self.get_parameter('imu_csv').value

        # Publisher
        self.pub_lidar = self.create_publisher(PointCloud2, '/scan_points_fixed', 10)
        self.pub_imu   = self.create_publisher(Imu, '/livox/imu_fixed', 50)

        # Gather files
        self.files = sorted(glob.glob(os.path.join(pcd_dir, '*.pcd')))
        if not self.files:
            self.get_logger().error(f'No PCD files in {pcd_dir}')
        self.idx = 0

        # Load IMU if given
        self.imu_data = None
        if imu_csv and os.path.exists(imu_csv):
            self.imu_data = pd.read_csv(imu_csv)
            self.imu_idx = 0

        self.timer = self.create_timer(1.0/self.rate, self.tick)

    def tick(self):
        if self.idx >= len(self.files):
            self.get_logger().info('Finished replay.')
            rclpy.shutdown()
            return
        fname = self.files[self.idx]
        cloud = o3d.io.read_point_cloud(fname)
        pts = np.asarray(cloud.points, dtype=np.float32)
        if pts.shape[0]==0:
            self.idx += 1
            return
        # Build PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        msg = create_cloud(header, fields, pts)
        self.pub_lidar.publish(msg)

        # Publish IMU if available
        if self.imu_data is not None and self.imu_idx < len(self.imu_data):
            row = self.imu_data.iloc[self.imu_idx]
            imu = Imu()
            imu.header = header
            imu.linear_acceleration = Vector3(x=row['ax'], y=row['ay'], z=row['az'])
            imu.angular_velocity = Vector3(x=row['gx'], y=row['gy'], z=row['gz'])
            imu.orientation = Quaternion(x=0.0,y=0.0,z=0.0,w=1.0)  # if not provided
            self.pub_imu.publish(imu)
            self.imu_idx += 1

        self.idx += 1

def main():
    rclpy.init()
    node = PCDReplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
