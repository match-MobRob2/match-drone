#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import math
import numpy as np
from sensor_msgs.msg import PointField


class CloudNanFilter(Node):
    def __init__(self):
        super().__init__('cloud_nan_filter')

        self.declare_parameter('topic_in', '/marvin_drohne_alles/front_depth/image/points')
        self.declare_parameter('topic_out', '/front_depth/points_filtered')

        topic_in = self.get_parameter('topic_in').value
        topic_out = self.get_parameter('topic_out').value

        self.sub = self.create_subscription(
            PointCloud2,
            topic_in,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            topic_out,
            10
        )

        self.get_logger().info(f'Filtering NaNs: {topic_in} -> {topic_out}')

    def callback(self, msg: PointCloud2):

        min_dist2 = 0.25
        max_dist2 = 900.0
        target_points = 10_000

        try:
            points = pc2.read_points_numpy(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        except AttributeError:
            points = np.asarray(
                list(
                    pc2.read_points(
                        msg,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float32,
            )

        if points.size == 0:
            return

        r2 = np.einsum('ij,ij->i', points, points)
        mask = (r2 > min_dist2) & (r2 < max_dist2)
        filtered = points[mask]

        filtered_count = filtered.shape[0]
        if filtered_count == 0:
            return

        if filtered_count > target_points:
            stride = max(1, filtered_count // target_points)
            keep_mask = (np.arange(filtered_count) % stride) == 0
            filtered = filtered[keep_mask]
            if filtered.shape[0] > target_points:
                filtered = filtered[:target_points]

        if filtered.size == 0:
            return

        cloud_xyz = np.ascontiguousarray(filtered, dtype=np.float32)

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()

        out_msg = PointCloud2()
        out_msg.header = header
        out_msg.height = 1
        out_msg.width = cloud_xyz.shape[0]
        out_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out_msg.is_bigendian = False
        out_msg.point_step = 12
        out_msg.row_step = out_msg.point_step * out_msg.width
        out_msg.is_dense = True
        out_msg.data = cloud_xyz.tobytes()

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudNanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
