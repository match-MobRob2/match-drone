#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
import math


class CloudNanFilter(Node):
    def __init__(self):
        super().__init__('cloud_nan_filter')

        self.declare_parameter('topic_in', '/match_drohne_alles/front_depth/image/points')
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
        points = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True            # WICHTIG: NaNs fliegen im C-Code raus
        )

        min_dist2 = 0.5 * 0.5
        max_dist2 = 30.0 * 30.0

        filtered = []
        for i, p in enumerate(points):
            if i % 5 != 0:
                continue  # subsample: nur jeden 5. Punkt behalten

            x, y, z = p
            r2 = x*x + y*y + z*z
            if min_dist2 < r2 < max_dist2:
                filtered.append((x, y, z))

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        out_msg = pc2.create_cloud_xyz32(header, filtered)
        self.pub.publish(out_msg)

        self.get_logger().info(f'Filtered pointcloud: {len(filtered)} points')


def main(args=None):
    rclpy.init(args=args)
    node = CloudNanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
