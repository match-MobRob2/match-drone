#!/usr/bin/env python3
"""
Projects LiDAR points onto a camera image to produce:
  - a colorized XYZRGB point cloud
  - a depth-map image
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
from cv_bridge import CvBridge

import message_filters
import tf2_ros


class LidarCameraFusion(Node):
    def __init__(self):
        super().__init__('lidar_camera_fusion')

        # --- parameters ---
        self.declare_parameter('lidar_topic', '/rgl_lidar')
        self.declare_parameter('image_topic', '/marvin_drohne_alles/front_rgb/image')
        self.declare_parameter('camera_info_topic', '/marvin_drohne_alles/front_rgb/camera_info')
        self.declare_parameter('colored_cloud_topic', '/lidar_camera/colored_cloud')
        self.declare_parameter('depth_image_topic', '/lidar_camera/depth_image')
        self.declare_parameter('max_sync_slop', 0.1)  # seconds

        lidar_topic = self.get_parameter('lidar_topic').value
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        colored_topic = self.get_parameter('colored_cloud_topic').value
        depth_topic = self.get_parameter('depth_image_topic').value
        sync_slop = self.get_parameter('max_sync_slop').value

        # --- QoS: sensor data often uses BEST_EFFORT ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # --- TF2 ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- synchronized subscribers ---
        self.sub_cloud = message_filters.Subscriber(self, PointCloud2, lidar_topic, qos_profile=sensor_qos)
        self.sub_image = message_filters.Subscriber(self, Image, image_topic, qos_profile=sensor_qos)
        self.sub_info = message_filters.Subscriber(self, CameraInfo, info_topic, qos_profile=sensor_qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_cloud, self.sub_image, self.sub_info],
            queue_size=10,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.callback)

        # --- publishers ---
        self.pub_cloud = self.create_publisher(PointCloud2, colored_topic, 10)
        self.pub_depth = self.create_publisher(Image, depth_topic, 10)

        self.bridge = CvBridge()

        self.get_logger().info(
            f'LidarCameraFusion: lidar={lidar_topic}  image={image_topic}  '
            f'info={info_topic}'
        )

    # --------------------------------------------------------------------- #
    def callback(self, cloud_msg: PointCloud2, image_msg: Image, info_msg: CameraInfo):
        # --- look up transform lidar_frame -> camera_frame ---
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                info_msg.header.frame_id,   # target: camera
                cloud_msg.header.frame_id,   # source: lidar
                rclpy.time.Time(),           # latest available
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        # --- build 4x4 transform matrix ---
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T = self._quat_translation_to_matrix(q.x, q.y, q.z, q.w, t.x, t.y, t.z)

        # --- read lidar points (x, y, z) ---
        points_xyz = self._read_xyz(cloud_msg)
        if points_xyz.shape[0] == 0:
            return

        # --- transform into camera frame ---
        ones = np.ones((points_xyz.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack([points_xyz, ones])                # (N, 4)
        pts_cam = (T @ pts_h.T).T[:, :3]                     # (N, 3)

        # --- filter: only points in front of camera (z > 0) ---
        mask_front = pts_cam[:, 2] > 0.0
        pts_cam = pts_cam[mask_front]
        pts_lidar = points_xyz[mask_front]
        if pts_cam.shape[0] == 0:
            return

        # --- camera intrinsics ---
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # --- project to pixel coordinates ---
        u = (fx * pts_cam[:, 0] / pts_cam[:, 2] + cx).astype(np.int32)
        v = (fy * pts_cam[:, 1] / pts_cam[:, 2] + cy).astype(np.int32)

        # --- decode image ---
        img = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        # --- filter: only points that fall inside the image ---
        mask_in = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u = u[mask_in]
        v = v[mask_in]
        pts_cam = pts_cam[mask_in]
        pts_lidar = pts_lidar[mask_in]
        if u.shape[0] == 0:
            return

        # --- sample colors ---
        bgr = img[v, u]  # (N, 3)

        # --- publish colored point cloud (in original lidar frame) ---
        self._publish_colored_cloud(cloud_msg.header, pts_lidar, bgr)

        # --- publish depth image ---
        self._publish_depth_image(image_msg.header, h, w, u, v, pts_cam[:, 2])

    # --------------------------------------------------------------------- #
    def _read_xyz(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Read x, y, z fields from a PointCloud2 message."""
        from sensor_msgs_py import point_cloud2 as pc2
        gen = list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if len(gen) == 0:
            return np.empty((0, 3), dtype=np.float32)
        structured = np.array(gen)
        return np.column_stack([structured['x'], structured['y'], structured['z']]).astype(np.float32)

    # --------------------------------------------------------------------- #
    def _publish_colored_cloud(self, header, xyz: np.ndarray, bgr: np.ndarray):
        """Publish XYZRGB PointCloud2."""
        n = xyz.shape[0]
        # pack RGB into a single float32 (ROS convention)
        rgb_uint32 = (bgr[:, 2].astype(np.uint32) << 16 |
                       bgr[:, 1].astype(np.uint32) << 8 |
                       bgr[:, 0].astype(np.uint32))
        rgb_float = rgb_uint32.view(np.float32)

        data = np.zeros((n, 4), dtype=np.float32)
        data[:, :3] = xyz.astype(np.float32)
        data[:, 3] = rgb_float

        msg = PointCloud2()
        msg.header = header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = np.ascontiguousarray(data).tobytes()

        self.pub_cloud.publish(msg)

    # --------------------------------------------------------------------- #
    def _publish_depth_image(self, header, h: int, w: int,
                             u: np.ndarray, v: np.ndarray, depth: np.ndarray):
        """Publish a 32FC1 depth image (meters)."""
        depth_img = np.zeros((h, w), dtype=np.float32)
        depth_img[v, u] = depth.astype(np.float32)

        msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='32FC1')
        msg.header = header
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_depth.publish(msg)

    # --------------------------------------------------------------------- #
    @staticmethod
    def _quat_translation_to_matrix(qx, qy, qz, qw, tx, ty, tz) -> np.ndarray:
        """Quaternion + translation -> 4x4 homogeneous matrix."""
        T = np.eye(4, dtype=np.float64)
        # rotation
        T[0, 0] = 1 - 2 * (qy*qy + qz*qz)
        T[0, 1] = 2 * (qx*qy - qz*qw)
        T[0, 2] = 2 * (qx*qz + qy*qw)
        T[1, 0] = 2 * (qx*qy + qz*qw)
        T[1, 1] = 1 - 2 * (qx*qx + qz*qz)
        T[1, 2] = 2 * (qy*qz - qx*qw)
        T[2, 0] = 2 * (qx*qz - qy*qw)
        T[2, 1] = 2 * (qy*qz + qx*qw)
        T[2, 2] = 1 - 2 * (qx*qx + qy*qy)
        # translation
        T[0, 3] = tx
        T[1, 3] = ty
        T[2, 3] = tz
        return T


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
