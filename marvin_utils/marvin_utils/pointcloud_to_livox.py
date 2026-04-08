#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except ImportError:
    from livox_ros_driver.msg import CustomMsg, CustomPoint

import sensor_msgs_py.point_cloud2 as pc2


def clamp_u8(v: int) -> int:
    return 0 if v < 0 else (255 if v > 255 else v)


class PointCloudToLivox(Node):
    def __init__(self):
        super().__init__('pointcloud_to_livox')

        self.input_topic = self.declare_parameter('input_topic', '/rgl_lidar').value
        self.output_topic = self.declare_parameter('output_topic', '/livox/lidar').value

        # If empty -> use incoming frame_id (recommended unless you apply TF yourself)
        self.output_frame_id = self.declare_parameter('output_frame_id', '').value

        # intensity mapping
        self.intensity_mode = self.declare_parameter('intensity_mode', 'raw_0_255').value  # or "normalized_0_1"
        self.reflectivity_fallback = int(self.declare_parameter('reflectivity_fallback', 30).value)  # used if intensity==0

        # time mapping
        # If your RGL time_stamp is already in nanoseconds -> set to 1
        # If it's microseconds -> 1000, milliseconds -> 1_000_000
        self.time_scale_to_ns = int(self.declare_parameter('time_scale_to_ns', 1).value)

        # Fallback if time is always 0
        self.use_synth_offset = bool(self.declare_parameter('use_synth_offset_if_time_zero', True).value)
        self.scan_period_ns = int(self.declare_parameter('scan_period_ns', 100_000_000).value)  # 100ms default

        # line mapping from ray_idx
        self.num_lines = int(self.declare_parameter('num_lines', 4).value)  # common placeholder; tune if needed
        self.line_mode = self.declare_parameter('line_mode', 'mod').value   # "mod" or "lowbyte"

        # tag mapping
        self.tag_mode = self.declare_parameter('tag_mode', 'zero').value    # "zero" or "entity_lowbyte"

        # Do NOT drop points by default; dropping can trigger "too few points"
        self.skip_nans = bool(self.declare_parameter('skip_nans', False).value)

        self.lidar_id = int(self.declare_parameter('lidar_id', 0).value)

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(CustomMsg, self.output_topic, 10)

        self.get_logger().info(f"Sub: {self.input_topic}  Pub: {self.output_topic}")

    def cb(self, cloud_msg: PointCloud2):
        livox_msg = CustomMsg()
        livox_msg.header = Header()
        livox_msg.header.stamp = cloud_msg.header.stamp
        livox_msg.header.frame_id = "mid_360_frame"

        timebase_ns = int(cloud_msg.header.stamp.sec) * 1_000_000_000 + int(cloud_msg.header.stamp.nanosec)
        livox_msg.timebase = timebase_ns

        # IMPORTANT: According to your raw bytes, tag behaves like INT32, line/time like UINT32.
        # Keep these types here:
        gen = pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z", "intensity", "tag", "line", "time"),
            skip_nans=self.skip_nans
        )

        tmp = []
        time_nonzero = False
        first_time = None

        for x, y, z, intensity, entity_id, ray_idx, tstamp in gen:
            t = int(tstamp)
            if first_time is None:
                first_time = t
            if t != 0:
                time_nonzero = True
            tmp.append((float(x), float(y), float(z), float(intensity), int(entity_id), int(ray_idx), t))

        n = len(tmp)
        if n == 0:
            self.get_logger().warn("No points in input cloud after reading.")
            return

        # decide offset strategy
        use_synth = self.use_synth_offset and (not time_nonzero)
        step_ns = (self.scan_period_ns // max(1, n - 1)) if use_synth else 0

        points = []
        append = points.append

        for i, (x, y, z, intensity, entity_id, ray_idx, t) in enumerate(tmp):
            cp = CustomPoint()
            cp.x = x
            cp.y = y
            cp.z = z

            # reflectivity from intensity; if intensity is always 0 in RGL, fallback helps downstream
            if self.intensity_mode == "normalized_0_1":
                refl = int(intensity * 255.0)
            else:
                refl = int(intensity)
            if refl == 0:
                refl = self.reflectivity_fallback
            cp.reflectivity = clamp_u8(refl)

            # tag: either 0 (often safer) or low byte of entity id
            if self.tag_mode == "entity_lowbyte":
                cp.tag = int(entity_id) & 0xFF
            else:
                cp.tag = 0

            # line: ray_idx -> "line"
            if self.line_mode == "mod":
                cp.line = int(ray_idx % max(1, self.num_lines)) & 0xFF
            else:
                cp.line = int(ray_idx) & 0xFF

            # offset_time
            if use_synth:
                cp.offset_time = int(i * step_ns)
            else:
                # make it relative; Livox offset is relative to timebase
                cp.offset_time = int((t - (first_time or 0)) * self.time_scale_to_ns)

            append(cp)

        livox_msg.points = points
        livox_msg.point_num = len(points)
        livox_msg.lidar_id = self.lidar_id
        livox_msg.rsvd = [0, 0, 0]

        self.pub.publish(livox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLivox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
