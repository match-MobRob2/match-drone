import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

class PCFrameFix(Node):
    def __init__(self):
        super().__init__('pc_frame_fix')
        q = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Set the desired frame that exists in your TF tree:
        self.out_frame = self.declare_parameter('out_frame', 'my_drone/livox_frame').value

        self.sub = self.create_subscription(PointCloud2, '/gz_scan/points', self.cb, q)
        self.pub = self.create_publisher(PointCloud2, '/scan_points_fixed', q)

    def cb(self, msg: PointCloud2):
        msg.header.frame_id = self.out_frame
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PCFrameFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
