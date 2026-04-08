#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LocalPoseToTF(Node):
    def __init__(self):
        super().__init__('mavros_local_to_tf')
        # Use QoS with BEST_EFFORT reliability to match MAVROS publisher
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.cb_pose,
            qos
        )
        self.br = TransformBroadcaster(self)
        self.base_link_frame = 'base_link'
        self.map_frame = 'map'

    def cb_pose(self, msg: PoseStamped):
        t = TransformStamped()
        # MAVROS sends wall-clock timestamps while the rest of the
        # pipeline runs on sim_time → use the node clock (sim_time)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_link_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPoseToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
