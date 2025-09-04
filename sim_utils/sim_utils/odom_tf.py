#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.br = TransformBroadcaster(self)
        # Subscribe to FAST-LIO2 odometry topic
        self.sub = self.create_subscription(Odometry, '/Odometry', self.cb, 10)

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomTF()
    rclpy.spin(node)
    rclpy.shutdown()
