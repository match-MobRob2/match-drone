import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.time import Time
import numpy as np

def pose_to_T(p):
    # p.pose.pose
    q = p.pose.pose.orientation
    t = p.pose.pose.position
    # quaternion to rotation matrix
    x,y,z,w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.x, t.y, t.z]
    return T

def tfmsg_to_T(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    x,y,z,w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])
    T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.x, t.y, t.z]
    return T

def T_to_tf(T, parent, child, stamp):
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = T[:3,3]
    # rotation matrix to quaternion
    R = T[:3,:3]
    w = np.sqrt(1.0 + np.trace(R)) / 2.0
    x = (R[2,1] - R[1,2])/(4*w)
    y = (R[0,2] - R[2,0])/(4*w)
    z = (R[1,0] - R[0,1])/(4*w)
    tf.transform.rotation.w = w
    tf.transform.rotation.x = x
    tf.transform.rotation.y = y
    tf.transform.rotation.z = z
    return tf

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.declare_parameter('odom_lio_topic', '/Odometry')
        self.declare_parameter('body_frame', 'body')          # from /odom_lio child_frame_id
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('lidar_link', 'lidar_link')    # assume body == lidar_link
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('world_frame', 'camera_init')  # FAST-LIO global

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        topic = self.get_parameter('odom_lio_topic').value
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, topic, self.cb, qos)

        self.get_logger().info('odom_bridge up: computing camera_init->odom from /Odometry')

    def cb(self, odom_msg: Odometry):
        try:
            base_link = self.get_parameter('base_link').value
            lidar_link = self.get_parameter('lidar_link').value
            odom = self.get_parameter('odom_frame').value
            world = self.get_parameter('world_frame').value

            now = Time()  # latest
            stamp = rclpy.time.Time.from_msg(odom_msg.header.stamp)
            # T(odom->base_link)
            tf_ob = self.buffer.lookup_transform(odom, base_link, stamp)
            T_ob = tfmsg_to_T(tf_ob)
            # T(base_link->lidar_link)
            tf_bl_ll = self.buffer.lookup_transform(base_link, lidar_link, stamp)
            T_bl_ll = tfmsg_to_T(tf_bl_ll)
            # T(lidar_link->base_link)
            T_ll_bl = np.linalg.inv(T_bl_ll)

            # T(camera_init->body) from /odom_lio (body==lidar_link)
            T_cb = pose_to_T(odom_msg)

            # T(camera_init->odom) = T_cb * T_ll_bl * inv(T_ob)
            T_co = T_cb @ T_ll_bl @ np.linalg.inv(T_ob)

            tf = T_to_tf(T_co, world, odom, odom_msg.header.stamp)
            self.broadcaster.sendTransform(tf)
        except Exception as e:
            self.get_logger().warn(f'bridge step failed: {e}')

def main():
    rclpy.init()
    rclpy.spin(OdomBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
