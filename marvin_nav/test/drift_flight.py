#!/usr/bin/env python3
"""Drift-/Konsistenztest in der Sim (von Hand starten, nicht CI).

Fliegt direkt ueber MAVROS (ohne Planer): Takeoff, Drehungen auf der Stelle,
Quadrat, zurueck zum Start, landen. Loggt dabei gegen Gazebo-Ground-Truth:
  px4   = /mavros/local_position/pose   (was PX4 glaubt)
  lio   = /Odometry                     (FAST-LIO roh, camera_init)
  map   = TF map->base_link             (relokalisiert, Sicht von Planer/Pursuit)
  corr  = TF map->camera_init           (Korrektur der Relokalisierung)

Voraussetzung: Sim laeuft, plus Ground-Truth-Bridge:
  ros2 run ros_gz_bridge parameter_bridge \\
    /world/scale3/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
    --ros-args -r /world/scale3/dynamic_pose/info:=/gt_poses

Aufruf: python3 drift_flight.py [csv_out] [side_m | "x,y;x,y;..."]   (CSV wird laufend geschrieben)
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
import tf2_ros

MODEL = 'marvin_drohne_alles_0'
HEADER = ['t', 'phase', 'gt_x', 'gt_y', 'gt_z', 'gt_yaw', 'px4_x', 'px4_y', 'px4_z', 'px4_yaw',
          'lio_x', 'lio_y', 'lio_z', 'lio_yaw', 'map_x', 'map_y', 'map_z', 'map_yaw',
          'corr_x', 'corr_y', 'corr_z', 'corr_yaw']
ALT = 1.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DriftFlight(Node):
    def __init__(self):
        super().__init__('drift_flight', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.px4 = self.lio = self.gt = None
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda m: setattr(self, 'px4', m.pose), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry',
                                 lambda m: setattr(self, 'lio', m.pose.pose), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/gt_poses', self.gt_cb, qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arm = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode = self.create_client(SetMode, '/mavros/set_mode')
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.sp = None  # (x, y, z, yaw) im PX4-Frame
        self.csv = csv.writer(open(sys.argv[1] if len(sys.argv) > 1 else 'drift.csv', 'w', newline='',
                                   buffering=1))
        self.csv.writerow(HEADER)
        self.phase = 'init'
        self.create_timer(0.05, self.stream)
        self.create_timer(0.2, self.log)

    def gt_cb(self, msg):
        # ros_gz_bridge verliert bei Pose_V die Namen -> Modell ist Eintrag 0
        named = [t for t in msg.transforms if t.child_frame_id == MODEL]
        self.gt = (named or msg.transforms)[0].transform

    def stream(self):
        if self.sp is None:
            return
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.sp[:3]
        m.pose.orientation.z, m.pose.orientation.w = math.sin(self.sp[3] / 2), math.cos(self.sp[3] / 2)
        self.sp_pub.publish(m)

    def lookup(self, parent, child):
        try:
            return self.tf.lookup_transform(parent, child, rclpy.time.Time()).transform
        except Exception:  # noqa: B902
            return None

    def log(self):
        if self.px4 is None or self.gt is None:
            return
        g, p = self.gt, self.px4
        row = [self.get_clock().now().nanoseconds * 1e-9, self.phase,
               g.translation.x, g.translation.y, g.translation.z, yaw_of(g.rotation),
               p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)]
        lio = self.lio
        row += ([lio.position.x, lio.position.y, lio.position.z, yaw_of(lio.orientation)]
                if lio else [math.nan] * 4)
        for tf in (self.lookup('map', 'base_link'), self.lookup('map', 'camera_init')):
            row += ([tf.translation.x, tf.translation.y, tf.translation.z, yaw_of(tf.rotation)]
                    if tf else [math.nan] * 4)
        self.csv.writerow(row)

    # --- Ablauf ---
    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, req):
        client.wait_for_service(timeout_sec=10)
        fut = client.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result()

    def goto(self, x, y, z, yaw, phase, settle=2.0, timeout=40.0):
        self.phase = phase
        self.sp = (x, y, z, yaw)
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.spin_for(0.2)
            p = self.px4.position
            dyaw = abs((yaw_of(self.px4.orientation) - yaw + math.pi) % (2 * math.pi) - math.pi)
            if math.dist((p.x, p.y, p.z), (x, y, z)) < 0.2 and dyaw < 0.1:
                break
        else:
            self.get_logger().warn(f'{phase}: Timeout')
        self.get_logger().info(f'{phase} erreicht')
        self.spin_for(settle)

    def run(self, side, route=None):
        self.get_logger().info('warte auf PX4-Pose + Ground Truth ...')
        while self.px4 is None or self.gt is None:
            self.spin_for(0.5)
        p0 = self.px4.position
        x0, y0, yaw0 = p0.x, p0.y, yaw_of(self.px4.orientation)
        self.sp = (x0, y0, p0.z, yaw0)
        self.spin_for(2.0)  # Setpoint-Stream muss vor OFFBOARD laufen
        self.get_logger().info(f'OFFBOARD: {self.call(self.mode, SetMode.Request(custom_mode="OFFBOARD")).mode_sent}')
        for _ in range(20):  # PX4 verweigert kurz nach dem Start gern (Heading/EKF noch nicht stabil)
            if self.call(self.arm, CommandBool.Request(value=True)).success:
                break
            self.spin_for(3.0)
        else:
            raise RuntimeError('Armen verweigert — PX4-Log pruefen')
        self.get_logger().info('armed')

        self.goto(x0, y0, ALT, yaw0, 'takeoff', settle=5)
        self.goto(x0, y0, ALT, yaw0, 'hover0', settle=10)
        if route:
            # Route relativ zum Start (PX4-Frame), Blick jeweils in Flugrichtung
            px, py = x0, y0
            for i, (dx, dy) in enumerate(route):
                x, y = x0 + dx, y0 + dy
                yaw = math.atan2(y - py, x - px) if math.hypot(x - px, y - py) > 0.1 else yaw0
                self.goto(px, py, ALT, yaw, f'turn{i}', settle=0.5)
                self.goto(x, y, ALT, yaw, f'wp{i}', settle=1.0, timeout=90.0)
                px, py = x, y
            self.goto(x0, y0, ALT, yaw0, 'hover_end', settle=15)
            self.phase = 'land'
            self.call(self.mode, SetMode.Request(custom_mode='AUTO.LAND'))
            self.spin_for(15)
            return
        for k in range(1, 9):  # 2 volle Drehungen in 90°-Schritten
            self.goto(x0, y0, ALT, yaw0 + k * math.pi / 2, f'yaw{k}', settle=1)
        self.goto(x0, y0, ALT, yaw0, 'hover1', settle=10)
        c, s = math.cos(yaw0), math.sin(yaw0)
        for i, (dx, dy) in enumerate([(side, 0), (side, side), (0, side), (0, 0)]):
            self.goto(x0 + c * dx - s * dy, y0 + s * dx + c * dy, ALT, yaw0, f'square{i}')
        self.goto(x0, y0, ALT, yaw0, 'hover_end', settle=15)
        self.phase = 'land'
        self.call(self.mode, SetMode.Request(custom_mode='AUTO.LAND'))
        self.spin_for(15)


def main():
    # Argument 2: Quadrat-Seitenlaenge [m] oder Route "x,y;x,y;..." relativ zum Start
    arg = sys.argv[2] if len(sys.argv) > 2 else '5.0'
    route = [tuple(map(float, wp.split(','))) for wp in arg.split(';')] if ';' in arg else None
    rclpy.init()
    DriftFlight().run(0.0 if route else float(arg), route)


if __name__ == '__main__':
    main()
