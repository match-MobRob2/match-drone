#!/usr/bin/env python3
"""Mission fliegen und Lokalisierung gegen Gazebo-Wahrheit messen (von Hand, Sim, nicht CI).

    ros2 run ros_gz_bridge parameter_bridge /world/scale3/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
        --ros-args -r /world/scale3/dynamic_pose/info:=/gt_poses -p use_sim_time:=true &
    python3 -s mission_gt.py <mission> [karte]

TF map->base_link wird zum Stempel jeder Gazebo-Pose abgefragt (zeitgenau — `gz model -p`
braucht ~0.9 s und taeuschte im Flug Meter-Fehler vor). Gazebo -> map ueber `origin` aus
maps/<karte>/map.yaml; gilt, solange die Kartierung am Gazebo-Ursprung mit Blick +x begann.
z-Versatz ist konstant (map z=0 = Sensorhoehe beim Kartierstart).
"""
import collections, json, math, sys, time
import numpy as np, rclpy, tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration
from marvin_msgs.srv import Command
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
import os
import yaml
_o = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'maps',
                                      sys.argv[2] if len(sys.argv) > 2 else 'scale3', 'map.yaml')))['origin'] or {}
ORG = (_o.get('x', 0.0), _o.get('y', 0.0), math.radians(_o.get('yaw_deg', 0.0)))
def to_map(x, y, yaw):
    dx, dy = x - ORG[0], y - ORG[1]; c, s = math.cos(-ORG[2]), math.sin(-ORG[2])
    return c*dx - s*dy, s*dx + c*dy, yaw - ORG[2]
def yaw_x(q):
    f = R.from_quat(q).apply([1, 0, 0]); return math.atan2(f[1], f[0])
class N(Node):
    def __init__(s):
        super().__init__('mission_gt2', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        s.cli = s.create_client(Command, '/marvin/command'); s.status = None; s.gt = collections.deque(maxlen=200)
        s.create_subscription(String, '/marvin/status', lambda m: setattr(s, 'status', json.loads(m.data)), 10)
        s.create_subscription(TFMessage, '/gt_poses', s.gt_cb, qos_profile_sensor_data)
        s.buf = tf2_ros.Buffer(cache_time=Duration(seconds=30)); s.tl = tf2_ros.TransformListener(s.buf, s)
    def gt_cb(s, m):
        t = ([x for x in m.transforms if x.child_frame_id == 'marvin_drohne_alles_0'] or m.transforms)[0]
        q = t.transform.rotation
        s.gt.append((t.header.stamp, t.transform.translation, yaw_x([q.x, q.y, q.z, q.w])))
    def cmd(s, **r):
        s.cli.wait_for_service(); f = s.cli.call_async(Command.Request(request=json.dumps(r)))
        rclpy.spin_until_future_complete(s, f); print(r['cmd'], f.result().success, f.result().response[:120], flush=True)
        return f.result().success
rclpy.init(); n = N(); t0 = time.time()
while time.time() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.1)
print('GT-Posen empfangen:', len(n.gt))
n.cmd(cmd='start_mission', name=sys.argv[1])
while n.status is None or n.status['mission']['state'] not in ('running', 'paused', 'waiting'): rclpy.spin_once(n, timeout_sec=0.1)
rows, last_print = [], 0
while time.time() - t0 < 900:
    for _ in range(5): rclpy.spin_once(n, timeout_sec=0.05)
    m = n.status['mission']
    if m['state'] == 'waiting' and n.cmd(cmd='continue_mission'):
        n.spin_wait = time.time() + 2
        while time.time() < n.spin_wait: rclpy.spin_once(n, timeout_sec=0.1)
    if len(n.gt) > 20:
        stamp, p, gyaw = n.gt[-10]  # etwas alt, damit TF zu diesem Zeitpunkt sicher da ist
        try:
            t = n.buf.lookup_transform('map', 'base_link', rclpy.time.Time.from_msg(stamp)).transform
        except Exception as e:  # noqa
            continue
        gx, gy, gyaw_m = to_map(p.x, p.y, gyaw)
        tyaw = yaw_x([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        e = math.hypot(t.translation.x - gx, t.translation.y - gy)
        ey = math.degrees((tyaw - gyaw_m + math.pi) % (2 * math.pi) - math.pi)
        rows.append((e, abs(ey), t.translation.z - p.z))
        if time.time() - last_print > 4:
            last_print = time.time()
            print(f'[{time.time()-t0:4.0f}s] {m["state"]:8} wp {m["index"]+1}/{m["total"]} TF ({t.translation.x:6.2f},{t.translation.y:6.2f}) '
                  f'wahr ({gx:6.2f},{gy:6.2f}) Fehler {e:.2f} m / {ey:5.1f}°', flush=True)
    if m['state'] in ('done', 'failed', 'aborted'): break
a = np.array(rows)
print(f'ERGEBNIS {m["state"]} — {m["text"]}; {len(a)} Vergleiche; xy-Fehler median {np.median(a[:,0]):.2f} p95 {np.percentile(a[:,0],95):.2f} max {a[:,0].max():.2f} m; '
      f'Yaw median {np.median(a[:,1]):.1f} p95 {np.percentile(a[:,1],95):.1f}°; z-Versatz {np.median(a[:,2]):.2f}±{np.std(a[:,2]):.2f} m')
