#!/usr/bin/env python3
"""marvin_supervisor — Bedienzentrale hinter der Web-Oberflaeche.

Laeuft dauerhaft (bringup.launch.py) auf Sim- bzw. Drohnen-PC und
  - startet/stoppt den Nav-Stack (mapping.launch.py / localization.launch.py) als
    Prozessgruppe und raeumt beim Stoppen garantiert alles ab
  - verwaltet Karten unter maps_dir (Liste, Optimieren, Nullpunkt, Ausschnitte,
    Papierkorb) und erzeugt Vorschauen (preview.bin) fuer den Browser
  - relokalisiert per /initialpose
  - bewertet die Flugbereitschaft (Ampel)
  - liefert die Web-Oberflaeche + Kartenvorschauen per HTTP (0.0.0.0:http_port)

Schnittstelle (rosbridge):
  Service /marvin/command (marvin_msgs/srv/Command): JSON {"cmd": ..., ...}
  Topic   /marvin/status  (std_msgs/String, JSON, 2 Hz)
  Topic   /marvin/pose    (geometry_msgs/PoseStamped, map->base_link, 10 Hz)
"""
import datetime
import functools
import http.server
import json
import math
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import urllib.parse

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from marvin_msgs.srv import Command
from mavros_msgs.msg import EstimatorStatus, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import Log
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from std_msgs.msg import Bool, Float32, String
import tf2_ros
import yaml

# Prozesse, die ein Stack-Lauf hinterlassen kann. Beim Stoppen wird alles davon
# beendet — eine uebrig gebliebene Sensor-Bridge publiziert sonst /clock doppelt
# (Zeitspruenge, FAST-LIO "loop back", PX4 wird nie bereit).
# Nur eindeutige Pfade aus diesem Workspace/ROS — generische Namen wie "recorder"
# trafen fremde Prozesse (Battlesnake ladder_recorder.py)!
LEFTOVERS = ['px4_sitl_default/bin/px4', 'gz sim.*marvin', 'mavros/mavros_node', 'fast_lio/fastlio_mapping',
             'rviz2.*marvin_launch', 'marvin_nav/mapper_planner_node', 'marvin_nav/lidar_relocalization_node',
             'marvin_utils/pursuit', 'marvin_utils/odometry_to_drone', 'tf2_ros/static_transform_publisher',
             'marvin_utils/keyframe_recorder', 'ros_gz_bridge/parameter_bridge', 'ros_gz_sim/create',
             'robot_state_publisher/robot_state_publisher', 'marvin_utils/imu_timemachine', 'marvin_ui/fake_gcs',
             'livox_ros_driver2/livox_ros_driver2_node', 'realsense2_camera/realsense2_camera_node',
             'marvin_utils/light_controller', 'marvin_utils/light_watchdog', 'marvin_utils/recorder']


def _ws_root():
    # <ws>/install/marvin_ui/share/marvin_ui -> <ws>
    share = get_package_share_directory('marvin_ui')
    return os.path.abspath(os.path.join(share, '..', '..', '..', '..'))


def _read_pcd_xyz(path):
    with open(path, 'rb') as f:
        n, fields, sizes = 0, [], []
        while True:
            line = f.readline().decode()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('SIZE'):
                sizes = [int(s) for s in line.split()[1:]]
            elif line.startswith('POINTS'):
                n = int(line.split()[1])
            elif line.startswith('DATA'):
                if 'binary' not in line or 'compressed' in line:
                    raise ValueError('nur unkomprimierte binaere PCD')
                step = sum(sizes)
                raw = np.frombuffer(f.read(n * step), dtype=np.uint8).reshape(n, step)
                offs = np.cumsum([0] + sizes[:-1])
                cols = [raw[:, offs[fields.index(c)]:offs[fields.index(c)] + 4].copy().view(np.float32)[:, 0]
                        for c in 'xyz']
                return np.stack(cols, 1)


def _rot(q):
    """Rotationsmatrix aus Quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_angle(a, b):
    """Winkel [rad] zwischen zwei Quaternionen (x, y, z, w), normiert."""
    na, nb = math.sqrt(sum(v * v for v in a)), math.sqrt(sum(v * v for v in b))
    d = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    return 2 * math.acos(min(1.0, d))


def _start_pose(map_dir):
    """x, y, Yaw [rad] von Keyframe 0 (poses_opt.csv, map-Frame) = wo die Kartierung begann."""
    try:
        with open(os.path.join(map_dir, 'poses_opt.csv')) as f:
            f.readline()
            _, _, x, y, _, qx, qy, qz, qw = (float(v) for v in f.readline().split(',')[:9])
    except (OSError, ValueError):
        return 0.0, 0.0, 0.0
    return x, y, math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


class Supervisor(Node):
    def __init__(self):
        super().__init__('marvin_supervisor')
        self.sim = self.declare_parameter('sim', True).value
        self.maps_dir = self.declare_parameter('maps_dir', '').value or os.path.join(_ws_root(), 'maps')
        self.http_port = self.declare_parameter('http_port', 8088).value
        # Zusatz-Argumente fuer jeden Stack-Start, z.B. "world:=scale3 lookahead:=1.0"
        self.launch_extra = self.declare_parameter('launch_extra', '').value
        self.display = self.declare_parameter('display', ':0').value  # Sim-GUI (Gazebo/RViz)
        # Live-Bild fuer die Seite (web_video_server auf video_port, encodiert nur solange jemand zuschaut)
        self.camera_topic = self.declare_parameter('camera_topic', '').value or (
            '/marvin_drohne_alles/front_rgb/image' if self.sim else '/camera/camera/color/image_raw')
        self.video_port = self.declare_parameter('video_port', 8089).value
        # Python der View-Planning-venv (System-numpy/scipy inkompatibel)
        self.vp_python = self.declare_parameter('vp_python', '').value or os.path.join(
            _ws_root(), '.venv_view_planning', 'bin', 'python')
        # Live-Lidar im map-Frame (Voxel 10 cm), nur beim Fliegen auf einer Karte: View Planning
        # sieht damit auch Hindernisse, die nach der Kartenaufnahme dazukamen
        self.live_pts = np.empty((0, 3), np.float32)
        self.live_t = 0.0
        self.scan = None            # letzte View-Planung {map, name, ok, text} -> UI zeigt Vorschau
        self.vp_defaults = None     # config.toml der Planung (gecacht)
        os.makedirs(self.maps_dir, exist_ok=True)
        # Der Supervisor besitzt den Stack: nach einem Absturz/Neustart keinen verwaisten
        # Stack weiterlaufen lassen (unbekannter Zustand, doppelte Bridges)
        self._kill_leftovers()

        self.lock = threading.Lock()
        self.proc = None            # laufender Stack (Popen, eigene Prozessgruppe)
        self.aux = []               # Hilfsprozesse (Sim: fake_gcs)
        self.mode = 'idle'          # idle | mapping | localization | stopping | optimizing
        self.map_name = ''
        self.last_start = None      # (cmd, args) fuer reset
        self.busy = ''              # laufende Hintergrundaufgabe (Text fuer die UI)
        self.message = 'bereit'
        self.started_at = 0.0

        # Telemetrie fuer die Ampel
        self.mav_state = None
        self.mav_state_t = 0.0
        self.battery = None
        self.est = None
        self.est_t = 0.0
        self.odom_stamps = []   # Header-Stempel (ROS-Zeit) der letzten /Odometry
        self.sim_now = None     # letzte /clock (Sim); Alter/Raten in ROS-Zeit messen
        self.reloc_t = 0.0
        self.reloc_pose = None      # letzte akzeptierte Schaetzung map->camera_init (PoseStamped)
        self.inlier = None          # Anteil des Scans auf der Karte beim letzten Match
        self.vision_dead = False
        self.bad_since = None       # Flug-Watchdog: seit wann ist eine Ampel rot
        self.failsafe_t = 0.0       # letzte Landungs-Ausloesung
        self.hold = False           # Failsafe-Halt aktiv (pursuit haelt PX4-Position)

        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(BatteryState, '/mavros/battery', lambda m: setattr(self, 'battery', m),
                                 qos_profile_sensor_data)
        self.create_subscription(EstimatorStatus, '/mavros/estimator_status', self._on_est, 10)
        self.create_subscription(Odometry, '/Odometry', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/relocalization/pose', self._on_reloc, 10)
        self.create_subscription(Float32, '/relocalization/inlier_ratio',
                                 lambda m: setattr(self, 'inlier', m.data), 10)
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self.create_subscription(PointCloud2, '/cloud_registered', self._on_scan, 10)
        # Timer laufen bewusst auf Wanduhr (sonst stehen sie ohne Sim still); Alter und
        # Raten aber in ROS-Zeit, sonst taeuscht eine langsame Sim (RTF < 1) Fehler vor
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        # Leerer lokaler Pfad = Planer hat abgebrochen (pursuit haelt) -> Mission scheitert
        self.empty_path_t = 0.0
        self.create_subscription(Path, '/local_planned_path',
                                 lambda m: setattr(self, 'empty_path_t', time.time()) if not m.poses else None, 10)
        self.mission = None         # Status der laufenden Mission (dict) fuer die UI
        self.mission_ctl = None     # 'pause' | 'abort' | None
        self.status_pub = self.create_publisher(String, '/marvin/status', 10)
        self.hold_pub = self.create_publisher(Bool, '/marvin/hold', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/marvin/pose', 10)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)

        self.create_service(Command, '/marvin/command', self._on_command)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.1, self._publish_pose)
        self._start_http()
        self.get_logger().info(
            f'Supervisor bereit: http://0.0.0.0:{self.http_port}  Karten: {self.maps_dir}  sim={self.sim}')

    # ------------------------------------------------------------------ Telemetrie
    def _on_state(self, m):
        self.mav_state, self.mav_state_t = m, time.time()

    def _on_reloc(self, m):
        self.reloc_t, self.reloc_pose = time.time(), m

    def _reloc_check(self, now):
        """Relokalisierung ok = frischer Match + genug Scan auf der Karte + gesendete TF passt zur
        Schaetzung (faengt Fehler zwischen Schaetzung und TF ab, z.B. den Quaternion-Bug 2026-09-24)."""
        ra = now - self.reloc_t
        if ra > 1e6 or self.reloc_pose is None:
            return False, 'noch kein Match'
        if ra > 10.0:
            return False, f'letzter Match vor {ra:.0f} s'
        if self.inlier is not None and self.inlier < 0.6:
            return False, f'nur {100 * self.inlier:.0f} % des Scans auf der Karte'
        t, _ = self._tf('map', 'camera_init')
        if t is None:
            return False, 'TF map->camera_init fehlt'
        p, q = self.reloc_pose.pose.position, self.reloc_pose.pose.orientation
        tr, tq = t.transform.translation, t.transform.rotation
        dpos = math.dist((p.x, p.y, p.z), (tr.x, tr.y, tr.z))
        dang = _quat_angle((q.x, q.y, q.z, q.w), (tq.x, tq.y, tq.z, tq.w))
        if dpos > 0.3 or dang > math.radians(3):
            return False, f'TF weicht von Schaetzung ab ({dpos:.2f} m / {math.degrees(dang):.1f}°)'
        return True, f'{100 * self.inlier:.0f} % auf Karte' if self.inlier is not None else 'ok'

    def _on_est(self, m):
        self.est, self.est_t = m, time.time()

    def _on_clock(self, m):
        now = m.clock.sec + m.clock.nanosec * 1e-9
        # Sim neu gestartet -> Uhr springt zurueck. Alte Stempel liegen dann "in der Zukunft":
        # tf2 verwirft alle neuen TFs (TF_OLD_DATA) und Alterspruefungen werden negativ = "frisch".
        if self.sim_now is not None and now < self.sim_now - 1.0:
            self.get_logger().info('Sim-Uhr zurueckgesprungen — TF-Puffer und Stempel geleert')
            self.tf.clear()
            self.odom_stamps = []
        self.sim_now = now

    def _ros_now(self):
        if self.sim and self.sim_now is not None:
            return self.sim_now
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom_stamps = [s for s in self.odom_stamps if t - s < 2.0] + [t]

    def _on_scan(self, m):
        if self.mode != 'localization' or time.time() - self.live_t < 1.0:  # 1 Scan/s reicht
            return
        t, _ = self._tf('map', m.header.frame_id)  # camera_init, relokalisierungskorrigiert
        if t is None:
            return
        self.live_t = time.time()
        tr, q = t.transform.translation, t.transform.rotation
        p = read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        p = (p @ _rot((q.x, q.y, q.z, q.w)).T + (tr.x, tr.y, tr.z)).astype(np.float32)
        # ponytail: np.unique ueber alles je Scan, O(n log n) — reicht bis ~1e6 Voxel, sonst Hash-Set
        pts = np.vstack([self.live_pts, p])
        _, idx = np.unique(np.floor(pts / 0.1).astype(np.int64), axis=0, return_index=True)
        self.live_pts = pts[idx]

    def _on_log(self, m):
        if m.name.endswith('odometry_to_drone') and m.level >= 50:  # FATAL (Log.FATAL ist bytes)
            self.vision_dead = True

    def _tf(self, parent, child):
        try:
            t = self.tf.lookup_transform(parent, child, rclpy.time.Time())
            s = t.header.stamp
            return t, abs(self._ros_now() - (s.sec + s.nanosec * 1e-9))  # negativ = Stempel aus altem Lauf
        except Exception:  # noqa: B902 — tf2 wirft diverse Typen
            return None, math.inf

    def _publish_pose(self):
        t, age = self._tf('map', 'base_link')
        if t is None or age > 1.0:
            return
        m = PoseStamped()
        m.header = t.header
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
        m.pose.orientation = t.transform.rotation
        self.pose_pub.publish(m)

    def _checks(self):
        """Flugbereitschaft: Liste (Name, ok, Detail). Alles gruen = bereit."""
        now = time.time()
        running = self.mode in ('mapping', 'localization') and self.proc and self.proc.poll() is None
        c = [('Stack', bool(running), self.mode + (f' ({self.map_name})' if self.map_name else ''))]
        st = self.mav_state
        c.append(('MAVROS', bool(st and st.connected and now - self.mav_state_t < 3),
                  'verbunden' if st and st.connected else 'keine Verbindung'))
        # Rate in ROS-Zeit aus den Stempeln; alt = FAST-LIO steht
        fresh = self.odom_stamps and self._ros_now() - self.odom_stamps[-1] < 1.0
        hz = len(self.odom_stamps) / 2.0 if fresh else 0.0
        c.append(('FAST-LIO', hz >= 7.0, f'{hz:.0f} Hz'))
        _, age = self._tf('map', 'base_link')
        c.append(('Pose map', age < 0.5, 'aktuell' if age < 0.5 else 'fehlt/alt'))
        if self.mode == 'localization':
            c.append(('Relokalisierung', *self._reloc_check(now)))
        est = self.est
        # const_pos_mode bleibt am Boden gesetzt, obwohl Vision fusioniert -> kein Kriterium
        ok = bool(est and now - self.est_t < 3 and est.pos_horiz_rel_status_flag)
        c.append(('PX4 Position', ok, 'EKF ok' if ok else 'EKF ohne Position'))
        c.append(('Vision-Gate', not self.vision_dead, 'ok' if not self.vision_dead else 'FAST-LIO unplausibel — Neustart'))
        b = self.battery
        pct = b.percentage * 100 if b and b.percentage >= 0 else None
        c.append(('Akku', pct is not None and pct >= 30, f'{pct:.0f} %' if pct is not None else 'unbekannt'))
        nodes = self.get_node_names()
        c.append(('Planer', 'nav_node' in nodes, 'laeuft' if 'nav_node' in nodes else 'fehlt'))
        # ohne pursuit keine Setpoints -> PX4-Offboard-Verlust
        c.append(('Regler', 'pure_pursuit_tracker' in nodes, 'laeuft' if 'pure_pursuit_tracker' in nodes else 'fehlt'))
        return c

    # Vision weg -> keine Position mehr, halten unmoeglich -> sofort landen
    FS_LAND = {'FAST-LIO', 'Vision-Gate', 'PX4 Position', 'Regler'}
    # Karte/Planer weg -> PX4 hat noch Position -> pursuit haelt im PX4-Frame (AUTO.LOITER
    # geht indoor nicht: verlangt globale Position), Bediener entscheidet
    FS_HOLD = {'Stack', 'Pose map', 'Relokalisierung', 'Planer'}

    def _watchdog(self, checks):
        """Nur im autonomen Flug (armed + OFFBOARD); manuelle Modi fasst er nicht an."""
        st, b = self.mav_state, self.battery
        if not (st and st.armed):
            self.hold = False  # gelandet/disarmed -> Halt erledigt
        if not (st and st.armed and st.mode == 'OFFBOARD'):
            self.bad_since = None
            return
        red = {n for n, ok, _ in checks if not ok}
        land = red & self.FS_LAND
        if b and 0 <= b.percentage < 0.15:
            land.add('Akku kritisch')
        hold = red & self.FS_HOLD
        if not land and not (hold and not self.hold):
            self.bad_since = None
            return
        now = time.time()
        if self.bad_since is None:
            self.bad_since = now
        if now - self.bad_since < (0.5 if land else 2.0):
            return
        self.bad_since = None
        if self.mission and self.mission['state'] in ('running', 'paused', 'waiting'):
            self.mission_ctl = 'abort'
        if land:
            if now - self.failsafe_t < 5.0:  # schon ausgeloest
                return
            self.failsafe_t = now
            self.message = f'FAILSAFE: {", ".join(sorted(land))} -> Landung'
            self._bg(lambda: self._call(self.mode_cli, SetMode.Request(custom_mode='AUTO.LAND')))
        else:
            self.hold = True
            self.message = f'FAILSAFE: {", ".join(sorted(hold))} -> Drohne haelt Position (Halt aufheben oder landen)'
        self.get_logger().error(self.message)

    def _bg(self, fn):
        """MAVROS-Aufrufe nie im Executor-Thread abwarten (single-threaded -> Deadlock bis Timeout)."""
        def run():
            try:
                fn()
            except Exception as e:  # noqa: B902
                self.get_logger().error(f'Hintergrundaufruf fehlgeschlagen: {e}')
        threading.Thread(target=run, daemon=True).start()

    def _publish_status(self):
        checks = self._checks()
        self._watchdog(checks)
        self.hold_pub.publish(Bool(data=self.hold))
        t, _ = self._tf('map', 'camera_init')
        m2o = None
        if t is not None:
            tr, q = t.transform.translation, t.transform.rotation
            m2o = [tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w]
        st = {
            'mode': self.mode, 'map': self.map_name, 'busy': self.busy, 'message': self.message,
            'sim': self.sim, 'uptime': round(time.time() - self.started_at) if self.proc else 0,
            'checks': [{'name': n, 'ok': o, 'detail': d} for n, o, d in checks],
            'ready': all(o for _, o, _ in checks),
            'map_to_odom': m2o,
            'mission': self.mission,
            'hold': self.hold,
            'scan': self.scan,
            'camera': {'topic': self.camera_topic, 'port': self.video_port},
        }
        self.status_pub.publish(String(data=json.dumps(st)))

    # ------------------------------------------------------------------ Kommandos
    def _on_command(self, req, res):
        try:
            args = json.loads(req.request or '{}')
            cmd = args.pop('cmd')
            handler = getattr(self, f'cmd_{cmd}', None)
            if handler is None:
                raise ValueError(f'unbekanntes Kommando: {cmd}')
            out = handler(**args)
            res.success, res.response = True, json.dumps(out if out is not None else {})
        except Exception as e:  # noqa: B902 — jede Fehlermeldung an die UI zurueck
            res.success, res.response = False, str(e)
            self.get_logger().warn(f'Kommando fehlgeschlagen: {e}')
        return res

    def _map_path(self, name):
        if not name or '/' in name or name.startswith('.'):
            raise ValueError(f'ungueltiger Kartenname: {name!r}')
        return os.path.join(self.maps_dir, name)

    def cmd_list_maps(self):
        maps = []
        for n in sorted(os.listdir(self.maps_dir)):
            d = os.path.join(self.maps_dir, n)
            if n.startswith('.') or not os.path.isdir(d):
                continue
            meta = {}
            if os.path.exists(os.path.join(d, 'map.yaml')):
                with open(os.path.join(d, 'map.yaml')) as f:
                    meta = yaml.safe_load(f) or {}
            maps.append({'name': n, 'ready': os.path.exists(os.path.join(d, 'map.bt')),
                         'preview': os.path.exists(os.path.join(d, 'preview.bin')),
                         'created': str(meta.get('created', '')),
                         'origin': meta.get('origin'), 'crop': meta.get('crop') or []})
        return {'maps': maps}

    def cmd_start_mapping(self, name):
        path = self._map_path(name)
        if os.path.exists(path):
            raise ValueError(f'Karte {name} existiert schon')
        self._start('mapping.launch.py', [f'map_dir:={path}'], name, ('start_mapping', {'name': name}))
        return {'map': name}

    def cmd_start_localization(self, map, x=None, y=None, yaw=None):  # noqa: A002 — JSON-Feldname
        path = self._map_path(map)
        if not os.path.exists(os.path.join(path, 'map.bt')):
            raise ValueError(f'Karte {map} hat keine map.bt — erst optimieren')
        if x is None:  # Standard: Startplatz der Kartierung (verschiebt sich mit dem Nullpunkt mit)
            x, y, yaw = _start_pose(path)
        self._start('localization.launch.py',
                    [f'map_dir:={path}', f'initial_x:={x}', f'initial_y:={y}', f'initial_yaw:={yaw}'],
                    map, ('start_localization', {'map': map, 'x': x, 'y': y, 'yaw': yaw}))
        return {'map': map}

    def cmd_stop(self):
        threading.Thread(target=self._stop, daemon=True).start()
        return {}

    def cmd_reset(self):
        if not self.last_start:
            raise ValueError('noch nichts gestartet')
        cmd, args = self.last_start
        if cmd == 'start_mapping':
            raise ValueError('Mapping laesst sich nicht neu starten (Ordner existiert) — neue Karte anlegen')

        def run():
            self._stop()
            getattr(self, f'cmd_{cmd}')(**args)
        threading.Thread(target=run, daemon=True).start()
        return {}

    def cmd_finish_mapping(self):
        if self.mode != 'mapping':
            raise ValueError('keine Kartenaufnahme aktiv')
        name = self.map_name

        def run():
            self._stop()
            self._optimize(name)
        threading.Thread(target=run, daemon=True).start()
        return {'map': name}

    def cmd_optimize(self, map):  # noqa: A002
        self._map_path(map)
        threading.Thread(target=self._optimize, args=(map,), daemon=True).start()
        return {}

    def _edit_yaml(self, map, fn):  # noqa: A002
        path = os.path.join(self._map_path(map), 'map.yaml')
        if not os.path.exists(path):
            raise ValueError('map.yaml fehlt — Karte erst einmal optimieren')
        with open(path) as f:
            meta = yaml.safe_load(f) or {}
        fn(meta)
        with open(path, 'w') as f:
            yaml.safe_dump(meta, f, sort_keys=False, allow_unicode=True)

    def cmd_set_origin(self, map, x, y, z=0.0, yaw_deg=0.0):  # noqa: A002
        """Nullpunkt relativ zum AKTUELLEN Kartenframe verschieben/drehen (akkumuliert)."""
        def upd(meta):
            o = meta.get('origin') or {}
            ox, oy, oz, oyaw = (float(o.get(k, 0.0)) for k in ('x', 'y', 'z', 'yaw_deg'))
            c, s = math.cos(math.radians(oyaw)), math.sin(math.radians(oyaw))
            meta['origin'] = {'x': ox + c * x - s * y, 'y': oy + s * x + c * y, 'z': oz + z,
                              'yaw_deg': (oyaw + yaw_deg + 180.0) % 360.0 - 180.0}
            meta['crop'] = []  # Ausschnitte lagen im alten Frame
        self._edit_yaml(map, upd)
        return self.cmd_optimize(map)

    def cmd_add_crop(self, map, min, max):  # noqa: A002
        box = {'min': [float(v) for v in min], 'max': [float(v) for v in max]}
        self._edit_yaml(map, lambda meta: meta.update(crop=(meta.get('crop') or []) + [box]))
        return self.cmd_optimize(map)

    def cmd_clear_crop(self, map):  # noqa: A002
        self._edit_yaml(map, lambda meta: meta.update(crop=[]))
        return self.cmd_optimize(map)

    def cmd_delete_map(self, map):  # noqa: A002
        if self.map_name == map and self.mode != 'idle':
            raise ValueError('Karte ist gerade in Benutzung')
        trash = os.path.join(self.maps_dir, '.papierkorb')
        os.makedirs(trash, exist_ok=True)
        dest = os.path.join(trash, f'{map}_{datetime.datetime.now():%Y%m%d_%H%M%S}')
        shutil.move(self._map_path(map), dest)
        return {'moved_to': dest}

    def cmd_relocalize(self, x, y, yaw, z=0.0):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = float(x), float(y), float(z)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.initialpose_pub.publish(m)
        self.reloc_t = 0.0  # Ampel rot, bis der Node die neue Lage bestaetigt (Suche dauert einige s)
        self.message = f'Startschaetzung ({x:.1f}, {y:.1f}, {math.degrees(yaw):.0f}°) gesendet'
        return {}

    # ------------------------------------------------------------------ Missionen
    # <map>/missions/<name>.json: {"waypoints": [...], "land_at_end": bool}
    # Wegpunkt: x, y, z [m], hold [s], yaw_deg (None = in Flugrichtung),
    #           confirm (True = erst nach "Weiter" in der UI zum naechsten Punkt)
    def _mission_path(self, map, name):  # noqa: A002
        if not name or '/' in name or name.startswith('.'):
            raise ValueError(f'ungueltiger Missionsname: {name!r}')
        return os.path.join(self._map_path(map), 'missions', f'{name}.json')

    def cmd_list_missions(self, map):  # noqa: A002
        d = os.path.join(self._map_path(map), 'missions')
        out = []
        for f in sorted(os.listdir(d)) if os.path.isdir(d) else []:
            if f.endswith('.json'):
                with open(os.path.join(d, f)) as fh:
                    out.append({'name': f[:-5], **json.load(fh)})
        return {'missions': out}

    def cmd_save_mission(self, map, name, waypoints, land_at_end=True):  # noqa: A002
        wps = [{'x': float(w['x']), 'y': float(w['y']), 'z': float(w['z']),
                'hold': max(0.0, float(w.get('hold') or 0.0)),
                'yaw_deg': None if w.get('yaw_deg') in (None, '') else float(w['yaw_deg']),
                'confirm': bool(w.get('confirm', False))} for w in waypoints]
        if not wps:
            raise ValueError('Mission ohne Wegpunkte')
        path = self._mission_path(map, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump({'waypoints': wps, 'land_at_end': bool(land_at_end)}, f, indent=1)
        return {'name': name}

    # ------------------------------------------------------------------ View Planning
    # <map>/scans/<name>.json = Vorschau (Posen, Facetten-Abdeckung, Punkte, Kennzahlen) — erst
    # "Als Mission uebernehmen" in der UI macht daraus eine Mission
    def _vp(self, *args, timeout=900):
        """marvin_view_planning.plan_box in der eigenen venv (numpy/scipy/open3d, siehe dortiges README)."""
        r = subprocess.run([self.vp_python, '-m', 'marvin_view_planning.plan_box', *args],
                           capture_output=True, text=True, timeout=timeout)
        if r.returncode != 0:
            raise RuntimeError(r.stderr.strip()[-300:] or 'View Planning fehlgeschlagen')
        return r.stdout

    def cmd_scan_defaults(self):
        if self.vp_defaults is None:
            self.vp_defaults = json.loads(self._vp('--defaults', timeout=30).strip().splitlines()[-1])
        return self.vp_defaults

    def _scan_path(self, map, name):  # noqa: A002
        if not name or '/' in name or name.startswith('.'):
            raise ValueError(f'ungueltiger Planungsname: {name!r}')
        return os.path.join(self._map_path(map), 'scans', f'{name}.json')

    def cmd_list_scans(self, map):  # noqa: A002
        d = os.path.join(self._map_path(map), 'scans')
        out = []
        # ponytail: liest jede Vorschau komplett (~0.3 MB) fuer die Kennzahlen — Index-Datei, wenn es viele werden
        for f in sorted(os.listdir(d), reverse=True) if os.path.isdir(d) else []:
            if f.endswith('.json'):
                with open(os.path.join(d, f)) as fh:
                    r = json.load(fh)
                out.append({'name': f[:-5], 'stats': r.get('stats'), 'box': r.get('box')})
        return {'scans': out}

    def cmd_delete_scan(self, map, name):  # noqa: A002
        os.remove(self._scan_path(map, name))
        return {}

    def cmd_plan_scan(self, map, min, max, options=None):  # noqa: A002
        """View Planning fuer Karte + Live-Lidar in der Box -> Vorschau scans/scan_<Zeit>.json (Hintergrund)."""
        pcd = os.path.join(self._map_path(map), 'map.pcd')
        if not os.path.exists(pcd):
            raise ValueError(f'Karte {map} hat keine map.pcd — erst optimieren')
        if self.busy:
            raise ValueError(f'beschaeftigt: {self.busy}')
        name = f'scan_{datetime.datetime.now():%Y%m%d_%H%M%S}'
        out_path = self._scan_path(map, name)
        lo, hi = np.asarray(min, np.float32), np.asarray(max, np.float32)
        if not np.all(lo < hi):
            raise ValueError('Box: min muss in jeder Achse kleiner als max sein')
        live = self.live_pts if (self.mode, self.map_name) == ('localization', map) else np.empty((0, 3))
        self.busy = 'View Planning laeuft ...'

        def run():
            try:
                pts = np.vstack([_read_pcd_xyz(pcd), live])
                pts = pts[np.all((pts >= lo) & (pts <= hi), axis=1)]
                n_live = int(np.all((live >= lo) & (live <= hi), axis=1).sum())
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                with tempfile.NamedTemporaryFile(suffix='.npy') as f, \
                        tempfile.NamedTemporaryFile('w', suffix='.json') as o:
                    np.save(f, pts.astype(np.float32))
                    f.flush()
                    json.dump(options or {}, o)
                    o.flush()
                    self._vp(f.name, out_path, o.name)
                with open(out_path) as fh:
                    res = json.load(fh)
                res['stats']['live_points'] = n_live
                res.update(name=name, map=map, box={'min': lo.tolist(), 'max': hi.tolist()}, options=options or {})
                with open(out_path, 'w') as fh:
                    json.dump(res, fh)
                st = res['stats']
                ok, self.message = True, (
                    f'{name}: {st["reachable"]}/{st["selected"]} Posen erreichbar, {100 * st["coverage"]:.0f} % '
                    f'erfasst ({100 * st["coverable"]:.0f} % erfassbar), {st["length_m"]} m, {st["runtime_s"]} s')
            except Exception as e:  # noqa: B902
                ok, self.message = False, f'View Planning fehlgeschlagen: {e}'
            self.busy = ''
            self.scan = {'map': map, 'name': name, 'ok': ok, 'text': self.message}
            self.get_logger().info(self.message)
        threading.Thread(target=run, daemon=True).start()
        return {'name': name}

    def cmd_delete_mission(self, map, name):  # noqa: A002
        os.remove(self._mission_path(map, name))
        return {}

    def cmd_release_hold(self):
        red = [n for n, ok, _ in self._checks() if not ok and n in self.FS_HOLD | self.FS_LAND]
        if red:
            raise ValueError('Halt bleibt — noch rot: ' + ', '.join(red))
        self.hold = False
        self.message = 'Failsafe-Halt aufgehoben'
        return {}

    def cmd_start_mission(self, name):
        with open(self._mission_path(self.map_name, name)) as f:
            self._launch(name, json.load(f))
        return {}

    def _launch(self, name, mission):
        if self.mode != 'localization':
            raise ValueError('Missionen nur im Modus "Auf Karte fliegen"')
        if self.mission and self.mission['state'] in ('running', 'paused', 'waiting'):
            raise ValueError('Es laeuft schon eine Mission')
        failed = [n for n, ok, _ in self._checks() if not ok]
        if failed:
            raise ValueError('Nicht flugbereit: ' + ', '.join(failed))
        self.mission_ctl = None
        self.hold = False  # Ampel ist gruen (oben geprueft) -> Failsafe-Halt aufheben
        self.mission = {'name': name, 'state': 'running', 'index': 0, 'total': len(mission['waypoints']),
                        'text': 'startet ...'}
        threading.Thread(target=self._run_mission, args=(mission,), daemon=True).start()

    # ------------------------------------------------------------------ Handsteuerung
    def _live(self):
        return bool(self.mission and self.mission['state'] in ('running', 'paused', 'waiting'))

    def cmd_takeoff(self, z):
        """Abheben (oder in der Luft: Hoehe aendern) ueber dem aktuellen Punkt = Mission mit 1 Wegpunkt
        -> Armen mit Preflight-Wartezeit, Abbruch, Watchdog und Fortschritt wie bei Missionen."""
        x, y, _ = self._pose()
        self._launch('Abheben', {'waypoints': [{'x': x, 'y': y, 'z': float(z), 'hold': 0,
                                                'yaw_deg': math.degrees(self._yaw()), 'confirm': False}],
                                 'land_at_end': False})
        return {}

    def cmd_goto(self, x, y, z, yaw=None):
        if self._live():
            raise ValueError(f'Mission {self.mission["name"]} laeuft — erst abbrechen/Halten')
        if not (self.mav_state and self.mav_state.armed):
            raise ValueError('Drohne ist nicht in der Luft — erst abheben')
        if self.hold:
            raise ValueError('Failsafe-Halt aktiv — erst aufheben')
        px, py, _ = self._pose()
        self._goal(x, y, z, math.atan2(y - py, x - px) if yaw is None else yaw)
        if self.mav_state.mode != 'OFFBOARD':  # z.B. nach AUTO.LAND: pursuit streamt ohnehin Setpoints
            self._bg(lambda: self._call(self.mode_cli, SetMode.Request(custom_mode='OFFBOARD')))
        self.message = f'fliege zu ({x:.1f}, {y:.1f}, {z:.1f})'
        return {}

    def cmd_hold(self):
        if self._live():
            self.mission_ctl = 'abort'  # Runner haelt selbst
        else:
            self._hold_here()
        self.message = 'Drohne haelt Position'
        return {}

    def cmd_arm(self, value=True):
        what = 'Armen' if value else 'Disarmen'

        def run():
            ok = self._call(self.arm_cli, CommandBool.Request(value=bool(value))).success
            self.message = f'{what} ' + ('ok' if ok else 'verweigert (PX4-Preflight bzw. in der Luft, siehe PX4-Log)')
        self._bg(run)
        return {}

    def cmd_pause_mission(self):
        if not self.mission or self.mission['state'] != 'running':
            raise ValueError('keine laufende Mission')
        self.mission_ctl = 'pause'
        return {}

    def cmd_resume_mission(self):
        if not self.mission or self.mission['state'] != 'paused':
            raise ValueError('Mission ist nicht pausiert')
        self.mission_ctl = None
        return {}

    def cmd_continue_mission(self):
        if not self.mission or self.mission['state'] != 'waiting':
            raise ValueError('Mission wartet nicht auf Bestaetigung')
        self.mission_ctl = 'continue'
        return {}

    def cmd_land(self):
        """Sofort landen (bricht eine laufende Mission ab)."""
        if self.mission and self.mission['state'] in ('running', 'paused', 'waiting'):
            self.mission_ctl = 'abort'
        # erst nach dem Abbruch (Runner haelt kurz) landen; nie im Executor-Thread warten
        self._bg(lambda: (time.sleep(0.5), self._call(self.mode_cli, SetMode.Request(custom_mode='AUTO.LAND'))))
        self.message = 'Landung eingeleitet'
        return {}

    def cmd_abort_mission(self):
        if not self.mission or self.mission['state'] not in ('running', 'paused', 'waiting'):
            raise ValueError('keine laufende Mission')
        self.mission_ctl = 'abort'
        return {}

    class _Abort(Exception):
        pass

    def _pose(self):
        t, age = self._tf('map', 'base_link')
        if t is None or age > 1.0:
            raise RuntimeError('keine aktuelle Pose (map->base_link)')
        tr = t.transform.translation
        return tr.x, tr.y, tr.z

    def _goal(self, x, y, z, yaw):
        m = PoseStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), float(z)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.goal_pub.publish(m)

    def _yaw(self):
        t, _ = self._tf('map', 'base_link')
        if t is None:
            raise RuntimeError('keine aktuelle Pose (map->base_link)')
        q = t.transform.rotation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _hold_here(self):
        try:
            x, y, z = self._pose()
            self._goal(x, y, z, self._yaw())
        except RuntimeError:
            pass

    def _call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service {cli.srv_name} nicht erreichbar')
        fut = cli.call_async(req)  # Antwort kommt ueber den Executor im Hauptthread
        end = time.time() + timeout
        while not fut.done():
            if time.time() > end:
                raise RuntimeError(f'Service {cli.srv_name}: Timeout')
            time.sleep(0.05)
        return fut.result()

    def _wait(self, cond, timeout, what):
        """Warten bis cond() wahr ist; Pause/Abbruch/Planerfehler beachten."""
        end = time.time() + timeout
        start = time.time()
        while not cond():
            if self.mission_ctl == 'abort':
                raise self._Abort()
            if self.mode != 'localization':
                raise RuntimeError('Stack wurde gestoppt')
            if self.empty_path_t > start + 1.0:
                raise RuntimeError(f'Planer findet keinen Weg ({what})')
            if time.time() > end:
                raise RuntimeError(f'Timeout: {what}')
            time.sleep(0.2)

    def _run_mission(self, mission):
        m = self.mission
        wps = mission['waypoints']
        tol_xy, tol_z = 0.4, 0.3

        def at(x, y, z):
            px, py, pz = self._pose()
            return math.hypot(px - x, py - y) < tol_xy and abs(pz - z) < tol_z

        try:
            if not (self.mav_state and self.mav_state.armed):
                # Abheben: Ziel senkrecht ueber dem Startpunkt -> Planer plant, pursuit
                # streamt Setpoints; OFFBOARD selbst anfordern (pursuit tut das nur beim
                # allerersten Pfad nach seinem Start), dann armen
                x0, y0, _ = self._pose()
                m['text'] = 'hebe ab ...'
                self._goal(x0, y0, wps[0]['z'], 0.0)
                time.sleep(2.0)
                self._call(self.mode_cli, SetMode.Request(custom_mode='OFFBOARD'))
                self._wait(lambda: self.mav_state and self.mav_state.mode == 'OFFBOARD', 10, 'OFFBOARD')
                # PX4-Preflight (EKF-Heading, IMU-Bias) wird oft erst Sekunden nach "gruen" fertig;
                # die Pruefung selbst sieht man ueber MAVROS nicht (nur Events) -> nachfassen
                for attempt in range(30):  # bis 90 s: unter Last braucht der EKF lange
                    if self._call(self.arm_cli, CommandBool.Request(value=True)).success:
                        break
                    if self.mission_ctl == 'abort' or attempt == 29:
                        raise RuntimeError('Armen verweigert (PX4-Preflight, siehe PX4-Log)')
                    m['text'] = f'warte auf PX4-Preflight ({3 * (attempt + 1)} s) ...'
                    time.sleep(3.0)
                self._wait(lambda: at(x0, y0, wps[0]['z']), 60, 'Abheben')
            for i, wp in enumerate(wps):
                m['index'] = i
                px, py, _ = self._pose()
                yaw = (math.radians(wp['yaw_deg']) if wp.get('yaw_deg') is not None
                       else math.atan2(wp['y'] - py, wp['x'] - px))
                m['text'] = f'fliege zu Wegpunkt {i + 1}/{len(wps)}'
                self._goal(wp['x'], wp['y'], wp['z'], yaw)
                dist = math.hypot(wp['x'] - px, wp['y'] - py)
                while True:
                    self._wait(lambda: self.mission_ctl == 'pause' or at(wp['x'], wp['y'], wp['z']),
                               60 + dist / 0.3, f'Wegpunkt {i + 1}')
                    if self.mission_ctl != 'pause':
                        break
                    m['state'], m['text'] = 'paused', f'pausiert vor Wegpunkt {i + 1}'
                    self._hold_here()
                    while self.mission_ctl == 'pause':
                        time.sleep(0.2)
                    if self.mission_ctl == 'abort':
                        raise self._Abort()
                    m['state'], m['text'] = 'running', f'fliege zu Wegpunkt {i + 1}/{len(wps)}'
                    self._goal(wp['x'], wp['y'], wp['z'], yaw)
                if wp['hold'] > 0:
                    m['text'] = f'warte {wp["hold"]:.0f} s an Wegpunkt {i + 1}'
                    end = time.time() + wp['hold']
                    while time.time() < end:
                        if self.mission_ctl == 'abort':
                            raise self._Abort()
                        time.sleep(0.2)
                if wp.get('confirm'):
                    m['state'], m['text'] = 'waiting', f'wartet an Wegpunkt {i + 1} auf „Weiter“'
                    while self.mission_ctl != 'continue':
                        if self.mission_ctl == 'abort':
                            raise self._Abort()
                        if self.mode != 'localization':
                            raise RuntimeError('Stack wurde gestoppt')
                        time.sleep(0.2)
                    self.mission_ctl = None
                    m['state'] = 'running'
            if mission.get('land_at_end', True):
                m['text'] = 'lande ...'
                self._call(self.mode_cli, SetMode.Request(custom_mode='AUTO.LAND'))
            m['state'], m['text'] = 'done', 'Mission abgeschlossen'
        except self._Abort:
            self._hold_here()
            m['state'], m['text'] = 'aborted', 'abgebrochen — Drohne haelt Position'
        except Exception as e:  # noqa: B902 — jeder Fehler: halten + melden
            self._hold_here()
            m['state'], m['text'] = 'failed', f'Fehler: {e} — Drohne haelt Position'
        self.mission_ctl = None
        self.get_logger().info(f'Mission {m["name"]}: {m["text"]}')

    # ------------------------------------------------------------------ Stack
    def _start(self, launch_file, args, map_name, start_cmd):
        with self.lock:
            if self.mode != 'idle':
                raise ValueError(f'Stack laeuft bereits ({self.mode}) — erst stoppen')
            self.mode = 'starting'
        cmd = ['ros2', 'launch', 'marvin_launch', launch_file, f'sim:={str(self.sim).lower()}', *args,
               *self.launch_extra.split()]
        env = dict(os.environ, PYTHONUNBUFFERED='1')  # sonst kommt der Log in 4-KB-Bloecken
        if self.sim and self.display:
            env['DISPLAY'] = self.display
            env.setdefault('XAUTHORITY', f'/run/user/{os.getuid()}/gdm/Xauthority')
        log = open(os.path.join(self.maps_dir, '.stack.log'), 'w')
        self.proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        if self.sim:
            # Sim: PX4 verlangt einen GCS-Link (NAV_DLL_ACT) — ersetzt QGC
            self.aux = [subprocess.Popen(['ros2', 'run', 'marvin_ui', 'fake_gcs'], stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL, start_new_session=True)]
        self.mode = 'mapping' if 'mapping' in launch_file else 'localization'
        self.map_name, self.last_start, self.started_at = map_name, start_cmd, time.time()
        self.vision_dead, self.reloc_t = False, 0.0
        self.live_pts = np.empty((0, 3), np.float32)
        self.message = f'{self.mode} gestartet: {" ".join(cmd[3:])}'
        self.get_logger().info(self.message)

    def _stop(self):
        with self.lock:
            if self.mode in ('idle', 'stopping'):
                return
            self.mode = 'stopping'
        self.message = 'stoppe Stack ...'
        for p in [self.proc, *self.aux]:
            if p and p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 25
        while time.time() < deadline and any(p and p.poll() is None for p in [self.proc, *self.aux]):
            time.sleep(0.5)
        for p in [self.proc, *self.aux]:
            if p and p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._kill_leftovers()  # Sicherheitsnetz: Prozesse, die sich aus der Gruppe geloest haben
        self.proc, self.aux = None, []
        self.mode, self.message = 'idle', 'Stack gestoppt'
        self.get_logger().info('Stack gestoppt')

    @staticmethod
    def _kill_leftovers():
        for pat in LEFTOVERS:
            subprocess.run(['pkill', '-9', '-f', f'[{pat[0]}]{pat[1:]}'], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

    # ------------------------------------------------------------------ Karten
    def _optimize(self, name):
        path = self._map_path(name)
        prev_mode = self.mode
        self.busy = f'optimiere {name} ...'
        if self.mode == 'idle':
            self.mode = 'optimizing'
        try:
            r = subprocess.run(['ros2', 'run', 'marvin_nav', 'map_optimizer', path],
                               capture_output=True, text=True, timeout=1800)
            summary = [ln for ln in r.stdout.splitlines() if 'Loop-Closures' in ln or 'Verschiebung' in ln]
            if r.returncode != 0:
                raise RuntimeError(r.stderr.strip()[-300:] or 'map_optimizer fehlgeschlagen')
            self._make_preview(path)
            self.message = f'Karte {name} fertig: ' + ' | '.join(summary)
        except Exception as e:  # noqa: B902
            self.message = f'Optimierung {name} fehlgeschlagen: {e}'
        finally:
            self.busy = ''
            if self.mode == 'optimizing':
                self.mode = prev_mode if prev_mode != 'optimizing' else 'idle'
        self.get_logger().info(self.message)

    @staticmethod
    def _make_preview(path, voxel=0.15):
        """map.pcd -> preview.bin (Float32 xyz, Voxel 15 cm) fuer den Browser."""
        p = _read_pcd_xyz(os.path.join(path, 'map.pcd'))
        keys = np.floor(p / voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        p[idx].astype('<f4').tofile(os.path.join(path, 'preview.bin'))

    # ------------------------------------------------------------------ HTTP
    def _start_http(self):
        web = os.path.join(get_package_share_directory('marvin_ui'), 'web')
        maps = self.maps_dir

        class Handler(http.server.SimpleHTTPRequestHandler):
            def translate_path(self, path):
                path = path.split('?', 1)[0]
                if path.startswith('/maps/'):  # Kartenvorschauen direkt aus maps_dir
                    rel = os.path.normpath(path[len('/maps/'):]).lstrip('/')
                    return os.path.join(maps, rel) if not rel.startswith('..') else maps
                return super().translate_path(path)

            def do_GET(self):
                if not self.path.startswith('/stacklog'):
                    return super().do_GET()
                # Stack-Log ab Byte-Offset ?from=N (Browser pollt inkrementell); X-Size = neuer Offset
                q = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)
                try:
                    with open(os.path.join(maps, '.stack.log'), 'rb') as f:
                        size = f.seek(0, 2)
                        start = int(q.get('from', ['-1'])[0])
                        if not 0 <= start <= size:  # erster Abruf oder Log neu angelegt (Neustart)
                            start = max(0, size - 200_000)
                        f.seek(start)
                        data = f.read(size - start)
                except FileNotFoundError:
                    size, data = 0, b''
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('X-Size', str(size))
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *a):
                pass

            def end_headers(self):
                self.send_header('Cache-Control', 'no-store')
                super().end_headers()

        srv = http.server.ThreadingHTTPServer(('0.0.0.0', self.http_port),
                                              functools.partial(Handler, directory=web))
        threading.Thread(target=srv.serve_forever, daemon=True).start()

    def shutdown(self):
        self._stop()


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
