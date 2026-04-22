#!/usr/bin/env python3
"""
Rosbag Recorder Node
Lauscht auf /mavros/state und startet/stoppt eine rosbag-Aufnahme
abhängig vom armed-Status:

  - armed == True  → Aufnahme starten
  - armed == False → Aufnahme beenden

Bag-Dateiname: <output_dir>/YYYY-MM-DD_HH-MM-SS/
"""

import subprocess
import signal
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from mavros_msgs.msg import State


RECORD_TOPICS = [
    '/mavros/battery',
    '/mavros/local_position/pose',
    '/camera/camera/color/image_raw',
    '/livox/lidar',
]


class RosbagRecorderNode(Node):

    def __init__(self):
        super().__init__('rosbag_recorder_node')

        self.declare_parameter('output_dir', '/home/marvin-drone/flight_logs')
        self.output_dir = Path(
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.callback_group = ReentrantCallbackGroup()

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
            callback_group=self.callback_group,
        )

        self._armed: bool | None = None
        self._bag_process: subprocess.Popen | None = None

        self.get_logger().info(
            f'Rosbag Recorder Node gestartet – Output: {self.output_dir}'
        )
        self.get_logger().info(f'Aufzunehmende Topics: {RECORD_TOPICS}')

    def state_callback(self, msg: State):
        if msg.armed == self._armed:
            return
        self._armed = msg.armed

        if msg.armed:
            self._start_recording()
        else:
            self._stop_recording()

    def _start_recording(self):
        if self._bag_process is not None:
            self.get_logger().warn('Aufnahme läuft bereits – ignoriere Start')
            return

        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bag_path = self.output_dir / timestamp

        cmd = ['ros2', 'bag', 'record', '-o', str(bag_path)] + RECORD_TOPICS

        self.get_logger().info(f'Starte Aufnahme → {bag_path}')

        try:
            self._bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().error('ros2 nicht gefunden – ist ROS2 gesourct?')

    def _stop_recording(self):
        if self._bag_process is None:
            return

        self.get_logger().info('Stoppe Aufnahme (SIGINT)...')

        try:
            self._bag_process.send_signal(signal.SIGINT)
            self._bag_process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Timeout – erzwinge SIGKILL')
            self._bag_process.kill()
            self._bag_process.wait()
        except ProcessLookupError:
            pass

        self.get_logger().info('Aufnahme beendet')
        self._bag_process = None

    def destroy_node(self):
        self._stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()