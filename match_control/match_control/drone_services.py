"""Service node providing basic drone utilities."""

from math import sqrt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from match_control.srv import DistanceToPoint


class DroneServiceNode(Node):
    """Provide simple services for drone state queries."""

    def __init__(self) -> None:
        super().__init__('drone_service_node')
        self._current_pose = None
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._pose_callback,
            10,
        )
        self.create_service(DistanceToPoint, 'distance_to_point', self._handle_distance)
        self.create_service(Trigger, 'is_ready', self._handle_is_ready)

    def _pose_callback(self, msg: PoseStamped) -> None:
        """Store the most recent pose of the drone."""
        self._current_pose = msg

    def _handle_distance(self, request: DistanceToPoint.Request, response: DistanceToPoint.Response) -> DistanceToPoint.Response:
        """Return the distance from the current pose to the requested point."""
        if self._current_pose is None:
            response.distance = float('nan')
        else:
            dx = self._current_pose.pose.position.x - request.x
            dy = self._current_pose.pose.position.y - request.y
            dz = self._current_pose.pose.position.z - request.z
            response.distance = sqrt(dx * dx + dy * dy + dz * dz)
        return response

    def _handle_is_ready(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:  # noqa: D401
        """Check whether the node has received a pose message."""
        ready = self._current_pose is not None
        response.success = ready
        response.message = 'ready' if ready else 'no position received'
        return response


def main() -> None:
    rclpy.init()
    node = DroneServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  # pragma: no cover
    main()
