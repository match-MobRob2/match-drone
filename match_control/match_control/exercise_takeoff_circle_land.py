import rclpy
from rclpy.node import Node


class ExerciseTakeoffCircleLand(Node):
    """Aufgabe: Abheben, Kreis fliegen und an Punkt X landen."""

    def __init__(self) -> None:
        super().__init__('exercise_takeoff_circle_land')
        self.get_logger().info('TODO: Implementiere Kreisflug und Landung')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExerciseTakeoffCircleLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
