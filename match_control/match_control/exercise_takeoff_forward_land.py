import rclpy
from rclpy.node import Node


class ExerciseTakeoffForwardLand(Node):
    """Aufgabe: Abheben, vorwärts fliegen und landen."""

    def __init__(self) -> None:
        super().__init__('exercise_takeoff_forward_land')
        self.get_logger().info('TODO: Implementiere Vorwärtsflug und Landung')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExerciseTakeoffForwardLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
