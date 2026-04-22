import rclpy
from rclpy.node import Node


class ExerciseTakeoffLand(Node):
    """Aufgabe: Drohne abheben und wieder landen."""

    def __init__(self) -> None:
        super().__init__('exercise_takeoff_land')
        self.get_logger().info('TODO: Implementiere Abheben und Landen')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExerciseTakeoffLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
