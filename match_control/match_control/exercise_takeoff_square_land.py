import rclpy
from rclpy.node import Node


class ExerciseTakeoffSquareLand(Node):
    """Aufgabe: Abheben, Quadrat fliegen und landen."""

    def __init__(self) -> None:
        super().__init__('exercise_takeoff_square_land')
        self.get_logger().info('TODO: Implementiere Quadratflug und Landung')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExerciseTakeoffSquareLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
