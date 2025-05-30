import rclpy
from drone_control.steuerung import Steuerung

def main():
    rclpy.init()
    node = Steuerung()
    node.arme_drohne()
    rclpy.spin(node)
    rclpy.shutdown()