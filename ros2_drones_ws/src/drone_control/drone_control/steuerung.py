from rclpy.node import Node
from std_msgs.msg import Bool
from mavros_msgs.srv import SetMode

class Steuerung(Node):
    def __init__(self):
        super().__init__('steuerung_node')
        self.arm_publisher = self.create_publisher(Bool, '/mavros/cmd/arming', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def arme_drohne(self):
        msg = Bool()
        msg.data = True
        self.arm_publisher.publish(msg)

    def setze_modus(self, mode='OFFBOARD'):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)
