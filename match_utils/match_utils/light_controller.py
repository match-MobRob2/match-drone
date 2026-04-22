# led_controller/led_controller/led_node.py
import rclpy
from rclpy.node import Node
import serial
from marvin_msgs.srv import SetAllLeds, SetSingleLed

LED_COUNT = 4

class LedControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller')

        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.serial = serial.Serial(port, baud, timeout=1.0)
        self.get_logger().info(f'Verbunden mit {port} @ {baud}')

        self.srv_all = self.create_service(
            SetAllLeds,
            'set_all_leds',
            self.handle_set_all
        )
        self.srv_single = self.create_service(
            SetSingleLed,
            'set_single_led',
            self.handle_set_single
        )
        self.get_logger().info('Services /set_all_leds und /set_single_led bereit')

    def _send(self, cmd: str):
        """Sendet einen Befehl und liest die Antwort."""
        self.serial.write(cmd.encode())
        return self.serial.readline().decode().strip()

    def handle_set_all(self, request, response):
        cmd = f'A:{request.r:02X}{request.g:02X}{request.b:02X}\n'
        try:
            reply = self._send(cmd)
            response.success = (reply == 'OK')
            response.message = reply
        except serial.SerialException as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Seriell-Fehler: {e}')
        return response

    def handle_set_single(self, request, response):
        if request.index >= LED_COUNT:
            response.success = False
            response.message = f'Index {request.index} ungültig (0–{LED_COUNT - 1})'
            return response

        cmd = f'S:{request.index}:{request.r:02X}{request.g:02X}{request.b:02X}\n'
        try:
            reply = self._send(cmd)
            response.success = (reply == 'OK')
            response.message = reply
        except serial.SerialException as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Seriell-Fehler: {e}')
        return response

    def destroy_node(self):
        self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LedControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()