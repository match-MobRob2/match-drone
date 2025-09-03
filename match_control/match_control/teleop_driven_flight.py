import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class TeleopOffboardBridge(Node):
    def __init__(self):
        super().__init__('teleop_offboard_bridge')

        self.current_twist = Twist()
        self.have_pose = False
        self.current_mode = ""
        self.armed = False
        
        # QoS
        qos = QoSProfile(depth=10)  # general
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability  = QoSDurabilityPolicy.VOLATILE
        sensor_qos.history     = QoSHistoryPolicy.KEEP_LAST

        # Inputs
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.on_pose, sensor_qos)
        self.create_subscription(State, '/mavros/state', self.on_state, qos)

        # Manual throttle state
        self._last_pose_warn_ns = 0

        # MAVROS publisher (stamped only)
        self.pub_stamped = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos)

        # Services
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # 20 Hz setpoint stream (must run before/offboard)
        self.timer_pub = self.create_timer(0.02, self.publish_setpoint)

        # Sequence + OFFBOARD retry
        self.seq_started = False
        self.timer_seq = self.create_timer(1.0, self.start_sequence)      # kick once ready
        self.mode_retry_timer = self.create_timer(0.5, self.try_offboard) # periodic OFFBOARD attempts
        self.mode_retry_timer.cancel()  # enabled later

    # --- Callbacks ---
    def on_cmd_vel(self, msg: Twist):
        self.current_twist = msg

    def on_pose(self, _msg: PoseStamped):
        self.have_pose = True

    def on_state(self, msg: State):
        self.current_mode = msg.mode
        self.armed = msg.armed

    # --- Setpoint stream ---
    def publish_setpoint(self):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'map'
        ts.twist = self.current_twist
        self.pub_stamped.publish(ts)

    # --- Sequence control ---
    def start_sequence(self):
        # start only when pose is valid and services are ready
        if self.seq_started:
            return
        if not self.have_pose:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_pose_warn_ns > 2_000_000_000:  # 2s
                self.get_logger().warn('Waiting for local pose…')
                self._last_pose_warn_ns = now_ns
            return
        self.seq_started = True

        # ARM first (idempotent)
        arm_req = CommandBool.Request(); arm_req.value = True
        self.arm_cli.call_async(arm_req)

        # enable OFFBOARD retry loop
        self.mode_retry_timer.reset()

    def try_offboard(self):
        # Keep trying until mode == OFFBOARD
        if self.current_mode == 'OFFBOARD':
            self.mode_retry_timer.cancel()
            return
        if not self.have_pose or not self.armed:
            return  # need EKF + armed
        req = SetMode.Request(); req.custom_mode = 'OFFBOARD'
        self.mode_cli.call_async(req)

def main():
    rclpy.init()
    node = TeleopOffboardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
