import math
import rclpy
from enum import Enum
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from tf_transformations import quaternion_from_euler

# ------------------------------ CONFIG ---------------------------------- #
TARGET_ALT = 3.0               # Meter: Höhe, auf die erst gestiegen wird
LOOP_STEPS = 120               # Schritte für 360° Pitch-Looping  (20 Hz → 6 s)
HOLD_TIME  = 20                # Sekunden Position halten, danach Node beenden

# PX4-Parameter, die wir (gefahrlos in SITL) ändern
LOOP_PARAMS = {
    'MC_AIRMODE': 1,        # voller Schub bei maximalem Tilt
    'MPC_TILTMAX_AIR': 120, # Neigungsgrenze (deg)
    'COM_DISARM_LAND': 0,   # kein auto-Disarm
    'COM_OBL_ACT': 0        # Offboard failsafe aus
}

# --------------------------- MISSION STATES ----------------------------- #
class MissionState(Enum):
    SET_PARAMS   = 0
    SEND_IDLE    = 1     # 2 s Dummy-Setpoints, bevor OFFBOARD erlaubt wird
    ARM_OFFBOARD = 2
    ASCEND       = 3
    LOOPING      = 4
    HOLD         = 5
    DONE         = 6

# ------------------------------  NODE  ---------------------------------- #
class LoopingMission(Node):
    def __init__(self):
        super().__init__('looping_mission')

        # Publisher
        self.pose_pub      = self.create_publisher(PoseStamped, 
                                                   '/mavros/setpoint_position/local', 10)
        self.att_pub       = self.create_publisher(AttitudeTarget, 
                                                   '/mavros/setpoint_raw/attitude', 10)

        # Service clients
        self.arming_cli    = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli      = self.create_client(SetMode,      '/mavros/set_mode')
        self.param_cli     = self.create_client(ParamSet,     '/mavros/param/set')

        # State subscription
        self.state_sub     = self.create_subscription(State, '/mavros/state', 
                                                     self.state_cb, 10)
        self.current_state = State()

        # Timing
        self.rate_hz = 20
        self.timer    = self.create_timer(1.0 / self.rate_hz, self.timer_cb)
        self.t_cnt    = 0          # generic counter for timers / loops

        # FSM
        self.state = MissionState.SET_PARAMS
        self.loop_step = 0
        self.hold_counter = 0
        self.get_logger().info('Looping mission node initialised')

    # ----------------------- Callback: PX4 State ------------------------ #
    def state_cb(self, msg):
        self.current_state = msg

    # ----------------------- Helper Functions -------------------------- #
    def set_param(self, name: str, value: int):
        req               = ParamSet_Request()
        req.param_id      = name
        req.value.integer = value
        future = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() and future.result().success:
            self.get_logger().info(f'Param {name} set → {value}')
        else:
            self.get_logger().warn(f'FAILED to set {name}')

    def arm_and_offboard(self):
        # Arm
        arm_req = CommandBool.Request(value=True)
        self.arming_cli.call_async(arm_req)
        # Offboard
        mode_req = SetMode.Request(custom_mode='OFFBOARD')
        self.mode_cli.call_async(mode_req)
        self.get_logger().info('Arming + OFFBOARD requested')

    # ----------------------- Main Timer FSM ---------------------------- #
    def timer_cb(self):
        # 1) publish appropriate setpoint every cycle
        if self.state in (MissionState.SEND_IDLE, MissionState.ARM_OFFBOARD, MissionState.ASCEND):
            self.pub_pose_setpoint(0.0, 0.0, TARGET_ALT)  # hold above origin
        elif self.state == MissionState.LOOPING:
            self.pub_looping_setpoint()
        elif self.state == MissionState.HOLD:
            self.pub_pose_setpoint(0.0, 0.0, TARGET_ALT)

        # 2) FSM transitions
        if self.state == MissionState.SET_PARAMS:
            # if not self.param_cli.wait_for_service(timeout_sec=0.5):
            #     self.get_logger().info('Waiting for param service...')
            #     return
            for k, v in LOOP_PARAMS.items():
                self.set_param(k, v)
            self.state = MissionState.SEND_IDLE
            self.get_logger().info('Params set. Sending idle setpoints for 2 s...')

        elif self.state == MissionState.SEND_IDLE:
            self.t_cnt += 1
            if self.t_cnt > 2 * self.rate_hz:   # ≈2 s
                self.state = MissionState.ARM_OFFBOARD
                self.get_logger().info('Requesting arm + offboard...')
                if self.arming_cli.wait_for_service(timeout_sec=0.5) and \
                   self.mode_cli.wait_for_service(timeout_sec=0.5):
                    self.arm_and_offboard()

        elif self.state == MissionState.ARM_OFFBOARD:
            if self.current_state.armed and self.current_state.mode == 'OFFBOARD':
                self.get_logger().info('Armed + OFFBOARD confirmed → Ascend')
                self.state = MissionState.ASCEND

        elif self.state == MissionState.ASCEND:
            # simple check: after reaching 90 % of target alt, proceed
            pos = getattr(self, 'current_z', None)
            if pos is None:
                return
            if pos > TARGET_ALT * 0.9:
                self.get_logger().info('Target altitude reached → Start Looping')
                self.state = MissionState.LOOPING

        elif self.state == MissionState.LOOPING:
            self.loop_step += 1
            if self.loop_step >= LOOP_STEPS:
                self.get_logger().info('Looping complete → Hold position')
                self.state = MissionState.HOLD

        elif self.state == MissionState.HOLD:
            self.hold_counter += 1
            if self.hold_counter >= HOLD_TIME * self.rate_hz:
                self.get_logger().info('Mission finished. Disarming...')
                self.state = MissionState.DONE
                disarm_req = CommandBool.Request(value=False)
                self.arming_cli.call_async(disarm_req)

        elif self.state == MissionState.DONE:
            pass  # Node will stay alive but do nothing

    # ------------------------ Publishers ------------------------------ #
    def pub_pose_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0  # neutral attitude
        self.pose_pub.publish(msg)
        # store z for altitude check
        self.current_z = z

    def pub_looping_setpoint(self):
        angle = (self.loop_step / LOOP_STEPS) * 2 * math.pi  # 0→2π
        q = quaternion_from_euler(0.0, angle, 0.0)           # roll, pitch, yaw
        msg = AttitudeTarget()
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.thrust = 0.7
        msg.type_mask = 128  # only orientation + thrust
        msg.header.stamp = self.get_clock().now().to_msg()
        self.att_pub.publish(msg)

# ------------------------------- main ---------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = LoopingMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
