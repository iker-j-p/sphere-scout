"""motor_state_machine: drives the sphere and runs the scan sequence.

Subscribes  /cmd_vel_teleop (geometry_msgs/Twist), /joint_states (sensor_msgs/JointState),
            /odometry/filtered (nav_msgs/Odometry, heading for the scan steps),
            /imu/data (sensor_msgs/Imu, core pitch for swing damping)
Publishes   /cmd_vel (geometry_msgs/Twist), /shell/cmd (std_msgs/Float64),
            /mission/state (std_msgs/String, latched)
Service     /mission/scan (std_srvs/Trigger), accepted only in DRIVE

States
  DRIVE    follows the latest teleop command (forward speed ramped at drive_accel)
  STOP     zero velocity for settle_time_s so the core stops swinging
  EXPAND   opens the shells (expanded_cmd) and waits for /joint_states to reach it
  SCAN     scan_steps turns of scan_step_deg at up to scan_yaw_rate, each followed by
           scan_hold_s at rest for the thermal camera. The turns are closed-loop on the EKF
           heading: opening the shells widens the track, so DiffDrive's open-loop turn rate
           is off. The robot ends facing the way it started.
  RETRACT  closes the shells (rolling_gap_cmd), waits for /joint_states, then DRIVE
Every state except DRIVE commands zero velocity, apart from SCAN's turns. Teleop is ignored
outside DRIVE.

Swing damping: the core hangs under the axle like a pendulum, and with nothing to slow it the
swing never dies (Gazebo has no rolling resistance; on hardware it takes seconds). In every
state the forward command gets swing_damping x (core pitch - its slow average) added, which
rolls the shells under the swinging core and settles it in about a second. The slow average
(swing_washout_s) keeps a steady tilt, such as on a slope, from turning into creep.

Run in simulation with --ros-args -p use_sim_time:=true.
"""

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

SHELL_JOINTS = ('left_shell_joint', 'right_shell_joint')
LOOP_PERIOD_S = 0.02
TURN_DECEL = 2.0          # rad/s^2: turn at full rate, then brake on this curve into each step
MIN_TURN_RATE = 0.15      # rad/s, so the last degrees of a step don't stall


class State(Enum):
    DRIVE = 'DRIVE'
    STOP = 'STOP'
    EXPAND = 'EXPAND'
    SCAN = 'SCAN'
    RETRACT = 'RETRACT'


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pitch_from_quaternion(q):
    return math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MotorStateMachine(Node):

    def __init__(self):
        super().__init__('motor_state_machine')
        self.declare_parameters('', [
            ('rolling_gap_cmd', 0.0),      # m, shells closed
            ('expanded_cmd', 0.01),        # m, shells open
            ('settle_time_s', 1.0),
            ('scan_step_deg', 30.0),
            ('scan_steps', 12),
            ('scan_hold_s', 0.5),          # 4 thermal frames at 8 Hz
            ('scan_yaw_rate', 0.5),        # rad/s
            ('shell_tolerance', 0.0005),   # m
            ('shell_timeout_s', 3.0),      # carry on (with a warning) if the shells stall
            ('yaw_tolerance_deg', 1.5),
            ('step_timeout_s', 10.0),      # carry on (with a warning) if a turn stalls
            ('drive_accel', 0.5),          # m/s^2, ramp on the teleop forward speed
            ('swing_damping', 0.5),        # m/s of forward command per rad of core swing; 0 = off
            ('swing_washout_s', 1.0),      # time constant of the slow pitch average
        ])
        if not self.get_parameter('use_sim_time').value:
            self.get_logger().warn(
                'use_sim_time is false; in simulation run with --ros-args -p use_sim_time:=true')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.shell_pub = self.create_publisher(Float64, '/shell/cmd', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/mission/state', latched)
        self.create_subscription(Twist, '/cmd_vel_teleop', self.on_teleop, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odometry, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_service(Trigger, '/mission/scan', self.on_scan)

        self.shell_pos = {}
        self.heading = None        # EKF yaw, unwrapped so a full turn reads 2 pi
        self.last_yaw = None
        self.swing = 0.0           # core pitch minus its slow average (rad)
        self.pitch_avg = None
        self.last_imu_t = None
        self.forward = 0.0         # ramped forward speed (m/s), before swing damping
        self.teleop = Twist()
        self.set_state(State.DRIVE)
        self.create_timer(LOOP_PERIOD_S, self.step)

    def param(self, name):
        return self.get_parameter(name).value

    def elapsed(self, since):
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    def set_state(self, state):
        self.state = state
        self.entered = self.get_clock().now()
        self.state_pub.publish(String(data=state.value))
        self.get_logger().info(f'state -> {state.value}')
        if state == State.SCAN:
            self.scan_start = self.heading
            self.scan_index = 1    # turning towards step 1
            self.holding = False
            self.phase_start = self.entered
        elif state == State.DRIVE:
            self.teleop = Twist()  # don't resume a teleop command from before the scan

    def on_teleop(self, msg):
        if self.state == State.DRIVE:
            self.teleop = msg

    def on_joint_states(self, msg):
        self.shell_pos.update(
            (name, pos) for name, pos in zip(msg.name, msg.position) if name in SHELL_JOINTS)

    def on_odometry(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.heading = yaw if self.heading is None else self.heading + wrap(yaw - self.last_yaw)
        self.last_yaw = yaw

    def on_imu(self, msg):
        pitch = pitch_from_quaternion(msg.orientation)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.pitch_avg is None or self.last_imu_t is None or t <= self.last_imu_t:
            self.pitch_avg = pitch
        else:
            alpha = min(1.0, (t - self.last_imu_t) / self.param('swing_washout_s'))
            self.pitch_avg += alpha * (pitch - self.pitch_avg)
        self.last_imu_t = t
        self.swing = pitch - self.pitch_avg

    def on_scan(self, request, response):
        if self.state != State.DRIVE:
            response.success, response.message = False, f'busy: {self.state.value}'
        elif self.heading is None:
            response.success, response.message = False, 'no /odometry/filtered yet'
        else:
            self.set_state(State.STOP)
            response.success, response.message = True, 'scan started'
        return response

    def shells_reached(self, target):
        tolerance = self.param('shell_tolerance')
        if all(abs(self.shell_pos.get(j, math.inf) - target) <= tolerance for j in SHELL_JOINTS):
            return True
        if self.elapsed(self.entered) > self.param('shell_timeout_s'):
            self.get_logger().warn(f'shells did not reach {target} m ({self.shell_pos}); continuing')
            return True
        return False

    def scan_turn_rate(self):
        """Turn rate for this loop of SCAN; moves on to the next step or to RETRACT."""
        if self.holding:
            if self.elapsed(self.phase_start) < self.param('scan_hold_s'):
                return 0.0
            if self.scan_index >= self.param('scan_steps'):
                self.set_state(State.RETRACT)
                return 0.0
            self.scan_index += 1
            self.holding = False
            self.phase_start = self.get_clock().now()

        target = self.scan_start + self.scan_index * math.radians(self.param('scan_step_deg'))
        error = target - self.heading
        stalled = self.elapsed(self.phase_start) > self.param('step_timeout_s')
        if abs(error) <= math.radians(self.param('yaw_tolerance_deg')) or stalled:
            if stalled:
                self.get_logger().warn(
                    f'scan step {self.scan_index} stopped {math.degrees(error):.1f} deg short')
            self.holding = True
            self.phase_start = self.get_clock().now()
            return 0.0
        braking = math.sqrt(2.0 * TURN_DECEL * abs(error))
        rate = min(self.param('scan_yaw_rate'), max(MIN_TURN_RATE, braking))
        return math.copysign(rate, error)

    def step(self):
        target_forward, turn = 0.0, 0.0
        if self.state == State.DRIVE:
            target_forward, turn = self.teleop.linear.x, self.teleop.angular.z
        elif self.state == State.STOP:
            if self.elapsed(self.entered) >= self.param('settle_time_s'):
                self.set_state(State.EXPAND)
        elif self.state == State.EXPAND:
            self.shell_pub.publish(Float64(data=float(self.param('expanded_cmd'))))
            if self.shells_reached(self.param('expanded_cmd')):
                self.set_state(State.SCAN)
        elif self.state == State.SCAN:
            turn = self.scan_turn_rate()
        elif self.state == State.RETRACT:
            self.shell_pub.publish(Float64(data=float(self.param('rolling_gap_cmd'))))
            if self.shells_reached(self.param('rolling_gap_cmd')):
                self.set_state(State.DRIVE)

        max_change = self.param('drive_accel') * LOOP_PERIOD_S
        self.forward += max(-max_change, min(max_change, target_forward - self.forward))
        twist = Twist()
        twist.linear.x = self.forward + self.param('swing_damping') * self.swing
        twist.angular.z = turn
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = MotorStateMachine()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
