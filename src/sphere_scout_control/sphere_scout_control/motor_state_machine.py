"""motor_state_machine: drives the sphere and runs the scan sequence.

Owner: TODO(kickoff)

TODO: rclpy node `motor_state_machine`, use_sim_time: true
TODO: subscribes /cmd_vel_teleop (geometry_msgs/Twist), /joint_states (sensor_msgs/JointState)
TODO: publishes  /cmd_vel (geometry_msgs/Twist), /shell/cmd (std_msgs/Float64),
                 /mission/state (std_msgs/String)
TODO: service    /mission/scan (std_srvs/Trigger)
TODO: states DRIVE, STOP, EXPAND, SCAN, RETRACT
      - DRIVE forwards teleop to /cmd_vel
      - every other state publishes zero velocity, except SCAN's rotation steps
TODO: parameters rolling_gap_cmd, expanded_cmd, settle_time_s (1.0), scan_step_deg,
      scan_steps, scan_hold_s, scan_yaw_rate
      (defaults other than settle_time_s: TODO(kickoff))
TODO: state enum, transitions, timer loop, publishers. Transition conditions can be simple
      timers or joint-position checks.
"""


def main():
    # TODO: implement
    pass
