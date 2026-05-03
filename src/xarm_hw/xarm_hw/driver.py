import math
import time
import threading

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.action import FollowJointTrajectory

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32
from std_srvs.srv import SetBool

import xarm  # pip xarm library for the robot
import inspect


class XArmHardwareDriver(Node):
    def __init__(self):
        super().__init__('xarm_hardware_driver')

        # ------------------ hardware connection ------------------
        self.arm = xarm.Controller("USB")
        self.get_logger().info("Connected to xArm over USB")
        self.get_logger().info("=== XArmHardwareDriver VERSION 2 (MoveIt Action) ===")
        self.get_logger().info(
            f"driver.py loaded from: {inspect.getfile(XArmHardwareDriver)}"
        )

        # Tunable peak velocity used by the zero-time-trajectory fallback below.
        # Override at launch with: --ros-args -p fallback_peak_velocity_rad_s:=0.5
        self.declare_parameter('fallback_peak_velocity_rad_s', 0.3)
        self.fallback_peak_v = float(
            self.get_parameter('fallback_peak_velocity_rad_s').value
        )
        self.get_logger().info(
            f"Fallback peak velocity (used when MoveIt sends zero-time traj): "
            f"{self.fallback_peak_v:.3f} rad/s"
        )

        # ROS joint names from URDF / SRDF / MoveIt
        self.joint_names = ["arm1", "arm2", "arm3", "arm4", "arm5", "arm6"]

        # Mapping from ROS joint name to servo ID on the real robot
        # Adjust these if your physical mapping is different.
        self.joint_to_servo = {
            "arm1": 1,
            "arm2": 2,
            "arm3": 3,
            "arm4": 4,
            "arm5": 5,
            "arm6": 6,
        }

        self.joint_direction = {
            "arm1": -1,
            "arm2":  1,
            "arm3":  1,
            "arm4":  1,
            "arm5":  1,
            "arm6":  1,
        }

        # Serialize all USB reads/writes — MultiThreadedExecutor runs timer
        # and action callbacks concurrently, so without this lock they collide.
        self._usb_lock = threading.Lock()

        # Read hardware every Nth timer tick (20 Hz timer / 10 = 2 Hz reads).
        # Keeps the USB bus free for setPosition during trajectory execution.
        self._read_counter = 0
        self._read_every_n = 10

        # Internal state in radians (our current estimate of joint angles)
        self.current_positions = [0.0] * len(self.joint_names)

        # ------------------ (optional) topic subscriber ------------------
        # Still keep the /joint_trajectory subscriber for manual testing,
        # but MoveIt will not use this; it will use the ActionServer below.
        self.cmd_sub = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.command_callback,
            10
        )

        # ------------------ joint state publisher ------------------
        self.js_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Periodic feedback timer (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # ------------------ battery voltage publisher ------------------
        self.batt_pub = self.create_publisher(Float32, 'xarm/battery_voltage', 10)
        self.create_timer(2.0, self.publish_battery_voltage)

        # ------------------ torque enable/disable service ------------------
        # True  -> re-hold current position (re-engages torque via setPosition)
        # False -> servoOff() on all joints (arm goes limp)
        self.torque_srv = self.create_service(
            SetBool, 'xarm/set_torque', self.set_torque_cb
        )

        # ------------------ FollowJointTrajectory Action Server ------------------
        # This name must match moveit_controllers.yaml:
        #   xarm_1s_arm_controller + action_ns: follow_joint_trajectory
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'xarm_1s_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    # ======================================================================
    #  Conversion helpers
    # ======================================================================

    def rad_to_units(self, rad: float) -> int:
        """Convert joint angle in radians to servo units (0–1000)."""
        deg = rad * 180.0 / math.pi
        # clamp to servo mechanical range
        deg = max(-90.0, min(90.0, deg))

        # 0 deg = 500 units, 1 deg = 4 units
        units = 500 + int(deg * 4.0)

        # extra clamp for safety
        units = max(100, min(900, units))
        return units

    def units_to_rad(self, units: int) -> float:
        """Inverse of rad_to_units: servo units (0–1000) to radians."""
        units = max(100, min(900, units))
        deg = (units - 500) / 4.0
        rad = deg * math.pi / 180.0
        return rad

    # ======================================================================
    #  Low-level command helper
    # ======================================================================

    def send_joint_positions(self, positions_rad, duration_ms: int):
        """
        Send target joint positions (in radians) to the robot over HID.

        positions_rad: list of len(self.joint_names), in joint_names order.
        duration_ms: how long the move should take, in milliseconds.
        """
        servo_cmds = []
        for idx, pos_rad in enumerate(positions_rad):
            joint_name = self.joint_names[idx]
            servo_id = self.joint_to_servo[joint_name]
            direction = self.joint_direction.get(joint_name, 1)
            units = self.rad_to_units(pos_rad * direction)
            servo_cmds.append([servo_id, units])

        try:
            with self._usb_lock:
                self.arm.setPosition(servo_cmds, duration=duration_ms, wait=False)
            self.current_positions = list(positions_rad)
        except Exception as e:
            self.get_logger().error(f"Failed to send HID command: {e}")

    # ======================================================================
    #  /joint_trajectory topic callback  (manual / legacy usage)
    # ======================================================================

    def command_callback(self, msg: JointTrajectory):
        """
        Receive a trajectory on /joint_trajectory (manual testing + gripper).
        Accepts a subset of joint_names — joints not listed retain their
        current value (used by the gripper CLI to command arm1 alone).
        Uses the final point's time_from_start as the move duration;
        falls back to 500 ms when the field is zero.
        MoveIt uses the FollowJointTrajectory action below, not this path.
        """
        if not msg.points:
            return

        final_point = msg.points[-1]

        name_to_index = {name: i for i, name in enumerate(self.joint_names)}
        target = list(self.current_positions)

        for name, pos in zip(msg.joint_names, final_point.positions):
            if name not in name_to_index:
                self.get_logger().warn(f"Unknown joint name in command: {name}")
                continue
            idx = name_to_index[name]
            target[idx] = pos

        t = final_point.time_from_start
        duration_s = t.sec + t.nanosec * 1e-9
        duration_ms = int(duration_s * 1000) if duration_s > 1e-6 else 500
        self.send_joint_positions(target, duration_ms=duration_ms)

    # ======================================================================
    #  FollowJointTrajectory Action callbacks (used by MoveIt)
    # ======================================================================

    def goal_callback(self, goal_request):
        # Always accept goals from MoveIt
        self.get_logger().info('Received new FollowJointTrajectory goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received for trajectory')
        return CancelResponse.ACCEPT

    def execute_trajectory_cb(self, goal_handle):
        """
        Execute a FollowJointTrajectory goal from MoveIt.
        We step through each point in order, sending commands with appropriate timing.
        """
        self.get_logger().info('Executing trajectory from MoveIt')

        traj = goal_handle.request.trajectory

        if not traj.points:
            self.get_logger().warn('Trajectory has no points')
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        # --- DIAGNOSTIC: dump trajectory shape so we can see what speed MoveIt sent ---
        last_t = traj.points[-1].time_from_start
        last_dur_s = last_t.sec + last_t.nanosec * 1e-9
        first_t = traj.points[0].time_from_start
        first_dur_s = first_t.sec + first_t.nanosec * 1e-9
        first_pos = list(traj.points[0].positions)
        last_pos = list(traj.points[-1].positions)
        max_delta = max(abs(b - a) for a, b in zip(first_pos, last_pos)) if first_pos and last_pos else 0.0
        avg_v = (max_delta / last_dur_s) if last_dur_s > 1e-6 else 0.0
        self.get_logger().info(
            f"TRAJ_DIAG: points={len(traj.points)} "
            f"first_t={first_dur_s:.3f}s last_t={last_dur_s:.3f}s "
            f"largest_joint_delta={max_delta:.4f}rad avg_peak_v~{avg_v:.4f}rad/s"
        )
        # --- END DIAGNOSTIC ---

        # Fallback: if MoveIt didn't time-parameterize the trajectory (last_t == 0),
        # synthesise per-segment timing here at a fixed target peak velocity. Without
        # this the loop below sees segment_dt=0 on every point and races the servo
        # at full speed regardless of any RViz/YAML scaling.
        TARGET_PEAK_V = self.fallback_peak_v
        MIN_SEG_S = 0.05      # never go below 50 ms per segment
        if last_dur_s < 1e-6 and len(traj.points) > 1:
            cumulative = 0.0
            traj.points[0].time_from_start.sec = 0
            traj.points[0].time_from_start.nanosec = 0
            for j in range(1, len(traj.points)):
                prev = list(traj.points[j - 1].positions)
                curr = list(traj.points[j].positions)
                seg_max_delta = max(abs(b - a) for a, b in zip(prev, curr)) if prev and curr else 0.0
                seg_dt = max(seg_max_delta / TARGET_PEAK_V, MIN_SEG_S)
                cumulative += seg_dt
                traj.points[j].time_from_start.sec = int(cumulative)
                traj.points[j].time_from_start.nanosec = int(
                    (cumulative - int(cumulative)) * 1e9
                )
            self.get_logger().warn(
                f"Trajectory had zero time_from_start (TOTG didn't run). "
                f"Recomputed timing: total={cumulative:.2f}s at peak_v={TARGET_PEAK_V:.3f}rad/s."
            )

        # Map from our joint_names to indices in traj.joint_names
        index_in_msg = {}
        for j_name in self.joint_names:
            if j_name in traj.joint_names:
                index_in_msg[j_name] = traj.joint_names.index(j_name)
            else:
                self.get_logger().warn(f'Joint {j_name} not found in trajectory.joint_names')

        start_time = time.time()
        prev_t = 0.0

        for i, point in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Trajectory execution canceled')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # Desired time from start for this point
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

            # Build full target vector in our joint order
            target = list(self.current_positions)
            for j_index, j_name in enumerate(self.joint_names):
                if j_name in index_in_msg and index_in_msg[j_name] < len(point.positions):
                    msg_idx = index_in_msg[j_name]
                    target[j_index] = point.positions[msg_idx]
                else:
                    # If missing, keep previous value
                    pass

            # Compute move duration for this segment
            segment_dt = max(t - prev_t, 0.2)  # at least 200 ms per segment
            duration_ms = int(segment_dt * 1000)

            self.get_logger().debug(
                f"Point {i}: t={t:.3f}s, segment_dt={segment_dt:.3f}s, duration_ms={duration_ms}"
            )

            # Send positions for this segment
            self.send_joint_positions(target, duration_ms)

            # Sleep until the target time-from-start (best effort)
            while True:
                elapsed = time.time() - start_time
                if elapsed >= t:
                    break
                time.sleep(0.001)

            prev_t = t

        self.get_logger().info('Trajectory execution finished successfully')
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    # ======================================================================
    #  Battery voltage + torque control
    # ======================================================================

    def publish_battery_voltage(self):
        try:
            with self._usb_lock:
                mv = self.arm.getBatteryVoltage()
            volts = float(mv) / 1000.0
            msg = Float32()
            msg.data = volts
            self.batt_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(
                f'Battery voltage read failed: {e}',
                throttle_duration_sec=5.0,
            )

    def set_torque_cb(self, request, response):
        if request.data:
            # Re-engage torque by commanding every servo to its last known position.
            try:
                self.send_joint_positions(self.current_positions, duration_ms=300)
                response.success = True
                response.message = 'Torque re-engaged at current position'
            except Exception as e:
                response.success = False
                response.message = f'Failed to re-engage torque: {e}'
        else:
            errors = []
            for joint_name in self.joint_names:
                servo_id = self.joint_to_servo[joint_name]
                try:
                    with self._usb_lock:
                        self.arm.servoOff(servo_id)
                except Exception as e:
                    errors.append(f'servo {servo_id}: {e}')
            if errors:
                response.success = False
                response.message = 'Errors: ' + '; '.join(errors)
            else:
                response.success = True
                response.message = 'All servos torque-off (arm is limp)'
        self.get_logger().info(response.message)
        return response

    # ======================================================================
    #  Joint state publishing (feedback)
    # ======================================================================

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)

        self._read_counter += 1
        if self._read_counter >= self._read_every_n:
            self._read_counter = 0
            try:
                actual = []
                for joint_name in self.joint_names:
                    servo_id = self.joint_to_servo[joint_name]
                    direction = self.joint_direction.get(joint_name, 1)
                    with self._usb_lock:
                        units = self.arm.getPosition(servo_id)
                    actual.append(self.units_to_rad(units) * direction)
                self.current_positions = actual
            except Exception as e:
                self.get_logger().warn(
                    f'Could not read servo positions, using last commanded: {e}',
                    throttle_duration_sec=2.0
                )

        js.position = list(self.current_positions)

        self.js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = XArmHardwareDriver()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
11