"""FollowJointTrajectory client that bypasses MoveIt.

Pairs with `trajectory.py` to deliver classical trajectories
(Craig §7 / MR §9.2 / §9.4) directly to the controller's
FollowJointTrajectory action — no OMPL planner, no MoveIt
post-processing, no TOTG. The geometric path is a straight line in
joint space and the time scaling comes from one of the four profiles
in `trajectory.py`.

This is appropriate for the xArm 1S tabletop pick-and-place workflow
because:

  • IK is solved upfront in `arm_ik.solve_ik`, so the path-planning
    step in MoveIt is redundant — we already have the goal joints.
  • The pick state machine moves through joint-space configurations
    (HOME → PRE_GRASP → GRASP → LIFT → ...) chosen by IK to be
    collision-free given the workspace, so OMPL has nothing to
    contribute on this rig.
  • Cutting MoveIt out of the loop removes a source of failure modes
    (TOTG silent zero-time output, plugin name mismatches,
    AddRuckigTrajectorySmoothing degenerate trajectories — all
    documented in this codebase's history).
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory

from . import trajectory


# Default joint order — must match the FollowJointTrajectory controller's
# `joints:` list in moveit_controllers.yaml (arm2..arm6, the planning
# group). arm1 is the gripper master and is driven separately by
# xarm_hw.gripper.move_gripper, so it's excluded here.
DEFAULT_JOINT_NAMES = ['arm2', 'arm3', 'arm4', 'arm5', 'arm6']


class LocalTrajectoryClient:
    """FollowJointTrajectory client driving classical profiles.

    Parameters
    ----------
    node :
        rclpy node used for the action client.
    joint_names :
        Order in which positions / velocities / accelerations are sent.
        Must include every joint the controller expects; extra names
        are tolerated by the driver (it indexes by name).
    action_name :
        Defaults to the controller's action namespace; matches the
        topic xarm_hw publishes.
    method :
        Default trajectory profile — one of "cubic", "quintic", "lspb",
        "trapezoid". Overridable per call to `move_to_joints`.
    v_max / a_max :
        Per-joint kinematic limits in rad/s and rad/s². Same shape and
        order as `joint_names`. Conservative defaults match
        joint_limits.yaml (0.3 / 0.3 with the global 0.45 / 0.30
        scaling already baked in here so callers don't double-apply).
    dt :
        Sample period in seconds — the controller's segment cadence.
    """

    def __init__(
        self,
        node: Node,
        joint_names: Optional[Iterable[str]] = None,
        action_name: str = '/xarm_1s_arm_controller/follow_joint_trajectory',
        method: str = 'trapezoid',
        v_max: Optional[Sequence[float]] = None,
        a_max: Optional[Sequence[float]] = None,
        dt: float = 0.02,
        safety_margin: float = 1.05,
        execute_timeout_s: float = 60.0,
    ):
        self.node = node
        self.joint_names = list(joint_names) if joint_names else list(DEFAULT_JOINT_NAMES)
        self.method = str(method)
        self.dt = float(dt)
        self.safety_margin = float(safety_margin)
        self.execute_timeout_s = float(execute_timeout_s)

        n = len(self.joint_names)
        # Defaults: 0.45 · 0.3 = 0.135 rad/s peak per joint, matching the
        # effective limit when MoveIt's scaling is applied to
        # joint_limits.yaml. Same for accel.
        if v_max is None:
            v_max = [0.135] * n
        if a_max is None:
            a_max = [0.090] * n
        if not (len(v_max) == len(a_max) == n):
            raise ValueError('v_max / a_max must match joint_names length')
        self.v_max = [float(x) for x in v_max]
        self.a_max = [float(x) for x in a_max]

        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._action_name = action_name

    # ------------------------------------------------------------------
    def wait_for_server(self, timeout_s: float = 10.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_s)

    # ------------------------------------------------------------------
    def build_trajectory(
        self,
        q_start_map: Dict[str, float],
        q_goal_map: Dict[str, float],
        method: Optional[str] = None,
        duration: Optional[float] = None,
        blend_frac: float = 1.0 / 3.0,
    ):
        """Compose a `JointTrajectory` message from start and goal dicts.

        Returns the unsent ROS message — useful for dry-run
        (`--no-execute`) inspection or unit tests. Use
        `move_to_joints` to actually run it.
        """
        q_s = [float(q_start_map[j]) for j in self.joint_names]
        q_g = [float(q_goal_map[j])  for j in self.joint_names]

        traj = trajectory.build(
            method or self.method, q_s, q_g, self.v_max, self.a_max,
            duration=duration, blend_frac=blend_frac,
            safety_margin=self.safety_margin,
        )
        samples = traj.discretise(dt=self.dt)
        msg = trajectory.to_joint_trajectory_msg(samples, self.joint_names)
        return msg, traj.duration, len(samples)

    # ------------------------------------------------------------------
    def move_to_joints(
        self,
        q_start_map: Dict[str, float],
        q_goal_map: Dict[str, float],
        method: Optional[str] = None,
        duration: Optional[float] = None,
        blend_frac: float = 1.0 / 3.0,
    ) -> bool:
        """Send a single point-to-point trajectory to the controller.

        Caller supplies the *current* joint state explicitly via
        `q_start_map` so we don't have to subscribe here. (`pick_2d`
        already subscribes to /joint_states for branch-pinning.)
        """
        msg, total_T, n_pts = self.build_trajectory(
            q_start_map, q_goal_map, method=method,
            duration=duration, blend_frac=blend_frac,
        )

        method_used = method or self.method
        self.node.get_logger().info(
            f'LocalTraj[{method_used}]: T={total_T:.2f}s, '
            f'{n_pts} pts, dt={self.dt:.3f}s'
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node, send_future, timeout_sec=5.0,
        )
        if not send_future.done():
            self.node.get_logger().error('LocalTraj: send_goal timed out')
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('LocalTraj: goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future,
            timeout_sec=max(self.execute_timeout_s, total_T + 5.0),
        )
        if not result_future.done():
            self.node.get_logger().error('LocalTraj: execution timed out')
            return False
        wrapped = result_future.result()
        if wrapped is None:
            self.node.get_logger().error('LocalTraj: empty result')
            return False
        return True
