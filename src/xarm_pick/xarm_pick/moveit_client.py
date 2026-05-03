"""MoveGroup action client — joint-space + pose-target planning.

Joint-space (`move_to_joints`): JointConstraints, no IK. Useful while
hand-eye calibration is in flux.

Pose-target (`move_to_pose`): position + orientation constraint on
`link` (default `tool0`); MoveIt's KDL plugin solves IK and plans a
joint-space path to it.
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


DEFAULT_JOINT_NAMES = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5', 'arm6']


class MoveGroupClient:
    def __init__(
        self,
        node: Node,
        group_name: str = 'arm',
        joint_names: Optional[Iterable[str]] = None,
        action_name: str = '/move_action',
        planning_time_s: float = 5.0,
        velocity_scaling: float = 0.30,
        acceleration_scaling: float = 0.30,
        execute_timeout_s: float = 60.0,
    ):
        self.node = node
        self.group_name = group_name
        self.joint_names = list(joint_names) if joint_names else list(DEFAULT_JOINT_NAMES)
        self.planning_time_s = float(planning_time_s)
        self.velocity_scaling = float(velocity_scaling)
        self.acceleration_scaling = float(acceleration_scaling)
        self.execute_timeout_s = float(execute_timeout_s)
        self._client = ActionClient(node, MoveGroup, action_name)

    def wait_for_server(self, timeout_s: float = 10.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_s)

    # ------------------------------------------------------------------
    def move_to_joints(
        self,
        joint_targets: Dict[str, float],
        joint_tolerance: float = 0.01,
        plan_only: bool = False,
    ) -> bool:
        constraints = Constraints()
        for name, value in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = float(joint_tolerance)
            jc.tolerance_below = float(joint_tolerance)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return self._send_constraints(constraints, plan_only=plan_only)

    # ------------------------------------------------------------------
    def move_to_pose(
        self,
        position: Tuple[float, float, float],
        orientation_quat: Tuple[float, float, float, float],
        frame: str = 'world',
        link: str = 'tool0',
        position_tolerance_m: float = 0.01,
        orientation_tolerance_rad: float = 0.20,
        free_yaw: bool = True,
        plan_only: bool = False,
    ) -> bool:
        """Plan + execute IK to (position, orientation_quat) for `link` in `frame`.

        orientation_quat is (x, y, z, w).

        free_yaw=True (default) allows free rotation about the link's local Z
        axis, leaving 5 effective constraints (3 pos + 2 orient). This is the
        right setting for a parallel-jaw gripper aligned with link Z — it's
        symmetric, so yaw doesn't matter. It's also necessary for the xArm 1S
        because it's a 5-DOF positioning arm (arm2..arm6) and can't satisfy
        full 6-DOF orientations in general.
        """
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(position_tolerance_m)]

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        region_pose = Pose()
        region_pose.position.x = float(position[0])
        region_pose.position.y = float(position[1])
        region_pose.position.z = float(position[2])
        region_pose.orientation.w = 1.0
        bv.primitive_poses.append(region_pose)
        pc.constraint_region = bv
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        oc.orientation.x = float(orientation_quat[0])
        oc.orientation.y = float(orientation_quat[1])
        oc.orientation.z = float(orientation_quat[2])
        oc.orientation.w = float(orientation_quat[3])
        oc.absolute_x_axis_tolerance = float(orientation_tolerance_rad)
        oc.absolute_y_axis_tolerance = float(orientation_tolerance_rad)
        # Yaw about the link's local Z is unconstrained by default — set to
        # ~2π so any rotation about Z passes the constraint.
        oc.absolute_z_axis_tolerance = (
            6.2832 if free_yaw else float(orientation_tolerance_rad)
        )
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        return self._send_constraints(constraints, plan_only=plan_only)

    # ------------------------------------------------------------------
    def _send_constraints(self, constraints: Constraints, plan_only: bool = False) -> bool:
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = self.planning_time_s
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.goal_constraints.append(constraints)
        request.start_state = RobotState()
        request.start_state.is_diff = True

        options = PlanningOptions()
        options.plan_only = bool(plan_only)
        options.replan = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node, send_future,
            timeout_sec=self.planning_time_s + 5.0,
        )
        if not send_future.done():
            self.node.get_logger().error('MoveGroup: send_goal timed out')
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('MoveGroup: goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=self.execute_timeout_s,
        )
        if not result_future.done():
            self.node.get_logger().error('MoveGroup: execution timed out')
            return False
        wrapped = result_future.result()
        if wrapped is None:
            self.node.get_logger().error('MoveGroup: empty result')
            return False
        result = wrapped.result
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f'MoveGroup: error_code={code} '
                f'(see moveit_msgs/MoveItErrorCodes)'
            )
            return False
        return True
