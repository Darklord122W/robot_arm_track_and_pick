"""xArm 1S pick state machine — joint-space waypoint demo.

v1: hardcoded joint waypoints. The user hand-tunes them so that
GRASP places the gripper around a cube on the table. The live 2D
homography pick is in pick_2d.py.

State sequence (each MOVE plans through MoveGroup over arm2..arm6;
each GRIPPER publishes a single-arm1 trajectory):

    HOME -> open -> PRE_GRASP -> GRASP -> grip
         -> LIFT -> PLACE -> open -> HOME

arm1 is NOT in the MoveIt planning group (the SRDF declares `arm`
as a chain base_link→tool0, which excludes the arm1 sibling branch).
arm1 is set by 'gripper' steps and persists across MOVE steps.

CLI:
    ros2 run xarm_pick pick                   # full sequence
    ros2 run xarm_pick pick --once HOME       # just go HOME
    ros2 run xarm_pick pick --once GRASP      # just go to GRASP
    ros2 run xarm_pick pick --grip-rad -1.5   # tune gripper grip
    ros2 run xarm_pick pick --skip-gripper    # MOVE-only dry run
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from xarm_hw.gripper import (
    ARM1_CLOSED_RAD,
    ARM1_OPEN_RAD,
    move_gripper,
)

from .moveit_client import MoveGroupClient


# Default grip closure for a 40 mm cube; tune via --grip-rad.
DEFAULT_GRIP_RAD = -1.5

# Joint-space waypoints (arm2..arm6 only — arm1 is the gripper master,
# controlled by the 'gripper' steps via xarm_hw.gripper, not by MoveIt).
# Reasonable starting values — hand-tune for your rig:
#   1. ros2 run xarm_pick pick --once HOME       # confirm HOME safe
#   2. drag-teach the arm to PRE_GRASP, read joint_states, edit here
#   3. repeat for GRASP / LIFT / PLACE
DEFAULT_POSES: Dict[str, Dict[str, float]] = {
    'HOME':      {'arm2': 0.0, 'arm3':  0.0, 'arm4':  0.0, 'arm5':  0.0, 'arm6':  0.0},
    'PRE_GRASP': {'arm2': 1.0, 'arm3':  0.5, 'arm4':  0.5, 'arm5':  0.0, 'arm6':  0.0},
    'GRASP':     {'arm2': 1.3, 'arm3':  0.8, 'arm4':  0.8, 'arm5':  0.0, 'arm6':  0.0},
    'LIFT':      {'arm2': 1.0, 'arm3':  0.5, 'arm4':  0.5, 'arm5':  0.0, 'arm6':  0.0},
    'PLACE':     {'arm2': 1.0, 'arm3':  0.5, 'arm4':  0.5, 'arm5':  0.0, 'arm6':  1.0},
}

# (step_type, name) — step_type in {'move', 'gripper'}.
SEQUENCE: List[Tuple[str, str]] = [
    ('move',    'HOME'),
    ('gripper', 'open'),
    ('move',    'PRE_GRASP'),
    ('move',    'GRASP'),
    ('gripper', 'grip'),
    ('move',    'LIFT'),
    ('move',    'PLACE'),
    ('gripper', 'open'),
    ('move',    'HOME'),
]


def resolve_pose(pose: Dict[str, float], grip_rad: float) -> Dict[str, float]:
    """Pass-through. arm1 is no longer in the planning group, so no
    joint-target patching is needed; grip_rad is unused here but kept
    for backward compat with the call sites."""
    del grip_rad  # explicitly unused
    return {joint: float(value) for joint, value in pose.items()}


def gripper_target(name: str, grip_rad: float) -> float:
    if name == 'open':
        return ARM1_OPEN_RAD
    if name == 'close':
        return ARM1_CLOSED_RAD
    if name == 'grip':
        return grip_rad
    raise ValueError(f'unknown gripper step: {name!r}')


def run_sequence(
    node: Node,
    mg: MoveGroupClient,
    sequence: List[Tuple[str, str]],
    poses: Dict[str, Dict[str, float]],
    grip_rad: float,
    skip_gripper: bool = False,
    settle_s: float = 0.5,
) -> bool:
    for step_type, name in sequence:
        node.get_logger().info(f'>>> {step_type} {name}')
        if step_type == 'move':
            target = resolve_pose(poses[name], grip_rad)
            if not mg.move_to_joints(target):
                node.get_logger().error(f'move to {name} failed; aborting')
                return False
        elif step_type == 'gripper':
            if skip_gripper:
                node.get_logger().info(f'(skip-gripper) would {name}')
            else:
                target = gripper_target(name, grip_rad)
                if not move_gripper(node, target, duration_s=1.0):
                    node.get_logger().error(f'gripper {name} failed; aborting')
                    return False
        else:
            node.get_logger().error(f'unknown step type {step_type!r}; aborting')
            return False
        time.sleep(settle_s)
    return True


def main():
    p = argparse.ArgumentParser(description='xArm 1S pick state machine.')
    p.add_argument('--once', metavar='POSE',
                   help='Skip the full sequence; just MOVE to this named pose')
    p.add_argument('--grip-rad', type=float, default=DEFAULT_GRIP_RAD,
                   help=f'arm1 angle (rad) used for "grip" step '
                        f'and for LIFT/PLACE while holding (default {DEFAULT_GRIP_RAD})')
    p.add_argument('--skip-gripper', action='store_true',
                   help='Skip gripper steps — useful for MOVE-only dry runs')
    p.add_argument('--settle-sec', type=float, default=0.5,
                   help='Seconds to pause between steps (default 0.5)')
    args = p.parse_args()

    rclpy.init()
    node = Node('pick_node')
    try:
        mg = MoveGroupClient(node)
        node.get_logger().info('Waiting for MoveGroup action server (/move_action)...')
        if not mg.wait_for_server(timeout_s=10.0):
            node.get_logger().error(
                'MoveGroup action server not available — is the move_group '
                'node running? Try: ros2 launch xarm_moveit_config '
                'xarm_1s_moveit.launch.py'
            )
            return 1
        node.get_logger().info('MoveGroup ready.')

        if args.once:
            if args.once not in DEFAULT_POSES:
                node.get_logger().error(
                    f'unknown pose {args.once!r}; available: {list(DEFAULT_POSES)}'
                )
                return 2
            target = resolve_pose(DEFAULT_POSES[args.once], args.grip_rad)
            return 0 if mg.move_to_joints(target) else 3

        ok = run_sequence(
            node, mg, SEQUENCE, DEFAULT_POSES,
            grip_rad=args.grip_rad,
            skip_gripper=args.skip_gripper,
            settle_s=args.settle_sec,
        )
        return 0 if ok else 4
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
