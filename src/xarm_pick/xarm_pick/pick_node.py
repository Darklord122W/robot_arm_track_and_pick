"""xArm 1S pick state machine — joint-space waypoint demo.

v1: hardcoded joint waypoints. The user hand-tunes them so that
GRASP places the gripper around a cube on the table. The live 2D
homography pick is in pick_2d.py.

State sequence (each MOVE goes through the local trajectory generator
over arm2..arm6; each GRIPPER publishes a single-arm1 trajectory):

    HOME -> open -> PRE_GRASP -> GRASP -> grip
         -> LIFT -> PLACE -> open -> HOME

arm1 is the gripper master and is driven separately by xarm_hw.gripper;
it is not part of the planning chain.

CLI:
    ros2 run xarm_pick pick                   # full sequence
    ros2 run xarm_pick pick --once HOME       # just go HOME
    ros2 run xarm_pick pick --once GRASP      # just go to GRASP
    ros2 run xarm_pick pick --grip-rad -1.5   # tune gripper grip
    ros2 run xarm_pick pick --skip-gripper    # MOVE-only dry run
    ros2 run xarm_pick pick --method quintic  # smoother profile
"""
from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from xarm_hw.gripper import (
    ARM1_CLOSED_RAD,
    ARM1_OPEN_RAD,
    move_gripper,
)

from .local_traj_client import LocalTrajectoryClient


# Default grip closure for a 40 mm cube; tune via --grip-rad.
DEFAULT_GRIP_RAD = -1.5

# Joint-space waypoints (arm2..arm6 only — arm1 is the gripper master,
# controlled by the 'gripper' steps via xarm_hw.gripper).
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


class _JointStateNode(Node):
    """Tiny rclpy node that just tracks the latest /joint_states.

    LocalTrajectoryClient.move_to_joints needs the current joint
    positions as the trajectory's start point; we subscribe here so
    every MOVE step can read the freshest values without each call
    re-creating its own subscription.
    """

    def __init__(self):
        super().__init__('pick_node')
        self._latest: Dict[str, float] = {}
        self.create_subscription(JointState, '/joint_states', self._cb, 10)

    def _cb(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self._latest[n] = float(p)

    def current_q(self, joint_names: List[str]) -> Optional[Dict[str, float]]:
        if not all(n in self._latest for n in joint_names):
            return None
        return {n: self._latest[n] for n in joint_names}


def gripper_target(name: str, grip_rad: float) -> float:
    if name == 'open':
        return ARM1_OPEN_RAD
    if name == 'close':
        return ARM1_CLOSED_RAD
    if name == 'grip':
        return grip_rad
    raise ValueError(f'unknown gripper step: {name!r}')


def _wait_for_q(node: _JointStateNode, joint_names: List[str],
                timeout_s: float = 2.0) -> Optional[Dict[str, float]]:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        q = node.current_q(joint_names)
        if q is not None:
            return q
    return None


def run_sequence(
    node: _JointStateNode,
    client: LocalTrajectoryClient,
    sequence: List[Tuple[str, str]],
    poses: Dict[str, Dict[str, float]],
    grip_rad: float,
    skip_gripper: bool = False,
    settle_s: float = 0.5,
) -> bool:
    for step_type, name in sequence:
        node.get_logger().info(f'>>> {step_type} {name}')
        if step_type == 'move':
            q_start = _wait_for_q(node, client.joint_names)
            if q_start is None:
                node.get_logger().error(
                    'no /joint_states received — is xarm_hw_driver running?'
                )
                return False
            if not client.move_to_joints(q_start, poses[name]):
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
                        f'(default {DEFAULT_GRIP_RAD})')
    p.add_argument('--skip-gripper', action='store_true',
                   help='Skip gripper steps — useful for MOVE-only dry runs')
    p.add_argument('--settle-sec', type=float, default=0.5,
                   help='Seconds to pause between steps (default 0.5)')
    p.add_argument('--method', default='trapezoid',
                   choices=['cubic', 'quintic', 'lspb', 'trapezoid'],
                   help='Trajectory profile (Craig §7 / MR §9.4). '
                        'trapezoid (default) is time-optimal.')
    args = p.parse_args()

    rclpy.init()
    node = _JointStateNode()
    try:
        client = LocalTrajectoryClient(node, method=args.method)
        node.get_logger().info(
            f'Waiting for FollowJointTrajectory action server '
            f'(method={args.method})...'
        )
        if not client.wait_for_server(timeout_s=10.0):
            node.get_logger().error(
                'FollowJointTrajectory action server not available — '
                'is xarm_hw_driver running?'
            )
            return 1
        node.get_logger().info('Controller ready.')

        if args.once:
            if args.once not in DEFAULT_POSES:
                node.get_logger().error(
                    f'unknown pose {args.once!r}; available: {list(DEFAULT_POSES)}'
                )
                return 2
            q_start = _wait_for_q(node, client.joint_names)
            if q_start is None:
                node.get_logger().error('no /joint_states received')
                return 5
            return 0 if client.move_to_joints(q_start, DEFAULT_POSES[args.once]) else 3

        ok = run_sequence(
            node, client, SEQUENCE, DEFAULT_POSES,
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
