#!/usr/bin/env python3
"""xArm 1S gripper CLI + library.

The xArm 1S parallel-jaw gripper is driven by a single actuated joint
`arm1`; the other three finger joints (arm0, arm0_left, arm1_left)
follow via URDF mimic directives. Commanding only arm1 closes/opens
both pads symmetrically.

Sends a single-joint JointTrajectory (just arm1) on /joint_trajectory
(the xarm_hw_driver subscribes to a relative `joint_trajectory` topic
under the global namespace). The driver picks it up via command_callback
and forwards to the servo.

CLI:
    ros2 run xarm_hw gripper open
    ros2 run xarm_hw gripper close
    ros2 run xarm_hw gripper move 0.5                # arm1 in radians
    ros2 run xarm_hw gripper move 0.5 --duration 2.0

Library (for the pick state machine):
    from xarm_hw.gripper import move_gripper
    move_gripper(node, target_rad=-1.0, duration_s=1.0)

Defaults:
    open  : arm1 = +1.5 rad   (jaws spread wide; near the +1.8326 limit)
    close : arm1 = -1.5 rad   (folded inward, matches SRDF 'home' pose)
For a 25 mm cube the closed-on-cube position will need empirical tuning;
start with `move -0.6` and adjust until the pads grip the cube.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM1_OPEN_RAD = 1.5
ARM1_CLOSED_RAD = -1.5

GRIPPER_TOPIC = '/joint_trajectory'


def move_gripper(
    node: Node,
    target_rad: float,
    duration_s: float = 1.0,
    wait_for_sub_s: float = 2.0,
) -> bool:
    """Publish a single-arm1 JointTrajectory; wait for the driver's
    subscription before sending, then spin briefly so the message
    actually leaves the queue.

    Returns True iff a subscriber was present and the publish ran."""
    pub = node.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)

    deadline = time.monotonic() + float(wait_for_sub_s)
    while time.monotonic() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)

    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f"no subscriber on {GRIPPER_TOPIC} — is xarm_hw_driver running?"
        )
        return False

    msg = JointTrajectory()
    msg.joint_names = ["arm1"]
    pt = JointTrajectoryPoint()
    pt.positions = [float(target_rad)]
    sec = int(duration_s)
    nanosec = int((float(duration_s) - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
    msg.points = [pt]
    pub.publish(msg)

    end = time.monotonic() + 0.4
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def main():
    p = argparse.ArgumentParser(description='xArm 1S gripper CLI.')
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('open', help=f'arm1 -> {ARM1_OPEN_RAD:+.2f} rad (extended)')
    sub.add_parser('close', help=f'arm1 -> {ARM1_CLOSED_RAD:+.2f} rad (folded)')
    move = sub.add_parser('move', help='arm1 -> POS rad')
    move.add_argument('pos', type=float, help='arm1 target in radians')
    p.add_argument('--duration', type=float, default=1.0,
                   help='time to reach target in seconds (default 1.0)')
    args = p.parse_args()

    if args.cmd == 'open':
        target = ARM1_OPEN_RAD
    elif args.cmd == 'close':
        target = ARM1_CLOSED_RAD
    else:
        target = float(args.pos)

    rclpy.init()
    node = Node('gripper_cli')
    try:
        ok = move_gripper(node, target, duration_s=float(args.duration))
        if ok:
            print(f'gripper: arm1 -> {target:+.3f} rad over {args.duration:.2f} s')
            return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
