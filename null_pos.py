#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

rclpy.init()
node = Node('null_pos')
pub = node.create_publisher(JointTrajectory, '/joint_trajectory', 10)
time.sleep(0.5)

msg = JointTrajectory()
msg.joint_names = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5', 'arm6']
pt = JointTrajectoryPoint()
pt.positions = [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0]
pt.time_from_start = Duration(sec=2)
msg.points = [pt]

pub.publish(msg)
print("Sent null position")
time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
