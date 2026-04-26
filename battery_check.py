#!/usr/bin/env python3
"""Standalone battery voltage read. Do NOT run while the ROS driver has USB open."""
import xarm
arm = xarm.Controller("USB")
mv = arm.getBatteryVoltage()
print(f"Battery: {mv} mV ({mv/1000:.2f} V)")
