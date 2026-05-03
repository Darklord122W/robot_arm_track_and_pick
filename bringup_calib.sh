#!/usr/bin/env bash
# Calibration session: display + driver + camera + marker (no MoveIt).
# Use when running `ros2 run xarm_pick calibrate_homography`.
exec "$(dirname "$0")/bringup_pick.sh" --profile calib "$@"
