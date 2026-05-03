#!/usr/bin/env bash
# Robot + MoveIt (no camera): display + driver + MoveIt.
# Use for joint-space planning experiments or executing canned poses
# (e.g. `ros2 run xarm_pick pick`) without needing the camera up.
exec "$(dirname "$0")/bringup_pick.sh" --profile moveit "$@"
