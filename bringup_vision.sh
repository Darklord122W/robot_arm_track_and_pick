#!/usr/bin/env bash
# Vision-only: camera + marker (no robot).
# Use for tuning the camera pipeline or verifying ArUco detection
# without powering up the arm.
exec "$(dirname "$0")/bringup_pick.sh" --profile vision "$@"
