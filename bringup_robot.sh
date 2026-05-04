#!/usr/bin/env bash
# Robot-only: display + USB driver (no camera).
# Use for drag-teach, joint-state inspection, or gripper tuning.
exec "$(dirname "$0")/bringup_pick.sh" --profile robot "$@"
