#!/usr/bin/env bash
# Bring up the xArm 1S stack in tmux. One pane per component.
#
# Components (each can be toggled independently):
#   display  T1  ros2 launch xarm_hw xarm_hw_display.launch.py
#                ↳ robot_state_publisher + RViz
#                  (URDF + TF chain world → ... → tool0)
#   driver   T2  ros2 run    xarm_hw xarm_hw_driver
#                ↳ USB driver, publishes /joint_states
#   camera   T3  ros2 launch astra_camera astra_pro_tuned.launch.py
#                ↳ Astra Pro RGB-D, /camera/color/* and /camera/depth/*
#   marker   T4  ros2 launch charuco_tf_publisher charuco_tf.launch.py
#                ↳ ArUco tracker; TF camera_color_optical_frame → handeye_target
#                  Override: --marker-id N --marker-length L --dictionary D
#   moveit   T5  ros2 launch xarm_moveit_config xarm_1s_moveit.launch.py
#                ↳ MoveIt move_group action server
#
# Profiles (preset combinations):
#   pick      (default)  display + driver + camera + marker + moveit
#   calib                display + driver + camera + marker
#   vision               camera  + marker
#   robot                display + driver
#   moveit               display + driver + moveit
#
# Usage:
#   ./bringup_pick.sh                              # pick profile (full stack)
#   ./bringup_pick.sh --profile calib              # calibration session
#   ./bringup_pick.sh --profile vision             # camera + marker only
#   ./bringup_pick.sh --profile robot              # display + driver only
#   ./bringup_pick.sh --profile moveit             # robot + MoveIt, no camera
#   ./bringup_pick.sh --no-moveit                  # legacy alias for calib
#   ./bringup_pick.sh --marker-id 5 --marker-length 0.030 \\
#                     --dictionary DICT_5X5_50     # custom marker
#   ./bringup_pick.sh --attach                     # bring up + attach
#   ./bringup_pick.sh --status                     # is a session running?
#   ./bringup_pick.sh --kill                       # tear down

set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/xarm_moveit}"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE/install/setup.bash"
SESSION_NAME="xarm_pick_bringup"

ACTION="up"
ATTACH_AFTER=0
PROFILE="pick"

# Per-component on/off flags (set by profile, can be overridden).
WITH_DISPLAY=1
WITH_DRIVER=1
WITH_CAMERA=1
WITH_MARKER=1
WITH_MOVEIT=1

# Marker config (only used when WITH_MARKER=1).
T4_MODE="single_aruco"
T4_MARKER_ID=2
T4_MARKER_LENGTH=0.030
T4_DICTIONARY="DICT_5X5_50"

apply_profile() {
    case "$1" in
        pick)   WITH_DISPLAY=1; WITH_DRIVER=1; WITH_CAMERA=1; WITH_MARKER=1; WITH_MOVEIT=1 ;;
        calib)  WITH_DISPLAY=1; WITH_DRIVER=1; WITH_CAMERA=1; WITH_MARKER=1; WITH_MOVEIT=0 ;;
        vision) WITH_DISPLAY=0; WITH_DRIVER=0; WITH_CAMERA=1; WITH_MARKER=1; WITH_MOVEIT=0 ;;
        robot)  WITH_DISPLAY=1; WITH_DRIVER=1; WITH_CAMERA=0; WITH_MARKER=0; WITH_MOVEIT=0 ;;
        moveit) WITH_DISPLAY=1; WITH_DRIVER=1; WITH_CAMERA=0; WITH_MARKER=0; WITH_MOVEIT=1 ;;
        *) echo "unknown profile: $1 (valid: pick|calib|vision|robot|moveit)" >&2; exit 2 ;;
    esac
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)         PROFILE="$2"; apply_profile "$2"; shift 2 ;;
        --kill)            ACTION="kill"; shift ;;
        --status)          ACTION="status"; shift ;;
        --attach)          ATTACH_AFTER=1; shift ;;
        # Legacy + fine-grained component toggles (override profile).
        --no-moveit)       WITH_MOVEIT=0; shift ;;
        --moveit)          WITH_MOVEIT=1; shift ;;
        --no-display)      WITH_DISPLAY=0; shift ;;
        --no-driver)       WITH_DRIVER=0; shift ;;
        --no-camera)       WITH_CAMERA=0; shift ;;
        --no-marker)       WITH_MARKER=0; shift ;;
        --marker-id)       T4_MARKER_ID="$2"; shift 2 ;;
        --marker-length)   T4_MARKER_LENGTH="$2"; shift 2 ;;
        --dictionary)      T4_DICTIONARY="$2"; shift 2 ;;
        --mode)            T4_MODE="$2"; shift 2 ;;
        -h|--help) sed -n '2,/^set -e/p' "$0" | sed 's/^# \?//' | head -n -1; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is required. Install with: sudo apt install tmux" >&2
    exit 1
fi

case "$ACTION" in
    kill)
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null \
            && echo "killed tmux session '$SESSION_NAME'" \
            || echo "no session named '$SESSION_NAME' was running"
        exit 0
        ;;
    status)
        if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
            echo "session '$SESSION_NAME' is RUNNING. Attach: tmux attach -t $SESSION_NAME"
            tmux list-panes -t "$SESSION_NAME" -a -F '  #{pane_index}: #{pane_current_command}'
        else
            echo "session '$SESSION_NAME' is NOT running"
        fi
        exit 0
        ;;
esac

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "session '$SESSION_NAME' already exists." >&2
    echo "  attach: $0 --attach" >&2
    echo "  kill:   $0 --kill" >&2
    exit 1
fi

[[ -f "$ROS_DISTRO_SETUP" ]] || { echo "missing $ROS_DISTRO_SETUP" >&2; exit 1; }
[[ -f "$WORKSPACE_SETUP" ]]  || { echo "missing $WORKSPACE_SETUP — run 'colcon build' first" >&2; exit 1; }

ENV_SOURCE="source $ROS_DISTRO_SETUP && source $WORKSPACE_SETUP"

# Build the ordered list of (label, sleep, cmd) tuples for enabled components.
LABELS=(); SLEEPS=(); CMDS=()
add_pane() {
    LABELS+=("$1"); SLEEPS+=("$2"); CMDS+=("$3")
}

[[ $WITH_DISPLAY -eq 1 ]] && add_pane \
    "T1 xarm_hw_display (robot_state_publisher + RViz)" 0 \
    "ros2 launch xarm_hw xarm_hw_display.launch.py"

[[ $WITH_DRIVER -eq 1 ]] && add_pane \
    "T2 xarm_hw_driver (USB)" 5 \
    "ros2 run xarm_hw xarm_hw_driver"

[[ $WITH_CAMERA -eq 1 ]] && add_pane \
    "T3 astra_pro camera" 0 \
    "ros2 launch astra_camera astra_pro_tuned.launch.py"

if [[ $WITH_MARKER -eq 1 ]]; then
    T4_ARGS="mode:=$T4_MODE marker_id:=$T4_MARKER_ID marker_length:=$T4_MARKER_LENGTH dictionary:=$T4_DICTIONARY"
    add_pane \
        "T4 charuco_tf_publisher (mode=$T4_MODE id=$T4_MARKER_ID len=$T4_MARKER_LENGTH dict=$T4_DICTIONARY)" 8 \
        "ros2 launch charuco_tf_publisher charuco_tf.launch.py $T4_ARGS"
fi

[[ $WITH_MOVEIT -eq 1 ]] && add_pane \
    "T5 MoveIt + RViz" 6 \
    "ros2 launch xarm_moveit_config xarm_1s_moveit.launch.py"

if [[ ${#LABELS[@]} -eq 0 ]]; then
    echo "no components enabled — nothing to do" >&2
    exit 2
fi

send_pane() {
    local pane="$1" label="$2" sleep_sec="$3" cmd="$4"
    tmux send-keys -t "$pane" \
        "$ENV_SOURCE; printf '\\n=== %s ===\\n' '$label'; sleep $sleep_sec; $cmd; echo '*** $label exited ***'; exec bash" C-m
}

# Create the first pane via new-session, then split-window for each subsequent.
tmux new-session -d -s "$SESSION_NAME" -n bringup -x 220 -y 60
send_pane "$SESSION_NAME:0.0" "${LABELS[0]}" "${SLEEPS[0]}" "${CMDS[0]}"

for ((i=1; i<${#LABELS[@]}; i++)); do
    tmux split-window -t "$SESSION_NAME:0"
    tmux select-layout -t "$SESSION_NAME:0" tiled >/dev/null
    pane="$SESSION_NAME:0.$i"
    send_pane "$pane" "${LABELS[$i]}" "${SLEEPS[$i]}" "${CMDS[$i]}"
done

tmux select-layout -t "$SESSION_NAME:0" tiled
tmux select-pane -t "$SESSION_NAME:0.0"

cat <<EOF

Started tmux session '$SESSION_NAME' [profile=$PROFILE] with ${#LABELS[@]} panes:
EOF
for label in "${LABELS[@]}"; do printf '  - %s\n' "$label"; done

if [[ $WITH_MARKER -eq 1 ]]; then
    cat <<EOF

T4 marker: mode=$T4_MODE  marker_id=$T4_MARKER_ID  length=${T4_MARKER_LENGTH}m  dict=$T4_DICTIONARY
EOF
fi

cat <<EOF

Wait ~15 s for stacks to settle. Sanity-check from another sourced terminal:
EOF
[[ $WITH_DRIVER -eq 1 ]] && echo "  ros2 topic hz /joint_states                                            # ~20 Hz"
[[ $WITH_CAMERA -eq 1 ]] && echo "  ros2 topic hz /camera/color/image_raw                                  # ~30 Hz"
[[ $WITH_MARKER -eq 1 ]] && echo "  ros2 run tf2_ros tf2_echo camera_color_optical_frame handeye_target    # marker pose"
[[ $WITH_DRIVER -eq 1 && $WITH_DISPLAY -eq 1 ]] && echo "  ros2 run tf2_ros tf2_echo world tool0                                  # arm pose"
[[ $WITH_MOVEIT -eq 1 ]] && echo "  ros2 action list | grep follow_joint_trajectory                       # MoveIt action up"

cat <<EOF

Useful viewers:
  ros2 run rqt_image_view rqt_image_view /charuco_tf/debug_image
  ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

Drag-teach the arm (limp, push by hand):
  ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: false}"
Re-engage torque:
  ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: true}"

Attach:    $0 --attach
Status:    $0 --status
Tear down: $0 --kill
EOF

if [[ $ATTACH_AFTER -eq 1 ]]; then
    exec tmux attach -t "$SESSION_NAME"
fi
