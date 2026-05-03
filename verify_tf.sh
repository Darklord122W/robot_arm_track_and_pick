#!/usr/bin/env bash
# verify_tf.sh — print every TF chain that matters for the xArm + camera setup.
#
# What this checks (in order):
#   1) /joint_states is alive           → arm driver up?
#   2) world → link2                    → URDF + driver feeding FK?
#   3) world → link0  (TCP / fingertip) → only if you've added the tool0 frame
#   4) camera_link → camera_color_optical_frame
#                                       → Astra driver's internal camera tree
#   5) world → camera_color_optical_frame
#                                       → calibration TF wired up?
#   6) camera_color_optical_frame → handeye_target
#                                       → ChArUco / ArUco detector publishing?
#   7) camera_color_optical_frame → cube
#                                       → cube_detector publishing?
#   8) world → cube                     → full pick chain resolves?
#
# Each lookup uses tf2_echo with a short timeout. NO TF lookup → printed in
# red as "MISSING". Successful lookup → green "OK" plus the translation and
# RPY-degrees. Run in a terminal that has ROS sourced (the script also
# sources Humble + the workspace install).
#
# Usage:
#   ./verify_tf.sh            # one-shot snapshot
#   ./verify_tf.sh --watch    # rerun every 2s (Ctrl+C to stop)
#   ./verify_tf.sh --rate 1   # change the watch refresh rate
#
# Implementation notes:
#  - tf2_echo blocks until it gets at least one transform OR a timeout — we
#    use `timeout 3 ros2 run tf2_ros tf2_echo ...` and grep the output. The
#    timeout has to come from the shell, NOT tf2_echo's --timeout flag,
#    which a) only exists on newer Humble patches, b) prints the help banner
#    and exits 5 when missing.
#  - The trailing "" guards against frames whose absence prints an
#    "Invalid frame ID" warning to stderr but still returns exit 0.

set -o pipefail

WORKSPACE="${WORKSPACE:-$HOME/xarm_moveit}"
TIMEOUT_SEC="${TIMEOUT_SEC:-3}"

# Source ROS if not already sourced (idempotent).
if [[ -z "${ROS_DISTRO:-}" ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
fi

GREEN=$'\033[0;32m'
RED=$'\033[0;31m'
YELLOW=$'\033[0;33m'
DIM=$'\033[2m'
RESET=$'\033[0m'

# Echo a single TF lookup. $1=parent  $2=child  $3=label
print_tf() {
  local parent="$1" child="$2" label="$3"
  local out
  out=$(timeout "$TIMEOUT_SEC" ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | head -25)

  # tf2_echo prints "At time ..." once a transform was looked up.
  if echo "$out" | grep -q "^At time "; then
    local trans rpy
    trans=$(echo "$out" | grep -m1 "Translation:" | sed 's/^[ -]*//')
    rpy=$(echo "$out"   | grep -m1 "RPY (degree)" | sed 's/^[ -]*//')
    printf "  %s%-12s%s %s -> %-32s %s%s%s\n" \
      "$GREEN" "[OK]" "$RESET" "$parent" "$child" "$DIM" "($label)" "$RESET"
    printf "      %s\n" "$trans"
    printf "      %s\n" "$rpy"
  else
    # Differentiate "frame doesn't exist" from "lookup timeout".
    if echo "$out" | grep -q "Invalid frame ID"; then
      printf "  %s%-12s%s %s -> %-32s %s%s%s\n" \
        "$RED" "[MISSING]" "$RESET" "$parent" "$child" "$DIM" "($label)" "$RESET"
    else
      printf "  %s%-12s%s %s -> %-32s %s%s%s\n" \
        "$YELLOW" "[TIMEOUT]" "$RESET" "$parent" "$child" "$DIM" "($label)" "$RESET"
    fi
  fi
}

print_topic_alive() {
  local topic="$1" label="$2"
  # Use `ros2 topic echo --once` instead of `topic hz` — hz needs ≥2
  # messages and waits past its own bucket, which times out for low-rate
  # topics. echo --once just confirms ANY message arrived, which is what
  # we actually want.
  local out
  out=$(timeout 3 ros2 topic echo --once "$topic" 2>&1 | head -3)
  if [[ -n "$out" ]] && ! echo "$out" | grep -q "Could not\|exception\|^Error"; then
    printf "  %s%-12s%s %-30s %s%s%s\n" \
      "$GREEN" "[OK]" "$RESET" "$topic" "$DIM" "($label)" "$RESET"
  else
    printf "  %s%-12s%s %-30s %s%s%s\n" \
      "$RED" "[SILENT]" "$RESET" "$topic" "$DIM" "($label)" "$RESET"
  fi
}

print_joint_state() {
  # Single-shot dump of current arm joint values.
  local js
  js=$(timeout 3 ros2 topic echo /joint_states --once 2>&1)
  if echo "$js" | grep -q "^name:"; then
    echo "  ${GREEN}[OK]${RESET}        /joint_states (current arm pose):"
    echo "$js" | awk '
      /^name:/         {in_name=1; in_pos=0; print "      name:    " ""; next}
      /^position:/     {in_name=0; in_pos=1; print "      position:" ""; next}
      /^velocity:|^effort:|^header:/ {in_name=0; in_pos=0; next}
      in_name {gsub(/^- /, "        "); print}
      in_pos  {gsub(/^- /, "        "); print}
    '
  else
    echo "  ${RED}[SILENT]${RESET}    /joint_states (arm driver not publishing)"
  fi
}

run_once() {
  local ts
  ts=$(date +"%Y-%m-%d %H:%M:%S")
  printf "\n%s=== xArm + Camera TF Verification @ %s ===%s\n" "$DIM" "$ts" "$RESET"

  printf "\n%s[1/4] Topics%s\n" "$DIM" "$RESET"
  print_topic_alive "/joint_states" "arm driver"
  print_topic_alive "/camera/color/image_raw" "camera RGB"
  print_topic_alive "/camera/depth/image_raw" "camera depth"
  print_topic_alive "/charuco_tf/debug_image" "ChArUco detector debug"
  print_topic_alive "/cube_detector/debug_image" "cube detector debug"

  printf "\n%s[2/4] Arm pose (joint values)%s\n" "$DIM" "$RESET"
  print_joint_state

  printf "\n%s[3/4] Robot model TF chain%s\n" "$DIM" "$RESET"
  print_tf world link2 "FK to robot effector frame"
  print_tf world link0 "FK to gripper base (TCP candidate)"

  printf "\n%s[4/4] Camera + calibration TF chain%s\n" "$DIM" "$RESET"
  print_tf camera_link camera_color_optical_frame "Astra driver internal tree"
  print_tf world camera_link "calibration TF (composed - what we publish)"
  print_tf world camera_color_optical_frame "calibration result wired to TF?"
  print_tf camera_color_optical_frame handeye_target "ChArUco / ArUco detector"
  print_tf world handeye_target "calibration -> board chain"
  print_tf camera_color_optical_frame cube "cube_detector"
  print_tf world cube "full chain world -> cube (pick-ready)"

  printf "\n%sLegend:%s ${GREEN}[OK]${RESET} alive  "\
"${RED}[MISSING]${RESET} frame absent  ${YELLOW}[TIMEOUT]${RESET} lookup slow  "\
"${RED}[SILENT]${RESET} topic not publishing\n" "$DIM" "$RESET"
  printf "%sNote:%s during arm motion the marker can leave the camera FOV — "\
"handeye_target / cube going [MISSING] is expected then. Check again at the next settle.%s\n" \
"$DIM" "$DIM" "$RESET"
}

case "${1:-}" in
  --watch)
    rate="${2:-2}"
    while true; do
      clear
      run_once
      sleep "$rate"
    done
    ;;
  --rate)
    rate="${2:-1}"
    while true; do
      clear
      run_once
      sleep "$rate"
    done
    ;;
  -h|--help)
    sed -n '1,40p' "$0"
    ;;
  *)
    run_once
    ;;
esac
