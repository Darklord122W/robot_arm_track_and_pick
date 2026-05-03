# xArm 1S — 2D Table Pick-and-Place

> **For Claude:** read this on session start. This is the current
> source of truth. Two reference docs survive in `docs/old/`:
> `depth_camera_commands.md` (Astra Pro CLI reference) and
> `robot_model_analysis.md` (URDF / link-length analysis). Everything
> else from the hand-eye-calibration era was deleted on 2026-05-02.

Last rewritten: **2026-05-02 night** — picks now reliably grasp +
hold the cube after the driver-clamp fix.

## 1. Goal

Pick a 40 mm cube off a flat table with the xArm 1S, optionally
place it at a target XY on the same table. Eye-to-hand camera
(tripod-mounted Astra Pro). Cube has a 30 mm ArUco marker
(`DICT_5X5_50`, `marker_id=2`) on top.

Long-term: multi-cube pick-and-place keyed on per-marker identity.
v1 below handles a single cube.

## 2. Current state

**Picks succeed.** The path is:

`bringup_pick.sh` → `calibrate_homography` (one-time per
camera/table pose) → `pick_2d` runs the pixel→world→IK→MoveIt loop.

The two breakthroughs that made it work:

- **2D homography pivot** (replaced full 3D hand-eye calibration).
  A 3×3 pixel→world matrix maps marker pixels directly to table
  XY. Calibrated by driving `tool0` over the cube and capturing
  pairs (camera pixel, world XY). See `calibrate_homography.py`.

- **Driver-clamp fix in IK** (2026-05-02 night). The hardware
  driver silently clamps every commanded joint to ±90°, but the
  URDF declares wider limits. Custom IK in `arm_ik.py` was picking
  joint values past ±π/2 that the driver then truncated, producing
  ~30 mm XY error and ~17° tilt the IK didn't predict. Fix: tighten
  IK's `JOINT_LIMITS` to ±1.5707 for all five arm joints, matching
  the driver's effective range.

## 3. Hardware

| Item | Detail |
|---|---|
| Arm | xArm 1S, 5-DOF positioning + 1-DOF gripper. USB via `xarm` Python lib. |
| Camera | Orbbec Astra Pro RGB-D, eye-to-hand on tripod, fixed in `world`. |
| Cube | 40 mm cube, 30 mm ArUco marker (`DICT_5X5_50`, `marker_id=2`) on top. |

## 4. Software stack

- ROS 2 Humble, MoveIt 2 (OMPL/RRTConnect for joint-space planning;
  KDL is bypassed because it returns NO_IK_SOLUTION on the 5-DOF arm).
- `xarm_hw` driver bridges `/joint_trajectory` → USB hardware via
  `topic_based_ros2_control`. **Silently clamps all joints to ±π/2**
  in `rad_to_units` — keep `arm_ik.JOINT_LIMITS` matched to this.
- `charuco_tf_publisher` (`single_aruco` mode) broadcasts
  `camera_color_optical_frame → handeye_target` for the cube's
  marker. (Frame name kept for backward compatibility.)
- `xarm_pick`:
  - `arm_ik.py` — custom 5-DOF numerical IK (scipy L-BFGS-B,
    24 random restarts, position-first cost, tilt < 90° filter).
  - `calibrate_homography.py` — interactive 2-phase calibration.
  - `pick_2d.py` — main pick state machine.
  - `gripper.py` (in `xarm_hw`) — drives only `arm1`.
- Vendored `ros2_astra_camera` for the Astra driver.

Workspace: `/home/darklord/xarm_moveit`. Branch: `main`.

## 5. Bring-up scripts (by use case)

Five tmux components: `display` (T1, RViz + robot_state_publisher),
`driver` (T2, USB), `camera` (T3, Astra), `marker` (T4, ArUco
tracker), `moveit` (T5). One script per use case:

| Script | Components | When to use |
|---|---|---|
| `./bringup_pick.sh` | display + driver + camera + marker + moveit | Full pick — `pick_2d`, `pick`. **Default.** |
| `./bringup_calib.sh` | display + driver + camera + marker | Run `calibrate_homography`. No MoveIt needed. |
| `./bringup_vision.sh` | camera + marker | Tune the camera or verify ArUco without powering the arm. |
| `./bringup_robot.sh` | display + driver | Drag-teach, gripper tuning, raw `/joint_trajectory` testing. No camera, no MoveIt. |
| `./bringup_moveit.sh` | display + driver + moveit | v1 joint-space pick (`pick --once HOME`), planner experiments. No camera. |

All five wrap a single `bringup_pick.sh` with `--profile {pick,calib,vision,robot,moveit}` plus fine-grained `--no-camera` / `--no-marker` / `--no-display` / `--no-driver` / `--no-moveit` overrides. Common flags:

```bash
./bringup_pick.sh --attach          # bring up + attach to tmux
./bringup_pick.sh --status          # is a session running?
./bringup_pick.sh --kill            # tear down
./bringup_pick.sh --marker-id 5     # different cube marker
```

Only one bringup session can run at a time (`SESSION_NAME=xarm_pick_bringup`). To switch profiles: `--kill` first, then start the new one.

## 6. End-to-end workflow

```bash
# 0. Build (once after pulls / edits)
cd ~/xarm_moveit && colcon build --symlink-install

# 1. Bring up the stack you need (see §5 for all options).
./bringup_pick.sh                    # full pick stack
# or: ./bringup_calib.sh              # for calibration only

# 2. Verify (in any sourced terminal — see §10 for sourcing):
ros2 topic hz /joint_states                                            # ~20 Hz
ros2 topic hz /camera/color/image_raw                                  # ~30 Hz
ros2 run tf2_ros tf2_echo camera_color_optical_frame handeye_target    # marker pose
ros2 run tf2_ros tf2_echo world tool0                                  # arm pose

# 3. Drag-teach helper (arm goes limp; push by hand):
ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: false}"
# ... move arm ...
ros2 service call /xarm/set_torque std_srvs/srv/SetBool "{data: true}"  # re-engage

# 4. Calibrate the pixel→world homography (8+ points recommended;
#    distribute across +Y AND -Y, +X AND -X for full coverage).
ros2 run xarm_pick calibrate_homography --num-points 8
# Per point:
#   Phase A — place cube on table, arm out of view, ENTER captures
#             marker pixel.
#   Phase B — drive gripper above cube without moving the cube,
#             ENTER captures tool0 world XY.
# Saves to ~/.ros2/xarm_pick/homography.yaml.

# 5. Verify the targets pick_2d would compute (no arm motion):
ros2 run xarm_pick pick_2d --no-execute

# 6. Pick + hold at HOME (no place):
ros2 run xarm_pick pick_2d --grip-rad -1.5 --table-z 0.1 --no-place

# 7. Pick + place:
ros2 run xarm_pick pick_2d --grip-rad -1.5 --table-z 0.1 \
    --place 0.10 -0.05
```

Useful flags on `pick_2d`:
- `--no-place` — pick the cube, lift, return to HOME holding it.
- `--pick-offset DX DY` / `--place-offset DX DY` — constant XY
  bias if you observe a consistent miss.
- `--pause-at-pre-grasp SEC` — diagnostic pause for `tf2_echo`.
- `--tilt-tol-deg N` — raise above 30° if a pose is being rejected
  for tilt and you're OK with the gripper angled.

## 7. Key files

| File | Role |
|---|---|
| `bringup_pick.sh` | tmux bringup (display / driver / camera / marker tracker / MoveIt). |
| `src/xarm/urdf/xarm_1s.urdf.xacro` | Robot model. `tool0` fixed to `link2` at `xyz="-0.0038 0 0.062"`. |
| `src/xarm_hw/xarm_hw/driver.py` | USB driver. **`rad_to_units` clamps every joint to ±90°.** |
| `src/xarm_hw/xarm_hw/gripper.py` | Gripper CLI + library: `move_gripper(node, target_rad, duration_s)`. |
| `src/xarm_pick/xarm_pick/arm_ik.py` | 5-DOF numerical IK. `JOINT_LIMITS` = ±1.5707 to match driver. |
| `src/xarm_pick/xarm_pick/calibrate_homography.py` | Two-phase pixel→world calibration. |
| `src/xarm_pick/xarm_pick/pick_2d.py` | Main pick state machine. |
| `src/xarm_pick/xarm_pick/moveit_client.py` | `MoveGroupClient.move_to_joints` (joint-space goals only). |
| `src/xarm_moveit_config/config/joint_limits.yaml` | `default_velocity_scaling_factor: 0.30`, accel same. Lower toward 0.05 if motions feel unsafe. |
| `src/xarm_moveit_config/launch/xarm_1s_moveit.launch.py` | OMPL legacy single-pipeline wiring (params under `ompl.*` namespace). |
| `src/charuco_tf_publisher/` | ArUco/ChArUco detector. Use `mode=single_aruco`. |
| `~/.ros2/xarm_pick/homography.yaml` | Saved calibration: 3×3 H, residuals, suggested table_z. |

## 8. Joint / frame quirks

- **Driver clamp at ±π/2**: see §2. `arm_ik.JOINT_LIMITS` MUST stay
  at ±1.5707; never widen without first checking the driver.
- **arm6 has a 180° yaw flip** baked into its origin
  (`urdf.xacro:39`, `rpy="0 0 3.14"`, `axis="0 0 -1"`). FK
  composes as `Rz(π) · Rz(-q6)`.
- **arm1 is the gripper master**; arm0/arm0_left/arm1_left mimic.
  Commanding only arm1 in a JointTrajectory works.
- **arm1 is OUTSIDE the MoveIt `arm` planning group**. The SRDF
  declares `<chain base_link="base_link" tip_link="tool0"/>`,
  excluding arm1 (a sibling branch off link2). MoveIt does NOT
  re-plan arm1 during MOVE steps — gripper state persists.
- **`pick_2d` sends joint-space goals**, not Cartesian. IK runs
  upfront in `arm_ik.solve_ik`, then `move_to_joints(ik.joints)`
  hands the joint values to MoveIt. KDL is never invoked.
- **Camera TF chain**: Astra driver publishes
  `camera_link → camera_color_frame → camera_color_optical_frame`.
  Don't add a `world → camera_color_optical_frame` static publisher
  (two-parents conflict). If you ever publish a camera world
  transform, target `world → camera_link`.
- **MoveIt OMPL config**: Humble's `move_group` reads OMPL params
  from `ompl.*` at top level (legacy single-pipeline path), not
  the newer `planning_pipelines.ompl.*` map. Adapter is
  `AddTimeOptimalParameterization`, not `AddTimeParameterization`.

## 9. Known limits / next steps

- **Gripper-down workspace**: with the ±π/2 clamp, reliable picks
  live in roughly an 80-200 mm radial ring around the base,
  centered on +Y (the calibration is fit to that region — see
  `homography.yaml` calibration_points). Targets outside the ring
  are rejected upfront with a tilt > tolerance error.
- **Calibration quality**: latest `homography.yaml` has 9/12
  inliers at ~2.5 mm residual mean. The 3 outliers (>13 mm) and
  the +Y-only point distribution are the next things to fix —
  recapture with a wider XY spread for full-table reach.
- **Multi-cube tracking**: `charuco_tf_publisher` `single_aruco`
  mode publishes one TF per marker_id. For multiple cubes, either
  run one publisher node per marker_id, or extend the publisher to
  emit one TF per detected marker (e.g., `cube_marker_2`,
  `cube_marker_5`).

## 10. Sourcing reminder

Every new terminal needs both setup files:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_moveit/install/setup.bash
```

Add both to `~/.bashrc` to get them in every new terminal:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/xarm_moveit/install/setup.bash' >> ~/.bashrc
```

`bringup_pick.sh` panes already source both; standalone terminals don't.
