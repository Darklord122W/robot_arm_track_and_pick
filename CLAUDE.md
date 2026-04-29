# xArm 1S Table Cube Pick-and-Place — Project Status

> **For Claude:** read this first on session start. It is the single source of truth
> for *what is built, what is broken, and what to do next*. The detailed reference
> docs in this repo (linked in §10) are the source of truth for *how things work*.

Last updated: **2026-04-29** (`moveit_calibration` source removed from the
workspace — it was unused since 2026-04-28 when we switched to `easy_handeye2`
+ custom `charuco_tf_publisher`. Camera is on a tripod — **eye-to-hand**; the
older eye-in-hand wording in some docs is stale).

---

## 1. Goal

Pick a 25 mm cube off a table with the xArm 1S, using the **eye-to-hand**
Orbbec Astra Pro (mounted on a tripod, fixed in `world`) to localise the
cube and MoveIt 2 to plan the motion. Long-term: multi-cube pick-and-place.

## 2. Hardware

| Item | Detail |
|---|---|
| Arm | xArm 1S, 6-DOF, parallel-jaw mimic gripper. USB-driven via the `xarm` Python lib. |
| Camera | Orbbec Astra Pro RGB-D, **eye-to-hand** on a tripod, fixed in `world`. The URDF still has a leftover `depth` link parented to `link2` (`xarm_1s.urdf.xacro:251-257`) from the previous eye-in-hand attempt — that block is now obsolete and must be removed/replaced after calibration. |
| Target object | 25 mm cube on a flat table. |
| Calibration board | calib.io ChArUco, 210×150 mm, 5×7 squares, 26 mm checker, 19 mm marker, `DICT_5X5`. PDF at `/home/darklord/Downloads/calib.io_charuco_210x150_5x7_26_19_DICT_5X5.pdf`. |

## 3. Software stack

- ROS 2 Humble.
- MoveIt 2 with KDL IK + OMPL/RRTConnect.
- Custom `xarm_hw` driver bridging `/xarm/joint_trajectory` to USB-controlled hardware
  (`topic_based_ros2_control` plugin in URDF).
- Custom `cube_detector` node — depth+RGB fusion or RGB-only PnP, publishes
  `cube` TF in the camera optical frame.
- Vendored `ros2_astra_camera` driver with a tuned launch file for depth-noise
  reduction.

Workspace root: `/home/darklord/xarm_moveit`. Branch: `main`.

## 4. What works today

- `colcon build` clean across all packages (`astra_camera`, `astra_camera_msgs`,
  `cube_detector`, `xarm`, `xarm_hw`, `xarm_moveit_config`).
- Camera streams `/camera/color/*`, `/camera/depth/*`, `/camera/depth/points`
  via `astra_pro_tuned.launch.py` with depth tuning applied.
- `cube_detector` node detects the cube and broadcasts its pose; debug image at
  `/cube_detector/debug_image`.
- MoveIt RViz session loads the URDF/SRDF and can plan in joint space.
- USB hardware driver (`xarm_hw/driver.py`) connects to the arm and accepts
  `FollowJointTrajectory` goals.

## 5. Critical gaps (priority order)

These are blockers for an end-to-end pick. Source: `robot_model_analysis.md` §11.

| # | Gap | Why it matters | Where to fix |
|---|---|---|---|
| 1 | **No `world → camera_color_optical_frame` TF** | The camera is on a tripod, so its pose in `world` is unknown until calibrated. Without it, the TF chain `world → cube` cannot be composed. | run eye-to-hand calibration → static TF or URDF fixed joint |
| 2 | **Stale `depth_joint` in URDF** | `xarm_1s.urdf.xacro:251-257` still parents a `depth` link off `link2` from the eye-in-hand era. Harmless but misleading; will conflict if anything else publishes `world → camera_*` and the same name appears via this chain. | delete the `depth_joint` block entirely after calibration |
| 3 | **No `tool0`/TCP link** | MoveIt currently plans for `link2`, not the fingertips. Approach offsets are wrong. | add fixed link in URDF between `link0` and `link0_left` midpoint |
| 4 | **IK timeout 5 ms** | Spurious "no IK solution" failures during planning. | bump to 0.05–0.1 s in `kinematics.yaml` |
| 5 | **No gripper action** | MoveIt can't drive `arm1`. | small node publishing JointTrajectory on `arm1` |
| 6 | Detector is `rgb_only` | RGB+Canny+IPPE is more brittle than the depth-fusion path. | flip `detection_mode: depth_fusion` in `cube_detector.yaml:6` |
| 7 | 5% velocity scaling | Picks crawl. | bump in `joint_limits.yaml:5-6` |

## 6. Active focus: hand-eye calibration

This is gap #1 (and folds in #2). Everything else waits until the
world-to-camera transform is measured.

### Decisions made

| Decision | Why |
|---|---|
| **Tool: `easy_handeye2`** + custom **`charuco_tf_publisher`** | Switched from `moveit_calibration` on 2026-04-28 (source then deleted on 2026-04-29). moveit_calibration's GUI was convenient (lived as an rviz Display in the same MoveIt RViz) but its detector + PnP path was opaque and gave noisy poses on this board. `easy_handeye2` is *tracker-agnostic* — it samples robot+marker TFs and runs AX=XB only, so we keep full control of the per-frame camera→marker pose pipeline by writing our own detector. The detector lives at `src/charuco_tf_publisher/`. |
| **Detector: `charuco_tf_publisher` (Python, OpenCV 4.5.4)** | Subscribes to color image + camera_info, detects ChArUco corners with **`CORNER_REFINE_SUBPIX`** (default detection params alone gave wobbly corners), and estimates pose with **`cv2.solvePnPGeneric(..., flags=SOLVEPNP_IPPE)`** picking the lower-reprojection-error of the two planar solutions. This kills the front/back pose ambiguity that `cv2.aruco.estimatePoseCharucoBoard` exhibits (it uses `SOLVEPNP_ITERATIVE` internally with no override). Symptom of the unfixed pipeline: drawn frame axes flip direction frame-to-frame, contaminating any sample taken in that moment. |
| **Mode: eye-to-hand** (`easy_handeye2` calls this `eye_on_base`) | Camera is rigidly mounted on a tripod, *not* on the arm. The board moves with the arm, the camera stays still. |
| **Target attachment: taped to `link2`** | The board must be rigidly attached to a controlled link. `link2` is the highest-numbered link upstream of the gripper mimics, so it's fully controlled by the 6 arm joints and immune to gripper jitter. *Do not* clamp in the gripper jaws — `arm0`/`arm0_left`/`arm1_left` are mimics of `arm1`, so any twitch shifts the board relative to the finger link. See §13 for taping procedure. |
| **Target: ChArUco** | The calib.io board has both checker squares and markers. |
| **Dictionary: `DICT_5X5_250`** | DICT_5X5 family; the 250-variant covers our IDs (board has ~17 markers). |

### Architecture

```
/camera/color/image_raw    ─┐
/camera/color/camera_info  ─┴─►  charuco_tf_publisher  ──► /tf  (camera_color_optical_frame → handeye_target)
                                                            │
/tf (world → link2 from URDF + arm joints) ─────►  handeye_server  (samples + AX=XB solve)
                                                            │
                                              rqt_calibrator  (Take Sample / Compute / Save GUI)
                                                            │
                                              ~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib
```

`handeye_calibrate.launch.py` (in `src/charuco_tf_publisher/launch/`) brings up
all of these together. easy_handeye2 also publishes a dummy `world →
camera_color_optical_frame` static TF in `eye_on_base` mode so its TF readiness
check passes; it is not consumed by the calibration math and is harmless.

### Calibration plan

1. **Print and mount** the calib.io board at 100% scale (NOT "Fit to page") on
   a rigid backing. Measure a square with calipers — must be 26 mm. If
   different, scale `square_length` and `marker_length` proportionally via
   launch args.
2. **Tape the board to `link2`** (see §13). Wiggle test: edges must move with
   the wrist as one rigid unit; tape must not slip under arm motion.
3. **Cover the arm's blue status LED** with opaque tape, and ensure even
   ambient light (no harsh single-source glare). The blue LED reflection on
   the board's white squares broke marker detection in the lower half during
   initial trials and was the dominant source of pose noise — fixing this
   alone gave ~3× improvement before any code change.
4. **Launch the stack** (§8.2): hardware → MoveIt → camera → calibration.
5. **Verify the detector before sampling** (§8.4 step 1): in
   `/charuco_tf/debug_image` you should see ≥18 green ChArUco corners and
   frame axes that hold steady when the arm holds steady. If axes flip
   direction or the corner count is low, fix lighting / board angle / occlusion
   before clicking anything.
6. **Take 15–20 samples** with deliberate rotation about ≥3 non-parallel
   wrist joints. Translation-only or yaw-only motion is *degenerate* for
   AX=XB and produces a result with rotation locked near identity and
   translation drifted to wherever the residual lands — even with hundreds of
   samples. See §8.4 step 3 for the explicit protocol.
7. **Cross-validate solvers**: hit Compute with multiple algorithms (Tsai-Lenz,
   Park, Daniilidis, Horaud). Translation should agree across solvers within
   ~1 cm. Disagreement > 1 cm = "not enough rotation diversity yet" — take
   more samples, don't Save.
8. **Sanity-check before Save**: the printed translation must match a
   tape-measure to where the tripod actually is. Recognizable failure mode:
   `rotation magnitude ≪ 90°` *and* translation near origin = "rotation
   underdetermined" — solver settled near identity. Visualize with §8.6 if
   in doubt.
9. **Save** — overwrites `~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib`.
10. **Wire into URDF** (§8.5).

## 7. Next steps (in execution order)

1. **Run the calibration session above.** This unblocks everything else.
2. **Add `tool0` / TCP frame** — a fixed link in the URDF at the midpoint of
   `link0` and `link0_left` (the gripper pads). Update SRDF if needed.
3. **Bump IK timeout** to `0.05` in `src/xarm_moveit_config/config/kinematics.yaml`.
4. **Switch detector to `depth_fusion`** in `src/cube_detector/config/cube_detector.yaml:6`.
5. **Write a gripper action node**. Publishes a JointTrajectory on `arm1` for
   open/close. The other 3 finger joints follow via URDF mimics.
6. **Pick state machine**: home → observe → MoveIt to pre-grasp (cube + 5 cm in
   `world +Z`, gripper-Z down) → cartesian descent → close gripper → cartesian
   ascent → place → home.
7. **Bump velocity scaling** in `joint_limits.yaml:5-6` once #1–6 are validated.

## 8. Run cheat-sheet

Every new terminal starts with both source lines:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_moveit/install/setup.bash
```

### 8.1 One-time builds (already done as of 2026-04-28)

```bash
cd ~/xarm_moveit

# Build everything from clean (sequential to avoid OOM on this 4-core/7.6 GB box)
colcon build --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or single-package rebuilds when you edit one thing:
colcon build --packages-select charuco_tf_publisher easy_handeye2 \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --packages-select astra_camera \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --packages-select xarm xarm_moveit_config \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 8.2 Hand-eye calibration session (eye-to-hand, easy_handeye2)

```bash
# T1 — hardware (run both lines, second one waits on the first)
ros2 launch xarm_hw xarm_hw_display.launch.py
ros2 run   xarm_hw xarm_hw_driver

# T2 — MoveIt + RViz (used to jog the arm to new sample poses)
ros2 launch xarm_moveit_config xarm_1s_moveit.launch.py

# T3 — camera (depth-tuned launch; publishers patched to RELIABLE QoS)
ros2 launch astra_camera astra_pro_tuned.launch.py

# T5 — detector + easy_handeye2 server + rqt_calibrator GUI
# (combined launch in src/charuco_tf_publisher/launch/handeye_calibrate.launch.py)
ros2 launch charuco_tf_publisher handeye_calibrate.launch.py
```

The combined launch in T5 brings up:
- `charuco_tf_publisher` — broadcasts `camera_color_optical_frame → handeye_target`
- `handeye_server` — easy_handeye2 backend (records samples, runs AX=XB)
- `handeye_rqt_calibrator` — the GUI window with `Take Sample` / `Compute` /
  `Save` buttons. Launched with `--force-discover` so a stale
  `~/.config/ros.org/rqt_gui.ini` cache cannot silently kill the panel
  (`qt_gui_main: found no plugin matching ...`, exit code 1).
- `dummy_publisher` — replicated from easy_handeye2's stock launch, satisfies
  its TF readiness check; not used by the math, harmless.

There is **no T4 placeholder TF**. The calibrator does not consume `world →
camera_*` TF (it is solving for that exact transform), so a placeholder is
not needed and trying to add one collides with the Astra driver's chain.
RViz will print `Frame [camera_link] not connected to fixed frame [world]`
during calibration — harmless.

Defaults match the calib.io 210×150 mm 5×7 / 26 mm / 19 mm DICT_5X5 board
(`squares_x=5 squares_y=7 square_length=0.026 marker_length=0.019
dictionary=DICT_5X5_250`). Override at the command line if your printed
square is not 26 mm:
```bash
ros2 launch charuco_tf_publisher handeye_calibrate.launch.py \
  square_length:=0.0255 marker_length:=0.0186
```

### 8.3 Sanity checks (in a 6th terminal during calibration)

```bash
# Topics live?
ros2 topic hz /joint_states               # ~30-50 Hz
ros2 topic hz /camera/color/image_raw     # ~30 Hz
ros2 topic info /camera/color/image_raw -v   # publisher should now say RELIABLE

# TF chain alive?
ros2 run tf2_ros tf2_echo world link2                                   # arm FK working
ros2 run tf2_ros tf2_echo camera_color_optical_frame handeye_target     # detector publishing TF
ros2 run tf2_ros tf2_echo world camera_color_optical_frame              # only resolves AFTER calibration is wired in

# Calibration nodes alive?
ros2 node list | grep -E "handeye|charuco"   # expect /charuco_tf_publisher /handeye_server /handeye_rqt_calibrator /dummy_publisher

# What's the detector publishing?
ros2 param dump /charuco_tf_publisher        # board / dictionary / frames / corner_refine
```

### 8.4 The rqt panel — verify, sample, compute, save

The `handeye_rqt_calibrator` GUI (separate window from rqt_image_view, titled
`easy_handeye2 Calibration tool` or just `rqt`) has three sections: an Info
block (frames, calibration name), an Actions group (`Take Sample` /
`Remove Sample` / `Save`), and a Samples list + Result text box with a
solver dropdown.

**1. Verify the detector before sampling.** Open `/charuco_tf/debug_image`:
```bash
ros2 run rqt_image_view rqt_image_view /charuco_tf/debug_image
```
You should see:
- ≥ 18 green ChArUco corner dots (24 max for the 5×7 board)
- frame axes (R/G/B triad) drawn on the board, **holding steady when the arm
  holds steady**

If axes flip direction or fewer than ~12 corners are detected, fix the
underlying issue first — the IPPE solver only kills the per-frame ambiguity
when it has enough corners and decent contrast. Common culprits:
- **Arm's blue status LED reflecting on board** → cover with opaque tape.
- **Dim / harsh single-source lighting** → diffuse, brighter ambient.
- **Board nearly perpendicular to camera** → tilt board 20–60° in pitch/roll.

**2. Move arm and take samples.** Use MoveIt in T2's RViz to jog the arm.
After each move, **wait 1–2 sec for the arm to settle** before clicking
`Take Sample` — the calibrator captures TF *at the moment* you click; if
the arm is still oscillating, the FK and the image will not be at the same
instant.

**3. Sampling protocol — rotation diversity is the single biggest determinant.**
Translation-only or yaw-only samples produce a degenerate AX=XB. Each row
below excites a *different* rotation axis; you need samples from each.

| Group | What to vary | Hold roughly constant | # samples |
|---|---|---|---|
| **A** | wrist `arm6` yaw — sweep ±60° | arm position | 4 |
| **B** | wrist `arm5` pitch — board tilts forward/back | arm position | 4 |
| **C** | wrist `arm4` roll — board rotates about its normal | arm position | 4 |
| **D** | mixed — translate arm to 3 different positions, vary wrist orientation | nothing | 4–6 |

**4. Cross-validate solvers.** With ~12+ samples, pick each algorithm in the
dropdown (`OpenCV/Tsai-Lenz`, `Park`, `Daniilidis`, `Horaud`) and click
`Compute`. Translation should agree across solvers within ~1 cm. If they
diverge more, take more samples — *do not* Save.

**5. Sanity-check the printed result before Save.** The `Result` text box
shows `Translation (x,y,z)` and `Rotation (x,y,z,w)` in `world →
camera_color_optical_frame` direction.
- **Translation** must match a tape-measure reading from the world origin
  (robot base) to the tripod camera lens, within a few cm.
- **Recognizable failure mode**: rotation magnitude `2·acos(w)` ≪ 90° and
  translation near origin → "rotation underdetermined" — solver settled
  near identity. Common when sample rotations didn't span ≥3 axes. Fix by
  taking samples from groups B and C above.
- **Visualize before committing** if uncertain — see §8.6.

**6. Save.** Writes `~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib`
(YAML). The transform is `world → camera_color_optical_frame`.

### 8.5 Wire the calibrated transform into the URDF

After saving, the YAML at `~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib`
contains xyz + quaternion. URDF takes rpy, so convert first:

```bash
# Print xyz + rpy from the saved file
python3 - <<'PY'
import yaml, math
from transforms3d.euler import quat2euler
d = yaml.full_load(open(__import__('os').path.expanduser(
    '~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib')))
t = d['transform']['translation']; r = d['transform']['rotation']
roll, pitch, yaw = quat2euler([r['w'], r['x'], r['y'], r['z']], axes='sxyz')
print(f'xyz="{t["x"]:.6f} {t["y"]:.6f} {t["z"]:.6f}"')
print(f'rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"   '
      f'# = ({math.degrees(roll):+.2f}, {math.degrees(pitch):+.2f}, {math.degrees(yaw):+.2f}) deg')
PY
```

Then:

```bash
# 1. Edit src/xarm/urdf/xarm_1s.urdf.xacro:
#    DELETE the depth_joint + depth link block at lines 251-257
#    ADD a new fixed joint (substitute the printed xyz + rpy):
#      <joint name="camera_mount_joint" type="fixed">
#        <parent link="world"/>
#        <child link="camera_color_optical_frame"/>
#        <origin xyz="X Y Z" rpy="R P Y"/>
#      </joint>
#      <link name="camera_color_optical_frame"/>
#
# 2. Set publish_tf: false in
#    src/ros2_astra_camera/astra_camera/params/astra_pro_tuned_params.yaml
#    so the Astra driver stops publishing camera_color_frame ->
#    camera_color_optical_frame (which would conflict with the URDF's
#    new world -> camera_color_optical_frame and trigger TF "two parents"
#    warnings — and given Astra publishes at 10 Hz, the URDF static
#    would lose).
#
# 3. Rebuild
colcon build --packages-select xarm xarm_moveit_config astra_camera \
  --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Kill T5 (the calibration session), restart T2 + T3.
```

### 8.6 Visualize the saved calibration before committing

You can sanity-check the saved transform without touching the URDF.
**Publish it under a different child frame name** (avoids the live-conflict
problem in §12) and view in RViz:

```bash
# Read xyz + qx/qy/qz/qw from the saved file:
cat ~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib

# Publish under a new child name. NOTE the `--` separator before the args:
# Humble's tf2_ros static_transform_publisher argparse misreads negative
# numeric values as flag names without it (`error parsing command line
# arguments: Extra unparsed arguments`).
ros2 run tf2_ros static_transform_publisher -- \
  --x X --y Y --z Z \
  --qx QX --qy QY --qz QZ --qw QW \
  --frame-id world --child-frame-id calibrated_camera
```

Then in T2's RViz: Fixed Frame `world`, Add → TF, locate the
`calibrated_camera` triad. Compare to the actual tripod position by eye
(or tape-measure). The triad's **blue arrow (Z, "into the scene")** should
point at the workspace, *not* at the ceiling. If it sits inside the robot
or points up, the calibration is bad — re-sample (likely insufficient
rotation diversity).

### 8.7 Verify (post-URDF wiring)

```bash
ros2 run tf2_ros tf2_echo world camera_color_optical_frame   # = saved values
ros2 run tf2_ros tf2_echo world cube                          # ±1-2 mm of tape-measure
```

### 8.8 Cube-pick run (post-calibration)

```bash
# T1 hardware, T2 MoveIt+RViz, T3 camera as in §8.2.
# No calibration session needed (URDF now publishes the calibrated camera fixed joint).

# T5 — cube detector
ros2 launch cube_detector cube_detector.launch.py

# Visualize
ros2 run rqt_image_view rqt_image_view /cube_detector/debug_image
ros2 topic echo /cube_detector/pose
ros2 run tf2_ros tf2_echo world cube
```

### 8.9 Helper scripts at the repo root

```bash
python3 ~/xarm_moveit/null_pos.py      # send arm to known null pose
python3 ~/xarm_moveit/battery_check.py # battery state of charge
```

## 9. Repo state at last update

Branch `main`, **uncommitted modifications** at last check:

- `M depth_camera_commands.md`
- `M src/cube_detector/config/cube_detector.yaml`
- `M src/cube_detector/cube_detector/detector.py`
- `M src/cube_detector/cube_detector/detector_node.py`
- `M src/ros2_astra_camera/astra_camera/params/astra_mini_params.yaml`
- `M src/ros2_astra_camera/astra_camera/params/astra_pro_tuned_params.yaml`
- `?? robot_model_analysis.md`
- `?? screenshots.md`
- `?? src/cube_detector/README.md`

Last commit: `69ba8af Initial commit: xarm_moveit workspace`. Run `git status` /
`git log` to confirm currency before assuming anything here.

## 10. Reference docs (read these for detail)

| Doc | What's in it |
|---|---|
| `robot_model_analysis.md` | Full URDF/MoveIt analysis: link lengths, joint limits, frame chain, gap list (§11), TL;DR. **Authoritative on the robot model.** |
| `src/cube_detector/README.md` | Detector pipeline, parameters, tuning guide, troubleshooting. **Authoritative on cube detection.** |
| `depth_camera_commands.md` | Astra Pro: launch, services, params, depth-noise tuning. **Authoritative on the camera.** |
| `screenshots.md` | Where the user's screenshots live (`/home/darklord/Pictures/Screenshots/`). |

## 11. File map (one-line each)

| Path | Purpose |
|---|---|
| `src/xarm/urdf/xarm_1s.urdf.xacro` | URDF: links, joints, gripper mimics, ros2_control. Lines 251-257 still hold the obsolete eye-in-hand `depth_joint` — remove after calibration and replace with a `world → camera_color_optical_frame` fixed joint (§8.5). |
| `src/xarm_moveit_config/config/xarm_1s.srdf` | MoveIt: planning group, virtual joint, named poses, collision matrix |
| `src/xarm_moveit_config/config/kinematics.yaml` | KDL IK plugin + timeout (currently 5 ms — too low) |
| `src/xarm_moveit_config/config/ompl_planning.yaml` | OMPL/RRTConnect pipeline |
| `src/xarm_moveit_config/config/joint_limits.yaml` | Per-joint velocity, default scaling (currently 5%) |
| `src/xarm_moveit_config/config/moveit_controllers.yaml` | FollowJointTrajectory bridge |
| `src/xarm_moveit_config/launch/xarm_1s_moveit.launch.py` | MoveIt + RViz launch |
| `src/cube_detector/config/cube_detector.yaml` | Detection mode, RANSAC, multi-cube settings |
| `src/cube_detector/cube_detector/detector.py` | Pure-Python detection (ROS-free, unit-testable) |
| `src/cube_detector/cube_detector/detector_node.py` | ROS 2 node wrapping the detector |
| `src/cube_detector/launch/cube_detector.launch.py` | Detector launch file |
| `src/charuco_tf_publisher/charuco_tf_publisher/charuco_tf_node.py` | **NEW** ChArUco detector → `camera_color_optical_frame → handeye_target` TF. Uses `CORNER_REFINE_SUBPIX` + `solvePnPGeneric(SOLVEPNP_IPPE)` to avoid the planar-PnP front/back ambiguity. |
| `src/charuco_tf_publisher/launch/charuco_tf.launch.py` | **NEW** Detector standalone (no easy_handeye2). |
| `src/charuco_tf_publisher/launch/handeye_calibrate.launch.py` | **NEW** Combined: detector + easy_handeye2 server + dummy TF + rqt_calibrator (with `--force-discover`). |
| `src/easy_handeye2/easy_handeye2/launch/calibrate.launch.py` | Stock easy_handeye2 launch (we replicate its content in `handeye_calibrate.launch.py` so we can pass `--force-discover` to `rqt_calibrator.py`). |
| `src/easy_handeye2/easy_handeye2/launch/publish.launch.py` | Stock easy_handeye2 launch that loads a saved `.calib` and broadcasts the static TF. Conflicts with the live Astra subtree — see §8.6 for a workaround using a separate frame name. |
| `src/ros2_astra_camera/astra_camera/params/astra_pro_tuned_params.yaml` | 640×480@30, depth_align=true, **publish_tf=true** (must flip to **false** when wiring calibration into URDF — §8.5). |
| `src/ros2_astra_camera/astra_camera/launch/astra_pro_tuned.launch.py` | Camera + auto-tuner |
| `src/xarm_hw/xarm_hw/driver.py` | USB driver: bridges /xarm/joint_trajectory to hardware |
| `~/.ros2/easy_handeye2/calibrations/xarm_handeye.calib` | Saved calibration YAML — `world → camera_color_optical_frame`. Outside the workspace. |
| `null_pos.py` | Helper: send the arm to a known null pose |
| `battery_check.py` | Helper: check robot battery |

## 12. Known gotchas

- `arm6` has `rpy="0 0 3.14"` baked into its origin (`urdf.xacro:39`) — gripper
  "forward" is opposite to `base_link +X` at `q=0`. Account for this when
  converting cube yaw to gripper yaw, or expect IK failures at one orientation
  and successes at the flipped one.
- Gripper joints (`arm0`, `arm0_left`, `arm1_left`) are URDF mimics of `arm1`
  and are **not** in the MoveIt planning group. Drive `arm1` directly via
  JointTrajectory; the others follow.
- The Astra driver was patched on 2026-04-28 to publish **RELIABLE** on
  color/depth/IR images and their `camera_info` topics
  (`uvc_camera_driver.cpp:73-75`, `ob_camera_node.cpp:303-306`). Original
  reason: `moveit_calibration`'s image_transport subscriber was RELIABLE by
  default with no QoS-arg overload in Humble, so a Best-Effort publisher was
  silently ignored. moveit_calibration is gone now, but we kept the patch —
  it's a strict superset of the old behaviour: Best-Effort subscribers (RViz,
  rqt, `cube_detector`) connect to a RELIABLE publisher just fine.
  *Old gotcha note (no longer needed):* "set RViz/rqt Reliability to Best
  Effort" — RViz now connects either way.
- `depth_align: true` is required in the astra params for the detector to work
  — otherwise depth and color pixels are in different frames and the height-band
  mask is wrong.
- `detection_mode` in `cube_detector.yaml` is read once at startup; restart the
  node after changing it (live-tunable params are listed in the README).
- **Eye-to-hand mount**: the camera lives on a tripod, *not* on `link2`. The
  URDF's `depth_joint` (lines 251-257) is leftover from the previous eye-in-hand
  attempt and must be removed once calibration is wired in. Until then, expect
  TF warnings about the orphan `depth` link.
- **Calibration board attachment**: tape to `link2`, *not* to the gripper jaws.
  Gripper joints are mimics — they twitch every time the gripper command
  oscillates, dragging the board with them. See §13.

### easy_handeye2 / charuco_tf_publisher gotchas

- **Planar PnP front/back ambiguity** — `cv2.aruco.estimatePoseCharucoBoard`
  uses `SOLVEPNP_ITERATIVE` internally with no override. On a planar target,
  this flips between the two valid pose solutions frame-to-frame ("axes
  shaking and changing direction" in the debug image). The fix in
  `charuco_tf_node.py` is to skip `estimatePoseCharucoBoard` and call
  `cv2.solvePnPGeneric(..., flags=SOLVEPNP_IPPE)` directly on the ChArUco
  corner correspondences, then pick the lower-reprojection-error of the two
  returned solutions.
- **Marker corners need subpixel refinement** — default
  `DetectorParameters_create()` uses simple thresholding; ChArUco gets
  ~3–5× tighter pose with `params.cornerRefinementMethod =
  cv2.aruco.CORNER_REFINE_SUBPIX`. `charuco_tf_node.py` sets this by
  default; override with the `corner_refine` param (`NONE | SUBPIX |
  CONTOUR | APRILTAG`).
- **Sample diversity matters more than sample count** — AX=XB needs samples
  that rotate the board about ≥3 non-parallel axes. With only yaw (or
  pure translation), the rotation is underdetermined regardless of how
  many samples you take, and the solver settles for the smallest rotation
  that fits (near identity), with translation drifting to wherever the
  residual lands. Visible signature: rotation magnitude ≪ 90° + translation
  near origin in the saved file.
- **Arm status LED reflection** — the xArm 1S's blue power LED reflects
  off the white squares of the ChArUco board and breaks marker detection
  in the lower half of the board. Cover with opaque tape during calibration.
- **rqt plugin discovery cache** — first-time launches of
  `rqt_calibrator.py` after building easy_handeye2 fail silently with
  `qt_gui_main: found no plugin matching ...` because
  `~/.config/ros.org/rqt_gui.ini` was scanned before the plugin existed.
  Our `handeye_calibrate.launch.py` always passes `--force-discover` so
  this self-heals. Manual fix when running outside our launch:
  `rqt --force-discover` once, then close.
- **Astra driver fights for `camera_color_optical_frame`** — with
  `publish_tf: true`, the driver publishes
  `camera_color_frame → camera_color_optical_frame` *continuously at 10 Hz*
  (not as a one-shot static). Any static publisher of `world →
  camera_color_optical_frame` (`handeye_publisher`, our dummy, or a
  static_transform_publisher) is silently overridden. Workarounds:
  (a) for visualization, use a different child frame name (§8.6);
  (b) for production, set `publish_tf: false` in the Astra params and
  rebuild (§8.5).
- **`tf2_ros static_transform_publisher` argparse misreads negative
  numbers as flag names** — `--x -0.026` and even `--x=-0.026` both fail
  with `Extra unparsed arguments on command-line` in Humble. Workaround:
  put `--` between the executable name and the args, e.g.
  `ros2 run tf2_ros static_transform_publisher -- --x -0.026 ...`. Also,
  the executable dropped positional args entirely (older
  `static_transform_publisher x y z r p y parent child` syntax no longer
  works).
- **easy_handeye2 saves with `move_group: manipulator`** in the YAML, but
  this project's MoveIt planning group is `arm` (`xarm_1s.srdf`). The field
  is informational only; the saved transform is what's load-bearing, so
  this discrepancy doesn't break anything.

## 13. Taping the calibration board to `link2`

This is for the eye-to-hand calibration session (§6). Goal: rigid mount, no
slipping under arm motion, board mostly visible to the tripod camera at the
home pose.

### Materials

- The printed ChArUco PDF, glued/spray-mounted to a flat backing (foam-core,
  back of a clipboard, or stiff thin cardboard ≤2 mm thick — must not warp).
  Final assembly should be ≤5 mm thick total, ~150×210 mm.
- **Painter's tape** (preferred — easy to remove, doesn't gum up the link)
  *or* electrical tape *or* zip-ties.
- Optional: a small piece of double-sided foam tape between board and link2
  for shock absorption.

### Which face of `link2` to use

`link2` is the wrist housing — the cylindrical/boxy link just above the
gripper finger pivots. In the **home pose** (`arm1 = -1.5 rad`, others 0),
`link2`'s body axes are:

- `+Z` of `link2` ≈ world `+Z` (pointing up, away from the table).
- The gripper hangs off `link2` along `link2 -Z` (downward).
- `link2 +X` and `link2 +Y` are the two sides of the wrist orthogonal to
  the up-axis. Which side faces the tripod depends on `arm6`'s current angle.

You want the board to face the **camera**. Easiest procedure:
1. Source ROS, launch hardware (`xarm_hw_driver`) and MoveIt + RViz.
2. Move the arm to a pose where the wrist is upright and the gripper hangs
   cleanly down (close to home). Note which side of the wrist faces the
   tripod.
3. **Power down the arm motors** (or send the home pose and don't issue any
   new trajectory) so the wrist holds still while you tape.
4. Tape the board flat against the *side facing the tripod*.

### Tape pattern that doesn't slip

A single strip across the middle is not enough — torque from arm motion will
pivot the board around that axis. Use a cross or "H" pattern:

```
   ┌────────────────────┐
   │     ┌────────┐     │  ← top tape strip
   │     │        │     │
   │     │  board │     │
   │     │        │     │
   │     └────────┘     │  ← bottom tape strip
   └────────────────────┘
```

Concretely: lay the board flat on link2, then put one strip of tape across
the **top edge** (over the board onto the link), one across the **bottom
edge**, and a third **diagonally** across one corner for torsional
resistance. ~5 cm of tape per strip past the board edge onto the link.

If using zip-ties: drill or punch two small holes in the *unprinted margin*
of the board (don't damage the printed pattern), thread a zip-tie through
each hole and around link2, snug them tight. Zip-ties hold better than tape
for fast/jerky motions, but require holes in your board.

### Verify before sampling

Wiggle test:
- Push the four corners of the board with a finger. It must move with the
  wrist as one rigid unit, not flex independently.
- Move the arm slowly to one extreme of `arm6` (e.g. ±1.5 rad) while watching
  the board. If it slides or the tape lifts, redo with more tape.

Visibility test:
- In `rqt_image_view` on `/camera/color/image_raw`, confirm at the home pose
  that the board fills roughly 1/4 to 1/2 of the frame and the green
  ChArUco-corner overlay (in the calibrator's preview, once configured)
  covers most squares.

### Removal

Painter's tape peels cleanly. If you used electrical tape, residue rubs off
with isopropyl alcohol on a paper towel — don't use acetone (it pits the
plastic on the wrist housing).
