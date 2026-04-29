# xArm 1S Robot Model — Full Analysis

This document captures everything about how the xArm 1S is modelled in this workspace: world coordinate system, every link's physical dimensions, joint limits, MoveIt planning configuration, the (now eye-to-hand, tripod-mounted) camera, the cube detector frame chain, and the gaps that need to be closed before a reliable pick task can run.

> **Mount note (2026-04-28):** the camera was previously eye-in-hand, bolted to `link2`. It has since been moved onto a tripod, so the system is now **eye-to-hand**. The URDF still carries the old `depth_joint` block (§7) — that block is obsolete and will be deleted once hand-eye calibration produces a `world → camera_color_optical_frame` transform.

All numbers below come from the URDF and config files in `/home/darklord/xarm_moveit/`. File and line citations are given for everything.

---

## 1. Robot at a glance

- **Arm**: xArm 1S, 6 active revolute DOF.
- **Gripper**: parallel-jaw 4-bar linkage driven entirely by mimicking `arm1`. Not present in the MoveIt planning group.
- **Base mounting**: rigidly fixed to `world` (no mobile base).
- **Camera**: Orbbec Astra Pro RGB-D, **eye-to-hand**, fixed in `world` on a tripod. (The URDF's `link2 → depth` joint is leftover from a previous eye-in-hand setup; ignore it until §7's surgery.)
- **Control**: ROS 2, `topic_based_ros2_control` plugin, `FollowJointTrajectory` action through MoveIt.
- **Planning**: MoveIt 2 with KDL IK and OMPL/RRTConnect.

Source files:
- URDF: `src/xarm/urdf/xarm_1s.urdf.xacro`
- SRDF: `src/xarm_moveit_config/config/xarm_1s.srdf`
- Kinematics / OMPL / limits / controllers: `src/xarm_moveit_config/config/`
- Cube detector: `src/cube_detector/`
- Camera driver: `src/ros2_astra_camera/`

---

## 2. World coordinate system

Defined at `src/xarm/urdf/xarm_1s.urdf.xacro:11-17`:

```xml
<link name="world"/>

<joint name="world_joint" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.043"/>
</joint>
```

| Property | Value |
|----------|-------|
| `world` frame convention | REP-103/REP-105: X forward, Y left, **Z up** |
| `world` location | At the table surface (Z = 0 plane is the table top) |
| `world_joint` type | `fixed` |
| Translation `world` -> `base_link` | `(0, 0, 0.043)` m, i.e. 43 mm up |
| Rotation `world` -> `base_link` | identity (rpy = 0, 0, 0) |

The SRDF re-declares this as a MoveIt **virtual joint** at `src/xarm_moveit_config/config/xarm_1s.srdf:5-9`, which is how MoveIt is told the base is rigidly anchored to `world`. The MoveIt planning frame is therefore `world`.

> **Important quirk** (`urdf.xacro:39`): the first joint after the base, `arm6`, has `rpy="0.0 0.0 3.14"`. That is a **yaw flip of 180°** about Z baked into the kinematic chain. So `link6` and everything below it are rotated 180° around Z relative to `base_link`. Whenever you reason about "forward / +X" of the gripper in `world`, remember this static flip. In zero pose the gripper points opposite to `base_link +X`.

---

## 3. Kinematic chain overview

Joint and link layout (numbering is **inverted**: arm6 is the base-side joint, arm1 is the gripper-side joint):

```
world
  |  world_joint  (fixed)
base_link
  |  arm6  (Z, yaw flip 180°)
link6
  |  arm5  (Y)
link5
  |  arm4  (-Y)
link4
  |  arm3  (Y)
link3
  |  arm2  (Z)
link2 ---- depth_joint (fixed) ---- depth (OBSOLETE — eye-in-hand leftover)
  |  arm1  (X, right finger)
link1
  |  arm0  (X, mimic, right pad)
link0  (right gripper pad)

link2
  |  arm1_left  (X, mimic of arm1 with -1)
link1_left
  |  arm0_left  (X, mimic, left pad)
link0_left  (left gripper pad)
```

---

## 4. Physical link lengths

The "length" of a link in the physical world is the distance between the joint origin where it begins and the joint origin where the next joint starts. This is the magnitude of the `xyz` vector on the child joint's `<origin>` tag.

| Segment | Source line | xyz offset (m) | Magnitude (mm) | Practical meaning |
|---------|-------------|----------------|----------------|-------------------|
| `world` -> `base_link` | `urdf.xacro:16` | `(0, 0, 0.043)` | **43.0** | base-plate riser; lifts the robot 43 mm above the table |
| `base_link` -> `link6` (arm6 axis) | `urdf.xacro:39` | `(0, 0, 0.043)` | **43.0** | base shell; first axis is 43 mm above the base mounting plate |
| `link6` -> `link5` (arm5 axis) | `urdf.xacro:63` | `(0.002, 0, 0.032)` | **32.06** | shoulder yoke; tiny 2 mm X offset is mechanical shim |
| `link5` -> `link4` (arm4 axis) | `urdf.xacro:87` | `(0, 0, 0.09775)` | **97.75** | upper-arm segment 1 (the long bicep section) |
| `link4` -> `link3` (arm3 axis) | `urdf.xacro:111` | `(0, 0, 0.099)` | **99.00** | upper-arm segment 2 (forearm) |
| `link3` -> `link2` (arm2 axis) | `urdf.xacro:135` | `(-0.00125, 0, 0.050)` | **50.02** | wrist roll housing |
| `link2` -> `link1` (arm1 axis, right finger pivot) | `urdf.xacro:159` | `(-0.0015, -0.014, 0.0315)` | **34.50** | gripper carrier to right finger pivot |
| `link2` -> `link1_left` (arm1_left axis) | `urdf.xacro:182` | `(-0.0015, +0.014, 0.0315)` | **34.50** | mirror of the above for the left finger |
| `link1` -> `link0` (arm0, right pad) | `urdf.xacro:206` | `(-0.0023, 0, 0.0305)` | **30.59** | right finger to pad pivot |
| `link1_left` -> `link0_left` (arm0_left, left pad) | `urdf.xacro:230` | `(-0.0023, 0, 0.0305)` | **30.59** | mirror of the above for the left pad |
| `link2` -> `depth` (obsolete) | `urdf.xacro:255` | `(0.044, -0.0115, 0.045)` | **64.0** straight-line | leftover eye-in-hand wrist-to-camera mount; will be deleted once eye-to-hand calibration is wired in |

Cumulative Z height in the **zero pose** (every revolute joint at 0 rad):

| Frame | Cumulative Z above `world` |
|-------|-----------------------------|
| `world` | 0 mm |
| `base_link` | 43.0 mm |
| `link6` | 86.0 mm |
| `link5` | 118.0 mm |
| `link4` | 215.75 mm |
| `link3` | 314.75 mm |
| `link2` | 364.75 mm |
| `link1` | 396.25 mm |
| `link0` (right gripper pad) | **426.85 mm** |

So in the zero / fully-extended-up pose, the gripper pad sits roughly **42.7 cm above the world plane**. In the configured `home` pose (`arm1 = -1.5 rad`) the fingers are folded inward, so the actual fingertip Z is lower.

**Practical workspace envelope**: With the upper-arm + forearm + wrist contributing ~280 mm of horizontal reach (`97.75 + 99 + 50 + 31.5 ≈ 278 mm`) and joint limits of ±2.0 rad on most joints, the reachable horizontal radius from `base_link` is approximately **25-30 cm**. Useful pick zone on the table is roughly an annulus of radius 10-28 cm centred on `base_link`, at table heights from ~5 cm up to ~35 cm.

---

## 5. Joint specifications

| Joint | Source line | Type | Axis | Lower (rad) | Upper (rad) | Effort | Velocity (rad/s) | Notes |
|-------|-------------|------|------|-------------|-------------|--------|------------------|-------|
| `arm6` | `urdf.xacro:35-41` | revolute | `(0, 0, -1)` | -2.10 | +2.10 | 1000 | 1.0 | base rotation; rpy yaw=π baked in |
| `arm5` | `urdf.xacro:59-65` | revolute | `(0, 1, 0)` | -1.5708 | +1.5708 | 1000 | 1.0 | shoulder bend (±90°) |
| `arm4` | `urdf.xacro:83-89` | revolute | `(0, -1, 0)` | -2.09 | +2.09 | 1000 | 1.0 | upper-arm pitch |
| `arm3` | `urdf.xacro:107-113` | revolute | `(0, 1, 0)` | -1.85 | +2.07 | 1000 | 1.0 | elbow / forearm pitch |
| `arm2` | `urdf.xacro:131-137` | revolute | `(0, 0, 1)` | -1.95 | +1.95 | 1000 | 1.0 | wrist roll |
| `arm1` | `urdf.xacro:155-161` | revolute | `(1, 0, 0)` | -1.8326 | +1.8326 | 1000 | 1.0 | right finger; drives all gripper mimics |
| `arm1_left` | `urdf.xacro:179-185` | continuous | `(1, 0, 0)` | — | — | — | — | mimic of `arm1` with multiplier `-1`, offset `0` |
| `arm0` | `urdf.xacro:203-209` | continuous | `(1, 0, 0)` | — | — | — | — | mimic of `arm1` with multiplier `-1.01`, offset `-0.3` |
| `arm0_left` | `urdf.xacro:227-233` | continuous | `(1, 0, 0)` | — | — | — | — | mimic of `arm1` with multiplier `+1.01`, offset `+0.3` |
| `world_joint` | `urdf.xacro:13-17` | fixed | — | — | — | — | — | `world` -> `base_link` |
| `depth_joint` | `urdf.xacro:252-256` | fixed | — | — | — | — | — | `link2` -> `depth`, rpy `(-π/2, 0, -π/2)` |

Note that the `ros2_control` block (`urdf.xacro:259-317`) reports symmetric command-interface limits of ±2.094 rad for arm2-6, slightly wider than the URDF `<limit>` tags. MoveIt uses the URDF `<limit>` tags as the planning limits; the controller block is the hardware abstraction limit.

Joint-limit overrides for MoveIt are in `src/xarm_moveit_config/config/joint_limits.yaml`:

- `default_velocity_scaling_factor: 0.05`
- `default_acceleration_scaling_factor: 0.05`
- All six arm joints have `has_velocity_limits: true`, `max_velocity: 1.0`, `has_acceleration_limits: false`.
- Gripper mimic joints (`arm0`, `arm0_left`, `arm1_left`) have `has_velocity_limits: false`.

The 5% scaling means MoveIt will plan trajectories at 5% of the URDF limits by default. Bump these closer to 1.0 once you trust the setup.

---

## 6. Gripper linkage in detail

The gripper is a **single-DOF parallel-jaw mechanism** built out of four links and four joints, but driven by exactly one actuated joint (`arm1`):

| Joint | Mimic source | Multiplier | Offset (rad) | Effect |
|-------|--------------|-----------|--------------|--------|
| `arm1` | (driver) | — | — | actuated; right finger angle |
| `arm1_left` | `arm1` | -1 | 0 | mirrors right finger -> left finger opens with right |
| `arm0` | `arm1` | -1.01 | -0.3 | counter-rotates right pad to keep it parallel; -0.3 rad bias accounts for pad geometry |
| `arm0_left` | `arm1` | +1.01 | +0.3 | mirror of `arm0` for the left pad |

Closing/opening the gripper: send a JointTrajectory targeting only `arm1`. The other three joints follow automatically through the URDF's `<mimic>` directives. Because the gripper is **not** in the MoveIt `arm` planning group (see SRDF `xarm_1s.srdf:11-19`), MoveIt cannot plan or grasp through `move_group`. You must publish gripper trajectories directly to `/xarm/joint_trajectory` (or expose a separate gripper action).

There is **no explicit `tcp` / `tool0` link** in the URDF. The kinematic tip MoveIt plans for is whatever last link it walks to from the `arm` group — effectively `link2` (since `arm1` and below are excluded). Practical consequence: when you say "approach the cube from 5 cm above" in MoveIt today, that's 5 cm above `link2`, **not** 5 cm above the fingertips. You will likely add a fixed `tool0` link between the two pads (midpoint of `link0` and `link0_left`) before doing real picks.

---

## 7. Camera mount (eye-to-hand, tripod)

The Astra Pro is **fixed in the world** on a tripod, looking at the table from
above-and-in-front. Its pose in `world` is unknown until hand-eye calibration
is run (see CLAUDE.md §6). Once calibrated, the workspace will gain a fixed
joint:

```xml
<joint name="camera_mount_joint" type="fixed">
  <parent link="world"/>
  <child link="camera_color_optical_frame"/>
  <origin xyz="X Y Z" rpy="R P Y"/>   <!-- from moveit_calibration save -->
</joint>
<link name="camera_color_optical_frame"/>
```

### Obsolete eye-in-hand block (still in the URDF as of 2026-04-28)

`src/xarm/urdf/xarm_1s.urdf.xacro:251-257` still contains a leftover joint
from the previous eye-in-hand mount:

```xml
<joint name="depth_joint" type="fixed">
  <parent link="link2"/>
  <child link="depth"/>
  <origin rpy="-1.5708 0 -1.5708" xyz="0.044 -0.0115 0.045"/>
</joint>
<link name="depth" />
```

This block does nothing useful in the eye-to-hand setup and is scheduled for
deletion as part of the calibration wire-up (§11 gap #1). The `depth` link is
an orphan with no geometry; the Astra driver publishes its own
`camera_color_optical_frame` chain rooted at `camera`, which currently has
**no parent in `world`** until either a static TF or the calibrated fixed
joint is added.

Camera driver settings (`src/ros2_astra_camera/astra_camera/params/astra_pro_tuned_params.yaml`):

| Parameter | Value | Importance |
|-----------|-------|------------|
| `color_width / color_height` | 640 / 480 | matches detector intrinsics assumption |
| `color_fps` | 30 Hz | |
| `depth_align` | `true` | required: registers depth into colour pixels |
| `publish_tf` | `true` at 10 Hz | publishes camera-internal TFs |
| Sensor VID/PID | `0x2bc5:0x0501` | Sonix UVC capture path |

---

## 8. MoveIt configuration

### 8.1 SRDF (`src/xarm_moveit_config/config/xarm_1s.srdf`)

- Virtual joint `world_joint`: fixed, `world` -> `base_link` (lines 5-9).
- Planning group `arm` (lines 12-19): contains `arm1`, `arm2`, `arm3`, `arm4`, `arm5`, `arm6`.
  - Note: gripper mimics (`arm0`, `arm0_left`, `arm1_left`) are deliberately excluded.
  - There is **no `<end_effector>` group** declared. MoveIt has no notion of a gripper.
- Named pose `home` (lines 21-28): `arm1=-1.5, arm2=0, arm3=0, arm4=0, arm5=0, arm6=0` — fingers folded back, arm pointing straight up otherwise.
- Disable-collision pairs (lines 32-61): adjacency rules for base, gripper linkage, and serial arm chain. Looks complete for the current model.

### 8.2 Kinematics (`src/xarm_moveit_config/config/kinematics.yaml`)

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

5 ms is **too aggressive** for a 6-DOF arm whose IK can have local minima. Bump to **0.05-0.1 s** to avoid spurious "no IK solution" failures during pick planning.

### 8.3 OMPL (`src/xarm_moveit_config/config/ompl_planning.yaml`)

Default planner pipeline is OMPL with **RRTConnect** as the primary algorithm. Reasonable for short pick-and-place motions. Check this file if you want to add CHOMP or STOMP for smoother trajectories.

### 8.4 Joint limits (`src/xarm_moveit_config/config/joint_limits.yaml`)

- Default velocity scaling: **5%**.
- Default acceleration scaling: **5%**.
- Acceleration limits not set (`has_acceleration_limits: false`) — MoveIt will use planner defaults.

### 8.5 Controllers (`src/xarm_moveit_config/config/moveit_controllers.yaml`)

```yaml
xarm_1s_arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  default: true
  joints: [arm1, arm2, arm3, arm4, arm5, arm6]
```

MoveIt sends trajectories to `xarm_1s_arm_controller/follow_joint_trajectory`. Underneath, that controller is a `topic_based_ros2_control` system bridging to `/xarm/joint_trajectory` and reading back `/xarm/joint_states` (URDF lines 265-266).

---

## 9. Cube detector and frame chain to MoveIt

### 9.1 Detector configuration (`src/cube_detector/config/cube_detector.yaml`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `detection_mode` | `rgb_only` | RGB + Canny edges + IPPE-SQUARE PnP. The README recommends `depth_fusion` instead (RGB + RANSAC table plane + depth) |
| `multi_cube` | `true`, `max_cubes: 3` | publishes `~/poses` (PoseArray) plus best cube on `~/pose` |
| `cube_size` | 0.025 m | 25 mm cube |
| `height_min` | 0.005 m | ignore objects within 5 mm of the table plane (depth_fusion only) |
| `height_max_tol` | 0.010 m | tolerance above cube height |
| `cube_frame_id` | `cube` | TF child published on detection |
| `publish_tf` | `true` | broadcasts the cube as a TF frame |
| `min_score` | 0.45 | reject low-quality polygons |
| `temporal_alpha` | 0.4 | IIR smoothing (single-cube only; disabled in multi mode) |
| `processing_period_s` | 0.1 | 10 Hz max detection rate |
| `rgb_min_area_px / rgb_max_area_px` | 80 / 8000 | pixel area band tuned for 25 mm cube at 30-90 cm with fx ≈ 525 |

Topics:
- in: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`
- out: `~/pose`, `~/poses`, `~/marker`, `~/markers`, `~/debug_image`, plus a TF broadcast `<camera frame> -> cube`

### 9.2 Frame the cube pose is expressed in

`src/cube_detector/cube_detector/detector_node.py:344-347`:

```python
header.frame_id = (src_header.frame_id or 'camera_color_optical_frame')
```

So the detector emits cube poses in **whatever frame the colour image header uses**, defaulting to `camera_color_optical_frame`. Camera optical convention: X right, Y down, Z forward.

### 9.3 Required TF chain to plan a pick (eye-to-hand)

For MoveIt to pick the cube, you need a complete TF chain from the planning frame `world` down to `cube`. With the camera now fixed in `world` (tripod), the chain splits into two branches that meet only at `cube`:

```
world
  -> base_link               (fixed, world_joint)
  -> link6 ... link2 ... gripper       (the arm chain — irrelevant for pose lookup of the cube,
                                        but needed for MoveIt planning)

world
  -> camera_color_optical_frame     (fixed, hand-eye calibration result; NOT YET PUBLISHED)
  -> cube                            (broadcast by cube_detector when publish_tf=true)
```

The missing edge is **`world → camera_color_optical_frame`**, produced by the eye-to-hand calibration session in CLAUDE.md §6. Before that runs, the `cube` frame is disconnected from `world` and MoveIt cannot reach for it. Three ways to supply this edge:
  1. A `static_transform_publisher` with the calibrated values (good for testing).
  2. A fixed joint in the URDF (recommended once values are stable; see §7).
  3. *(Old, eye-in-hand-only:)* a `link2 → camera_color_optical_frame` chain via the obsolete `depth_joint`. **No longer applicable** — delete that block.

---

## 10. End-to-end usage in MoveIt and a pick task

### 10.1 What MoveIt currently does with this model

1. **Loads URDF + SRDF**: builds the `arm` group with 6 joints, anchors `world -> base_link` via the virtual joint, applies the disable-collision matrix.
2. **IK queries**: KDL plugin solves IK from a target pose for the tip of the `arm` group. Tip = last link reachable from the planning group, effectively `link2` (because gripper joints are excluded). Search resolution 5 mrad, timeout 5 ms.
3. **Motion planning**: OMPL/RRTConnect produces a joint-space trajectory between current state and the target.
4. **Trajectory execution**: scales by `default_velocity_scaling_factor = 0.05`, sends `FollowJointTrajectory` goals to `xarm_1s_arm_controller`, which forwards to the `topic_based_ros2_control` plugin -> `/xarm/joint_trajectory` -> physical hardware via `xarm_hw`.
5. **Collision checking**: full state read from `robot_state_publisher`, including gripper state via the mimic joints. Disabled pairs from the SRDF skip the obvious adjacency checks.

### 10.2 What a pick task needs to add

| Missing piece | Why | Where to add |
|---------------|-----|--------------|
| Eye-to-hand calibration / `world -> camera_color_optical_frame` fixed joint | Cube poses cannot reach `world` otherwise (camera pose in world is unknown) | run `moveit_calibration` (eye-in-hand checkbox UNCHECKED) → save → patch URDF |
| Explicit `tcp` / `tool0` frame at the gripper midpoint | MoveIt approach poses target `link2`, not the fingertips | URDF: add a fixed link between `link0` and `link0_left` |
| Increased IK timeout (>=0.05 s) | KDL fails too often at 5 ms for 6-DOF | `kinematics.yaml` |
| Realistic velocity / acceleration scaling | 5% default makes picks crawl | `joint_limits.yaml` (per-call override is also fine) |
| Gripper action / open-close routine | MoveIt cannot drive the gripper directly | application code, sending JointTrajectory on `arm1` |
| Pre-defined named poses (`ready`, `pre_grasp`, `retract`) | Convenience, reproducibility | `xarm_1s.srdf` |
| Optional: switch detector to `depth_fusion` mode | More robust to lighting and clutter; uses the table-plane RANSAC | `cube_detector.yaml` `detection_mode` |

### 10.3 Effect of the `arm6` 180° yaw flip on planning

Because `arm6` has `rpy="0 0 3.14"` baked into its origin, the gripper's body-frame X axis at `q=0` points opposite to `base_link`'s X. When you compute a target pose from a cube observed in `world`:
- The yaw of the cube in `world` may need a 180° offset before being sent as the gripper target, depending on which side the gripper opens.
- IK solutions will exist for both orientations, but the wrist may prefer one branch. If you see "unreachable" errors at the edge of the workspace, try flipping the requested yaw by π.

### 10.4 Workspace summary for a pick

- **Cube must lie within roughly a 10-28 cm radius annulus around the projection of `base_link` onto the table**, at table height (Z just above 0 in `world`).
- **At zero pose** the gripper is at ~427 mm above the table; the home pose folds the fingers, bringing the effective fingertip Z lower.
- **Approach orientation**: top-down picks (gripper Z aligned with `world -Z`) are easiest because they stay clear of the wrist's joint limits. Side picks are reachable but check `arm5` ±π/2 limit.
- **Cube size**: 25 mm. Gripper opening must be configured to allow >25 mm open and clamp to ~22-23 mm closed for a firm grip.

---

## 11. Critical gaps before live picking

Ranked by how badly they will hurt a real pick:

1. **No `world` -> `camera_color_optical_frame` TF.** With the camera now on a tripod the pose is unknown until eye-to-hand calibration runs. Without this edge `cube` is disconnected from `world` and `tf2_echo world cube` fails. Fix: run `moveit_calibration` (eye-in-hand checkbox **unchecked**) and add the saved transform either as a static_transform_publisher or — preferred — a fixed joint in the URDF. While at it, delete the obsolete `depth_joint` block at `xarm_1s.urdf.xacro:251-257`. Verify with:
   ```bash
   ros2 run tf2_ros tf2_echo world camera_color_optical_frame
   ros2 run tf2_ros tf2_echo world cube
   ```
2. **No explicit TCP frame.** All approach / retreat reasoning is currently relative to `link2`. Add a `tool0` link at the gripper midpoint.
3. **5 ms IK timeout.** Will produce intermittent IK failures in cluttered workspace.
4. **5% velocity scaling.** Safe but slow; revisit before timing tests.
5. **Detector in `rgb_only` mode** while the README marks `depth_fusion` as recommended. RGB-only is more sensitive to shadows and false positives.
6. **No gripper action.** Need a small node that opens/closes `arm1` between MoveIt approach and retreat.

---

## 12. Quick reference — file map

| File | Purpose |
|------|---------|
| `src/xarm/urdf/xarm_1s.urdf.xacro` | Robot description: links, joints, gripper, ros2_control block. Lines 251-257 hold an obsolete eye-in-hand `depth_joint` — delete and replace with a `world → camera_color_optical_frame` fixed joint after calibration. |
| `src/xarm_moveit_config/config/xarm_1s.srdf` | Planning group, virtual joint, named poses, collision matrix |
| `src/xarm_moveit_config/config/kinematics.yaml` | IK solver (KDL), search resolution, timeout |
| `src/xarm_moveit_config/config/ompl_planning.yaml` | OMPL planner pipeline |
| `src/xarm_moveit_config/config/joint_limits.yaml` | Per-joint velocity / acceleration limits and default scaling |
| `src/xarm_moveit_config/config/moveit_controllers.yaml` | Controller bridge: FollowJointTrajectory on `xarm_1s_arm_controller` |
| `src/xarm_moveit_config/config/pilz_cartesian_limits.yaml` | Pilz industrial planner limits (alternative pipeline) |
| `src/cube_detector/config/cube_detector.yaml` | Detection mode, intrinsics-derived pixel band, RANSAC, multi-cube |
| `src/cube_detector/cube_detector/detector.py` | Vision pipeline (RGB / depth_fusion implementations) |
| `src/cube_detector/cube_detector/detector_node.py` | ROS 2 node; publishes pose, poses, markers, TF |
| `src/ros2_astra_camera/astra_camera/params/astra_pro_tuned_params.yaml` | Camera 640x480@30, depth_align=true, publish_tf=true |
| `src/xarm_hw/xarm_hw/driver.py` | USB driver; bridges `/xarm/joint_trajectory` to hardware |

---

## 13. TL;DR

- The robot is a 6-DOF xArm 1S, anchored 43 mm above the `world` frame on the table.
- Link lengths, in order from base to gripper: 43, 43, 32, 98, 99, 50, 35 mm, plus a 31 mm gripper finger and 31 mm gripper pad. Total kinematic stack from world to fingertip in zero pose ≈ **427 mm**.
- A 180° Z-axis rotation is baked into `arm6`, so gripper "forward" is opposite to `base_link +X`.
- The camera is **eye-to-hand** on a tripod, fixed in `world`. The URDF's old `link2 → depth` joint (a leftover eye-in-hand mount) is obsolete and slated for deletion.
- MoveIt plans the 6 arm joints with KDL/OMPL; the gripper is outside the planning group and must be commanded separately through `arm1` and its mimics.
- The single most important gap before picking works end-to-end is the missing `world` → `camera_color_optical_frame` transform (eye-to-hand calibration output). Everything else is tuning.
