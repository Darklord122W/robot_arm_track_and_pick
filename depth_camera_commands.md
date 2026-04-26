# Depth Camera (Orbbec Astra) — Useful Commands

Quick reference for the `ros2_astra_camera` package in this workspace.
All topics/services live under the `/camera` namespace by default.

Sections progress through the typical workflow:
**setup → run → inspect → configure → capture → tune → troubleshoot.**

| Part | Sections | Goal |
|---|---|---|
| I  — Setup & run            | §1–3   | Get the camera streaming |
| II — Inspect                | §4–6   | Read topics, intrinsics, TF |
| III — Configure at runtime  | §7–9   | Services that change the live device |
| IV — Static config          | §10    | `params/*.yaml` edits + rebuild |
| V  — Capture & process      | §11–12 | Bag recording, point-cloud filtering |
| VI — Reduce depth noise     | §13    | Astra Pro tuned launch |
| VII — Troubleshooting       | §14    | When something is wrong |

---

# Part I — Setup & run

## 1. Environment setup

Source these in every new terminal before running camera commands:

```bash
source /opt/ros/humble/setup.bash
source ~/xarm_moveit/install/setup.bash
```

Optional (recommended) DDS env:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

Rebuild after changing launch files or `params/*.yaml`:

```bash
cd ~/xarm_moveit
colcon build --packages-select astra_camera astra_camera_msgs \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 2. Launch the camera

Pick the launch that matches your hardware:

```bash
ros2 launch astra_camera astra_mini.launch.py        # Astra Mini / Astra Pro (vanilla)
ros2 launch astra_camera astra_pro_tuned.launch.py   # Astra Pro with depth-noise tuning (see §13)
```

Multi-camera variants (edit the matching `params/*.yaml` with serial numbers first):

```bash
ros2 launch astra_camera multi_astra_mini.launch.py
```

List connected devices and get their serial numbers:

```bash
ros2 run astra_camera list_devices_node
```

---

## 3. Visualize

RViz2 with the bundled point cloud config:

```bash
rviz2 -d ~/xarm_moveit/src/ros2_astra_camera/astra_camera/rviz/pointcloud.rviz
```

Multi-camera RViz config:

```bash
rviz2 -d ~/xarm_moveit/src/ros2_astra_camera/astra_camera/rviz/multi_camera.rviz
```

Quick 2D viewers (no RViz needed):

```bash
ros2 run rqt_image_view rqt_image_view            # GUI, pick any image topic
ros2 run image_view image_view --ros-args -r image:=/camera/color/image_raw
ros2 run image_view image_view --ros-args -r image:=/camera/depth/image_raw
```

> **QoS tip:** default publisher QoS is `Best Effort`. In RViz2, set the image/PointCloud2 display's *Reliability Policy* to `Best Effort`, or it will silently display nothing.

---

# Part II — Inspect

## 4. Topics, services, params

```bash
ros2 topic list
ros2 topic list | grep /camera
ros2 service list | grep /camera
ros2 param list /camera/camera
```

Common topics:

```bash
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
/camera/depth/camera_info
/camera/depth/points          # organized point cloud (XYZ)
/camera/depth/color/points    # colored point cloud (needs depth_align: true)
/camera/ir/image_raw
```

Inspect a topic (rate, type, QoS):

```bash
ros2 topic hz   /camera/color/image_raw
ros2 topic bw   /camera/depth/image_raw
ros2 topic info -v /camera/depth/points
ros2 topic echo --once /camera/depth/camera_info
```

---

## 5. Camera intrinsics & extrinsics

```bash
# Intrinsics
ros2 topic echo /camera/color/camera_info
ros2 topic echo /camera/depth/camera_info
ros2 topic echo /camera/ir/camera_info

# Depth → color extrinsics (latched)
ros2 topic echo --qos-durability=transient_local \
  /camera/extrinsic/depth_to_color --qos-profile=services_default

# Full camera params + device info
ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo '{}'
ros2 service call /camera/get_sdk_version  astra_camera_msgs/srv/GetString '{}'
```

---

## 6. TF frames

The driver publishes TF from `camera_link` to `camera_depth_frame`, `camera_color_frame`, etc.

```bash
ros2 run tf2_tools view_frames          # writes frames.pdf in cwd
ros2 run tf2_ros   tf2_echo camera_link camera_depth_optical_frame
```

---

# Part III — Configure at runtime (services)

> **Apply order matters.** Services are presented in the order you should call them: toggles → LDP/laser/fan → exposure/gain. LDP toggling internally restarts streams, which would wipe any exposure / gain values you set first.

## 7. Toggle individual sensors

```bash
ros2 service call /camera/toggle_color       std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/toggle_depth       std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/toggle_ir          std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/toggle_uvc_camera  std_srvs/srv/SetBool '{data: true}'
```

---

## 8. Laser, fan, LDP, mirror

```bash
# Laser (IR projector) on/off
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: false}'

# Fan
ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: true}'

# LDP (Laser Detect Protection) — disable for static scenes; LDP can cut
# the laser intermittently and cause depth dropouts. Toggling LDP
# internally calls stopStreams/startStreams, so set it BEFORE pinning
# exposure / gain in §9 (otherwise those settings get wiped).
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'

# Mirror each stream
ros2 service call /camera/set_color_mirror     std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ir_mirror        std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_depth_mirror     std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_uvc_color_mirror std_srvs/srv/SetBool '{data: true}'
```

---

## 9. Exposure, gain, white balance

Auto-exposure must be **off** before setting manual values.

```bash
# Disable auto-exposure first
ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}'
ros2 service call /camera/set_ir_auto_exposure    std_srvs/srv/SetBool '{data: false}'
ros2 service call /camera/set_uvc_auto_exposure   std_srvs/srv/SetBool '{data: false}'

# Set exposure
ros2 service call /camera/set_color_exposure astra_camera_msgs/srv/SetInt32 '{data: 2000}'
ros2 service call /camera/set_ir_exposure    astra_camera_msgs/srv/SetInt32 '{data: 2000}'
ros2 service call /camera/set_uvc_exposure   astra_camera_msgs/srv/SetInt32 '{data: 2000}'

# Read exposure back
ros2 service call /camera/get_color_exposure astra_camera_msgs/srv/GetInt32 '{}'
ros2 service call /camera/get_ir_exposure    astra_camera_msgs/srv/GetInt32 '{}'
ros2 service call /camera/get_uvc_exposure   astra_camera_msgs/srv/GetInt32 '{}'

# Gain
ros2 service call /camera/set_color_gain astra_camera_msgs/srv/SetInt32 '{data: 200}'
ros2 service call /camera/set_ir_gain    astra_camera_msgs/srv/SetInt32 '{data: 200}'
ros2 service call /camera/set_uvc_gain   astra_camera_msgs/srv/SetInt32 '{data: 200}'
ros2 service call /camera/get_color_gain astra_camera_msgs/srv/GetInt32 '{}'
```

> **Astra Pro gotcha — `set_ir_*` vs `set_depth_*`:** the driver only registers exposure / gain / auto-exposure services for *enabled* streams. With the default `enable_ir: false`, the `set_ir_*` services do **not** exist — but `set_depth_*` controls the same underlying IR-sensor device property. Use `set_depth_exposure`, `set_depth_gain`, `set_depth_auto_exposure` whenever IR is disabled.

```bash
# IR-sensor controls when enable_ir is false (Astra Pro default)
ros2 service call /camera/set_depth_auto_exposure std_srvs/srv/SetBool '{data: false}'
ros2 service call /camera/set_depth_exposure      astra_camera_msgs/srv/SetInt32 '{data: 2000}'
ros2 service call /camera/set_depth_gain          astra_camera_msgs/srv/SetInt32 '{data: 64}'
ros2 service call /camera/get_depth_gain          astra_camera_msgs/srv/GetInt32 '{}'

# Auto white balance
ros2 service call /camera/set_color_auto_white_balance std_srvs/srv/SetBool '{data: false}'
ros2 service call /camera/get_color_auto_white_balance astra_camera_msgs/srv/GetInt32 '{}'
```

---

# Part IV — Static config

## 10. Config quick edits

Params live in `src/ros2_astra_camera/astra_camera/params/<model>_params.yaml`.
Common tweaks:

```yaml
color_width: 640
color_height: 480
color_fps: 30

depth_width: 640
depth_height: 480
depth_fps: 30

depth_align: true     # REQUIRED for colored point clouds
enable_ir: false
serial_number: ""     # fill in for multi-camera
```

For the Astra Pro noise-reduction config, see also `params/astra_pro_tuned_params.yaml` (used by `astra_pro_tuned.launch.py` — §13).

After editing the yaml, **rebuild** — the runtime reads from `install/`, not `src/`:

```bash
colcon build --packages-select astra_camera
```

---

# Part V — Capture & process

## 11. Record & replay

Record a handful of topics for offline debugging:

```bash
ros2 bag record \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/depth/points \
  /camera/color/camera_info \
  /camera/depth/camera_info \
  /tf /tf_static \
  -o depth_session
```

Play back:

```bash
ros2 bag play depth_session
ros2 bag info depth_session
```

---

## 12. Point-cloud processing quick-hits

Grab one point-cloud message to disk:

```bash
ros2 topic echo --once /camera/depth/points > /tmp/cloud.yaml
```

Filter a point cloud with PCL voxel grid (if `pcl_ros` is installed):

```bash
ros2 run pcl_ros voxel_grid_node --ros-args \
  -r input:=/camera/depth/points \
  -r output:=/camera/depth/points_voxel \
  -p leaf_size:=0.01
```

---

# Part VI — Reduce depth noise (Astra Pro tuned launch)

## 13. Reduce depth noise

This section documents the tuning workflow added to fix the five common Astra Pro noise modes: **flickering / temporal jitter**, **holes / dropouts**, **fuzzy thick edges**, **wavy bias on flat surfaces**, and **salt-and-pepper random pixels**.

### 13.1 What was added

| File | Purpose |
|---|---|
| `params/astra_pro_tuned_params.yaml` | Astra Pro params with a physical-setup checklist in the header |
| `scripts/apply_astra_tuning.py` | One-shot ROS2 node that applies driver settings via `ros2 service call` |
| `launch/astra_pro_tuned.launch.py` | Launches the driver with tuned params + runs the tuner |
| `CMakeLists.txt` | New `install(PROGRAMS …)` rule for the tuner script |
| `package.xml` | Added `<exec_depend>rclpy</exec_depend>` |

### 13.2 One-line usage

```bash
ros2 launch astra_camera astra_pro_tuned.launch.py
```

Override any value at launch:

```bash
ros2 launch astra_camera astra_pro_tuned.launch.py \
  depth_exposure:=4000 depth_gain:=64
```

Available launch args (defaults shown):

| Arg | Default | Meaning |
|---|---|---|
| `depth_exposure` | `2000` | IR-sensor integration time. Sweep `1000–8000`. |
| `depth_gain` | `200` | IR-sensor analog gain. Lower = less speckle. |
| `laser_enable` | `true` | IR projector on. Required for depth. |
| `ldp_enable` | `false` | LDP off — avoids intermittent laser cutoff. |
| `fan_enable` | `true` | Active cooling — reduces thermal drift / wavy bias. |
| `depth_auto_exposure` | `false` | AE off — locks the IR-sensor exposure for static scenes. |
| `camera_namespace` | `camera` | Service-namespace prefix. |

### 13.3 What the tuner actually does (and why this order)

The tuner shells out to `ros2 service call` (subprocess) — same path as the README examples — to avoid rclpy + composable-container quirks. Calls run in this order:

1. `set_ldp_enable false` — **must run first.** The driver's LDP callback internally does `stopStreams() → setProperty → startStreams()`, which would wipe any later exposure / gain settings.
2. `set_laser_enable true` — ensure IR projector is on.
3. `set_fan_mode true` — reduces sensor thermal drift, the dominant cause of the wavy-bias artifact on flat surfaces.
4. `set_depth_auto_exposure false` — kills auto-exposure hunting (a major source of temporal jitter).
5. `set_depth_exposure <N>` — fixed integration time.
6. `set_depth_gain <N>` — fixed analog gain.

The tuner uses `set_depth_*` (not `set_ir_*`) because the params file has `enable_ir: false`, and the driver only registers exposure / gain services for enabled streams. Both names target the same device-level OpenNI property (`OBEXTENSION_ID_IR_EXP` / `OBEXTENSION_ID_IR_GAIN`).

### 13.4 Mapping each setting → noise it reduces

| Setting | Noise mode addressed |
|---|---|
| `depth_auto_exposure: false` + fixed exposure | Temporal flicker on static scenes |
| Lower `depth_gain` | Salt-and-pepper, fuzzy edges |
| Higher `depth_exposure` (within IR-pattern saturation) | Holes / dropouts on dim or far surfaces |
| `fan_enable: true` + 5–10 min warm-up | Wavy bias on flat surfaces (thermal drift) |
| `ldp_enable: false` | Intermittent dropouts (LDP turning off the laser) |
| `laser_enable: true` | All depth (no laser → no depth) |

### 13.5 Tuning the IR sensor empirically

Watch the IR pattern in `/camera/ir/image_raw` (set `enable_ir: true` temporarily) or just `/camera/depth/image_raw`. Dots should be crisp and bright but **not saturated** (no white blooming).

```bash
# Sweep exposure with gain held low
for v in 1500 2500 4000 6000 8000; do
  ros2 service call /camera/set_depth_exposure astra_camera_msgs/srv/SetInt32 "{data: $v}"
  sleep 1
done

# Then sweep gain to taste
for v in 8 32 64 96 128 200; do
  ros2 service call /camera/set_depth_gain astra_camera_msgs/srv/SetInt32 "{data: $v}"
  sleep 1
done

# Read back to detect firmware clipping
ros2 service call /camera/get_depth_gain astra_camera_msgs/srv/GetInt32 '{}'
```

#### Gain range reference

The driver does **no clamping** — it forwards any int to the device, which silently clips out-of-range values. Empirically:

| Gain | Effect |
|---|---|
| 8–32 | Low noise, needs more exposure |
| 64–96 | Balanced — most scenes land here |
| 128+ | Visible speckle |
| 200+ | Noise dominates (only useful in very dim setups) |

Default after power-on is typically 16–32. If you set a value and `get_depth_gain` returns something different, you found the clip ceiling.

### 13.6 Physical-setup checklist (do these first — biggest impact)

Tuning can't fix what physics breaks. Verify before tuning:

- [ ] **Block direct sunlight** — the single largest source of dropouts and flicker. Sunlight IR swamps the structured-light pattern.
- [ ] **Warm up 5–10 min** before capture — reduces wavy bias.
- [ ] **Target between 0.6 m and 2.5 m** — Astra Pro sweet spot.
- [ ] **Matte, textured surfaces** — glossy / matte-black / translucent surfaces are physically hard for structured light.
- [ ] **No other IR sources** — other Astras, IR heaters, IR remotes interfere.
- [ ] Fluorescent lighting is fine (slight flicker, usually negligible at 30 fps); LED is best.

### 13.7 What this does NOT do

- **No spatial / temporal / edge-preserving filtering** is applied — the driver doesn't expose those. If raw depth is still too noisy after sections 13.5 + 13.6, the next step is a post-processing node (temporal median + bilateral + speckle filter) chained after `/camera/depth/image_raw`. Not yet built.
- **No intrinsic recalibration** — the Astra Pro factory calibration is in firmware. Recalibration requires Orbbec's `OBCalibration` tool and is rarely the cause of noise (it would cause geometric error, not jitter).

---

# Part VII — Troubleshooting

## 14. Troubleshooting

| Symptom | Check |
|---|---|
| RViz2 shows nothing | Set display Reliability to **Best Effort** |
| `list_devices_node` sees nothing | Reinstall udev rules: `sudo bash src/ros2_astra_camera/astra_camera/scripts/install.sh && sudo udevadm control --reload-rules && sudo udevadm trigger`, then replug USB |
| No colored point cloud | `depth_align: true` in the params yaml, then rebuild |
| Multi-cam: one camera black | Use a **powered** USB hub; don't chain through the same bus; lower resolution |
| Laggy or dropped frames | Lower `*_fps` / resolution; check `ros2 topic hz`; tune CycloneDDS |
| Driver silently exits | `ros2 launch ... --ros-args --log-level debug`; check `dmesg` for USB errors |

Verify the USB device is enumerating:

```bash
lsusb | grep -i orbbec
dmesg | tail -30
```
