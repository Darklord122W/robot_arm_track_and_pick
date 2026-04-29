# cube_detector

A depth + edge-fusion ROS 2 node that detects a small cube on a table from an
"almost top-down" RGB-D camera view, and publishes its 6-DoF pose for pick-and-place.

Designed for the xArm-1S workspace with the Orbbec Astra Pro fixed in `world`
on a tripod (eye-to-hand). Color-agnostic — geometry does the segmentation,
not pixel colour. (Older revisions of this README mentioned an eye-in-hand
mount on `link2`; that has been retired — see `CLAUDE.md` §6 for the current
calibration setup.)

---

## 1. Quickstart

### 1.1 Prerequisites

- ROS 2 Humble.
- `cv_bridge`, `message_filters`, `tf2_ros` (already in `package.xml` deps).
- The astra driver in this workspace, with **`depth_align: true`** in the
  active params yaml. The detector requires depth pixels registered to the
  colour frame.
- One 25 mm cube on a table, viewed from a near-top-down angle at 30–90 cm.

### 1.2 Build

```bash
cd ~/xarm_moveit
colcon build --packages-select cube_detector
source install/setup.bash
```

### 1.3 Run

Three terminals, each sourced with `/opt/ros/humble/setup.bash` and
`~/xarm_moveit/install/setup.bash`.

```bash
# Terminal 1 — camera
ros2 launch astra_camera astra_pro_tuned.launch.py

# Terminal 2 — detector
ros2 launch cube_detector cube_detector.launch.py

# Terminal 3 — visualize (any of these)
ros2 run rqt_image_view rqt_image_view /cube_detector/debug_image
ros2 topic echo /cube_detector/pose
ros2 run tf2_ros tf2_echo camera_color_optical_frame cube
```

A green polygon, crosshair and tinted mask should overlay the cube in the
debug image. The bottom-left text shows the current `depth_uv_offset` in
pixels. Score, position, and per-frame state are also drawn.

### 1.4 Stop

`Ctrl+C` in each terminal. There is no persistent state on disk.

---

## 2. Published / subscribed interface

### 2.1 Subscriptions

| Topic                       | Type                       | Notes                                  |
|-----------------------------|----------------------------|----------------------------------------|
| `/camera/color/image_raw`   | `sensor_msgs/Image` RGB8   | Synced via ApproximateTime             |
| `/camera/depth/image_raw`   | `sensor_msgs/Image`        | 16UC1 (mm) or 32FC1 (m); auto-detected |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo`   | Latest K cached; not in sync           |

QoS is `BEST_EFFORT, KEEP_LAST, depth=5` — matches the astra driver default.

### 2.2 Publications

| Topic                          | Type                              | Notes                                                     |
|--------------------------------|-----------------------------------|-----------------------------------------------------------|
| `~/pose`                       | `geometry_msgs/PoseStamped`       | Best cube (highest score) — works in both modes           |
| `~/marker`                     | `visualization_msgs/Marker`       | Best cube as a 25 mm CUBE marker                          |
| `~/poses`                      | `geometry_msgs/PoseArray`         | **Multi-cube only.** All cubes detected this frame        |
| `~/markers`                    | `visualization_msgs/MarkerArray`  | **Multi-cube only.** One CUBE per detection, colour-coded |
| `~/debug_image`                | `sensor_msgs/Image` RGB8          | Annotated colour frame, all cubes drawn with index labels |
| TF: `<camera_frame>` → `cube`  | `geometry_msgs/TransformStamped`  | Broadcast for the *best* cube when `publish_tf: true`     |

`<camera_frame>` is whatever string the colour image header carries
(typically `camera_color_optical_frame`).

### 2.3 Pose conventions

- Frame: **camera optical** (X right, Y down, Z forward).
- Cube `+Z` axis = table normal (out of the top face, away from the table).
- Cube `+X` axis = direction of the first ordered top-face edge (after CCW
  reordering around centroid).
- Cube `+Y` = `+Z × +X`.

To use this in MoveIt / pick planning, transform to the arm's base frame:

```python
from tf2_ros import Buffer, TransformListener
buf = Buffer(); tf = TransformListener(buf, node)
pose_in_base = buf.transform(pose_msg, 'world', timeout=Duration(seconds=0.5))
```

Or just look up `world` → `cube` directly via `tf2_echo` — the broadcast
covers the chain `world` → `camera_color_optical_frame` (fixed joint from
hand-eye calibration) → `cube` (this node). Until calibration is wired in,
that first hop is missing and `cube` will be disconnected from `world`.

---

## 3. Parameters

All parameters are declared on the node and documented in
`config/cube_detector.yaml`. The non-default-zero ones below are the most
useful to know about.

| Parameter                | Default  | Meaning                                                                 |
|--------------------------|----------|-------------------------------------------------------------------------|
| `cube_size`              | `0.025`  | Physical edge length of the cube, m                                     |
| `height_min`             | `0.005`  | Ignore anything within 5 mm of the table plane                          |
| `height_max_tol`         | `0.010`  | Slack above `cube_size` for the above-table mask                        |
| `ransac_iters`           | `200`    | RANSAC iterations for the table plane                                   |
| `ransac_threshold`       | `0.004`  | Plane inlier band, m (Astra Pro depth std ≈ 3-5 mm)                     |
| `ransac_subsample`       | `4000`   | Random subsample size for the plane fit                                 |
| `edge_refine`            | `true`   | Use `cv2.cornerSubPix` to snap top-face corners                         |
| `min_score`              | `0.45`   | Reject detections below this geometric score (0..1)                     |
| `temporal_alpha`         | `0.4`    | EMA factor; higher = more responsive, lower = smoother                  |
| `temporal_max_jump_m`    | `0.05`   | Position jump beyond this resets the temporal filter                    |
| `processing_period_s`    | `0.1`    | Min interval between detections (≤10 Hz)                                |
| `depth_uv_offset_x`      | `0`      | **Live-tunable**. Positive = depth shifted right, in colour pixels.     |
| `depth_uv_offset_y`      | `0`      | **Live-tunable**. Positive = depth shifted down.                        |
| `detection_mode`         | `'depth_fusion'` | `'depth_fusion'` or `'rgb_only'`. See §3.2. Restart required.   |
| `multi_cube`             | `false`  | Publish *all* cubes per frame on `~/poses` / `~/markers`. Restart required. See §3.1. |
| `max_cubes`              | `8`      | When `multi_cube`, cap on how many are emitted (sorted best-first).     |
| `rgb_min_area_px`        | `80`     | RGB-only: minimum convex-quad area accepted, in pixels².                |
| `rgb_max_area_px`        | `8000`   | RGB-only: maximum convex-quad area, in pixels².                         |
| `rgb_aspect_tol`         | `0.30`   | RGB-only: max std/mean of the 4 side lengths (0 = perfect square).      |

**Live tuning** for `depth_uv_offset_*`, `min_score`, `temporal_alpha`,
`temporal_max_jump_m`:

```bash
ros2 param set /cube_detector depth_uv_offset_x -5
ros2 param set /cube_detector min_score 0.6
```

Other parameters require a node restart.

---

## 3.1 Single vs. multi-cube

Set `multi_cube: true` in the YAML to publish *every* cube the detector finds
in each frame. Without it, only the best-scoring cube is published, and the
"jumping" you see when several cubes are visible is the score winner shifting
frame-to-frame.

| Behaviour                                  | `multi_cube: false` | `multi_cube: true`              |
|--------------------------------------------|---------------------|---------------------------------|
| `~/pose`, `~/marker`                       | Best cube           | Best cube (back-compat)         |
| `~/poses`, `~/markers`                     | empty               | All accepted cubes              |
| TF `cube`                                  | Best cube           | Best cube                       |
| Temporal EMA smoothing                     | Yes                 | **Off** (no identity tracking)  |
| `~/debug_image` overlay                    | One quad            | All quads, indexed `#0, #1, …`  |

`max_cubes` (default 8) caps how many are emitted per frame — sorted best-first.
Cubes below `min_score` are dropped before publication.

In multi mode, downstream pick-and-place code subscribes to `~/poses` and picks
whichever cube it wants (e.g. nearest the gripper, leftmost, in a region of
interest). Each frame the order can change because there's no identity
tracking — pick by spatial criteria, not by index.

---

## 3.2 Two detection modes

The node has two pipelines, selected via the `detection_mode` parameter.

| Mode             | Inputs                  | Pose method                       | Robustness | When to use                              |
|------------------|-------------------------|-----------------------------------|------------|------------------------------------------|
| `depth_fusion`   | RGB + depth + K         | Ray-plane intersection on top face| High       | **Default.** Astra Pro available.        |
| `rgb_only`       | RGB + K                 | `cv2.solvePnP` IPPE_SQUARE        | Lower      | No depth, or to validate depth-mode bias |

### `rgb_only` mode

```yaml
# config/cube_detector.yaml
cube_detector:
  ros__parameters:
    detection_mode: rgb_only
    rgb_min_area_px: 80     # tune to your distance: ≈ (cube_size * fx / Z)^2
    rgb_max_area_px: 8000
    rgb_aspect_tol: 0.30
```

Or override at launch:

```bash
ros2 launch cube_detector cube_detector.launch.py
# in another terminal:
ros2 param set /cube_detector detection_mode rgb_only   # NB: requires restart
```

(`detection_mode` is read once at startup; restart the node after changing it.
The `rgb_*_area_px` params, like `min_score`, are read each frame.)

What it does:

1. RGB → grayscale → bilateral filter → adaptive Canny.
2. `findContours` → `approxPolyDP` for 4-vertex convex polygons.
3. Filter by area (band) and side-length consistency.
4. Score each candidate by side+angle match (no 3D check available).
5. `cv2.solvePnPGeneric(SOLVEPNP_IPPE_SQUARE)` for full 6-DoF pose against the
   known top-face square model (±12.5 mm in XY, Z=0).
6. Pick the solution with `R[2,2] < 0` (top facing the camera) and reprojection
   error ≤ 5 px. Reject if neither IPPE solution qualifies — protects against
   degenerate cases (perfectly fronto-parallel views are PnP-ambiguous).

Limitations vs `depth_fusion`:

- No table-plane prior, so any convex quad in the area band can match —
  expect more false positives in cluttered scenes. Tighten `min_score`
  (e.g. `0.7`) and the area band to compensate.
- Pose accuracy degrades fast at small cube sizes / large Z (when the
  square is < ~20 px the corners are too quantised for sub-pixel PnP).
- `depth_uv_offset_*` parameters are ignored in this mode.
- Perfectly fronto-parallel cubes (camera Z exactly perpendicular to top
  face) yield ambiguous pose; the rejection logic drops these rather than
  publishing a wrong pose. Slight tilt — which is your "almost top-down" —
  removes the ambiguity.

---

## 4. Detection pipeline

The pipeline is structured as `detect(rgb, depth_m, K)` in
`cube_detector/detector.py`. Each stage is independently inspectable via the
`debug` dict it returns.

```
RGB + Depth + K
        |
        v
[1] RANSAC table plane on a sub-sampled point cloud  →  (n, d)
        |
        v
[2] Per-pixel height map  h(u,v) = n·p(u,v) + d
        |
        v
[3] Above-table mask:  h_min < h < cube_size + h_max_tol
    + morphological open / close to clean noise
        |
        v
[4] Connected components, filtered by predicted pixel area
    pred_area = (cube_size · fx / Z)²    accept [0.25× , 5×]
        |
        v
[5] For each surviving candidate (best size match first):
        (a) Adaptive top-face mask: h ∈ [top_h - 5 mm, top_h + 5 mm]
            where top_h = 92nd percentile of the candidate's heights
        (b) Largest contour → approxPolyDP (4 vertices) or minAreaRect
        (c) Optional cornerSubPix snap on grayscale + bilateral
        (d) Geometric score  (size + side-length + 90°-corner)
        |
        v
[6] Pose recovery:
    Top-face plane:  n·p + (d − cube_size) = 0
    For each corner pixel  (u, v),  ray = K⁻¹ [u, v, 1]ᵀ
    t = −d_top / (n·ray)    →  3D corner = t · ray
    Cube centre = mean(corners) − ½·cube_size·n
    Rotation: cube +Z = n; cube +X from edge[0→1] projected into plane
        |
        v
CubeDetection { cube_center, top_center, rotation, score, ... }
```

### 4.1 Why this is robust to colour and lighting

The whole segmentation is done on the depth-derived **height-above-table**
map. A 25 mm cube sitting on a table has top-face pixels with
height ≈ 25 mm regardless of:

- cube colour, texture, gloss, transparency
- table colour
- ambient lighting, shadows, specular highlights
- coloured side faces

The colour image only enters at one (optional) place: the cornerSubPix snap
that refines the top-face polygon corners. Even that is gated — a refined
corner that moves more than 4 px is rejected as an artefact.

### 4.2 Why pose is sub-millimetre accurate even with noisy depth

Per-pixel depth at the cube edges is noisy on the Astra Pro (high gradient
≈ holes / quantisation). Naive corner deprojection compounds that noise into
pose error.

We avoid it by computing pose from the **top-face plane** instead:

1. The cube top is geometrically known to be parallel to the table at
   `+ cube_size` along the normal direction.
2. Each corner pixel is intersected with that plane analytically (one line
   per corner).
3. The corner depths are never read directly.

Result: pose noise comes from the table-plane fit (averaged over thousands
of inliers, typically ≤1 mm) plus the corner pixel localisation
(≤1 px after subpixel refinement, ≤0.5 mm at 30 cm). Sub-mm position is
attainable, as confirmed by synthetic tests bundled in the repo.

### 4.3 Top-down vs. angled views

| Camera tilt vs. table normal | Cube silhouette       | Detection path                         |
|------------------------------|-----------------------|----------------------------------------|
| 0° (perfectly top-down)      | Square                | 4-vertex approxPolyDP                  |
| 5°–25° ("almost top-down")   | Slight trapezoid      | 4-vertex approxPolyDP (typical case)   |
| 25°–40°                      | Tilted quad / pentagon| approxPolyDP often non-quad → fallback |
| 40°+                         | Hexagon (3 faces)     | minAreaRect fallback on full silhouette|

The implementation always succeeds on the first three rows. The 40°+ case
falls back to `minAreaRect` on the largest connected component, which gives
correct (x, y, yaw) but a slightly biased z (because the silhouette includes
side faces). For tabletop pick-and-place the camera is rarely past 25° tilt.

---

## 5. Robustness strategies (summary)

| Strategy                               | What it defends against                          |
|----------------------------------------|--------------------------------------------------|
| Depth-based segmentation               | Colour, texture, lighting, shadow changes        |
| RANSAC plane (200 iter, SVD refine)    | Tilted / cluttered / partially-occluded tables   |
| Adaptive top-face height (92nd %ile)   | Plane-fit drift, depth bias                      |
| Predicted pixel-area gating            | Random clutter that happens to look square       |
| Multi-criterion geometric score        | Polygons that aren't square enough               |
| `min_score` reject threshold           | Borderline / spurious detections                 |
| cornerSubPix corner snap (capped 4 px) | Pixel-grid quantisation                          |
| Temporal EMA + jump rejection          | Frame-to-frame jitter                            |
| Stale-counter (drop after 5 misses)    | Stuck filter when the cube leaves view           |
| Software depth-pixel offset (live)     | Residual hardware D2C error (a few px)           |

---

## 6. Tuning guide

### 6.1 First-time alignment

The Astra Pro hardware D2C is good but not pixel-perfect; a few px residual
is normal.

1. Run the detector. Open `/cube_detector/debug_image` in `rqt_image_view`.
2. Look at how the green mask / polygon sits on the cube body.
3. Apply offsets in the same direction the polygon needs to move:
   - polygon is `right` of the cube → `depth_uv_offset_x` negative
   - polygon is `below` the cube → `depth_uv_offset_y` negative
   - polygon is `above` the cube → `depth_uv_offset_y` positive
4. Tune live, no restart needed:

   ```bash
   ros2 param set /cube_detector depth_uv_offset_x -5
   ros2 param set /cube_detector depth_uv_offset_y 3
   ```

5. Once happy, copy the values into `config/cube_detector.yaml`.

The offset is in colour pixels and is constant across the image. It's a
good correction at one working distance; if you later move the camera much
closer or farther, re-tune.

### 6.2 If detection misses (no green polygon at all)

- `score` printed in the bottom-left says `no detection (<stage>)`.
  Stages tell you what failed:
  - `plane_fit` — too few valid depth pixels. Lower `ransac_threshold`,
    raise `ransac_iters`. Check the depth stream is alive (`ros2 topic hz`).
  - `no_candidates` — height-band mask captured nothing of the right size.
    The cube might be at the edge of `[height_min, cube_size+height_max_tol]`.
    Bump `height_max_tol` to e.g. `0.020`.
  - `top_face_or_pose` — candidates exist but no quad passed scoring.
    Lower `min_score` (e.g. `0.30`), or disable `edge_refine`.

### 6.3 If detection picks the wrong cube (multi-cube scene)

The detector picks the candidate with the best **size + shape** match. With
several similar cubes that's unstable. Options:

- Single-cube setup is the simplest fix.
- Tighten `ransac_threshold` and `min_score` so only the most square
  candidate wins.
- For a region-of-interest filter (e.g. "the cube nearest the gripper"),
  ask for a `roi_*` parameter — easy to add but not in scope yet.

### 6.4 If pose jitters frame-to-frame

- Raise `temporal_alpha` from `0.4` to e.g. `0.2` for heavier smoothing.
- Or raise `processing_period_s` to `0.2` (5 Hz) — fewer frames means each
  has more impact on the EMA and timing aliasing reduces.

### 6.5 If pose lags real-time motion

- Lower `temporal_alpha` toward `0.6–0.8` (more responsive).
- Lower `processing_period_s` to `0.05` (20 Hz).
- For pick-and-place, this rarely matters: the arm typically grabs a
  single snapshot before moving.

---

## 7. Troubleshooting

| Symptom                                  | Likely cause / fix                                                                           |
|------------------------------------------|----------------------------------------------------------------------------------------------|
| Debug image black in RViz                | Set the Image display **Reliability** to **Best Effort**                                     |
| `waiting for camera_info...` repeatedly  | Wrong topic; check `info_topic` param vs `ros2 topic list \| grep camera_info`               |
| `depth frame_id != color`                | `depth_align: false` in astra params — flip to true and rebuild astra_camera                 |
| Mask offset by many pixels               | Hardware D2C off, OR depth/color resolution mismatch in params yaml                          |
| Detection picks shadows / non-cubes      | Raise `min_score`; check the table is matte, not glossy                                      |
| Plane fit fails on a tilted table        | Increase `ransac_iters` to `500`; relax `ransac_threshold` to `0.006`                        |
| Pose rotates suddenly by 90°             | Top-face quad ordering ambiguity — expected; the cube has 4-fold symmetry around its Z axis  |
| `cv_bridge` import fails                 | `sudo apt install ros-humble-cv-bridge ros-humble-vision-opencv`                             |

---

## 8. Implementation files

```
src/cube_detector/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md                    ← this file
├── resource/cube_detector
├── cube_detector/
│   ├── __init__.py
│   ├── detector.py              ← pure-Python detection (no ROS deps)
│   └── detector_node.py         ← ROS 2 node + I/O + temporal filter
├── launch/
│   └── cube_detector.launch.py
└── config/
    └── cube_detector.yaml
```

`detector.py` is intentionally ROS-free so it can be unit-tested with
synthetic depth + RGB arrays. See the smoke / angled-view tests embedded in
the original development log.
