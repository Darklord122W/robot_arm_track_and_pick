# charuco_tf_publisher

Detects a ChArUco board in a color image stream and broadcasts the board pose
as a TF (`camera_color_optical_frame → handeye_target` by default). Built to
feed `easy_handeye2` as the *tracking marker* source — easy_handeye2 is
tracker-agnostic, so it ships with no detector of its own.

## Topology

```
/camera/color/image_raw    ─┐
/camera/color/camera_info  ─┴─►  charuco_tf_publisher  ──► /tf  (camera → handeye_target)

/tf (world → link2 from URDF + arm joints)
                                                          ─► easy_handeye2 (samples + solves AX=XB)
```

The end-effector frame (`link2` here) is whatever the calibration board is
rigidly taped to.

## Defaults (calib.io 210x150mm 5x7 / 26mm / 19mm DICT_5X5 board)

| Parameter | Default |
|---|---|
| `squares_x` | 5 |
| `squares_y` | 7 |
| `square_length` | 0.026 m |
| `marker_length` | 0.019 m |
| `dictionary` | DICT_5X5_250 |
| `image_topic` | /camera/color/image_raw |
| `camera_info_topic` | /camera/color/camera_info |
| `parent_frame` | camera_color_optical_frame |
| `child_frame` | handeye_target |
| `min_corners` | 6 |
| `publish_debug_image` | true |
| `debug_topic` | /charuco_tf/debug_image |
| `image_qos_reliable` | true |

If you re-measure the printed square and it isn't 26 mm, override `square_length`
and `marker_length` proportionally.

## Run standalone

```bash
ros2 launch charuco_tf_publisher charuco_tf.launch.py
```

Verify:

```bash
ros2 run rqt_image_view rqt_image_view /charuco_tf/debug_image
ros2 run tf2_ros tf2_echo camera_color_optical_frame handeye_target
```

## Run as part of eye-to-hand calibration

See `launch/handeye_calibrate.launch.py` — combines this detector with
`easy_handeye2`'s `calibrate.launch.py` preset for eye-on-base.

```bash
ros2 launch charuco_tf_publisher handeye_calibrate.launch.py
```

`calibration_type` is hard-wired to `eye_on_base` (easy_handeye2's name for
eye-to-hand). Override via launch args if you need different frames.
