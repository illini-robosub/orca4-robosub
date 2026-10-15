# orca_zed_publisher

A ROS 2 (Humble) C++ node that opens a **ZED camera directly through the ZED SDK** and publishes:

- an **XYZ point cloud** where every point carries its **color** (the color buffer),
- the **left color image** at the same resolution, so pixel `(u, v)` in the image is point `(u, v)` in the cloud,
- **depth**, **confidence** and **camera calibration** (`CameraInfo`),
- **IMU** data, plus the **magnetometer**, **barometer** and **temperatures** when the camera has them,
- optional **visual odometry** from ZED positional tracking.

It is a small, single-file alternative to the full
[zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper). It's easier to read and modify, and
it only does the work for topics that have subscribers.

> **Only one process can open the camera at a time.** Don't run this node and `zed_wrapper` together.

---

## Topics

All topics live under the node name (`/zed_publisher/...` by default).

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `~/point_cloud` | `sensor_msgs/PointCloud2` | `camera_frame` | Organized (`height x width`). Fields `x, y, z` (float32, meters) and `rgb` (float32 holding packed BGRA bytes, the layout RViz/PCL expect). Points with no depth are `NaN`. |
| `~/left/image_rect_color` | `sensor_msgs/Image` | `left_optical_frame` | `bgra8`, rectified. Same size as the point cloud. |
| `~/left/camera_info` | `sensor_msgs/CameraInfo` | `left_optical_frame` | Intrinsics scaled to the output resolution; no distortion (rectified). |
| `~/depth/depth_registered` | `sensor_msgs/Image` | `left_optical_frame` | `32FC1`, meters, aligned with the left image. `NaN`/`inf` = no depth. |
| `~/confidence/confidence_map` | `sensor_msgs/Image` | `left_optical_frame` | `32FC1`, 1 (most confident) to 100 (least). |
| `~/right/image_rect_color` | `sensor_msgs/Image` | `right_optical_frame` | Only if `publish_right: true`. |
| `~/right/camera_info` | `sensor_msgs/CameraInfo` | `right_optical_frame` | Only if `publish_right: true`. `P[3] = -fx * baseline`. |
| `~/imu/data` | `sensor_msgs/Imu` | `camera_frame` | Orientation (ZED onboard filter), angular velocity (rad/s), acceleration (m/s²), with covariances. |
| `~/imu/mag` | `sensor_msgs/MagneticField` | `camera_frame` | Tesla. Only on cameras with a magnetometer (ZED 2/2i, **not** ZED X). |
| `~/atm_press` | `sensor_msgs/FluidPressure` | `camera_frame` | Pascals. Only on cameras with a barometer (ZED 2/2i). Measures air pressure inside the camera, **not** water depth. |
| `~/temperature/imu`, `~/temperature/left`, `~/temperature/right` | `sensor_msgs/Temperature` | `camera_frame` | °C; each one only if that sensor exists. |
| `~/odom` | `nav_msgs/Odometry` | `odom_frame` → `camera_frame` | Only if `enable_tracking: true`. |

### Using the point cloud and color buffer together

The cloud is **organized**, so it has the same `height x width` as the image:

```
point (u, v)  ->  byte offset  v * row_step + u * 16
   x = float at +0,  y = float at +4,  z = float at +8,  color = 4 bytes (B, G, R, A) at +12
```

The color at `+12` is the same pixel you'd read at `(u, v)` in `~/left/image_rect_color`.

---

## Coordinate frames

| Frame | Axes | Used by |
|---|---|---|
| `camera_frame` (`zed_left_camera_frame`) | x forward, y left, z up (ROS REP-103 body frame) | point cloud, IMU, odometry |
| `left_optical_frame` / `right_optical_frame` | z forward, x right, y down (camera optical frame) | images, depth, camera_info |

The node publishes **static TFs** from `camera_frame` to both optical frames, with the right one offset by the
stereo baseline. To place the camera on the sub, publish a transform from your robot frame (e.g.
`base_link`) to `zed_left_camera_frame`.

---

## Parameters

Set in [`config/zed_publisher.yaml`](config/zed_publisher.yaml).

| Parameter | Default | Description |
|---|---|---|
| `serial_number` | `0` | Camera to open; `0` = first one found. |
| `svo_file` | `""` | Play a `.svo` recording instead of the live camera (loops at the end). |
| `resolution` | `AUTO` | `AUTO`, `HD1200`, `HD1080`, `SVGA` (ZED X); `HD2K`, `HD720`, `VGA` (USB ZEDs). |
| `fps` | `30` | Camera frame rate. |
| `depth_mode` | `NEURAL_LIGHT` | `PERFORMANCE`, `QUALITY`, `ULTRA`, `NEURAL_LIGHT`, `NEURAL`, `NEURAL_PLUS`. Neural modes are more accurate but use more GPU. |
| `depth_min` / `depth_max` | `0.3` / `20.0` | Depth range in meters. |
| `confidence_threshold` | `95` | 1–100. Lower values drop more uncertain depth pixels. |
| `texture_confidence_threshold` | `100` | 1–100. Lower values drop depth in low-texture areas. |
| `downscale` | `2` | Output resolution = camera resolution / `downscale`. Applies to the cloud, images, depth and confidence so they stay aligned. |
| `publish_right` | `false` | Also publish the right image and camera info. |
| `enable_sensors` | `true` | Publish IMU, magnetometer, barometer and temperatures. |
| `sensor_rate` | `200.0` | How often (Hz) to poll the sensors. |
| `enable_tracking` | `false` | Enable ZED visual odometry on `~/odom`. |
| `publish_odom_tf` | `false` | Also broadcast `odom_frame` → `camera_frame` on TF. Leave off if another node (e.g. an EKF) owns that transform. |
| `camera_frame`, `left_optical_frame`, `right_optical_frame`, `odom_frame` | see YAML | TF frame names. |
| `sensor_qos` | `false` | `true` = best-effort QoS (lower latency, may drop). Subscribers must use a compatible QoS. |

---

## Build

Requires the **ZED SDK 5.x** (installed in `/usr/local/zed`) and CUDA. Both are already in this repo's Docker image.

```bash
cd ~/colcon_ws
colcon build --packages-select orca_zed_publisher
source install/setup.bash
```

## Run

```bash
# Live camera
ros2 launch orca_zed_publisher zed_publisher_launch.py

# Play back a recording (no camera needed)
ros2 launch orca_zed_publisher zed_publisher_launch.py svo_file:=/path/to/recording.svo

# Use your own parameter file
ros2 launch orca_zed_publisher zed_publisher_launch.py config:=/path/to/my_params.yaml
```

Quick checks:

```bash
ros2 topic list | grep zed_publisher
ros2 topic hz /zed_publisher/point_cloud
ros2 topic echo /zed_publisher/imu/data --once
```

In **RViz**, set the fixed frame to `zed_left_camera_frame`, add a **PointCloud2** display on
`/zed_publisher/point_cloud` and set *Color Transformer* to **RGB8**.

---

## Notes

- **Bandwidth:** at HD1200 with `downscale: 1`, each point cloud is about 37 MB (1920 × 1200 × 16 bytes).
  The default `downscale: 2` cuts that to about 9 MB. Raise it if the cloud rate drops.
- **Nothing is computed for topics with no subscribers.** The cost scales with what you actually listen to.
  `camera_info` is always published because it's tiny.
- **Package name:** the folder is `zed_camera_publisher/` but the package is `orca_zed_publisher`.
  `docker/Dockerfile` builds with `--packages-skip-regex '^zed_.*'`, which would skip a package named `zed_...`.
- **Timestamps** come from the camera's capture time (`TIME_REFERENCE::IMAGE`), not from when the message was sent.

## Troubleshooting

**Node crashes on startup with `vector::_M_range_check` / "sl::Camera::Open has not been called"**
The ZED X driver daemon is in a bad state (common after a reboot or re-plugging the camera). Restart it:

```bash
sudo systemctl restart zed_x_daemon
```

Then check that the SDK can see the camera before launching again:

```bash
/usr/local/zed/tools/ZED_Explorer --all    # should list the ZED-X as "Camera Available"
```

If it still fails, reboot the Jetson.

**`ros2 launch` says `Package 'orca_zed_publisher' not found` after building and sourcing**
colcon is missing its ROS plugin (`colcon-ros`). It built the package as plain CMake, so sourcing doesn't add it
to `AMENT_PREFIX_PATH`. Check with `colcon list`: the package should show `(ros.ament_cmake)`, not `(cmake)`.
Fix it, then rebuild:

```bash
sudo apt install python3-colcon-common-extensions
colcon build --packages-select orca_zed_publisher --cmake-clean-cache
```

The Docker image already installs this, so it only affects builds on the host.

## Sources

The implementation was checked against:

- ZED SDK 5.0 API header: `/usr/local/zed/include/sl/Camera.hpp`
- Official ZED SDK C++ sample build files: `/usr/local/zed/samples/tutorials/`
- Official Stereolabs `zed-ros2-wrapper` (`zed_camera_component.cpp`) for the point cloud layout, the
  right-camera `Tx`, and IMU unit and covariance conversions
- ROS 2 message definitions in `/opt/ros/humble/share/sensor_msgs/msg/` (`CameraInfo`, `Imu`,
  `MagneticField`, `FluidPressure`)
- ROS REP-103 for body vs. optical frame conventions
