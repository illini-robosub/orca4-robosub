# vision_detect_package

A ROS 2 (Humble) C++ node that runs a **YOLOv8 / YOLO11 object detector with TensorRT** on the
left color image from [`orca_zed_publisher`](../zed_camera_publisher/orca_zed_publisher) and publishes:

- the **left image with boxes drawn on it**,
- **every detection** (class, confidence, box center and size, in pixels),
- the **center of the most confident box**, for nodes that only want one target.

The model is an Ultralytics `.onnx` file. On first start the node turns it into a TensorRT `.engine`
file optimized for this Jetson's GPU (takes a few minutes), and reuses that engine on every start after.

> **The node needs `weights/best.onnx`, not `best.pt`.** The repo currently has only `best.pt`, so
> export it first (see [Getting a model](#getting-a-model)). Without a model the node still runs and
> republishes the camera image, just with no detections.

---

## How it works

```
/zed_publisher/left/image_rect_color  (bgra8)
        │
        ▼  cv_bridge → bgr8
  letterbox to model size (gray 114 padding, like Ultralytics)
        │
        ▼  BGR→RGB, 0–255 → 0–1, split into 3 color planes
  TensorRT inference on the GPU  (output: 1 × (4 + classes) × anchors)
        │
        ▼  best class per anchor, drop below conf_threshold
  undo letterbox → box in original image pixels
        │
        ▼  NMS (per class)
  publish ~/detections, ~/center, ~/image
```

The code is split in two:

| File | What it does |
|---|---|
| [`src/yolo_detector.cpp`](src/yolo_detector.cpp) / [`include/.../yolo_detector.hpp`](include/vision_detect_package/yolo_detector.hpp) | `YoloDetector` class. No ROS: OpenCV image in, boxes out. Builds/loads the engine, preprocesses, runs inference, decodes and runs NMS. |
| [`src/detect_node.cpp`](src/detect_node.cpp) | The ROS node. Reads parameters, loads the model, subscribes to the image, fills the messages and draws the boxes. |

The engine build/load and output decoding follow the Stereolabs sample in
`/usr/local/zed/samples/object detection/custom detector/cpp/tensorrt_yolov5-v6-v8_onnx/`.

---

## Topics

All published topics live under the node name (`/vision_detect/...` by default).

| Topic | Direction | Type | Notes |
|---|---|---|---|
| `/zed_publisher/left/image_rect_color` | sub | `sensor_msgs/Image` | Set by `image_topic`. Best-effort QoS, keep last 1, so old frames get dropped if YOLO is slower than the camera. |
| `~/image` | pub | `sensor_msgs/Image` | `bgr8`, same header as the camera image. Green box, center dot and `label confidence (x, y)` per detection. Only published when something subscribes. |
| `~/detections` | pub | `robosub_msgs/DetectionArray` | Published every frame, even when empty. See the fields below. |
| `~/center` | pub | `geometry_msgs/PointStamped` | `point.x`, `point.y` = box center of the most confident detection, in pixels. `point.z` is always 0. Only published when there is at least one detection. |

### `robosub_msgs/Detection3D` fields

All pixel values are in the coordinates of the **input image** (x = column, y = row, origin at the top left).

| Field | Filled with |
|---|---|
| `header` | Copied from the camera image (same stamp and `left_optical_frame`). |
| `class_id` | Name from `class_names`, or `class_<number>` if no name was given. |
| `confidence` | 0 to 1. |
| `bbox_x`, `bbox_y` | Box center. |
| `bbox_width`, `bbox_height` | Box size. |
| `depth` / `depth_valid` | Not filled in yet: always `-1.0` / `false`. |
| `bearing_x`, `bearing_y` | Not filled in yet: always `0`. |

Depth could come later from `/zed_publisher/depth/depth_registered` or the point cloud, which have the
same size as the left image, so pixel `(bbox_x, bbox_y)` maps straight across.

---

## Parameters

Set in [`config/detect_node.yaml`](config/detect_node.yaml).

| Parameter | Default | Description |
|---|---|---|
| `onnx_path` | `""` | Empty = `<install>/share/vision_detect_package/weights/best.onnx`. |
| `engine_path` | `""` | Empty = same as `onnx_path` but ending in `.engine`. Built automatically if missing. |
| `image_topic` | `/zed_publisher/left/image_rect_color` | Image to run on. |
| `conf_threshold` | `0.5` | 0–1. Detections below this are dropped. |
| `nms_threshold` | `0.7` | 0–1. Two boxes of the same class overlapping more than this (IoU) count as one. 0.7 is the Ultralytics default. |
| `fp16` | `true` | Build a half-precision engine. About 2× faster on Jetson with almost no accuracy loss. Only used when building. |
| `class_names` | `[]` | Class names **in training order** (copy them from the dataset's `data.yaml`). The engine doesn't store names, so without this labels show up as `class_0`, `class_1`, ... The node warns if the count doesn't match the model. |

---

## Getting a model

The node needs an ONNX model exported from Ultralytics with a **fixed input size**:

```bash
pip install ultralytics
yolo export model=best.pt format=onnx simplify=True dynamic=False imgsz=640
```

`ultralytics` isn't installed on the Jetson, so it's easiest to run this on a laptop and copy the
resulting `best.onnx` over. Use the same `imgsz` you trained with.

Then put it in the package and rebuild, so it gets installed:

```bash
cp best.onnx ~/Documents/orca4-robosub/vision_detect_package/weights/best.onnx
```

Only YOLOv8 / YOLO11 **detection** models work (one input `1×3×H×W`, one output `1×(4+classes)×anchors`).
YOLOv5, segmentation and pose models have a different output and the node will refuse to load them.

### Changing the model

The `.engine` file is built for one model and one TensorRT version. After changing the `.onnx` (or
updating JetPack), **delete the old engine** so it gets rebuilt:

```bash
rm ~/Documents/orca4-robosub/install/vision_detect_package/share/vision_detect_package/weights/best.engine
```

`*.engine` is in `.gitignore`, since engines only work on the GPU they were built on.

---

## Build

Needs JetPack (CUDA and TensorRT 10, checked with TensorRT 10.3), OpenCV, `cv_bridge` and
`robosub_msgs`. CUDA and TensorRT come from JetPack, not rosdep.

```bash
cd ~/Documents/orca4-robosub
colcon build --packages-up-to vision_detect_package
source install/setup.bash
```

---

## Run

Camera and detector together:

```bash
ros2 launch vision_detect_package detect_launch.py with_camera:=true
```

Detector only (if `orca_zed_publisher` is already running):

```bash
ros2 launch vision_detect_package detect_launch.py
```

Use your own parameter file:

```bash
ros2 launch vision_detect_package detect_launch.py config:=/path/to/my_detect.yaml
```

> The launch file looks up `orca_zed_publisher` even when `with_camera:=false`, so that package must be
> built and sourced too.

### Checking it works

```bash
ros2 topic echo /vision_detect/detections
ros2 topic echo /vision_detect/center
ros2 topic hz /vision_detect/detections   # detection rate
rqt_image_view /vision_detect/image       # boxes drawn on the image
```

On first start, expect the log to say `Building TensorRT engine ... This takes a few minutes...`,
then `Loaded ...: input 640x640, N classes`.

---

## Troubleshooting

| Log message | Fix |
|---|---|
| `No model at .../best.onnx` | Export the `.onnx`, copy it into `weights/` and rebuild (see [Getting a model](#getting-a-model)). |
| `Could not load ... .engine` | Engine is from an old model or TensorRT version. Delete it so it gets rebuilt. |
| `Model has a dynamic input size` | Re-export with `dynamic=False`. |
| `Output is not YOLOv8/YOLO11 style` | Wrong model type (YOLOv5, segmentation, pose...). Export a YOLOv8/YOLO11 detection model. |
| `class_names has X names but the model has Y classes` | Fix `class_names` in the YAML to match `data.yaml`. |
| No messages on `~/image` at all | Check the camera topic: `ros2 topic hz /zed_publisher/left/image_rect_color`. |
