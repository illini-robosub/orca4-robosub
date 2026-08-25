# ZED X Vision and Object Detection System

## 1. Project Overview

The goal of this project was to develop a **ROS 2 vision pipeline** that allows the submarine to detect objects in the water and determine both:

- **Where the object is in the camera image**
- **How far the object is from the ZED X camera**

This information is intended to become a major input to the submarine's autonomous decision-making system.

The ZED X is one of the submarine's most capable sensors because it provides both color imagery and depth information. The vision system combines these capabilities with a YOLOv8 object detection model to identify objects and associate each detection with a distance measurement.

The current pipeline produces structured ROS messages containing object detections, pixel positions, depth, object class, and confidence.

The next major step is to feed these detections into an autonomous decision-making system, most likely a **behavior tree**, so that the submarine can reason about what it sees and determine how it should respond.

---

# 2. Current State of the Project

The current system is a functional end-to-end object detection pipeline running inside the team's Docker environment.

It currently:

- Connects to the ZED X camera
- Receives color and depth data
- Synchronizes the color and depth streams
- Converts ROS image messages into OpenCV-compatible images
- Runs YOLOv8 object detection
- Filters low-confidence detections
- Determines the pixel location of each detected object
- Estimates the object's distance using ZED depth data
- Packages the detection information into a custom ROS message
- Publishes the detections as a ROS topic
- Supports RViz visualization
- Uses the Jetson GPU for accelerated inference
- Runs YOLOv8 using half-precision inference

For testing, the system currently uses a **pretrained card-detection YOLOv8 model**. This is only a placeholder; it will eventually need to be replaced with a model trained to recognize the objects relevant to the RoboSub competition.

---

# 3. High-Level Architecture

The overall pipeline is:

```text
                         ZED X Camera
                              │
                ┌─────────────┴─────────────┐
                │                           │
          Color Image                  Depth Image
                │                           │
                └─────────────┬─────────────┘
                              │
                     Time Synchronization
                              │
                              ▼
                         ROS 2 Vision Node
                              │
                       Convert via CvBridge
                              │
                              ▼
                         OpenCV Image
                              │
                              ▼
                         YOLOv8 Model
                              │
                     Object Detections
                              │
              ┌───────────────┴───────────────┐
              │                               │
       Object Pixel Position             Object Class
              │                               │
              └───────────────┬───────────────┘
                              │
                    Sample ZED Depth Data
                              │
                              ▼
                     Distance Estimation
                              │
                              ▼
                    Custom ROS Detection
                           Message
                              │
                              ▼
                  /vision/detections
                              │
                              ▼
                    Future Behavior Tree
```

The important distinction is that the current project handles **perception**, but not yet **decision-making**.

The vision system answers:

> "What objects do I see, where are they in the image, and how far away are they?"

The future behavior-tree system will answer:

> "Given what I see, what should the submarine do?"

---

# 4. ZED X Camera Data

The ZED X camera provides multiple data streams.

The vision pipeline uses two primary streams:

1. The **left camera color image**
2. The **depth image**

The color image provides the visual information needed by YOLOv8 to identify objects.

The depth image provides a distance measurement associated with pixels in the image.

This allows the system to combine:

```text
Color:
"What is this?"

Depth:
"How far away is it?"
```

Together, these provide substantially more useful information than object detection from RGB imagery alone.

---

# 5. Color and Depth Synchronization

A critical part of the pipeline is making sure the color image and depth image correspond to approximately the same point in time.

The system synchronizes the two ROS topics using their timestamps, allowing a color/depth pair to be processed when their timestamps are within approximately **0.1 seconds** of one another.

This is important because the submarine and objects in the environment may be moving.

Without synchronization, the system could theoretically perform:

```text
Current color image
        +
Older depth image
```

This could result in the detected object's pixel location being matched with an incorrect depth measurement.

The synchronization therefore ensures that the depth measurement is associated with approximately the same scene that YOLOv8 is analyzing.

---

# 6. ROS Image Conversion

The ZED X publishes its camera data through ROS 2 image messages.

These messages are converted into formats that can be processed by OpenCV using **CvBridge**.

The processing pipeline is therefore approximately:

```text
ZED X
  │
  ▼
ROS Image Message
  │
  ▼
CvBridge
  │
  ▼
OpenCV Image
  │
  ▼
YOLOv8
```

This allows the ROS infrastructure to handle communication while OpenCV and the YOLOv8 framework handle image processing and inference.

---

# 7. YOLOv8 Object Detection

Once the color image has been converted into an OpenCV-compatible image, it is passed into a **YOLOv8** object detection model.

YOLO identifies objects within the image and provides information including:

- Object class
- Bounding box
- Confidence
- Pixel coordinates

For the current testing implementation, a pretrained **card detection model** is being used.

This model was selected as a convenient way to test the end-to-end pipeline. It is not intended to be the final competition model.

Eventually, the model will need to be replaced with a model trained to recognize the objects relevant to the RoboSub competition.

---

# 8. Confidence Filtering

Each YOLO detection includes a confidence value indicating how confident the model is that the detected object belongs to the predicted class.

The current system discards detections with confidence below **30%**.

Conceptually:

```text
YOLO Detection
      │
      ▼
Confidence >= 0.30?
    /       \
  Yes        No
   │          │
   ▼          ▼
Process     Discard
detection   detection
```

This prevents very uncertain detections from being passed downstream into the autonomy system.

The confidence threshold is currently hardcoded and would be better exposed as a ROS parameter in the future so that it can be tuned without modifying the source code.

---

# 9. Determining Object Position

Once an object is detected, the system determines the **center point of its bounding box**.

For example:

```text
        Object Bounding Box
      ┌───────────────────┐
      │                   │
      │         ●         │
      │      Center       │
      │                   │
      └───────────────────┘
```

The center point is represented using image pixel coordinates:

```text
(x, y)
```

where:

- `x` represents the horizontal pixel position
- `y` represents the vertical pixel position

This provides the object's location from the perspective of the camera.

The pixel location can be useful to the autonomy system because it provides directional information even before a full 3D coordinate transformation is performed.

---

# 10. Estimating Object Distance

The next step is determining how far the detected object is from the camera.

The ZED X depth image provides a depth measurement for each pixel.

A simple approach would be to take the depth value at the object's center pixel:

```text
depth[center_y][center_x]
```

However, relying on a single pixel is unreliable because depth sensors can contain invalid or noisy measurements.

For example, the center pixel could return:

```text
0
```

or:

```text
infinity
```

even though the object has a valid depth measurement.

A single bad pixel would therefore cause the entire detection's distance estimate to be incorrect.

---

# 11. 10×10 Depth Sampling Region

Instead of using a single pixel, the system samples a **10×10 region** surrounding the detected center point.

The region extends approximately five pixels in each direction:

```text
             10 pixels
        ┌───────────────┐
        │               │
        │               │
        │       ●       │
        │     center    │
        │               │
        │               │
        └───────────────┘
             10 pixels
```

This produces approximately **100 depth samples**.

Before calculating the final distance, invalid depth values are discarded.

For example:

```text
Valid:
    2.1 m
    2.0 m
    2.2 m

Invalid:
    0
    infinity
```

Only valid measurements are used in the final calculation.

---

# 12. Why the Median Is Used

The system calculates the **median** of the remaining valid depth measurements rather than the average.

This is important because depth measurements can contain significant outliers.

For example, suppose the measured depths were:

```text
2.0
2.1
2.0
2.2
∞
2.1
```

An average could be heavily affected by the invalid/outlier measurement.

The median is much more resistant to outliers.

The general process is:

```text
100 depth pixels
       │
       ▼
Remove 0 / invalid / infinite values
       │
       ▼
Remaining valid depth values
       │
       ▼
Calculate median
       │
       ▼
Object distance
```

This provides a more stable distance measurement than using a single pixel.

---

# 13. Why Multiple Pixels Are Important

Sampling multiple pixels protects the system against isolated bad depth measurements.

For example:

```text
Object:

┌───────────────┐
│ 2.0  2.1  2.0 │
│ 2.1  ∞    2.0 │
│ 2.0  2.1  2.1 │
└───────────────┘
```

The single invalid center measurement does not destroy the entire estimate because the surrounding pixels still provide valid information.

This makes the resulting depth estimate significantly more stable.

---

# 14. Limitations of Center-Based Depth Sampling

Although the 10×10 median approach is more robust than using a single pixel, it introduces several potential failure cases.

## 14.1 Background Contamination

The 10×10 region may extend beyond the detected object.

For example:

```text
┌──────────────────────┐
│       Background     │
│    ┌────────────┐    │
│    │   Object   │    │
│    │     ●      │    │
│    └────────────┘    │
│       Background     │
└──────────────────────┘
```

Some of the sampled depth pixels may therefore belong to the background rather than the detected object.

If the background is significantly closer or farther away, it can affect the median.

---

## 14.2 Object Holes

Another problem occurs if an object has a hole or empty region near its center.

For example, the detector could correctly identify the center of a ring-shaped object:

```text
       Object
    ┌─────────┐
    │   ┌─┐   │
    │   │●│   │
    │   └─┘   │
    └─────────┘
```

The detected center could fall inside the hole rather than on the physical object.

The depth measurement could then correspond to the background behind the object rather than the object itself.

---

# 15. Potential Improvement: Segmentation-Based Depth

One potential solution is to use **instance segmentation** rather than relying only on the bounding box center.

Instead of sampling a fixed 10×10 region, the system could generate a mask identifying the actual pixels belonging to the detected object.

The pipeline would become:

```text
Object Detection
       │
       ▼
Object Segmentation
       │
       ▼
Object Pixel Mask
       │
       ▼
Extract depth only from object pixels
       │
       ▼
Remove invalid values
       │
       ▼
Calculate median
       │
       ▼
Object distance
```

This would reduce the likelihood of including background pixels in the depth calculation.

It could also address the issue of objects with holes because the system would know which pixels actually belong to the object.

However, segmentation would add:

- Computational cost
- Model complexity
- Additional implementation complexity

Whether this is worthwhile depends on the requirements of the final RoboSub autonomy system.

---

# 16. Detection Message

Once the system has processed a detection, the relevant information is packaged into a **custom ROS message**.

The message contains information such as:

- Object class
- Detection confidence
- Pixel X position
- Pixel Y position
- Distance from the camera

This provides a standardized interface between the perception system and the rest of the autonomy stack.

The vision system therefore does not need to know what the submarine will eventually do with the detection.

It simply publishes:

```text
"I detected object X
 at image position (x, y)
 approximately D meters away
 with confidence C."
```

The downstream autonomy system can then decide what to do with that information.

---

# 17. ROS Topic Interface

The detection information is published through:

```text
/vision/detections
```

The topic can be inspected directly from the terminal using:

```text
ros2 topic echo /vision/detections
```

This provides a useful debugging interface because it allows developers to verify that the vision node is producing reasonable detection data without needing RViz.

---

# 18. RViz Visualization

The vision package also includes support for publishing an RViz-compatible image with bounding boxes drawn around detected objects.

This provides a visual debugging tool:

```text
ZED X Image
     │
     ▼
YOLO Detection
     │
     ▼
Bounding Boxes
     │
     ▼
RViz
```

The annotated image publishing is currently commented out/disabled because it adds processing overhead.

The underlying detection pipeline therefore does not need to continuously generate visualization images during normal operation.

This is useful because visualization is primarily a development/debugging feature rather than a requirement for autonomous operation.

---

# 19. ROS 2 Launch System

A launch file was created to provide a consistent environment for running the vision system.

Rather than requiring developers to manually start each component, the launch file can automatically start the vision node and configure the RViz environment.

The ZED X camera is started inside the Docker container using:

```text
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
```

The vision launch file can then be started with:

```text
ros2 launch vision_pkg vision_launch.py
```

This creates a consistent workflow for new team members and reduces the amount of manual ROS configuration required.

The launch file is particularly useful for development because everyone can use the same setup rather than maintaining individual RViz configurations.

---

# 20. Docker and GPU Acceleration

A major portion of the implementation involved getting the vision pipeline working efficiently inside the team's Docker environment.

The initial version of the pipeline was extremely slow because YOLOv8 inference was running on the **CPU**.

For the Jetson platform, GPU acceleration is essential for achieving a useful inference rate.

The Docker environment therefore needed:

- GPU access
- Compatible CUDA dependencies
- Compatible PyTorch dependencies
- Compatible torchvision dependencies

The Dockerfile was modified so that the appropriate NVIDIA/Jetson PyTorch packages were installed before the remaining Python requirements.

The relevant installation step was:

```text
RUN pip3 install torch torchvision \
    --index-url https://developer.download.nvidia.com/compute/redist/jp/v60
```

This allows PyTorch to access the Jetson GPU from within the Docker container.

---

# 21. Inference Performance

After enabling GPU acceleration, the YOLOv8 pipeline achieved approximately:

**85–100 ms per frame**

during testing.

This corresponds to roughly 10–12 inference cycles per second under those conditions, although the actual detection throughput should not be interpreted simply as the ZED camera's FPS.

Several implementation details contribute to the observed performance.

---

# 22. Half-Precision Inference

YOLOv8 inference is run using **half precision**:

```text
half=True
```

This allows the model to use lower-precision floating-point representations during inference.

On the Jetson GPU, this can significantly reduce the computational cost of inference while maintaining acceptable model performance.

Half-precision inference is therefore one of the factors contributing to the approximately 85–100 ms inference time.

---

# 23. Frame Dropping and Inference Lock

The node intentionally does **not queue every incoming frame**.

Instead, it uses a simple lock/flag:

```text
self.inferring
```

When inference begins:

```text
self.inferring = True
```

While the model is processing a frame, newly arriving synchronized color/depth pairs are dropped.

Once inference finishes:

```text
self.inferring = False
```

and the next available frame can be processed.

The resulting behavior is:

```text
Camera frames:

Frame 1 ──► Inference
Frame 2 ──► Dropped
Frame 3 ──► Dropped
Frame 4 ──► Dropped
Frame 5 ──► Inference
Frame 6 ──► Dropped
...
```

This design prevents a backlog of old frames from forming.

That is important for autonomy because processing a large queue of old images would mean the system could be making decisions based on stale information.

The tradeoff is that some camera frames are discarded.

Therefore, the effective detection rate is primarily determined by the **inference processing time**, rather than simply by the ZED camera's frame rate.

---

# 24. Known Depth Sampling Bug

There is currently a correctness issue in the depth sampling implementation that should be fixed.

The system currently extracts the 10×10 depth region approximately as:

```text
depth[center_y-5:center_y+5, center_x-5:center_x+5]
```

The problem is that the indices are not currently clamped to the image boundaries.

For detections near the top or left edge of the image, values such as:

```text
center_y - 5
```

or:

```text
center_x - 5
```

can become negative.

NumPy does not necessarily treat this as an error. Negative indices can wrap around to the opposite side of the array.

Therefore, an object near the edge of the image could potentially have its depth calculated using pixels from the opposite edge of the image.

This can silently produce an incorrect distance measurement.

The bounds should instead be clamped:

```text
y_min = max(0, center_y - 5)
y_max = min(height, center_y + 5)

x_min = max(0, center_x - 5)
x_max = min(width, center_x + 5)
```

This is a correctness issue rather than merely an optimization and should be addressed before relying heavily on edge-of-frame detections.

---

# 25. Known ZED Depth Limitation

From testing, depth measurements need to be approximately **greater than 0.3 meters** to register properly.

This is an important practical limitation when interpreting detection distances.

A detection very close to the camera may still be recognized by YOLO while producing an invalid or unreliable depth value.

Any downstream autonomy system should therefore account for the possibility that:

```text
Object detected ≠ Valid distance available
```

---

# 26. Coordinate Frame Considerations

The current system reports the object's position in terms of its **raw camera image coordinates**.

This is useful for describing where an object appears in the camera image, but a behavior tree will likely need a more meaningful spatial representation.

For example:

```text
Image coordinates:
    (x = 420, y = 280)

Potential autonomy representation:
    Object is 15° right of vehicle center
    Object is 3.2 m away
```

Or, more generally, the detection could eventually be transformed into a vehicle-relative coordinate frame.

This transformation will likely be necessary before the detection information can be used effectively for tasks such as:

- Heading toward an object
- Avoiding an obstacle
- Aligning with a target
- Determining whether an object is on the left/right of the submarine
- Planning a trajectory around an object

---

# 27. Object Tracking

The current vision system treats detections on a frame-by-frame basis.

There is currently no persistent object ID or tracking system.

For example, if the same object appears in ten consecutive frames, the system effectively sees:

```text
Frame 1: Object A
Frame 2: Object A
Frame 3: Object A
...
```

rather than explicitly understanding that all ten detections represent the same physical object.

Object tracking could become useful for the behavior tree.

For example, the autonomy system may eventually need to reason:

> "I have been approaching the same gate for the last several seconds."

rather than treating every detection as a completely new observation.

Potential future approaches include adding an object tracker or maintaining detection identities between frames.

---

# 28. Parameterization

Several important parameters are currently hardcoded.

Examples include:

- Detection confidence threshold
- Depth sampling window size
- Depth validity criteria
- Synchronization tolerance
- Other filtering/performance parameters

These would ideally become **ROS parameters**.

For example:

```text
confidence_threshold = 0.30
depth_window_size = 10
sync_tolerance = 0.10
```

This would allow team members to tune the system without modifying and rebuilding the source code.

It would also make experimentation easier during competition testing.

---

# 29. Underwater Testing Considerations

Much of the initial development and testing was focused on validating the software pipeline.

The final system still needs extensive validation under actual underwater conditions.

Important factors include:

### Turbidity

Suspended particles can make objects harder for YOLO to recognize and can degrade the quality of visual features used by the camera.

### Lighting

Underwater lighting can differ significantly from training data.

Changes in brightness, shadows, reflections, and artificial lighting can affect detection confidence.

### Depth Quality

The ZED X depth estimate may also behave differently underwater than in controlled environments.

The system therefore needs to be tested under representative competition conditions rather than assuming that performance observed during bench testing will translate directly to the water.

---

# 30. Future Integration with the Behavior Tree

The most important remaining software integration is connecting the vision system to the submarine's decision-making architecture.

Currently:

```text
ZED X
  │
  ▼
Vision Node
  │
  ▼
/vision/detections
```

The desired architecture is:

```text
ZED X
  │
  ▼
Vision Node
  │
  ▼
/vision/detections
  │
  ▼
Behavior Tree
  │
  ├──► Navigate toward object
  ├──► Avoid obstacle
  ├──► Search for target
  ├──► Align with target
  └──► Perform task
```

The vision node should remain primarily responsible for **perception**.

The behavior tree should be responsible for interpreting those detections and deciding what action to take.

This separation makes the system easier to develop and debug because perception and decision-making can be tested independently.

---

# 31. Current Limitations and Recommended Future Work

The most important improvements, roughly in order of priority, are:

### 1. Fix depth sampling bounds

Prevent negative NumPy indices from wrapping around the image.

**Priority: High**

This is a correctness issue that can produce completely incorrect depth values for objects near the image boundary.

---

### 2. Replace the placeholder YOLO model

Train or obtain a model capable of detecting the actual objects relevant to RoboSub.

**Priority: High**

The current card detector is only intended for pipeline development and testing.

---

### 3. Integrate detections with the behavior tree

Create the interface between `/vision/detections` and the autonomy decision-making system.

**Priority: High**

This is the primary missing step between the current perception system and useful autonomous behavior.

---

### 4. Validate underwater performance

Test detection and depth estimation in realistic underwater conditions.

**Priority: High**

This will determine whether the current approach is actually robust enough for competition.

---

### 5. Transform detections into useful coordinate frames

Convert raw image coordinates and camera-relative information into a vehicle-relative representation appropriate for navigation and decision-making.

**Priority: Medium–High**

---

### 6. Add object tracking

Maintain identities for objects across frames.

**Priority: Medium**

This could become important for higher-level behaviors that need to reason about objects over time.

---

### 7. Consider segmentation-based depth

Investigate using segmentation masks to isolate depth measurements belonging specifically to the detected object.

**Priority: Medium**

This could improve distance reliability but comes at the cost of computational complexity.

---

### 8. Expose parameters through ROS

Convert hardcoded thresholds and window sizes into ROS parameters.

**Priority: Medium**

This will make testing and tuning significantly easier.

---

### 9. Improve inference speed

The current 85–100 ms inference time is a reasonable starting point, but further optimization could improve responsiveness.

Potential areas include:

- Model architecture
- Model size
- TensorRT optimization
- Further GPU optimization
- Input resolution
- Pre/post-processing overhead

This should be balanced against the accuracy requirements of the final system.

---

# 32. Key Lessons for Future Team Members

### 1. Perception is only one part of autonomy

The vision system does not decide what the submarine should do. It provides information that another system can use to make decisions.

### 2. RGB and depth complement each other

YOLO provides semantic information:

> "This is a gate."

The ZED X depth data provides geometric information:

> "The gate is approximately 4 meters away."

Combining the two produces a much more useful perception system.

### 3. Synchronization matters

Color and depth measurements need to correspond to approximately the same scene. Otherwise, a correct object detection can be paired with an incorrect depth measurement.

### 4. Depth measurements should not be trusted blindly

Depth cameras can produce invalid measurements, outliers, and background contamination. Sampling multiple pixels and using the median provides significantly better robustness than relying on a single depth pixel.

### 5. More filtering is not always better

The goal is to produce a useful measurement, not necessarily a perfectly smooth measurement. Additional processing can improve reliability but can also increase latency and reduce responsiveness.

### 6. Real-time systems should avoid processing stale data

The inference lock intentionally drops frames rather than allowing them to accumulate in a queue. For autonomy, a current but slightly lower-rate detection can be more useful than a high-rate stream of detections that are several hundred milliseconds old.

### 7. The Docker environment is part of the system

The vision pipeline depends not only on the ROS and Python code but also on the GPU, CUDA, PyTorch, torchvision, and Jetson environment being configured correctly.

A future member modifying the vision system therefore needs to consider both the application code and the container environment.

---

# 33. End-to-End Summary

The current vision system can be summarized as:

```text
ZED X Camera
     │
     ├──────────────► Color Image
     │
     └──────────────► Depth Image
                          │
            Synchronize Color + Depth
                          │
                          ▼
                     Vision Node
                          │
                       CvBridge
                          │
                          ▼
                       YOLOv8
                          │
                   Detect Objects
                          │
                          ▼
              Filter Confidence < 30%
                          │
                          ▼
                Find Object Center
                          │
                          ▼
               Sample 10×10 Depth Area
                          │
                Remove Invalid Values
                          │
                          ▼
                Calculate Depth Median
                          │
                          ▼
                 Build ROS Message
                          │
                          ▼
                 /vision/detections
                          │
                          ▼
                   Future Behavior
                        Tree
```

The current system therefore provides the **perception foundation** for RoboSub autonomy.

It can take raw ZED X camera data and turn it into structured information about objects in the environment:

> **Object → Class → Confidence → Image Position → Distance**

The next stage is to turn that perception information into action.

The long-term architecture should therefore separate the system into three major layers:

```text
┌───────────────────────────────────────┐
│              Perception               │
│                                       │
│ ZED X → YOLO → Object + Depth         │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│           Decision Making             │
│                                       │
│ Behavior Tree → Determine Response    │
└───────────────────┬───────────────────┘
                    │
                    ▼
┌───────────────────────────────────────┐
│             Control                   │
│                                       │
│ Execute movement/navigation command   │
└───────────────────────────────────────┘
```

The work completed so far establishes the first layer of this architecture. The most important remaining challenge is connecting perception to decision-making and validating that the complete pipeline remains reliable under real underwater conditions.
