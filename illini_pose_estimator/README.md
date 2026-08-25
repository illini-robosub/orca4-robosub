# IMU-Based Dead Reckoning and Localization System

## 1. Project Overview

The goal of this project was to develop a **dead reckoning system** that could provide the submarine with a rough estimate of its position while underwater.

At the time development began, the primary available navigation sensor was the **Cube Orange flight controller**, which provided measurements from an onboard IMU, including:

- Linear acceleration
- Angular velocity
- Orientation

The basic idea was to use the IMU to determine how the submarine was moving and integrate that motion over time to estimate its position. A barometer was incorporated separately to provide a more stable estimate of vertical position/depth.

The resulting position estimate was published through **ROS 2** as a coordinate transform and visualized in **RViz**, allowing the system to be tested against the actual movement of the submarine.

The final system was able to produce and visualize a real-time position estimate, but accumulated too much drift to be reliable for navigation. After extensive filtering and calibration work, the team ultimately transitioned to the **ZED X stereo camera's visual-inertial odometry (VIO)** as the primary localization source.

---

# 2. System Architecture

The overall localization pipeline was approximately:

```text
                  Cube Orange
                      │
        ┌─────────────┴─────────────┐
        │                           │
   IMU Acceleration           IMU Orientation
        │                           │
        └─────────────┬─────────────┘
                      │
              Gravity Compensation
                      │
               Bias Calibration
                      │
                  Filtering
                      │
                 Acceleration
                      │
                 Integration
                      ▼
                  Velocity
                      │
                 Integration
                      ▼
             X/Y/Z Position
                      │
                      │
Barometer ──► Depth Conversion
                      │
                      ▼
              Position Estimate
                      │
             + IMU Orientation
                      │
                      ▼
             ROS 2 Transform
                      │
                      ▼
                     RViz
```

The system operated inside a ROS 2 node. The node subscribed to the required sensor topics, processed the measurements, generated the position estimate, and published the resulting transform.

---

# 3. IMU Data Processing

## 3.1 Raw Measurements

The Cube Orange provided three primary pieces of information used by the system:

- Accelerometer measurements
- Gyroscope/angular velocity measurements
- Orientation estimated by the flight controller's IMU

The accelerometer provided acceleration measurements relative to the sensor's coordinate frame. However, these measurements included the effect of gravity and therefore could not be directly integrated into a position estimate.

---

## 3.2 Gravity Compensation

The first major processing step was removing gravity from the acceleration measurement.

The accelerometer measurement was initially represented in the IMU's coordinate frame. The orientation measurement from the IMU was therefore used to rotate the acceleration vector into the **world frame**.

Once the acceleration was represented in the world frame, the known gravity vector could be subtracted.

Conceptually:

```text
Raw acceleration
       │
       ▼
Rotate using IMU orientation
       │
       ▼
Acceleration in world frame
       │
       ▼
Subtract gravity
       │
       ▼
Linear acceleration
```

The resulting vector represented the estimated translational acceleration of the submarine.

This was a critical step because even a small error in gravity compensation could produce a significant velocity and position error after integration.

---

# 4. From Acceleration to Position

Once gravity had been removed, the acceleration was integrated over time.

For each measurement:

```text
velocity += acceleration × Δt
```

where `Δt` was the time elapsed since the previous measurement.

The resulting velocity was then integrated again:

```text
position += velocity × Δt
```

This produced an X/Y/Z position estimate starting at:

```text
(0, 0, 0)
```

From there, the position continuously changed according to the measured acceleration.

In principle, the pipeline was:

```text
Acceleration
     │
     │ integrate
     ▼
Velocity
     │
     │ integrate
     ▼
Position
```

This is the fundamental principle behind inertial dead reckoning.

---

# 5. Barometer-Based Depth

Although the IMU could theoretically estimate movement along all three axes, the vertical position estimate was particularly susceptible to drift.

The Cube Orange also provided barometric pressure measurements. These were converted into an estimated underwater depth using a pressure-to-depth relationship based on the expected pressure increase with depth.

The resulting depth measurement was significantly more stable than estimating vertical position by double-integrating acceleration.

The system therefore used the barometer-derived depth to replace the IMU-based vertical position estimate.

Conceptually:

```text
IMU ──────────────► X/Y position
                      │
                      │
Barometer ─► Depth ───┴──► Full position estimate
```

This allowed the IMU to primarily contribute horizontal translational movement while the barometer provided a more reliable vertical measurement.

---

# 6. Sensor Synchronization

The system subscribed to both the raw IMU and barometer topics.

A major consideration was ensuring that the measurements being used together represented approximately the same point in time.

Without synchronization, the system could potentially combine:

```text
Old IMU measurement
        +
New barometer measurement
```

or the reverse.

This could introduce inconsistencies into the resulting position estimate.

To avoid this, the ROS subscriptions were synchronized so that IMU and barometer measurements were processed together rather than independently. This ensured that the generated transform was based on temporally consistent sensor data.

---

# 7. ROS 2 Localization Output

Once the position estimate had been calculated, it was combined with the known orientation from the IMU.

The resulting position and orientation represented the estimated pose of the submarine.

The ROS 2 node published this information as a **transform**, allowing the estimated movement of the submarine to be visualized in RViz.

The system also published the pose directly. However, the pose required an additional pose-republisher node to convert the message into a format that RViz could use for visualization.

The resulting workflow was:

```text
Sensor Data
    │
    ▼
Localization ROS 2 Node
    │
    ├──► Transform
    │
    └──► Pose
           │
           ▼
    Pose Republisher
           │
           ▼
          RViz
```

This provided a convenient way to observe the localization estimate in real time during testing.

---

# 8. Initial Problem: Severe Drift

The initial implementation successfully produced a moving position estimate, but the output was immediately unusable due to **significant drift**.

Even when the submarine was stationary, the estimated position would continue to move.

This behavior is expected to some degree from IMU-based dead reckoning. Small errors in acceleration become larger velocity errors when integrated, and those velocity errors become even larger position errors after a second integration.

The error therefore compounds:

```text
Small acceleration error
          │
          ▼
Velocity error
          │
          ▼
Position error
          │
          ▼
Large accumulated drift
```

The project therefore shifted from simply implementing the dead reckoning equations to trying to determine how much the estimate could be stabilized through calibration and filtering.

---

# 9. Accelerometer Bias Calibration

The first major source of error addressed was **accelerometer bias**.

Even when the submarine was stationary, the accelerometer would not necessarily report exactly zero acceleration after gravity compensation.

For example:

```text
Expected:
    [0, 0, 0]

Measured:
    [0.03, -0.01, 0.02]
```

Those small errors become significant after integration.

To address this, the localization node included an initialization/calibration period.

For the first **five seconds after startup**:

- Position updates were disabled.
- Acceleration measurements were collected.
- The submarine was assumed to be stationary.
- The average residual acceleration was treated as sensor bias.

The resulting bias vector was then subtracted from future acceleration measurements.

Conceptually:

```text
Startup
   │
   ▼
5-second calibration period
   │
   ├── Measure residual acceleration
   │
   └── Estimate acceleration bias
   │
   ▼
Normal operation
   │
   ▼
Raw acceleration - estimated bias
```

This significantly reduced the initial drift caused by a constant accelerometer offset.

---

# 10. Acceleration Filtering

Removing the bias was not sufficient because the accelerometer still contained substantial measurement noise.

Several filtering techniques were therefore implemented.

## 10.1 Acceleration Deadzone

Very small acceleration measurements were treated as zero.

For example, measurements below a small threshold were assumed to be noise rather than actual submarine movement.

```text
if |acceleration| < threshold:
    acceleration = 0
```

This prevented small amounts of sensor noise from continuously contributing to the integrated velocity.

---

## 10.2 Moving Average Filter

A moving average was also applied to the acceleration measurements.

Rather than using a single instantaneous measurement, the system considered a window of recent measurements and calculated their average.

This reduced sudden spikes in the acceleration signal.

The motivation was that an isolated acceleration spike could otherwise have a disproportionate effect after integration.

Conceptually:

```text
Raw acceleration:

     /\        /\
____/  \______/  \____

Moving average:

____/¯¯\____/¯¯\____
```

The filter traded some responsiveness for a smoother signal.

---

# 11. Velocity Filtering

Additional filtering was applied after integrating acceleration into velocity.

## 11.1 Velocity Deadzone

A small deadzone was applied to velocity measurements.

Very small velocities were treated as zero.

This helped prevent the system from accumulating small velocity errors indefinitely while the submarine was stationary.

---

## 11.2 Velocity Decay

A velocity decay factor was also introduced.

If the system estimated a small amount of residual velocity while the submarine was stationary, that velocity would gradually decay toward zero.

Conceptually:

```text
velocity = velocity × decay_factor
```

This was motivated by testing in RViz where the submarine could be physically stationary while the estimated transform continued moving slowly.

Velocity decay helped suppress this behavior.

However, it also introduced another tradeoff: legitimate low-speed movement could eventually be suppressed as well.

---

# 12. Angular Velocity Compensation

One of the most significant issues discovered during testing was the relationship between **rotation and apparent translational movement**.

When the submarine rotated, the position estimate could suddenly move by a large amount even though there was little or no actual translational movement.

This indicated that errors in the acceleration/orientation processing were becoming especially problematic during periods of high angular velocity.

To reduce this effect, the system used the gyroscope's angular velocity measurement to reduce the influence of acceleration measurements when the submarine was rotating rapidly.

Conceptually:

```text
Low angular velocity
        │
        ▼
Normal acceleration contribution


High angular velocity
        │
        ▼
Reduced acceleration contribution
```

This substantially reduced some of the large position jumps observed during rotation.

However, this technique introduced another important tradeoff: real translational movement occurring while the submarine was rotating could also be suppressed.

---

# 13. Filter Tuning and Tradeoffs

A large portion of development involved repeatedly testing and tuning the filtering parameters.

The central problem was balancing:

**Stability**

against

**Responsiveness**

Aggressive filtering made the position estimate much more stable, but also caused legitimate movement to disappear.

For example:

```text
More filtering
     │
     ├── Less noise
     ├── Less drift
     └── Less responsiveness


Less filtering
     │
     ├── More responsive
     ├── More real movement captured
     └── More noise and drift
```

This was particularly difficult because dead reckoning errors accumulate over time. A filter that appears reasonable at the acceleration level can have a much larger effect after the signal has been integrated once into velocity and again into position.

After multiple iterations, the system became substantially more stable than the initial implementation, but the resulting estimate still did not provide sufficient reliability for use as the submarine's primary localization system.

---

# 14. Transition to ZED X Visual-Inertial Odometry

The final conclusion was that the IMU-only dead reckoning approach was not sufficiently reliable for the team's navigation requirements.

The team therefore transitioned to the **ZED X stereo camera**, which provides visual-inertial odometry.

Instead of relying primarily on inertial measurements, VIO combines:

- Camera-based visual information
- IMU measurements

The visual information provides additional information about how the camera—and therefore the submarine—is moving through the environment.

This produced a substantially more stable localization estimate than could be achieved through the custom IMU dead reckoning system alone.

The transition was therefore not a failure of the original project. The IMU dead reckoning system served as a useful exploration of the limitations of inertial-only localization and provided a working baseline that could be tested, visualized, and compared against the ZED X solution.

---

# 15. Final System Status

The final state of the project was:

### Implemented

- Raw Cube Orange IMU acceleration processing
- IMU orientation processing
- Gravity compensation
- Five-second accelerometer bias calibration
- Acceleration deadzone
- Acceleration moving-average filtering
- Velocity integration
- Velocity deadzone
- Velocity decay
- Angular-velocity-based suppression
- Barometer-to-depth conversion
- IMU/barometer synchronization
- Position estimation
- ROS 2 transform publishing
- Pose publishing
- RViz visualization

### Limitations

The primary limitation was accumulated drift from inertial dead reckoning.

Even after calibration and filtering, the system could not simultaneously achieve:

- Low drift
- High responsiveness
- Reliable translational movement detection

The filtering required to stabilize the estimate also suppressed legitimate motion.

### Final Decision

For reliable underwater localization, the team chose to use the **ZED X visual-inertial odometry estimate** instead of relying on the custom IMU-only dead reckoning system.

---

# 16. Key Lessons for Future Members

This project demonstrates several important concepts that are useful when working on the submarine's autonomy stack.

### 1. IMU integration accumulates error very quickly

Accelerometer noise and bias may look small when viewing raw sensor data, but integration turns small acceleration errors into velocity errors, and a second integration turns them into position drift.

### 2. Gravity compensation is highly dependent on orientation accuracy

Even a small orientation error can cause gravity to be incorrectly projected into the translational acceleration estimate. Because gravity is large compared with the submarine's typical acceleration, small orientation errors can have significant consequences.

### 3. Filtering always involves a tradeoff

Filtering can make a localization estimate appear much more stable, but excessive filtering can remove legitimate movement. The goal is not simply to eliminate all noise; it is to preserve useful signal while controlling error.

### 4. Sensor synchronization matters

When combining multiple sensors, measurements need to correspond to approximately the same point in time. Otherwise, the localization system can unintentionally combine incompatible measurements.

### 5. Different sensors have different strengths

The barometer was much better suited for estimating depth than double-integrating vertical acceleration. Similarly, the ZED X's combination of visual information and inertial measurements was much better suited for localization than an IMU-only system.

### 6. A working baseline is valuable even if it is eventually replaced

The custom dead reckoning system provided a complete end-to-end localization pipeline that could be visualized and tested in ROS 2/RViz. Building that baseline made it possible to understand the limitations of the approach and motivated the transition to a more appropriate localization technology.

---

# 17. High-Level Takeaway

The project began as an attempt to answer a relatively simple question:

> **Can we estimate the submarine's position underwater using only the sensors available on the Cube Orange?**

The answer was **partially**.

A complete real-time dead reckoning pipeline was developed using IMU acceleration, orientation, and barometric depth. The system included calibration, filtering, synchronization, ROS 2 integration, and RViz visualization. Extensive testing showed that the estimate could be stabilized significantly, but inertial drift and the resulting tradeoff between stability and responsiveness prevented it from being reliable enough for primary navigation.

The project ultimately demonstrated why robust localization generally requires more than inertial measurements alone and led the team toward using the ZED X's visual-inertial odometry as the more reliable localization solution.
