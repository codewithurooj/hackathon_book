# Section 3: Sensor Simulation

## Introduction to Sensor Simulation

Sensors are critical for robot perception - they allow robots to understand their environment. Gazebo simulates realistic sensor data including:

- **LIDAR (Light Detection and Ranging)**: 2D/3D laser scans for obstacle detection
- **Cameras**: RGB images, depth maps, stereo vision
- **IMU (Inertial Measurement Unit)**: Acceleration, angular velocity, orientation
- **Force/Torque Sensors**: Contact forces on grippers and feet
- **GPS**: Global positioning (outdoor scenarios)

Simulated sensors generate data in the same format as real sensors, allowing you to develop perception algorithms before deploying to hardware.

## Why Simulate Sensors?

1. **Safe Testing**: Test perception algorithms without damaging real sensors
2. **Perfect Ground Truth**: Know exact robot state for algorithm validation
3. **Controllable Noise**: Add realistic noise models to test robustness
4. **Rapid Iteration**: Test multiple sensor configurations quickly
5. **Cost Effective**: Experiment with expensive sensors (3D LIDAR) before purchasing

## LIDAR Simulation

LIDAR sensors emit laser beams and measure time-of-flight to detect obstacles. They're essential for navigation and mapping.

### 2D LIDAR (Laser Scan)

Emits laser beams in a single plane (like a "slice" through the environment).

**Key Parameters**:
- **Range**: Maximum detection distance (e.g., 10 meters)
- **Resolution**: Number of beams (e.g., 720 samples = 0.5° resolution)
- **Scan Rate**: Updates per second (e.g., 10 Hz)
- **Field of View**: Angular coverage (e.g., 270° or full 360°)

**Applications**:
- Mobile robot navigation
- Obstacle avoidance
- SLAM (Simultaneous Localization and Mapping)

### 3D LIDAR (Point Cloud)

Emits laser beams in multiple planes to create 3D point clouds.

**Applications**:
- Autonomous vehicles
- 3D mapping
- Object recognition

## Camera Simulation

Gazebo simulates various camera types:

### RGB Camera

Generates color images like a standard camera.

**Parameters**:
- **Resolution**: Image size (e.g., 640x480, 1920x1080)
- **Field of View (FoV)**: Horizontal and vertical viewing angle
- **Update Rate**: Frames per second (e.g., 30 Hz)
- **Noise**: Gaussian noise to simulate real camera imperfections

### Depth Camera (RGB-D)

Provides both color images and depth maps (distance to each pixel).

**Examples**: Intel RealSense, Microsoft Kinect

**Applications**:
- Object grasping
- Obstacle detection
- 3D reconstruction

### Stereo Camera

Two cameras separated by a baseline (like human eyes) to compute depth through disparity.

**Applications**:
- Depth estimation
- Visual odometry
- SLAM

## IMU Simulation

IMU (Inertial Measurement Unit) measures:

- **Linear Acceleration**: X, Y, Z acceleration (m/s²)
- **Angular Velocity**: Roll, pitch, yaw rates (rad/s)
- **Orientation**: Quaternion or Euler angles

**Applications**:
- Robot balance (humanoids)
- Orientation estimation
- Sensor fusion with other sensors

**Noise Considerations**: Real IMUs have bias, drift, and noise. Gazebo can simulate these imperfections.

## Adding Sensors to Gazebo Plugins

Gazebo uses **plugins** to simulate sensors. These plugins:

1. Generate sensor data based on simulation state
2. Publish data to ROS 2 topics
3. Simulate realistic noise and imperfections

We'll cover how to add sensors to URDF files in [Section 4: URDF with Sensors](./04-urdf-sensors.md).

## Visualizing Sensor Data in RViz

RViz is ROS 2's visualization tool. You can visualize:

- **LaserScan**: 2D LIDAR beams as red lines
- **PointCloud2**: 3D LIDAR as colored points
- **Image**: Camera images in a separate window
- **Camera Info**: Camera calibration parameters
- **IMU**: Orientation as arrows or axes

### Example: Visualizing LIDAR

1. Launch Gazebo with a robot that has LIDAR
2. Launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
3. In RViz:
   - Set Fixed Frame to `base_link` or `odom`
   - Add → By Topic → `/scan` → LaserScan
   - You'll see red lines representing laser beams

### Example: Visualizing Camera

1. Add → By Topic → `/camera/image_raw` → Image
2. A separate window shows the camera feed

## Sensor Noise Models

Real sensors aren't perfect. Gazebo simulates:

### Gaussian Noise

Random variation around the true value.

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

**When to use**: Camera pixels, LIDAR ranges, IMU measurements

### Custom Noise

For advanced simulation, you can implement custom noise models (e.g., bias drift for IMU, motion blur for cameras).

## Sensor Performance Considerations

Simulating sensors is computationally expensive. Tips:

1. **Reduce Resolution**: 640x480 camera instead of 1920x1080
2. **Lower Update Rates**: 10 Hz LIDAR instead of 40 Hz
3. **Limit Ray Count**: 360 laser beams instead of 720
4. **Disable Unused Sensors**: Comment out sensors you're not using

## Sensor Data Topics

Gazebo plugins publish sensor data to ROS 2 topics:

| Sensor Type | Message Type | Example Topic |
|------------|--------------|---------------|
| 2D LIDAR | `sensor_msgs/LaserScan` | `/scan` |
| 3D LIDAR | `sensor_msgs/PointCloud2` | `/points` |
| RGB Camera | `sensor_msgs/Image` | `/camera/image_raw` |
| Depth Camera | `sensor_msgs/Image` | `/camera/depth/image_raw` |
| IMU | `sensor_msgs/Imu` | `/imu` |
| GPS | `sensor_msgs/NavSatFix` | `/gps/fix` |

### Inspecting Topics

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /scan

# Echo topic data (view messages)
ros2 topic echo /scan

# Check topic frequency
ros2 topic hz /scan
```

## Sensor Coordinate Frames

Each sensor has a coordinate frame (transform). Understanding frames is crucial for:

- Interpreting sensor data correctly
- Fusing data from multiple sensors
- Visualizing in RViz

**Common frame convention**:
- **X**: Forward
- **Y**: Left
- **Z**: Up

**Example**: A camera at the front of a robot has its X-axis pointing forward (viewing direction).

## Multi-Sensor Fusion

Advanced robots combine multiple sensors:

- **LIDAR + Camera**: Object detection with range and visual information
- **IMU + Wheel Odometry**: More accurate position estimation
- **Stereo Camera + LIDAR**: Redundant depth estimation

Gazebo allows you to test sensor fusion algorithms in simulation before deploying to hardware.

## Simulating Sensor Failures

To test robustness, you can simulate sensor failures:

- **Occlusion**: Objects blocking sensor view
- **Noise Spikes**: Temporary high noise
- **Dropout**: Sensor stops publishing data

This prepares your algorithms for real-world imperfections.

## What You Learned

In this section, you:

- Understood different sensor types (LIDAR, cameras, IMU)
- Learned sensor parameters (range, resolution, FoV, update rate)
- Explored how Gazebo plugins generate sensor data
- Visualized sensor data in RViz
- Applied noise models for realistic simulation
- Inspected ROS 2 sensor topics
- Considered performance optimization for sensor simulation

## Next Steps

Now that you understand sensor simulation concepts, you're ready to:

- Add sensors to robot URDF files ([URDF with Sensors](./04-urdf-sensors.md))
- Configure sensor parameters for your application
- Process sensor data for perception algorithms

## Exercises

1. **Topic Inspection**: Launch a Gazebo world with sensors and list all available sensor topics
2. **RViz Visualization**: Visualize LIDAR and camera data from a simulated robot in RViz
3. **Noise Analysis**: Compare sensor data with and without noise models enabled

Solutions are available in [Section 9: Exercises](./09-exercises.md).

## Additional Resources

- [Gazebo Sensor Plugins](http://gazebosim.org/tutorials?cat=sensors)
- [ROS 2 Sensor Messages](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
