# Section 4: URDF with Sensors

## Introduction

In Chapter 2, you learned to create URDF robot descriptions. Now you'll extend those models by adding **Gazebo sensor plugins** that simulate LIDAR, cameras, and IMUs.

Sensors in URDF require:
1. A **link** representing the sensor's physical location
2. A **Gazebo plugin** that generates sensor data
3. **ROS 2 topic configuration** for publishing data

## Adding LIDAR to a Robot

Let's add a 2D LIDAR sensor to a simple mobile robot.

### Step 1: Add Sensor Link

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.07</length>
      </cylinder>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.07</length>
      </cylinder>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
</joint>
```

This creates a small cylinder representing the LIDAR, mounted 0.2m forward and 0.15m up from the robot base.

### Step 2: Add Gazebo LIDAR Plugin

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>

    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>

      <range>
        <min>0.10</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>

    <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LIDAR Parameter Explanation

| Parameter | Description | Example Value |
|-----------|-------------|---------------|
| `samples` | Number of laser beams | 720 (0.5° resolution) |
| `min_angle` / `max_angle` | Scan range in radians | -π to +π (360°) |
| `min` / `max` range | Detection distance (m) | 0.1m to 10m |
| `update_rate` | Scans per second (Hz) | 10 Hz |
| `noise stddev` | Gaussian noise std dev | 0.01m (1cm) |

### Step 3: Test the LIDAR

```bash
# Launch Gazebo with your robot
ros2 launch my_robot_description gazebo.launch.py

# Check if LIDAR is publishing
ros2 topic list | grep scan

# View LIDAR data
ros2 topic echo /robot/scan

# Visualize in RViz
ros2 run rviz2 rviz2
```

In RViz:
- Set Fixed Frame to `base_link`
- Add → By Topic → `/robot/scan` → LaserScan
- You'll see red lines showing laser beams

## Adding an RGB Camera

Cameras provide visual information for object recognition and navigation.

### Step 1: Add Camera Link

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.05"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
</joint>
```

### Step 2: Add Gazebo Camera Plugin

```xml
<gazebo reference="camera_link">
  <sensor name="camera_sensor" type="camera">
    <update_rate>30</update_rate>
    <visualize>true</visualize>

    <camera name="camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>

      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>

    <plugin name="gazebo_ros_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Parameter Explanation

| Parameter | Description | Example Value |
|-----------|-------------|---------------|
| `horizontal_fov` | Field of view (radians) | 1.3962634 (80°) |
| `width` x `height` | Image resolution | 640x480 pixels |
| `format` | Color format | R8G8B8 (RGB 8-bit) |
| `near` / `far` clip | Render distance (m) | 0.02m to 300m |
| `update_rate` | Frames per second | 30 Hz |

### Step 3: View Camera Feed

```bash
# Check camera topics
ros2 topic list | grep camera

# View image data
ros2 run rqt_image_view rqt_image_view /robot/camera/image_raw
```

## Adding a Depth Camera

Depth cameras (like Intel RealSense) provide RGB + depth maps.

### Gazebo Depth Camera Plugin

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>20</update_rate>
    <visualize>true</visualize>

    <camera name="depth_camera">
      <horizontal_fov>1.047198</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>3.0</far>
      </clip>
    </camera>

    <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/image_raw:=depth_camera/image_raw</remapping>
        <remapping>~/depth/image_raw:=depth_camera/depth/image_raw</remapping>
        <remapping>~/points:=depth_camera/points</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>depth_camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

The depth camera publishes:
- `/robot/depth_camera/image_raw`: RGB image
- `/robot/depth_camera/depth/image_raw`: Depth map (distance to each pixel)
- `/robot/depth_camera/points`: 3D point cloud

## Adding an IMU

IMU (Inertial Measurement Unit) measures acceleration, angular velocity, and orientation.

### Step 1: Add IMU Link

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
```

### Step 2: Add Gazebo IMU Plugin

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>

    <plugin name="gazebo_ros_imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Data

The IMU publishes `sensor_msgs/Imu` messages with:
- **Orientation**: Quaternion (x, y, z, w)
- **Angular Velocity**: rad/s (roll, pitch, yaw rates)
- **Linear Acceleration**: m/s² (X, Y, Z acceleration)

### Test IMU

```bash
# View IMU data
ros2 topic echo /robot/imu

# Visualize orientation in RViz
# Add → By Topic → /robot/imu → Imu
```

## Complete Example: Mobile Robot with Full Sensor Suite

Here's a complete URDF excerpt with LIDAR, camera, and IMU:

```xml
<?xml version="1.0"?>
<robot name="mobile_robot_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LIDAR -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.15" rpy="0 0 0"/>
  </joint>

  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
        </range>
      </ray>
      <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_plugin">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin filename="libgazebo_ros_camera.so" name="camera_plugin">
        <ros>
          <remapping>~/image_raw:=camera/image_raw</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <remapping>~/out:=imu</remapping>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Sensor Placement Best Practices

1. **LIDAR**: Mount high and centered for best 360° coverage
2. **Camera**: Mount at "eye level" pointing forward, avoid occlusions
3. **Depth Camera**: Similar to camera, but consider range (0.3-3m typical)
4. **IMU**: Place near center of mass for accurate acceleration measurement
5. **Multiple Sensors**: Ensure no physical collisions between sensor links

## Troubleshooting Sensor Issues

### Sensor Not Publishing Data

**Check**:
```bash
ros2 topic list  # Is the topic present?
ros2 topic hz /scan  # Is it publishing?
```

**Common fixes**:
- Verify plugin filename (e.g., `libgazebo_ros_ray_sensor.so`)
- Check Gazebo console for error messages
- Ensure `<always_on>true</always_on>` or simulation is playing

### Sensor Data Looks Wrong

**LIDAR shows no obstacles**: Check range parameters and ensure objects are within detection range

**Camera shows black image**: Check lighting in Gazebo world (add sun or lights)

**IMU orientation wrong**: Verify link coordinate frame and joint orientation

## What You Learned

In this section, you:

- Added LIDAR sensors to robot URDF files
- Configured RGB and depth cameras with Gazebo plugins
- Integrated IMU sensors for orientation and acceleration
- Learned sensor parameters (FoV, resolution, update rate, noise)
- Created a complete mobile robot with multiple sensors
- Debugged common sensor simulation issues

## Next Steps

Now that you can add sensors to robots, you're ready to:

- Fine-tune physics properties ([Physics Properties](./05-physics-properties.md))
- Simulate complete humanoid robots ([Humanoid Simulation](./07-humanoid-simulation.md))
- Process sensor data for perception algorithms

## Exercises

1. **Add LIDAR**: Take a simple mobile robot URDF and add a 360° LIDAR sensor
2. **Camera Configuration**: Experiment with different camera resolutions and FoV values
3. **Multi-Sensor Robot**: Create a robot with LIDAR, camera, and IMU, then visualize all three in RViz

Solutions are available in [Section 9: Exercises](./09-exercises.md).

## Additional Resources

- [Gazebo ROS Plugins](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [sensor_msgs Documentation](https://github.com/ros2/common_interfaces/tree/rolling/sensor_msgs)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
