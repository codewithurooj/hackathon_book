# Troubleshooting Guide

## Introduction

Gazebo simulation can be challenging, especially for beginners. This guide covers common issues, their root causes, and step-by-step solutions. Use this as a reference when things don't work as expected.

**Troubleshooting Strategy**:
1. **Identify symptoms**: What specifically is broken?
2. **Check logs**: Gazebo terminal output often reveals errors
3. **Isolate the problem**: Simplify to minimal reproducible case
4. **Test incrementally**: Fix one thing at a time, verify each fix
5. **Ask for help**: After trying solutions here, seek community support

---

## Gazebo Won't Launch

### Symptom

Running `gazebo` command does nothing, or crashes immediately with no GUI.

### Possible Causes

1. **Gazebo not installed correctly**
2. **Missing dependencies**
3. **Graphics driver issues**
4. **Conflicting Gazebo versions**

### Solutions

#### 1. Verify Installation

```bash
# Check if Gazebo is installed
which gazebo
# Should output: /usr/bin/gazebo

# Check version
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x
```

If not installed or wrong version:

```bash
# Remove old versions
sudo apt remove gazebo*

# Install Gazebo Classic 11
sudo apt update
sudo apt install gazebo11 libgazebo11-dev
```

#### 2. Check Dependencies

```bash
# Install missing dependencies
sudo apt install libgazebo11-dev gazebo11-plugin-base gazebo11-common
```

#### 3. Test Graphics Drivers

```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
# Should show OpenGL 3.0 or higher

# If not available, install Mesa drivers
sudo apt install mesa-utils libgl1-mesa-glx libgl1-mesa-dri
```

For NVIDIA GPUs:

```bash
# Check NVIDIA driver
nvidia-smi

# If not working, install proprietary drivers
sudo ubuntu-drivers autoinstall
sudo reboot
```

#### 4. Test Headless Mode

If GUI fails but headless works:

```bash
# Launch without GUI
gzserver

# In another terminal, launch GUI separately
gzclient
```

If `gzserver` works but `gzclient` fails, it's a graphics driver issue.

#### 5. Clear Gazebo Cache

Corrupted cache can cause crashes:

```bash
# Remove Gazebo cache
rm -rf ~/.gazebo/

# Relaunch Gazebo
gazebo
```

---

## Gazebo Crashes During Simulation

### Symptom

Gazebo launches successfully but crashes after loading a world or spawning a model.

### Possible Causes

1. **Invalid SDF/URDF syntax**
2. **Missing mesh files**
3. **Physics instability** (objects exploding)
4. **Memory issues**

### Solutions

#### 1. Check Terminal Output

Look for error messages:

```bash
gazebo worlds/my_world.world
```

Common errors:
- `Unable to find uri[model://...]`: Missing model
- `Invalid SDF`: Syntax error in world/model file
- `Segmentation fault`: Physics explosion or bad geometry

#### 2. Validate SDF/URDF Syntax

```bash
# Validate SDF file
gz sdf -k my_world.world
# Should output: Check complete

# Validate URDF file
check_urdf my_robot.urdf
# Should output: robot name is: my_robot
```

Fix syntax errors reported by these tools.

#### 3. Check for Missing Meshes

If error mentions `mesh://` or `model://`:

```bash
# List Gazebo model paths
echo $GAZEBO_MODEL_PATH

# Add custom model path
export GAZEBO_MODEL_PATH=/path/to/models:$GAZEBO_MODEL_PATH

# Or copy models to default location
cp -r my_models/* ~/.gazebo/models/
```

#### 4. Reduce Physics Complexity

If physics causes crashes:

```xml
<!-- Increase time step (less accurate but more stable) -->
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Increased from 0.001 -->
  <ode>
    <solver>
      <iters>50</iters>  <!-- Decreased from 100 -->
    </solver>
  </ode>
</physics>
```

#### 5. Monitor Memory Usage

```bash
# Watch memory while Gazebo runs
watch -n 1 free -h

# If memory fills up, reduce scene complexity:
# - Fewer objects
# - Simpler meshes
# - Lower camera resolution
```

---

## Robot Model Not Visible in Gazebo

### Symptom

Robot spawns (appears in model list) but is invisible in Gazebo GUI.

### Possible Causes

1. **Missing visual elements**
2. **Transparent materials**
3. **Mesh file not found**
4. **Model spawned far from origin**

### Solutions

#### 1. Check Visual Elements in URDF

Ensure each link has `<visual>` tag:

```xml
<link name="base_link">
  <!-- Collision is present but visual missing! -->
  <collision>
    <geometry><box size="1 1 1"/></geometry>
  </collision>

  <!-- Add this: -->
  <visual>
    <geometry><box size="1 1 1"/></geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>  <!-- NOT transparent -->
    </material>
  </visual>
</link>
```

#### 2. Check Material Alpha

Material with alpha=0 is invisible:

```xml
<!-- Bad: Invisible -->
<color rgba="1 0 0 0"/>  <!-- Last value (alpha) is 0 -->

<!-- Good: Visible red -->
<color rgba="1 0 0 1"/>  <!-- Alpha = 1 (opaque) -->
```

#### 3. Verify Mesh Path

If using mesh:

```xml
<visual>
  <geometry>
    <mesh>
      <uri>file:///full/path/to/mesh.stl</uri>  <!-- Use absolute path for testing -->
    </mesh>
  </geometry>
</visual>
```

Test mesh loads correctly:

```bash
# Check file exists
ls -l /full/path/to/mesh.stl

# View mesh in MeshLab (if installed)
meshlab mesh.stl
```

#### 4. Find Model in GUI

Model might be spawned far from camera:

1. In Gazebo GUI, go to **World → Models**
2. Right-click on your robot → **Move To**
3. Set position (0, 0, 1)
4. Click **Apply**

Or use GUI camera controls:
- **Scroll wheel**: Zoom
- **Middle-click drag**: Pan
- **Right-click drag**: Orbit

---

## Physics Unstable: Objects Exploding or Vibrating

### Symptom

Objects jitter violently, fly apart, or fall through the ground.

### Possible Causes

1. **Time step too large**
2. **Incorrect inertia properties**
3. **Collision geometry issues**
4. **Contact parameters too stiff/soft**

### Solutions

#### 1. Reduce Time Step

```xml
<physics type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- Decrease from 0.001 -->
  <real_time_update_rate>2000</real_time_update_rate>  <!-- Increase accordingly -->
</physics>
```

**Rule of thumb**: Smaller time step = more stable (but slower).

#### 2. Verify Inertia Tensors

Common mistakes:

```xml
<!-- WRONG: Zero inertia -->
<inertia ixx="0" iyy="0" izz="0" .../>

<!-- WRONG: Negative inertia -->
<inertia ixx="-0.1" .../>

<!-- WRONG: Inconsistent with geometry -->
<visual><geometry><box size="1 1 1"/></box></geometry></visual>
<inertial>
  <mass>10</mass>
  <inertia ixx="0.001" iyy="0.001" izz="0.001" .../>  <!-- Too small for 1m box! -->
</inertial>
```

**Fix**: Recalculate inertia using formulas or CAD software. For a 1×1×1m box with mass 10kg:

```python
m = 10.0
x, y, z = 1.0, 1.0, 1.0

Ixx = (1/12) * m * (y**2 + z**2)  # = 1.667
Iyy = (1/12) * m * (x**2 + z**2)  # = 1.667
Izz = (1/12) * m * (x**2 + y**2)  # = 1.667
```

```xml
<inertia ixx="1.667" iyy="1.667" izz="1.667" ixy="0" ixz="0" iyz="0"/>
```

#### 3. Simplify Collision Geometry

Complex meshes cause instability:

```xml
<!-- BEFORE: Complex mesh (1000s of triangles) -->
<collision>
  <geometry>
    <mesh><uri>model://robot/meshes/complex_body.stl</uri></mesh>
  </geometry>
</collision>

<!-- AFTER: Primitive approximation -->
<collision>
  <geometry>
    <box size="0.6 0.4 0.2"/>  <!-- Much faster! -->
  </geometry>
</collision>
```

Visual mesh can remain detailed; only collision needs simplification.

#### 4. Tune Contact Parameters

If objects penetrate or bounce excessively:

```xml
<surface>
  <contact>
    <ode>
      <!-- Increase stiffness if penetrating -->
      <kp>1000000.0</kp>  <!-- Try 1e6 to 1e8 -->

      <!-- Increase damping if bouncing -->
      <kd>10.0</kd>  <!-- Try 1.0 to 100.0 -->

      <!-- Limit correction velocity -->
      <max_vel>0.01</max_vel>

      <!-- Minimum penetration before contact -->
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
</surface>
```

#### 5. Increase Solver Iterations

```xml
<ode>
  <solver>
    <iters>100</iters>  <!-- Increase from 50 -->
  </solver>
</ode>
```

More iterations = better convergence, but slower.

---

## Sensors Not Publishing Data

### Symptom

LIDAR, camera, or IMU plugin loads but topics don't appear or have no data.

### Possible Causes

1. **Plugin not loaded**
2. **Wrong topic name**
3. **ROS 2 namespace issues**
4. **Sensor update rate zero**

### Solutions

#### 1. Check Gazebo Output

Look for sensor loading messages:

```bash
gazebo my_world.world
# Expected output:
# [INFO] [gazebo_ros_ray_sensor]: Publishing LaserScan to [/scan]
# [INFO] [gazebo_ros_camera]: Publishing Image to [/camera/image_raw]
```

If missing, plugin didn't load. Check filename:

```xml
<!-- Check plugin filename matches library -->
<plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
  <!-- ... -->
</plugin>
```

Verify library exists:

```bash
# Find Gazebo plugins
find /opt/ros/humble -name "libgazebo_ros_ray_sensor.so"
```

#### 2. List ROS 2 Topics

```bash
# List all topics
ros2 topic list

# Check specific topic
ros2 topic info /scan

# Echo topic to see data
ros2 topic echo /scan
```

If topic missing, check namespace:

```xml
<plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
  <ros>
    <namespace>/my_robot</namespace>  <!-- Topic will be /my_robot/scan -->
    <remapping>~/out:=scan</remapping>
  </ros>
  <!-- ... -->
</plugin>
```

#### 3. Check Update Rate

```xml
<sensor name="lidar" type="ray">
  <update_rate>10.0</update_rate>  <!-- NOT 0! -->
  <!-- ... -->
</sensor>
```

Zero update rate = no data.

#### 4. Verify Frame Name

```xml
<plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
  <frame_name>lidar_link</frame_name>  <!-- Must match URDF link name -->
</plugin>
```

If frame doesn't exist, sensor won't publish.

#### 5. Test Minimal Sensor

Create minimal test SDF:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="sensor_test">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <model name="test_lidar">
      <pose>0 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <sensor name="lidar" type="ray">
          <update_rate>10</update_rate>
          <ray>
            <scan>
              <horizontal>
                <samples>10</samples>
                <min_angle>-1.57</min_angle>
                <max_angle>1.57</max_angle>
              </horizontal>
            </scan>
            <range><min>0.1</min><max>10</max></range>
          </ray>
          <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
            <ros><remapping>~/out:=scan</remapping></ros>
            <output_type>sensor_msgs/LaserScan</output_type>
            <frame_name>link</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
```

```bash
gazebo sensor_test.world
ros2 topic echo /scan  # Should see data
```

If this works, problem is in your URDF/SDF.

---

## Simulation Running Slowly

### Symptom

Real-time factor < 1.0 (simulation slower than real-time).

### Possible Causes

1. **Scene too complex**
2. **High-resolution sensors**
3. **Inefficient collision geometry**
4. **Physics time step too small**

### Solutions

#### 1. Check Real-Time Factor

```bash
# While Gazebo running, in another terminal:
gz stats

# Output:
# Factor[0.65] SimTime[10.50] RealTime[16.15] Paused[F]
#        ^^^^ < 1.0 = too slow
```

#### 2. Simplify Scene

- Remove unnecessary models
- Use `<static>true</static>` for objects that don't move
- Disable shadows:

```xml
<model name="my_model">
  <link name="link">
    <visual>
      <cast_shadows>false</cast_shadows>  <!-- Faster rendering -->
      <!-- ... -->
    </visual>
  </link>
</model>
```

#### 3. Reduce Sensor Rates

```xml
<!-- BEFORE: High camera frame rate -->
<sensor name="camera" type="camera">
  <update_rate>60.0</update_rate>  <!-- 60 FPS -->
  <camera>
    <image>
      <width>1920</width>  <!-- 1080p -->
      <height>1080</height>
    </image>
  </camera>
</sensor>

<!-- AFTER: Lower for development -->
<sensor name="camera" type="camera">
  <update_rate>10.0</update_rate>  <!-- 10 FPS -->
  <camera>
    <image>
      <width>640</width>  <!-- VGA -->
      <height>480</height>
    </image>
  </camera>
</sensor>
```

#### 4. Optimize Collision Meshes

Replace complex meshes with primitives:

```bash
# Count triangles in mesh
meshlab mesh.stl
# Info → Show Layer Dialog → See triangle count

# Simplify in MeshLab:
# Filters → Remeshing → Quadric Edge Collapse Decimation
# Target: < 1000 triangles for collision
```

#### 5. Increase Time Step (Trade Accuracy)

```xml
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Increase from 0.001 -->
  <real_time_update_rate>500</real_time_update_rate>  <!-- Decrease from 1000 -->
</physics>
```

**Warning**: May reduce stability.

#### 6. Run Headless

GUI rendering is slow. For batch tests:

```bash
# Launch only server (no GUI)
gzserver my_world.world

# Use RViz for visualization instead
rviz2
```

---

## Robot State Publisher Errors

### Symptom

Errors like:
```
[robot_state_publisher]: Could not find joint 'wheel_joint' in URDF
[robot_state_publisher]: URDF not published on /robot_description
```

### Solutions

#### 1. Check URDF Loaded

```bash
# Check if robot_description parameter exists
ros2 param list | grep robot_description

# View URDF content
ros2 param get /robot_state_publisher robot_description
```

If not found, URDF not loaded. Check launch file:

```python
# In launch file
with open(urdf_file, 'r') as f:
    robot_description = f.read()

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]  # Must pass URDF content
)
```

#### 2. Verify Joint Names Match

Joint names in URDF must match exactly (case-sensitive):

```bash
# Extract joint names from URDF
grep '<joint name=' my_robot.urdf

# Compare to joint_states topic
ros2 topic echo /joint_states
```

Fix mismatches in URDF or controller configuration.

---

## RViz Not Showing Robot

### Symptom

RViz opens but robot model doesn't appear or is all grey.

### Solutions

#### 1. Check Fixed Frame

In RViz, **Global Options → Fixed Frame** must match a frame published by `robot_state_publisher`:

```bash
# List available frames
ros2 run tf2_ros tf2_echo base_link odom

# If error, frames not published
```

Set Fixed Frame to `base_link` or `odom` in RViz.

#### 2. Add RobotModel Display

In RViz:
1. Click **Add**
2. Select **RobotModel**
3. Set **Description Topic** to `/robot_description`

#### 3. Check TF Tree

```bash
# Visualize TF tree
ros2 run tf2_tools view_frames

# Opens PDF showing all frames
evince frames.pdf
```

Ensure all links connected.

---

## Common Error Messages and Fixes

### `Error [parser.cc:581] Unable to find uri[model://...]`

**Cause**: Model not found in Gazebo model path.

**Fix**:
```bash
# Add model directory to path
export GAZEBO_MODEL_PATH=/path/to/models:$GAZEBO_MODEL_PATH

# Or use file:// URI
<uri>file:///absolute/path/to/model</uri>
```

### `Warning [Convenience.cc:1070] Deleting a connection right after creation`

**Cause**: Plugin loading issue (usually harmless).

**Fix**: Ignore if simulation works. Or update Gazebo/ROS packages:
```bash
sudo apt update && sudo apt upgrade
```

### `gzclient: symbol lookup error: ... undefined symbol`

**Cause**: Library version mismatch.

**Fix**:
```bash
# Rebuild Gazebo plugins
cd ~/ros2_ws
colcon build --packages-select gazebo_ros_pkgs --cmake-clean-cache
source install/setup.bash
```

### `QXcbConnection: Could not connect to display`

**Cause**: No X server (GUI) available.

**Fix**: Run headless:
```bash
gzserver my_world.world  # No GUI
```

Or enable X forwarding (SSH):
```bash
ssh -X user@robot
```

---

## Performance Optimization Checklist

When simulation is too slow:

- [ ] Reduce sensor update rates (cameras to 10 Hz)
- [ ] Simplify collision meshes (< 1000 triangles)
- [ ] Use primitives instead of meshes where possible
- [ ] Mark static objects as `<static>true</static>`
- [ ] Disable shadows (`<cast_shadows>false</cast_shadows>`)
- [ ] Reduce physics rate (`max_step_size` to 0.002)
- [ ] Run headless (`gzserver` only)
- [ ] Close unnecessary programs (web browsers, etc.)
- [ ] Use GPU for rendering (check `nvidia-smi`)

---

## Getting Additional Help

If issues persist after trying these solutions:

1. **Gazebo Answers**: https://answers.gazebosim.org/
   - Search existing questions
   - Post new question with error logs

2. **ROS Discourse**: https://discourse.ros.org/
   - Active ROS 2 community

3. **GitHub Issues**:
   - `gazebo_ros_pkgs`: https://github.com/ros-simulation/gazebo_ros_pkgs/issues

4. **Check System Requirements**:
   - Gazebo 11 requires Ubuntu 20.04+ (or equivalent)
   - OpenGL 3.0+ graphics
   - 4GB+ RAM recommended

**When Asking for Help, Include**:
- Gazebo version (`gazebo --version`)
- ROS 2 version (`ros2 --version`)
- Ubuntu version (`lsb_release -a`)
- Full error messages from terminal
- Minimal reproducible example (simplest SDF/URDF that shows issue)

---

## Summary

Most Gazebo issues fall into these categories:

1. **Installation/Environment**: Verify Gazebo version, dependencies, graphics drivers
2. **Syntax Errors**: Validate SDF/URDF with `gz sdf` and `check_urdf`
3. **Physics Instability**: Tune time step, inertia, contact parameters
4. **Missing Data**: Check plugin loading, topic names, update rates
5. **Performance**: Simplify scene, reduce sensor rates, optimize meshes

**Debugging Approach**: Start simple (minimal SDF), add complexity incrementally, test after each change.

With systematic troubleshooting, most issues can be resolved quickly. Don't get discouraged—simulation challenges are part of the learning process!
