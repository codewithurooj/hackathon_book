# Humanoid Robot Simulation

## Introduction

Humanoid robots present unique simulation challenges due to their complexity: high degree-of-freedom (DOF) kinematic chains, complex sensor suites, bipedal balance requirements, and intricate contact dynamics. This section teaches you how to successfully simulate humanoid robots in Gazebo, covering model loading, sensor integration, physics tuning, and control.

By the end of this section, you'll be able to:

- Load humanoid robot models (NAO, PR2, custom designs) in Gazebo
- Configure realistic sensor suites (cameras, LIDAR, IMU)
- Tune physics for stable bipedal stance and motion
- Implement basic joint control
- Validate simulated behavior against specifications

## Why Humanoid Simulation is Challenging

Humanoid robots are among the most difficult systems to simulate accurately:

### 1. High DOF Kinematic Chains

Typical humanoids have 20-50+ joints:
- **NAO**: 25 DOF (head, arms, legs, hands)
- **PR2**: 32 DOF (mobile base, arms, head, torso)
- **Atlas**: 28 DOF (full-body bipedal)
- **Custom humanoids**: 30-60+ DOF with articulated hands

**Challenges**:
- Complex inverse kinematics
- Joint coupling and limits
- Computational cost of collision detection
- Numerical stability in physics solver

### 2. Complex Contact Dynamics

Unlike wheeled robots with continuous ground contact, humanoids have:
- **Intermittent contacts**: Feet lift during walking
- **Multiple simultaneous contacts**: Both feet, hands, knees
- **Small contact areas**: Foot soles are small relative to body mass
- **High contact forces**: Balancing requires large ground reaction forces

**Consequences**:
- Simulation instability (falling, jitter, penetration)
- Requires careful physics tuning (time step, solver iterations, contact stiffness)

### 3. Sensor Suite Complexity

Humanoids carry diverse sensors:
- **Vision**: Stereo cameras, RGB-D cameras (head-mounted)
- **Ranging**: LIDAR (chest or head), ultrasonic sensors
- **Proprioception**: Joint encoders, IMU (torso/pelvis)
- **Force/torque**: Foot pressure sensors, wrist F/T sensors

**Integration challenges**:
- Sensor placement and coordinate frames
- Realistic noise models
- Computational load (especially cameras)

### 4. Balance and Control

Bipedal balance requires:
- **Center of Mass (CoM) estimation**: Accurate inertia tensors
- **Zero Moment Point (ZMP) control**: Depends on contact forces
- **Fast control loops**: 200-1000 Hz for stabilization

**Simulation requirements**:
- High physics update rate (≥1000 Hz)
- Accurate inertia properties
- Realistic joint dynamics (damping, friction)

## Loading Humanoid Models

We'll use the **NAO** humanoid as an example (popular in education and research).

### Obtaining Humanoid Models

#### Option 1: Public Repositories

```bash
# Clone NAO model repository
cd ~/ros2_ws/src
git clone https://github.com/ros-naoqi/nao_robot.git -b ros2

# Clone PR2 model (if desired)
git clone https://github.com/pr2/pr2_common.git -b ros2

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

#### Option 2: Export from CAD

For custom humanoids:

1. Design in CAD (SolidWorks, Fusion 360, Onshape)
2. Export to URDF using SolidWorks to URDF Exporter or sw_urdf_exporter plugin
3. Manually add Gazebo plugins for sensors and control

### Inspecting the NAO URDF

```bash
# View URDF file
cat ~/ros2_ws/src/nao_robot/nao_description/urdf/nao_robot_v4.urdf

# Visualize in RViz (no physics)
ros2 launch nao_description display.launch.py
```

### Spawning in Gazebo

Create a launch file to spawn NAO in Gazebo:

**File**: `~/ros2_ws/src/chapter3_simulation/launch/nao_gazebo.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to Gazebo worlds
    gazebo_worlds_path = get_package_share_directory('gazebo_ros')

    # Path to NAO description
    nao_description_path = get_package_share_directory('nao_description')
    urdf_file = os.path.join(nao_description_path, 'urdf', 'nao_robot_v4.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': 'empty.world'}.items()
    )

    # Spawn NAO in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'nao',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.34'  # Spawn at standing height
        ],
        output='screen'
    )

    # Robot state publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

**Launch**:

```bash
ros2 launch chapter3_simulation nao_gazebo.launch.py
```

NAO should appear standing in Gazebo. If it falls immediately, physics tuning is needed (see below).

## Configuring Sensor Suites

Humanoids typically have multiple sensors. We'll add cameras, IMU, and LIDAR to NAO.

### Adding a Head Camera

Modify NAO URDF to add a camera plugin to the head:

```xml
<robot name="nao">
  <!-- Existing links and joints -->

  <!-- Head link with camera -->
  <link name="CameraTop_frame">
    <!-- Visual and collision elements -->
  </link>

  <!-- Gazebo plugin for camera -->
  <gazebo reference="CameraTop_frame">
    <sensor type="camera" name="head_camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.0472</horizontal_fov>  <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>

      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/nao</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
        </ros>
        <camera_name>head_camera</camera_name>
        <frame_name>CameraTop_frame</frame_name>
        <hack_baseline>0.07</hack_baseline>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**Verify camera topic**:

```bash
ros2 topic list | grep camera
# Output: /nao/camera/image_raw, /nao/camera/camera_info

# View camera feed in RViz
rviz2
# Add Image display, topic: /nao/camera/image_raw
```

### Adding an IMU Sensor

IMU is critical for balance and orientation estimation. Add to torso:

```xml
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <update_rate>100.0</update_rate>  <!-- 100 Hz for balance control -->
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/nao</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>torso</frame_name>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>

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
  </sensor>
</gazebo>
```

**Monitor IMU data**:

```bash
ros2 topic echo /nao/imu/data
```

Expected output includes orientation (quaternion), angular velocity, and linear acceleration.

### Adding Chest-Mounted LIDAR

For navigation, add a 2D LIDAR to the chest:

```xml
<link name="chest_lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.02"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass>0.1</mass>
    <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="chest_to_lidar" type="fixed">
  <parent link="torso"/>
  <child link="chest_lidar_link"/>
  <origin xyz="0.05 0 0.2" rpy="0 0 0"/>  <!-- 20cm above torso, 5cm forward -->
</joint>

<gazebo reference="chest_lidar_link">
  <sensor type="ray" name="chest_lidar">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
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

    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/nao</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>chest_lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Visualize LIDAR**:

```bash
# In RViz, add LaserScan display
# Topic: /nao/scan
# Fixed frame: torso or odom
```

## Physics Tuning for Stability

Humanoids often fall or jitter in simulation due to physics issues. Systematic tuning is essential.

### Step 1: Verify Inertia Properties

Incorrect inertia is the #1 cause of instability.

**Check**:
- All links have `<inertial>` tags with realistic mass and inertia
- No zero or negative inertia values
- Center of mass (`<pose>` within `<inertial>`) is accurate

**Calculate inertia** for each link using CAD or formulas (see Physics Properties section).

**Example**: Thigh link (cylinder, radius 0.05m, length 0.3m, mass 2kg)

```
Ixx = Iyy = (1/12) * m * (3*r² + h²) = (1/12) * 2 * (3*0.05² + 0.3²) = 0.0162
Izz = (1/2) * m * r² = (1/2) * 2 * 0.05² = 0.0025
```

### Step 2: Adjust Physics Time Step

Humanoids need smaller time steps than wheeled robots.

**In world SDF**:

```xml
<physics type="ode">
  <max_step_size>0.0005</max_step_size>  <!-- 0.5ms, was 1ms -->
  <real_time_update_rate>2000</real_time_update_rate>  <!-- 2000 Hz -->
  <ode>
    <solver>
      <iters>100</iters>  <!-- Increase from 50 -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Trade-off**: Smaller time step = more stable, but slower simulation.

### Step 3: Tune Contact Parameters

Increase foot-ground contact stiffness to prevent sinking:

```xml
<gazebo reference="left_foot">
  <collision name="left_foot_collision">
    <surface>
      <contact>
        <ode>
          <kp>10000000.0</kp>  <!-- High stiffness -->
          <kd>10.0</kd>         <!-- Moderate damping -->
          <max_vel>0.01</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>1.0</mu>   <!-- High friction for no slip -->
          <mu2>1.0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</gazebo>

<!-- Repeat for right_foot -->
```

### Step 4: Add Joint Damping

Prevent oscillations in joints:

```xml
<joint name="left_knee" type="revolute">
  <parent>left_thigh</parent>
  <child>left_shin</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0.0</lower>
      <upper>2.3</upper>  <!-- ~130 degrees -->
      <effort>100.0</effort>
      <velocity>3.0</velocity>
    </limit>
    <dynamics>
      <damping>1.0</damping>  <!-- Add damping -->
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

**Tuning guide**:
- Start with damping = 0.5
- If joints oscillate: increase to 1.0-2.0
- If motion is too slow: decrease to 0.2-0.5

### Step 5: Disable Self-Collision (Initially)

During initial testing, disable collisions between adjacent links:

```xml
<gazebo>
  <self_collide>false</self_collide>
</gazebo>
```

Once stable, re-enable and add collision filters for specific link pairs.

### Step 6: Use Simbody for Complex Humanoids

If instability persists with ODE, switch to Simbody:

```xml
<physics type="simbody">
  <max_step_size>0.001</max_step_size>
  <!-- Simbody-specific tuning -->
</physics>
```

Simbody is slower but more stable for high-DOF systems.

## Joint Control

Once stable, add joint position/velocity control.

### Using ros2_control

**Add ros2_control plugin to URDF**:

```xml
<ros2_control name="nao_control" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <!-- Joint 1: Left Knee -->
  <joint name="left_knee">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">2.3</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <!-- Repeat for all controlled joints -->

</ros2_control>

<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find nao_control)/config/nao_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**Controller configuration** (`nao_controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: joint_trajectory_controller/JointTrajectoryController

position_controller:
  ros__parameters:
    joints:
      - left_knee
      - right_knee
      - left_hip_pitch
      - right_hip_pitch
      # ... all controlled joints

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0
```

**Launch controllers**:

```bash
ros2 launch nao_control nao_control.launch.py
```

**Send joint commands**:

```bash
# Example: Bend left knee to 1 radian
ros2 topic pub /position_controller/joint_trajectory trajectory_msgs/JointTrajectory "
joint_names: ['left_knee']
points:
  - positions: [1.0]
    time_from_start:
      sec: 1
      nanosec: 0
" --once
```

### Simple Joint Publisher (for Testing)

For quick tests, publish directly to Gazebo joint topics:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher = self.create_publisher(Float64, '/nao/left_knee_position_controller/command', 10)
        self.timer = self.create_timer(0.01, self.publish_command)  # 100 Hz
        self.angle = 0.0

    def publish_command(self):
        msg = Float64()
        msg.data = self.angle
        self.publisher.publish(msg)

        # Oscillate knee
        self.angle += 0.01
        if self.angle > 1.5:
            self.angle = 0.0

def main():
    rclpy.init()
    node = JointCommander()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Validation Checklist

Before using humanoid simulation for algorithm development, validate:

### 1. Visual Inspection

- [ ] Model loads without errors
- [ ] All links visible and correctly positioned
- [ ] Joints move within expected ranges
- [ ] No unexpected collisions or interpenetration

### 2. Physics Stability

- [ ] Robot stands without falling for ≥10 seconds
- [ ] No jittering or vibration in stationary pose
- [ ] Foot contact points visible and stable
- [ ] Real-time factor ≥ 0.8 (simulation not too slow)

### 3. Sensor Functionality

- [ ] Camera publishes images at expected rate
- [ ] IMU data shows reasonable orientation (quaternion norm = 1)
- [ ] LIDAR scans show environment correctly
- [ ] All sensor topics appear in `ros2 topic list`

### 4. Joint Control

- [ ] Joint commands move robot as expected
- [ ] No joint limit violations (check `/joint_states`)
- [ ] Smooth motion without jerking
- [ ] Position/velocity feedback matches commands

### 5. Performance

- [ ] Gazebo runs at ≥0.8x real-time on your hardware
- [ ] CPU usage acceptable (&lt;80% on one core)
- [ ] No memory leaks during long runs

If any check fails, revisit physics tuning or model verification.

## Common Humanoid Simulation Issues

### Issue 1: Robot Falls Immediately

**Causes**:
- Feet not on ground (spawn height wrong)
- Incorrect inertia causing imbalance
- Joint limits preventing stable pose
- Time step too large

**Fixes**:
- Adjust spawn height: `-z 0.34` (foot sole to ground)
- Recalculate inertia from CAD
- Check joint limits allow standing pose
- Reduce `max_step_size` to 0.0005

### Issue 2: Jittering/Vibration

**Causes**:
- Contact stiffness too high
- Damping too low
- Solver iterations insufficient

**Fixes**:
- Reduce contact `kp` to 1e6
- Increase joint damping to 1.0-2.0
- Increase solver `iters` to 100-200

### Issue 3: Feet Sink Into Ground

**Causes**:
- Contact stiffness too low
- Max correcting velocity too low
- Time step too large

**Fixes**:
- Increase `kp` to 1e7 or higher
- Increase `contact_max_correcting_vel` to 100
- Decrease time step to 0.0005

### Issue 4: Simulation Too Slow

**Causes**:
- Too many collision meshes
- Time step too small
- Solver iterations too high
- Camera update rate too high

**Fixes**:
- Simplify collision geometry to primitives
- Increase time step to 0.001 (if stable)
- Reduce solver iterations to 50
- Reduce camera to 10 Hz for development

## Example: NAO Standing Balance

Full example: NAO with IMU-based balance controller.

**Balance controller** (simplified):

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

class SimpleBalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(Imu, '/nao/imu/data', self.imu_callback, 10)

        # Publish joint commands
        self.joint_pub = self.create_publisher(Float64MultiArray, '/nao/joint_commands', 10)

        self.current_pitch = 0.0
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def imu_callback(self, msg):
        # Extract pitch from quaternion (simplified)
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self.current_pitch = 2 * (qw * qy - qz * qx)  # Approximation for small angles

    def control_loop(self):
        # Simple PD controller for ankle pitch
        Kp = 2.0
        Kd = 0.5

        # Target pitch: upright
        target_pitch = 0.0
        error = target_pitch - self.current_pitch

        # Command ankle joints to correct pitch
        ankle_command = Kp * error  # + Kd * derivative (omitted for simplicity)

        # Publish command
        msg = Float64MultiArray()
        msg.data = [ankle_command, ankle_command]  # Both ankles
        self.joint_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleBalanceController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

This is a toy example. Real balance controllers use full-body dynamics and ZMP computation.

## Summary

Simulating humanoid robots requires careful attention to:

- **Model accuracy**: Correct inertia and geometry
- **Physics tuning**: Small time steps, high solver iterations, tuned contacts
- **Sensor integration**: Cameras, IMU, LIDAR with realistic parameters
- **Joint control**: ros2_control or direct Gazebo interfaces
- **Validation**: Systematic checks for stability, sensors, and control

Humanoid simulation is challenging but essential for safe algorithm development. Invest time in tuning—it pays off when deploying to real hardware.

## Review Questions

<details>
<summary>1. Why are humanoid robots more difficult to simulate than wheeled mobile robots?</summary>

**Answer**: Humanoids are more difficult because of:

1. **High DOF kinematic chains**: 20-50+ joints create complex dynamics and increase computational cost. Numerical errors accumulate through long kinematic chains.

2. **Complex contact dynamics**: Intermittent contacts (feet lift during walking), small contact areas, and high contact forces make collisions difficult to solve stably. Wheeled robots have continuous, predictable ground contact.

3. **Balance requirements**: Bipedal stance requires accurate center of mass estimation, precise inertia tensors, and fast control loops (200-1000 Hz). Small errors cause falling.

4. **Sensor complexity**: Humanoids need diverse sensors (cameras, LIDAR, IMU, force sensors) with coordinated frames, adding integration complexity.

These factors require smaller time steps, more solver iterations, and careful physics tuning compared to simpler robots.
</details>

<details>
<summary>2. What is the recommended physics time step for humanoid simulation, and why?</summary>

**Answer**: The recommended time step is **0.0005 seconds (0.5ms)**, which is smaller than the typical 0.001s used for wheeled robots.

**Reasoning**:
- **Fast dynamics**: Humanoid joint motions and contact events happen on millisecond timescales
- **Stability**: Smaller steps allow the solver to respond to contacts before penetration occurs
- **Balance control**: High-frequency feedback (200-1000 Hz) requires correspondingly fast physics

**Trade-off**: Smaller time steps mean slower simulation (may run at 0.5-0.8x real-time), but this is necessary for stability.

Some very complex humanoids may need 0.0001s (0.1ms) for research-grade accuracy, but 0.0005s is a good starting point.
</details>

<details>
<summary>3. A humanoid robot's feet are sinking into the ground. List three parameters to adjust and their recommended values.</summary>

**Answer**: Three key parameters to adjust:

1. **Contact stiffness (kp)**: Increase from default ~1e6 to **1e7 or 1e8**
   - Higher stiffness creates harder contacts with less penetration
   - In `<surface><contact><ode><kp>10000000.0</kp>`

2. **Physics time step (max_step_size)**: Decrease from 0.001s to **0.0005s or smaller**
   - Smaller steps allow solver to resolve contacts before excessive penetration
   - In `<physics><max_step_size>0.0005</max_step_size>`

3. **Solver iterations (iters)**: Increase from 50 to **100-200**
   - More iterations allow better convergence for complex contacts
   - In `<physics><ode><solver><iters>100</iters>`

Also verify foot collision geometry matches visual geometry and mass/inertia are realistic.
</details>

<details>
<summary>4. Explain the purpose of an IMU sensor in humanoid simulation and what data it provides.</summary>

**Answer**: The **IMU (Inertial Measurement Unit)** is critical for humanoid balance and orientation estimation.

**Purpose**:
- **Orientation estimation**: Determines which way is "up" for balance control
- **Motion detection**: Detects falling, tipping, or unexpected accelerations
- **Sensor fusion**: Combined with joint encoders for state estimation
- **Balance feedback**: Drives controllers to maintain upright stance

**Data provided** (in `sensor_msgs/Imu`):
1. **Orientation** (quaternion): Rotation relative to world frame
2. **Angular velocity** (rad/s): Rotation rates around x, y, z axes (from gyroscope)
3. **Linear acceleration** (m/s²): Acceleration along x, y, z axes (from accelerometer)

**Typical update rate**: 100-200 Hz for balance control

**Noise**: IMUs have realistic Gaussian noise (gyro: ~2e-4 rad/s, accel: ~1.7e-2 m/s²) which must be filtered in practice.
</details>

<details>
<summary>5. What is the difference between disabling self-collision globally versus using collision filters?</summary>

**Answer**:

**Disabling self-collision globally** (`<self_collide>false</self_collide>`):
- **Effect**: No collisions between ANY links of the robot
- **Pros**: Simpler, faster, good for initial stability testing
- **Cons**: Unrealistic—hand can pass through torso, limbs can interpenetrate
- **Use case**: Early development, getting robot to stand initially

**Using collision filters** (bitmasks on specific links):
- **Effect**: Selectively disable collisions between specific link pairs (e.g., adjacent links)
- **Pros**: Realistic—prevents impossible collisions while allowing valid ones (hand touches chest)
- **Cons**: More complex to configure, requires understanding collision bitmasks
- **Use case**: Production simulation, after initial tuning

**Best practice**:
1. Start with global self-collision disabled for initial physics tuning
2. Once stable, enable self-collision and add filters for adjacent links:
   ```xml
   <collision><surface><contact>
     <collide_bitmask>0x01</collide_bitmask>
   </contact></surface></collision>
   ```
3. Test grasping, reaching, walking to ensure realistic interactions

Collision filters prevent performance issues (adjacent links in constant contact) while maintaining physical realism.
</details>
