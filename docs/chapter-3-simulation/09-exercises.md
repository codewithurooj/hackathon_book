# Hands-On Exercises

## Introduction

These exercises provide hands-on practice with Gazebo simulation concepts covered in this chapter. Each exercise builds on the previous sections and includes:

- **Learning objectives**: What you'll practice
- **Difficulty level**: Beginner, Intermediate, or Advanced
- **Estimated time**: How long to complete
- **Starter code**: Templates to get you started
- **Solution hints**: Guidance when stuck
- **Validation**: How to verify your solution works

Work through exercises sequentially for best results. Don't skip ahead—each exercise reinforces critical skills.

---

## Exercise 1: Custom Warehouse World

**Difficulty**: Beginner
**Estimated Time**: 45-60 minutes
**Learning Objectives**:
- Create SDF world files from scratch
- Add models and configure lighting
- Spawn robot in custom environment
- Visualize in Gazebo GUI

### Problem Statement

Create a warehouse simulation environment with:

- 20m × 15m floor
- 4 walls (0.2m thick, 3m high)
- 6 storage shelves (randomly positioned)
- Realistic lighting (sun + 2 spotlights)
- Ground plane with grid texture

### Starter Code

Create file: `worlds/warehouse.world`

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse">

    <!-- TODO: Add physics configuration -->

    <!-- TODO: Add lighting -->

    <!-- TODO: Add ground plane -->

    <!-- TODO: Add walls -->

    <!-- TODO: Add storage shelves -->

  </world>
</sdf>
```

### Tasks

1. **Configure physics**:
   - Time step: 0.001s
   - Real-time factor: 1.0
   - Gravity: -9.81 m/s² (Z-axis)

2. **Add lighting**:
   - Sun model (ambient light)
   - Two spotlights at positions (5, 5, 3) and (15, 10, 3)
   - Spotlights point downward

3. **Create floor**:
   - 20m × 15m rectangle at z=0
   - Static object
   - Friction coefficient: 0.8

4. **Add walls** (4 total):
   - North wall: 20m long, 0.2m thick, 3m high, at y=7.5
   - South wall: 20m long, 0.2m thick, 3m high, at y=-7.5
   - East wall: 15m long, 0.2m thick, 3m high, at x=10
   - West wall: 15m long, 0.2m thick, 3m high, at x=-10

5. **Add 6 storage shelves**:
   - Use box geometry: 1m × 2m × 2m (shelf unit)
   - Randomly position within warehouse boundaries
   - Leave 3m clear space near origin for robot spawn

### Hints

<details>
<summary>Hint 1: Physics Configuration</summary>

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <gravity>0 0 -9.81</gravity>
</physics>
```
</details>

<details>
<summary>Hint 2: Spotlight Syntax</summary>

```xml
<light name="spotlight1" type="spot">
  <pose>5 5 3 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>0 0 -1</direction>  <!-- Points down -->
  <attenuation>
    <range>20</range>
    <linear>0.1</linear>
    <constant>0.5</constant>
    <quadratic>0.01</quadratic>
  </attenuation>
  <spot>
    <inner_angle>0.6</inner_angle>
    <outer_angle>1.0</outer_angle>
    <falloff>1.0</falloff>
  </spot>
  <cast_shadows>true</cast_shadows>
</light>
```
</details>

<details>
<summary>Hint 3: Wall Example</summary>

```xml
<model name="north_wall">
  <static>true</static>
  <pose>0 7.5 1.5 0 0 0</pose>  <!-- Center at y=7.5, z=1.5 (half height) -->
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>20 0.2 3</size></box>  <!-- length, thickness, height -->
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>20 0.2 3</size></box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```
</details>

### Validation

1. Launch world:
   ```bash
   gazebo worlds/warehouse.world
   ```

2. Verify in GUI:
   - Floor is visible and level
   - All 4 walls form enclosed space
   - 6 shelves are inside warehouse
   - Lighting looks realistic (shadows visible)

3. Inspect physics:
   - Check gravity: Drop a box model, should fall
   - Check walls: Drive robot into wall, should collide

### Extension (Optional)

Add dynamic elements:
- Moving forklift (simple animated model)
- Rotating ceiling fan
- Opening/closing door

---

## Exercise 2: LIDAR-Equipped Robot

**Difficulty**: Intermediate
**Estimated Time**: 60-90 minutes
**Learning Objectives**:
- Add sensors to URDF robot models
- Configure Gazebo plugins
- Subscribe to sensor topics in ROS 2
- Visualize sensor data in RViz

### Problem Statement

Take a simple differential-drive robot URDF and add:

- 2D LIDAR sensor (360°, 10m range, 10Hz)
- RGB camera (640×480, 30Hz, 60° FOV)
- IMU sensor (100Hz)

Then write a ROS 2 node to subscribe to sensor topics and print statistics.

### Starter Code

**URDF**: `urdf/sensor_robot.urdf`

```xml
<?xml version="1.0"?>
<robot name="sensor_robot">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.15" iyy="0.25" izz="0.35" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- TODO: Add lidar_link -->

  <!-- TODO: Add camera_link -->

  <!-- TODO: Add imu_link -->

  <!-- TODO: Add Gazebo plugins for sensors -->

</robot>
```

**ROS 2 Node**: `sensor_robot_subscriber.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # TODO: Create subscriptions

        # Statistics
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

        # Timer for printing stats
        self.timer = self.create_timer(1.0, self.print_stats)

    # TODO: Add callbacks

    def print_stats(self):
        self.get_logger().info(
            f"Messages/sec - LIDAR: {self.lidar_count}, "
            f"Camera: {self.camera_count}, IMU: {self.imu_count}"
        )
        # Reset counts
        self.lidar_count = 0
        self.camera_count = 0
        self.imu_count = 0

def main():
    rclpy.init()
    node = SensorSubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Tasks

1. **Add LIDAR link to URDF**:
   - Small cylinder (radius 0.05m, height 0.02m)
   - Positioned on top-center of base_link
   - Fixed joint

2. **Add LIDAR Gazebo plugin**:
   - 360° scan (360 samples)
   - Range: 0.1m to 10m
   - Update rate: 10Hz
   - Gaussian noise: stddev 0.01m
   - Publishes to `/scan`

3. **Add camera link**:
   - Box (0.05 × 0.1 × 0.05m)
   - Positioned on front of base_link
   - Fixed joint

4. **Add camera Gazebo plugin**:
   - Resolution: 640×480
   - Horizontal FOV: 60° (1.047 radians)
   - Update rate: 30Hz
   - Publishes to `/camera/image_raw`

5. **Add IMU link**:
   - Small box inside base_link
   - Fixed joint at base_link origin

6. **Add IMU Gazebo plugin**:
   - Update rate: 100Hz
   - Angular velocity noise: stddev 2e-4 rad/s
   - Linear acceleration noise: stddev 1.7e-2 m/s²
   - Publishes to `/imu/data`

7. **Complete ROS 2 subscriber node**:
   - Subscribe to all three sensor topics
   - Increment counters in callbacks
   - Print message rates every second

### Hints

<details>
<summary>Hint 1: LIDAR Link and Joint</summary>

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.02"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.11" rpy="0 0 0"/>  <!-- On top of base -->
</joint>
```
</details>

<details>
<summary>Hint 2: LIDAR Gazebo Plugin</summary>

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar">
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
        <min>0.1</min>
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
        <namespace>/</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
</details>

<details>
<summary>Hint 3: ROS 2 Subscriber Callbacks</summary>

```python
def __init__(self):
    super().__init__('sensor_subscriber')

    self.lidar_sub = self.create_subscription(
        LaserScan, '/scan', self.lidar_callback, 10)
    self.camera_sub = self.create_subscription(
        Image, '/camera/image_raw', self.camera_callback, 10)
    self.imu_sub = self.create_subscription(
        Imu, '/imu/data', self.imu_callback, 10)

    # ... rest of init ...

def lidar_callback(self, msg):
    self.lidar_count += 1

def camera_callback(self, msg):
    self.camera_count += 1

def imu_callback(self, msg):
    self.imu_count += 1
```
</details>

### Validation

1. **Spawn robot in Gazebo**:
   ```bash
   ros2 launch chapter3_simulation sensor_robot.launch.py
   ```

2. **Check topics**:
   ```bash
   ros2 topic list
   # Should see: /scan, /camera/image_raw, /imu/data
   ```

3. **Run subscriber node**:
   ```bash
   ros2 run chapter3_simulation sensor_robot_subscriber.py
   ```

   Expected output (approximately):
   ```
   Messages/sec - LIDAR: 10, Camera: 30, IMU: 100
   ```

4. **Visualize in RViz**:
   ```bash
   rviz2
   ```
   - Add LaserScan display (topic: `/scan`)
   - Add Image display (topic: `/camera/image_raw`)
   - Set fixed frame to `base_link`

### Extension (Optional)

Add depth camera (RGB-D) using `libgazebo_ros_camera.so` with depth output.

---

## Exercise 3: Physics-Based Object Interaction

**Difficulty**: Intermediate
**Estimated Time**: 90-120 minutes
**Learning Objectives**:
- Configure realistic physics properties
- Implement grasping with contact sensors
- Tune friction and contact parameters
- Validate physics behavior

### Problem Statement

Create a gripper robot that can:

1. Detect contact with an object using touch sensors
2. Grasp a 1kg box using parallel-jaw gripper
3. Lift box to 0.5m height without dropping
4. Transport box and place at target location

Requires realistic friction, contact forces, and stable physics.

### Starter Files

**World**: `worlds/manipulation_test.world` (provided)
- Includes 1kg box at (0.5, 0, 0.05)
- Table surface

**Robot URDF**: `urdf/gripper_robot.urdf` (partial)
- Mobile base with two-finger gripper
- Gripper fingers have prismatic joints (0 to 0.08m)

### Tasks

1. **Add contact sensors** to gripper fingers:
   - Bumper plugin on each finger
   - Publishes to `/left_finger/contact` and `/right_finger/contact`

2. **Tune friction parameters**:
   - Gripper fingers: High friction (μ=1.5) for grip
   - Box: Moderate friction (μ=0.6)
   - Table: Wood-like friction (μ=0.4)

3. **Configure contact properties**:
   - High contact stiffness (kp=1e7) to prevent penetration
   - Appropriate damping (kd=10) for stable contact

4. **Write grasp controller** (`grasp_controller.py`):
   - Subscribe to contact sensors
   - When both fingers touch object, close gripper
   - Monitor contact force (if both sensors still triggered, grasp successful)
   - Command lift to 0.5m
   - Monitor for object drop (contact sensors released unexpectedly)

5. **Validate grasp stability**:
   - Box should not slip during lift
   - Box should not rotate excessively
   - Gripper force should be sufficient but not excessive

### Starter Code: Grasp Controller

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from gazebo_msgs.msg import ContactsState

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')

        # Subscriptions
        self.left_contact_sub = self.create_subscription(
            ContactsState, '/left_finger/contact', self.left_contact_callback, 10)
        self.right_contact_sub = self.create_subscription(
            ContactsState, '/right_finger/contact', self.right_contact_callback, 10)

        # Publishers
        self.gripper_pub = self.create_publisher(Float64, '/gripper/command', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift/command', 10)

        # State
        self.left_in_contact = False
        self.right_in_contact = False
        self.grasp_state = "OPEN"  # OPEN, CLOSING, GRASPED, LIFTING

        # Timer for state machine
        self.timer = self.create_timer(0.1, self.state_machine)

    def left_contact_callback(self, msg):
        self.left_in_contact = len(msg.states) > 0

    def right_contact_callback(self, msg):
        self.right_in_contact = len(msg.states) > 0

    def state_machine(self):
        # TODO: Implement grasp state machine
        pass

    def close_gripper(self):
        msg = Float64()
        msg.data = 0.04  # Close to 4cm (box is ~5cm wide)
        self.gripper_pub.publish(msg)

    def lift_gripper(self, height):
        msg = Float64()
        msg.data = height
        self.lift_pub.publish(msg)

def main():
    rclpy.init()
    node = GraspController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Hints

<details>
<summary>Hint 1: Contact Sensor Plugin</summary>

```xml
<gazebo reference="left_finger">
  <sensor name="left_contact_sensor" type="contact">
    <contact>
      <collision>left_finger_collision</collision>
    </contact>
    <update_rate>10.0</update_rate>
    <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/</namespace>
        <remapping>bumper_states:=left_finger/contact</remapping>
      </ros>
      <frame_name>left_finger</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
</details>

<details>
<summary>Hint 2: State Machine Logic</summary>

```python
def state_machine(self):
    if self.grasp_state == "OPEN":
        if self.left_in_contact and self.right_in_contact:
            self.get_logger().info("Both fingers in contact, closing gripper")
            self.close_gripper()
            self.grasp_state = "CLOSING"

    elif self.grasp_state == "CLOSING":
        # Wait 2 seconds for gripper to close
        # Then check if still in contact
        if self.left_in_contact and self.right_in_contact:
            self.get_logger().info("Grasp successful, lifting")
            self.lift_gripper(0.5)
            self.grasp_state = "GRASPED"

    elif self.grasp_state == "GRASPED":
        # Monitor for object drop
        if not (self.left_in_contact and self.right_in_contact):
            self.get_logger().error("Object dropped!")
            self.grasp_state = "FAILED"
        else:
            self.get_logger().info("Object stable")
```
</details>

### Validation

1. Launch simulation and controller:
   ```bash
   ros2 launch chapter3_simulation manipulation_test.launch.py
   ros2 run chapter3_simulation grasp_controller.py
   ```

2. **Expected behavior**:
   - Robot approaches box
   - Both contact sensors trigger
   - Gripper closes around box
   - Box lifts to 0.5m without slipping
   - Contact sensors remain triggered throughout

3. **Success criteria**:
   - Box does not slip during grasp
   - Box does not fall during lift
   - Box position at end: z > 0.45m

### Extension (Optional)

- Add force/torque sensors to estimate object weight
- Implement adaptive grasp force based on object mass
- Test with objects of varying friction (0.3 to 1.0)

---

## Exercise 4: Humanoid Balance Tuning

**Difficulty**: Advanced
**Estimated Time**: 2-3 hours
**Learning Objectives**:
- Load and configure humanoid robots
- Tune physics for bipedal stability
- Implement simple balance controller
- Debug complex physics issues

### Problem Statement

Load a NAO humanoid robot in Gazebo and tune physics so it:

1. Stands upright without falling for 30 seconds
2. Maintains balance when small external forces applied (gentle push)
3. Recovers from 5° pitch perturbation

This requires systematic physics tuning and a basic IMU-based balance controller.

### Starter Files

- **NAO URDF**: Provided in `nao_description` package
- **World**: `worlds/humanoid_test.world` (flat ground)

### Tasks

1. **Spawn NAO in Gazebo**:
   - Install `nao_robot` package or use provided URDF
   - Create launch file to spawn at height 0.34m

2. **Initial stability check**:
   - Launch Gazebo, observe if NAO stands or falls
   - If falls, proceed to tuning

3. **Verify inertial properties**:
   - Check all links have `<inertial>` tags
   - Verify masses are realistic (total ~5kg for NAO)
   - Recalculate inertia tensors for major links

4. **Tune physics parameters**:
   - Time step: Start at 0.001s, decrease to 0.0005s if unstable
   - Solver iterations: Increase to 100-200
   - Contact stiffness (feet): Increase to 1e7

5. **Add joint damping**:
   - Ankle, knee, hip joints: damping = 0.5-1.0
   - Prevent oscillations without making motion too stiff

6. **Implement balance controller**:
   - Subscribe to `/imu/data`
   - Compute pitch angle from quaternion
   - Apply PD control to ankle joints to correct pitch
   - Test with gains Kp=1.0, Kd=0.3 (tune as needed)

7. **Perturbation test**:
   - Use Gazebo GUI to apply small force to torso
   - Verify robot returns to upright stance

### Starter Code: Balance Controller

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers for ankle joints
        self.left_ankle_pub = self.create_publisher(Float64, '/nao/left_ankle/command', 10)
        self.right_ankle_pub = self.create_publisher(Float64, '/nao/right_ankle/command', 10)

        # State
        self.current_pitch = 0.0
        self.previous_pitch = 0.0

        # Control gains
        self.Kp = 1.0
        self.Kd = 0.3

        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def imu_callback(self, msg):
        # Extract pitch from quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Compute pitch (rotation around Y-axis)
        # TODO: Implement quaternion to Euler conversion

        self.current_pitch = pitch

    def control_loop(self):
        # Target pitch: upright (0 radians)
        target_pitch = 0.0
        error = target_pitch - self.current_pitch

        # Derivative (change in error)
        derivative = (self.current_pitch - self.previous_pitch) / 0.01

        # PD control law
        command = self.Kp * error - self.Kd * derivative

        # Publish to ankle joints
        msg = Float64()
        msg.data = command
        self.left_ankle_pub.publish(msg)
        self.right_ankle_pub.publish(msg)

        # Update previous pitch
        self.previous_pitch = self.current_pitch

def main():
    rclpy.init()
    node = BalanceController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### Hints

<details>
<summary>Hint 1: Quaternion to Pitch Conversion</summary>

```python
# Pitch (rotation around Y-axis)
sin_pitch = 2.0 * (qw * qy - qz * qx)
cos_pitch = 1.0 - 2.0 * (qy * qy + qx * qx)
pitch = math.atan2(sin_pitch, cos_pitch)
```
</details>

<details>
<summary>Hint 2: Physics Tuning for Humanoid</summary>

In world SDF:
```xml
<physics type="ode">
  <max_step_size>0.0005</max_step_size>
  <real_time_update_rate>2000</real_time_update_rate>
  <ode>
    <solver>
      <iters>100</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

Foot contact (in URDF):
```xml
<gazebo reference="left_foot">
  <collision>
    <surface>
      <contact>
        <ode>
          <kp>10000000.0</kp>
          <kd>10.0</kd>
        </ode>
      </contact>
      <friction>
        <ode><mu>1.0</mu></ode>
      </friction>
    </surface>
  </collision>
</gazebo>
```
</details>

### Validation

1. **Stability test**:
   - Launch Gazebo with NAO
   - NAO should stand upright without falling for 30 seconds
   - No visible jittering or oscillation

2. **Balance controller test**:
   - Launch balance controller node
   - Tilt robot ~5° (use Gazebo GUI or apply force)
   - Verify robot returns to upright within 5 seconds

3. **Metrics**:
   - Pitch angle remains within ±0.1 radians (±5.7°)
   - No falls or collisions
   - Real-time factor > 0.8 (simulation not too slow)

### Extension (Optional)

- Implement full 3D balance (pitch and roll)
- Add ZMP (Zero Moment Point) estimation
- Simulate walking (very challenging!)

---

## Exercise 5: Automated Sensor Validation

**Difficulty**: Advanced
**Estimated Time**: 2-3 hours
**Learning Objectives**:
- Design systematic test scenarios
- Automate simulation testing
- Log and analyze sensor data
- Compare simulation to ground truth

### Problem Statement

Create an automated test suite that validates LIDAR sensor accuracy by:

1. Spawning robot in known test environments
2. Recording LIDAR scans
3. Comparing measured distances to ground truth
4. Generating test report with pass/fail for each scenario

### Test Scenarios

1. **Empty room**: Detect walls at 2m, 3m, 5m distances
2. **Single obstacle**: Box at 1.5m directly ahead
3. **Multiple obstacles**: 3 boxes at varying distances and angles
4. **Occluded obstacle**: Box partially hidden behind another

### Tasks

1. **Create test worlds**:
   - 4 SDF world files (one per scenario)
   - Walls/obstacles at precisely known positions

2. **Write test harness** (`test_lidar_validation.py`):
   - Launch Gazebo with each test world
   - Wait for LIDAR data
   - Extract range at specific angles
   - Compare to expected values (±5cm tolerance)
   - Log results to CSV

3. **Generate report**:
   - Summary: X/4 tests passed
   - Detailed table: Scenario, Expected, Measured, Error, Pass/Fail

4. **Automate with launch file**:
   - Sequentially run all 4 test scenarios
   - Collect results

### Starter Code

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import math

class LidarValidator(Node):
    def __init__(self, test_name, expected_ranges):
        super().__init__('lidar_validator')
        self.test_name = test_name
        self.expected_ranges = expected_ranges  # Dict: {angle: distance}

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.scan_data = None
        self.results = []

    def scan_callback(self, msg):
        self.scan_data = msg

    def run_test(self):
        # Wait for scan data
        timeout = 5.0
        start = self.get_clock().now()
        while self.scan_data is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                raise Exception("No LIDAR data received")

        # For each expected range
        for angle_deg, expected_dist in self.expected_ranges.items():
            # Convert angle to index
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - self.scan_data.angle_min) / self.scan_data.angle_increment)

            # Get measured range
            measured_dist = self.scan_data.ranges[index]

            # Compute error
            error = abs(measured_dist - expected_dist)
            passed = error < 0.05  # 5cm tolerance

            # Store result
            result = {
                'test': self.test_name,
                'angle': angle_deg,
                'expected': expected_dist,
                'measured': measured_dist,
                'error': error,
                'pass': passed
            }
            self.results.append(result)

            self.get_logger().info(
                f"{self.test_name} @ {angle_deg}°: "
                f"Expected {expected_dist:.2f}m, Measured {measured_dist:.2f}m, "
                f"Error {error:.3f}m - {'PASS' if passed else 'FAIL'}"
            )

        return self.results

def run_all_tests():
    rclpy.init()

    # Test 1: Empty room, walls at 2m
    test1 = LidarValidator('Empty Room', {0: 2.0, 90: 2.0, 180: 2.0, 270: 2.0})
    results1 = test1.run_test()

    # Test 2: Single obstacle at 1.5m ahead
    test2 = LidarValidator('Single Obstacle', {0: 1.5})
    results2 = test2.run_test()

    # TODO: Add Test 3 and Test 4

    # Write results to CSV
    all_results = results1 + results2  # + results3 + results4
    with open('lidar_validation_results.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['test', 'angle', 'expected', 'measured', 'error', 'pass'])
        writer.writeheader()
        writer.writerows(all_results)

    # Print summary
    total = len(all_results)
    passed = sum(1 for r in all_results if r['pass'])
    print(f"\n{'='*50}")
    print(f"LIDAR Validation Summary: {passed}/{total} tests passed")
    print(f"{'='*50}\n")

    rclpy.shutdown()

if __name__ == '__main__':
    run_all_tests()
```

### Validation

1. Run test suite:
   ```bash
   python3 test_lidar_validation.py
   ```

2. Expected output:
   ```
   Empty Room @ 0°: Expected 2.00m, Measured 1.98m, Error 0.020m - PASS
   Empty Room @ 90°: Expected 2.00m, Measured 2.03m, Error 0.030m - PASS
   ...
   ==================================================
   LIDAR Validation Summary: 10/10 tests passed
   ==================================================
   ```

3. Check CSV file contains detailed results

### Extension (Optional)

- Add statistical analysis (mean error, std dev)
- Test with varying noise levels
- Compare multiple LIDAR models (different resolution, range)

---

## Submission Guidelines

For each completed exercise:

1. **Code**: Submit all modified files (URDF, SDF, Python, launch files)
2. **Documentation**: Brief README explaining your approach
3. **Validation**: Screenshot or video demonstrating working solution
4. **Reflection**: What challenges did you face? How did you debug?

## Additional Resources

- **Gazebo Tutorials**: http://gazebosim.org/tutorials
- **ROS 2 Gazebo Integration**: https://github.com/ros-simulation/gazebo_ros_pkgs
- **Example Solutions**: (Available after chapter completion)

## Summary

These exercises provide hands-on practice with:

- **Exercise 1**: World creation and environment design
- **Exercise 2**: Sensor integration and ROS 2 communication
- **Exercise 3**: Physics tuning and object manipulation
- **Exercise 4**: Humanoid simulation and balance control
- **Exercise 5**: Automated testing and validation

Complete all exercises to master Gazebo simulation for robotics development. Take your time, experiment, and don't hesitate to revisit earlier sections when stuck.

Happy simulating!
