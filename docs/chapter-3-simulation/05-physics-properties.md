# Physics Properties

## Introduction

Physics simulation is the heart of Gazebo. While visual appearance matters for human observers, accurate physics is critical for developing algorithms that transfer to real robots. In this section, you'll learn how to configure and tune physics properties to create realistic simulations that closely match real-world behavior.

Understanding physics properties is essential for:

- **Realistic motion**: Ensuring robots move naturally with proper dynamics
- **Collision detection**: Preventing interpenetration and handling contacts correctly
- **Stability**: Avoiding simulation explosions and jitter
- **Performance**: Balancing accuracy with computation speed
- **Sim-to-real transfer**: Minimizing the gap between simulation and reality

## Physics Engines in Gazebo

Gazebo Classic supports multiple physics engines, each with different characteristics:

### ODE (Open Dynamics Engine)

**Default engine** in Gazebo Classic. Good general-purpose choice.

**Pros:**
- Fast for simple scenarios
- Well-tested and stable
- Good documentation
- Supports friction pyramids

**Cons:**
- Can be unstable with complex contact scenarios
- Limited constraint solver options
- May require careful tuning for humanoids

### Bullet

Alternative physics engine with different solver characteristics.

**Pros:**
- More stable contacts in some scenarios
- Better multithreading support
- Advanced constraint solvers

**Cons:**
- Slower than ODE for simple cases
- Less commonly used (fewer community examples)

### Simbody

High-fidelity physics engine designed for biomechanics.

**Pros:**
- Very accurate for complex mechanisms
- Excellent for humanoid robots
- Sophisticated contact modeling

**Cons:**
- Significantly slower than ODE
- Requires more computational resources
- Steeper learning curve

### Choosing a Physics Engine

For this course, we use **ODE** as the default. It provides a good balance of speed and accuracy for learning. For production humanoid simulations, consider Simbody or Bullet if you encounter stability issues.

## Configuring the Physics Engine

Physics properties are set in the world SDF file:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="physics_demo">

    <!-- Physics engine configuration -->
    <physics type="ode">
      <!-- Time step: smaller = more accurate but slower -->
      <max_step_size>0.001</max_step_size>

      <!-- Real-time factor: 1.0 = real-time, >1 = faster, &lt;1 = slower -->
      <real_time_factor>1.0</real_time_factor>

      <!-- Update rate: how often to step physics (Hz) -->
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- ODE-specific settings -->
      <ode>
        <solver>
          <type>quick</type>  <!-- quick or world -->
          <iters>50</iters>   <!-- solver iterations -->
          <sor>1.3</sor>      <!-- successive over-relaxation -->
        </solver>

        <constraints>
          <cfm>0.0</cfm>      <!-- constraint force mixing -->
          <erp>0.2</erp>      <!-- error reduction parameter -->
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Gravity vector (m/s^2) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Include ground and lighting -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

  </world>
</sdf>
```

### Key Parameters Explained

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `max_step_size` | Physics time step (seconds) | 0.001 (1ms) |
| `real_time_factor` | Speed multiplier | 1.0 (real-time) |
| `real_time_update_rate` | Physics updates per second | 1000 Hz |
| `iters` | Solver iterations per step | 50-200 |
| `cfm` | Constraint force mixing (softness) | 0.0 (rigid) |
| `erp` | Error reduction (correction speed) | 0.2 |

### Time Step Considerations

The `max_step_size` is **critical** for simulation stability:

- **Too large** (>0.001): Fast but may miss collisions, unstable contacts
- **Too small** (&lt;0.0001): Accurate but very slow
- **Recommended**: 0.001s (1ms) for most robots, 0.0005s for complex humanoids

**Rule of thumb**: Time step should be **< 1/10** of the smallest time constant in your system.

## Gravity Configuration

Gravity affects all objects with mass. You can modify gravity for special scenarios:

```xml
<!-- Earth gravity (default) -->
<gravity>0 0 -9.81</gravity>

<!-- Moon gravity (1/6th Earth) -->
<gravity>0 0 -1.62</gravity>

<!-- Zero gravity (space) -->
<gravity>0 0 0</gravity>

<!-- Custom direction (e.g., tilted surface) -->
<gravity>1.0 0 -9.81</gravity>
```

## Material Properties: Friction and Restitution

Materials define how surfaces interact during contact. These properties are set in model SDF files:

```xml
<model name="box">
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>

      <!-- Surface properties -->
      <surface>
        <friction>
          <ode>
            <!-- Friction coefficients (0 = frictionless, 1 = high friction) -->
            <mu>1.0</mu>    <!-- Primary friction direction -->
            <mu2>1.0</mu2>  <!-- Secondary friction direction -->

            <!-- Friction direction (optional, for anisotropic friction) -->
            <fdir1>0 0 0</fdir1>

            <!-- Slip parameters -->
            <slip1>0.0</slip1>
            <slip2>0.0</slip2>
          </ode>
        </friction>

        <contact>
          <ode>
            <!-- Bounciness (0 = no bounce, 1 = perfect bounce) -->
            <kp>1000000.0</kp>  <!-- Contact stiffness -->
            <kd>1.0</kd>         <!-- Contact damping -->
            <max_vel>0.01</max_vel>
            <min_depth>0.0</min_depth>
          </ode>
        </contact>

        <bounce>
          <restitution_coefficient>0.0</restitution_coefficient>
          <threshold>0.0</threshold>
        </bounce>
      </surface>
    </collision>

    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

### Friction Coefficients

Friction determines how much surfaces resist sliding:

| Material Pair | μ (mu) Value | Description |
|---------------|--------------|-------------|
| Metal on metal | 0.15-0.25 | Low friction |
| Rubber on concrete | 0.6-0.85 | Medium-high friction |
| Rubber on rubber | 1.0-1.5 | High friction |
| Ice on ice | 0.02-0.05 | Very low friction |
| Wood on wood | 0.25-0.50 | Medium friction |

**For robot wheels on floor**: Use μ ≈ 0.8 for realistic traction.

### Restitution (Bounciness)

Restitution coefficient determines energy retention in collisions:

- **0.0**: Perfectly inelastic (no bounce)
- **0.5**: Moderate bounce (typical for many objects)
- **1.0**: Perfectly elastic (ball bounces to same height)

**For robots**: Typically use 0.0-0.1 to avoid unwanted bouncing.

### Contact Parameters

- **kp (stiffness)**: Higher = harder contact, less penetration. Typical: 1e6 to 1e9
- **kd (damping)**: Higher = less bouncy. Typical: 1.0 to 100.0
- **max_vel**: Maximum correction velocity to prevent explosions
- **min_depth**: Minimum penetration before contact force is applied

## Inertial Properties

Accurate inertia is crucial for realistic motion. Each link needs mass and inertia tensor:

```xml
<link name="body">
  <inertial>
    <!-- Mass in kilograms -->
    <mass>10.0</mass>

    <!-- Center of mass offset from link origin -->
    <pose>0 0 0.05 0 0 0</pose>

    <!-- Inertia tensor (kg*m^2) -->
    <inertia>
      <ixx>0.083</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.083</iyy>
      <iyz>0.0</iyz>
      <izz>0.083</izz>
    </inertia>
  </inertial>

  <!-- Collision and visual elements -->
  <collision name="collision">
    <geometry>
      <box><size>0.5 0.5 0.5</size></box>
    </geometry>
  </collision>
</link>
```

### Calculating Inertia

For simple shapes, use these formulas:

**Box (dimensions: x, y, z)**
```
Ixx = (1/12) * m * (y² + z²)
Iyy = (1/12) * m * (x² + z²)
Izz = (1/12) * m * (x² + y²)
```

**Cylinder (radius: r, height: h, axis along z)**
```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

**Sphere (radius: r)**
```
Ixx = Iyy = Izz = (2/5) * m * r²
```

**For complex shapes**: Use CAD software (SolidWorks, Fusion 360) or MeshLab to compute inertia from mesh files.

### Common Inertia Mistakes

1. **Zero inertia**: Causes simulation crashes
2. **Negative values**: Physically impossible, causes instability
3. **Asymmetric tensor**: Non-zero off-diagonal terms without rotation
4. **Wrong units**: Inertia is in kg·m², not kg·cm²
5. **Mismatch with geometry**: Visual and collision geometry should match inertial properties

## Collision Properties

Collisions are detected between `<collision>` elements. Optimize collision meshes for performance:

```xml
<collision name="collision">
  <geometry>
    <!-- Simple primitives (fastest) -->
    <box><size>1 1 1</size></box>

    <!-- Or cylinder -->
    <!-- <cylinder><radius>0.5</radius><length>1.0</length></cylinder> -->

    <!-- Or sphere -->
    <!-- <sphere><radius>0.5</radius></sphere> -->

    <!-- Or mesh (slowest, use sparingly) -->
    <!-- <mesh><uri>model://my_model/meshes/collision.stl</uri></mesh> -->
  </geometry>

  <!-- Optional: collision-specific surface properties -->
  <surface>
    <friction><ode><mu>0.8</mu></ode></friction>
  </surface>
</collision>
```

### Best Practices for Collisions

1. **Use primitives when possible**: Boxes, cylinders, spheres are 10-100x faster than meshes
2. **Simplify collision meshes**: Reduce triangle count drastically (100-1000 triangles max)
3. **Combine primitives**: Use multiple simple shapes instead of one complex mesh
4. **Disable unnecessary collisions**: Use collision filters for links that never touch
5. **Scale appropriately**: Match collision size to visual for realistic interactions

### Collision Filtering

Disable collisions between specific links (e.g., adjacent robot links):

```xml
<gazebo>
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
    <ros>
      <namespace>/demo</namespace>
    </ros>
  </plugin>

  <!-- Disable self-collisions between adjacent links -->
  <self_collide>false</self_collide>
</gazebo>
```

In URDF, specify adjacent links don't collide:

```xml
<robot name="my_robot">
  <!-- ... links and joints ... -->

  <!-- Disable collision checking between adjacent links -->
  <gazebo reference="link1">
    <collision name="link1_collision">
      <surface>
        <contact>
          <collide_bitmask>0x01</collide_bitmask>
        </contact>
      </surface>
    </collision>
  </gazebo>

  <gazebo reference="link2">
    <collision name="link2_collision">
      <surface>
        <contact>
          <collide_bitmask>0x02</collide_bitmask>
        </contact>
      </surface>
    </collision>
  </gazebo>
</robot>
```

## Damping and Friction

Joint damping and friction affect motion smoothness:

```xml
<joint name="joint1" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>10.0</effort>
      <velocity>1.0</velocity>
    </limit>
    <dynamics>
      <!-- Damping: resists velocity (Nm/(rad/s)) -->
      <damping>0.7</damping>

      <!-- Friction: constant resistance (Nm) -->
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

### Tuning Damping

- **Low damping (0.0-0.5)**: Free-moving joints, oscillations possible
- **Medium damping (0.5-2.0)**: Typical for robot joints
- **High damping (2.0+)**: Slow, heavily damped motion

**Humanoid joints**: Start with damping = 0.5-1.0, adjust based on observed motion.

## Physics Debugging Techniques

### 1. Visualize Contacts

Enable contact visualization in Gazebo GUI:

```
View → Contacts (checkbox)
```

Red spheres appear at contact points.

### 2. Check for Penetration

If objects sink into each other:
- Increase contact stiffness (`kp`)
- Decrease time step
- Simplify collision geometry
- Check mass/inertia values

### 3. Monitor Real-Time Factor

```bash
# In terminal while Gazebo is running
gz stats
```

Output:
```
Factor[1.00] SimTime[10.50] RealTime[10.52] Paused[F]
```

**Factor < 1.0**: Simulation running slower than real-time (increase time step or simplify scene)
**Factor > 1.0**: Simulation running faster than real-time (good for batch testing)

### 4. Log Physics Data

Use Gazebo's logging to record and replay simulations:

```bash
# Start recording
gazebo worlds/my_world.world --record

# Replay (opens Gazebo)
gazebo --playback /path/to/log/file
```

### 5. Disable GUI for Faster Testing

Run headless for automated tests:

```bash
# Launch without GUI
gzserver worlds/my_world.world
```

Use RViz for visualization instead.

## Example: Tuning a Mobile Robot

Let's tune physics for a differential-drive robot:

```xml
<sdf version="1.6">
  <model name="mobile_robot">

    <!-- Base link -->
    <link name="base_link">
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.3</ixx><iyy>0.3</iyy><izz>0.5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>0.6 0.4 0.2</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- Low friction for smooth sliding -->
              <mu>0.01</mu>
              <mu2>0.01</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>0.6 0.4 0.2</size></box>
        </geometry>
      </visual>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.01</ixx><iyy>0.01</iyy><izz>0.02</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder><radius>0.15</radius><length>0.05</length></cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <!-- High friction for traction -->
              <mu>1.0</mu>
              <mu2>1.0</mu2>

              <!-- Anisotropic friction: low slip perpendicular to rolling -->
              <fdir1>0 1 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.1</slip2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder><radius>0.15</radius><length>0.05</length></cylinder>
        </geometry>
      </visual>
    </link>

    <!-- Joint with damping -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.01</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Right wheel (similar to left) -->
    <!-- ... -->

  </model>
</sdf>
```

### Tuning Process

1. **Set realistic masses**: Weigh real robot or estimate from CAD
2. **Start with default friction**: μ = 0.8 for wheels, 0.01 for chassis
3. **Test motion**: Drive robot in simulation
4. **Adjust friction**: If slipping, increase μ; if too sticky, decrease
5. **Tune damping**: Reduce oscillations without making motion too slow
6. **Validate**: Compare simulated and real robot speeds/accelerations

## Performance Optimization

Large simulations can slow down. Optimize with these strategies:

### 1. Reduce Physics Rate

```xml
<physics type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Increase from 0.001 -->
  <real_time_update_rate>500</real_time_update_rate>  <!-- Decrease from 1000 -->
</physics>
```

**Trade-off**: Less accurate, but faster.

### 2. Simplify Collision Geometry

Replace complex meshes with primitive approximations.

### 3. Disable Shadows

```xml
<model name="my_model">
  <static>true</static>  <!-- Static objects are cheaper -->
  <link name="link">
    <visual name="visual">
      <cast_shadows>false</cast_shadows>  <!-- Disable shadows -->
      <!-- ... -->
    </visual>
  </link>
</model>
```

### 4. Use Static Models

Objects that don't move should be marked `<static>true</static>`.

### 5. Limit Sensor Update Rates

```xml
<sensor name="camera" type="camera">
  <update_rate>10.0</update_rate>  <!-- 10 Hz instead of 30 Hz -->
  <!-- ... -->
</sensor>
```

### 6. Run Headless

Use `gzserver` without GUI for batch tests.

## Summary

In this section, you learned:

- **Physics engines**: ODE, Bullet, Simbody and when to use each
- **Time stepping**: Choosing appropriate `max_step_size` for stability vs. speed
- **Material properties**: Friction, restitution, and contact parameters
- **Inertial properties**: Mass, center of mass, and inertia tensors
- **Collision optimization**: Using primitives and filtering
- **Debugging**: Visualization tools and logging
- **Performance tuning**: Strategies to speed up large simulations

Accurate physics is the foundation of useful simulation. Take time to tune these parameters—it pays off when transferring to real hardware.

## Review Questions

<details>
<summary>1. What is the recommended physics time step for most robots, and why?</summary>

**Answer**: The recommended time step is **0.001 seconds (1ms)**. This provides a good balance between:
- **Accuracy**: Small enough to capture fast dynamics and avoid missing collisions
- **Speed**: Large enough to run at real-time or faster on typical hardware
- **Stability**: Prevents numerical instability in the solver

For complex humanoid robots, you may need to decrease to 0.0005s (0.5ms) for better stability. The rule of thumb is the time step should be less than 1/10 of the smallest time constant in your system.
</details>

<details>
<summary>2. Explain the difference between friction coefficient (mu) and restitution coefficient.</summary>

**Answer**:
- **Friction coefficient (μ)**: Determines resistance to sliding between surfaces. Higher values (0.8-1.5) mean more grip, lower values (0.01-0.2) mean slippery surfaces. It affects how much force is needed to slide one object across another.

- **Restitution coefficient**: Determines bounciness during collisions. A value of 0.0 means perfectly inelastic (no bounce), while 1.0 means perfectly elastic (ball bounces back to original height). It controls energy retention in impacts.

For robots, you typically want high friction (μ ≈ 0.8) for wheels to grip the floor, and low restitution (≈ 0.0-0.1) to avoid unwanted bouncing.
</details>

<details>
<summary>3. How do you calculate the inertia tensor for a box with dimensions 0.5m × 0.3m × 0.2m and mass 10kg?</summary>

**Answer**: For a box with dimensions x=0.5, y=0.3, z=0.2 and mass m=10:

```
Ixx = (1/12) * m * (y² + z²) = (1/12) * 10 * (0.3² + 0.2²) = 0.108 kg·m²
Iyy = (1/12) * m * (x² + z²) = (1/12) * 10 * (0.5² + 0.2²) = 0.242 kg·m²
Izz = (1/12) * m * (x² + y²) = (1/12) * 10 * (0.5² + 0.3²) = 0.283 kg·m²
Ixy = Ixz = Iyz = 0 (for a box aligned with principal axes)
```

In SDF format:
```xml
<inertia>
  <ixx>0.108</ixx><iyy>0.242</iyy><izz>0.283</izz>
  <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
</inertia>
```
</details>

<details>
<summary>4. What are three strategies to improve simulation performance without significantly reducing accuracy?</summary>

**Answer**: Three effective strategies:

1. **Simplify collision geometry**: Replace complex meshes with primitive shapes (boxes, cylinders, spheres) which are 10-100x faster. The visual mesh can still be detailed; only collision geometry needs simplification.

2. **Mark static objects as static**: Objects that never move (walls, floor, buildings) should have `<static>true</static>` to skip unnecessary physics calculations.

3. **Reduce sensor update rates**: Cameras and LIDAR sensors don't need to run at maximum frame rates. Reducing from 30 Hz to 10 Hz can significantly improve performance with minimal impact on algorithm development.

Other good strategies: disable shadows, run headless (gzserver only), limit solver iterations for distant objects.
</details>

<details>
<summary>5. A robot is sinking into the ground in simulation. What are three potential causes and fixes?</summary>

**Answer**: Three potential causes and their fixes:

1. **Time step too large**: The physics engine can't respond fast enough to prevent penetration.
   - **Fix**: Reduce `max_step_size` from 0.001 to 0.0005 or smaller

2. **Contact stiffness too low**: Contacts are too soft, allowing excessive penetration.
   - **Fix**: Increase `kp` (contact stiffness) in surface properties from default to 1e7 or 1e8

3. **Incorrect mass/inertia**: Object is too heavy for its collision geometry or inertia values are wrong.
   - **Fix**: Verify mass is realistic, recalculate inertia tensor using proper formulas or CAD software, ensure inertia matches actual geometry

Other possible causes: solver iterations too low (increase `iters`), collision geometry too complex (simplify to primitives), center of mass offset incorrectly.
</details>
