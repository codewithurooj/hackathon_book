# Section 2: World Creation

## Introduction to SDF (Simulation Description Format)

Gazebo worlds are defined using **SDF (Simulation Description Format)**, an XML-based format that describes:

- Physics properties (gravity, physics engine settings)
- Lighting (sun, point lights, spot lights)
- Models (robots, obstacles, furniture)
- Scene settings (sky, fog, ambient lighting)

Think of SDF as the "blueprint" for your simulation environment.

## Anatomy of an SDF World File

Here's a minimal world file:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>
  </world>
</sdf>
```

Let's break down each component:

### 1. SDF Version

```xml
<sdf version="1.6">
```

Specifies the SDF format version. Gazebo 11 uses SDF 1.6.

### 2. World Definition

```xml
<world name="my_world">
```

Container for the entire simulation. The `name` attribute identifies the world.

### 3. Physics Settings

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

- `type`: Physics engine (ode, bullet, dart, simbody)
- `max_step_size`: Time step for physics updates (seconds)
- `real_time_factor`: Target speed (1.0 = real-time)
- `real_time_update_rate`: Physics updates per second (Hz)

**Rule of thumb**: `real_time_update_rate = 1 / max_step_size`

### 4. Gravity

```xml
<gravity>0 0 -9.81</gravity>
```

Gravity vector in m/s² (X Y Z). Default is Earth gravity pointing down (-Z axis).

### 5. Including Pre-made Models

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

Loads a model from Gazebo's model database. Common models:
- `model://sun`: Directional light
- `model://ground_plane`: Flat ground
- `model://cafe_table`: Furniture

## Creating Your First World

Let's create a simple obstacle course for robot testing.

### Step 1: Create the File

Create `simple_obstacle_course.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_obstacle_course">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Obstacle 1: Box -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 2: Cylinder -->
    <model name="cylinder_obstacle">
      <pose>-2 0 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch the World

```bash
gazebo simple_obstacle_course.sdf
```

You should see a ground plane with a red box and blue cylinder.

## Understanding Model Components

Every model has:

### 1. Pose

```xml
<pose>X Y Z Roll Pitch Yaw</pose>
```

Position and orientation:
- **X, Y, Z**: Position in meters
- **Roll, Pitch, Yaw**: Rotation in radians

Example: `<pose>2 3 1 0 0 1.57</pose>` means:
- 2m forward (X), 3m left (Y), 1m up (Z)
- Rotated 90° (1.57 rad) around Z-axis (yaw)

### 2. Static vs Dynamic

```xml
<static>true</static>
```

- **Static (true)**: Object doesn't move, no physics calculations (walls, floors)
- **Dynamic (false)**: Object affected by physics (robots, movable objects)

Static objects are computationally cheaper.

### 3. Collision Geometry

```xml
<collision name="collision">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
</collision>
```

Defines the shape for physics collisions. Can be:
- **Box**: `<box><size>X Y Z</size></box>`
- **Sphere**: `<sphere><radius>R</radius></sphere>`
- **Cylinder**: `<cylinder><radius>R</radius><length>L</length></cylinder>`
- **Mesh**: `<mesh><uri>model://my_model/meshes/collision.stl</uri></mesh>`

**Performance tip**: Use simple collision shapes (box, sphere, cylinder) instead of complex meshes.

### 4. Visual Geometry

```xml
<visual name="visual">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
  <material>
    <ambient>1 0 0 1</ambient>
    <diffuse>1 0 0 1</diffuse>
    <specular>0.1 0.1 0.1 1</specular>
  </material>
</visual>
```

Defines the appearance. Can be different from collision geometry for performance.

**Material properties**:
- **Ambient**: Color in shadow (RGBA)
- **Diffuse**: Main color (RGBA)
- **Specular**: Shininess (RGBA)
- **Emissive**: Glow effect (RGBA)

## Advanced World Features

### Custom Lighting

Add point lights for indoor environments:

```xml
<light name="point_light" type="point">
  <pose>3 3 3 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <cast_shadows>false</cast_shadows>
</light>
```

### Scene Configuration

Control ambient lighting and fog:

```xml
<scene>
  <ambient>0.4 0.4 0.4 1</ambient>
  <background>0.7 0.7 0.7 1</background>
  <shadows>true</shadows>
  <fog>
    <color>0.5 0.5 0.5 1</color>
    <type>linear</type>
    <start>10</start>
    <end>100</end>
    <density>0.1</density>
  </fog>
</scene>
```

### Complex Models with Meshes

Load custom 3D models:

```xml
<model name="custom_object">
  <pose>0 0 0 0 0 0</pose>
  <static>false</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <mesh>
          <uri>model://my_model/meshes/collision.stl</uri>
          <scale>1 1 1</scale>
        </mesh>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://my_model/meshes/visual.dae</uri>
          <scale>1 1 1</scale>
        </mesh>
      </geometry>
    </visual>
  </link>
</model>
```

Supported mesh formats:
- **STL**: Simple, no materials (used for collisions)
- **DAE (Collada)**: Includes materials and textures
- **OBJ**: Widely supported 3D format

## Building an Indoor Environment

Let's create a simple room with furniture:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="indoor_environment">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <!-- Indoor lighting -->
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.9 0.9 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <include><uri>model://ground_plane</uri></include>

    <!-- Walls -->
    <model name="wall_north">
      <pose>0 5 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <pose>0 -5 1.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 3</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 3</size></box></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Furniture -->
    <include>
      <uri>model://cafe_table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

    <!-- Ceiling light -->
    <light name="ceiling_light" type="point">
      <pose>0 0 2.8 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <attenuation>
        <range>10</range>
        <constant>0.5</constant>
      </attenuation>
    </light>
  </world>
</sdf>
```

## World Organization Best Practices

1. **Group Related Objects**: Use comments to organize your SDF file
2. **Consistent Naming**: Use descriptive model names (`wall_north`, not `model_1`)
3. **Appropriate Static/Dynamic**: Static objects are faster
4. **Simple Collision Geometry**: Use primitive shapes when possible
5. **Lighting Strategy**: Indoor = point lights, Outdoor = sun

## Testing Your Worlds

Before using a world for robot testing:

1. **Launch and Inspect**: Verify all objects appear correctly
2. **Check Physics**: Drop a dynamic object to test gravity
3. **Test Collisions**: Spawn objects and check for physics stability
4. **Monitor Performance**: Check real-time factor (should be ≥ 0.5)

## Common Issues and Fixes

### Objects Fall Through Ground

**Problem**: Physics instability

**Solution**: Ensure ground plane is static and has proper collision geometry:

```xml
<include>
  <uri>model://ground_plane</uri>
</include>
```

### World Loads Slowly

**Problem**: Too many complex meshes or physics calculations

**Solutions**:
- Use simple collision shapes (boxes, cylinders)
- Set complex decorations to `<static>true</static>`
- Disable shadows: `<shadows>false</shadows>`
- Reduce physics update rate

### Models Not Found

**Problem**: Gazebo can't locate the model

**Solution**:
- Use Gazebo's built-in models: `model://model_name`
- Set `GAZEBO_MODEL_PATH` environment variable:
  ```bash
  export GAZEBO_MODEL_PATH=/path/to/your/models:$GAZEBO_MODEL_PATH
  ```

## What You Learned

In this section, you:

- Understood SDF format and world file structure
- Created custom worlds with physics, lighting, and objects
- Built an obstacle course for robot testing
- Created an indoor environment with walls and furniture
- Learned model components (pose, collision, visual, materials)
- Applied best practices for world organization
- Debugged common world file issues

## Next Steps

Now that you can create custom worlds, you're ready to:

- Add sensors to your robots ([Sensor Simulation](./03-sensor-simulation.md))
- Fine-tune physics properties ([Physics Properties](./05-physics-properties.md))
- Load robots into your worlds ([URDF with Sensors](./04-urdf-sensors.md))

## Exercises

1. **Simple World**: Create a world with 5 different colored boxes arranged in a pattern
2. **Obstacle Course**: Design a navigation course with 3 obstacles (box, cylinder, sphere)
3. **Indoor Environment**: Create a 10m x 10m room with 4 walls, a table, and proper lighting

Solutions are available in [Section 9: Exercises](./09-exercises.md).

## Additional Resources

- [SDF Specification](http://sdformat.org/)
- [Gazebo Model Database](https://github.com/osrf/gazebo_models)
- [Building a World Tutorial](http://gazebosim.org/tutorials?tut=build_world)
