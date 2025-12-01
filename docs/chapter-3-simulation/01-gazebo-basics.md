# Section 1: Gazebo Basics

## Introduction to Gazebo

Gazebo is an open-source 3D robotics simulator that provides accurate physics simulation, realistic sensor data, and seamless integration with ROS 2. It's the industry-standard tool for testing robot algorithms before deploying to hardware.

### What is Gazebo?

Gazebo simulates:

- **Physics**: Gravity, collisions, friction, joint dynamics
- **Sensors**: LIDAR, cameras, IMU, force/torque sensors, GPS
- **Actuators**: Joint controllers, wheel drives, grippers
- **Environments**: Indoor/outdoor scenes with lighting and materials

Think of Gazebo as a "digital twin" of your robot and its environment. What happens in Gazebo should closely mirror what would happen with real hardware.

### Gazebo vs Gazebo Classic

There are two versions of Gazebo:

- **Gazebo Classic** (also called Gazebo 11): The stable, widely-used version with excellent ROS 2 Humble support
- **Gazebo (Ignition/New Gazebo)**: The next-generation simulator with modern architecture

**For this course, we use Gazebo Classic 11** because it has better ROS 2 Humble compatibility, more tutorials, and lower hardware requirements.

## Installing Gazebo 11

### Ubuntu 22.04 Installation

```bash
# Install Gazebo 11
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev

# Install ROS 2 Gazebo integration
sudo apt-get install ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
# Should output: Gazebo multi-robot simulator, version 11.x.x
```

### Testing Your Installation

Launch Gazebo to verify it works:

```bash
gazebo
```

You should see the Gazebo window open with an empty world. If you encounter errors, check the [Troubleshooting](./10-troubleshooting.md) section.

### Hardware Requirements

**Minimum**:
- CPU: Intel i5 or AMD Ryzen 5
- RAM: 8GB
- GPU: Integrated graphics with OpenGL 3.3 support

**Recommended**:
- CPU: Intel i7 or AMD Ryzen 7
- RAM: 16GB
- GPU: Dedicated GPU (NVIDIA GeForce GTX 1050 or better)

If your hardware struggles with Gazebo, see the performance optimization section in [Troubleshooting](./10-troubleshooting.md).

## Understanding the Gazebo Interface

When you launch Gazebo, you'll see several key areas:

### 1. Scene View (Center)

The main 3D view where your simulation runs. You can:

- **Rotate**: Click and drag with middle mouse button
- **Pan**: Shift + click and drag with middle mouse button
- **Zoom**: Scroll wheel

### 2. World Panel (Left)

Shows the hierarchy of objects in your simulation:

- **Scene**: Lighting, sky, fog
- **Physics**: Gravity, physics engine settings
- **Models**: All objects in the world (robots, obstacles, etc.)
- **Lights**: Directional, point, and spot lights

### 3. Insert Panel (Left)

Allows you to add objects to the world:

- Simple shapes (box, sphere, cylinder)
- Models from online libraries
- Custom URDF/SDF models

### 4. Toolbar (Top)

Quick access to common tools:

- **Select Mode**: Click to select objects
- **Translate Mode**: Move objects
- **Rotate Mode**: Rotate objects
- **Scale Mode**: Resize objects

### 5. Time Display (Bottom)

Shows simulation time and real-time factor:

- **Real Time Factor**: 1.0 means simulation runs at real-time speed
- If RTF < 1.0, simulation is slower than real-time (common on modest hardware)
- If RTF > 1.0, simulation runs faster than real-time

## The Physics Engine

Gazebo uses physics engines to simulate realistic behavior. The default engine is **ODE (Open Dynamics Engine)**, but you can also use:

- **Bullet**: Good for robotics applications
- **DART**: Advanced dynamics with constraint solving
- **Simbody**: Biomechanical simulations

For most robotics applications, **ODE is sufficient**.

### Key Physics Concepts

1. **Gravity**: Default is -9.81 m/s² in the Z-axis (downward)
2. **Collisions**: Objects interact when they touch
3. **Friction**: Surfaces resist sliding motion
4. **Inertia**: Objects resist changes in motion based on mass distribution
5. **Damping**: Simulates energy loss over time

We'll explore these in detail in [Physics Properties](./05-physics-properties.md).

## Running Your First Simulation

### Example 1: Empty World

```bash
# Launch Gazebo with an empty world
gazebo
```

This launches a minimal world with:
- A ground plane
- A sun (directional light)
- Gravity enabled

### Example 2: Adding a Simple Object

1. Launch Gazebo
2. In the Insert panel, click "Box"
3. Click in the scene to place the box
4. Press Play (bottom toolbar)
5. Watch the box fall due to gravity

### Example 3: Pre-made Worlds

Gazebo includes several example worlds:

```bash
# List available worlds
ls /usr/share/gazebo-11/worlds/

# Launch a specific world
gazebo /usr/share/gazebo-11/worlds/willowgarage.world
```

Try these worlds:
- `shapes.world`: Various geometric shapes
- `willowgarage.world`: Office environment
- `mud.world`: Outdoor terrain

## Gazebo Architecture

Understanding how Gazebo works helps you debug issues and optimize performance.

### Components

1. **gzserver**: The physics simulation server (runs in background)
2. **gzclient**: The graphical user interface
3. **Plugins**: Extensions that add functionality (sensors, controllers)

You can run `gzserver` without `gzclient` for headless simulation (faster, useful for automated testing).

### Communication

Gazebo uses a **publish/subscribe** architecture similar to ROS:

- **Topics**: Named channels for data (e.g., `/gazebo/link_states`)
- **Services**: Request/response communication
- **Messages**: Data structures passed between components

## Integrating Gazebo with ROS 2

Gazebo and ROS 2 communicate through the `gazebo_ros` packages:

### Key Packages

- `gazebo_ros`: Core integration
- `gazebo_plugins`: Sensor and actuator plugins
- `gazebo_msgs`: ROS 2 message definitions for Gazebo

### Example: Launching Gazebo from ROS 2

```python
# launch/gazebo_example.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose'],
            output='screen'
        )
    ])
```

Launch it:

```bash
ros2 launch my_package gazebo_example.launch.py
```

## Common Gazebo Commands

### Command-Line Tools

```bash
# Launch Gazebo
gazebo

# Launch specific world
gazebo worlds/my_world.world

# Run server only (no GUI)
gzserver worlds/my_world.world

# Run client only (connect to existing server)
gzclient

# Get Gazebo version
gazebo --version

# Verbose output (helpful for debugging)
gazebo --verbose
```

### Keyboard Shortcuts

- **Space**: Pause/play simulation
- **Ctrl + R**: Reset world
- **T**: Show/hide time display
- **Ctrl + G**: Show/hide grid
- **F**: Focus camera on selected object

## World vs Model vs Link vs Joint

Understanding these terms is crucial:

- **World**: The entire simulation environment (physics, lighting, all objects)
- **Model**: A collection of links and joints (e.g., a robot)
- **Link**: A single rigid body with mass, inertia, collision, and visual properties
- **Joint**: A connection between two links (revolute, prismatic, fixed, etc.)

Example hierarchy:
```
World
├── Ground Plane (Model)
│   └── link (Link)
├── Sun (Light)
└── My Robot (Model)
    ├── base_link (Link)
    ├── wheel_left (Link)
    ├── wheel_right (Link)
    ├── wheel_left_joint (Joint)
    └── wheel_right_joint (Joint)
```

## Best Practices

1. **Start Simple**: Begin with empty worlds and add complexity gradually
2. **Check Real-Time Factor**: If RTF < 0.5, simplify your simulation
3. **Use Appropriate Physics Rates**: Default 1000 Hz is good for most cases
4. **Disable GUI for Batch Testing**: Run headless (`gzserver` only) for faster execution
5. **Save Your Work**: Export models and worlds regularly

## What You Learned

In this section, you:

- Installed Gazebo 11 and verified it works
- Explored the Gazebo interface (scene view, panels, toolbar)
- Understood the physics engine and key concepts
- Ran your first simulations with pre-made worlds
- Learned Gazebo architecture and ROS 2 integration
- Practiced common commands and shortcuts

## Next Steps

Now that you understand Gazebo basics, you're ready to:

- Create custom simulation worlds ([World Creation](./02-world-creation.md))
- Add sensors to your robots ([Sensor Simulation](./03-sensor-simulation.md))
- Configure physics properties ([Physics Properties](./05-physics-properties.md))

## Exercises

1. **Gazebo Exploration**: Launch 3 different pre-made worlds and identify the models, lights, and physics settings
2. **Object Interaction**: Add 5 boxes to an empty world, stack them, and observe collision behavior
3. **Performance Test**: Check your real-time factor with an empty world vs a complex world (like `willowgarage.world`)

Solutions are available in [Section 9: Exercises](./09-exercises.md).

## Additional Resources

- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [Gazebo API Documentation](http://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/index.html)
- [ROS 2 + Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)
