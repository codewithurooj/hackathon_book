# Unity Overview for Robotics

## Introduction

While Gazebo is the primary simulation tool for robotics development, **Unity** offers unique capabilities for specific use cases, particularly high-fidelity visualization and human-robot interaction (HRI) research. This section introduces Unity in the context of robotics, explains when to use it versus Gazebo, and demonstrates ROS integration.

Unity is a professional game engine used by millions of developers. Its strengths in graphics, UI, and cross-platform deployment make it valuable for:

- **Photorealistic rendering** for presentations and marketing
- **VR/AR applications** for robot teleoperation and training
- **Human-robot interaction studies** with realistic human models
- **User interfaces** for robot control and monitoring
- **Digital twin visualization** for stakeholders

## Gazebo vs Unity: When to Use Each

### Use Gazebo When:

✅ **Developing navigation algorithms** - Gazebo's LIDAR and sensor plugins are optimized for robotics
✅ **Testing manipulation** - Physics accuracy is critical
✅ **Multi-robot systems** - Gazebo handles swarms efficiently
✅ **Sensor development** - Built-in noise models and ROS integration
✅ **CI/CD testing** - Headless operation for automated tests
✅ **Hardware-in-the-loop** - Direct ROS 2 integration

### Use Unity When:

✅ **High-fidelity visualization** needed for demos or publications
✅ **VR/AR interfaces** for robot control or training
✅ **HRI research** with realistic human avatars
✅ **Cross-platform deployment** (web, mobile, desktop)
✅ **Marketing materials** requiring AAA graphics
✅ **User studies** where visual realism affects results

### Comparison Table

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Physics accuracy** | Excellent (ODE, Bullet, Simbody) | Good (PhysX, limited tuning) |
| **Graphics quality** | Basic | Photorealistic (AAA games) |
| **ROS integration** | Native (ros_gz) | Plugin required (ROS-TCP-Connector) |
| **Sensor simulation** | Extensive, accurate | Limited, requires custom work |
| **Learning curve** | Moderate (roboticists) | Steep (game dev background helps) |
| **Headless mode** | Yes (gzserver) | Possible but complex |
| **VR/AR support** | No | Excellent (native) |
| **Cost** | Free, open-source | Free (Personal), paid (Pro) |
| **Community** | Robotics-focused | Game dev, general |
| **Performance** | Good for robots | Optimized for graphics |
| **Asset ecosystem** | Limited | Massive (Unity Asset Store) |

### Decision Framework

```
Need accurate physics and sensor data?
  ├─ YES → Use Gazebo
  └─ NO → Continue

Need photorealistic graphics or VR/AR?
  ├─ YES → Use Unity
  └─ NO → Use Gazebo

Doing HRI with realistic humans?
  ├─ YES → Use Unity
  └─ NO → Use Gazebo

Building for web/mobile deployment?
  ├─ YES → Use Unity
  └─ NO → Use Gazebo
```

**Hybrid Approach**: Some teams use Gazebo for algorithm development and Unity for visualization/HRI, running them simultaneously and bridging data via ROS topics.

## Unity Basics for Roboticists

### Installation

1. **Download Unity Hub**: https://unity.com/download
2. **Install Unity Editor**: LTS version recommended (e.g., 2022.3 LTS)
3. **Add Linux Build Support** (if deploying to Linux robots)

```bash
# On Ubuntu, download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

### Key Concepts

#### GameObjects and Components

Unity uses an entity-component architecture:

- **GameObject**: Basic object in scene (similar to Gazebo model)
- **Component**: Adds functionality (Transform, Mesh Renderer, Rigidbody)
- **Transform**: Position, rotation, scale
- **Rigidbody**: Adds physics simulation

Example hierarchy:
```
Robot (GameObject)
├─ Base (GameObject)
│  ├─ Transform (Component)
│  ├─ Mesh Renderer (Component)
│  └─ Box Collider (Component)
├─ Wheel_Left (GameObject)
│  ├─ Transform
│  ├─ Mesh Renderer
│  ├─ Wheel Collider
│  └─ Rigidbody
└─ Camera (GameObject)
   ├─ Transform
   └─ Camera (Component)
```

#### Scripting in C#

Unity uses C# for scripting (not Python like ROS):

```csharp
using UnityEngine;

public class WheelController : MonoBehaviour
{
    public float motorTorque = 100f;
    private WheelCollider wheelCollider;

    void Start()
    {
        wheelCollider = GetComponent<WheelCollider>();
    }

    void FixedUpdate()
    {
        // Apply motor torque (called every physics step)
        wheelCollider.motorTorque = motorTorque;
    }
}
```

#### Physics in Unity

Unity uses **NVIDIA PhysX** engine:

- **Rigidbody**: Adds mass, gravity, physics simulation
- **Colliders**: Define collision shapes (Box, Sphere, Mesh)
- **Joints**: Connect rigidbodies (Hinge, Fixed, Configurable)
- **Materials**: Define friction and bounciness (Physics Material)

**Limitations compared to Gazebo**:
- Fewer tuning options for contact resolution
- Less accurate for complex contact scenarios
- Fixed time step (usually 0.02s, vs. Gazebo's 0.001s)

## ROS Integration with Unity

Unity doesn't natively support ROS. Integration is achieved via **ROS-TCP-Connector**, developed by Unity Technologies.

### Architecture

```
[Unity Application]
       ↕ TCP/IP
[ROS-TCP-Endpoint (Python node)]
       ↕ ROS 2 DDS
[Other ROS 2 Nodes]
```

Unity communicates with a Python ROS 2 node via TCP, which translates messages to/from ROS topics.

### Setting Up ROS-TCP-Connector

#### 1. Install Unity Packages

In Unity Editor:

1. Open **Window → Package Manager**
2. Click **+ → Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Install **Robotics Visualizations** package similarly:
   `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations`

#### 2. Install ROS 2 Endpoint

```bash
# Create ROS 2 workspace
mkdir -p ~/unity_ros2_ws/src
cd ~/unity_ros2_ws/src

# Clone ROS-TCP-Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/unity_ros2_ws
colcon build
source install/setup.bash
```

#### 3. Configure Unity

In Unity, go to **Robotics → ROS Settings**:

- **ROS IP Address**: IP of machine running ROS 2 (e.g., `127.0.0.1` for localhost)
- **ROS Port**: Default `10000`
- **Protocol**: ROS 2

#### 4. Start ROS-TCP-Endpoint

```bash
source ~/unity_ros2_ws/install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Output:
```
Starting server on 0.0.0.0:10000
```

### Publishing from Unity to ROS 2

Example: Publish robot position to `/unity/robot_pose`

**Unity C# Script**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotPosePublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/unity/robot_pose";
    public float publishRate = 10f; // Hz

    private float timeSinceLastPublish = 0f;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        timeSinceLastPublish += Time.deltaTime;

        if (timeSinceLastPublish >= 1f / publishRate)
        {
            PublishPose();
            timeSinceLastPublish = 0f;
        }
    }

    void PublishPose()
    {
        // Create ROS message
        PoseStampedMsg msg = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "unity_world"
            },
            pose = new PoseMsg
            {
                position = new PointMsg
                {
                    x = transform.position.z,  // Unity Z = ROS X (coordinate conversion)
                    y = -transform.position.x, // Unity -X = ROS Y
                    z = transform.position.y   // Unity Y = ROS Z
                },
                orientation = new QuaternionMsg
                {
                    x = transform.rotation.z,
                    y = -transform.rotation.x,
                    z = transform.rotation.y,
                    w = transform.rotation.w
                }
            }
        };

        // Publish to ROS
        ros.Publish(topicName, msg);
    }
}
```

**Coordinate Conversion**: Unity uses left-handed Y-up, ROS uses right-handed Z-up. Always convert!

### Subscribing from ROS 2 to Unity

Example: Control robot from ROS 2 `/cmd_vel` topic

**Unity C# Script**:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocitySubscriber : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/cmd_vel";

    public float linear_x = 0f;
    public float angular_z = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ReceiveVelocity);
    }

    void ReceiveVelocity(TwistMsg msg)
    {
        linear_x = (float)msg.linear.x;
        angular_z = (float)msg.angular.z;

        // Apply to robot (in another component)
        Debug.Log($"Received cmd_vel: linear={linear_x}, angular={angular_z}");
    }

    void FixedUpdate()
    {
        // Apply velocities to rigidbody
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.velocity = transform.forward * linear_x;
            rb.angularVelocity = transform.up * angular_z;
        }
    }
}
```

### Visualizing ROS Topics in Unity

Unity Robotics Visualizations can display:

- **Point clouds** (sensor_msgs/PointCloud2)
- **Laser scans** (sensor_msgs/LaserScan)
- **Odometry** (nav_msgs/Odometry)
- **Transforms** (tf2_msgs/TFMessage)
- **Markers** (visualization_msgs/Marker)

Enable in Unity: **Robotics → Visualization Settings**

## Use Case: HRI Visualization

Unity excels at human-robot interaction scenarios with realistic humans.

### Example: Robot Approaching a Human

**Scenario**: Visualize a service robot navigating to a human in a living room.

**Steps**:

1. **Import human model**: Use Unity Asset Store (free characters available)
2. **Add robot model**: Import URDF or create from primitives
3. **Subscribe to robot pose**: Get position from ROS 2 navigation stack
4. **Animate human**: Use Unity's Animator for realistic motions
5. **Add UI overlay**: Display robot status, distance to human

**Benefits**:
- Realistic lighting and materials for video demos
- Animate human reactions (waving, pointing, walking)
- VR mode: Researchers can "be" the human in the scene
- Export video for publications

### Human Avatar Animation

```csharp
using UnityEngine;

public class HumanController : MonoBehaviour
{
    private Animator animator;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Detect robot proximity (simplified)
        GameObject robot = GameObject.Find("Robot");
        float distance = Vector3.Distance(transform.position, robot.transform.position);

        if (distance < 2.0f)
        {
            // Robot nearby: wave animation
            animator.SetBool("IsWaving", true);
        }
        else
        {
            // Robot far: idle animation
            animator.SetBool("IsWaving", false);
        }
    }
}
```

## Unity Sensors vs Gazebo Sensors

Unity's sensor simulation is less developed than Gazebo's. Comparison:

| Sensor Type | Gazebo | Unity |
|-------------|--------|-------|
| **Camera (RGB)** | Excellent, realistic noise | Excellent, photorealistic |
| **Depth Camera** | Good (GPU-based) | Good (native depth buffer) |
| **LIDAR** | Excellent, realistic | Manual raycasting required |
| **IMU** | Built-in plugins | Manual implementation |
| **GPS** | Built-in plugins | Manual implementation |

### Implementing LIDAR in Unity

Unity doesn't have built-in LIDAR. Implement with raycasting:

```csharp
using UnityEngine;

public class LidarSensor : MonoBehaviour
{
    public int numRays = 360;
    public float maxRange = 10f;
    public float scanRate = 10f; // Hz

    private float[] ranges;

    void Start()
    {
        ranges = new float[numRays];
        InvokeRepeating("PerformScan", 0f, 1f / scanRate);
    }

    void PerformScan()
    {
        for (int i = 0; i < numRays; i++)
        {
            float angle = (360f / numRays) * i;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxRange))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxRange;
            }
        }

        // Publish to ROS (via ROS-TCP-Connector)
        PublishLaserScan();
    }

    void PublishLaserScan()
    {
        // Convert to sensor_msgs/LaserScan and publish
        // (Implementation omitted for brevity)
    }
}
```

**Performance**: Raycasting is expensive. Limit to 360 rays at 10 Hz for real-time.

## Limitations of Unity for Robotics

While powerful, Unity has drawbacks:

1. **Physics accuracy**: PhysX is optimized for games, not robotics precision
2. **Time step**: Fixed at 0.02s (50 Hz) by default, vs. Gazebo's 0.001s (1000 Hz)
3. **Contact handling**: Less tunable than Gazebo's ODE/Bullet
4. **Sensor ecosystem**: No built-in LIDAR, IMU, GPS plugins
5. **ROS integration overhead**: TCP adds latency vs. Gazebo's native ros_gz
6. **Closed-source**: Can't debug engine internals
7. **Licensing**: Free for personal, but Pro license ($150/month) for companies >$200k revenue

**Recommendation**: Use Unity for visualization and HRI, but validate algorithms in Gazebo first.

## Best Practices for Unity in Robotics

1. **Validate physics in Gazebo first**: Don't trust Unity for critical physics
2. **Use Unity for its strengths**: Graphics, VR, UI, not primary simulation
3. **Minimize latency**: ROS-TCP-Connector adds ~5-10ms; keep loops simple
4. **Test on target platform early**: Unity performance varies (Windows, Linux, mobile)
5. **Version control scenes**: Use Unity's YAML scene serialization
6. **Optimize for real-time**: Limit raycasting, reduce draw calls, profile frequently

## Example Workflow: Gazebo + Unity

Professional workflow for a humanoid service robot:

### Development Phase (Gazebo)

1. Develop navigation in Gazebo with accurate LIDAR
2. Test manipulation with realistic physics
3. Tune PID controllers and validate sensor fusion
4. Run automated test suite in headless Gazebo

### Visualization Phase (Unity)

5. Import robot model to Unity
6. Subscribe to `/tf`, `/cmd_vel`, `/joint_states` from Gazebo
7. Add photorealistic environment (living room with humans)
8. Record video for presentations
9. Build VR demo for stakeholders

### Deployment Phase

10. Deploy code from Gazebo to real robot (minimal changes)
11. Use Unity for remote monitoring visualization

**Key insight**: Gazebo for development, Unity for presentation.

## Resources for Learning Unity

- **Unity Learn**: https://learn.unity.com/ (official tutorials)
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **ROS-TCP-Connector Docs**: https://github.com/Unity-Technologies/ROS-TCP-Connector
- **Unity for Robotics Course** (Unity Learn): Free, covers ROS integration
- **Asset Store**: https://assetstore.unity.com/ (environments, characters, props)

## Summary

Unity is a powerful complementary tool to Gazebo for robotics visualization:

- **Use Gazebo for**: Algorithm development, sensor testing, physics accuracy
- **Use Unity for**: Photorealistic rendering, VR/AR, HRI studies, marketing
- **ROS integration**: Via ROS-TCP-Connector (TCP bridge to ROS 2)
- **Limitations**: Less accurate physics, requires manual sensor implementation
- **Best practice**: Develop in Gazebo, visualize in Unity

For this course, we focus on Gazebo as the primary simulation tool. Unity is optional for students interested in visualization or HRI applications.

## Review Questions

<details>
<summary>1. What are three scenarios where Unity is a better choice than Gazebo?</summary>

**Answer**: Unity is better for:

1. **VR/AR applications**: Unity has native support for VR/AR headsets (Oculus, HoloLens, etc.), enabling immersive robot control interfaces or training simulations

2. **Human-robot interaction studies**: Unity's access to realistic human models and animations from the Asset Store makes it ideal for HRI research where visual realism affects study results

3. **Photorealistic marketing demos**: When creating videos or interactive demos for stakeholders, Unity's AAA-quality graphics (raytracing, advanced materials, realistic lighting) far exceed Gazebo's capabilities

Other valid scenarios: cross-platform deployment (web, mobile), complex UI development, or when integrating with existing Unity-based systems.
</details>

<details>
<summary>2. How does Unity communicate with ROS 2, and what is the disadvantage of this approach?</summary>

**Answer**: Unity communicates with ROS 2 via the **ROS-TCP-Connector** architecture:

```
Unity (C#) ↔ TCP/IP ↔ ROS-TCP-Endpoint (Python node) ↔ ROS 2 DDS ↔ Other nodes
```

Unity scripts send/receive messages over TCP to a Python ROS 2 node, which translates them to/from ROS topics.

**Disadvantages**:
1. **Added latency**: TCP communication adds 5-10ms vs. Gazebo's native ROS integration
2. **Complexity**: Requires running an additional Python bridge node
3. **Reliability**: TCP connection can drop, requiring reconnection logic
4. **Debugging difficulty**: Harder to trace issues across Unity/TCP/ROS layers

In contrast, Gazebo's `ros_gz_bridge` uses native ROS 2 DDS for direct, low-latency communication.
</details>

<details>
<summary>3. Why is Unity's physics less suitable for robotics simulation than Gazebo's?</summary>

**Answer**: Unity's PhysX engine has several limitations for robotics:

1. **Time step**: Unity typically runs physics at 50 Hz (0.02s steps) vs. Gazebo's 1000 Hz (0.001s), missing fast dynamics

2. **Contact resolution**: PhysX is optimized for game-like interactions (bouncing, explosions) not precise robotic manipulation. Fewer tuning parameters than Gazebo's ODE/Bullet engines

3. **Accuracy focus**: Game engines prioritize visual plausibility over physical accuracy. Robotics needs predictable, repeatable behavior for algorithm validation

4. **Joint limitations**: Unity's joints can be unstable for complex kinematic chains (humanoids with 30+ DOF)

**Result**: Algorithms developed in Unity may not transfer well to real robots. Always validate physics-critical code in Gazebo.
</details>

<details>
<summary>4. Explain the coordinate system conversion between Unity and ROS.</summary>

**Answer**: Unity and ROS use different coordinate conventions:

**Unity**: Left-handed, Y-up
- Forward: +Z
- Right: +X
- Up: +Y

**ROS (REP 103)**: Right-handed, Z-up
- Forward: +X
- Left: +Y
- Up: +Z

**Conversion formulas** (Unity → ROS):
```
ROS_X = Unity_Z
ROS_Y = -Unity_X
ROS_Z = Unity_Y
```

**For rotations** (quaternions):
```
ROS_qx = Unity_qz
ROS_qy = -Unity_qx
ROS_qz = Unity_qy
ROS_qw = Unity_qw
```

This conversion must be applied in **every** script that publishes/subscribes to ROS pose/transform messages. Failure to convert causes robots to move in wrong directions or appear sideways.
</details>

<details>
<summary>5. What is a hybrid Gazebo + Unity workflow, and when is it useful?</summary>

**Answer**: A **hybrid workflow** uses both simulators simultaneously for their respective strengths:

**Workflow**:
1. Run Gazebo for physics, sensors, and algorithm development
2. Run Unity in parallel, subscribing to ROS topics from Gazebo
3. Unity visualizes the same scene with high-fidelity graphics
4. Gazebo provides accurate sensor data, Unity provides visualization

**Example architecture**:
```
[Gazebo] → /tf, /scan, /camera → [ROS 2 Topics] → [Unity visualization]
                                           ↓
                                    [Navigation stack]
```

**When useful**:
- **Demos**: Run algorithms in Gazebo, show photorealistic Unity view to stakeholders
- **HRI studies**: Simulate robot in Gazebo, visualize with Unity's realistic humans
- **VR teleoperation**: Control real/simulated robot in Gazebo, view through Unity VR

**Overhead**: Requires both simulators running, high computational cost, but separates concerns (physics vs. graphics).
</details>
