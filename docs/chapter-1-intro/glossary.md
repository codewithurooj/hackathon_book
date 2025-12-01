# Glossary: Chapter 1 Terms

## Core Concepts

### Physical AI
Artificial intelligence systems that are embodied in physical agents (robots, vehicles, drones, etc.) and interact directly with the physical environment through sensors and actuators. Unlike digital AI that operates in virtual environments, Physical AI must handle real-world constraints like physics, safety, and real-time processing.

**Example**: A warehouse robot that uses cameras and LIDAR to navigate, identify packages, and manipulate objects autonomously.

---

### Embodied Intelligence
The principle that having a physical body fundamentally changes how an AI system learns, reasons, and behaves. Physical constraints and sensorimotor feedback shape intelligence in ways that purely virtual learning cannot replicate.

**Example**: A robot learns what "fragile" means by experiencing the consequences of gripping too hard, rather than just reading a text definition.

---

### Sensor Fusion
The process of combining data from multiple sensors (cameras, LIDAR, IMU, force sensors, etc.) to create a more accurate and complete understanding of the environment than any single sensor could provide alone.

**Example**: Combining camera images (color, texture) with LIDAR data (precise 3D geometry) to both identify and locate objects in space.

---

## Sensor Technologies

### LIDAR (Light Detection and Ranging)
A sensor that uses laser pulses to measure distances to objects, creating 3D point clouds representing the environment's geometry. LIDAR provides accurate depth information regardless of lighting conditions but does not capture color or texture.

**How it works**: Emits laser beams in multiple directions, measures the time it takes for reflections to return, calculates distance (distance = speed of light × time / 2).

**Applications**: Autonomous vehicle navigation, 3D mapping, obstacle detection

---

### IMU (Inertial Measurement Unit)
A sensor that combines accelerometers (measuring linear acceleration) and gyroscopes (measuring angular velocity) to track motion and orientation. Essential for balance control in bipedal robots.

**Typical Components**:
- 3-axis accelerometer (measures acceleration in x, y, z directions)
- 3-axis gyroscope (measures rotation rates around x, y, z axes)
- Often includes magnetometer (digital compass) for absolute heading

**Applications**: Robot balance, orientation tracking, dead reckoning navigation

---

### Depth Camera
A camera that captures both color images and per-pixel depth information, providing 3D understanding of the scene. Common types include stereo cameras (using triangulation) and structured light cameras (projecting patterns).

**Example Technologies**:
- Intel RealSense (stereo and structured light)
- Microsoft Kinect (time-of-flight)
- Stereo camera pairs (passive depth estimation)

**Applications**: Object detection and localization, 3D scene reconstruction, gesture recognition

---

### Multi-Modal Sensing
The use of multiple sensor modalities (vision, depth, touch, sound, etc.) simultaneously to perceive the environment. Each modality provides complementary information, and fusing them creates robustness.

**Example**: A humanoid robot grasping an object uses:
- Vision (identify and locate object)
- Depth (measure distance)
- Force/torque sensors (detect contact and grip force)
- IMU (maintain balance during reaching motion)

---

## Robotics Middleware

### ROS 2 (Robot Operating System 2)
The industry-standard middleware for robot software development, providing tools and libraries for inter-process communication, hardware abstraction, device drivers, and common robot algorithms. ROS 2 is used in 90%+ of modern robotics projects.

**Key Features**:
- Publish/subscribe messaging (topics)
- Request/response communication (services)
- Long-running tasks (actions)
- Distributed architecture
- Language support (Python, C++, and more)

**Not an actual operating system**: Despite the name, ROS 2 runs on top of Linux (usually Ubuntu).

---

## AI & Learning

### Vision-Language-Action (VLA)
AI models that integrate visual perception, natural language understanding, and action generation. VLA enables robots to understand commands like "bring me the red mug from the kitchen" and translate them into executable action sequences.

**Components**:
- Vision: Object detection, scene understanding
- Language: Natural language processing, intent recognition
- Action: Task planning, motion generation

**Example Systems**: RT-2 (Google), PaLM-E, GPT-4 + Whisper integration

---

### Sim-to-Real Transfer
The process of training AI models (especially reinforcement learning policies) in simulation and then deploying them successfully on real robots despite differences between simulated and real-world physics.

**Challenge**: Simulations are imperfect - friction, contact dynamics, sensor noise differ from reality.

**Solutions**: Domain randomization, system identification, reality gap modeling

---

## Development Tools

### URDF (Unified Robot Description Format)
An XML format for describing robot models, including kinematics (links and joints), dynamics (mass, inertia), visual appearance, and collision geometry. Used extensively in ROS and Gazebo.

**Example Use**: Defining a humanoid robot's structure - where each joint is located, how it moves, what it looks like.

---

### Gazebo
An open-source 3D robot simulator that provides realistic physics simulation, sensor simulation (cameras, LIDAR, IMU), and integration with ROS. Widely used for testing robot algorithms before hardware deployment.

**Key Features**:
- Physics engines (ODE, Bullet, DART)
- Sensor plugins
- World creation tools
- Plugin system for custom behaviors

---

### Isaac Sim (NVIDIA)
A photorealistic, GPU-accelerated robot simulation platform built on NVIDIA Omniverse. Provides 10-100x faster physics simulation than CPU-based simulators and high-fidelity rendering for computer vision training.

**Advantages over Gazebo**:
- GPU acceleration (parallel physics)
- Photorealistic rendering
- Synthetic data generation at scale
- Integration with AI frameworks

---

## Career Terms

### Embodied AI Researcher
A researcher who develops AI models that learn from and interact with the physical world. Focus areas include reinforcement learning for robotics, vision-language models for task understanding, and sim-to-real transfer.

**Required Skills**: Deep learning, robotics, simulation, experimental design

---

### Robotics Software Engineer
An engineer who develops software for robot systems, including perception algorithms, motion planning, control systems, and integration with hardware. Works extensively with ROS 2, C++/Python, and real-time systems.

**Required Skills**: ROS 2, computer vision, motion planning, real-time programming

---

### Robotics Simulation Engineer
An engineer who builds high-fidelity simulation environments for training and testing robots. Focuses on physics engines, synthetic data generation, and sim-to-real techniques.

**Required Skills**: Physics engines (Isaac Sim, MuJoCo), 3D modeling, GPU programming, domain randomization

---

## Acronyms

- **AI**: Artificial Intelligence
- **DL**: Deep Learning
- **DOF**: Degrees of Freedom (number of independent joint movements)
- **FPS**: Frames Per Second
- **GPU**: Graphics Processing Unit
- **Hz**: Hertz (cycles per second, used for control rates)
- **IMU**: Inertial Measurement Unit
- **LIDAR**: Light Detection and Ranging
- **LLM**: Large Language Model
- **ML**: Machine Learning
- **RGB**: Red Green Blue (standard color camera)
- **ROS**: Robot Operating System
- **SLAM**: Simultaneous Localization and Mapping
- **URDF**: Unified Robot Description Format
- **VLA**: Vision-Language-Action

---

## Additional Resources

**For Deeper Understanding**:
- ROS 2 Documentation: https://docs.ros.org/
- Gazebo Tutorials: https://gazebosim.org/tutorials
- NVIDIA Isaac Sim: https://developer.nvidia.com/isaac-sim
- Physical AI Research: Papers on RT-2, PaLM-E, SayCan from Google DeepMind

**Academic References**:
- Brooks, R. (1991). "Intelligence without representation" - Foundational paper on embodied intelligence
- Thrun, S. (2002). "Probabilistic Robotics" - Classic textbook on robot perception and localization

---

**Return to**: [Chapter 1 Index](index.md)
