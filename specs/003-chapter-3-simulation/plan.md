# Content Development Plan: Chapter 3 - Module 2: The Digital Twin (Gazebo & Unity)

**Branch**: `003-chapter-3-simulation` | **Date**: 2025-11-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/003-chapter-3-simulation/spec.md`

## Summary

Chapter 3 (Module 2) teaches students how to **simulate humanoid robots** in physics engines before deploying to real hardware. This module covers **Gazebo** (primary focus for physics/sensors) and **Unity** (secondary focus for visualization/HRI). Students learn to create simulation worlds, configure physics properties, simulate sensors (LIDAR, cameras, IMU), and test robot behaviors in safe virtual environments.

**Primary Requirement**: Enable students to create Gazebo simulation environments, add sensors to URDF robot models, configure physics parameters, visualize in RViz, and validate robot behaviors through simulation before proceeding to advanced perception (Module 3) and capstone deployment.

**Technical Approach**: Markdown content + **Gazebo SDF world files** + **URDF with sensor plugins** + **ROS 2 launch files** + hands-on exercises + troubleshooting guide. Unity section is introductory/conceptual (when to use, not deep implementation).

## Technical Context

**Content Format**: Markdown + SDF (XML) + URDF (XML) + Python launch files
**Primary Dependencies**:
- Gazebo 11 (Gazebo Classic) or Gazebo Fortress/Harmonic (newer)
- ROS 2 Humble + gz_ros2_control
- Sensor plugins: gazebo_ros_ray (LIDAR), gazebo_ros_camera, gazebo_ros_imu
- RViz for visualization
- Unity (optional): Unity 2022 LTS with ROS-TCP-Connector

**Storage**:
- Content: Docusaurus `docs/chapter-3-simulation/`
- Simulation files: `examples/chapter-3-simulation/worlds/`, `models/`, `launch/`

**Testing**:
- All Gazebo worlds launch without errors
- All URDF+sensor models load correctly
- Sensor data streams verified with `ros2 topic echo`
- Physics behavior validated (gravity, collisions work as expected)

**Target Platform**:
- Primary: Ubuntu 22.04 + Gazebo 11 (most compatible with ROS 2 Humble)
- Alternative: Docker with Gazebo pre-configured
- Unity: Windows/macOS/Linux with Unity Hub

**Project Type**: Educational content with simulation files

**Performance Goals**:
- Student completion time: 8-12 hours across Weeks 6-7
- Simulation startup: <10 seconds for simple worlds
- Real-time factor: ≥0.5x (simulation runs at least half real-time speed on student hardware)

**Constraints**:
- Must run on modest hardware (integrated GPU, 8GB RAM)
- Provide performance optimization guidance for struggling systems
- Gazebo version must be compatible with ROS 2 Humble
- Unity section must be beginner-accessible (no prior game dev experience)

**Scale/Scope**:
- 8,000-12,000 words explanatory text
- 5-8 Gazebo world files (simple → complex environments)
- 3-5 URDF files with sensors
- 5-7 launch files
- 4-6 hands-on exercises
- 10-15 diagrams/screenshots
- 8-12 hours student time

## Constitution Check

✅ **Spec-Driven Development**: Specification complete in `spec.md`
✅ **AI-Native Content**: Simulation concepts + practical examples with clear explanations
✅ **Interactive Learning**: Hands-on exercises require running simulations and observing behavior
✅ **Modular Architecture**: Module 2 builds on Module 1 (ROS 2), prepares for Module 3 (Isaac perception)
✅ **Code Quality**: All simulation files tested and validated
✅ **Testing**: Simulation files execute correctly, sensor outputs verified

**Result**: ✅ Proceed to Phase 0.

## Project Structure

### Documentation
```text
specs/003-chapter-3-simulation/
├── spec.md
├── plan.md (this file)
├── research.md              # Gazebo vs Isaac pedagogy, sensor simulation best practices
├── content-outline.md       # Section breakdown
├── simulation-specs.md      # World/model file specifications
├── exercise-designs.md      # Exercise specs with solutions
├── quickstart.md
└── tasks.md                 # Created by /sp.tasks
```

### Content Delivery
```text
docs/chapter-3-simulation/
├── index.md                        # Module overview
├── 01-gazebo-basics.md             # Gazebo interface, physics engine
├── 02-world-creation.md            # SDF worlds, objects, lighting
├── 03-sensor-simulation.md         # LIDAR, cameras, IMU plugins
├── 04-urdf-sensors.md              # Adding sensors to URDF
├── 05-physics-properties.md        # Gravity, friction, collisions, inertia
├── 06-unity-overview.md            # When to use Unity, ROS-Unity integration
├── 07-humanoid-simulation.md       # Loading humanoid URDF with sensors
├── 08-testing-validation.md        # Simulation workflows, data logging
├── 09-exercises.md
├── 10-troubleshooting.md
└── assets/                         # Screenshots, diagrams
```

### Simulation Files
```text
examples/chapter-3-simulation/
├── README.md
├── worlds/
│   ├── empty_world.sdf             # Minimal world
│   ├── simple_obstacle_course.sdf  # Basic navigation testing
│   ├── indoor_environment.sdf      # House/office layout
│   └── humanoid_test_world.sdf     # Full humanoid simulation
├── models/
│   ├── mobile_robot_with_lidar/    # URDF + sensor example
│   └── humanoid_with_sensors/      # Complete sensor suite
├── launch/
│   ├── empty_world.launch.py
│   ├── spawn_robot.launch.py
│   └── humanoid_sim.launch.py
└── exercises/
    ├── exercise_1_add_lidar/
    ├── exercise_2_obstacle_course/
    └── exercise_3_humanoid_sensors/
```

## Phase 0: Research

1. **Gazebo 11 vs Gazebo Harmonic/Fortress**
   - Which version to teach (Gazebo Classic vs new Gazebo)
   - ROS 2 Humble compatibility
   - Student hardware requirements

2. **Sensor Simulation Accuracy**
   - Realistic sensor noise models
   - LIDAR ray patterns vs real sensors
   - Camera distortion simulation

3. **Gazebo Performance Optimization**
   - LOD (Level of Detail) techniques
   - Physics update rates
   - GPU vs CPU rendering

4. **Unity-ROS Integration**
   - ROS-TCP-Connector setup
   - When Unity adds value over Gazebo
   - HRI visualization examples

5. **Sim-to-Real Transfer**
   - Domain randomization basics
   - What transfers well vs poorly
   - Setting student expectations

## Phase 1: Deliverables

1. **Content Outline** - Section-by-section with concepts + files
2. **Simulation Specs** - Every world/model file specified
3. **Exercise Designs** - 4-6 progressive simulation exercises
4. **Diagram Specs** - Gazebo interface, sensor placement, physics concepts
5. **Troubleshooting Guide** - Gazebo crashes, sensor data issues, performance problems

## Architecture Decisions

### Decision 1: Gazebo 11 (Classic) vs Gazebo Harmonic

**Decision**: Gazebo 11 (Classic)

**Rationale**:
- Better ROS 2 Humble compatibility
- More tutorials/community support
- Lower hardware requirements
- Sufficient for educational needs

**Trade-offs**: Miss new Gazebo features, but prioritize stability.

### Decision 2: Deep Unity Coverage vs Overview

**Decision**: Unity as conceptual overview, not deep tutorial

**Rationale**:
- Gazebo sufficient for physics/sensors (primary focus)
- Unity setup complex (requires large download, game dev knowledge)
- Most students don't need Unity for capstone
- Brief overview + "when to use Unity" guidance

**Trade-offs**: Students wanting HRI visualization miss deep Unity content, but can explore independently.

### Decision 3: Custom Worlds vs Pre-Made Worlds

**Decision**: Provide both pre-made worlds and teach world creation

**Rationale**:
- Pre-made: Get started quickly, focus on robot behavior
- Custom: Understand SDF structure, create test scenarios
- Progressive: Use pre-made first, then customize

## Success Metrics

### Content Quality
- All world files launch in Gazebo without errors
- All sensor URDFs publish data to ROS topics
- Physics behavior matches real-world expectations (gravity, collisions)

### Learning Outcomes
- **SC-001**: 80% can create Gazebo worlds and configure physics
- **SC-002**: 85% can add LIDAR/cameras and visualize in RViz
- **SC-003**: 75% can load humanoids and verify sensor topics
- **SC-004**: 70% explain Gazebo vs Unity differences
- **SC-005**: 80% create test environments and run simulations

## Risk Mitigation

### Risk 1: Gazebo Performance on Student Hardware

**Mitigation**:
- Provide lightweight worlds (minimal graphics)
- Optimization guide (reduce physics rate, disable shadows)
- Cloud simulation option (AWS, Google Cloud)
- Pre-recorded demos for students with incompatible hardware

### Risk 2: Gazebo Installation/Version Confusion

**Mitigation**:
- Specify Gazebo 11 explicitly
- Detailed installation guide with verification steps
- Troubleshooting common install issues
- Docker fallback

### Risk 3: Physics Instability (Robots Fall Through Floor, Jitter)

**Mitigation**:
- Teach physics debugging (inertia, collision geometry)
- Provide validated robot models
- Common physics issues in troubleshooting guide

## Next Steps

1. Phase 0: Research and create `research.md`
2. Phase 1: Generate all deliverables
3. `/sp.tasks`: Create atomic tasks
4. Content development: Write sections, create worlds, test
5. Validation: Test all simulation files, exercises
6. Integration: Add to Docusaurus, commit simulation files
