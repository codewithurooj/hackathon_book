# Diagram Specifications

## Diagram 1: ROS 2 Architecture Overview

**File**: `assets/ros2-architecture-diagram.svg`
**Type**: Architecture diagram
**Content**:
- High-level overview of ROS 2 components.
- Nodes, topics, services, actions, parameters, RMW (ROS Middleware), DDS (Data Distribution Service).
- Arrows indicating communication flow.
- Emphasize the distributed nature.
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text describing overall ROS 2 architecture with key components and communication.

## Diagram 2: Pub-Sub Pattern

**File**: `assets/pub-sub-pattern.svg`
**Type**: Architecture diagram
**Content**:
- Two nodes: "Publisher Node" and "Subscriber Node".
- A topic in the middle: "/robot_data" (or similar).
- Arrows showing: Publisher → publishes → Topic, Topic → delivers → Subscriber.
- Message type label: "geometry_msgs/Twist" (or similar).
- Annotations: "Asynchronous", "One-to-many possible".
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text describing publisher-subscriber communication pattern in ROS 2.

## Diagram 3: Service Pattern

**File**: `assets/service-pattern.svg`
**Type**: Architecture diagram
**Content**:
- Two nodes: "Service Server Node" and "Service Client Node".
- A service in the middle: "/add_two_ints" (or similar).
- Arrows showing: Client → sends request → Service, Service → sends response → Client.
- Request/response message type labels: e.g., "example_interfaces/srv/AddTwoInts".
- Annotations: "Synchronous", "Request-response".
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text describing service request-response communication pattern in ROS 2.

## Diagram 4: Action Pattern

**File**: `assets/action-pattern.svg`
**Type**: Architecture diagram
**Content**:
- Three main components: "Action Client Node", "Action Server Node", and "Action Goal/Result/Feedback".
- Arrows showing: Client → sends goal → Server, Server → sends feedback → Client, Server → sends result → Client.
- Action interface type label: e.g., "example_interfaces/action/Fibonacci".
- Annotations: "Long-running tasks", "Preemptable", "Feedback loop".
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text describing action goal, feedback, and result communication pattern in ROS 2.

## Diagram 5: Package Structure

**File**: `assets/package-structure.svg`
**Type**: File structure diagram
**Content**:
- Visual representation of a typical ROS 2 Python package structure.
- Include `package.xml`, `setup.py`, `src/`, `resource/`, `launch/` directories.
- Show examples of Python nodes within `src/your_package_name/`.
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text illustrating the directory structure of a ROS 2 Python package.

## Diagram 6: URDF Kinematic Chain

**File**: `assets/urdf-kinematic-chain.svg`
**Type**: Conceptual diagram
**Content**:
- Visual representation of a simple kinematic chain (e.g., 2-link arm).
- Highlight `base_link`, `link1`, `link2` and `joint1`, `joint2`.
- Show how links are connected by joints.
- Annotations for parent/child links.
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text depicting a robotic kinematic chain with links and joints.

## Diagram 7: Launch File Flow

**File**: `assets/launch-file-flow.svg`
**Type**: Flowchart/Sequence diagram
**Content**:
- Illustrate the execution flow of a Python launch file.
- Steps: `ros2 launch` command → `LaunchDescription` → `Node` actions, `IncludeLaunchDescription` actions.
- Show how multiple nodes are started and how parameters are passed.
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text showing the process of a ROS 2 launch file starting nodes and managing parameters.

## Diagram 8: AI-ROS Integration Architecture

**File**: `assets/ai-ros-integration.svg`
**Type**: Architecture diagram
**Content**:
- Depict a high-level architecture for integrating an AI agent (e.g., LLM) with ROS 2.
- Show the AI agent communicating with ROS 2 nodes (e.g., via a bridge node).
- Indicate flow of natural language commands to ROS messages/actions and robot state feedback to AI.
- Highlight concepts like SayCan or RT-2 (abstractly).
**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text illustrating an architecture where an AI agent interacts with a ROS 2 robotic system.
