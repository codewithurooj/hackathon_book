# Content Development Plan: Chapter 2 - Module 1: The Robotic Nervous System (ROS 2)

**Branch**: `002-chapter-2-ros2` | **Date**: 2025-11-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/002-chapter-2-ros2/spec.md`

## Summary

Chapter 2 (Module 1) is the **first hands-on module** where students transition from conceptual understanding to practical ROS 2 development. This chapter teaches distributed robotics architecture, Python node development with rclpy, URDF robot modeling, launch files, and AI-ROS integration patterns. Unlike Chapter 1 (conceptual), this module requires **extensive working code examples, exercises, and troubleshooting guidance**.

**Primary Requirement**: Enable students to create ROS 2 packages, write publisher/subscriber/service/action nodes in Python, understand URDF for robot modeling, and prepare for simulation (Module 2) and perception (Module 3).

**Technical Approach**: Markdown educational content + **tested Python code examples** + URDF files + launch files + hands-on exercises with solutions + troubleshooting guides. All code must run on Ubuntu 22.04 + ROS 2 Humble.

## Technical Context

**Content Format**: Markdown (MD) + Python (.py) + XML (URDF, launch files, package.xml)
**Primary Dependencies**:
- ROS 2 Humble (Hawksbill) - primary target
- Python 3.10+ with rclpy
- Ubuntu 22.04 LTS (recommended environment)
- colcon build system
- RViz (robot visualization)

**Storage**:
- Content: Docusaurus `docs/chapter-2-ros2/`
- Code examples: Repository `examples/chapter-2-ros2/` with ROS 2 workspace structure
- URDF files: `examples/chapter-2-ros2/urdf/`

**Testing**:
- All Python code examples tested on Ubuntu 22.04 + ROS 2 Humble
- Code linting (ruff, black for formatting)
- URDF validation (check_urdf tool)
- Integration testing (launch all examples, verify communication)
- Student pilot testing of exercises

**Target Platform**:
- Primary: Ubuntu 22.04 LTS
- Alternative: Docker container with ROS 2 Humble (for non-Linux users)
- Cloud option: AWS RoboMaker, Google Cloud with ROS 2 (for students without local setup)

**Project Type**: Educational content with executable code examples

**Performance Goals**:
- Student completion time: 8-12 hours across Weeks 3-5
- Code example execution: Immediate (<1 second startup for simple nodes)
- Build time: <30 seconds for student packages

**Constraints**:
- All code must be beginner-friendly (clear variable names, extensive comments)
- Must work on ROS 2 Humble (most common in education as of 2024-2025)
- Cannot assume C++ knowledge (Python-only)
- Must provide fallback options for students without Ubuntu (Docker, cloud)
- Code examples must be copy-pasteable and immediately runnable

**Scale/Scope**:
- 10,000-14,000 words explanatory text
- 12-18 Python code examples (complete, tested)
- 3-5 URDF files (simple robot → humanoid progression)
- 4-6 launch file examples
- 5-7 hands-on exercises with solutions
- 8-12 diagrams (architecture, data flow, URDF visualization)
- 8-12 hours student time investment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Spec-Driven Development**: Specification complete and validated in `spec.md`

✅ **AI-Native Educational Content**:
- Code examples include natural language explanations
- Progressive difficulty: hello-world node → complex multi-node systems
- Conceptual explanations separated from code implementation (better RAG retrieval)

✅ **Interactive Learning Experience**:
- Hands-on exercises require students to write and run code
- Exercises have verifiable outputs (node publishes message, subscriber receives, etc.)
- RAG chatbot can answer questions about specific code examples

✅ **Modular Content Architecture**:
- Module 1 aligns with Weeks 3-5 of course
- Prerequisites: Chapter 1 (Physical AI concepts), basic Python, command-line familiarity
- Prepares for: Module 2 (Simulation with Gazebo), Module 3 (Isaac perception)

✅ **Code Quality & Reproducibility**:
- All code examples tested on specified environment (Ubuntu 22.04 + Humble)
- Dependencies documented with versions (rclpy version, Python version)
- Clear installation and setup instructions
- Troubleshooting guide for common issues

✅ **Testing & Validation**:
- All code examples execute successfully
- URDF files validated with check_urdf
- Exercises tested with student cohort (if available)
- Integration tests verify multi-node communication

**Result**: ✅ All constitution principles satisfied. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/002-chapter-2-ros2/
├── spec.md                    # Feature specification (✅ complete)
├── plan.md                    # This file (content development plan)
├── research.md                # Phase 0: ROS 2 best practices, rclpy patterns
├── content-outline.md         # Phase 1: Detailed section outline with code examples
├── code-examples-spec.md      # Phase 1: Specifications for all code examples
├── exercise-designs.md        # Phase 1: Exercise specifications with solutions
├── diagram-specs.md           # Phase 1: Visual content requirements
├── troubleshooting-guide.md   # Phase 1: Common issues and solutions
├── quickstart.md              # Phase 1: Developer quick reference
└── tasks.md                   # Phase 2: Task list (created by /sp.tasks command)
```

### Content Delivery (Docusaurus repository)

```text
docs/
└── chapter-2-ros2/
    ├── index.md                          # Module overview and learning objectives
    ├── 01-ros2-architecture.md           # ROS 2 concepts (nodes, topics, services, actions)
    ├── 02-package-development.md         # Creating packages, package structure
    ├── 03-python-nodes.md                # Writing nodes with rclpy
    ├── 04-publishers-subscribers.md      # Pub/sub pattern with examples
    ├── 05-services-actions.md            # Request/response and long-running tasks
    ├── 06-urdf-basics.md                 # URDF structure, links, joints
    ├── 07-urdf-humanoid.md               # Humanoid URDF examples
    ├── 08-launch-files.md                # Launch file creation and parameters
    ├── 09-ai-ros-integration.md          # Bridging AI agents with ROS 2
    ├── 10-development-tools.md           # ROS 2 CLI tools, RViz, debugging
    ├── 11-exercises.md                   # Hands-on exercises
    ├── 12-troubleshooting.md             # Common issues and solutions
    └── assets/
        ├── ros2-architecture-diagram.svg
        ├── pub-sub-pattern.svg
        ├── service-pattern.svg
        ├── action-pattern.svg
        ├── package-structure.svg
        ├── urdf-kinematic-chain.svg
        ├── launch-file-flow.svg
        ├── ai-ros-integration.svg
        └── rviz-screenshots/
            ├── robot-visualization.png
            └── topic-monitoring.png
```

### Code Examples (Git repository)

```text
examples/
└── chapter-2-ros2/
    ├── README.md                      # Setup instructions, how to run examples
    ├── ros2_ws/                       # ROS 2 workspace
    │   └── src/
    │       ├── hello_ros2/            # Example 1: Minimal node
    │       │   ├── package.xml
    │       │   ├── setup.py
    │       │   └── hello_ros2/
    │       │       └── hello_node.py
    │       ├── talker_listener/       # Example 2: Publisher-subscriber
    │       │   ├── package.xml
    │       │   ├── setup.py
    │       │   ├── launch/
    │       │   │   └── talker_listener.launch.py
    │       │   └── talker_listener/
    │       │       ├── talker.py
    │       │       └── listener.py
    │       ├── simple_service/        # Example 3: Service pattern
    │       │   ├── package.xml
    │       │   ├── setup.py
    │       │   └── simple_service/
    │       │       ├── add_server.py
    │       │       └── add_client.py
    │       ├── robot_description/     # Example 4: URDF examples
    │       │   ├── package.xml
    │       │   ├── urdf/
    │       │   │   ├── simple_arm.urdf
    │       │   │   ├── mobile_robot.urdf
    │       │   │   └── humanoid_basic.urdf
    │       │   └── launch/
    │       │       └── display_robot.launch.py
    │       └── ai_ros_bridge/         # Example 5: AI-ROS integration
    │           ├── package.xml
    │           ├── setup.py
    │           ├── requirements.txt   # Python deps (openai, langchain, etc.)
    │           └── ai_ros_bridge/
    │               ├── simple_agent_publisher.py
    │               └── llm_command_bridge.py
    └── exercises/                     # Exercise starter code + solutions
        ├── exercise_1_my_first_node/
        │   ├── README.md
        │   ├── starter/               # Student starting point
        │   └── solution/              # Complete solution
        ├── exercise_2_robot_controller/
        ├── exercise_3_urdf_modification/
        ├── exercise_4_multi_node_system/
        └── exercise_5_ai_integration/
```

**Structure Decision**:
- **Multi-file content**: 12 focused pages for better navigation and RAG retrieval
- **Separate code repository**: Examples in `examples/` directory with full ROS 2 workspace structure
- **Exercise scaffolding**: Each exercise has starter code (for students) and solution (for self-check/instructors)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - No constitution violations. Module complexity is appropriate for hands-on technical training.

---

## Phase 0: Research & Best Practices

### Research Tasks

1. **ROS 2 Humble Teaching Best Practices**
   - **Question**: What are proven pedagogical approaches for teaching ROS 2 to beginners?
   - **Research Focus**:
     - Minimal viable example (hello world node) vs complex realistic examples
     - Order of concepts: nodes → topics → services → actions vs all-at-once
     - Common student misconceptions about pub/sub pattern
     - How much ROS 1 background to assume (many online resources still ROS 1)

2. **rclpy Code Style and Patterns**
   - **Question**: What are current best practices for writing rclpy nodes in 2024-2025?
   - **Research Focus**:
     - Node class structure (OOP vs functional)
     - Proper shutdown and cleanup patterns
     - QoS (Quality of Service) settings - when to teach vs when to use defaults
     - Type hints and modern Python 3.10+ features
     - Logging best practices

3. **URDF Pedagogy**
   - **Question**: How to teach URDF without overwhelming students with XML complexity?
   - **Research Focus**:
     - Minimal URDF (2-link arm) vs realistic humanoid URDF
     - When to introduce collision geometry vs visual geometry
     - Inertial properties: realistic vs "good enough for learning"
     - Tools: raw XML vs xacro vs URDF Python library
     - URDF validation and debugging tools

4. **Common ROS 2 Installation Issues (Ubuntu 22.04 + Humble)**
   - **Question**: What are the most frequent student setup problems?
   - **Research Focus**:
     - ROS 2 Humble installation pitfalls
     - Environment setup (sourcing setup.bash, ROS_DOMAIN_ID conflicts)
     - Python virtual environments vs system Python with ROS 2
     - Docker alternatives for Windows/macOS users
     - Cloud development environments (AWS Cloud9, GitHub Codespaces with ROS 2)

5. **AI-ROS Integration Patterns**
   - **Question**: What are established patterns for connecting LLMs/AI agents to ROS 2?
   - **Research Focus**:
     - Literature review: SayCan, RT-2, other research integrating LLMs with robotics
     - Code patterns: running LLM in separate process vs same process as ROS node
     - Message conversion: natural language → ROS messages
     - State management: feeding robot state to LLM context
     - Latency considerations: when is LLM planning too slow?

**Output**: Detailed findings documented in `research.md` with code examples and architectural recommendations.

---

## Phase 1: Content Design & Artifacts

### Deliverables

#### 1. Content Outline (`content-outline.md`)

**Purpose**: Section-by-section breakdown with word counts, key concepts, code examples, and exercises.

**Example Structure**:
```markdown
# Chapter 2 Content Outline

## Section 1: ROS 2 Architecture (01-ros2-architecture.md)
- **Word Count Target**: 1,200-1,500 words
- **Key Concepts**:
  - Distributed architecture vs monolithic programs
  - Nodes as independent processes
  - Communication paradigms: topics (pub/sub), services (req/res), actions (long-running)
  - ROS 2 vs ROS 1 improvements (DDS, real-time, security)
- **Diagrams**:
  - ROS 2 architecture overview
  - Pub/sub pattern visualization
  - Service and action patterns
- **Code**: No code in this section (concepts only)
- **Learning Objectives**:
  - Explain why distributed architecture benefits robotics
  - Identify when to use topics vs services vs actions
  - Understand DDS as underlying middleware

## Section 4: Publishers and Subscribers (04-publishers-subscribers.md)
- **Word Count Target**: 1,800-2,200 words
- **Key Concepts**:
  - Publish-subscribe pattern
  - Topic naming conventions
  - Message types (std_msgs, geometry_msgs, custom messages)
  - Callbacks and asynchronous processing
- **Code Examples**:
  - Example 1: Minimal publisher (string messages)
  - Example 2: Minimal subscriber with callback
  - Example 3: Publisher-subscriber pair (talker-listener)
  - Example 4: Publishing sensor data (geometry_msgs/Twist)
- **Exercises**:
  - Exercise 1: Modify publisher rate and message content
  - Exercise 2: Create subscriber that processes messages (counter, filter, etc.)
- **Troubleshooting Notes**: Common issues with topic names, callback syntax

[Continue for all 12 sections...]
```

#### 2. Code Examples Specification (`code-examples-spec.md`)

**Purpose**: Detailed specification for every code example including purpose, inputs, outputs, and testing criteria.

**Example Format**:
```markdown
# Code Examples Specification

## Example 1: Minimal Hello World Node

**File**: `examples/chapter-2-ros2/ros2_ws/src/hello_ros2/hello_ros2/hello_node.py`

**Purpose**: Simplest possible ROS 2 node to demonstrate node creation and spinning.

**Learning Objective**: Students learn node lifecycle and basic rclpy API.

**Code Structure**:
```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select hello_ros2
source install/setup.bash
ros2 run hello_ros2 hello_node
```

**Expected Output**:
```
[INFO] [hello_node]: Hello, ROS 2!
```

**Testing Criteria**:
- [ ] Node starts without errors
- [ ] Log message appears in terminal
- [ ] Node shows up in `ros2 node list`
- [ ] Can be stopped with Ctrl+C cleanly

**Common Student Errors**:
- Forgetting to source setup.bash → "package not found"
- Incorrect package name in setup.py → build failures

[Specifications for 15+ more examples...]
```

#### 3. Exercise Designs (`exercise-designs.md`)

**Purpose**: Complete exercise specifications with learning objectives, starter code, solution code, and grading rubrics.

**Example Format**:
```markdown
# Exercise Designs

## Exercise 1: My First ROS 2 Node

**Difficulty**: Beginner
**Time Estimate**: 30-45 minutes
**Prerequisites**: Completed reading through Section 3 (Python Nodes)

**Learning Objectives**:
- Create a ROS 2 package using `ros2 pkg create`
- Write a simple node that publishes messages
- Build and run the node

**Instructions** (provided to students):
```
1. Create a new ROS 2 package called "my_robot_controller"
2. Create a Python node that publishes "Robot says hi!" to a topic called /robot_greetings
3. The node should publish at 1 Hz (once per second)
4. Build the package and run the node
5. Verify messages are publishing using `ros2 topic echo /robot_greetings`
```

**Starter Code** (in `exercises/exercise_1_my_first_node/starter/`):
- Empty package structure (package.xml, setup.py templates)
- Comments indicating where to add code

**Solution Code** (in `exercises/exercise_1_my_first_node/solution/`):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotGreeter(Node):
    def __init__(self):
        super().__init__('robot_greeter')
        self.publisher = self.create_publisher(String, '/robot_greetings', 10)
        self.timer = self.create_timer(1.0, self.publish_greeting)  # 1 Hz

    def publish_greeting(self):
        msg = String()
        msg.data = 'Robot says hi!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotGreeter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Grading Rubric** (for instructors/self-assessment):
- Package builds successfully (2 points)
- Node runs without errors (2 points)
- Messages publish to correct topic (2 points)
- Publishing rate is 1 Hz (2 points)
- Message content is correct (2 points)

**Common Mistakes to Watch For**:
- Timer period confusion (1.0 seconds vs 1.0 Hz)
- Forgetting to add executable to setup.py
- Topic name typos

[Designs for 5-7 more exercises...]
```

#### 4. Diagram Specifications (`diagram-specs.md`)

**Purpose**: Visual content requirements for all architecture diagrams, flow charts, and URDF visualizations.

**Example**:
```markdown
# Diagram Specifications

## Diagram 1: ROS 2 Publisher-Subscriber Pattern

**File**: `assets/pub-sub-pattern.svg`
**Type**: Architecture diagram
**Content**:
- Two nodes: "Publisher Node" and "Subscriber Node"
- Topic in the middle: "/robot_data"
- Arrows showing: Publisher → publishes → Topic, Topic → delivers → Subscriber
- Message type label: "geometry_msgs/Twist"
- Annotations: "Asynchronous", "One-to-many possible"

**Tool**: Draw.io, Figma, or Inkscape (export as SVG)
**Accessibility**: Alt text describing pub/sub pattern flow

[Specifications for 10+ more diagrams...]
```

#### 5. Troubleshooting Guide (`troubleshooting-guide.md`)

**Purpose**: Common errors students encounter and step-by-step solutions.

**Example**:
```markdown
# Troubleshooting Guide

## Error: "Package 'my_package' not found"

**Symptom**: When running `ros2 run my_package my_node`, get error about package not found.

**Likely Causes**:
1. Forgot to build: Run `colcon build` first
2. Forgot to source: Run `source install/setup.bash` in workspace
3. Wrong workspace: Check you're in the correct ROS 2 workspace directory

**Solution Steps**:
```bash
# Step 1: Verify you're in workspace root
pwd  # Should show .../ros2_ws

# Step 2: Build the package
colcon build --packages-select my_package

# Step 3: Source the setup file
source install/setup.bash

# Step 4: Try again
ros2 run my_package my_node
```

[30+ common errors documented...]
```

#### 6. Quickstart Guide (`quickstart.md`)

**Purpose**: Quick reference for content developers and instructors.

**Example**:
```markdown
# Chapter 2 Development Quickstart

## Code Example Testing Checklist
- [ ] All examples build with `colcon build`
- [ ] All examples run without errors
- [ ] Output matches expected output in spec
- [ ] ROS 2 CLI tools (`ros2 topic list`, etc.) show expected resources
- [ ] URDF files validate with `check_urdf`

## Exercise Validation
- [ ] Starter code provides enough scaffolding
- [ ] Solution code runs correctly
- [ ] Time estimates are realistic
- [ ] Instructions are clear and unambiguous

## Content Review
- [ ] All 36 functional requirements (FR-001 to FR-036) covered
- [ ] Code follows PEP 8 and includes type hints
- [ ] All diagrams have alt text
- [ ] Troubleshooting guide addresses common issues from pilot testing
```

---

## Phase 2: Task Generation

**Note**: Phase 2 task generation will be handled by the `/sp.tasks` command.

Tasks will include:
- Write Section 1: ROS 2 Architecture (acceptance: FR-001 to FR-007 satisfied)
- Create Example 1: Hello World Node (acceptance: code builds, runs, produces expected output)
- Create Example 2: Publisher-Subscriber (acceptance: communication verified)
- Write Exercise 1: My First Node (acceptance: starter + solution code tested)
- Create Diagram 1: Pub/Sub Pattern (acceptance: meets diagram spec, has alt text)
- Write Troubleshooting Guide section on "Package not found" errors
- Etc.

---

## Architecture Decisions

### Decision 1: Python-Only vs Python + C++

**Context**: ROS 2 supports both Python (rclpy) and C++ (rclcpp). Many robotics systems use C++ for performance.

**Options Considered**:
1. Python-only (rclpy)
2. Python + C++ (both rclpy and rclcpp)
3. C++-only (rclcpp)

**Decision**: Python-only (rclpy)

**Rationale**:
- **Accessibility**: Curriculum assumes basic programming, not C++ expertise
- **Rapid prototyping**: Python faster for learning concepts
- **AI integration**: Module 4 (VLA) uses Python (OpenAI SDK, LangChain)
- **Sufficient performance**: Python adequate for capstone project scope

**Trade-offs**: Students miss C++ exposure, but can learn later if pursuing robotics careers.

### Decision 2: ROS 2 Humble vs Iron vs Rolling

**Context**: ROS 2 has multiple distributions. Humble (LTS, 2022-2027), Iron (2023-2024), Rolling (latest).

**Options Considered**:
1. Humble (LTS)
2. Iron (newer)
3. Rolling (cutting-edge)

**Decision**: ROS 2 Humble (Hawksbill)

**Rationale**:
- **Long-term support**: Humble supported until 2027
- **Stability**: Fewer breaking changes, better for education
- **Ecosystem maturity**: More packages, tutorials, community support
- **Ubuntu 22.04 LTS pairing**: Matches with LTS OS

**Trade-offs**: Miss newest features, but prioritize stability for learning.

### Decision 3: Raw URDF vs Xacro

**Context**: URDF files can be written in XML or using Xacro macros for modularity.

**Options Considered**:
1. Raw URDF XML only
2. Xacro for all examples
3. Start with URDF, introduce Xacro later

**Decision**: Raw URDF for learning, mention Xacro as advanced topic

**Rationale**:
- **Conceptual clarity**: Raw URDF shows structure without abstraction layer
- **Debugging**: Easier to understand errors in raw URDF
- **Sufficient for scope**: Capstone project doesn't require complex URDF modularization

**Trade-offs**: Students miss Xacro benefits (DRY, parameterization), but reduces cognitive load.

### Decision 4: Package Structure (colcon vs ament_python)

**Context**: ROS 2 packages can use different build systems.

**Options Considered**:
1. Python packages with ament_python (simpler for Python-only)
2. CMake packages with ament_cmake (C++ style, more complex)

**Decision**: ament_python for all Python packages

**Rationale**:
- **Consistency**: Python-only approach aligns with Decision 1
- **Simplicity**: Less boilerplate than CMake
- **Adequate**: ament_python handles all course needs

**Trade-offs**: None significant for Python-only course.

---

## Success Metrics

### Content Quality Metrics
- **Code quality**: All examples pass linting (ruff), formatting (black), type checking (mypy)
- **URDF validation**: All URDF files pass `check_urdf` with no errors
- **Build success**: 100% of examples build with `colcon build` on Ubuntu 22.04 + Humble
- **Completeness**: All 36 functional requirements (FR-001 to FR-036) addressed in content

### Learning Outcome Metrics
- **SC-001**: 85% can create ROS 2 packages from scratch
- **SC-002**: 90% can write working publisher-subscriber pairs
- **SC-003**: 80% can explain differences between topics, services, actions
- **SC-004**: 75% can interpret URDF files and identify links/joints
- **SC-005**: 85% can use ROS 2 CLI tools (topic, node, service)
- **SC-007**: 70%+ complete 3+ hands-on exercises successfully
- **SC-008**: 75% feel prepared for Module 2 (Simulation)

### Engagement Metrics
- **SC-006**: Completion time 8-12 hours (measured via student survey)
- Student satisfaction: 4+ out of 5 on module quality
- Exercise completion rate: 70%+ complete at least 3 exercises
- Code example reuse: Students successfully modify examples for new use cases

---

## Risk Mitigation

### Risk 1: Environment Setup Failures (Ubuntu/ROS 2 Installation)

**Mitigation**:
- Provide **detailed installation guide** with screenshots
- Offer **Docker container** with pre-configured ROS 2 Humble environment
- Cloud alternative: **AWS RoboMaker** or **Google Cloud** with ROS 2 (if budget allows)
- **Troubleshooting guide** covers common installation issues
- **Installation verification script** checks environment before starting exercises

### Risk 2: Code Examples Break Due to ROS 2 Updates

**Mitigation**:
- **Pin ROS 2 Humble** (LTS until 2027, unlikely to break)
- **Version all dependencies** in requirements.txt
- **CI/CD pipeline** tests all examples on every commit
- **Annual review** before course offering to catch any deprecations
- **GitHub repo** allows community contributions for fixes

### Risk 3: Students Overwhelmed by Complexity (XML, CLI tools, new concepts)

**Mitigation**:
- **Progressive complexity**: Start with minimal hello-world, gradually add features
- **Provide complete starter code** for exercises (not blank files)
- **Troubleshooting guide** addresses common frustrations
- **Video walkthroughs** (optional) showing how to run examples
- **Fast-track option**: Pre-built packages students can use directly

### Risk 4: Gap Between Simple Examples and Complex Capstone

**Mitigation**:
- **Exercise 5** is capstone-preview: multi-node system with AI integration
- **Intermediate exercises** bridge gap (Exercise 2-4)
- **Integration examples** show complete systems, not just isolated nodes
- Module 3 (Simulation) and Module 4 (VLA) provide additional practice before capstone

---

## Next Steps

1. **Phase 0 Complete**: Research ROS 2 pedagogy and create `research.md`
2. **Phase 1 Complete**: Generate all Phase 1 deliverables (content outline, code specs, exercises, diagrams, troubleshooting, quickstart)
3. **Ready for `/sp.tasks`**: Task generation command will create atomic development tasks
4. **Content Development**: Write markdown, implement code examples, create diagrams
5. **Testing & Validation**: Run all code examples, test exercises, peer review
6. **Integration**: Add to Docusaurus, commit code examples to repository, test with students
