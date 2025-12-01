# Package Development: Creating ROS 2 Packages

## Introduction

In ROS 2, a **package** is the fundamental unit of organization. Think of it as a folder that contains related code, configuration files, and metadata. Whether you're building a simple sensor driver or a complex navigation system, everything lives inside packages.

In this section, you'll learn how to create Python-based ROS 2 packages, understand their structure, configure dependencies, and build them using the `colcon` build system.

## What is a ROS 2 Package?

A ROS 2 package is a directory containing:
- **Source code** (Python scripts, C++ files)
- **Configuration files** (package.xml, setup.py/CMakeLists.txt)
- **Launch files** (for starting multiple nodes)
- **Data files** (URDF, configuration YAML files)
- **Tests** (unit tests, integration tests)

**Key Characteristics**:
- **Self-Contained**: Each package has its own dependencies and metadata
- **Reusable**: Packages can be shared across projects
- **Buildable**: The build system (colcon) knows how to compile and install them
- **Versionable**: Can specify version numbers and maintain compatibility

## Package Types

ROS 2 supports three main package types:

1. **ament_python**: Python-only packages (what we'll use)
2. **ament_cmake**: C++ packages or mixed Python/C++
3. **ament_cmake_python**: CMake-based Python packages (less common)

For this course, we'll focus on **ament_python** packages since we're working exclusively with Python.

## Creating Your First Package

### Step 1: Navigate to Your Workspace

First, ensure you're in the `src` directory of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
```

**Workspace Structure**:
```
ros2_ws/
├── src/               # Source code goes here
│   └── my_package/    # Your packages
├── build/             # Build artifacts (auto-generated)
├── install/           # Installed files (auto-generated)
└── log/               # Build logs (auto-generated)
```

### Step 2: Create the Package

Use the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_python --node-name my_first_node my_robot_pkg
```

**Command Breakdown**:
- `ros2 pkg create`: Package creation tool
- `--build-type ament_python`: Specifies Python package
- `--node-name my_first_node`: Creates a sample Python node (optional)
- `my_robot_pkg`: Package name (must be unique in your workspace)

**Output**:
```
going to create a new package
package name: my_robot_pkg
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['user <user@todo.com>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: []
node_name: my_first_node
creating folder ./my_robot_pkg
creating ./my_robot_pkg/package.xml
creating source folder
creating folder ./my_robot_pkg/my_robot_pkg
creating ./my_robot_pkg/setup.py
creating ./my_robot_pkg/setup.cfg
creating folder ./my_robot_pkg/resource
creating ./my_robot_pkg/resource/my_robot_pkg
creating ./my_robot_pkg/my_robot_pkg/__init__.py
creating folder ./my_robot_pkg/test
creating ./my_robot_pkg/test/test_copyright.py
creating ./my_robot_pkg/test/test_flake8.py
creating ./my_robot_pkg/test/test_pep257.py
creating ./my_robot_pkg/my_robot_pkg/my_first_node.py
```

## Package Structure Explained

After creation, your package has this structure:

```
my_robot_pkg/
├── my_robot_pkg/              # Python module (same name as package)
│   ├── __init__.py            # Makes this a Python module
│   └── my_first_node.py       # Sample node
├── resource/                  # Package resources
│   └── my_robot_pkg           # Marker file for package discovery
├── test/                      # Unit tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                # Package metadata and dependencies
├── setup.py                   # Python package configuration
└── setup.cfg                  # Configuration for setup.py
```

### Key Files

#### 1. package.xml

This XML file contains package metadata:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Key Elements**:
- `<name>`: Package identifier
- `<version>`: Semantic versioning (major.minor.patch)
- `<description>`: Human-readable description
- `<maintainer>`: Who maintains this package
- `<license>`: Open-source license (Apache-2.0, MIT, BSD, etc.)
- `<buildtool_depend>`: Build system dependency
- `<exec_depend>`: Runtime dependencies (rclpy, message packages)
- `<test_depend>`: Testing dependencies

#### 2. setup.py

This file configures the Python package:

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='My first ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_robot_pkg.my_first_node:main',
        ],
    },
)
```

**Key Sections**:
- `packages`: List of Python modules to install
- `data_files`: Non-Python files (package.xml, launch files, URDF)
- `entry_points`: Executable scripts (how `ros2 run` finds your nodes)

**Entry Points Syntax**:
```python
'executable_name = package_name.module_name:function_name'
```

Example: `'my_first_node = my_robot_pkg.my_first_node:main'`
- Run with: `ros2 run my_robot_pkg my_first_node`

## Adding Dependencies

When your package uses other ROS 2 packages or Python libraries, declare them in `package.xml`:

### Example: Adding geometry_msgs Dependency

```xml
<exec_depend>geometry_msgs</exec_depend>
```

### Common Dependencies

```xml
<!-- Core ROS 2 Python library -->
<exec_depend>rclpy</exec_depend>

<!-- Standard message types -->
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>

<!-- TF2 (coordinate transformations) -->
<exec_depend>tf2_ros</exec_depend>
<exec_depend>tf2_geometry_msgs</exec_depend>
```

## Building Your Package

### Step 1: Navigate to Workspace Root

```bash
cd ~/ros2_ws
```

### Step 2: Build with colcon

```bash
colcon build --packages-select my_robot_pkg
```

**Build Options**:
- `--packages-select my_robot_pkg`: Build only this package (faster)
- `--symlink-install`: Symlink Python files instead of copying (useful during development)
- `--parallel-workers 4`: Use 4 parallel jobs (faster on multi-core systems)

**Development Tip**: Use `--symlink-install` for faster iteration:
```bash
colcon build --packages-select my_robot_pkg --symlink-install
```

With symlinks, you don't need to rebuild after modifying Python code—just re-source your workspace.

### Step 3: Source the Workspace

After building, source the setup file to add your package to the ROS 2 environment:

```bash
source install/setup.bash
```

**Important**: You must source `setup.bash` in every new terminal, or add it to your `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Running Your Node

```bash
ros2 run my_robot_pkg my_first_node
```

**Verify It's Running**:
```bash
# In another terminal
ros2 node list
```

You should see `/my_first_node` in the output.

## Common Package Development Workflow

1. **Create package structure**:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_package
   ```

2. **Add Python node files** to `my_package/my_package/` directory

3. **Update setup.py** with entry points:
   ```python
   entry_points={
       'console_scripts': [
           'my_node = my_package.my_node:main',
       ],
   },
   ```

4. **Add dependencies** to `package.xml`

5. **Build the package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_package --symlink-install
   ```

6. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

7. **Run your node**:
   ```bash
   ros2 run my_package my_node
   ```

8. **Iterate**: Edit Python code, re-source workspace, re-run (no rebuild needed with --symlink-install)

## Best Practices

1. **Naming Conventions**:
   - Package names: `snake_case` (e.g., `my_robot_controller`)
   - Node names: `snake_case` (e.g., `motor_controller.py`)
   - Executable names: match node names (e.g., `motor_controller`)

2. **One Package Per Functionality**:
   - `my_robot_description`: URDF files
   - `my_robot_control`: Control nodes
   - `my_robot_vision`: Computer vision nodes

3. **Version Control**:
   - Add `build/`, `install/`, and `log/` to `.gitignore`
   - Only commit `src/` directory

4. **Documentation**:
   - Update `<description>` in package.xml
   - Add README.md explaining what the package does

5. **Testing**:
   - Keep test files in the `test/` directory
   - Run tests with: `colcon test --packages-select my_package`

## Summary

Creating ROS 2 packages involves:

1. **Structure**: Packages are folders with Python modules, metadata (package.xml), and configuration (setup.py)
2. **Creation**: Use `ros2 pkg create` with `--build-type ament_python`
3. **Dependencies**: Declare in `<exec_depend>` tags in package.xml
4. **Entry Points**: Register executables in setup.py for `ros2 run`
5. **Building**: Use `colcon build` to compile and install packages
6. **Sourcing**: Run `source install/setup.bash` to use the package
7. **Workflow**: Create → Add code → Update metadata → Build → Source → Run

In the next section, we'll write actual ROS 2 nodes inside these packages using the rclpy library.

## Review Questions

1. **What are the three main files in a ROS 2 Python package?**
   <details>
   <summary>Answer</summary>
   package.xml (metadata and dependencies), setup.py (Python package configuration), and setup.cfg (setup.py configuration).
   </details>

2. **What is the purpose of the `entry_points` section in setup.py?**
   <details>
   <summary>Answer</summary>
   It registers executable scripts so they can be run with `ros2 run`. Each entry point maps an executable name to a Python function (typically `main()`).
   </details>

3. **Why should you use `--symlink-install` during development?**
   <details>
   <summary>Answer</summary>
   It creates symlinks to Python files instead of copying them, so you don't need to rebuild after modifying Python code—just re-source the workspace.
   </details>

4. **Where do you declare runtime dependencies like rclpy or geometry_msgs?**
   <details>
   <summary>Answer</summary>
   In the package.xml file, using `<exec_depend>` tags. Example: `<exec_depend>rclpy</exec_depend>`.
   </details>

5. **What command builds only a specific package named `my_package`?**
   <details>
   <summary>Answer</summary>
   `colcon build --packages-select my_package`
   </details>

---

**Next**: [Python Nodes](03-python-nodes.md) - Learn how to write ROS 2 nodes using rclpy
