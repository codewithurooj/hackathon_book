# Launch Files and Parameters: Orchestrating ROS 2 Systems

## Introduction

As your ROS 2 systems grow, manually starting each node in separate terminals becomes tedious and error-prone. **Launch files** solve this by starting multiple nodes simultaneously with proper configuration. **Parameters** allow you to configure node behavior without modifying code. In this section, you'll learn to create launch files and manage parameters effectively.

## What Are Launch Files?

Launch files are Python scripts that:
- Start multiple nodes simultaneously
- Set parameters for each node
- Configure remappings (rename topics/services)
- Group related nodes together
- Handle complex startup sequences

**Benefits**:
- One command starts entire system
- Reproducible configurations
- Easy to share and version control
- Support conditional logic and includes

## Launch File Basics

### Simple Launch File

**File**: `my_robot_pkg/launch/talker_listener.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='talker',
            name='talker_node'
        ),
        Node(
            package='my_robot_pkg',
            executable='listener',
            name='listener_node'
        ),
    ])
```

### Launch File Structure

1. **Import statements**: Import launch utilities
2. **`generate_launch_description()` function**: Required entry point
3. **Return `LaunchDescription`**: Contains list of actions (nodes, parameters, etc.)

### Running Launch Files

```bash
ros2 launch my_robot_pkg talker_listener.launch.py
```

**Output**: Both talker and listener nodes start simultaneously.

## Launch File Installation

Update `setup.py` to install launch files:

```python
import os
from glob import glob
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
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    # ... rest of setup.py
)
```

**Key Addition**: The `glob('launch/*.launch.py')` line installs all launch files.

## Parameters in ROS 2

Parameters allow runtime configuration without code changes.

### Declaring Parameters in a Node

```python
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('my_string', 'default_value')
        self.declare_parameter('my_int', 42)
        self.declare_parameter('my_float', 3.14)

        # Get parameter values
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value

        self.get_logger().info(f'Parameters: string={my_string}, int={my_int}')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Setting Parameters via Command Line

```bash
ros2 run my_robot_pkg configurable_node --ros-args -p my_string:="Hello" -p my_int:=100
```

### Setting Parameters in Launch Files

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='configurable_node',
            name='configurable_node',
            parameters=[
                {'my_string': 'Configured via launch'},
                {'my_int': 999},
                {'my_float': 2.718}
            ]
        ),
    ])
```

## Parameter Files (YAML)

For complex configurations, use YAML files.

**File**: `my_robot_pkg/config/params.yaml`

```yaml
configurable_node:
  ros__parameters:
    my_string: "From YAML file"
    my_int: 777
    my_float: 1.414
    robot_name: "MyRobot"
    max_speed: 2.5
```

**Launch file with YAML parameters**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to config file
    config = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='configurable_node',
            name='configurable_node',
            parameters=[config]
        ),
    ])
```

**Install config files in `setup.py`**:

```python
(os.path.join('share', package_name, 'config'),
    glob('config/*.yaml')),
```

## Advanced Launch File Features

### Namespacing Nodes

```python
Node(
    package='my_robot_pkg',
    executable='talker',
    name='talker',
    namespace='robot1'  # Topic becomes /robot1/chatter
),
```

### Remapping Topics

```python
Node(
    package='my_robot_pkg',
    executable='listener',
    name='listener',
    remappings=[
        ('chatter', 'custom_topic')  # Rename topic
    ]
),
```

### Conditional Execution

```python
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare argument
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode'
    )

    return LaunchDescription([
        use_sim_arg,

        # Start node only if use_sim is true
        Node(
            package='my_robot_pkg',
            executable='simulator',
            name='simulator',
            condition=IfCondition(LaunchConfiguration('use_sim'))
        ),
    ])
```

**Run with argument**:
```bash
ros2 launch my_robot_pkg demo.launch.py use_sim:=true
```

### Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('other_package'),
                'launch',
                'other.launch.py'
            )
        ])
    )

    return LaunchDescription([
        other_launch,
        # ... your nodes
    ])
```

## Parameter Introspection

**List parameters**:
```bash
ros2 param list
```

**Get parameter value**:
```bash
ros2 param get /configurable_node my_string
```

**Set parameter at runtime**:
```bash
ros2 param set /configurable_node my_int 555
```

**Dump all parameters**:
```bash
ros2 param dump /configurable_node
```

## Best Practices

### 1. Organize Launch Files

```
my_robot_pkg/
├── launch/
│   ├── robot.launch.py       # Main launch file
│   ├── sensors.launch.py     # Sensor nodes
│   └── navigation.launch.py  # Navigation nodes
└── config/
    ├── robot_params.yaml
    └── sensor_params.yaml
```

### 2. Use Descriptive Names

```python
# Good
Node(package='my_robot_pkg', executable='motor_controller', name='left_wheel_controller')

# Bad
Node(package='my_robot_pkg', executable='node1', name='n1')
```

### 3. Provide Default Parameters

```python
self.declare_parameter('max_speed', 2.0)  # Always provide defaults
```

### 4. Document Launch Arguments

```python
DeclareLaunchArgument(
    'robot_name',
    default_value='robot1',
    description='Name of the robot (used for namespacing)'
)
```

## Summary

Launch files and parameters enable scalable ROS 2 system management:

1. **Launch Files**: Start multiple nodes with one command using Python launch files
2. **Installation**: Add launch files to `data_files` in `setup.py`
3. **Parameters**: Configure nodes at runtime without code changes
4. **YAML Files**: Store complex configurations in YAML format
5. **Advanced Features**: Namespacing, remapping, conditional execution, includes
6. **CLI Tools**: Use `ros2 param list/get/set/dump` for runtime parameter management

With launch files and parameters, you can now orchestrate complex multi-node ROS 2 systems efficiently!

## Review Questions

1. **What is the main benefit of using launch files?**
   <details>
   <summary>Answer</summary>
   Launch files allow you to start multiple nodes simultaneously with a single command, provide reproducible configurations, and manage complex startup sequences.
   </details>

2. **How do you declare a parameter with a default value in a node?**
   <details>
   <summary>Answer</summary>
   Use `self.declare_parameter('param_name', default_value)` in the node's `__init__()` method.
   </details>

3. **What is the required function name in a launch file?**
   <details>
   <summary>Answer</summary>
   `generate_launch_description()` - This function must return a `LaunchDescription` object.
   </details>

4. **How do you set parameters from a YAML file in a launch file?**
   <details>
   <summary>Answer</summary>
   Pass the path to the YAML file in the `parameters` argument: `parameters=[config_file_path]`
   </details>

5. **What command lists all parameters for a running node?**
   <details>
   <summary>Answer</summary>
   `ros2 param list` lists all nodes and their parameters. Use `ros2 param dump /node_name` to see all parameter values for a specific node.
   </details>

---

**Congratulations!** You've completed the core ROS 2 fundamentals. You now understand:
- ROS 2 architecture and communication patterns
- Package development and node creation
- Topics, services, and actions
- Launch files and parameter management

**Next Steps**: Apply these skills in simulation environments (Gazebo/Unity) and advanced platforms (NVIDIA Isaac) in the following modules!
