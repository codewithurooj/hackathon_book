# Python Nodes: Writing ROS 2 Nodes with rclpy

## Introduction

Now that you understand ROS 2 architecture and can create packages, it's time to write actual **nodes**—the fundamental building blocks of ROS 2 systems. In this section, you'll learn how to use **rclpy** (ROS Client Library for Python) to create nodes that initialize, log messages, use timers, and shutdown gracefully.

## What is rclpy?

**rclpy** is the Python client library for ROS 2. It provides the API to:
- Create and manage nodes
- Publish and subscribe to topics
- Create services and action servers/clients
- Use timers for periodic execution
- Handle parameters and logging

Think of rclpy as the bridge between your Python code and the underlying DDS middleware. You write Python, and rclpy translates it into ROS 2 communication primitives.

## Anatomy of a ROS 2 Node

Every ROS 2 Python node follows this basic structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Let's break this down step by step:**

### 1. Imports

```python
import rclpy
from rclpy.node import Node
```

- `rclpy`: Core ROS 2 Python library
- `Node`: Base class for all nodes

### 2. Node Class Definition

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node initialized')
```

**Key Points**:
- Inherit from `rclpy.node.Node`
- Call `super().__init__(node_name)` to initialize the base class
- Node name must be unique in the ROS 2 system (or use namespaces)
- Use `self.get_logger()` for ROS 2 logging (not `print()`)

### 3. Main Function

```python
def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2 context
    node = MyNode()                 # Create node instance
    rclpy.spin(node)                # Keep node running
    node.destroy_node()             # Cleanup
    rclpy.shutdown()                # Shutdown ROS 2 context
```

**Execution Flow**:
1. **`rclpy.init()`**: Initializes ROS 2 (must be called before creating nodes)
2. **Create node**: Instantiate your custom node class
3. **`rclpy.spin()`**: Blocks and processes callbacks (timers, subscriptions)
4. **Cleanup**: Destroy node and shutdown ROS 2 context when interrupted (Ctrl+C)

### 4. Entry Point

```python
if __name__ == '__main__':
    main()
```

Allows running the script directly: `python my_node.py`

## Example 1: Hello World Node

Let's create the simplest possible node:

**File**: `my_robot_pkg/my_robot_pkg/hello_node.py`

```python
#!/usr/bin/env python3
"""
Hello World ROS 2 Node
Demonstrates basic node initialization and logging
"""

import rclpy
from rclpy.node import Node

class HelloNode(Node):
    """A simple node that logs a message on startup"""

    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello from ROS 2!')
        self.get_logger().info('Node is running. Press Ctrl+C to exit.')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)

    # Cleanup on shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Register in setup.py**:
```python
entry_points={
    'console_scripts': [
        'hello_node = my_robot_pkg.hello_node:main',
    ],
},
```

**Run the node**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg --symlink-install
source install/setup.bash
ros2 run my_robot_pkg hello_node
```

**Output**:
```
[INFO] [hello_node]: Hello from ROS 2!
[INFO] [hello_node]: Node is running. Press Ctrl+C to exit.
```

## Logging in ROS 2

Never use `print()` in ROS 2 nodes. Instead, use the built-in logger:

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Informational message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal error message')
```

**Benefits of ROS 2 Logging**:
- Timestamps automatically added
- Log level filtering (can hide debug messages in production)
- Integration with ROS 2 tools (`ros2 topic echo /rosout`)
- Logged to `/rosout` topic for remote monitoring

**Setting Log Levels**:
```bash
# Run node with debug level
ros2 run my_robot_pkg hello_node --ros-args --log-level debug
```

## Using Timers for Periodic Execution

Timers allow you to execute code at regular intervals (e.g., publishing sensor data, updating controllers).

**Example 2: Counter Node**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class CounterNode(Node):
    """A node that counts up every second"""

    def __init__(self):
        super().__init__('counter_node')
        self.counter = 0

        # Create timer that calls timer_callback every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Counter node started')

    def timer_callback(self):
        """Called every second by the timer"""
        self.counter += 1
        self.get_logger().info(f'Count: {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    node = CounterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key API**:
```python
self.create_timer(timer_period_sec, callback_function)
```

- `timer_period_sec`: How often to call the callback (in seconds)
- `callback_function`: Method to execute (no arguments)

**Use Cases**:
- Publishing sensor data at fixed rates (e.g., 10 Hz, 100 Hz)
- Updating controller outputs
- Periodic health checks
- Heartbeat signals

## Node Lifecycle and Shutdown

Nodes should handle shutdown gracefully. The `rclpy.spin()` call blocks until interrupted (Ctrl+C), then cleanup code runs.

**Best Practices for Shutdown**:

```python
def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
```

**Cleanup Tasks**:
- Stop timers
- Close file handles
- Release hardware resources
- Send final messages (e.g., "shutting down")

## Node Introspection

While your node is running, you can inspect it using ROS 2 CLI tools:

**List all running nodes**:
```bash
ros2 node list
```

**Get node information**:
```bash
ros2 node info /counter_node
```

**Output**:
```
/counter_node
  Subscribers:

  Publishers:

  Services:
    /counter_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /counter_node/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    ...
```

Every node automatically provides services for parameter management, even if you haven't explicitly created any.

## Multiple Nodes in One Script

You can run multiple nodes in a single process using `MultiThreadedExecutor`:

```python
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    node1 = HelloNode()
    node2 = CounterNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

**Use Cases**:
- Testing multiple nodes together
- Combining related functionality
- Resource-constrained systems (fewer processes)

**Caution**: Usually better to keep nodes separate for fault isolation.

## Summary

Writing ROS 2 Python nodes involves:

1. **Inherit from Node**: Create a class inheriting from `rclpy.node.Node`
2. **Initialize**: Call `super().__init__(node_name)` in `__init__()`
3. **Use logging**: `self.get_logger().info()` instead of `print()`
4. **Timers**: Use `self.create_timer()` for periodic execution
5. **Main function**: Initialize ROS 2, create node, spin, cleanup
6. **Graceful shutdown**: Use try/finally blocks for cleanup
7. **Introspection**: Use `ros2 node list` and `ros2 node info` to inspect running nodes

In the next section, we'll extend these nodes to communicate with each other using **publishers and subscribers**.

## Review Questions

1. **What is the purpose of `rclpy.init()` and `rclpy.shutdown()`?**
   <details>
   <summary>Answer</summary>
   `rclpy.init()` initializes the ROS 2 context (must be called before creating nodes). `rclpy.shutdown()` cleans up ROS 2 resources when the program exits.
   </details>

2. **Why should you use `self.get_logger().info()` instead of `print()`?**
   <details>
   <summary>Answer</summary>
   ROS 2 logging provides timestamps, log levels, integration with `/rosout` topic for remote monitoring, and filtering capabilities. It's the standard way to output information in ROS 2 nodes.
   </details>

3. **What does `rclpy.spin(node)` do?**
   <details>
   <summary>Answer</summary>
   It blocks the main thread and processes callbacks (timers, subscriptions, services) for the node. It runs until interrupted (Ctrl+C) or `rclpy.shutdown()` is called.
   </details>

4. **How do you create a timer that calls a function every 0.5 seconds?**
   <details>
   <summary>Answer</summary>
   `self.create_timer(0.5, self.callback_function)` where `callback_function` is a method with no arguments.
   </details>

5. **What is the purpose of the `super().__init__('node_name')` call?**
   <details>
   <summary>Answer</summary>
   It initializes the base `Node` class with the node's name. This name is used to identify the node in the ROS 2 system and must be unique (or use namespaces).
   </details>

---

**Next**: [Publishers & Subscribers](04-publishers-subscribers.md) - Learn how to communicate between nodes using topics
