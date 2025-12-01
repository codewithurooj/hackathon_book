# Services and Actions: Request-Response Communication

## Introduction

While topics excel at streaming data, they're not ideal when you need a **response** to a request or want to track the **progress** of a long-running task. That's where **Services** (request-response) and **Actions** (goal-feedback-result) come in. In this section, you'll learn when and how to use these synchronous communication patterns.

## Services: Request-Response Pattern

### What Are Services?

Services implement a **client-server** pattern:
- **Service Server**: Waits for requests, processes them, returns responses
- **Service Client**: Sends requests, blocks until response arrives
- **Synchronous**: Client waits for server (blocking call)
- **One-to-One**: Each request gets exactly one response

### When to Use Services

**Good Use Cases**:
- Trigger one-time actions (reset, calibrate, save data)
- Query current state (get robot position, check battery level)
- Compute results (inverse kinematics, path planning)
- Configuration changes (set parameters, load maps)

**Bad Use Cases**:
- High-frequency data streaming (use topics instead)
- Long-running tasks without feedback (use actions instead)
- Broadcasting to multiple listeners (use topics)

### Service Types

Services are defined in `.srv` files with two parts: request and response.

**Example**: `std_srvs/SetBool.srv`
```
bool data    # Request
---
bool success  # Response
string message
```

**Common Service Packages**:
- **std_srvs**: Trigger, SetBool, Empty
- **rcl_interfaces**: GetParameters, SetParameters, DescribeParameters
- **nav2_msgs**: LoadMap, ClearCostmap

## Creating a Service Server

### Example: Add Two Numbers Service

**File**: `my_robot_pkg/my_robot_pkg/add_two_ints_server.py`

```python
#!/usr/bin/env python3
"""
Service Server - Adds two integers and returns the sum
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    """Service server that adds two integers"""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            'add_two_ints',          # Service name
            self.add_two_ints_callback  # Callback function
        )

        self.get_logger().info('Add Two Ints service ready')

    def add_two_ints_callback(self, request, response):
        """Handle service requests"""
        # Access request data
        a = request.a
        b = request.b

        # Compute result
        response.sum = a + b

        self.get_logger().info(f'Request: {a} + {b} = {response.sum}')

        # Return response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server API

```python
self.srv = self.create_service(ServiceType, 'service_name', callback)
```

**Callback Signature**:
```python
def callback(self, request, response):
    # Process request
    response.field = value
    return response
```

- **Parameters**: `request` (input data), `response` (output data)
- **Return**: Must return the `response` object
- **Blocking**: Callback should complete quickly (milliseconds, not seconds)

## Creating a Service Client

**File**: `my_robot_pkg/my_robot_pkg/add_two_ints_client.py`

```python
#!/usr/bin/env python3
"""
Service Client - Calls the add_two_ints service
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    """Service client that requests addition"""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        """Send service request"""
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (blocking)
        self.get_logger().info(f'Calling service: {a} + {b}')
        future = self.client.call_async(request)

        # Wait for result
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: add_two_ints_client <a> <b>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    client = AddTwoIntsClient()
    result = client.send_request(a, b)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client API

```python
self.client = self.create_client(ServiceType, 'service_name')
```

**Calling the Service**:
```python
future = self.client.call_async(request)  # Asynchronous call
rclpy.spin_until_future_complete(self, future)  # Wait for result
response = future.result()
```

### Running Services

**Terminal 1** (Server):
```bash
ros2 run my_robot_pkg add_two_ints_server
```

**Terminal 2** (Client):
```bash
ros2 run my_robot_pkg add_two_ints_client 5 7
```

**Output**:
```
[INFO] [add_two_ints_client]: Calling service: 5 + 7
[INFO] [add_two_ints_client]: Result: 12
```

### Service Introspection

**List services**:
```bash
ros2 service list
```

**Call service from CLI**:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

**Get service type**:
```bash
ros2 service type /add_two_ints
```

## Actions: Goal-Feedback-Result Pattern

### What Are Actions?

Actions are for **long-running tasks** that need:
- **Goals**: What you want to accomplish
- **Feedback**: Progress updates during execution
- **Results**: Final outcome (success/failure)
- **Cancellation**: Ability to abort mid-execution

### When to Use Actions

**Good Use Cases**:
- Navigation to a goal (feedback: distance remaining)
- Grasping an object (feedback: gripper position)
- Executing a trajectory (feedback: percentage complete)
- Long computations (feedback: progress %)

**Bad Use Cases**:
- Quick operations (&lt;1 second) - use services
- Continuous streaming - use topics
- Fire-and-forget commands - use topics

### Action Structure

Actions are defined in `.action` files with three parts:

**Example**: `Fibonacci.action`
```
int32 order          # Goal
---
int32[] sequence     # Result
---
int32[] partial_sequence  # Feedback
```

## Creating an Action Server

**File**: `my_robot_pkg/my_robot_pkg/fibonacci_action_server.py`

```python
#!/usr/bin/env python3
"""
Action Server - Computes Fibonacci sequence
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    """Action server that computes Fibonacci sequence"""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci action server ready')

    def execute_callback(self, goal_handle):
        """Execute the goal"""
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Prepare feedback and result
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()

        # Compute Fibonacci sequence
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result

            # Compute next number
            sequence.append(sequence[i] + sequence[i-1])

            # Publish feedback
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Feedback: {sequence}')

            # Simulate work (1 second per step)
            time.sleep(1)

        # Goal succeeded
        goal_handle.succeed()
        result.sequence = sequence

        self.get_logger().info(f'Goal succeeded! Result: {sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating an Action Client

**File**: `my_robot_pkg/my_robot_pkg/fibonacci_action_client.py`

```python
#!/usr/bin/env python3
"""
Action Client - Sends Fibonacci goal
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    """Action client that requests Fibonacci sequence"""

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

    def send_goal(self, order):
        """Send goal to action server"""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback updates"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)

    client = FibonacciActionClient()
    client.send_goal(10)

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running Actions

**Terminal 1** (Server):
```bash
ros2 run my_robot_pkg fibonacci_action_server
```

**Terminal 2** (Client):
```bash
ros2 run my_robot_pkg fibonacci_action_client
```

**Output (Client)**:
```
[INFO] [fibonacci_action_client]: Sending goal: order=10
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2, 3]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

## Services vs Actions: Decision Guide

| Criterion | Service | Action |
|-----------|---------|--------|
| **Duration** | Quick (&lt;1 sec) | Long (>1 sec) |
| **Feedback** | No | Yes (progress updates) |
| **Cancelable** | No | Yes |
| **Use Case** | Get data, trigger action | Execute trajectory, navigate |
| **Example** | Reset odometry | Navigate to waypoint |

## Best Practices

### 1. Service Timeouts

```python
# Wait for service with timeout
if not self.client.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Service not available')
    return
```

### 2. Action Cancellation

```python
# In action client
future = goal_handle.cancel_goal_async()
```

### 3. Error Handling

```python
# Service client error handling
if future.result() is not None:
    response = future.result()
else:
    self.get_logger().error('Service call failed')
```

### 4. Naming Conventions

```python
# Good: Descriptive, verb-based names
'/robot/reset_odometry'
'/navigation/compute_path'
'/gripper/grasp_object'

# Bad: Vague names
'/service1'
'/do_thing'
```

## Summary

Services and Actions provide synchronous communication in ROS 2:

1. **Services**: Request-response for quick operations (trigger actions, query state)
2. **Actions**: Goal-feedback-result for long-running tasks (navigation, grasping)
3. **Service Server/Client**: Use `create_service()` and `create_client()`
4. **Action Server/Client**: Use `ActionServer` and `ActionClient`
5. **Decision**: Use services for &lt;1 sec tasks, actions for longer tasks with feedback
6. **Best Practices**: Timeouts, error handling, descriptive naming, cancellation support

In the next section, we'll explore **URDF (Unified Robot Description Format)** for describing robot structures.

## Review Questions

1. **What is the main difference between a service and a topic?**
   <details>
   <summary>Answer</summary>
   Services are synchronous (client waits for response) and implement request-response, while topics are asynchronous (fire-and-forget) for streaming data. Services guarantee a response; topics don't.
   </details>

2. **When should you use an action instead of a service?**
   <details>
   <summary>Answer</summary>
   Use actions for long-running tasks (>1 second) that need progress feedback and/or cancellation capability. Examples: navigation, trajectory execution, long computations.
   </details>

3. **What are the three parts of an action message?**
   <details>
   <summary>Answer</summary>
   1) Goal (what to accomplish), 2) Result (final outcome), 3) Feedback (progress updates during execution).
   </details>

4. **How do you check if a service is available before calling it?**
   <details>
   <summary>Answer</summary>
   Use `self.client.wait_for_service(timeout_sec=X)` which returns True if service is available within the timeout period.
   </details>

5. **Can an action be canceled mid-execution?**
   <details>
   <summary>Answer</summary>
   Yes, actions support cancellation. The server checks `goal_handle.is_cancel_requested` and the client can call `goal_handle.cancel_goal_async()`.
   </details>

---

**Next**: [URDF Basics](06-urdf-basics.md) - Learn how to describe robot structures with URDF
