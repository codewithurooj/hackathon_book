# Publishers and Subscribers: Topic-Based Communication

## Introduction

Topics are the backbone of ROS 2 communication. They implement the **publish-subscribe pattern**, allowing nodes to exchange data asynchronously without needing to know about each other. In this section, you'll learn how to create publishers that send messages and subscribers that receive them, using standard ROS 2 message types.

## The Publish-Subscribe Pattern

### How It Works

1. **Publishers** send messages to named topics (e.g., `/camera/image`, `/robot/velocity`)
2. **Topics** are communication channels identified by names
3. **Subscribers** listen to topics and execute callbacks when messages arrive
4. **Many-to-Many**: Multiple publishers and subscribers can use the same topic

### Key Characteristics

- **Asynchronous**: Publishers don't wait for subscribers
- **Decoupled**: Publishers and subscribers don't know about each other
- **Fire-and-Forget**: Publishers send messages regardless of who's listening
- **Type-Safe**: Topics have a specific message type (e.g., `String`, `Twist`)

### Real-World Analogy

Think of topics like radio broadcasts:
- **Topic**: A radio frequency (e.g., 101.5 FM)
- **Publisher**: Radio station transmitting on that frequency
- **Subscriber**: Car radio tuned to that frequency
- **Message**: Audio content being transmitted

Just like radio, multiple listeners can tune in, and the broadcaster doesn't know who's listening.

## ROS 2 Message Types

Before creating publishers/subscribers, you need to understand message types. ROS 2 provides standard message packages:

### Common Message Packages

- **std_msgs**: Basic types (String, Int32, Float64, Bool)
- **geometry_msgs**: Geometric data (Point, Pose, Twist, Transform)
- **sensor_msgs**: Sensor data (Image, LaserScan, Imu, JointState)
- **nav_msgs**: Navigation (Odometry, Path)

### Message Structure

Each message type is defined in `.msg` files. For example, `std_msgs/String`:

```
string data
```

A more complex example, `geometry_msgs/Twist` (velocity command):

```
Vector3  linear
  float64 x
  float64 y
  float64 z
Vector3  angular
  float64 x
  float64 y
  float64 z
```

## Creating a Publisher

### Basic Publisher Node

Let's create a node that publishes string messages:

**File**: `my_robot_pkg/my_robot_pkg/talker.py`

```python
#!/usr/bin/env python3
"""
Talker Node - Publishes string messages to /chatter topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    """Publishes messages at 1 Hz"""

    def __init__(self):
        super().__init__('talker')

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer to publish every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Talker node started')

    def timer_callback(self):
        """Called every second to publish a message"""
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher API Breakdown

```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

**Parameters**:
1. **Message Type**: `String` (from `std_msgs.msg`)
2. **Topic Name**: `'chatter'` (can be any valid topic name)
3. **Queue Size**: `10` (how many messages to buffer)

**Queue Size Explained**:
- If publishing faster than subscribers can process, queue buffers messages
- When queue is full, oldest messages are dropped
- Typical values: 10 (default), 1 (only latest), 100 (large buffer)

### Publishing Messages

```python
msg = String()
msg.data = 'Hello!'
self.publisher_.publish(msg)
```

**Steps**:
1. Create message instance
2. Fill in message fields
3. Call `publish()` method

## Creating a Subscriber

### Basic Subscriber Node

**File**: `my_robot_pkg/my_robot_pkg/listener.py`

```python
#!/usr/bin/env python3
"""
Listener Node - Subscribes to /chatter topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    """Subscribes to string messages"""

    def __init__(self):
        super().__init__('listener')

        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        self.get_logger().info('Listener node started')

    def listener_callback(self, msg):
        """Called whenever a message is received"""
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber API Breakdown

```python
self.subscription = self.create_subscription(
    String,           # Message type
    'chatter',        # Topic name
    self.listener_callback,  # Callback function
    10                # Queue size
)
```

**Callback Function**:
- Must accept one argument (the message)
- Called automatically when messages arrive
- Should be fast (long operations block other callbacks)

## Running Publisher and Subscriber

### Setup

1. Add to `setup.py`:
```python
entry_points={
    'console_scripts': [
        'talker = my_robot_pkg.talker:main',
        'listener = my_robot_pkg.listener:main',
    ],
},
```

2. Add dependency to `package.xml`:
```xml
<exec_depend>std_msgs</exec_depend>
```

3. Build:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg --symlink-install
source install/setup.bash
```

### Run the Nodes

**Terminal 1** (Publisher):
```bash
ros2 run my_robot_pkg talker
```

**Output**:
```
[INFO] [talker]: Talker node started
[INFO] [talker]: Published: "Hello ROS 2! Count: 0"
[INFO] [talker]: Published: "Hello ROS 2! Count: 1"
[INFO] [talker]: Published: "Hello ROS 2! Count: 2"
```

**Terminal 2** (Subscriber):
```bash
ros2 run my_robot_pkg listener
```

**Output**:
```
[INFO] [listener]: Listener node started
[INFO] [listener]: Received: "Hello ROS 2! Count: 0"
[INFO] [listener]: Received: "Hello ROS 2! Count: 1"
[INFO] [listener]: Received: "Hello ROS 2! Count: 2"
```

## Using Complex Messages: Twist Example

### Publishing Velocity Commands

**File**: `my_robot_pkg/my_robot_pkg/velocity_publisher.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    """Publishes velocity commands for a robot"""

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info('Velocity publisher started')

    def publish_velocity(self):
        """Publish a velocity command"""
        msg = Twist()

        # Linear velocity (forward/backward, meters/sec)
        msg.linear.x = 0.5   # Move forward at 0.5 m/s
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # Angular velocity (rotation, radians/sec)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.2  # Rotate at 0.2 rad/s

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Add dependency**:
```xml
<exec_depend>geometry_msgs</exec_depend>
```

## Topic Introspection Tools

### List Active Topics

```bash
ros2 topic list
```

**Output**:
```
/chatter
/cmd_vel
/parameter_events
/rosout
```

### Get Topic Info

```bash
ros2 topic info /chatter
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Echo Topic Messages

```bash
ros2 topic echo /chatter
```

**Output**:
```
data: 'Hello ROS 2! Count: 0'
---
data: 'Hello ROS 2! Count: 1'
---
```

### Check Publishing Rate

```bash
ros2 topic hz /chatter
```

**Output**:
```
average rate: 1.000
  min: 1.000s max: 1.000s std dev: 0.00000s window: 10
```

### Publish from Command Line

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"
```

Useful for testing subscribers without writing a publisher.

## Best Practices

### 1. Topic Naming Conventions

```python
# Good: Descriptive, hierarchical names
'/robot/camera/image'
'/robot/sensors/imu'
'/navigation/cmd_vel'

# Bad: Vague, flat names
'/img'
'/data'
'/output'
```

### 2. Message Frequency Considerations

- **High frequency** (100 Hz+): Sensor data, control loops
- **Medium frequency** (1-10 Hz): State updates, diagnostics
- **Low frequency** (&lt;1 Hz): Configuration changes, heartbeats

### 3. Avoid Heavy Processing in Callbacks

```python
# Bad: Slow processing blocks other callbacks
def listener_callback(self, msg):
    result = expensive_computation(msg.data)  # Takes 5 seconds!

# Good: Offload to separate thread or use async
def listener_callback(self, msg):
    self.latest_msg = msg  # Just store the message

# Process in timer callback or separate thread
```

### 4. Type Safety

```python
# Import specific message types
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Type hints for clarity
def listener_callback(self, msg: String) -> None:
    self.get_logger().info(msg.data)
```

## Common Patterns

### Pattern 1: Republishing Transformed Data

```python
class DataTransformer(Node):
    def __init__(self):
        super().__init__('data_transformer')

        # Subscribe to input topic
        self.sub = self.create_subscription(String, 'input', self.callback, 10)

        # Publish to output topic
        self.pub = self.create_publisher(String, 'output', 10)

    def callback(self, msg):
        # Transform data
        transformed_msg = String()
        transformed_msg.data = msg.data.upper()  # Convert to uppercase

        # Republish
        self.pub.publish(transformed_msg)
```

### Pattern 2: Multiple Subscribers

```python
class MultiSubscriber(Node):
    def __init__(self):
        super().__init__('multi_subscriber')

        self.sub1 = self.create_subscription(String, 'topic1', self.callback1, 10)
        self.sub2 = self.create_subscription(Twist, 'topic2', self.callback2, 10)

    def callback1(self, msg: String):
        self.get_logger().info(f'Topic1: {msg.data}')

    def callback2(self, msg: Twist):
        self.get_logger().info(f'Topic2: linear.x={msg.linear.x}')
```

## Summary

Publishers and subscribers enable decoupled, asynchronous communication in ROS 2:

1. **Publishers**: Use `create_publisher(MessageType, 'topic_name', queue_size)`
2. **Subscribers**: Use `create_subscription(MessageType, 'topic_name', callback, queue_size)`
3. **Message Types**: Import from standard packages (std_msgs, geometry_msgs, sensor_msgs)
4. **Topic Tools**: Use `ros2 topic list/echo/info/hz/pub` for debugging
5. **Best Practices**: Descriptive topic names, appropriate frequencies, fast callbacks
6. **Patterns**: Data transformation, multi-topic subscriptions, republishing

In the next section, we'll explore **Services and Actions** for request-response and goal-oriented communication.

## Review Questions

1. **What is the difference between a publisher and a subscriber?**
   <details>
   <summary>Answer</summary>
   A publisher sends messages to a topic (`create_publisher()`), while a subscriber receives messages from a topic (`create_subscription()`). Publishers are asynchronous (don't wait for subscribers), and subscribers use callbacks to process incoming messages.
   </details>

2. **What are the three parameters required for `create_publisher()`?**
   <details>
   <summary>Answer</summary>
   1) Message type (e.g., `String`, `Twist`), 2) Topic name (e.g., `'chatter'`), 3) Queue size (e.g., `10`).
   </details>

3. **Why should callbacks be fast and avoid heavy processing?**
   <details>
   <summary>Answer</summary>
   Slow callbacks block other callbacks from executing (timers, other subscriptions). Long processing should be offloaded to separate threads or processed in timer callbacks.
   </details>

4. **How do you check the publishing rate of a topic from the command line?**
   <details>
   <summary>Answer</summary>
   `ros2 topic hz /topic_name` shows the average publishing rate, min/max, and standard deviation.
   </details>

5. **What command lists all active topics in the ROS 2 system?**
   <details>
   <summary>Answer</summary>
   `ros2 topic list`
   </details>

---

**Next**: [Services & Actions](05-services-actions.md) - Learn request-response and goal-oriented communication
