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

## Example 2: Simple Publisher (Talker)

**File**: `examples/chapter-2-ros2/ros2_ws/src/talker_listener/talker_listener/talker.py`

**Purpose**: Demonstrates creating a publisher node to send string messages.

**Learning Objective**: Students learn to create publishers and send messages on a topic.

**Code Structure**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select talker_listener
source install/setup.bash
ros2 run talker_listener talker
```

**Expected Output**:
```
[INFO] [talker]: Publishing: "Hello World: 0"
[INFO] [talker]: Publishing: "Hello World: 1"
...
```

**Testing Criteria**:
- [ ] Node starts without errors
- [ ] Messages published to 'topic'
- [ ] Message content increments correctly
- [ ] Publishing rate is approximately 0.5 seconds

## Example 3: Simple Subscriber (Listener)

**File**: `examples/chapter-2-ros2/ros2_ws/src/talker_listener/talker_listener/listener.py`

**Purpose**: Demonstrates creating a subscriber node to receive string messages.

**Learning Objective**: Students learn to create subscribers and process incoming messages.

**Code Structure**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
# In a new terminal, after starting the talker node
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 run talker_listener listener
```

**Expected Output**:
```
[INFO] [listener]: I heard: "Hello World: 0"
[INFO] [listener]: I heard: "Hello World: 1"
...
```

**Testing Criteria**:
- [ ] Node starts without errors
- [ ] Receives messages published by `talker`
- [ ] Message content matches published data
- [ ] Callback function is invoked correctly

## Example 4: Publishing Twist Messages

**File**: `examples/chapter-2-ros2/ros2_ws/src/talker_listener/talker_listener/twist_publisher.py`

**Purpose**: Demonstrates publishing `geometry_msgs/Twist` messages to control a robot.

**Learning Objective**: Students learn to use common message types for robot control.

**Code Structure**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.linear_x = 0.1
        self.angular_z = 0.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select talker_listener
source install/setup.bash
ros2 run talker_listener twist_publisher
```

**Expected Output**:
```
[INFO] [twist_publisher]: Publishing Twist: linear.x=0.1, angular.z=0.0
...
```

**Testing Criteria**:
- [ ] Node starts without errors
- [ ] `Twist` messages published to 'cmd_vel'
- [ ] `linear.x` and `angular.z` values are correct
- [ ] Publishing rate is approximately 1.0 second

## Example 5: Simple Service (Server and Client)

**Server File**: `examples/chapter-2-ros2/ros2_ws/src/simple_service/simple_service/add_server.py`
**Client File**: `examples/chapter-2-ros2/ros2_ws/src/simple_service/simple_service/add_client.py`

**Purpose**: Demonstrates creating a service server and client for a simple addition operation.

**Learning Objective**: Students learn to implement request/response communication using services.

**Service Interface (AddTwoInts.srv)**:
```
int64 a
int64 b
---
int64 sum
```

**Server Code Structure**:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Service Server is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
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

**Client Code Structure**:
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client_node = AddTwoIntsClient()

    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        client_node.get_logger().info('Usage: ros2 run simple_service add_client <a> <b>')
        rclpy.shutdown()
        sys.exit(1)

    response = client_node.send_request(a, b)
    client_node.get_logger().info(f'Result of add_two_ints: for {a} + {b} = {response.sum}')
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
# In one terminal
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select simple_service
source install/setup.bash
ros2 run simple_service add_server

# In another terminal
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 run simple_service add_client 5 7
```

**Expected Output**:
- **Server**: `Incoming request: a=5, b=7`, `Sending back response: 12`
- **Client**: `Result of add_two_ints: for 5 + 7 = 12`

**Testing Criteria**:
- [ ] Server starts and awaits requests
- [ ] Client sends request successfully
- [ ] Server processes request and sends correct response
- [ ] Client receives correct response

## Example 6: Simple Action (Server and Client)

**Server File**: `examples/chapter-2-ros2/ros2_ws/src/simple_action/simple_action/fibonacci_action_server.py`
**Client File**: `examples/chapter-2-ros2/ros2_ws/src/simple_action/simple_action/fibonacci_action_client.py`

**Purpose**: Demonstrates creating an action server and client for a Fibonacci sequence generation task.

**Learning Objective**: Students learn to implement long-running tasks with feedback using actions.

**Action Interface (Fibonacci.action)**:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**Server Code Structure**:
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info('Fibonacci Action Server is ready.')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1) # Simulate long-running task

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
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

**Client Code Structure**:
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10) # Request Fibonacci sequence up to order 10
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
# In one terminal
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select simple_action
source install/setup.bash
ros2 run simple_action fibonacci_action_server

# In another terminal
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 run simple_action fibonacci_action_client
```

**Expected Output**:
- **Server**: Logs execution steps and feedback.
- **Client**: Logs goal acceptance, feedback, and final result sequence.

**Testing Criteria**:
- [ ] Server starts and processes goal
- [ ] Client sends goal and receives feedback/result
- [ ] Fibonacci sequence is generated correctly
- [ ] Action lifecycle (goal, feedback, result) is correctly implemented

## URDF 1: Simple 2-link Arm

**File**: `examples/chapter-2-ros2/ros2_ws/src/robot_description/urdf/simple_arm.urdf`

**Purpose**: Defines a simple robotic arm with two links and one revolute joint.

**Learning Objective**: Students learn basic URDF structure, links, and joints.

**Code Structure**:
```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

</robot>
```

**How to View**:
```bash
# After building robot_description package
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 launch robot_description display_robot.launch.py model:=simple_arm.urdf
```

**Expected Output**: RViz displays a red base link and a green cylinder link connected by a revolute joint.

**Testing Criteria**:
- [ ] URDF file is syntactically valid (checked with `check_urdf`)
- [ ] RViz displays the arm correctly
- [ ] Links and joint are correctly defined and connected

## URDF 2: Mobile Robot with Wheels

**File**: `examples/chapter-2-ros2/ros2_ws/src/robot_description/urdf/mobile_robot.urdf`

**Purpose**: Defines a simple mobile robot with a base and two differential drive wheels.

**Learning Objective**: Students learn to model more complex structures, including multiple joints and cylindrical links for wheels.

**Code Structure**:
```xml
<?xml version="1.0"?>
<robot name="mobile_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="left_wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <link name="right_wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel_link"/>
    <origin xyz="0 0.15 0" rpy="1.5707 -1.5707 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel_link"/>
    <origin xyz="0 -0.15 0" rpy="1.5707 -1.5707 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

**How to View**:
```bash
# After building robot_description package
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 launch robot_description display_robot.launch.py model:=mobile_robot.urdf
```

**Expected Output**: RViz displays a blue box base and two black cylinder wheels.

**Testing Criteria**:
- [ ] URDF file is syntactically valid
- [ ] RViz displays the mobile robot correctly
- [ ] Wheels are correctly positioned and have continuous joints

## URDF 3: Basic Humanoid Structure

**File**: `examples/chapter-2-ros2/ros2_ws/src/robot_description/urdf/humanoid_basic.urdf`

**Purpose**: Defines a basic humanoid robot structure with multiple links and joints.

**Learning Objective**: Students learn to model complex multi-articulated robots and understand hierarchical URDF structures.

**Code Structure**:
```xml
<?xml version="1.0"?>
<robot name="humanoid_basic">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
  </link>

  <!-- Head Link and Joint -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- Left Arm Links and Joints -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.15 0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.03"/>
      </geometry>
      <material name="grey"/>
    </visual>
  </link>

  <joint name="left_upper_arm_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 0 0" rpy="0 1.5707 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <!-- ... (similar structure for right arm, left leg, right leg) ... -->

</robot>
```

**How to View**:
```bash
# After building robot_description package
cd examples/chapter-2-ros2/ros2_ws
source install/setup.bash
ros2 launch robot_description display_robot.launch.py model:=humanoid_basic.urdf
```

**Expected Output**: RViz displays a basic humanoid robot structure.

**Testing Criteria**:
- [ ] URDF file is syntactically valid
- [ ] RViz displays the humanoid correctly
- [ ] All links and joints are defined hierarchically

## Example: Multi-node Launch File (Talker-Listener)

**File**: `examples/chapter-2-ros2/ros2_ws/src/talker_listener/launch/talker_listener.launch.py`

**Purpose**: Demonstrates launching multiple ROS 2 nodes simultaneously using a Python launch file.

**Learning Objective**: Students learn to orchestrate multi-node systems with launch files.

**Code Structure**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker_listener',
            executable='talker',
            name='my_talker',
            output='screen'
        ),
        Node(
            package='talker_listener',
            executable='listener',
            name='my_listener',
            output='screen'
        )
    ])
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select talker_listener
source install/setup.bash
ros2 launch talker_listener talker_listener.launch.py
```

**Expected Output**: Both `my_talker` and `my_listener` nodes start, and messages are exchanged and logged in the terminal.

**Testing Criteria**:
- [ ] Launch file executes without errors
- [ ] Both talker and listener nodes start successfully
- [ ] Communication between nodes is observed

## Example: Launch File with Parameters

**File**: `examples/chapter-2-ros2/ros2_ws/src/param_example/launch/param_example.launch.py`

**Purpose**: Demonstrates how to pass and manage parameters for ROS 2 nodes using a launch file.

**Learning Objective**: Students learn to configure node behavior dynamically via launch parameters.

**Node Code (param_node.py)**:
```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter('my_parameter', 'default_value')
        param = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Hello with parameter: {param}')

def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Launch File Code (param_example.launch.py)**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='param_example',
            executable='param_node',
            name='my_param_node',
            output='screen',
            parameters=[
                {'my_parameter': 'custom_value'}
            ]
        )
    ])
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select param_example
source install/setup.bash
ros2 launch param_example param_example.launch.py
```

**Expected Output**: `Hello with parameter: custom_value`

**Testing Criteria**:
- [ ] Node starts and reads its parameter
- [ ] Parameter value is correctly overridden by launch file

## Example: Composable Launch Files (Includes)

**File**: `examples/chapter-2-ros2/ros2_ws/src/composed_launch/launch/composed_main.launch.py`

**Purpose**: Demonstrates how to include other launch files within a main launch file for modular system definition.

**Learning Objective**: Students learn to build complex launch systems from smaller, reusable components.

**Included Launch File (included_node.launch.py)**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='param_example',
            executable='param_node',
            name='included_param_node',
            output='screen',
            parameters=[
                {'my_parameter': 'value_from_included_launch'}
            ]
        )
    ])
```

**Main Launch File (composed_main.launch.py)**:
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_example_dir = get_package_share_directory('composed_launch') # Assuming included_node.launch.py is in composed_launch/launch
    included_launch_file = os.path.join(param_example_dir, 'launch', 'included_node.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_launch_file)
        )
    ])
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select composed_launch param_example # Build both packages
source install/setup.bash
ros2 launch composed_launch composed_main.launch.py
```

**Expected Output**: `Hello with parameter: value_from_included_launch`

**Testing Criteria**:
- [ ] Main launch file executes successfully
- [ ] Included launch file starts its node
- [ ] Parameters from included launch file are applied correctly

## Example: Simple Agent Publisher (AI Agent to ROS Topic)

**File**: `examples/chapter-2-ros2/ros2_ws/src/ai_ros_bridge/ai_ros_bridge/simple_agent_publisher.py`

**Purpose**: Demonstrates a basic AI agent (Python script) publishing commands to a ROS 2 topic.

**Learning Objective**: Students learn how to send commands from an external Python script/AI agent to ROS 2.

**Code Structure**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleAgentPublisher(Node):
    def __init__(self):
        super().__init__('simple_agent_publisher')
        self.publisher_ = self.create_publisher(String, '/agent_commands', 10)
        self.timer = self.create_timer(2.0, self.publish_command) # Publish every 2 seconds
        self.commands = [
            "move forward",
            "turn left",
            "stop",
            "move forward"
        ]
        self.command_index = 0

    def publish_command(self):
        if self.command_index < len(self.commands):
            command = self.commands[self.command_index]
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
            self.get_logger().info(f'Agent publishing command: "{command}"')
            self.command_index += 1
        else:
            self.get_logger().info('All commands published. Shutting down.')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAgentPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select ai_ros_bridge
source install/setup.bash
ros2 run ai_ros_bridge simple_agent_publisher
```

**Expected Output**: Agent publishes commands to `/agent_commands` topic, which can be seen with `ros2 topic echo /agent_commands`.

**Testing Criteria**:
- [ ] Node starts without errors
- [ ] Commands are published to the correct topic
- [ ] Sequence of commands is correct

## Example: LLM Command Bridge (Text to ROS Actions)

**File**: `examples/chapter-2-ros2/ros2_ws/src/ai_ros_bridge/ai_ros_bridge/llm_command_bridge.py`

**Purpose**: Demonstrates an LLM-like bridge that interprets natural language text commands and translates them into ROS 2 actions (or services/topics).

**Learning Objective**: Students learn conceptual integration of LLMs with ROS 2 for higher-level control.

**Code Structure**:
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci # Reusing Fibonacci for demonstration
from std_msgs.msg import String

class LLMCommandBridge(Node):
    def __init__(self):
        super().__init__('llm_command_bridge')
        self.subscription = self.create_subscription(
            String,
            '/llm_input_text',
            self.text_command_callback,
            10)
        self.subscription  # prevent unused variable warning
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci') # Reusing action from Example 6
        self.get_logger().info('LLM Command Bridge ready. Waiting for text commands.')

    def text_command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f'Received text command: "{command_text}"')

        if 'fibonacci' in command_text and 'order' in command_text:
            try:
                order = int(command_text.split('order ')[1].split(' ')[0])
                self.send_fibonacci_goal(order)
            except (ValueError, IndexError):
                self.get_logger().warn('Could not parse Fibonacci order from command.')
        elif 'hello' in command_text:
            self.get_logger().info('Bridge says hello back!')
        else:
            self.get_logger().info(f'Unknown command: {command_text}')

    def send_fibonacci_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Fibonacci goal rejected :(')
            return
        self.get_logger().info('Fibonacci goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Fibonacci Action Result: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**How to Run**:
```bash
# First, ensure the fibonacci_action_server from Example 6 is running in a separate terminal.
# Then, in one terminal
cd examples/chapter-2-ros2/ros2_ws
colcon build --packages-select ai_ros_bridge simple_action
source install/setup.bash
ros2 run ai_ros_bridge llm_command_bridge

# In another terminal, publish text commands
ros2 topic pub /llm_input_text std_msgs/String "data: 'calculate fibonacci order 5'"
ros2 topic pub /llm_input_text std_msgs/String "data: 'hello'"
```

**Expected Output**:
- Bridge node logs received commands.
- If Fibonacci command, action client interacts with server, logs results.

**Testing Criteria**:
- [ ] Bridge node starts and subscribes to `/llm_input_text`
- [ ] Correctly parses "fibonacci order X" command and triggers action
- [ ] Interacts with Fibonacci action server successfully
- [ ] Handles other simple commands (e.g., "hello")
- [ ] Gracefully handles unparseable commands
