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

**Starter Code** (in `examples/chapter-2-ros2/exercises/exercise_1_my_first_node/starter/`):
- Empty package structure (package.xml, setup.py templates)
- Comments indicating where to add code

**Solution Code** (in `examples/chapter-2-ros2/exercises/exercise_1_my_first_node/solution/`):
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

## Exercise 2: Robot Controller (Publisher/Subscriber)

**Difficulty**: Intermediate
**Time Estimate**: 45-60 minutes
**Prerequisites**: Completed reading through Section 4 (Publishers and Subscribers)

**Learning Objectives**:
- Implement a ROS 2 node that subscribes to commands and publishes robot status
- Use `geometry_msgs/Twist` for robot control
- Implement basic state management within a node

**Instructions** (provided to students):
```
1. Extend the "my_robot_controller" package from Exercise 1.
2. Create a new Python node called "robot_controller" that subscribes to `/cmd_vel` (Twist messages).
3. When a Twist message is received, log the linear and angular velocities.
4. Additionally, publish a `std_msgs/String` message to `/robot_status` indicating "Moving Forward", "Turning Left", "Stopped", etc., based on the received `cmd_vel`.
5. Build and run your new node. Use `ros2 topic pub /cmd_vel geometry_msgs/Twist ...` to test.
```

**Starter Code** (in `examples/chapter-2-ros2/exercises/exercise_2_robot_controller/starter/`):
- `my_robot_controller` package structure
- Skeleton for `robot_controller.py` with basic imports

**Solution Code** (in `examples/chapter-2-ros2/exercises/exercise_2_robot_controller/solution/`):
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)
        self.get_logger().info('Robot Controller node started.')

    def cmd_vel_callback(self, msg):
        status = "Stopped"
        if msg.linear.x > 0:
            status = "Moving Forward"
        elif msg.linear.x < 0:
            status = "Moving Backward"

        if msg.angular.z > 0:
            status = "Turning Left"
        elif msg.angular.z < 0:
            status = "Turning Right"

        status_msg = String()
        status_msg.data = status
        self.publisher_.publish(status_msg)
        self.get_logger().info(f'Received Twist: linear.x={msg.linear.x}, angular.z={msg.angular.z}. Status: {status}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Grading Rubric**:
- Node subscribes to `/cmd_vel` (2 points)
- Logs velocities correctly (2 points)
- Publishes status to `/robot_status` (2 points)
- Status message accurately reflects robot movement (2 points)
- Code is clean and runs without errors (2 points)

## Exercise 3: URDF Modification (Adding a Sensor)

**Difficulty**: Intermediate
**Time Estimate**: 45-60 minutes
**Prerequisites**: Completed reading through Section 6 (URDF Basics)

**Learning Objectives**:
- Modify an existing URDF file
- Add a new link and joint to a robot description
- Understand how to position components relative to existing links

**Instructions** (provided to students):
```
1. Take the `simple_arm.urdf` from the code examples.
2. Add a new `link` to represent a simple camera sensor.
3. Add a new `joint` to attach this camera sensor to the end of `link1`.
4. Position the camera appropriately (e.g., at the end of `link1`, facing forward).
5. Visualize your modified URDF in RViz to verify its structure.
```

**Starter Code** (in `examples/chapter-2-ros2/exercises/exercise_3_urdf_modification/starter/`):
- Copy of `simple_arm.urdf`

**Solution Code** (in `examples/chapter-2-ros2/exercises/exercise_3_urdf_modification/solution/`):
```xml
<?xml version="1.0"?>
<robot name="simple_arm_with_camera">

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

  <!-- New Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- New Camera Joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="link1"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.275"/> <!-- Position at end of link1 -->
  </joint>

</robot>
```

**Grading Rubric**:
- New camera link and joint are added (3 points)
- Camera is correctly attached to `link1` (3 points)
- Modified URDF is valid (2 points)
- Visualization in RViz is correct (2 points)

## Exercise 4: Multi-Node System (Launch File)

**Difficulty**: Advanced
**Time Estimate**: 60-90 minutes
**Prerequisites**: Completed reading through Section 8 (Launch Files)

**Learning Objectives**:
- Create a launch file to start multiple nodes with different parameters
- Use node remapping and namespace concepts
- Orchestrate a simple robotic system from a single command

**Instructions** (provided to students):
```
1. Create a new ROS 2 Python package called "my_robot_system".
2. Create a launch file in this package that starts two instances of the `talker` node (from Example 2).
3. Each `talker` node should publish to a different topic (e.g., `/robot_voice1` and `/robot_voice2`).
4. Each `talker` node should have a different name.
5. Start a single `listener` node (from Example 3) that subscribes to both `/robot_voice1` and `/robot_voice2` (you might need to modify the listener to subscribe to multiple topics or create two listeners).
6. Bonus: Try to pass a unique parameter to each `talker` node to modify its message content (e.g., "Hello from Talker A!").
7. Build and run your launch file. Verify communication with `ros2 topic echo` and `ros2 node list`.
```

**Starter Code** (in `examples/chapter-2-ros2/exercises/exercise_4_multi_node_system/starter/`):
- Empty package structure for `my_robot_system`
- Example `talker` and `listener` nodes (from code examples)

**Solution Code** (in `examples/chapter-2-ros2/exercises/exercise_4_multi_node_system/solution/`):
```python
# my_robot_system/launch/multi_talker_listener.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker_listener',
            executable='talker',
            name='talker_a',
            output='screen',
            remappings=[
                ('topic', '/robot_voice1')
            ],
            parameters=[
                {'my_parameter': 'Hello from Talker A!'} # Assuming talker node can read this
            ]
        ),
        Node(
            package='talker_listener',
            executable='talker',
            name='talker_b',
            output='screen',
            remappings=[
                ('topic', '/robot_voice2')
            ],
            parameters=[
                {'my_parameter': 'Hello from Talker B!'} # Assuming talker node can read this
            ]
        ),
        Node(
            package='talker_listener',
            executable='listener',
            name='listener_all',
            output='screen',
            remappings=[
                ('topic', '/robot_voice1') # Listener will need to subscribe to both, or have two listeners
            ]
        ), # This listener only subscribes to voice1. For both, would need another listener node, or modify listener itself.
        Node(
            package='talker_listener',
            executable='listener',
            name='listener_all_2',
            output='screen',
            remappings=[
                ('topic', '/robot_voice2')
            ]
        )
    ])
```

**Grading Rubric**:
- Launch file starts two talker nodes (2 points)
- Talker nodes publish to different topics (2 points)
- Listener node(s) subscribe to appropriate topics (2 points)
- Communication is verified (2 points)
- Bonus: Parameters are passed and used (2 points)

## Exercise 5: AI Integration (Simple Command Interface)

**Difficulty**: Advanced
**Time Estimate**: 60-90 minutes
**Prerequisites**: Completed reading through Section 9 (AI-ROS Integration)

**Learning Objectives**:
- Create a simple AI interface that publishes commands to ROS 2
- Use Python's `input()` for text-based commands (simulating LLM input)
- Integrate with an existing ROS 2 subscriber (e.g., `RobotController` from Exercise 2)

**Instructions** (provided to students):
```
1. Create a Python script (outside of a ROS 2 node initially) that prompts the user for text commands (e.g., "move forward", "turn left").
2. Use this script to publish `std_msgs/String` messages to a ROS 2 topic (e.g., `/ai_commands`).
3. Modify your `RobotController` node from Exercise 2 to subscribe to `/ai_commands` and act on these commands (e.g., set `Twist` messages based on text input and publish to `/cmd_vel`).
4. Ensure your `RobotController` still publishes its status to `/robot_status`.
5. Run your AI script and the `RobotController` node. Test with various text commands.
```

**Starter Code** (in `examples/chapter-2-ros2/exercises/exercise_5_ai_integration/starter/`):
- Skeleton for AI command script
- `RobotController` node from Exercise 2 solution

**Solution Code** (in `examples/chapter-2-ros2/exercises/exercise_5_ai_integration/solution/`):

**AI Command Publisher (ai_command_input.py)**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class AICommandInput(Node):
    def __init__(self):
        super().__init__('ai_command_input')
        self.publisher_ = self.create_publisher(String, '/ai_commands', 10)
        self.get_logger().info('AI Command Input node ready. Type commands and press Enter.')

    def publish_command(self, command_text):
        msg = String()
        msg.data = command_text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing AI command: "{command_text}"')

def main(args=None):
    rclpy.init(args=args)
    node = AICommandInput()

    while rclpy.ok():
        try:
            command = input("Enter command (or 'quit'): ")
            if command.lower() == 'quit':
                break
            node.publish_command(command)
        except EOFError:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Modified Robot Controller (robot_controller.py)**:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Subscribe to AI commands
        self.ai_command_subscription = self.create_subscription(
            String,
            '/ai_commands',
            self.ai_command_callback,
            10)
        self.ai_command_subscription # prevent unused variable warning

        # Publisher for cmd_vel (to control robot)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for robot status
        self.status_publisher_ = self.create_publisher(String, '/robot_status', 10)
        self.get_logger().info('Robot Controller node started, awaiting AI commands.')

    def ai_command_callback(self, msg):
        command_text = msg.data.lower()
        self.get_logger().info(f'Received AI command: "{command_text}"')

        twist_msg = Twist()
        robot_status = "Unknown Command"

        if "move forward" in command_text:
            twist_msg.linear.x = 0.5
            robot_status = "Moving Forward"
        elif "move backward" in command_text:
            twist_msg.linear.x = -0.5
            robot_status = "Moving Backward"
        elif "turn left" in command_text:
            twist_msg.angular.z = 0.5
            robot_status = "Turning Left"
        elif "turn right" in command_text:
            twist_msg.angular.z = -0.5
            robot_status = "Turning Right"
        elif "stop" in command_text:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            robot_status = "Stopped"

        self.cmd_vel_publisher_.publish(twist_msg)
        status_msg = String()
        status_msg.data = robot_status
        self.status_publisher_.publish(status_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}. Status: {robot_status}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Grading Rubric**:
- AI command publisher script sends `String` messages (2 points)
- `RobotController` subscribes to AI commands (2 points)
- `RobotController` translates text commands to `Twist` messages (3 points)
- `RobotController` publishes status correctly (2 points)
- System responds as expected to various commands (1 point)
