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

## Error: "Failed to create publisher: rcl error in rclpy_pybind_c_destroy_rcl_publisher()"

**Symptom**: Node crashes or fails to create a publisher with an error related to `rclpy_pybind_c_destroy_rcl_publisher()`.

**Likely Causes**:
1.  **Multiple nodes with same name**: If you have multiple nodes with the same name running or trying to start.
2.  **Improper shutdown**: Previous node instances were not cleanly shut down.
3.  **DDS configuration issues**: Underlying Data Distribution Service (DDS) might be misconfigured.

**Solution Steps**:
```bash
# Step 1: Ensure unique node names.
# If running multiple nodes, give each a unique name or run in different namespaces.
# For example, in Python:
# super().__init__('my_unique_node_name')

# Step 2: Ensure clean shutdown.
# Always ensure rclpy.spin() is followed by node.destroy_node() and rclpy.shutdown().

# Step 3: Check for ghost nodes.
# Use `ros2 node list` to see if any nodes with the same name are already running.
# If so, try killing the process or restarting your terminal.

# Step 4: Reset ROS 2 environment.
# Sometimes a full reset helps. Close all terminals, then reopen and re-source your setup file.
```

## Error: "Failed to create subscription: rcl error in rclpy_pybind_c_destroy_rcl_subscription()"

**Symptom**: Similar to publisher error, node crashes or fails to create a subscription.

**Likely Causes**:
1.  **Duplicate subscriptions**: Trying to create multiple subscriptions to the same topic with the same callback from within the same node instance without proper management.
2.  **Improper shutdown**: Previous node instances were not cleanly shut down.

**Solution Steps**:
```bash
# Step 1: Ensure unique subscription handling within a node.
# A node can subscribe to the same topic multiple times if needed, but ensure each subscription object is managed correctly.
# Check for accidental duplicate `create_subscription` calls in `__init__`.

# Step 2: Ensure clean shutdown.
# As with publishers, proper shutdown is critical.
# Always ensure rclpy.spin() is followed by node.destroy_node() and rclpy.shutdown().

# Step 3: Check for ghost nodes/topics.
# Use `ros2 node list` and `ros2 topic list` to ensure no stale resources are present.
```

## Error: "URDF parse error: could not open file"

**Symptom**: When trying to load a URDF file, an error indicates the file cannot be opened.

**Likely Causes**:
1.  **Incorrect path**: The path to the URDF file in the launch file or command is wrong.
2.  **File not found**: The URDF file does not exist at the specified location.
3.  **Permissions**: Lack of read permissions for the file.

**Solution Steps**:
```bash
# Step 1: Verify file path.
# Use `ls` or `cat` to confirm the file exists at the given path.
ls /path/to/your_robot.urdf

# Step 2: Check launch file arguments.
# Ensure the `model` argument in your launch file correctly points to the URDF.
# Example: ros2 launch robot_description display_robot.launch.py model:=simple_arm.urdf

# Step 3: Ensure sourcing is correct for package_share_directory.
# If using ament_index_python.packages.get_package_share_directory, ensure your workspace is sourced.
source install/setup.bash
```

## Error: "Failed to load plugin: rviz_default_plugins/RobotModel"

**Symptom**: RViz starts but cannot display the robot model.

**Likely Causes**:
1.  **`robot_state_publisher` not running**: The `robot_state_publisher` node is essential for publishing the robot's TF (transforms) based on the URDF.
2.  **Incorrect `robot_description` parameter**: The `robot_description` parameter in the ROS 2 parameter server is not set correctly or contains an invalid URDF string.
3.  **Missing dependencies**: `rviz_default_plugins` or other necessary packages are not installed.

**Solution Steps**:
```bash
# Step 1: Ensure `robot_state_publisher` is launched.
# Your launch file should include:
# Node(
#     package='robot_state_publisher',
#     executable='robot_state_publisher',
#     name='robot_state_publisher',
#     output='screen',
#     parameters=[{'robot_description': robot_description_content}]
# )

# Step 2: Verify `robot_description` parameter.
# Use `ros2 param get /robot_state_publisher robot_description` to inspect the parameter.
# It should contain the full URDF XML as a string.

# Step 3: Check RViz configuration.
# In RViz, ensure the 'RobotModel' display is enabled and its 'Robot Description' property is set to 'robot_description'.

# Step 4: Install missing RViz plugins.
# sudo apt install ros-humble-rviz-default-plugins
```

## Error: `colcon build` fails with "Python.h: No such file or directory" or similar Python header errors

**Symptom**: `colcon build` fails for Python packages with errors related to missing Python header files.

**Likely Causes**:
1.  **Missing Python development headers**: The necessary development headers for Python are not installed.
2.  **Incorrect Python environment**: Using a different Python version than the one ROS 2 expects, or issues with virtual environments.

**Solution Steps**:
```bash
# Step 1: Install Python development headers.
# For Python 3.10 (Ubuntu 22.04 default):
sudo apt update
sudo apt install python3.10-dev

# If you are using a different Python version, adjust accordingly (e.g., python3.8-dev).

# Step 2: Ensure correct Python environment is active.
# If using a virtual environment, activate it before building.
# Make sure the Python version used for ROS 2 is consistent.
```

## Warning: "[WARN] [rclpy]: detected ROS 1 parameter /use_sim_time on the global parameter server, ignoring it."

**Symptom**: A warning message about `use_sim_time` related to ROS 1 appears when starting ROS 2 nodes.

**Likely Causes**:
1.  **Stale ROS 1 environment variables**: Some ROS 1 environment variables might still be sourced, causing ROS 2 to detect them.
2.  **Lingering ROS 1 setup**: Previous ROS 1 installations or overlays are interfering.

**Solution Steps**:
```bash
# Step 1: Ensure only ROS 2 environment is sourced.
# In your ~/.bashrc or equivalent, comment out or remove any ROS 1 sourcing lines.
# Example: # source /opt/ros/noetic/setup.bash

# Step 2: Open a fresh terminal.
# Close all terminals and open a new one to ensure a clean environment where only your ROS 2 setup file is sourced.
```

## Problem: Nodes cannot communicate (e.g., talker/listener don't see each other)

**Symptom**: ROS 2 nodes (e.g., a publisher and subscriber) start without errors but do not exchange messages.

**Likely Causes**:
1.  **Incorrect Topic Name**: Publisher and subscriber are using different topic names.
2.  **Different QoS Settings**: Incompatible Quality of Service (QoS) settings between publisher and subscriber.
3.  **Network/DDS Issues**: Multicast not enabled, firewall blocking, or different ROS_DOMAIN_ID.
4.  **Nodes in different namespaces**: If nodes are launched in different namespaces, they won't see each other's topics directly.

**Solution Steps**:
```bash
# Step 1: Verify Topic Names.
# Use `ros2 topic list` and `ros2 topic info <topic_name>` to confirm the exact topic name being used by both nodes.

# Step 2: Check QoS Settings (if explicitly set).
# For beginners, usually default QoS works. If you are modifying QoS, ensure compatibility.
# Example: history depth, reliability.

# Step 3: Verify Network and ROS_DOMAIN_ID.
# Ensure multicast is enabled on your network adapter.
# Check `echo $ROS_DOMAIN_ID` in all terminals running ROS 2 nodes. They must be the same.
# Temporarily disable firewall (if applicable) to rule it out.

# Step 4: Check Node Namespaces.
# If using launch files, ensure nodes are in the same namespace or topics are remapped correctly.
# Use `ros2 node info <node_name>` to see its namespace and topics.
```

## Problem: `ros2 launch` fails to find package or executable

**Symptom**: When running `ros2 launch my_package my_launch_file.launch.py`, the command fails, stating it cannot find the package or executable.

**Likely Causes**:
1.  **Workspace not sourced**: The ROS 2 workspace's `setup.bash` (or `setup.zsh`, etc.) has not been sourced in the current terminal session.
2.  **Package not built**: The target package has not been built successfully with `colcon build`.
3.  **Incorrect package/executable name**: A typo in the package or executable name in the launch command or launch file.

**Solution Steps**:
```bash
# Step 1: Source your ROS 2 workspace.
cd /path/to/your/ros2_ws
source install/setup.bash

# Step 2: Build the package.
colcon build --packages-select <your_package_name>

# Step 3: Verify package and executable names.
# Use `ros2 pkg list` to see installed packages.
# Use `ros2 pkg executables <your_package_name>` to see available executables.
```

