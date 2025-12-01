# Code Example Generator Skill

**Skill Type**: Code Generation
**Context**: ROS 2, Python, C++ for Robotics
**Reusable**: Yes - Can be used for any robotics codebase

## Purpose

Generate production-quality, well-documented code examples for robotics textbook with ROS 2, Python, and C++ that students can learn from and run.

## When to Use

- Creating ROS 2 node examples
- Demonstrating robotics algorithms
- Writing URDF/SDF models
- Generating launch files
- Creating configuration YAML files

## Input Requirements

```yaml
framework: "ROS 2 Humble | Gazebo | Isaac Sim"
language: "Python | C++ | YAML | XML"
concept: "What the code demonstrates (e.g., 'Publisher/Subscriber pattern')"
complexity: "beginner | intermediate | advanced"
file_type: "node | launch_file | config | urdf | sdf"
```

## Output Format

### For Python ROS 2 Nodes:
```python
#!/usr/bin/env python3
"""
[Brief description of what this node does]

Learning objectives:
- [Objective 1]
- [Objective 2]
"""

import rclpy
from rclpy.node import Node
# ... imports

class ExampleNode(Node):
    """[Docstring explaining the node's purpose]"""

    def __init__(self):
        super().__init__('node_name')
        # [Inline comments explaining initialization]

    def callback(self, msg):
        """[Docstring for callback]"""
        # [Inline comments explaining logic]

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality Standards

1. **Code Quality**
   - Follow PEP 8 (Python) or Google C++ Style Guide
   - No magic numbers - use named constants
   - Proper error handling
   - Type hints for Python

2. **Documentation**
   - Module docstring at top
   - Class/function docstrings
   - Inline comments for complex logic
   - README with usage instructions

3. **Testing**
   - Code must be runnable as-is
   - Include expected output
   - Mention dependencies

4. **Educational Value**
   - Show best practices
   - Demonstrate one concept clearly
   - Include common pitfall warnings

## Example Invocation

```
Use the code-example-generator skill to create:
- Framework: ROS 2 Humble
- Language: Python
- Concept: Service client for robot navigation
- Complexity: beginner
- File type: node
```

## Templates Available

- `ros2_python_node.py` - Basic ROS 2 Python node
- `ros2_launch_file.py` - ROS 2 launch file
- `ros2_params.yaml` - Parameter configuration
- `urdf_robot.urdf` - Basic URDF model
- `gazebo_world.sdf` - Gazebo world file

## Success Criteria

- ✅ Code runs without errors
- ✅ Follows language best practices
- ✅ Clearly demonstrates concept
- ✅ Well-documented
- ✅ Appropriate complexity level
