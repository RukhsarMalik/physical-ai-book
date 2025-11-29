--- 
id: python-integration
title: Python Integration with rclpy
sidebar_position: 3
---

## 1. Introduction to rclpy

`rclpy` is the official Python client library for ROS 2. It provides a Pythonic interface to all the core ROS 2 functionalities, allowing developers to write nodes, publish/subscribe to topics, provide/request services, and manage actions using familiar Python syntax. `rclpy` sits on top of `rcl` (ROS Client Library), which is a C library providing the core logic, ensuring performance and consistency across different language bindings.

Using `rclpy` allows you to leverage Python's ease of use, extensive libraries for data processing, and rapid prototyping capabilities, making it an excellent choice for many ROS 2 applications, especially in areas like AI, data analysis, and high-level robot control.

## 2. Setting up rclpy Environment

Before you can write `rclpy` code, you need a functional ROS 2 installation. Assuming you have ROS 2 installed (e.g., following the official ROS 2 documentation for your specific OS and distribution), you just need to ensure your development environment is correctly sourced.

**Prerequisites:**
* A working ROS 2 installation (e.g., Humble Hawksbill, Iron Irwini).
* Python 3 installed.

**Installation Steps (usually part of ROS 2 setup):**
`rclpy` is typically installed as part of the standard ROS 2 desktop or base installation. If you installed ROS 2 using the recommended methods, `rclpy` should already be available. You can verify this by trying to import `rclpy` in a Python interpreter after sourcing your ROS 2 environment.

**Sourcing your ROS 2 environment:**
Every time you open a new terminal, you *must* source your ROS 2 installation to make ROS 2 commands and libraries available.

**Try it yourself:**

```bash
# For Linux/macOS (replace humble with your ROS 2 distribution)
source /opt/ros/humble/setup.bash

# For Windows (using PowerShell, replace humble with your ROS 2 distribution)
# . C:\dev\ros2_humble\setup.ps1
```

After sourcing, you should be able to run ROS 2 commands like `ros2 run` and Python scripts that import `rclpy`.

## 3. Creating a Simple Publisher Node

A publisher node sends messages to a topic. Let's create a node that publishes a "Hello ROS 2" message every second to a topic named `/chat_message`.

**File**: `simple_publisher.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the String message type

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher') # Node name
        self.publisher_ = self.create_publisher(String, 'chat_message', 10) # Topic name and QoS
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log what we're publishing
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication
    simple_publisher = SimplePublisher() # Create the node
    rclpy.spin(simple_publisher) # Keep the node alive until Ctrl+C
    simple_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()
```

**Line-by-line Explanation:**
*   `import rclpy` and `from rclpy.node import Node`: Imports the necessary ROS 2 Python client library.
*   `from std_msgs.msg import String`: Imports the `String` message type from the `std_msgs` package, which is used for simple text messages.
*   `class SimplePublisher(Node):`: Defines our node class, inheriting from `rclpy.node.Node`.
*   `super().__init__('simple_publisher')`: Calls the base class constructor and sets the unique name of our node to `simple_publisher`.
*   `self.publisher_ = self.create_publisher(String, 'chat_message', 10)`: Creates a publisher.
    *   `String`: The message type to publish.
    *   `'chat_message'`: The name of the topic.
    *   `10`: The QoS (Quality of Service) history depth, a buffer for messages.
*   `self.timer = self.create_timer(timer_period, self.timer_callback)`: Creates a timer that will call `self.timer_callback` every `timer_period` seconds.
*   `self.timer_callback(self)`: This method is executed by the timer. It creates a `String` message, populates its `data` field, publishes it, and logs the action.
*   `main(args=None)`: The entry point for our ROS 2 Python executable.
*   `rclpy.init(args=args)`: Initializes the `rclpy` library, enabling ROS 2 communication.
*   `rclpy.spin(simple_publisher)`: Keeps the node running until it's explicitly stopped (e.g., by `Ctrl+C`). It processes callbacks from the timer.
*   `simple_publisher.destroy_node()`: Cleans up the node resources.
*   `rclpy.shutdown()`: Shuts down the `rclpy` library.

**Try it yourself:**
Save the code as `simple_publisher.py`. Before running, ensure your ROS 2 environment is sourced.

```bash
# In your terminal, after sourcing ROS 2
python3 simple_publisher.py
```

## 4. Creating a Simple Subscriber Node

A subscriber node listens for messages on a topic. Let's create a node that subscribes to the `/chat_message` topic and prints any messages it receives.

**File**: `simple_subscriber.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the String message type

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') # Node name
        self.subscription = self.create_subscription(
            String,
            'chat_message',
            self.listener_callback,
            10) # Topic name, QoS
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"') # Log the received message

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Line-by-line Explanation:**
*   `self.subscription = self.create_subscription(...)`: Creates a subscriber.
    *   `String`: The message type to subscribe to.
    *   `'chat_message'`: The name of the topic.
    *   `self.listener_callback`: The callback function that will be executed every time a message is received.
    *   `10`: The QoS history depth, matching the publisher.
*   `self.listener_callback(self, msg)`: This method is called when a message is received. It takes the received `msg` object and logs its data.

**Try it yourself:**
Save the code as `simple_subscriber.py`. Before running, ensure your ROS 2 environment is sourced.

```bash
# In your terminal, after sourcing ROS 2
python3 simple_subscriber.py
```

## 5. Running the Nodes Together

To see the publisher and subscriber communicate, you need to run them concurrently in separate terminals.

**Try it yourself:**

1.  **Open Terminal 1:**
    ```bash
    # Source your ROS 2 environment
    source /opt/ros/humble/setup.bash # or your Windows equivalent
    python3 simple_publisher.py
    ```
    This terminal will show messages being published.

2.  **Open Terminal 2:**
    ```bash
    # Source your ROS 2 environment
    source /opt/ros/humble/setup.bash # or your Windows equivalent
    python3 simple_subscriber.py
    ```
    This terminal will show messages being received.

You should observe the `simple_subscriber` terminal displaying the messages published by the `simple_publisher`.

## 6. Troubleshooting Common Issues

*   **"command not found: ros2" or "ImportError: cannot import name 'Node' from 'rclpy.node'"**:
    *   **Cause**: ROS 2 environment is not sourced.
    *   **Solution**: Ensure you run `source /opt/ros/<distro>/setup.bash` (Linux/macOS) or its Windows equivalent in *every new terminal* before running ROS 2 commands or Python scripts.
*   **Nodes not communicating**:
    *   **Cause**: Mismatched topic names, message types, or QoS settings between publisher and subscriber. Or, one of the nodes failed to start.
    *   **Solution**:
        *   Double-check topic names (`ros2 topic list`, `ros2 topic info <topic_name>`).
        *   Verify message types match.
        *   Ensure QoS history depth is compatible (e.g., both 10).
        *   Look for error messages in the terminal where each node is running.
*   **"Address already in use"**:
    *   **Cause**: A previous instance of a ROS 2 process might still be running in the background.
    *   **Solution**: Close all terminals that were running ROS 2 nodes. If the issue persists, try restarting your terminal or machine.
*   **Python syntax errors**:
    *   **Cause**: Typos in your Python code.
    *   **Solution**: Carefully review your code against the examples, paying attention to indentation and syntax. Use a good IDE with Python linting.
