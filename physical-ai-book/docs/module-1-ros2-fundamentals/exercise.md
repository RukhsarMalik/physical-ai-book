---
id: exercise
title: Practical Exercise - Build a ROS 2 Package
sidebar_position: 5
---

## 1. Exercise Overview

This hands-on exercise will guide you through the process of creating a basic ROS 2 workspace, building a custom package, and implementing simple publisher and subscriber nodes in Python. By the end of this exercise, you will have a working ROS 2 system that demonstrates inter-node communication.

**What you'll build:**
*   A ROS 2 workspace: The standardized directory structure for developing ROS 2 applications.
*   A custom ROS 2 package: A container for your nodes, launch files, and other resources.
*   A Python publisher node (`talker`): Publishes a "Hello World" message.
*   A Python subscriber node (`listener`): Subscribes to the message and prints it.

**Prerequisites:**
*   A working ROS 2 installation (e.g., Humble, Iron).
*   A Linux environment (Ubuntu is recommended) with a terminal.
*   Basic familiarity with Python and command-line operations.

**Estimated time:** 30-45 minutes.

## 2. Step 1: Create a ROS 2 Workspace

A ROS 2 workspace is a collection of packages where you develop, build, and install your ROS 2 code. It provides a structured environment for managing your projects.

**Commands:**

```bash
# Create a new directory for your workspace
mkdir -p ~/ros2_ws/src

# Navigate into the src directory
cd ~/ros2_ws/src
```

**Folder structure explanation:**
*   `~/ros2_ws`: This is your workspace root directory.
*   `~/ros2_ws/src`: This directory will contain all your ROS 2 packages.

## 3. Step 2: Create a ROS 2 Package

A ROS 2 package is the basic unit of organization for ROS 2 code. It bundles related nodes, libraries, launch files, and other resources.

**Commands:**

```bash
# Ensure you are in the ~/ros2_ws/src directory
cd ~/ros2_ws/src

# Create a new Python package named 'my_ros2_package'
# --build-type ament_python specifies it's a Python package
ros2 pkg create --build-type ament_python my_ros2_package
```

**Package structure:**
After running the command, you will find a new directory `~/ros2_ws/src/my_ros2_package`. Inside it, you'll see:
*   `my_ros2_package/my_ros2_package/`: A Python module with an `__init__.py` file. This is where your Python nodes will live.
*   `setup.py`: A standard Python setup file for installing your package.
*   `package.xml`: Contains metadata about your package (name, description, dependencies).

## 4. Step 3: Write the Publisher Node (`talker`)

Now, let's create the Python code for our publisher node. This node will publish messages to a topic.

**Create Python file:**
Navigate to the Python module directory of your package and create a file named `talker_node.py`.

```bash
cd ~/ros2_ws/src/my_ros2_package/my_ros2_package
touch talker_node.py
```

**Complete code for `talker_node.py`:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.publisher_ = self.create_publisher(String, 'chat_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback) # Publish every 0.5 seconds
        self.i = 0
        self.get_logger().info('Talker Node has started, publishing messages...')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Talker! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    rclpy.spin(node) # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
This code is very similar to the `simple_publisher.py` example from the "Python Integration" page. It creates a node named `talker_node` that publishes `String` messages to the `chat_topic` topic every 0.5 seconds.

## 5. Step 4: Write the Subscriber Node (`listener`)

Next, we'll create the Python code for our subscriber node. This node will listen for messages on the same topic and print them.

**Create Python file:**
In the same directory (`~/ros2_ws/src/my_ros2_package/my_ros2_package`), create a file named `listener_node.py`.

```bash
cd ~/ros2_ws/src/my_ros2_package/my_ros2_package
touch listener_node.py
```

**Complete code for `listener_node.py`:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.subscription = self.create_subscription(
            String,
            'chat_topic',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info('Listener Node has started, subscribing to messages...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    rclpy.spin(node) # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
This code is also very similar to the `simple_subscriber.py` example. It creates a node named `listener_node` that subscribes to the `chat_topic` topic and prints the received `String` messages.

## 6. Step 5: Update `setup.py`

For ROS 2 to know about your new executable nodes, you need to update your package's `setup.py` file to include entry points for them.

**Commands:**
Navigate back to your package's root directory:

```bash
cd ~/ros2_ws/src/my_ros2_package
```

Open `setup.py` (you can use a text editor like `nano` or `gedit` or directly modify it via the agent if you have editing tools):

```python
from setuptools import find_packages, setup

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['my_ros2_package']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name', # <--- REPLACE with your name
    maintainer_email='your_email@example.com', # <--- REPLACE with your email
    description='A simple ROS 2 package for publisher-subscriber example',
    license='Apache-2.0', # <--- Choose appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_ros2_package.talker_node:main',
            'listener = my_ros2_package.listener_node:main',
        ],
    },
)
```

**Important Modifications:**
*   **`maintainer` and `maintainer_email`**: Replace `"your_name"` and `"your_email@example.com"` with your actual information.
*   **`entry_points`**: This is the crucial part. We've added two entries:
    *   `'talker = my_ros2_package.talker_node:main'`: Maps the executable `talker` to the `main` function in `talker_node.py` within the `my_ros2_package` module.
    *   `'listener = my_ros2_package.listener_node:main'`: Maps the executable `listener` to the `main` function in `listener_node.py` within the `my_ros2_package` module.

## 7. Step 6: Build the Package

Before you can run your new nodes, you need to build your workspace. This process compiles your code (if C++), installs Python dependencies, and makes your package discoverable by ROS 2.

**Commands:**
Navigate to the root of your workspace:

```bash
cd ~/ros2_ws
```

Build the workspace using `colcon build`:

```bash
colcon build
```

**Source the workspace:**
After building, you must *source* the workspace's setup file. This adds your package's executables and libraries to your environment. Do this in *every new terminal* you open to use your package.

```bash
# Source the ROS 2 environment first (if not already sourced)
source /opt/ros/humble/setup.bash # (replace humble with your ROS 2 distro)

# Then source your workspace
source install/setup.bash
```

## 8. Step 7: Run the Nodes

Now that everything is built and sourced, you can run your publisher and subscriber nodes.

**Terminal Commands:**
Open **two separate terminals**. In *each* terminal, follow these steps:
1.  **Source ROS 2 and your workspace**:
    ```bash
    source /opt/ros/humble/setup.bash # (replace humble)
    source ~/ros2_ws/install/setup.bash
    ```
2.  **In Terminal 1 (Publisher):**
    ```bash
    ros2 run my_ros2_package talker
    ```
3.  **In Terminal 2 (Subscriber):**
    ```bash
    ros2 run my_ros2_package listener
    ```

**Expected output:**
*   **Terminal 1 (Publisher)**: You will see messages like `[INFO] [talker_node]: Published: "Hello from Talker! Count: 0"`, incrementing the count.
*   **Terminal 2 (Subscriber)**: You will see messages like `[INFO] [listener_node]: Heard: "Hello from Talker! Count: 0"`, also incrementing.

**Verification steps:**
The successful exchange of messages between the `talker` and `listener` nodes confirms that your ROS 2 workspace, package, and nodes are correctly configured and communicating.

## 9. Step 8: Experiment and Modify

Now that you have a basic working system, try experimenting!

**Suggestions for modifications:**
*   Change the message content in `talker_node.py`.
*   Modify the publishing rate in `talker_node.py`.
*   Add more nodes to the same package or create a new package.
*   Change the topic name (remember to change it in both publisher and subscriber).

**Challenge exercises:**
*   Can you make the `listener_node` only print messages when the `Count` is an even number?
*   Implement a third node that subscribes to `chat_topic` and publishes to a *new* topic.

## 10. Troubleshooting Common Issues

*   **`ros2: command not found` or `python3: No module named rclpy`**:
    *   **Solution**: You haven't sourced your ROS 2 environment (and possibly your workspace) in the current terminal. Run `source /opt/ros/humble/setup.bash` (and `source ~/ros2_ws/install/setup.bash`) again.
*   **Node not found (e.g., `ros2 run my_ros2_package talker` fails)**:
    *   **Solution**:
        *   Did you build your workspace (`colcon build`)?
        *   Did you source your workspace (`source install/setup.bash`)?
        *   Are the `entry_points` in your `setup.py` correct?
*   **Nodes running but no communication**:
    *   **Solution**:
        *   Check topic names and message types match exactly (`ros2 topic list`, `ros2 topic info /chat_topic`).
        *   Verify both nodes are actually running (check their terminal outputs for `started` messages).
        *   Ensure QoS settings are compatible (default 10 is usually fine).
*   **Python syntax errors**:
    *   **Solution**: Use a Python linter or run `python3 -m py_compile your_node.py` to check for basic syntax errors before running with ROS 2.
