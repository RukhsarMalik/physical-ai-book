---
id: gazebo-basics
title: Gazebo Fundamentals
sidebar_position: 2
---

## 1. Physics Simulation: The Core of Gazebo

At its heart, Gazebo is a **physics simulator**. This means it calculates how objects interact in a virtual 3D world based on physical laws. This capability is crucial for robotics, as it allows us to test robot locomotion, manipulation, and interaction with objects and environments without needing physical hardware.

**Key physics concepts in Gazebo:**

*   **Gravity**: Gazebo applies gravitational forces to all simulated objects by default, simulating realistic falling and acceleration. You can configure the gravity vector in your world files.
*   **Collisions**: Gazebo accurately detects and models collisions between objects. This is vital for robot safety (preventing self-collisions or collisions with the environment) and for tasks involving physical contact, such as grasping. Collision properties are defined for each link in a robot's URDF (or model SDF).
*   **Joints**: The connections between a robot's links are defined by joints (as discussed in the URDF module). Gazebo uses these definitions to simulate realistic joint kinematics and dynamics, allowing you to apply forces or torques to joints and observe the resulting motion.
*   **Inertial Properties**: Each link in a robot model is assigned inertial properties (mass, inertia matrix). These properties dictate how the link responds to forces and torques, ensuring that the robot's movement in simulation is physically accurate.

Gazebo typically uses specialized physics engines under the hood, such as ODE (Open Dynamics Engine), Bullet, or DART, which handle the complex calculations of rigid body dynamics and collision detection.

## 2. World and Model Files: Defining the Simulation Environment

Gazebo simulations are defined by **world files** (with a `.world` extension) and **model files**.

*   **World Files (.world)**:
    A world file is an XML-based file that describes the entire simulation environment. It's like a blueprint for your virtual world. A world file can define:
    *   **Physics Engine**: Which physics engine to use (e.g., ODE, Bullet).
    *   **Gravity**: The direction and magnitude of gravity.
    *   **Lights**: Ambient, directional, and spot lights to illuminate the scene.
    *   **Ground Plane**: A flat surface for objects to rest on.
    *   **Models**: It references and places various robot and object models within the scene.
    *   **Plugins**: Gazebo plugins that add custom functionality to the world (e.g., controlling a specific sensor behavior).

    ```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="my_simple_world">
        <gravity>0 0 -9.8</gravity>
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>
        <include>
          <uri>model://box</uri>
          <name>my_box</name>
          <pose>0 0 0.5 0 0 0</pose>
        </include>
        <!-- Robot models are often added via ROS launch files -->
      </world>
    </sdf>
    ```

*   **Model Files**:
    Models are individual objects within the Gazebo world. A model can be a simple cube, a complex robot, a tree, or a building. Gazebo models are typically defined using **SDF (Simulation Description Format)**, which is Gazebo's native XML format for describing robots and objects. While URDF can describe a robot, SDF is more comprehensive and can describe an entire environment, including static objects, lights, and sensors that are not part of a robot.
    Gazebo can import URDF files, converting them internally to SDF.

    A model file (often residing in a `~/.gazebo/models` directory or a ROS package) specifies:
    *   **Links**: The rigid body parts (like in URDF).
    *   **Joints**: Connections between links.
    *   **Visuals**: How the model looks (color, textures, mesh files).
    *   **Collisions**: How the model interacts physically.
    *   **Inertials**: Mass properties.
    *   **Sensors**: Simulated sensors attached to the model.
    *   **Plugins**: Model-specific Gazebo plugins.

## 3. Launching Simulations: `ros2 launch` and Gazebo

In ROS 2, you typically launch Gazebo simulations using **launch files**. These are Python or XML files that orchestrate the starting of multiple ROS 2 nodes and external processes like Gazebo.

**Key steps in a Gazebo launch file:**

1.  **Start Gazebo Server and Client**: Launch the `gzserver` (the Gazebo physics server) and `gzclient` (the graphical user interface) processes. You usually specify a world file for `gzserver`.
2.  **Spawn the Robot**: If you have a robot defined by a URDF or SDF, you'll use a `spawn_entity` node from the `gazebo_ros` package to insert your robot into the simulated world. This node needs the robot's description (e.g., from `robot_description` parameter) and its initial pose.
3.  **Robot State Publisher**: Launch the `robot_state_publisher` node to publish the robot's current joint states and its kinematic tree (TF transforms). This is crucial for RViz2 visualization.
4.  **Joint State Publisher (GUI)**: Often, a `joint_state_publisher_gui` node is launched, allowing you to manually control the robot's joints using sliders in a GUI, which helps in debugging and understanding the robot's movement.

**Example Launch Command (conceptual):**

```bash
# Assuming you have a ROS 2 package with a launch file named 'my_robot_simulation.launch.py'
ros2 launch my_robot_package my_robot_simulation.launch.py
```

**Content of a conceptual `my_robot_simulation.launch.py`:**

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Define your robot's URDF file
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_package'),
        'urdf',
        urdf_file_name
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}],
    )

    # Gazebo launch
    gazebo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items(), # Use an empty world or your custom world
    )

    # Spawn your robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_ros_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```

This launch file would first start Gazebo (with an `empty.world` in this case), then the `robot_state_publisher` to process your `URDF`, and finally spawn your robot model into Gazebo. This powerful orchestration allows for complex simulation setups with ease.
