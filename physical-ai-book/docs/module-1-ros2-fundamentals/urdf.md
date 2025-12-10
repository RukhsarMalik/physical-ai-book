---
id: urdf
title: URDF Format Explained
sidebar_position: 4
---

## 1. Introduction to URDF (Unified Robot Description Format)

The Unified Robot Description Format (URDF) is an XML format used in ROS 2 (and ROS 1) to describe all aspects of a robot. It's a standard way to represent the kinematic and dynamic properties of a robot, as well as its visual and collision characteristics. Essentially, a URDF file defines the physical structure of a robot, including its joints, links, sensors, and other attached elements, in a machine-readable format.

While URDF is powerful, it's important to note its limitations:
*   **Single Robot Description**: A URDF file describes a single robot. For multi-robot systems or complex environments, additional tools like SDF (Simulation Description Format) or Gazebo models are often used.
*   **Static Structure**: URDF is primarily for static descriptions of a robot. For dynamic reconfigurations (e.g., a robot changing its end-effector), dynamic URDF generation or other mechanisms are needed.

## 2. Why URDF is Important for Robotics

URDF serves as a crucial backbone for many robotics applications, making it an indispensable tool for physical AI development:

*   **Simulation**: URDF files are used by physics simulators (like Gazebo) to accurately model the robot's kinematics, dynamics, and interactions with its environment. This allows for testing and development of AI algorithms in a safe, virtual setting before deployment on physical hardware.
*   **Visualization**: Tools like RViz (ROS Visualization) use URDF to display a 3D model of the robot, showing its current pose, sensor data, and planned movements. This is vital for debugging, monitoring, and understanding robot behavior.
*   **Motion Planning**: Motion planning libraries (e.g., MoveIt 2) rely on the robot's kinematic and dynamic description from URDF to calculate collision-free paths for manipulators and mobile bases.
*   **Robot Control**: The joint limits, types, and inertial properties defined in URDF are essential inputs for robot controllers that govern the movement of the robot's actuators.
*   **Hardware Abstraction**: URDF provides a standardized way to represent robots, allowing software components to be more generic and reusable across different physical robot platforms, as long as their URDFs are accurate.

## 3. URDF XML Structure

A URDF file is an XML document with a root `<robot>` element. Inside this, you define the fundamental building blocks of your robot: `<link>` elements for the physical segments and `<joint>` elements for the connections between them.

**Basic Syntax:**
```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Define links (physical parts) -->
  <link name="base_link">
    <!-- Visual, collision, and inertial properties -->
  </link>

  <link name="arm_link">
    <!-- Visual, collision, and inertial properties -->
  </link>

  <!-- Define joints (connections between links) -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <!-- Other joint properties -->
  </joint>

</robot>
```

**Key Elements:**
*   `<robot>`: The root element, encapsulating the entire robot description. It must have a `name` attribute.
*   `<link>`: Represents a rigid body part of the robot (e.g., a chassis, an arm segment, a wheel).
*   `<joint>`: Describes a kinematic and dynamic connection between two links.

## 4. Links (Robot Body Parts)

A `<link>` element defines a rigid body segment of the robot. It can contain several sub-elements describing its physical and visual properties.

**Definition and Attributes:**
The primary attribute for a link is its `name`.

**Sub-elements:**
*   `<visual>`: Describes the visual properties of the link. This includes the geometry (e.g., `<box>`, `<cylinder>`, `<sphere>`, or `<mesh>` for 3D models) and its material properties (color, texture). This is what you see in RViz.
    ```xml
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    ```
*   `<collision>`: Defines the collision properties of the link. This geometry is used by physics engines for collision detection. It can be simpler than the visual geometry for computational efficiency.
    ```xml
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    ```
*   `<inertial>`: Specifies the mass properties of the link, crucial for physics simulation. It includes the mass (`mass`) and the inertia matrix (`ixx`, `ixy`, etc., or a simplified `inertia` element for common shapes).
    ```xml
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
    ```

## 5. Joints (Connections Between Links)

A `<joint>` element describes the kinematic and dynamic relationship between a `<parent>` link and a `<child>` link.

**Key Attributes:**
*   `name`: A unique identifier for the joint.
*   `type`: Defines the degrees of freedom (DOF) of the joint. Common types include:
    *   `revolute`: A single rotational DOF around an axis, with upper and lower limits.
    *   `continuous`: A single rotational DOF around an axis, without limits.
    *   `prismatic`: A single translational DOF along an axis, with upper and lower limits.
    *   `fixed`: No DOF, effectively rigidly attaching the child link to the parent.
    *   `planar`: 2 translational DOFs and 1 rotational DOF in a plane.
    *   `floating`: 3 translational DOFs and 3 rotational DOFs (a full 6-DOF joint).

**Sub-elements:**
*   `<parent link="parent_link_name"/>`: Specifies the parent link of the joint.
*   `<child link="child_link_name"/>`: Specifies the child link of the joint.
*   `<origin xyz="X Y Z" rpy="ROLL PITCH YAW"/>`: Defines the transformation (position and orientation) of the child link's coordinate frame relative to the parent link's coordinate frame. `xyz` is in meters, `rpy` (roll, pitch, yaw) is in radians.
*   `<axis xyz="X Y Z"/>`: For rotational or prismatic joints, defines the axis of motion relative to the joint frame.
*   `<limit lower="LOWER" upper="UPPER" effort="EFFORT" velocity="VELOCITY"/>`: Specifies the limits of the joint's motion (for `revolute` and `prismatic` types) and its dynamic properties.

## 6. Complete Humanoid Robot URDF Example

Let's build a simplified URDF for a humanoid robot. This example will include a `base_link`, `torso_link`, `head_link`, `left_arm_link`, and `right_arm_link`.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- BASE LINK -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- TORSO LINK -->
  <link name="torso_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.05"/>
    </inertial>
  </link>

  <!-- HEAD LINK -->
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0"
               iyy="0.005" iyz="0.0"
               izz="0.005"/>
    </inertial>
  </link>

  <!-- LEFT ARM LINK -->
  <link name="left_arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- RIGHT ARM LINK -->
  <link name="right_arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="yellow"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>
  
  <!-- JOINTS -->

  <!-- Base to Torso Joint (Fixed) -->
  <joint name="base_to_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.075" rpy="0 0 0"/>
  </joint>

  <!-- Torso to Head Joint (Revolute - Yaw) -->
  <joint name="torso_to_head_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.18" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Torso to Left Arm Joint (Revolute - Shoulder Roll) -->
  <joint name="torso_to_left_arm_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="left_arm_link"/>
    <origin xyz="0 0.15 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Torso to Right Arm Joint (Revolute - Shoulder Roll) -->
  <joint name="torso_to_right_arm_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_arm_link"/>
    <origin xyz="0 -0.15 0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

## 7. Loading URDF in ROS 2

Once you have a URDF file, you can use ROS 2 tools to parse it, visualize the robot, and even spawn it in a simulator.

**Example: Visualization in RViz2**

1.  **Start `robot_state_publisher`**: This node reads the URDF file and publishes the robot's state (joint positions) to the `/tf` (transform) topic.
    ```bash
    # Ensure your URDF is in a package that ROS 2 can find
    # For a standalone file, you might use:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="<robot_description_string_from_urdf>"
    # Or, if you have it in a package:
    ros2 launch my_robot_description display.launch.py urdf_file:=path/to/my_robot.urdf
    ```
2.  **Start RViz2**:
    ```bash
    rviz2
    ```
    In RViz2, add a `RobotModel` display and ensure its `Description Source` is set to `robot_description` and `TF Prefix` is empty. You should see your robot model appear. You can then use `JointStatePublisher` to manipulate the joints.

**Example: Spawning in Gazebo**

For more complex simulations, you would typically use Gazebo. Gazebo can directly interpret URDF files, but for full simulation capabilities (e.g., sensors, plugins), you might convert your URDF to an SDF or augment it with Gazebo-specific tags.

```bash
# Example command to spawn a robot in Gazebo (requires Gazebo-ROS integration)
ros2 launch gazebo_ros spawn_entity.launch.py entity_name:=my_humanoid -file path/to/my_humanoid.urdf -x 0 -y 0 -z 1
```
This would place your robot in the Gazebo simulation environment.
