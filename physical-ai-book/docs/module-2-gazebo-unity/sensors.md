---
id: sensors
title: Sensor Simulation
sidebar_position: 3
---

## 1. Introduction to Sensor Simulation

Robots perceive their environment primarily through **sensors**. For Physical AI, accurate sensor data is paramount for tasks like navigation, object recognition, manipulation, and decision-making. In simulation, we don't have physical sensors, so we rely on **simulated sensors** that mimic the behavior and data output of their real-world counterparts.

Gazebo provides a rich set of built-in simulated sensors, allowing you to attach virtual LiDARs, cameras, IMUs, sonars, and more to your robot models. These simulated sensors publish data in ROS 2 message formats, making them seamlessly compatible with actual robot control and AI algorithms developed in ROS 2.

The benefits of sensor simulation include:
*   **Cost-Effectiveness**: No need to buy expensive physical sensors for initial development and testing.
*   **Safety**: Test algorithms in hazardous or extreme conditions without risk to physical hardware.
*   **Reproducibility**: Generate identical sensor data streams repeatedly for robust debugging and algorithm evaluation.
*   **Parameter Tuning**: Easily change sensor parameters (e.g., LiDAR range, camera resolution) to understand their impact.
*   **Ground Truth Access**: In simulation, you have access to the "ground truth" (e.g., exact object positions, velocities), which is invaluable for debugging perception algorithms and generating training data for AI.

## 2. LiDAR Simulation

**LiDAR (Light Detection and Ranging)** sensors are crucial for robotic navigation and mapping. They generate a point cloud, which is a set of data points in a 3D coordinate system, representing the environment's shape.

In Gazebo, a simulated LiDAR (often referred to as a "Ray Sensor" or "GPU Ray Sensor" for performance) can be configured within your robot's SDF/URDF file.

**Key parameters to configure for a simulated LiDAR:**
*   **`ray` element**: Defines the sensor's characteristics.
    *   `scan`:
        *   `horizontal` and `vertical`: Specify the angle range, resolution (number of rays), and maximum range.
    *   `range`: Defines minimum, maximum, and resolution of a single ray.
*   **`always_on`**: Whether the sensor is always active.
*   **`update_rate`**: How often the sensor publishes data (Hz).
*   **`visualize`**: Whether to visualize the rays in Gazebo GUI.
*   **`plugin`**: A ROS 2 Gazebo plugin (e.g., `libgazebo_ros_laser.so`) to bridge the simulated data to ROS 2 topics, typically publishing `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`.

**Example (SDF Snippet for a LiDAR Sensor):**

```xml
<sensor name="laser_sensor" type="ray">
  <pose>0.1 0 0.2 0 0 0</pose> <!-- Position relative to its parent link -->
  <visualize>true</visualize>
  <update_rate>10</update_rate> <!-- 10 Hz -->
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>      <!-- Number of rays -->
        <resolution>1</resolution>  <!-- Resolution factor -->
        <min_angle>-1.5708</min_angle> <!-- -90 degrees -->
        <max_angle>1.5708</max_angle>  <!-- +90 degrees -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="laser_controller" filename="libgazebo_ros_laser.so">
    <topicName>scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

## 3. Depth Camera Simulation

**Depth cameras** (like Intel RealSense or Microsoft Kinect) provide both a standard color image and a depth map, which represents the distance of objects from the camera. This 3D information is critical for tasks like object pose estimation, obstacle avoidance, and 3D mapping.

In Gazebo, a simulated depth camera (often `depth_camera` type or a `camera` with depth configuration) can also be defined in your model's SDF/URDF.

**Key parameters:**
*   **`camera` element**: Defines optical properties.
    *   `horizontal_fov`, `image` (width, height, format), `clip` (near, far).
*   **`plugin`**: A ROS 2 Gazebo plugin (e.g., `libgazebo_ros_depth_camera.so`) to publish data to ROS 2 topics, typically:
    *   `sensor_msgs/msg/Image` (color image)
    *   `sensor_msgs/msg/Image` (depth image)
    *   `sensor_msgs/msg/PointCloud2` (3D point cloud)
    *   `sensor_msgs/msg/CameraInfo`

**Example (SDF Snippet for a Depth Camera):**

```xml
<sensor name="depth_camera_sensor" type="depth_camera">
  <pose>0 0 0 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30.0</update_rate> <!-- 30 Hz -->
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
    <baseline>0.2</baseline>
    <alwaysOn>true</alwaysOn>
    <imageTopicName>color/image_raw</imageTopicName>
    <cameraInfoTopicName>color/camera_info</cameraInfoTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <pointCloudTopicName>depth/color/points</pointCloudTopicName>
    <pointCloudCutoff>0.5</pointCloudCutoff>
    <frameName>camera_depth_frame</frameName>
  </plugin>
</sensor>
```

## 4. IMU Sensors

An **IMU (Inertial Measurement Unit)** sensor measures a robot's orientation, angular velocity, and linear acceleration. This data is critical for tasks like robot localization, stabilization, and control.

In Gazebo, a simulated IMU typically uses the `imu` sensor type.

**Key parameters:**
*   **`imu` element**:
    *   `angular_velocity`, `linear_acceleration`: Define noise properties.
*   **`plugin`**: A ROS 2 Gazebo plugin (e.g., `libgazebo_ros_imu_sensor.so`) to publish `sensor_msgs/msg/Imu` messages to ROS 2 topics.

**Example (SDF Snippet for an IMU Sensor):**

```xml
<sensor name="imu_sensor" type="imu">
  <pose>0 0 0.05 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>100</update_rate> <!-- 100 Hz -->
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <!-- y and z similar -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <!-- y and z similar -->
    </linear_acceleration>
  </imu>
  <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/imu</namespace>
      <argument>--ros-args -r /imu:=imu/data</argument>
    </ros>
    <topicName>data</topicName>
    <frameName>imu_link</frameName>
  </plugin>
</sensor>
```

## 5. Reading Sensor Data in ROS 2

Once your simulated sensors are configured in Gazebo and connected via ROS 2 plugins, they will publish data to specific ROS 2 topics. You can then write ROS 2 nodes (e.g., using `rclpy` or `rclcpp`) to subscribe to these topics and process the sensor data.

**General steps to read sensor data:**

1.  **Identify the topic name and message type**: Use `ros2 topic list` and `ros2 topic info <topic_name>` to find out what topics your simulated sensors are publishing to and what message types they use.
2.  **Create a subscriber node**: Write a ROS 2 node that subscribes to the sensor data topic.
3.  **Implement a callback function**: This function will be executed every time new sensor data is received. Inside the callback, you can access and process the data fields of the received message.

**Example Python Snippet for Subscribing to a LaserScan:**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic name where LiDAR data is published
            self.laser_callback,
            10
        )
        self.get_logger().info('LaserScan Subscriber Node has started.')

    def laser_callback(self, msg):
        # Process the LaserScan message
        # msg.header: Contains timestamp and frame_id
        # msg.angle_min, msg.angle_max, msg.angle_increment: Scan angles
        # msg.range_min, msg.range_max: Min/Max range values
        # msg.ranges: Array of range values (distances)
        # msg.intensities: Array of intensity values (if available)

        self.get_logger().info(f"Received LaserScan from frame: {msg.header.frame_id}")
        # Example: print the first and last range reading
        if len(msg.ranges) > 0:
            self.get_logger().info(f"  First range: {msg.ranges[0]:.2f}m")
            self.get_logger().info(f"  Last range: {msg.ranges[-1]:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This subscriber would run alongside your Gazebo simulation, receiving and processing the data generated by your simulated LiDAR sensor. The same principles apply to depth camera (`Image`, `PointCloud2`) and IMU (`Imu`) sensor data.
