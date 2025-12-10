---
id: navigation
title: Nav2 Path Planning
sidebar_position: 4
---

## 1. Introduction to Nav2

**Nav2** is the second generation of ROS's navigation stack, built entirely on ROS 2. It's a comprehensive framework for enabling a robot to autonomously navigate from a starting pose to a goal pose while avoiding obstacles. Nav2 is highly modular and configurable, allowing it to be adapted to a wide range of mobile robot platforms, from wheeled robots to humanoids (though humanoid navigation often involves additional complexities).

The core function of Nav2 is **path planning** and **motion execution**. It doesn't just compute a single path; it continuously re-plans and adapts the robot's movement based on real-time sensor data and dynamic changes in the environment.

## 2. Nav2 Core Components

Nav2 is typically comprised of several interconnected ROS 2 nodes, each performing a specific function within the navigation process:

*   **Map Server**: Provides the robot with a map of the environment (either a static map loaded from a file or a dynamically built map from a SLAM system).
*   **AMCL (Adaptive Monte Carlo Localization)**: Localizes the robot within a known map using sensor data (e.g., LiDAR).
*   **Behavior Tree**: Orchestrates the high-level decision-making for the navigation task, defining behaviors like "go to goal," "recover from collision," or "patrol."
*   **Global Planner**: Plans a collision-free path from the robot's current location to the goal location on the global map. This path is usually long-term and ignores very small, temporary obstacles.
*   **Local Planner (Controller)**: Takes the global path and generates short-term, dynamically feasible velocity commands for the robot, continuously adjusting to local obstacles detected by sensors.
*   **Obstacle Layer / Costmap Filter**: Creates a "costmap" (a grid representation of the environment with obstacle costs) based on sensor readings, which is used by planners to avoid collisions.
*   **Waypoint Follower**: Enables the robot to follow a sequence of waypoints.

## 3. Path Planning for Bipedal Humanoid Movement (Special Considerations)

While Nav2 is powerful for mobile robots, applying it directly to **bipedal humanoids** introduces significant challenges due to their complex kinematics, dynamics, and balance requirements.

*   **Complex Kinematics**: Humanoids have many degrees of freedom, and their movement involves solving inverse kinematics for stable foot placement and balance. Nav2's planners, by default, might assume simpler, wheeled robot kinematics.
*   **Dynamic Stability and Balance**: Every step a humanoid takes involves maintaining balance against gravity and inertial forces. Path planning must not only be collision-free but also dynamically stable. This often requires integration with whole-body control and balance algorithms.
*   **Footstep Planning**: Instead of continuous paths, humanoids often require discrete "footstep plans" that define where each foot will land to maintain stability.
*   **Environment Interaction**: Humanoids can potentially interact with the environment in more complex ways (e.g., stepping over obstacles, climbing stairs) than typical wheeled robots, requiring more sophisticated perception and planning.

**Integration Approach for Humanoids:**
For humanoid robots, Nav2 would typically provide high-level global path planning, but the local planning and motion execution would be handled by specialized humanoid control libraries (often integrated into ROS 2) that manage balance, footstep planning, and whole-body control. Nav2's costmaps would still be crucial for identifying walkable surfaces and obstacles.

## 4. Obstacle Avoidance

**Obstacle avoidance** is a critical function within Nav2, ensuring the robot can safely reach its goal without colliding with its environment or dynamic objects (e.g., people).

**How Nav2 handles obstacle avoidance:**

*   **Costmaps**: Nav2 uses local and global costmaps. These are 2D or 3D grid maps that represent the environment, assigning costs to cells based on proximity to obstacles. Cells directly occupied by obstacles have a very high cost, while cells near obstacles have a lower, increasing cost, creating a "cushion" around obstacles.
*   **Sensor Input**: Real-time sensor data (LiDAR, depth cameras, sonar) feeds into the costmap updates. As new obstacles are detected, the costmap is updated dynamically.
*   **Local Planner**: The local planner continuously samples possible robot trajectories and evaluates them against the local costmap. It selects the trajectory that best moves the robot towards the global path while avoiding obstacles and respecting kinematic/dynamic constraints.
*   **Recovery Behaviors**: Nav2 includes a behavior tree that can trigger recovery behaviors if the robot gets stuck or cannot find a path. These might include rotating in place, backing up, or trying a different local planning strategy.

The modular design of Nav2, combined with its reliance on standard ROS 2 interfaces, makes it a flexible and powerful tool for autonomous navigation, capable of being extended and adapted for advanced robot platforms like humanoids.
