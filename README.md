# Autonomous Navigation and Path planning using Probabilistic Roadmap Algorithm

## Overview
This project develops an autonomous navigation system for a differential drive robot using the ROS framework. It integrates advanced path planning with real-time obstacle detection and avoidance, leveraging the Probabilistic Roadmap (PRM) method and simulated LiDAR sensor data.

## System Architecture
- **GoalNavigationNode:** Initializes with world file paths, managing navigation control.
- **ROS Topics:**
  - Subscribes to: `/map`, `/goal_pose`, `/current_position` for environmental data and navigation targets.
  - Publishes to: `/cmd_vel` for movement commands, `/path` for path visualization.

## Key Features
- **PRM Algorithm for Path Planning:** Utilizes a graph-based approach for navigating through complex environments.
- **Dijkstra's Algorithm:** Finds the shortest path on the PRM graph, ensuring efficient route planning.
- **Simulated LiDAR Sensor:** Enhances obstacle detection, contributing to dynamic path adjustment.

## Implementation Highlights
- Path planning and execution within known environments with obstacles.
- Reactive control mechanism to adjust robot's trajectory based on real-time data.
- Visualization of planned paths and robot movement in Rviz.

## Results
The integration of PRM and reactive control strategies successfully demonstrated autonomous navigation in a ROS-based simulation, meeting project expectations with effective goal achievement and obstacle navigation.

![image](https://github.com/khullarsanket/Autonomous-Navigation-and-Path-planning-using-Probabilistic-Roadmap-Algorithm/assets/119709438/678aec42-80ab-4379-b98e-956f281a0d41)
