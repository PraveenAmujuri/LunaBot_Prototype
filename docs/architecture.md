# 🏛️ System Architecture

## Overview
The LunaBot architecture is designed as a distributed system comprising three distinct layers:
1.  **Physical Simulation Layer (Unity 3D):** Handles physics, sensor generation, and actuator dynamics.
2.  **Robotic Control Layer (ROS 1 Noetic):** Handles perception, SLAM, decision making, and navigation.
3.  **User Interface Layer (Flask/Web):** Handles telemetry visualization and command inputs.

## Process Flow Diagram
*(High-level architecture, simplified from the actual `rqt_graph` output)*

![Architecture Diagram](images/system_architecture.png)

## Communication Bridges
The system relies on a high-throughput **TCP/WebSocket bridge** to connect the disparate components:

* **Unity ↔ ROS:**  
  Uses `rosbridge_websocket` for sensor streaming (Images, Depth, LiDAR) and for receiving `/cmd_vel` velocity commands.

* **ROS ↔ Web Dashboard:**  
  Uses `rosbridge_websocket` to stream JSON-serialized state, detections, telemetry, and SLAM information to the frontend UI.

### Critical Subsystems
| Subsystem | Tech Stack | Role |
| :--- | :--- | :--- |
| **Perception** | YOLOv8 + OpenCV | Real-time object detection and semantic labeling. |
| **Mapping** | RTAB-Map (RGB-D) | Simultaneous Localization and Mapping with loop closure. |
| **Navigation** | Custom `goal_based_avoider` | Local path planning and dynamic obstacle avoidance. |
| **Simulation** | Unity (URP/Built-in RP) | Photorealistic lunar terrain and physics-based sensor simulation. |
