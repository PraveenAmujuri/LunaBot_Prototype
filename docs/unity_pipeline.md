# 🎮 Unity Simulation & Sim-to-Real Bridge

Unlike standard Gazebo simulations, this project utilizes a custom Unity 3D environment to simulate high-fidelity sensor noise and lunar physics.

![Unity Simulation](images/unity_sim.png)

## 1. Coordinate System Synchronization
One of the primary challenges in Unity-ROS integration is the coordinate mismatch. Unity uses a **Left-Handed** (Y-Up) system, while ROS uses a **Right-Handed** (Z-Up) system.

**Implementation Strategy:**
We engineered a custom C# bridge (`ROSControlledRover.cs`) to manually transform vectors in real-time, adhering to ROS **REP-103** standards.

```csharp
Vector3 p = transform.position;
float rosX = p.z;      // Unity Z is ROS X
float rosY = -p.x;     // Unity -X is ROS Y
float rosZ = p.y;      // Unity Y is ROS Z

```
### 2. Custom Sensor Drivers
We wrote raw data encoders to simulate hardware drivers within the game engine.

**LiDAR Simulation (LidarSensor.cs)**

Technique: Raycast-based scanning (180 rays per scan).

Performance: Raycasting occurs at physics timesteps to prevent "tunneling" through thin obstacles.

Output: Serializes data into standard sensor_msgs/LaserScan packets.

Depth Camera (DepthCameraPublisher.cs)
Technique: Shader-based depth extraction using Unity's DepthTextureMode.

Encoding: Raw float data is encoded into 32FC1 byte arrays to match RealSense camera outputs.

Optimization: GPU-accelerated rendering ensures the depth stream maintains 30FPS without stalling the main physics thread.

### 3. Physics-Based Control
Instead of kinematic translation (teleporting), the rover is driven by Torque and Force application to a RigidBody. This allows us to test control algorithms against:

Wheel slip on low-friction lunar regolith.

Inertia and momentum during emergency stops.

Center-of-mass shifts on inclines.