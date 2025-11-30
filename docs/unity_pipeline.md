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
## 2. Custom Sensor Drivers
We wrote raw data encoders to simulate hardware drivers within the game engine.

**LiDAR Simulation (LidarSensor.cs)**

Technique: Raycast-based scanning (180 rays per scan).

Performance: Raycasting occurs at physics timesteps to prevent "tunneling" through thin obstacles.

Output: Serializes data into standard sensor_msgs/LaserScan packets.

Depth Camera (DepthCameraPublisher.cs)
Technique: Shader-based depth extraction using Unity's DepthTextureMode.

Encoding: Raw float data is encoded into 32FC1 byte arrays to match RealSense camera outputs.

Optimization: GPU-accelerated rendering ensures the depth stream maintains 30FPS without stalling the main physics thread.

## 3. Physics-Based Control
Instead of kinematic translation (teleporting), the rover is driven by Torque and Force application to a RigidBody. This allows us to test control algorithms against:

Wheel slip on low-friction lunar regolith.

Inertia and momentum during emergency stops.

Center-of-mass shifts on inclines.

## 4. Message Parsing & Timestamps

Unity acts as both a publisher and subscriber for ROS topics.

### `/cmd_vel`
Velocity commands received via rosbridge are parsed and applied to wheel forces inside Unity.

### Timestamp Behavior

| Component        | Timestamp Type      |
|------------------|----------------------|
| **Depth Camera** | ROS epoch            |
| **RGB Camera**   | Unity `Time.time`    |
| **Live Camera**  | Unity `Time.time`    |

**Future Improvement:**  
Unify all timestamps to **ROS epoch** for perfect multi-sensor synchronization.

---

## 5. ROS Topic Summary

### Unity → ROS (Published Topics)

| Topic                        | Source                   | Type          |
|------------------------------|--------------------------|---------------|
| `/scan`                      | LidarSensor.cs           | LaserScan     |
| `/camera/color/image_raw`    | RoverCameraPublisher.cs  | Image         |
| `/camera/depth/image_raw`    | DepthCameraPublisher.cs  | Image (32FC1) |
| `/odom`                      | ROSControlledRover.cs    | Odometry      |
| `/tf`                        | ROSControlledRover.cs    | TFMessage     |

### ROS → Unity (Subscribed Topics)

| Topic      | Description               |
|------------|----------------------------|
| `/cmd_vel` | Velocity command input     |

---

## Summary

The Unity simulation pipeline provides:

- High-fidelity sensor streams  
- Accurate ROS coordinate conversion  
- Physics-driven rover motion  
- Realistic RGB, Depth, and LiDAR publishing  
- Full compatibility with RTAB-Map, YOLOv8, and the LunaBot autonomy stack  
