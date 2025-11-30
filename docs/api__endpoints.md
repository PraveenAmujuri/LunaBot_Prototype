# 🌐 Dashboard API & Communication Protocol

The LunaBot Dashboard serves as the "Mission Control" interface. To ensure real-time responsiveness (<50ms latency) for teleoperation and telemetry, the backend utilizes a hybrid **HTTP + WebSockets** architecture powered by **Flask** and **Flask-SocketIO**.

![System Architecture](images/flask_api_flow.png)

## 📡 Architecture Overview
Unlike traditional REST APIs that poll for data, this system establishes a persistent **bi-directional WebSocket connection**.
* **Upstream (Client → Robot):** Sends control commands and navigation goals.
* **Downstream (Robot → Client):** Streams high-frequency telemetry, SLAM maps, and AI detections.
* **Bridge:** The Flask server acts as a middleman, translating JSON WebSocket events into ROS 1 topics via `rosbridge`.

---

## 1. HTTP Routes
The HTTP layer is minimal, serving primarily as the entry point for the single-page application (SPA).

| Method | Route | Description |
| :--- | :--- | :--- |
| `GET` | `/` | Serves the main Dashboard UI (`index.html`) and initializes the Socket.IO client. |
| `GET` | `/static/*` | Serves optimized assets (CSS, JS, Icons). |

---

## 2. WebSocket API: Client → Server (Commands)
These events allow the web user to control the robot.

### 🕹️ `rover_command`
**Purpose:** Direct teleoperation (manual control).
* **ROS Mapping:** Publishes to `/cmd_vel` (`geometry_msgs/Twist`).
* **Payload:**
    ```json
    {
      "cmd_vel": {
        "linear": { "x": 0.5, "y": 0.0, "z": 0.0 },
        "angular": { "x": 0.0, "y": 0.0, "z": 0.1 }
      }
    }
    ```

### 📍 `navigation_goal`
**Purpose:** Sends a target coordinate for autonomous path planning.
* **ROS Mapping:** Publishes to `/move_base_simple/goal` (`geometry_msgs/PoseStamped`).
* **Payload:**
    ```json
    {
      "goal": {
        "position": { "x": 2.5, "y": -1.2 }
      }
    }
    ```

### 🛑 `reset_navigation`
**Purpose:** Emergency abort. Clears current path and stops the robot.
* **ROS Mapping:** Publishes to `/move_base/cancel` (`actionlib_msgs/GoalID`).
* **Payload:** `null`

---

## 3. WebSocket API: Server → Client (Telemetry)
The server pushes updates asynchronously via background worker threads to ensure the UI never freezes.

### 📊 `rover_status`
* **Frequency:** 10Hz
* **Purpose:** Live position and velocity feedback.
```json
{
  "position": {"x": 1.2, "y": 0.5},
  "velocity": {"linear_x": 0.2, "angular_z": 0.0},
  "mode": "MOVING",
  "path": [{"x":0,"y":0}, {"x":1.2,"y":0.5}]
}
```
### 🗺️ `map_update`
* **Frequency:** On Change (Event-driven)
* **Purpose:** Displays the live SLAM occupancy grid.

Optimization: The raw ROS grid is downscaled and compressed into a Base64 JPEG on the server to reduce bandwidth usage by ~90%.

```json
{ 
  "map": "/9j/4AAQSkZJRg...", 
  "timestamp": 1690000000.0 
}
```
### 👁️ `semantic_update`
* **Frequency:** Real-time
* **Purpose:** Overlays YOLOv8 detections onto the UI map.

```json
{ 
  "semantic_map": { 
    "rocks": [{"x":1.2, "y":-0.4}], 
    "flags": [], 
    "antennas": [] 
  } 
}
```
### 💻 `system_metrics`
* **Frequency:** 1Hz
* **Purpose:** Remote health monitoring of the onboard computer (LOQ Laptop).
```json
{
  "cpu": 12.3,
  "gpu": 45,
  "ram_used": 6.2,
  "gpu_temp": 58,
  "gpu_mem_percent": 27.5
}
```
### 🔌 `Integration Examples`
**JavaScript Client (Browser)**

```JavaScript

const socket = io('http://localhost:5000');

// 1. Listen for Telemetry
socket.on('rover_status', (data) => {
    updateGauge(data.speed);
    updateMapPosition(data.position);
});

// 2. Send Command
function driveForward() {
    socket.emit('rover_command', { 
        cmd_vel: { 
            linear: { x: 0.5 }, 
            angular: { z: 0.0 } 
        } 
    });
}
```
**Python Client (Automated Testing)**
```Python

import socketio

sio = socketio.Client()
sio.connect('http://localhost:5000')

@sio.event
def initial_state(data):
    print(f"Connected. Battery Level: {data['slam_data']['mission_status']['battery']}%")
```
# Simulate a navigation goal
sio.emit('navigation_goal', {'goal': {'position': {'x': 5.0, 'y': 5.0}}})
### 🛠️ `Engineering Challenges & Solutions`
**1. Bandwidth Optimization**

Transferring raw SLAM maps (nav_msgs/OccupancyGrid) over WiFi is bandwidth-intensive.

Solution: Implemented a server-side image processor that converts raw grid arrays into compressed JPEG images before transmission, reducing payload size from 2MB to <50KB.

**2. Backpressure Management**

High-frequency ROS topics (like /odom at 50Hz) can overwhelm the WebSocket connection.

Solution: Implemented STATUS_QUEUE and MAP_QUEUE with fixed sizes on the Flask server. This acts as a rate-limiter, dropping older frames if the web client lags, ensuring the UI always displays the latest data rather than old buffered data.

**3. Unity Sim Sync**

Large coordinate jumps (teleportation) in simulation often break standard navigation stacks.

Solution: Added a unity_restart_detected event logic. The server monitors odometry deltas; if a jump >10m is detected in one frame, it automatically resets the frontend path visualization.