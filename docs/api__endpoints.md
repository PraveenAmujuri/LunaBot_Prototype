
---

# **📄 4. docs/api_endpoints.md**

```markdown
# 🌐 Dashboard API Documentation

The Web Dashboard serves as the Mission Control Center. It interacts with the ROS system via a **Flask** backend that wraps `rosbridge_websocket`.

![Flask API Flow](images/flask_api_flow.png)

## API Routes
The following endpoints were verified using `flask routes`:

| Endpoint | Method | Rule | Description |
| :--- | :--- | :--- | :--- |
| `dashboard` | GET | `/` | Serves the main HTML/JS interface. |
| `api_status` | GET | `/api/status` | Returns overall system health. |
| `api_mission_status` | GET | `/api/mission_status` | Returns current mission state and battery level. |
| `api_detected_objects` | GET | `/api/detected_objects` | YOLOv8 detections (Class, Confidence, 3D position). |
| `api_map_image` | GET | `/api/map_image` | Returns compressed Occupancy Grid. |
| `api_reset_navigation` | POST | `/api/reset_navigation` | Forces mission manager to reset navigation. |

## Data Streaming
Real-time telemetry (Odometry, LiDAR, SLAM updates) uses:

- **Socket Server:** `rosbridge_server`
- **Client:** `roslibjs` in browser

WebSockets minimize latency and avoid inefficient polling.
