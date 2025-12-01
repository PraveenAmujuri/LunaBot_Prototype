# 🛠️ LunaBot Setup & Installation Guide

This guide details how to set up the development environment for the LunaBot Autonomous System. The project runs as a distributed system across **Windows (Unity)** and **WSL/Linux (ROS 1)**.

## 📋 System Prerequisites (Critical)
Before installing, ensure your system matches these exact versions. Mismatches (especially Python/ROS) will cause immediate failures.

| Component | Required Version | Why? |
| :--- | :--- | :--- |
| **OS** | Ubuntu 20.04 (LTS) | Required for ROS 1 Noetic. |
| **ROS** | **ROS Noetic Ninjemys** | The autonomy stack is built on ROS 1. ROS 2 is *not* supported. |
| **Python (ROS)** | **Python 3.8.x** | Default for Ubuntu 20.04. Do not upgrade system Python. |
| **Python (Dashboard)** | Python 3.8 - 3.10 | The Flask Dashboard is flexible but 3.8 is recommended. |
| **Unity** | **2022.3.62f1 (LTS)** | Required for the physics engine and C# scripts compatibility. |
| **GPU Drivers** | CUDA 11.8+ | Required for YOLOv8 and Unity HDRP rendering. |

---

## 🚀 1. Installation Steps

### Step 1: Clone the Repository
Clone the repository into your WSL/Ubuntu environment.

```bash
git clone https://github.com/PraveenAmujuri/LunaBot_Prototype.git
cd LunaBot_Prototype
```
### Step 2: Install Global ROS Dependencies
Your ROS nodes (YOLO, Navigation) run in the global ROS environment, not in a virtual environment.

```bash
Copy code
sudo apt-get update
sudo apt-get install python3-pip ros-noetic-rtabmap-ros \
    ros-noetic-rosbridge-suite ros-noetic-web-video-server
```
Install Python libraries required for ROS-side AI nodes:

```bash
pip3 install ultralytics opencv-python numpy psutil
```
### step 3: Setup Dashboard Environment (Flask)
The Web Dashboard uses its own isolated venv.

```bash
cd LunaBotDashboard

# Create venv
python3 -m venv dashboard_env

# Activate it
source dashboard_env/bin/activate

# Install Dashboard requirements
pip install -r ../requirements.txt
```

### step 4: Build ROS Workspace
Compile the custom ROS packages (luna_bot_ros, messages, etc.).
```bash
cd ../catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### Step 5: Setup Unity Simulation (Windows)
Open Unity Hub on Windows.

Click Add and select the LunaBot-Unity folder from the repository.

**Important:** Ensure you are using Unity `2022.3.62f1`.

**Open the scene:** Assets/Scenes/LunarLandscape3D.unity.

Ensure the RosBridgeClient script on the RosConnector object is set to `ws://localhost:9090` (or your WSL IP).

## 🏃‍♂️ How to Run

### Option A: One-Click Launch (Recommended)
We have provided a comprehensive shell script that initializes the entire stack (ROS Core, Bridge, AI, SLAM, and Dashboard) in new terminal tabs.

**Start Unity:** Press the Play button in the Unity Editor first.

Make Scripts Executable:

```Bash
chmod +x start_project.sh stop_project.sh
```
Run the Script (in WSL):

```Bash
./start_project.sh
```
**Access Dashboard:** Open your browser to `http://localhost:5000`.

### Option B: Manual Launch (Debugging)
If you need to debug specific nodes, launch them individually:

**Terminal 1:** Infrastructure

```Bash
roscore
roslaunch rosbridge_server rosbridge_websocket.launch
```
**Terminal 2:** AI & SLAM

```Bash
roslaunch luna_bot_ros rtabmap_rgbd.launch
python3 catkin_ws/src/luna_bot_ros/nodes/object_detection_node.py
```
**Terminal 3:** Dashboard

```Bash
cd LunaBotDashboard
source dashboard_env/bin/activate
export FLASK_APP=app.py
python app.py
```

## 🛑 How to Stop
To ensure all background processes (like YOLO and ROS nodes) are cleanly terminated, use the stop script:

```Bash
./stop_project.sh
```

## 🔧 Troubleshooting
**1. `ModuleNotFoundError: No module named 'cv2'`**

**Cause:** You are trying to run the dashboard or YOLO node without the correct dependencies installed.

**Fix:** Ensure you have activated the virtual environment before running the dashboard.

```Bash
source LunaBotDashboard/dashboard_env/bin/activate
```
**2. `Permission denied: ./start_project.sh`**

**Cause:** Linux file permissions were lost during transfer from Windows.

**Fix:** Make the scripts executable.

```Bash
chmod +x *.sh
```
**3. `'export' is not recognized (Windows Error)`**

**Cause:** You are trying to run Linux commands in a Windows Command Prompt (CMD).

**Fix:** Use WSL (Ubuntu terminal) or Git Bash. Do not use CMD or PowerShell for the ROS components.


**4. `Unity Robot Not Moving`**

**Cause:** The ROS Bridge is disconnected.

**Fix:** 

- 1. Check the Unity Console. If it says "ROSBridge Error", check the IP address. 
- 2. Ideally, run Unity on the same machine as ROS (localhost).
- 3. Ensure start_project.sh successfully launched goal_based_avoider.py.

**5. `SLAM Map Not Appearing in Dashboard`**

**Cause:** rtabmap or web_video_server failed to start.

**Fix:** Run `rosnode list`. If you don't see `/rtabmap/rtabmap`, restart the system. Check if the camera topics (`/camera/color/image_raw`) are active using rostopic hz.