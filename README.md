🌑 LunaBot: Autonomous Lunar Rover PrototypeLunaBot is a distributed autonomous system designed to simulate lunar exploration in high-fidelity environments. It bridges the gap between Unity 3D physics and ROS 1 navigation stacks, featuring a custom Sim-to-Real pipeline, semantic perception (YOLOv8), and a real-time telemetry dashboard.Key Innovation: Unlike standard Gazebo simulations, LunaBot utilizes a custom C# bridge to handle coordinate synchronization (Left-Handed vs. Right-Handed), sensor noise simulation, and wheel-slip physics on lunar regolith.📹 System Demo(Replace the placeholder below with a GIF or a screenshot linking to your 15MB video hosted on YouTube/Vimeo/Drive)🏛️ System ArchitectureThe project operates as a distributed system across three distinct layers, communicating via high-throughput TCP/WebSockets.LayerComponentFunctionSimulationUnity 3D (Windows)Physics engine, LiDAR/Depth sensor emulation, terrain interaction.ControlROS Noetic (WSL/Ubuntu)SLAM (RTAB-Map), Semantic Perception (YOLO), Navigation.InterfaceFlask + React (Web)Mission control, telemetry visualization, manual override.Data Flow DiagramThe system utilizes a hybrid architecture where Unity generates sensor data, ROS processes it for autonomy, and Flask visualizes the state.Code snippetgraph TD
    subgraph Unity [Physical Simulation]
        A[Lidar & Depth Sensors] -->|TCP Bridge| B(ROS Bridge)
        P[Physics Engine] -->|Odom| B
    end
    
    subgraph ROS [Autonomy Stack]
        B <-->|/scan /odom| C[SLAM & Nav Stack]
        C -->|/cmd_vel| B
        C -->|Map & Pose| D[WebSocket Interface]
    end
    
    subgraph Web [Mission Control]
        D <-->|JSON Events| E[Flask Dashboard]
        E -->|User Commands| D
    end
🚀 Key Features1. Advanced Sim-to-Real BridgeCoordinate Synchronization: Implements a custom C# transform layer to map Unity's Left-Handed (Y-Up) system to ROS's Right-Handed (Z-Up) system, adhering to REP-103.Custom Sensor Drivers: * LiDAR: Raycast-based scanning (180 rays) serialized to sensor_msgs/LaserScan.Depth Camera: Shader-based depth extraction encoded to 32FC1 format.2. Perception & AutonomyRGB-D SLAM: Utilizes RTAB-Map for simultaneous localization and mapping with visual bag-of-words loop closure.Semantic Fusion: Integrates YOLOv8 bounding boxes with depth data to project 2D detections into 3D semantic markers (/semantic_annotator node).Reactive Navigation: Custom goal_based_avoider handles dynamic obstacles without the overhead of the full move_base stack.3. Low-Latency DashboardArchitecture: Hybrid HTTP + WebSocket (Flask-SocketIO).Performance: Handles high-frequency topics (50Hz Odom) via server-side rate limiting and JPEG map compression.Features: Real-time occupancy grid visualization, semantic overlays, and system health monitoring (CPU/GPU temps).⚡ Performance BenchmarksMetrics captured during full autonomous operation (SLAM + YOLO + Nav) on Lenovo LOQ 15IAx9.MetricValueNotesTotal CPU Load~10–15%High efficiency headroomInference Speed12–15 msYOLOv8 per frameEnd-to-End Latency< 50 msPerception to ActuationDepth Stream30 FPS32FC1 formatDev Velocity20 DaysTime to MVP📂 Repository StructureThe project follows a standard catkin workspace structure combined with Unity assets.PlaintextLunaBot_Prototype/
├── catkin_ws/                  # ROS Noetic Workspace
│   ├── src/
│   │   ├── luna_bot_ros/       # Main autonomy nodes (Python)
│   │   ├── messages/           # Custom ROS msg definitions
│   │   └── ...
├── LunaBotDashboard/           # Web Interface
│   ├── app.py                  # Flask entry point
│   ├── static/                 # JS/CSS assets
│   └── templates/              # HTML views
├── LunaBot-Unity/              # Unity Project Files (Assets/Scenes)
├── docs/                       # Technical Documentation
│   ├── architecture.md
│   ├── performance.md
│   └── ...
├── start_project.sh            # One-click launch script
├── stop_project.sh             # Cleanup script
└── requirements.txt            # Python dependencies
🛠️ Installation & SetupFor detailed instructions, please refer to docs/setup.md.PrerequisitesOS: Ubuntu 20.04 (WSL2 recommended)ROS: Noetic NinjemysUnity: 2022.3.62f1 (LTS)Quick StartClone the RepositoryBashgit clone https://github.com/PraveenAmujuri/LunaBot_Prototype.git
cd LunaBot_Prototype
Install DependenciesBash# System dependencies
sudo apt-get install ros-noetic-rtabmap-ros ros-noetic-rosbridge-suite

# Python dependencies
pip3 install -r ros_requirements.txt
Build ROS WorkspaceBashcd catkin_ws && catkin_make
source devel/setup.bash
Launch the SystemOpen Unity and press Play, then run the automation script in your terminal:Bashchmod +x start_project.sh
./start_project.sh
Access the dashboard at http://localhost:5000📚 DocumentationDetailed technical breakdowns are available in the docs/ directory:ROS Pipeline & Node GraphUnity Sim-to-Real BridgeAPI & WebSocket ProtocolHardware Benchmarks📄 LicenseThis project is licensed under the MIT License - see the LICENSE file for details.Author: Amujuri Sai Praveen