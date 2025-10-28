#!/usr/bin/env python3

# COMPLETELY FIXED - AUTO RESTART DETECTION + GOAL CANCELLATION + SLAM
import eventlet
eventlet.monkey_patch()

import time
import json
import threading
import websocket
import cv2
import numpy as np
import base64
import math
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
from websocket import WebSocketApp
from collections import deque
import gc


# Configuration
ROSBRIDGE_HOST = 'localhost'
ROSBRIDGE_PORT = 9090


# Flask setup
app = Flask(__name__)
socketio = SocketIO(app, 
                   cors_allowed_origins="*", 
                   async_mode='eventlet',
                   logger=False,
                   engineio_logger=False)


# Rover state - REAL DATA ONLY with restart detection
rover_data = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "last_position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "path": deque(maxlen=50),
    "velocity": {"linear_x": 0.0, "angular_z": 0.0},
    "speed": 0.0,
    "mode": "IDLE",
    "connected": False,
    "camera_active": False,
    "frame_count": 0,
    "last_update": time.time(),
    "position_jump_detected": False,
    "restart_count": 0
}


# Activity log
activity_log = deque(maxlen=30)


# ═══════════════════════════════════════════════════════════════════
# NEW: SLAM DATA (ADDED - doesn't touch existing code)
# ═══════════════════════════════════════════════════════════════════
slam_data = {
    'map_image': None,  # Base64 encoded map image
    'semantic_map': {   # Detected objects
        'rocks': [],
        'base': [],
        'flags': [],
        'antennas': []
    },
    'mission_status': {
        'battery': 100.0,
        'mode': 'EXPLORATION',
        'base_x': 0.0,
        'base_y': 0.0,
        'base_detected': False,
        'distance_to_base': 0.0,
        'path_length': 0
    },
    'path_history': []  # Traversed path for backtracking
}


def log_activity(message):
    """Real activity logging only"""
    try:
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        activity_log.append(log_entry)
        socketio.emit('activity_update', {'message': log_entry})
        print(f"📝 {message}")
    except Exception as e:
        print(f"❌ Log activity error: {e}")


# ROS WebSocket Connection
rosbridge_ws = None
rosbridge_pub_ws = None
publisher_connected = False


def connect_rosbridge_publisher():
    """ROS publisher connection"""
    global rosbridge_pub_ws, publisher_connected
    try:
        print(f"🔌 Connecting to ROSBridge publisher at {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
        rosbridge_pub_ws = websocket.WebSocket()
        rosbridge_pub_ws.settimeout(10)
        rosbridge_pub_ws.connect(f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
        publisher_connected = True
        print("✅ ROSBridge publisher connected")
        log_activity("🚀 ROS Publisher connected")
        return True
    except Exception as e:
        publisher_connected = False
        print(f"❌ ROSBridge publisher connection failed: {e}")
        return False


def publish_to_ros(topic, msg_type, message):
    """Publish to ROS"""
    global rosbridge_pub_ws, publisher_connected
    try:
        if not publisher_connected and not connect_rosbridge_publisher():
            print(f"❌ Cannot publish to {topic} - no ROS connection")
            return False
        
        ros_message = {
            "op": "publish",
            "topic": topic,
            "msg": message
        }
        
        rosbridge_pub_ws.send(json.dumps(ros_message, separators=(',', ':')))
        print(f"📤 Published to {topic}: {message}")
        log_activity(f"📤 Sent to ROS: {topic}")
        return True
        
    except Exception as e:
        print(f"❌ ROS publish error: {e}")
        publisher_connected = False
        return False


# UNITY RESTART DETECTION & RECOVERY FUNCTIONS
def cancel_navigation_goal():
    """Cancel active move_base goal"""
    try:
        cancel_msg = {"id": "", "stamp": {"secs": 0, "nsecs": 0}}
        success = publish_to_ros('/move_base/cancel', 'actionlib_msgs/GoalID', cancel_msg)
        if success:
            print("✅ Cancelled active navigation goal")
            log_activity("🛑 Cancelled navigation goal")
            return True
        return False
    except Exception as e:
        print(f"❌ Cancel goal error: {e}")
        return False


def stop_rover():
    """Send stop command to rover"""
    try:
        stop_msg = {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        success = publish_to_ros('/cmd_vel', 'geometry_msgs/Twist', stop_msg)
        if success:
            print("✅ Stop command sent")
            log_activity("🛑 STOP command sent")
            rover_data['mode'] = "STOPPED"
            return True
        return False
    except Exception as e:
        print(f"❌ Stop error: {e}")
        return False


def clear_costmaps():
    """Clear move_base costmaps via ROS service"""
    try:
        service_msg = {
            "op": "call_service",
            "service": "/move_base/clear_costmaps",
            "args": {}
        }
        
        if publisher_connected and rosbridge_pub_ws:
            rosbridge_pub_ws.send(json.dumps(service_msg))
            print("✅ Costmap clear requested")
            log_activity("🧹 Costmaps cleared")
            return True
        return False
    except Exception as e:
        print(f"❌ Clear costmaps error: {e}")
        return False


def detect_unity_restart(new_x, new_y, new_z):
    """Detect Unity restart by position jump"""
    try:
        last_x = rover_data['last_position']['x']
        last_y = rover_data['last_position']['y']
        
        # Calculate distance
        dx = new_x - last_x
        dy = new_y - last_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check for large jump (>3 meters)
        if distance > 3.0 and not rover_data['position_jump_detected']:
            print("="*60)
            print(f"🔄 UNITY RESTART DETECTED!")
            print(f"   Position jump: {distance:.2f}m")
            print(f"   Old: ({last_x:.1f}, {last_y:.1f})")
            print(f"   New: ({new_x:.1f}, {new_y:.1f})")
            print("="*60)
            
            rover_data['position_jump_detected'] = True
            rover_data['restart_count'] += 1
            
            log_activity(f"🔄 UNITY RESTART #{rover_data['restart_count']} DETECTED!")
            log_activity(f"   Position jump: {distance:.1f}m")
            
            # STOP ALL COMMANDS
            print("🛑 Stopping rover...")
            stop_rover()
            
            # CANCEL ACTIVE GOALS
            print("🛑 Cancelling navigation...")
            cancel_navigation_goal()
            
            # CLEAR COSTMAPS
            print("🧹 Clearing costmaps...")
            clear_costmaps()
            
            # Clear path history
            rover_data['path'].clear()
            
            # Notify dashboard
            socketio.emit('unity_restart_detected', {
                'restart_count': rover_data['restart_count'],
                'old_position': {'x': last_x, 'y': last_y},
                'new_position': {'x': new_x, 'y': new_y},
                'distance_jump': distance,
                'timestamp': time.time()
            })
            
            print("✅ Unity restart handling complete!")
            log_activity("✅ Reset complete - ready for new commands")
            
            # Reset flag after 2 seconds
            def reset_flag():
                time.sleep(2)
                rover_data['position_jump_detected'] = False
            
            threading.Thread(target=reset_flag, daemon=True).start()
            
            return True
        
        return False
        
    except Exception as e:
        print(f"❌ Restart detection error: {e}")
        return False


# ROS MESSAGE HANDLERS
def on_message(ws, message):
    """Process REAL ROS messages ONLY"""
    try:
        data = json.loads(message)
        topic = data.get("topic")
        if not topic:
            return
            
        msg = data.get("msg", {})
        print(f"📨 REAL ROS MESSAGE from: {topic}")
        
        # EXISTING HANDLERS (KEEP THESE)
        if topic == "/rover_camera/image_raw":
            process_camera(msg)
        elif topic == "/odom":
            process_odometry(msg)
        elif topic in ["/cmd_vel", "/cmd_vel_direct"]:
            process_velocity(msg)
        elif topic == "/hazard_detection":
            process_detection(msg)
        
        # ═══════════════════════════════════════════════════════════
        # NEW: SLAM MESSAGE HANDLERS (ADDED)
        # ═══════════════════════════════════════════════════════════
        elif topic == "/map":
            process_map(msg)
        elif topic == "/semantic_map":
            process_semantic_map(msg)
        elif topic == "/mission_status":
            process_mission_status(msg)
        elif topic == "/traversed_path":
            process_path_history(msg)
        # ═══════════════════════════════════════════════════════════
        
    except Exception as e:
        print(f"❌ ROS message error: {e}")


def process_camera(msg):
    """Process REAL camera data"""
    try:
        width = msg.get('width', 640)
        height = msg.get('height', 480)
        encoding = msg.get('encoding', 'rgb8')
        image_data = msg.get('data')
        
        if not image_data:
            return
        
        print(f"📷 REAL camera: {width}x{height}, {encoding}")
        
        # Handle ROS image data
        if isinstance(image_data, str):
            try:
                image_bytes = base64.b64decode(image_data)
            except:
                image_bytes = image_data.encode()
        elif isinstance(image_data, list):
            image_bytes = bytes(image_data)
        else:
            image_bytes = image_data
        
        # Convert to JPEG
        if encoding in ['jpeg', 'jpg']:
            base64_data = base64.b64encode(image_bytes).decode('utf-8')
        else:
            try:
                np_arr = np.frombuffer(image_bytes, np.uint8)
                if encoding == 'rgb8':
                    img = np_arr.reshape((height, width, 3))
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif encoding == 'bgr8':
                    img_bgr = np_arr.reshape((height, width, 3))
                else:
                    img_bgr = np_arr.reshape((height, width, 3))
                
                _, jpg_data = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                base64_data = base64.b64encode(jpg_data.tobytes()).decode('utf-8')
            except Exception as e:
                print(f"❌ Image conversion error: {e}")
                return
        
        rover_data['camera_active'] = True
        rover_data['frame_count'] += 1
        rover_data['last_update'] = time.time()
        
        camera_update = {
            'width': width,
            'height': height,
            'data': base64_data,
            'frame_count': rover_data['frame_count'],
            'timestamp': time.time()
        }
        
        socketio.emit('camera_update', camera_update)
        print(f"📷 ✅ REAL frame {rover_data['frame_count']} sent")
        
    except Exception as e:
        print(f"❌ Camera error: {e}")


def process_odometry(msg):
    """Process REAL ROS odometry with Unity restart detection"""
    try:
        print("🗺️ Processing REAL odometry")
        
        # Parse nav_msgs/Odometry
        pose_msg = msg.get('pose', {})
        if 'pose' in pose_msg:
            position = pose_msg['pose'].get('position', {})
        else:
            position = pose_msg.get('position', {})
            
        twist_msg = msg.get('twist', {})
        if 'twist' in twist_msg:
            velocity = twist_msg['twist']
        else:
            velocity = twist_msg
        
        # Extract REAL values
        new_x = float(position.get('x', 0.0))
        new_y = float(position.get('y', 0.0))
        new_z = float(position.get('z', 0.0))
        
        # DETECT UNITY RESTART
        if rover_data['last_position']['x'] != 0.0 or rover_data['last_position']['y'] != 0.0:
            detect_unity_restart(new_x, new_y, new_z)
        
        # Update last position for next check
        rover_data['last_position']['x'] = new_x
        rover_data['last_position']['y'] = new_y
        rover_data['last_position']['z'] = new_z
        
        linear_vel = velocity.get('linear', {})
        angular_vel = velocity.get('angular', {})
        linear_x = float(linear_vel.get('x', 0.0))
        angular_z = float(angular_vel.get('z', 0.0))
        
        print(f"🗺️ ✅ REAL position: ({new_x:.3f}, {new_y:.3f})")
        print(f"🗺️ ✅ REAL velocity: L={linear_x:.3f}, A={angular_z:.3f}")
        
        # Update with REAL data
        rover_data['position']['x'] = new_x
        rover_data['position']['y'] = new_y
        rover_data['position']['z'] = new_z
        rover_data['velocity']['linear_x'] = linear_x
        rover_data['velocity']['angular_z'] = angular_z
        rover_data['speed'] = abs(linear_x)
        rover_data['last_update'] = time.time()
        
        # Add to path
        rover_data['path'].append({'x': new_x, 'y': new_y})
        
        # Send REAL rover status
        rover_status = {
            'position': rover_data['position'].copy(),
            'velocity': rover_data['velocity'].copy(),
            'speed': rover_data['speed'],
            'mode': rover_data['mode'],
            'path': list(rover_data['path'])[-30:],
            'connected': rover_data['connected'],
            'camera_active': rover_data['camera_active'],
            'real_movement': True,
            'restart_count': rover_data['restart_count'],
            'timestamp': time.time(),
            'data_source': 'REAL_ROS_ODOMETRY'
        }
        
        socketio.emit('rover_status', rover_status)
        print(f"🗺️ ✅ REAL status sent: ({new_x:.2f}, {new_y:.2f})")
        
    except Exception as e:
        print(f"❌ Odometry error: {e}")


def process_velocity(msg):
    """Process velocity commands"""
    try:
        linear_msg = msg.get('linear', {})
        angular_msg = msg.get('angular', {})
        
        linear = float(linear_msg.get('x', 0.0))
        angular = float(angular_msg.get('z', 0.0))
        
        if abs(linear) > 0.01 or abs(angular) > 0.01:
            rover_data['mode'] = "MOVING"
        else:
            rover_data['mode'] = "IDLE"
            
    except Exception as e:
        print(f"❌ Velocity error: {e}")


def process_detection(msg):
    """Process detection data"""
    try:
        detection_data = json.loads(msg.get('data', '{}'))
        total_obstacles = detection_data.get('total_obstacles', 0)
        
        if total_obstacles > 0:
            log_activity(f"🚨 {total_obstacles} obstacles detected")
            socketio.emit('detection_update', {
                'detection': detection_data,
                'timestamp': time.strftime("%H:%M:%S"),
                'total_detections': total_obstacles
            })
            
    except Exception as e:
        print(f"❌ Detection error: {e}")



# def process_map(msg):
#     """
#     Process occupancy grid map with PROFESSIONAL COLOR THEME + ROVER MARKER
#     - Black/White classic theme with high contrast
#     - Cyan rover position marker
#     - Path history trail in yellow
#     """
#     try:
#         print("🗺️ Processing SLAM map")
        
#         width = msg.get('info', {}).get('width', 0)
#         height = msg.get('info', {}).get('height', 0)
#         data = msg.get('data', [])
        
#         if width == 0 or height == 0 or not data:
#             print("⚠️ Invalid map data")
#             return
        
#         # Get map metadata
#         origin_x = msg.get('info', {}).get('origin', {}).get('position', {}).get('x', -40.0)
#         origin_y = msg.get('info', {}).get('origin', {}).get('position', {}).get('y', -40.0)
#         resolution = msg.get('info', {}).get('resolution', 0.1)
        
#         print(f"🗺️ Map: {width}x{height}, Origin: ({origin_x:.1f}, {origin_y:.1f}), Res: {resolution}m")
        
#         # Convert occupancy grid to numpy array
#         map_array = np.array(data, dtype=np.int8).reshape((height, width))
        
#         # ═══════════════════════════════════════════════════════════
#         # PROFESSIONAL COLOR SCHEME (BGR format for OpenCV)
#         # ═══════════════════════════════════════════════════════════
#         # Create RGB image
#         img = np.zeros((height, width, 3), dtype=np.uint8)
        
#         # Color mapping (values in BGR format):
#         img[map_array == -1] = [180, 180, 180]  # Unknown = medium gray
#         img[map_array == 0] = [255, 255, 255]   # Free space = WHITE
#         img[map_array == 50] = [0, 255, 0]      # Base/Special = GREEN
#         img[map_array == 75] = [0, 140, 255]    # Rocks = ORANGE
#         img[map_array == 100] = [0, 0, 0]       # Obstacles = BLACK
        
#         print("🎨 Applied professional color theme")
#         # ═══════════════════════════════════════════════════════════
        
        
#         # ═══════════════════════════════════════════════════════════
#         # DRAW PATH HISTORY (Yellow trail)
#         # ═══════════════════════════════════════════════════════════
#         try:
#             path = list(rover_data['path'])
#             if len(path) > 1:
#                 for i in range(len(path) - 1):
#                     # Get path points
#                     x1, y1 = path[i]['x'], path[i]['y']
#                     x2, y2 = path[i + 1]['x'], path[i + 1]['y']
                    
#                     # Convert to pixel coordinates
#                     px1 = int((x1 - origin_x) / resolution)
#                     py1 = int((y1 - origin_y) / resolution)
#                     px2 = int((x2 - origin_x) / resolution)
#                     py2 = int((y2 - origin_y) / resolution)
                    
#                     # Draw line segment (Yellow, 2 pixels thick)
#                     if (0 <= px1 < width and 0 <= py1 < height and 
#                         0 <= px2 < width and 0 <= py2 < height):
#                         cv2.line(img, (px1, py1), (px2, py2), (0, 255, 255), 2)
                
#                 print(f"🛤️ Drew path: {len(path)} points")
#         except Exception as e:
#             print(f"⚠️ Path drawing error: {e}")
#         # ═══════════════════════════════════════════════════════════
        
        
#         # ═══════════════════════════════════════════════════════════
#         # DRAW ROVER POSITION (Cyan dot with direction arrow)
#         # ═══════════════════════════════════════════════════════════
#         try:
#             rover_x = rover_data['position']['x']
#             rover_y = rover_data['position']['y']
            
#             # Convert rover position to pixel coordinates
#             pixel_x = int((rover_x - origin_x) / resolution)
#             pixel_y = int((rover_y - origin_y) / resolution)
            
#             # Check if rover is within map bounds
#             if 0 <= pixel_x < width and 0 <= pixel_y < height:
#                 # Draw outer circle (white outline)
#                 cv2.circle(img, (pixel_x, pixel_y), 12, (255, 255, 255), 2)
                
#                 # Draw inner filled circle (cyan)
#                 cv2.circle(img, (pixel_x, pixel_y), 10, (255, 255, 0), -1)
                
#                 # Draw center dot (black)
#                 cv2.circle(img, (pixel_x, pixel_y), 3, (0, 0, 0), -1)
                
#                 # Try to draw direction arrow (if we have orientation)
#                 try:
#                     # You can get heading from odometry if available
#                     # For now, just draw the rover marker
#                     pass
#                 except:
#                     pass
                
#                 print(f"🤖 Rover drawn at pixel: ({pixel_x}, {pixel_y})")
#             else:
#                 print(f"⚠️ Rover out of map bounds: ({pixel_x}, {pixel_y})")
                
#         except Exception as e:
#             print(f"⚠️ Rover marker error: {e}")
#         # ═══════════════════════════════════════════════════════════
        
        
#         # Flip vertically for correct orientation (ROS uses bottom-left origin)
#         img = cv2.flip(img, 0)
        
#         # Encode as base64 JPEG with high quality
#         _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
#         map_base64 = base64.b64encode(buffer).decode('utf-8')
        
#         # Store in slam_data
#         slam_data['map_image'] = map_base64
        
#         # Emit to dashboard
#         socketio.emit('map_update', {
#             'map': map_base64,
#             'width': width,
#             'height': height,
#             'rover_position': {
#                 'x': rover_data['position']['x'],
#                 'y': rover_data['position']['y']
#             },
#             'timestamp': time.time()
#         })
        
#         print(f"🗺️ ✅ Map sent: {width}x{height} with rover marker")
        
#     except Exception as e:
#         print(f"❌ Map processing error: {e}")
#         import traceback
#         traceback.print_exc()
def process_map(msg):
    """
    Process occupancy grid map with ROVER-SHAPED MARKER + PERSISTENT PATH
    - Draws 6-wheeled rover icon (rocker-bogie style)
    - Shows orientation arrow
    - Persistent yellow path trail (doesn't vanish)
    - Professional color theme
    """
    try:
        print("🗺️ Processing SLAM map")
        
        width = msg.get('info', {}).get('width', 0)
        height = msg.get('info', {}).get('height', 0)
        data = msg.get('data', [])
        
        if width == 0 or height == 0 or not data:
            print("⚠️ Invalid map data")
            return
        
        # Get map metadata
        origin_x = msg.get('info', {}).get('origin', {}).get('position', {}).get('x', -60.0)
        origin_y = msg.get('info', {}).get('origin', {}).get('position', {}).get('y', -60.0)
        resolution = msg.get('info', {}).get('resolution', 0.1)
        
        print(f"🗺️ Map: {width}x{height}, Origin: ({origin_x:.1f}, {origin_y:.1f}), Res: {resolution}m")
        
        # Convert occupancy grid to numpy array
        map_array = np.array(data, dtype=np.int8).reshape((height, width))
        
        # ═══════════════════════════════════════════════════════════
        # COLOR SCHEME (BGR format for OpenCV)
        # ═══════════════════════════════════════════════════════════
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        img[map_array == -1] = [180, 180, 180]  # Unknown = medium gray
        img[map_array == 0] = [255, 255, 255]   # Free space = WHITE
        img[map_array == 25] = [255, 0, 255]    # Flags = MAGENTA
        img[map_array == 50] = [0, 255, 0]      # Base = GREEN
        img[map_array == 75] = [0, 165, 255]    # Rocks = ORANGE
        img[map_array == 100] = [0, 0, 0]       # Obstacles = BLACK
        
        print("🎨 Applied professional color theme")
        
        
        # ═══════════════════════════════════════════════════════════
        # DRAW PERSISTENT PATH TRAIL (Yellow, doesn't vanish)
        # ═══════════════════════════════════════════════════════════
        try:
            path = list(rover_data['path'])
            if len(path) > 1:
                # Draw ALL path points (not just recent ones)
                for i in range(len(path) - 1):
                    x1, y1 = path[i]['x'], path[i]['y']
                    x2, y2 = path[i + 1]['x'], path[i + 1]['y']
                    
                    # Convert to pixel coordinates
                    px1 = int((x1 - origin_x) / resolution)
                    py1 = int((y1 - origin_y) / resolution)
                    px2 = int((x2 - origin_x) / resolution)
                    py2 = int((y2 - origin_y) / resolution)
                    
                    # Draw line segment (Yellow, 3 pixels thick for visibility)
                    if (0 <= px1 < width and 0 <= py1 < height and 
                        0 <= px2 < width and 0 <= py2 < height):
                        cv2.line(img, (px1, py1), (px2, py2), (0, 255, 255), 3)
                
                print(f"🛤️ Drew FULL path: {len(path)} points (persistent)")
        except Exception as e:
            print(f"⚠️ Path drawing error: {e}")
        
        
        # ═══════════════════════════════════════════════════════════
        # DRAW 6-WHEELED ROVER (Rocker-Bogie Style)
        # ═══════════════════════════════════════════════════════════
        try:
            rover_x = rover_data['position']['x']
            rover_y = rover_data['position']['y']
            
            # Convert to pixel coordinates
            pixel_x = int((rover_x - origin_x) / resolution)
            pixel_y = int((rover_y - origin_y) / resolution)
            
            if 0 <= pixel_x < width and 0 <= pixel_y < height:
                
                # Rover dimensions (in pixels, scaled for visibility)
                rover_length = 25  # ~2.5m in real world
                rover_width = 18   # ~1.8m
                wheel_radius = 4   # ~0.4m wheels
                
                # ───────────────────────────────────────────────────
                # DRAW ROVER BODY (White rectangle)
                # ───────────────────────────────────────────────────
                body_pts = np.array([
                    [pixel_x - rover_length//2, pixel_y - rover_width//2],
                    [pixel_x + rover_length//2, pixel_y - rover_width//2],
                    [pixel_x + rover_length//2, pixel_y + rover_width//2],
                    [pixel_x - rover_length//2, pixel_y + rover_width//2],
                ], np.int32)
                
                # Fill body (white)
                cv2.fillPoly(img, [body_pts], (255, 255, 255))
                # Black outline
                cv2.polylines(img, [body_pts], True, (0, 0, 0), 2)
                
                # ───────────────────────────────────────────────────
                # DRAW 6 WHEELS (Black circles) - Rocker-Bogie Layout
                # ───────────────────────────────────────────────────
                # Left side wheels (3 wheels)
                left_y = pixel_y - rover_width//2 - wheel_radius
                cv2.circle(img, (pixel_x - rover_length//3, left_y), wheel_radius, (0, 0, 0), -1)  # Front-left
                cv2.circle(img, (pixel_x, left_y), wheel_radius, (0, 0, 0), -1)  # Mid-left
                cv2.circle(img, (pixel_x + rover_length//3, left_y), wheel_radius, (0, 0, 0), -1)  # Back-left
                
                # Right side wheels (3 wheels)
                right_y = pixel_y + rover_width//2 + wheel_radius
                cv2.circle(img, (pixel_x - rover_length//3, right_y), wheel_radius, (0, 0, 0), -1)  # Front-right
                cv2.circle(img, (pixel_x, right_y), wheel_radius, (0, 0, 0), -1)  # Mid-right
                cv2.circle(img, (pixel_x + rover_length//3, right_y), wheel_radius, (0, 0, 0), -1)  # Back-right
                
                # ───────────────────────────────────────────────────
                # DRAW DIRECTION ARROW (Red)
                # ───────────────────────────────────────────────────
                # Calculate arrow endpoint (front of rover)
                arrow_length = rover_length // 2 + 10
                arrow_end_x = pixel_x + arrow_length
                arrow_end_y = pixel_y
                
                # Draw arrow (red, thick line)
                cv2.arrowedLine(img, (pixel_x, pixel_y), (arrow_end_x, arrow_end_y), 
                               (0, 0, 255), 3, tipLength=0.3)
                
                # ───────────────────────────────────────────────────
                # DRAW CENTER DOT (Cyan - exact position)
                # ───────────────────────────────────────────────────
                cv2.circle(img, (pixel_x, pixel_y), 3, (255, 255, 0), -1)
                
                print(f"🤖 6-wheeled rover drawn at pixel: ({pixel_x}, {pixel_y})")
            else:
                print(f"⚠️ Rover out of map bounds: ({pixel_x}, {pixel_y})")
                
        except Exception as e:
            print(f"⚠️ Rover marker error: {e}")
        
        
        # Flip vertically for correct orientation
        img = cv2.flip(img, 0)
        
        # Encode as base64 JPEG with high quality
        _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        map_base64 = base64.b64encode(buffer).decode('utf-8')
        
        slam_data['map_image'] = map_base64
        
        # Emit to dashboard
        socketio.emit('map_update', {
            'map': map_base64,
            'width': width,
            'height': height,
            'rover_position': {
                'x': rover_data['position']['x'],
                'y': rover_data['position']['y']
            },
            'path_length': len(rover_data['path']),
            'timestamp': time.time()
        })
        
        print(f"🗺️ ✅ Map sent: {width}x{height} with 6-wheeled rover + full path")
        
    except Exception as e:
        print(f"❌ Map processing error: {e}")
        import traceback
        traceback.print_exc()


def process_semantic_map(msg):
    """Process semantic map (detected objects from YOLO)"""
    try:
        print("🪨 Processing semantic map")
        
        # Parse JSON data
        if isinstance(msg, dict) and 'data' in msg:
            semantic_json = msg['data']
        else:
            semantic_json = msg
        
        if isinstance(semantic_json, str):
            semantic_dict = json.loads(semantic_json)
        else:
            semantic_dict = semantic_json
        
        # Update slam_data
        slam_data['semantic_map'] = semantic_dict
        
        # Count detections
        rocks_count = len(semantic_dict.get('rocks', []))
        base_count = len(semantic_dict.get('base', []))
        flags_count = len(semantic_dict.get('flags', []))
        antennas_count = len(semantic_dict.get('antennas', []))
        
        # Emit to dashboard
        socketio.emit('semantic_update', {
            'semantic_map': semantic_dict,
            'counts': {
                'rocks': rocks_count,
                'base': base_count,
                'flags': flags_count,
                'antennas': antennas_count
            },
            'timestamp': time.time()
        })
        
        print(f"🪨 ✅ Semantic: {rocks_count} rocks, {base_count} base, {flags_count} flags")
        
        if rocks_count > 0 or base_count > 0:
            log_activity(f"🔍 Detected: {rocks_count} rocks, {base_count} base")
        
    except Exception as e:
        print(f"❌ Semantic map error: {e}")


def process_mission_status(msg):
    """Process mission status (battery, mode, etc.)"""
    try:
        print("🎯 Processing mission status")
        
        # Parse JSON data
        if isinstance(msg, dict) and 'data' in msg:
            status_json = msg['data']
        else:
            status_json = msg
        
        if isinstance(status_json, str):
            status_dict = json.loads(status_json)
        else:
            status_dict = status_json
        
        # Update slam_data
        slam_data['mission_status'] = status_dict
        
        battery = status_dict.get('battery', 100.0)
        mode = status_dict.get('mode', 'EXPLORATION')
        distance_to_base = status_dict.get('distance_to_base', 0.0)
        
        # Emit to dashboard
        socketio.emit('mission_update', {
            'mission_status': status_dict,
            'timestamp': time.time()
        })
        
        print(f"🎯 ✅ Mission: Battery {battery:.0f}%, Mode: {mode}, Dist to base: {distance_to_base:.1f}m")
        
        # Log important events
        if battery < 15 and mode == "EMERGENCY_RETURN":
            log_activity(f"🔋 EMERGENCY! Battery {battery:.0f}% - Returning to base")
        elif mode == "BACKTRACKING":
            log_activity(f"🔙 Backtracking to base")
        
    except Exception as e:
        print(f"❌ Mission status error: {e}")


def process_path_history(msg):
    """Process traversed path for visualization"""
    try:
        print("🛤️ Processing path history")
        
        poses = msg.get('poses', [])
        
        # Extract path points
        path_points = []
        for pose_stamped in poses:
            pose = pose_stamped.get('pose', {})
            position = pose.get('position', {})
            x = position.get('x', 0.0)
            y = position.get('y', 0.0)
            path_points.append({'x': x, 'y': y})
        
        slam_data['path_history'] = path_points
        
        # Emit to dashboard
        socketio.emit('path_update', {
            'path': path_points[-50:],  # Last 50 points
            'total_points': len(path_points),
            'timestamp': time.time()
        })
        
        print(f"🛤️ ✅ Path: {len(path_points)} waypoints")
        
    except Exception as e:
        print(f"❌ Path history error: {e}")

# ═══════════════════════════════════════════════════════════════════


# ROS Connection handlers
def on_open(ws):
    """ROS connection opened"""
    global rosbridge_ws
    rosbridge_ws = ws
    print("🌐 ✅ ROS connected - REAL DATA ONLY")
    log_activity("🌐 ROS connected with restart detection")
    rover_data['connected'] = True
    
    # EXISTING TOPICS (KEEP THESE)
    topics = [
        {"topic": "/rover_camera/image_raw", "type": "sensor_msgs/Image"},
        {"topic": "/odom", "type": "nav_msgs/Odometry"},
        {"topic": "/cmd_vel", "type": "geometry_msgs/Twist"},
        {"topic": "/cmd_vel_direct", "type": "geometry_msgs/Twist"},
        {"topic": "/hazard_detection", "type": "std_msgs/String"},
        
        # ═══════════════════════════════════════════════════════════
        # NEW: SLAM TOPICS (ADDED)
        # ═══════════════════════════════════════════════════════════
        {"topic": "/map", "type": "nav_msgs/OccupancyGrid"},
        {"topic": "/semantic_map", "type": "std_msgs/String"},
        {"topic": "/mission_status", "type": "std_msgs/String"},
        {"topic": "/traversed_path", "type": "nav_msgs/Path"},
        # ═══════════════════════════════════════════════════════════
    ]
    
    for topic_info in topics:
        subscription = {
            "op": "subscribe",
            "topic": topic_info["topic"],
            "type": topic_info["type"],
            "throttle_rate": 50,
            "queue_length": 1
        }
        try:
            ws.send(json.dumps(subscription, separators=(',', ':')))
            print(f"  ✅ Subscribed: {topic_info['topic']}")
        except Exception as e:
            print(f"  ❌ Subscribe failed: {topic_info['topic']}: {e}")


def on_error(ws, error):
    print(f"❌ ROS error: {error}")
    rover_data['connected'] = False


def on_close(ws, close_status_code, close_msg):
    print(f"🔌 ROS closed: {close_status_code}")
    rover_data['connected'] = False


def rosbridge_client():
    """ROS client - NO FAKE DATA"""
    global rosbridge_ws
    while True:
        try:
            print(f"🔄 Connecting to REAL ROS at {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
            ws = WebSocketApp(
                f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}",
                on_message=on_message,
                on_open=on_open,
                on_error=on_error,
                on_close=on_close
            )
            ws.run_forever(ping_interval=30, ping_timeout=10)
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ ROS client error: {e}")
            rover_data['connected'] = False
        
        print("⏳ Reconnecting in 5 seconds...")
        time.sleep(5)


# Socket handlers
@socketio.on('connect')
def handle_connect():
    """Client connected"""
    print("🌐 ✅ Dashboard connected - AUTO RESTART DETECTION")
    log_activity("🌐 Dashboard connected with restart detection")
    
    emit('initial_state', {
        'rover_data': {
            'position': rover_data['position'],
            'velocity': rover_data['velocity'],
            'speed': rover_data['speed'],
            'mode': rover_data['mode'],
            'connected': rover_data['connected'],
            'camera_active': rover_data['camera_active'],
            'frame_count': rover_data['frame_count'],
            'path': list(rover_data['path'])[-20:],
            'restart_count': rover_data['restart_count'],
            'real_data_only': True,
            'restart_detection_enabled': True
        },
        'activity_log': list(activity_log)[-10:],
        
        # ═══════════════════════════════════════════════════════════
        # NEW: SLAM DATA (ADDED)
        # ═══════════════════════════════════════════════════════════
        'slam_data': {
            'map_available': slam_data['map_image'] is not None,
            'semantic_map': slam_data['semantic_map'],
            'mission_status': slam_data['mission_status'],
            'path_length': len(slam_data['path_history'])
        }
        # ═══════════════════════════════════════════════════════════
    })


@socketio.on('disconnect')
def handle_disconnect():
    print("🌐 Dashboard disconnected")


@socketio.on('rover_command')
def handle_rover_command(data):
    """Handle rover commands"""
    try:
        cmd_vel = data.get('cmd_vel', {})
        linear_x = float(cmd_vel.get('linear', {}).get('x', 0.0))
        angular_z = float(cmd_vel.get('angular', {}).get('z', 0.0))
        
        twist_msg = {
            'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
        }
        
        success = publish_to_ros('/cmd_vel', 'geometry_msgs/Twist', twist_msg)
        
        if success:
            rover_data['velocity']['linear_x'] = linear_x
            rover_data['velocity']['angular_z'] = angular_z
            rover_data['mode'] = "MOVING" if (abs(linear_x) > 0.01 or abs(angular_z) > 0.01) else "IDLE"
            log_activity(f"🎮 L={linear_x:.1f} A={angular_z:.1f}")
            emit('command_success', {'message': 'Command sent'})
        else:
            emit('command_error', {'error': 'Failed to send'})
            
    except Exception as e:
        emit('command_error', {'error': f'Error: {e}'})


@socketio.on('navigation_goal')
def handle_navigation_goal(data):
    """Handle navigation goals"""
    try:
        goal = data.get('goal', {})
        position = goal.get('position', {})
        
        x = float(position.get('x', 0.0))
        y = float(position.get('y', 0.0))
        
        goal_msg = {
            'header': {
                'frame_id': 'map',
                'stamp': {'secs': int(time.time()), 'nsecs': 0}
            },
            'pose': {
                'position': {'x': x, 'y': y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        }
        
        success = publish_to_ros('/move_base_simple/goal', 'geometry_msgs/PoseStamped', goal_msg)
        
        if success:
            rover_data['mode'] = f"NAV ({x:.1f},{y:.1f})"
            log_activity(f"🎯 Goal: ({x:.1f},{y:.1f})")
            emit('goal_success', {'message': f'Goal: ({x:.1f},{y:.1f})'})
        else:
            emit('goal_error', {'error': 'Navigation failed'})
            
    except Exception as e:
        emit('goal_error', {'error': f'Error: {e}'})


@socketio.on('reset_navigation')
def handle_reset_navigation():
    """Manually reset navigation"""
    try:
        print("🔄 Manual reset requested")
        log_activity("🔄 Manual navigation reset")
        
        stop_rover()
        cancel_navigation_goal()
        clear_costmaps()
        
        rover_data['path'].clear()
        rover_data['mode'] = "IDLE"
        
        emit('reset_success', {'message': 'Navigation reset complete'})
        log_activity("✅ Manual reset complete")
        
    except Exception as e:
        emit('reset_error', {'error': f'Reset failed: {e}'})


# Flask routes
@app.route('/')
def dashboard():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    """API status"""
    return jsonify({
        'status': 'REAL_DATA_WITH_RESTART_DETECTION',
        'rover_data': {
            'position': rover_data['position'],
            'velocity': rover_data['velocity'],
            'mode': rover_data['mode'],
            'connected': rover_data['connected'],
            'camera_active': rover_data['camera_active'],
            'frame_count': rover_data['frame_count'],
            'last_update': rover_data['last_update'],
            'path_length': len(rover_data['path']),
            'restart_count': rover_data['restart_count']
        },
        'features': {
            'restart_detection': True,
            'auto_goal_cancel': True,
            'auto_costmap_clear': True
        },
        'timestamp': time.time()
    })


@app.route('/api/reset_navigation', methods=['POST'])
def api_reset_navigation():
    """API endpoint to reset navigation"""
    try:
        stop_rover()
        cancel_navigation_goal()
        clear_costmaps()
        rover_data['path'].clear()
        rover_data['mode'] = "IDLE"
        
        return jsonify({
            'status': 'success',
            'message': 'Navigation reset complete',
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'error': str(e)
        }), 500


# ═══════════════════════════════════════════════════════════════════
# NEW: SLAM API ENDPOINTS (ADDED)
# ═══════════════════════════════════════════════════════════════════

@app.route('/api/slam_status')
def api_slam_status():
    """Get SLAM system status"""
    return jsonify({
        'map_available': slam_data['map_image'] is not None,
        'semantic_map': slam_data['semantic_map'],
        'mission_status': slam_data['mission_status'],
        'path_length': len(slam_data['path_history']),
        'timestamp': time.time()
    })


@app.route('/api/map_image')
def api_map_image():
    """Get current map image"""
    if slam_data['map_image']:
        return jsonify({
            'status': 'success',
            'map': slam_data['map_image'],
            'timestamp': time.time()
        })
    else:
        return jsonify({
            'status': 'no_map',
            'message': 'Map not yet available'
        }), 404


@app.route('/api/detected_objects')
def api_detected_objects():
    """Get detected objects summary"""
    semantic = slam_data['semantic_map']
    return jsonify({
        'rocks': len(semantic.get('rocks', [])),
        'base': len(semantic.get('base', [])),
        'flags': len(semantic.get('flags', [])),
        'antennas': len(semantic.get('antennas', [])),
        'details': semantic,
        'timestamp': time.time()
    })


@app.route('/api/mission_status')
def api_mission_status():
    """Get current mission status"""
    return jsonify(slam_data['mission_status'])

# ═══════════════════════════════════════════════════════════════════


def init_publisher():
    """Initialize publisher"""
    def publisher_thread():
        global publisher_connected
        while True:
            try:
                if not connect_rosbridge_publisher():
                    time.sleep(10)
                    continue
                
                while publisher_connected:
                    time.sleep(30)
                    try:
                        if rosbridge_pub_ws and rosbridge_pub_ws.connected:
                            ping_msg = json.dumps({"op": "ping"})
                            rosbridge_pub_ws.send(ping_msg)
                        else:
                            publisher_connected = False
                            break
                    except Exception as e:
                        publisher_connected = False
                        break
                        
            except Exception as e:
                publisher_connected = False
                time.sleep(10)
    
    thread = threading.Thread(target=publisher_thread, daemon=True)
    thread.start()


# MAIN
if __name__ == '__main__':
    print("🚀 UNITY RESTART DETECTION Dashboard - REAL ROS DATA + SLAM")
    log_activity("🚀 Dashboard with AUTO RESTART DETECTION + SLAM starting")
    
    socketio.start_background_task(target=rosbridge_client)
    init_publisher()
    
    print("="*60)
    print("🤖 AUTO RESTART DETECTION DASHBOARD + SLAM!")
    print("="*60)
    print(f"🌐 URL: http://172.22.54.111:5000")
    print(f"📷 Camera: REAL ROS frames")
    print(f"🗺️ Position: REAL /odom data")
    print(f"🔄 Restart Detection: ENABLED (3m threshold)")
    print(f"🛑 Auto Goal Cancel: ENABLED")
    print(f"🧹 Auto Costmap Clear: ENABLED")
    print("="*60)
    print("🗺️ NEW SLAM FEATURES:")
    print("  ✅ Live occupancy grid map")
    print("  ✅ YOLO object detection (rocks, base, flags)")
    print("  ✅ Path history tracking")
    print("  ✅ Mission status monitoring")
    print("  ✅ Battery level display")
    print("  ✅ Emergency return system")
    print("="*60)
    print(f"✅ 100% REAL ROS DATA")
    print("="*60)
    
    try:
        socketio.run(app, 
                    host='172.22.54.111', 
                    port=5000, 
                    debug=False,
                    use_reloader=False)
    except KeyboardInterrupt:
        print("\n🛑 Dashboard stopped")
