#!/usr/bin/env python3
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
from concurrent.futures import ThreadPoolExecutor
from queue import Queue, Empty

# Configuration
ROSBRIDGE_HOST = 'localhost'
ROSBRIDGE_PORT = 9090

# Flask + SocketIO
app = Flask(__name__)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='eventlet',
    logger=False,
    engineio_logger=False
)

# Rover state
rover_data = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "last_position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "path": deque(maxlen=500),
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

# SLAM data
slam_data = {
    'map_image': None,
    'semantic_map': {'rocks': [], 'base': [], 'flags': [], 'antennas': []},
    'mission_status': {
        'battery': 100.0,
        'mode': 'EXPLORATION',
        'base_x': 0.0,
        'base_y': 0.0,
        'base_detected': False,
        'distance_to_base': 0.0,
        'path_length': 0
    },
    'path_history': []
}

# Queues and workers
PROCESSING_QUEUE = Queue(maxsize=100)
EMIT_QUEUE = Queue(maxsize=200)
CPU_POOL = ThreadPoolExecutor(max_workers=2)
_WORKERS_STARTED = False

rosbridge_ws = None
rosbridge_pub_ws = None
publisher_connected = False


def log_activity(message):
    try:
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        activity_log.append(log_entry)
        try:
            EMIT_QUEUE.put_nowait(('activity_update', {'message': log_entry}))
        except Exception:
            pass
        print(f"📝 {message}")
    except Exception as e:
        print(f"❌ Log activity error: {e}")


def start_background_workers():
    global _WORKERS_STARTED
    if _WORKERS_STARTED:
        return
    _WORKERS_STARTED = True

    def processing_worker():
        while True:
            try:
                job = PROCESSING_QUEUE.get()
                if job is None:
                    break
                job_type, payload = job
                CPU_POOL.submit(_process_job, job_type, payload)
            except Exception as e:
                print("❌ processing_worker error:", e)

    def emit_worker():
        DEFAULT_MIN_DELAY = 0.02
        EVENT_INTERVALS = {
            'camera_update': 0.2,
            'map_update': 1.0,
            'semantic_update': 0.5
        }
        last_emit = {}
        while True:
            try:
                item = EMIT_QUEUE.get(timeout=1.0)
                if item is None:
                    break
                event, payload = item
                try:
                    now = time.time()
                    min_delay = EVENT_INTERVALS.get(event, DEFAULT_MIN_DELAY)
                    elapsed = now - last_emit.get(event, 0.0)
                    if elapsed < min_delay:
                        time.sleep(min_delay - elapsed)
                    socketio.emit(event, payload)
                    last_emit[event] = time.time()
                except Exception as e:
                    print("❌ emit_worker error:", e)
            except Empty:
                continue
            except Exception as e:
                print("❌ emit_worker outer error:", e)

    threading.Thread(target=processing_worker, daemon=True).start()
    threading.Thread(target=emit_worker, daemon=True).start()


def _process_job(job_type, payload):
    try:
        if job_type == 'map':
            _process_map_heavy(payload)
        elif job_type == 'camera':
            _process_camera_heavy(payload)
        elif job_type == 'semantic':
            process_semantic_map(payload)
        elif job_type == 'mission_status':
            process_mission_status(payload)
        elif job_type == 'path_history':
            process_path_history(payload)
        else:
            print("⚠️ Unknown job type:", job_type)
    except Exception as e:
        print("❌ _process_job error:", e)


def _process_map_heavy(msg):
    try:
        width = msg.get('info', {}).get('width', 0)
        height = msg.get('info', {}).get('height', 0)
        data = msg.get('data', [])
        if width == 0 or height == 0 or not data:
            return

        origin_x = msg.get('info', {}).get('origin', {}).get('position', {}).get('x', -60.0)
        origin_y = msg.get('info', {}).get('origin', {}).get('position', {}).get('y', -60.0)
        resolution = msg.get('info', {}).get('resolution', 0.1)

        map_array = np.array(data, dtype=np.int8).reshape((height, width))
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Color mapping (BGR)
        img[map_array == -1] = [180, 180, 180]
        img[map_array == 0] = [255, 255, 255]
        img[map_array == 25] = [255, 0, 255]
        img[map_array == 50] = [0, 255, 0]
        img[map_array == 75] = [0, 165, 255]
        img[map_array == 100] = [0, 0, 0]

        # Draw path (downsample if too long)
        path = list(rover_data['path'])
        if len(path) > 1:
            step = max(1, len(path) // 200)
            for i in range(0, len(path) - 1, step):
                p1 = path[i]
                p2 = path[min(i + step, len(path) - 1)]
                px1 = int((p1['x'] - origin_x) / resolution)
                py1 = int((p1['y'] - origin_y) / resolution)
                px2 = int((p2['x'] - origin_x) / resolution)
                py2 = int((p2['y'] - origin_y) / resolution)
                if (0 <= px1 < width and 0 <= py1 < height and 0 <= px2 < width and 0 <= py2 < height):
                    cv2.line(img, (px1, py1), (px2, py2), (0, 255, 255), 2)

        # Rover marker
        rover_x = rover_data['position']['x']
        rover_y = rover_data['position']['y']
        pixel_x = int((rover_x - origin_x) / resolution)
        pixel_y = int((rover_y - origin_y) / resolution)
        if 0 <= pixel_x < width and 0 <= pixel_y < height:
            cv2.circle(img, (pixel_x, pixel_y), 4, (255, 255, 0), -1)

        img = cv2.flip(img, 0)
        _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        map_base64 = base64.b64encode(buffer).decode('utf-8')

        payload = {
            'map': map_base64,
            'width': width,
            'height': height,
            'rover_position': {'x': rover_x, 'y': rover_y},
            'path_length': len(rover_data['path']),
            'timestamp': time.time()
        }
        try:
            EMIT_QUEUE.put_nowait(('map_update', payload))
        except Exception:
            pass

    except Exception as e:
        print("❌ _process_map_heavy error:", e)


def _process_camera_heavy(msg):
    try:
        width = int(msg.get('width', 0))
        height = int(msg.get('height', 0))
        encoding = msg.get('encoding', 'rgb8')
        image_data = msg.get('data')
        if not image_data:
            return

        if encoding in ('jpeg', 'jpg'):
            base64_data = image_data if isinstance(image_data, str) else base64.b64encode(bytes(image_data)).decode('utf-8')
        else:
            if isinstance(image_data, str):
                try:
                    image_bytes = base64.b64decode(image_data)
                except Exception:
                    image_bytes = image_data.encode()
            elif isinstance(image_data, list):
                image_bytes = bytes(image_data)
            else:
                image_bytes = image_data

            try:
                np_arr = np.frombuffer(image_bytes, np.uint8)
                if encoding == 'rgb8' and width > 0 and height > 0 and np_arr.size == width * height * 3:
                    img = np_arr.reshape((height, width, 3))
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                else:
                    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if img_bgr is None:
                        return
                max_dim = 800
                if max(img_bgr.shape[:2]) > max_dim:
                    scale = max_dim / max(img_bgr.shape[:2])
                    new_w = int(img_bgr.shape[1] * scale)
                    new_h = int(img_bgr.shape[0] * scale)
                    img_bgr = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
                _, jpg_data = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 50])
                base64_data = base64.b64encode(jpg_data.tobytes()).decode('utf-8')
            except Exception as e:
                print("❌ _process_camera_heavy encode error:", e)
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
        try:
            EMIT_QUEUE.put_nowait(('camera_update', camera_update))
        except Exception:
            pass

        print(f"📷 frame {rover_data['frame_count']} queued for emit")

    except Exception as e:
        print("❌ _process_camera_heavy error:", e)


def connect_rosbridge_publisher():
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
    global rosbridge_pub_ws, publisher_connected
    try:
        if not publisher_connected and not connect_rosbridge_publisher():
            print(f"❌ Cannot publish to {topic} - no ROS connection")
            return False

        ros_message = {"op": "publish", "topic": topic, "msg": message}
        rosbridge_pub_ws.send(json.dumps(ros_message, separators=(',', ':')))
        print(f"📤 Published to {topic}: {message}")
        log_activity(f"📤 Sent to ROS: {topic}")
        return True

    except Exception as e:
        print(f"❌ ROS publish error: {e}")
        publisher_connected = False
        return False


def cancel_navigation_goal():
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
    try:
        service_msg = {"op": "call_service", "service": "/move_base/clear_costmaps", "args": {}}
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
    try:
        last_x = rover_data['last_position']['x']
        last_y = rover_data['last_position']['y']
        dx = new_x - last_x
        dy = new_y - last_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance > 3.0 and not rover_data['position_jump_detected']:
            print("="*60)
            print("🔄 UNITY RESTART DETECTED!")
            print(f"   Position jump: {distance:.2f}m")
            print(f"   Old: ({last_x:.1f}, {last_y:.1f})")
            print(f"   New: ({new_x:.1f}, {new_y:.1f})")
            print("="*60)

            rover_data['position_jump_detected'] = True
            rover_data['restart_count'] += 1

            log_activity(f"🔄 RESTART #{rover_data['restart_count']} DETECTED!")
            log_activity(f"   Position jump: {distance:.1f}m")

            print("🛑 Stopping rover...")
            stop_rover()

            print("🛑 Cancelling navigation...")
            cancel_navigation_goal()

            print("🧹 Clearing costmaps...")
            clear_costmaps()

            rover_data['path'].clear()

            socketio_payload = {
                'restart_count': rover_data['restart_count'],
                'old_position': {'x': last_x, 'y': last_y},
                'new_position': {'x': new_x, 'y': new_y},
                'distance_jump': distance,
                'timestamp': time.time()
            }
            try:
                EMIT_QUEUE.put_nowait(('unity_restart_detected', socketio_payload))
            except Exception:
                pass

            print("✅ Restart handling complete!")
            log_activity("✅ Reset complete - ready for new commands")

            def reset_flag():
                time.sleep(2)
                rover_data['position_jump_detected'] = False

            threading.Thread(target=reset_flag, daemon=True).start()
            return True
        return False

    except Exception as e:
        print(f"❌ Restart detection error: {e}")
        return False


def on_message(ws, message):
    try:
        data = json.loads(message)
        topic = data.get("topic")
        if not topic:
            return
        msg = data.get("msg", {})

        if topic == "/rover_camera/image_raw":
            try:
                PROCESSING_QUEUE.put_nowait(('camera', msg))
            except Exception:
                pass
        elif topic == "/odom":
            process_odometry(msg)
        elif topic in ["/cmd_vel", "/cmd_vel_direct"]:
            process_velocity(msg)
        elif topic == "/hazard_detection":
            process_detection(msg)
        elif topic == "/rtabmap/grid_map":
            try:
                PROCESSING_QUEUE.put_nowait(('map', msg))
            except Exception:
                pass
        elif topic == "/semantic_detections":
            try:
                PROCESSING_QUEUE.put_nowait(('semantic', msg))
            except Exception:
                pass
        elif topic == "/mission_status":
            try:
                PROCESSING_QUEUE.put_nowait(('mission_status', msg))
            except Exception:
                pass
        elif topic == "/traversed_path":
            try:
                PROCESSING_QUEUE.put_nowait(('path_history', msg))
            except Exception:
                pass

    except Exception as e:
        print(f"❌ ROS message error: {e}")


def process_odometry(msg):
    try:
        print("🗺️ Processing odometry")
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

        new_x = float(position.get('x', 0.0))
        new_y = float(position.get('y', 0.0))
        new_z = float(position.get('z', 0.0))

        if rover_data['last_position']['x'] != 0.0 or rover_data['last_position']['y'] != 0.0:
            detect_unity_restart(new_x, new_y, new_z)

        rover_data['last_position']['x'] = new_x
        rover_data['last_position']['y'] = new_y
        rover_data['last_position']['z'] = new_z

        linear_vel = velocity.get('linear', {})
        angular_vel = velocity.get('angular', {})
        linear_x = float(linear_vel.get('x', 0.0))
        angular_z = float(angular_vel.get('z', 0.0))

        print(f"🗺️ ✅ position: ({new_x:.3f}, {new_y:.3f})")
        print(f"🗺️ ✅ velocity: L={linear_x:.3f}, A={angular_z:.3f}")

        rover_data['position']['x'] = new_x
        rover_data['position']['y'] = new_y
        rover_data['position']['z'] = new_z
        rover_data['velocity']['linear_x'] = linear_x
        rover_data['velocity']['angular_z'] = angular_z
        rover_data['speed'] = abs(linear_x)
        rover_data['last_update'] = time.time()

        rover_data['path'].append({'x': new_x, 'y': new_y})

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
            'data_source': 'ROS_ODOMETRY'
        }

        try:
            EMIT_QUEUE.put_nowait(('rover_status', rover_status))
        except Exception:
            pass

        print(f"🗺️ ✅ status sent: ({new_x:.2f}, {new_y:.2f})")

    except Exception as e:
        print(f"❌ Odometry error: {e}")


def process_velocity(msg):
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
    try:
        detection_data = json.loads(msg.get('data', '{}'))
        total_obstacles = detection_data.get('total_obstacles', 0)

        if total_obstacles > 0:
            log_activity(f"🚨 {total_obstacles} obstacles detected")
            try:
                EMIT_QUEUE.put_nowait(('detection_update', {
                    'detection': detection_data,
                    'timestamp': time.strftime("%H:%M:%S"),
                    'total_detections': total_obstacles
                }))
            except Exception:
                pass

    except Exception as e:
        print(f"❌ Detection error: {e}")


def process_semantic_map(msg):
    try:
        print("🪨 Processing semantic map")
        if isinstance(msg, dict) and 'data' in msg:
            semantic_json = msg['data']
        else:
            semantic_json = msg

        if isinstance(semantic_json, str):
            semantic_dict = json.loads(semantic_json)
        else:
            semantic_dict = semantic_json

        slam_data['semantic_map'] = semantic_dict

        rocks_count = len(semantic_dict.get('rocks', []))
        base_count = len(semantic_dict.get('base', []))
        flags_count = len(semantic_dict.get('flags', []))
        antennas_count = len(semantic_dict.get('antennas', []))

        payload = {
            'semantic_map': semantic_dict,
            'counts': {
                'rocks': rocks_count,
                'base': base_count,
                'flags': flags_count,
                'antennas': antennas_count
            },
            'timestamp': time.time()
        }
        try:
            EMIT_QUEUE.put_nowait(('semantic_update', payload))
        except Exception:
            pass

        print(f"🪨 ✅ Semantic: {rocks_count} rocks, {base_count} base, {flags_count} flags")

        if rocks_count > 0 or base_count > 0:
            log_activity(f"🔍 Detected: {rocks_count} rocks, {base_count} base")

    except Exception as e:
        print(f"❌ Semantic map error: {e}")


def process_mission_status(msg):
    try:
        print("🎯 Processing mission status")
        if isinstance(msg, dict) and 'data' in msg:
            status_json = msg['data']
        else:
            status_json = msg

        if isinstance(status_json, str):
            status_dict = json.loads(status_json)
        else:
            status_dict = status_json

        slam_data['mission_status'] = status_dict

        battery = status_dict.get('battery', 100.0)
        mode = status_dict.get('mode', 'EXPLORATION')
        distance_to_base = status_dict.get('distance_to_base', 0.0)

        payload = {'mission_status': status_dict, 'timestamp': time.time()}
        try:
            EMIT_QUEUE.put_nowait(('mission_update', payload))
        except Exception:
            pass

        print(f"🎯 ✅ Mission: Battery {battery:.0f}%, Mode: {mode}, Dist to base: {distance_to_base:.1f}m")

        if battery < 15 and mode == "EMERGENCY_RETURN":
            log_activity(f"🔋 EMERGENCY! Battery {battery:.0f}% - Returning to base")
        elif mode == "BACKTRACKING":
            log_activity("🔙 Backtracking to base")

    except Exception as e:
        print(f"❌ Mission status error: {e}")


def process_path_history(msg):
    try:
        print("🛤️ Processing path history")
        poses = msg.get('poses', [])

        path_points = []
        for pose_stamped in poses:
            pose = pose_stamped.get('pose', {})
            position = pose.get('position', {})
            x = position.get('x', 0.0)
            y = position.get('y', 0.0)
            path_points.append({'x': x, 'y': y})

        slam_data['path_history'] = path_points

        payload = {
            'path': path_points[-50:],
            'total_points': len(path_points),
            'timestamp': time.time()
        }
        try:
            EMIT_QUEUE.put_nowait(('path_update', payload))
        except Exception:
            pass

        print(f"🛤️ ✅ Path: {len(path_points)} waypoints")

    except Exception as e:
        print(f"❌ Path history error: {e}")


def on_open(ws):
    global rosbridge_ws
    rosbridge_ws = ws
    print("🌐 ✅ ROS connected")
    log_activity("🌐 ROS connected with restart detection")
    rover_data['connected'] = True

    topics = [
        {"topic": "/rover_camera/image_raw", "type": "sensor_msgs/Image"},
        {"topic": "/odom", "type": "nav_msgs/Odometry"},
        {"topic": "/cmd_vel", "type": "geometry_msgs/Twist"},
        {"topic": "/cmd_vel_direct", "type": "geometry_msgs/Twist"},
        {"topic": "/hazard_detection", "type": "std_msgs/String"},
        {"topic": "/rtabmap/grid_map", "type": "nav_msgs/OccupancyGrid"},
        {"topic": "/semantic_detections", "type": "std_msgs/String"},
        {"topic": "/mission_status", "type": "std_msgs/String"},
        {"topic": "/traversed_path", "type": "nav_msgs/Path"},
    ]

    for topic_info in topics:
        subscription = {
            "op": "subscribe",
            "topic": topic_info["topic"],
            "type": topic_info["type"],
            "throttle_rate": 500 if '/grid_map' in topic_info["topic"] else 100,
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
    global rosbridge_ws
    while True:
        try:
            print(f"🔄 Connecting to ROS at {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
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


@socketio.on('connect')
def handle_connect():
    print("🌐 ✅ Dashboard connected")
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
        'slam_data': {
            'map_available': slam_data['map_image'] is not None,
            'semantic_map': slam_data['semantic_map'],
            'mission_status': slam_data['mission_status'],
            'path_length': len(slam_data['path_history'])
        }
    })


@socketio.on('disconnect')
def handle_disconnect():
    print("🌐 Dashboard disconnected")


@socketio.on('rover_command')
def handle_rover_command(data):
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
    try:
        goal = data.get('goal', {})
        position = goal.get('position', {})
        x = float(position.get('x', 0.0))
        y = float(position.get('y', 0.0))

        goal_msg = {
            'header': {'frame_id': 'map', 'stamp': {'secs': int(time.time()), 'nsecs': 0}},
            'pose': {'position': {'x': x, 'y': y, 'z': 0.0},
                     'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
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


@app.route('/')
def dashboard():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify({
        'status': 'ROS_DATA_WITH_RESTART_DETECTION',
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
        'features': {'restart_detection': True, 'auto_goal_cancel': True, 'auto_costmap_clear': True},
        'timestamp': time.time()
    })


@app.route('/api/reset_navigation', methods=['POST'])
def api_reset_navigation():
    try:
        stop_rover()
        cancel_navigation_goal()
        clear_costmaps()
        rover_data['path'].clear()
        rover_data['mode'] = "IDLE"

        return jsonify({'status': 'success', 'message': 'Navigation reset complete', 'timestamp': time.time()})
    except Exception as e:
        return jsonify({'status': 'error', 'error': str(e)}), 500


@app.route('/api/slam_status')
def api_slam_status():
    return jsonify({
        'map_available': slam_data['map_image'] is not None,
        'semantic_map': slam_data['semantic_map'],
        'mission_status': slam_data['mission_status'],
        'path_length': len(slam_data['path_history']),
        'timestamp': time.time()
    })


@app.route('/api/map_image')
def api_map_image():
    if slam_data['map_image']:
        return jsonify({'status': 'success', 'map': slam_data['map_image'], 'timestamp': time.time()})
    else:
        return jsonify({'status': 'no_map', 'message': 'Map not yet available'}), 404


@app.route('/api/detected_objects')
def api_detected_objects():
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
    return jsonify(slam_data['mission_status'])


def init_publisher():
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
                    except Exception:
                        publisher_connected = False
                        break

            except Exception:
                publisher_connected = False
                time.sleep(10)

    thread = threading.Thread(target=publisher_thread, daemon=True)
    thread.start()


if __name__ == '__main__':
    print("🚀 RESTART DETECTION Dashboard - ROS DATA + SLAM")
    log_activity("🚀 Dashboard with restart detection + SLAM starting")

    start_background_workers()
    socketio.start_background_task(target=rosbridge_client)
    init_publisher()

    print("="*60)
    print("🤖 RESTART DETECTION DASHBOARD + SLAM")
    print("="*60)
    print(f"🌐 URL: http://172.22.54.111:5000")
    print(f"📷 Camera: ROS frames")
    print(f"🗺️ Position: /odom data")
    print(f"🔄 Restart Detection: ENABLED (3m threshold)")
    print(f"🛑 Auto Goal Cancel: ENABLED")
    print(f"🧹 Auto Costmap Clear: ENABLED")
    print("="*60)
    print("🗺️ SLAM FEATURES:")
    print("  ✅ Live occupancy grid map")
    print("  ✅ Object detection (rocks, base, flags)")
    print("  ✅ Path history tracking")
    print("  ✅ Mission status monitoring")
    print("  ✅ Battery level display")
    print("  ✅ Emergency return system")
    print("="*60)
    print(f"✅ ROS DATA mode")
    print("="*60)

    try:
        socketio.run(
            app,
            host='172.22.54.111',
            port=5000,
            debug=False,
            use_reloader=False
        )
    except KeyboardInterrupt:
        print("\n🛑 Dashboard stopped")
