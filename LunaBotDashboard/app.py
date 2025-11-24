#!/usr/bin/env python3
import time
import json
import threading
import websocket
import cv2
import numpy as np
import base64
# near top of file with other imports
import os

import math
from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
from websocket import WebSocketApp
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from queue import Queue, Empty

import psutil
import threading
import time
try:
    import pynvml
    pynvml_available = True
except ImportError:
    pynvml_available = False
    print("⚠️ 'nvidia-ml-py' not installed. GPU stats will be 0.")




# Configuration
ROSBRIDGE_HOST = 'localhost'
ROSBRIDGE_PORT = 9090

app = Flask(__name__)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode='threading', 
    logger=False,
    engineio_logger=False
)

# Rover state
rover_data = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "last_position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "path": deque(maxlen=100), 
    "velocity": {"linear_x": 0.0, "angular_z": 0.0},
    "speed": 0.0,
    "mode": "IDLE",
    "connected": False,
    "restart_count": 0
}

# SLAM data
slam_data = {
    'map_image': None,
    'semantic_map': {'rocks': [], 'base': [], 'flags': [], 'antennas': []},
    'mission_status': {'battery': 100.0, 'mode': 'EXPLORATION', 'distance_to_base': 0.0},
}

STATUS_QUEUE = Queue(maxsize=1)  
MAP_QUEUE = Queue(maxsize=1)      
PROCESSING_QUEUE = Queue(maxsize=10) 

rosbridge_ws = None
rosbridge_pub_ws = None
publisher_connected = False

def start_background_workers():
    def processing_worker():
        while True:
            try:
                job = PROCESSING_QUEUE.get()
                job_type, payload = job
                if job_type == 'map':
                    _process_map_heavy(payload)
                elif job_type == 'semantic':
                    socketio.emit('semantic_update', {'semantic_map': payload})
            except Exception: pass

    def status_emit_worker():
        while True:
            try:
                payload = STATUS_QUEUE.get()
                socketio.emit('rover_status', payload)
                time.sleep(0.05) 
            except Exception: pass

    def map_emit_worker():
        while True:
            try:
                payload = MAP_QUEUE.get()
                socketio.emit('map_update', payload)
                time.sleep(4.0) 
            except Exception: pass

    threading.Thread(target=processing_worker, daemon=True).start()
    threading.Thread(target=status_emit_worker, daemon=True).start()
    threading.Thread(target=map_emit_worker, daemon=True).start()

def _process_map_heavy(msg):
    try:
        width = msg.get('info', {}).get('width', 0)
        height = msg.get('info', {}).get('height', 0)
        data = msg.get('data', [])
        if not data: return

        map_array = np.array(data, dtype=np.int8).reshape((height, width))
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[map_array == -1] = [180, 180, 180]
        img[map_array == 0] = [255, 255, 255]
        img[map_array == 100] = [0, 0, 0]
        
        path = list(rover_data['path'])
        if len(path) > 1:
            origin_x = msg.get('info', {}).get('origin', {}).get('position', {}).get('x', -60.0)
            origin_y = msg.get('info', {}).get('origin', {}).get('position', {}).get('y', -60.0)
            res = msg.get('info', {}).get('resolution', 0.1)
            
            path_slice = path[-30:] 
            for i in range(len(path_slice)-1):
                p1, p2 = path_slice[i], path_slice[i+1]
                px1, py1 = int((p1['x']-origin_x)/res), int((p1['y']-origin_y)/res)
                px2, py2 = int((p2['x']-origin_x)/res), int((p2['y']-origin_y)/res)
                if 0<=px1<width and 0<=py1<height:
                    cv2.line(img, (px1, py1), (px2, py2), (0, 255, 255), 2)

        if width > 320:
            scale = 320 / width
            img = cv2.resize(img, (0,0), fx=scale, fy=scale)

        img = cv2.flip(img, 0)

        _, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 35]) 
        map_b64 = base64.b64encode(buffer).decode('utf-8')

        if MAP_QUEUE.full():
            try: MAP_QUEUE.get_nowait()
            except: pass
        MAP_QUEUE.put({'map': map_b64, 'timestamp': time.time()})
            
    except Exception as e:
        print(f"Map Error: {e}")

# ... (Standard ROS Functions) ...
def connect_rosbridge_publisher():
    global rosbridge_pub_ws, publisher_connected
    try:
        rosbridge_pub_ws = websocket.WebSocket()
        rosbridge_pub_ws.settimeout(5)
        rosbridge_pub_ws.connect(f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
        publisher_connected = True
        return True
    except: return False

def publish_to_ros(topic, msg_type, message):
    global rosbridge_pub_ws
    if not publisher_connected: connect_rosbridge_publisher()
    try:
        rosbridge_pub_ws.send(json.dumps({"op": "publish", "topic": topic, "msg": message}))
        return True
    except: return False

def on_message(ws, message):
    try:
        data = json.loads(message)
        topic = data.get("topic")
        msg = data.get("msg", {})

        if topic == "/odom":
            process_odometry(msg)
        elif topic == "/rtabmap/grid_map":
            if not PROCESSING_QUEUE.full():
                PROCESSING_QUEUE.put(('map', msg))
        elif topic == "/semantic_detections":
            PROCESSING_QUEUE.put(('semantic', json.loads(msg['data'])))
        elif topic == "/mission_status":
            slam_data['mission_status'] = json.loads(msg['data'])
            socketio.emit('mission_update', {'mission_status': slam_data['mission_status']})

    except Exception: pass

def process_odometry(msg):
    try:
        pos = msg['pose']['pose']['position']
        vel = msg['twist']['twist']
        new_x, new_y = float(pos['x']), float(pos['y'])
        
        if math.dist([new_x, new_y], [rover_data['last_position']['x'], rover_data['last_position']['y']]) > 10.0:
            rover_data['path'].clear()
            socketio.emit('unity_restart_detected', {'distance_jump': 100})

        rover_data['last_position'] = {'x': new_x, 'y': new_y}
        rover_data['position'] = {'x': new_x, 'y': new_y}
        rover_data['path'].append({'x': new_x, 'y': new_y})
        
        lx = float(vel['linear']['x'])
        rover_data['velocity'] = {'linear_x': lx, 'angular_z': float(vel['angular']['z'])}
        rover_data['speed'] = abs(lx)
        rover_data['mode'] = "MOVING" if abs(lx) > 0.01 else "IDLE"

        payload = {
            'position': rover_data['position'],
            'velocity': rover_data['velocity'],
            'speed': rover_data['speed'],
            'mode': rover_data['mode'],
            'path': list(rover_data['path']),
            'server_time': time.time() * 1000 
        }

        if STATUS_QUEUE.full():
            try: STATUS_QUEUE.get_nowait()
            except: pass
        STATUS_QUEUE.put(payload)

    except Exception: pass

def background_metrics_emitter():
    """Background thread to emit system metrics every 1 second"""
    print("✅ Metrics thread started")
    
    # Initialize NVML 
    nvml_ok = False
    if pynvml_available:
        try:
            pynvml.nvmlInit()
            try:
                drv = pynvml.nvmlSystemGetDriverVersion()
                # nvmlSystemGetDriverVersion may return bytes or str
                if isinstance(drv, bytes):
                    drv = drv.decode(errors='ignore')
                print(f"✅ NVIDIA Driver detected: {drv}")
            except Exception:
                print("✅ NVIDIA Driver detected (couldn't read version string)")
            nvml_ok = True
        except Exception as e:
            print(f"❌ Failed to init NVIDIA ML: {e}")
            nvml_ok = False

    while True:
        try:
            # 1. CPU usage
            cpu = psutil.cpu_percent(interval=None)
            
            # 2. RAM
            mem = psutil.virtual_memory()
            ram_percent = mem.percent
            ram_used = round(mem.used / (1024 ** 3), 1)
            ram_total = round(mem.total / (1024 ** 3), 1)

            # 3. GPU usage (via NVML) - fallback to 0
            gpu = 0
            gpu_temp = 0
            gpu_mem_used = 0
            gpu_mem_total = 0
            gpu_mem_percent = 0

            if nvml_ok:
                try:
                    # choose GPU index 0 (change if you want to pick another GPU)
                    handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                    util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                    gpu = getattr(util, 'gpu', 0) or 0

                    # temperature 
                    try:
                        gpu_temp = pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
                    except Exception:
                        try:
                            gpu_temp = pynvml.nvmlDeviceGetTemperature(handle, 0)
                        except Exception:
                            gpu_temp = 0

                    # memory info
                    try:
                        mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
                        # mem_info.total / used are ints (bytes)
                        gpu_mem_total = int(getattr(mem_info, 'total', mem_info[0] if hasattr(mem_info, '__getitem__') else 0))
                        gpu_mem_used  = int(getattr(mem_info, 'used',  mem_info[1] if hasattr(mem_info, '__getitem__') else 0))
                        if gpu_mem_total > 0:
                            gpu_mem_percent = round((gpu_mem_used / gpu_mem_total) * 100, 1)
                        else:
                            gpu_mem_percent = 0
                    except Exception:
                        gpu_mem_used = gpu_mem_total = gpu_mem_percent = 0

                except Exception as e:
                    # fallback if single-device access failed
                    try:
                        device_count = pynvml.nvmlDeviceGetCount()
                        if device_count > 0:
                            handle = pynvml.nvmlDeviceGetHandleByIndex(0)
                            util = pynvml.nvmlDeviceGetUtilizationRates(handle)
                            gpu = getattr(util, 'gpu', 0) or 0
                            mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
                            gpu_mem_total = int(getattr(mem_info, 'total', 0))
                            gpu_mem_used  = int(getattr(mem_info, 'used', 0))
                            gpu_mem_percent = round((gpu_mem_used / gpu_mem_total) * 100, 1) if gpu_mem_total > 0 else 0
                    except Exception:
                        gpu = gpu_temp = gpu_mem_used = gpu_mem_total = gpu_mem_percent = 0


            # 4. Temp debug for cases where NVML not present
            temp = gpu_temp or 0

            # Emit data - include both gpu_temp and temp
            socketio.emit('system_metrics', {
                'cpu': cpu,
                'gpu': gpu,
                'ram': ram_percent,
                'ram_used': ram_used,
                'ram_total': ram_total,
                'gpu_temp': gpu_temp,
                'temp': temp,
                'gpu_mem_used': gpu_mem_used,    # bytes
                'gpu_mem_total': gpu_mem_total,  # bytes
                'gpu_mem_percent': gpu_mem_percent
            })

        except Exception as e:
            print(f"Metrics Error: {e}")
            time.sleep(1)
        time.sleep(1)



def on_open(ws):
    global rosbridge_ws
    rosbridge_ws = ws
    print("✅ ROS Connected (Micro-Map Mode)")
    topics = [
        {"topic": "/odom", "type": "nav_msgs/Odometry"},
        {"topic": "/rtabmap/grid_map", "type": "nav_msgs/OccupancyGrid"},
        {"topic": "/semantic_detections", "type": "std_msgs/String"},
        {"topic": "/mission_status", "type": "std_msgs/String"},
    ]
    for t in topics:
        throttle = 5000 if 'grid_map' in t['topic'] else 50
        ws.send(json.dumps({"op": "subscribe", "topic": t['topic'], "type": t['type'], "throttle_rate": throttle}))

def rosbridge_client():
    while True:
        try:
            ws = WebSocketApp(f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}", on_message=on_message, on_open=on_open)
            ws.run_forever()
        except: time.sleep(5)

@socketio.on('rover_command')
def handle_command(data):
    cmd = data.get('cmd_vel', {})
    lx = float(cmd.get('linear', {}).get('x', 0))
    az = float(cmd.get('angular', {}).get('z', 0))
    publish_to_ros('/cmd_vel', 'geometry_msgs/Twist', {'linear': {'x': lx, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': az}})

@socketio.on('navigation_goal')
def handle_goal(data):
    pos = data.get('goal', {}).get('position', {})
    publish_to_ros('/move_base_simple/goal', 'geometry_msgs/PoseStamped', {'header': {'frame_id': 'map'}, 'pose': {'position': {'x': float(pos.get('x',0)), 'y': float(pos.get('y',0)), 'z': 0}, 'orientation': {'w': 1}}})

@socketio.on('reset_navigation')
def handle_reset():
    publish_to_ros('/move_base/cancel', 'actionlib_msgs/GoalID', {})
    rover_data['path'].clear()

@app.route('/')
def index(): return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    rover_data_safe = rover_data.copy()
    rover_data_safe['path'] = list(rover_data['path'])
    emit('initial_state', {'rover_data': rover_data_safe, 'slam_data': slam_data})

if __name__ == '__main__':
    start_background_workers()
    threading.Thread(target=rosbridge_client, daemon=True).start()
    threading.Thread(target=background_metrics_emitter, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)