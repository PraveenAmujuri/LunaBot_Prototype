

#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import json
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from inference_sdk import InferenceHTTPClient

class YoloLunarDetector:
    def __init__(self):
        rospy.loginfo("🌙 YOLOv8 Lunar Detector Started (Custom Model)")
        
        # Load model configuration
        cache_dir = os.path.expanduser("~/.luna_bot_models")
        config_file = os.path.join(cache_dir, "model_config.json")
        
        if not os.path.exists(config_file):
            rospy.logerr("❌ Model config JSON not found. Run LunaBotModelSetup first.")
            self.yolo_available = False
            return
        
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        self.model_id = config.get('model_id', None)
        self.api_key = config.get('api_key', None)
        
        if self.model_id is None or self.api_key is None:
            rospy.logerr("❌ Model ID or API key missing in config JSON.")
            self.yolo_available = False
            return
        
        # Initialize YOLO client
        self.yolo_client = InferenceHTTPClient(
            api_url="https://detect.roboflow.com", 
            api_key=self.api_key
        )
        
        # YOUR YOLO MODEL CLASSES - Safe vs Obstacle
        # Safe classes (can traverse)
        self.safe_classes = {
            'clear path',
            'austroads',
            'flag',         # Markers are safe but should be recorded
            'antenna',      # Landmarks are safe but should be recorded
        }
        
        # Obstacle classes (must avoid)
        self.obstacle_classes = {
            'rocks',
            'objects',
            'space capsule',
        }
        
        # Caution classes (proceed carefully)
        self.caution_classes = {
            'shadows',
        }
        
        self.yolo_available = True
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = rospy.Publisher('/yolo_detections', String, queue_size=10)
        
        # Subscribe to image topics
        image_topics = [
            '/camera/image_raw', 
            '/usb_cam/image_raw', 
            '/rover_camera/image_raw', 
            '/webcam/image_raw'
        ]
        compressed_topics = [
            '/camera/image_raw/compressed', 
            '/usb_cam/image_raw/compressed',
            '/rover_camera/image_raw/compressed', 
            '/webcam/image_raw/compressed'
        ]
        
        for topic in image_topics:
            rospy.Subscriber(topic, Image, self.image_callback)
        
        for topic in compressed_topics:
            rospy.Subscriber(topic, CompressedImage, self.compressed_callback)
        
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # Detection thread
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()
        
        rospy.loginfo("✅ YOLO Detector Ready!")
        rospy.loginfo(f"   Safe classes: {self.safe_classes}")
        rospy.loginfo(f"   Obstacle classes: {self.obstacle_classes}")
        rospy.loginfo(f"   Caution classes: {self.caution_classes}")
    
    def image_callback(self, msg):
        try:
            if 'jpeg' in msg.encoding.lower() or 'jpg' in msg.encoding.lower():
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if msg.encoding.lower() == 'rgb8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            if cv_image is not None:
                with self.image_lock:
                    self.current_image = cv_image.copy()
        except Exception as e:
            rospy.logwarn_throttle(5, f"Image callback exception: {e}")
    
    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                with self.image_lock:
                    self.current_image = cv_image.copy()
        except Exception as e:
            rospy.logwarn_throttle(5, f"Compressed image callback exception: {e}")
    
    def detection_loop(self):
        rate = rospy.Rate(2)  # 2 Hz for YOLO detection
        
        while not rospy.is_shutdown():
            img = None
            with self.image_lock:
                if self.current_image is not None:
                    img = self.current_image.copy()
            
            if img is None:
                rate.sleep()
                continue
            
            # Run detection
            detections = self.detect_objects(img)
            
            # Publish detections
            if detections is not None:
                detection_msg = String()
                detection_msg.data = json.dumps(detections)
                self.detection_pub.publish(detection_msg)
            
            rate.sleep()
    
    def detect_objects(self, img):
        """Run YOLO detection on image using YOUR custom model"""
        try:
            tmp_path = "/tmp/temp_roboflow_infer.jpg"
            cv2.imwrite(tmp_path, img)
            result = self.yolo_client.infer(tmp_path, model_id=self.model_id)
            os.remove(tmp_path)
            
            detections = []
            
            if 'predictions' in result:
                for pred in result['predictions']:
                    cls = pred.get('class', 'unknown').lower()
                    conf = pred.get('confidence', 0.0)
                    x = int(pred.get('x', 0))
                    y = int(pred.get('y', 0))
                    w = int(pred.get('width', 0))
                    h = int(pred.get('height', 0))
                    
                    # Classify detection type
                    is_obstacle = cls in self.obstacle_classes
                    is_safe = cls in self.safe_classes
                    is_caution = cls in self.caution_classes
                    
                    detection = {
                        'class': cls,
                        'confidence': conf,
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h,
                        'is_obstacle': is_obstacle,
                        'is_safe': is_safe,
                        'is_caution': is_caution
                    }
                    detections.append(detection)
                
                # Log summary of detections
                obstacles = [d['class'] for d in detections if d['is_obstacle']]
                safe_paths = [d['class'] for d in detections if d['is_safe']]
                
                if obstacles:
                    rospy.loginfo_throttle(3, f"⚠️  Obstacles: {obstacles}")
                elif safe_paths:
                    rospy.loginfo_throttle(5, f"✅ Safe paths: {safe_paths}")
                else:
                    rospy.loginfo_throttle(5, f"📡 Detected: {[d['class'] for d in detections]}")
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"Detection error: {e}")
            return None

def main():
    rospy.init_node('yolo_lunar_detector')
    detector = YoloLunarDetector()
    
    if detector.yolo_available:
        rospy.spin()

if __name__ == "__main__":
    main()
